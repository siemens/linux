/*
 * drivers/aud/auddevice/rt_thread.c
 *
 * 2007-19-01:  Karl-Heinz.Krause@siemens.com
 * Copyright (c) 2007 Siemens AG
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, Inc., 675 Mass Ave, Cambridge MA 02139,
 * USA; either version 2 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 * This file contains all the thread management functions needed by the RT domain.
 * 
 */
#include <linux/aud/rt_timer.h>
#include <linux/aud/rt_futex.h>

DECLARE_PER_CPU(struct list_head, rt_tl);

/* Check if policy is a RT-policy. */
static inline int is_rt_policy(int policy)
{
	if (likely(policy == SCHED_FIFO || policy == SCHED_RR))
		return 1;
	return 0;
}

/*
 * Set the scheduling policy of a thread.
 */
void rt_set_policy(int policy, struct task_struct *p)
{
	if (policy != -1) {
		p->policy = policy;
		switch(p->policy) {
			case SCHED_NORMAL:
				p->sched_class = rtx_get_fair_sched_class();
				break;
			case SCHED_FIFO:
			case SCHED_RR:
				p->sched_class = rtx_get_rt_sched_class();
				break;
		}
	}
}

/*
 * Scan all the RT threads looking for a special pid.
 */
struct task_struct * rt_find_task_by_pid(pid_t pid)
{
	struct task_struct *proc = NULL;
	struct list_head *lp, *head;

redo_scan:
	head = &__raw_get_cpu_var(rt_tl);
	lp = head->next;
	while(lp != head)
	{
		local_irq_disable_hw();
		proc = (struct task_struct*)list_entry(lp, struct task_struct, rt_pid_list );
		if (list_empty(&proc->rt_pid_list)) {
			local_irq_enable_hw();
			goto redo_scan;
		}
		lp = lp->next;
		local_irq_enable_hw();
		if (pid == proc->pid)
			break;
		proc = NULL;
	}
	return proc;
}
  
/*
 * A thread calls either pthread_exit() or exit().
 * Both calls are routed to Linux, from there all exit handling will take place.
 */
asmlinkage long sys_rt_exit(int error_code)
{
	if (RT_SHUTDOWN_VERBOSE)
		printkGen(NULL, "called exit in RT (time: usr=%d sys=%d)\n", current->utime, current->stime);
	return rt_task_leave();					// re-executed in the Linux domain
}

asmlinkage long sys_rt_exit_group(int error_code)
{	
	if (RT_SHUTDOWN_VERBOSE)
		printkGen(NULL, "called exit group in RT (time: usr=%d sys=%d)\n", current->utime, current->stime);

	current->rt_state2 |= RT_TASK_IN_EXIT;
	down_rt(&__raw_get_cpu_var(rt_tl_sem));
	rt_mark_exit_pending(NULL);					// no further RT activity will take place
	up_rt(&__raw_get_cpu_var(rt_tl_sem));
	return rt_task_leave();						// re-executed in the Linux domain
}

asmlinkage long sys_rt_getpid(void)
{
	return current->tgid;
}

asmlinkage long sys_rt_getuid(void)
{
	return current_uid();
}


asmlinkage long sys_rt_gettid(void)
{
	return current->pid;
}

/*
 * This routine locks the realtime thread list (rt_tl_sem) looking it up for a special pid.
 * If the thread with this pid was found the realtime thread list remains locked.
 * If the thread was not found the realtime thread list semaphore rt_tl_sem is released and execution
 * is propagated to the Linux domain. In this case the caller must return, so that the system call
 * execution can get repeated in the Linux domain.
 */
static struct task_struct *rt_get_task_locked(pid_t pid, int lock_flag)
{
	struct task_struct *p;
	
	down_rt(&__raw_get_cpu_var(rt_tl_sem));
	if (current->pid == pid)
		return current;						// most frequent case (for priority ceiling)

	// If the task is found 
	// - it may be running/waiting in the RT-domain
	// - it may be executing temporarily in the LX-domain (RTLX_TASK)
	// - it may be on its way to the LX-domain (RT_LEAVE_PENDING-flags)
	if (((p = rt_find_task_by_pid(pid)) == NULL))
	{
		up_rt(&__raw_get_cpu_var(rt_tl_sem));
		rtx_migrate_to_lx(RTLX_TASK | lock_flag, p);		// retry call in the LX-domain
		p = NULL;
	} 
	return p;
}


static long rt_do_setscheduler(pid_t pid, int policy, struct sched_param __user *param)
{
	struct sched_param lp;
	struct task_struct *p;
	int prio;

	/* Initialize the pid according to the POSIX definition. */
	if (!pid)
		pid = current->pid;
	
	if (rt_copy_from_user(&lp, param, sizeof(struct sched_param)))
		return -EFAULT;	

	if (policy == SCHED_NORMAL) {
		if (lp.sched_priority != 0)
			return EINVAL;

		// SCHED_OTHER uses priority 100 (to 140).
		prio = MAX_USER_RT_PRIO;
	}
	else {
		if (is_rt_policy(policy)) {
			if ((lp.sched_priority < 1) || (lp.sched_priority > (MAX_USER_RT_PRIO - 1)))
				return -EINVAL;
		}
		prio = (MAX_USER_RT_PRIO - 1) - lp.sched_priority;
	}	

	// When returning NULL, we already migrated to the LX domain.
	if ((p = rt_get_task_locked(pid, RTLX_LOCK_MIGRATION)) == NULL) {	// task not in realtime thread list
		if (RT_THREAD_VERBOSE)
			printkGen(NULL,"setting lx-prio=%d:%d for pid=%d\n", lp.sched_priority, prio, pid);		
		return 0;								// execute the call in the Linux domain
	}
	
	if (policy != -1)
	{
		if (!((policy == SCHED_FIFO) || (policy == SCHED_RR) || (policy == SCHED_NORMAL))) 
		{
			up_rt(&__raw_get_cpu_var(rt_tl_sem));
			if (RT_THREAD_VERBOSE)
				printkGen(NULL, "%s: wrong policy=%d\n", __func__, policy);
			return -EINVAL;	
		}
	}
	else {
		/* Check for valid policy/priority values. */
		if ((is_rt_policy(p->policy) != (lp.sched_priority != 0)) || prio < 0) {
			up_rt(&__raw_get_cpu_var(rt_tl_sem));
			if (RT_THREAD_VERBOSE)
				printkGen(NULL, "%s: invalid priority (policy=%d prio=%d)\n", __func__, p->policy, lp.sched_priority);
			return -EINVAL;	
		}
		/* We can't change the priority of a thread with policy SCHED_NORMAL. */
		if (p->policy == SCHED_NORMAL) {
			up_rt(&__raw_get_cpu_var(rt_tl_sem));
			return 0;
		}
	}

	// The priority change of a thread may concern
	// the current thread (p == current) or 
	// another thread (p != current).
	if (prio >= MAX_NO_OF_RT_PRIO)
	{
		// A RT-Thread is being involved to migrate to LX.
		// But there are situations the migration call cannot be executed.
		if (p->rt_notify_state && rt_notification(p))
		{
			// Can't migrate to LX, there are notifications
			// in the RT-domain for this thread.
			up_rt(&__raw_get_cpu_var(rt_tl_sem));
			return -EINVAL;
		}
		local_irq_disable_hw();

		// The priority of a RT thread is going to be changed
		// to a priority which causes this thread to migrate
		// back to LX.
		// We cannot allow a RT thread to migrate back to LX
		// while it has PI waiters or is waiting itself on a futex.
		if (rt_task_has_pi_waiters(p) || p->rt_pi_blocked_on) {
			local_irq_enable_hw();
			up_rt(&__raw_get_cpu_var(rt_tl_sem));
			return -EPERM;
		}
		
		/*
		* We want to change the priority of a RT-thread which is temporarily
		* executing in LX. The priority to be set is a LX priority. Thus we
		* have to defer the priority change until the RT-thread is back in RT.
		* After changing the priority the RT-thread has to migrate back to LX.
		* Note: In this case we cannot handle a policy change and a priority
		* change for the SCHED_RR policy (we cannot setup the LX activities
		* which are usually handled by the sched_setscheduler() system-call).
		*/
		if (p->rt_state == RTLX_TASK) {
			if (policy != -1 && (p->policy != policy || policy == SCHED_RR)) {
				local_irq_enable_hw();
				up_rt(&__raw_get_cpu_var(rt_tl_sem));
				printkGen(KERN_ALERT, "%s: invalid policy=%d for deferred priority change (cpid=%d ppid=%d ppolicy=%d\n", __func__, policy, current->pid, p->pid, p->policy);
				return -EPERM;
			}
			p->rt_dyn_prio = -lp.sched_priority;	// cannot be prio 0
			p->rt_dyn_policy = -1;
			local_irq_enable_hw();
			up_rt(&__raw_get_cpu_var(rt_tl_sem));
			return 0;
		}
		/* 
		* We do not remove the thread from the pid list (rt_tl),
		* otherwise it couldn't be found anymore, which may cause
		* that the RT-process cannot be shutdown properly
		* (e.g. by pressing CTRL+C).
		*/
		if (THREAD_IS_REALTIME(p)) {
			if (p == current)
				list_del_init(&p->rt_pid_list);		//will migrate to LX immediately
			else
				p->rt_state2 |= RT_LEAVE_PENDING_PID_LIST;
		}
		local_irq_enable_hw();
	}
	up_rt(&__raw_get_cpu_var(rt_tl_sem));

	// The selected thread is currently executing in the
	// realtime domain or it is a non-active RT thread.
	if (THREAD_IS_REALTIME(p))
	{
		local_irq_disable_hw();
		rt_set_policy(policy, p);
		p->rt_priority = lp.sched_priority;
		p->normal_prio = prio;
		// Make sure that no PI waiters arrive or leave
		// while we are changing the priority.
		// Set the new priority which may be boosted
		// by PI waiters.
		p->prio = rt_futex_getprio(p);

		// May be there is still a pending request for setting 
		// a LX-priority for this thread (overwrite it).
		if (prio < MAX_NO_OF_RT_PRIO)
			p->rt_state2 &= ~(RT_LEAVE_PENDING_PID_LIST | RT_LEAVE_PENDING_TASK);

		// May be we have to boost/de-boost the owner thread.
		if (p->rt_pi_blocked_on)
			rt_fixup_owner_prio(p->rt_owner_futex, p->rt_pi_blocked_on);

		local_irq_enable_hw();

		if (RT_THREAD_VERBOSE)
			printkGen(NULL,"setting rt-prio=%d:%d for pid=%d\n", lp.sched_priority, prio, pid);		
		return rt_reschedule(p);
	}

	// We want to change the priority of a RT-thread which is
	// temporarily executing in LX (execute_in_lx). The priority 
	// to be set is a LX priority. This setting must be deferred
	// and is handled when the thread migrates back to RT.
	// Both cases (p == current and p != current) are handled
	// identical.
	if (RT_THREAD_VERBOSE)
		printkGen(NULL, "need to change prio during call of RT thread in LX (pid=%d prio=%d)\n", pid, prio);
	rtx_migrate_to_lx(RTLX_TASK, NULL);			// no synchronization necessary
	return 0;
}

asmlinkage long sys_rt_sched_setparam(pid_t pid, struct sched_param __user *param)
{
	long ret;
	ret = rt_do_setscheduler(pid, -1, param);
	return ret;
}

asmlinkage long sys_rt_sched_setscheduler(pid_t pid, int policy, struct sched_param __user *param)
{
	long ret;
	ret = rt_do_setscheduler(pid, policy, param);
	return ret;
}	

asmlinkage long sys_rt_sched_getparam(pid_t pid, struct sched_param __user *param)
{
	struct sched_param lp;
	struct task_struct *p;

	/* Initialize the pid according to the POSIX definition. */
	if (!pid)
		pid = current->pid;

	// If no task found, execute the call in the LX-domain.
	if ((p = rt_get_task_locked(pid, 0)) == NULL)
		return 0;
	lp.sched_priority	= p->rt_priority;
	up_rt(&__raw_get_cpu_var(rt_tl_sem));

	return (rt_copy_to_user(param, &lp, sizeof(struct sched_param)) ? -EFAULT : 0);
}

asmlinkage long sys_rt_sched_getscheduler(pid_t pid)
{
	struct task_struct *p;
	int policy;

	/* Initialize the pid according to the POSIX definition. */
	if (!pid)
		pid = current->pid;

	// If no task found, execute the call in the LX-domain.
	if ((p = rt_get_task_locked(pid, 0)) == NULL)
		return 0;
	policy = p->policy;
	up_rt(&__raw_get_cpu_var(rt_tl_sem));
	return policy;									
}	


