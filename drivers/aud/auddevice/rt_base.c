/*
 * drivers/aud/auddevice/rt_base.c
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
 * This file contains all the base functions for setting up and 
 * shutting down the RT domain and for thread migration between
 * RT and Linux domain.  
 */
#include <linux/aud/rt_timer.h>
#include <linux/aud/rt_futex.h>
#include <linux/rtx_arith.h>
#ifdef CONFIG_RTX_EXTENDED_RT_DEBUGGING_SUPPORT
#include <linux/aud/rt_debug.h>
#endif

DEFINE_PER_CPU(struct task_struct *, interdomain_daemon);
DEFINE_PER_CPU(struct sync_param, rt_sync_migration);
DEFINE_PER_CPU(struct list_head, wake_lx_list);
DEFINE_PER_CPU(struct semaphore, to_rt_op);
static DEFINE_PER_CPU(struct rt_semaphore, sync_mig_sem);
static DEFINE_PER_CPU(struct rt_semaphore, wait_mig_sem);

DECLARE_PER_CPU(struct rt_event *, aud_evpr);
DECLARE_PER_CPU(uint64_t, isr_time);
DECLARE_PER_CPU(struct list_head, rt_tl);

RTX_DECLARE_SPINLOCK(futex_list_lock);
#ifdef CONFIG_LTT
#include <linux/ltt-core.h>
#include <trace/fs.h>
#endif

#ifdef CONFIG_LTT
extern unsigned actLttTraceId;
#endif
 

/*
* Reset the sync-migration data structures,
* if the migration source/target is shutting down.
*/
irqreturn_t cancel_sync_migration(int irq, void *cookie)
{
	struct task_struct *ftsk = rtx_get_cpu_member(rt_sync_migration, first_waiter);

	/* Wake up the waiting buddy. */
	if (ftsk && (ftsk != current)) {
		up_rt_from_virq(&__raw_get_cpu_var(wait_mig_sem));
		return IRQ_HANDLED;
	}
	/* There is no target thread waiting. */
	rtx_set_cpu_member(rt_sync_migration, source, NULL);
	rtx_set_cpu_member(rt_sync_migration, target, NULL);
	rtx_set_cpu_member(rt_sync_migration, first_waiter, NULL);
	up_rt_from_virq(&__raw_get_cpu_var(sync_mig_sem));
	return IRQ_HANDLED;
}

/*
 * This Linux ISR is used to propagate actions which need to be executed by a thread
 * in the linux domain, the interdomain_daemon. The actions are:
 * - time-of-day
 * - do_futex
 * - migrate to rt
 */
irqreturn_t to_lx_isr(int irq, void *cookie)
{
	struct task_struct *p = rtx_get_cpu_var(interdomain_daemon);

	if (p != NULL && ((p->state & TASK_RUNNING) == 0))
		wake_up_process(p);
	return IRQ_HANDLED;
}

/*
 * This realtime ISR notifies the realtime domain. It gets called whenever the Linux domain brings a
 * thread into the runqueue of the realtime domain. Since the check for preemption is done in
 * rtx_call_rt_domain, the isr can be a noop.
 */
irqreturn_t to_rt_isr(int irq, void *cookie)
{
	/* Only to have a testing facility for events. */
	if (rtx_get_cpu_var(aud_evpr)) {
		__raw_get_cpu_var(isr_time) = rtx_gettime();
		rt_send_event(rtx_get_cpu_var(aud_evpr));
		rtx_set_cpu_var(aud_evpr, NULL);
	}
	return IRQ_HANDLED;
}

/*
 * For migrating a thread to Linux or having a thread checking for an available message
 * the wake_up_process can be done directly from ISR level.
 */
irqreturn_t wake_up_lx_isr(int irq, void *cookie)
{
	struct proc_wait_queue *req;
	struct list_head *p;
	unsigned long flags;
	struct list_head *plih;

	local_irq_save_hw(flags);
	plih = &__raw_get_cpu_var(wake_lx_list);
	while (!list_empty(plih)) {
		p = plih->next;
		list_del_init(p);  /* mqueue wait sources require an empty list */
		req = (struct proc_wait_queue *)p;
		local_irq_restore_hw(flags);
		wake_up_process(req->task);
		local_irq_save_hw(flags);
	}
	local_irq_restore_hw(flags);
	return IRQ_HANDLED;
}

/*
* The clone system call needs special handling in the RT-domain.
*/
asmlinkage int sys_rt_clone(struct pt_regs regs)
{
	if (RT_SYSCALL_IN_LX_VERBOSE)
		printkGen(NULL, "syscall id=%d (sys_rt_clone) delegated to LX\n", RTX_SYSCALL_ID());
	rtx_migrate_to_lx(RTLX_TASK | RTLX_LOCK_MIGRATION, NULL);
	return 0;
}

/*
* Invoked by system calls which are not implemented in the RT domain.
*/
asmlinkage long sys_rt_ni_syscall(struct pt_regs regs)
{
	/* Syscall id 57 (setpgid) is used by glibc to detect execution environment. */
	if ((RT_FAULT_VERBOSE && (RTX_SYSCALL_ID() != RTX_SYSCALL_SETPGID)) || RT_SYSCALL_NOT_IMPL_VERBOSE)
		printkGen(NULL, "syscall id=%d not implemented\n", RTX_SYSCALL_ID());
	return -ENOSYS;
}

/*
 * Enqueue the RT-thread p in the migration list. This happens when the thread
 * migrates temporarily from the RT to the LX domain (RTLX_TASK, LXRT_TASK).
 * Note: Interrupts must be off.
 */
inline void rt_enqueue_migration_thread(migqueue_t *pmq, struct task_struct *p)
{
	int idx = p->prio;

	list_add_tail(&p->rt_mig_list, &pmq->queue[idx].qhead);
	__set_bit(idx, &pmq->bitmapPrio);
 }

/*
 * Dequeue the RT-thread p from the migration list. This happens when the thread
 * migrates back from the LX to the RT domain or if the thread is calling do_exit.
 */
void rt_dequeue_migration_thread(migqueue_t *pmq, struct task_struct *p)
{
	struct list_head *lp;
	struct rq_head *qp;
	unsigned long flags;

	local_irq_save_hw(flags);

	// For a LXRT_TASK the list head is not initialized.
	if (p->rt_mig_list.next && !list_empty(&p->rt_mig_list)) {
		lp = p->rt_mig_list.prev;
		list_del_init(&p->rt_mig_list);
		if (list_empty(lp)) {
			qp = (struct rq_head *)list_entry(lp, struct rq_head, qhead);
			__clear_bit(qp->idx, &pmq->bitmapPrio);
		}
	}
	local_irq_restore_hw(flags);
}

/*
 * Add a new request and wakeup the interdomain daemon to process this request.
 * A free list of request elements is only needed for futex requests.
 * For thread delegation the request element is built on the stack of the requesting thread.
 */
int add_interdomain_request(void *arg)
{
	unsigned long flags;
	struct lx_cmd *req = (struct lx_cmd *)arg;

	if (!rtx_get_cpu_var(interdomain_daemon) || !list_empty(&req->req_link))
		return 0;
	if (RTX_SYS_IS_RT_DOMAIN) {
		local_irq_save_hw(flags);
		list_add_tail(&req->req_link, &current->rt_proc->to_lx_list);
		log_interrupt_as_pending(to_lx_virq);
		local_irq_restore_hw(flags);
	}
	else {
		local_irq_save_hw(flags);
		list_add_tail(&req->req_link, &current->rt_proc->to_lx_list);
		local_irq_restore_hw(flags);
		wake_up_process(rtx_get_cpu_var(interdomain_daemon));
	}
	return 0;
}


/*
 * Initialize core specific data structures.
 * Note: It is assumed that dev_sem is already locked.
 */
int rt_init_realtime(struct probe_descr * wbuf)
{
#ifndef CONFIG_RTX_DISABLE_WRITE_DAEMON
	if (init_write_service(wbuf))
		return -1;
#endif
	sema_init_rt(&__raw_get_cpu_var(rt_tl_sem), 1);		// access to the realtime thread list
	sema_init(&__raw_get_cpu_var(to_rt_op), 1);			// for locking the migration action
	sema_init_rt(&__raw_get_cpu_var(sync_mig_sem), 1);	// to allow only one single synchronized migration in the RT-domain
	sema_init_rt(&__raw_get_cpu_var(wait_mig_sem), 0);	// for a migration target thread to continue the migration source thread or vice versa
	rtx_set_cpu_var(prt_runqueue, &__raw_get_cpu_var(rt_runqueue));
	rtx_set_cpu_var(rt_timer_daemon, NULL);
	rtx_set_cpu_var(interdomain_daemon, NULL);
	INIT_LIST_HEAD(&aud_rt_hdr.reg_list);	
	INIT_LIST_HEAD(&__raw_get_cpu_var(wake_lx_list));   // must be done in register_process
	INIT_LIST_HEAD(&__raw_get_cpu_var(rt_tl));
	rt_init_sched();
	rt_init_futex_queue();
	rt_init_nfb_pool();	
	__raw_get_cpu_var(rr_timer.it_period) = 0LL;
	__raw_get_cpu_var(rt_new_offset) = 0LL;
	return 0;
}

/*
 * A thread migrates from the LX-domain to the RT-domain.  
 * The thread notifies the interdomain daemon, puts itself out of the runqueue 
 * by doing a schedule. The interdomain daemon waits until the thread is out of the runqueue. 
 * Then it calls the function rt_wake_up_thread() to get it into the runqueue of the RT
 * scheduler and triggers a soft-interrupt for the realtime domain. 
 * For migrating kernel threads this function is called explicitly. For user threads this 
 * function is always called when leaving or entering a system call in the LX-domain
 * and the thread has LXRT_TASK_LEAVE_PENDING or RTLX_TASK set.
 */
int rtx_migrate_to_rt(int sys_retval)
{	
	long rt_state, orgRtState3, lx_pcount;
	struct lx_cmd  req;	

	orgRtState3 = current->rt_state3;
	// clear rt_debug flags
	current->rt_state3 &= ~(RT_TASK_RT_DEBUG_PENDING
		| RT_TASK_LX_DEBUG_PENDING);
	rt_state = current->rt_state;	
	lx_pcount = preempt_count();
	if (!IS_REALTIME_PRIO_NORMALIZED(current->prio)) 
	{
		printkGen(KERN_ALERT, "tried to migrate to RT with prio=%d:%d\n", current->rt_priority, current->prio);
		return 0;								// can't migrate when prio too low	
	}
#ifdef CONFIG_SMP
	/* Force thread to run on its RT-cpu.
	 * Note: It shouldn't be necessary to migrate the thread.
	 */
	if (set_cpus_allowed(current, current->rt_proc->cpu_mask) != 0)
		printkGen(KERN_ALERT, "forcing thread pid=%d to run on RT cpu=%d failed\n", current->pid, current->rt_proc->cpu);
#endif /* CONFIG_SMP */

	// Serialize competing migrations.
	// The interdomain daemon unlocks the semaphore to_rt_op.
	down(&__raw_get_cpu_var(to_rt_op));
	if (unlikely(rtx_get_cpu_var(rt_system_state) & RT_SHUTDOWN))
	{
		up(&__raw_get_cpu_var(to_rt_op));
		if (current == rtx_get_cpu_member(rt_sync_migration, source) ||
				current == rtx_get_cpu_member(rt_sync_migration, target))
			__rtx_call_rt_domain(to_mg_virq);
		do_exit(0);
	}
	INIT_LIST_HEAD(&req.req_link);
	req.type = MIGRATE_TO_RT;
	req._req._proc = current;	

	sigemptyset(&current->real_blocked);		// is used as ->blocked in the realtime domain
	preempt_disable();	
	add_interdomain_request(&req);
	__set_current_state(TASK_UNINTERRUPTIBLE);	
	rtx_preempt_enable_no_resched();
	schedule();									// in case of SHUTDOWN it will stay until KILL signal
												// from here it runs under control of the realtime scheduler

	local_irq_enable_hw();
#ifdef CONFIG_RTX_EXTENDED_RT_DEBUGGING_SUPPORT
	if (orgRtState3 & RT_TASK_LX_DEBUG_PENDING) {
		testForExtendedDebugWaitToRt();
	}
#endif

	if (RT_MIGRATE_VERBOSE)	
		printkGen(NULL, "transition lx (rt_state=0x%04x preempt_count=%#x) -> rt (rt_state=0x%04x preempt_count=%#x)\n",
                rt_state, lx_pcount, current->rt_state, preempt_count());		

	/* Handle synchronization of a migration (source/target) to the RT-domain. */
	if (current == rtx_get_cpu_member(rt_sync_migration, source) ||
			current == rtx_get_cpu_member(rt_sync_migration, target)) {

		if (rtx_get_cpu_member(rt_sync_migration, target) == CANCEL_MIGRATION)
			goto cancel_migration;
		if (rtx_get_cpu_member(rt_sync_migration, first_waiter) == NULL) {
			/* Check whether the system-call of the initiator was successful or not. */
			if (current == rtx_get_cpu_member(rt_sync_migration, source) &&	(sys_retval < 0))
				goto cancel_migration;
			rtx_set_cpu_member(rt_sync_migration, first_waiter, current);
			down_rt(&__raw_get_cpu_var(wait_mig_sem));
cancel_migration:
			rtx_set_cpu_member(rt_sync_migration, source, NULL);
			rtx_set_cpu_member(rt_sync_migration, target, NULL);
			rtx_set_cpu_member(rt_sync_migration, first_waiter, NULL);
			/* Allow for the next migration */
			up_rt(&__raw_get_cpu_var(sync_mig_sem));
		}
		else {
			/* End of synchronization, wake up the synchronization buddy. */
			up_rt(&__raw_get_cpu_var(wait_mig_sem));
		}
	}

	// Check if the current thread has to be removed from the migration queue.
        // do this after parent-child synchronisation to allow child to migrate to rt
	rt_dequeue_migration_thread(&__raw_get_cpu_var(rt_migqueue), current);
        
	/*
	* Deferred dynamic LX priority change for a RT-thread.
	* We cannot handle policy changes for the LX-thread.
	*/
	if (current->rt_dyn_prio) {
		current->rt_priority = current->rt_dyn_prio;	// LX-priority
		current->normal_prio = (MAX_USER_RT_PRIO - 1) - current->rt_priority;
		current->prio = current->normal_prio;
		current->rt_dyn_prio = 0;
		current->rt_dyn_policy = 0;
		rt_task_leave();				// migrate back to LX
		return 0;
	}

	// If the setting of the scheduling policy was deferred we may have
	// to setup the rr timer.
	if (rt_state & LXRT_TASK_LEAVE_PENDING || current->rt_dyn_policy == SCHED_RR) {
		if (current->policy == SCHED_RR)	
			setup_rr_timer(current);
		// Initial migration of user threads.
		if (rt_state & LXRT_TASK_LEAVE_PENDING) {
			RTX_UNLAZY_FPU(current);
			if (RT_INIT_VERBOSE)
				printkGen(NULL, "initial migration to RT domain completed (prio=%d)\n", current->prio);
		}
	}
	current->rt_dyn_policy = 0;
	THREAD_RUNTIME_POST_SWITCH();				// runtime accounting 
#ifdef CONFIG_RTX_THREAD_GROUP_SUPPORT
	quota_post_switch_from_lx(rtx_get_cpu_var(pquota_head));
#endif

#ifdef CONFIG_RTX_EXTENDED_RT_DEBUGGING_SUPPORT
        if (current->rt_state3
                   & (RT_TASK_LX_DEBUG_PENDING | RT_TASK_RT_DEBUG_PENDING)) {
            // we caught a next debugging event
            rtx_migrate_to_lx(RTLX_TASK, NULL);
            return(0);
        }
#endif
	return 1;
}


/* 
 * A thread migrates from the RT-domain to the LX-domain.  
 * There are different migration reasons:
 * 1) System-call execution in the LX-domain: The thread stays in the rt_pid_list.
 *    Therefore it stays a target for notification. Sending a signal or creating
 *    a new notification for that thread does no harm.
 * 2) Priority change: It can only take place if there are no notifications for that thread.
 *    The thread is taken out of the rt_pid_list upfront.
 * 3) Thread termination (either a RT shutdown or exiting a thread): All notification channels 
 *    for that thread are deleted upfront. The thread is taken out of the rt_pid_list upfront.
 * 4) The thread is being debugged.
 *
 */
void rtx_migrate_to_lx(int lx_rt_state, struct task_struct *p)
{
	int rt_pcount, rt_state;
	struct task_struct *prev;
	struct proc_wait_queue req;
	
#ifdef CONFIG_RTX_EXTENDED_RT_DEBUGGING_SUPPORT
	if (current->rt_state3 & RT_TASK_RT_DEBUG_PENDING) {
		testForExtendedDebugWaitToLx();
	}
#endif
	// In some special cases (e.g. sys_rt_clone) the migration back to
	// the RT-domain is synchronized between source and target thread.
	if (lx_rt_state & RTLX_LOCK_MIGRATION) {
 		lx_rt_state &= ~RTLX_LOCK_MIGRATION;
 		down_rt(&__raw_get_cpu_var(sync_mig_sem));
 		rtx_set_cpu_member(rt_sync_migration, source, current);
 		rtx_set_cpu_member(rt_sync_migration, target, p);	// target may be NULL, sys_rt_clone()
	}

	rt_state = current->rt_state;
	rt_pcount = preempt_count();

	/* The following sequence must be atomic. */
	local_irq_disable_hw();

	// Check if the current thread has to be added to the migration queue.
	// We ignore LXRT_DAEMON and LXRT_TASK without RT-prio.
	if (IS_REALTIME_PRIO_NORMALIZED(current->prio) && (lx_rt_state & (RTLX_TASK | LXRT_TASK))) {
		rt_enqueue_migration_thread(&__raw_get_cpu_var(rt_migqueue), current);
	}

	// Adjust the runqueue cnt.
	rtx_adjust_rq_cnt();

	// Wakeup the LX thread
	INIT_LIST_HEAD(&req.list);
	req.task = current;
	__rtx_wake_lx_handling(&req);
    prev = rt_schedule(lx_rt_state); 	 

    // Now the thread executes in the LX-domain.
    // Execute the bottom half of the LX schedule.
    rtx_reenter_lx(prev);

	if (RT_MIGRATE_VERBOSE)	
		printkGen(NULL, "transition rt (rt_state=0x%04x preempt_count=%#x) -> lx (rt_state=0x%04x preempt_count=%#x)\n",
                rt_state, rt_pcount, current->rt_state, preempt_count());		

	if (rt_state & (RT_TASK_EXIT_PENDING | RT_TASK_MAIN))
	{
		if (current == rtx_get_cpu_member(rt_sync_migration, source) ||
				current == rtx_get_cpu_member(rt_sync_migration, target))
			__rtx_call_rt_domain(to_mg_virq);
		if (rt_state & RT_TASK_MAIN)
		{														// got called via kill_pg
			if (RT_SHUTDOWN_VERBOSE)
				printkGen("(mn)", "signal=%d received on cpu=%d executing in RT state=%#lx getting forced to shut down\n", rtx_get_cpu_var(sig_to_deliver), rtx_get_cpu_var(cpu_received_sig), rtx_get_cpu_var(mn_rt_state));
			rtx_shutdown_rt_process(0);
			do_group_exit(0);
		}
		else
			do_exit(0);
	}
}

/*
 * Execute a system call in the LX domain.
 */
asmlinkage long execute_in_lx(struct pt_regs regs)
{
	if (RT_SYSCALL_IN_LX_VERBOSE) 
		printkGen(NULL, "syscall id=%d delegated to LX\n", RTX_SYSCALL_ID());	
	rtx_migrate_to_lx(RTLX_TASK, NULL);
	return 0;	
}

int rt_task_leave(void)	
{
	current->rt_state2 |= RT_TASK_IN_EXIT;
	down_rt(&__raw_get_cpu_var(rt_tl_sem));
	local_irq_disable_hw();	
	list_del_init(&current->rt_pid_list);			// no new notifications can get created for that thread	 	
	local_irq_enable_hw();	
	rt_delete_event_notification(current);			// remove existing ones
	rt_delete_timer_notification(current);			// remove existing ones	
	current->rt_notify_state = 0;
	up_rt(&__raw_get_cpu_var(rt_tl_sem));

	rtx_migrate_to_lx(LXRT_TASK, NULL);
	return 0;
} 

/* 
* Called from interrupted user-mode context. 
* Do not synchronize with rt_tl_sem.
*/
int rt_task_leave_no_sync(void)	
{
	long flags;

	local_irq_save_hw(flags);	
	list_del_init(&current->rt_pid_list);			// no new notifications can get created for that thread	 	
	local_irq_restore_hw(flags);	
	rt_delete_event_notification(current);			// remove existing ones
	rt_delete_timer_notification(current);			// remove existing ones	
	current->rt_notify_state = 0;

	rtx_migrate_to_lx(LXRT_TASK, NULL);
	return 0;
} 

#ifdef CONFIG_LTT
int rtx_do_function_in_linux(void *function, unsigned par1, unsigned par2, unsigned par3)
{
	struct lx_cmd *cptr;
	
	if ((cptr = get_lx_request()) == NULL)
		return -EFAULT;
	cptr->_req._fctop.functionPtr = function;
        cptr->_req._fctop.par1 = par1;
        cptr->_req._fctop.par2 = par2;
        cptr->_req._fctop.par3 = par3;

	cptr->type = FUNCTION_IN_LX;
	add_interdomain_request((void*)cptr);

        return(0);
}
#endif

/*
 * This daemon either
 * -- executes FUTEX_WAKE (and FUTEX_CMP_REQUEUE) actions 
 * -- gets periodically called from LX CLOCK_REALTIME to synchronize it with RT
 * -- propagates the migration of threads from the LX- to the RT-domain 
 */
int interdomain_daemon_thread(void *arg)
{
	struct lx_cmd *req;
	struct list_head *p;
	struct fu *fp;
#ifdef CONFIG_LTT
        struct fct *lxFct;
#endif
	struct task_struct *tp;
	struct sched_param sched_par;
	int prio;

	strcpy(current->comm, INTERDOMAIN_DAEMON_NAME);
	sched_par.sched_priority = 99;								// must the highest to avoid priority inversion
	sched_setscheduler(current, SCHED_FIFO, &sched_par);
	current->rt_state = LX_DAEMON_STATE;
	rtx_set_cpu_var(interdomain_daemon, current);
	if (RT_INIT_VERBOSE)
		printkGen("(id)", "interdomain daemon installed prio=%d:%d\n", current->rt_priority, current->prio);	
	do
	{
		while (list_empty(&current->rt_proc->to_lx_list))
		{
			set_current_state(TASK_UNINTERRUPTIBLE);			
			schedule();
			if (rtx_get_cpu_var(rt_system_state) & RT_SHUTDOWN) {
				while (!list_empty(&current->rt_proc->to_lx_list)) {
					local_irq_disable_hw();
					p = current->rt_proc->to_lx_list.next;
					list_del_init(p);
					local_irq_enable_hw();
					req = list_entry(p, struct lx_cmd, req_link);
#ifdef CONFIG_SMP
					rtx_spin_lock(&futex_list_lock);
#endif
					list_add(&req->req_link, &aud_rt_hdr.free_list);	// recycle request element
#ifdef CONFIG_SMP
					rtx_spin_unlock(&futex_list_lock);
#endif
					local_irq_enable_hw();
				}
				goto rt_shutdown;
			}
		}
		local_irq_disable_hw(); 
		p = current->rt_proc->to_lx_list.next;
		list_del_init(p);
		local_irq_enable_hw(); 	
		req = list_entry(p, struct lx_cmd, req_link);				
		switch (req->type) {
			case GET_LX_TOD:
				rtx_synchronize_time(NULL);
				continue;
			case EXEC_FUTEX_LX:
				fp = &req->_req._fuop;
			    do_futex((u32 __user *)fp->uaddr, fp->op, fp->val, 0, (u32 __user *)fp->uaddr2, fp->val2, fp->val3);
				local_irq_disable_hw(); 
#ifdef CONFIG_SMP
				rtx_spin_lock(&futex_list_lock);
#endif
				list_add(&req->req_link, &aud_rt_hdr.free_list);	// recycle request element
#ifdef CONFIG_SMP
				rtx_spin_unlock(&futex_list_lock);
#endif
				local_irq_enable_hw(); 	
				continue;
			case MIGRATE_TO_RT:	
				tp = req->_req._proc;
				while (tp->se.on_rq)
				{
					set_current_state(TASK_UNINTERRUPTIBLE);
					schedule_timeout(FAST_POLL_TIME);
				}
				local_irq_disable_hw();
				INIT_LIST_HEAD(&tp->rt_run_list);			
				local_irq_enable_hw();
				/*
				 * rt_dyn_prio > 0:
				 * Adjust the priority of the current thread because
				 * of (de)boosting the owner thread of a PI mutex while
				 * the owner thread is running in LX.
				 * rt_dyn_prio < 0:
				 * Adjust the priority because of a dynamic prio change. 
				 * The priority can be a RT- or a LX-priority (negated).
				 */
				if (tp->rt_dyn_prio) {
					if (tp->rt_dyn_prio > 0) {
						tp->prio = tp->rt_dyn_prio;
						tp->rt_dyn_prio = 0;
					}
					else {
						prio = -tp->rt_dyn_prio;	// positive value
						if (IS_REALTIME_PRIO(prio)) {
							tp->rt_priority = prio;
							tp->normal_prio = (MAX_USER_RT_PRIO - 1) - tp->rt_priority;
							tp->prio = rt_futex_getprio(tp);
							if (tp->rt_dyn_policy != -1) {
								/* If rt_dyn_policy is set to SCHED_RR
							 	* we may have to setup the rr timer.
							 	* But this must be accomplished in the RT-domain.
							 	*/
								rt_set_policy(tp->rt_dyn_policy, tp);
							}
							tp->rt_dyn_prio = 0;
						}
						else {
							/* A LX-priority has to be handled in the RT-domain. */
							tp->rt_dyn_prio = prio;
						}		
					}
				}
				lx_wake_up_rt_thread_id(tp);
				up(&__raw_get_cpu_var(to_rt_op));				// allow for next migration
				continue;
#ifdef CONFIG_LTT
			case FUNCTION_IN_LX:
				lxFct = &req->_req._fctop;
				((int(*)(unsigned, unsigned, unsigned)) lxFct->functionPtr)
                                    (lxFct->par1, lxFct->par2, lxFct->par3);
				continue;
#endif
			default:
				printkGen(KERN_ALERT, "interdomain_daemon got wrong command type=%d\n", req->type);
		}	
	}
	while(1);
rt_shutdown:
	if (RT_SHUTDOWN_VERBOSE)					
		printkGen("(id)", "interdomain_daemon shutting down\n");
	rtx_set_cpu_var(interdomain_daemon, NULL);
	sched_par.sched_priority = 0;
	sched_setscheduler(current, SCHED_NORMAL, &sched_par);
	up(&__raw_get_cpu_var(to_rt_op));							// in case there are migrations waiting
	return 0;
}


/*
 * Just an initial test for switching between the LX- and the RT-domain
 * without doing system calls. Serves no other purpose.
 */
int rt_check_daemon_thread(void *arg)
{
	unsigned myCount = 0;
	struct sched_param sched_par;
	struct timespec maxtime;
	_INT64 timeout_max;
    struct rt_proc_hdr *pproc = NULL;
	
	_INT64 timeout = NSEC_TO_RT_JIFFIES(1000000000);				// 1sec

	maxtime.tv_sec = 10;
	maxtime.tv_nsec = 0;
	convert_to_internal(&timeout_max, &maxtime);
	INIT_LIST_HEAD(&current->rt_mig_list);

	strcpy(current->comm, CHECK_DAEMON_NAME);
	current->rt_state = LXRT_TASK;
	sched_par.sched_priority = aud_config.td_prio.val + 1;
	sched_setscheduler(current, SCHED_FIFO, &sched_par);
	set_current_state(TASK_UNINTERRUPTIBLE);
	schedule_timeout(FAST_POLL_TIME);
	if (rtx_migrate_to_rt(0))
	{
#ifdef CONFIG_RTX_THREAD_GROUP_SUPPORT
		INIT_LIST_HEAD(&current->rt_thread_list);
		INIT_LIST_HEAD(&current->rt_blocked_list);
#endif
		printkGen("(cd)", "check daemon installed prio=%d:%d\n", current->rt_priority, current->prio);		
		/* Note: This thread is created while dev_sem is already locked. */
		pproc = current->rt_proc = get_rt_proc(CHECK_DAEMON_ID);
		while (!(rtx_get_cpu_var(rt_system_state) & RT_SHUTDOWN)) {
			myCount++;
			printkGen(NULL, "check daemon looping cnt=%d\n", myCount);
			__set_current_rt_state(RT_TASK_UNINTERRUPTIBLE);
			rt_schedule_timeout(&__raw_get_cpu_var(isr_list_hdr), timeout);
			if (timeout < timeout_max)
				timeout += NSEC_TO_RT_JIFFIES(1000000000);				 			
		}
	}
	if (RT_SHUTDOWN_VERBOSE)
		printkGen("(cd)", "check daemon shutting down\n");
	if (pproc) {
		pproc->rt_check_daemon = NULL;
		current->rt_proc = NULL;
	}
	rtx_migrate_to_lx(LXRT_DAEMON, NULL);
	sched_par.sched_priority = 0;
	sched_setscheduler(current, SCHED_NORMAL, &sched_par);
	return 0;
}
#ifdef CONFIG_LTT

/*
 * we experience page faults when starting ltt. Some of these
 * faults cannot be handled with the mechanism provided within
 * the realtime domain (e.g., during rt_schedule or migration).
 * So we use a linux thread within the realtime application
 * to make a first ltt tracing entry (and resolve the page
 * faults within the linux domain).
 * Remark: page faults in the vmalloc area can be handled
 * within the realtime domain (see arch/mips/mm/fault.c)
 */

int ltt_enable_daemon(void *arg)
{
	unsigned myCount = 0;
	struct sched_param sched_par;
//	lttng_sequence_fs_exec_filename lttng_name;
	
        strcpy(current->comm, LTT_DAEMON_NAME);
	current->rt_state = LXRT_TASK;
	sched_par.sched_priority = 1;
	sched_setscheduler(current, SCHED_FIFO, &sched_par);
	aud_rt_hdr.ltt_daemon = current;

	while (!(rtx_get_cpu_var(rt_system_state) & RT_SHUTDOWN)) {

            myCount++;

            /* to prevent page faults within the realtime domain,
             * we enable realtime tracing after the first (Linux) thread of a
             * realtime application makes an entry in the trace buffer.
             */
            if (ltt_traces.suspendRtTraceFlag) {
                current->ltt_trace_id = actLttTraceId;
//                lttng_name.len = strlen(current->comm) + 1;
//                lttng_name.array = current->comm;
//                trace_fs_exec(&lttng_name);
                  trace_fs_exec(current->comm);
                
                ltt_traces.suspendRtTraceFlag = 0;
                printkGen(NULL, "realtime tracing enabled\n");
            }

            /* make sure this task will not be interrupted during debugging */
            /* (and run permanently)                                        */
            set_current_state(TASK_UNINTERRUPTIBLE);
            schedule_timeout(msecs_to_jiffies(1000));
        }

	aud_rt_hdr.ltt_daemon = NULL;
	return 0;
}        

#endif // CONFIG_LTT


