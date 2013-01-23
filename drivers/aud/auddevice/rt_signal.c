/*
 * drivers/aud/auddevice/rt_signal.c
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
 * This file contains all the signal functions of the rt kernel
 *
 */
#include <linux/aud/rt_timer.h>

struct nfb_pool {
	struct list_head free_list;
	struct notify rt_notify[NO_OF_SIGQUEUE_EL];
};
static DEFINE_PER_CPU(struct nfb_pool, nfb_pool);

#define PSEUDO_SIG  0x0f0f0f0f

/*
 * At least for realtime signals, elements with lower signal number should be served first
 * Since multiple sigq elements are a rare case. We take the hit here for sorting them in correctly
 * This routine is called under lock.
 */
void sigq_add_ordered(struct list_head *new, struct list_head *head)
{
	struct list_head *curr = head;
	struct sigqueue *q, *p;

	p = list_entry(new, struct sigqueue, list);
	for (curr = head->next; curr != head; curr = curr->next)
	{
		q = list_entry(curr, struct sigqueue, list);
		if (q->info.si_signo >=  p->info.si_signo)
			break;
	}
	list_add(new, curr);
}


/*
 * Dequeues the sigq element from whatever queue it is in
 */
int rt_dequeue_sigq (struct sigqueue* sigq)
{
	unsigned long flags;

	local_irq_save_hw(flags);
	list_del_init(&sigq->list);				// POSIX allows to cut corners
	local_irq_restore_hw(flags);
	return 0;
}

INLINE int copy_siginfo_to_user_rt(int overrun, siginfo_t *info, siginfo_t *src)
{
	int err = 0;
	int si_code = (int)((short)src->si_code);

	if (info == NULL)
		return(0);
	err = rt_copy_to_user(&info->si_signo, &src->si_signo, sizeof(src->si_signo));
	err |= rt_copy_to_user(&info->si_errno, &src->si_errno, sizeof(src->si_errno));
	err |= rt_copy_to_user(&info->si_code, &si_code, sizeof(src->si_code));
	err |= rt_copy_to_user(&info->si_int, &src->si_int, sizeof(src->si_int));
	err |= rt_copy_to_user(&info->si_tid, &src->si_tid, sizeof(src->si_tid));
	err |= rt_copy_to_user(&info->si_overrun, &overrun, sizeof(overrun));
	return err;
}

static int rt_dequeue_signal(struct task_struct *tsk, siginfo_t *info, sigset_t *set)
{
	int overrun;
	struct sigqueue *q;
	struct list_head *head, *curr;
	int sig;

	head = &tsk->rt_sigqueue_head;
	local_irq_disable_hw();

	for (curr = head->next; curr != head; curr = curr->next)
	{
		q = list_entry(curr, struct sigqueue, list);
		sig = q->info.si_signo;
		if (likely(sigismember(set, sig)))
		{
			list_del_init(curr);						// dequeue element
			overrun = q->info.si_overrun;				// in case of PREALLOC elements
			q->info.si_overrun = 0;						// reset it for overrun detection
			if (likely(set == &current->real_blocked))
				sigemptyset(&current->real_blocked);	// got woken up
	        local_irq_enable_hw();
			if ((sig == SIGTIMER) && (current->rt_carrier_thread_hdr != NULL) && (q->info.si_code == SI_LHDR)) // carrier thread ?
				return PSEUDO_SIG;
			if (copy_siginfo_to_user_rt(overrun, info, &q->info)) {
				q->info.si_signo = q->info.si_sys_sig;		// maybe we have to replace the overrun signal
			   return -EFAULT;
			}

			/* Note: q->info.si_signo may contain the signal sent to the monitoring thread
			 * while the "original" signal has been replaced. */
			q->info.si_signo = q->info.si_sys_sig;
			if (q->info.si_sys_cycle < 0)
				*q->info.si_sys_addr_cycle = 0;
			q->info.si_sys_ovpending = 0;


			if (likely(q->flags == SIGQUEUE_PREALLOC))
				return(sig);
	        local_irq_disable_hw();
			list_add(curr, &(__raw_get_cpu_var(nfb_pool).free_list));
	        local_irq_enable_hw();
			return sig;
		}
	}													// no signal found
	if (likely(set == &current->real_blocked))
	{													// thread was waiting
		sigemptyset(&current->real_blocked);
	    local_irq_enable_hw();
		return -EINTR;
	}
	current->real_blocked = *set;						// thread tried before going to wait
	__set_current_rt_state(RT_TASK_UNINTERRUPTIBLE);
	return 0;
}


asmlinkage  long sys_rt_sigtimedwait_rt(const sigset_t __user *uthese, siginfo_t __user *uinfo,
		    const struct timespec __user *uts, size_t sigsetsize)
{
	int error, sig;
	sigset_t waitset;
	struct timespec ts;
	_INT64 timeout64 = 0;
	struct notify *nfb;
	struct itimer *itmr;
	int flag;
	struct rt_list_hdr *hdr;
	siginfo_t infotmp, *pinfo;


	if (current->rt_state3 & RT_TASK_LX_DEBUG_PENDING) {
      // We migrated from LX to RT and got a debugging event
	  return -ERESTARTNOINTR;
	}

	if (rt_copy_from_user(&waitset, uthese, sizeof(waitset)))
		return -EFAULT;
	if ((current->rt_carrier_thread_hdr != NULL) && (sigismember(&waitset, SIGTIMER)))
	{
carrier_thread_handling:
		hdr = current->rt_carrier_thread_hdr;
 		local_irq_disable_hw();
		if ((itmr = get_timer_elapsed(hdr)))
		{
			nfb = (struct notify *)itmr->action_par;
			local_irq_enable_hw();
			if (RT_TIMER_VERBOSE)
				printkGen(NULL, "%s: carrier thread resuming for timerid=%d\n", __func__, nfb->sigq.info._sifields._timer._tid);
			pinfo = &nfb->sigq.info;
			// Special handling for sync-clock timers (marked with SI_LHDR).
			if (pinfo->si_code == SI_LHDR) {
				copy_siginfo(&infotmp, pinfo);
				infotmp.si_code = SI_TIMER;
				pinfo = &infotmp;
			}
			if (copy_siginfo_to_user_rt(0, uinfo, pinfo))
				return -EFAULT;
			return SIGTIMER;
		}
		local_irq_enable_hw();
	}
#ifndef CONFIG_RTX_OPTIMIZE_FOR_RELEASE
	else
		if (RT_SIGNAL_VERBOSE)
			printkGen(NULL, "%s: waiting for signal=%#llx\n", __func__, waitset);
#endif

	if (!(sig = rt_dequeue_signal(current, uinfo, &waitset)))
	{
		// No matching signal is queued, RT_TASK_UNINTERRUPTIBLE is set,
		// interrupts are disabled.
		flag = 0;
		if (uts) {
			// A check for a valid timespec is only needed
			// in case of waiting, POSIX allows for that.
			if (rt_copy_from_user(&ts, uts, sizeof(ts))) {
				__set_current_rt_state(RT_TASK_RUNNING);
				local_irq_enable_hw();
				return -EFAULT;
			}
			if ((error = convert_to_internal(&timeout64, &ts))) {
				__set_current_rt_state(RT_TASK_RUNNING);
				local_irq_enable_hw();
				return error;
			}
			flag = 1;
			if (!timeout64) {
				__set_current_rt_state(RT_TASK_RUNNING);
				local_irq_enable_hw();
				return -EAGAIN;
			}
		    // Note: The interrupts are enabled when returning from rt_schedule_timeout().
			timeout64 = rt_schedule_timeout(&__raw_get_cpu_var(isr_list_hdr), timeout64);
		}
		else {
		    // Note: The interrupts will be enabled when returning from rt_schedule().
			rt_schedule(0);				// RT_TASK_UNINTERRUPTIBLE is already set
		}
		sig = rt_dequeue_signal(current, uinfo, &current->real_blocked);

#ifndef CONFIG_RTX_OPTIMIZE_FOR_RELEASE
		if (RT_SIGNAL_VERBOSE)
			printkGen(NULL, "%s: after schedule signal=%d dequeued\n", __func__, sig);
#endif

		if (sig == -EFAULT)
			return sig;

		if (sig == PSEUDO_SIG) {
			goto carrier_thread_handling;
		}
		if (sig > 0) {
			return sig;								// valid signal number
		}
		if (flag && !timeout64) {
			return -EAGAIN;
		}
		// We have been interrupted (debugging).
		// Now we restart the system call.
		return -ERESTARTNOINTR;
	}

#ifndef CONFIG_RTX_OPTIMIZE_FOR_RELEASE
	if (RT_SIGNAL_VERBOSE)
		printkGen(NULL, "%s: signal=%d dequeued\n", __func__, sig);
#endif

	if (sig == PSEUDO_SIG) {
		goto carrier_thread_handling;
	}
	return sig;										// either signo or  -EINTR
}

/*
 * This is the path critical for response behavior. The function can be called from the ISR-level as well.
 * It assumes, that there will be no system call getoverrun. For events we don't have it anyway.
 * Therefore the overrun will always only be passed in the siginfo area.
 */
int rt_send_sigqueue(struct notify *nfb)
{
	struct sigqueue *q = &nfb->sigq;
	struct task_struct *p = nfb->thread;
	int sig;
	unsigned long flags;

	sig = q->info.si_signo;
#ifndef CONFIG_RTX_OPTIMIZE_FOR_RELEASE
	if (RT_SIGNAL_VERBOSE)
          printkGen(NULL, "%s: sig=%d for pid=%d\n", __func__, sig, p->pid);
#endif
	local_irq_save_hw(flags);
	if (likely(list_empty(&p->rt_sigqueue_head)))			// most likely situation
	{														// then q->list also must be empty
		__list_add(&q->list,&p->rt_sigqueue_head, &p->rt_sigqueue_head); 		// somewhat faster
	}
	else
	{
		if (!list_empty(&q->list))
		{													// sigq element is already queued in
			q->info.si_overrun++;
			goto out;
		}
		sigq_add_ordered(&q->list, &p->rt_sigqueue_head);	// ordering according to bit no as required by POSIX
	}
    /* In the RT-domain real_blocked is used for waiting.
	 * In the very unlikely case that the thread is executing
	 * under Linux it never leaves the kernel.
     */
	if (likely(sigismember(&p->real_blocked, sig)))	{
#ifdef CONFIG_RTX_RETURN_TO_LINUX
		if (R2L_CHECK_SIGNAL_DELIVERY(p) && !(list_empty(&p->rt_run_list)))
			rt_try_next_thread(p);
#endif
		rt_wake_up_thread(p);
		local_irq_restore_hw(flags);
		return 0;
	}
out:
#ifdef CONFIG_RTX_RETURN_TO_LINUX
	/* If there is a thread which voluntarily returned
	 * control to linux, make the RT-domain running again. */
	if (R2L_CHECK_SIGNAL_DELIVERY(p))
		rt_try_next_thread(p);
#endif
	local_irq_restore_hw(flags);
	return 0;
}


/* The function rt_send_alt_sigqueue() provides for supervising the timeliness of a thread to be notified.
 * The assumption is, that a monitor thread is waiting for some signal to be notified and will pick up the signal,
 * if the target thread is not already waiting. This may be the same signal the target thread is waiting for
 * or it may be another signal specified by sigevent_set_monitor().
 */
int rt_send_alt_sigqueue(struct notify *nfb)
{
	unsigned long flags;
	struct task_struct *p;

#ifndef CONFIG_RTX_OPTIMIZE_FOR_RELEASE
	if (RT_SIGNAL_VERBOSE)
	  printkGen(NULL, "%s: sig=%d for pid=%d\n", __func__, 
		  nfb->sigq.info.si_signo, nfb->alt_thread->pid);
#endif
	local_irq_save_hw(flags);

	/* Check if the monitor thread has already picked up the signal. 
     * Whenever the monitor thread is blocked while a new signal arrives,
     * we dequeue the signal from the monitor thread and increment the
     * overrun count. */
	if (nfb->sigq.info.si_sys_ovpending) {
		/* Dequeue signal from monitor thread. */
		list_del_init(&nfb->sigq.list);
		nfb->sigq.info.si_overrun++;
		nfb->sigq.info.si_sys_ovpending = 0;
		nfb->sigq.info.si_signo = nfb->sigq.info.si_sys_sig;		
	}

	nfb->cycle++;
	if (nfb->sigq.info.si_sys_cycle > 0) {
		if (nfb->cycle != nfb->sigq.info.si_sys_cycle) {
			goto to_norm;
		}
		nfb->cycle = 0;			// this cycle is monitored
	}
	else {
		/* si_sys_cycle < 0 */
		if ((sigismember(&nfb->thread->real_blocked, nfb->sigq.info.si_signo))) {
			nfb->cycle = 0;
			goto to_norm;
		}
		else {
			if (nfb->cycle > abs(nfb->sigq.info.si_sys_cycle)) {			// the cycle has to be monitored?
				goto check_monitor;
			}
			goto to_norm;
		}
	}

	/* It is a cycle that has to be monitored. */
	if (!(sigismember(&nfb->thread->real_blocked, nfb->sigq.info.si_signo))) {			// normal thread is not waiting?
check_monitor:
		if (unlikely(!list_empty(&nfb->sigq.list))) {
			nfb->sigq.info.si_overrun++;
			/* Send signal to monitor thread which may be waiting or not. */
			list_del_init(&nfb->sigq.list);												// dequeue the signal from normal thread
		}
	    local_irq_restore_hw(flags);

	    /* Send a signal to the monitoring thread. */
		p = nfb->thread;
		nfb->thread = nfb->alt_thread;
		nfb->sigq.info.si_signo = nfb->sigq.info.si_sys_ovsig;
		nfb->sigq.info.si_sys_ovpending = 1;
		rt_send_sigqueue(nfb);
		nfb->thread = p;
		return 0;
	}

to_norm:
	/* Send signal to normal thread. */
	local_irq_restore_hw(flags);
	return(rt_send_sigqueue(nfb));
}


asmlinkage long sys_rt_sigpending_rt(sigset_t __user *set, size_t sigsetsize)
{
	sigset_t pending_set;
	struct sigqueue *q;
	struct list_head *head, *curr;
	int sig;

	sigemptyset(&pending_set);

	head = &current->rt_sigqueue_head;
	local_irq_disable_hw();
	for (curr = head->next; curr != head; curr = curr->next)
	{
		q = list_entry(curr, struct sigqueue, list);
		sig = q->info.si_signo;
		sigaddset(&pending_set, sig);
	}
	local_irq_enable_hw();
	if (rt_copy_to_user(set, &pending_set, sizeof(*set)))
		return -EFAULT;
	return 0;
}

/*
 * initializes a pool of struct notify elements for use so that rt_do_kill() can use rt_send_sigqueue()
 */
void rt_init_nfb_pool(void)
{
	struct sigqueue *qp;
	int i;

	struct nfb_pool *pnfb = &__raw_get_cpu_var(nfb_pool);

	INIT_LIST_HEAD(&pnfb->free_list);
	for (i = 0; i < NO_OF_SIGQUEUE_EL; i++)
	{
		qp = &pnfb->rt_notify[i].sigq;
		qp->flags = 0;
		qp->info.si_errno = 0;
		qp->info.si_overrun = 0;
		qp->info.si_code = SI_TKILL;
		list_add(&qp->list, &pnfb->free_list);
	}
}

long rt_do_kill(int pid, int sig)
{
	struct task_struct *p;
	struct sigqueue *qp;
	struct notify *nfb;
	struct list_head *lh;

	// Scan the list without holding a semaphore.
	p = rt_find_task_by_pid(pid);

	if (p == NULL)
		return 0;
	
	// The task p may on its way to linux while we are proceeding.
	// However, the task is still accessible, as long as this code
	// is running.
	// We will queue this signal, but we will wake up the task only
	// when this task is in sys_rt_sigtimedwait_rt (see rt_send_sigqueue).
	if (sig == 0)
		return 0;
	if ((sig == SIGTERM) || (sig == SIGABRT) || (sig == SIGKILL) || (sig == SIGCANCEL))
	{
		if (p == current)
		{
			rtx_migrate_to_lx(RTLX_TASK, NULL);
			return 0;
		}
		down_rt(&__raw_get_cpu_var(rt_tl_sem));
		rt_mark_exit_pending(p);
		up_rt(&__raw_get_cpu_var(rt_tl_sem));
		return 0;
	}
	/* Note: SIGTIMER is the first valid RT-signal. */
	if ((sig >= SIGTIMER) && (sig <= SIGRTMAX))
	{
	    local_irq_disable_hw();
		if (list_empty(&(__raw_get_cpu_var(nfb_pool).free_list)))
		{
	        local_irq_enable_hw();
			return -EAGAIN;
		}
		lh = rtx_get_cpu_member(nfb_pool, free_list.next);
		list_del_init(lh);
	    local_irq_enable_hw();
		nfb = (struct notify *)list_entry(lh, struct notify, sigq.list);
		nfb->thread = p;
		qp = &nfb->sigq;
		qp->info.si_pid = current->tgid;
		qp->info.si_uid = current_uid();
		qp->info.si_signo  = sig;
		qp->info.si_threadid = current->rt_threadid;
		qp->info.si_code = SI_USER;
		rt_send_sigqueue(nfb);
		rt_schedule(0);
		return 0;
	}
	return -EINVAL;
}

asmlinkage long sys_rt_tkill(int pid, int sig)
{
	return rt_do_kill(pid, sig);
}

/*
 * tgkill does only additional checking for group membership.
 * Since all threads found in the rt_thread_list belong to the same group, the
 * execution in the realtime domain is identical.
 */
asmlinkage long sys_rt_tgkill(int tgid, int pid, int sig)
{
	return rt_do_kill(pid, sig);
}

