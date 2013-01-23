/*
 * drivers/aud/auddevice/rt_io_event.c
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
 * This file provides all the functions needed for RT capable
 * notification for IO-events and timers through a sigevent based API.
 * For timers both notification types (SIGEV_THREAD and the one set by sigevent_set_notification) 
 * are implemented by sending a signal to a thread (SIGEV_THREAD_ID/SIGEV_SIGNAL_SV).
 * For IO-events only the notification type set by sigevent_set_notification is done through sending a signal.
 * The implementation of SIGEV_THREAD is based on ioctls. 
 * No process wide signalling is supported.
 * 
 * Notification targets:
 * For IO-events:
 * - Threads of the RT-domain
 * - Threads of the LX-domain (also threads of regular LX processes)
 * For timers:
 * CLOCK_REALTIME and CLOCK_MONOTONIC:
 * - Threads of the RT domain
 * CLOCK_SYNC:
 * - Threads of the RT-domain
 * - If the RT-doman is not active, then threads within the RT-priority range of the RT-process
 *
 * All the necessary distinction between the different targets is made during setup of an notification
 * by inserting the matching function pointer.
 */
#include <linux/aud/rt_timer.h>

static struct semaphore evar_sem;
static atomic_t evar_sem_init = ATOMIC_INIT(1);

static struct rt_evar_hdr evar_list[RT_EVAR_MAX];

int noop(void *arg)
{
	return 0;
}

/*
 * It is only needed for signal based notification in the LX-domain.
 * Note: HW interrupts must be off.
 */
int lx_notify_thread(void *arg)
{
	struct notify *nfb = (struct notify*) arg;
	int ret = -1;

#ifndef CONFIG_RTX_OPTIMIZE_FOR_RELEASE
	if (RT_EVENT_VERBOSE) {
		if (nfb->thread) {
          		printkGen(NULL, "%s: sig=%d for pid=%d\n", __func__,
                                  nfb->sigq.info.si_signo, nfb->thread->pid);
		}
        else {
          	printkGen(NULL, "%s: sig=%d\n", __func__,
          				nfb->sigq.info.si_signo);
        }
	}
#endif

	if (list_empty(&nfb->sigq.list))
	{
		nfb->sigq.info.si_overrun = 0;
		local_irq_enable_hw();
		if (nfb->thread)
			ret = send_sigqueue(&nfb->sigq, nfb->thread, nfb->shared);
		local_irq_disable_hw();
		return ret;
	}
	nfb->sigq.info.si_overrun++;
	return 1;
}

/*
 * This routine is called when a device driver detects that the event "event" has occurred.
 * The function ev_action may be one of the signal functions (rt_send_sigqueue, rt_send_alt_sigqueue, propagate_to_proxy)
 * or in case of SIGEV_THREAD for IO-events one of the wake_up functions below. In this case
 * the distinction between private and shared is made for performance reasons. The case where a carrier serves
 * only one event of a device (one handler function) should be as fast as possible, ideally as fast as a handmade
 * ioctl which waits for a device interrupt.
 */
int rt_send_event(struct rt_event *evpr)
{
	return evpr->ev_action(&evpr->rt_notify);
}

int rt_wake_up_private_carrier(void *arg)
{
	struct notify *nb = (struct notify *)arg;

	nb->ev_notify |= MASTER_EVENT;
	if (nb->ev_notify & CARRIER_WAITS)
		return rt_wake_up_thread(nb->thread);
	return 0;
}

int rt_wake_up_shared_carrier(void *arg)
{
	struct notify *snb = (struct notify *)arg;
	struct notify *nb = &snb->ev_master->rt_notify;

	nb->ev_notify |= SLAVE_EVENT;
	snb->ev_notify |= SLAVE_EVENT;
	if (nb->ev_notify & CARRIER_WAITS)
		return	rt_wake_up_thread(nb->thread);
	return 0;
}


/*
 * In this case a LX carrier thread serves the event.
 */
int wake_up_private_carrier(void *arg)
{
	struct notify *nb = (struct notify *)arg;

	nb->ev_notify |= MASTER_EVENT;
	if (nb->ev_notify & CARRIER_WAITS)
		return wake_up_process(nb->thread);
	return 0;
}

int wake_up_shared_carrier(void *arg)
{
	struct notify *snb = (struct notify *)arg;
	struct notify *nb = &snb->ev_master->rt_notify;

	nb->ev_notify |= SLAVE_EVENT;
	snb->ev_notify |= SLAVE_EVENT;
	if (nb->ev_notify & CARRIER_WAITS)
		return wake_up_process(nb->thread);
	return 0;
}

/* Checks sigevent structure for valid specifications and returns thread where to send the signal.
 * In case rt_check_sigevent is called in the RT domain, the lock rt_task_list_sem is already taken.
 * If called from the LX-domain the task list lock must be taken here.
 */
struct task_struct *rt_check_sigevent(int notify, pid_t pid, int signo)
{

	struct task_struct *rtn = current->group_leader;

	// Assumes that SIGEV_SIGNAL_SV incorporates the SIGEV_THREAD_ID bit.
	if (notify & SIGEV_THREAD_ID) {
		if (IS_REALTIME) {
		    	if (!(rtn = rt_find_task_by_pid(pid)))
				return NULL;
		}
		else {
			read_lock_irq(&tasklist_lock);
			rtn = find_task_by_vpid(pid);
			read_unlock_irq(&tasklist_lock);
		    	if (!rtn ||(rtn->tgid != current->tgid))
					return NULL;
		}
	}
	if ((notify & ~(SIGEV_SIGNAL_SV |SIGEV_NONE)) || ((signo < 0) || (signo > SIGRTMAX)) ||
	    ((notify == (SIGEV_SIGNAL_SV)) && (signo < SIGRTMIN))) {
		return NULL;
	}
	return rtn;
}

/*
 * Initialize evar_sem needs to be a singleton pattern since 
 * rt_init/destroy_event_area may be called by a device driver
 * prior to the initialization of the RTX driver.
 */
static void init_evar_sem(void)
{
	if (atomic_dec_and_test(&evar_sem_init)) {
		sema_init(&evar_sem, 1);
	}
}

/*
 * This routine has to be called when a device driver initializes.
 * It initializes the ev_list in the driver and assigns a header element of the evar_list to it.
 */
int rt_init_event_area(struct rt_event *evpr, int no_of_events)
{
	int i, ix, ret;

	init_evar_sem();
	down(&evar_sem);
	ret = -ENOMEM;
	for (ix = 0; ix < RT_EVAR_MAX; ix++)
	{
		if (evar_list[ix].evpr == NULL)
		{
			evar_list[ix].ev_no = no_of_events;
			evar_list[ix].no_of_proxy_user = 0;
			evar_list[ix].sum_cnt = 0;
			evar_list[ix].ev_proxy = NULL;
			evar_list[ix].evpr = evpr;
			sema_init(&evar_list[ix].proxy_sem, 0);
			sema_init(&evar_list[ix].sync_sem, 1);
			for (i = 0; i < no_of_events; i++, evpr++)
			{
				INIT_LIST_HEAD(&evpr->ev_link);
				atomic_set(&evpr->state, EV_EMPTY);
				evpr->ev_action = noop;
				evpr->hdpl = &evar_list[ix];
			}
			ret = ix;
			break;
		}
	}
	up(&evar_sem);
	return(ret);
}

/* 
 * Within the RT domain a signal can be sent directly from the ISR waking up a waiting thread. 
 * Within the LX-domain no signaling can be done directly from an ISR (non threaded ISRs assumed).
 * For this situation a proxy is introduced to do the signaling.
 */
int propagate_to_proxy(void *arg)
{
	struct notify *nfb = (struct notify *)arg;
	struct rt_evar_hdr *evar;
	struct rt_event *evpr;
	unsigned long flags;

	evpr = list_entry(nfb, struct rt_event, rt_notify);
	evar = evpr->hdpl;
	if (evar->ev_proxy == NULL)
	{
		printkGen(KERN_ALERT, "%s: rt_send_event failed: no proxy event=%#x\n", __func__, evpr->ev_id);
		return -EINVAL;
	}
	local_irq_save_hw(flags);
	evar->sum_cnt++;
	nfb->ev_notify++;
	local_irq_restore_hw(flags);
	if (evar->sum_cnt >= 1) {
		// Let the proxy do the signalling.
		wake_up_process(evar->ev_proxy);
	}
	return 0;
}

/*
 * Sets up all what is needed for timer or IO-event signalling.
 * For RT the routine is called under lock rt_tl_sem.
 * For IO-events LX and RT can call it. Protection is provided by the EV_BUSY state of the entry.
 * Is not called for IO-events with notification type SIGEV_THREAD and SIGEV_NONE.
 */
int rt_setup_notification(void **action_ptr, int id, struct sigevent *rt_sig, struct notify *npr, int sicode)
{
	struct task_struct *ov_thread = current;	// default

	init_rt_notify(npr, sicode);
	npr->sigq.info.si_sys_sig = rt_sig->sigev_signo;
	npr->sigq.info.si_sys_ovsig = rt_sig->sigev_signo;

	/* Check for a sigevent_set_monitor() call. */
	if ((rt_sig->sigev_notify == SIGEV_SIGNAL_SV) && (rt_sig->si_cycle != 0)) {
		if (rt_sig->si_ovtid != USE_CREATOR) {
			/* Try to get the overrun thread. */
			if ((ov_thread = rt_check_sigevent(rt_sig->sigev_notify, rt_sig->si_ovtid, rt_sig->si_ovsig)) == NULL)
				return -EINVAL;
			npr->sigq.info.si_sys_ovsig = rt_sig->si_ovsig;
		}
		/* The monitoring thread must be different from normal thread. */
		if (ov_thread == npr->thread)
			return -EINVAL;
	}
	else {
		/* Check for an invalid sigevent_set_monitor() call. */
		if (rt_sig->si_ovtid == INVALID_CREATOR)
			return -EINVAL;
	}

	npr->sigev_notify = rt_sig->sigev_notify;
	npr->sigq.info.si_signo = rt_sig->sigev_signo;
	npr->sigq.info.si_tid = id;
	if (rt_sig->sigev_notify == SIGEV_THREAD_ID)
		npr->sigq.info.si_ptr = rt_sig->sigev_value.sival_ptr;
	else {
		// SIGEV_SIGNAL_SV needs cycles/cycle-num.
		if (rt_sig->sigev_notify == SIGEV_SIGNAL_SV) {
			npr->sigq.info.si_ptr = rt_sig->sigev_value.sival_ptr;
			npr->sigq.info.si_sys_cycle = rt_sig->si_cycle;
			npr->sigq.info.si_sys_addr_cycle = &npr->cycle;
		}
	}

	if (rt_sig->sigev_notify == SIGEV_NONE) {
		*action_ptr = noop;
		return 0;
	}
	if (IS_REALTIME)
	{
		*action_ptr = noop;
		if (rt_sig->sigev_notify == SIGEV_THREAD_ID)
			*action_ptr = rt_send_sigqueue;
		else {
			if (rt_sig->sigev_notify == (SIGEV_SIGNAL_SV))
			{
				if (rt_sig->si_cycle == 0)
					*action_ptr = rt_send_sigqueue;
				else
				{
					*action_ptr = rt_send_alt_sigqueue;
					npr->alt_thread = ov_thread;
				}
			}
		}
		return 0;
	}
	*action_ptr = propagate_to_proxy;		// default for LX domain
	return 0;
}

/*
 * Proxy kernel daemon for the LX domain used for sending ISR based signals.
 */
int proxy_send_event_thread(void *arg)
{
	struct rt_evar_hdr *evar = (struct rt_evar_hdr *)arg;
	struct rt_event *evpr;
	struct sched_param sched_par;
	int i;

	if (current->rt_state && current->rt_state != LXRT_TASK) {
		printkGen(KERN_ALERT, "%s daemon cannot be installed (pid=%d rt_state=%#lx)\n", __func__, current->pid, current->rt_state);
		return(0);
	}

	strcpy(current->comm, EVENT_PROXY_NAME);
	sched_par.sched_priority = 99;					// must be the highest to avoid priority inversion
	current->rt_state3 |= RT_TASK_ALLOW_SETSCHEDULER;
	sched_setscheduler(current, SCHED_FIFO, &sched_par);

	if (RT_EVENT_VERBOSE)
		printkGen(NULL, "%s daemon installed prio=%d:%d\n", __func__, current->rt_priority, current->prio);
	daemonize("event_proxy");						// its life is independent of the creators life
	evar->ev_proxy = current;
	up(&evar->proxy_sem);							// continue thread which caused proxy installation
	__set_current_state(TASK_UNINTERRUPTIBLE);

	while (1)
	{
		schedule();
		if (current != evar->ev_proxy)  			// set to NULL if event area is going to be destroyed
			break;
		local_irq_disable_hw();
		while (evar->sum_cnt)
		{
			for (i = 0, evpr = evar->evpr; i < evar->ev_no; i++, evpr++)
				if (evpr->rt_notify.ev_notify)
				{
					lx_notify_thread(&evpr->rt_notify);  // lx_notify_thread enables/disables local irqs
					evpr->rt_notify.ev_notify--;
					evar->sum_cnt--;
					if (!evar->sum_cnt)
						break;
				}
		}
		__set_current_state(TASK_UNINTERRUPTIBLE);
		local_irq_enable_hw();
	}
	up(&evar->proxy_sem);							// to resume the thread which woke up the proxy for suicide
	if (RT_EVENT_VERBOSE)
		printkGen(NULL, "%s daemon shutting down\n", __func__);
	sched_par.sched_priority = 0;
	current->rt_state3 |= RT_TASK_ALLOW_SETSCHEDULER;
	sched_setscheduler(current, SCHED_NORMAL, &sched_par);
	current->exit_signal = -1;
	return(0);
}

/*
 * This function is not protected by a blocking lock. It protects itself by the busy state.
 */
int rt_unregister_event(struct rt_event *evpr)
{
	struct rt_event *ep, *mevpr;
	struct list_head *sl;
	struct rt_evar_hdr *evar;
	struct task_struct *proxy;
	unsigned long flags;
	int state;
	int error = -EINVAL;

	if ((state = atomic_cmpxchg(&evpr->state, EV_REGISTERED, EV_BUSY)) == EV_REGISTERED)
	{
		if (RT_EVENT_VERBOSE)
			printkGen(NULL, "unregister_event  evpr: %#p event: %#x\n",  evpr, evpr->ev_id);
		if (current->tgid == evpr->rt_notify.thread->tgid)				// only thread of registering process may unregister
		{
			if (evpr->ev_disable)
				evpr->ev_disable(evpr->endisable_par, evpr);
			if (evpr->rt_notify.sigev_notify != SIGEV_THREAD_ID)
			{
				// May be the sigqueue element is still queued.
				if (evpr->ev_action == propagate_to_proxy) {
					if (IS_LINUX) {
						/* We are running in LX and the thread is LX based. */
						spin_lock_irq(&current->sighand->siglock);
						list_del_init(&evpr->rt_notify.sigq.list);			// POSIX allows to cut corners
						spin_unlock_irq(&current->sighand->siglock);
						evar = evpr->hdpl;
						if (!(--evar->no_of_proxy_user))
						{
							evar->sum_cnt = 0;
							if ((proxy = evar->ev_proxy))
							{
								evar->ev_proxy = NULL;					// termination criteria for proxy thread
								wake_up_process(proxy);					// wake up proxy for suicide
								down(&evar->proxy_sem);					// to be continued by proxy thread
							}
							else
								printkGen(KERN_ALERT,"%s: error event proxy setup\n", __func__);
						}
					}
					else {
						/* We are running in RT, but the event thread is LX based.
						 * Therefore we have to prevent the action to be called
						 * for now. But we do the real thing later from LX (see above). */
						evpr->ev_action = noop;
						atomic_set(&evpr->state, state);
						return 0;
					}

				}
				else {
					/* The event thread is RT based:
					 * The previous test: if (evpr->rt_notify.thread->rt_notify_state & RT_EVENT_REGISTERED
					 * is usually called from RT, but can also be called from LX. */
					rt_dequeue_sigq(&evpr->rt_notify.sigq);
				}
			}
			else
			{
				local_irq_save_hw(flags);
				if(!(list_empty(&evpr->ev_link)))
				{															
					// There are slaves to be handled.
					sl = evpr->ev_link.next;
					list_del_init(&evpr->ev_link);
					if (evpr->rt_notify.ev_master == evpr)
					{														
						// It was a master.
						mevpr = list_entry(sl, struct rt_event, ev_link);	// the next slave becomes the new master
						mevpr->ev_action = evpr->ev_action;					// may be wake up private carrier
						mevpr->rt_notify.ev_master = mevpr;
						mevpr->rt_notify.ev_notify = evpr->rt_notify.ev_notify;
						evpr->rt_notify.thread->rt_mevpr = mevpr;

						// In case there are additional slaves.
						for (sl = sl->next; sl != &mevpr->ev_link; )
						{													
							// Reference to the new master.
							ep = list_entry(sl, struct rt_event, ev_link);
							ep->rt_notify.ev_master = mevpr;
							sl = sl->next;
						}
					}
				}
				local_irq_restore_hw(flags);
			}
			evpr->ev_action = noop;
			state = EV_EMPTY;
			error = 0;
		}
	}
	atomic_set(&evpr->state, state);
	return 0;
}


/*
 * This routine is called from the device driver as result of an event_create(fd,..., ...) call
 * which gets converted inside the library to an ioctl(fd, AuD_EVENT_CREATE,...).
 * The arg-pointer is assumed to point to kernel space, meaning the calling device driver 
 * has to copy it from user space.
 */
int do_rt_register_event(int idx, struct rt_ev_desc *arg )
{
	struct rt_evar_hdr *evar;
	struct task_struct *proxy;
	struct notify *nfb;
	sigevent_t *sigev_ptr;
	struct rt_event *evpr, *mevpr;
	int event;
	int error = -EINVAL;
	int ix, state;
	int pid;

	if ((arg == NULL) || (idx >= RT_EVAR_MAX))
		return -EINVAL;
	event = arg->event;
	evar = &evar_list[idx];
	evpr = evar->evpr;
	for (ix = 0; ix < evar->ev_no; ix++, evpr++)
		if ((event == evpr->ev_id))
			break;
	if (ix == evar->ev_no)
	{
		if (RT_EVENT_VERBOSE)
			printkGen(NULL, "%s: evar=%#p no_of_events=%d event=%#x not found\n", __func__, evar, evar->ev_no, event);
		return -EINVAL;
	}
	sigev_ptr = &arg->sigevent;
	if (sigev_ptr->sigev_notify == SIGEV_NONE)
		return rt_unregister_event(evpr);
	if (RT_EVENT_VERBOSE)
		printkGen(NULL, "%s: evpr=%#p event=%#x par=%#p\n", __func__, evpr, event, sigev_ptr->sigev_value.sival_ptr);
	nfb = &evpr->rt_notify;

	// Check for a good sigevent structure.
	if ((nfb->thread = rt_check_sigevent(sigev_ptr->sigev_notify, sigev_ptr->sigev_notify_thread_id, sigev_ptr->sigev_signo)) == NULL)
	{
		if (RT_EVENT_VERBOSE)
			printkGen(NULL, "%s: bad sigevent structure signo=%#x, pid=%d\n", __func__, sigev_ptr->sigev_signo, (unsigned)sigev_ptr->sigev_notify_thread_id);
		return -EINVAL;
	}
	error = -EINVAL;
	if ((state = atomic_cmpxchg(&evpr->state, EV_EMPTY, EV_BUSY)) == EV_EMPTY)
	{																	
		// Further register/unregister events are locked out.
		if (sigev_ptr->sigev_notify == SIGEV_SIGNAL_SV)
		{																
			// Signal based notification.
			if (current->rt_state == RTLX_TASK) {
				printkGen(KERN_ALERT, "%s: bad thread state - RT thread pid=%d is currently executing in LX\n", __func__, current->pid);
				goto err_out;
			}
			if (IS_LINUX)
			{
				down(&evar->sync_sem);
				if (evar->ev_proxy == NULL)
				{															
					// Signals from ISR cannot be sent in LX.
					// Create proxy for sending signals.
					if ((pid = kernel_thread(proxy_send_event_thread, evar, CLONE_KERNEL | CLONE_THREAD)) <= 0) {
						up(&evar->sync_sem);
						goto err_out;
					}
					down(&evar->proxy_sem);									// wait for the proxy to be ready
					error = rt_setup_notification((void**)&evpr->ev_action, evpr->ev_id, sigev_ptr, nfb, SI_IOEVENT);
					/* In case of an error we have to shut down the proxy thread. */
					if (error) {
						proxy = evar->ev_proxy;
						evar->ev_proxy = NULL;
						wake_up_process(proxy);
						down(&evar->proxy_sem);		// wait for the proxy to be shut down
						up(&evar->sync_sem);
						goto err_out;
					}
				}
				else {
					if ((error = rt_setup_notification((void**)&evpr->ev_action, evpr->ev_id, sigev_ptr, nfb, SI_IOEVENT))) {
						up(&evar->sync_sem);
						goto err_out;
					}
				}
				up(&evar->sync_sem);
				evar->no_of_proxy_user++;
			}
			else {
				if ((error = rt_setup_notification((void**)&evpr->ev_action, evpr->ev_id, sigev_ptr, nfb, SI_IOEVENT)))
					goto err_out;
			}
		}
		else
		{																
			// Handle ioctl based notification.
			if ((evpr->rt_notify.sigev_notify = sigev_ptr->sigev_notify) != SIGEV_THREAD_ID)
				goto err_out;
			evpr->rt_notify.ev_notify = 0;
			nfb->sigq.info.si_ptr = sigev_ptr->sigev_value.sival_ptr;	// value to be returned by ioctl(... WAIT_FOR_INT, ..)

			// A carrier may serve more than one event of a device, but it can wait at only one event structure.
			// Therefore the notion of a master and a slave event resp. the notion of a private and a shared carrier.
			if (!(mevpr = (struct rt_event *)nfb->thread->rt_mevpr))
			{															
				// It is the first event for this carrier. 
				// In case another event will register with the same carrier, this event will act as master event.
				if (IS_REALTIME)
					evpr->ev_action = rt_wake_up_private_carrier;
				else
					evpr->ev_action = wake_up_private_carrier;
				evpr->rt_notify.ev_master = evpr;
				nfb->thread->rt_mevpr = evpr;					// now the new thread is marked as carrier
				if (RT_EVENT_VERBOSE)
					printkGen(NULL, "%s: new carrier hdr=%#p pid=%d  rt_state=%#x\n", __func__, evpr, nfb->thread->pid, nfb->thread->rt_state);
			}
			else
			{															
				// It is a carrier shared with a master event.
				if (IS_REALTIME)
					evpr->ev_action = rt_wake_up_shared_carrier;
				else
					evpr->ev_action = wake_up_shared_carrier;
				evpr->rt_notify.ev_master = mevpr;
				local_irq_disable_hw();								// adjacent elements are not protected
				list_add(&evpr->ev_link, &mevpr->ev_link);
				local_irq_enable_hw();
			}
		}
		error = 0;
		state = EV_REGISTERED;
		if (IS_REALTIME) {
			nfb->thread->rt_notify_state |= RT_EVENT_REGISTERED;
			if (nfb->alt_thread)
				nfb->alt_thread->rt_notify_state |= RT_EVENT_REGISTERED;
		}
		if (evpr->ev_enable)
			evpr->ev_enable(evpr->endisable_par, evpr);
err_out:
		atomic_set(&evpr->state, state);
	}
	return error;
}

/*
 *  For the LX-domain no overall locking is needed.
 *  For the RT domain it must get synchronized with getting a thread in/out of the RT thread list.
 */
int rt_register_event(int evar_index, struct rt_ev_desc *arg)
{
	int result;

	if (IS_REALTIME)
		down_rt(&__raw_get_cpu_var(rt_tl_sem));
	result = do_rt_register_event(evar_index, arg);
	if (IS_REALTIME)
		up_rt(&__raw_get_cpu_var(rt_tl_sem));
	return result;
}


/*
 * This routine is called when the device driver receives an ioctl(.., WAIT_FOR_EVENT, ...) from a carrier thread.
 * Waiting thread and ISR are assumed to always execute on the same processor core.
 */
int rt_wait_for_event(void)
{
	struct list_head *sl;
	struct rt_event *evpr;
	struct rt_event *mevpr = current->rt_mevpr;

	if (unlikely(mevpr == NULL))
	{
		printkGen(KERN_ALERT,"%s: no carrier thread\n", __func__);
		return -EFAULT;
	}
	local_irq_disable_hw();

	while(1)
	{
		if (!(mevpr->rt_notify.ev_notify))
		{
			// No event needs to be served, therefore carrier will wait.
			mevpr->rt_notify.ev_notify = CARRIER_WAITS;
			if (likely(IS_REALTIME))
			{
				rt_schedule(RT_TASK_UNINTERRUPTIBLE);
				mevpr = current->rt_mevpr;			// the master element may have changed
			}
			else
			{
				set_current_state(TASK_INTERRUPTIBLE);
				local_irq_enable_hw();
				schedule();
				mevpr = current->rt_mevpr;			// the master element may have changed
				if (mevpr->rt_notify.ev_notify == CARRIER_WAITS)
					return -EINTR;					// got a signal for termination
			}
			local_irq_disable_hw();					// rt_schedule enables when leaving
		}

		if (likely(mevpr->rt_notify.ev_notify & MASTER_EVENT))
		{
			mevpr->rt_notify.ev_notify &= SLAVE_EVENT;	// clears both other bits
			local_irq_enable_hw();
			return mevpr->rt_notify.sigq.info.si_int;
		}
		
		// Check slaves completes in a subsequent run.
		for (sl = mevpr->ev_link.next; sl!= &mevpr->ev_link;)
		{
			evpr = list_entry(sl, struct rt_event, ev_link);
			if (evpr->rt_notify.ev_notify)
			{
				evpr->rt_notify.ev_notify = 0;			// only slave bit can be set
				local_irq_enable_hw();
				return evpr->rt_notify.sigq.info.si_int;
			}
			sl = sl->next;
		}												// no slave element has a slave bit set
		mevpr->rt_notify.ev_notify = 0;
	}
	return 0;
}


/*
 * If a priority change calls for a migration to LX, it can only take place if there are no signal based notifications
 * in the RT domain for this thread. This routine is called while holding the rt_tl sem.
 */
int rt_event_notification(struct task_struct *p)
{
	int i, ix;
	struct rt_event *evpr;

	for (ix = 0; ix < RT_EVAR_MAX; ix++)
	{
		if ((evpr = evar_list[ix].evpr) == NULL)
			continue;
		for (i = 0; i < evar_list[ix].ev_no; i++, evpr++)
			if (atomic_read(&evpr->state) == EV_REGISTERED)
				if ((evpr->rt_notify.thread == p) || (evpr->rt_notify.alt_thread == p))
					return 1;		// event notifications for this thread
	}
	return 0;			// no event notification for this thread
}

/*
 * This routine unregisters events which have a signal based notification channel to the specified task p.
 * It is called when either a thread in the RT-domain terminates individually or when the RT-domain is shut down.
 * It is also called whenever a regular LX thread terminates. Therefore it can be called from both domains.
 * Data consistency is maintained without locking. If called from the RT-domain, the rt_tl_sem is hold
 * in order to synchronize with adding or deleting a thread to/from the rt_pid_list.
 */
void rt_delete_event_notification(struct task_struct *p)
{
	int i, ix;
	struct rt_event *evpr;

	for (ix = 0; ix < RT_EVAR_MAX; ix++)
	{
		if ((evpr = evar_list[ix].evpr) == NULL)
			continue;
		for (i = 0; i < evar_list[ix].ev_no; i++, evpr++)
			if (atomic_read(&evpr->state) == EV_REGISTERED)
				if ((evpr->rt_notify.thread == p) || (evpr->rt_notify.alt_thread == p))
					rt_unregister_event(evpr);
	}
	return;
}

/*
 * The function rt_destroy_event_area is to be called when a driver unloads.
 */
int rt_destroy_event_area(int idx)
{
	init_evar_sem();
	if (idx < 0 || idx >= RT_EVAR_MAX)
		return -EINVAL;
	down(&evar_sem);
	evar_list[idx].evpr = NULL;
	up(&evar_sem);
	return 0;
}

EXPORT_SYMBOL(rt_send_event);
EXPORT_SYMBOL(rt_register_event);
EXPORT_SYMBOL(rt_init_event_area);
EXPORT_SYMBOL(rt_destroy_event_area);
EXPORT_SYMBOL(rt_wait_for_event);

