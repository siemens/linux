/*
 * drivers/aud/auddevice/rt_sync_timer.c
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
 * This file contains the routines for the CLOCK_SYNC Timers extension of the LINUX kernel
 * and the RT-Kernel. It also contains the registration of the CLOCK_SYNC in both domains.
 */
#include <linux/aud/rt_timer.h>

struct semaphore scl_sem;
int init_timer_done;

static int sync_timer_settime(struct posix_timer *, int, struct itimerspec *, struct itimerspec *);
static void sync_timer_gettime(struct posix_timer *, struct itimerspec *);
static int sync_timer_delete(struct posix_timer *);
static int sync_timer_create(struct posix_timer *, sigevent_t *);

static int lx_sync_timer_settime(struct k_itimer *, int, struct itimerspec *, struct itimerspec *);
static void lx_sync_timer_gettime(struct k_itimer *, struct itimerspec *);
static int lx_sync_timer_delete(struct k_itimer *);
static int lx_sync_timer_create(struct k_itimer * );

static int clock_sync_settime(clockid_t, struct timespec *);
static int clock_sync_gettime(clockid_t, struct timespec *);
static int clock_sync_getres(clockid_t, struct timespec *);

static void clock_sync_tick_0(void);
static void clock_sync_tick_1(void);
static void clock_sync_tick_2(void);

static int return_error(void);
static int wake_up_proxy(void *);
int sync_nsleep (clockid_t, int, struct timespec *, struct timespec __user * );   // see posix-timers.h

extern int hrtimer_get_res(const clockid_t which_clock, struct timespec *tp);

#define MAX_SYNC_CLOCKS			3			// maximum number of sync_clocks to get registered
#define ONE_SHOT				0x1			// oneshot timer		

DEFINE_PER_CPU(_INT64, act_clock);
DEFINE_PER_CPU(struct rt_list_hdr, isr_list_hdr);

/* 
 * For the RT domain one rtg_clock structure is maintained. 
 * When registering the pointer to the structure is passed.
 * For the Linux domain a template k_clock structure is maintained. 
 * When registering the template is copied. 
 */
static struct rtg_clock rt_clock_sync = {
		 .clock_set = clock_sync_settime, 
		 .clock_get = clock_sync_gettime,
		 .clock_getres = clock_sync_getres,		 
		 .timer_set = sync_timer_settime, 
		 .timer_del = sync_timer_delete,
		 .timer_get = sync_timer_gettime,
		 .timer_create = sync_timer_create,							
	};

static struct k_clock lx_clock_sync = {
		.res = 1000000,
		.clock_set = clock_sync_settime,		
		.clock_get = clock_sync_gettime,
		.clock_getres = clock_sync_getres,
		.nsleep = sync_nsleep,
		.timer_set = lx_sync_timer_settime,
		.timer_del = lx_sync_timer_delete,
		.timer_get = lx_sync_timer_gettime,
		.timer_create = lx_sync_timer_create,
	};
	
static struct k_clock lx_clock_sync_false = {
		.res = 1000000,
		.clock_set = (int (*)(clockid_t, struct timespec *))return_error, 
		.clock_get = clock_sync_gettime,
		.clock_getres = clock_sync_getres,
		.nsleep = (int (*)(clockid_t, int, struct timespec *, struct timespec __user *)) return_error,
		.timer_create = (int (*)(struct k_itimer *))return_error,
		.timer_set = (int (*)(struct k_itimer *, int, struct itimerspec *, struct itimerspec *))return_error,
		.timer_del = (int (*)(struct k_itimer *))return_error,
		.timer_get = (void (*)(struct k_itimer *, struct itimerspec *))return_error,
	};	

static DEFINE_PER_CPU(struct rt_list_hdr, sync_clock_arr)[MAX_SYNC_CLOCKS] = {
		{.fp = NULL, .clock_sync_isr = &clock_sync_tick_0},
		{.fp = NULL, .clock_sync_isr = &clock_sync_tick_1},
		{.fp = NULL, .clock_sync_isr = &clock_sync_tick_2},
};

DEFINE_PER_CPU(struct posix_timer_pool, rcl) = {
		.used_timer = 0,
		.used_hdr = 0,
		.id_count = 1,
		.bitmap =  NULL,
		.tmr_map = NULL,
};
DEFINE_PER_CPU(struct posix_timer_pool *, prcl);

struct rtg_clock *rt_clock_arr[MAX_CLOCKS];

unsigned long rt_timer_res_nsec;

extern struct dom_hdr aud_rt_hdr;
extern struct lx_cmd get_lx_tod_cmd;

/*
 * This operation has to be called under lock
 * The maximum length of the lock path is roughly determined by either the length of a secondary timer list
 * (SIGEV_THREAD timer) or the length of the primary timer list (SIGEV_THREAD_ID).
 * Because of the small number of either list, locking causes only a small worst case jitter.
 */
int enqueue_timer(struct itimer *itmr)
{
	struct rt_list_hdr *hdr;
	struct list_head *lh;
	struct rt_list_hdr *pilh = &__raw_get_cpu_var(isr_list_hdr);
	
	hdr = itmr->lhdr;
	if (hdr != pilh)
	{
		lh = &hdr->it_link;
		if (list_empty(lh))
			list_add(&itmr->it_link, lh);	
		else
		{
			if (!insert_timer(hdr, itmr))
				return 0;									// it is not the first element
			list_del_init(&hdr->itmr.it_link);				// dequeue header timer from the primary list
		}
		hdr->itmr.activation_time = itmr->activation_time;
		itmr = &hdr->itmr;									// now the timer for the secondary list needs to be enqueued
	}
	return insert_timer(pilh, itmr);
}


/*
 * It has to be called with local interrupts disabled. Only the dequeuing is done.
 * The hardware timer isn't touched. In case an update of the hardware is necessary,
 * it is done later by the caller. If not, then an unnecessary timer interrupt may occur.
 * This is a rare case and does no harm.
 */
int dequeue_timer(struct itimer *itmr)
{
	struct rt_list_hdr *hdr;
	struct list_head *felp;					// pointer to first element
	struct rt_list_hdr *pilh = &__raw_get_cpu_var(isr_list_hdr);

	if (list_empty(&itmr->it_link))
		return 0;
	hdr = itmr->lhdr;
	felp = hdr->it_link.next;	
	list_del_init(&itmr->it_link);

	/* If it wasn't the first element,
	 * nothing else needs to be done. */
	if (felp != &itmr->it_link)		
		return 0;
	if (hdr != pilh)
	{
		/* Get the secondary list_hdr itimer
		 * out of the primary queue. */
		list_del_init(&hdr->itmr.it_link);
		if (!list_empty(&hdr->it_link))	
		{
			itmr = list_entry(hdr->itmr.it_link.next, struct itimer, it_link);
			hdr->itmr.activation_time = itmr->activation_time;	
			insert_timer(pilh, itmr);
		}			
	}
	return 1;	
}


static int return_error(void)
{
	return -EINVAL;
}

/*
 * Only the process which registered with HARD_REALTIME can use CLOCK_SYNC timers
 * If a realtime domain is installed, it can only be used from the realtime domain
 */
int not_authorized(void)
{
	if (!IS_REALTIME_PROCESS(current))		// only realtime process is allowed to use it
		return 1;
	if (IS_REALTIME_PRIO_NORMALIZED(current->prio))		// exclusive use is guaranteed for realtime prios only (no shared interrupts)
		return 0;					// therefore it is only allowed there
	return 1;
}

/*
 * No operation for timer action.
 */ 
static int tmr_noop(void *arg)
{
	return 0;
}

/*
 * No operation for sync clock function.
 */ 
static void isr_noop(void)
{

}

static struct rt_list_hdr *get_sync_list_hdr(clockid_t clockid)
{
	int i;
	struct rt_list_hdr *psca;
	
	for (i = 0; i < MAX_SYNC_CLOCKS; i++) {
		psca = &__raw_get_cpu_var(sync_clock_arr[i]);
		if (clockid == psca->clockid)
			return psca;
	}
	return NULL;
}

/*
 * Partial initialisation of a notification element.
 * Note: Do not use memset() with a zero value.
 */
void init_rt_notify(struct notify *ntp, int code)
{
	INIT_LIST_HEAD(&ntp->sigq.list);
	ntp->alt_thread = NULL;		
	ntp->sigq.flags = SIGQUEUE_PREALLOC;
	ntp->sigq.info.si_errno = 0;
	ntp->sigq.info.si_overrun = 0;
	ntp->sigq.info.si_code = code;
	ntp->cycle = 0;
	ntp->sigq.info.si_sys_cycle = 0;
	ntp->sigq.info.si_sys_sig = 0;
	ntp->sigq.info.si_sys_ovsig = 0;
	ntp->sigq.info.si_sys_ovpending = 0;
}

struct posix_timer *get_tmr(void)
{
	struct posix_timer *tmr = NULL;
	unsigned long flags;
	
	local_irq_save_hw(flags);

	if (!list_empty(&(rtx_get_cpu_var(prcl)->available_timer)))
	{
		tmr = list_entry(rtx_get_cpu_var(prcl)->available_timer.next, struct posix_timer, it_link);
		list_del_init(&tmr->it_link);			// remove it from available_timer list
		INIT_LIST_HEAD(&tmr->itmr.it_link);
		INIT_LIST_HEAD(&tmr->abs_link);
		tmr->itmr.flags = 0;
		tmr->itmr.lhdr = NULL;
		tmr->itmr.chdr = NULL;					// carrier header
		tmr->itmr.action_par = (void*)&tmr->rt_notify;
		tmr->it_state = 0;
		init_rt_notify(&tmr->rt_notify, SI_TIMER);
		rtx_get_cpu_var(prcl)->used_timer++;
	}
	local_irq_restore_hw(flags);

	if (!tmr) {
		if (RT_TIMER_VERBOSE)
			printkGen(NULL, "%s: run out of posix timers\n", __func__);
	}
	return tmr;  
}

struct posix_timer *clone_tmr(struct posix_timer *tmr)
{
	struct posix_timer *new_tmr;
	
	if ((new_tmr = get_tmr()) == NULL)
		return NULL;
	new_tmr->itmr.activation_time = tmr->itmr.activation_time;		
	new_tmr->itmr.count = tmr->itmr.count;
	new_tmr->itmr.lhdr = tmr->itmr.lhdr;
	new_tmr->itmr.chdr = tmr->itmr.chdr;
	new_tmr->rt_notify.thread = tmr->rt_notify.thread;
	new_tmr->rt_notify.alt_thread = tmr->rt_notify.alt_thread;		
	new_tmr->rt_notify.sigq.info.si_signo = tmr->rt_notify.sigq.info.si_signo;
	new_tmr->rt_notify.sigq.info.si_tid = tmr->rt_notify.sigq.info.si_tid;
	new_tmr->rt_notify.sigq.info.si_value = tmr->rt_notify.sigq.info.si_value;
	new_tmr->rt_notify.sigq.info.si_sys_private = 0; 			// in LX prevents rescheduling of timer
	new_tmr->rt_notify.sigq.info.si_sys_cycle = tmr->rt_notify.sigq.info.si_sys_cycle;
	new_tmr->rt_notify.sigq.info.si_sys_sig = tmr->rt_notify.sigq.info.si_sys_sig;
	new_tmr->rt_notify.sigq.info.si_sys_ovsig = tmr->rt_notify.sigq.info.si_sys_ovsig;
	new_tmr->rt_notify.sigq.info.si_code = tmr->rt_notify.sigq.info.si_code;
	new_tmr->rt_notify.sigq.info.si_sys_ovpending = 0;
	new_tmr->rt_notify.sigq.info.si_sys_addr_cycle = &new_tmr->rt_notify.cycle;
	new_tmr->it_clock = tmr->it_clock;
	new_tmr->it_id = tmr->it_id;
	new_tmr->itmr.it_period = tmr->itmr.it_period;	
	new_tmr->itmr.it_cycle = tmr->itmr.it_cycle;	
	new_tmr->itmr.it_interval = tmr->itmr.it_interval;	
	new_tmr->itmr.timer_action = tmr->itmr.timer_action;		// same action, but must point to
	new_tmr->itmr.action_par = &new_tmr->rt_notify;				// rt_notify of new instance
	return new_tmr;
}

void put_tmr(struct posix_timer *tmr)
{
	unsigned long flags;

	local_irq_save_hw(flags);
	list_add(&tmr->it_link, &(rtx_get_cpu_var(prcl)->available_timer));
	rtx_get_cpu_var(prcl)->used_timer--;
	local_irq_restore_hw(flags);
}


static int sync_timer_create(struct posix_timer *tmr, sigevent_t *sigevent)
{
	struct rt_list_hdr *hdp, *chdp;
	struct task_struct *rt_thread;

	if (!(hdp = get_sync_list_hdr(tmr->it_clock)))
		return -EINVAL;
	
	if (GROUP_ACTIVE(hdp))
	 	return -EBUSY;

	if ((sigevent->sigev_notify != SIGEV_SIGNAL_SV) && (sigevent->sigev_notify != SIGEV_THREAD_ID))
	{
		printkGen(NULL, "%s: invalid notification=%#x\n" ,__func__, sigevent->sigev_notify);
		return -EINVAL;
	}

	// Carrier thread handling.
	if (sigevent->sigev_notify == SIGEV_THREAD_ID) {
		if ((rt_thread = rt_find_task_by_pid(sigevent->sigev_notify_thread_id)) == NULL)
			return -EINVAL;
		if ((chdp = rt_thread->rt_carrier_thread_hdr) != NULL)
		{
			// A carrier thread cannot handle timers of different sync-clock sources.
			if (chdp->cs_hdr != hdp)
				return -EINVAL;

			tmr->itmr.chdr = chdp;				// thread is already carrier thread and points to list_hdr
			tmr->rt_notify.sigq.info.si_code = SI_LHDR;		//fake a list header for rt_dequeue_signal

			if (RT_TIMER_VERBOSE)
				printkGen(NULL, "carrier list sync-timer created clockid=%d tid=%d existing carrier pid=%d\n",
							(int)tmr->it_clock, tmr->it_id, rt_thread->pid);
			goto sync_init;
		}
		if ((chdp = get_hdr()) == NULL)
		{
			if (RT_TIMER_VERBOSE)
				printkGen(NULL, "%s: run out of rt_list_hdr\n", __func__);
				return -EAGAIN;
		}
		init_rt_list(chdp);
		chdp->cs_hdr = hdp;
		rt_thread->rt_carrier_thread_hdr = chdp;		// indicator for a thread being a carrier thread

		tmr->itmr.chdr = chdp;
		tmr->rt_notify.sigq.info.si_code = SI_LHDR;		// fake a list header for rt_dequeue_signal()
	}

sync_init:
	// Initialize it for sync_timer_gettime().
	tmr->itmr.it_cycle = 0;
	tmr->itmr.activation_time = 0;

	tmr->itmr.lhdr = hdp;	
	list_add(&tmr->it_link, &hdp->aux_link);

    tmr->rt_notify.thread->rt_notify_state |= RT_SYNC_TIMER_REGISTERED;
    if (tmr->rt_notify.alt_thread)
        tmr->rt_notify.alt_thread->rt_notify_state |= RT_SYNC_TIMER_REGISTERED;

	if (RT_TIMER_VERBOSE)
		printkGen(NULL, "%s: tid=%d clockid=%d\n",__func__, (int)tmr->it_id, (int)tmr->it_clock);
	return 0;
}

/* 	tmr->itmr.action_par = &tmr->rt_notify;	
 * To get best possible decoupling from Linux while still allowing for unified handling code a struct posix_timer element is set up
 * and an additional element clock_sync_timer of the Linux struct k_itimer object refers to it. For all subsequent timer calls the 
 * struct k_itimer object is only used for routing
 */
static int lx_sync_timer_create(struct k_itimer *lx_tmr)
{
	int error;
	
	struct posix_timer *tmr;
	sigevent_t proxy_sigevent;
	
	if (not_authorized())
		return -EINVAL;	
	
	// SIGEV_SIGNAL_SV is translated to SIGEV_THREAD_ID | 0x10.
	if (!(lx_tmr->it_sigev_notify & SIGEV_SIGNAL_SV))
		return -EINVAL;	
	proxy_sigevent.sigev_notify = SIGEV_SIGNAL_SV;
	if (!(tmr = get_tmr()))
		return -EAGAIN;				
	rcu_read_lock();
	if (!(tmr->rt_notify.thread = pid_task(lx_tmr->it_pid, PIDTYPE_PID)) ||
		!(IS_REALTIME_PRIO_NORMALIZED(tmr->rt_notify.thread->prio))) {
		rcu_read_unlock();
		put_tmr(tmr);
		return -EINVAL;
	}
	tmr->rt_notify.ppid = lx_tmr->it_pid;
	tmr->rt_notify.shared = !(lx_tmr->it_sigev_notify & SIGEV_THREAD_ID);
	rcu_read_unlock();

	lx_tmr->it.clock_sync_timer = (void *)tmr;
	tmr->rt_notify.sigq.info.si_tid = lx_tmr->it_id;
	tmr->rt_notify.sigq.info.si_signo = lx_tmr->sigq->info.si_signo;
	tmr->rt_notify.sigq.info.si_value = lx_tmr->sigq->info.si_value;
	tmr->rt_notify.sigq.info.si_sys_private = 0; 			// in Linux prevents rescheduling of timer	
	tmr->it_clock = lx_tmr->it_clock;
	tmr->it_id = lx_tmr->it_id;	
	tmr->itmr.timer_action = lx_notify_thread;
	tmr->itmr.action_par = &tmr->rt_notify;	
	if ((error = sync_timer_create(tmr, &proxy_sigevent) == 0))		
		return 0;
	put_tmr(tmr);
	return error;
}

/*
 * The activation_time of a Posix timer is always stored in tmr->itmr.activation_time.
 * A timer which fires more than once in a period is defined by several Posix timer
 * which are cloned. Now to get the right activation_time depends on the point in time
 * the corresponding timer_gettime() is invoked.
 */
static _INT64 get_activation_time(_INT64 act_clock, _INT64 activation_base, struct posix_timer *tmr)
{
	struct itimer *itmr = &tmr->itmr;
	_INT64 activation_time = itmr->activation_time;	// default
	_INT64 first_activation_time = -1;
	struct rt_list_hdr *hdp = itmr->lhdr;
	struct list_head *lp, *lh;
	struct posix_timer *ptmr;

	lh = &hdp->it_link;
	if (itmr->flags == ONE_SHOT)
		return activation_time;
	/*
	 * The timer (tmr) and possibly existing clones are
	 * sorted according to the activation_time (earliest
	 * activation time first).
	 */
	for (lp = lh->next; lp != lh; lp = lp->next)
	{
		ptmr = list_entry(lp, struct posix_timer, itmr.it_link);
		itmr = &ptmr->itmr;
		if (ptmr->it_id == tmr->it_id)
		{
			if (act_clock < (activation_base + itmr->activation_time)) {
				return itmr->activation_time;
			}
			if (first_activation_time == -1) {
				first_activation_time = itmr->activation_time;
			}
		}
	}
	return first_activation_time ? first_activation_time + hdp->period : activation_time + hdp->period;
}

/* 
 * When executing in the RT-domain, this routine gets called via .timer_get of rt_clock_sync.
 */
static void sync_timer_gettime(struct posix_timer *tmr, struct itimerspec *value)
{
	_INT64 act_clock, activation_time, activation_base;
	_INT64 it_interval = 0, it_value;
	struct itimer *itmr = &tmr->itmr;
	struct rt_list_hdr *hdp = itmr->lhdr;

	if (itmr->flags != ONE_SHOT)
		it_interval = itmr->it_interval;

	// Timer is disarmed?
	if (hdp->itp && (itmr->count != RT_TIMER_DISARMED)) {
		activation_base = hdp->activation_base ? hdp->activation_base : hdp->old_act_base;
		local_irq_disable_hw();
		update_act_clock();
		act_clock = __raw_get_cpu_var(act_clock);
		activation_time = get_activation_time(act_clock, activation_base, tmr);
		local_irq_enable_hw();
		it_value = activation_base + activation_time;
		it_value += (_INT64)(itmr->it_cycle - itmr->count) * hdp->period;

		if (itmr->flags == ONE_SHOT) {
			it_value = (hdp->start_act_base + itmr->it_interval + activation_time);
			if (itmr->count != 1)
				it_value -= hdp->period;
		}

		if (act_clock < it_value)
			it_value -= act_clock;
		else
			it_value += hdp->period;
	}
	else
		it_value = 0;

	convert_to_external(&it_interval, &value->it_interval);
	convert_to_external(&it_value, &value->it_value);
	return;
}

static void lx_sync_timer_gettime(struct k_itimer *lx_tmr, struct itimerspec * value)
{
	struct posix_timer *tmr = (struct posix_timer *)lx_tmr->it.clock_sync_timer;

	sync_timer_gettime(tmr, value);
}


/* 
 * When executing in the RT-domain, this routine gets called via .timer_set of rt_clock_sync
 */
static int sync_timer_settime(struct posix_timer *tmr, int flags, struct itimerspec  *value, struct itimerspec *ovalue)
{
	_INT64 it_interval, it_value, maxval;
	_INT64 ll_period, ll_offset, ll_interval;
	long rem_offset, rem_interval;
	int period, offset, interval;
	struct rt_list_hdr *hdp;
	struct list_head *lh, *lp;
	struct posix_timer *ptmr;
	struct itimer *itmr = &tmr->itmr;
	long error, fire_val;
	int loop, req_period;
	_INT64 req_interval;
	int disarmed = 0;

	hdp = tmr->itmr.lhdr;
	lh = &hdp->it_link;	
	if (ovalue != NULL) 												
		sync_timer_gettime(tmr, ovalue);					// read back old value
	if (flags & TIMER_ABSTIME)		
		return -EINVAL;
	flags = 0;												// used to distinguish between cyclic and one shot

	// Convert to internal representation.
	if ((error = convert_to_internal(&it_interval, &value->it_interval)))
		return error;
	if ((error = convert_to_internal(&it_value, &value->it_value)))
		return error;

	// Check for max values.
	maxval = hdp->period * (_INT64)RT_MAX_INT;
	if (it_interval > maxval || it_value > maxval)
		return -EINVAL;
	// This check is only for cyclic timers.
	if (it_interval != 0 && (it_value > it_interval || it_value > hdp->period))
			return -EINVAL;

	local_irq_disable_hw();
	// Disarm the timer?
	if ((it_interval == 0) && (it_value == 0)) {
		if (itmr->count == RT_TIMER_DISARMED) {
			local_irq_enable_hw();
			return 0;					// it is already disarmed
		}

		// Scan the dynamic list (armed timers).
		// - Remove the timer (and possibly all its clones) from the dynamic list.
		// - If the timer group is active and the timer to be removed is the next to fire we don't remove it.
		//   In this case do not deactivate the timer group. do_clock_sync_list() cares if the list is empty.
		//
		for (lp = lh->next; lp != lh; ) {
			ptmr = list_entry(lp, struct posix_timer, itmr.it_link);
			lp = lp->next;
			if (ptmr->it_id == tmr->it_id) {
				// Note: hdp->itp may be NULL.
				if (hdp->itp != &ptmr->itmr) {
					list_del_init(&ptmr->itmr.it_link);		// remove timer (and clones) from dynamic list
					disarmed = 1;
				}
				ptmr->itmr.count = RT_TIMER_DISARMED;		// set timer disarmed
			}
		}
		if (disarmed && list_empty(&hdp->it_link)) {
			hdp->itp = NULL;								// deactivate group (system view)
			if (hdp->ptmr && dequeue_timer(hdp->ptmr)) 		// disable period timer
				CHECK_TIMER(0);
		}
		local_irq_enable_hw();
		return 0;
	}

	if (GROUP_ACTIVE(hdp))	{								// group is already armed
		local_irq_enable_hw();
		return -EBUSY;
	}	
	local_irq_enable_hw();


	interval = (int)it_interval;			
	offset = (int)it_value;
	period = (int)hdp->period;		// always <= 0x7fffffff
	ll_interval = it_interval;
	ll_offset = it_value;		// only for oneshot timer
	ll_period = hdp->period;

	// Handle a oneshot timer.
	if (ll_interval == 0) {
		if (ll_offset <= ll_period) {
			itmr->count = fire_val = loop = 1;			// fires each period
			if (offset != period)
				offset = offset%period;
		}
		else {
			rem_offset = do_div(ll_offset, period);
			fire_val = (long)(ll_offset + 2LL);			// period the timer fires
			loop = 1;
			itmr->count = 2;
			offset = (int)(rem_offset);
		}
		flags = ONE_SHOT;
		goto update_itmr;
	}

	// Handle cyclic timers.
	if (ll_period >= ll_interval)
	{
		if (interval < IT_INTERVAL_MIN)				// 31,25 us
			return -EINVAL;

		fire_val = 1;								// fires each period
		loop = period/interval + (period%interval + interval/2)/interval;
		req_period = interval * loop;
		if (loop < abs(req_period - period))
		{
			printkGen(KERN_ALERT, "%s: period=%d isn't a multiple of interval=%d req_period=%d loop=%d\n", __func__, period, interval, req_period, loop);
			return -EINVAL;
		}
		if (offset != interval)
			offset = offset%interval;
		itmr->count = 1;
	}
	else
	{
		long diff_interval;

		rem_interval = do_div(ll_interval, period);
		fire_val = (long)ll_interval + (rem_interval + period/2)/period;		// fires each nth period
		req_interval = (_INT64)fire_val * ll_period;
		diff_interval = req_interval > it_interval ? (long)(req_interval - it_interval) : (long)(it_interval - req_interval);
		if (fire_val < diff_interval)
		{
			printkGen(KERN_ALERT, "%s: interval=%lld isn't a multiple of period=%d req_interval=%lld fire_val=%ld\n", __func__, it_interval, period, req_interval, fire_val);
			return -EINVAL;
		}				
		loop = 1;
		itmr->count = fire_val - (offset/period);
		offset = offset%period;
	}
update_itmr:
	// Scan the static list because the disarmed timers can only be found there.
	// Reset the last state of the timer because now we are sure that there is
	// no further error return possible (except ENOMEM).
	local_irq_disable_hw();
	for (lp = hdp->aux_link.next; lp != &hdp->aux_link;)
	{
		ptmr = list_entry(lp, struct posix_timer, it_link);
		lp = lp->next;
		if (ptmr->it_id == tmr->it_id)
		{
			list_del_init(&ptmr->itmr.it_link);			// remove timer from "dynamic" list
			// Handle the cloned timers.
			if (ptmr != tmr) {
				list_del_init(&ptmr->it_link);			// remove timer from "static" list
				put_tmr(ptmr);
			}
		}
	}
	local_irq_enable_hw();

	itmr->activation_time = offset;	
	itmr->it_cycle = fire_val;
	itmr->it_interval = it_interval;
	itmr->flags = flags;
	
	// Now queue it in at hdp->it_link according to activation time.
	do {
		local_irq_disable_hw();
		insert_timer(hdp, itmr);
		if (--loop > 0)
		{		
			if (!(tmr = clone_tmr(tmr)))			// for every activation of a cyclic timer within a period
			{										// a new instance of a "one shot" timer is linked in
				local_irq_enable_hw();
				return -ENOMEM;
			}			
			itmr = &tmr->itmr;	
			itmr->activation_time += it_interval;
			list_add(&tmr->it_link, &tmr->itmr.lhdr->aux_link);	
		}
		local_irq_enable_hw();
	} while (loop > 0);
	
	if (RT_TIMER_VERBOSE)
	{
		lp = hdp->it_link.next;
		do {			
			itmr = list_entry(lp, struct itimer, it_link);
			ptmr = list_entry(itmr, struct posix_timer, itmr);
			printkGen(NULL, "%s: tid=%d offset=%llu count=%d cycle=%ld signo=%d\n", __func__, ptmr->it_id, itmr->activation_time,
									itmr->count, itmr->it_cycle, ptmr->rt_notify.sigq.info.si_signo);
			lp = lp->next;		 
		} while (lp != &hdp->it_link);	
		printkGen(NULL, "\n");
	}	
	return 0;
}

static int lx_sync_timer_settime(struct k_itimer *ktmr, int flags, struct itimerspec  *value, struct itimerspec *ovalue)
{
	struct posix_timer *tmr = (struct posix_timer *)ktmr->it.clock_sync_timer;	

	return sync_timer_settime(tmr, flags, value, ovalue);
}

/*
 * This routine  gets called from both domains. To avoid any races in case of an active group, 
 * the timers only get disabled. 
 * when called from the rt domain only the pid entry of the leading timer gets recycled from the caller
 * when called from the lx domain, lx cares only for its timer completely, the leading timer is a proxy
 * So in case of an inactive group for both domains all timers can get recycled. No check for queued signal
 * elements needs to be done.
 */
static int sync_timer_delete(struct posix_timer *tmr)
{	
	timer_t it_id = tmr->it_id;		
	struct rt_list_hdr *hdp = tmr->itmr.lhdr;
	struct list_head *lp;

	if (RT_TIMER_VERBOSE)
		printkGen(NULL, "%s: tid=%d\n", __func__, tmr->it_id);

	local_irq_disable_hw();	
	if	(GROUP_ACTIVE(tmr->itmr.lhdr))
	{
		local_irq_enable_hw();
		return -EBUSY;
	}
	else
	{
		for (lp = hdp->aux_link.next; lp != &hdp->aux_link;)
		{
			tmr = list_entry(lp, struct posix_timer, it_link);
			lp = lp->next;
			if (tmr->it_id == it_id)
			{				
				list_del_init(&tmr->it_link);				// the "static" created posix_timer.it_link
				list_del_init(&tmr->itmr.it_link);			// the dynamic link
				put_tmr(tmr);
			}		
		}
	}
	local_irq_enable_hw();	
	return 0;
}


/* Gets only called inside the LX-domain. */
static int lx_sync_timer_delete(struct k_itimer *ktmr)
{
	struct posix_timer *tmr = (struct posix_timer *)ktmr->it.clock_sync_timer;
	return sync_timer_delete(tmr);
}

static int clock_sync_settime (clockid_t clockid, struct timespec *tp)
{
	struct rt_list_hdr *hdp;
	struct itimer *ptmr;
	_INT64 timeval = 0;
	int error;

	if (not_authorized())
		return -EINVAL;	
	if ((error = convert_to_internal(&timeval, tp)) < 0) 
		return error;			
	if ((hdp = get_sync_list_hdr(clockid)) == NULL)
		return -EINVAL;
	local_irq_disable_hw();		
	if (GROUP_ACTIVE(hdp))
	{
		if (timeval == 0)
		{
			// Deactivate timer group.
			hdp->active = 0;				// application view
			hdp->itp = NULL;				// system view
			if (hdp->ptmr)
				dequeue_timer(hdp->ptmr); 	// disable period timer 
			dequeue_timer(&hdp->itmr);		// disable oneshot proxy timer 
			CHECK_TIMER(0);
			local_irq_enable_hw();	
			return 0;
		}
		local_irq_enable_hw();	
		return -EBUSY;
	}
	else {
		/* Must be the same value as the period of the device. */
		if (hdp->period != timeval) {
			local_irq_enable_hw();	
			return -EINVAL;
		}
	}	
	if (list_empty(&hdp->it_link))
	{
		local_irq_enable_hw();			
		return 0;
	}		
	
	// Activate group: Mark it as active and armed
	hdp->active = 1;								// application view
	hdp->itp = list_entry(hdp->it_link.next, struct itimer, it_link);	// system view

	hdp->activation_base = 0;
	update_act_clock();
	hdp->start_act_base = hdp->old_act_base = __raw_get_cpu_var(act_clock);
	if ((ptmr = hdp->ptmr))
	{																// CLOCK_SYNC_TIMER	
		ptmr->activation_time = timeval + hdp->old_act_base;
		if (enqueue_timer(ptmr))
			CHECK_TIMER(0);
	}
	local_irq_enable_hw();
	if (RT_TIMER_VERBOSE)
		printkGen(NULL, "%s: CLOCK_SYNC group enabled\n", __func__);
	return 0;
}


static int clock_sync_gettime(clockid_t clockid, struct timespec *value)
{
	struct rt_list_hdr *hdp;
	_INT64 clock_sync_time = 0;	
	
	if ((hdp = get_sync_list_hdr(clockid)))	
	{
		if (hdp->itp)
		{
			local_irq_disable_hw();
			update_act_clock();
			if (hdp->activation_base)
				clock_sync_time = __raw_get_cpu_var(act_clock) - hdp->activation_base;
			else
				clock_sync_time = __raw_get_cpu_var(act_clock) - hdp->old_act_base;
			local_irq_enable_hw();
			if (clock_sync_time < 0)
				clock_sync_time += hdp->period;
		}
		return(convert_to_external(&clock_sync_time, value));
	}
	return -EINVAL;	
}

static int clock_sync_getres(clockid_t clockid, struct timespec *value)
{
	struct rt_list_hdr *hdp;
	_INT64 period;				
	
	if ((hdp = get_sync_list_hdr(clockid)))
	{	
		period = hdp->period;
		return(convert_to_external(&period, value));
	}
	return -EINVAL;	
}

int sync_nsleep (clockid_t clockid, int flags, struct timespec *new_setting ,struct timespec __user *utp)
{
	return 0;
}

/*
 * Handling of carrier threads for CLOCK_SYNC.
 * Note: Assumption is that interrupts are disabled.
 */
struct itimer *get_sync_timer_elapsed(struct rt_list_hdr *hdr)
{
	struct posix_timer *ptmr;
	struct itimer *itmr;
	_INT64 activation_time, act_base, act_clock;
	struct list_head *lp, *lh;
	struct rt_list_hdr *cs_hdr = hdr->cs_hdr;

	 // There may be the very rare case that a oneshot timer fired and
	 // because it was the last timer in the dynamic list, the timer
	 // group has been de-activated. In this case the timer is in the
	 // oneshot_carrier list.
	if (!cs_hdr || (list_empty(&hdr->oneshot_carrier) && !cs_hdr->itp))
		return NULL;

	lh = &cs_hdr->it_link;

	 // The timer (tmr) and possibly existing clones are
	 // sorted according to the activation_time (earliest
	 // activation time first).
	 // Oneshot timers which have already fired are removed
	 // from this list.
	update_act_clock();
	act_clock = __raw_get_cpu_var(act_clock);
	act_base = (cs_hdr->activation_base ? cs_hdr->activation_base : cs_hdr->old_act_base);

	for (lp = lh->next; lp != lh; lp = lp->next) {
		ptmr = list_entry(lp, struct posix_timer, itmr.it_link);
		itmr = &ptmr->itmr;
		/* The timer is handled by a carrier thread? */
		if (itmr->chdr == hdr) {
			//nfb = (struct notify *)itmr->action_par;
			//nfb->sigq.info.si_code = SI_LHDR;			// fake a list header
			activation_time = act_base + itmr->activation_time;

			if ((act_clock >= activation_time) && (activation_time > hdr->last_wait)) {
				hdr->last_wait = activation_time;
				//nfb->sigq.info.si_code = SI_TIMER;		// reset the fake (the carrier checks for SI_TIMER)
				return itmr;
			}
		}
	}

	// Scan the oneshot carrier list.
	if (!list_empty(&hdr->oneshot_carrier)) {
		lh = &hdr->oneshot_carrier;

		for (lp = lh->next; lp != lh; ) {
			ptmr = list_entry(lp, struct posix_timer, itmr.it_link);
			itmr = &ptmr->itmr;
			lp = lp->next;
			if (itmr->chdr == hdr) {
				activation_time = act_base + itmr->activation_time;

				if ((act_clock >= activation_time) && (activation_time > hdr->last_wait)) {
					hdr->last_wait = activation_time;
					list_del_init(&itmr->it_link);			// remove timer from oneshot_carrier list
					return itmr;
				}
			}
		}
	}
	return NULL;
}

/*
 * Wake up of the list (start of the sequence) is done either by the registered ISR or in case of a device
 * without ISR by the cyclic timer element which is installed when registering the clock
 * within a sequence the struct itimer element of the rt_list header is queued in the timer queue
 * the entire routine runs with HW-interrupts disabled
 */
static int do_clock_sync_list(void *arg)
{
	struct rt_list_hdr *hdp = (struct rt_list_hdr*)arg;
	struct itimer *itmr;
	struct list_head *lp;
	struct rt_list_hdr *pilh = &__raw_get_cpu_var(isr_list_hdr);
	
	if (unlikely((itmr = hdp->itp) == NULL))
		return 0;			// group is disarmed
	lp = &itmr->it_link;
	
	do 	
	{
		if ((itmr->activation_time + hdp->activation_base) > (__raw_get_cpu_var(act_clock) + aud_rt_timer->gravity))
		{													// no expired timer anymore, setup for timer to fire next
			hdp->itmr.activation_time = hdp->activation_base + itmr->activation_time;
			insert_timer(pilh, &hdp->itmr);
			hdp->itp = itmr;								// entry point for the next scan								
			return 0;			
		}
		lp = lp->next;										// reference to next element

		// Timer is disarmed?
		if (itmr->count == RT_TIMER_DISARMED) {
			list_del_init(&itmr->it_link);
			if (list_empty(&hdp->it_link)) {
				hdp->itp = NULL;				// deactivate group
				if (hdp->ptmr) {
					dequeue_timer(hdp->ptmr); 	// disable period timer
					CHECK_TIMER(0);
				}
				break;
			}
			goto next_itmr;				// remove the timer from dynamic list
		}

		if(itmr->count == itmr->it_cycle)
		{ 						
			itmr->timer_action(itmr->action_par);
			itmr->count = 1;		
			
			// If this timer is last timer in the active list
			// then we have to deactivate the group.
			if (itmr->flags == ONE_SHOT) {					// real one shot
				list_del_init(&itmr->it_link);

				// Controlled by a carrier thread?
				if (itmr->chdr)
					list_add_tail(&itmr->it_link, &itmr->chdr->oneshot_carrier);

				if (list_empty(&hdp->it_link)) {
					hdp->itp = NULL;				// deactivate group
					if (hdp->ptmr) {
						dequeue_timer(hdp->ptmr); 	// disable period timer
						CHECK_TIMER(0);
					}
					break;
				}
			}

			if (rtx_get_cpu_var(rtx_mode) == LX_DOMAIN_SOFT)
				update_act_clock();							// in Linux too much time may have passed
		}
		else													
			itmr->count++;									// no firing in this period	

next_itmr:
		itmr = list_entry(lp, struct itimer, it_link);							
		if (lp == &hdp->it_link)
		{
			itmr = list_entry(hdp->it_link.next, struct itimer, it_link);
			lp = &itmr->it_link;
			hdp->itp = itmr;			
	 		hdp->old_act_base = hdp->activation_base;
	 		hdp->activation_base = 0;
			if (hdp->new_act_base)
			{												// next period is already due
				hdp->activation_base = 	hdp->new_act_base;
				hdp->new_act_base = 0;
				continue;
			}
			break;
		}
	} while (1);
	
	// CLOCK_SYNC_HARD or CLOCK_SYNC_TIMER 
	// Periodic timer or external event isr triggers next cycle.
	return 0;
}

void new_period_io(struct rt_list_hdr *hdp)
{	
	if ((hdp->itp == NULL))									// itp arms the group
		return;
	update_act_clock();	
	if (hdp->activation_base)							// period still active
	{
		hdp->new_act_base = __raw_get_cpu_var(act_clock);
		return;
	}
	hdp->activation_base = __raw_get_cpu_var(act_clock);
	if (rtx_get_cpu_var(rtx_mode) == RT_DOMAIN)
	{
		do_clock_sync_list(hdp);						// for hard realtime do it directly at ISR level
		CHECK_TIMER(0);
	}
	else
		up(&hdp->sem);									// buffered activation of proxy
	return;
}


static void clock_sync_tick_0(void)
{
	return new_period_io(&__raw_get_cpu_var(sync_clock_arr[0]));
}

static void clock_sync_tick_1(void)
{
	return new_period_io(&__raw_get_cpu_var(sync_clock_arr[1]));
}

static void clock_sync_tick_2(void)
{
	return new_period_io(&__raw_get_cpu_var(sync_clock_arr[2]));
}


/*
 *  This tick routine  executes as softirq/daemon inside the linux kernel 
 */
 int lx_clock_proxy_thread(void* arg)
{
	struct rt_list_hdr * hdp = (struct rt_list_hdr *)arg;
	struct sched_param sched_par;
	struct itimer *itmr_first;
	_INT64 time_rel;

	strcpy(current->comm, CLOCK_PROXY_NAME);
	sched_par.sched_priority = 99;			// must be highest priority
	current->rt_state3 |= RT_TASK_ALLOW_SETSCHEDULER;
	sched_setscheduler(current, SCHED_FIFO, &sched_par);
	current->rt_state = LXRT_TASK;
		
	if (RT_INIT_VERBOSE)
		printkGen("(cp)", "clock proxy daemon installed prio=%d:%d\n", current->rt_priority, current->prio);
	up(&scl_sem);							// continue creator thread
	while (hdp->fp) 						// NULL when unregistering
	{
		down(&hdp->sem);
		if (hdp->fp)						// check for unregister
		{
			local_irq_disable_hw();	
			update_act_clock();				// some time may have passed meanwhile			
			do_clock_sync_list(hdp);
			itmr_first = list_entry(rtx_get_cpu_member(isr_list_hdr, it_link.next), struct itimer, it_link);
			if (itmr_first == &hdp->itmr)	// otherwise the ISR or other proxies (must) care for it
			{
				time_rel = itmr_first->activation_time - __raw_get_cpu_var(act_clock);
				if (RTX_TIMER_SET(time_rel))
					;			
			}
	 		local_irq_enable_hw();				
		}
	}
	sched_par.sched_priority = 0;
	current->rt_state3 |= RT_TASK_ALLOW_SETSCHEDULER;
	sched_setscheduler(current, SCHED_NORMAL, &sched_par);
	if (RT_SHUTDOWN_VERBOSE)
		printkGen("(cp)", "clock proxy daemon shutting down\n");
	return 0;
}

int wake_up_proxy(void * arg)
{
 	struct rt_list_hdr * hdp = (struct rt_list_hdr *)arg; 
 	
 	up(&hdp->sem);
 	return 0;
}
 
/*
 * The CLOCK_SYNC period is controlled by a timer. In case of CLOCK_SYNC_TIMER it is a cyclic timer.
 */
int new_period_timer(void *arg)
{
	_INT64 activation_base;
 	struct rt_list_hdr *hdp = (struct rt_list_hdr *)arg;

 	if (hdp->ptmr) {
 		activation_base = hdp->ptmr->activation_time - hdp->ptmr->it_period;
		if (hdp->activation_base)							// period still active
		{													// only possible for CLOCK_SYNC_TIMER
			hdp->new_act_base = activation_base;
			return 0;
		}
		hdp->activation_base = activation_base;
		if (rtx_get_cpu_var(rtx_mode) == RT_DOMAIN)
			do_clock_sync_list(hdp);						// for hard realtime do it directly at ISR level
		else
			up(&hdp->sem);									// buffered activation of proxy
 	}
	return 0;
} 
 
/*
 * Partial initialisation of a rt_list_hdr element.
 * Note: Do not use memset() with a zero value.
 */
void init_rt_list(struct rt_list_hdr *lhdr)
{
	INIT_LIST_HEAD(&lhdr->it_link);
	INIT_LIST_HEAD(&lhdr->aux_link);	
	INIT_LIST_HEAD(&lhdr->itmr.it_link);
    lhdr->itp = NULL;
    lhdr->start_act_base = 0;
    lhdr->last_wait = 0;
    lhdr->cs_hdr = NULL;
	INIT_LIST_HEAD(&lhdr->oneshot_carrier);
    lhdr->active = 0;
	lhdr->isr_fp = NULL;
	lhdr->itmr.lhdr = &__raw_get_cpu_var(isr_list_hdr);
	lhdr->itmr.it_period = 0LL;
	lhdr->itmr.it_cycle = 0;
	lhdr->itmr.activation_time = 0x7fffffffL;
	lhdr->itmr.action_par = &lhdr->rt_notify;
	init_rt_notify(&lhdr->rt_notify, SI_LHDR);
	lhdr->rt_notify.sigq.info.si_signo = SIGTIMER;
}

/*
 * Can be called from both domains. Because of the very static nature of the CLOCK_SYNC timer we are
 * cutting corners by only nooping the element, so that a terminating thread doesn't 
 * crash the system. The recycling is left to the shutdown of the realtime process.
 */
void clock_sync_delete_timer_notification(struct task_struct *p)
{
	int i;
	struct rt_list_hdr *hdp;
	struct posix_timer *tmr;
	struct list_head *lp;
	int count = 0;
	
	for (i = 0; i < MAX_SYNC_CLOCKS; i++) {
		hdp = &__raw_get_cpu_var(sync_clock_arr[i]);
		if (hdp->fp) {
			local_irq_disable_hw();	
			for (lp = hdp->aux_link.next; lp != &hdp->aux_link; lp = lp->next) {
				tmr = list_entry(lp, struct posix_timer, it_link);
				if ((tmr->rt_notify.thread == p) || (tmr->rt_notify.alt_thread == p)) {
					if (tmr->itmr.timer_action != tmr_noop)
						count++;					
					tmr->itmr.timer_action = tmr_noop;	
				}
			}			
			local_irq_enable_hw();				
		}
	}
	if (RT_TIMER_VERBOSE)
		printkGen(NULL, "%s: deleting %d notifications for thread=%d\n", __func__, count, p->pid);
}

/*
 * Note: This function is called under rt_tl_sem lock.
 */
int check_sync_timer_notification(struct task_struct *p)
{
	int i;
	struct rt_list_hdr *hdp;
	struct posix_timer *tmr;
	struct list_head *lp;
	
	for (i = 0; i < MAX_SYNC_CLOCKS; i++) { 
		hdp = &__raw_get_cpu_var(sync_clock_arr[i]);
		if (hdp->fp) {
			if (RT_TIMER_VERBOSE)
				printkGen(NULL, "%s: checking notifications\n", __func__);
			local_irq_disable_hw();	
			for (lp = hdp->aux_link.next; lp != &hdp->aux_link; lp = lp->next)
			{
				tmr = list_entry(lp, struct posix_timer, it_link);
				if ((tmr->rt_notify.thread == p) || (tmr->rt_notify.alt_thread == p))
				{
					local_irq_enable_hw();					
					return 1;			
				}
			}			
			local_irq_enable_hw();			
		}
	}
	return 0;
}


/*
 * Default handler for LX (only to be registered).
 */
static irqreturn_t default_handler(int irq, void *dev)
{
	return IRQ_HANDLED;
}

/*
 * ISR for the timer device.
 */
irqreturn_t rt_clock_isr(int irq, void *dev_id)
{
#ifndef CONFIG_RTX_DOMAIN_HRT
	__raw_get_cpu_var(act_clock)++;
#endif
	CHECK_TIMER(0);
#ifdef CONFIG_RTX_DOMAIN_HRT	
	return IRQ_HANDLED;
#else
	return IRQ_NONE;
#endif
}

int init_timer_management(int no_posix_timer, int no_of_list_hdr)
{
	int i, size; 
	struct posix_timer *tmr;
	struct rt_list_hdr *hdr;
	unsigned long seq;
	int ret; 		

	init_rt_list(&__raw_get_cpu_var(isr_list_hdr));

	rtx_set_cpu_var(prcl, &__raw_get_cpu_var(rcl));

	INIT_LIST_HEAD(&(rtx_get_cpu_var(prcl)->available_timer));
	INIT_LIST_HEAD(&(rtx_get_cpu_var(prcl)->available_list_hdr));
	INIT_LIST_HEAD(&(rtx_get_cpu_var(prcl)->created_timer));

	/* Allocate space for the maximum number of timers (CLOCK_REALTIME and CLOCK_SYNC)
	 * and timer list headers allowed for the RT-Domain. Initialize the bitmap and the
	 * pointer list and queue the timers in the list of available timers.
	 */
	size = (((no_posix_timer + 31) >> 3) + no_posix_timer * 4 + no_posix_timer * sizeof(struct posix_timer)
			+ no_of_list_hdr * sizeof(struct rt_list_hdr));

	if ((rtx_get_cpu_var(prcl)->bitmap = (unsigned int *) KMALLOC(size, GFP_KERNEL)) == NULL)
		return -ENOMEM;

	rtx_get_cpu_var(prcl)->tmr_map = (struct posix_timer **) (rtx_get_cpu_var(prcl)->bitmap + ((no_posix_timer + 31) >> 5));
	tmr = (struct posix_timer *)(rtx_get_cpu_var(prcl)->tmr_map + no_posix_timer);

	for (i = 0; i < no_posix_timer ; i++)
	{
		tmr->it_id = 0;							  	// invalid timer entry
		tmr->itmr.action_par = (void*)&tmr->rt_notify;
		tmr->rt_notify.sigq.info.si_code = SI_TIMER;
		tmr->itmr.flags = 0;
		INIT_LIST_HEAD(&tmr->it_link);
		INIT_LIST_HEAD(&tmr->itmr.it_link);
		INIT_LIST_HEAD(&tmr->rt_notify.sigq.list);
		list_add (&tmr->it_link, &(rtx_get_cpu_var(prcl)->available_timer));
		tmr++;
		*(rtx_get_cpu_var(prcl)->bitmap + (i >> 5)) = 0;
		*(rtx_get_cpu_var(prcl)->tmr_map + i) = 0;
	}
	rtx_get_cpu_var(prcl)->used_timer = 0;
	rtx_get_cpu_var(prcl)->max_timer = i;
	rtx_get_cpu_var(prcl)->id_count = 1;

	/*
	 * Queue the  list headers used for handler notification via carrier threads
	 * and for CLOCK_SYNC headers in the list of available list headers.
	 */
	hdr = (struct rt_list_hdr *)tmr;
	for (i = 0; i < no_of_list_hdr ; i++)
	{
		init_rt_list(hdr);
		list_add (&hdr->it_link, &(rtx_get_cpu_var(prcl)->available_list_hdr));
		hdr++;
	}
	if (!init_timer_done) {
		for (i = 0; i < MAX_CLOCKS; i ++)				// init clock_arr
			rt_clock_arr[i] = NULL;
	}

	/* A fixed number of CLOCK_SYNC list headers are allocated
	 * statically in an array to allow indexing into it. */
	for (i = 0; i < MAX_SYNC_CLOCKS; i++)
		__raw_get_cpu_var(sync_clock_arr[i]).fp = NULL;

	if (RT_INIT_VERBOSE)
		printkGen(NULL, "%s: max_timer=%d no_of_list_hdr=%d\n", __func__,
				rtx_get_cpu_var(prcl)->max_timer, no_of_list_hdr);
	
	if (!aud_rt_timer) {
		printkGen(KERN_ALERT, "%s: no timer device available\n", __func__);
		ret = -ENODEV;
		goto timer_err;
	}

	if (aud_rt_timer->ops.rt_register() != 0) {
		printkGen(KERN_ALERT, "%s: timer registration failed\n", __func__);
		ret = -ENODEV;
		goto timer_err;
	}

	if (!init_timer_done) {
		// Setup resolution and normalize the gravity interval.
		if ((rt_timer_res_nsec = aud_rt_timer->res_nsec) == 0) {
	 		aud_rt_timer->ops.rt_unregister();
			printkGen(KERN_ALERT, "%s: timer resolution is not initialized\n", __func__);
			ret = -ENODEV;
			goto timer_err;
		}
		aud_rt_timer->gravity = RTX_GRAVITY_INTERVAL;

		do {
			seq = rtx_read_seqbegin(&xtime_lock);
			rt_tomono = wall_to_monotonic;
		} while (rtx_read_seqretry(&xtime_lock, seq));
		init_timer_done = 1;
	}

	/*
	 * Note: rt_start() doesn't mean to enable the
	 * timer interrupt. It can be used to set an
	 * initial value for rt_time_last if the timer
	 * is already running.
	 */
    if (aud_rt_timer->ops.rt_start() != 0) {
 		aud_rt_timer->ops.rt_unregister();
 		printkGen(KERN_ALERT, "%s: timer start failed\n", __func__);
		ret = -ENODEV;
		goto timer_err;
    }
	rt_clock_init();			// initialize CLOCK_REALTIME	

	if ((ret = rt_request_irq(aud_rt_timer->irq, (void *)rt_clock_isr, 0, aud_rt_timer->name, aud_rt_timer->dev_id, default_handler)) < 0)	
	{	
		aud_rt_timer->ops.rt_unregister();
		printkGen(KERN_ALERT, "%s: rt_request_irq failed irq=%d\n", __func__, aud_rt_timer->irq);
		goto timer_err;
	}

	/*
	 * Activate a periodic timer request which depends
	 * on the architecture supported:
	 * - cyclic drift control if LX and RT timer devices have different sources
	 * - trigger of the LX timer tick in case of an Mips/Ertec-ARM based system
	 * Note: aud_rt_timer.preq must already be initialized.
	 */
	if (aud_rt_timer->mode & RT_PERIODIC_LX_TICK)
		setup_clock_periodic_timer();

	if (RT_INIT_VERBOSE) {
		printkGen(NULL, "%s: name=%s irq=%d res_nsec=%lu gravity=%d delta=%ld dev_id=%#x\n", __func__,
			aud_rt_timer->name, aud_rt_timer->irq, rt_timer_res_nsec, aud_rt_timer->gravity, aud_rt_timer->delta, aud_rt_timer->dev_id);
	}
	rtx_or_cpu_var(rt_system_state, RT_TIMER_INITIALIZED);
	return 0;

timer_err:
	KFREE((char *)rtx_get_cpu_var(prcl)->bitmap);
	return ret;
}

static int register_new_posix_clock(clockid_t clockid, struct k_clock *new_clock)
{
	int i;

	if ((clockid >= 0) && (clockid < MAX_CLOCKS))
	{
		posix_clocks[clockid] = *new_clock;
		return clockid;
	}
	for (i = MAX_CLOCKS -1; i > 0; i--) {
		if (posix_clocks[i].res == 0)			// in a final version this must be done under a lock here
		{											// currently it is done outside
			posix_clocks[i] = *new_clock;
			return i;
		}
	}
	return -ESRCH;
}

int unregister_posix_clock(clockid_t clockid)
{
	if (((unsigned) clockid >= MAX_CLOCKS) || ((unsigned) clockid < 2))
		return -EINVAL;
	posix_clocks[clockid].res = 0; 
	return clockid;
}

#ifndef CONFIG_RTX_DOMAIN_HRT
unsigned long clock_getres_int(int clockid)
{
	if (clockid >= MAX_CLOCKS)
		return -EINVAL;
	return TICK_NSEC;
}
#endif

/* This routine is called from a device driver as result of a register_clock(fd, flags)
 * which gets converted inside the library to a ioctl(fd,..,AuD_REGISTER_CLOCK,..).
 * Sync timers only make sense for a real time environment. Therefore we allow registration
 * of a clock only for a process that is registered as hard realtime process.
 */
int rt_register_sync_clock(struct file* fp, struct rt_clock_desc *clock_desc, int flags, void (**fn)(void))
{
    int i;
    struct rt_list_hdr *hdp;
    struct itimer *itmr;
    struct posix_timer *tmr;
    _INT64 period;
    int ret;
    
    clockid_t sync_clockid = -EBUSY;

	if (IS_REALTIME)
		rtx_migrate_to_lx(RTLX_TASK, NULL);

	if ((flags != CLOCK_SYNC_HARD) && (flags != CLOCK_SYNC_TIMER))
		return -EINVAL;
    if ((clock_desc == NULL) || (clock_desc->clock_type != CLOCK_SYNC))
    	return -EINVAL;
	if (!(rtx_get_cpu_var(rt_system_state) & RT_TIMER_INITIALIZED))
    	return -EINVAL;
	if ((fn == NULL) && (flags != CLOCK_SYNC_TIMER))
		return -EINVAL;	
	if ((ret = convert_to_internal(&period, &clock_desc->clock_spec)) < 0) 
		return ret;	
	if ((period <= (_INT64)RT_PERIOD_MIN) || (period > (_INT64)RT_PERIOD_MAX))
		return -EINVAL;
	if (period < (_INT64)CLOCK_SYNC_GRANULARITY)
		return -EINVAL;

	down(&scl_sem);		
	for (i = 0; i < MAX_SYNC_CLOCKS; i++) {
		if (__raw_get_cpu_var(sync_clock_arr[i]).fp == fp)
			goto errout;
	}
	for (i = 0; i < MAX_SYNC_CLOCKS; i++) {
		if (__raw_get_cpu_var(sync_clock_arr[i]).fp == NULL)
			break;
	}
	if (i >= MAX_SYNC_CLOCKS)						
	{	
		sync_clockid = -ESRCH;	
		goto errout;
		
	}
	hdp = &__raw_get_cpu_var(sync_clock_arr[i]);
	init_rt_list(hdp);
	hdp->fp = fp;

	/* Both are used in LX as an indicator
	 * for a used clock. */
	lx_clock_sync.res = (int)period;
	lx_clock_sync_false.res = (int)period;

	/* If installed the RT-domain can use this clockid. */
	if (rtx_get_cpu_var(rtx_mode) == RT_DOMAIN)
	{
		/* Make sure clockid can't be used in Linux. */
		sync_clockid = register_new_posix_clock((clockid_t)-1, &lx_clock_sync_false);
		rt_clock_arr[sync_clockid] = &rt_clock_sync;
		hdp->itmr.timer_action = (void*)do_clock_sync_list;			
	}
	else
	{									
		sema_init(&hdp->sem, 0);
		sync_clockid = register_new_posix_clock((clockid_t)-1, &lx_clock_sync);
		kernel_thread(lx_clock_proxy_thread, (void *)hdp, CLONE_KERNEL | CLONE_THREAD);
		hdp->itmr.timer_action = wake_up_proxy;	 // action_par is set by the created thread
		down(&scl_sem);							 // wait for proxy to initialize
	}
	hdp->itmr.action_par = hdp;	
	hdp->ptmr = NULL;
    if (fn)
	{
		*fn = hdp->clock_sync_isr;					// CLOCK_SYNC_HARD
		hdp->isr_fp = fn;							// to cut the connection when unregistering
	    if (RT_INIT_VERBOSE) 
 		 	printkGen(NULL, "%s: registered HW clockid=%d period=%d\n", __func__, sync_clockid, (int)period);
	}	
	if (flags != CLOCK_SYNC_HARD)
	{												// period begin is based on timer
		tmr = get_tmr();
		itmr = &tmr->itmr;
		itmr->it_period = period;
		itmr->flags = 0;	
		itmr->lhdr = &__raw_get_cpu_var(isr_list_hdr);
		init_rt_notify(&hdp->rt_notify, SI_IOEVENT);
		itmr->timer_action = new_period_timer;	
		itmr->action_par = hdp;	
		hdp->ptmr = itmr;						
	    if (RT_INIT_VERBOSE) 
 		 	printkGen(NULL, "%s: registered SW clockid=%d period=%d\n", __func__, sync_clockid, (int)period);
	}
	hdp->clockid = sync_clockid;
	hdp->period = period;
errout:
	up(&scl_sem);	
	if (current->rt_state & RTLX_TASK)	
		rtx_migrate_to_rt(0);
	return sync_clockid;	
}

/*
 * This routine gets called from a device driver 
 * - when it receives a close from a process which has successfully registered.
 * - a sync_clock, meaning the release() routine of the driver has to call it. 
 *
 * This routine may be called from the RT-domain or the LX-domain (shutdown).
 */
int rt_unregister_sync_clock(struct file* fp, clockid_t cid)
{
	int i, clockid;
	struct rt_list_hdr *hdp;
	struct list_head *lp;
	struct itimer *ptmr;
	struct posix_timer *tmr;
	int lx_migration = 0;
	int clock_supported = 0;
	int ret = 0;

	if (IS_REALTIME) {	
		rtx_migrate_to_lx(RTLX_TASK, NULL);
		lx_migration = 1;
	}

	down(&scl_sem);		
	for (i = 0; i < MAX_SYNC_CLOCKS; i++) {
		hdp = &__raw_get_cpu_var(sync_clock_arr[i]);
		if ((hdp->fp && (fp == NULL)) || (fp && (hdp->fp == fp)))
		{
			if (fp) {
				/* Check if the clock is stopped and if all timers are deleted. */
				clock_supported++;
				if (cid != hdp->clockid)
					continue;
				/* When the clock is already stopped hdp->active is set to 0. 
				 * The aux_link queue is empty if all the timers are deleted. */
				if (GROUP_ACTIVE(hdp) || !(list_empty(&hdp->aux_link))) {
					ret = -EBUSY;
					goto out_err;
				}
			}
			if (hdp->isr_fp)
				*hdp->isr_fp = isr_noop;	// a device driver cannot call in anymore
			hdp->fp = NULL;
			hdp->itp = NULL;				// no new activation can take place
			hdp->active = 0;
			clockid = hdp->clockid;
			unregister_posix_clock(clockid);
			rt_clock_arr[clockid] = NULL;
			hdp->clockid = 0;
			if (rtx_get_cpu_var(rtx_mode) == LX_DOMAIN_SOFT)
				up(&hdp->sem);					// wake up proxy for termination (soft-RT)
			if ((ptmr = hdp->ptmr))
			{
				put_tmr(list_entry(ptmr, struct posix_timer, itmr));
				hdp->ptmr = NULL;
			}
			while (!(list_empty(&hdp->aux_link)))
			{
				lp = hdp->aux_link.next;
				list_del_init(lp);
				tmr = list_entry(lp, struct posix_timer, it_link);
				local_irq_disable_hw();
				FREE_TIMERID(tmr->it_id);
				put_tmr(tmr);
				local_irq_enable_hw();
				/* In case of LX there is still a reference from k_itimer objects,
				 * but it cannot get reached anymore. */
			}
		    if (RT_SHUTDOWN_VERBOSE)
	 		 	printkGen(NULL, "%s: unregistered clockid=%d\n", __func__, clockid);
			if (fp)
				break;
		}
	}
	if (fp && i >= MAX_SYNC_CLOCKS)
		ret = clock_supported ? -EINVAL : -EOPNOTSUPP;

out_err:
	up(&scl_sem);
	if (lx_migration)	
		rtx_migrate_to_rt(0);
	return ret;
}

EXPORT_SYMBOL(rt_register_sync_clock);
EXPORT_SYMBOL(rt_unregister_sync_clock);


