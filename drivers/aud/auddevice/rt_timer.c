/*
 * drivers/aud/auddevice/rt_timer.c
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
 * This file contains all the common functions POSIX timers and the specific
 * routines  for CLOCK_REALTIME timers.The specific routines
 * for the CLOCK_SYNC timers for the RT domain as well as the Linux domain are
 * in file sync_timer.c
 */
#include <linux/aud/rt_timer.h>
#include <linux/aud/rt_tgroup.h>

static int rt_timer_settime(struct posix_timer *, int, struct itimerspec *, struct itimerspec *);
static void clrt_timer_gettime(struct posix_timer *, struct itimerspec *);
static int clrt_timer_delete(struct posix_timer *);
static int common_timer_delete(struct posix_timer *);
static int rt_clock_settime_in_lx(clockid_t, struct timespec *);
static int clrt_clock_gettime(clockid_t, struct timespec *);
static int clrt_clock_getres(clockid_t, struct timespec *);
static int clmn_clock_settime(clockid_t, struct timespec *);
static int clmn_clock_gettime(clockid_t, struct timespec *);
static int assoc_timer_with_rt_list(struct posix_timer *, sigevent_t *);

DEFINE_PER_CPU(struct task_struct *, rt_timer_daemon);
DEFINE_PER_CPU(int, td_error);
DEFINE_PER_CPU(_INT64, rt_new_offset);
DEFINE_PER_CPU(_INT64, rt_clock_offset);
DEFINE_PER_CPU(RTX_TIMER_TYPE, rt_time_last);
DEFINE_PER_CPU(struct rt_semaphore, rt_tl_sem);
DEFINE_PER_CPU(struct itimer, lx_itmr);
static DEFINE_PER_CPU(struct task_struct *, delayedTask) = { NULL };

struct timespec rt_tomono;								// LX wall_to_monotonic

/* The CLOCK_REALTIME interrupt operates on struct itimer elements, which are queued at isr_list_hdr.it_link.
 * which is an object of type rt_list_hdr. Each of these timers may be
 * - an internal timer (rr_timer or clock_update_timer)
 * - an element of struct posix_timer
 * - an element of struct rt_list_hdr. In this case the rt_list_hdr is the head of a subqueue
 *   of timers. CLOCK_SYNC subqueue headers are also handled at ISR-level . The other subqueues
 *   are handled at thread level. SIGEV_THREAD headers are handled by the associated carrier thread.
 * This allows to maintain the best balance between realtime and performance
 */
struct lx_cmd get_lx_tod_cmd = {					// Note: currently unused
        .type = GET_LX_TOD,
};

struct rtg_clock rt_clock = {
		.clock_set = rt_clock_settime_in_lx,
		.clock_get = clrt_clock_gettime,
		.clock_getres = clrt_clock_getres,
		.nsleep = NULL,
		.timer_set = rt_timer_settime,
		.timer_del = clrt_timer_delete,
		.timer_get = clrt_timer_gettime,
		.timer_create = assoc_timer_with_rt_list
	};

struct rtg_clock mn_clock = {
		.clock_set = clmn_clock_settime,
		.clock_get = clmn_clock_gettime,
		.clock_getres = clrt_clock_getres,
		.nsleep = NULL,
		.timer_set = rt_timer_settime,
		.timer_del = clrt_timer_delete,
		.timer_get = clrt_timer_gettime,
		.timer_create = assoc_timer_with_rt_list
	};

DEFINE_PER_CPU(struct itimer, rr_timer) = {
	.it_period = 0LL,
};

#ifdef CONFIG_SMP
struct smp_inittime {
	_INT64 act_clock;
	_INT64 rt_clock_offset;
	_INT64 rt_new_offset;
	RTX_TIMER_TYPE rt_time_last;
};
#endif

struct rt_list_hdr *get_hdr(void)
{
	struct rt_list_hdr *hdp = NULL;
	local_irq_disable_hw();

	if (!list_empty(&(rtx_get_cpu_var(prcl)->available_list_hdr)))  			// check for available list headers
	{
		hdp = (struct rt_list_hdr *)rtx_get_cpu_var(prcl)->available_list_hdr.next;
		list_del_init(rtx_get_cpu_var(prcl)->available_list_hdr.next);
		rtx_get_cpu_var(prcl)->used_hdr++;
	}
	local_irq_enable_hw();
	return hdp;
}

static void put_hdr(struct rt_list_hdr *hdr)
{
	local_irq_disable_hw();
	list_add(&hdr->it_link, &(rtx_get_cpu_var(prcl)->available_list_hdr));
	rtx_get_cpu_var(prcl)->used_hdr--;
	local_irq_enable_hw();
}

static void clock_realtime_init(_INT64 *hosttime, _INT64 *cputime, _INT64 *p_tmp)
{
	struct timespec tspec;
 	struct timeval tval;

	do_gettimeofday(&tval);
	if (cputime)
		*cputime = RTX_TIMER_GET;
	tspec.tv_sec = tval.tv_sec;
	tspec.tv_nsec = tval.tv_usec * 1000;

	// Provide a positive value for tv_nsec.
	if (tspec.tv_nsec < rt_timer_res_nsec/2) {
		tspec.tv_sec--;
		tspec.tv_nsec += NSEC_PER_SEC;
 	}
	tspec.tv_nsec -= rt_timer_res_nsec/2;

	convert_to_internal(hosttime, &tspec);

	local_irq_disable_hw();
	update_act_clock();
	if (p_tmp)
		*p_tmp = __raw_get_cpu_var(act_clock);
	local_irq_enable_hw();
	return;
}

/*
 * Checks whether the clock with the given clockid exists and returns the pointer to the clock structure.
 */
static INLINE struct rtg_clock * rt_get_clock(clockid_t clockid)
{
	if ((clockid >= MAX_CLOCKS) || (clockid < 0))
		return NULL;
	return rt_clock_arr[clockid];
}

/*
 * Checks whether the timer with the given timerid exists for that process and returns the pointer to it.
 */
static INLINE struct posix_timer* rt_get_timer(timer_t timerid)
{
	struct posix_timer *tmr;

	if ((timerid & 0xffff) >= rtx_get_cpu_var(prcl)->max_timer)
		return NULL;

	if	((tmr = (struct posix_timer *) *(rtx_get_cpu_var(prcl)->tmr_map +(timerid & 0xffff))) != NULL) {
		if ((tmr->it_id == timerid) && (current->tgid == tmr->rt_notify.thread->tgid))
			return tmr;
	}
	return NULL;
}

/*
 * Copy a user time to the internal 64 value based on realtime clock
 * (used for timeouts a.s.o.)
 */
long copy_conv_to_internal(_INT64 *itr, struct timespec __user *uvalue)
{
	struct timespec value;
	if (rt_copy_from_user(&value, uvalue, sizeof(struct timespec)))
		return -EFAULT;
	return convert_to_internal(itr, &value);
}


/*
 * Copy the internal 64 value based on realtime clock to a user time
 * (used for timeouts a.s.o.)
 */
static long copy_conv_to_external(_INT64 *itr, struct timespec __user *uvalue)
{
	struct timespec value;

	convert_to_external(itr, &value);
	return rt_copy_to_user(uvalue, &value, sizeof(struct timespec));
}


static void activate_timer(struct itimer *itmr, _INT64 offset, int force_sched)
{
	unsigned long flags;

	local_irq_save_hw(flags);
	update_act_clock();
	itmr->activation_time = offset + __raw_get_cpu_var(act_clock); 	// only relative times may get specified
	if (enqueue_timer(itmr))
		CHECK_TIMER(force_sched);
	local_irq_restore_hw(flags);
}


/* For internal nanosleep or timeout timers in the real time domain.
 * The field id can be used for an arbitrary id. It can be retrieved again as tmr->it_id
 */
void setup_internal_timer(struct rt_list_hdr *lhdr, struct itimer *itmr, _INT64 value, int (*timer_action) (void *))
{
    itmr->flags = 0;
	itmr->it_period = 0LL;
	itmr->timer_action = timer_action;
	itmr->action_par = (void *)current;
	itmr->lhdr = lhdr;
	activate_timer(itmr, value, 0);
	return;
}

/*
 * There are three cases to be considered:
 * - There is no LX request in the timer queue, dequeue_timer()
 *   acts as a nop.
 * - There is a LX request in the timer queue, but it isn't
 *   the first element. So we dequeue it and forget it. Add
 *   the new LX request to the timer queue, using lx_itmr.
 * - There is a LX request in the timer queue and it is the
 *   first element. Thus this request caused the timer HW
 *   to be programmed. We only dequeue the request because
 *   we assume that the expiration time of the new request
 *   is sooner than the previous LX request. So it will cause
 *   the timer HW to be programmed.
 */
void rt_emulate_lx_tick(_INT64 rel_delay_ns, int (*timer_action)(void *))
{
	struct itimer *plxi;

	local_irq_disable_hw();
	plxi = &__raw_get_cpu_var(lx_itmr);
	// Remove a possibly existing request.
	// We do not check the timer HW.
	dequeue_timer(plxi);

	plxi->flags = 0;
	plxi->it_period = 0LL;
	plxi->timer_action = timer_action;
	plxi->action_par = NULL;
	plxi->lhdr = &__raw_get_cpu_var(isr_list_hdr);
	activate_timer(plxi, rel_delay_ns, 0);
	local_irq_enable_hw();
}

/*
 * Note: Interrupts must be off.
 */
void rt_release_emulated_lx_tick(void)
{
	// Remove the LX tick request.
	// If it was the first element
	// then reprogram the timer HW.
	if (dequeue_timer(&__raw_get_cpu_var(lx_itmr)))
		CHECK_TIMER(0);
}

_INT64 delete_internal_timer(struct itimer *itmr)
{
	_INT64 remaining_time = 0;

	local_irq_disable_hw();
	if (dequeue_timer(itmr))						// remove if still in active list
		CHECK_TIMER(0);
	local_irq_enable_hw();
	remaining_time = itmr->activation_time - __raw_get_cpu_var(act_clock);
	if (remaining_time < 0)
		remaining_time = 0;
	return remaining_time;
}

#ifdef CONFIG_RTX_THREAD_GROUP_SUPPORT
/*
 * Start the quota timer.
 * - It will be used as an interval timer (controls the assignment interval) or
 * - The timer controls the quota of a thread which is member of a thread group
 * Note: Interrupts must be off.
 */
void rt_set_quota_timer(_INT64 rel_delay_ns, int (*timer_action)(void *), struct task_struct *proc, rt_tgroup_head_t *pqh, int force_sched)
{
	struct itimer *pqit = &__raw_get_cpu_var(quota_itmr);

	// Remove a possibly existing request.
	dequeue_timer(pqit);

	pqit->flags = 0;
	pqit->it_period = 0LL;
	pqit->timer_action = timer_action;
	pqit->action_par = proc;			// may be NULL
	pqh->quota_group_active = (unsigned long)proc;
	pqit->lhdr = &__raw_get_cpu_var(isr_list_hdr);
	activate_timer(pqit, rel_delay_ns, force_sched);
}

/*
 * Stop the quota timer.
 * Note: Interrupts must be off.
 */
void rt_release_quota_timer(void)
{
	// Remove the quota tick request.
	// If it was the first element
	// then reprogram the timer HW.
	if (dequeue_timer(&__raw_get_cpu_var(quota_itmr)))
		CHECK_TIMER(0);
}
#endif


/*
 * Timer handling is based on list_hdr.
 * - for signaling type SIGEV_THREAD_ID the isr_list_hdr is statically assigned
 * - for internal timers the isr_list_hdr is also statically assigned
 * - for signaling type SIGEV_SIGNAL_SV (handler) a list_hdr is dynamically assigned
 * The isr_list_hdr is directly managed at the ISR level.
 * The other type is managed at thread level by the corresponding carrier threads.
 * This scheme of list headers allows for best possible RT behavior of time critical user threads.
 * The entire operation is called under rt_tl_sem lock.
 */
static int assoc_timer_with_rt_list (struct posix_timer *tmr, sigevent_t *rt_sig)
{
	struct rt_list_hdr *hdp;
	struct task_struct *rt_thread;

	/*
	 * All existing CLOCK_REALTIME/CLOCK_MONOTONIC notification paths
	 * can be found in this list.
	 */
	list_add(&tmr->it_link, &(rtx_get_cpu_var(prcl)->created_timer));

	/* Only SIGEV_SIGNAL_SV timers are handled at ISR level. */
	tmr->itmr.lhdr = &__raw_get_cpu_var(isr_list_hdr);
	if (rt_sig->sigev_notify != SIGEV_THREAD_ID)
	{
		if (RT_TIMER_VERBOSE)
			printkGen(NULL, "isr_list timer created clockid=%d tid=%d\n", tmr->it_clock, tmr->it_id);
		return 0;
	}
	if ((rt_thread = rt_find_task_by_pid(rt_sig->sigev_notify_thread_id)) == NULL)
		return -EINVAL;

	if ((hdp = rt_thread->rt_carrier_thread_hdr) != NULL)
	{
		tmr->itmr.lhdr = hdp;				// thread is already carrier thread and points to list_hdr
		if (RT_TIMER_VERBOSE)
			printkGen(NULL, "carrier list timer created clockid=%d tid=%d existing carrier pid=%d\n",
						(int)tmr->it_clock, tmr->it_id, rt_thread->pid);
		return 0;
	}
	if ((hdp = get_hdr()) == NULL)
	{
		if (RT_TIMER_VERBOSE)
			printkGen(NULL, "%s: run out of rt_list_hdr\n", __func__);
			return -EAGAIN;
	}
	init_rt_list(hdp);
	hdp->itmr.timer_action = (void*)rt_send_sigqueue;
	hdp->rt_notify.thread = rt_thread;
	rt_thread->rt_carrier_thread_hdr = hdp;		// indicator for a thread being a carrier thread (different handling of sigwait)
	tmr->itmr.lhdr = hdp;
    tmr->rt_notify.thread->rt_notify_state |= RT_TIMER_REGISTERED;
    if (tmr->rt_notify.alt_thread)
        tmr->rt_notify.alt_thread->rt_notify_state |= RT_TIMER_REGISTERED;
	if (RT_TIMER_VERBOSE)
		printkGen(NULL, "carrier list timer created clockid=%d tid=%d new carrier pid=%d\n",
						tmr->it_clock, tmr->it_id, rt_thread->pid);
	return 0;
}


/*
 * For fast allocation of a timer id a bit list is maintained. The bit number serves as the index in an array of pointers
 * to timers. The final timerid is composed of the bit number(lob) and a counter value (hob).
 * Because the bitlist is handled in entities of 32 bits, lock time stays short enough also for a larger number of timers.
 */
asmlinkage long sys_rt_timer_create(clockid_t clockid, sigevent_t __user *sigevent_ptr, timer_t __user *created_timer_id)
{
    struct posix_timer *tmr = NULL;
    struct itimer *itmr;
    struct notify *npr;
	sigevent_t rt_sig;
	int error = -EINVAL;
	int i;
	unsigned int bw, *bwp;
	timer_t timerid;
	struct rtg_clock  *clock;
	unsigned int mask = 0x1;

	if ((clock = rt_get_clock(clockid)) == NULL)
		return -EINVAL;
	if (!sigevent_ptr)
		return -EINVAL;
	if (rt_copy_from_user(&rt_sig, sigevent_ptr, sizeof(sigevent_t)))
		return -EFAULT;

	bwp = rtx_get_cpu_var(prcl)->bitmap;

	/* Get a new timer element. */
	if (!(tmr = get_tmr()))
	{
		if (RT_TIMER_VERBOSE)
			printkGen(NULL, "%s: run out of posix timers\n", __func__);
		return -EAGAIN;
	}

	down_rt(&__raw_get_cpu_var(rt_tl_sem));

	/*
	 * We always find a free slot, otherwise
	 * we didn't get a new timer element.
	 * Note: This is 64-bit safe, "int" has
	 * 32 bits.
	 */
	for (i = 0; i < rtx_get_cpu_var(prcl)->max_timer; i += 32)	// find a free slot in the table
	{															// scan the bit-list
		if ((bw = *bwp) == -1) {								// all bits set
			bwp++;												// next word
			continue;
		}
		for(; ((bw & mask) != 0); i++)							// may be optimized by assembly programming
			mask <<= 1;											// find first bit set instruction
		*bwp |= mask;
		break;
	}
	timerid = (((rtx_get_cpu_var(prcl)->id_count++ & 0xffff) << 16) | i);
	if (rt_copy_to_user(created_timer_id, &timerid, sizeof(timer_t))) {
		put_tmr(tmr);
		*bwp &= ~mask;
		up_rt(&__raw_get_cpu_var(rt_tl_sem));
		return -EFAULT;
	}
	up_rt(&__raw_get_cpu_var(rt_tl_sem));

	tmr->it_clock = clockid;
	tmr->it_id = timerid;			// needed by common_timer_delete()
	itmr = &tmr->itmr;
	itmr->lhdr = NULL;
	error =  -EINVAL;
	npr = &tmr->rt_notify;

	/* Creating a new notification path must be atomic with respect to
	 * adding/deleting threads from the realtime thread list. */
	down_rt(&__raw_get_cpu_var(rt_tl_sem));
	if ((npr->thread = rt_check_sigevent(rt_sig.sigev_notify, rt_sig.sigev_notify_thread_id, rt_sig.sigev_signo)))	{	// good sigevent structure ?
		if (!(error = rt_setup_notification((void **)&itmr->timer_action, timerid, &rt_sig, npr, SI_TIMER))) {
			if (!(error = clock->timer_create(tmr, &rt_sig))) {
				*(rtx_get_cpu_var(prcl)->tmr_map + i) = tmr;		// now the timer can be reached
				up_rt(&__raw_get_cpu_var(rt_tl_sem));
				return 0;
			}
		}
	}
	/* The timer is not fully created, so we must not use common_timer_delete(). */
	FREE_TIMERID(tmr->it_id);
	put_tmr(tmr);
	up_rt(&__raw_get_cpu_var(rt_tl_sem));
	return error;
}


static void clrt_timer_gettime(struct posix_timer *tmr, struct itimerspec *value)
{
	_INT64 remaining_time, period, cur_time;

	struct itimer *itmr = &tmr->itmr;

	if (RT_TIMER_VERBOSE)
		printkGen(NULL, "clrt_timer_gettime tid=%d\n", (unsigned int)tmr->it_id);

	local_irq_disable_hw();
	update_act_clock();
	cur_time = __raw_get_cpu_var(act_clock);
	remaining_time = cur_time > itmr->activation_time ? 0LL : itmr->activation_time - cur_time;
	period = itmr->it_period;
	local_irq_enable_hw();

	convert_to_external(&remaining_time, &value->it_value);
	convert_to_external(&period, &value->it_interval);
	return;
}

asmlinkage long sys_rt_timer_gettime(timer_t timerid, struct itimerspec __user *uvalue)
{
	struct posix_timer *tmr;
	struct itimerspec rtn;

	if ((tmr = rt_get_timer(timerid)) == NULL)
		return -EINVAL;
	rt_clock_arr[tmr->it_clock]->timer_get(tmr, &rtn);
	if (uvalue)
		return rt_copy_to_user(uvalue, &rtn, sizeof(struct itimerspec)) ? -EFAULT : 0;
	return 0;
}

static int rt_timer_settime(struct posix_timer *tmr, int flags, struct itimerspec __user *value, struct itimerspec __user *ovalue)
{
	_INT64 it_interval, it_value;
	struct itimer *itmr = &tmr->itmr;
	long error, chk_mark;
	struct timespec cl_mono;

	if (RT_TIMER_VERBOSE)
		printkGen(NULL, "%s: tid=%d\n", __func__, (unsigned int)tmr->it_id);

	if (ovalue != NULL)
		clrt_timer_gettime(tmr, ovalue); 							// read back old value
	if ((error = convert_to_internal(&it_interval, &value->it_interval)) < 0)
		return error;
	if ((tmr->it_clock == CLOCK_MONOTONIC) && (flags & TIMER_ABSTIME)) {
		set_normalized_timespec(&cl_mono, value->it_value.tv_sec - rt_tomono.tv_sec, value->it_value.tv_nsec - rt_tomono.tv_nsec);
		if ((error = convert_to_internal(&it_value, &cl_mono)) < 0)
			return error;
	}
	else
		if ((error = convert_to_internal(&it_value, &value->it_value)) < 0)
			return error;

	if (it_interval && (it_interval < IT_INTERVAL_MIN))
		return -EINVAL;

	local_irq_disable_hw();

	// Between getting the pointer and locking a delete may have taken place,
	// therefore we check again inside the lock.
	if (tmr->it_state & RT_TIMER_DELETED) {
		local_irq_enable_hw();
		return -EINVAL;
	}

	// Remove timer request if queued in active list.
	chk_mark = dequeue_timer(itmr);
	// Do not remove an already queued signal.
	if (!it_value) {
		flags = 0;
		goto out_check;
	}
	update_act_clock();
	itmr->it_period = it_interval;
	if (!(flags & TIMER_ABSTIME))
		itmr->activation_time = it_value + __raw_get_cpu_var(act_clock) + 1;
	else
	{
		if (tmr->it_clock == CLOCK_REALTIME) {
			it_value -= __raw_get_cpu_var(rt_clock_offset);
			tmr->insert_offset = __raw_get_cpu_var(rt_clock_offset);
		}

		/* Time has already passed? */
		if (__raw_get_cpu_var(act_clock) > it_value)
			it_value = __raw_get_cpu_var(act_clock);
		itmr->activation_time = it_value;
	}
	itmr->flags = flags;
	if (tmr->rt_notify.sigev_notify == SIGEV_NONE) {
		flags = 0;
		goto out_check;
	}
	chk_mark |= enqueue_timer(itmr);
out_check:
	if (chk_mark) {
		CHECK_TIMER(0);
		rt_schedule(0);
	}
	local_irq_enable_hw();
	if ((flags & TIMER_ABSTIME) && (tmr->it_clock == CLOCK_REALTIME))
	{
		down_rt(&__raw_get_cpu_var(rt_tl_sem));
        if (list_empty(&tmr->abs_link))
		    list_add(&tmr->abs_link, &__raw_get_cpu_var(isr_list_hdr).aux_link);
		up_rt(&__raw_get_cpu_var(rt_tl_sem));
	}
	return error;
}

asmlinkage long sys_rt_timer_settime(timer_t timerid, int flags, struct itimerspec __user *uvalue, struct itimerspec __user *ovalue)
{
	struct posix_timer *tmr;

	struct itimerspec new_spec, old_spec;
	int error = 0;
	struct itimerspec *rtn = ovalue ? &old_spec : NULL;

	if (!uvalue)
		return -EINVAL;
	if (rt_copy_from_user(&new_spec, uvalue, sizeof (struct itimerspec)))
        return -EFAULT;
	if (!(tmr = rt_get_timer(timerid)))
		return -EINVAL;

	tmr->it_state = 0;
	if ((error = rt_clock_arr[tmr->it_clock]->timer_set(tmr, flags, &new_spec, rtn)))
		return error;
	if (rtn)
		return rt_copy_to_user(ovalue, &old_spec, sizeof(struct itimerspec)) ? -EFAULT : 0;
	return 0;
}

/*
 * Note: It is assumed that rcl_list_lock is already locked.
 */
static int clrt_timer_delete(struct posix_timer *tmr)
{
	if (RT_TIMER_VERBOSE)
		printkGen(NULL, "clrt_timer_delete tid=%d\n", tmr->it_id);
	rt_dequeue_sigq(&tmr->rt_notify.sigq);			// POSIX allows to cut corners
	local_irq_disable_hw();
	list_del_init(&tmr->it_link);					// remove from created list
	list_del_init(&tmr->abs_link);					// remove if in abs list
	tmr->it_state = RT_TIMER_DELETED;
	tmr->itmr.flags = 0;
	if (dequeue_timer(&tmr->itmr))					// remove it if in active list
		CHECK_TIMER(0);
	local_irq_enable_hw();
	put_tmr(tmr);								// put it back into available list
	return 0;
}

/*
 *  It may be called either from sys_rt_timer_delete(), rt_delete_timer_notification() or sys_rt_timer_create().
 *  Note: It is assumed that rcl_list_lock is already locked.
 */
static int common_timer_delete(struct posix_timer *tmr)
{
	int ret_val = 0;

	if (tmr->itmr.lhdr)									// only if the timer was fully created
		ret_val = rt_clock_arr[tmr->it_clock]->timer_del(tmr);	// clrt_timer_delete or sync_timer_delete
	if (!ret_val)
		FREE_TIMERID(tmr->it_id);
	return ret_val;
}


asmlinkage long sys_rt_timer_delete(timer_t timerid)
{
	struct posix_timer *tmr;
	int ret_val;

	if ((tmr = rt_get_timer(timerid)) == NULL)
		return -EINVAL;
	down_rt(&__raw_get_cpu_var(rt_tl_sem));
	ret_val = common_timer_delete(tmr);
	up_rt(&__raw_get_cpu_var(rt_tl_sem));
	return ret_val;
}


asmlinkage long sys_rt_clock_settime(clockid_t clockid, struct timespec __user *value)
{
	struct rtg_clock *clock;
	struct timespec new_spec;

	if (!(clock = rt_get_clock(clockid)))
		return -EINVAL;
	if (rt_copy_from_user(&new_spec, value, sizeof (struct timespec)))
        return -EFAULT;
	if (clock->clock_set)
		return clock->clock_set(clockid, &new_spec);
	return -EPERM;
}

static int rt_clock_settime_in_lx(clockid_t clockid, struct timespec *tp)
{
	rtx_migrate_to_lx(RTLX_TASK, NULL);
	return 0;
}

static int clmn_clock_settime(clockid_t clockid, struct timespec *tp)
{
	return -EINVAL;
}


/*
 * All abs_timers need to get adjusted. There may be also entries in the queue which are not
 * active TIMER_ABSTIMER timers anymore. They get just removed from the queue.
 */
static int rt_clock_settime(void)
{
	struct posix_timer *tmr;
	struct list_head *lp;
	int chk_mark;
	struct rt_list_hdr *pilh = &__raw_get_cpu_var(isr_list_hdr);
	_INT64 *prco = &__raw_get_cpu_var(rt_clock_offset);

	*prco = __raw_get_cpu_var(rt_new_offset);
	__raw_get_cpu_var(rt_new_offset) = 0LL;
	down_rt(&__raw_get_cpu_var(rt_tl_sem));
	for (lp = pilh->aux_link.next; lp != &pilh->aux_link;)
	{
		tmr = list_entry(lp, struct posix_timer, abs_link);
		lp = lp->next;

		if (!(tmr->itmr.flags & TIMER_ABSTIME))
		{
			if (RT_TIMER_VERBOSE)
				printkGen(NULL, "remove non abs timer tid=%d\n", tmr->it_id);
			list_del_init(&tmr->abs_link);
			continue;
		}
		else
		{
			if (RT_TIMER_VERBOSE)
				printkGen(NULL, "update abs timer tid=%d\n", tmr->it_id);
			if (*prco - tmr->insert_offset)
			{
				local_irq_disable_hw();
				chk_mark = dequeue_timer(&tmr->itmr);
				tmr->itmr.activation_time -= (*prco - tmr->insert_offset);
				tmr->insert_offset = *prco;
				update_act_clock();
				if (chk_mark |= enqueue_timer(&tmr->itmr))
					CHECK_TIMER(0);
				local_irq_enable_hw();
			}
		}
	}
	up_rt(&__raw_get_cpu_var(rt_tl_sem));
	return 0;
};

#ifdef CONFIG_SMP
/* This function is always invoked on the rt_timer_cpu
 * and provides the actual RT timer values for
 * other CPUs a RT-process is assigned to. */
static void smp_rt_clock_init(void *cmd)
{
	struct smp_inittime *pinfo = (struct smp_inittime *)cmd;

	local_irq_disable_hw();
	pinfo->act_clock = __raw_get_cpu_var(act_clock);
	pinfo->rt_clock_offset = __raw_get_cpu_var(rt_clock_offset);
	pinfo->rt_new_offset = __raw_get_cpu_var(rt_new_offset);
	pinfo->rt_time_last = __raw_get_cpu_var(rt_time_last);
	local_irq_enable_hw();
}

/* This function is always invoked on a non-rt_timer_cpu
 * and is asking for the RT timer values of the rt_timer_cpu. */
void rt_timer_cpu_gettime(void)
{
	struct smp_inittime info;

	/* Get the values by synchronized access and wait for the result. */
    smp_call_function_single(rt_timer_cpu, smp_rt_clock_init, (void *)&info, 1);
	__raw_get_cpu_var(act_clock) = info.act_clock;
	__raw_get_cpu_var(rt_clock_offset) = info.rt_clock_offset;
	__raw_get_cpu_var(rt_new_offset) = info.rt_new_offset;
	__raw_get_cpu_var(rt_time_last) = info.rt_time_last;
}
#endif

/*
* Initialize CLOCK_REALTIME on startup.
*/
void rt_clock_init(void)
{
	_INT64 cpu_time, host_time = 0;

#ifdef CONFIG_SMP
	if (rt_timer_cpu == RT_TIMER_CPU_NONE) {
		rt_timer_cpu = current->rt_proc->cpu;
#endif
		__raw_get_cpu_var(act_clock) = 0;
	
		clock_realtime_init(&host_time, &cpu_time, NULL);
	
		// Initial setting of the clock in the RT domain.
		INIT_LIST_HEAD(&get_lx_tod_cmd.req_link);			// Note: currently unused
		__raw_get_cpu_var(rt_time_last) = RTX_TIMER_GET;
		__raw_get_cpu_var(act_clock) = host_time + (__raw_get_cpu_var(rt_time_last) - cpu_time);
		__raw_get_cpu_var(rt_clock_offset) = 0;
#ifdef CONFIG_SMP
	}
	else {
		/* Get the time from rt_timer_cpu. */
		rt_timer_cpu_gettime();
	}
#endif
	return;
}

#ifdef CONFIG_SMP
/* The CLOCK_REALTIME of LX has been changed. All the non-rt_timer_cpu
 * CPUs a RT-process is assigned to have to adopt the new time. */
void rt_adopt_time(void *arg)
{
	int *wakeup_td = (int *)arg;

	if (!(rtx_get_cpu_var(rt_system_state) & RT_TIMER_INITIALIZED))
		return;

	if (rt_timer_cpu == rtx_processor_id())
		return;

	/* Get time from rt_timer_cpu. */
	rt_timer_cpu_gettime();

	if (*wakeup_td && rtx_get_cpu_var(rt_timer_daemon) != NULL) {
		lx_wake_up_rt_thread(rtx_get_cpu_var(rt_timer_daemon));
	}
}
#endif

/*
 * This routine is executed in the Linux domain either
 * - by a thread which executes clock_settime for CLOCK_REALTIME
 *   (if called in the realtime domain its execution is delegated to Linux)
 * - when the interdomain daemon gets a GET_LX_TOD request which is set up
 *   checking for drifts (which depends on the timer architecture). In case
 *   of the same clock device for LX and RT (e.g. x86: LAPIC timer), there
 *   is no drift checking necessary. Note: currently unused.
 */
void rtx_lx_clock_was_set(void)
{
	int wakeup_td = 0;

#ifdef CONFIG_SMP
	down(&aud_rt_hdr.dev_sem);
#endif
	/* Trigger the timer update on the rt_timer_cpu. */
	on_each_cpu(rtx_synchronize_time, &wakeup_td, 1);
#ifdef CONFIG_SMP
	/* Now we are sure that the rt_timer_cpu has already been updated.
	 * All the other CPUs a RT-process is assigned to
	 * have to take over the time from the rt_timer_cpu. */
	on_each_cpu(rt_adopt_time, &wakeup_td, 1);

	up(&aud_rt_hdr.dev_sem);
#endif
}

/*
 * Called on each cpu. Has to check whether there
 * is an active RT-process.
 */
void rtx_synchronize_time(void *arg)
{
	_INT64 offset, act_clock_tmp;
	_INT64 act_clock_norm;
	_INT64 clock_val = 0;
#ifdef CONFIG_SMP
	int *wakeup_td = (int *)arg;
#endif

	if (!(rtx_get_cpu_var(rt_system_state) & RT_TIMER_INITIALIZED))
		return;

#ifdef CONFIG_SMP
	/* Make sure that this code will be invoked only by one CPU.
	 * Note: dev_sem must already be locked to prevent that
	 * rt_timer_cpu may change. */
	if (rt_timer_cpu != rtx_processor_id())
		return;
#endif

	clock_realtime_init(&clock_val, NULL, &act_clock_tmp);

	act_clock_norm = act_clock_tmp + __raw_get_cpu_var(rt_clock_offset);
	if (clock_val > act_clock_norm)
		offset = clock_val - act_clock_norm;
	else
		offset = act_clock_norm - clock_val;
	if (offset < TIME_CORR_THRESHOLD)
		return;

	__raw_get_cpu_var(rt_new_offset) = clock_val - act_clock_tmp;

	/* Wakeup the timer daemon. */
	if (rtx_get_cpu_var(rt_timer_daemon) != NULL) {
#ifdef CONFIG_SMP
		if (wakeup_td)
			*wakeup_td = 1;
#endif
		lx_wake_up_rt_thread(rtx_get_cpu_var(rt_timer_daemon));
	}
}

/*
 * Get value into kernel space.
 */
long rtx_clock_gettime_internal(struct timespec *value)
{
	_INT64 real_time;

	local_irq_disable_hw();
	update_act_clock();
	real_time = __raw_get_cpu_var(act_clock) + __raw_get_cpu_var(rt_clock_offset);
	local_irq_enable_hw();
	return convert_to_external(&real_time, value);
};


static int clrt_clock_gettime(clockid_t clockid, struct timespec *value)
{
	rtx_clock_gettime_internal(value);
	if (RT_TIMER_VERBOSE)
		printkGen(NULL, "%s: sec=%d ns=%ld\n", __func__, value->tv_sec, value->tv_nsec);
	return 0;
};

static int clmn_clock_gettime(clockid_t clockid, struct timespec *value)
{
	_INT64 real_time;

	local_irq_disable_hw();
	update_act_clock();
	real_time = __raw_get_cpu_var(act_clock);
	local_irq_enable_hw();
	convert_to_external(&real_time, value);
	set_normalized_timespec(value, value->tv_sec + rt_tomono.tv_sec, value->tv_nsec + rt_tomono.tv_nsec);
	if (RT_TIMER_VERBOSE)
		printkGen(NULL, "%s: sec=%d ns=%ld\n", __func__, value->tv_sec, value->tv_nsec);
	return 0;
};

asmlinkage long sys_rt_clock_gettime(clockid_t clockid, struct timespec __user *value)
{
	struct rtg_clock *clock;
	struct timespec clock_val;

	if (!(clock = rt_get_clock(clockid)))
		return -EINVAL;
	clock->clock_get(clockid, &clock_val);
	return rt_copy_to_user(value, &clock_val, sizeof(struct timespec)) ? -EFAULT : 0;
}

static int clrt_clock_getres(clockid_t clockid, struct timespec *value)
{
	if (RT_TIMER_VERBOSE)
		printkGen(NULL, "%s: resolution=%d nsec\n", __func__, rt_timer_res_nsec);
	value->tv_nsec = rt_timer_res_nsec;
	value->tv_sec = 0;
	return 0;
};


asmlinkage long sys_rt_clock_getres(clockid_t clockid, struct timespec __user *uvalue)
{
	struct rtg_clock *clock;
	struct timespec clock_res;

	if (uvalue == NULL)
		return 0;

	if (!(clock = rt_get_clock(clockid)))
		return -EINVAL;
	clock->clock_getres(clockid, &clock_res);
	return rt_copy_to_user(uvalue, &clock_res, sizeof(struct timespec)) ? -EFAULT : 0;
}

/*
*  This system call is added to be supported by the AuD-API. We also support
*  absolute times with CLOCK_REALTIME with the following limitation:
*  If the absolute time of the system is changed after waiting for the timer
*  to expire the timer request is _not_ adjusted (isr_list_hdr does not handle the
*  adjusting of absolute time requests if somebody changes the absolute time).
*  For a more suitable implementation of this case we have to apply
*  Posix timer structures.
*/
asmlinkage long sys_rt_clock_nanosleep(const clockid_t which_clock, int flags,
		    const struct timespec __user *rqtp,
		    struct timespec __user *rmtp)
{
	struct rtg_clock *clock;
	struct timespec t, nowts;
	_INT64 expire;
	long err;

	if (!(clock = rt_get_clock(which_clock)))
		return -EINVAL;

	// We only support CLOCK_REALTIME and CLOCK_MONOTONIC.
	if (which_clock != CLOCK_REALTIME && which_clock != CLOCK_MONOTONIC)
		return -ENOSYS;

	if (rt_copy_from_user(&t, rqtp, sizeof (struct timespec)))
		return -EFAULT;

	if (!timespec_valid(&t))
		return -EINVAL;

	// Convert an absolute time into a relative time.
	// FIXME: We currently do not consider the situation
	// when CLOCK_REALTIME gets shifted while waiting
	// for the timer to fire.
	if (flags & TIMER_ABSTIME) {
		clock->clock_get(which_clock, &nowts);
		t.tv_sec -= nowts.tv_sec;
		if (t.tv_nsec < nowts.tv_nsec) {
			t.tv_nsec += NSEC_PER_SEC;
			t.tv_sec--;
		}
		t.tv_nsec -= nowts.tv_nsec;
		if (t.tv_sec < 0)
			return 0;
	}
	if ((err = convert_to_internal(&expire, &t)))
		return err;

	if (expire == 0)  // no wait time
		return 0;
	
	if (current->rt_state3 & RT_TASK_LX_DEBUG_PENDING) {
		// We migrated from LX to RT and got a debugging event.
        // FIXME: do a ERESTART_RESTARTBLOCK (see below)
		return 0;
	}

	__set_current_rt_state(RT_TASK_UNINTERRUPTIBLE);
	expire = rt_schedule_timeout(&__raw_get_cpu_var(isr_list_hdr), expire);

	if (expire) {
		if (rmtp && !(flags & TIMER_ABSTIME)) {
			if (copy_conv_to_external(&expire, rmtp))
				return -EFAULT;
		}
		// We have been interrupted (debugging).
		// Now we restart the system call.
		return -ERESTARTNOINTR;
	}
	return 0;
}

asmlinkage long sys_rt_nanosleep(struct timespec __user *rqtp, struct timespec __user *rmtp)
{
	_INT64 expire;
	long ret;

	if ((ret = copy_conv_to_internal(&expire, rqtp)) < 0)
		return ret;

	if (expire == 0)  // no wait time
		return 0;
	if (current->rt_state3 & (RT_TASK_LX_DEBUG_PENDING | RT_TASK_RT_DEBUG_PENDING)) {
		// We migrated from LX to RT and got a debugging event.
		// FIXME: do a ERESTART_RESTARTBLOCK (see below)
		return 0;
	}
	__set_current_rt_state(RT_TASK_UNINTERRUPTIBLE);

	expire = rt_schedule_timeout(&__raw_get_cpu_var(isr_list_hdr), expire);
	if (expire) {
		if (rmtp) {
			if (copy_conv_to_external(&expire, rmtp))
				return -EFAULT;
		}
		// We have been interrupted (debugging).
		// FIXME: restart the system call with the remaining time.
		// return -ERESTART_RESTARTBLOCK;
	}
	return 0;
}

asmlinkage time_t sys_rt_time(time_t __user *tp)
{
	struct timespec tval;

	rtx_clock_gettime_internal(&tval);

	if (tp) {
		if (rt_copy_to_user(tp, &tval.tv_sec, sizeof(time_t)))
			return -EFAULT;
	}
	return tval.tv_sec;
}

asmlinkage long sys_rt_gettimeofday(struct timeval __user *tv, struct timezone __user *tz)
{
	struct timespec tval;

	if (likely(tv != NULL)) {
		rtx_clock_gettime_internal(&tval);
		tval.tv_nsec /= 1000;
		if (rt_copy_to_user(tv, &tval, sizeof(struct timeval)))
			return -EFAULT;
	}
	if (unlikely(tz != NULL)) {
		if (rt_copy_to_user(tz, &sys_tz, sizeof(struct timezone)))
			return -EFAULT;
	}
	return 0;
}


/*
 * This routine gets called when "change priority" call would cause a migration to Linux
 * since only a thread which is not a notification target is allowed to migrate
 * Note: This routine is called under rt_tl_sem lock.
 */
int rt_notification(struct task_struct *p)
{
	struct list_head *tp;
	struct posix_timer *tmr;
	int ret;

	for (tp = rtx_get_cpu_var(prcl)->created_timer.next; tp != &(rtx_get_cpu_var(prcl)->created_timer); tp = tp->next) {
		tmr = (struct posix_timer*)tp;
		if ((tmr->rt_notify.thread == p) || (tmr->rt_notify.alt_thread == p)) {
			return 1;
		}
	}
	ret = rt_event_notification(p);
	ret |= check_sync_timer_notification(p);

	/* reset notify state to speed up future priority changes */
	if (!ret) {
		p->rt_notify_state = 0;
	}
	return ret;
}


/*
 * Deletes all timer based notifications to the current thread.
 * In addition if the current thread is a carrier thread, the associated list_hdr gets deleted.
 * Note: It is assumed that rt_tl_sem lock is already locked.
 */
void rt_delete_timer_notification(struct task_struct *p)
{
	struct list_head *tp;
	struct posix_timer *tmr;

	for (tp = rtx_get_cpu_var(prcl)->created_timer.next; tp != &(rtx_get_cpu_var(prcl)->created_timer); ) {
		tmr = list_entry(tp, struct posix_timer, it_link);
		tp = tp->next;
		if ((tmr->rt_notify.thread == p) || (tmr->rt_notify.alt_thread == p))
			common_timer_delete(tmr);
	}

	if (p->rt_carrier_thread_hdr)					// does a carrier thread get deleted ?
	{
		put_hdr(p->rt_carrier_thread_hdr);			// then recycle the associated list-header
		p->rt_carrier_thread_hdr = NULL;
	}
	clock_sync_delete_timer_notification(p);
	return;
}

/*
 * This routine handles a rt_list_hdr at the thread level.
 * It is called under lock.
 */
struct itimer *get_timer_elapsed(struct rt_list_hdr *hdr)
{
	struct list_head *tp;
	struct itimer *itmr;

	if ((tp = hdr->it_link.next) != &hdr->it_link)
	{
		update_act_clock();
		itmr = list_entry(tp, struct itimer, it_link);

		if ((__raw_get_cpu_var(act_clock) + aud_rt_timer->gravity) < itmr->activation_time)
		{
			if (list_empty(&hdr->itmr.it_link))				// another instance may have executed enqueue_timer()
			{
				hdr->itmr.activation_time = itmr->activation_time;
				if (enqueue_timer(&hdr->itmr))					// queue the secondary into the primary again
					CHECK_TIMER(0);
			}
			return NULL;
		}
		list_del_init(&itmr->it_link);
		itmr->flags = 0;									// can't be absolute time anymore
		if (itmr->it_period > 0LL)
		{
			itmr->activation_time += itmr->it_period;
			insert_timer(hdr, itmr);						// check_timer will be done next run
		}
		return itmr;
	}
	// Handle sync-timer carrier threads.
	return get_sync_timer_elapsed(hdr);
}

/*
 * The execution interval for rt_sched_rr() is calculated by RR_INTERVAL * RT_TIME_SLICE_VAL.
 * If there are RT-threads that have migrated to the LX domain and the RT priority of
 * the thread with the highest priority is higher than the priority of the current thread,
 * then sleep for a while to give LX a chance to proceed.
 *
 * There are two reasons for RT-threads migrating to the LX domain:
 * - Execution of a system call which is not provided by the RT domain.
 * - Exit handling of a RT-thread.
 *
 */
static int schedule_rr(void *arg)
{
	if (rtx_get_cpu_var(delayedTask)) {
		/* The thread may have left the RT-domain by now.*/
		if (rtx_get_cpu_var(delayedTask)->rt_state & RT_TASK_UNINTERRUPTIBLE)	
			rt_wake_up_thread(rtx_get_cpu_var(delayedTask));
		rtx_set_cpu_var(delayedTask, NULL);
	}

	/*
	* Previously we tested for (IS_REALTIME).
	* However, we should not do anything, when the thread is on the way to wait
	* on a (realtime) kernel waiting point:
	* Our delay strategy may cause an unwanted restart from this waiting point
	* - and cause incorrect error returns, e.g. during sys_rt_sigtimedwait_rt().
	* Therefore, we only do a round-robin scheduling test, when the thread is
	* really running.
	* Ideally, we should only do this, when we are in user mode. However,
	* the current call back of a timer doesn't provide us with the register set.
	* So we have to accept the problem that a waiting rr thread my own a
	* kernel semaphore a.o. for a certain time and block other threads in the kernel.
	*/
	if ((current->rt_state & RT_TASK_RUNNING) &&
			(current->policy == SCHED_RR) && !--current->rt.time_slice)
	{
		if (rt_sched_has_delay()) {
			current->rt.time_slice = RT_TIME_SLICE_VAL;
			rtx_set_cpu_var(delayedTask, current);
			__set_current_rt_state(RT_TASK_UNINTERRUPTIBLE);
			rt_sched_rr_delay();
		}
		else {
			current->rt.time_slice = RT_TIME_SLICE_VAL;
			rt_sched_rr();
		}
		/*
		* Since we are called from (realtime) interrupt context a
		* scheduling change will be done with rtx_preemption_handling
		* at the end of the interrupt handling.
		*/
	}
	return 0;
}

int setup_rr_timer(struct task_struct *pts)
{
	struct itimer *prr = &__raw_get_cpu_var(rr_timer);
	unsigned long flags;

	pts->rt.time_slice = RT_TIME_SLICE_VAL;

	local_irq_save_hw(flags);
	if (prr->it_period) {
		local_irq_restore_hw(flags);
		return 0;
	}
	prr->it_period = RR_INTERVAL;				// every 10 ms
	local_irq_restore_hw(flags);

	INIT_LIST_HEAD(&prr->it_link);
	prr->flags = 0;
	prr->timer_action = schedule_rr;
	prr->action_par = NULL;
	prr->lhdr = &__raw_get_cpu_var(isr_list_hdr);
	activate_timer(prr, RR_INTERVAL, 0);
	return 0;
}

/*
 * Depends on the system architecture:
 * - The LAPIC timer implementation doesn't need this functionality because LX and RT have the same time
 *   base. The LX tick is emulated by the RT system.
 */
int setup_clock_periodic_timer(void)
{
	struct rt_list_hdr *pilh;

	local_irq_disable_hw();
	pilh = &__raw_get_cpu_var(isr_list_hdr);
	INIT_LIST_HEAD(&pilh->itmr.it_link);
	pilh->itmr.it_period = aud_rt_timer->preq.period;
	pilh->itmr.flags = 0;
	pilh->itmr.timer_action = aud_rt_timer->preq.timer_action;
	pilh->itmr.action_par = aud_rt_timer->preq.action_par;
	pilh->itmr.lhdr = pilh;
    activate_timer(&pilh->itmr, aud_rt_timer->preq.period, 0);
	local_irq_enable_hw();
	return 0;
}


/* This daemon starts in the LX domain migrates to the RT domain. Here it handles all the timeout timers for time bound calls
 * (futex, sigtimedwait, mq_send and mq_receive calls).
 * In addition it sets up a timer which periodically checks for synchrony of CLOCK_REALTIME between LX and RT.
 * At the very end it manages the shutdown by marking the threads and waking them up for migration and exit
 */
int rt_timer_daemon_thread(void *arg)
{
	struct sched_param sched_par;

	strcpy(current->comm, TIMER_DAEMON_NAME);
	sched_par.sched_priority = aud_config.td_prio.val;
	current->rt_state3 |= RT_TASK_ALLOW_SETSCHEDULER;
	sched_setscheduler(current, SCHED_FIFO, &sched_par);
	current->rt_state = RT_TIMER_DAEMON_STATE;
	INIT_LIST_HEAD(&current->rt_mig_list);

	rt_clock_arr[CLOCK_REALTIME] = &rt_clock;
	rt_clock_arr[CLOCK_MONOTONIC] = &mn_clock;
#ifdef CONFIG_RTX_THREAD_GROUP_SUPPORT
	rt_tgroup_init();
#endif

	if (rtx_migrate_to_rt(0) == 0)
	{
		printkGen(KERN_ALERT, "timer daemon could not migrate to RT domain\n");
		rtx_set_cpu_var(td_error, -1);
		return rtx_get_cpu_var(td_error);
	}
	// From here we are executing in the RT-domain
#ifdef CONFIG_RTX_THREAD_GROUP_SUPPORT
	INIT_LIST_HEAD(&current->rt_thread_list);
	INIT_LIST_HEAD(&current->rt_blocked_list);
#endif
	rtx_set_cpu_var(rt_timer_daemon, current);

	if (RT_INIT_VERBOSE)
		printkGen("(td)", "timer daemon installed prio=%d:%d\n", current->rt_priority, current->prio);

	while (!(rtx_get_cpu_var(rt_system_state) & RT_SHUTDOWN)) {
		rt_schedule(RT_TASK_UNINTERRUPTIBLE);
		if (rtx_get_cpu_var(rt_system_state) & RT_SHUTDOWN)
			break;
		// CLOCK_REALTIME of lx has some deviation caused by clock_settime.
		if (__raw_get_cpu_var(rt_new_offset))
			rt_clock_settime();
	}
#ifdef CONFIG_RTX_THREAD_GROUP_SUPPORT
	rt_tgroup_destroy();
#endif

	if (RT_SHUTDOWN_VERBOSE)
		printkGen("(td)", "timer daemon shutting down\n");

	rtx_set_cpu_var(rt_timer_daemon, NULL);
	rtx_migrate_to_lx(LXRT_DAEMON, NULL);
	sched_par.sched_priority = 0;
	current->rt_state3 |= RT_TASK_ALLOW_SETSCHEDULER;
	sched_setscheduler(current, SCHED_NORMAL, &sched_par);
	return 0;
}

