/*
 * linux/aud/rt_timer.h
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
 * Timer specific definitions and common functions.
 */
#ifndef _LINUX_AUD_RT_TIMER_H
#define _LINUX_AUD_RT_TIMER_H

#include <linux/aud/rt_base.h>

#include <linux/timex.h>
#include <linux/timer.h>
#include <linux/posix-timers.h>
#include <linux/rtx_system.h>
#include <linux/aud/rt_driver.h>
#include <asm/io.h>
#include <asm/rtx_timer.h>

#define FAST_POLL_TIME	(usecs_to_jiffies(1))		// waiting for a LX task to be out of the runqueue
#define SLOW_POLL_TIME	(msecs_to_jiffies(1))		// only for daemon synchronization

#define RT_TIMER_DELETED		0x80000000
#define SI_LHDR					0xf0f0f0f0		// si_code distinction for list_hdr notification and sync-timer notification
#define SYNC_RCVD				4
#define RT_TIME_SLICE_VAL		1
#define CLOCK_SYNC_GRANULARITY	99			// within a CLOCK_SYNC period a reasonable timer resolution is required
#define GROUP_ACTIVE(hdr)       (hdr && hdr->active)

// All the stuff which doesn't need ultraprecision.
#ifdef CONFIG_RTX_DOMAIN_HRT
#define NSEC_TO_RT_JIFFIES(val)		RTX_NSEC_TO_RT_JIFFIES(val)
#else
#define NSEC_TO_RT_JIFFIES(val)		((val)/rt_timer_res_nsec == 0 ? 1 : (val)/rt_timer_res_nsec)
#endif
#define RT_PERIOD_MAX		   		(NSEC_TO_RT_JIFFIES(RT_MAX_INT))
#define RT_PERIOD_MIN		   		(NSEC_TO_RT_JIFFIES(250000))
#define IT_INTERVAL_MIN		   		(NSEC_TO_RT_JIFFIES(10000))
#define RR_INTERVAL					(NSEC_TO_RT_JIFFIES(CONFIG_RTX_SCHED_RR_TIMESLICE_US*1000))
#define SLEEP_TIME_YIELD	 		(NSEC_TO_RT_JIFFIES(1000000))	// 1 ms
#define YIELD_MAX_DELAY_OF_SLEEP	10
#define TIME_CORR_THRESHOLD			(NSEC_TO_RT_JIFFIES(1000000))	// 1 ms
#define CLOCK_UPDATE_INTERVAL		(NSEC_TO_RT_JIFFIES(1000000000))

#define FORCE_SCHED		1			// force scheduling

#define	RT_TIMER_CPU_NONE	-1		// no timer cpu initialized
#define RT_TIMER_DISARMED	0xffffffff		// timer is disarmed

extern int rt_timer_cpu;
extern unsigned long rt_timer_res_nsec;
extern struct timespec rt_tomono;

DECLARE_PER_CPU(struct rt_list_hdr, isr_list_hdr);

struct posix_timer;


 /*
  * Internal interval timer structure optimized for the RT domain.
  */
struct itimer {
	struct list_head it_link;			// code assumes this to be the first element
	_INT64	activation_time;			// specified in jiffies for CLOCK_REALTIME timers
	unsigned long count;				// for CLOCK_SYNC timers (also used for a disarmed timer)
	_INT64 it_period;					// for CLOCK_REALTIME timers period (reload value)
	unsigned long it_cycle;				// for CLOCK_SYNC timers, cycle number when to fire
	_INT64 it_interval;					// CLOCK_SYNC needs for timer_gettime()
	unsigned flags;						// TIMER_ABSTIME for CLOCK_REALTIME timers, ONE_SHOT for CLOCK_SYNC timer
	int (*timer_action) (void *);
	void *action_par;					// address notification element or (for internal timers) thread to be notified
	struct rt_list_hdr *lhdr;			// address of rt_list_hdr, where to insert when a timer becomes active
	struct rt_list_hdr *chdr;			// address CLOCK_SYNC carrier header
};

/*
 * For synchronization with an external synchronization event  the nominal_sync_offset is compared
 * with the actual one and the beginning of the new period is set accordingly.
 */
struct rt_list_hdr {
	struct list_head it_link;			// an active header queues the timers, otherwise used for queueing empty headers
	struct list_head aux_link;			// isr_list_hdr queue here the abs timers, sync_clock_hdr its associated timers
	struct itimer itmr;					// proxy timer if rt_list_hdr is sublist header
	struct notify rt_notify;			// in case notification is done via sending a signal
										// the remaining part is for CLOCK_SYNC headers only
	clockid_t clockid;					// for finding the right header (the clock-..() calls specify the clockid
	struct	file* fp;					// for finding an (empty) CLOCK_SYNC header  register/unregister calls
	struct	semaphore sem;
	_INT64	activation_base;			// time base for the current cycle
	_INT64	new_act_base;				// in case new activation hits an active period
	_INT64	old_act_base;				// time base for current cycle when activation_base is set to zero
	_INT64	start_act_base;				// time when the clock starts running
	_INT64	last_wait;					// time of the last successful sigwait operation
	_INT64	period;						// length of a cycle
	int 	active;						// group is active or not (set by clock_settime())
	struct	itimer *itp;					// points to timer which fires next, NULL for inactive groups or other list headers
	void	(*clock_sync_isr)(void);	// to get a common parameterized ISR from parameterless clock specific ISRs
	void	(**isr_fp)(void);				// function pointer for clock_sync_isr
	struct	itimer *ptmr;				// in case the cycle gets started by a period timer, NULL if CLOCK_SYNC_HARD
	struct	rt_list_hdr *cs_hdr;			// address CLOCK_SYNC list header
	struct	list_head oneshot_carrier;	// queues the oneshot timers using a carrier thread
};

/*
 * POSIX interval timer structure optimized for the RT domain with a  notification component
 * also used for IO-events.
 */
struct posix_timer {
	struct list_head it_link;			// timer linked either in free_list, created_list
	struct list_head abs_link;
	_INT64 insert_offset;
	clockid_t it_clock;
	timer_t it_id;
	int it_state;						// DELETED is the only state
	struct itimer itmr;
	struct notify rt_notify;			// uniform notification component
};

struct posix_timer_pool {
	struct		list_head available_timer;
	struct		list_head created_timer;
	struct		list_head available_list_hdr;
	int			used_hdr;
	int			used_timer;
	int			max_timer;
	int			id_count;
	unsigned	*bitmap;
	struct posix_timer	**tmr_map;
};

DECLARE_PER_CPU(struct posix_timer_pool, rcl);
DECLARE_PER_CPU(struct posix_timer_pool *, prcl);

struct rtg_clock {
	int (*clock_set) (clockid_t, struct timespec *);
	int (*clock_get) (clockid_t , struct timespec *);
	int (*clock_getres) (clockid_t, struct timespec *);
	int (*nsleep)	 (clockid_t clockid, int flags, struct timespec * new_setting);
	int (*timer_set) (struct posix_timer *, int flags, struct itimerspec * new_setting, struct itimerspec * old_setting);
	int (*timer_del) (struct posix_timer * );
	void (*timer_get) (struct posix_timer *, struct itimerspec * cur_setting);
	int (*timer_create) (struct posix_timer *, sigevent_t *); // if additional setup such as list_hdr is required
};


/*
 * Converts the external POSIX (timespec) representation to an internal 64-Bit representation:
 * HRT: For high resolution timer the internal representation is in nsecs.
 * non-HRT: The internal representation is in entities of clock ticks.
 * Returns zero if okay or a negative value in case of a fault or an invalid time specification.
 */
static INLINE long convert_to_internal(_INT64 *itr, struct timespec *tpr)
{
	uint64_t timeval;

	if (((unsigned long)tpr->tv_nsec >= NSEC_PER_SEC) || (tpr->tv_sec < 0))
	   return -EINVAL;
#ifdef CONFIG_RTX_DOMAIN_HRT
	RTX_CONVERT_TO_INTERNAL(&timeval, tpr);
#else
	timeval =  (tpr->tv_sec * 1000000000ULL) + tpr->tv_nsec + rt_timer_res_nsec - 1;
	do_div(timeval, rt_timer_res_nsec);
#endif
	*itr = timeval;
	return(0);
}

/*
 * Converts the internal 64-Bit representation (nsecs or clock ticks) to the external
 * (POSIX) representation. In case  internal representation is not read atomically,
 * repeating takes care of it.
 */
static INLINE long convert_to_external(_INT64 *itr, struct timespec *tpr)
{
	uint64_t timeval = *itr;

#ifdef CONFIG_RTX_DOMAIN_HRT
	RTX_CONVERT_TO_EXTERNAL(timeval, tpr);
#else
	timeval *= rt_timer_res_nsec;
	tpr->tv_nsec = do_div(timeval, NSEC_PER_SEC);
	tpr->tv_sec = timeval;
#endif
	return(0);
}

DECLARE_PER_CPU(_INT64, act_clock);

extern struct aud_timer *aud_rt_timer;
extern struct itimer sys_timer;

extern struct timezone sys_tz;
extern struct k_clock posix_clocks[];

/* This routine checks all for timers ready to fire in the isr_list_hdr
 * and they get handled immediately.
 * It may be called either by the ISR or by a thread (interrupts disabled).
 * If a timer in a thread_list_hdr is ready to fire, then the corresponding
 * thread is woken up and scanning of the rt_list_headers continues at the ISR-level.
 * A thread_list_hdr is then handled by the thread woken up.
 */
#define CHECK_TIMER(force_sched) \
{ \
	struct list_head *tp; \
	struct itimer *itmr; \
	_INT64 time_rel; \
	int schedule = 0; \
	struct rt_list_hdr *pilh = &__raw_get_cpu_var(isr_list_hdr); \
\
	while ((tp = pilh->it_link.next) != &pilh->it_link) { \
		itmr = list_entry(tp, struct itimer, it_link); \
		update_act_clock(); \
		if ((time_rel = (itmr->activation_time - __raw_get_cpu_var(act_clock))) > (_INT64)aud_rt_timer->gravity) { \
			if (RTX_TIMER_SET((RTX_TIMER_TYPE)time_rel)) \
				break; \
		} \
		list_del_init(&itmr->it_link); \
		itmr->flags = 0; \
		if (itmr->it_period > 0LL) { \
			itmr->activation_time += itmr->it_period; \
			insert_timer(pilh, itmr); \
		} \
		itmr->timer_action(itmr->action_par); \
		if (force_sched) \
			schedule = 1; \
	} \
	if (schedule) \
		rt_schedule(0); \
}


static INLINE int insert_timer(struct rt_list_hdr *hdp, struct itimer *itmr)
{
	struct list_head *tp;
	struct itimer *itp;
	int first_entry = 1;

	for (tp = hdp->it_link.next; tp != &hdp->it_link; tp = tp->next)
	{
		itp = list_entry(tp, struct itimer, it_link);
		if (itmr->activation_time  < itp->activation_time)
			break;
		first_entry = 0;
	}
	list_add_tail(&itmr->it_link, tp);						// queue it in before later element
	return first_entry;
}

/*
 * Set the actual time in resolution entities.
 */
static INLINE  void update_act_clock(void)
{
#ifdef CONFIG_RTX_DOMAIN_HRT
	RTX_TIMER_TYPE time_act;

	time_act = RTX_TIMER_GET;
	__raw_get_cpu_var(act_clock) += (_INT64)(time_act - __raw_get_cpu_var(rt_time_last));
	__raw_get_cpu_var(rt_time_last) = time_act;
#endif
}

/*
 * Deallocate timerid in bit-list.
 */
#define FREE_TIMERID(timerid) \
{ \
	int no_of_bitword; \
    unsigned int bit_mask; \
\
	*(rtx_get_cpu_var(prcl)->tmr_map + (timerid & 0xffff)) = NULL; \
	no_of_bitword = (timerid & 0xffff) >> 5; \
	bit_mask = ~(0x1 << (timerid & 0x1f)); \
	*(rtx_get_cpu_var(prcl)->bitmap + no_of_bitword) &= bit_mask; \
}

#include <linux/aud/rt_link.h>
#endif /* _LINUX_AUD_RT_TIMER_H */

