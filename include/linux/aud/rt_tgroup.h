/*
 * linux/aud/rt_tgroup.h
 *
 * Copyright (C) 2011 Siemens AG
 * Contributed by Wolfgang Hartmann <wolfgang.hartmann@siemens.com>, 2011.
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
 * This file contains the definitions for RT thread group quota handling.
 *
 */
#ifndef _LINUX_AUD_RT_TGROUP_H
#define _LINUX_AUD_RT_TGROUP_H

#include <linux/aud/rt_internal.h>
#include <asm/rtx_timer.h>


DECLARE_PER_CPU(struct itimer, quota_itmr);
DECLARE_PER_CPU(struct list_head *, prt_blocked_queue);

/* This is a kernel config option. */
#define RT_TGROUP_MAX				CONFIG_RTX_MAX_THREAD_GROUPS		// maximum number of thread groups supported

#define RT_TGROUP_QUOTA_OFF			100		// no quota limit

#define RT_TGROUP_BASE_ID			0x8000	// add the index of the group array

#define RT_TGROUP_TO_INDEX(group)	(group & ~RT_TGROUP_BASE_ID)
#define RT_TGROUP_IS_VALID(group)	((group & RT_TGROUP_BASE_ID) && (RT_TGROUP_TO_INDEX(group) < RT_TGROUP_MAX))

#define RT_QUOTA_IS_EXCEEDED(p)		(p->rt_state2 & RT_TASK_QUOTA_EXCEEDED)
#define RT_QUOTA_IS_ENABLED(ptg)	(ptg && ptg->quota && (ptg->quota < RT_TGROUP_QUOTA_OFF))
#define RT_QUOTA_IS_ON(p)			(p->rt_thread_group && p->rt_thread_group->quota != RT_TGROUP_QUOTA_OFF)

#define RT_QUOTA_MIN_NS				10	// minimal quota value in nsec

/* Flags for shifting threads from blocked queue to run queue. */
#define RT_QUOTA_KEEP_ZERO_BLOCKED	0	// don't touch threads which are blocked by a zero quota
#define RT_QUOTA_FORCE_ALL			1	// all threads are shifted to the run queue

/* Flags for set_quota_timer(). */
#define RT_QUOTA_GROUP_TIMER		0
#define RT_QUOTA_INTERVAL_TIMER		1

/* Note: The time values (*_ns) are stated in nano-seconds */
typedef struct rt_tgroup {
	tgroup_t id;					// thread group id
	int aspect;						// aspect of the thread group
	int quota;						// processing time quota in per cent (0 = exceeded, 100 = off)
	int quota_peak;					// processing time maximum quota in per cent
	long quota_ns;					// processing time quota
	long quota_peak_ns;
	long quota_rem_ns;				// remaining processing time quota
	long quota_credit_ns;			// max credit that can be used for one interval
	long quota_credit_acc_ns;		// totally accumulated credit
	struct list_head thread_list;	// list of all threads in the thread group
	RTX_TIMER_TYPE quota_start_ns;	// time a thread starts running
} rt_tgroup_t;

typedef struct rt_tgroup_head {
	int quota_enabled;					// quota handling is on (1) or off (0)
	unsigned long quota_group_active;	// interval timer active (0), group timer active (>0)
	unsigned long quota_interval_ns;
	unsigned long quota_interval_rem_ns;
	int quota_threads;					// total number of threads in thread groups
	RTX_TIMER_TYPE quota_start_ns;		// quota interval start
} rt_tgroup_head_t;

extern int rt_shift_single_thread_to_runq(struct list_head *, struct task_struct *);
extern int rt_shift_all_threads_to_runq(struct list_head *, rt_tgroup_t *, int);
extern int rt_shift_all_threads_to_blockedq(struct list_head *, struct task_struct *);
extern void rt_check_quota_thread_from_lx(void);
extern void rt_dequeue_get_next_thread(void);
extern void quota_interval_stop(rt_tgroup_head_t *);
extern void quota_pre_switch(rt_tgroup_head_t *, struct task_struct *);
extern void quota_post_switch(rt_tgroup_head_t *);
extern void quota_post_switch_from_lx(rt_tgroup_head_t *);
extern void rt_reset_exceeded_state(rt_tgroup_t *, int);
extern int rt_remove_thread_from_group(struct task_struct *);


DECLARE_PER_CPU(rt_tgroup_head_t *, pquota_head);

#endif /* _LINUX_AUD_RT_TGROUP_H */
