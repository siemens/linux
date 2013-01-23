/*
 * linux/aud/rt_sched.h
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
 * Scheduler specific definitions.
 */
#ifndef _LINUX_AUD_RT_SCHED_H
#define _LINUX_AUD_RT_SCHED_H

#include <linux/version.h>

/* 
 * State values for rt_state	
 */
#define LX_TASK					0		/* must be zero gets initialized in fork.c for threads of regular processes */
#define RTLX_TASK				0x1		/* RT thread executing temporarily under Linux but still member of RT thread list */
#define RT_TASK_EXIT_PENDING	0x2		/* when a RT thread is forced to exit */
#define RT_TASK_MAIN_EXIT		0x4		/* mark exit of main task */
#define RT_TASK_RUNNING			0x8		/* RT thread is executing */
#define RT_TASK_UNINTERRUPTIBLE	0x10	/* RT thread is waiting */
#define RT_TASK_TIMER_CLEANUP	0x20	/* when a RT thread is forced to continue (debugger, ^C) while holding an internal timer */
#define LXRT_TASK				0x40	/* Linux thread of realtime process */
#define	LXRT_TASK_LEAVE_PENDING	0x80	/* will become RT_TASK_RUNNING with insertion into RT thread list */
#define RT_TASK_MAIN			0x100
#define LXRT_DAEMON				0x200	/* no thread group change while migrating */
//#define xxx				0x400 	/* can be reused */
#define RTLX_LOCK_MIGRATION		0x800 	/* locks the semaphore sync_mig_sem */
#define RT_TASK_FUTEX_CLEANUP	0x1000	/* when a RT thread is forced to continue (debugger, ^C) and is waiting on a futex */
#define RT_TASK_SEM_CLEANUP		0x2000	/* when a RT thread is forced to continue (debugger, ^C) and is waiting on a semaphore */

#define RT_TIMER_DAEMON_STATE		(LXRT_DAEMON | RTLX_TASK)	/* when migrating stays out of the rt thread list */
#define LX_DAEMON_STATE				LXRT_DAEMON					/* no migration possible/needed */

#define LX_MIGRATE_TO_RT			(LXRT_TASK_LEAVE_PENDING | RTLX_TASK) 	/* has to migrate after system call exit in LX */
#define IS_EXIT_PENDING(p)		(p->rt_state & RT_TASK_EXIT_PENDING)

/* 
 * State values for rt_state2
 * Note: When leaving a system call (LX or RT) rt_state2 is set to 0.	
 */
#define RT_LEAVE_PENDING_TASK		0x1		/* thread becomes LXRT_TASK */
#define RT_LEAVE_PENDING_PID_LIST	0x2		/* thread should be removed from rt_tl list */
#define RT_SYS_QUOTA_HANDLING		0x4		/* thread has locked rt_quota_sem */
#define RT_DEFER_QUOTA_HANDLING		0x8		/* quota handling for this thread is deferred */
#define RT_TASK_QUOTA_EXCEEDED		0x10	/* quota for this thread is exceeded */
#define RT_TASK_RETURNED_TO_LX		0x20	/* RT-thread voluntarily returned to LX */
#define RT_TASK_SHUTDOWN			0x40	/* shutdown a thread */
#define RT_TASK_IN_EXIT				0x80	/* exit handling already reached */
#define RT_TASK_SIGNAL_RECEIVED		0x100	/* RT-thread which gives up control to LX received a signal */
#define RT_TASK_DEADLOCK_DETECT		0x200	/* deadlock detection for PI-mutexes */

#define IS_TASK_LEAVE_PENDING(p)		(p->rt_state2 & RT_LEAVE_PENDING_TASK)
#define IS_PID_LIST_LEAVE_PENDING(p)	(p->rt_state2 & RT_LEAVE_PENDING_PID_LIST)

/* 
 * State values for rt_state3
 * Note: These flags must be reset explicitly, they are not touched implicitly.	
 */
#define RT_TASK_ALLOW_SETAFFINITY	0x1		/* allow CPU migration for this thread */
#define RT_TASK_REALTIME			0x2		/* CPU migration for a RT-capable thread */
#define RT_TASK_FAULT_DETECTED		0x4		/* migrate back to RT after exception handling in LX */
#define RT_TASK_ALLOW_SETSCHEDULER	0x8		/* allow sched_setscheduler() call for this thread */
#define RT_TASK_RT_DEBUG_PENDING	0x100 	/* force thread in RT to migrate to LX for debugging */
#define RT_TASK_LX_DEBUG_PENDING	0x200 	/* thread is on its way to LX for debugging */
#define RT_TASK_RQ_DEBUG_PENDING	0x400 	/* TBD */

#define SETAFFINITY_IS_ALLOWED(p)		(p->rt_state3 & RT_TASK_ALLOW_SETAFFINITY)
#define TASK_IS_RT_CAPABLE(p)			(p->rt_state3 & RT_TASK_REALTIME)
#define SETSCHEDULER_IS_ALLOWED(p)		(p->rt_state3 & RT_TASK_ALLOW_SETSCHEDULER)

#define CLEAR_AFFINITY_MASK(p)			(p->rt_state3 &= ~(RT_TASK_ALLOW_SETAFFINITY | RT_TASK_REALTIME))
#define CLEAR_SCHEDULER_MASK(p)			(p->rt_state3 &= ~RT_TASK_ALLOW_SETSCHEDULER)

#define RT_EXIT_BY_SIGNAL			1		/* shutdown of the RT process caused by a signal */

/*
 * state values for rt_notify_state
 */
#define RT_EVENT_REGISTERED          0x01  /* states for things done in the RT domain */
#define RT_SYNC_TIMER_REGISTERED     0x02
#define RT_TIMER_REGISTERED          0x04

#ifndef __ASSEMBLY__

#define IS_REALTIME_PROCESS(p)  		(p->rt_state != LX_TASK)
#define IS_REALTIME_PRIO_NORMALIZED(prio)    	(prio < MAX_NO_OF_RT_PRIO) 
#define IS_REALTIME_PRIO(prio)    		(prio >= RT_PMIN && prio <= RT_PIR) 
#define MAX_NO_OF_RT_PRIO  			32

#define __set_current_rt_state(state_value)			\
	do { current->rt_state = (state_value); } while (0)

#define IS_LINUX		(!(current->rt_state & (RT_TASK_RUNNING | RT_TASK_UNINTERRUPTIBLE)))
#define IS_REALTIME  	(current->rt_state & (RT_TASK_RUNNING | RT_TASK_UNINTERRUPTIBLE))
#define THREAD_IS_LINUX(p) (!(p->rt_state & (RT_TASK_RUNNING | RT_TASK_UNINTERRUPTIBLE)))
#define THREAD_IS_REALTIME(p) (p->rt_state & (RT_TASK_RUNNING | RT_TASK_UNINTERRUPTIBLE))

#define CANCEL_MIGRATION	((struct task_struct *)0xFFFFFFFF)

#ifdef CONFIG_RTX_THREAD_RUNTIME
#define THREAD_RUNTIME_PRE_SWITCH() \
	current->rt_thread_runtime += (uint64_t)(RTX_TIMER_GET - current->rt_thread_switched_in); \
	current->se.sum_exec_runtime = current->rt_thread_runtime;

#define THREAD_RUNTIME_POST_SWITCH() \
	current->rt_thread_switched_in = RTX_TIMER_GET;
#else
#define THREAD_RUNTIME_PRE_SWITCH()
#define THREAD_RUNTIME_POST_SWITCH()
#endif

int rtx_migrate_to_rt(int);
void rtx_migrate_to_lx(int, struct task_struct *);

int rtx_is_rt_capable(int, int);
int rtx_sched_change(struct task_struct *, int, int);
void rtx_check_migration_source(void);
int rtx_thread_init_common(struct task_struct *);
int rtx_set_rt_state(struct task_struct *);
int rtx_exit_handling(struct task_struct *);
int rtx_shutdown_rt_process(int);
struct task_struct * rtx_handle_signal_delivery(struct task_struct *, int);
void rtx_delegate_sig_to_cpu(struct task_struct *, int);

int rtx_futex_wake_from_lx(unsigned long, int, int);
int rtx_waiter_pi_futex(u32 __user *, int);

void rtx_synchronize_time(void *);
void rt_clock_init(void);
void rtx_mark_debug_pending(struct task_struct *);
int rtx_is_rt_kernel_thread(struct task_struct *, int);
void rtx_lx_clock_was_set(void);
void rtx_setup_affinity(int);
void rt_try_next_thread(struct task_struct *);
#endif

#endif /* _LINUX_AUD_RT_SCHED_H */
