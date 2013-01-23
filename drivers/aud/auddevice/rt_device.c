/*
 * drivers/aud/auddevice/rt_device.c
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
 * AuD-Device-Driver for Linux
 * This file provides for the ioctl-based user API to manage the realtime extensions
 */
#include <linux/aud/rt_timer.h>
#include <asm/siginfo.h>
#include <linux/unistd.h>
#include <linux/device.h>

#ifdef CONFIG_RTX_THREAD_GROUP_SUPPORT
#include <linux/aud/rt_tgroup.h>
#endif

#ifdef CONFIG_RTX_EXTENDED_RT_DEBUGGING_SUPPORT

#include <linux/aud/rt_debug.h>

int initDebugControl(void);

#endif

static int init_AuD_dev(void);
static void exit_AuD_dev(void);

MODULE_AUTHOR("Karl-Heinz Krause");
MODULE_DESCRIPTION (AUDIS_DRIVER_NAME);
MODULE_LICENSE("GPL");	     /* "Proprietary" returns warning during loading */

unsigned dev_version = RT_DEV_VERSION;

#ifdef CONFIG_SMP
int rt_timer_cpu = RT_TIMER_CPU_NONE;
#endif

unsigned long aud_diagnostic = RT_FAULT_VERBOSE_BIT
// 	| RT_SHUTDOWN_VERBOSE_BIT
//	| RT_START_VERBOSE_BIT
//	| RT_INIT_VERBOSE_BIT
//	| RT_EVENT_VERBOSE_BIT
//	| RT_MIGRATE_VERBOSE_BIT
//	| RT_CHECK_VERBOSE_BIT
//	| AUD_DEV_VERBOSE_BIT
//	| RT_SYSCALL_IN_LX_VERBOSE_BIT
//	| RT_SYSCALL_NOT_IMPL_VERBOSE_BIT
//  | RT_THREAD_VERBOSE_BIT
//  | RT_TIMER_VERBOSE_BIT
//  | RT_SIGNAL_VERBOSE_BIT
//  | RT_FUTEX_VERBOSE_BIT
  | RT_EXCEPTION_VERBOSE_BIT
//  | RT_FP_EXCEPTION_VERBOSE_BIT
//  | RT_DEBUG_VERBOSE_BIT
//  | RT_UNALIGNMENT_VERBOSE_BIT
  | RT_WARN_VERBOSE_BIT
//  | RT_ALL_EXCEPTION_BITS			// enables all exceptions
//	| RT_PRINT_STDOUT_BIT
// 	| RT_PRINT_STDERR_BIT
	;

DEFINE_PER_CPU(unsigned long, rt_system_state);
DEFINE_PER_CPU(int, sig_to_deliver);
DEFINE_PER_CPU(int, cpu_received_sig);
DEFINE_PER_CPU(int, rtx_mode) = {LX_DOMAIN_NATIVE};
DEFINE_PER_CPU(long, mn_rt_state);
DEFINE_PER_CPU(struct rt_event *, aud_evpr);
DEFINE_PER_CPU(uint64_t, isr_time);
DEFINE_PER_CPU(struct list_head, rt_tl);

cpumask_t rtx_cpu_onlinemask;	// currently active CPUs with a running RT process (hard or soft)
int rtx_mode_rt;		// is set when at least one RT process is active with mode RT_DOMAIN

#ifdef CONFIG_SMP
struct smp_delegate_sig {
	struct task_struct *p;
	int sig;
};
#endif

struct rt_proc_hdr rt_proc[RT_MAX_CPUS];
static struct class *class_audis;
static struct device *dev_audis;
static int major_audis;	            		// major number of the device
DEFINE_PER_CPU(int, rt_dev_open_cnt);

static long AuD_dev_ioctl (struct file *, unsigned int, unsigned long);
static int AuD_dev_open (struct inode *, struct file *);
static int AuD_dev_release (struct inode *, struct file *);

/*
 * The event stuff is only for using the Audis driver as a test
 * vehicle for the event functionality without having real hardware.
 */
#define NO_OF_EVENTS	4
#define AUD_DEV_EV_1	0x40000003
#define AUD_DEV_EV_2	0x40000001
#define AUD_DEV_EV_3	0x40000002
#define AUD_DEV_EV_4	0x40000007

static DEFINE_PER_CPU(struct rt_event, aud_dev_ev_arr[NO_OF_EVENTS]) = {
						{.ev_id = AUD_DEV_EV_1},
						{.ev_id = AUD_DEV_EV_2},
						{.ev_id = AUD_DEV_EV_3},
						{.ev_id = AUD_DEV_EV_4},
			     };
DEFINE_PER_CPU(int, aud_dev_ev_area_id);

/*
 * This structure holds all the setup parameters for the realtime system.
 * It gets initialized with default values, but may be overriden later via
 * an ioctl(.., AuD_DEV_RT_PARAM, ..)
 * It must be in line with the structure rt_config in rtime_lib.h  which holds the
 * requested values only.
 */
struct rt_config_cntrl aud_config = {
	.no_of_timers.max = 160,
	.no_of_timers.val = 120,
	.no_of_timers.min = 5,
	.no_of_sigq.max = 128,
	.no_of_sigq.val = 32,
	.no_of_sigq.min = 8,
	.no_of_headers.max = 16,
	.no_of_headers.val = 4,
	.no_of_headers.min = 1,
	.no_of_fu_req.max = 100,
	.no_of_fu_req.val = 40,
	.no_of_fu_req.min = 10,
	.buf_size.max = 16384,
	.buf_size.val = 4096,
	.buf_size.min = 2048,

	.td_prio.max = RT_P31,
	.td_prio.val = RT_PRIORITY_MIN,		// timer daemon: priority relevant for realtime
	.td_prio.min = RT_PRIORITY_MIN,
	.wd_prio.max = SRT_PMAX,
	.wd_prio.val = SRT_PMAX,		// write daemon: priority relevant for Linux
	.wd_prio.min = 1,
};


/*
 * NOTE:
 * read, write, poll, fsync, readv, writev, unlocked_ioctl and compat_ioctl
 * can be called without the big kernel lock held in all filesystems.
 */
static struct file_operations AuD_dev_fops = {
	.unlocked_ioctl = 	AuD_dev_ioctl,
	.open = 		AuD_dev_open,
	.release = 	AuD_dev_release,
};

struct dom_hdr aud_rt_hdr;

struct aud_timer *aud_rt_timer;

#ifndef CONFIG_RTX_DISABLE_WRITE_DAEMON
static struct type_info type_arr[] = {  {1,LX_WD_MSG, 5},
										{1,RT_WD_MSG, 5},
										{2,LX_WD_MSG, 5},
										{2,RT_WD_MSG, 5},
};
#endif

/*
 * Clear the corresponding rt_proc structure.
 */
static void rt_unregister_process(struct rt_proc_hdr *pproc)
{
	memset(pproc, 0, sizeof(*pproc));
}

int check_par(int new_val, confpar_t *op)
{
	if (new_val == 0)
		return 0; 					// keep current value
	if ((new_val > op->max) || (new_val < op->min))
		return -EINVAL;
	op->val = new_val;				// set to new value
	return 0;
}


int parameterize_configuration(struct rt_config *ncfg)
{

	if (check_par(ncfg->no_of_timers, &aud_config.no_of_timers))
		return -EINVAL;
	if (check_par(ncfg->no_of_sigq, &aud_config.no_of_sigq))
		return -EINVAL;
	if (check_par(ncfg->no_of_headers, &aud_config.no_of_headers))
		return -EINVAL;
	if (check_par(ncfg->no_of_fu_req, &aud_config.no_of_fu_req))
		return -EINVAL;
	if (check_par(ncfg->buf_size, &aud_config.buf_size))
		return -EINVAL;
	if (check_par(ncfg->td_prio, &aud_config.td_prio))
		return -EINVAL;
	if (check_par(ncfg->wd_prio, &aud_config.wd_prio))
		return -EINVAL;
	return 0;
}

/*
 * Called from hook in sched.c (__sched_setscheduler)
 * and from rtx_sched_change().
 */
int rtx_is_rt_capable(int policy, int prio)
{
	if (policy != SCHED_FIFO && policy != SCHED_RR)
		return 0;

	if (IS_REALTIME_PRIO(prio) && rtx_get_cpu_var(rtx_mode) == RT_DOMAIN)
		return 1;
	return 0;
}

/*
 * Initialize thread specific data structures in the
 * the context of a fork().
 * Note: The type of a LX-thread may change to NRT (see
 * register_process()). Whenever a NRT-thread migrates
 * to RT by changing its priority we must not initialize
 * the thread group specific data structures again.
 */
int rtx_thread_init_common(struct task_struct *p)
{
	p->rt_state2 = 0;
	p->rt_state3 = 0;
#ifdef CONFIG_RTX_THREAD_GROUP_SUPPORT
	local_irq_disable_hw();
	p->rt_thread_group = NULL;
	INIT_LIST_HEAD(&p->rt_thread_list);
	INIT_LIST_HEAD(&p->rt_blocked_list);
	local_irq_enable_hw();
#endif
	return 0;
}

/*
 * Called from hook in fork.c or indirectly from hook in rt_sched.c.
 * Whenever a new thread gets created its rt_state must be initialized.
 */
int rtx_set_rt_state(struct task_struct *p)
{
	unsigned long flags;

	/* Note: Don't use rtx_mode instead of rt_timer_daemon. */
	if (IS_REALTIME_PRIO_NORMALIZED(p->prio) && rtx_get_cpu_var(rt_timer_daemon) != NULL) {
		if (p->rt_state & (RT_TASK_UNINTERRUPTIBLE | RT_TASK_TIMER_CLEANUP | RT_TASK_FUTEX_CLEANUP
				| RT_TASK_SEM_CLEANUP | RT_TASK_EXIT_PENDING) || IS_TASK_LEAVE_PENDING(p)) {
			/*
			 * If this thread is a migration source thread and is
			 * on its way back to RT, then we have to consider two cases:
			 * - The target thread p is already blocked on wait_mig_sem.
			 *   The source thread must synchronize with the target thread.
			 * - The target thread is blocked somewhere else in the RT-domain.
			 *   The source thread must not synchronize.
			 */
			if (current == rtx_get_cpu_member(rt_sync_migration, source) &&
							p != rtx_get_cpu_member(rt_sync_migration, target)) {
				rtx_set_cpu_member(rt_sync_migration, target, CANCEL_MIGRATION);
			}
			return 1;
		}

		/* Identify the migration target. */
		if (current == rtx_get_cpu_member(rt_sync_migration, source)) {
			// May already be set by by the source thread
			// calling rtx_migrate_to_lx().
			rtx_set_cpu_member(rt_sync_migration, target, p);
			/*
			* Make sure we do not leak PI boosting priority to the child.
			* Note: In case of a dynamic priority change (rt_dyn_prio < 0)
			* the child should adopt it.
			*/
			if (p->rt_dyn_prio > 0)
				p->rt_dyn_prio = 0;
		}

		if (!(p->rt_state & LXRT_TASK_LEAVE_PENDING)) {
			/* Initial migration of user threads. */
			p->rt_state = LXRT_TASK_LEAVE_PENDING;
			p->rt_carrier_thread_hdr = NULL;
			p->rt_mevpr = NULL;
			p->rt_notify_state = 0;
			p->fpu_counter = 0;
			INIT_LIST_HEAD(&p->rt_sigqueue_head);
			INIT_LIST_HEAD(&p->rt_mig_list);
			INIT_LIST_HEAD(&p->rt_pi_waiters);
			local_irq_save_hw(flags);
			list_add_tail(&p->rt_pid_list, &__raw_get_cpu_var(rt_tl));
			local_irq_restore_hw(flags);
		}
		return 1;
	}
	else {
		if (p->rt_state != LXRT_DAEMON)
			p->rt_state = LXRT_TASK;
		rtx_check_migration_source();
	}
	return 0;
}

/*
 * Shutting down the realtime domain is a subset of shutting down the realtime process.
 * In case of failure setting up the realtime domain may be incomplete. Therefore daemons are shutdown as they exist.
 * Shutdown of the realtime system starts always from the Linux domain. An exit() call in the realtime domain
 * is propagated to Linux. An exit() call in the Linux domain is intercepted to call this rtx_shutdown_rt_process()
 * function first. We have to make sure that
 * - no further RT activity takes place by marking all RT threads as RT_TASK_EXIT_PENDING
 * - no new migration takes place by setting the state to RT_SHUTDOWN
 *
 * From here the timer daemon is woken up, which causes all RT threads to migrate to Linux.
 * After all realtime threads are migrated, the daemons have shut down and the LX do_group_exit() processing continues.
 */
int rtx_shutdown_rt_process(int flag)
{
	struct rt_proc_hdr *pproc = current->rt_proc;
	struct task_struct *p = NULL;
	struct task_struct *ptd;
	struct siginfo info;
	struct pid *pgrp;
	int sig;

	if (current->rt_state == RT_TASK_MAIN_EXIT)
		return 0;

	down(&aud_rt_hdr.dev_sem);

	if (rtx_get_cpu_var(rt_system_state) & RT_SHUTDOWN_PROCESS) {
		up(&aud_rt_hdr.dev_sem);
		return 0;
	}
	/*
	 * If the current thread doesn't disappear (detach from RT-domain),
	 * we have to decrement the thread count, otherwise the slot remains
	 * allocated and there is no way to attach this process again.
	 */
	if (flag != RT_EXIT_BY_SIGNAL && rtx_get_cpu_var(rt_system_state) & RT_ATTACH_PROCESS) {
		atomic_dec(&pproc->thread_cnt);
		rtx_or_cpu_var(rt_system_state, RT_DETACH_PROCESS);
	}

	rtx_or_cpu_var(rt_system_state, (RT_SHUTDOWN | RT_SHUTDOWN_PROCESS));
	up(&aud_rt_hdr.dev_sem);

	if (RT_SHUTDOWN_VERBOSE)
	    printkGen(NULL, "starting shutdown realtime process rt_system_state=%#x\n", rtx_get_cpu_var(rt_system_state));

	if (rtx_get_cpu_var(rtx_mode) == RT_DOMAIN)
	{
		down(&__raw_get_cpu_var(to_rt_op));			// current migration finishes, no new migration possible
		__raw_get_cpu_var(rr_timer.it_period) = 0LL;	// deactivate RR-timer (if enabled)
		rt_mark_exit_pending(NULL);				// RT_SHUTDOWN gets set, no further RT activity will take place
	}
	if (rtx_get_cpu_var(interdomain_daemon))
		wake_up_process(rtx_get_cpu_var(interdomain_daemon));

	if (rtx_get_cpu_var(rt_system_state) & RT_TIMER_INITIALIZED) {
		rtx_and_cpu_var(rt_system_state, ~RT_TIMER_INITIALIZED);
		aud_rt_timer->ops.rt_unregister();			// disable timer interrupt
		rt_free_irq(aud_rt_timer->irq, aud_rt_timer->dev_id);
		rt_unregister_sync_clock(NULL, 0);				// all SYNC_CLOCK clocks get unregistered
		KFREE((char*)rtx_get_cpu_var(prcl)->bitmap);
	}

#ifdef CONFIG_SMP
	rtx_unpin_irq(-1);
#endif
	if ((ptd = rtx_get_cpu_var(rt_timer_daemon)) != NULL) {
		ptd->prio = 0;				// set highest priority
		lx_wake_up_rt_thread(ptd); 		// all threads will leave RT domain
	}

	if (pproc->write_daemon)
		wake_up_process(pproc->write_daemon);

	// Sends a delivered signal again to the threads of the process group,
	// shutting down the threads properly.
	if ((sig = rtx_get_cpu_var(sig_to_deliver))) {
		pgrp = task_pgrp(current);
		do_each_pid_task(pgrp, PIDTYPE_PGID, p) {
			if (sigismember(&p->signal->shared_pending.signal, sig)) {
				sigdelset(&p->signal->shared_pending.signal, sig);
				group_send_sig_info(sig, &info, p);
			}
		} while_each_pid_task(pgrp, PIDTYPE_PGID, p);
		rtx_set_cpu_var(sig_to_deliver, 0);
	}
	return 0;
}


/*
 * Wake up the main thread in RT to force a shutdown of the RT-process.
 * This function may be called synchronously or in the context of an IPI.
 * Notes:
 * - Assumption is that we are running on the right RT-CPU.
 * - Do _not_ use printk() in this function.
 */
void wake_up_main_in_rt(struct task_struct *p_main, int sig)
{
#ifdef CONFIG_RTX_THREAD_GROUP_SUPPORT
	rt_tgroup_head_t *pqh = rtx_get_cpu_var(pquota_head);
#endif
	local_irq_disable_hw();
	rtx_set_cpu_var(sig_to_deliver, sig);
	rtx_set_cpu_var(mn_rt_state, p_main->rt_state);
	p_main->rt_state  |= (RT_TASK_EXIT_PENDING | RT_TASK_MAIN);
#ifdef CONFIG_RTX_THREAD_GROUP_SUPPORT
	/* Deactivate quota handling and move all the threads
	 * from blocked list to run queue. */
	if (pqh->quota_enabled) {
		quota_interval_stop(pqh);
		rt_shift_all_threads_to_runq(rtx_get_cpu_var(prt_blocked_queue), NULL, RT_QUOTA_FORCE_ALL);
	}
#endif
#ifdef CONFIG_RTX_RETURN_TO_LINUX
	/* Note: A RT-thread which returned to LX is still
	 * in the rt_run_list. So rt_req_task is _not_ set
	 * when waking up the RT-thread. */
	if (p_main->rt_state2 & RT_TASK_RETURNED_TO_LX)
		rt_try_next_thread(p_main);
#endif
	lx_wake_up_rt_thread(p_main);
	local_irq_enable_hw();
}

#ifdef CONFIG_SMP
/*
 * Receive function call IPI from a remote CPU.
 */
static void smp_rtx_delegate_sig_to_cpu(void *cmd)
{
	struct smp_delegate_sig *info = (struct smp_delegate_sig *)cmd;

	wake_up_main_in_rt(info->p, info->sig);
}

/*
 * The rtx_handle_signal_delivery() function is _not_ invoked on the CPU the corresponding RT-process
 * is assigned to. So we have to delegate this function call to the right CPU.
 */
void rtx_delegate_sig_to_cpu(struct task_struct *p_main, int sig)
{
	struct smp_delegate_sig info;

	info.p = p_main;
	info.sig = sig;
	smp_call_function_single(p_main->rt_proc->cpu, smp_rtx_delegate_sig_to_cpu, (void *)&info, 1);
}
#endif

/*
 * Hook for catching signals, e.g. SIGKILL, SIGINT. For user threads running in the RT domain the regular path doesn't work.
 * In this case we force main to migrate to the LX domain and to execute a rtx_shutdown_rt_process resp. do_group_exit().
 * This function is called by a kernel hook in complete_signal(). It may be running on an arbitrary CPU.
 * So we have to consider that wake_up_main_in_rt() must be called on the right RT-CPU (setting rt_state SMP-safe).
 */
struct task_struct *rtx_handle_signal_delivery(struct task_struct *p, int sig)
{
	int cpu;
	struct rt_proc_hdr *pproc = p->rt_proc;

	// If the thread which is selected to handle the signal
	// is already running in LX, we delegate the handling to LX.
	if (p->rt_state & (LXRT_TASK | RTLX_TASK)) {
		if (p == pproc->main)
			pproc->main->rt_state = LXRT_TASK;		// prevent main from migrating back
		return p;
	}

	// We only handle termination signals.
	if (sig != SIGINT && sig != SIGKILL && sig != SIGTERM && sig != SIGQUIT && sig != SIGHUP)
		return p;

	// Remember the cpu this function is invoked on.
	cpu = rtx_processor_id();
	per_cpu(cpu_received_sig, pproc->cpu) = cpu;

	// The main() thread is in the RT domain (executing/waiting).
	if (pproc->main && THREAD_IS_REALTIME(pproc->main))
	{
#ifdef CONFIG_SMP
		if (cpu == pproc->cpu)
			wake_up_main_in_rt(pproc->main, sig);
		else
			current->rt_main = pproc->main;		// delegate call to the right CPU
#else
		wake_up_main_in_rt(pproc->main, sig);
#endif
		return NULL;
	}
	return p;
}

/*
 * Called by the hook rtx_exit_handling in do_exit of LX.
 * The hook gets called when the Audis driver is installed. No matter of realtime,
 * since the check for event notifications needs to be done for all processes.
 */
int rtx_exit_handling(struct task_struct *p)
{
	unsigned int *bwp;

	rt_delete_event_notification(p);
	if (!(IS_REALTIME_PROCESS(p)))
		return 0;

	if (RT_SHUTDOWN_VERBOSE)
		printkGen(NULL, "thread exit signal=%#x usage=%d rt_state=%#lx name=%s\n", current->exit_signal, current->usage, current->rt_state, current->comm);

	if (current->rt_state == RT_TASK_MAIN_EXIT)
		return 0;

	if ((IS_REALTIME_PRIO_NORMALIZED(p->prio)) && (rtx_get_cpu_var(rtx_mode) != RT_DOMAIN))
		clock_sync_delete_timer_notification(p);

	// Check if the exiting thread has to be removed from the migration queue.
	rt_dequeue_migration_thread(&__raw_get_cpu_var(rt_migqueue), p);

#ifdef CONFIG_RTX_THREAD_GROUP_SUPPORT
	/* If the thread is still in a thread group, then remove it. */
	rt_check_quota_thread_from_lx();
#endif
	down(&aud_rt_hdr.dev_sem);

	/* If this is the last thread of the RT-process,
	 * then deallocate its slot in rt_proc. */
	if (atomic_dec_and_test(&p->rt_proc->thread_cnt)) {
		if (p->rt_proc->main) {
			p->rt_proc->main->rt_state = RT_TASK_MAIN_EXIT;
			p->rt_proc->main = NULL;
		}
		cpu_clear(p->rt_proc->cpu, rtx_cpu_onlinemask);
		p->rt_proc->slot_state = 0;
#ifdef CONFIG_SMP
		/* Check if we have to move/delete the timer CPU. */
		if (rt_timer_cpu == rtx_processor_id()) {
			if (cpus_empty(rtx_cpu_onlinemask))
				rt_timer_cpu = RT_TIMER_CPU_NONE;
			else
				rt_timer_cpu = first_cpu(rtx_cpu_onlinemask);
		}
#endif

		/* Check rt_dev_open_cnt. */
		if (rtx_get_cpu_var(rt_dev_open_cnt)) {
			if (rtx_get_cpu_var(rt_system_state) & RT_DETACH_PROCESS) {
				if (rtx_get_cpu_var(rt_dev_open_cnt) != 1)
					printkGen(KERN_ALERT,"WARNING: wrong rt_dev_open_cnt=%d (detach process)\n", rtx_get_cpu_var(rt_dev_open_cnt));
			}
			else
				printkGen(KERN_ALERT,"WARNING: wrong rt_dev_open_cnt=%d\n", rtx_get_cpu_var(rt_dev_open_cnt));
		}

		rtx_set_cpu_var(rtx_mode, LX_DOMAIN_NATIVE);

		/* Check used timer and timer header. */
		if (rtx_get_cpu_var(prcl)->used_timer || rtx_get_cpu_var(prcl)->used_hdr)
			printkGen(KERN_ALERT, "WARNING: timer requests still allocated (used_timer=%d used_hdr=%d)\n", rtx_get_cpu_var(prcl)->used_timer, rtx_get_cpu_var(prcl)->used_hdr);

		/* Check timer bitmap. */
		bwp = rtx_get_cpu_var(prcl)->bitmap;
		if (*bwp || *(bwp+1) || *(bwp+2) & 0xffff)
			printkGen(KERN_ALERT,"WARNING: timer bitmap still allocated (bw=%#x, bw1=%#x bw2=%#x)\n", *bwp, *(bwp+1), *(bwp+2) & 0xffff);

		if (RT_SHUTDOWN_VERBOSE)
			printkGen(NULL, "shutdown realtime process completed\n");

		/* The last thread at all disappears. */
		if (cpus_empty(rtx_cpu_onlinemask)) {
			struct list_head *lp;
			int cnt = 0;

			init_timer_done = 0;
			rtx_mode_rt = LX_DOMAIN_NATIVE;

#ifdef CONFIG_RTX_THREAD_GROUP_SUPPORT
			if (rtx_get_cpu_var(pquota_head))
				if (rtx_get_cpu_var(pquota_head)->quota_threads != 0)
					printkGen(KERN_ALERT,"WARNING: quota threads still allocated (quota_threads=%d)\n", rtx_get_cpu_var(pquota_head)->quota_threads);
#endif

			/* Check futex requests. */
			for (lp = aud_rt_hdr.free_list.next; lp != &aud_rt_hdr.free_list; lp = lp->next) {
				cnt++;
			}
			if (cnt != NO_OF_FU_REQUESTS)
				printkGen(KERN_ALERT,"WARNING: futex requests still allocated (freecnt=%d expected=%d)\n", cnt, NO_OF_FU_REQUESTS);
		}
	}
	up(&aud_rt_hdr.dev_sem);
	return 0;
}

/*
 * Find the appropriate rt_proc array slot for the
 * current thread.
 * This function may be called from LX or from RT.
 * Note: Assumption is that dev_sem is locked.
 */
struct rt_proc_hdr *get_rt_proc(int daemon)
{
	struct rt_proc_hdr *phdr = NULL;
	int islot;

	for (islot = 0; islot < RT_MAX_CPUS; islot++) {
		/* Check for a valid process registration. */
		if (rt_proc[islot].slot_state) {
			if (rt_proc[islot].curr_tgid != current->tgid)
				continue;
			phdr = &rt_proc[islot];
			break;
		}
	}
	if (phdr == NULL)
		return phdr;

	switch (daemon) {
	case WRITE_DAEMON_ID:
		phdr->write_daemon = current;
		break;
	case CHECK_DAEMON_ID:
		phdr->rt_check_daemon = current;
		break;
	}
	return phdr;
}

/*
 * Find the appropriate rt_proc array slot for the
 * current thread.
 * Check whether the specified daemon is running.
 * Note: It is assumed that dev_sem is already locked.
 */
int rt_is_running(int daemon)
{
	struct rt_proc_hdr *phdr = NULL;
	int islot;

	for (islot = 0; islot < RT_MAX_CPUS; islot++) {
		/* Check for a valid process registration. */
		if (rt_proc[islot].slot_state) {
			if (rt_proc[islot].curr_tgid != current->tgid)
				continue;
			phdr = &rt_proc[islot];
			break;
		}
	}
	if (phdr == NULL)
		return 0;

	switch (daemon) {
	case WRITE_DAEMON_ID:
		return phdr->write_daemon ? 1 : 0;
	case CHECK_DAEMON_ID:
		return phdr->rt_check_daemon ? 1 : 0;	// currently unused
	}
	return 0;
}

#ifdef CONFIG_SMP
/*
* Pin down the current thread to a CPU.
*/
void rt_pin_rt_process(int cpu)
{
	int new_cpu, old_cpu = rtx_processor_id();
	cpumask_t rt_cpumask;

	if (num_online_cpus() > 1)
	{
		cpus_clear(rt_cpumask);
		cpu_set(cpu, rt_cpumask);

		// Pinning the current thread to the given cpu.
		if (set_cpus_allowed(current, rt_cpumask) != 0)
		    printkGen(KERN_ALERT, "Cannot pin RT-process to cpu=%d\n", cpu);

		if (RT_INIT_VERBOSE)
			if ((new_cpu = rtx_processor_id()) != old_cpu)
		    	printkGen(NULL, "RT-process migrated from cpu=%d to cpu=%d\n", old_cpu, new_cpu);
	}
}
#endif

/*
 * Register a LX-process
 * - as a RT-process (RT_HARD/RT_SOFT) or
 * - for memory residency only
 */
static int register_process(rt_exec_t __user *arg)
{
	int error;
	struct exec_rt_kernel cmd_par;
	struct reg_proc *reg;
	struct list_head *curr, *head;
	int icpu, islot;

	if (!arg)
		return -EINVAL;
	if (copy_from_user(&cmd_par, (char*)arg, sizeof(struct exec_rt_kernel)))
		return (-EFAULT);
	error = -EINVAL;

	// We don't support tryhard (RT_HARD | RT_SOFT) any longer.
	if ((cmd_par.exec_flags & ~RT_EXEC_MASK) != 0
			|| (cmd_par.exec_flags == RT_EXEC_MASK))
		return error;

	down(&aud_rt_hdr.dev_sem);

#ifdef CONFIG_RTX_DOMAIN_EXCLUSIVE_MODE
	/* The exclusive mode doesn't allow more than
	 * one realtime process to be running. */
	if (rtx_mode_rt != LX_DOMAIN_NATIVE) {
		error = -EBUSY;
		goto errout;
	}
#endif

	/*
	 * Try to register a LX-process as a RT-process (RT_HARD or RT_SOFT).
	 */
	if (cmd_par.exec_flags)
	{
		for (icpu = 0; icpu < RT_MAX_CPUS; icpu++) {
			/*
			 * At this point we have to reserve a CPU for this
			 * process to run as a RT-process. There is at least
			 * one online cpu.
			 */
			if (!cpu_online(icpu))
				continue;

			/*
			 * We have to check that
			 * - the current thread is not the thread
			 *   of an already registered process.
			 * - there is no mix between RT_HARD and RT_SOFT
			 * - this CPU is not already in use.
			 */
			for (islot = 0; islot < RT_MAX_CPUS; islot++) {
				if (rt_proc[islot].slot_state) {
					if (rt_proc[islot].curr_tgid == current->tgid) {
						printkGen(KERN_ALERT, "cannot register process, same tgid\n");
						error = -EBUSY;
						goto errout;
					}
					if (!(rt_proc[islot].state & cmd_par.exec_flags)) {
						printkGen(KERN_ALERT, "cannot register process, exec_flags mismatch\n");
						error = -EINVAL;
						goto errout;
					}
					if (rt_proc[islot].cpu == icpu)
						break;
				}
			}
			if (islot < RT_MAX_CPUS) {
				islot = RT_MAX_CPUS;
				continue;
			}

			/* This CPU is available for a RT-process.
			 * Find a free slot.
			 */
			for (islot = 0; islot < RT_MAX_CPUS; islot++) {
				if (!rt_proc[islot].slot_state) {
					// Initialize the new slot.
					rt_unregister_process(&rt_proc[islot]);
					rt_proc[islot].slot_state = RT_PROC_SLOT_ALLOC;
					rt_proc[islot].slot = islot;
					rt_proc[islot].curr_tgid = current->tgid;
					rt_proc[islot].aud_lx_tgid = 0;
					rt_proc[islot].state = cmd_par.exec_flags;
					rt_proc[islot].cpu = icpu;
					cpu_set(icpu, rt_proc[islot].cpu_mask);
					cpu_set(icpu, rtx_cpu_onlinemask);
					atomic_set(&rt_proc[islot].thread_cnt, 1);
					INIT_LIST_HEAD(&rt_proc[islot].to_lx_list);
					break;
				}
			}
			break;
		}
		if (islot >= RT_MAX_CPUS) {
			printkGen(KERN_ALERT, "cannot register process, no empty slot\n");
			error = -EBUSY;
			goto errout;
		}
		current->rt_proc = &rt_proc[islot];

#ifdef CONFIG_SMP
		rt_pin_rt_process(icpu);
#endif
		/* Now it is safe to initialize core-specific variables */
		rtx_set_cpu_var(rt_system_state, 0);
		current->rt_state = LXRT_TASK;
		error = rt_proc[islot].cpu;
	}
	else
	{											// asks only for memory residence
		head = &aud_rt_hdr.reg_list;
		for (curr = head->next; curr != head; curr = curr->next) {
			if (current->tgid == ((struct reg_proc *)curr)->tgid) {
				error = -EBUSY;							// got already registered
				goto errout;
			}
		}
		if ((reg = (struct reg_proc *) KMALLOC(sizeof(struct reg_proc), GFP_KERNEL)) == NULL) {
				error = -EAGAIN;							// no memory
				goto errout;
		}
		reg->tgid = current->tgid;
		list_add_tail(&reg->p_link, &aud_rt_hdr.reg_list);
		error = 0;
	}
errout:
	up(&aud_rt_hdr.dev_sem);
	return error;
}

/*
 * Initialize the RT-system for RT_HARD or RT_SOFT.
 * In case of memory residency free the entry.
 * Note: It is assumed that the process is already registered.
 */
int probe_realtime(struct probe_descr *pdescr)
{
	struct list_head *curr, *head;
	struct probe_descr wbuf;
	struct timeval now;
	int rtx_mode_prev;
	pid_t pid;
	int ret = 0;
#ifdef CONFIG_HW_LOGGING
	int hwlog = 1;
#else
	int hwlog = 0;
#endif
	int islot = -1;

	down(&aud_rt_hdr.dev_sem);
	if (current->rt_state == LXRT_TASK) {
		for (islot = 0; islot < RT_MAX_CPUS; islot++) {
			/* Check for a valid process registration. */
			if (rt_proc[islot].slot_state) {
				if (rt_proc[islot].curr_tgid != current->tgid)
					continue;
				/* It's another probe realtime for this process? */
				if (rt_proc[islot].aud_lx_tgid) {
					printkGen(KERN_ALERT, "%s failed, already invoked\n", __func__);
					up(&aud_rt_hdr.dev_sem);
					return -EPERM;
				}
				break;
			}
		}
		if (islot >= RT_MAX_CPUS) {
			printkGen(KERN_ALERT, "%s failed, missing process registration\n", __func__);
			up(&aud_rt_hdr.dev_sem);
			return -EPERM;
		}

		/* Verify slot address. */
		if (current->rt_proc != &rt_proc[islot]) {
			printkGen(KERN_ALERT, "%s: FATAL - wrong rt_proc (%d)\n", __func__, islot);
			up(&aud_rt_hdr.dev_sem);
			return -EPERM;
		}

		/* Indication for a successful probe realtime */
		rt_proc[islot].aud_lx_tgid = current->tgid;
	}

	if (pdescr == NULL) {
	    printkGen(KERN_ALERT, "%s failed, missing descriptor\n", __func__);
	    ret = -EINVAL;
	    goto exit_clear;
    }
    if (copy_from_user(&wbuf, (void *)pdescr, sizeof(struct probe_descr))) {
	    ret = -EFAULT;
	    goto exit_clear;
    }
    if (wbuf.version != PROBE_DESCR_VERSION)
    {
	 	printkGen(KERN_ALERT, "%s failed, wrong version\n", __func__);
	    ret = -EINVAL;
	    goto exit_clear;
    }

	if (current->rt_state == LXRT_TASK)
	{
		rtx_or_cpu_var(rt_system_state, current->rt_proc->state);
		/* Set the domain state before the timer daemon starts running. */
		rtx_set_cpu_var(rtx_mode, ((current->rt_proc->state & RT_HARD) ? RT_DOMAIN : LX_DOMAIN_SOFT));
		rtx_mode_prev = rtx_mode_rt;
		rtx_mode_rt = rtx_get_cpu_var(rtx_mode);
		do_gettimeofday(&now);

		if (RT_INIT_VERBOSE || RT_START_VERBOSE) {
		    printkGen("****", "%s realtime process is starting up [progname=%s] ToD=%ld secs\n", (rtx_get_cpu_var(rtx_mode) == RT_DOMAIN) ? "HARD" : "SOFT", current->comm, now.tv_sec);
			printkGen(NULL, "Version: Linux=%s Audis=%s Interrupt-Logging=%s\n", init_uts_ns.name.release, RT_VERSION_STRING, (hwlog == 1) ? "HW" : "SW");
		}

#ifdef CONFIG_SMP
		/* Pin down the RT-IRQs and exclude LX-IRQs from the cpu the RT-process is assigned to. */
		rtx_pin_irq(-1, CPU_MASK_NONE);
#endif

	    if ((ret = init_timer_management(aud_config.no_of_timers.val, aud_config.no_of_headers.val)) < 0)
		{
			rtx_mode_rt = rtx_mode_prev;
			rtx_set_cpu_var(rtx_mode, LX_DOMAIN_NATIVE);
			up(&aud_rt_hdr.dev_sem);
			return ret;
		}
	    if (rtx_get_cpu_var(rtx_mode) == RT_DOMAIN)
		{
			if ((rt_init_realtime(&wbuf)) == 0)
			{
				if ((pid = kernel_thread(interdomain_daemon_thread, 0, CLONE_KERNEL | CLONE_THREAD)) > 0)
				{
					while(!rtx_get_cpu_var(interdomain_daemon))
					{
						set_current_state(TASK_UNINTERRUPTIBLE);
						schedule_timeout(SLOW_POLL_TIME);
					}
					if ((pid = kernel_thread(rt_timer_daemon_thread, 0, CLONE_KERNEL | CLONE_THREAD)) > 0) 	// setup execution of the realtime domain
					{
						while(!rtx_get_cpu_var(rt_timer_daemon) && !rtx_get_cpu_var(td_error))
						{
							set_current_state(TASK_UNINTERRUPTIBLE);
							schedule_timeout(SLOW_POLL_TIME);
						}
						rtx_set_cpu_var(td_error, 0);
						if (RT_CHECK_VERBOSE)
							pid = kernel_thread(rt_check_daemon_thread, 0, CLONE_KERNEL | CLONE_THREAD); 	// kernel thread for checking out timer functions
#ifdef CONFIG_LTT
                                                kernel_thread(ltt_enable_daemon, 0, CLONE_KERNEL | CLONE_THREAD); 	// kernel thread to trigger ltt realtime startup
#endif
					}
				}
			}
			if (rtx_get_cpu_var(rt_timer_daemon) != NULL)		// everything is OK
			{
				ret = RT_HARD;
				if (RT_INIT_VERBOSE)
					printkGen("(mn)", "realtime process initialized\n");
			}
			else
			{
				rtx_mode_rt = rtx_mode_prev;
				rtx_set_cpu_var(rtx_mode, LX_DOMAIN_NATIVE);
				up(&aud_rt_hdr.dev_sem);
				rtx_shutdown_rt_process(0);
				printkGen(KERN_ALERT, "%s: setup realtime process failed\n", __func__);
				return -ENXIO;
			}
		}
		else
		{
			ret = RT_SOFT;
		}
		current->rt_proc->main = current;
		goto exit_up;
	}
	else
	{
		/* memory residency: it can get released afterwards */
		head = &aud_rt_hdr.reg_list;
		ret = -EINVAL;										// in case it is a regular process
		for (curr = head->next; curr != head; curr = curr->next) {
			if (current->tgid == ((struct reg_proc *)curr)->tgid)
			{
				list_del_init(curr);
				KFREE(curr);
				ret = 0;					// it needs proactive mlockall() only
				break;
			}
		}
		goto exit_up;
	}
exit_clear:
	/* Undo register_process */
	if (islot != -1)
		rt_unregister_process(&rt_proc[islot]);
exit_up:
#ifdef CONFIG_RTX_EXTENDED_RT_DEBUGGING_SUPPORT
	initDebugControl();
#endif
	up(&aud_rt_hdr.dev_sem);
	return ret;
}


/*
 * This ioctl is called via the f_ops unlocked_ioctl.
 */
static long AuD_dev_ioctl (struct file *filp, unsigned int cmd, unsigned long arg)
{
   	int ret = 0;
   	int i;
	struct rt_config new_config;
  	struct rt_clock_desc reg_clock_info;
	struct aud_version version_all;
	struct list_head *curr, *head;
 	struct rt_ev_desc event;
	struct task_struct *p = NULL;
	struct thrd_name_buf thrdName;
#ifdef CONFIG_RTX_THREAD_GROUP_SUPPORT
	tgroup_descr_t tgrp_descr;
#endif
#ifdef CONFIG_RTX_TRACE
	trace_descr_t trace_descr;
#endif
	int get_name = 0;

	switch(cmd) {

	case AuD_REGISTER_PROCESS_ATTACH:
	case AuD_REGISTER_PROCESS:
		if (IS_REALTIME)
			return -EINVAL;
	    if (AUD_DEV_VERBOSE)
 		 	printkGen(NULL, "%s: register process pid=%d\n", __func__, current->pid);
		ret = register_process((rt_exec_t *)arg);
		if (cmd == AuD_REGISTER_PROCESS_ATTACH && ret == 0)
			rtx_or_cpu_var(rt_system_state, RT_ATTACH_PROCESS);
		return ret;

	case AuD_PROBE_REALTIME:
		if (IS_REALTIME)
			return -EINVAL;
		return probe_realtime((struct probe_descr *)arg);

	case AuD_IS_REALTIME:
 		return(IS_REALTIME);

 	case AuD_TEST_RT:
	 	if (IS_REALTIME_PROCESS(current)) {
	 		if (rtx_get_cpu_var(rtx_mode) == RT_DOMAIN)
 				return RT_HARD;
 			return RT_SOFT;
		}
		else {
			/* Note: Do not delete (used by startup code) */
			head = &aud_rt_hdr.reg_list;
			ret = -EINVAL;
			for (curr = head->next; curr != head; curr = curr->next) {
				if (current->tgid == ((struct reg_proc *)curr)->tgid)
					return 0;	// it is already registered
			}
			return ret;
		}

 	case AuD_REGISTER_CLOCK:
		if (current->rt_state == LX_TASK)
			return -EPERM;
		if (rt_copy_from_user(&reg_clock_info, (void *)arg, sizeof(struct rt_clock_desc)))
			return -EFAULT;
		return rt_register_sync_clock(filp, &reg_clock_info, CLOCK_SYNC_TIMER, NULL);

 	case AuD_UNREGISTER_CLOCK:
	{
		clockid_t clockid = (clockid_t)arg;

		if (current->rt_state == LX_TASK)
			return -EPERM;

		return rt_unregister_sync_clock(filp, clockid);
	}

 	case AuD_SET_DIAGNOSIS:
		if (IS_REALTIME)
			return -EINVAL;
		down(&aud_rt_hdr.dev_sem);
		{
		if (arg == 0)
			aud_diagnostic = 0;
		else
			aud_diagnostic |= arg;
		}
		break;

 	case AuD_GET_VERSION:
		if (IS_REALTIME)
			return -EINVAL;
		down(&aud_rt_hdr.dev_sem);
		version_all.version_dev = dev_version;
		version_all.version_rt = dev_version;
		ret = rt_copy_to_user((struct aud_version *)arg, &version_all, sizeof(struct aud_version)) ? -EFAULT : 0;
		break;

 	case AuD_UNREGISTER_PROCESS_DETACH:
 	case AuD_UNREGISTER_PROCESS:
		if (IS_REALTIME)
			return -EINVAL;
		if (current->rt_proc && (current->rt_proc->aud_lx_tgid == current->tgid))
		{
			if (cmd == AuD_UNREGISTER_PROCESS_DETACH && rtx_get_cpu_var(rt_system_state) & RT_ATTACH_PROCESS)
				rtx_or_cpu_var(rt_system_state, RT_DETACH_PROCESS);
			rtx_shutdown_rt_process(0);		// a realtime process is installed
			return 0;
		}
	  	down(&aud_rt_hdr.dev_sem);
		head = &aud_rt_hdr.reg_list;
		for (curr = head->next; curr != head; curr = curr->next)
			if (current->tgid == ((struct reg_proc *)curr)->tgid)
			{
				list_del_init(curr);
				KFREE(curr);
				ret = 1;
				break;
			}
		break;

    case AuD_DEV_RT_PARAM:
		if (IS_REALTIME)
			return -EINVAL;
    	down(&aud_rt_hdr.dev_sem);
		if (!arg)
		{
			ret = -EINVAL;
			break;
		}
		if (rt_copy_from_user(&new_config, (struct rt_config *)arg, sizeof(struct rt_config)) == 0)
			ret = parameterize_configuration(&new_config);
		else
			ret = -EFAULT;
		break;
	
	// Only for test purposes, one user assumed (no locking).
    case AuD_EVENT_CREATE:
		if (rt_copy_from_user(&event, (struct rt_ev_desc *)arg, sizeof(event)))
			return -EFAULT;
		return rt_register_event(rtx_get_cpu_var(aud_dev_ev_area_id), &event);

    case AuD_DEV_SEND_EVENT:
    	rtx_set_cpu_var(aud_evpr, NULL);
    	for (i = 0; i < NO_OF_EVENTS; i++) {
    		struct rt_event *pevarr = &__raw_get_cpu_var(aud_dev_ev_arr[i]);
    		if ((pevarr->ev_id == (int)arg)) {
    			rtx_set_cpu_var(aud_evpr, pevarr);
				break;
			}
    	}
    	if (!rtx_get_cpu_var(aud_evpr))
    		return -1;
    	if (!IS_REALTIME && RTX_SYS_IS_RT_DOMAIN) {
    		__rtx_call_rt_domain(to_rt_virq);
    	}
		else
		{
	    	__raw_get_cpu_var(isr_time) = rtx_gettime();
			rt_send_event(rtx_get_cpu_var(aud_evpr));
		}
    	return 0;

	case AuD_WAIT_FOR_EVENT:
		return rt_wait_for_event();

    case AuD_DEV_GET_TIME:
	 	return (rt_copy_to_user((uint64_t *)arg, &__raw_get_cpu_var(isr_time), sizeof(uint64_t)) ? -EFAULT : 0);

    case AuD_DEV_GET_TIME_STAMP:
	{
		uint64_t time_nsec = RTX_TIMER_GET;
	 	return (rt_copy_to_user((uint64_t *)arg, &time_nsec, sizeof(uint64_t)) ? -EFAULT : 0);
	}

	case AuD_DEV_GET_THREAD_RUNTIME:
#ifdef CONFIG_RTX_THREAD_RUNTIME
    {
    	uint64_t acc_runtime;
    	struct thread_runtime_par time_par;
		if (rt_copy_from_user(&time_par, (struct thread_runtime_par *)arg, sizeof(time_par)))
			return -EFAULT;

		if (time_par.pid == current->pid) {
			if (IS_LINUX)
				acc_runtime = current->rt_thread_runtime;
			else
				acc_runtime = (RTX_TIMER_GET - current->rt_thread_switched_in) + current->rt_thread_runtime;
		}
		else {
			if (IS_LINUX) {
				read_lock(&tasklist_lock);
				p = find_task_by_vpid(time_par.pid);
				read_unlock(&tasklist_lock);
			} else {
				p = rt_find_task_by_pid(time_par.pid);
			}
			if (p == NULL)
				return -ESRCH;

			acc_runtime = p->rt_thread_runtime;
		}
		/* Return the accumulated runtime of a thread in the RT-domain */
		if (rt_copy_to_user(time_par.time, &acc_runtime, sizeof(uint64_t)))
			return -EFAULT;
    	return 0;
    }
#else
		return -EOPNOTSUPP;		// corresponds to ENOTSUP in glibc
#endif
    case AuD_DEV_GET_CPU:
		return rtx_processor_id();

    case AuD_DEV_GET_THRD_NAME:
		get_name = 1;
    case AuD_DEV_SET_THRD_NAME:
       /*
        * The AuD API defines thread names; we use the task structure element "comm"
        * to store this name within the kernel (and replacing the name of the process).
        */
		if (rt_copy_from_user(&thrdName, (struct thrd_name_buf *)arg, sizeof(thrdName)))
			return -EFAULT;
		/*
		 * Assumption: We are always in the same domain as the new thread is.
		 */
		if (IS_LINUX) {
			read_lock(&tasklist_lock);
			p = find_task_by_vpid(thrdName.thrdPid);
			read_unlock(&tasklist_lock);
		} 
		else {
			// It's no longer necessary to use a blocking semaphore.
			p = rt_find_task_by_pid(thrdName.thrdPid);
		}
		if (p == NULL)
			return -EINVAL;
		
		/* Get the thread name from task_struct */
		if (get_name) {
			if (rt_copy_to_user(thrdName.thrdName, p->comm, TASK_COMM_LEN))
				return -EFAULT;
			return 0;
		}
		/* Set a new thread name */
		if (rt_copy_from_user(p->comm, thrdName.thrdName, TASK_COMM_LEN))
			return -EFAULT;
		/* Be sure to make a string */
		p->comm[TASK_COMM_LEN - 1] = '\0';
#ifdef CONFIG_LTT
		p->ltt_trace_id = 0; // be sure to restart thread naming tracing
#endif
		return 0;

    case AuD_DEV_RETURN_TO_LINUX:
#ifdef CONFIG_RTX_RETURN_TO_LINUX
		return rt_sched_return_to_lx((int)arg);		// go temporarily back to Linux
#else
		return -ENOSYS;
#endif

    case AuD_DEV_ALLOW_SETSCHEDULER:
    {
		current->rt_state3 |= RT_TASK_ALLOW_SETSCHEDULER;			// used by glibc
		return 0;
    }

    case AuD_DEV_ALLOW_SETAFFINITY:
    {
    	struct allow_set_par setpar;

		if (rt_copy_from_user(&setpar, (struct allow_set_par *)arg, sizeof(setpar)))
			return -EFAULT;

		/* Allow a thread to migrate to another CPU. */
		current->rt_state3 |= RT_TASK_ALLOW_SETAFFINITY;			// used by glibc
		if (rtx_is_rt_capable(setpar.policy, setpar.prio))
			current->rt_state3 |= RT_TASK_REALTIME;
		return 0;
    }

    case AuD_DEV_RESET_SCHED_FLAGS:
    {
		CLEAR_SCHEDULER_MASK(current);
		CLEAR_AFFINITY_MASK(current);
		return 0;
    }

    case AuD_DEV_STORE_TID:
    	current->rt_threadid = arg;			// thread-id (not pid) of the calling thread for pthread_kill
    	return 0;

    case AuD_TGROUP_CREATE:
    case AuD_TGROUP_DELETE:
    case AuD_TGROUP_THREAD_ADD:
    case AuD_TGROUP_THREAD_REMOVE:
    case AuD_TGROUP_SETQUOTA:
    case AuD_TGROUP_GETQUOTA:
#ifdef CONFIG_RTX_THREAD_GROUP_SUPPORT
    	if (IS_LINUX)
    		return -ENOSYS;
		if (rt_copy_from_user(&tgrp_descr, (tgroup_descr_t *)arg, sizeof(tgrp_descr)))
			return -EFAULT;
		return rt_tgroup_common(cmd, &tgrp_descr);
#else
		return -ENOSYS;
#endif

    case AuD_TRACE_MAX_BEGIN:
    case AuD_TRACE_MAX_END:
    case AuD_TRACE_MAX_RESET:
    case AuD_TRACE_USER_START:
    case AuD_TRACE_USER_STOP:
    case AuD_TRACE_USER_FREEZE:
    case AuD_TRACE_SPECIAL:
    case AuD_TRACE_SPECIAL_U64:
#ifdef CONFIG_RTX_TRACE
		if (rt_copy_from_user(&trace_descr, (trace_descr_t *)arg, sizeof(trace_descr)))
			return -EFAULT;
		return rt_trace_common(cmd, &trace_descr);
#else
    	return -ENOSYS;
#endif

    case AuD_DEV_ATTACH_DEBUG:
    case AuD_DEV_DETACH_DEBUG:
    case AuD_DEV_WAIT_FOR_DEBUG_EVENT:
    case AuD_DEV_HANDLE_DEBUG_EVENT:
#ifdef CONFIG_RTX_EXTENDED_RT_DEBUGGING_SUPPORT
	return(rt_debug_action(cmd, arg));
#else
	return -ENOSYS;
#endif

    case AuD_DEV_TEST_WATCHDOG_EVENT:
    case AuD_DEV_FORCE_USER_BREAK:
    case AuD_DEV_TEST_WATCHDOG_IN_DEBUG:
#ifdef CONFIG_RTX_RT_WATCHDOG_SUPPORT
	return(rt_watchdog_action(cmd, arg));
#else
	return -ENOSYS;
#endif

	default:
		printkGen(KERN_ALERT, "wrong ioctl command=%#x\n", (unsigned)cmd);
	   return -EINVAL;
   }
   up(&aud_rt_hdr.dev_sem);
   return(ret);
}


static int AuD_dev_open (struct inode *inode, struct file *filp)
{
   int ret;


    if (filp->f_flags & O_EXCL)
    {
		printkGen(KERN_ALERT, "can't open %s exclusively\n", AUDIS_DRIVER_PATH);
		return -EINVAL;
   	}

    if (current->rt_state) {
  	     if ((ret = rt_allow_access(filp, RT_IO_IOCTL))< 0)
	   	     return ret;
	}

	if (AUD_DEV_VERBOSE)
		printkGen(NULL, "%s: %s open rt_dev_open_cnt=%d rt_state=%#lx\n", __func__, AUDIS_DRIVER_PATH,
				rtx_get_cpu_var(rt_dev_open_cnt), current->rt_state);

   	return 0;
}

/*
 * Gets called when the last close() on a filp gets executed.
 */
static int AuD_dev_release (struct inode *inode, struct file *filp)
{
	if (current->rt_state) {
		rt_remove_access(filp);
	}
	if (AUD_DEV_VERBOSE)
		printkGen(NULL, "%s: %s open rt_dev_open_cnt=%d rt_state=%d\n", __func__, AUDIS_DRIVER_PATH, rtx_get_cpu_var(rt_dev_open_cnt), current->rt_state);
	return 0;
}

/*
 * Is called when the module gets loaded.
 */
int init_AuD_dev(void)
{
	void *ptr_err;
	int ret, cpu;
	int major = 0;

	printk("init %s\n", AUDIS_DRIVER_NAME);

#ifndef CONFIG_RTX_DYNAMIC_MAJOR_NUMBER
	major = CONFIG_RTX_STATIC_MAJOR_NUMBER;
#endif

	/* Register the Audis device with a statically or dynamically
	 * allocated major number. */
	if ((major_audis = register_chrdev(major, AUDIS_DRIVER_NAME, &AuD_dev_fops)) < 0)
	{
		printk("init %s: register_chrdev failed (%d)\n", AUDIS_DRIVER_NAME, major_audis);
		return(major_audis);
   	}
#ifndef CONFIG_RTX_DYNAMIC_MAJOR_NUMBER
	major_audis = major;		// static allocation
#endif

	/* Create the Audis driver device node dynamically. */
	class_audis = class_create(THIS_MODULE, AUDIS_DRIVER_NAME);
	if (IS_ERR(ptr_err = class_audis)) {
		printk("init %s: class_create failed (%d) major=%d\n", AUDIS_DRIVER_NAME, (int)PTR_ERR(ptr_err), major_audis);
err:
		unregister_chrdev(major_audis, AUDIS_DRIVER_NAME);
		major_audis = 0;
		return PTR_ERR(ptr_err);
	}
	dev_audis = device_create(class_audis, NULL, MKDEV(major_audis, 0), NULL, AUDIS_DRIVER_NAME);
	if (IS_ERR(ptr_err = dev_audis)) {
		printk("init %s: device_create failed (%d) major=%d\n", AUDIS_DRIVER_NAME, (int)PTR_ERR(ptr_err), major_audis);
		class_destroy(class_audis);
		goto err;
	}

	rtx_init_isr_management();
	INIT_LIST_HEAD(&aud_rt_hdr.reg_list);
	sema_init(&aud_rt_hdr.dev_sem, 1);
	sema_init(&scl_sem, 1);

	for (cpu = 0; cpu < NR_CPUS; cpu++) {
		if ((ret = rt_init_event_area(&per_cpu(aud_dev_ev_arr[0], cpu), NO_OF_EVENTS)) < 0)
		{
			printk("init %s: could not initialize event area for cpu=%d\n", AUDIS_DRIVER_NAME, cpu);
			device_destroy(class_audis, MKDEV(major_audis, 0));
			class_destroy(class_audis);
			unregister_chrdev(major_audis, AUDIS_DRIVER_NAME);   // unregister the driver
			major_audis = 0;
			return ret;
		}
		per_cpu(aud_dev_ev_area_id, cpu) = ret;
		per_cpu(rt_system_state, cpu) = 0;
		per_cpu(rt_dev_open_cnt, cpu) = 0;
	}

	init_io_management();
	rt_init_futex();
   	return 0;
}


void exit_AuD_dev(void)
{
	printk("cleanup module %s\n", AUDIS_DRIVER_NAME);
	if (major_audis > 0)
	{
		device_destroy(class_audis, MKDEV(major_audis, 0));
		class_destroy(class_audis);
		unregister_chrdev(major_audis, AUDIS_DRIVER_NAME);   // unregister the driver
	}
}


/*
 * generic printk for Audis
 * @ctrl_str: control string
 * @format:   format string
 * The control string may be NULL (nothing happens).
 * If the control string has the format "<x>" then it is interpreted
 * as a kernel log level which is added to printk.
 * Otherwise, the control string must have the format "xxxx" and
 * it is prefixed to the string to be displayed.
 */
int printkGen(char *ctrl_str, char *format, ...)
{
	int len, len1;
  	char printBuff[MAX_PRINT_BUFF + MAX_PREFIX];
#ifndef CONFIG_RTX_DISABLE_WRITE_DAEMON
  	struct type_info *tp = &type_arr[0];
#endif
  	char *log_str = NULL;
#ifdef CONFIG_SMP
  	int cur_cpu = rtx_processor_id();
#endif
  	pid_t cur_pid = current->pid;

	va_list args;
	va_start(args, format);

	// Check whether the control string is a log level string.
	if (ctrl_str && ctrl_str[0] == '<' && ctrl_str[2] == '>') {
		log_str = ctrl_str;
		ctrl_str = NULL;
	}

	// Start with a control string if available.
	len1 = ctrl_str ? sprintf(printBuff, "%s ", ctrl_str)
                        : sprintf(printBuff, "     ");

	len = vsnprintf(printBuff + len1, MAX_PRINT_BUFF, format, args);
	va_end(args);
	len += len1;

#ifndef CONFIG_RTX_DISABLE_WRITE_DAEMON
	if ((RT_PRINT_STDOUT || RT_PRINT_STDERR) && WRITE_DAEMON_IS_ACTIVE(current))
	{
		if (RT_PRINT_STDERR)
			tp += 2;
		if (RTX_SYS_IS_RT_DOMAIN)
			tp += 1;
		return rt_write_buffered(0, tp, printBuff, len);
	}
#endif
#ifdef CONFIG_SMP
	log_str ? printk("%s%sP%5d: C%2d: %s", log_str, RTX_SYS_IS_RT_DOMAIN ?  RT_PRINTK_MSG : LX_PRINTK_MSG, cur_pid, cur_cpu, printBuff)
			: printk("%sP%5d: C%2d: %s", RTX_SYS_IS_RT_DOMAIN ?  RT_PRINTK_MSG : LX_PRINTK_MSG, cur_pid, cur_cpu, printBuff);
#else
	log_str ? printk("%s%sP%5d: %s", log_str, RTX_SYS_IS_RT_DOMAIN ?  RT_PRINTK_MSG : LX_PRINTK_MSG, cur_pid, printBuff)
			: printk("%sP%5d: %s", RTX_SYS_IS_RT_DOMAIN ?  RT_PRINTK_MSG : LX_PRINTK_MSG, cur_pid, printBuff);
#endif

	return(len);
}

EXPORT_SYMBOL(aud_rt_timer);
EXPORT_SYMBOL(printkGen);

/* **************************************************************** */
/* if a module acts as a driver it needs modul_init / modul_exit 	*/
/* for a module only init_module / cleanup_module is sufficient		*/
/* **************************************************************** */

module_init(init_AuD_dev);
module_exit(exit_AuD_dev);



