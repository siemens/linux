/*
 * drivers/aud/auddevice/rt_sched.c
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
 * This file contains all the  routines needed for UP preemptive scheduling
 * 
 */
#ifdef CONFIG_LTT
#include <linux/ltt-core.h>
#endif
#include <linux/aud/rt_timer.h>
#include <linux/aud/rt_futex.h>
#include <asm/mmu_context.h>
#ifdef CONFIG_RTX_THREAD_GROUP_SUPPORT
#include <linux/aud/rt_tgroup.h>
#endif
#ifdef CONFIG_RTX_EXTENDED_RT_DEBUGGING_SUPPORT
#include <linux/aud/rt_debug.h>
#endif
#include <trace/events/sched.h>

#ifdef CONFIG_LTT
#include <trace/rt.h>
DEFINE_TRACE(rt_process_schedchange);
DEFINE_TRACE(rt_fs_exec);
extern unsigned actLttTraceId;
#endif

struct smp_sched_change {
	struct task_struct *p;
	int policy;
	int prio;
	int ret;
};

static DEFINE_PER_CPU(unsigned int, rt_yield_count) = 0;
static DEFINE_PER_CPU(struct task_struct *, rt_req_task) = {NULL};
DEFINE_PER_CPU(struct task_struct *, lx_curr_task);
DEFINE_PER_CPU(runqueue_t, rt_runqueue);
DEFINE_PER_CPU(runqueue_t *, prt_runqueue);
DEFINE_PER_CPU(migqueue_t, rt_migqueue);

DECLARE_PER_CPU(struct list_head, rt_tl);

/*
 * We have to initialize the core specific parts
 * only for the current CPU. This function is called in LX context,
 * but the LX-thread has already been pinned to the current CPU.
 */
void rt_init_sched(void)
{
	int ii;
	migqueue_t *pmq = &__raw_get_cpu_var(rt_migqueue);
	
	for (ii = 0; ii < MAX_NO_OF_RT_PRIO; ii++) {
		INIT_LIST_HEAD(&(rtx_get_cpu_var(prt_runqueue)->queue[ii].qhead));
		rtx_get_cpu_var(prt_runqueue)->queue[ii].idx = ii;
	}
	rtx_get_cpu_var(prt_runqueue)->nr_running = 0;
	rtx_get_cpu_var(prt_runqueue)->bitmapPrio = 0;

	/* Task requested for execution. */
	rtx_set_cpu_var(rt_req_task, NULL);

	for (ii = 0; ii < MAX_NO_OF_RT_PRIO; ii++) {
		INIT_LIST_HEAD(&pmq->queue[ii].qhead);
		pmq->queue[ii].idx = ii;
	}
	pmq->bitmapPrio = 0;
	rtx_set_cpu_var(rt_yield_count, 0);
}


/*
 * Called for the RT domain after executing an ISR. 
 * Note: HW interrupts must be off.
 */
void rtx_preemption_handling(void)
{
    struct task_struct *rt_curr_task;
    struct task_struct *preq_task = rtx_get_cpu_var(rt_req_task);
#ifdef CONFIG_PREEMPT_NOTIFIERS
    struct preempt_notifier *notifier;
    struct hlist_node *node;
#endif
#ifdef CONFIG_RTX_THREAD_GROUP_SUPPORT
	rt_tgroup_head_t *pqh = rtx_get_cpu_var(pquota_head);
#endif
	
	if (preq_task == NULL || (preq_task == current))
		return;

	if (IS_LINUX)
	{
		rtx_sys_check_sched_integrity();

#ifdef CONFIG_PREEMPT_NOTIFIERS
		// Call the sched_out notifier (so far this can only be kvm_sched_out) when
		// switching from Linux to RT. This restores the complete host state after a
		// VM exit and allows RT threads to safely preempt VCPU threads.
		// Note: Adapted from the adeos-ipipe patch.
		hlist_for_each_entry(notifier, node, &current->preempt_notifiers, link) {
			notifier->ops->sched_out(notifier, preq_task);
		}
#endif

		// Current stack is LX stack.
		rtx_set_cpu_var(lx_curr_task, current);
		if (current->rt_state == LX_TASK) {				// task of another process has other address space
			rt_switch_mm(current->active_mm, preq_task->active_mm, preq_task);  // switch address space
		}
	}
	else {
		THREAD_RUNTIME_PRE_SWITCH();		// runtime accounting
#ifdef CONFIG_RTX_THREAD_GROUP_SUPPORT
		quota_pre_switch(pqh, preq_task);
#endif
	}
	rt_curr_task = current;
        trace_sched_switch_rt(NULL, rt_curr_task, preq_task);
#ifdef CONFIG_LTT
	/* First step to integrate ltt support: show rt thread with negative state */
	trace_rt_process_schedchange(rt_curr_task->pid, preq_task->pid, -rt_curr_task->rt_state);
#endif
#if defined(CONFIG_RTX_TRACE) && defined(CONFIG_RTX_TRACE_CONTEXT_SWITCH)
	/* Trace the RT context switch. */
	rt_trace_pid(rt_curr_task->pid, rt_curr_task->rt_state);
	rt_trace_pid(preq_task->pid, preq_task->rt_state);
#endif
	rt_switch_to(rt_curr_task, preq_task, rt_curr_task);
	if (IS_LINUX)
	{
		/* 
		 * Returning to a native Linux process:
		 * For the previous active_mm we need a reference to
		 * the RT-thread, which was the requested task on
		 * entry of this function (see above).
		 */							
		if (current->rt_state == LX_TASK && current->active_mm) {
			/* Switch the address space. */
			rt_switch_mm(preq_task->active_mm, current->active_mm, current);
		}
		rtx_set_cpu_var(rt_req_task, NULL);
		rtx_set_cpu_var(lx_curr_task, NULL);
		return;
	}	
	THREAD_RUNTIME_POST_SWITCH();		// runtime accounting
#ifdef CONFIG_RTX_THREAD_GROUP_SUPPORT
	quota_post_switch(pqh);
#endif
}
 

/*
 * Return the highest priority RT thread.
 */
inline struct task_struct * rt_get_next_thread(runqueue_t *prq)
{
	int ii;
	struct list_head *queue;
	
	ii = __ffs(prq->bitmapPrio);	// search for the first bit, starting at top
	queue = prq->queue[ii].qhead.next;
	return((struct task_struct *)list_entry(queue, struct task_struct, rt_run_list));
}

#ifdef CONFIG_RTX_RETURN_TO_LINUX
/*
 * Try to set the highest priority RT thread.
 * Note: Interrupts must be off.
 */
void rt_try_next_thread(struct task_struct *p)
{
	runqueue_t *prq = rtx_get_cpu_var(prt_runqueue);

	if (prq->bitmapPrio) {
		rtx_set_cpu_var(rt_req_task, rt_get_next_thread(prq));
		if (p->rt_state2 & RT_TASK_RETURNED_TO_LX) {
			p->rt_state2 |= RT_TASK_SIGNAL_RECEIVED;
		}
		else
			p->rt_proc->r2l &= ~RT_SIG_WAIT_R2L;
	}
}
#endif

/*
 * Add a thread to the run queue.
 * Note: HW interrupts must be off.
 */
inline void rt_enqueue_thread(runqueue_t *prq, struct task_struct *p)
{
	int idx;
	
	if (unlikely((idx = p->prio) >= MAX_NO_OF_RT_PRIO))
	{											// must be a thread which is due to migration
		idx = MAX_NO_OF_RT_PRIO - 1;			// For executing migration we queue the thread 
		p->rt_state2 |= RT_LEAVE_PENDING_TASK;		// at the lowest priority
	}
#ifdef CONFIG_RTX_THREAD_GROUP_SUPPORT
	else {
		 /* As long as the thread is in the blocked queue p->prio
		 * must not be changed to a value in the range of LX-priorities. */
		if (RT_QUOTA_IS_EXCEEDED(p)) {
			if (list_empty(&p->rt_blocked_list)) {
				list_add_tail(&p->rt_blocked_list, rtx_get_cpu_var(prt_blocked_queue));
			}
			return;
		}
	}
#endif
	list_add_tail(&p->rt_run_list, &prq->queue[idx].qhead);
	__set_bit(idx, &prq->bitmapPrio);
	rtx_set_cpu_var(rt_req_task, rt_get_next_thread(prq));
}

/*
 * Remove a thread from the run queue.
 * Note: HW interrupts must be off.
 */
inline void rt_dequeue_thread(runqueue_t *prq, struct task_struct *p)
{
	struct list_head *lp;
	struct rq_head *qp;

#ifdef CONFIG_RTX_THREAD_GROUP_SUPPORT
	/* If a thread is not in the runq, we are done. */
	if (list_empty(&p->rt_run_list)) {
		/* If a thread is already in the blocked list, we are done. */
		return;
	}
#endif

	lp = p->rt_run_list.prev;						// in case p was the only one,
	list_del_init(&p->rt_run_list);                	// .prev points to the queue head
	if (list_empty(lp))								// which is then empty
	{
		qp = (struct rq_head *)list_entry(lp, struct rq_head, qhead);
		__clear_bit(qp->idx, &prq->bitmapPrio);
	}
#ifdef CONFIG_RTX_THREAD_GROUP_SUPPORT
	if (RT_QUOTA_IS_EXCEEDED(p)) {
		list_add_tail(&p->rt_blocked_list, rtx_get_cpu_var(prt_blocked_queue));
	}
#endif
}

/*
 * For an optimized rt scheduling code no spinlock is used for the runqueue.
 * Wake up of a RT thread from the Linux domain can only be done from the same CPU.
 * This is safe, since we do not allow a LX-thread of the RT-process to
 * migrate to another cpu.
 */
void lx_wake_up_rt_thread(struct task_struct *p)
{
	if (p)
		rt_wake_up_thread(p);
	__rtx_call_rt_domain(to_rt_virq);
}

/* Special call for the interdomain daemon. */
void lx_wake_up_rt_thread_id(struct task_struct *p)
{
	if (p)
		rt_wake_up_thread_id(p);
	__rtx_call_rt_domain(to_rt_virq);
}

/* This routine may be called
 * - from a rt thread, a subsequent rt_schedule() may lead to a switch
 * - from an ISR
 * - from a Linux thread, a subsequent soft-interrupt (VIRQ) for the RT-domain leads to a switch
 */
int rt_wake_up_thread(void *arg)
{
	struct task_struct *p = (struct task_struct*) arg;
	unsigned long flags;
	
	local_irq_save_hw(flags); 					// we may be called from ISR with interrupts already disabled

	/*
	 * When the thread is already on its way to LX
	 * then don't wake it up again.
	 */
	if (p->rt_state & (RTLX_TASK | LXRT_TASK)) {
	    local_irq_restore_hw(flags);
	    return 0;
	}

	p->rt_state &= (RT_TASK_EXIT_PENDING | RT_TASK_TIMER_CLEANUP | RT_TASK_FUTEX_CLEANUP
			| RT_TASK_SEM_CLEANUP | RT_TASK_MAIN | LXRT_TASK);
	
	/*
	 * The thread may already be queued in or
	 * it may be marked for execution in LX (LXRT_TASK).
	 */
	p->rt_state |= RT_TASK_RUNNING;
	if (likely(list_empty(&p->rt_run_list))) {
		rt_enqueue_thread(rtx_get_cpu_var(prt_runqueue), p);
	}
    local_irq_restore_hw(flags);  				// we may be called from ISR with interrupts disabled   		   
    return 0;
}

/* Special call for the interdomain daemon. */
int rt_wake_up_thread_id(void *arg)
{
	struct task_struct *p = (struct task_struct*) arg;
	unsigned long flags;

	local_irq_save_hw(flags); 					// we may be called from ISR with interrupts already disabled

	p->rt_state &= (RT_TASK_EXIT_PENDING | RT_TASK_TIMER_CLEANUP | RT_TASK_FUTEX_CLEANUP
			| RT_TASK_SEM_CLEANUP | RT_TASK_MAIN | LXRT_TASK);

	/*
	 * The thread may already be queued in or
	 * it may be marked for execution in LX (LXRT_TASK).
	 */
	p->rt_state |= RT_TASK_RUNNING;
	if (likely(list_empty(&p->rt_run_list))) {
		rt_enqueue_thread(rtx_get_cpu_var(prt_runqueue), p);
	}
    local_irq_restore_hw(flags);  				// we may be called from ISR with interrupts disabled
    return 0;
}

/* 
 * rt_schedule gets only called from a RT thread 
 * Note: It uses the pair local_irq_disable_hw()/local_irq_enable_hw(). 
 * First, for the RT domain it is guaranteed, that there is no nesting and 
 * second when continuing a thread after a domain switch the interrupts 
 * must be enabled unconditionally.
 */
struct task_struct *rt_schedule(int rt_state)
{
	struct task_struct *preq_task;
	struct task_struct *rt_curr_task;
	runqueue_t *prq = rtx_get_cpu_var(prt_runqueue);
#ifdef CONFIG_RTX_THREAD_GROUP_SUPPORT
	rt_tgroup_head_t *pqh = rtx_get_cpu_var(pquota_head);
#endif

#ifdef CONFIG_LTT
	if (ltt_traces.num_active_traces) {
		/*
		* When LTT traces are active, we need to get the names
		* of the currently active threads. We check a flag, if not
		* set to our id, we transfer the thread name to LTT. 
		*/
		if (current->ltt_trace_id != actLttTraceId) {
/*			lttng_sequence_fs_exec_filename lttng_name;
			current->ltt_trace_id = actLttTraceId;
			lttng_name.len = strlen(current->comm) + 1;
			lttng_name.array = current->comm;
			trace_rt_fs_exec(&lttng_name);*/
			trace_rt_fs_exec(current->comm);
		}
	}
	else {
		current->ltt_trace_id = 0;
	}
#endif
	local_irq_disable_hw(); 

	if (rt_state)
		current->rt_state = rt_state;	
	if (!(current->rt_state & RT_TASK_RUNNING))
	{ 		
		rt_dequeue_thread(prq, current);
		/*
		* Make sure that rt_wake_up_thread
		* doesn't re-enqueue this thread.
		*/
		if (current->rt_state & LXRT_TASK)
			current->rt_run_list.next = NULL;		// fake a non-empty list

		if (!(prq->bitmapPrio))
			rtx_set_cpu_var(rt_req_task, rtx_get_cpu_var(lx_curr_task));				// we have to switch to the Linux domain
		else
			rtx_set_cpu_var(rt_req_task, rt_get_next_thread(prq));
	}		
	rt_curr_task = current;
	if (rtx_get_cpu_var(rt_req_task) != current)
	{
		preq_task = rtx_get_cpu_var(rt_req_task);
                trace_sched_switch_rt(NULL, rt_curr_task, preq_task);
#ifdef CONFIG_LTT
		/* First step to integrate ltt support: show rt thread with negative state */
		trace_rt_process_schedchange(rt_curr_task->pid, preq_task->pid, -rt_curr_task->rt_state);
#endif
#if defined(CONFIG_RTX_TRACE) && defined(CONFIG_RTX_TRACE_CONTEXT_SWITCH)
		/* Trace the RT context switch. */
		rt_trace_pid(rt_curr_task->pid, rt_curr_task->rt_state);
		rt_trace_pid(preq_task->pid, preq_task->rt_state);
#endif
#ifdef CONFIG_RTX_THREAD_GROUP_SUPPORT
		quota_pre_switch(pqh, preq_task);
#endif
		THREAD_RUNTIME_PRE_SWITCH();		// runtime accounting
		rt_switch_to(rt_curr_task, preq_task, rt_curr_task);
		if (IS_LINUX)						// rt_schedule was called in execute_in_lx
			return rt_curr_task;			// unlocking was done when leaving the rt domain
		THREAD_RUNTIME_POST_SWITCH();		// runtime accounting
#ifdef CONFIG_RTX_THREAD_GROUP_SUPPORT
		quota_post_switch(pqh);
#endif
	}			 
	local_irq_enable_hw();	
	
	// RT_TASK_DEBUG_PENDING: Migration to LX is done later as part of the system call handling.

	if (IS_EXIT_PENDING(current) &&
			!(current->rt_state & (RT_TASK_TIMER_CLEANUP | RT_TASK_FUTEX_CLEANUP | RT_TASK_SEM_CLEANUP)))
		rt_task_leave();		 
	return rt_curr_task;
}

/*
 * This routine is needed for dynamic priority changes.
 */
int rt_reschedule(struct task_struct *p)
{
	local_irq_disable_hw();  						
	if ((p->prio >= MAX_NO_OF_RT_PRIO) && (p == current))
	{
		/* Need to migrate to Linux. */
		local_irq_enable_hw();
		rtx_migrate_to_lx(LXRT_TASK, NULL);		// check for notifications was already done
		return 0;
	}	

	if (p->policy == SCHED_RR)					
		setup_rr_timer(p);

	if (list_empty(&p->rt_run_list))
	{
		/* Thread is not active. */
		local_irq_enable_hw();			
		return 0;
	}
	rt_dequeue_thread(rtx_get_cpu_var(prt_runqueue), p);
	rt_enqueue_thread(rtx_get_cpu_var(prt_runqueue), p);
	local_irq_enable_hw();							
	if (IS_REALTIME)
		rt_schedule(0);
	else
		__rtx_call_rt_domain(to_rt_virq); // currently unused
	return 0;	
}

/*
 * Short subroutine to re-insert a task into the rt_runqueue.
 * The scheduling request must be done separately.
 * Note: HW interrupts must be off.
 */
int requeue_task(struct task_struct *p)
{
	if (!(list_empty(&p->rt_run_list))) {
		/* Thread is active. */
		rt_dequeue_thread(rtx_get_cpu_var(prt_runqueue), p);
		rt_enqueue_thread(rtx_get_cpu_var(prt_runqueue), p);
		return 1;
	}
	return 0;
}

_INT64 rt_schedule_timeout(struct rt_list_hdr *lhdr, _INT64 timeout)
{
	struct itimer itmr;
	_INT64 remaining_time;
	
	setup_internal_timer(lhdr, &itmr, timeout, rt_wake_up_thread);
	/* a very short timer may already be expired and we don't want to miss the runnable bit */
    local_irq_disable_hw();
	current->rt_state |= RT_TASK_TIMER_CLEANUP;
	rt_schedule(0);	
	remaining_time = delete_internal_timer(&itmr);
	current->rt_state &= ~RT_TASK_TIMER_CLEANUP;
	if (IS_EXIT_PENDING(current) && !(current->rt_state & RT_TASK_FUTEX_CLEANUP))
		return rt_task_leave();
	return(remaining_time);
}

/*
 * gets called from either sys_rt_sched_yield or 
 * from the tick routine when the time slice has expired
 */
void rt_sched_rr(void)
{
	unsigned long flags;
	runqueue_t *prq = rtx_get_cpu_var(prt_runqueue);

	local_irq_save_hw(flags); 		
    rt_dequeue_thread(prq, current);
	rt_enqueue_thread(prq, current);
#ifdef CONFIG_RTX_THREAD_GROUP_SUPPORT
	if (!prq->bitmapPrio)
		rtx_set_cpu_var(rt_req_task, rtx_get_cpu_var(lx_curr_task));
	else
#endif
	rtx_set_cpu_var(rt_req_task, rt_get_next_thread(prq));
	local_irq_restore_hw(flags);	
}

/*
 * rr task: timeout a certain amount to allow higher prior rt threads
 * currently running in the linux domain to work and return
 * <ToDo>: the timeout value may be too long (1 timer tick is SLEEP_TIME_YIELD)
 */
void rt_sched_rr_delay(void)
{
	unsigned long flags;
	runqueue_t *prq = rtx_get_cpu_var(prt_runqueue);

	local_irq_save_hw(flags);
    rt_dequeue_thread(prq, current);
	if (!(prq->bitmapPrio))
		rtx_set_cpu_var(rt_req_task, rtx_get_cpu_var(lx_curr_task));
	else
		rtx_set_cpu_var(rt_req_task, rt_get_next_thread(prq));
	local_irq_restore_hw(flags);
}

/*
 * Check if we have a situation which may cause priority inversion.
 */
int rt_sched_has_delay(void)
{
	migqueue_t *pmq;

#ifdef CONFIG_RTX_THREAD_GROUP_SUPPORT
	/* Don't touch threads which are member of an active thread group. */
	if (RT_QUOTA_IS_ON(current))
		return 0;
#endif

	// A RT-thread which migrated to LX should get a chance to proceed,
	// if the priority of the current RT-thread is below the priority of the
	// migrated thread.
	// Note: The prio value is inverse to its significance, e.g. prio=31
	// means priority 0 (the lowest RT priority) and prio=0 means
	// priority 31 (the highest RT priority).
	pmq = &__raw_get_cpu_var(rt_migqueue);
	if (pmq->bitmapPrio && current->prio >= __ffs(pmq->bitmapPrio))
		return 1;
	return 0;
}

/*
 * To avoid priority inversion, we sleep a small amount of time.
 */
asmlinkage long sys_rt_sched_yield(void)
{
	if (rt_sched_has_delay()) {
		if (rtx_get_cpu_var(rt_yield_count) < YIELD_MAX_DELAY_OF_SLEEP)
			rtx_add_cpu_var(rt_yield_count, 1);
	}
	else
		/* no rt thread does some intermediate work in lx */
		/* clear the yield count                          */
		rtx_set_cpu_var(rt_yield_count, 0);

	if (rtx_get_cpu_var(rt_yield_count) > 0) {
		__set_current_rt_state(RT_TASK_UNINTERRUPTIBLE);
		rt_schedule_timeout(&__raw_get_cpu_var(isr_list_hdr),
				rtx_get_cpu_var(rt_yield_count) * SLEEP_TIME_YIELD);
	}
	else {
		rt_sched_rr();
		rt_schedule(0);
	}
	return (0);	
}

/*
 * Some info about debugging of realtime threads
 * manfred.neugebauer@siemens.com
 * 
 * General idea: when hitting a breakpoint we will migrate
 * the realtime threads to Linux as LXRT threads using a special
 * RT_TASK_DEBUG_PENDING flag. All debugging stuff will be done 
 * in Linux. When the process continues, we will migrate the 
 * realtime threads back to the realtime domain.
 *
 * There are different ways to initiate a debugging session:
 * attach to a process: this will be done using a call to sys_ptrace()
 * Within ptrace_attach() (in kernel/ptrace.c) the corresponding task
 * gets a SIGSTOP signal
 * (the gdbserver calls this function for every task within the process).
 * Additionally, all realtime tasks of this process are marked for debugging.
 * They will leave the realtime domain and are capable to be debugged.
 *
 * A different task of the process hits a breakpoint: all other tasks
 * get a kill call with signal SIGSTOP (see kernel/signal.c - do_tkill).
 * realtime tasks will also be marked for debugging and leave the 
 * realtime domain.
 *
 * Remark: leaving the realtime domain will be achieved in the following way:
 * in rt_schedule() within this source file we test the debug flag and migrate
 * to linux. Then we fall back to the calling system call - realtime path -
 * and do the normal Linux system call end handling which results in the 
 * handling for the SIGSTOP exception.
 * when we resume (normally restarting the stopped system call), we test
 * the debug flag of this (currently in Linux context running) task and
 * migrate back to realtime. Subsequently the system call now to be executed
 * will be done within the realtime domain.
 * T B D: here we have a small hole: this code will be executed within the
 * Linux context and not - as it normally would be - in the realtime context.
 * We may change this by migrating when leaving the signal handling code.
 * 
 * Hit a breakpoint within a realtime task: We execute the realtime 
 * exception handler and migrate to Linux (also marked for debugging).
 * (see arch/.../rtx_adapt.c). 
 * After returning from this exception (see architecture specific 
 * assembler routine (in arch/kernel/xxx.S) we migrate back to realtime
 */
/* called from linux domain */

#ifdef CONFIG_RTX_EXTENDED_RT_DEBUGGING_SUPPORT
int rtx_testStopAllRtThreadsFromLx(struct task_struct *markedThread);
int rtx_doStopAllRtThreadsFromLx(struct task_struct *markedThread,
	int fromLxThreadLevel);
#endif

#ifdef CONFIG_SMP
/* Now we are on the right CPU. */
void smp_wakeup_rt_thread(void *cmd)
{
	struct task_struct *p = (struct task_struct *)cmd;

	local_irq_disable_hw();
	lx_wake_up_rt_thread(p);
	local_irq_enable_hw();
}

// this runs on lx interrupt level on rt cpu
static void smp_mark_debug_pending(void *cmd)
{
	struct task_struct *p = (struct task_struct *)cmd;
	int ret = 1;

	local_irq_disable_hw();
#ifdef CONFIG_RTX_EXTENDED_RT_DEBUGGING_SUPPORT
	ret = rtx_doStopAllRtThreadsFromLx(p, 0);
#endif
	p->rt_state3 |= RT_TASK_LX_DEBUG_PENDING;
	if (ret)
		lx_wake_up_rt_thread(p);
	local_irq_enable_hw();
}
#endif

void rtx_mark_debug_pending(struct task_struct *p)
{
	int ret = 1;

	if ((RT_DEBUG_VERBOSE) && p->rt_state)
		printkGen(NULL, "rtx_mark_debug_pending of %s(%d) state=%#x:%#x\n",
			p->comm, p->pid, p->rt_state, p->rt_state3);
	// Do not handle LX threads and RT kernel threads.
	if (!p->rt_state || rtx_is_rt_kernel_thread(p, p->rt_proc->cpu))
		return;
	if (p->rt_state3 & RT_TASK_LX_DEBUG_PENDING)
		return; // already done; why do we get this twice??
 
	local_irq_disable_hw();
	if (THREAD_IS_REALTIME(p)) {
#ifdef CONFIG_RTX_EXTENDED_RT_DEBUGGING_SUPPORT
		ret = rtx_testStopAllRtThreadsFromLx(p);
#endif
#ifdef CONFIG_SMP
		/* Check if we are running on the right CPU. */
		if (p->rt_proc->cpu == rtx_processor_id()) {
#endif
#ifdef CONFIG_RTX_EXTENDED_RT_DEBUGGING_SUPPORT
			ret = rtx_doStopAllRtThreadsFromLx(p, 1);
#endif
			p->rt_state3 |= RT_TASK_LX_DEBUG_PENDING;
			if (ret)
				lx_wake_up_rt_thread(p);
#ifdef CONFIG_SMP
		}
		else {
			/* Delegate execution to the right CPU  */
			/* don't wait within this call          */
			/* for completion, see below            */
			local_irq_enable_hw();
			smp_call_function_single(p->rt_proc->cpu,
				smp_mark_debug_pending, (void *)p, 0);
			/* wait here until we see the result               */
			/* limit the loop: we may starve: we may have some */
			/* locks which prevent the ipi irq code to run     */
                        if (!(p->rt_state3 & RT_TASK_LX_DEBUG_PENDING)) {
				unsigned maxLoop = 10000;
				while (--maxLoop) {
					if (p->rt_state3 & RT_TASK_LX_DEBUG_PENDING) {
						break;
					}
				}
			}
			return;
		}
#endif
	}
	local_irq_enable_hw();
}
EXPORT_SYMBOL(rtx_mark_debug_pending);

/* Check for a kernel thread of the RT process. */
int rtx_is_rt_kernel_thread(struct task_struct *p, int cpu)
{
	struct rt_proc_hdr *pproc = p->rt_proc;
	struct task_struct *helpTask;
    
	if (!p->rt_state || pproc == NULL)
		return 0;
	helpTask = rtx_per_cpu(rt_timer_daemon, cpu);
	if (helpTask == p)
		return 1;
	if (pproc->rt_check_daemon == p)
		return 1;
	helpTask = rtx_per_cpu(interdomain_daemon, cpu);
	if (helpTask == p)
		return 1;
	if (pproc->write_daemon == p)
		return 1;
#ifdef CONFIG_LTT
	if (aud_rt_hdr.ltt_daemon == p)
		return 1;
#endif
	return 0;
}

/*
 * Only a thread itself can migrate to Linux. In case of a shutdown:
 * - It gets called with p == NULL.
 * - It may get called twice if exit() was called in the realtime domain. To force shutdown
 *   in this case all threads must get marked before re-executing exit() in the Linux domain.
 */
void rt_mark_exit_pending(struct task_struct *p)	
{
	struct list_head *lh;
	struct task_struct *t;
	struct list_head *prt_tl;
#ifdef CONFIG_RTX_THREAD_GROUP_SUPPORT
	rt_tgroup_head_t *pqh = rtx_get_cpu_var(pquota_head);
#endif

	local_irq_disable_hw(); 	
	if (p) {
		rt_delete_event_notification(p);
		p->rt_state |= RT_TASK_EXIT_PENDING;
#ifdef CONFIG_RTX_THREAD_GROUP_SUPPORT
		/* Remove thread from thread group. */
		if (rt_remove_thread_from_group(p))
			rt_schedule(0);			// enables HW interrupts
#endif
	}
	else										// prepare RT domain for shutting down
	{
		rtx_or_cpu_var(rt_system_state, RT_SHUTDOWN);
#ifdef CONFIG_RTX_THREAD_GROUP_SUPPORT
		/* Deactivate quota handling and move all the threads
		 * from quota (shadow) list to run queue. */
		if (pqh->quota_enabled) {
			quota_interval_stop(pqh);
			rt_shift_all_threads_to_runq(rtx_get_cpu_var(prt_blocked_queue), NULL, RT_QUOTA_FORCE_ALL);
		}
#endif
		prt_tl = &__raw_get_cpu_var(rt_tl);
		for (lh = prt_tl->next; lh != prt_tl; lh = lh->next)
		{
			t = list_entry(lh, struct task_struct, rt_pid_list);
			if (!(t->rt_state2 & RT_TASK_IN_EXIT))
				rt_delete_event_notification(t);
		}
		for (lh = prt_tl->next; lh != prt_tl; )
		{
			t = list_entry(lh, struct task_struct, rt_pid_list);
			lh = lh->next;

			if (t->rt_state == RTLX_TASK) {
				t->rt_state = LXRT_TASK;
				list_del_init(&t->rt_pid_list);
			}
			else {
				if (t != current)
				{	
					if (!(t->rt_state2 & RT_TASK_IN_EXIT)) {
						t->rt_state |= RT_TASK_EXIT_PENDING;				
						if (THREAD_IS_REALTIME(t))
								rt_wake_up_thread(t);
					}
				}
			}
		}
	}
	local_irq_enable_hw();
}

/* 
 * Check whether a sched_setscheduler()in the LX domain has affected a thread of the RT process.
 * Before calling this routine via the hook, LX has already changed the scheduling parameters.
 * Since the thread is already out of the LX runqueue, no further action was taken in LX code,
 * but now there may be a thread either in the RT thread list which is no longer eligible for
 * executing in the RT domain or a LX thread which has to migrate to the RT domain at system call exit.
 */
#ifdef CONFIG_SMP
static void smp_rtx_sched_change(void *cmd)
{
	struct smp_sched_change *pinfo = (struct smp_sched_change *)cmd;
	/* We are running on RT-CPU */
	pinfo->ret = rtx_sched_change(pinfo->p, pinfo->policy, pinfo->prio);
}
#endif

/*
 * This function may be called from two different context scenarios:
 * - From __sched_setscheduler()
 * - From LX ISR context (call function IPI)
 */
int rtx_sched_change(struct task_struct *p, int policy, int prio)
{
	int ret;
#ifdef CONFIG_SMP
	struct smp_sched_change info;

	if (rtx_processor_id() != p->rt_proc->cpu) {
		info.p = p;
		info.policy = policy;
		info.prio = prio;
	    smp_call_function_single(p->rt_proc->cpu, smp_rtx_sched_change, (void *)&info, 1);
		return info.ret;
	}
#endif
	local_irq_disable_hw();

	if (rtx_is_rt_capable(policy, prio)) {
		// We have to check whether p is currently
		// executing in LX. In this case we must not change
		// the priority/policy. It must be deferred until
		// the thread migrates back to the RT-domain.
		// In this case we may have changed the policy from
		// SCHED_FIFO to SCHED_RR, so we must start the rr-timer
		// if not already done. But this may only be accomplished
		// in the RT-domain.
		if (p->rt_state == RTLX_TASK) {
			p->rt_dyn_prio = -prio;						// negative value
			p->rt_dyn_policy = policy; 					// SCHED_FIFO or SCHED_RR
			return 1;
		}
		p->policy = policy;
		p->sched_class = rtx_get_rt_sched_class();

		p->rt_priority = prio;
		p->normal_prio = (MAX_RT_PRIO-1) - p->rt_priority;
		/* We are holding p->pi_lock already. */
		p->prio = p->normal_prio;
	}
	/* If p is already in RT, then rtx_set_rt_state() returns immediately. */
	ret = rtx_set_rt_state(p);
	local_irq_enable_hw();
	return  ret;
}

/*
 * This function prevents a RT-thread source from being blocked
 * on wait_mig_sem, when the RT-thread which is currently running
 * in the LX-domain is migrating back to the RT-domain.
 * The target thread remains in the LX-domain and therefore there
 * is no necessity to synchronize this sequence.
 */
void rtx_check_migration_source(void)
{
	if (current == rtx_get_cpu_member(rt_sync_migration, source))
		rtx_set_cpu_member(rt_sync_migration, target, CANCEL_MIGRATION);
}
EXPORT_SYMBOL(rtx_check_migration_source);

#ifdef CONFIG_RTX_RETURN_TO_LINUX
/*
 * The current thread voluntarily gives control to Linux.
 * - flag: return to LX, if
 * 	 R2L_FORCE: in any case
 * 	 R2L_THREAD: there is no pending signal for the current thread
 * 	 R2L_PROCESS: there is no pending signal for the RT-process
 */
int rt_sched_return_to_lx(int flag)
{
	struct list_head *lh;
	struct list_head *prt_tl;
	struct task_struct *p;
	int no_pending_sig = 1;
	int ret = R2L_RET_SIG_WAIT_INTERRUPTED;

	if ((flag != R2L_FORCE) && (flag != R2L_THREAD) && (flag != R2L_PROCESS))
		return -EINVAL;

	if (!IS_REALTIME)
		return -EPERM;

	local_irq_disable_hw();
	if (flag != R2L_FORCE) {
		/* Check for pending signals (current thread or RT-process). */
		if (flag == R2L_PROCESS) {
			prt_tl = &__raw_get_cpu_var(rt_tl);
			for (lh = prt_tl->next; lh != prt_tl; lh = lh->next)
			{
				p = list_entry(lh, struct task_struct, rt_pid_list);
				no_pending_sig = list_empty(&p->rt_sigqueue_head);
				if (!no_pending_sig) {
					if (p == current) {
						ret = R2L_RET_SIG_PENDING_THREAD | R2L_RET_SIG_PENDING_PROCESS;
						break;
					}
					else
						ret = R2L_RET_SIG_PENDING_PROCESS;
				}
			}
		}
		else { /* R2L_THREAD */
			no_pending_sig = list_empty(&current->rt_sigqueue_head);
			if (!no_pending_sig) {
				ret = R2L_RET_SIG_PENDING_THREAD;
			}
		}
	}
	
	/*
	 * If there is no pending signal for this thread/process then we return control to LX,
	 * waiting to be continued in the RT-domain by some signal or RT interrupt.
	 */
	if (likely(no_pending_sig)) {
		rtx_set_cpu_var(rt_req_task, rtx_get_cpu_var(lx_curr_task));
		current->rt_state2 |= RT_TASK_RETURNED_TO_LX;
		current->rt_proc->r2l = (RT_SIG_WAIT_R2L | RT_SIG_WAIT_R2L_IN_PROGRESS);
		rt_schedule(0);
		local_irq_disable_hw();
		if (current->rt_state2 & RT_TASK_SIGNAL_RECEIVED) {
			if (flag == R2L_THREAD)
				ret = R2L_RET_SIG_WAIT_THREAD;
			else
				ret = (R2L_RET_SIG_WAIT_THREAD | R2L_RET_SIG_WAIT_PROCESS);
		}
		if (!(current->rt_proc->r2l & RT_SIG_WAIT_R2L))
			ret = R2L_RET_SIG_WAIT_PROCESS;
		current->rt_state2 &= ~(RT_TASK_RETURNED_TO_LX | RT_TASK_SIGNAL_RECEIVED);
		current->rt_proc->r2l &= ~(RT_SIG_WAIT_R2L | RT_SIG_WAIT_R2L_IN_PROGRESS);
	}
	local_irq_enable_hw();
	return ret;
}
#endif

#ifdef CONFIG_RTX_THREAD_GROUP_SUPPORT
/*
 * Shift a single thread from the blocked queue to the run queue.
 * Note: Interrupts must be off.
 */
int rt_shift_single_thread_to_runq(struct list_head *head, struct task_struct *p)
{
	int idx;
	struct list_head *lp;
	struct task_struct *proc;
	runqueue_t *prq = rtx_get_cpu_var(prt_runqueue);

	p->rt_state2 &= ~(RT_TASK_QUOTA_EXCEEDED | RT_DEFER_QUOTA_HANDLING);

	lp = head->next;
	while (lp != head)
	{
		proc = (struct task_struct *)list_entry(lp, struct task_struct, rt_blocked_list );
		lp = lp->next;
		if (p == proc) {
			list_del_init(&p->rt_blocked_list);		// dequeue from blocked queue
#ifndef CONFIG_RTX_OPTIMIZE_FOR_RELEASE
			BUG_ON(p->prio >= MAX_NO_OF_RT_PRIO);
#endif
			idx = p->prio;
			list_add_tail(&p->rt_run_list, &prq->queue[idx].qhead);
			__set_bit(idx, &prq->bitmapPrio);
			rtx_set_cpu_var(rt_req_task, rt_get_next_thread(prq));
			return 1;
		}
	}
	return 0;
}

/*
 * Shift all RT-threads from the blocked queue to the run queue.
 * Note: Interrupts must be off.
 */
int rt_shift_all_threads_to_runq(struct list_head *head, rt_tgroup_t *ptg, int flag)
{
	int idx;
	struct list_head *lp;
	struct task_struct *proc;
	runqueue_t *prq = rtx_get_cpu_var(prt_runqueue);
	int threads_shifted = 0;

	/* The head points to the blocked queue. */
	lp = head->next;
	while(lp != head)
	{
		proc = (struct task_struct *)list_entry(lp, struct task_struct, rt_blocked_list );
		lp = lp->next;

		/* Don't touch threads of another thread group. */
		if (ptg && proc->rt_thread_group != ptg)
			continue;

		/* Don't shift a thread to the run queue when
		 * the quota of the thread group it belongs to is blocked.
		 * That case has only to be considered when the quota
		 * assignment interrupt occurs. */
		if (!proc->rt_thread_group->quota && (flag == RT_QUOTA_KEEP_ZERO_BLOCKED))
			continue;

		list_del_init(&proc->rt_blocked_list);		// unchain from blocked queue

		idx = proc->prio;
		list_add_tail(&proc->rt_run_list, &prq->queue[idx].qhead);
		__set_bit(idx, &prq->bitmapPrio);
		threads_shifted++;
	}
	rt_reset_exceeded_state(ptg, flag);

	/* Select a runnable thread with the highest priority for rtx_preemption_handling(). */
	if (threads_shifted)
		rtx_set_cpu_var(rt_req_task, rt_get_next_thread(prq));
	return threads_shifted;
}

/*
 * Shift all threads of the thread group from the run queue
 * to the blocked queue.
 * Note: Interrupts must be off.
 */
int rt_shift_all_threads_to_blockedq(struct list_head *head, struct task_struct *p)
{
	struct list_head *lp;
	struct task_struct *proc;
	runqueue_t *prq = rtx_get_cpu_var(prt_runqueue);
	int threads_shifted = 0;

	/* The head points to the thread list of the thread group. */
	lp = head->next;
	while(lp != head)
	{
		proc = (struct task_struct *)list_entry(lp, struct task_struct, rt_thread_list );
		lp = lp->next;

		if (p && p == proc)
			continue;

		/* Note: A RT-thread may be waiting for some event and therefore
		 * it is not in the run queue or the thread is temporarily in LX. 
		 * We only set the EXCEEDED flag. */
		proc->rt_state2 |= RT_TASK_QUOTA_EXCEEDED;

		/* Assumption is proc != current */
		if (proc->rt_state & RT_TASK_RUNNING) {
			/* Dequeue the thread from run queue and
			 * add it to the blocked queue.
			 */
			if (!list_empty(&proc->rt_run_list)) {
				rt_dequeue_thread(rtx_get_cpu_var(prt_runqueue), proc);
				threads_shifted++;
			}
		}
	}
	/* Select a runnable thread with the highest priority for rtx_preemption_handling(). */
	if (threads_shifted) {
		if (prq->bitmapPrio)
			rtx_set_cpu_var(rt_req_task, rt_get_next_thread(prq));
		else
			rtx_set_cpu_var(rt_req_task, rtx_get_cpu_var(lx_curr_task));
		return 1;
	}
	return 0;
}

/* Dequeue the current thread from runq and get the next thread which is runnable. */
void rt_dequeue_get_next_thread(void)
{
	int ii;
	struct list_head *queue;
	runqueue_t *prq = rtx_get_cpu_var(prt_runqueue);

	rt_dequeue_thread(prq, current);

	if (prq->bitmapPrio) {	
		ii = __ffs(prq->bitmapPrio);	// search for the first bit, starting at top
		queue = prq->queue[ii].qhead.next;
		rtx_set_cpu_var(rt_req_task, (struct task_struct *)list_entry(queue, struct task_struct, rt_run_list));
	}
	else {
		rtx_set_cpu_var(rt_req_task, rtx_get_cpu_var(lx_curr_task));
	}
	return;
}
#endif

