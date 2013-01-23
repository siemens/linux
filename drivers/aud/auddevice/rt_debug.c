/*
 * drivers/aud/auddevice/rt_debug.c
 *
 * Copyright (C) 2012 Siemens AG
 * Contributed by manfred.neugebauer@siemens.com, 2012.
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
 * This file contains the functions to support extended debugging for
 * realtime threads (immediate stop of all threads).
 * Additionally there are functions available to support a user watchdog
 * handling.
 * 
 */

#include <linux/aud/rt_timer.h>
#include <linux/aud/rt_debug.h>

#ifdef CONFIG_RTX_EXTENDED_RT_DEBUGGING_SUPPORT

static int contAllRtThreads(struct task_struct *activationThread);
static int __stopAllRtThreads(struct task_struct *breakpointThread,
	unsigned action);

DECLARE_PER_CPU(struct list_head, rt_tl);

static struct task_struct *activeDebugThread = NULL;
static struct task_struct *activeBreakThread = NULL;
static struct task_struct *activeWatchdogThread = NULL;

static unsigned countOfRtLxThreads, prevCountOfRtLxThreads;
static unsigned waitForRealtimeContAfterPostcode;
static unsigned activeDebugThreadState;
static unsigned breakWasInLxFlag, newBreakWasInLxFlag;
static unsigned activeBreakInRtCount;
static unsigned activeBreakToRtCount;
static int watchdogThreadInDebug;
static int runWithGdbFlag;
static int watchdogInOperation;


#define MAX_WAIT_LOOPS			100
#define DEBUG_POLL_TIME			(10 * HZ) 

#define DEBUG_STOP_STOP_ACTION 		0x1
#define DEBUG_STOP_WAKE_ACTION		0x2

#define WATCHDOG_FLAG_DEBUG		0x1
#define WATCHDOG_FLAG_SHUTDOWN		0x2

/*
 * general information:
 * 
 * we may need to move the threads to a separate support runqueue
 * (e.g., in the case that rt threads run on several cpus)
 * currently we assume that the debug thread runs with highest
 * rt priority.
 */
int rt_debug_action(int cmd, unsigned long arg)
{
	unsigned long flags;
	int retval = 0;
	unsigned debugStateFlag;

	if (RT_DEBUG_VERBOSE)
		printkGen(NULL, "debug_action cmd=%#x\n", cmd);

	switch(cmd) {
	case AuD_DEV_ATTACH_DEBUG:
		if (activeDebugThread) {
			return(-EBUSY);
		}
		if (IS_LINUX) {
			return(-EINVAL);
		}
		activeDebugThread = current;
		activeDebugThreadState = 0;
		break;

	case AuD_DEV_DETACH_DEBUG:
		if (activeDebugThread == NULL) {
			return(-EINVAL);
		}
		if (activeDebugThread != current) {
			return(-EBUSY);
		}
		if (IS_LINUX) {
			return(-EINVAL);
		}
		activeDebugThread = NULL;
		activeDebugThreadState = 0;
		runWithGdbFlag = 0;
		break;

	case AuD_DEV_WAIT_FOR_DEBUG_EVENT:
		if (activeDebugThread == NULL) {
			return(-EINVAL);
		}
		if (activeDebugThread != current) {
			return(-EBUSY);
		}
		if (IS_LINUX) {
			return(-EINVAL);
		}

		if (waitForRealtimeContAfterPostcode) {
			contAllRtThreads(current);
			waitForRealtimeContAfterPostcode = 0;
		}
		activeDebugThreadState = 10;

		__set_current_rt_state(RT_TASK_UNINTERRUPTIBLE);
		rt_schedule(0);
		activeDebugThreadState = 14;

		if (RT_DEBUG_VERBOSE)
			printkGen(NULL, "returned from wait_for_debug(%#x)\n",
                                  current->rt_state);
		// wakeup of sleeping rt threads now done earlier
		// when stopping these threads
		//__stopAllRtThreads(NULL, DEBUG_STOP_WAKE_ACTION);
		break;

	case AuD_DEV_HANDLE_DEBUG_EVENT:

		if (activeDebugThread == NULL) {
			return(-EINVAL);
		}
		if (activeDebugThread != current) {
			return(-EBUSY);
		}
		if (IS_LINUX) {
			return(-EINVAL);
		}

		if (rt_copy_from_user(&debugStateFlag, (char *) arg,
				sizeof(unsigned)))
			return (-EFAULT);

		if (!debugStateFlag) {
			// pre-code was done by the debug thread
			// transfer work to gdb for debug analysis
			debugStateFlag = 1;
			if (rt_copy_to_user((unsigned *)arg, &debugStateFlag,
					sizeof(unsigned)))
				return (-EFAULT);

			activeDebugThreadState = 20;
                        // wait to be continued by gdbserver
			__set_current_rt_state(RT_TASK_UNINTERRUPTIBLE);
			rt_schedule(0);
			if (current->rt_state3 & RT_TASK_LX_DEBUG_PENDING) {
				//printkGen(NULL, "move to lx debugging\n");
				local_irq_save_hw(flags);
				activeDebugThreadState = 30;
				local_irq_restore_hw(flags);
				// redo this system call
				return(-ERESTARTNOINTR);
			}
			printkGen(NULL, "problem dbgthrd moving to lx\n");
			return(-EINVAL);
 		}
		else if (debugStateFlag == 1) {
			// back again
			// gdb wants to continue the application
			// return to debug thread to do post-code
			//printkGen(NULL, "returning from lx debugging\n");
			debugStateFlag = 0;
			activeDebugThreadState = 40;
			if (rt_copy_to_user((unsigned *)arg, &debugStateFlag,
					sizeof(unsigned)))
				return (-EFAULT);
 		}
		else {
			printkGen(NULL, "problem debugStateFlag(%d)\n",
				debugStateFlag);
			return(-EINVAL);
		}

		break;

	default:
		return(-EINVAL);
	}

	return(retval);
}

#endif

#ifdef CONFIG_RTX_RT_WATCHDOG_SUPPORT

int rt_watchdog_action(int cmd, unsigned long arg)
{
	unsigned long flags;
	int helpInfo;

	if (RT_DEBUG_VERBOSE)
		printkGen(NULL, "watchdog_action cmd=%#x\n", cmd);

	if (!watchdogInOperation) {
		// we need an initialization information whether we 
		// run within the context of a debugger 
		watchdogInOperation = 1;
		runWithGdbFlag = current->ptrace;
		if (RT_DEBUG_VERBOSE) {
			printkGen(NULL, "watch ptrace=%#x\n", runWithGdbFlag);
		}
	}

	switch(cmd) {
	case AuD_DEV_TEST_WATCHDOG_EVENT:
		// here we may add code to handle serial line hot keys 
		return(0);

	case AuD_DEV_TEST_WATCHDOG_IN_DEBUG:
		// we want to know whether we had a debug break before
		// in this case we don't do a watchdog break
		activeWatchdogThread = current;
		helpInfo = watchdogThreadInDebug;
		if (rtx_get_cpu_var(rt_system_state)
			& (RT_SHUTDOWN | RT_SHUTDOWN_PROCESS)) {
			helpInfo |= WATCHDOG_FLAG_SHUTDOWN;
		}
		watchdogThreadInDebug = 0;
		return(helpInfo);

	case AuD_DEV_FORCE_USER_BREAK:
		printkGen(NULL, "enter force user watchdog break\n");
		if (runWithGdbFlag) {
			// we run within a gdb
			// we can do a normal user breakpoint
			// or signal action
			return(-EAGAIN);
		}

		local_irq_save_hw(flags);
		rtx_stopAllRtThreads(current);
                current->rt_state3 |= RT_TASK_RT_DEBUG_PENDING;
                __set_current_rt_state(RT_TASK_UNINTERRUPTIBLE);
		local_irq_restore_hw(flags);
		// we wait for the attach of the gdbserver
		rt_schedule(0);
		//printkGen(NULL, "leave force user break(rt_state3=%#x)\n",
		//	current->rt_state3);
		return(0);

	default:
		return(-EINVAL);
    }
}

int forceUserBreakFromKernel(void)
{
	unsigned long flags;
    
	//if (RT_DEBUG_VERBOSE)
		printkGen(NULL, "forceUserBreakFromKernel\n");
	local_irq_save_hw(flags);
	rtx_stopAllRtThreads(current);
        current->rt_state3 |= RT_TASK_RT_DEBUG_PENDING;
        __set_current_rt_state(RT_TASK_UNINTERRUPTIBLE);
	local_irq_restore_hw(flags);
	// we wait for the attach of the gdbserver
	rt_schedule(0);
	printkGen(NULL, "leave force user break(rt_state3=%#x)\n",
		current->rt_state3);
	return(0);
}

#endif

#ifdef CONFIG_RTX_EXTENDED_RT_DEBUGGING_SUPPORT

static int setRtDebugPending(struct task_struct *task)
{
	task->rt_state3 |= RT_TASK_RT_DEBUG_PENDING;

	return(0);
}

// called with interrupts disabled
static int __stopAllRtThreads(struct task_struct *breakpointThread,
	unsigned action)
{
	struct task_struct *proc = NULL;
	struct list_head *lp, *head;

	head = &__raw_get_cpu_var(rt_tl);
	lp = head->next;
	while(lp != head)
	{
		proc = (struct task_struct *)list_entry(lp,
			struct task_struct, rt_pid_list );
		// step over debug thread
		if (proc == activeDebugThread) {
			countOfRtLxThreads++;
			goto contRtTaskList;
		}
		// step over breakpoint thread
		if (proc == breakpointThread) {
			countOfRtLxThreads++;
			goto contRtTaskList;
		}
		if (rtx_is_rt_kernel_thread(proc, proc->rt_proc->cpu)) {
			goto contRtTaskList;
		}
		if (list_empty(&proc->rt_pid_list)) {
			goto contRtTaskList;
		}
		if (action & DEBUG_STOP_STOP_ACTION) {
			setRtDebugPending(proc);
			countOfRtLxThreads++;
			if (RT_DEBUG_VERBOSE)
				printkGen(NULL, "stop_stop RtThrd %s(%d)(%#x:%#x)\n",
					proc->comm, proc->pid, proc->rt_state,
					proc->rt_state3);
		}
		if (action & DEBUG_STOP_WAKE_ACTION) {
			if (RT_DEBUG_VERBOSE)
				printkGen(NULL, "stop_wake RtThrd %s(%d)(%#x:%#x)\n",
					proc->comm, proc->pid, proc->rt_state,
					proc->rt_state3);
			if (proc->rt_state & RT_TASK_UNINTERRUPTIBLE) {
				rt_wake_up_thread(proc);
			}
		}
contRtTaskList:
		lp = lp->next;
	}
        prevCountOfRtLxThreads = countOfRtLxThreads;

	return(0);
}

// called with hard interrupts disabled
int contAllRtThreads(struct task_struct *activationThread)
{
	struct task_struct *proc = NULL;
	struct list_head *lp, *head;

	head = &__raw_get_cpu_var(rt_tl);
	lp = head->next;
	while(lp != head)
	{
		proc = (struct task_struct *)list_entry(lp,
			struct task_struct, rt_pid_list );
		// step over activation thread
		if (proc == activationThread) {
			goto contRtTaskList;
		}
		if (rtx_is_rt_kernel_thread(proc, proc->rt_proc->cpu)) {
			goto contRtTaskList;
		}
		if (list_empty(&proc->rt_pid_list)) {
			goto contRtTaskList;
		}
		if (THREAD_IS_REALTIME(proc)) {
			if (RT_DEBUG_VERBOSE)
				printkGen(NULL, "cont RtThrd %s(%d)(%#x:%#x)\n",
					proc->comm, proc->pid, proc->rt_state,
					proc->rt_state3);
			rt_wake_up_thread(proc);
		}
		else {
			if (RT_DEBUG_VERBOSE)
				printkGen(NULL, "problem cont RtThrd %s(%d)(%#x:%#x)\n",
					proc->comm, proc->pid, proc->rt_state,
					proc->rt_state3);
		}
contRtTaskList:
		lp = lp->next;
	}

	return(0);
}

int rtx_stopAllRtThreads(struct task_struct *breakpointThread)
{
	unsigned long flags;

	local_irq_save_hw(flags);

        if (breakpointThread->rt_state3 & RT_TASK_RT_DEBUG_PENDING) {
		local_irq_restore_hw(flags);
		printkGen(NULL, "stopAllRtThreads: already transfering to debug\n");
		return(0);
	}

	activeBreakThread = breakpointThread;
        activeBreakInRtCount++;
	if (activeDebugThread) {
		// work with debug thread 
		if (activeDebugThreadState == 10) {
			// debug thread waits for a breakpoint event
			countOfRtLxThreads = 0;
			waitForRealtimeContAfterPostcode = 0;

			__stopAllRtThreads(breakpointThread, DEBUG_STOP_STOP_ACTION
				| DEBUG_STOP_WAKE_ACTION);

			// debug thread: let him do some pre work
			rt_wake_up_thread(activeDebugThread);
			rt_schedule(0);
		}
		else {
			// debug thread or other rt threads were caught by
			// lx breakpoint; the return_to_rt code must resolve
			// this issue
			// use the old count value for the next return from lx
			countOfRtLxThreads = prevCountOfRtLxThreads;
			if (RT_DEBUG_VERBOSE)
				printkGen(NULL, "rtx_stopAllRt: different debugState=%d cnt=%d\n",
					activeDebugThreadState, countOfRtLxThreads);
		}
	}
	else {
		// running without a debug thread
		countOfRtLxThreads = 0;
		waitForRealtimeContAfterPostcode = 0;

		__stopAllRtThreads(breakpointThread, DEBUG_STOP_STOP_ACTION
			| DEBUG_STOP_WAKE_ACTION);

		// standard case: no debug thread:
		// schedule realtime to let the threads run to the lx migration point
		// migrate the rt threads to linux
		rt_schedule(0);
	}
        
	local_irq_restore_hw(flags);

	return(0);
}

// previously we wanted to test here whether the debug thread is in the correct
// state to start the next debug cycle
// however, a preempt kernel doesn't like to be scheduled in this situation
// so we need to do the special considerations later when the rt threads
// come back from lx (see later)
int rtx_testStopAllRtThreadsFromLx(struct task_struct *markedThread)
{
	unsigned long flags;
	int retState = 0;
	unsigned activeBreakStartCount;
	unsigned maxScheduleLoop = 800000;

	local_irq_save_hw(flags);
	activeBreakStartCount = activeBreakInRtCount;
	local_irq_restore_hw(flags);

	//if (RT_DEBUG_VERBOSE)
	//      printkGen(NULL, "enter testStop_from_lx\n");

	while (--maxScheduleLoop) {
		// assume gdbserver running on Linux prio 0 (SCHED_OTHER)
		// therefore schedule() is fine to get other tasks running
		// but a preempt kernel doesn't like schedule at this level
		// so we resolve this issue when the rt threads return from lx
		// (see later)
		if (activeDebugThreadState) {
			// working with debug thread
			// previous debug cycle done?
			if ((activeDebugThreadState < 33)
					|| (activeDebugThreadState > 34)) {
				break;
			}
			retState = -1;
			break;
		}
		else {  // no debug thread
			// no breakpoint in rt so move ahead
			if (!activeBreakThread 
			// realtime may already hit the next breakpoint: move ahead
			// else wait
					|| (activeBreakStartCount != activeBreakToRtCount)) {
				break;
			}
			retState = -1;
			break;
		}
	}
	if (!maxScheduleLoop) {
		// we hit a linux breakpoint in the meantime
		// so we will not see some threads back in rt
		// (especially the last breakpoint thread)
		// we need to continue the remaining threads
		// especially the controlling debug thread

		// we don't count down now
		// the special cases are done when the rt threads return
		// see below
		retState = 1;
	}

	if (RT_DEBUG_VERBOSE)
		printkGen(NULL, "leave testStop_from_lx(cnt=%d loop=%d ret=%d)\n",
			countOfRtLxThreads, maxScheduleLoop, retState);
	return(retState);
}

// we may stop all rt threads when a lx breakpoint is hit and the debug thread is
// waiting for this (activeDebugThreadState is 10).
// multicore: we must run on the rt cpu
// fromLxThreadLevel: this flag helps to differentiate between a call from thread and
// interrupt context (currently no longer used)
int rtx_doStopAllRtThreadsFromLx(struct task_struct *markedThread, int fromLxThreadLevel)
{
	unsigned long flags;
	int retVal = 0;

	if (RT_DEBUG_VERBOSE)
		printkGen(NULL, "enter doStop_from_lx(dbg=%d)\n", activeDebugThreadState);

        if (activeDebugThreadState) {
		// work with debug thread 
		if (activeDebugThreadState == 10) {
			// debug thread waits for a breakpoint event
			// and we have a lx breakpoint: stop rt threads
			local_irq_save_hw(flags);
			countOfRtLxThreads = 0;
			__stopAllRtThreads(NULL, DEBUG_STOP_STOP_ACTION
				| DEBUG_STOP_WAKE_ACTION);
			breakWasInLxFlag = 1;
			if (activeDebugThread) { // be sure it's here
				// debug thread: let him do some pre work
				rt_wake_up_thread(activeDebugThread);
			}
			local_irq_restore_hw(flags);
			// in the case rt was not yet woken up, do it now
			//if (fromLxThreadLevel)
			lx_wake_up_rt_thread(NULL);
		}
		else if (activeDebugThreadState == 34) {
			// all rt threads are on the way migrating back to rt
			// but will immediately go back to lx
			// caught by next lx debug event
			local_irq_save_hw(flags);
			// use the old count for the next debug step
			countOfRtLxThreads = prevCountOfRtLxThreads;
			newBreakWasInLxFlag = 1;
			local_irq_restore_hw(flags);
		}
		else {
		}

		if (markedThread->rt_state3 & RT_TASK_RT_DEBUG_PENDING)
			// this thread was stopped within rt
			// set ret value to continue its move to lx
			retVal = 1;
		else if (markedThread == activeDebugThread) {
			// the debug thread also waits for the next step
			// set ret value to continue its move to lx
			if (activeDebugThreadState == 20)
				retVal = 1;  // ready to move to lx
		}
	}
	else {
		local_irq_save_hw(flags);
		if (!activeBreakThread && !breakWasInLxFlag) {
			countOfRtLxThreads = 0;
        		__stopAllRtThreads(NULL, DEBUG_STOP_STOP_ACTION
				| DEBUG_STOP_WAKE_ACTION);
			if (fromLxThreadLevel) // dont't do this from interrupt context
				__rtx_call_rt_domain(to_rt_virq);
			breakWasInLxFlag = 1;
		}
                retVal = 1;
		local_irq_restore_hw(flags);
        }

	if (RT_DEBUG_VERBOSE)
		printkGen(NULL, "leave doStop_from_lx(cnt=%d ret=%d)\n",
                          countOfRtLxThreads, retVal);
	return(retVal);
}


// migration point before moving to linux
// normally we should be woken up by the gdbserver
int testForExtendedDebugWaitToLx(void)
{
	unsigned long flags;

	if (RT_DEBUG_VERBOSE)
		printkGen(NULL, "rtx_migrate_to_lx: enter debug wait\n"); 
	local_irq_save_hw(flags);
        // we want to be woken up by gdbserver debug action only
        while (!(current->rt_state3 & RT_TASK_LX_DEBUG_PENDING)) {
		__set_current_rt_state(RT_TASK_UNINTERRUPTIBLE);
		local_irq_restore_hw(flags);
		rt_schedule(0);
		local_irq_disable_hw();
        }
	local_irq_restore_hw(flags);
	if (RT_DEBUG_VERBOSE)
		printkGen(NULL, "rtx_migrate_to_lx: leave debug wait(cnt=%d)\n",
			countOfRtLxThreads); 
	return(0);
}

// normally the rt threads besides that rt thread which hits the breakpoint
// will wait until the breakpoint threads returns from linux.
// the breakpoint thread will continue the debug thread (if available)
// the debug thread continues all the other rt threads when done with
// its post code work.
// however, when the application hits a linux breakpoint not all rt threads
// will return to rt but caught before by linux debugging.
// waitAndTestForNewDebugEvent() polls certain debug states to allow the
// rt threads waiting to be continued within rt to go back to lx for
// the next debug session
static int waitAndTestForNewDebugEvent(int waitForBreakThread)
{
	int ret = 0;
	unsigned long flags;
	_INT64 expire = (_INT64) NSEC_TO_RT_JIFFIES(10000000); // 10 ms

	local_irq_save_hw(flags);
	local_irq_restore_hw(flags);
	while (1) {
		local_irq_disable_hw();
		__set_current_rt_state(RT_TASK_UNINTERRUPTIBLE);
		local_irq_restore_hw(flags);
		rt_schedule_timeout(&__raw_get_cpu_var(isr_list_hdr),
			expire);

		if (waitForBreakThread) {
			// breakthread wants to continue us
			if (!activeBreakThread) {
				ret = 1;
				break;
			}
		}
                else {
			// handle all the other rt threads
			if (activeDebugThreadState) {
				if ((activeDebugThreadState <= 20)
						|| (activeDebugThreadState > 34)) {
					ret = 2;
					break;
				}
			}
			// breakthread wants to continue us
			else if (!activeBreakThread) {
				ret = 3;
				break;
			}
		}

		// new lx breakpoint was set: move back to lx
		if (current->rt_state3
				& (RT_TASK_LX_DEBUG_PENDING | RT_TASK_RT_DEBUG_PENDING)) {
			ret = 4;
			break;
		}
	}
	//printkGen(NULL, "leave(%d) waitAndTest(%d)\n", ret, current->pid);

	return(ret);
}

// the breakpoint threads polls until all rt threads are back from lx
// alternatively the debug threads does this job (when we habe a lx breakpoint)
static int testOfReturnOfRtThreads(void)
{
	unsigned long flags;
	_INT64 expire = (_INT64) NSEC_TO_RT_JIFFIES(10000000); // 10 ms
	int maxBreakLoop;
	unsigned oldCount;
	int redoCount = 0;
        unsigned startCount;

	if (RT_DEBUG_VERBOSE)
	        printkGen(NULL, "enter testReturnRt cnt=%d\n", countOfRtLxThreads);
	local_irq_save_hw(flags);
	startCount = oldCount = countOfRtLxThreads;
	maxBreakLoop = MAX_WAIT_LOOPS;
	while (--maxBreakLoop) {
		if (countOfRtLxThreads == 0) {
			break;
		}
		if (oldCount != countOfRtLxThreads) {
			maxBreakLoop = MAX_WAIT_LOOPS;
			oldCount = countOfRtLxThreads;
			redoCount++;
		}
		__set_current_rt_state(RT_TASK_UNINTERRUPTIBLE);
		local_irq_restore_hw(flags);
		rt_schedule_timeout(&__raw_get_cpu_var(isr_list_hdr),
			expire);
		local_irq_disable_hw();
	}
	local_irq_restore_hw(flags);
	if (RT_DEBUG_VERBOSE)
		printkGen(NULL, "testReturnRt(%d:%d:%d)\n",
			maxBreakLoop, countOfRtLxThreads, startCount);
	return(redoCount);
}

// all rt threads enter this subroutine when migrating back to rt
// after a lx debug session
int testForExtendedDebugWaitToRt(void)
{
	unsigned long flags;
	int redoCount;
	unsigned actCount = countOfRtLxThreads;
        int breakPid = 0;

	if (RT_DEBUG_VERBOSE) {
		if (activeBreakThread)
			breakPid = activeBreakThread->pid;
		printkGen(NULL, "rtx_migrate_to_rt: enter debug wait(%#x:%#x) brk=%d(%#x)\n",
			current->rt_state, current->rt_state3, breakPid,
			((struct thread_info *)current->stack)->flags);
	}

	local_irq_save_hw(flags);
	if (countOfRtLxThreads > 0)
		countOfRtLxThreads--;
	local_irq_restore_hw(flags);
	if (current == activeBreakThread) {
		local_irq_disable_hw();
		activeBreakToRtCount = activeBreakInRtCount;
		local_irq_restore_hw(flags);
		redoCount = testOfReturnOfRtThreads();
		local_irq_disable_hw();
		// test whether debug thread is in RT
		if (activeDebugThread && (THREAD_IS_REALTIME(activeDebugThread))) {
			waitForRealtimeContAfterPostcode = 1;
			activeBreakThread = NULL;
			if (RT_DEBUG_VERBOSE)
				printkGen(NULL, "activeBreakThread wait for cont by debugThread(%d:%d)\n",
					countOfRtLxThreads, redoCount);
			local_irq_disable_hw();
			rt_wake_up_thread(activeDebugThread);
			// wait until debug thread is done with postcode
			__set_current_rt_state(RT_TASK_UNINTERRUPTIBLE);
			local_irq_restore_hw(flags);
			rt_schedule(0);
		}
		else {
			if (RT_DEBUG_VERBOSE)
				printkGen(NULL, "activeBreakThread continues realtime\n");
			// we must continue the other rt threads alone
			contAllRtThreads(current);
			activeBreakThread = NULL;
		}
		actCount = countOfRtLxThreads;
		//countOfRtLxThreads = 0;
		local_irq_restore_hw(flags);
	}
	else if (current == activeDebugThread) {
		if (breakWasInLxFlag) {
			// breakpoint was in lx, no rt breakpoint thread
			// we have to wait for the other rt threads alone
			activeDebugThreadState = 33;
			redoCount = testOfReturnOfRtThreads();
			//countOfRtLxThreads = 0;
			activeBreakThread = NULL;
			waitForRealtimeContAfterPostcode = 1;
		}
		else {
			// breakpoint was in rt, the breakpoint thread
			// continues the debug thread when all rt threads
			// are back
			// when the breakpoint thread is caught by a new breakpoint
			// in lx; we must continue by our own
			activeDebugThreadState = 34;
			waitAndTestForNewDebugEvent(1);
		}
		local_irq_disable_hw();
		actCount = countOfRtLxThreads;
		breakWasInLxFlag = 0;
		activeDebugThreadState = 36;
		if (current->rt_state3
				& (RT_TASK_LX_DEBUG_PENDING | RT_TASK_RT_DEBUG_PENDING)) {
			// we will not have a chance to move to rt
			// caught by a breakpoint within a linux thread
			// therefore, bring the rt threads waiting here also back to lx
			if (waitForRealtimeContAfterPostcode) {
				contAllRtThreads(current);
				waitForRealtimeContAfterPostcode = 0;
			}
			// the reason was a lx breakpoint save it as new state
			// take also the old rt thread count
 			countOfRtLxThreads = prevCountOfRtLxThreads;
			breakWasInLxFlag = newBreakWasInLxFlag;
			activeDebugThreadState = 15;
		}
		local_irq_restore_hw(flags);
	}
	else if ((activeBreakThread) || breakWasInLxFlag) {
		// all other rt threads wait until they are continued
		// by the debug thread
		// (after the debug thread did the post code action)
		// or by the breakpoint thread (when all rt threads are back)
		// we continue immediately, when there is non extended debug
		// thread and we had a breakpoint in a Linux thread
		// (nobody can continue us)
		if ((activeBreakThread) || activeDebugThread) {
			waitAndTestForNewDebugEvent(0);
		}
		rt_schedule(0);
		actCount = countOfRtLxThreads;
	}

	if (activeWatchdogThread == current)
		// remember debugging with the watchdog thread
		watchdogThreadInDebug = 1; 

	if (RT_DEBUG_VERBOSE)
		printkGen(NULL, "rtx_migrate_to_rt: leave debug wait (%d)\n",
			actCount);
	return(0);
}

int initDebugControl(void)
{
	countOfRtLxThreads = 0;
	waitForRealtimeContAfterPostcode = 0;
	activeBreakThread = NULL;
	activeDebugThread = NULL;
	activeWatchdogThread = NULL;
	activeDebugThreadState = 0;
	breakWasInLxFlag = 0;
	runWithGdbFlag = 0;
	watchdogInOperation = 0;
	watchdogThreadInDebug = 0;

	return(0);
}

#endif
