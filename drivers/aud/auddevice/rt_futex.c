/*
 * drivers/aud/auddevice/rt_futex.c
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
 *
 * In order to maintain the realtime capability we cannot use the linux
 * mm-semaphore. Therefore the virtual address inside the realtime process 
 * is solely used as key. This is possible since 
 * -- the complete RT process is locked into memory
 * -- only one RT-process is supported per CPU
 * -- (so far) no futexes on shared memory are supported. 
 * In case futexes on shared memory need to get supported
 * we would have to map to physical and then do a reverse map to virtual to the 
 * address space of the realtime process before calling realtime . Since this is 
 * all done at the Linux side, there is no problem with the mm-lock
 * A scan through a hash queue is protected by a spinlock which disables interrupts.
 * Because of the size of the local RT-hashqueue this is viewed to be acceptable also 
 * for the worst case szenario.
 */
#include <linux/aud/rt_timer.h>
#include <linux/aud/rt_futex.h>
#include <linux/jhash.h>

/* synchronize access to cmd_arr/free_list */
RTX_DEFINE_SPINLOCK(futex_list_lock);

struct lx_cmd cmd_arr[NO_OF_FU_REQUESTS];

/*
 * Futexes take the virtual address as key. 
 */
struct futex_q {
	struct list_head list;
	unsigned long key;
	struct task_struct *proc;
};

/*
 * Get the hash bucket
 */
static inline struct futex_hash_bucket *hash_futex(unsigned long key)
{
	u32 hash = jhash_1word(key, key);
	return &(current->rt_proc->futex_queues[hash & ((1 << FUTEX_HASHBITS)-1)]);
}

void rt_init_futex_queue(void)
{
	unsigned int i;
	struct rt_proc_hdr *pproc = current->rt_proc;

	for (i = 0; i < ARRAY_SIZE(pproc->futex_queues); i++)
		INIT_LIST_HEAD(&(pproc->futex_queues[i].chain));
}

/*
 * Called only once on startup.
 * Don't need a lock.
 */
void rt_init_futex(void)
{
	struct lx_cmd *cptr;
	unsigned int i;

	cptr = &cmd_arr[0];	
	INIT_LIST_HEAD(&aud_rt_hdr.free_list);						// free list of futex delegation requests
	for (i = 0; i < NO_OF_FU_REQUESTS ; i++, cptr++)
	{
		INIT_LIST_HEAD(&cptr->req_link);
		list_add(&cptr->req_link, &aud_rt_hdr.free_list);
	}	
}


/* Execute the futex system call in the LX-domain. */
int do_in_linux(unsigned long uaddr, int op, int val, unsigned long uaddr2, int val2, int val3)
{
	struct lx_cmd *cptr;

	if ((cptr = get_lx_request()) == NULL)
		return -EFAULT;						// should we try to pause some time?
	cptr->_req._fuop.uaddr = uaddr;
	cptr->_req._fuop.op = op;
	cptr->_req._fuop.val = val;
	cptr->_req._fuop.uaddr2 = uaddr2;
	cptr->_req._fuop.val2 = val2;
	cptr->_req._fuop.val3 = val3;
	cptr->type = EXEC_FUTEX_LX;
	add_interdomain_request((void*)cptr);
	return val;								// we cannot wait for the actual number
}

struct lx_cmd* get_lx_request(void)
{
	struct list_head *lp;
	
	if (aud_rt_hdr.free_list.next == &aud_rt_hdr.free_list)
		return NULL;	
	local_irq_disable_hw(); 
#ifdef CONFIG_SMP
	rtx_spin_lock(&futex_list_lock);
#endif
	lp = aud_rt_hdr.free_list.next;	
	if (lp == &aud_rt_hdr.free_list)
		lp = NULL;
	else
		list_del_init(lp);
#ifdef CONFIG_SMP
	rtx_spin_unlock(&futex_list_lock);
#endif
	local_irq_enable_hw(); 
	return list_entry(lp, struct lx_cmd, req_link);
}


/*
 * Lookup the thread with the highest priority waiting for this futex.
 * Returns the number of waiters - 1.
 * Note: interrupts must be off.
 */
static inline struct futex_q *rt_top_waiter(unsigned long uaddr, int *nrwaiter)
{
	struct futex_hash_bucket *bh;
	struct list_head *lp, *curr_lp = NULL;
	struct futex_q *fu;
	int waiter_cnt = 0;

	bh = hash_futex(uaddr);

	/* Lookup the futex list and find the waiter with the highest priority. */
	for (lp = bh->chain.next; lp != &bh->chain; lp = lp->next)
	{
		fu = (struct futex_q *)lp;
		if (fu->key == uaddr)
		{
        		if (!curr_lp)
        			curr_lp = lp;
        		else {
        			if (((struct futex_q *)curr_lp)->proc->prio > fu->proc->prio)
        				curr_lp = lp;
        			waiter_cnt++;
        		}
		}
	}
	if (nrwaiter)
		*nrwaiter = waiter_cnt;
	return (struct futex_q *)curr_lp;
}

/*
 * Lookup the thread with the highest priority which is blocked on a futex held by this task.
 * Note: interrupts must be off.
 * FIXME: The order of the futex requests should be sorted according to descending priority,
 * having the highest priority thread as the first list_element.
 */
static inline struct futex_q *rt_top_pi_waiter(struct task_struct *p)
{
	struct list_head *lp, *curr_lp = NULL;
	struct futex_q *fu;

	 /* Lookup the waiter list for all mutexes the thread already owns. */
	for (lp = p->rt_pi_waiters.next; lp != &p->rt_pi_waiters; lp = lp->next) {
		fu = (struct futex_q *)lp;
		if (!curr_lp)
			curr_lp = lp;
		else
			if (((struct futex_q *)curr_lp)->proc->prio > fu->proc->prio)
				curr_lp = lp;
	}
	return (struct futex_q *)curr_lp;
}


int rt_futex_getprio(struct task_struct *p)
{
	if (likely(!rt_task_has_pi_waiters(p)))
		return p->normal_prio;
	return (min(rt_top_pi_waiter(p)->proc->prio, p->normal_prio));
}


/* Defer printing of kernel messages until an atomic sequence has been left. */
static inline void rt_check_deferred_print(int ret, struct task_struct *owner_task)
{
	switch (ret) {
	case 1:
		if (RT_WARN_VERBOSE) {
			if (owner_task)
				printkGen(KERN_ALERT, "%s: no prio boost - owner thread is in LX opid=%d oprio=%d cpid=%d cprio=%d\n", __func__,
						owner_task->pid, owner_task->prio, current->pid, current->prio);
			else
				printkGen(KERN_ALERT, "%s: no prio boost - owner thread is in LX cpid=%d cprio=%d\n", __func__,
						current->pid, current->prio);
		}
		break;
	case 2:
		/* No common policy: ignore a prio change request */
		printkGen(KERN_ALERT, "%s: no prio boost - policy mismatch opid=%d opol=%d cpid=%d cpol=%d\n", __func__,
				owner_task->pid, owner_task->policy, current->pid, current->policy);
		break;
	default:
		break;
	}
}


/*
 * The owner thread of the futex will get the priority of the current waiting thread.
 * If the owner is currently blocked on a PI-futex then we also have to boost the
 * owner of this PI-futex. This has to be done for the complete rt_pi_blocked_on chain.
 *
 * Note: HW interrupts must be off.
 */
int requeue_task(struct task_struct *p); // see rt_sched.c

static inline int rt_boost_owner_prio(struct task_struct *owner_task_orig, unsigned long uaddr, struct futex_q *pi)
{
	int ret = 0;
	struct task_struct *owner_task = owner_task_orig;

	INIT_LIST_HEAD(&pi->list);
	/*
	 * The owner_task may be NULL when the owner thread is
	 * running in the LX domain. In this case no priority
	 * boosting takes place and thus we can't avoid priority
	 * inversion for this special use case.
	 * Maybe the owner thread is already on its way to LX.
	 */
	if (!owner_task || list_empty(&owner_task->rt_pid_list) || IS_PID_LIST_LEAVE_PENDING(owner_task)) {
		return 1;
	}

	if (owner_task->policy != current->policy) {
		return 2;
	}

	/* Deadlock detection */
	while (owner_task->rt_pi_blocked_on) {
		if (owner_task->rt_state2 & RT_TASK_DEADLOCK_DETECT)	{
			owner_task = owner_task_orig;
			while (owner_task->rt_pi_blocked_on && (owner_task->rt_state2 & RT_TASK_DEADLOCK_DETECT)) {
				owner_task->rt_state2 &= ~RT_TASK_DEADLOCK_DETECT;
				owner_task = owner_task->rt_pi_blocked_on;
			}
			return -EBUSY;
		}
		owner_task->rt_state2 |= RT_TASK_DEADLOCK_DETECT;
		owner_task = owner_task->rt_pi_blocked_on;
	}
	owner_task = owner_task_orig;
	while (owner_task->rt_pi_blocked_on && (owner_task->rt_state2 & RT_TASK_DEADLOCK_DETECT)) {
		owner_task->rt_state2 &= ~RT_TASK_DEADLOCK_DETECT;
		owner_task = owner_task->rt_pi_blocked_on;
	}
	owner_task = owner_task_orig;

	// FIXME: Add the request according to its priority.
	pi->key = uaddr;
	pi->proc = current;
	list_add(&pi->list, &owner_task->rt_pi_waiters);

	// The owner thread may have been already boosted and
	// may be running with the highest priority.
boost_owner:
	if (owner_task->prio > current->prio) {
		// Maybe the owner task is currently executing in LX (e.g. execute_in_lx).
		if (!THREAD_IS_REALTIME(owner_task)) {
			owner_task->rt_dyn_prio = current->prio;
			goto boost_exit;
		}
		owner_task->prio = current->prio;
		ret |= requeue_task(owner_task);
	}
boost_exit:
	// Boost also the priority of the owner
	// the current owner is blocked on.
	if (owner_task->rt_pi_blocked_on) {
		owner_task = owner_task->rt_pi_blocked_on;
		goto boost_owner;
	}
	return ret;
}

/*
 * Reset priority of the owner thread to its default value.
 * @return: 1 scheduling is required
 *          0 no scheduling
 *
 * Note: Interrupts must be off.
 */
static inline int rt_reset_futex_prio(struct task_struct *owner_task)
{
	/* Nothing to do if the owner-task has its default priority. */
	if (owner_task->prio == owner_task->normal_prio)
		return 0;

	// Maybe the owner task is currently executing in LX (e.g. execute_in_lx).
	if (!THREAD_IS_REALTIME(owner_task)) {
		owner_task->rt_dyn_prio = owner_task->normal_prio;
		return 0;
	}

	owner_task->prio = owner_task->normal_prio;
	return requeue_task(owner_task);
}


/*
 * Remove all PI waiters for the top_waiter futex
 * from the PI waiters list of the previous owner.
 * Add these requests to the PI waiters list
 * of the new owner except the request of the new
 * owner.
 * De-boost the priority of the previous owner.
 *
 * Note: Interrupts must be off.
 */
static inline int rt_fixup_pi_chain(struct futex_q *top_waiter)
{
	struct list_head *lp;
	struct futex_q *curr_fup;
	struct futex_q *pi_fup = NULL;

	// Walk the complete PI waiter chain of the previous owner (current).
	for (lp = current->rt_pi_waiters.next; lp != &current->rt_pi_waiters; ) {
		curr_fup = (struct futex_q *)lp;
		lp = lp->next;

		if (curr_fup->key == top_waiter->key) {
			// Delete the PI waiter request
			// from the PI waiter chain of
			// the previous owner.
			list_del_init((struct list_head *)curr_fup);

			// For PI waiters different from the new owner
			// add the PI waiter request to the PI waiters
			// list of the new owner. These waiters have a
			// lower priority than the new owner and therefore
			// no priority adjustment is necessary.
			if (curr_fup->proc != top_waiter->proc)
				list_add((struct list_head *)curr_fup, &top_waiter->proc->rt_pi_waiters);
		}
		else {
			// Check for de-boosting the previous owner (current).
			// The list may contain PI waiters which didn't boost
			// the priority of the previous owner.
			if (curr_fup->proc->prio > current->prio && current->normal_prio > curr_fup->proc->prio) {
				if (!pi_fup)
					pi_fup = curr_fup;
				else
					if (curr_fup->proc->prio < pi_fup->proc->prio)
						pi_fup = curr_fup;
			}
		}
	}
	// We may have to de-boost the previous owner, resetting its priority
	// to normal_prio if there are no PI waiters which have boosted its
	// priority. Otherwise we have to adjust the priority to the highest
	// boost priority.
	if (unlikely(pi_fup)) {
		current->prio = pi_fup->proc->prio;  // (de-)boost priority
		return requeue_task(current);
	}
	else {
		return rt_reset_futex_prio(current);	// go back to normal priority
	}
}


/*
 * Changing the priority of a thread dynamically we have to fixup the
 * owner priority if the thread is currently blocked on a PI-futex.
 *
 * Note: HW interrupts must be off.
 */
int rt_fixup_owner_prio(unsigned long uaddr, struct task_struct *owner_task)
{
	struct futex_q *top_waiter;
	struct futex_q *top_pi_waiter;
	struct task_struct *top_waiter_task;
	int ret = 0;

	if (!owner_task)
		return 0; 						/* owner thread has its default priority */

	// Maybe the owner thread is already on its way to LX.
	if (list_empty(&owner_task->rt_pid_list) || IS_PID_LIST_LEAVE_PENDING(owner_task))
		return 0;

	// Get the futex request with the highest priority thread
	// waiting for this futex.
	top_waiter = rt_top_waiter(uaddr, NULL);

	// Get the futex request with the highest priority thread
	// which is blocked on one of the futexes held by the owner task.
	top_pi_waiter = rt_top_pi_waiter(owner_task);

	// Set top_waiter to the request with the highest priority.
	if (top_waiter && top_pi_waiter) {
		if (top_pi_waiter->proc->prio < top_waiter->proc->prio)
			top_waiter = top_pi_waiter;
	}
	else {
		if (top_pi_waiter)
			top_waiter = top_pi_waiter;
	}

	// Note: top_waiter may be NULL. This happens when the current
	// thread was the only one waiting for the futex and the
	// owner thread doesn't hold other futexes with threads blocked on.
fixup_owner:
	if (top_waiter) {
		top_waiter_task = top_waiter->proc;

		//Check if we must boost or de-boost the priority of the owner task.
		if ((owner_task->prio > top_waiter_task->prio) ||				// boost
				((owner_task->prio < top_waiter_task->prio) &&			// de-boost
				(owner_task->normal_prio > top_waiter_task->prio))) {
			// Maybe the owner task is currently executing in LX (e.g. execute_in_lx).
			if (!THREAD_IS_REALTIME(owner_task)) {
				owner_task->rt_dyn_prio = top_waiter_task->prio;
				goto fixup_exit;
			}
			owner_task->prio = top_waiter_task->prio;
			requeue_task(owner_task);
			ret |= 1;
			goto fixup_exit;
		}
		goto fixup_exit;
	}
	ret |= rt_reset_futex_prio(owner_task);
fixup_exit:
	if (owner_task->rt_pi_blocked_on) {
		owner_task = owner_task->rt_pi_blocked_on;
		goto fixup_owner;
	}
	return ret;
}

/*
 * Is only called for a private futex.
 */
static int rt_futex_wake(unsigned long uaddr, int val)
{

	struct futex_hash_bucket *bh;
	struct list_head *lp, *curr_lp;
	struct futex_q *fu;
	int found_key_count = 0;

	bh = hash_futex(uaddr);
redoWake:
	curr_lp = NULL;
	local_irq_disable_hw();
	for (lp = bh->chain.next; lp != &bh->chain; lp = lp->next)
	{
		fu = (struct futex_q *)lp;
		if (fu->key == uaddr)
		{
        		if (!curr_lp)
        			curr_lp = lp;
        		else if (((struct futex_q *)curr_lp)->proc->prio > fu->proc->prio)
        			curr_lp = lp;
		}
	}

	if (curr_lp) {
		list_del_init(curr_lp);
		fu = (struct futex_q *)curr_lp;
		local_irq_enable_hw();

		// We are always executing in the same process which is expected to run on a single cpu.			
		// So we don't need to delegate the wakeup in case of a SMP system.
        rt_wake_up_thread(fu->proc);
		if (++found_key_count < val)
			goto redoWake;
	}
	else {
		local_irq_enable_hw();
	}
	return found_key_count;
}

/*
 * Unlock for PI futexes.
 * This function may also be called from the LX domain
 * and only unlocks a private futex.
 */
int rt_futex_wake_pi(unsigned long uaddr)
{
	struct futex_q *top_waiter;
	u32 curval;
	int nr_waiters = 0;

	// Make sure that it can be accessed safely.
	if (!access_ok(VERIFY_WRITE, (long unsigned __user *)uaddr, sizeof(unsigned long)))
		return -EFAULT;

	local_irq_disable_hw();

	curval = *(u32 *)uaddr;

	/* In case the owner already died, the TID is set to 0. */
	if (!(curval & FUTEX_OWNER_DIED)) {
		/* We release only a lock we actually own. */
		if ((curval & FUTEX_TID_MASK) != current->pid) {
			local_irq_enable_hw();
			return -EPERM;
		}

		/* If there is no waiter and the owner didn't die,
		 * do the atomic transition TID -> 0. */
		if (curval == current->pid) {
			*(u32 *)uaddr = 0;
			local_irq_enable_hw();
			return 0;
		}
	}

	/* If there is a LX waiter wake it up,
	 * repeating the system call in the LX domain.
	 * Note: Do not use the interdomain daemon
	 * because its pid is different from the pid
	 * of the futex owner.
	  */
	if (curval & FUTEX_WAITERS_PI_LX) {
		local_irq_enable_hw();
		rtx_migrate_to_lx(RTLX_TASK, NULL);
		return 0;
	}

	top_waiter = rt_top_waiter(uaddr, &nr_waiters);

	/* The highest priority waiter will be the new owner. */
	if (top_waiter) {
		list_del_init((struct list_head *)top_waiter);

		// Preserve OWNER_DIED-Bit.
		*(u32 *)uaddr = top_waiter->proc->pid | (curval & FUTEX_OWNER_DIED);		// new owner
		if (nr_waiters)
			*(u32 *)uaddr |= (FUTEX_WAITERS | FUTEX_WAITERS_PI_RT);

		// We are always executing in the same process which is expected to run on a single cpu.			
		// So we don't need to delegate the wakeup in case of a SMP system.
		rt_wake_up_thread(top_waiter->proc);
		if (IS_REALTIME) {
			// The new owner is no longer waiting for the futex.
			top_waiter->proc->rt_pi_blocked_on = NULL;
			top_waiter->proc->rt_owner_futex = 0;

			// Delete the PI waiter request of the new owner and move
			// the remaining PI waiter requests for this futex from
			// the previous owner (current) to the new owner.
			// If there are PI waiters we have to consider the highest
			// priority for de-boosting the previous owner.
			if (rt_fixup_pi_chain(top_waiter)) {
				/* Note: Interrupts will be enabled by rt_schedule() */
				rt_schedule(0);
			}
			local_irq_enable_hw();
			return 1;
		}
		local_irq_enable_hw();
		return 1;
	}
	/* We have no waiters. */
	if (!(curval & FUTEX_OWNER_DIED))
		*((int *)uaddr) = 0;
	local_irq_enable_hw();
	return 0;
}

/*
 * This routine gets executed in the Linux domain. So far futexes work only inside the realtime process.
 * In case it has to work also between the realtime process and other processes, an address mapping has
 * to be done, virtual -> physical, physical -> virtual of the realtime process.
 * Note: The current implementation guarantees that all LX-threads of a RT-process are pinned to the
 * CPU the RT-process is running on.
 */
int rtx_futex_wake_from_lx(unsigned long uaddr, int val, int inherit)
{
	int nr;

	if (rtx_get_cpu_var(rtx_mode) != RT_DOMAIN)
		return 0;

	if (inherit == FUTEX_PI_MODE)
	    nr = rt_futex_wake_pi(uaddr);		// note: may return error
	else
		nr = rt_futex_wake(uaddr, val);

	if (nr > 0)
		lx_wake_up_rt_thread(NULL); 	
	return nr;	
}

/* The futex wait function for private non-PI futexes. */
int rt_futex_wait(unsigned long uaddr, int val, int timeout_flag, long long time)
{
	struct futex_hash_bucket *bh;
	u32 curval;
	struct futex_q q;

	// We migrated from LX to RT and got a debugging event.
	if (current->rt_state3 & RT_TASK_LX_DEBUG_PENDING)
		return 0;

	// Make sure that it can be accessed safely.
	if (!access_ok(VERIFY_WRITE, (long unsigned __user *)uaddr, sizeof(unsigned long)))
		return -EFAULT;

	bh = hash_futex(uaddr);
	INIT_LIST_HEAD(&q.list);
	q.key = uaddr;
	q.proc = current;

	local_irq_disable_hw();
	curval = *(u32 *)uaddr;
	if (curval != val) {
		local_irq_enable_hw();
		return -EWOULDBLOCK;
	}
	list_add_tail(&q.list, &bh->chain);
	__set_current_rt_state(RT_TASK_UNINTERRUPTIBLE | RT_TASK_FUTEX_CLEANUP);

	/* Note: Interrupts will be enabled by rt_schedule()
	* resp. rt_schedule_timeout() 
	*/
	if (!timeout_flag)
	{
		rt_schedule(0);

		// We may have been thrown out due to a debugging signal
		// or we are forced to exit by ^C.
		current->rt_state &= ~RT_TASK_FUTEX_CLEANUP;
		local_irq_disable_hw();
		list_del_init(&q.list);
		local_irq_enable_hw();
		if (IS_EXIT_PENDING(current))
			return rt_task_leave();
		return 0;
	}
	time = rt_schedule_timeout(&__raw_get_cpu_var(isr_list_hdr), time);

	// We may have been thrown out due to a debugging signal.
	// or we are forced to exit by ^C
	// or a timeout has occurred.
	current->rt_state &= ~RT_TASK_FUTEX_CLEANUP;
	local_irq_disable_hw();
	list_del_init(&q.list);
	local_irq_enable_hw();
	if (IS_EXIT_PENDING(current))
		return rt_task_leave();
	return (time == 0) ? -ETIMEDOUT : 0;
}

/* The futex wait function for private PI futexes. */
int rt_futex_wait_pi(unsigned long uaddr, int val, int timeout_flag, long long time, unsigned try_flag)
{
	struct futex_hash_bucket *bh;
	struct task_struct *owner_task;
	u32 curval, curval1 = 0;
	struct futex_q q;
	struct futex_q pi;
	pid_t owner_pid;
	int ret;

	// We migrated from LX to RT and got a debugging event.
	if (current->rt_state3 & RT_TASK_LX_DEBUG_PENDING)
		return -ERESTARTNOINTR;

	// Make sure that it can be accessed safely.
	if (!access_ok(VERIFY_WRITE, (long unsigned __user *)uaddr, sizeof(unsigned long)))
		return -EFAULT;
				
	bh = hash_futex(uaddr);
	INIT_LIST_HEAD(&q.list);
	q.key = uaddr;
	q.proc = current;

	/* test whether we still should wait and get the owner */
	local_irq_disable_hw();
retry_wait:
	curval1 = *(int*)uaddr;					// it is accessible without fault
	owner_pid = curval1 & FUTEX_TID_MASK;

	if (!curval1) {
		/* it's free, let's lock it */
		*(int *)uaddr = current->pid;
		ret = 0;
		goto fpi_exit_enable;
	}

	/* Detect deadlock situation. */
	if (owner_pid == current->pid) {
		ret = -EDEADLK;
		goto fpi_exit_enable;
	}

	/*
	 * If the futex has already a LX waiter
	 * then it is not accessible for RT.
	 */
	if (curval1 & FUTEX_WAITERS_PI_LX) {
		ret = -EPERM;
		goto fpi_exit_enable;
	}

	/* May be the owner died, we take over the lock
	 * and preserve the FUTEX_OWNER_DIED bit. */
	if ((curval1 & FUTEX_OWNER_DIED) || !owner_pid) {
		*(int *)uaddr = current->pid | (curval1 & ~FUTEX_TID_MASK);
		ret = 0;
		goto fpi_exit_enable;
	}
	local_irq_enable_hw();

	// Get task descriptor without a semaphore lock.
	if ((owner_task = rt_find_task_by_pid(owner_pid)) == NULL) {
		// May be the owner thread died (robust PI futexes).
		if (!((*(int *)uaddr) & FUTEX_OWNER_DIED)) {
			// The owner task is a LX-thread. This may be the rare case when a PI futex
			// is shared between LX and RT domain. There is only one use case supported
			// and for this use case we cannot avoid priority inversion:
			// - LX thread (NRT) acquires the futex (owner).
			// - A RT thread (which has higher priority in any case) tries to lock this futex.
			//   The priority of the LX owner thread is not boosted and there is no error return.
			// - The LX owner thread releases the futex and wakes up the waiting RT thread.
			//
			// May be the owner thread has left the RT domain because of an exit. Then it is
			// also a LX thread never releasing the futex.
			if (RT_WARN_VERBOSE)
				printkGen(KERN_ALERT, "%s: owner_pid=%d not found\n", __func__, owner_pid);
		}
	}
	local_irq_disable_hw();
	curval = *(u32 *)uaddr;
	owner_pid = curval & FUTEX_TID_MASK;

	/* When the futex is free then take it.*/
	if (!curval) {
		*(int *)uaddr = current->pid;
		ret = 0;
		goto fpi_exit_enable;
	}

	/* Detect deadlock situation. */
	if (owner_pid == current->pid) {
		ret = -EDEADLK;
		goto fpi_exit_enable;
	}

	/*
	 * If the futex has already a LX waiter
	 * then it is not accessible for RT.
	 */
	if (curval & FUTEX_WAITERS_PI_LX) {
		ret = -EPERM;
		goto fpi_exit_enable;
	}

	/* May be the owner died, we take over the lock
	 * and preserve the FUTEX_OWNER_DIED bit. */
	if ((curval & FUTEX_OWNER_DIED) || !owner_pid) {
		*(int *)uaddr = current->pid | (curval & ~FUTEX_TID_MASK);
		ret = 0;
		goto fpi_exit_enable;
	}

	/* Somebody changed the futex or the owner task. */
	if (likely(curval != curval1) || (owner_task && 
		(list_empty(&owner_task->rt_pid_list) || IS_PID_LIST_LEAVE_PENDING(owner_task)))) {
		goto retry_wait;
	}

	/*
	 * FUTEX_TRYLOCK_PI never gets blocked.
	 */
	if (try_flag) {
		ret = -EWOULDBLOCK;
		goto fpi_exit_enable;
	}

	*(int*)uaddr |= ((FUTEX_WAITERS | FUTEX_WAITERS_PI_RT));
	/* Boost owner priority */
	if ((ret = rt_boost_owner_prio(owner_task, uaddr, &pi)) < 0) {
		local_irq_enable_hw();
		printkGen(KERN_ALERT, "%s: owner boost - deadlock detected opid=%d oprio=%d cpid=%d cprio=%d\n", __func__,
                              owner_task->pid, owner_task->prio, current->pid, current->prio);
		return ret;
	}

	current->rt_pi_blocked_on = owner_task;
	current->rt_owner_futex = uaddr;
	list_add_tail(&q.list, &bh->chain);		
	__set_current_rt_state(RT_TASK_UNINTERRUPTIBLE | RT_TASK_FUTEX_CLEANUP);

	// We are in a critical section, so do not enable interrupts
	// which is done on return of rt_schedule()/rt_schedule_timeout().
	if (!timeout_flag)
	{
		rt_schedule(0);
		
		// Maybe there is a kernel message pending that has to be shown.
		if (ret > 0)
			rt_check_deferred_print(ret, owner_task);

		// We may have been thrown out due to a debugging signal
		// or we are forced to exit by ^C.
		local_irq_disable_hw();
		current->rt_state &= ~RT_TASK_FUTEX_CLEANUP;
		list_del_init(&q.list);
		current->rt_pi_blocked_on = NULL;
		current->rt_owner_futex = 0;
		list_del_init(&pi.list);
		local_irq_enable_hw();
		if (IS_EXIT_PENDING(current))
			return rt_task_leave();
		if (current->rt_state3 & RT_TASK_LX_DEBUG_PENDING)
			return -ERESTARTNOINTR;
		return 0;
	}
	time = rt_schedule_timeout(&__raw_get_cpu_var(isr_list_hdr), time);

	// Maybe there is a kernel message pending that has to be shown.
	if (ret > 0)
		rt_check_deferred_print(ret, owner_task);

	// We may have been thrown out due to a debugging signal
	// or we are forced to exit by ^C
	// or a timeout has occurred.
	local_irq_disable_hw();
	current->rt_state &= ~RT_TASK_FUTEX_CLEANUP;
	list_del_init(&q.list);
	current->rt_pi_blocked_on = NULL;
	current->rt_owner_futex = 0;
	list_del_init(&pi.list);
	if (IS_EXIT_PENDING(current)) {
		local_irq_enable_hw();
		return rt_task_leave();
	}
	if (current->rt_state3 & RT_TASK_LX_DEBUG_PENDING) {
		local_irq_enable_hw();
		return -ERESTARTNOINTR;
	}

	if (time == 0) 
	{
	     /*
		 * Adjust PI priority:
		 * This sequence will only be effective when the owner task has a waiting point
		 * otherwise the timeout task will be queued after the owner task in the scheduling
		 * list and releases the futex before we can run this code.
		 */
		ret = rt_fixup_owner_prio(uaddr, owner_task);
		/* Note: Interrupts will be enabled by rt_schedule() */
		if (ret)
			rt_schedule(0);
		local_irq_enable_hw();
		return -ETIMEDOUT;
	}
	local_irq_enable_hw();
	return 0;

fpi_exit_enable:
	local_irq_enable_hw();
	return ret;
}

asmlinkage long sys_rt_futex(u32 __user *uaddr, int op, int val, struct timespec __user *utime, u32 __user *uaddr2, int val3)
{
	long long timeout = 0;
	long unsigned offset;
	int val2 = 0;
	int timeoutFlag = 0; 	// wait indefinitely
	int nr = 0;
	int cmd;
	long ret;

#ifndef CONFIG_RTX_OPTIMIZE_FOR_RELEASE
	if (RT_FUTEX_VERBOSE)								
		printkGen(NULL, "%s enter: pid=%2d prio=%2d:%d (%p(%#x):%#x:%d:%p)\n",
			__func__, current->pid, current->prio, current->rt_priority, uaddr, *uaddr, op, val, utime);
#endif

	/*
	 * In newer kernel versions the op code may specify
	 * additional flags (FUTEX_PRIVATE_FLAG and
	 * FUTEX_CLOCK_REALTIME).
	 */
	if ((cmd = op & FUTEX_CMD_MASK) == FUTEX_LOCK_PI_REL)
		cmd = FUTEX_LOCK_PI;

#ifndef CONFIG_RTX_OPTIMIZE_FOR_RELEASE
	if (cmd == FUTEX_WAIT_BITSET_REL)
		op &= ~FUTEX_CLOCK_REALTIME;
#endif

	/* We don't provide shared futexes. */
	if (!(op & FUTEX_PRIVATE_FLAG)) {
	    	/* ENOSYS may cause endless loops 
		* (glibc: e.g. __lll_reltimedlock_wait */
#ifndef CONFIG_RTX_OPTIMIZE_FOR_RELEASE
		printkGen(KERN_ALERT, "%s: FUTEX_PRIVATE_FLAG not specified pid=%d op=%#x\n", __func__, current->pid, op);
#endif
		return -EINVAL;
	}

#ifndef CONFIG_RTX_OPTIMIZE_FOR_RELEASE
	if (op & FUTEX_CLOCK_REALTIME) {
		printkGen(KERN_ALERT, "%s: FUTEX_CLOCK_REALTIME not supported pid=%d op=%#x\n", __func__, current->pid, op);
		return -EINVAL;
	}
#endif

	offset = (long unsigned)uaddr % PAGE_SIZE;			// check for a naturally aligned futex
	if (unlikely((offset % sizeof(u32)) != 0)) {
		if (RT_WARN_VERBOSE)
			printkGen(KERN_ALERT, "%s: futex not aligned pid=%d\n", __func__, current->pid);
		return -EINVAL;	
	}

	switch (cmd)
	{
		case FUTEX_WAKE: 
		{
			nr = rt_futex_wake((unsigned long) uaddr, val);
			if (nr)
				rt_schedule(0);

			// Number of requested wakeups not contended?
			if (nr < val)	
				do_in_linux((unsigned long)uaddr, op, val-nr, (unsigned long)uaddr2, val2, val3);

#ifndef CONFIG_RTX_OPTIMIZE_FOR_RELEASE
			if (RT_FUTEX_VERBOSE)
				printkGen(NULL, "%s leave: pid=%d prio=%d(%d) nr=%d val=%d uaddr=%#lx\n",
						__func__, current->pid, current->prio, current->rt_priority, nr, val, *uaddr);
#endif
			return nr;		
		}

		case FUTEX_LOCK_PI:
		case FUTEX_WAIT_BITSET_REL:
			/* FALLTHROUGH */
		case FUTEX_WAIT:
		{
			if (utime) {
				if ((ret = copy_conv_to_internal(&timeout, utime)) < 0)
					return(ret);
#ifndef CONFIG_RTX_OPTIMIZE_FOR_RELEASE
				if (RT_FUTEX_VERBOSE) {
					printkGen(NULL, "%s: timeout=%ld\n", __func__, timeout);
				}
#endif
				if (timeout == 0)
					timeout = 1;
				timeoutFlag = 1;
			}
			if (cmd == FUTEX_LOCK_PI)
				ret = rt_futex_wait_pi((unsigned long)uaddr, val, timeoutFlag, timeout, 0);
			else
				ret = rt_futex_wait((unsigned long)uaddr, val, timeoutFlag, timeout);

#ifndef CONFIG_RTX_OPTIMIZE_FOR_RELEASE
			if (RT_FUTEX_VERBOSE)
				printkGen(NULL, "%s leave: pid=%d prio=%d(%d) ret=%#lx uaddr=%#lx pi=%d\n",
						__func__, current->pid, current->prio, current->rt_priority, ret, *uaddr, cmd == FUTEX_LOCK_PI);
#endif
			return ret;
		}

		case FUTEX_TRYLOCK_PI:
			return rt_futex_wait_pi((unsigned long)uaddr, val, 0, 0, 1);
			break;

		case FUTEX_UNLOCK_PI:
		{
			ret = rt_futex_wake_pi((unsigned long) uaddr);
#ifndef CONFIG_RTX_OPTIMIZE_FOR_RELEASE
			if (RT_FUTEX_VERBOSE)
			printkGen(NULL, "%s leave: FUTEX_UNLOCK_PI pid=%d prio=%d(%d) ret=%d uaddr=%#lx\n",
				__func__, current->pid, current->prio, current->rt_priority, ret, *uaddr);
#endif
			return ret < 0 ? ret : 0;
		}

		default:
			if (RT_FAULT_VERBOSE || RT_SYSCALL_NOT_IMPL_VERBOSE)
				printkGen(NULL, "%s: op=%#x not implemented\n",__func__, op);
			return -ENOSYS;
	}
	return 0;
}

/*
 * Checks whether there are RT threads that are already
 * waiting on this futex.
 *
 * Return
 *  < 0: Futex is not accessible (EINVAL, EFAULT, ENOSYS).
 *    1: At least one RT thread is waiting on the futex.
 *    0: No RT thread waiting.
 */
int rtx_waiter_pi_futex(u32 __user *uaddr, int fshared)
{
	unsigned long addr = (unsigned long)uaddr;

	/* Futex naturally aligned? */
	if (unlikely((addr % sizeof(u32)) != 0))
		return -EINVAL;

	/* Futex can be accessed? */
	if (unlikely(!access_ok(rw, uaddr, sizeof(u32))))
		return -EFAULT;

	/* Check if there is at least one RT thread which is
	 * waiting for the futex to be released.
	 * Note: The RT domain only handles private futexes. */
	if (*uaddr & FUTEX_WAITERS_PI_RT) {
		if (unlikely(fshared)) {
			return -ENOSYS;
		}
		return 1;				// it's a RT pi-futex
	}
	return 0;
}

asmlinkage int sys_rt_set_robust_list(struct robust_list_head __user * head, size_t len)
{
	if (!futex_cmpxchg_enabled) {
		return -ENOSYS;
	}
	/*
	 * The kernel knows only one size for now:
	 */
	if (unlikely(len != sizeof(*head)))
		return -EINVAL;

	current->robust_list = head;

	return 0;
}
