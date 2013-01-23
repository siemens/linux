/*
 * drivers/aud/auddevice/rt_tgroup.c
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
 * This file contains the functions for RT thread group quota handling.
 * Note: We do not support the quota inheritance protocol.
 * 
 */

#include <linux/aud/rt_timer.h>
#include <linux/aud/rt_tgroup.h>

#ifdef CONFIG_RTX_THREAD_GROUP_SUPPORT
static DEFINE_PER_CPU(struct rt_semaphore, tgroup_sem);
static DEFINE_PER_CPU(rt_tgroup_t, tgroup_arr)[RT_TGROUP_MAX];

DEFINE_PER_CPU(rt_tgroup_head_t, quota_head);
DEFINE_PER_CPU(rt_tgroup_head_t *, pquota_head);
DEFINE_PER_CPU(struct itimer, quota_itmr);

DEFINE_PER_CPU(struct list_head, rt_blocked_queue);
DEFINE_PER_CPU(struct list_head *, prt_blocked_queue);


extern void rt_set_quota_timer(_INT64 rel_delay_ns, int (*timer_action)(void *), struct task_struct *, rt_tgroup_head_t *, int);
extern void rt_release_quota_timer(void);

void quota_interval_start(rt_tgroup_head_t *);
void quota_interval_stop(rt_tgroup_head_t *);
int quota_timer_isr(void *arg);


/*
 * This function will be called in case of initializing
 * the RT-process (timer daemon).
 */
void rt_tgroup_init(void)
{
	int i;
	rt_tgroup_t *ptg;
	rt_tgroup_head_t *pqh;

	sema_init_rt(&__raw_get_cpu_var(tgroup_sem), 1);

	for (i = 0; i < RT_TGROUP_MAX; i++) {
		ptg = &__raw_get_cpu_var(tgroup_arr[i]);
		ptg->id = 0;
		ptg->quota = 0;
		ptg->quota_peak = 0;
		ptg->quota_ns = 0;
		ptg->quota_peak_ns = 0;
		ptg->quota_rem_ns = 0;
		INIT_LIST_HEAD(&ptg->thread_list);
	}
	rtx_set_cpu_var(pquota_head, &__raw_get_cpu_var(quota_head));
	pqh = rtx_get_cpu_var(pquota_head);
	pqh->quota_enabled = 0;
	pqh->quota_threads = 0;
	pqh->quota_interval_ns = (CONFIG_RTX_THREAD_GROUP_INTERVAL_US * 1000);
	pqh->quota_start_ns = 0;

	/* Initialize the blocked queue. */
	INIT_LIST_HEAD(&__raw_get_cpu_var(rt_blocked_queue));
	rtx_set_cpu_var(prt_blocked_queue, &__raw_get_cpu_var(rt_blocked_queue));

	INIT_LIST_HEAD(&__raw_get_cpu_var(quota_itmr.it_link));

	/* The quota assignment timer will be started immediately
	 * to avoid jitter when it would be activated on demand. */
	local_irq_disable_hw();
	quota_interval_start(pqh);
	local_irq_enable_hw();
}

/*
 * This function will be called in case of a shutdown (timer daemon),
 * to make sure that all thread groups will be deleted.
 */
void rt_tgroup_destroy(void)
{
	struct task_struct *proc;
	struct list_head *lp, *head;
	int i;
	rt_tgroup_t *ptg;
	rt_tgroup_head_t *pqh = rtx_get_cpu_var(pquota_head);

	down_rt(&__raw_get_cpu_var(tgroup_sem));
	local_irq_disable_hw();
	quota_interval_stop(pqh);

	// We only delete the group id. The proper
	// initialization will be accomplished by
	// rt_tgroup_init().
	for (i = 0; i < RT_TGROUP_MAX; i++) {
		ptg = &__raw_get_cpu_var(tgroup_arr[i]);
		if (ptg->id) {
			ptg->id = 0;

			/* Remove all the threads from this group. */
			head = &ptg->thread_list;
			lp = head->next;

			/*
			 * A thread may have already called exit in LX. In this case
			 * it has removed itself from the thread group in the shutdown sequence.
			 */
			while(lp != head)
			{
				proc = (struct task_struct *)list_entry(lp, struct task_struct, rt_thread_list);
				proc->rt_thread_group = NULL;
				pqh->quota_threads--;
				lp = lp->next;
				list_del_init(&proc->rt_thread_list);	// unchain

				proc->rt_state2 &= ~(RT_TASK_QUOTA_EXCEEDED | RT_DEFER_QUOTA_HANDLING);
			}
		}
	}
	local_irq_enable_hw();
	up_rt(&__raw_get_cpu_var(tgroup_sem));
}

/*
 * Replace the group timer (if activated) by the interval timer or a new group timer.
 */
static inline void set_quota_timer(int flag, rt_tgroup_head_t *pqh, rt_tgroup_t *ptg, int force_sched)
{
	RTX_TIMER_TYPE curr_time;
	unsigned long period;

	if (ptg->quota_start_ns || flag == RT_QUOTA_GROUP_TIMER) {
		curr_time = RTX_TIMER_GET;
		period = (unsigned long)(curr_time - pqh->quota_start_ns);

		if (period < pqh->quota_interval_ns) {
			if (flag == RT_QUOTA_INTERVAL_TIMER) {
				ptg->quota_start_ns = 0;
				rt_set_quota_timer(pqh->quota_interval_ns - period, quota_timer_isr, NULL, pqh, force_sched);
			}
			else {
				ptg->quota_start_ns = curr_time;
				rt_set_quota_timer(min(ptg->quota_rem_ns, (long)(pqh->quota_interval_ns - period)), quota_timer_isr, current, pqh, force_sched);
			}
		}
	}
}

/*
 * Return the sum of quotas for all created thread groups.
 * Note: tgroup_sem has to be locked before.
 */
static int get_quota_sum(void)
{
	int i, quota_sum;
	rt_tgroup_t *ptg;

	for (i = 0, quota_sum = 0; i < RT_TGROUP_MAX; i++) {
		ptg = &__raw_get_cpu_var(tgroup_arr[i]);
		if (ptg->id)
			quota_sum += ptg->quota;
	}
	return quota_sum;
}

/*
 * Check whether there is a thread in the thread group which is
 * ready to run (but couldn't run because of higher priority threads).
 * If the current thread is interrupted by the quota assignment
 * interrupt, its remaining quota (for the next interval) should be
 * accumulated to the credit.
 */
static inline int group_has_threads_ready_to_run(rt_tgroup_t *ptg)
{
	struct task_struct *proc = NULL;
	struct list_head *lp, *head;

	head = &ptg->thread_list;
	lp = head->next;
	while(lp != head)
	{
		proc = (struct task_struct *)list_entry(lp, struct task_struct, rt_thread_list );

		if (proc->rt_state & RT_TASK_RUNNING)
			return 1;
		lp = lp->next;
	}
	return 0;
}

/* Lookup a thread in a thread group. */
struct task_struct *rt_find_thread_in_group(rt_tgroup_t *ptg, struct task_struct *p)
{
	struct task_struct *proc = NULL;
	struct list_head *lp, *head;

	head = &ptg->thread_list;
	lp = head->next;
	while (lp != head)
	{
		proc = (struct task_struct *)list_entry(lp, struct task_struct, rt_thread_list );
		if (p == proc)
			break;
		lp = lp->next;
		proc = NULL;
	}
	return proc;
}

/*
 * Check whether the thread (specified by p) is a member of a thread group.
 *
 * Returns:
 *  0:	The thread is not a member of a thread group
 *  1:	The thread is a member of the thread group specified by group
 * -1:  The thread is a member of another thread group
 */
static int thread_is_group_member(tgroup_t group, struct task_struct *p)
{
	int i;
	rt_tgroup_t *ptg;

	for (i = 0; i < RT_TGROUP_MAX; i++) {
		ptg = &__raw_get_cpu_var(tgroup_arr[i]);
		if (list_empty(&ptg->thread_list))
			continue;
		if (rt_find_thread_in_group(ptg, p)) {
			if (ptg->id == group)
				return 1;
			else
				return -1;
		}
	}
	return 0;

}

/*
 * This function will only be called in LX
 * for the current thread on exit.
 */
void rt_check_quota_thread_from_lx(void)
{
	local_irq_disable_hw();
	if (current->rt_thread_group) {
		current->rt_thread_group = NULL;
		rtx_get_cpu_var(pquota_head)->quota_threads--;
		list_del_init(&current->rt_thread_list);	// unchain the thread
	}
	local_irq_enable_hw();
}

/*
 * Reset the quota exceeded state from threads of all thread groups
 * or of a specific thread group. When invoked by the quota assignment
 * interval interrupt, then keep the quota exceeded state for threads
 * of a thread group the quota of which is currently set to 0.
 */
void rt_reset_exceeded_state(rt_tgroup_t *ptg, int flag)
{
	int i;
	struct list_head *head, *lp;
	struct task_struct *proc;

	if (!ptg) {
		/* Remove the EXCEEDED state in all thread groups. */
		for (i = 0; i < RT_TGROUP_MAX; i++) {
			ptg = &__raw_get_cpu_var(tgroup_arr[i]);
			if (ptg->id) {

				/* Don't reset the thread state when
				 * the quota of the thread group it belongs to is set to 0.
				 * That case has only to be considered when the quota
				 * assignment interrupt occurs.
				 */
				if (!ptg->quota && (flag == RT_QUOTA_KEEP_ZERO_BLOCKED))
					continue;

				head = &ptg->thread_list;
				lp = head->next;
				while(lp != head)
				{
					proc = (struct task_struct *)list_entry(lp, struct task_struct, rt_thread_list );
					lp = lp->next;

					/* Reset the EXCEEDED/DEFER flags. */
					proc->rt_state2 &= ~(RT_TASK_QUOTA_EXCEEDED | RT_DEFER_QUOTA_HANDLING);
				}
			}
		}
	}
	else {
		/* Only the specified thread group is involved. */
		head = &ptg->thread_list;
		lp = head->next;
		while(lp != head)
		{
			proc = (struct task_struct *)list_entry(lp, struct task_struct, rt_thread_list );
			lp = lp->next;

			/* Reset the EXCEEDED/DEFER flags. */
			proc->rt_state2 &= ~(RT_TASK_QUOTA_EXCEEDED | RT_DEFER_QUOTA_HANDLING);
		}
	}
}

/*
 * Start global quota handling.
 * Activate the quota interval timer.
 * Note: Interrupts must be off.
 */
void quota_interval_start(rt_tgroup_head_t *pqh)
{
	pqh->quota_start_ns = RTX_TIMER_GET;
	rt_set_quota_timer(pqh->quota_interval_ns, quota_timer_isr, NULL, pqh, 0);
	pqh->quota_enabled = 1;
}


/*
 * Stop the quota handling for the current thread.
 * Note: Interrupts must be off.
 */
int quota_thread_stop(rt_tgroup_head_t *pqh, rt_tgroup_t *ptg)
{
	unsigned long period;
	RTX_TIMER_TYPE curr_time;
	int threads_shifted = 0;

	curr_time = RTX_TIMER_GET;

	/* Update the remaining quota of the group. */
	period = (unsigned long)(curr_time - ptg->quota_start_ns);
	if (period < ptg->quota_rem_ns)
		ptg->quota_rem_ns -= period;
	else {
		/* The current thread has consumed the remaining quota. */
		threads_shifted = rt_shift_all_threads_to_blockedq(&ptg->thread_list, current);
		ptg->quota_rem_ns = 0;
	}
	/* The HW timer is still running. */
	ptg->quota_start_ns = 0;

	/* Start the interval timer for the rest of the assignment interval. */
	period = (unsigned long)(curr_time - pqh->quota_start_ns);
	if (period < pqh->quota_interval_ns) {
		rt_set_quota_timer(pqh->quota_interval_ns - period, quota_timer_isr, NULL, pqh, 0);
	}
	return threads_shifted;
}

/*
 * Will be called from rt_schedule/rtx_preemption_handling
 * A quota thread gives up the CPU voluntarily.
 * Note: Interrupts must be off
*/
void quota_pre_switch(rt_tgroup_head_t *pqh, struct task_struct *preq)
{
	unsigned long period;
	RTX_TIMER_TYPE curr_time;
	rt_tgroup_t *ptg = current->rt_thread_group;
	rt_tgroup_t *ptg_req = preq->rt_thread_group;

	if (!pqh->quota_enabled)
		return;

	/* The current thread is controlled by a group timer. */
	if (ptg && ptg->quota_start_ns) {
		curr_time = RTX_TIMER_GET;
		period = (unsigned long)(curr_time - ptg->quota_start_ns);
		if (period < ptg->quota_rem_ns) {
			ptg->quota_rem_ns -= period;
		}
		else {
			/*
			 * The quota is exceeded, so give the thread a minimal quota
			 * which causes the thread to be handled as having no more quota
			 * when it starts running again (quota_post_switch).
			 * */
			ptg->quota_rem_ns = RT_QUOTA_MIN_NS;
		}
		/* The HW-timer is still armed. */
		ptg->quota_start_ns = 0;

		/* Start the quota timer for the remaining assignment interval. */
		if (THREAD_IS_LINUX(preq) || !ptg_req || (ptg_req->quota == RT_TGROUP_QUOTA_OFF) || !ptg_req->quota) {
			period = (unsigned long)(curr_time - pqh->quota_start_ns);
			if (period < pqh->quota_interval_ns) {
				rt_set_quota_timer(pqh->quota_interval_ns - period, quota_timer_isr, NULL, pqh, 0);
			}
		}
	}
}

/*
 * Activate the quota for the current RT-thread if its quota is enabled.
 * Note: Interrupts must be off.
 */
void quota_post_switch(rt_tgroup_head_t *pqh)
{
	rt_tgroup_t *ptg = current->rt_thread_group;

	if (!pqh->quota_enabled)
		return;

	/*
	 * When the quota for the current thread is enabled,
	 * we have to start the group timer for this thread.
	 * Note: If the remaining quota value is small enough then the
	 * HW-timer will not be armed, the quota_timer_isr() will be
	 * invoked synchronously.
	 */
	if (RT_QUOTA_IS_ENABLED(ptg)) {
		set_quota_timer(RT_QUOTA_GROUP_TIMER, pqh, ptg, FORCE_SCHED);
	}

}

/*
 * Activate the quota for the current RT-thread if its quota is enabled.
 * Since the thread is currently migrating from LX to RT, the quota may
 * already be exceeded. So the thread has to give up the CPU. 
 */
void quota_post_switch_from_lx(rt_tgroup_head_t *pqh)
{
	rt_tgroup_t *ptg;

	if (!pqh->quota_enabled)
		return;

	local_irq_disable_hw();

	/*
	 * If the quota of the current thread is exceeded
	 * the thread has to be shifted to the blocked queue.
	 */
	if (RT_QUOTA_IS_EXCEEDED(current)) {
		rt_dequeue_get_next_thread();
		rt_schedule(0);
		local_irq_disable_hw();		
	}

	/*
	 * When the quota for the current thread is enabled,
	 * we have to start the group timer for this thread.
	 * Note: If the remaining quota value is small enough then the
	 * HW-timer will not be armed, the quota_timer_isr() will be
	 * invoked synchronously.
	 */
	ptg = current->rt_thread_group;
	if (RT_QUOTA_IS_ENABLED(ptg)) {
		set_quota_timer(RT_QUOTA_GROUP_TIMER, pqh, ptg, FORCE_SCHED);
	}
	local_irq_enable_hw();
}


/*
 * Stop quota handling.
 * Deactivate the quota interval timer.
 * Note: Interrupts must be off.
 */
void quota_interval_stop(rt_tgroup_head_t *pqh)
{
	if (!pqh->quota_enabled)
		return;

	rt_release_quota_timer();
	pqh->quota_enabled = 0;
}

/*
 * Invoked from rt_mark_exit_pending().
 * Assumption: p != current
 * Note: Interrupts must be off.
 */
int rt_remove_thread_from_group(struct task_struct *p)
{
	rt_tgroup_head_t *pqh = rtx_get_cpu_var(pquota_head);

	if (!p->rt_thread_group)
		return 0;

	p->rt_thread_group = NULL;
	pqh->quota_threads--;
	list_del_init(&p->rt_thread_list);
	return rt_shift_single_thread_to_runq(rtx_get_cpu_var(prt_blocked_queue), p);
}



/*
 * Implementation of the thread group API functions.
 */

/*
 * Create a new thread group for quota handling.
 * Note: The aspect parameter is already checked in the glibc.
 */
int rt_tgroup_create(tgroup_t *group, int aspect)
{
	int i;
	rt_tgroup_t *ptg;
	tgroup_t gid;
	rt_tgroup_head_t *pqh = rtx_get_cpu_var(pquota_head);

	down_rt(&__raw_get_cpu_var(tgroup_sem));
	local_irq_disable_hw();
	for (i = 0; i < RT_TGROUP_MAX; i++) {
		ptg = &__raw_get_cpu_var(tgroup_arr[i]);
		if (ptg->id)
			continue;

		/* Allocate a new group-id and pass it
		 * to the user application.
		 */
		gid = (tgroup_t)(RT_TGROUP_BASE_ID | i);
		if (rt_copy_to_user(group, &gid, sizeof(*group))) {
			up_rt(&__raw_get_cpu_var(tgroup_sem));
			return -EFAULT;
		}
		ptg->id = gid;
		ptg->quota = RT_TGROUP_QUOTA_OFF;
		ptg->aspect = aspect;
		ptg->quota_peak = RT_TGROUP_QUOTA_OFF;
		ptg->quota_ns = pqh->quota_interval_ns;
		ptg->quota_peak_ns = pqh->quota_interval_ns;
		ptg->quota_rem_ns = pqh->quota_interval_ns;
		ptg->quota_credit_ns = 0;
		ptg->quota_credit_acc_ns = 0;
		ptg->quota_start_ns = 0;
		INIT_LIST_HEAD(&ptg->thread_list);
		break;
	}
	local_irq_enable_hw();

	/* The limit of thread groups may be exceeded. */
	if (i == RT_TGROUP_MAX) {
		up_rt(&__raw_get_cpu_var(tgroup_sem));
		return -ENOMEM;
	}
	up_rt(&__raw_get_cpu_var(tgroup_sem));
	return 0;
}	

/* Delete a thread group. */
int rt_tgroup_delete(tgroup_t group)
{
	int idx;
	rt_tgroup_t *ptg;
	struct list_head *lp, *head;
	struct task_struct *proc;
	rt_tgroup_head_t *pqh = rtx_get_cpu_var(pquota_head);
	int threads_shifted = 0;

	if (!RT_TGROUP_IS_VALID(group))
		return -EINVAL;

	idx = RT_TGROUP_TO_INDEX(group);
	ptg = &__raw_get_cpu_var(tgroup_arr[idx]);

	down_rt(&__raw_get_cpu_var(tgroup_sem));

	if (ptg->id != group) {
		up_rt(&__raw_get_cpu_var(tgroup_sem));
		return -EINVAL;
	}
	ptg->id = 0;		// first free the slot

	/* Now it is not possible to assign a new quota to this group
	 * whenever a quota assignment interrupt occurs. */
	ptg->quota = 0;
	ptg->quota_peak = 0;

	/* Remove all the threads from this group. */
	head = &ptg->thread_list;
	local_irq_disable_hw();
	lp = head->next;

	/*
	 * A thread may have already called exit in LX. In this case
	 * it has removed itself from the thread group in the shutdown sequence.
	 */
	while(lp != head)
	{
		proc = (struct task_struct *)list_entry(lp, struct task_struct, rt_thread_list);
		proc->rt_thread_group = NULL;
		pqh->quota_threads--;
		lp = lp->next;
		list_del_init(&proc->rt_thread_list);	// unchain

		if (current == proc) {
			set_quota_timer(RT_QUOTA_INTERVAL_TIMER, pqh, ptg, 0);
		}
		else {
			/* Check if we have to move the thread from the blocked queue to the run queue.
			 * Note: proc != current must be TRUE in any case. */
			threads_shifted |= rt_shift_single_thread_to_runq(rtx_get_cpu_var(prt_blocked_queue), proc);
		}
	}
	local_irq_enable_hw();

	up_rt(&__raw_get_cpu_var(tgroup_sem));

	/* Maybe there is a new thread set in rt_req_task. */
	if (threads_shifted)
		rt_schedule(0);
	return 0;
}

/* Add a thread to the thread group. */
int rt_tgroup_thread_add(tgroup_t group, pid_t pid)
{
	int idx, ret;
	rt_tgroup_t *ptg;
	struct task_struct *p;
	rt_tgroup_head_t *pqh = rtx_get_cpu_var(pquota_head);
	int thread_shifted = 0;

	if (!RT_TGROUP_IS_VALID(group))
		return -EINVAL;

	idx = RT_TGROUP_TO_INDEX(group);
	ptg = &__raw_get_cpu_var(tgroup_arr[idx]);

	down_rt(&__raw_get_cpu_var(tgroup_sem));
	if (ptg->id != group) {
		up_rt(&__raw_get_cpu_var(tgroup_sem));
		return -EINVAL;
	}

	/* We do not allow to add a LX-thread to a RT thread group. */
	if ((p = rt_find_task_by_pid(pid)) == NULL || IS_PID_LIST_LEAVE_PENDING(p)) {
		up_rt(&__raw_get_cpu_var(tgroup_sem));
		return -EINVAL;
	}

	if ((ret = thread_is_group_member(group, p))) {
		up_rt(&__raw_get_cpu_var(tgroup_sem));
		if (ret > 0)
			return 0;		// thread is already in this group
		else
			return -EBUSY;	// thread is already in another group
	}

	/* Add the thread to this thread group. */
	local_irq_disable_hw();
	p->rt_thread_group = ptg;
	list_add_tail(&p->rt_thread_list, &ptg->thread_list);
	pqh->quota_threads++;

	/*
	 * Activate quota interval for the current thread.
	 */
	if (p == current) {
		if (ptg->quota) {
			if (ptg->quota < RT_TGROUP_QUOTA_OFF)
				set_quota_timer(RT_QUOTA_GROUP_TIMER, pqh, ptg, 0);
		}
		else {
			/* The current thread must give up the CPU. */
			current->rt_state2 |= RT_TASK_QUOTA_EXCEEDED;

			/* Shifts the current thread from run queue into the quota queue. */
			rt_dequeue_get_next_thread();
			thread_shifted++;
		}
	}
	local_irq_enable_hw();

	up_rt(&__raw_get_cpu_var(tgroup_sem));

	if (thread_shifted)
		rt_schedule(0);

	return 0;
}

/* Remove a thread from the thread group. */
int rt_tgroup_thread_remove(tgroup_t group, pid_t pid)
{
	int idx, ret;
	rt_tgroup_t *ptg;
	struct task_struct *proc;
	rt_tgroup_head_t *pqh = rtx_get_cpu_var(pquota_head);
	int threads_shifted = 0;

	if (!RT_TGROUP_IS_VALID(group))
		return -EINVAL;

	idx = RT_TGROUP_TO_INDEX(group);
	ptg = &__raw_get_cpu_var(tgroup_arr[idx]);

	down_rt(&__raw_get_cpu_var(tgroup_sem));
	if (ptg->id != group) {
		up_rt(&__raw_get_cpu_var(tgroup_sem));
		return -EINVAL;
	}

	/*
	 * Here we will find threads which are
	 * - executed in the RT-domain
	 * - temporarily executing in LX
	 * - which are on the way to LX because of a priority change.
	 * Note: We cannot remove a thread from the thread group
	 * which has migrated to LX because of a priority change.
	 */
	if ((proc = rt_find_task_by_pid(pid)) == NULL) {
		up_rt(&__raw_get_cpu_var(tgroup_sem));
		return -EINVAL;
	}

	ret = thread_is_group_member(group, proc);

	/* Check if thread is in this group. */
	if (!ret || ret == -1) {
		up_rt(&__raw_get_cpu_var(tgroup_sem));
		return 0;		// no
	}

	local_irq_disable_hw();
	list_del_init(&proc->rt_thread_list);
	proc->rt_thread_group = NULL;
	pqh->quota_threads--;

	if (proc == current) {
		if (ptg->quota_start_ns)
			threads_shifted= quota_thread_stop(pqh, ptg);
	}
	else {
		/* Move the thread from the blocked queue to the run queue. */
		threads_shifted = rt_shift_single_thread_to_runq(rtx_get_cpu_var(prt_blocked_queue), proc);
	}
	local_irq_enable_hw();
	up_rt(&__raw_get_cpu_var(tgroup_sem));

	/* Maybe there is a new thread set in rt_req_task. */
	if (threads_shifted)
		rt_schedule(0);

	return 0;
}

/* Set the processing time quota for a thread group.
 * A new quota value must be effective immediately,
 * especially if the current thread is concerned.
 *
 * We do not consider the fact that the threads of the
 * thread group concerned have already consumed quota
 * in the current assignment interval.
 * Note: There is one special value to be considered
 * - quota/quota_peak>=100: Disable quota handling for the group
 */
int rt_tgroup_setquota(tgroup_t group, int quota, int quota_peak)
{
	int idx, quota_sum;
	rt_tgroup_t *ptg;
	rt_tgroup_head_t *pqh = rtx_get_cpu_var(pquota_head);
	int threads_shifted = 0;

	if (!RT_TGROUP_IS_VALID(group))
		return -EINVAL;

	idx = RT_TGROUP_TO_INDEX(group);
	ptg = &__raw_get_cpu_var(tgroup_arr[idx]);

	/* Check quota value. */
	if (quota == RT_TGROUP_QUOTA_OFF) {
		quota_peak = quota;
	}
	else {
		if (quota < PROCESSING_TIME_QUOTA_MIN)
			quota = PROCESSING_TIME_QUOTA_MIN;

		if (quota > RT_TGROUP_QUOTA_OFF)
			quota = RT_TGROUP_QUOTA_OFF;
		else
			if (quota > PROCESSING_TIME_QUOTA_MAX)
				quota = PROCESSING_TIME_QUOTA_MAX;
	}

	/*
	 * The quota_peak value decides on credit accumulation.
	 * If quota and quota_peak have the same value no credit
	 * accumulation will be supported.
	 */
	if (quota_peak > PROCESSING_TIME_QUOTA_MAX)
		quota_peak = PROCESSING_TIME_QUOTA_MAX;
	if (quota_peak < quota)
		quota_peak = quota;

	down_rt(&__raw_get_cpu_var(tgroup_sem));
	if (ptg->id != group) {
		up_rt(&__raw_get_cpu_var(tgroup_sem));
		return -EINVAL;
	}

	/* Set a new quota peak value. */
	ptg->quota_peak = quota_peak;

	/* When the new quota is the same as the old one,
	 * we have nothing to do. */
	if (ptg->quota == quota) {
		quota_sum = get_quota_sum();
		up_rt(&__raw_get_cpu_var(tgroup_sem));
		return quota_sum;
	}
	local_irq_disable_hw();

	/* Set a new quota value. */
	ptg->quota = quota;
	ptg->quota_ns = (pqh->quota_interval_ns * quota) / 100;
	ptg->quota_rem_ns = ptg->quota_ns;
	ptg->quota_peak_ns = (pqh->quota_interval_ns * quota_peak) / 100;

	if (quota_peak != quota)
		ptg->quota_credit_ns = ((pqh->quota_interval_ns * quota_peak) / 100) - ptg->quota_ns;
	else {
		ptg->quota_credit_ns = 0;
	}

	/*
	 * For the new settings we don't take into account an already consumed quota
	 * in the current assignment interval.
	 * The current thread is member of a thread group, the quota of which will be changed.
	 */
	if (current->rt_thread_group && current->rt_thread_group->id == group) {
		if (!ptg->quota) {		// quota is blocked
			rt_shift_all_threads_to_blockedq(&ptg->thread_list, current);
			current->rt_state2 |= RT_TASK_QUOTA_EXCEEDED;
			threads_shifted = 1;

			/* When the group timer is running we have to stop it and
			 * to replace it by the interval timer. Otherwise, the
			 * interval timer is already activated.
			 */
			set_quota_timer(RT_QUOTA_INTERVAL_TIMER, pqh, ptg, 0);
		}
		else
			if (ptg->quota == RT_TGROUP_QUOTA_OFF) {	// quota off
				/* In this case a group timer must be active.
				 * So replace it by the interval timer. */
				set_quota_timer(RT_QUOTA_INTERVAL_TIMER, pqh, ptg, 0);
		} else {	// active quota
			/* In this case a group timer may be active.
			 * So replace it by a new group timer. */
			set_quota_timer(RT_QUOTA_GROUP_TIMER, pqh, ptg, 0);
		}
	}
	else {
		/*
		 * The current thread is not member of a thread group or
		 * the current thread is member of another thread group,
		 * the quota of which will not be changed.
		 */
		if (!ptg->quota) { // quota exceeded
			if (!list_empty(&ptg->thread_list))
				threads_shifted = rt_shift_all_threads_to_blockedq(&ptg->thread_list, NULL);
		}
		else  { // quota off or active
			/* Shift all the threads of this group from the blocked queue to the run queue.
			 * Note: The group may be empty. */
			if (!list_empty(&ptg->thread_list))
				threads_shifted = rt_shift_all_threads_to_runq(rtx_get_cpu_var(prt_blocked_queue), ptg, RT_QUOTA_FORCE_ALL);
		}
	}
	local_irq_enable_hw();

	quota_sum = get_quota_sum();

	up_rt(&__raw_get_cpu_var(tgroup_sem));

	/* Maybe there is a new thread set in rt_req_task. */
	if (threads_shifted) {
		local_irq_disable_hw();
		/* This will dequeue the current thread in rt_schedule()
		 * and will shift it to the blocked queue. */
		if (current->rt_state2 & RT_TASK_QUOTA_EXCEEDED)
			rt_dequeue_get_next_thread();
		rt_schedule(0);		// enables interrupts implicitly
	}
	return quota_sum;
}

/* Get the processing time quota for a thread group
 * and also return the quota sum of all thread groups. */
int rt_tgroup_getquota(tgroup_t group, int *quota, int *quota_peak)
{
	int idx, quota_sum;
	rt_tgroup_t *ptg;

	if (!RT_TGROUP_IS_VALID(group))
		return -EINVAL;

	idx = RT_TGROUP_TO_INDEX(group);
	ptg = &__raw_get_cpu_var(tgroup_arr[idx]);

	down_rt(&__raw_get_cpu_var(tgroup_sem));
	if (ptg->id != group) {
		up_rt(&__raw_get_cpu_var(tgroup_sem));
		return -EINVAL;
	}
	if (rt_copy_to_user(quota, &ptg->quota, sizeof(quota))) {
		up_rt(&__raw_get_cpu_var(tgroup_sem));
		return -EFAULT;
	}
	if (rt_copy_to_user(quota_peak, &ptg->quota_peak, sizeof(quota_peak))) {
		up_rt(&__raw_get_cpu_var(tgroup_sem));
		return -EFAULT;
	}
	quota_sum = get_quota_sum();
	up_rt(&__raw_get_cpu_var(tgroup_sem));
	return quota_sum;
}
#endif /* CONFIG_RTX_THREAD_GROUP_SUPPORT */

/* Common thread group function, called from ioctl. */
int rt_tgroup_common(unsigned int cmd, tgroup_descr_t *tgrp_descr)
{
#ifdef CONFIG_RTX_THREAD_GROUP_SUPPORT
	rt_tgroup_head_t *pqh;
	int ret;

	pqh = rtx_get_cpu_var(pquota_head);

	/* We are in a shutdown sequence? */
	if (rtx_get_cpu_var(rt_system_state) & RT_SHUTDOWN || !pqh->quota_enabled)
		return -ENOSYS;

	current->rt_state2 |= RT_SYS_QUOTA_HANDLING;

	switch (cmd) {
    case AuD_TGROUP_CREATE:
    	ret = rt_tgroup_create(tgrp_descr->pgroup, tgrp_descr->aspect);
    	break;
    case AuD_TGROUP_DELETE:
    	ret =  rt_tgroup_delete(tgrp_descr->group);
    	break;
    case AuD_TGROUP_THREAD_ADD:
    	ret =  rt_tgroup_thread_add(tgrp_descr->group, tgrp_descr->pid);
    	break;
    case AuD_TGROUP_THREAD_REMOVE:
    	ret =  rt_tgroup_thread_remove(tgrp_descr->group, tgrp_descr->pid);
    	break;
    case AuD_TGROUP_SETQUOTA:
    	ret =  rt_tgroup_setquota(tgrp_descr->group, tgrp_descr->quota, tgrp_descr->quota_peak);
    	break;
    case AuD_TGROUP_GETQUOTA:
    	ret =  rt_tgroup_getquota(tgrp_descr->group, tgrp_descr->pquota, tgrp_descr->pquota_peak);
    	break;
    default:
    	current->rt_state2 &= ~(RT_SYS_QUOTA_HANDLING | RT_DEFER_QUOTA_HANDLING);
    	return -ENOSYS;
	}
	/* A quota interrupt causes this thread (current) to give up the CPU because its
	 * quota is exceeded, but this thread was currently executing a RT quota system call.
	 * So the handling must be deferred until the tgroup_sem has been released.
	 */
	if (current->rt_state2 & RT_DEFER_QUOTA_HANDLING) {
		local_irq_disable_hw();
		/* The quota for this thread is still exceeded. */
		rt_dequeue_get_next_thread();

		/* Start the interval timer for the remaining time (end of interval). */
		rt_set_quota_timer(pqh->quota_interval_rem_ns, quota_timer_isr, NULL, pqh, 0);
		rt_schedule(0);			// enables HW interrupts implicitly
	}
	current->rt_state2 &= ~(RT_SYS_QUOTA_HANDLING | RT_DEFER_QUOTA_HANDLING);
	return ret;
#else
	return -ENOSYS;
#endif
}

#ifdef CONFIG_RTX_THREAD_GROUP_SUPPORT
/*
 * The ISR for the quota timer.
 * It is invoked when
 * - the quota assignment interval ends
 * - the quota of a thread group is exceeded
 * Both events may overlap.
 * Note: Interrupts are off.
 */
int quota_timer_isr(void *arg)
{
	int i;
	rt_tgroup_t *ptg, *ptg_curr_thread;
	unsigned long period;
	long rem_quota_ns_new;
	int group_active = 0;
	RTX_TIMER_TYPE curr_time = RTX_TIMER_GET;
	rt_tgroup_head_t *pqh = rtx_get_cpu_var(pquota_head);

	/* In case of a shutdown quota handling may be disabled. */
	if (!pqh->quota_enabled) {
		rt_shift_all_threads_to_runq(rtx_get_cpu_var(prt_blocked_queue), NULL, RT_QUOTA_FORCE_ALL);
		return 0;
	}

	/* Check for the end of the quota interval */
	if ((period = (unsigned long)(curr_time - pqh->quota_start_ns)) >= pqh->quota_interval_ns) {
		/* The current thread is member of a thread group and the group timer is started.
		 * A new quota will be computed for this group. */
		if ((ptg_curr_thread = current->rt_thread_group) && ptg_curr_thread->quota_start_ns) {
			group_active = 1;
			ptg_curr_thread->quota_rem_ns -= (unsigned long)(curr_time - ptg_curr_thread->quota_start_ns);  // will be set below
			ptg_curr_thread->quota_start_ns = 0;
		}

		/*
		 * Now all the threads of the blocked queue have to be shifted to the run queue.
		 * Keep threads in the blocked queue which are member of a thread group which is currently blocked.
		 */
		rt_shift_all_threads_to_runq(rtx_get_cpu_var(prt_blocked_queue), NULL, RT_QUOTA_KEEP_ZERO_BLOCKED); // also removes EXCEEDED flags

		/* Assign a new quota to all thread groups.
		 * The quota credit has also to be considered. */
		for (i = 0; i < RT_TGROUP_MAX; i++) {
			ptg = &__raw_get_cpu_var(tgroup_arr[i]);
			if (ptg->id && !list_empty(&ptg->thread_list) && RT_QUOTA_IS_ENABLED(ptg)) {
				if (ptg->quota != ptg->quota_peak) {
					/* A credit for this group will only be accumulated
					 * when there is at least one thread in the group
					 * which is currently ready-to-run. */
					rem_quota_ns_new = ptg->quota_rem_ns + ptg->quota_ns;
					if (group_has_threads_ready_to_run(ptg)) {
						if (rem_quota_ns_new > ptg->quota_peak_ns) {
							ptg->quota_credit_acc_ns += rem_quota_ns_new - ptg->quota_peak_ns;
							ptg->quota_rem_ns = ptg->quota_peak_ns;
						}
						else {
							long credit_to_consume_ns = 0;
							if (ptg->quota_credit_acc_ns) {
								credit_to_consume_ns = ptg->quota_peak_ns - rem_quota_ns_new;
								if (ptg->quota_credit_acc_ns >= credit_to_consume_ns) {
									ptg->quota_credit_acc_ns -= credit_to_consume_ns;
								}
								else {
									credit_to_consume_ns = ptg->quota_credit_acc_ns;
									ptg->quota_credit_acc_ns = 0;
								}
							}
							ptg->quota_rem_ns = rem_quota_ns_new + credit_to_consume_ns;
						}
					}
					else {
						/* An accumulated credit will get lost. */
						ptg->quota_rem_ns = ptg->quota_ns;
						ptg->quota_credit_acc_ns = 0;
					}

				}
				else {
					ptg->quota_rem_ns = ptg->quota_ns;
				}
			}
		}

		pqh->quota_start_ns = RTX_TIMER_GET;

		/* For an active quota thread we have to start the group timer. */
		if (RT_QUOTA_IS_ENABLED(ptg_curr_thread) && IS_REALTIME && group_active) {
			ptg_curr_thread->quota_start_ns = pqh->quota_start_ns;
			rt_set_quota_timer(ptg_curr_thread->quota_rem_ns, quota_timer_isr, current, pqh, 0);
		}
		else {
			/* Start the next assignment interval. */
			rt_set_quota_timer(pqh->quota_interval_ns, quota_timer_isr, NULL, pqh, 0);
		}
	}
	else {
		/* The quota of the current thread is used up.
		 * Prevent the threads of this thread group from being scheduled. */
#ifndef RTX_OPTIMIZE_FOR_RELEASE
		BUG_ON(!IS_REALTIME);
#endif
		/* The interrupt may occur while a group_delete()
		 * has already deleted the thread group this thread belongs to. */
		if (!(ptg = current->rt_thread_group)) {
			/* Start the interval timer for the remaining time (end of interval). */
			rt_set_quota_timer(pqh->quota_interval_ns - period, quota_timer_isr, NULL, pqh, 0);
			return 0;
		}

		if (current->rt_state2 & RT_SYS_QUOTA_HANDLING)
			current->rt_state2 |= RT_DEFER_QUOTA_HANDLING;
		ptg->quota_rem_ns = 0;		// quota exceeded
		ptg->quota_start_ns = 0;

		/* Move all the threads of this group from run queue to the blocked queue.
		 * Prepare the current thread to be scheduled away. */
		rt_shift_all_threads_to_blockedq(&ptg->thread_list, current);
		current->rt_state2 |= RT_TASK_QUOTA_EXCEEDED;

		pqh->quota_interval_rem_ns = pqh->quota_interval_ns - period;
		if (!(current->rt_state2 & RT_DEFER_QUOTA_HANDLING) ) {
			/* We dequeue the current thread explicitly, because we
			 * invoke rtx_preemption_handling() next (not rt_schedule()). */
			rt_dequeue_get_next_thread();

			/* Start the interval timer for the remaining time (end of interval). */
			rt_set_quota_timer(pqh->quota_interval_rem_ns, quota_timer_isr, NULL, pqh, 0);
		}
	}
	return 0;
}
#endif
