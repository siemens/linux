/*
 * drivers/aud/auddevice/rt_semaphore.c
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
 * This file contains the functions for rt kernel internal semaphores.
 * 
 */

#include <linux/aud/rt_timer.h>

void sema_init_rt(struct rt_semaphore *sem, int val)
{
	sem->count = val;
	INIT_LIST_HEAD(&sem->wait.list);
}	


void up_rt(struct rt_semaphore *sem)
{
	struct rt_wait *wq;
	unsigned long flags;
	
	local_irq_save_hw(flags);
	if (list_empty(&sem->wait.list))
	{
		sem->count++;
		local_irq_restore_hw(flags);
		return;
	}
	wq = list_entry(sem->wait.list.next, struct rt_wait, list);
	list_del_init(&wq->list);
	wq->busy = 0;
	rt_wake_up_thread(wq->proc);
	local_irq_restore_hw(flags);
	rt_schedule(0);
}

/* Special up_rt() without rt_schedule(). */
void up_rt_from_virq(struct rt_semaphore *sem)
{
	struct rt_wait *wq;
	unsigned long flags;

	local_irq_save_hw(flags);
	if (list_empty(&sem->wait.list))
	{
		sem->count++;
		local_irq_restore_hw(flags);
		return;
	}
	wq = list_entry(sem->wait.list.next, struct rt_wait, list);
	list_del_init(&wq->list);
	wq->busy = 0;
	rt_wake_up_thread(wq->proc);
	local_irq_restore_hw(flags);
}


int down_rt(struct rt_semaphore *sem)
{
	struct rt_wait wq;
	unsigned long flags;
	
	local_irq_save_hw(flags);
	if (sem->count > 0)
	{
		sem->count--;
		local_irq_restore_hw(flags);
		return 0;
	}	
	wq.proc = current;
	wq.busy = 1;
	INIT_LIST_HEAD(&wq.list);
	list_add_tail(&wq.list, &sem->wait.list);
	current->rt_state = RT_TASK_UNINTERRUPTIBLE | RT_TASK_SEM_CLEANUP;
	local_irq_restore_hw(flags);
	rt_schedule(0);
	local_irq_disable_hw();
	current->rt_state &= ~RT_TASK_SEM_CLEANUP;

	if (wq.busy) {
		list_del_init(&wq.list);
	}
	local_irq_restore_hw(flags);
	if (IS_EXIT_PENDING(current))
		rt_task_leave();
	return wq.busy;
}
