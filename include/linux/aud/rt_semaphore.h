/*
 * linux/aud/rt_semaphore.h
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
 * Semaphore specific definitions.
 */
#ifndef _LINUX_AUD_RT_SEMAPHORE_H
#define _LINUX_AUD_RT_SEMAPHORE_H

struct rt_wait {
	struct list_head list;
	struct task_struct *proc;
	int busy;
};
	
struct rt_wait_queue_head {
	struct list_head list;
};

struct rt_semaphore {
	int count;
	int sleepers;
	struct rt_wait_queue_head wait;
};

void up_rt(struct rt_semaphore *);
void up_rt_from_virq(struct rt_semaphore *);
int down_rt(struct rt_semaphore *);
void sema_init_rt(struct rt_semaphore *, int);
#endif /* _LINUX_AUD_RT_SEMAPHORE_H */
