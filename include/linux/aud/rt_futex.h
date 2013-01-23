/*
 * linux/aud/rt_futex.h
 *
 * Contributed by <wolfgang.hartmann@siemens.com>, 2010
 * Copyright (c) 2010 Siemens AG
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
 * Futex specific definitions.
 */
#ifndef _LINUX_AUD_RT_FUTEX_H
#define _LINUX_AUD_RT_FUTEX_H

#define FUTEX_HASHBITS 8

extern const struct sched_class *rtx_get_fair_sched_class(void);
extern const struct sched_class *rtx_get_rt_sched_class(void);

extern int rt_futex_getprio(struct task_struct *);
extern int rt_fixup_owner_prio(unsigned long uaddr, struct task_struct *owner_task);

struct futex_hash_bucket {
       struct list_head       chain;
};

static inline int rt_task_has_pi_waiters(struct task_struct *p)
{
	return !list_empty(&p->rt_pi_waiters);
}


#endif /* _LINUX_AUD_RT_FUTEX_H */
