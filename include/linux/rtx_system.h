/*
 * linux/rtx_system.h
 *
 * Copyright (c) 2009 Siemens AG
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
 */
#ifndef _RTX_SYSTEM_H_
#define _RTX_SYSTEM_H_
#include <linux/mqueue.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <linux/irq.h>
#include <linux/utsname.h>

#include <linux/aud/rt_base.h>
#include <asm/rtx_adapt.h>

#ifndef __ASSEMBLY__
extern int rtx_oops;
extern int rtx_mode_rt;

extern int to_rt_virq;				// irq for waking up the realtime domain
extern int to_lx_virq;
extern int to_mg_virq;
extern int wake_up_lx_virq;
extern int printk_virq;

void __rtx_wake_lx_handling(struct proc_wait_queue *);
void rtx_preemption_handling(void);
long rtx_clock_gettime_internal(struct timespec *);
void __rtx_call_rt_domain(int);

#define RTX_SYS_IS_RT_DOMAIN	(rtx_get_cpu_member(rtx_percpu_data, rtx_curr_dom) == RT_DOMAIN)

DECLARE_PER_CPU(struct task_struct *, lx_curr_task);

static inline void rtx_sys_check_sched_integrity(void)
{
#ifdef CONFIG_RTX_DOMAIN_INTEGRITY_CHECK
	if (rtx_get_cpu_var(lx_curr_task)) {
		panic("%s: LX-task error (current=%d lx_curr_task=%d)\n", __func__, current->pid, rtx_get_cpu_var(lx_curr_task)->pid);
	}
#endif
}
#endif /* __ASSEMBLY__ */
#endif /* _RTX_SYSTEM_H_ */
