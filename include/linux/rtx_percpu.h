/*
 * linux/rtx_percpu.h
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
#ifndef _RTX_PERCPU_H_
#define _RTX_PERCPU_H_
#include <linux/irqreturn.h>
#include <asm/percpu.h>
#include <asm/rtx_base.h>

#ifndef __ASSEMBLY__
/*
 * Per CPU data
 */
struct rtx_pcd {
	unsigned long rtx_irq_control;			// Note: Must be the first element (see arch/../irqflags.h)
	unsigned long rtx_irq_pending_words;
	irqset_t rtx_irq_pending;
	unsigned long rtx_softirq_pending_words;
	irqset_t rtx_softirq_pending;
	int rtx_curr_dom;
	struct isr_cntrl rtx_irqs[RTX_NR_IRQS];
};

DECLARE_PER_CPU(struct rtx_pcd, rtx_percpu_data);

#define rtx_percpu_var(var, cpu)	((&per_cpu(rtx_percpu_data, cpu))->var)
#define rtx_get_cpu_data(var)	(rtx_get_cpu_member(rtx_percpu_data, var))

#define rtx_pending_irqs(pcd)	(unlikely(pcd->rtx_irq_pending_words != 0) || unlikely(pcd->rtx_softirq_pending_words != 0))
#endif /* __ASSEMBLY__ */
#endif /* _RTX_PERCPU_H_ */
