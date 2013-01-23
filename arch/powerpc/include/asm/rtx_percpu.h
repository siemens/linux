/*
 * arch/powerpc/include/asm/rtx_percpu.h
 *
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
 */
#ifndef _POWERPC_RTX_PERCPU_H_
#define _POWERPC_RTX_PERCPU_H_
#include <linux/rtx_percpu.h>
#include <asm/irqflags.h>

#ifdef CONFIG_SMP

#error "SMP not implemented."

#else
static inline void __rtx_set_stalled_bit(void)
{
	volatile struct rtx_pcd *p = &__raw_get_cpu_var(rtx_percpu_data);
	unsigned long flags;
	local_irq_save_hw(flags);
	p->rtx_irq_control |= (1<<LX_IRQS_STALLED);
	local_irq_restore_hw(flags);
}

static inline void __rtx_reset_stalled_bit(void)
{
	__rtx_local_irq_enable();
}

static inline unsigned long __rtx_test_and_set_stalled_bit(void)
{
	volatile struct rtx_pcd *p = &__raw_get_cpu_var(rtx_percpu_data);
	unsigned long flags, res;

	local_irq_save_hw(flags);
	res = p->rtx_irq_control;
	p->rtx_irq_control = res | (1<<LX_IRQS_STALLED);
	local_irq_restore_hw(flags);

	return res & (1<<LX_IRQS_STALLED);
}

static inline unsigned long __rtx_test_stalled_bit(void)
{
    volatile struct rtx_pcd *p = &__raw_get_cpu_var(rtx_percpu_data);
	return p->rtx_irq_control & (1<<LX_IRQS_STALLED);
}

static inline void __rtx_restore_stalled_bit(unsigned long unstalled)
{
    if (unstalled) __rtx_reset_stalled_bit();
    else __rtx_set_stalled_bit();
}

#endif

#endif /* _POWERPC_RTX_PERCPU_H_ */
