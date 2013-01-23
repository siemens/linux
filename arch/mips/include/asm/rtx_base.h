/*
 * arch/mips/include/asm/rtx_base.h
 *
 * Copyright (c) 2011 Siemens AG
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

#ifndef _ARCH_MIPS_INCLUDE_ASM_RTX_BASE_H_
#define _ARCH_MIPS_INCLUDE_ASM_RTX_BASE_H_

#include <asm/mach-aud_soc/irq.h>

#include <linux/autoconf.h>
#include <asm/bitsperlong.h>
#include <linux/irqreturn.h>

struct irq_desc;

#define RTX_VIRQ_BASE		NR_IRQS
#define RTX_NR_VIRQS		5
#define RTX_NR_IRQS		(NR_IRQS + RTX_NR_VIRQS)

#define TO_RT_VIRQ				(RTX_VIRQ_BASE+0)
#define TO_MG_VIRQ				(RTX_VIRQ_BASE+1)
#define TO_LX_VIRQ				(RTX_VIRQ_BASE+2)
#define WAKE_UP_LX_VIRQ				(RTX_VIRQ_BASE+3)
#define PRINTK_VIRQ				(RTX_VIRQ_BASE+4)

// rt one shot timer: overtake max value from mips linux implementation
// (see kernel/rtx_adapt.c)
#define RTX_MAX_DELTA_NS		0

/* Reserved for internal use. */
#define LX_IRQS_STALLED			0	// LX interrupts are stalled (must no be processed immediately)

#define LX_IRQS_STALLED_MASK	(1L << LX_IRQS_STALLED)

#define RTX_IRQ_WORDS		((RTX_NR_IRQS + BITS_PER_LONG -1) / BITS_PER_LONG)
#define RTX_IRQ_WORDS_SHIFT	5			// for 32 bit system
#define RTX_IRQ_MASK		(BITS_PER_LONG -1)

#define RTX_IRQMASK_ANY			(~0L)
#define RTX_IRQMASK_VIRT		(RTX_IRQMASK_ANY << (RTX_VIRQ_BASE / BITS_PER_LONG))

typedef unsigned long irqset_t[RTX_IRQ_WORDS];

typedef irqreturn_t (*rtx_handler_t)(int, void *);
//typedef void (*rtx_handler_lx_t)(unsigned irq, void *cookie);
//typedef void (*rtx_ack_lx_t)(unsigned irq, struct irq_desc *desc);

#ifndef __ASSEMBLY__
struct isr_cntrl {
	rtx_handler_t handler;		// RT handler and VIRQ-Handler
	void *cookie;                   // dev_id
	unsigned flag;
};
#endif

extern void __rtx_handle_nonrt_pending(unsigned long sync_mask);
extern void __rtx_local_irq_enable(void);

extern void __rtx_pin_range_globally(unsigned long, unsigned long);

#endif /* _ARCH_MIPS_INCLUDE_ASM_RTX_BASE_H_ */
          
