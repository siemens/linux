#ifdef __KERNEL__
#ifndef _ASM_POWERPC_IRQ_INTERNAL_H
#define _ASM_POWERPC_IRQ_INTERNAL_H

/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

/*
 * manfred.neugebauer@siemens.com
 * we moved the definitions out of irq.h since we have
 * definitions problems within the realtime headers
 * when including the complete irq.h
 */

/* This number is used when no interrupt has been assigned */
#define NO_IRQ			(0)

/* This is a special irq number to return from get_irq() to tell that
 * no interrupt happened _and_ ignore it (don't count it as bad). Some
 * platforms like iSeries rely on that.
 */
#define NO_IRQ_IGNORE		((unsigned int)-1)

/* Total number of virq in the platform (make it a CONFIG_* option ? */
#define NR_IRQS		512

/* Number of irqs reserved for the legacy controller */
#define NUM_ISA_INTERRUPTS	16

#endif /* _ASM_POWERPC_IRQ_INTERNAL_H */

#endif /* __KERNEL__ */

