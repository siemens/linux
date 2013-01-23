/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 *
 * Copyright (C) 2006 Ralf Baechle (ralf@linux-mips.org)
 */
#ifndef __ASM_IRQ_REGS_H
#define __ASM_IRQ_REGS_H

#define ARCH_HAS_OWN_IRQ_REGS

#include <linux/thread_info.h>

#ifdef CONFIG_RTX_DOMAIN
DECLARE_PER_CPU(struct pt_regs, rtx_tick_regs);
#endif

static inline struct pt_regs *get_irq_regs(void)
{
#ifdef CONFIG_RTX_DOMAIN
	struct pt_regs *tick_regs = &__raw_get_cpu_var(rtx_tick_regs);
	return tick_regs;
#else
	return current_thread_info()->regs;
#endif
}

#endif /* __ASM_IRQ_REGS_H */
