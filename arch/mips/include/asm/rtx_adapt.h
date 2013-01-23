/*
 * arch/mips/include/asm/rtx_adapt.h
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
#ifndef _ARCH_MIPS_INCLUDE_ASM_RTX_ADAPT_H_
#define _ARCH_MIPS_INCLUDE_ASM_RTX_ADAPT_H_

/* MIPS traps */
/* see genex.S, tlbex.c, tlbex-fault.S */
#define RTX_TRAP_UNKNOWN 	 0	/* dummy */
#define RTX_TRAP_ACCESS 	 1	/* tlb / page fault exception (1, 2, 3) */
#define RTX_TRAP_ALIGNMENT 	 2	/* aligned error exception (4 and 5) */
#define RTX_TRAP_BE	 	 3	/* bus error exception (6 and 7) */
/* vector 8 is system call interface we interact there */
#define RTX_TRAP_BREAK	 4	/* break exception (9) */
/* used vector 10 ri / rdhwr directly - see also below RTX_TRAP_RI */
#define RTX_TRAP_CPU		 5	/* cpu exception (11) */
#define RTX_TRAP_FPE_OV	 6	/* fpe_ov exception (12) */
#define RTX_TRAP_DEBUG	 7	/* trap / debug exception (13) */
#define RTX_TRAP_FPE		 8	/* floating point exception (15) */
#define RTX_TRAP_MDMX		 9	/* MDMX exception (22) */
#define RTX_TRAP_MCE		10	/* Machine check exception (24) */
#define RTX_TRAP_MT		11	/* MT Thread exception (25) */
#define RTX_TRAP_DSP		12	/* MT Thread exception (26) */
#define RTX_TRAP_RI 		13	/* reserved instruction exception */
#define RTX_TRAP_RESERVED 	14	/* reserved number can be used */

extern int __rtx_trap_handling(int vector, struct pt_regs *regs, unsigned mode);

/* Call virtual interrupt handler */
static inline void __rtx_call_lx_virq_handler(unsigned irq,
						  rtx_handler_t handler,
						  void *cookie)
{
   irq_enter();
   handler(irq,cookie);
   irq_exit();
}

#ifdef CONFIG_RTX_DOMAIN_INTEGRITY_CHECK
static inline void rtx_check_intr_flag(void)
{
#if (0)
		unsigned long chk_flags;
		local_save_flags_hw(chk_flags);
		if (chk_flags & 0x200)				// Interrupt-Flag
                  panic("%s: HW must be off (flags=%#lx)\n", __func__, chk_flags);
#endif

}
#endif

/*
 * The memory for the RT extensions can be allocated in an architecture specific way.
 */
extern void kfree(const void *);
#define KMALLOC kmalloc
#define KFREE	kfree

/*
 * Disable HW interrupts for context switch.
 * See context_switch()/sched.c
 */
#define prepare_arch_switch(next) \
	do {					\
		local_irq_disable_hw();		\
	} while (0)

#define IS_RTX_TIMER_IRQ(irq)		(irq == __rtx_timer_irq)	// legacy timer or HPET
#define rtx_processor_id()		raw_smp_processor_id()

/* get the real interrupt status of the processor */
#define __rtx_root_tick_p(regs) (!raw_irqs_disabled_flags(regs->cp0_status))
#define rtx_update_tick_evtdev(evtdev) do { } while (0)

/* Mark an interrupt as pending.
 * Note: HW interrupts must be off. */
#define log_interrupt_as_pending(irq) \
({ \
	struct rtx_pcd *pcd = &__raw_get_cpu_var(rtx_percpu_data); \
	prefetchw(pcd); \
	__set_bit(irq & RTX_IRQ_MASK, &pcd->rtx_irq_pending[irq >> RTX_IRQ_WORDS_SHIFT]); \
	__set_bit(irq >> RTX_IRQ_WORDS_SHIFT, &pcd->rtx_irq_pending_words); \
})

#define log_softirq_as_pending(irq) \
({ \
	struct rtx_pcd *pcd = &__raw_get_cpu_var(rtx_percpu_data); \
	prefetchw(pcd); \
	__set_bit(irq & RTX_IRQ_MASK, &pcd->rtx_softirq_pending[irq >> RTX_IRQ_WORDS_SHIFT]); \
	__set_bit(irq >> RTX_IRQ_WORDS_SHIFT, &pcd->rtx_softirq_pending_words); \
})

/*
 * Per CPU access definitions
 */
#ifdef CONFIG_SMP

/* Native atomic access for mips necessary: T B D */

#else /* ! SMP */

#include <asm-generic/percpu.h>

#define rtx_get_cpu_var(var)		__get_cpu_var(var) 
#define rtx_set_cpu_var(var, val)	(__get_cpu_var(var) = val)
#define rtx_and_cpu_var(var, val)	(__get_cpu_var(var) &= val)
#define rtx_or_cpu_var(var, val)	(__get_cpu_var(var) |= val)
#define rtx_add_cpu_var(var, val)	(__get_cpu_var(var) += val)

#define rtx_get_cpu_member(var, field)		__get_cpu_var(var.field)
#define rtx_set_cpu_member(var, field, val)	(__get_cpu_var(var.field) = val)

#define rtx_per_cpu(variable, cpu)		per_cpu(variable, cpu)

#endif

/* Floating point */
#define RTX_UNLAZY_FPU(tsk)		do { } while (0);

/* System call */
#define RTX_SYSCALL_ID()		current->rt_syscall_id
#define RTX_SYSCALL_SETPGID		4057		/* system call number */

#endif /* _ARCH_MIPS_INCLUDE_ASM_RTX_ADAPT_H_ */
          
