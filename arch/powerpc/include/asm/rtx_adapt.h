/*
 * asm-powerpc/rtx_adapt.h
 *
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
 */
#ifndef _ASM_POWERPC_RTX_ADAPT_H_
#define _ASM_POWERPC_RTX_ADAPT_H_

/* Powerpc traps */
#define RTX_TRAP_UNKNOWN_EXCEPTION		1 /* Unknown exception */
#define RTX_TRAP_INSTR_BP_EXCEPTION		2 /* Instruction breakpoint */
#define RTX_TRAP_RUN_MODE_EXCEPTION		3
#define RTX_TRAP_SINGLE_STEP_EXCEPTION		4
#define RTX_TRAP_ALIGNMENT_EXCEPTION		5
#define RTX_TRAP_NONRECOVERABLE_EXCEPTION	6
#define RTX_TRAP_DEBUG_EXCEPTION		7
#define RTX_TRAP_ALTIVEC_UNAVAILABLE_EXCEPTION	8
#define RTX_TRAP_ALTIVEC_ASSIST_EXCEPTION	9 /* Data or instruction access exception */
#define RTX_TRAP_SPE_FP_EXCEPTION		10 /* Floating point exception */
#define RTX_TRAP_ACCESS_EXCEPTION		11

/* old values from 2.6.15.4 - no longer necessary ? */
#define RTX_TRAP_SECTION	1	/* Section fault */
#define RTX_TRAP_DABT		2	/* Generic data abort */
#define RTX_TRAP_FPU		5	
#define RTX_TRAP_VFP		6	/* VFP floating point exception */
#define RTX_TRAP_UNDEFINSTR	7	/* Undefined instruction */

#ifndef __ASSEMBLY__

#include <asm/rtx_base.h>
#include <linux/rtx_time_conv.h>

extern int init_timer_done;

#define __rtx_root_tick_p(regs) (!raw_irqs_disabled_flags(regs->msr))
#define rtx_update_tick_evtdev(evtdev) do { } while (0)

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

extern int __rtx_trap_handling(int vector, struct pt_regs *regs);


void irq_enter(void);
void irq_exit(void);
/* Call virtual interrupt handler */
static inline void __rtx_call_lx_virq_handler(unsigned irq,
						  rtx_handler_t handler,
						  void *cookie)
{
   irq_enter();
   handler(irq,cookie);
   irq_exit();
}

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

#define IS_RTX_TIMER_IRQ(irq)		(irq == __rtx_timer_irq)	// legacy timer
#define rtx_processor_id()		raw_smp_processor_id()

#define __rtx_move_lx_irq(irq)	do { } while (0)

/* Floating point */
#define RTX_UNLAZY_FPU(tsk)		do { } while (0);

/* System call */
#define RTX_SYSCALL_ID()		current->rt_syscall_id
#define RTX_SYSCALL_SETPGID		57		/* system call number */

/* in the case the system timer needs special handling to restart */
#define RTX_SYS_CLEAR_INTR()


/*
 * Per CPU access definitions
 */
#ifdef CONFIG_SMP

/* Native atomic access for powerpc necessary: T B D */

#else /* ! SMP */

#include <asm-generic/percpu.h>

#define rtx_get_cpu_var(var)		__get_cpu_var(var) 
#define rtx_set_cpu_var(var, val)	(__get_cpu_var(var) = val)
#define rtx_and_cpu_var(var, val)	(__get_cpu_var(var) &= val)
#define rtx_or_cpu_var(var, val)	(__get_cpu_var(var) |= val)
#define rtx_add_cpu_var(var, val)	(__get_cpu_var(var) += val)

#define rtx_get_cpu_member(var, field)	__get_cpu_var(var.field)
#define rtx_set_cpu_member(var, field, val) (__get_cpu_var(var.field) = val)

#define rtx_per_cpu(variable, cpu)		per_cpu(variable, cpu)

#endif

#else /* __ASSEMBLY__ */

#endif /* __ASSEMBLY__ */

#endif /* _ASM_POWERPC_RTX_ADAPT_H_ */
          
