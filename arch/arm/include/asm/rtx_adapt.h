/*
 * asm-arm/rtx_adapt.h
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
#ifndef _RTX_ADAPT_H_
#define _RTX_ADAPT_H_

/* ARM traps */
#define RTX_TRAP_ACCESS	 	0	/* Data or instruction access exception */
#define RTX_TRAP_SECTION	1	/* Section fault */
#define RTX_TRAP_DABT		2	/* Generic data abort */
#define RTX_TRAP_UNKNOWN	3	/* Unknown exception */
#define RTX_TRAP_BREAK		4	/* Instruction breakpoint */
#define RTX_TRAP_FPU		5	/* Floating point exception */
#define RTX_TRAP_VFP		6	/* VFP floating point exception */
#define RTX_TRAP_UNDEFINSTR	7	/* Undefined instruction */

#ifndef __ASSEMBLY__

DECLARE_PER_CPU(struct pt_regs, rtx_tick_regs);

extern int init_timer_done;

#ifdef CONFIG_RTX_DOMAIN_INTEGRITY_CHECK
static inline void rtx_check_intr_flag(void)
{
		unsigned long chk_flags;
		local_save_flags_hw(chk_flags);
		if (!(chk_flags & 0x80))
			panic("%s: HW must be off (flags=%#lx)\n", __func__, chk_flags);
}
#endif

/*
 * The memory for the RT extensions can be allocated in an architecture specific way.
 */
extern void kfree(const void *);
#define KMALLOC kmalloc
#define KFREE	kfree

#include <asm/rtx_base.h>

#define __rtx_root_tick_p(regs) (!raw_irqs_disabled_flags(regs->ARM_cpsr))
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


extern int __rtx_trap_handling(int vector, struct pt_regs *regs);

/* Call virtual interrupt handler */
static inline void __rtx_call_lx_virq_handler(unsigned irq,
						  rtx_handler_t handler,
						  void *cookie)
{
   irq_enter();
   handler(irq,cookie);
   irq_exit();
}

#define IS_RTX_TIMER_IRQ(irq)		(irq == __rtx_timer_irq)	// legacy timer
#define rtx_processor_id()		raw_smp_processor_id()

#define __rtx_move_lx_irq(irq)	do { } while (0)

/* Floating point */
#define RTX_UNLAZY_FPU(tsk)		do { } while (0);

/* System call */
#define RTX_SYSCALL_ID()		current->rt_syscall_id
#define RTX_SYSCALL_SETPGID		57		/* system call number */

#ifdef CONFIG_ARCH_AT91
#if defined (CONFIG_ATMEL_TCB_CLKSRC)
/* Single Interrupt  */
void rtx_at91_fake_tcb_irq(void);
void rtx_at91_handle_tcb(void);
#elif defined (CONFIG_ARCH_AT91SAM9G45)
/* Shared Interrupt  */
void rtx_at91_handle_pit(void);
void rtx_at91_handle_usart(void);
#ifdef CONFIG_RTC_DRV_AT91SAM9
void rtx_at91_handle_rtt(void);
#define RTX_HANDLE_SHARED_INTERRUPT()  \
			rtx_at91_handle_pit();			\
			rtx_at91_handle_rtt();			\
			rtx_at91_handle_usart();
#else
#define RTX_HANDLE_SHARED_INTERRUPT()  \
			rtx_at91_handle_pit();			\
			rtx_at91_handle_usart();
#endif /* CONFIG_RTC_DRV_AT91SAM9 */
#endif /* CONFIG_ATMEL_TCB_CLKSRC || CONFIG_ARCH_AT91SAM9G45*/
#endif /* CONFIG_ARCH_AT91 */

/* Timer Interrupt Reset  */
#if defined (CONFIG_ARCH_AT91)
#if defined (CONFIG_ATMEL_TCB_CLKSRC)
/* The tcb-interrupt isn't shared! */
#define RTX_SYS_CLEAR_INTR()	 rtx_at91_handle_tcb()
#else
#define RTX_SYS_CLEAR_INTR()
#endif
#elif defined (CONFIG_ARCH_FEROCEON)
#define RTX_SYS_CLEAR_INTR()		mv_clear_bridge_int()
#elif defined (CONFIG_ARCH_MX6Q)
unsigned long clk_get_timer_rate(void);
void gpt_irq_acknowledge(void);
#define RTX_SYS_CLEAR_INTR()		gpt_irq_acknowledge()
#else
#define RTX_SYS_CLEAR_INTR()
#endif

//#define RTX_SYSCALL_ID()		regs.ARM_r7
#define RTX_SYSCALL_RETVAL()		regs.ARM_r7

/*
 * Per CPU access definitions
 */
#ifdef CONFIG_SMP

/* Native atomic access for arm necessary: T B D */

#else /* ! SMP */

#include <asm-generic/percpu.h>

#define rtx_get_cpu_var(var)		__get_cpu_var(var) 
#define rtx_set_cpu_var(var, val)	(__get_cpu_var(var) = val)
#define rtx_and_cpu_var(var, val)	(__get_cpu_var(var) &= val)
#define rtx_or_cpu_var(var, val)	(__get_cpu_var(var) |= val)
#define rtx_add_cpu_var(var, val)	(__get_cpu_var(var) += val)

#define rtx_get_cpu_member(var, field)	__get_cpu_var(var.field)
#define rtx_set_cpu_member(var, field, val) (__get_cpu_var(var.field) = val)

#define rtx_per_cpu(variable, cpu)			per_cpu(variable, cpu)

#endif

#endif /* __ASSEMBLY__ */

#endif /* _RTX_ADAPT_H_ */
