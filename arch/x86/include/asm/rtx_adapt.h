/*
 * arch/x86/include/asm/rtx_adapt.h
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
 * Parts of this file are based on the adeos-ipipe/Xenomai patch.
 * Adaption to Audis by <wolfgang.hartmann@siemens.com>
 *
 */
#ifndef _X86_RTX_ADAPT_H_
#define _X86_RTX_ADAPT_H_

#ifndef __ASSEMBLY__
#include <asm/i387.h>
#include <asm/apic.h>

DECLARE_PER_CPU(struct pt_regs, rtx_tick_regs);

extern int init_timer_done;

/* Local Apic IPI handling */
typedef void (*rtx_apic_handler_t) (struct pt_regs *regs);
typedef int (* rtx_apic_ack_t)(unsigned int);

struct rtx_apic_funcs {
	rtx_apic_handler_t handler;
	rtx_apic_ack_t ack;
};
typedef struct rtx_apic_funcs rtx_apic_funcs_t;

/* Conversion macros for vector, irq, array index */
#define rtx_apic_vector_irq(vec) ((vec) - FIRST_SYSTEM_VECTOR + NR_IRQS)
#define rtx_apic_irq_vector(irq) ((irq) - NR_IRQS + FIRST_SYSTEM_VECTOR)

/*
 * Handles the mapping between device IRQs resp. APIC IRQs
 * and the corresponding vector.
 */
static inline unsigned __rtx_get_irq_vector(int irq)
{
#ifdef CONFIG_X86_IO_APIC
	unsigned __rtx_get_ioapic_irq_vector(int irq);
	return __rtx_get_ioapic_irq_vector(irq);
#elif defined(CONFIG_X86_LOCAL_APIC)
        return irq >= FIRST_SYSTEM_VECTOR && irq < RTX_VIRQ_BASE ?
                rtx_apic_irq_vector(irq) : irq + IRQ0_VECTOR;
#else
	return irq + IRQ0_VECTOR;
#endif
}

#ifdef CONFIG_RTX_DOMAIN_INTEGRITY_CHECK
static inline void rtx_check_intr_flag(void)
{
		unsigned long chk_flags;
		local_save_flags_hw(chk_flags);
		if (chk_flags & 0x200)
			panic("%s: HW must be off (flags=%#lx)\n", __func__, chk_flags);
}
#endif

/* Call LX device interrupt handler */
static inline void __rtx_call_lx_irq_handler(unsigned irq,
						  rtx_handler_lx_t handler)
{
	struct pt_regs *regs = &__raw_get_cpu_var(rtx_tick_regs);

	regs->orig_ax = ~__rtx_get_irq_vector(irq);

	__asm__ __volatile__("pushfl\n\t"
				 "orl %[x86if],(%%esp)\n\t"
			     "pushl %%cs\n\t"
			     "pushl $__xirq_end\n\t"
			     "pushl %%eax\n\t"
				 "pushl %%gs\n\t"
			     "pushl %%fs\n\t"
			     "pushl %%es\n\t"
			     "pushl %%ds\n\t"
			     "pushl %%eax\n\t"
			     "pushl %%ebp\n\t"
			     "pushl %%edi\n\t"
			     "pushl %%esi\n\t"
			     "pushl %%edx\n\t"
			     "pushl %%ecx\n\t"
			     "pushl %%ebx\n\t"
			     "movl  %2,%%eax\n\t"
			     "call *%1\n\t"
			     "jmp ret_from_intr\n\t"
			     "__xirq_end:\n"
			     : /* no output */
			     : "a" (~irq), "r" (handler), "rm" (regs),
			     [x86if]"i"(X86_EFLAGS_IF));
}

void irq_enter(void);
void irq_exit(void);

/* Call virtual interrupt handler */
static inline void __rtx_call_lx_virq_handler(unsigned irq,
						  rtx_handler_t handler,
						  void *cookie)
{
	irq_enter();
	__asm__ __volatile__("pushfl\n\t"
				 "orl %[x86if],(%%esp)\n\t"
			     "pushl %%cs\n\t"
			     "pushl $__virq_end\n\t"
			     "pushl $-1\n\t"
				 "pushl %%gs\n\t"
			     "pushl %%fs\n\t"
			     "pushl %%es\n\t"
			     "pushl %%ds\n\t"
			     "pushl %%eax\n\t"
			     "pushl %%ebp\n\t"
			     "pushl %%edi\n\t"
			     "pushl %%esi\n\t"
			     "pushl %%edx\n\t"
			     "pushl %%ecx\n\t"
			     "pushl %%ebx\n\t"
			     "pushl %2\n\t"
			     "pushl %%eax\n\t"
			     "call *%1\n\t"
			     "addl $8,%%esp\n"
			     : /* no output */
			     : "a" (irq), "r" (handler), "d" (cookie),
			     [x86if]"i"(X86_EFLAGS_IF));
	irq_exit();
	__asm__ __volatile__("jmp ret_from_intr\n\t"
			     "__virq_end:\n"
			     : /* no output */
			     : /* no input */);
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
	do {							\
		local_irq_disable_hw();		\
	} while (0)

#define IS_RTX_TIMER_IRQ(irq)		(irq == __rtx_tick_irq)	// legacy timer (e.g. PIT, lapic)

#define rtx_processor_id()			raw_smp_processor_id()

#define __rtx_root_tick_p(regs)		(regs->flags & X86_EFLAGS_IF)
/*
 * The Linux kernel should only provide two tick devices:
 * - PIT: irq 0
 * - LAPIC: if there is a local apic timer available.
 * The Linux _oneshot_ tick timer is grabbed by the RT system.
 * Linux ticks will be emulated. There is only one clock event
 * device that will be used by Linux and the RT system.
 * Note: The HPET timers shouldn't be used for a tick device.
 */
extern int __rtx_tick_irq;

#define rtx_update_tick_evtdev(evtdev) \
	do { \
		if (strcmp((evtdev)->name, "lapic") == 0) \
			__rtx_tick_irq = rtx_apic_vector_irq(LOCAL_TIMER_VECTOR); \
		else \
			__rtx_tick_irq = 0; \
	} while (0)

#ifdef CONFIG_SMP
#define __rtx_move_lx_irq(irq)					\
	do {								\
		if (irq < NR_IRQS) {			\
			struct irq_chip *chip = irq_to_desc(irq)->chip;	\
			if (chip->move)					\
				chip->move(irq);			\
		}									\
	} while (0)
#else /* !CONFIG_SMP */
#define __rtx_move_lx_irq(irq)	do { } while (0)
#endif /* !CONFIG_SMP */

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
/* Native atomic access for x86 */
#define rtx_get_cpu_var(var)					percpu_read(var)
#define rtx_set_cpu_var(var, val)				percpu_write(var, val)
#define rtx_and_cpu_var(var, val)				percpu_and(var, val)
#define rtx_or_cpu_var(var, val)				percpu_or(var, val)
#define rtx_add_cpu_var(var, val)				percpu_add(var, val)

#define rtx_get_cpu_member(var, field)			percpu_read(var.field)
#define rtx_set_cpu_member(var, field, val)		percpu_write(var.field, val)

#define rtx_per_cpu(variable, cpu)			per_cpu(variable, cpu)

/* Floating point */
#define RTX_UNLAZY_FPU(tsk)			unlazy_fpu(tsk)

/* System call */
#define RTX_SYSCALL_ID()			regs.orig_ax
#define RTX_SYSCALL_SETPGID			57		/* system call number */

#endif /* __ASSEMBLY__ */

#define ex_do_divide_error					0
#define ex_do_debug							1
/* NMI not pipelined. */
#define ex_do_int3							3
#define ex_do_overflow						4
#define ex_do_bounds						5
#define ex_do_invalid_op					6
#define ex_do_device_not_available			7
/* Double fault not pipelined. */
#define ex_do_coprocessor_segment_overrun 	9
#define ex_do_invalid_TSS					10
#define ex_do_segment_not_present			11
#define ex_do_stack_segment					12
#define ex_do_general_protection			13
#define ex_do_page_fault					14
#define ex_do_spurious_interrupt_bug		15
#define ex_do_coprocessor_error				16
#define ex_do_alignment_check				17
#define ex_machine_check_vector				18
#define ex_do_simd_coprocessor_error		19
#define ex_do_iret_error					32

#endif /* _X86_RTX_ADAPT_H_ */
