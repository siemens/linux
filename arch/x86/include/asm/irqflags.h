#ifndef _X86_IRQFLAGS_H_
#define _X86_IRQFLAGS_H_

#include <asm/processor-flags.h>

#ifndef __ASSEMBLY__
/*
 * Interrupt control:
 */
#ifdef CONFIG_RTX_DOMAIN
#include <asm/rtx_base.h>
#define local_irq_save_hw_notrace(x) \
	__asm__ __volatile__("pushf ; pop %0 ; cli":"=g" (x): /* no input */ :"memory")
#define local_irq_restore_hw_notrace(x) \
	__asm__ __volatile__("push %0 ; popf": /* no output */ :"g" (x):"memory", "cc")

#define local_save_flags_hw(x)	__asm__ __volatile__("pushf ; pop %0":"=g" (x): /* no input */)

#define irqs_disabled_hw()		\
    ({					\
	unsigned long x;		\
	local_save_flags_hw(x);		\
	!((x) & X86_EFLAGS_IF);		\
    })

#ifdef CONFIG_RTX_TRACE_IRQSOFF
#include <linux/aud/rt_trace.h>

#define local_irq_disable_hw() do {                     \
                if (!irqs_disabled_hw()) {              \
                        local_irq_disable_hw_notrace(); \
                        rt_trace_begin(0x80000000);  \
                }                                       \
        } while (0)
#define local_irq_enable_hw() do {                      \
                if (irqs_disabled_hw()) {               \
                        rt_trace_end(0x80000000);    \
                        local_irq_enable_hw_notrace();  \
                }                                       \
        } while (0)
#define local_irq_save_hw(x) do {                       \
                local_save_flags_hw(x);                 \
                if ((x) & X86_EFLAGS_IF) {              \
                        local_irq_disable_hw_notrace(); \
                        rt_trace_begin(0x80000001);  \
                }                                       \
        } while (0)
#define local_irq_restore_hw(x) do {                    \
                if ((x) & X86_EFLAGS_IF)                \
                        rt_trace_end(0x80000001);    \
                local_irq_restore_hw_notrace(x);        \
        } while (0)
#else /* !CONFIG_RTX_TRACE_IRQSOFF */
#define local_irq_save_hw(x)            local_irq_save_hw_notrace(x)
#define local_irq_restore_hw(x)         local_irq_restore_hw_notrace(x)
#define local_irq_enable_hw()           local_irq_enable_hw_notrace()
#define local_irq_disable_hw()          local_irq_disable_hw_notrace()
#endif /* CONFIG_RTX_TRACE_IRQSOFF */

#define local_irq_enable_hw_notrace()	__asm__ __volatile__("sti": : :"memory")
#define local_irq_disable_hw_notrace()	__asm__ __volatile__("cli": : :"memory")
#endif /* CONFIG_RTX_DOMAIN */

static inline unsigned long native_save_fl(void)
{
	unsigned long flags;

#ifdef CONFIG_RTX_DOMAIN
	flags = (!__rtx_test_stalled_bit()) << 9;
	barrier();
#else
	/*
	 * Note: this needs to be "=r" not "=rm", because we have the
	 * stack offset from what gcc expects at the time the "pop" is
	 * executed, and so a memory reference with respect to the stack
	 * would end up using the wrong address.
	 */
	asm volatile("# __raw_save_flags\n\t"
		     "pushf ; pop %0"
		     : "=r" (flags)
		     : /* no input */
		     : "memory");
#endif
	return flags;
}

static inline void native_restore_fl(unsigned long flags)
{
#ifdef CONFIG_RTX_DOMAIN
	struct rtx_pcd *pcd;
	barrier();
	if (!(flags & X86_EFLAGS_IF)) {
		__rtx_set_stalled_bit();
	}
	else {
		local_irq_disable_hw();
		pcd = &__raw_get_cpu_var(rtx_percpu_data);
		__clear_bit(LX_IRQS_STALLED, &pcd->rtx_irq_control);
		barrier();
		if (rtx_pending_irqs(pcd))
			__rtx_handle_nonrt_pending(RTX_IRQMASK_ANY);
		local_irq_enable_hw();
	}
#else
	asm volatile("push %0 ; popf"
		     : /* no output */
		     :"g" (flags)
		     :"memory", "cc");
#endif
}

static inline void native_irq_disable(void)
{
#ifdef CONFIG_RTX_DOMAIN
	__rtx_set_stalled_bit();
	barrier();
#else
	asm volatile("cli": : :"memory");
#endif
}

static inline void native_irq_enable(void)
{
#ifdef CONFIG_RTX_DOMAIN
	barrier();
	__rtx_local_irq_enable();
#else
	asm volatile("sti": : :"memory");
#endif
}

static inline void native_safe_halt(void)
{
#ifdef CONFIG_RTX_DOMAIN
	barrier();
	__rtx_halt_lx();
#else
	asm volatile("sti; hlt": : :"memory");
#endif
}

static inline void native_halt(void)
{
	asm volatile("hlt": : :"memory");
}

#endif

#ifdef CONFIG_X86_64
/*
 * Only returns from a trap or exception to a NMI context (intra-privilege
 * level near return) to the same SS and CS segments. Should be used
 * upon trap or exception return when nested over a NMI context so no iret is
 * issued. It takes care of modifying the eflags, rsp and returning to the
 * previous function.
 *
 * The stack, at that point, looks like :
 *
 * 0(rsp)  RIP
 * 8(rsp)  CS
 * 16(rsp) EFLAGS
 * 24(rsp) RSP
 * 32(rsp) SS
 *
 * Upon execution :
 * Copy EIP to the top of the return stack
 * Update top of return stack address
 * Pop eflags into the eflags register
 * Make the return stack current
 * Near return (popping the return address from the return stack)
 */
#define NATIVE_INTERRUPT_RETURN_NMI_SAFE	pushq %rax;		\
						movq %rsp, %rax;	\
						movq 24+8(%rax), %rsp;	\
						pushq 0+8(%rax);	\
						pushq 16+8(%rax);	\
						movq (%rax), %rax;	\
						popfq;			\
						ret
#else
/*
 * Protected mode only, no V8086. Implies that protected mode must
 * be entered before NMIs or MCEs are enabled. Only returns from a trap or
 * exception to a NMI context (intra-privilege level far return). Should be used
 * upon trap or exception return when nested over a NMI context so no iret is
 * issued.
 *
 * The stack, at that point, looks like :
 *
 * 0(esp) EIP
 * 4(esp) CS
 * 8(esp) EFLAGS
 *
 * Upon execution :
 * Copy the stack eflags to top of stack
 * Pop eflags into the eflags register
 * Far return: pop EIP and CS into their register, and additionally pop EFLAGS.
 */
#define NATIVE_INTERRUPT_RETURN_NMI_SAFE	pushl 8(%esp);	\
						popfl;		\
						lret $4
#endif

#ifdef CONFIG_PARAVIRT
#include <asm/paravirt.h>
#else
#ifndef __ASSEMBLY__

static inline unsigned long __raw_local_save_flags(void)
{
	return native_save_fl();
}

static inline void raw_local_irq_restore(unsigned long flags)
{
	native_restore_fl(flags);
}

#ifdef CONFIG_RTX_DOMAIN
static inline unsigned long raw_mangle_irq_bits(int virt, unsigned long real)
{
	/*
	 * Merge virtual and real interrupt mask bits into a single
	 * (32bit) word.
	 */
	return (real & ~(1L << 31)) | ((unsigned long)(virt != 0) << 31);
}

static inline int raw_demangle_irq_bits(unsigned long *x)
{
	int virt = (*x & (1L << 31)) != 0;
	*x &= ~(1L << 31);
	return virt;
}
#endif

static inline void raw_local_irq_disable(void)
{
	native_irq_disable();
}

static inline void raw_local_irq_enable(void)
{
	native_irq_enable();
}

/*
 * Used in the idle loop; sti takes one instruction cycle
 * to complete:
 */
static inline void raw_safe_halt(void)
{
	native_safe_halt();
}

/*
 * Used when interrupts are already enabled or to
 * shutdown the processor:
 */
static inline void halt(void)
{
	native_halt();
}

/*
 * For spinlocks, etc:
 */
static inline unsigned long __raw_local_irq_save(void)
{
#ifdef CONFIG_RTX_DOMAIN
	unsigned long flags;

	flags = (!__rtx_test_and_set_stalled_bit()) << 9;
	barrier();
#else
	unsigned long flags = __raw_local_save_flags();

	raw_local_irq_disable();
#endif
	return flags;
}
#else

#define ENABLE_INTERRUPTS(x)		sti
#define DISABLE_INTERRUPTS(x)		cli

#ifdef CONFIG_RTX_DOMAIN
#define ENABLE_INTERRUPTS_HW_COND	sti
#define DISABLE_INTERRUPTS_HW_COND	cli
#else /* !CONFIG_RTX_DOMAIN */
#define ENABLE_INTERRUPTS_HW_COND
#define DISABLE_INTERRUPTS_HW_COND
#endif /* !CONFIG_RTX_DOMAIN */

#define INTERRUPT_RETURN_NMI_SAFE	NATIVE_INTERRUPT_RETURN_NMI_SAFE

#ifdef CONFIG_X86_64
#define SWAPGS	swapgs
/*
 * Currently paravirt can't handle swapgs nicely when we
 * don't have a stack we can rely on (such as a user space
 * stack).  So we either find a way around these or just fault
 * and emulate if a guest tries to call swapgs directly.
 *
 * Either way, this is a good way to document that we don't
 * have a reliable stack. x86_64 only.
 */
#define SWAPGS_UNSAFE_STACK	swapgs

#define PARAVIRT_ADJUST_EXCEPTION_FRAME	/*  */

#define INTERRUPT_RETURN	iretq
#define USERGS_SYSRET64				\
	swapgs;					\
	sysretq;
#define USERGS_SYSRET32				\
	swapgs;					\
	sysretl
#define ENABLE_INTERRUPTS_SYSEXIT32		\
	swapgs;					\
	sti;					\
	sysexit

#else
#define INTERRUPT_RETURN		iret
#define ENABLE_INTERRUPTS_SYSEXIT	sti; sysexit
#define GET_CR0_INTO_EAX		movl %cr0, %eax
#endif


#endif /* __ASSEMBLY__ */
#endif /* CONFIG_PARAVIRT */

#ifndef __ASSEMBLY__
#define raw_local_save_flags(flags)				\
	do { (flags) = __raw_local_save_flags(); } while (0)

#define raw_local_irq_save(flags)				\
	do { (flags) = __raw_local_irq_save(); } while (0)

static inline int raw_irqs_disabled_flags(unsigned long flags)
{
	return !(flags & X86_EFLAGS_IF);
}

static inline int raw_irqs_disabled(void)
{
	unsigned long flags = __raw_local_save_flags();

	return raw_irqs_disabled_flags(flags);
}

#else

#ifdef CONFIG_X86_64
#define ARCH_LOCKDEP_SYS_EXIT		call lockdep_sys_exit_thunk
#define ARCH_LOCKDEP_SYS_EXIT_IRQ	\
	TRACE_IRQS_ON; \
	sti; \
	SAVE_REST; \
	LOCKDEP_SYS_EXIT; \
	RESTORE_REST; \
	cli; \
	TRACE_IRQS_OFF;

#else
#define ARCH_LOCKDEP_SYS_EXIT			\
	pushl %eax;				\
	pushl %ecx;				\
	pushl %edx;				\
	call lockdep_sys_exit;			\
	popl %edx;				\
	popl %ecx;				\
	popl %eax;

#define ARCH_LOCKDEP_SYS_EXIT_IRQ
#endif

#ifdef CONFIG_TRACE_IRQFLAGS
#  define TRACE_IRQS_ON		call trace_hardirqs_on_thunk;
#  define TRACE_IRQS_OFF	call trace_hardirqs_off_thunk;
#else
#  define TRACE_IRQS_ON
#  define TRACE_IRQS_OFF
#endif
#ifdef CONFIG_DEBUG_LOCK_ALLOC
#  define LOCKDEP_SYS_EXIT	ARCH_LOCKDEP_SYS_EXIT
#  define LOCKDEP_SYS_EXIT_IRQ	ARCH_LOCKDEP_SYS_EXIT_IRQ
# else
#  define LOCKDEP_SYS_EXIT
#  define LOCKDEP_SYS_EXIT_IRQ
# endif

#endif /* __ASSEMBLY__ */
#endif
