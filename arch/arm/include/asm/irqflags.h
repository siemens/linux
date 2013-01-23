#ifndef __ASM_ARM_IRQFLAGS_H
#define __ASM_ARM_IRQFLAGS_H

#ifdef __KERNEL__

#include <asm/ptrace.h>

/*
 * CPU interrupt mask handling.
 */

#ifdef CONFIG_RTX_DOMAIN
#include <asm/rtx_base.h>

#if __LINUX_ARM_ARCH__ >= 6

#define local_irq_save_hw(x)					\
	({							\
	__asm__ __volatile__(					\
	"mrs	%0, cpsr		@ local_irq_save_hw\n"	\
	"cpsid	i"						\
	: "=r" (x) : : "memory", "cc");				\
	})

#define local_irq_enable_hw()  __asm__("cpsie i	@ __sti" : : : "memory", "cc")
#define local_irq_disable_hw() __asm__("cpsid i	@ __cli" : : : "memory", "cc")
#define local_fiq_enable_hw()  __asm__("cpsie f	@ __stf" : : : "memory", "cc")
#define local_fiq_disable_hw() __asm__("cpsid f	@ __clf" : : : "memory", "cc")

#else /* ! __LINUX_ARM_ARCH__  >= 6 */

/*
 * Save the current interrupt enable state & disable IRQs
 */
#define local_irq_save_hw(x)					\
	({							\
		unsigned long temp;				\
		(void) (&temp == &x);				\
	__asm__ __volatile__(					\
	"mrs	%0, cpsr		@ local_irq_save_hw\n"	\
"	orr	%1, %0, #128\n"					\
"	msr	cpsr_c, %1"					\
	: "=r" (x), "=r" (temp)					\
	:							\
	: "memory", "cc");					\
	})
	


/*
 * Enable IRQs
 */
#define local_irq_enable_hw()					\
	({							\
		unsigned long temp;				\
	__asm__ __volatile__(					\
	"mrs	%0, cpsr		@ local_irq_enable_hw\n"\
"	bic	%0, %0, #128\n"					\
"	msr	cpsr_c, %0"					\
	: "=r" (temp)						\
	:							\
	: "memory", "cc");					\
	})

/*
 * Disable IRQs
 */
#define local_irq_disable_hw()					\
	({							\
		unsigned long temp;				\
	__asm__ __volatile__(					\
	"mrs	%0, cpsr		@ local_irq_disable_hw\n"\
"	orr	%0, %0, #128\n"					\
"	msr	cpsr_c, %0"					\
	: "=r" (temp)						\
	:							\
	: "memory", "cc");					\
	})

/*
 * Enable FIQs
 */
#define local_fiq_enable_hw()					\
	({							\
		unsigned long temp;				\
	__asm__ __volatile__(					\
	"mrs	%0, cpsr		@ stf\n"		\
"	bic	%0, %0, #64\n"					\
"	msr	cpsr_c, %0"					\
	: "=r" (temp)						\
	:							\
	: "memory", "cc");					\
	})

/*
 * Disable FIQs
 */
#define local_fiq_disable_hw()					\
	({							\
		unsigned long temp;				\
	__asm__ __volatile__(					\
	"mrs	%0, cpsr		@ clf\n"		\
"	orr	%0, %0, #64\n"					\
"	msr	cpsr_c, %0"					\
	: "=r" (temp)						\
	:							\
	: "memory", "cc");					\
	})

#endif /* __LINUX_ARM_ARCH__  >= 6 */

/*
 * Save the current interrupt enable state.
 */
#define local_save_flags_hw(x)					\
	({							\
	__asm__ __volatile__(					\
	"mrs	%0, cpsr		@ local_save_flags_hw"	\
	: "=r" (x) : : "memory", "cc");				\
	})

/*
 * restore saved IRQ & FIQ state
 */
#define local_irq_restore_hw(x)					\
	__asm__ __volatile__(					\
	"msr	cpsr_c, %0		@ local_irq_restore_hw\n"\
	:							\
	: "r" (x)						\
	: "memory", "cc")

#define local_irq_save_hw_notrace(x) 		local_irq_save_hw(x)	
#define local_irq_restore_hw_notrace(x) 	local_irq_restore_hw(x)	

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

/* PSR_I_BIT is bit no. 7 and is set if interrupts are _disabled_ */
#define raw_local_irq_save(flags)		((flags) = __rtx_test_and_set_stalled_bit() << 7)
#define raw_local_irq_enable()		__rtx_local_irq_enable()
#define raw_local_irq_disable()		__rtx_set_stalled_bit()
#define local_fiq_enable()		__rtx_local_irq_enable()
#define local_fiq_disable()		__rtx_set_stalled_bit()
#define raw_local_save_flags(flags)	((flags) = __rtx_test_stalled_bit() << 7)
#define raw_local_irq_restore(flags)	__rtx_restore_stalled_bit(flags & (1 << 7))

#define irqs_disabled_hw()		\
    ({					\
	unsigned long x;		\
	local_save_flags_hw(x);		\
	raw_irqs_disabled_flags(x);	\
    })

#else /* !CONFIG_RTX_DOMAIN */ 

#if __LINUX_ARM_ARCH__ >= 6

#define raw_local_irq_save(x)					\
	({							\
	__asm__ __volatile__(					\
	"mrs	%0, cpsr		@ local_irq_save\n"	\
	"cpsid	i"						\
	: "=r" (x) : : "memory", "cc");				\
	})

#define raw_local_irq_enable()  __asm__("cpsie i	@ __sti" : : : "memory", "cc")
#define raw_local_irq_disable() __asm__("cpsid i	@ __cli" : : : "memory", "cc")
#define local_fiq_enable()  __asm__("cpsie f	@ __stf" : : : "memory", "cc")
#define local_fiq_disable() __asm__("cpsid f	@ __clf" : : : "memory", "cc")

#else /* ! __LINUX_ARM_ARCH__  >= 6 */

/*
 * Save the current interrupt enable state & disable IRQs
 */
#define raw_local_irq_save(x)					\
	({							\
		unsigned long temp;				\
		(void) (&temp == &x);				\
	__asm__ __volatile__(					\
	"mrs	%0, cpsr		@ local_irq_save\n"	\
"	orr	%1, %0, #128\n"					\
"	msr	cpsr_c, %1"					\
	: "=r" (x), "=r" (temp)					\
	:							\
	: "memory", "cc");					\
	})
	
/*
 * Enable IRQs
 */
#define raw_local_irq_enable()					\
	({							\
		unsigned long temp;				\
	__asm__ __volatile__(					\
	"mrs	%0, cpsr		@ local_irq_enable\n"	\
"	bic	%0, %0, #128\n"					\
"	msr	cpsr_c, %0"					\
	: "=r" (temp)						\
	:							\
	: "memory", "cc");					\
	})

/*
 * Disable IRQs
 */
#define raw_local_irq_disable()					\
	({							\
		unsigned long temp;				\
	__asm__ __volatile__(					\
	"mrs	%0, cpsr		@ local_irq_disable\n"	\
"	orr	%0, %0, #128\n"					\
"	msr	cpsr_c, %0"					\
	: "=r" (temp)						\
	:							\
	: "memory", "cc");					\
	})

/*
 * Enable FIQs
 */
#define local_fiq_enable()					\
	({							\
		unsigned long temp;				\
	__asm__ __volatile__(					\
	"mrs	%0, cpsr		@ stf\n"		\
"	bic	%0, %0, #64\n"					\
"	msr	cpsr_c, %0"					\
	: "=r" (temp)						\
	:							\
	: "memory", "cc");					\
	})

/*
 * Disable FIQs
 */
#define local_fiq_disable()					\
	({							\
		unsigned long temp;				\
	__asm__ __volatile__(					\
	"mrs	%0, cpsr		@ clf\n"		\
"	orr	%0, %0, #64\n"					\
"	msr	cpsr_c, %0"					\
	: "=r" (temp)						\
	:							\
	: "memory", "cc");					\
	})

#endif /* __LINUX_ARM_ARCH__  >= 6 */

/*
 * Save the current interrupt enable state.
 */
#define raw_local_save_flags(x)					\
	({							\
	__asm__ __volatile__(					\
	"mrs	%0, cpsr		@ local_save_flags"	\
	: "=r" (x) : : "memory", "cc");				\
	})

/*
 * restore saved IRQ & FIQ state
 */
#define raw_local_irq_restore(x)				\
	__asm__ __volatile__(					\
	"msr	cpsr_c, %0		@ local_irq_restore\n"	\
	:							\
	: "r" (x)						\
	: "memory", "cc")

#endif /* CONFIG_RTX_DOMAIN */

#define raw_irqs_disabled_flags(flags)	\
({					\
	(int)((flags) & PSR_I_BIT);	\
})

#endif /* __KERNEL__ */
#endif /* __ASM_ARM_IRQFLAGS_H */
