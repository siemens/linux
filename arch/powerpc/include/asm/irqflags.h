/*
 * IRQ flags handling
 */
#ifndef _ASM_IRQFLAGS_H
#define _ASM_IRQFLAGS_H

#ifndef __ASSEMBLY__
/*
 * Get definitions for raw_local_save_flags(x), etc.
 */
#include <asm/hw_irq.h>

static inline unsigned long raw_mangle_irq_bits(int virt, unsigned long real)
{
	/*
	 * Merge virtual and real interrupt mask bits into a single
	 * (32bit) word.
	 */
	return (real & ~(1L << 31)) | ((virt != 0) << 31);
}

static inline int raw_demangle_irq_bits(unsigned long *x)
{
	int virt = (*x & (1L << 31)) != 0;
	*x &= ~(1L << 31);
	return virt;
}

#else
#ifdef CONFIG_TRACE_IRQFLAGS
/*
 * Most of the CPU's IRQ-state tracing is done from assembly code; we
 * have to call a C function so call a wrapper that saves all the
 * C-clobbered registers.
 */
#define TRACE_ENABLE_INTS	bl .trace_hardirqs_on
#define TRACE_DISABLE_INTS	bl .trace_hardirqs_off
#define TRACE_AND_RESTORE_IRQ_PARTIAL(en,skip)	\
	cmpdi	en,0;				\
	bne	95f;				\
	stb	en,PACASOFTIRQEN(r13);		\
	bl	.trace_hardirqs_off;		\
	b	skip;				\
95:	bl	.trace_hardirqs_on;		\
	li	en,1;
#define TRACE_AND_RESTORE_IRQ(en)		\
	TRACE_AND_RESTORE_IRQ_PARTIAL(en,96f);	\
	stb	en,PACASOFTIRQEN(r13);	        \
96:
#else
#define TRACE_ENABLE_INTS
#define TRACE_DISABLE_INTS
#define TRACE_AND_RESTORE_IRQ_PARTIAL(en,skip)
#define TRACE_AND_RESTORE_IRQ(en)		\
	stb	en,PACASOFTIRQEN(r13)
#endif
#endif

#endif
