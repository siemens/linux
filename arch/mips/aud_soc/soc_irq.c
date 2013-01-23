/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2008 and 2011 Manfred.Neugebauer@siemens.com
 * Copyright (C) 2009 wolfgang.hartmann@siemens.com
 * Copyright (C) 2012 alexander.kubicki@siemens.com
 *
 * derived from work of 
 * Copyright (C) 1992 Linus Torvalds
 * Copyright (C) 1994 - 2000 Ralf Baechle
 */

/*
 * TBD: use set_irq_chip_and_handler()
 */ 
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/module.h>

#include <socGen.h>
#include <asm/mach-aud_soc/irq.h>

//#define SHOW_SOC_INTR_DETAIL	1
#define IRQ_SOC_TIMER		(54)

#define SOC1_IRQ_REG_NR_MAX	5
#define SOC_IRQ_NR_PER_WORD	32
#define SOC1_IRQ_LOWEST_PRIO	(NR_IRQ_ICU - 1)

#define SOC1_FIQ_REG_NR_MAX 8
#define SOC1_FIQ_LOWEST_PRIO	(SOC1_FIQ_REG_NR_MAX - 1)

struct icu_soc1_descr {
	unsigned irqLock;
	unsigned irqVector;
	unsigned irqVectorAndAck;
	unsigned irqClear;
	unsigned irqMaskAll;
	unsigned irqMaskReg[SOC1_IRQ_REG_NR_MAX];
	unsigned irqEnd;
	unsigned irqIrr[SOC1_IRQ_REG_NR_MAX];
	unsigned irqIsr[SOC1_IRQ_REG_NR_MAX];
	unsigned irqTrig[SOC1_IRQ_REG_NR_MAX];
	unsigned irqEdge[SOC1_IRQ_REG_NR_MAX];
	unsigned irqSwir[SOC1_IRQ_REG_NR_MAX];
	unsigned irqPrio[SOC1_IRQ_REG_NR_MAX * SOC_IRQ_NR_PER_WORD];
	unsigned irqStackReg;
	unsigned fiqUnlockRdOnlyAck;
	unsigned fiqVector;
	unsigned fiqAck;
	unsigned fiqClear;
	unsigned fiqMaskReg;
	unsigned fiqEnd;
	unsigned fiqIrr;
	unsigned fiqIsr;
	unsigned fiqTrig;
	unsigned fiqEdge;
	unsigned fiqSwirReg;
	unsigned fiqPrio[SOC1_FIQ_REG_NR_MAX];
	unsigned fiqSel[SOC1_FIQ_REG_NR_MAX];
	// others follow currently not yet used
};

static volatile struct icu_soc1_descr *icu_soc1_ref =
	(volatile struct icu_soc1_descr *) SOC_ICU_START;


#ifdef CONFIG_RTX_DOMAIN

#include <linux/aud/rt_timer.h>

asmlinkage int __rtx_handle_irq(int irq, struct pt_regs *regs);

#endif

#ifdef CONFIG_AuD_FAST_SPIN_LOCK_WORKAROUND

unsigned getPhysAddrInterruptSystem(void)
{
	unsigned mySubsystemAddr = SOC_ICU_START & 0x1fffffff;
        //printk("PhysAddr = %#x\n", mySubsystemAddr);
	return(mySubsystemAddr);
}

int getInterruptSystemDescr(struct audisInterruptSystemDescr *info)
{
    int maxNr = info->number;

    if (maxNr < 1)
      return(-EINVAL);

    info->descr[0].offset = (unsigned)&(icu_soc1_ref->irqMaskAll) -
        (unsigned) icu_soc1_ref;
    info->descr[0].lockValue = 1;
    info->descr[0].unlockValue = 0;
    info->number = 1;
    printk("interruptDescr nr=%d offset=%#x off=%#x on=%#x\n",
           info->number, info->descr[0].offset,
           info->descr[0].lockValue, info->descr[0].unlockValue);
    return(0);
}

#endif

unsigned getMaskAllState(void)
{
    return(icu_soc1_ref->irqMaskAll);
}

int setMaskAllState(int value)
{
    icu_soc1_ref->irqMaskAll = value;

    return(0);
}

int init_soc1_icu(void)
{
	int irq;
	int ii;

	// disable all icu interrupts
	icu_soc1_ref->irqMaskAll = 1;

	// unmask all interrupts
	for (ii = 0; ii < SOC1_IRQ_REG_NR_MAX; ii++) {
		icu_soc1_ref->irqMaskReg[ii] = 0xffffffff;
	}
	icu_soc1_ref->fiqMaskReg = 0xffffffff;
	// default: give every interrupt the lowest priority
        // clear interrupt request bit
	for (irq = 0; irq < SOC1_IRQ_REG_NR_MAX * SOC_IRQ_NR_PER_WORD; irq++) {
		icu_soc1_ref->irqPrio[irq] = SOC1_IRQ_LOWEST_PRIO;
		icu_soc1_ref->irqClear = irq;
	}
	for (irq = 0; irq < SOC1_FIQ_REG_NR_MAX; irq++) {
		icu_soc1_ref->fiqPrio[irq] = SOC1_FIQ_LOWEST_PRIO;
		icu_soc1_ref->fiqClear = irq;
		icu_soc1_ref->fiqSel[irq] = 0;
	}

        /* serial lines use level triggering */
        irq = IRQ_SERIAL_0 - SOC1_ICU_BASE;
	ii = irq / SOC_IRQ_NR_PER_WORD;
        icu_soc1_ref->irqTrig[ii] |= (1 << (irq % SOC_IRQ_NR_PER_WORD));
        irq = IRQ_SERIAL_1 - SOC1_ICU_BASE;
	ii = irq / SOC_IRQ_NR_PER_WORD;
        icu_soc1_ref->irqTrig[ii] |= (1 << (irq % SOC_IRQ_NR_PER_WORD));


        /* set icu prios for basic io devices */
        irq = IRQ_TIMERTOP_0 - SOC1_ICU_BASE;
	icu_soc1_ref->irqPrio[irq] = IRQ_TIMERTOP_0_ICU_PRIO;
        irq = IRQ0_SP - SOC1_ICU_BASE;
	icu_soc1_ref->irqPrio[irq] = IRQ_SP0_ICU_PRIO;
        irq = IRQ1_SP - SOC1_ICU_BASE;
	icu_soc1_ref->irqPrio[irq] = IRQ_SP1_ICU_PRIO;
        irq = IRQ_SERIAL_0 - SOC1_ICU_BASE;
	icu_soc1_ref->irqPrio[irq] = IRQ_SERIAL_0_ICU_PRIO;
        irq = IRQ_SERIAL_1 - SOC1_ICU_BASE;
	icu_soc1_ref->irqPrio[irq] = IRQ_SERIAL_1_ICU_PRIO;
        
	return(0);
}

// allow a modification of the prio from a kernel driver
// warning irq argument is linux irq number, not the index to the hardware array
int set_irq_icu_prio(int irq_nr, int prio)
{
	int ii;
    
	ii = irq_nr - SOC1_ICU_BASE;
	if ((ii < 0) || (ii >= NR_IRQ_ICU)) {
		return(-EINVAL);
        }
	if ((prio < 0) || (prio >= NR_IRQ_ICU)) {
		return(-EINVAL);
        }

	icu_soc1_ref->irqPrio[ii] = SOC1_IRQ_LOWEST_PRIO;

        return(0);
}
EXPORT_SYMBOL (set_irq_icu_prio);

// edge triggered interrupt
int set_irq_icu_edge(int irq_nr, int edge)
{
	int ii;
	int irq;
    
	
	irq = irq_nr - SOC1_ICU_BASE;
	ii = irq / SOC_IRQ_NR_PER_WORD;

	if (edge)
	{
		icu_soc1_ref->irqEdge[ii] |= (1 << (irq % SOC_IRQ_NR_PER_WORD));
	}
	else
	{
		icu_soc1_ref->irqEdge[ii] &= ~(1 << (irq % SOC_IRQ_NR_PER_WORD));
	}

        // be sure we have an edge triggered mode
	icu_soc1_ref->irqTrig[ii] &= ~(1 << (irq % SOC_IRQ_NR_PER_WORD));
        return(0);
}
EXPORT_SYMBOL (set_irq_icu_edge);


int enable_soc1_icu(void)
{
	// enable all unmasked icu interrupts
	icu_soc1_ref->irqMaskAll = 0;

	return(0);
}

static inline int readIntrRequ(int irqBase)
{
	int intr;

	if ((intr = icu_soc1_ref->irqVectorAndAck) != 0)
		return(intr);

	return(-1);
}

static int mips_soc_irqdispatch(int intrNr)
{
	int retVal;
	int irq;
#ifdef CONFIG_RTX_DOMAIN
	struct pt_regs *regs;
#endif

	intrNr *= 16; /* we start with 32 */
	// read vector number and ack
	retVal = readIntrRequ(intrNr);

#ifdef SHOW_SOC_INTR_DETAIL
	if (retVal != (IRQ_SOC_TIMER - intrNr))
		printk("enter mips_soc_irqdispatch intnr=%d ret=%#x\n",
			intrNr, retVal);
#endif

	if (unlikely(retVal < 0))
          return(0);

	irq = intrNr + retVal;
        
#ifdef CONFIG_RTX_DOMAIN
	regs = current_thread_info()->regs;
	retVal = __rtx_handle_irq(irq, regs);
#else
	do_IRQ(irq);
        retVal = 0;
#endif
        return(retVal);
}


void showIrqState(unsigned int irq_nr)
{
	unsigned helpVal;

	if (irq_nr < SOC1_ICU_BASE) {
		printk("showIrqState: wrong configuration (irq=%d)\n", irq_nr);
		return;
	}
	if (irq_nr < (SOC1_IRQ_RESERVE_BASE_2)) {
		irq_nr -= SOC1_ICU_BASE;
		helpVal = irq_nr / SOC_IRQ_NR_PER_WORD;
                printkGen(NULL, "irq=%d mask=%#x irr=%#x isr=%#x\n",
                          irq_nr, 
                          icu_soc1_ref->irqMaskReg[helpVal],
                          icu_soc1_ref->irqIrr[helpVal],
                          icu_soc1_ref->irqIsr[helpVal]);
		return;
	}
}


void disable_soc_irq_direct(unsigned int irq_nr)
{
	if (irq_nr < 8) {
		clear_c0_status(CAUSEF_IP0 << irq_nr);
	}
}

void enable_soc_irq_direct(unsigned int irq_nr)
{
	if (irq_nr < 8) {
//		set_c0_status(CAUSEF_IP0 << irq_nr);
	}
}

static unsigned int startup_soc_irq_direct(unsigned int irq)
{
	enable_soc_irq_direct(irq);
	return 0; /* never anything pending */
}

#define shutdown_soc_irq_direct	disable_soc_irq_direct

#define mask_and_ack_soc_irq_direct disable_soc_irq_direct

static void end_soc_irq_direct(unsigned int irq)
{
	disable_soc_irq_direct(irq);
//??	if (!(irq_desc[irq].status & (IRQ_DISABLED|IRQ_INPROGRESS)))
//??		enable_atlas_irq(irq);
}

// m.n. cascading

void disable_soc_irq_cascade(unsigned int irq_nr)
{
	unsigned helpVal;

#ifdef SHOW_SOC_INTR_DETAIL
	if (irq_nr != IRQ_SOC_TIMER)
		printk("disable_soc_irq_cascade intnr=%d\n", irq_nr);
#endif

	if (irq_nr < SOC1_ICU_BASE) {
		printk("disable_soc_irq_cascade: wrong configuration (irq=%d)\n", irq_nr);
		return;
	}
	if (irq_nr < (SOC1_IRQ_RESERVE_BASE_2)) {
		irq_nr -= SOC1_ICU_BASE;
		helpVal = irq_nr / SOC_IRQ_NR_PER_WORD;
		icu_soc1_ref->irqMaskReg[helpVal] |=
			(1 << (irq_nr % SOC_IRQ_NR_PER_WORD));
#if defined(CONFIG_RTX_DOMAIN) && defined(CONFIG_HW_LOGGING)
		rtx_shadow_mask[helpVal+1] |= (1 << (irq_nr % SOC_IRQ_NR_PER_WORD));	// skip placeholder
#endif
		return;
	}

	printk("disable_soc_irq_cascade: wrong configuration (irq=%d)\n", irq_nr);

	return;
}

void enable_soc_irq_cascade(unsigned int irq_nr)
{
	unsigned helpVal;

#ifdef SHOW_SOC_INTR_DETAIL
	if (irq_nr != IRQ_SOC_TIMER)
		printk("enable_soc_irq_cascade intnr=%d\n", irq_nr);
#endif
	if (irq_nr < SOC1_ICU_BASE) {
		printk("enable_soc_irq_cascade: wrong configuration (irq=%d)\n", irq_nr);
		return;
	}
	if (irq_nr < (SOC1_IRQ_RESERVE_BASE_2)) {
		irq_nr -= SOC1_ICU_BASE;
		helpVal = irq_nr / SOC_IRQ_NR_PER_WORD;
#if defined(CONFIG_RTX_DOMAIN) && defined(CONFIG_HW_LOGGING)
		if (!test_bit(LX_IRQS_STALLED, &rtx_irq_control))
#endif
		icu_soc1_ref->irqMaskReg[helpVal] &=
			~(1 << (irq_nr % SOC_IRQ_NR_PER_WORD));
#if defined(CONFIG_RTX_DOMAIN) && defined(CONFIG_HW_LOGGING)
		rtx_shadow_mask[helpVal+1] &= ~(1 << (irq_nr % SOC_IRQ_NR_PER_WORD));	// skip placeholder
#endif
		return;
	}

	printk("enable_soc_irq_cascade: wrong configuration (irq=%d)\n", irq_nr);

	return;
}


#ifdef CONFIG_HW_LOGGING
void enable_soc_irq(unsigned int irq_nr)
{
	unsigned helpVal;

	irq_nr -= SOC1_ICU_BASE;
	helpVal = irq_nr / SOC_IRQ_NR_PER_WORD;

	icu_soc1_ref->irqMaskReg[helpVal] &= ~(1 << (irq_nr % SOC_IRQ_NR_PER_WORD));
	return;
}
#endif

#if defined(CONFIG_RTX_DOMAIN) && defined(CONFIG_HW_LOGGING)
void get_soc_irq_cascade(unsigned long *mask)
{
	int ii;

	for (ii=0; ii < RTX_IRQ_HW_PARTS; ii++)
		mask[ii+1] = icu_soc1_ref->irqMaskReg[ii];		// skip placeholder in the mask
}

void set_soc_irq_cascade(unsigned long *mask)
{
	int ii;

	for (ii=0; ii < RTX_IRQ_HW_PARTS; ii++)
		icu_soc1_ref->irqMaskReg[ii] = mask[ii+1];		// skip placeholder in the mask
}
#endif

static unsigned int startup_soc_irq_cascade(unsigned int irq_nr)
{
	unsigned irqAct = irq_nr;

	// we do things here which are only required once and will never be disabled during operation

#ifdef SHOW_SOC_INTR_DETAIL
	if (irq_nr != IRQ_SOC_TIMER)
		printk("startup_soc_irq_cascade intnr=%d\n", irq_nr);
#endif
	if (irq_nr < SOC1_ICU_BASE) {
		printk("enable_soc_irq_cascade: wrong configuration (irq=%d)\n", irq_nr);
		return(0);
	}
	if (irq_nr < (SOC1_IRQ_RESERVE_BASE_2)) {
		enable_soc_irq_cascade(irqAct);
		set_c0_status(CAUSEF_IP2);
		return(0);
	}

	printk("startup_soc_irq_cascade: wrong configuration (irq=%d)\n", irq_nr);

	return 0; /* never anything pending */
}

#define shutdown_soc_irq_cascade	disable_soc_irq_cascade

void mask_and_ack_soc_irq_cascade(unsigned int irq_nr)
{
	unsigned helpVal;

#ifdef SHOW_SOC_INTR_DETAIL
	if (irq_nr != IRQ_SOC_TIMER)
		printk("mask_and_ack_soc_irq_cascade intnr=%d\n", irq_nr);
#endif
	if (irq_nr < SOC1_ICU_BASE) {
		printk("mask_ack_soc_irq_cascade: wrong configuration (irq=%d)\n",
			irq_nr);
		return;
	}
	if (irq_nr < (SOC1_IRQ_RESERVE_BASE_2)) {
		irq_nr -= SOC1_ICU_BASE;
		helpVal = irq_nr / SOC_IRQ_NR_PER_WORD;
		icu_soc1_ref->irqMaskReg[helpVal] |=
			(1 << (irq_nr % SOC_IRQ_NR_PER_WORD));
#if defined(CONFIG_RTX_DOMAIN) && defined(CONFIG_HW_LOGGING)
		rtx_shadow_mask[helpVal+1] |= (1 << (irq_nr % SOC_IRQ_NR_PER_WORD));	// skip placeholder
#endif
		icu_soc1_ref->irqEnd = 0;
		// we did an early EOI and blocked the MIPS interrupt
		// now it is time to enable other interrupts from
		// the interrupt controller again
		// m.n. is this really true???
		set_c0_status(CAUSEF_IP2);
		return;
	}

	printk("mask_and_ack_soc_irq_cascade: wrong configuration (irq=%d)\n", irq_nr);

	return;
}

#if defined(CONFIG_RTX_DOMAIN) && defined(CONFIG_HW_LOGGING)
void eoi_soc_irq(void)
{
	icu_soc1_ref->irqEnd = 0;
	set_c0_status(CAUSEF_IP2);
	return;
}
void mask_and_ack_soc_irq(unsigned int irq_nr)
{
	unsigned helpVal;

	irq_nr -= SOC1_ICU_BASE;
	helpVal = irq_nr / SOC_IRQ_NR_PER_WORD;
	icu_soc1_ref->irqMaskReg[helpVal] |= (1 << (irq_nr % SOC_IRQ_NR_PER_WORD));
	icu_soc1_ref->irqEnd = 0;
	// we did an early EOI and blocked the MIPS interrupt
	// now it is time to enable other interrupts from
	// the interrupt controller again
	// m.n. is this really true???
	set_c0_status(CAUSEF_IP2);
	return;
}
#endif

static void end_soc_irq_cascade(unsigned int irq)
{
#ifdef SHOW_SOC_INTR_DETAIL
	if (irq != IRQ_SOC_TIMER)
		printk("end_soc_irq_cascade intnr=%d\n", irq);
#endif
	if (!(irq_desc[irq].status & (IRQ_DISABLED|IRQ_INPROGRESS)))
		enable_soc_irq_cascade(irq);
}


// read the interrupt state of the interrupt register
// (not masked) and clear this bit
int get_and_reset_irq_state_cascade(unsigned int irq_nr)
{
	int state = 0, helpVal;

	if (irq_nr < SOC1_ICU_BASE) {
error_get_and_reset:
		printk("get_and_reset_irq_state_cascade: wrong configuration (irq=%d)\n",
			irq_nr);
		return(state);
	}
	if (irq_nr < (SOC1_IRQ_RESERVE_BASE_2)) {
		irq_nr -= SOC1_ICU_BASE;
		helpVal = irq_nr / SOC_IRQ_NR_PER_WORD;
		icu_soc1_ref->irqMaskReg[helpVal] |=
			(1 << (irq_nr % SOC_IRQ_NR_PER_WORD));
#if (0)
		printk("get_and_reset:%d:%#x\n", irq_nr,
			icu_soc1_ref->irqIrr[helpVal]);
#endif
		state = icu_soc1_ref->irqIrr[helpVal]
			& (1 << (irq_nr % SOC_IRQ_NR_PER_WORD));
		icu_soc1_ref->irqClear = irq_nr;
	}
	else
		goto error_get_and_reset;

	return(state);
}


void disable_soc_irq_nop(unsigned int irq_nr)
{
}

void enable_soc_irq_nop(unsigned int irq_nr)
{
}

static unsigned int startup_soc_irq_nop(unsigned int irq)
{
	return 0; /* never anything pending */
}

#define shutdown_soc_irq_nop	disable_soc_irq_nop

#define mask_and_ack_soc_irq_nop disable_soc_irq_nop

static void end_soc_irq_nop(unsigned int irq)
{
}


static struct hw_interrupt_type soc_irq_type_direct = {
	.typename = "AuD Soc direct",
	.startup = startup_soc_irq_direct,
	.shutdown = shutdown_soc_irq_direct,
	.enable = enable_soc_irq_direct,
	.disable = disable_soc_irq_direct,
	.ack = mask_and_ack_soc_irq_direct,
	.end = end_soc_irq_direct,
};

static struct hw_interrupt_type soc_irq_type_cascade = {
	.typename = "AuD Soc cascade",
	.startup = startup_soc_irq_cascade,
	.shutdown = shutdown_soc_irq_cascade,
	.enable = enable_soc_irq_cascade,
	.unmask = enable_soc_irq_cascade,
	.disable = disable_soc_irq_cascade,
	.mask = disable_soc_irq_cascade,
	.ack = mask_and_ack_soc_irq_cascade,
	.mask_ack = mask_and_ack_soc_irq_cascade,
	.end = end_soc_irq_cascade,
};

static struct hw_interrupt_type soc_irq_type_nop = {
	.typename = "AuD Soc nop",
	.startup = startup_soc_irq_nop,
	.shutdown = shutdown_soc_irq_nop,
	.enable = enable_soc_irq_nop,
	.disable = disable_soc_irq_nop,
	.ack = mask_and_ack_soc_irq_nop,
	.end = end_soc_irq_nop,
};


/*
 * interrupt setup on mips
 *
 * in kernel/traps.c trap_init()
 * set_handler() copies a short assembler routine which starts at address 0x80000180 or others
 * set_except_vector(int nr, void *handler) sets handler in an array (0 ... 31)
 *         the 32 indexes correspond with the 32 exception code in the cause register
 *         (index 0 is for interrupts, that's what we use below)
 * the exception entry at address 0x80000180 (a.o.) uses the exception code to jump to the handler in the array
 *
 * consequence: mips_soc_IRQ() is our global handler for all hardware (I/O) interrupts
 */
void __init arch_init_irq(void)
{
	int ii;

	printk("mips soc1 arch_init_irq\n");

	init_soc1_icu();
        
	// T B D: we may use the subroutines defined in kernel/irq_cpu.c
	//mips_cpu_irq_init (MIPSCPU_INT_BASE);

	for (ii = SOC1_AUD_INTR_BASE; ii < NR_DIRECT_IRQ; ii++) {
		irq_desc[ii].status	= IRQ_DISABLED;
		irq_desc[ii].action	= 0;
		irq_desc[ii].depth	= 1;
		irq_desc[ii].chip	= &soc_irq_type_direct;
		irq_desc[ii].handle_irq = handle_level_irq;
		spin_lock_init(&irq_desc[ii].lock);
	}

	for (ii = 0; ii < NR_FAST_DIRECT_IRQ; ii++) {
		irq_desc[ii + SOC1_IRQ_FAST_DIRECT_BASE].status	= IRQ_DISABLED;
		irq_desc[ii + SOC1_IRQ_FAST_DIRECT_BASE].action	= 0;
		irq_desc[ii + SOC1_IRQ_FAST_DIRECT_BASE].depth	= 1;
		irq_desc[ii + SOC1_IRQ_FAST_DIRECT_BASE].chip	= &soc_irq_type_nop;
		irq_desc[ii + SOC1_IRQ_FAST_DIRECT_BASE].handle_irq = handle_level_irq;
		spin_lock_init(&irq_desc[ii + SOC1_IRQ_FAST_DIRECT_BASE].lock);
	}

	for (ii = 0; ii < NR_IRQ_RESERVE_1; ii++) {
		irq_desc[ii + SOC1_IRQ_RESERVE_BASE_1].status	= IRQ_DISABLED;
		irq_desc[ii + SOC1_IRQ_RESERVE_BASE_1].action	= 0;
		irq_desc[ii + SOC1_IRQ_RESERVE_BASE_1].depth	= 1;
		irq_desc[ii + SOC1_IRQ_RESERVE_BASE_1].chip	= &soc_irq_type_nop;
		irq_desc[ii + SOC1_IRQ_RESERVE_BASE_1].handle_irq = handle_level_irq;
		spin_lock_init(&irq_desc[ii + SOC1_IRQ_RESERVE_BASE_1].lock);
	}

	for (ii = 0; ii < NR_IRQ_ICU; ii++) {
		irq_desc[ii + SOC1_ICU_BASE].status	= IRQ_DISABLED;
		irq_desc[ii + SOC1_ICU_BASE].action	= 0;
		irq_desc[ii + SOC1_ICU_BASE].depth	= 1;
		irq_desc[ii + SOC1_ICU_BASE].chip	= &soc_irq_type_cascade;
		irq_desc[ii + SOC1_ICU_BASE].handle_irq = handle_level_irq;
		spin_lock_init(&irq_desc[ii + SOC1_ICU_BASE].lock);
	}

	for (ii = 0; ii < NR_IRQ_RESERVE_2; ii++) {
		irq_desc[ii + SOC1_IRQ_RESERVE_BASE_2].status	= IRQ_DISABLED;
		irq_desc[ii + SOC1_IRQ_RESERVE_BASE_2].action	= 0;
		irq_desc[ii + SOC1_IRQ_RESERVE_BASE_2].depth	= 1;
		irq_desc[ii + SOC1_IRQ_RESERVE_BASE_2].chip	= &soc_irq_type_nop;
		irq_desc[ii + SOC1_IRQ_RESERVE_BASE_2].handle_irq = handle_level_irq;
		spin_lock_init(&irq_desc[ii + SOC1_IRQ_RESERVE_BASE_2].lock);
	}

	/* Now safe to set the exception vector.        */
	/* will be done by the general startup code */

        enable_soc1_icu();

}

/*
 * Version of ffs that only looks at bits 12..15.
 * (clz and ffs from malta-int.c)
 */
static inline int clz(unsigned long x)
{
	__asm__(
	"	.set	push					\n"
	"	.set	mips32					\n"
	"	clz	%0, %1					\n"
	"	.set	pop					\n"
	: "=r" (x)
	: "r" (x));

	return x;
}

static inline unsigned int irq_ffs(unsigned int pending)
{
#if defined(CONFIG_CPU_MIPS32) || defined(CONFIG_CPU_MIPS64)
	return -clz(pending) + 31 - CAUSEB_IP;
#else
	unsigned int a0 = 7;
	unsigned int t0;

	t0 = pending & 0xf000;
	t0 = t0 < 1;
	t0 = t0 << 2;
	a0 = a0 - t0;
	pending = pending << t0;

	t0 = pending & 0xc000;
	t0 = t0 < 1;
	t0 = t0 << 1;
	a0 = a0 - t0;
	pending = pending << t0;

	t0 = pending & 0x8000;
	t0 = t0 < 1;
	/* t0 = t0 << 2; */
	a0 = a0 - t0;
	/* pending = pending << t0; */

	return a0;
#endif
}

/*
 * The IRQs on the MIPS board look basically (barring software
 * IRQs which we don't use at all) like:
 * IRQs on the Malta board look basically (barring software IRQs which we
 * don't use at all and all external interrupt sources are combined together
 * on hardware interrupt 0 (MIPS IRQ 2)) like:
 *
 *	MIPS IRQ	Source
 *      --------        ------
 *             0	Software (ignored)
 *             1        Software (ignored)
 *             2        Combined hardware interrupt
 *             3        Combined hardware interrupt (not yet used)
 *             4        Hardware (ignored)
 *             5        Hardware (ignored)
 *             6        Hardware (ignored)
 *             7        R4k timer (we may use it later)
 *
 * We handle the IRQ according to _our_ priority which is:
 *
 * Highest ----     R4k Timer (not yet used)
 * Lowest  ----     Combined hardware interrupts
 *
 * then we just return, if multiple IRQs are pending then we will just take
 * another exception, big deal.
 */

#define CASCADE_IRQ		2

asmlinkage int plat_irq_dispatch(void)
{
	unsigned int pending = read_c0_cause() & read_c0_status() & ST0_IM;
	int irq;
        int retVal;

	irq = irq_ffs(pending);

	if (irq == CASCADE_IRQ)
		retVal = mips_soc_irqdispatch(irq);
	else {
		spurious_interrupt();
		retVal = 0;
        }

        return(retVal);
}


/*
 * support for software triggered interrupts
 * to force linux soft irq handling
 */
static irqreturn_t mips_soc_soft_interrupt(int irq, void *dev_id)
{
	if (irq == IRQ_SOFTWARE_0) {
		clear_c0_cause(CAUSEF_IP0);
	}
	if (irq == IRQ_SOFTWARE_1) {
		clear_c0_cause(CAUSEF_IP1);
	}
        /*	printk("enter soft interrupt %d\n", irq); */
	return(IRQ_HANDLED);
}


int initSoftwareHandler(void)
{
	int retVal;

	retVal = request_irq(IRQ_SOFTWARE_0, mips_soc_soft_interrupt, 0,
		"mips_soc_soft_irq0", NULL);
	retVal = request_irq(IRQ_SOFTWARE_1, mips_soc_soft_interrupt, 0,
		"mips_soc_soft_irq1", NULL);

	return(retVal);
}

int rtx_raise_soft_irq(int irqnr)
{
	unsigned long flags;
	int retVal = -EINVAL;
        
	local_irq_save_hw(flags);
	if (irqnr == IRQ_SOFTWARE_0) {
		set_c0_cause(CAUSEF_IP0);
		retVal = 0;
	}
	else if (irqnr == IRQ_SOFTWARE_1) {
		set_c0_cause(CAUSEF_IP1);
		retVal = 0;
	}

	local_irq_restore_hw(flags); 
	return(retVal);
}
EXPORT_SYMBOL(rtx_raise_soft_irq);
