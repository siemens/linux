/*
 * arch/powerpc/kernel/rtx_adapt_32.c
 *
 * Copyright (c) 2011 Siemens AG manfred.neugebauer@siemens.com
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
 * This file contains functions for the architecture dependent adaption.
 */

#include <linux/rtx_tickdev.h>
#include <linux/aud/rt_timer.h>
#ifdef CONFIG_RTX_EXTENDED_RT_DEBUGGING_SUPPORT
#include <linux/aud/rt_debug.h>
#endif

#include <asm/rtx_adapt.h>
#include <linux/rtx_percpu.h>
#include <linux/kernel_stat.h>

#include <asm/machdep.h>

#include <trace/events/irq.h>

/* Control structure for requested interrupts. */
static struct isr_cntrl rtx_req_irqs[RTX_NR_IRQS];

/* IRQ mode for /proc/interrupts. */
#define IRQ_MODE_HARD '*'
#define IRQ_MODE_SOFT '+'

irqreturn_t __rtx_flush_printk(int , void *);
irqreturn_t cancel_sync_migration(int irq, void *cookie);
irqreturn_t to_lx_isr(int irq, void *cookie);
irqreturn_t to_rt_isr(int irq, void *cookie);
irqreturn_t wake_up_lx_isr(int irq, void *cookie);

#ifdef CONFIG_RTX_DOMAIN_HRT

// calculation values for high res decrementer calculation
// TBD: maybe provided by  rtx_request_tickdev
#ifdef CONFIG_PPC_83xx
#define MY_MULT 0x10e56041 // values for division
#define MY_SHIFT 32
#define MY_MULT_REV 496485 // values for multiply
#define MY_SHIFT_REV 15
#elif defined(CONFIG_44x)
#define MY_MULT 0xccccccf7 // values for division
#define MY_SHIFT 32
#define MY_MULT_REV 5  // values for multiply
#define MY_SHIFT_REV 2 
#else
// these are dummy values to force an abortion of a realtime application
#define MY_MULT 0x0 // values for division
#define MY_SHIFT 32
#define MY_MULT_REV 0  // values for multiply
#define MY_SHIFT_REV 2 
#endif

unsigned long rtx_timer_freq;
static int rtx_next_htick_shot(unsigned long delay, struct clock_event_device *cdev);
static void rtx_switch_htick_mode(enum clock_event_mode mode, struct clock_event_device *cdev);
int rtx_ppc_decrementer_set_next_event(unsigned long evt, struct clock_event_device *dev);
DECLARE_PER_CPU(struct itimer, lx_itmr);

static int rtx_hres_active;
static int verboseDecr = 0;

#endif

#define DECREMENTER_MAX      0x7fffffff

int rtx_timer_init(void);
int rtxTimerStart(void);
int rtxTimerStop(void);

int rtx_oops;

int to_rt_virq = TO_RT_VIRQ;		// irq for waking up the realtime domain;
int to_lx_virq = TO_LX_VIRQ;		// irq for notifying the linux daemon for futex
					//and execution in Linux
int to_mg_virq = TO_MG_VIRQ;		// cancel synchronized migration
int wake_up_lx_virq = WAKE_UP_LX_VIRQ;	// irq for functions which require only
					// a wake_up_process and can therefore
					// executed directly from ISR level
int printk_virq = PRINTK_VIRQ;

int __rtx_timer_irq, __linux_timer_irq;	// the tick device may be different from the timer device

DECLARE_PER_CPU(struct list_head, wake_lx_list);
DEFINE_PER_CPU(struct pt_regs, rtx_tick_regs);
/* LX-interrupts are disabled on each CPU at startup. */
DEFINE_PER_CPU(struct rtx_pcd, rtx_percpu_data) ____cacheline_aligned =
		{ .rtx_irq_control = LX_IRQS_STALLED_MASK};

// structure to provide a dummy action for rt interrupt tracing
static struct irqaction rtDummyAction =  {
	.name = "RtAction",
};

extern void rt_task_leave_no_sync(void);

static void __rtx_end_irq(unsigned irq)
{
	struct irq_desc *desc;

        /* the timer (decrementer) irq runs separately */
	if (irq == __linux_timer_irq)
		return;

	desc = irq_to_desc(irq);
	desc->rtx_end(irq, desc);
}

/*
 * Called by the host kernel early during the boot procedure.
 */
void rtx_init(void)
{
	int cpu, i;

        /* we use number 0 as dummy interrupt number for the decrementer */
        __linux_timer_irq = 0; 
	rtx_timer_init();

	for_each_online_cpu(cpu) {
		per_cpu(rtx_mode, cpu) = LX_DOMAIN_NATIVE;
		rtx_percpu_var(rtx_irq_pending_words, cpu) = 0;
		rtx_percpu_var(rtx_softirq_pending_words, cpu) = 0;

		rtx_percpu_var(rtx_irqs, cpu)[TO_RT_VIRQ].handler = to_rt_isr;
		rtx_percpu_var(rtx_irqs, cpu)[TO_RT_VIRQ].cookie = NULL;
		rtx_percpu_var(rtx_irqs, cpu)[TO_MG_VIRQ].handler = cancel_sync_migration;
		rtx_percpu_var(rtx_irqs, cpu)[TO_MG_VIRQ].cookie = NULL;

		// VIRQs for the LX domain.
		rtx_percpu_var(rtx_irqs, cpu)[TO_LX_VIRQ].handler = to_lx_isr;
		rtx_percpu_var(rtx_irqs, cpu)[TO_LX_VIRQ].cookie = NULL;
		rtx_percpu_var(rtx_irqs, cpu)[WAKE_UP_LX_VIRQ].handler = wake_up_lx_isr;
		rtx_percpu_var(rtx_irqs, cpu)[WAKE_UP_LX_VIRQ].cookie = NULL;
		rtx_percpu_var(rtx_irqs, cpu)[PRINTK_VIRQ].handler = __rtx_flush_printk;
		rtx_percpu_var(rtx_irqs, cpu)[PRINTK_VIRQ].cookie = NULL;

		for (i = 0; i < RTX_IRQ_WORDS; i++) {
			rtx_percpu_var(rtx_irq_pending, cpu)[i] = 0;
			rtx_percpu_var(rtx_softirq_pending, cpu)[i] = 0;
		}
	}
}


/*
 * Handle pending interrupts. HW-Interrupts must be off.
 */
void __rtx_handle_nonrt_pending(unsigned long sync_mask)
{
	unsigned long part, pending_part, position, mask;
	struct rtx_pcd *pcd = &__raw_get_cpu_var(rtx_percpu_data);
	//struct pt_regs regs;
	unsigned int irq;

        //if (showFlag) printk("enter nonrt_pending\n");
	// Don't play the LX interrupts if we are
	// currently executing in the RT-domain.
	if (pcd->rtx_curr_dom == RT_DOMAIN)
		return;

	// SOFT-Mode: Handle pending RT-ISRs.
restart_softirq:
	while (pcd->rtx_softirq_pending_words) {
		part = __ffs(pcd->rtx_softirq_pending_words);

		while ((pending_part = pcd->rtx_softirq_pending[part])) {
			position = __ffs(pending_part);
			irq = (part << RTX_IRQ_WORDS_SHIFT) + position;
			__clear_bit(position, &pcd->rtx_softirq_pending[part]);
			if (pcd->rtx_softirq_pending[part] == 0)
				__clear_bit(part, &pcd->rtx_softirq_pending_words);
			__set_bit(LX_IRQS_STALLED, &pcd->rtx_irq_control);
			barrier();
			// The SOFT RT-ISR may set the LX-ISR pending.
                        trace_hardirqs_off();
                        //if (pcd->rtx_irqs[irq].handler)
			pcd->rtx_irqs[irq].handler(irq, pcd->rtx_irqs[irq].cookie);
			if (irq < NR_IRQS) {
				__rtx_end_irq(irq);
                        }
			local_irq_disable_hw();
			barrier();
#ifdef CONFIG_SMP
			/* we may have changed the CPU */
			pcd = &__raw_get_cpu_var(rtx_percpu_data);
#endif
                        trace_hardirqs_on();
			__clear_bit(LX_IRQS_STALLED, &pcd->rtx_irq_control);
		}
	}

	// Handle pending VIRQs, APIC-interrupts and LX-ISRs.
	while ((mask = (pcd->rtx_irq_pending_words & sync_mask)) != 0) {
		part = __ffs(mask);

		while ((pending_part = pcd->rtx_irq_pending[part]) != 0) {
			position = __ffs(pending_part);
			irq = (part << RTX_IRQ_WORDS_SHIFT) + position;
			__clear_bit(position, &pcd->rtx_irq_pending[part]);
			if (pcd->rtx_irq_pending[part] == 0)
				__clear_bit(part, &pcd->rtx_irq_pending_words);

			__set_bit(LX_IRQS_STALLED, &pcd->rtx_irq_control);
			barrier();
                        trace_hardirqs_off();
			/*
			 * As long as the RT-domain is registered,
			 * interrupts are enabled before calling the handler.
			 * We may have different types of pending interrupts:
			 * - VIRQs (soft-IRQ)
			 * - APIC interrupts (e.g. IPIs)
			 * - IRQs (Linux device interrupts)
			 */
                        /* m.n. this should also be true for soft realtime:
                           the soft realtime handler must have a higher priority
                           than the linux handler (problem with realtime serial driver */
			//if (rtx_get_cpu_var(rtx_mode) == RT_DOMAIN)
				local_irq_enable_hw();

			if (unlikely(irq >= RTX_VIRQ_BASE)) {
				__rtx_call_lx_virq_handler(irq, pcd->rtx_irqs[irq].handler, NULL);
			}
			else {
                            __rtx_move_lx_irq(irq);
                                if (irq == __linux_timer_irq) {
                                    struct pt_regs *tick_regs =
                                        &__raw_get_cpu_var(rtx_tick_regs);
                                    timer_interrupt(tick_regs);
                                }
	                        else {
                                    do_IRQ_from_rt(irq, NULL);
                                }
			}

			local_irq_disable_hw();
			barrier();
#ifdef CONFIG_SMP
			/* we may have changed the CPU */
			pcd = &__raw_get_cpu_var(rtx_percpu_data);
#endif
                        trace_hardirqs_on();
			__clear_bit(LX_IRQS_STALLED, &pcd->rtx_irq_control);
			if (pcd->rtx_softirq_pending_words)
				/* interrupted by soft-realtime interrupts
				   handle them first */
				goto restart_softirq;
		}
	}
}

/*
 * Calling the RT domain from LX.
 * VIRQs supported:
 * - to_rt_virq
 * - to_mg_virq
 */
void __rtx_call_rt_domain(int irq)
{ 
    	struct rtx_pcd *pcd;
	unsigned long domain;
	unsigned long flags;
        
	local_irq_save_hw(flags);

	pcd = &__raw_get_cpu_var(rtx_percpu_data);
	domain = pcd->rtx_curr_dom;
	pcd->rtx_curr_dom = RT_DOMAIN;			// RT domain is active

	pcd->rtx_irqs[irq].handler(irq, NULL);
	local_irq_disable_hw();

#ifdef CONFIG_RTX_DOMAIN_INTEGRITY_CHECK
	rtx_check_intr_flag();
#endif
	// RT interrupts enabled while threads are executing.
	// Interrupts off again when returning.
	rtx_preemption_handling();
	pcd->rtx_curr_dom = domain;				// preceding domain

	if (pcd->rtx_curr_dom == RT_DOMAIN || test_bit(LX_IRQS_STALLED, &pcd->rtx_irq_control)) {
		local_irq_restore_hw(flags);
		return;
	}
	if (rtx_pending_irqs(pcd))
		__rtx_handle_nonrt_pending(RTX_IRQMASK_ANY);
                local_irq_restore_hw(flags); 
}

#ifdef CONFIG_RTX_DOMAIN_HRT

static int rt_decr_register(void);
static int rt_decr_start(void);
static int rt_decr_unregister(void);

static struct  aud_timer decr_aud_timer = {
	.name = "RT_DECREMENTER",
	.ops = {
		.rt_register 	= rt_decr_register,
		.rt_start	= rt_decr_start,
		.rt_unregister	= rt_decr_unregister,
	},
};

static int rt_decr_start(void)
{
	if (verboseDecr)
		printkGen(NULL, "%s\n", __func__);
	return 0;
}


static int rt_decr_unregister(void)
{
	if (verboseDecr)
		printkGen(NULL, "%s\n", __func__);

	return 0;
}

static int rt_decr_register(void)
{
	if (verboseDecr)
		printkGen(NULL, "%s\n", __func__);
	// Set up for a tickless timing
	decr_aud_timer.irq = __rtx_timer_irq = __linux_timer_irq;
	decr_aud_timer.res_nsec = MY_MULT_REV >> MY_SHIFT_REV;
	return 0;
}

int rtx_timer_init(void)
{
	if (verboseDecr)
		printkGen(NULL, "%s:%s\n", __func__, "decr");
	aud_rt_timer = &decr_aud_timer;

	return(0);
}

int ack_ppc_timer(void)
{
	return(0);
}

int rtxTimerStart(void)
{
	if (verboseDecr)
		printkGen(NULL, "%s:%s\n", __func__, "decr");
	return(0);
}

int rtxTimerStop(void)
{
	if (verboseDecr)
		printkGen(NULL, "%s:%s\n", __func__, "decr");
	return(0);
}

//#define HIGHRES_USE_SIMPLE_CALCULATION

#define DECREMENTER_MIN_NEW_VALUE 32
void hres_setRefValue(unsigned newValue)
{
	unsigned long long setValue;
	unsigned long flags;

	local_irq_save_hw(flags);
	/* be sure that the setValue result is always bigger than zero */
	if (newValue < DECREMENTER_MIN_NEW_VALUE)
		newValue = DECREMENTER_MIN_NEW_VALUE;
#ifdef  HIGHRES_USE_SIMPLE_CALCULATION
	set_dec(((unsigned) newValue / 15)); // simple approach
#else
	setValue = (unsigned long long) newValue;
	setValue = setValue * MY_MULT;
	setValue >>= MY_SHIFT;
	set_dec(((unsigned)setValue));
#endif
	local_irq_restore_hw(flags);
	//printkGen(NULL, "%s: act=%#llx new=%#x:%#llx:%#llx\n",
	//	__func__, get_tb(), newValue, setValue);
}

#define COUNTER_OVERFLOW_MASK	0xfffff00000000000ULL
#define COUNTER_WORKING_MASK	0x00000fffffffffffULL

static unsigned long long myMaxNsValue =
	((((unsigned long long) (COUNTER_WORKING_MASK + 1))
		* MY_MULT_REV) >> MY_SHIFT_REV);
static unsigned long long myOverflowNsValue;
static unsigned long long myOldOvlValue;

RTX_TIMER_TYPE hres_getCntValue(void)
{
	u64 nsValue, cntValue, actOverflowValue;
	unsigned long flags;

	local_irq_save_hw(flags);

	cntValue = get_tb();
	actOverflowValue = cntValue & COUNTER_OVERFLOW_MASK;
	cntValue &= COUNTER_WORKING_MASK;
	if (actOverflowValue != myOldOvlValue) {
		myOverflowNsValue += myMaxNsValue;
		myOldOvlValue = actOverflowValue;
	}

#ifdef  HIGHRES_USE_SIMPLE_CALCULATION
	nsValue = cntValue * 15; // simple approach
#else
	nsValue = cntValue * MY_MULT_REV;
	nsValue >>= MY_SHIFT_REV;
	nsValue += myOverflowNsValue;
#endif
	local_irq_restore_hw(flags);

	//printkGen(NULL, "%s:act=%#llx:%#llx\n",
	//	__func__, cntValue, get_tb());

	return(nsValue);
}
#else // not CONFIG_RTX_DOMAIN_HRT

/*
 * tick based RT timing support was implemented for a first realtime
 * implementation with PPC 83xx
 */
int ack_ppc_timer(void);

#endif // CONFIG_RTX_DOMAIN_HRT

/*
 * This is the main interrupt handler called by
 * RT/LX device interrupts and LAPIC-interrupts.
 * Note: This entry is not used for VIRQs.
 * HW-Interrupts must be off.
 */

int __rtx_handle_irq(int irqnr, struct pt_regs *regs);

/* the decrementer interrupt comes in from a different interrupt channel */
void __rtx_handle_irq_timer(struct pt_regs *regs)
{
	struct rtx_pcd *pcd = &__raw_get_cpu_var(rtx_percpu_data);
	struct pt_regs *tick_regs = &__raw_get_cpu_var(rtx_tick_regs);
        
        local_irq_disable_hw();
	tick_regs->msr = regs->msr;
	tick_regs->nip = regs->nip;
	if (pcd->rtx_curr_dom == RT_DOMAIN) {
		//tick_regs->msr |= MSR_EE;
		if (user_mode(regs))
			current->utime++;
		else
			current->stime++;
	}
	/* Ensure a positive value is written to the decrementer, or else
	 * some CPUs will continuue to take decrementer exceptions */
	set_dec(DECREMENTER_MAX);

	__rtx_handle_irq(__linux_timer_irq, regs);
        local_irq_enable_hw();
}

/*
 * powerpc chip sets may have different version of interrupt handling
 * 
 * 83xx:
 * we see all interrupts coming from one type of controller
 * (the normal case) - beside the timer (see above).
 *
 * 44x:
 * some interrupts come from sub-interrupt controller (is_chained)
 * (e.g. interrupt nr. 16 and 30). In this case
 * the rtx_ack routine is not really an ack routine but calls a
 * real interrupt handler (e.g., uic_irq_cascade). (this modification
 * is done in kernel/irq/chip.c).
 * Within the cascade handler the main
 * interrupt is masked and acknowledged and the sub-interrupt number
 * is analyzed. This handler is also responsible to call the handler
 * responsible for the sub-interrupt.
 * With realtime instead of calling the (linux) handler of the
 * sub-interrupt we log this interrupt as (linux) pending during
 * the activities within the cascade interrupt.
 * When returning from this cascade interrupt we run through
 * __rtx_handle_irq() and __rtx_handle_nonrt_pending(). Here all logged
 * linux interrupts are handled. The cascade interrupt handler call
 * is only a dummy, but within the loop the handler of the cascade
 * sub-interrupt is also called.
 * Remark: currently no realtime interrupt within the cascade interrupt
 * are supported. This will require to transfer the realtime handling
 * within __rtx_handle_irq() to the cascade interrupt handler.
 */

void __rtx_handle_irq_ppc(struct pt_regs *regs)
{
	int irq;
	struct irq_desc *desc;

        local_irq_disable_hw();
	irq = ppc_md.get_irq();
        if (irq > 0) {
                desc = irq_to_desc(irq);
	        /* mask the interrupt. */
                desc->rtx_ack(irq, desc);
                if (irq == __rtx_timer_irq)
                	ack_ppc_timer();
	        __rtx_handle_irq(irq, regs);
        }
        local_irq_enable_hw();
}

//#define SHOW_ACTIVE_THREAD_DURING_RT_TIMING 1
#ifdef CONFIG_AUD_LOW_LEVEL_KERNEL_DUMP
int printkDirect(char *string);
int printkDirectHex(unsigned value);
void show_trace_direct(struct pt_regs *regs);
#ifdef SHOW_ACTIVE_THREAD_DURING_RT_TIMING
static unsigned myRtIrqCnt;
#endif
#endif
int __rtx_handle_irq(int irq, struct pt_regs *regs)
{
	struct rtx_pcd *pcd = &__raw_get_cpu_var(rtx_percpu_data);
	unsigned long domain;

	if (likely(pcd->rtx_irqs[irq].handler)) {

		trace_irq_entry_rt(-irq, regs, &rtDummyAction);
#ifdef CONFIG_RTX_DOMAIN_STOP_ON_PANIC
		// In case of a panic situation we leave RT interrupts 
		// on hold.
		if (rtx_oops) {
			return 0;
		}
#endif
                
		if (unlikely(rtx_get_cpu_var(rtx_mode) == LX_DOMAIN_SOFT)) {
			log_softirq_as_pending(irq);		// defer the SOFT RT-Isr
		}
		else {
			// This IRQ is using a handle_simple_irq flow.
			domain = pcd->rtx_curr_dom;
			pcd->rtx_curr_dom = RT_DOMAIN;
#ifdef SHOW_ACTIVE_THREAD_DURING_RT_TIMING
			if (irq == __rtx_timer_irq) {
				myRtIrqCnt++;
				if ((myRtIrqCnt % 5000) == 1) {
					printkDirect("rt_clock_isr:");
					printkDirect(current->comm);
					printkDirect(":");
					printkDirectHex(current->pid);
					printkDirect(":eip=");
					printkDirectHex(regs->nip);
					printkDirect("\r\n");
				}
			}
#endif
			pcd->rtx_irqs[irq].handler(irq, pcd->rtx_irqs[irq].cookie);
			local_irq_disable_hw();

			/*
			 * Note: We increment the interrupt count after calling
			 * the RT handler (to optimize the latency path).
			 */
			if (irq != __rtx_timer_irq) {
				rtx_kstat_incr_irqs_this_cpu(irq, irq_desc + irq);
			}

			if (irq < NR_IRQS)
				__rtx_end_irq(irq);

			// RT interrupts are enabled while threads are executing.
			rtx_preemption_handling();
			if (IS_REALTIME)
				pcd->rtx_curr_dom = domain;	// preceding (rt) domain
			else
				pcd->rtx_curr_dom = LX_DOMAIN_NATIVE; // we switched to lx
               }
        }
	else {
		// Interrupt logging (LX-interrupts)
		log_interrupt_as_pending(irq);
	}
        
	if (irq == __rtx_timer_irq) {
		if (pcd->rtx_curr_dom == RT_DOMAIN) {
			if (user_mode(regs))
				current->utime++;
			else
				current->stime++;
		}
	}

	/*
	* Threads that don't use system calls may also have
	* to migrate from RT to LX and vice versa.
	*/
	if (user_mode(regs)) {
		if (pcd->rtx_curr_dom == RT_DOMAIN) {
			if (IS_TASK_LEAVE_PENDING(current) || IS_EXIT_PENDING(current)) {
				current->rt_state2 = 0;
				rt_task_leave_no_sync();
				local_irq_disable_hw();
			}
#ifdef CONFIG_RTX_EXTENDED_RT_DEBUGGING_SUPPORT
			if (current->rt_state3 & RT_TASK_RT_DEBUG_PENDING) {
				rtx_migrate_to_lx(RTLX_TASK, NULL);
				local_irq_disable_hw();
                                // we are now in lx and should not be stalled
				return 1;
			}
#endif
		}
		else {
			if (current->rt_state & LXRT_TASK_LEAVE_PENDING) {
				rtx_migrate_to_rt(0);
				local_irq_disable_hw();
			}
		}
	}

	if (pcd->rtx_curr_dom == RT_DOMAIN || test_bit(LX_IRQS_STALLED, &pcd->rtx_irq_control))
		return 0;

	__rtx_handle_nonrt_pending(RTX_IRQMASK_ANY);

	return 1;
}


/* Interrupt depending stuff.*/
void __rtx_wake_lx_handling(struct proc_wait_queue *req)
{
	unsigned long flags;

	local_irq_save_hw(flags);
	list_add_tail(&req->list, &__raw_get_cpu_var(wake_lx_list));
	log_interrupt_as_pending(wake_up_lx_virq);
	local_irq_restore_hw(flags);
}


void rtx_init_isr_management(void)
{
	/* Nothing to do */
}


/*
 * Depending on the requirements a driver may need to split up its work having a realtime part constituted
 * by a "rt_handler" and an additional non-realtime part constituted by a "nonrt_handler".
 * The "rt_handler" always gets installed in the rtx_handler array, whereas the "nonrt_handler" gets installed
 * as an LX-handler. There is no difference between a system with a RT domain and a LX-only system.
 */
int execute_nonrt_handler(int flags, int irq)
{
	/* Interrupts are (must be) disabled */
	log_interrupt_as_pending(irq);
	return IRQ_NONE;
}


/*
 * Calling rt_request_irq() indicates, that an ISR for the realtime domain should be installed.
 * If no realtime domain is present, this call gets mapped to a regular request_irq() meaning the ISR gets
 * installed for the Linux domain. In case a driver has a non realtime part, it gets always handled in the
 * Linux domain by the nonrt_handler.
 */
int rt_request_irq(unsigned int irq, irqreturn_t (*rt_handler)(int, void *),
				   unsigned long irqflags, const char * devname, void *dev_id,
				   irqreturn_t (*nonrt_handler)(int, void *))
{
	irq_desc_t *desc;
	//struct rtx_pcd *pcd = &__raw_get_cpu_var(rtx_percpu_data);
	irqreturn_t (*act_handler)(int, void *) = nonrt_handler ? nonrt_handler : rt_handler;
#ifndef CONFIG_SMP
	int ii;
#endif

	if (RT_INIT_VERBOSE)
		printkGen(NULL, "rt_request_irq: irq=%d\n", irq);

	if (!rt_handler)
		return -EINVAL;

	if (irq >= NR_IRQS)
	{
		printkGen(NULL, "rt_request_irq: Invalid irq=%d\n", irq);
		return -EINVAL;
	}

	if (irqflags & IRQF_DISABLED) {
		printkGen(NULL, "rt_request_irq: IRQF_DISABLED flag not allowed\n");
		return -EINVAL;
	}

	irqflags |= IRQF_NOBALANCING;

	desc = irq_to_desc(irq);
	if ((irq != __linux_timer_irq) && 
			(!desc->chip->mask_ack || !desc->chip->unmask)) {
		printkGen(NULL, "rt_request_irq: (un)mask(ack) action not defined for irq %d\n",
			irq);
		return -EINVAL;
        }

        if (irq != __linux_timer_irq) 
	{
		if (request_irq(irq, act_handler, irqflags, devname, dev_id))
		{
			if (RT_FAULT_VERBOSE)
				printkGen(NULL, "could not install nonrt_handler as LX-ISR for irq=%d\n", irq );
			return -EFAULT;
		}
		if (RT_INIT_VERBOSE)
			printkGen(NULL, "registered nonrt_handler for irq=%d in LX domain\n", irq);
	}

	/*
	 * Since we are not sure that we are running on the target CPU,
	 * we only keep rt_handler/devid in mind and set it later on.
	 */
	rtx_req_irqs[irq].handler = rt_handler;
	rtx_req_irqs[irq].cookie = dev_id;

	/* 
	 * The remaining stuff has to be handled by rt_enable_irq_affinity().
	 * If we are already running on the RT-cpu the RT-process is assigned to,
	 * we can implicitly invoke rt_enable_irq_affinity().
	 * Otherwise, the caller is responsible to invoke this function explicitly,
	 * when the RT-process is assigned to its RT-cpu.
	 */
	if (IS_REALTIME_PROCESS(current)) {
		rt_enable_irq_affinity(irq);
		/*
		 * currently we are working for a consistent design
		 * for user driver interrupts with SMP / non-SMP configurations.
		 * the following is a workaround to bring this type of driver
		 * to operation.
		 */
#ifndef CONFIG_SMP
		if (irq == __rtx_timer_irq) {
			for (ii = 0; ii < NR_IRQS; ii++) {
				if (ii == __rtx_timer_irq) {
					continue;
				}
				if (rtx_req_irqs[ii].handler) {
					rt_enable_irq_affinity(ii);
				}
			}
		}
#endif
		return 1;
	}

	return 0;
}




void rt_free_irq(int irq, void *devid)
{
#ifndef CONFIG_SMP
	int ii;
#endif

	if (RT_SHUTDOWN_VERBOSE)
		printkGen(NULL, "rt_free_irq: irq=%d\n", irq);

	if (irq >= NR_IRQS)
	{
		printkGen(NULL, "rt_free_irq: Invalid irq=%d\n", irq);
		return;
	}

	/*
	 * If we are still running on the RT-cpu the RT-process is assigned to,
	 * we can implicitly invoke rt_disable_irq_affinity().
	 * Otherwise, the caller is responsible to invoke this function explicitly,
	 * when the RT-process is assigned to its RT-cpu.
	 */
	if (IS_REALTIME_PROCESS(current)) {
		rt_disable_irq_affinity(irq);
		/* see rt_request_irq() for explanation of the following code */
#ifndef CONFIG_SMP
		if (irq == __rtx_timer_irq) {
			for (ii = 0; ii < NR_IRQS; ii++) {
				if (ii == __rtx_timer_irq) {
					continue;
				}
				if (rtx_req_irqs[ii].handler) {
					rt_disable_irq_affinity(ii);
				}
			}
		}
#endif
        }

        if (irq != __linux_timer_irq) 
	{
		free_irq(irq, devid);				// remove LX ISR
		if ((RT_SHUTDOWN_VERBOSE) || (RT_INIT_VERBOSE))
			printkGen(NULL, "unregistered isr for irq=%d in LX domain\n", irq);
	}

	return;
}


/*
 * Set the RT-handler/cookie and adjust the affinity
 * for this IRQ.
 * Note: In this case we are running on the target CPU,
 * the RT-process is assigned to.
 * FIXME: Must be adapted for Multicore.
 */
int rt_enable_irq_affinity(int irq)
{
	irq_desc_t *desc;
	struct rtx_pcd *pcd = &__raw_get_cpu_var(rtx_percpu_data);

	if (RT_INIT_VERBOSE)
		printkGen(NULL, "%s: irq=%d\n", __func__, irq);

	if (irq < 0 || irq >= NR_IRQS)
	{
		printkGen(KERN_ALERT, "%s: invalid irq=%d\n", __func__, irq);
		return -EINVAL;
	}

	if (!IS_REALTIME_PROCESS(current)) {
		printkGen(KERN_ALERT, "%s: invalid process context for setting irq affinity (irq=%d)\n", __func__, irq);
		return -EINVAL;
	}

	desc = irq_to_desc(irq);
	// Install the rt_handler in the RT domain.
        // TBD: special action for timer?
	local_irq_disable_hw();
	pcd->rtx_irqs[irq].handler = rtx_req_irqs[irq].handler;
	pcd->rtx_irqs[irq].cookie = rtx_req_irqs[irq].cookie;

	// Activate the simple flow for the LX-Isr.
	desc->handle_lx = desc->handle_irq;
	desc->handle_irq = handle_simple_irq;

	if (irq == __rtx_timer_irq) {
#ifdef CONFIG_RTX_DOMAIN_HRT
		int res;

		// From now we have to emulate the LX ticks in the RT domain.
		if ((res = rtx_request_tickdev(HRES_LX_DEVICE_NAME,
                                               rtx_switch_htick_mode, rtx_next_htick_shot, &rtx_timer_freq))
					!= CLOCK_EVT_MODE_ONESHOT) {
			local_irq_enable_hw();
			printkGen(KERN_ALERT,
				"highres timer is not running in oneshot mode (res=%d)\n", res);
			return -EFAULT;
		}
		INIT_LIST_HEAD(&__raw_get_cpu_var(lx_itmr.it_link));
                // Fake a timer interrupt for linux to resolve old timer requests
                rtx_hres_active = 1;
		log_interrupt_as_pending(__linux_timer_irq);
#endif
		rtxTimerStart();
	}

	// Set the IRQ mode for /proc/interrupts.
	if (desc && desc->action) {
		if (rtx_get_cpu_var(rtx_mode))
			desc->action->mode = rtx_get_cpu_var(rtx_mode) ?
				IRQ_MODE_HARD : IRQ_MODE_SOFT;
	}

	local_irq_enable_hw();

#ifdef CONFIG_RTX_DOMAIN_HRT
	if (irq == __rtx_timer_irq) {
                printkGen(NULL, "rtx_request_tickdev done for %s freq=%d\n",
                          HRES_LX_DEVICE_NAME, rtx_timer_freq);
	}
#endif

	return 0;
}


/*
 * Uninstall the RT-handler/cookie and turn back the affinity
 * for this IRQ to be handled only in Linux.
 * Note: In this case we are running on the target CPU,
 * the RT-process is assigned to.
 * FIXME: Must be adapted for Multicore.
 */
int rt_disable_irq_affinity(int irq)
{
	irq_desc_t *desc;
	struct rtx_pcd *pcd = &__raw_get_cpu_var(rtx_percpu_data);

	if (RT_SHUTDOWN_VERBOSE)
		printkGen(NULL, "%s: irq=%d\n", __func__, irq);

	if (irq < 0 || irq >= NR_IRQS) {
		printkGen(KERN_ALERT, "%s: invalid irq=%d\n", __func__, irq);
		return -EINVAL;
	}

	if (!IS_REALTIME_PROCESS(current)) {
            printkGen(KERN_ALERT,
                      "%s: invalid process context for setting irq affinity (irq=%d)\n",
                      __func__, irq);
		return -EINVAL;
	}

	desc = irq_to_desc(irq);

	// Uninstall the rt_handler from the RT domain.
	local_irq_disable_hw();
	pcd->rtx_irqs[irq].handler = NULL;
	pcd->rtx_irqs[irq].cookie = NULL;

	// Deactivate the simple flow for the LX-Isr.
	desc->handle_irq = desc->handle_lx;

	// Set the IRQ mode for /proc/interrupts.
	if (desc && desc->action)
		desc->action->mode = 0;  // LX-only
	
	if (irq == __rtx_timer_irq) {
#ifdef CONFIG_RTX_DOMAIN_HRT
                rtx_hres_active = 0;
		// From now the LX ticks are handled native again.
		rtx_release_tickdev();
		rt_release_emulated_lx_tick();
		// Fake a timer interrupt for linux to restart timer requests
		log_interrupt_as_pending(irq);
#endif
		rtxTimerStop();
	}

	local_irq_enable_hw();

#ifdef CONFIG_RTX_DOMAIN_HRT
	if (irq == __rtx_timer_irq) {
                printkGen(NULL, "rtx_tickdev for %s removed\n", HRES_LX_DEVICE_NAME);
	}
#endif

	return 0;
}


/*
* All faults are handled in the Linux domain e.g. an installed handler gets executed in the Linux domain.
 * The base assumption is, that within time critical paths a thread has to avoid running into an exception.
 * So execution in the Linux domain should not pose a problem.
 * On the other hand it is the most elegant solution saving a lot of implementation efforts.
 */
int __rtx_trap_handling(int vector, struct pt_regs *regs)
{
	struct rtx_pcd *pcd;

	pcd = &__raw_get_cpu_var(rtx_percpu_data);
	if (IS_REALTIME) {
		switch(vector) {
		case RTX_TRAP_SINGLE_STEP_EXCEPTION:
		case RTX_TRAP_INSTR_BP_EXCEPTION:
		case RTX_TRAP_DEBUG_EXCEPTION:
			if (RT_DEBUG_VERBOSE)
				printkGen(NULL,
				"debug exception %d in RT domain at pc=%#lx (dar=%#x)\n",
				vector, regs->nip, regs->dar);
#ifdef CONFIG_RTX_EXTENDED_RT_DEBUGGING_SUPPORT
			rtx_stopAllRtThreads(current);
#endif
                        current->rt_state3 |= RT_TASK_LX_DEBUG_PENDING;
                        rtx_migrate_to_lx(RTLX_TASK, NULL);
	    		return 0;	// execute exception in the Linux domain

		case RTX_TRAP_ALIGNMENT_EXCEPTION:
			printkGen(KERN_ALERT, "alignment fault in RT domain at pc=%lx\n",
				regs->nip);
			rtx_migrate_to_lx(RTLX_TASK, NULL);
			show_regs(regs);
			return 0;

		default:
		if (RT_EXCEPTION_VERBOSE) {
			printkGen(KERN_ALERT,
				"fault %d in RT domain at pc=%#lx (dar=%#x)\n",
				vector, regs->nip, regs->dar);
		}
		current->rt_state3 |= RT_TASK_FAULT_DETECTED;
		rtx_migrate_to_lx(RTLX_TASK, NULL);
		return 0;	// execute exception in the Linux domain

		}
	}
	else if (pcd->rtx_curr_dom) {
#ifdef CONFIG_AUD_LOW_LEVEL_KERNEL_DUMP
		local_irq_disable_hw();
		printkDirect ("__rtx_trap_handling\n");
		printkDirectHex ((unsigned) current);
		printkDirect ("\n");
		show_trace_direct(regs);
		printkDirectHex (current->pid);
#endif
		panic("__rtx_trap_handling: RT domain without thread\n");
	}
	return 0;
}


/*
 * Implementation of the virtualized interrupt macros
 * ( see arch/powerpc/include/asm/irqflags.h).
 * Note: Only __rtx_local_irq_enable() has to be provided as a C-function because it is
 * called in entry-xxx.S.
 */
void __rtx_local_irq_enable(void)
{
	struct rtx_pcd *pcd;

	local_irq_disable_hw();
	pcd = &__raw_get_cpu_var(rtx_percpu_data);
	__clear_bit(LX_IRQS_STALLED, &pcd->rtx_irq_control);
	if (rtx_pending_irqs(pcd))
		__rtx_handle_nonrt_pending(RTX_IRQMASK_ANY);
	local_irq_enable_hw();
}


/* Spinlock functions. */
void __rtx_spin_lock_irq(raw_spinlock_t *lock)
{
	struct rtx_pcd *pcd;
	local_irq_disable_hw();
	pcd = &__raw_get_cpu_var(rtx_percpu_data);
	__raw_spin_lock(lock);
	__set_bit(LX_IRQS_STALLED, &pcd->rtx_irq_control);
}

void __rtx_spin_unlock_irq(raw_spinlock_t *lock)
{
	struct rtx_pcd *pcd = &__raw_get_cpu_var(rtx_percpu_data);
	__raw_spin_unlock(lock);
	__clear_bit(LX_IRQS_STALLED, &pcd->rtx_irq_control);
	local_irq_enable_hw();
}

unsigned long __rtx_spin_lock_irqsave(raw_spinlock_t *lock)
{
	struct rtx_pcd *pcd;
	unsigned long flags;
	int s;

	local_irq_save_hw(flags);
	pcd = &__raw_get_cpu_var(rtx_percpu_data);
	__raw_spin_lock(lock);
	s = __test_and_set_bit(LX_IRQS_STALLED, &pcd->rtx_irq_control);

	return raw_mangle_irq_bits(s, flags);
}

void __rtx_spin_unlock_irqrestore(raw_spinlock_t *lock, unsigned long x)
{
	struct rtx_pcd *pcd = &__raw_get_cpu_var(rtx_percpu_data);
	__raw_spin_unlock(lock);
	if (!raw_demangle_irq_bits(&x))
		__clear_bit(LX_IRQS_STALLED, &pcd->rtx_irq_control);
	local_irq_restore_hw(x);
}

void __rtx_spin_unlock_irqbegin(rtx_spinlock_t *lock)
{
    __raw_spin_unlock(&lock->__raw_lock);
}

void __rtx_spin_unlock_irqcomplete(unsigned long x)
{
	struct rtx_pcd *pcd = &__raw_get_cpu_var(rtx_percpu_data);
    if (!raw_demangle_irq_bits(&x))
        __clear_bit(LX_IRQS_STALLED, &pcd->rtx_irq_control);
    local_irq_restore_hw(x);
}

/*
 * Critical lock:
 * Excluding all CPUs but the current one from a critical section.
 */
unsigned long rtx_critical_enter(void)
{
	unsigned long flags;

	local_irq_save_hw(flags);

	return flags;
}

/*
 * Release the critical lock.
 */
void rtx_critical_exit(unsigned long flags)
{
	local_irq_restore_hw(flags);
}


#ifdef CONFIG_RTX_DOMAIN_HRT

/*
 * Callback for an emulated Linux tick.
 * This routine is called in ISR context.
 * Note: HW interrupts must be off.
 */
int rtx_set_lx_tick_pending(void *arg)
{
	//printkGen(NULL, "rtx_set_lx_tick_pending:%llx\n", get_tb());
	log_interrupt_as_pending(__linux_timer_irq);
	return 0;
}

/* Program the next shot for the host tick on the current CPU.
*
* @param delay - The time delta from the current date to the next tick,
* expressed as a count of nanoseconds.
*
* @param cdev  - An pointer to the clock device which notifies us.
*
* Environment:
*
* This routine is a callback invoked from the kernel's clock event
* handlers.
*
* @note - Only Linux kernel releases which support clock event devices
* (CONFIG_GENERIC_CLOCKEVENTS) would call this routine when the
* latter are programmed in oneshot mode. Otherwise, periodic host
* tick emulation is directly handled by the nucleus, and does not
* involve any callback mechanism from the Linux kernel.
*
* delay is nsec
* 
* Rescheduling: never.
*/
static int rtx_next_htick_shot(unsigned long delay, struct clock_event_device *cdev)
{
    unsigned long long clc = (unsigned long long) delay;
    
	clc = clc * MY_MULT;
	clc >>= MY_SHIFT;

	rtx_ppc_decrementer_set_next_event((unsigned long) clc, cdev);

	//printkGen(NULL, "rtx_next_htick delay=%#x:%llx\n", delay, get_tb());
	rt_emulate_lx_tick(delay, rtx_set_lx_tick_pending);

	return 0;
}

// delay is tick of decrementer
int rtx_repeat_next_htick_lx_shot(unsigned long delay)
{
	u64 cntValue = (u64) delay;

	cntValue = cntValue * MY_MULT_REV;
	cntValue >>= MY_SHIFT_REV;
	// RT domain mode
	if (rtx_hres_active) { 
		//printkGen(NULL, "rtx_repeat_next_htick delay=%#x:%#x:%llx\n",
			//delay, (unsigned) cntValue, get_tb());
		rt_emulate_lx_tick((unsigned) cntValue, rtx_set_lx_tick_pending);
		return(0);
	}

	// Linux only mode
	return(1);
}

/*
 * This callback announces a switch of the LX timer mode
 * (which is not expected).
 */
static void rtx_switch_htick_mode(enum clock_event_mode mode, struct clock_event_device *cdev)
{
	printkGen(NULL, "rtx_switch_htick_mode mode=%d\n", mode);
	// We always expect oneshot mode.
	if (mode != CLOCK_EVT_MODE_ONESHOT) {
		panic("%s: Unexpected timer mode switch for the RT domain (mode=%d)\n",
			__func__, mode);
		return;
	}
}

#endif

EXPORT_SYMBOL(rt_request_irq);
EXPORT_SYMBOL(execute_nonrt_handler);
EXPORT_SYMBOL(rt_free_irq);
EXPORT_SYMBOL(__rtx_handle_nonrt_pending);
EXPORT_SYMBOL(__rtx_local_irq_enable);
EXPORT_PER_CPU_SYMBOL(rtx_percpu_data);
