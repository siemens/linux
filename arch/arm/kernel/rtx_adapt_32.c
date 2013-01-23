/*
 * arch/arm/kernel/rtx_adapt_32.c
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
 * This file contains functions for the architecture dependent adaption.
 */
#include <linux/rtx_tickdev.h>
#include <linux/aud/rt_timer.h>
#include <asm/traps.h>
#ifdef CONFIG_RTX_EXTENDED_RT_DEBUGGING_SUPPORT
#include <linux/aud/rt_debug.h>
#endif

#include <asm/rtx_adapt.h>
#include <linux/rtx_percpu.h>
#include <linux/kernel_stat.h>

#include <linux/tick.h>

#include <trace/events/irq.h>

unsigned int cpu_khz;
EXPORT_SYMBOL(cpu_khz);

#ifdef CONFIG_AUD_LOW_LEVEL_KERNEL_DUMP
int printkDirect(char *string);
int printkDirectHex(unsigned value);
void show_trace_direct(struct pt_regs *regs);
#endif

#ifdef CONFIG_ARCH_FEROCEON
unsigned long mvBoardTclkGet(void);
#endif

static unsigned long long rtx_cpu_freq;
unsigned long long clockfreq;
#ifdef RTX_HAVE_LLMULSHFT
unsigned int tsc_scale, tsc_shift;
#ifdef RTX_HAVE_NODIV_LLIMD
rtx_u32frac_t tsc_frac;
rtx_u32frac_t bln_frac;
#endif
#endif	

#ifdef CONFIG_RTX_DOMAIN_HRT
unsigned long rtx_timer_freq;

static int rtx_next_htick_shot(unsigned long delay, struct clock_event_device *cdev);
static void rtx_switch_htick_mode(enum clock_event_mode mode, struct clock_event_device *cdev);

unsigned long long clockfreq;
#endif

/* IRQ mode for /proc/interrupts. */
#define IRQ_MODE_HARD '*'
#define IRQ_MODE_SOFT '+'

DECLARE_PER_CPU(struct list_head, wake_lx_list);
DEFINE_PER_CPU(struct pt_regs, rtx_tick_regs);
/* LX-interrupts are disabled on each CPU at startup. */
DEFINE_PER_CPU(struct rtx_pcd, rtx_percpu_data) ____cacheline_aligned =
		{ .rtx_irq_control = LX_IRQS_STALLED_MASK};

extern cpumask_t rtx_cpu_onlinemask;

extern irqreturn_t __rtx_flush_printk(int , void *);
extern void rt_task_leave_no_sync(void);

irqreturn_t cancel_sync_migration(int irq, void *cookie);
irqreturn_t to_lx_isr(int irq, void *cookie);
irqreturn_t to_rt_isr(int irq, void *cookie);
irqreturn_t wake_up_lx_isr(int irq, void *cookie);

DECLARE_PER_CPU(struct itimer, lx_itmr);
DECLARE_PER_CPU(struct list_head, wake_lx_list);

int rtx_oops;

int to_rt_virq = TO_RT_VIRQ;				// irq for waking up the realtime domain;
int to_lx_virq = TO_LX_VIRQ;				// irq for notifying the linux daemon for futex and execution in Linux
int to_mg_virq = TO_MG_VIRQ;				// cancel synchronized migration
int wake_up_lx_virq = WAKE_UP_LX_VIRQ;		// irq for functions which require only a wake_up_process and can therefore
											// executed directly from ISR level
int printk_virq = PRINTK_VIRQ;

int __rtx_timer_irq, __rtx_tick_irq;		// the tick device may be different from the timer device

// structure to provide a dummy action for rt interrupt tracing
static struct irqaction rtDummyAction =  {
	.name = "RtAction",
};

#ifndef CONFIG_RTX_DOMAIN_HRT
static int tick_register(void);

static int tick_noop(void)
{
	return 0;
}

static struct  aud_timer default_aud_timer = {
	.name = "LEGACY",
	.ops = {
		.rt_register 	= tick_register,
		.rt_start		= tick_noop,
		.rt_unregister	= tick_noop,
	},
};

static int tick_register(void)
{
	// Set up for a tick based timing
	default_aud_timer.irq = __rtx_tick_irq;
	default_aud_timer.res_nsec = clock_getres_int(CLOCK_REALTIME);
	return 0;
}
#else
static int hres_timer_register(void);
static int hres_timer_start(void);
static int hres_timer_unregister(void);
static struct aud_timer hres_aud_timer = {
	.name = "rtx_tick",
	.ops = {
		.rt_register   = hres_timer_register,
		.rt_start      = hres_timer_start,
        	.rt_unregister = hres_timer_unregister,
	},
};

static int hres_timer_register(void)
{
	struct tick_device *tdev = &__raw_get_cpu_var(tick_cpu_device);

	if (RT_INIT_VERBOSE)
		printkGen(NULL,"hres_timer_register: clock event device name=%s\n", tdev->evtdev->name);

	// Check if the timer is running in oneshot mode.
	if (tdev->evtdev->mode != CLOCK_EVT_MODE_ONESHOT) {
		printkGen(KERN_ALERT, "The %s timer service is not aperiodic!\n", tdev->evtdev->name);
		printkGen(KERN_ALERT, "> Enable High Resolution Timer or Tickless System support in kernel config\n");
		return -1;
	} else {
		//printkGen(NULL, "The %s timer service is aperiodic!\n", tdev->evtdev->name);
		//printkGen(NULL, "The nanosecond to cycles multiplier is %d!\n", tdev->evtdev->mult);
		//printkGen(NULL, "The nanosecond to cycles divisor (power of two) is %d!\n", tdev->evtdev->shift);
	}

	// Set up for a tick based timing
	hres_aud_timer.irq = __rtx_tick_irq;
	hres_aud_timer.mode = 0; //CLOCK_EVT_MODE_ONESHOT;
	hres_aud_timer.res_nsec = RTX_HIGH_RES_NSEC;
	hres_aud_timer.dev_id = (void *)&hres_aud_timer;
	return 0;
}

static int hres_timer_start(void)
{
	// Only to have an initialised value.
	__raw_get_cpu_var(rt_time_last) = RTX_TIMER_GET;
	return 0;
}

static int hres_timer_unregister(void)
{
	return 0;
}
#endif

static void __rtx_ack_irq(unsigned irq, struct irq_desc *desc)
{
	desc->rtx_ack(irq, desc);
}

static void __rtx_end_irq(unsigned irq, struct irq_desc *desc)
{
	desc->rtx_end(irq, desc);
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

/*
 * Called by the host kernel early during the boot procedure.
 */
void rtx_init(void)
{
	int cpu, i;
//	unsigned irq;

#if defined (CONFIG_ARCH_AT91)
	#if defined (CONFIG_ATMEL_TCB_CLKSRC)
		/* Timer Counter Interrupt 
			- arch/arm/mach-at91/include/mach/at91sam9g45.h (tcb_clksrc.c) */
		__rtx_timer_irq = __rtx_tick_irq = AT91SAM9G45_ID_TCB;
		/* clock-frequency != cpu-frequency => rtx_cpu_freq is clocksource frequency */
		//rtx_cpu_freq = rtx_llimd(1000000000LL, 1, rtx_tcb_clk_res);
		/* rtx_tcb_clk_res will be set in tcb_clksrc_init; now it is 1*/
		rtx_cpu_freq = rtx_llimd(1000000000LL, 1, 60);
	#elif defined (CONFIG_ARCH_AT91SAM9G45)
		/* System Controller Interrupt 
			- arch/arm/mach-at91/include/mach/at91sam9g45.h (at91sam926x_time.c) */
	__rtx_timer_irq = __rtx_tick_irq = AT91_ID_SYS;
		// clock-frequency != cpu-frequency
		//rtx_cpu_freq = rtx_llimd(1000000000, 1, rtx_pit_clk_res);
		rtx_cpu_freq = rtx_llimd(1000000000, 1, 120);
		/* The pit is no high resolution timer (clocksource rating = 175) ! */
	#else
	#endif
#elif defined (CONFIG_ARCH_FEROCEON)
	/* TIME_IRQ - arch/arm/mach-feroceon-kw/include/mach/irqs.h */
	__rtx_timer_irq = __rtx_tick_irq = 1; // TIME_IRQ - arch/arm/mach-feroceon-kw/include/mach/irqs.h
	cpu_khz = mvBoardTclkGet() / 1000;
	rtx_cpu_freq = 1000LL * cpu_khz;
#elif defined (CONFIG_ARCH_MX6Q)
	/* MXC_INT_GPT - arch/arm/plat-mxc/include/mach/mx6.h */
	__rtx_timer_irq = __rtx_tick_irq = MXC_INT_GPT;
	cpu_khz = clk_get_timer_rate() / 1000;
	rtx_cpu_freq = 1000LL * cpu_khz;
#else
	__rtx_timer_irq = __rtx_tick_irq = 0;					// default is PIT
	rtx_cpu_freq = 1000LL * cpu_khz;
#endif

#ifndef CONFIG_RTX_DOMAIN_HRT
	aud_rt_timer = &default_aud_timer;
#else
	aud_rt_timer = &hres_aud_timer;
#endif

	/* for time-conversion we need clocksource frequency */
	/* sometimes this is the cpu frequency !!! */
	rtx_init_timeconv(rtx_cpu_freq);		// for scaled math

	for (cpu = 0; cpu < NR_CPUS; cpu++) {
		per_cpu(rtx_mode, cpu) = LX_DOMAIN_NATIVE;
		rtx_percpu_var(rtx_curr_dom, cpu) = LX_DOMAIN_NATIVE;
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
	struct pt_regs *regs = &__raw_get_cpu_var(rtx_tick_regs);
	int irq;	

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
			pcd->rtx_irqs[irq].handler(irq, pcd->rtx_irqs[irq].cookie);
			if (irq < NR_IRQS) {
				__rtx_end_irq(irq, irq_to_desc(irq));
			}

			local_irq_disable_hw();
			barrier();
#ifdef CONFIG_SMP
			/* We are still on the same CPU? */
			pcd = &__raw_get_cpu_var(rtx_percpu_data);
#endif
			trace_hardirqs_on();
			__clear_bit(LX_IRQS_STALLED, &pcd->rtx_irq_control);

			// NOTE: HW interrupts must be off
#ifndef CONFIG_RTX_DOMAIN_HRT
			if (IS_RTX_TIMER_IRQ(irq))
				log_interrupt_as_pending(irq);			// execute LX-ISR
#endif
		}
	}

	// Handle pending VIRQs, APIC-interrupts and LX-ISRs.
	while ((mask = (pcd->rtx_irq_pending_words & sync_mask)) != 0) {
		part = __ffs(mask);

		while ((pending_part = pcd->rtx_irq_pending[part])) {
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
			//if (rtx_get_cpu_var(rtx_mode) == RT_DOMAIN)
				local_irq_enable_hw();

			if (unlikely(irq >= RTX_VIRQ_BASE)) {
				__rtx_call_lx_virq_handler(irq, pcd->rtx_irqs[irq].handler, NULL);
			}
			else {
				__rtx_move_lx_irq(irq);
				asm_do_IRQ(irq, regs);
			}

			local_irq_disable_hw();
			barrier();
#ifdef CONFIG_SMP
			/* We are still on the same CPU? */
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

/*
 * This is the main interrupt handler called by
 * RT/LX device interrupts and LAPIC-interrupts.
 * Note: This entry is not used for VIRQs.
 * HW-Interrupts must be off.
 */
int __rtx_handle_irq(unsigned int irq, struct pt_regs *regs)
{
	struct rtx_pcd *pcd = &__raw_get_cpu_var(rtx_percpu_data);
	struct irq_desc *desc = irq_to_desc(irq);
	unsigned long domain;

	/* Ack the interrupt. */
	__rtx_ack_irq(irq, desc);
#ifdef CONFIG_ARCH_AT91
	/* AT91 specific workaround */
	irq_finish(irq);
#endif
	if (IS_RTX_TIMER_IRQ(irq)) {
            RTX_SYS_CLEAR_INTR(); // clear the timer-interrupt for RT and LX
        }

	if (likely(pcd->rtx_irqs[irq].handler))
	{
		trace_irq_entry_rt(-irq, regs, &rtDummyAction);
#ifdef CONFIG_RTX_DOMAIN_STOP_ON_PANIC
		// be acknowledged. Do not enter the RT domain again.
		if (rtx_oops) {
			return 0;
		}
#endif

		if (unlikely((rtx_get_cpu_var(rtx_mode) == LX_DOMAIN_SOFT) && test_bit(LX_IRQS_STALLED, &pcd->rtx_irq_control))) {
			log_softirq_as_pending(irq);		// defer the SOFT RT-Isr
		}
		else {
			// This IRQ is using a handle_simple_irq flow.
			domain = pcd->rtx_curr_dom;
			pcd->rtx_curr_dom = RT_DOMAIN;

			pcd->rtx_irqs[irq].handler(irq, pcd->rtx_irqs[irq].cookie);
			local_irq_disable_hw();


			/*
		 	* Note: We increment the interrupt count after calling
		 	* the RT handler (to optimize the latency path).
		 	*/
			if (irq != __rtx_tick_irq)
				rtx_kstat_incr_irqs_this_cpu(irq, irq_desc + irq);

#if defined (CONFIG_ARCH_AT91SAM9G45) && !defined (CONFIG_ATMEL_TCB_CLKSRC)
			/* If we use the pit as timer, we have to handle all shared interrupts! */
			if (IS_RTX_TIMER_IRQ(irq)) RTX_HANDLE_SHARED_INTERRUPT()
#endif
			// Do not call this function for LAPIC-IRQs.
			if (irq < NR_IRQS)
				__rtx_end_irq(irq, irq_to_desc(irq));


			// RT interrupts are enabled while threads are executing.
			rtx_preemption_handling();
			pcd->rtx_curr_dom = domain;				// preceding domain

#ifndef CONFIG_RTX_DOMAIN_HRT
			if (IS_RTX_TIMER_IRQ(irq)) {
				log_interrupt_as_pending(irq);	// execute LX-ISR
			}
#endif
		}
	}
	else {
		// Interrupt logging (LX-interrupts)
		log_interrupt_as_pending(irq);
	}

	if (irq == __rtx_tick_irq) {
		struct pt_regs *tick_regs = &__raw_get_cpu_var(rtx_tick_regs);
		tick_regs->ARM_cpsr = regs->ARM_cpsr;
		tick_regs->ARM_pc = regs->ARM_pc;
		if (pcd->rtx_curr_dom == RT_DOMAIN) {
			tick_regs->ARM_cpsr |= PSR_I_BIT;
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
			}
#ifdef CONFIG_RTX_EXTENDED_RT_DEBUGGING_SUPPORT
			if (current->rt_state3 & RT_TASK_RT_DEBUG_PENDING) {
                            //printkDirect("rtx_irq handle debug\r\n");
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
	//list_add_tail(&req->list, &aud_rt_hdr.wake_lx_list);
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
	struct rtx_pcd *pcd = &__raw_get_cpu_var(rtx_percpu_data);
	irqreturn_t (*act_handler)(int, void *) = nonrt_handler ? nonrt_handler : rt_handler;

	if (RT_INIT_VERBOSE)
		printkGen(NULL, "%s: irq=%d\n", __func__, irq);

	if (!rt_handler) {
		printkGen(KERN_ALERT, "%s: missing rt_handler for irq=%d\n", __func__, irq);
		return -EINVAL;
	}

	if (irq >= NR_IRQS)
	{
		printkGen(KERN_ALERT, "%s: invalid irq=%d\n", __func__, irq);
		return -EINVAL;
	}

	if (irqflags & IRQF_DISABLED) {
		printkGen(KERN_ALERT, "%s: IRQF_DISABLED flag not allowed\n", __func__);
		return -EINVAL;
	}

	irqflags |= IRQF_NOBALANCING;

	// Install the nonrt handler in the LX domain.
	if (!IS_RTX_TIMER_IRQ(irq))
	{
		if (request_irq(irq, act_handler, irqflags, devname, dev_id))
		{
			if (RT_FAULT_VERBOSE)
				printkGen(KERN_ALERT, "%s: could not install nonrt_handler as LX-ISR for irq=%d\n", __func__, irq );
			return -EFAULT;
		}
		if (RT_INIT_VERBOSE)
			printkGen(NULL, "registered nonrt_handler for irq=%d in LX domain\n", irq);
	}

	desc = irq_to_desc(irq);

	// Set the IRQ mode for /proc/interrupts.
	if (desc && desc->action) {
		if (rtx_get_cpu_var(rtx_mode))
			desc->action->mode = rtx_get_cpu_var(rtx_mode) == RT_DOMAIN ? IRQ_MODE_HARD : IRQ_MODE_SOFT;
	}

	// Install the rt_handler in the RT domain.
	local_irq_disable_hw();
	pcd->rtx_irqs[irq].handler = rt_handler;
	pcd->rtx_irqs[irq].cookie = dev_id;

	{
		// Activate the simple flow for the LX-Isr.
		desc->handle_lx = desc->handle_irq;
		desc->handle_irq = handle_simple_irq;
	}

#ifdef CONFIG_RTX_DOMAIN_HRT
	{
		int res;
		// From now we have to emulate the LX ticks in the RT domain.

#if defined (CONFIG_ARCH_AT91) && defined (CONFIG_ATMEL_TCB_CLKSRC)
		if ((res = rtx_request_tickdev("tc_clkevt", rtx_switch_htick_mode, rtx_next_htick_shot, &rtx_timer_freq)) != CLOCK_EVT_MODE_ONESHOT) {
			printkGen(NULL, "tickdev: tc_clkevt is not running in oneshot mode (res=%d)\n", res);
			return -EFAULT;
		}
#elif defined (CONFIG_ARCH_FEROCEON)
		if ((res = rtx_request_tickdev("kw_tick", rtx_switch_htick_mode, rtx_next_htick_shot, &rtx_timer_freq)) != CLOCK_EVT_MODE_ONESHOT) {
			printkGen(NULL, "tickdev: kw_tick is not running in oneshot mode (res=%d)\n", res);
			return -EFAULT;
		}
#elif defined (CONFIG_ARCH_MX6Q)
		if ((res = rtx_request_tickdev("mxc_timer1", rtx_switch_htick_mode, rtx_next_htick_shot, &rtx_timer_freq)) != CLOCK_EVT_MODE_ONESHOT) {
			printkGen(NULL, "tickdev: mxc_timer1 is not running in oneshot mode (res=%d)\n", res);
			return -EFAULT;
		}
#else
		return -EFAULT;
#endif

		// Fake a timer interrupt.
#if defined (CONFIG_ATMEL_TCB_CLKSRC)
		rtx_at91_fake_tcb_irq();
#endif
		log_interrupt_as_pending(irq);

		INIT_LIST_HEAD(&__raw_get_cpu_var(lx_itmr.it_link));
	}
#endif

	local_irq_enable_hw();

	return 0;
}


void rt_free_irq(int irq, void *devid)
{
	irq_desc_t *desc;
	struct rtx_pcd *pcd = &__raw_get_cpu_var(rtx_percpu_data);

	if (RT_INIT_VERBOSE)
		printkGen(NULL, "%s: irq=%d\n", __func__, irq);

	if (irq >= NR_IRQS)
	{
		printkGen(KERN_ALERT, "%s: invalid irq=%d\n", __func__, irq);
		return;
	}

	desc = irq_to_desc(irq);

	// Uninstall the rt_handler from the RT domain.
	local_irq_disable_hw();
	pcd->rtx_irqs[irq].handler = NULL;
	pcd->rtx_irqs[irq].cookie = NULL;
	// Deactivate the simple flow for the LX-Isr.

	desc->handle_irq = desc->handle_lx;

#ifdef CONFIG_RTX_DOMAIN_HRT
	// From now the LX ticks are handled native again.
	rtx_release_tickdev();
	rt_release_emulated_lx_tick();

	// Fake a lapic timer interrupt.
#if defined (CONFIG_ATMEL_TCB_CLKSRC)
	rtx_at91_fake_tcb_irq();
#endif
	log_interrupt_as_pending(irq);
#endif

	local_irq_enable_hw();

	// Set the IRQ mode for /proc/interrupts.
	if (desc && desc->action)
	    desc->action->mode = 0;  // LX-only
	
	if (!IS_RTX_TIMER_IRQ(irq))
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
	return 0;
}


static inline void __local_irq_restore_nosync(unsigned long x)
{
	struct rtx_pcd *pcd;
	unsigned long flags;

	local_irq_save_hw(flags);
	pcd = &__raw_get_cpu_var(rtx_percpu_data);
	if (raw_irqs_disabled_flags(x))
		set_bit(LX_IRQS_STALLED, &pcd->rtx_irq_control);
	else
		clear_bit(LX_IRQS_STALLED, &pcd->rtx_irq_control);
	local_irq_restore_hw(flags);
}

/*
 * Fixup interrupt flag.
 * Have the saved hw state look like the domain stall bit, so
 * that __rtx_unstall_iret_root() restores the proper state for
 * the LX-domain upon exit.
 */
static inline void __fixup_if(int s, struct pt_regs *regs)
{
	if (s)
		regs->ARM_cpsr |= PSR_I_BIT;
	else
		regs->ARM_cpsr &= ~PSR_I_BIT;
}

#define TRAP_DEV_NOT_AVAILABLE	7

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
		case RTX_TRAP_BREAK:
			if (RT_DEBUG_VERBOSE)
				printkGen(NULL, "Debug exception (trap %d) in RT domain at epc=%#lx\n",
					vector, regs->ARM_pc);
#ifdef CONFIG_RTX_EXTENDED_RT_DEBUGGING_SUPPORT
			rtx_stopAllRtThreads(current);
#endif
                        current->rt_state3 |= RT_TASK_LX_DEBUG_PENDING;
                        rtx_migrate_to_lx(RTLX_TASK, NULL);
			return 0;

	    	default:
			if (RT_EXCEPTION_VERBOSE) {
				static int trap_access_printed = 0;
				console_loglevel = 15;
				if((vector != RTX_TRAP_ACCESS) || !trap_access_printed) {
					if (!trap_access_printed) trap_access_printed = 1;
					printkGen(KERN_ALERT,
						"fault %d in RT domain at pc=%#lx\n",
						vector, regs->ARM_pc);
				}
			}
			current->rt_state3 |= RT_TASK_FAULT_DETECTED;
	    		rtx_migrate_to_lx(RTLX_TASK, NULL);
	    		return 0;					// execute exception in the Linux domain
	    	}
	}
	else if (RTX_SYS_IS_RT_DOMAIN) {
#ifdef CONFIG_AUD_LOW_LEVEL_KERNEL_DUMP
                local_irq_disable_hw();
                printkDirect ("__rtx_trap_handling  current ");
                printkDirectHex (current);
                printkDirect ("  pid ");
                printkDirectHex (current->pid);
                printkDirect (" ");
                printkDirect(current->comm);
                printkDirect ("\n\r");
                show_trace_direct(regs);
#endif
			panic("__rtx_trap_handling: RT domain without thread context\n");
        }
	return 0;
}

#ifdef CONFIG_PREEMPT
asmlinkage void preempt_schedule_irq(void);

void __rtx_preempt_schedule_irq(void)
{
	struct rtx_pcd *pcd;
	unsigned long flags;

	/*
	* We have no IRQ state fixup on entry to exceptions, so we have to
	* stall the root stage before rescheduling.
	*/
	BUG_ON(!irqs_disabled_hw());
	local_irq_save(flags);
	local_irq_enable_hw();
	preempt_schedule_irq(); /* OK, may reschedule now. */
	local_irq_disable_hw();

	/*
	* Flush any pending interrupt that may have been logged after
	* preempt_schedule_irq() stalled the root stage before
	* returning to us, and now.
	*/
	pcd = &__raw_get_cpu_var(rtx_percpu_data);
	if (unlikely(rtx_pending_irqs(pcd))) {
		add_preempt_count(PREEMPT_ACTIVE);
		trace_hardirqs_on();
		__clear_bit(LX_IRQS_STALLED, &pcd->rtx_irq_control);
		__rtx_handle_nonrt_pending(RTX_IRQMASK_ANY);
		sub_preempt_count(PREEMPT_ACTIVE);
	}
	__local_irq_restore_nosync(flags);
}
#endif /* CONFIG_PREEMPT */

int __rtx_divert_exception(struct pt_regs *regs, int vector)
{
	unsigned long flags;

	/* Pick up the LX domain state of the interrupted context. */
	local_save_flags(flags);

    if (!RTX_SYS_IS_RT_DOMAIN) {
		/*
		 * Replicate HW interrupt state into the virtual mask before
		 * calling the I-pipe event handler over the root domain. Also
		 * required later when calling the Linux exception handler.
		 */
    	if (irqs_disabled_hw())
    		local_irq_disable();
    }
    if (unlikely(__rtx_trap_handling(vector,regs))) {
    	__local_irq_restore_nosync(flags);
    	return 1;
    }
	/*
	 * 32-bit: Due to possible migration inside the event handler, we have
	 * to restore IF so that low-level return code sets the LX-domain
	 * state correctly.
	 */
    __fixup_if(raw_irqs_disabled_flags(flags), regs);
    return 0;
}
	// RT interrupts enabled while threads are executing.
/*
 * Implementation of the virtualized interrupt macros ( see arch/arm/include/asm/irqflags.h).
 * Note: Only __rtx_local_irq_enable() has to be provided as a C-function because it is
 * called in entry-armv.S.
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

#if defined(CONFIG_RTX_DOMAIN_HRT) && defined(CONFIG_GENERIC_CLOCKEVENTS)
/*
 * Callback for an emulated Linux tick.
 * This routine is called in ISR context.
 * Note: HW interrupts must be off.
 */
int rtx_set_lx_tick_pending(void *arg)
{
	log_interrupt_as_pending(__rtx_timer_irq);
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
* Rescheduling: never.
*/
static int rtx_next_htick_shot(unsigned long delay, struct clock_event_device *cdev)
{
	rt_emulate_lx_tick(delay, rtx_set_lx_tick_pending);
	return 0;
}

/*
 * This callback announces a switch of the LX timer mode
 * (which is not expected).
 */
static void rtx_switch_htick_mode(enum clock_event_mode mode, struct clock_event_device *cdev)
{
	// We always expect oneshot mode.
	if (mode != CLOCK_EVT_MODE_ONESHOT) {
		panic("%s: Unexpected timer mode switch for the RT domain (mode=%d)\n", __func__, mode);
		return;
	}
}

#if defined (CONFIG_ARCH_MX6Q)
//static RTX_TIMER_TYPE maxNsValue = ((((unsigned long long) 0xffffffff)
//                                    * ARCH_TIMER_MULT) >> ARCH_TIMER_SHIFT);

RTX_TIMER_TYPE hres_getCntValue(struct clocksource *clks)
{
	RTX_TIMER_TYPE nsValue;
	unsigned long actCycVal;
//	static unsigned oldCycVal = 0;
//	static RTX_TIMER_TYPE overflowNsValue=0;

	if(clks != NULL) { 
		actCycVal = clks->read(clks);
		nsValue = (RTX_TIMER_TYPE)actCycVal;
		nsValue = (nsValue * ARCH_TIMER_MULT) >> ARCH_TIMER_SHIFT;

// bh3672: still faulty !!!
//   		if (actCycVal < oldCycVal) {
//			overflowNsValue += maxNsValue;
//			printkGen(NULL, "%s: nsValue=%#x + overflowNsValue=%#x = %#x\n", __func__,
//                  			nsValue, overflowNsValue, nsValue += overflowNsValue);
//			nsValue += overflowNsValue;
//		}
//
//		oldCycVal = actCycVal;
		return(nsValue);
	} else return (0);
}

int hres_setRefValue(unsigned nsDelay, struct clock_event_device *dev)
{
	unsigned long long calcCycVal;
	unsigned long clc;
	unsigned changeValue = 0; // 3;
	// signal correct timer setup
	int retVal = 1;
	unsigned long flags;
	int showInfo = 0;

	if(dev != NULL) { 
		if (nsDelay > dev->max_delta_ns)
			nsDelay = dev->max_delta_ns;
		if (nsDelay < dev->min_delta_ns)
			nsDelay = dev->min_delta_ns;

		local_irq_save_hw(flags);
restart:
		//calcCycVal = (unsigned long long)RTX_NS2CYC(nsDelay);
		calcCycVal = (RTX_TIMER_TYPE)nsDelay;
		calcCycVal = (calcCycVal * ARCH_TIMER_MULT_REV) >> ARCH_TIMER_SHIFT_REV;
		clc = (unsigned long) calcCycVal;

		// set delay to next event; use a longer delay, if failed
		//if( 0 > itd->real_set_tick(clc, dev) ) {
		if( 0 != ARCH_TIMER_SET(clc) ) {
			retVal = 0;
			showInfo = 1;
			// be sure to have a valid new compare entry
			changeValue++;
			nsDelay <<= changeValue;
			goto restart; 
		}
		local_irq_restore_hw(flags);

		if (showInfo)
			printkGen(NULL, "%s: nsDelay=%#x clc=%#x changeValue=%d (%#x %#x)\n",
                  			__func__,
                  			nsDelay, clc, changeValue,
                  			dev->max_delta_ns, dev->min_delta_ns);	
	}

	return(retVal);
}

void showHighResCounts(void)
{
	printkGen(NULL, "%s_a: cyc=%#x\n", __func__,
                  ARCH_TIMER_GET);
#if (0)
	printkGen(NULL, "%s_b: cyc=%#x\n", __func__,
                  ARCH_TIMER_GET);
#endif
}
#endif /* CONFIG_ARCH_MX6Q */

#endif /* defined(CONFIG_RTX_DOMAIN_HRT) && defined(CONFIG_GENERIC_CLOCKEVENTS) */

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

EXPORT_SYMBOL(rt_request_irq);
EXPORT_SYMBOL(execute_nonrt_handler);
EXPORT_SYMBOL(rt_free_irq);
EXPORT_SYMBOL(rt_enable_irq_affinity);
EXPORT_SYMBOL(rt_disable_irq_affinity);
EXPORT_SYMBOL(__rtx_handle_nonrt_pending);
EXPORT_SYMBOL(__rtx_local_irq_enable);
EXPORT_PER_CPU_SYMBOL(rtx_percpu_data);

