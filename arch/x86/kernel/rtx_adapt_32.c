/*
 * arch/x86/kernel/rtx_adapt_32.c
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
 */
#include <linux/rtx_tickdev.h>
#include <linux/aud/rt_timer.h>
#include <asm/i387.h>
#include <asm/apic.h>
#include <asm/traps.h>
#ifdef CONFIG_RTX_EXTENDED_RT_DEBUGGING_SUPPORT
#include <linux/aud/rt_debug.h>
#endif

#include <asm/nmi.h>

#include <linux/tick.h>

#include <trace/events/irq.h>

static unsigned long long rtx_cpu_freq;
unsigned long long clockfreq;
#ifdef RTX_HAVE_LLMULSHFT
unsigned int tsc_scale, tsc_shift;
#ifdef RTX_HAVE_NODIV_LLIMD
rtx_u32frac_t tsc_frac;
rtx_u32frac_t bln_frac;
#endif
#endif

#ifdef CONFIG_X86_LOCAL_APIC
unsigned long rtx_timer_freq;

#ifdef CONFIG_RTX_DOMAIN_HRT
static int rtx_next_htick_shot(unsigned long delay, struct clock_event_device *cdev);
static void rtx_switch_htick_mode(enum clock_event_mode mode, struct clock_event_device *cdev);
#endif
#endif /* CONFIG_X86_LOCAL_APIC */

#ifdef CONFIG_SMP
static int lx_cpu;
#endif

#ifndef CONFIG_RTX_OPTIMIZE_FOR_RELEASE
static unsigned long rt_fault_addr;
#endif

/* Control structure for requested interrupts. */
struct isr_cntrl rtx_req_irqs[RTX_NR_IRQS];
struct isr_cntrl rtx_req_clock;

DEFINE_PER_CPU(struct pt_regs, rtx_tick_regs);

/* IRQ mode for /proc/interrupts */
#define IRQ_MODE_HARD '*'
#define IRQ_MODE_SOFT '+'

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

struct {
	rtx_handler_lx_t handler;
	rtx_ack_lx_t ack;
} ____cacheline_aligned lx_irqs[RTX_NR_IRQS];

int to_rt_virq = TO_RT_VIRQ;				// irq for waking up the realtime domain;
int to_lx_virq = TO_LX_VIRQ;				// irq for notifying the linux daemon for futex and execution in Linux
int to_mg_virq = TO_MG_VIRQ;				// cancel synchronized migration
int wake_up_lx_virq = WAKE_UP_LX_VIRQ;		// irq for functions which require only a wake_up_process and can therefore
											// executed directly from ISR level
int printk_virq = PRINTK_VIRQ;

int __rtx_tick_irq;							// the Linux tick device and the RT timer device are always the same.

#ifdef CONFIG_SMP
static cpumask_t __rtx_cpu_sync_map;
static cpumask_t __rtx_cpu_lock_map;
static RTX_DEFINE_SPINLOCK(__rtx_cpu_barrier);
static atomic_t __rtx_critical_count = ATOMIC_INIT(0);
#ifdef CONFIG_RTX_DOMAIN_EXCLUSIVE_MODE
static atomic_t __rtx_exclusive_count = ATOMIC_INIT(0);
#endif
#endif

// dummy irq action information for tracepoint handling
struct irqaction rtDummyAction =  {
	.name = "RtAction",
};

#ifndef CONFIG_RTX_DOMAIN_HRT
static int default_register(void);

static int default_noop(void)
{
	return 0;
}

static struct  aud_timer default_aud_timer = {
	.name = "LEGACY",
	.ops = {
		.rt_register 	= default_register,
		.rt_start		= default_noop,
		.rt_unregister	= default_noop,
	},
};

static int default_register(void)
{
	struct tick_device *tdev = &__raw_get_cpu_var(tick_cpu_device);

	if (RT_INIT_VERBOSE)
		printkGen(NULL,"%s: clock event device name=%s\n", __func__, tdev->evtdev->name);

	// Check if the timer is running in periodic mode.
	if (tdev->evtdev->mode != CLOCK_EVT_MODE_PERIODIC) {
		printkGen(NULL, "The %s timer service is not periodic!\n", tdev->evtdev->name);
		printkGen(NULL, "> Disable High Resolution Timer and Tickless System support in kernel config\n");
		return -1;
	}

	// Set up for a periodic tick based timing.
	default_aud_timer.irq = __rtx_tick_irq;
	default_aud_timer.mode = 0;
	default_aud_timer.res_nsec = clock_getres_int(CLOCK_REALTIME);
	default_aud_timer.dev_id = (void *)&default_aud_timer;
	return 0;
}
#else
#ifdef CONFIG_X86_LOCAL_APIC
static int lapic_timer_register(void);
static int lapic_timer_start(void);
static int lapic_timer_unregister(void);
static struct aud_timer lapic_aud_timer = {
	.name = "lapic",
	.ops = {
		.rt_register   = lapic_timer_register,
		.rt_start      = lapic_timer_start,
        .rt_unregister = lapic_timer_unregister,
	},
};
#endif
#endif

static void __rtx_ack_irq(unsigned irq, struct irq_desc *desc)
{
	desc->rtx_ack(irq, desc);
}

static void __rtx_end_irq(unsigned irq, struct irq_desc *desc)
{
	desc->rtx_end(irq, desc);
}

#ifdef CONFIG_SMP
static cpumask_t lx_irq_affinity[NR_IRQS];
unsigned rtx_irq_pinning;
#endif

#ifdef CONFIG_X86_LOCAL_APIC
void smp_apic_timer_interrupt(struct pt_regs *regs);
void smp_spurious_interrupt(struct pt_regs *regs);
void smp_error_interrupt(struct pt_regs *regs);
#ifdef CONFIG_X86_THERMAL_VECTOR
void smp_thermal_interrupt(void);
#endif
#ifdef CONFIG_X86_MCE_THRESHOLD
asmlinkage void smp_threshold_interrupt(void);
#endif
#ifdef CONFIG_X86_NEW_MCE
asmlinkage void smp_mce_self_interrupt(void);
#endif
#ifdef CONFIG_X86_UV
asmlinkage void uv_bau_message_interrupt(struct pt_regs *regs);
#endif
#endif

#ifdef CONFIG_SMP
void smp_reschedule_interrupt(struct pt_regs *regs);
void smp_invalidate_interrupt(struct pt_regs *regs);
void smp_call_function_interrupt(struct pt_regs *regs);
void smp_call_function_single_interrupt(struct pt_regs *regs);
extern asmlinkage void smp_reboot_interrupt(void);
#endif

#ifdef CONFIG_X86_LOCAL_APIC
/*
 * Acknowledge the APIC interrupt.
 */
static void __rtx_ack_apic(unsigned irq, struct irq_desc *desc)
{
	__ack_APIC_irq();
}

/*
 * No acknowledge function.
 */
static void __rtx_noack_apic(unsigned irq, struct irq_desc *desc)
{
}
#endif

static void rtx_apic_handler_init(unsigned int irq, rtx_handler_lx_t handler, rtx_ack_lx_t ack)
{
	lx_irqs[irq].handler = handler;
	lx_irqs[irq].ack = ack;
}

/*
 * HW interrupts must be off.
 */
#ifdef CONFIG_SMP
void rtx_handle_critical_sync(unsigned irq, void *cookie)
{
	int cpu = rtx_processor_id();

	cpu_set(cpu, __rtx_cpu_sync_map);

	/* Now we have synchronized with lock requestor. */
	rtx_spin_lock(&__rtx_cpu_barrier);

	/* Got it, nothing to do, leave it. */

	rtx_spin_unlock(&__rtx_cpu_barrier);

	cpu_clear(cpu, __rtx_cpu_sync_map);
}

#ifdef CONFIG_RTX_DOMAIN_EXCLUSIVE_MODE
/*
 * This function is called by an IPI, which causes the
 * receiver CPU to execute an idle loop, as long as
 * the exclusive count is set.
 */
static inline void rtx_handle_exclusive_mode(unsigned irq, void *cookie)
{
	while (atomic_read(&rt_exclusive_count)) {
		cpu_relax();
	}
}

/* Enter exclusive mode. */
static inline void rtx_enter_exclusive_mode(void)
{
	atomic_inc(&rt_exclusive_count);
	apic->send_IPI_allbutself(RTX_EXCLUSIVE_VECTOR);
}

/* Leave exclusive mode. */
static inline void rtx_leave_exclusive_mode(void)
{
	atomic_dec(&rt_exclusive_count);
}
#endif
#endif

/*
 * Critical lock:
 * Excluding all CPUs but the current one from a critical section.
 */
unsigned long rtx_critical_enter(void)
{
	unsigned long flags;

	local_irq_save_hw(flags);

#ifdef CONFIG_SMP
	if (unlikely(num_online_cpus() == 1))
		return flags;

	{
		int cpu = rtx_processor_id();
		cpumask_t lock_map;

		if (!cpu_test_and_set(cpu, __rtx_cpu_lock_map)) {
			while (cpu_test_and_set(BITS_PER_LONG - 1, __rtx_cpu_lock_map)) {
				int n = 0;
				do {
					cpu_relax();
				} while (++n < cpu);
			}

			rtx_spin_lock(&__rtx_cpu_barrier);

			/* Send the sync IPI to all processors but the current one. */
			apic->send_IPI_allbutself(RTX_CRITICAL_VECTOR);

			cpus_andnot(lock_map, cpu_online_map, __rtx_cpu_lock_map);

			while (!cpus_equal(__rtx_cpu_sync_map, lock_map))
				cpu_relax();
		}

		atomic_inc(&__rtx_critical_count);
	}
#endif	/* CONFIG_SMP */

	return flags;
}

/*
 * Release the critical lock.
 */
void rtx_critical_exit(unsigned long flags)
{
#ifdef CONFIG_SMP
	if (num_online_cpus() == 1) {
		local_irq_restore_hw(flags);
		return;
	}

	if (atomic_dec_and_test(&__rtx_critical_count)) {
		rtx_spin_unlock(&__rtx_cpu_barrier);

		while (!cpus_empty(__rtx_cpu_sync_map))
			cpu_relax();

		cpu_clear(rtx_processor_id(), __rtx_cpu_lock_map);
		cpu_clear(BITS_PER_LONG - 1, __rtx_cpu_lock_map);
	}
#endif	/* CONFIG_SMP */

	local_irq_restore_hw(flags);
}

/*
 * Called by the host kernel early during the boot procedure.
 */
void rtx_init(void)
{
	int cpu, i;
	unsigned irq;

#ifndef CONFIG_RTX_DOMAIN_HRT
	// Setup of __rtx_tick_irq is handled in tick_setup_device()/rtx_update_tick_evtdev().
	aud_rt_timer = &default_aud_timer;
#else
#ifdef CONFIG_X86_LOCAL_APIC
	aud_rt_timer = &lapic_aud_timer;
#endif /* CONFIG_X86_LOCAL_APIC */
#endif /* CONFIG_RTX_DOMAIN_HRT */

	rtx_cpu_freq = 1000LL * cpu_khz;
	rtx_init_timeconv(rtx_cpu_freq);	// for scaled math

	// Virtualize ISA and IOAPIC interrupts.
	// Note: IRQ 0x20 is reprogrammed by IRQ_MOVE_CLEANUP_VECTOR.
	for (irq = 0; irq < NR_IRQS; irq++) {
		rtx_apic_handler_init(irq, (rtx_handler_lx_t)&do_IRQ,(rtx_ack_lx_t)&__rtx_ack_irq);
	}

#ifdef CONFIG_X86_LOCAL_APIC
	/* Map the APIC system vectors. */
	rtx_apic_handler_init(rtx_apic_vector_irq(LOCAL_TIMER_VECTOR), (rtx_handler_lx_t)&smp_apic_timer_interrupt,(rtx_ack_lx_t)&__rtx_ack_apic);
	rtx_apic_handler_init(rtx_apic_vector_irq(SPURIOUS_APIC_VECTOR), (rtx_handler_lx_t)&smp_spurious_interrupt,(rtx_ack_lx_t)&__rtx_noack_apic);
	rtx_apic_handler_init(rtx_apic_vector_irq(ERROR_APIC_VECTOR), (rtx_handler_lx_t)&smp_error_interrupt,(rtx_ack_lx_t)&__rtx_ack_apic);
#ifdef CONFIG_X86_THERMAL_VECTOR
	rtx_apic_handler_init(rtx_apic_vector_irq(THERMAL_APIC_VECTOR), (rtx_handler_lx_t)&smp_thermal_interrupt,(rtx_ack_lx_t)&__rtx_ack_apic);
#endif
#ifdef CONFIG_X86_MCE_THRESHOLD
	rtx_apic_handler_init(rtx_apic_vector_irq(THRESHOLD_APIC_VECTOR), (rtx_handler_lx_t)&smp_threshold_interrupt,(rtx_ack_lx_t)&__rtx_ack_apic);
#endif
#ifdef CONFIG_X86_NEW_MCE
	rtx_apic_handler_init(rtx_apic_vector_irq(MCE_SELF_VECTOR), (rtx_handler_lx_t)&smp_mce_self_interrupt,(rtx_ack_lx_t)&__rtx_ack_apic);
#endif
#ifdef CONFIG_X86_UV
	rtx_apic_handler_init(rtx_apic_vector_irq(UV_BAU_MESSAGE), (rtx_handler_lx_t)&uv_bau_message_interrupt,(rtx_ack_lx_t)&__rtx_ack_apic);
#endif
	rtx_apic_handler_init(rtx_apic_vector_irq(GENERIC_INTERRUPT_VECTOR), (rtx_handler_lx_t)&smp_generic_interrupt,(rtx_ack_lx_t)&__rtx_ack_apic);
#ifdef CONFIG_PERF_COUNTERS
	rtx_apic_handler_init(rtx_apic_vector_irq(LOCAL_PENDING_VECTOR), (rtx_handler_lx_t)&perf_pending_interrupt,(rtx_ack_lx_t)&__rtx_ack_apic);
#endif
#endif /* CONFIG_X86_LOCAL_APIC */

#ifdef CONFIG_SMP
	rtx_apic_handler_init(rtx_apic_vector_irq(RESCHEDULE_VECTOR), (rtx_handler_lx_t)&smp_reschedule_interrupt,(rtx_ack_lx_t)&__rtx_ack_apic);

	for (i = INVALIDATE_TLB_VECTOR_START;
			i <= INVALIDATE_TLB_VECTOR_END; ++i) {
		rtx_apic_handler_init(rtx_apic_vector_irq(i), (rtx_handler_lx_t)&smp_invalidate_interrupt,(rtx_ack_lx_t)&__rtx_ack_apic);
	}

	rtx_apic_handler_init(rtx_apic_vector_irq(CALL_FUNCTION_VECTOR), (rtx_handler_lx_t)&smp_call_function_interrupt,(rtx_ack_lx_t)&__rtx_ack_apic);
	rtx_apic_handler_init(rtx_apic_vector_irq(CALL_FUNCTION_SINGLE_VECTOR), (rtx_handler_lx_t)&smp_call_function_single_interrupt,(rtx_ack_lx_t)&__rtx_ack_apic);
	rtx_apic_handler_init(IRQ_MOVE_CLEANUP_VECTOR, (rtx_handler_lx_t)&smp_irq_move_cleanup_interrupt,(rtx_ack_lx_t)&__rtx_ack_apic);
	rtx_apic_handler_init(rtx_apic_vector_irq(REBOOT_VECTOR), (rtx_handler_lx_t)&smp_reboot_interrupt,(rtx_ack_lx_t)&__rtx_ack_apic);
	rtx_apic_handler_init(rtx_apic_vector_irq(RTX_CRITICAL_VECTOR), (rtx_handler_lx_t)&rtx_handle_critical_sync,(rtx_ack_lx_t)&__rtx_ack_apic);
#ifdef CONFIG_RTX_DOMAIN_EXCLUSIVE_MODE
	/* In this case we don't need an ISR-handler for LX,
	 * but we need an acknowledge function. */
	rtx_apic_handler_init(rtx_apic_vector_irq(RTX_EXCLUSIVE_VECTOR), NULL,(rtx_ack_lx_t)&__rtx_ack_apic);
#endif
#endif /* CONFIG_SMP */

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
#ifdef CONFIG_PRINTK
		rtx_percpu_var(rtx_irqs, cpu)[PRINTK_VIRQ].handler = __rtx_flush_printk;
		rtx_percpu_var(rtx_irqs, cpu)[PRINTK_VIRQ].cookie = NULL;
#endif

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
			trace_hardirqs_off();

			// The SOFT RT-ISR may set the LX-ISR pending.
			pcd->rtx_irqs[irq].handler(irq, pcd->rtx_irqs[irq].cookie);
			if (irq < NR_IRQS)
				__rtx_end_irq(irq, irq_to_desc(irq));
			/* Ensure that the interrupt handler doesn't make harm. */
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

		while ((pending_part = pcd->rtx_irq_pending[part]) != 0) {
			position = __ffs(pending_part);
			irq = (part << RTX_IRQ_WORDS_SHIFT) + position;
			__clear_bit(position, &pcd->rtx_irq_pending[part]);

			if (pcd->rtx_irq_pending[part] == 0)
				__clear_bit(part, &pcd->rtx_irq_pending_words);

			__set_bit(LX_IRQS_STALLED, &pcd->rtx_irq_control);
			barrier();
			trace_hardirqs_off();

			/* Soft-RT: Allow a high priority interrupt to be handled 
			 * before the handling of all the native LX-interrupts is done. */
			local_irq_enable_hw();

			/*
			 * As long as the RT-domain is registered,
			 * interrupts are enabled before calling the handler.
			 * We may have different types of pending interrupts:
			 * - VIRQs (soft-IRQ)
			 * - APIC interrupts (e.g. IPIs)
			 * - IRQs (Linux device interrupts)
			 */
			if (unlikely(irq >= RTX_VIRQ_BASE)) {
				__rtx_call_lx_virq_handler(irq, pcd->rtx_irqs[irq].handler, NULL);
			}
			else {
				__rtx_move_lx_irq(irq);
				__rtx_call_lx_irq_handler(irq, lx_irqs[irq].handler);
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
				goto restart_softirq;
		}
	}
}

/*
 * Calling the RT domain from LX.
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

#ifdef CONFIG_RTX_DOMAIN_EXCLUSIVE_MODE
	if (IS_LINUX)
		rtx_enter_exclusive_mode();
#endif
	pcd->rtx_irqs[irq].handler(irq, NULL);

#ifdef CONFIG_RTX_DOMAIN_INTEGRITY_CHECK
	rtx_check_intr_flag();
#endif

	// RT interrupts are enabled while RT-threads are running.
	// Interrupts are off again when returning.
	rtx_preemption_handling();
#ifdef CONFIG_RTX_DOMAIN_EXCLUSIVE_MODE
	if (IS_LINUX)
		rtx_leave_exclusive_mode();
#endif
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
int __rtx_handle_irq(struct pt_regs *regs)
{
	struct rtx_pcd *pcd = &__raw_get_cpu_var(rtx_percpu_data);
	unsigned long domain;
	unsigned vector = ~regs->orig_ax, irq;

#ifdef CONFIG_X86_LOCAL_APIC
	if (vector == IRQ_MOVE_CLEANUP_VECTOR) {
		irq = IRQ_MOVE_CLEANUP_VECTOR;
	}
	else if (vector >= FIRST_SYSTEM_VECTOR) {
		irq = rtx_apic_vector_irq(vector);
	}
	else
#endif
	{
		irq = __raw_get_cpu_var(vector_irq)[vector];
	}
	lx_irqs[irq].ack(irq, irq_desc + irq);

#ifdef CONFIG_RTX_DOMAIN_EXCLUSIVE_MODE
	if (vector == RTX_EXCLUSIVE_VECTOR)
		rtx_handle_exclusive_mode(irq, NULL);
#endif

	if (likely(pcd->rtx_irqs[irq].handler))
	{
		trace_irq_entry_rt(-irq, regs, &rtDummyAction);
#ifdef CONFIG_RTX_DOMAIN_STOP_ON_PANIC
		// In case of a panic situation RT interrupts will only
		// be acknowledged but not unmasked.
		// We leave RT interrupts on hold.
		if (rtx_oops) {
			return 0;
		}
#endif
		if (unlikely(rtx_get_cpu_var(rtx_mode) == LX_DOMAIN_SOFT)) {
			log_softirq_as_pending(irq);
		}
		else {
#ifdef CONFIG_RTX_DOMAIN_EXCLUSIVE_MODE
			if (IS_LINUX)
				rtx_enter_exclusive_mode();
#endif
			// This IRQ is handled by a handle_simple_irq flow.
			domain = pcd->rtx_curr_dom;
			pcd->rtx_curr_dom = RT_DOMAIN;
			pcd->rtx_irqs[irq].handler(irq, pcd->rtx_irqs[irq].cookie);
			/* Ensure that the interrupt handler doesn't make harm. */
			local_irq_disable_hw();
		
			/*
			* Note: We increment the interrupt count after calling
			* the RT handler (to optimize the latency path).
			*/
			if (irq != __rtx_tick_irq)
				rtx_kstat_incr_irqs_this_cpu(irq, irq_desc + irq);
	
			// Do not call this function for LAPIC-IRQs.
			if (irq < NR_IRQS)
				__rtx_end_irq(irq, irq_to_desc(irq));
		
			// RT interrupts are enabled while RT-threads are running.
			// Interrupts are off again when returning.
			rtx_preemption_handling();
#ifdef CONFIG_RTX_DOMAIN_EXCLUSIVE_MODE
			if (IS_LINUX)
				rtx_leave_exclusive_mode();
#endif
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
#ifdef CONFIG_RTX_DOMAIN_EXCLUSIVE_MODE
		if (vector != RTX_EXCLUSIVE_VECTOR)
#endif
			log_interrupt_as_pending(irq);
	}

	if (irq == __rtx_tick_irq) {
		struct pt_regs *tick_regs = &__raw_get_cpu_var(rtx_tick_regs);
		tick_regs->flags = regs->flags;
		tick_regs->cs = regs->cs;
		tick_regs->ip = regs->ip;
		tick_regs->bp = regs->bp;
		if (pcd->rtx_curr_dom == RT_DOMAIN) {
			tick_regs->flags &= ~X86_EFLAGS_IF;
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

/*
 * Called only once at module init (see init_AuD_dev()).
 */
void rtx_init_isr_management(void)
{
	/* Nothing to do */
}

/*
 * Depending on the requirements a driver may need to split up its work having a realtime part constituted
 * by a "rt_handler" and an additional non-realtime part constituted by a "nonrt_handler".
 * The "rt_handler" always gets installed in the rtx_irqs array, whereas the "nonrt_handler" gets installed
 * as an LX-handler. There is no difference between a system with a RT domain and a LX-only system.
 */
int execute_nonrt_handler(int flags, int irq)
{
	/* Interrupts are (must be) disabled */
	log_interrupt_as_pending(irq);
	return IRQ_NONE;
}

#ifdef CONFIG_SMP
/*
 * Called from LX on request_irq().
 * Set desc->affinity according to rtx_cpu_onlinemask.
 *
 * Note: This code is called in an atomic sequence,
 * so it must not sleep. It is assumed that desc->lock
 * is already locked by a spinlock.
 */
void rtx_setup_affinity(int irq)
{
	cpumask_t lx_mask;
	struct irq_desc *desc = irq_to_desc(irq);

	if (cpus_empty(rtx_cpu_onlinemask) || num_online_cpus() == 1)
		return;

	/*
	* Affinity handling for LX-interrupts.
	* - save the current affinity
	* - compute the new affinity
	* - set the new affinity
	*/
	cpumask_copy(&lx_irq_affinity[irq], desc->affinity);
	if (lx_cpu)
		cpumask_copy(&lx_mask, cpumask_of(lx_cpu));
	else
		cpus_andnot(lx_mask, cpu_online_map, rtx_cpu_onlinemask);

	cpumask_copy(desc->affinity, &lx_mask);
	return;
}

/*
 * Make sure that the interrupt controller provides a corresponding
 * function to set the affinity for this IRQ.
 * Note: It is assumed that this function is already safeguarded
 * by spin_lock_irqsave(&desc->lock, flags).
 */
static int rt_irq_set_affinity(int irq, cpumask_t cpu_mask)
{
	struct irq_desc *desc = irq_to_desc(irq);

	// set_affinity() copies the cpu_mask to desc->affinity.
	desc->chip->set_affinity(irq, &cpu_mask);
	return 0;
}

/*
 * Checking whether the IRQ is currently handled.
 * If so, we have to wait until done.
 */
static inline void _rtx_synchronize_irq(int irq, struct irq_desc *desc, unsigned long flags)
{
	while (desc->status & IRQ_INPROGRESS) {
		rtx_spin_unlock_irqrestore(&desc->lock, flags);
		synchronize_irq(irq);
		rtx_spin_lock_irqsave(&desc->lock, flags);
	}
}

/*
 * This function is called when initializing the RT domain or
 * when a device drivers registers a single IRQ.
 * The affinity of all LX-interrupts (except those that are
 * registered by the RT domain) is changed, excluding the cpus
 * a RT-proces is assigned to.
 * Note: The semaphore dev_sem is assumed to be locked.
 */
int rtx_pin_irq(int irq, cpumask_t cpu_mask)
{
	unsigned long flags;
	cpumask_t rt_irq_lx_mask = cpu_online_map;
	struct irq_desc *desc;

	if (num_online_cpus() == 1)
		return 0;

	/* Detach all LX-IRQs from cpus a RT-process is assigned to. */
	if (irq == -1) {
		rtx_irq_pinning++;

		/* Remove the RT cpus from the mask. */
		cpus_andnot(rt_irq_lx_mask, rt_irq_lx_mask, rtx_cpu_onlinemask);

		/*
		 * In the case a RT-process is assigned to each cpu,
		 * the last cpu has to handle LX-interrupts.
		 */
		if (cpus_empty(rt_irq_lx_mask) || lx_cpu) {
			if (!lx_cpu) {
				lx_cpu = current->rt_proc->cpu;
				if (RT_WARN_VERBOSE)
					printkGen(KERN_ALERT, "WARNING - Cannot exclude LX-interrupts from cpu=%d\n", lx_cpu);
			}
			return 0;
		}

		for (irq=0; irq < NR_IRQS; irq++)
		{
			desc = irq_to_desc(irq);
			rtx_spin_lock_irqsave(&desc->lock, flags);

			/* All the LX-IRQs are detached from RT cpus. */
			if (rtx_percpu_var(rtx_irqs, current->rt_proc->cpu)[irq].handler == NULL) {
				if (cpus_empty(lx_irq_affinity[irq]) && desc->chip->set_affinity) {
					cpumask_copy(&lx_irq_affinity[irq], desc->affinity);
				}

				if (desc->chip->set_affinity) {
					_rtx_synchronize_irq(irq, desc, flags);
					desc->status &= ~IRQ_MOVE_PENDING;			// do not retarget the IRQ
					rt_irq_set_affinity(irq, rt_irq_lx_mask);
				}
			}
			rtx_spin_unlock_irqrestore(&desc->lock, flags);
		}
		return 0;
	}
	else {
		/* Pin down a single RT-IRQ to the cpu
		 * this RT-process is assigned to. */
		desc = irq_to_desc(irq);

		if (!desc->chip->set_affinity)
			return 1;

		rtx_spin_lock_irqsave(&desc->lock, flags);

		/* Maybe it has already been copied by request_irq(). */
		if (cpus_empty(lx_irq_affinity[irq]))
			cpumask_copy(&lx_irq_affinity[irq], desc->affinity);
		_rtx_synchronize_irq(irq, desc, flags);
		desc->status &= ~IRQ_MOVE_PENDING;
		rt_irq_set_affinity(irq, cpu_mask);
		rtx_spin_unlock_irqrestore(&desc->lock, flags);
		return 0;
	}
	return 0;
}

/*
 * This function is called when shutting down the RT domain or
 * when a device drivers unregisters a single IRQ.
 * The affinity of all LX-interrupts is set to their
 * original affinity mask.
 * Note: For a single IRQ the semaphore dev_sem is assumed to be locked.
 */
int rtx_unpin_irq(int irq)
{
	unsigned long flags;
	struct irq_desc *desc;

	if (num_online_cpus() == 1)
		return 0;

	/* If this is the last RT-process which disappears,
	 * then set the affinity of all LX-IRQs back to the
	 * state which was saved previously.
	 */
	if (irq == -1) {
		down(&aud_rt_hdr.dev_sem);

		/*
		 * Shutting down the last RT-process will
		 * re-install the affinity for the LX-interrupts.
		 */
		if (rtx_irq_pinning == 1) {
			for (irq=0; irq < NR_IRQS; irq++)
			{
				desc = irq_to_desc(irq);
				rtx_spin_lock_irqsave(&desc->lock, flags);
				if (rtx_percpu_var(rtx_irqs, current->rt_proc->cpu)[irq].handler == NULL) {
					if (!cpus_empty(lx_irq_affinity[irq])) {
						_rtx_synchronize_irq(irq, desc, flags);
						rt_irq_set_affinity(irq, lx_irq_affinity[irq]);
						cpus_clear(lx_irq_affinity[irq]);
					}
				}
				rtx_spin_unlock_irqrestore(&desc->lock, flags);
			}
			lx_cpu = 0;
		}
		rtx_irq_pinning--;
		up(&aud_rt_hdr.dev_sem);
		return 0;
	}
	else {
		/* Unpin a single IRQ from RT-cpu.
		 * dev_sem is already locked.
		 */
		if (!cpus_empty(lx_irq_affinity[irq])) {
			desc = irq_to_desc(irq);
			rtx_spin_lock_irqsave(&desc->lock, flags);
			_rtx_synchronize_irq(irq, desc, flags);
			rt_irq_set_affinity(irq, lx_irq_affinity[irq]);
			rtx_spin_unlock_irqrestore(&desc->lock, flags);
			cpus_clear(lx_irq_affinity[irq]);
		}
		else
			if (RT_WARN_VERBOSE)
				printkGen(KERN_ALERT, "%s: WARNING - irq=%d is already unpinned\n", __func__, irq);
	}
	return 0;
}
#endif


/*
 * Calling rt_request_irq() indicates, that an ISR for the realtime domain should be installed.
 * If no realtime domain is present, this call gets mapped to a regular request_irq() meaning the ISR gets
 * installed for the Linux domain. In case a driver has a non-realtime part, it gets always handled in the
 * Linux domain by the nonrt_handler.
 * Note: The function may be called on system startup, so it is not guaranteed, that it is already
 * running on the right CPU, which may be assigned to the RT-process later on (see register_process()).
 */
int rt_request_irq(unsigned int irq, irqreturn_t (*rt_handler)(int, void *),
				   unsigned long irqflags, const char *devname, void *dev_id,
				   irqreturn_t (*nonrt_handler)(int, void *))
{
	irq_desc_t *desc;
	irqreturn_t (*act_handler)(int, void *) = nonrt_handler ? nonrt_handler : rt_handler;
#ifdef CONFIG_X86_LOCAL_APIC
	struct rt_proc_hdr *pproc = current->rt_proc;
#endif

	if (RT_INIT_VERBOSE)
		printkGen(NULL, "%s: irq=%d\n", __func__, irq);

	if (!rt_handler) {
		printkGen(KERN_ALERT, "%s: missing rt_handler for irq=%d\n", __func__, irq);
		return -EINVAL;
	}

	if (irq < 0 || irq >= NR_IRQS)
#if defined(CONFIG_X86_LOCAL_APIC)
		if (irq != rtx_apic_vector_irq(LOCAL_TIMER_VECTOR))
#endif
	{
		printkGen(KERN_ALERT, "%s: invalid irq=%d\n", __func__, irq);
		return -EINVAL;
	}

	if (irqflags & IRQF_DISABLED) {
		printkGen(KERN_ALERT, "%s: IRQF_DISABLED flag not allowed\n", __func__);
		return -EINVAL;
	}
	irqflags |= IRQF_NOBALANCING;


	/* Install the nonrt_handler in the LX domain. */
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

	/* For the lapic timer irq 'desc' will be NULL. */
	desc = irq_to_desc(irq);

	/*
	 * Set the IRQ mode for the interrupt to be displayed by /proc/interrupts.
	 * It doesn't matter if we are not running on the CPU, the irq is pinned
	 * down later on by rt_enable_irq_affinity().
	 */
	if (desc && desc->action) {
		if (rtx_get_cpu_var(rtx_mode))
			desc->action->mode = rtx_get_cpu_var(rtx_mode) == RT_DOMAIN ? IRQ_MODE_HARD : IRQ_MODE_SOFT;
	}
#if defined(CONFIG_X86_LOCAL_APIC)
	/* In this case we are already on the right CPU. */
	if (irq == rtx_apic_vector_irq(LOCAL_TIMER_VECTOR)) {
		if (rtx_get_cpu_var(rtx_mode))
			(&per_cpu(irq_stat, pproc->cpu))->apic_timer_mode = rtx_get_cpu_var(rtx_mode) == RT_DOMAIN ? IRQ_MODE_HARD : IRQ_MODE_SOFT;
	}
#endif

	/*
	 * Since we are not sure that we are running on the target CPU,
	 * we only keep rt_handler/devid in mind and set it later on.
	 */
	if (IS_RTX_TIMER_IRQ(irq)) {
		rtx_req_clock.handler = rt_handler;
		rtx_req_clock.cookie = dev_id;
	}
	else {
		rtx_req_irqs[irq].handler = rt_handler;
		rtx_req_irqs[irq].cookie = dev_id;
	}

	/* 
	 * The remaining stuff has to be handled by rt_enable_irq_affinity().
	 * If we are already running on the RT-cpu the RT-process is assigned to,
	 * we can implicitly invoke rt_enable_irq_affinity().
	 * Otherwise, the caller is responsible to invoke this function explicitly,
	 * when the RT-process is assigned to its RT-cpu.
	 */
	if (IS_REALTIME_PROCESS(current)) {
		rt_enable_irq_affinity(irq);
		return 1;
	}
	return 0;
}

/*
 * Free the Linux ISR for a non-RTX_TIMER IRQ.
 */
void rt_free_irq(int irq, void *devid)
{
	irq_desc_t *desc;

	if (RT_SHUTDOWN_VERBOSE)
		printkGen(NULL, "%s: irq=%d\n", __func__, irq);

	/*
	 * If we are still running on the RT-cpu the RT-process is assigned to,
	 * we can implicitly invoke rt_disable_irq_affinity().
	 * Otherwise, the caller is responsible to invoke this function explicitly,
	 * when the RT-process is assigned to its RT-cpu.
	 */
	if (IS_REALTIME_PROCESS(current))
		rt_disable_irq_affinity(irq);

	if (irq < 0 || irq >= NR_IRQS)
#if defined(CONFIG_X86_LOCAL_APIC)
		if (irq != rtx_apic_vector_irq(LOCAL_TIMER_VECTOR))
#endif
	{
		printkGen(KERN_ALERT, "%s: invalid irq=%d\n", __func__, irq);
		return;
	}

	desc = irq_to_desc(irq);
	if (desc && desc->action)
		desc->action->mode = 0;		// reset

	if (!IS_RTX_TIMER_IRQ(irq))
	{
		free_irq(irq, devid);								// remove LX ISR
		if (RT_SHUTDOWN_VERBOSE)
			printkGen(NULL, "unregistered isr for irq=%d in LX domain\n", irq);
	}
	return;
}

/*
 * Set the RT-handler/cookie and adjust the affinity
 * for this IRQ.
 * Note: In this case we have to be running on the target CPU,
 * the RT-process is assigned to.
 */
int rt_enable_irq_affinity(int irq)
{
	irq_desc_t *desc;
#ifdef CONFIG_SMP
	struct rt_proc_hdr *pproc = current->rt_proc;
#endif
	struct rtx_pcd *pcd = &__raw_get_cpu_var(rtx_percpu_data);

	if (RT_INIT_VERBOSE)
		printkGen(NULL, "%s: irq=%d\n", __func__, irq);

	if (irq < 0 || irq >= NR_IRQS)
#if defined(CONFIG_X86_LOCAL_APIC)
		if (irq != rtx_apic_vector_irq(LOCAL_TIMER_VECTOR))
#endif
	{
		printkGen(KERN_ALERT, "%s: invalid irq=%d\n", __func__, irq);
		return -EINVAL;
	}

	if (!IS_REALTIME_PROCESS(current)) {
		printkGen(KERN_ALERT, "%s: invalid process context for setting irq affinity (irq=%d)\n", __func__, irq);
		return -EINVAL;
	}

	local_irq_disable_hw();

	if (IS_RTX_TIMER_IRQ(irq)) {
		pcd->rtx_irqs[irq].handler = rtx_req_clock.handler;
		pcd->rtx_irqs[irq].cookie = rtx_req_clock.cookie;
	}
	else {
		pcd->rtx_irqs[irq].handler = rtx_req_irqs[irq].handler;
		pcd->rtx_irqs[irq].cookie = rtx_req_irqs[irq].cookie;
	}

	desc = irq_to_desc(irq);

#if defined(CONFIG_X86_LOCAL_APIC)
	if (irq != rtx_apic_vector_irq(LOCAL_TIMER_VECTOR))
#endif
	{
		/* Activate the simple flow for the LX-Isr. */
		desc->handle_lx = desc->handle_irq;
		desc->handle_irq = handle_simple_irq;
	}
#if defined(CONFIG_X86_LOCAL_APIC) && defined(CONFIG_RTX_DOMAIN_HRT)
	else {
		int res;

		// From now we have to emulate the LX ticks in the RT domain.
		if ((res = rtx_request_tickdev("lapic", rtx_switch_htick_mode, rtx_next_htick_shot, &rtx_timer_freq)) != CLOCK_EVT_MODE_ONESHOT) {
			local_irq_enable_hw();
			printkGen(KERN_ALERT, "highres timer: lapic is not running in oneshot mode (res=%d)\n", res);
			return -EFAULT;
		}
		// Fake a lapic timer interrupt.
		log_interrupt_as_pending(irq);

		INIT_LIST_HEAD(&__raw_get_cpu_var(lx_itmr.it_link));
	}
#endif
	local_irq_enable_hw();

	if (desc && desc->action) {
		if (rtx_get_cpu_var(rtx_mode))
			desc->action->mode = rtx_get_cpu_var(rtx_mode) == RT_DOMAIN ? IRQ_MODE_HARD : IRQ_MODE_SOFT;
	}

#ifdef CONFIG_SMP
    /* Pin down IRQ to the same CPU the RT-process is assigned to. */
	if (!IS_RTX_TIMER_IRQ(irq))
	{
		int ret;

		down(&aud_rt_hdr.dev_sem);
		if ((ret = rtx_pin_irq(irq, pproc->cpu_mask)))
			printkGen(KERN_ALERT, "set irq affinity to cpumask=%#x (pin down irq=%d) failed reason=%d\n", pproc->cpu_mask, irq, ret);
		else
			if (RT_INIT_VERBOSE)
				printkGen(NULL, "set irq affinity to cpumask=%#x (pin down irq=%d)\n", pproc->cpu_mask, irq);
		up(&aud_rt_hdr.dev_sem);
	}
#endif
	return 0;
}

/*
 * Uninstall the RT-handler/cookie and turn back the affinity
 * for this IRQ to be handled only in Linux.
 * Note: In this case we have to be running on the target CPU,
 * the RT-process is assigned to.
 */
int rt_disable_irq_affinity(int irq)
{
	irq_desc_t *desc;
#ifdef CONFIG_X86_LOCAL_APIC
	struct rt_proc_hdr *pproc = current->rt_proc;
#endif
	struct rtx_pcd *pcd = &__raw_get_cpu_var(rtx_percpu_data);
#ifdef CONFIG_SMP
	int ret;
#endif

	if (RT_SHUTDOWN_VERBOSE)
		printkGen(NULL, "%s: irq=%d\n", __func__, irq);

	if (irq < 0 || irq >= NR_IRQS)
#if defined(CONFIG_X86_LOCAL_APIC)
		if (irq != rtx_apic_vector_irq(LOCAL_TIMER_VECTOR))
#endif
	{
		printkGen(KERN_ALERT, "%s: invalid irq=%d\n", __func__, irq);
		return -EINVAL;
	}

	if (!IS_REALTIME_PROCESS(current)) {
		printkGen(KERN_ALERT, "%s: invalid process context for setting irq affinity (irq=%d)\n", __func__, irq);
		return -EINVAL;
	}

	// Uninstall the rt_handler from the RT domain.
	local_irq_disable_hw();
	pcd->rtx_irqs[irq].handler = NULL;
	pcd->rtx_irqs[irq].cookie = NULL;

	desc = irq_to_desc(irq);

	// Deactivate the simple flow for the LX-Isr.
#if defined(CONFIG_X86_LOCAL_APIC)
	if (irq != rtx_apic_vector_irq(LOCAL_TIMER_VECTOR))
#endif
	{
		desc->handle_irq = desc->handle_lx;
	}
#if defined(CONFIG_X86_LOCAL_APIC)
	else {
		(&per_cpu(irq_stat, pproc->cpu))->apic_timer_mode = 0;
#ifdef CONFIG_RTX_DOMAIN_HRT
		// From now the LX ticks are handled native again.
		rtx_release_tickdev();
		rt_release_emulated_lx_tick();

		// Fake a lapic timer interrupt.
		log_interrupt_as_pending(irq);
#endif
	}
#endif

	local_irq_enable_hw();

	if (desc && desc->action)
		desc->action->mode = 0;

#ifdef CONFIG_SMP
	// Do not unpin the RTX_TIMER irq.
	if (!IS_RTX_TIMER_IRQ(irq))
	{
		down(&aud_rt_hdr.dev_sem);
		// Unpin the IRQ.
		if ((ret = rtx_unpin_irq(irq)))
			printkGen(KERN_ALERT, "set irq affinity to cpumask=%#x (unpin irq=%d) failed reason=%d\n", lx_irq_affinity[irq], irq, ret);
		else
			if (RT_INIT_VERBOSE)
				printkGen(NULL, "set irq affinity to cpumask=%#x (unpin irq=%d)\n", lx_irq_affinity[irq], irq);
		up(&aud_rt_hdr.dev_sem);
	}
#endif
	return 0;
}


static inline void __local_irq_restore_nosync(unsigned long x)
{
	struct rtx_pcd *pcd;
	unsigned long flags;

	local_irq_save_hw(flags);
	pcd = &__raw_get_cpu_var(rtx_percpu_data);
	if (raw_irqs_disabled_flags(x))
		__set_bit(LX_IRQS_STALLED, &pcd->rtx_irq_control);
	else
		__clear_bit(LX_IRQS_STALLED, &pcd->rtx_irq_control);
	local_irq_restore_hw(flags);
}

/*
 * Fixup interrupt flag.
 * Have the saved hw state look like the domain stall bit.
 */
static inline void __fixup_if(int s, struct pt_regs *regs)
{
	if (s)
		regs->flags &= ~X86_EFLAGS_IF;
	else
		regs->flags |= X86_EFLAGS_IF;
}

static void do_machine_check_vector(struct pt_regs *regs, long error_code)
{
#ifdef CONFIG_X86_MCE
	extern void (*machine_check_vector)(struct pt_regs *, long);
	machine_check_vector(regs,error_code);
#endif /* CONFIG_X86_MCE */
}

typedef void dotraplinkage __rtx_exhandler(struct pt_regs *, long);

typedef __rtx_exhandler *__rtx_exptr;

static __rtx_exptr __rtx_extable[] = {

	[ex_do_divide_error] = &do_divide_error,
	[ex_do_overflow] = &do_overflow,
	[ex_do_bounds] = &do_bounds,
	[ex_do_invalid_op] = &do_invalid_op,
	[ex_do_coprocessor_segment_overrun] = &do_coprocessor_segment_overrun,
	[ex_do_invalid_TSS] = &do_invalid_TSS,
	[ex_do_segment_not_present] = &do_segment_not_present,
	[ex_do_stack_segment] = &do_stack_segment,
	[ex_do_general_protection] = do_general_protection,
	[ex_do_page_fault] = (__rtx_exptr)&do_page_fault,
	[ex_do_spurious_interrupt_bug] = &do_spurious_interrupt_bug,
	[ex_do_coprocessor_error] = &do_coprocessor_error,
	[ex_do_alignment_check] = &do_alignment_check,
	[ex_machine_check_vector] = &do_machine_check_vector,
	[ex_do_simd_coprocessor_error] = &do_simd_coprocessor_error,
	[ex_do_device_not_available] = &do_device_not_available,
	[ex_do_iret_error] = &do_iret_error,
};

#define TRAP_DEV_NOT_AVAILABLE	7

/*
 * With the exception of fp error for saving/restoring the fpu context all faults are handled in the Linux domain
 * e.g. an installed handler gets executed in the Linux domain. The base assumption is, that within time critical
 * paths a thread has to avoid running into an exception. So execution in the Linux domain should not pose a problem.
 * On the other hand it is the most elegant solution saving a lot of implementation efforts.
 */
static int rtx_trap_handling(struct pt_regs *regs, long error_code, int vector)
{
	unsigned long flags;

#ifndef CONFIG_RTX_OPTIMIZE_FOR_RELEASE
	rt_fault_addr = regs->ip;
#endif

	/* We have also to consider exceptions
	 * during a task switch. */
	if (IS_REALTIME) {
		switch(vector) {
		case ex_do_debug:
		case ex_do_int3:
			if (RT_DEBUG_VERBOSE)
				printkGen(NULL, "debug exception %d in RT domain at address=%#lx\n", vector, regs->ip);

#ifdef CONFIG_RTX_EXTENDED_RT_DEBUGGING_SUPPORT
			rtx_stopAllRtThreads(current);
#endif
			current->rt_state3 |= RT_TASK_LX_DEBUG_PENDING;
			/* Handle debug exception in the LX domain. */
			rtx_migrate_to_lx(RTLX_TASK, NULL);
			return 0;	
		case TRAP_DEV_NOT_AVAILABLE:
			if (RT_FP_EXCEPTION_VERBOSE)
				printkGen(NULL, "FP exception (trap %d) in RT domain at address=%#lx error=%#lx\n", TRAP_DEV_NOT_AVAILABLE, regs->ip, error_code);
			local_irq_save_hw(flags);
			math_state_restore();
			local_irq_restore_hw(flags);
			current->fpu_counter = 6;	// force math_restore for all subsequent context switches
			return 1;
		default:
			if (RT_EXCEPTION_VERBOSE)
				printkGen(NULL, "fault %d in RT domain at address=%#lx error=%#lx\n", vector, regs->ip, error_code);
			current->rt_state3 |= RT_TASK_FAULT_DETECTED;
			rtx_migrate_to_lx(RTLX_TASK, NULL);
			return 0;					// execute exception in the Linux domain
		}
	}
	else
		if (RTX_SYS_IS_RT_DOMAIN)
			panic("%s: fault %d in RT domain (no thread context) at address=%#lx error=%#lx\n", __func__, vector, regs->ip, error_code);
	return 0;
}

/* Handle all kinds of exceptions (LX and RT). */
int __rtx_handle_exception(struct pt_regs *regs, long error_code, int vector)
{
	unsigned long flags;

	/* Pick up the LX-domain state of the interrupted context. */
	local_save_flags(flags);

    if (!RTX_SYS_IS_RT_DOMAIN) {
		/*
		 * Replicate HW interrupt state into the virtual mask before
		 * calling the RT trap handler over the LX-domain. Also
		 * required later when calling the LX exception handler.
		 */
    	if (irqs_disabled_hw())
    		local_irq_disable();
    }

    if (unlikely(rtx_trap_handling(regs, error_code, vector))) {
    	__local_irq_restore_nosync(flags);
    	return 1;
    }

    /*
	 * 32-bit: In case we migrated to LX-domain inside the trap
	 * handler, restore the original IF from exception entry as the
	 * low-level return code will evaluate it.
	 */
   __fixup_if(raw_irqs_disabled_flags(flags), regs);

    /* Note: If the exception couldn't be handled in the RT-domain
     * we have already migrated to the LX-domain.
     */
    __rtx_extable[vector](regs,error_code);

    /*
	 * Relevant for 64-bit: Restore LX-domain state as the low-level
	 * return code will not align it to regs.flags.
	 */
	__local_irq_restore_nosync(flags);
    return 0;
}


int __rtx_divert_exception(struct pt_regs *regs, int vector)
{
	unsigned long flags;

	/* Pick up the LX domain state of the interrupted context. */
	local_save_flags(flags);

    if (!RTX_SYS_IS_RT_DOMAIN) {
	/*
	* Replicate HW interrupt state into the virtual mask.
	*/
    	if (irqs_disabled_hw())
    		local_irq_disable();
    }
    if (unlikely(rtx_trap_handling(regs, 0, vector))) {
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

void __rtx_halt_lx(void)
{
	struct rtx_pcd *pcd;

	/* Emulate sti+hlt sequence over the LX-domain. */
	local_irq_disable_hw();
	pcd = &__raw_get_cpu_var(rtx_percpu_data);
	__clear_bit(LX_IRQS_STALLED, &pcd->rtx_irq_control);
	if (rtx_pending_irqs(pcd)) {
		__rtx_handle_nonrt_pending(RTX_IRQMASK_ANY);
		local_irq_enable_hw();
	} else {
#ifdef CONFIG_RTX_TRACE_IRQSOFF
		rt_trace_end(0x8000000E);
#endif
		asm volatile("sti; hlt": : :"memory");
	}
}

/*
 * Implementation of the virtualized interrupt macros ( see arch/x86/include/asm/irqflags.h).
 * Note: Only __rtx_local_irq_enable() has to be provided as a C-function because it is
 * called in entry_32.S.
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

#if defined(CONFIG_RTX_DOMAIN_HRT) && defined(CONFIG_GENERIC_CLOCKEVENTS)
/*
 * Callback for an emulated Linux tick.
 * This routine is called in ISR context.
 * Note: HW interrupts must be off.
 */
int rtx_set_lx_tick_pending(void *arg)
{
	log_interrupt_as_pending(rtx_apic_vector_irq(LOCAL_TIMER_VECTOR));
	return 0;
}

#ifdef CONFIG_X86_LOCAL_APIC
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

/*
 * The LAPIC timer register function is called in LX context,
 * but the LX-thread should be pinned (no migration possible).
 */
static int lapic_timer_register(void)
{
	struct tick_device *tdev = &__raw_get_cpu_var(tick_cpu_device);

	if (RT_INIT_VERBOSE)
		printkGen(NULL,"%s: clock event device name=%s\n", __func__, tdev->evtdev->name);

	// Check TSC availability.
	if (!cpu_has_tsc) {
		printkGen(KERN_ALERT, "TSC absent or disabled!\n");
		printkGen(KERN_ALERT, "> This will disable the LAPIC as a clock device, and\n");
		printkGen(KERN_ALERT, "> cause Audis to fail providing any timing service.\n");
		return -1;
	}

	// Scaled math (rtx_imuldiv_ceil)
	// depends on cpu frequencies < 4 GHz.
	if (rtx_cpu_freq >= 0x100000000LL) {
		printkGen(KERN_ALERT, "Cpu frequencies > 4 GHz are not supported!\n");
		return -1;
	}

	// Check if the timer is running in oneshot mode.
	if (tdev->evtdev->mode != CLOCK_EVT_MODE_ONESHOT) {
		printkGen(KERN_ALERT, "The %s timer service is not aperiodic!\n", tdev->evtdev->name);
		printkGen(KERN_ALERT, "> Enable High Resolution Timer or Tickless System support in kernel config\n");
		return -1;
	}
	if (!boot_cpu_has(X86_FEATURE_APIC)) {
		printkGen(KERN_ALERT, "Local APIC absent or disabled!\n");
		printkGen(KERN_ALERT, "> Disable APIC support in kernel config or pass \"lapic=1\" as bootparam.\n");
		return -1;
	}

	if (nmi_watchdog == NMI_IO_APIC) {
		printkGen(KERN_ALERT, "NMI kernel watchdog set to NMI_IO_APIC (nmi_watchdog=1).\n");
		printkGen(KERN_ALERT, "> This will disable the LAPIC as a clock device, and\n");
		printkGen(KERN_ALERT, "> cause Audis to fail providing any timing service.\n");
		printkGen(KERN_ALERT, "> Use NMI_LOCAL_APIC (nmi_watchdog=2), or disable the\n");
		printkGen(KERN_ALERT, "> NMI support entirely (nmi_watchdog=0).\n");
		return -1;
	}

	// Set the resolution of this timer.
	if (!init_timer_done) {
		aud_rt_timer->res_nsec = RTX_HIGH_RES_NSEC;

		aud_rt_timer->irq = __rtx_tick_irq;
		aud_rt_timer->mode = 0;

		// No compensation for a timer read
		aud_rt_timer->delta = 0;

		aud_rt_timer->dev_id = (void *)aud_rt_timer;
	}
	return 0;
}

static int lapic_timer_start(void)
{
	// Only to have an initialized value.
	__raw_get_cpu_var(rt_time_last) = RTX_TIMER_GET;
	return 0;
}

static int lapic_timer_unregister(void)
{
	return 0;
}
#endif

#endif

EXPORT_SYMBOL(rt_request_irq);
EXPORT_SYMBOL(rt_free_irq);
EXPORT_SYMBOL(rt_enable_irq_affinity);
EXPORT_SYMBOL(rt_disable_irq_affinity);
EXPORT_SYMBOL(execute_nonrt_handler);
EXPORT_SYMBOL(__rtx_handle_nonrt_pending);
EXPORT_SYMBOL(__rtx_local_irq_enable);
EXPORT_SYMBOL(__rtx_halt_lx);
EXPORT_PER_CPU_SYMBOL(rtx_percpu_data);
#ifdef RTX_HAVE_LLMULSHFT
EXPORT_SYMBOL(tsc_scale);
EXPORT_SYMBOL(tsc_shift);
#endif

