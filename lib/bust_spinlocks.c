/*
 * lib/bust_spinlocks.c
 *
 * Provides a minimal bust_spinlocks for architectures which don't have one of their own.
 *
 * bust_spinlocks() clears any spinlocks which would prevent oops, die(), BUG()
 * and panic() information from reaching the user.
 *
 * RTX_DOMAIN is based on the adeos-ipipe patch.
 */

#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/tty.h>
#include <linux/wait.h>
#include <linux/vt_kern.h>
#include <linux/console.h>
#ifdef CONFIG_RTX_DOMAIN
#include <linux/rtx_system.h>
#include <asm/rtx_percpu.h>
#endif

void __attribute__((weak)) bust_spinlocks(int yes)
{
#ifdef CONFIG_RTX_DOMAIN
	unsigned long flags;
#endif
	if (yes) {
		++oops_in_progress;
#ifdef CONFIG_RTX_DOMAIN
		local_irq_save_hw(flags);
		if (!rtx_oops) {
			++rtx_oops;
			rtx_set_cpu_member(rtx_percpu_data, rtx_curr_dom, LX_DOMAIN_NATIVE);	// set native mode
			local_irq_restore_hw(flags);
			/* Check if there is at least one RT process active. */
			if (rtx_mode_rt == RT_DOMAIN) {
#ifdef CONFIG_RTX_DOMAIN_STOP_ON_PANIC
				printk("RT domain handling is stopped because of a kernel panic\n");
#else
				printk("RT domain handling continues although a kernel panic has been detected\n");
#endif
				rtx_trace_panic();
			}
		}
		else
			local_irq_restore_hw(flags);
#endif

	} else {
#ifdef CONFIG_VT
		unblank_screen();
#endif
		console_unblank();
#if defined(CONFIG_RTX_DOMAIN) && defined(CONFIG_RTX_TRACE_PANIC)
  		rt_trace_panic_dump();
#endif
		if (--oops_in_progress == 0)
			wake_up_klogd();
	}
}


