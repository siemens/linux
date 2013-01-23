/*
 * kernel/rtx_adapt.c
 *
 * Copyright (c) 2010 Siemens AG
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
 * This file is based on the adeos-ipipe/Xenomai patch.
 * Adaption to Audis by <wolfgang.hartmann@siemens.com>
 */
#include <linux/version.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/tick.h>
#include <linux/rtx_tickdev.h>
#include <linux/aud/rt_base.h>

#if defined CONFIG_RTX_DOMAIN_HRT && defined CONFIG_GENERIC_CLOCKEVENTS

static DEFINE_PER_CPU(struct rtx_tick_device, rtx_tick_cpu_device);

/*
 * Allocate a clock event device for a CPU-local timer.
 * Note: HW interrupts must be off.
 */
int rtx_request_tickdev(const char *devname,
			  void (*emumode)(enum clock_event_mode mode,
					  struct clock_event_device *cdev),
			  int (*emutick)(unsigned long delta,
					 struct clock_event_device *cdev),
			  unsigned long *tmfreq)
{
	struct rtx_tick_device *itd;
	struct tick_device *slave;
	struct clock_event_device *evtdev;
	unsigned long long freq;
	int status;

	itd = &__raw_get_cpu_var(rtx_tick_cpu_device);


	slave = &__raw_get_cpu_var(tick_cpu_device);

	if (strcmp(slave->evtdev->name, devname)) {
		/*
		 * No conflict so far with the current tick device,
		 * check whether the requested device is sane and has
		 * been blessed by the kernel.
		 */
		//status = __rtx_check_tickdev(devname) ?
		//	CLOCK_EVT_MODE_UNUSED : CLOCK_EVT_MODE_SHUTDOWN;

		// For Audis this is an error case.
		status = CLOCK_EVT_MODE_SHUTDOWN;
		goto out;
	}

	/*
	 * Our caller asks for using the same clock event device for
	 * ticking than we do, let's create a tick emulation device to
	 * interpose on the set_next_event() method, so that we may
	 * both manage the device in oneshot mode. Only the tick
	 * emulation code will actually program the clockchip hardware
	 * for the next shot, though.
	 *
	 * CAUTION: we still have to grab the tick device even when it
	 * current runs in periodic mode, since the kernel may switch
	 * to oneshot dynamically (highres/no_hz tick mode).
	 */

	evtdev = slave->evtdev;
	status = evtdev->mode;

    if (status == CLOCK_EVT_MODE_SHUTDOWN)
    	goto out;

	itd->slave = slave;
	itd->emul_set_mode = emumode;
	itd->emul_set_tick = emutick;
	itd->real_set_mode = evtdev->set_mode;
	itd->real_set_tick = evtdev->set_next_event;
	itd->real_max_delta_ns = evtdev->max_delta_ns;
	itd->real_mult = evtdev->mult;
	itd->real_shift = evtdev->shift;
	freq = (1000000000ULL * evtdev->mult) >> evtdev->shift;
	*tmfreq = (unsigned long)freq;
	evtdev->set_mode = emumode;
	evtdev->set_next_event = emutick;
	if (RTX_MAX_DELTA_NS != 0)
		evtdev->max_delta_ns = RTX_MAX_DELTA_NS;
	evtdev->mult = 1;
	evtdev->shift = 0;

out:
	return status;
}

/*
 * Release a clock event device for a CPU-local timer.
 * Note: HW interrupts must be off.
 */
void rtx_release_tickdev(void)
{
	struct rtx_tick_device *itd;
	struct tick_device *slave;
	struct clock_event_device *evtdev;

	itd = &__raw_get_cpu_var(rtx_tick_cpu_device);

	if (itd->slave != NULL) {
		slave = &__raw_get_cpu_var(tick_cpu_device);
		evtdev = slave->evtdev;
		evtdev->set_mode = itd->real_set_mode;
		evtdev->set_next_event = itd->real_set_tick;
		evtdev->max_delta_ns = itd->real_max_delta_ns;
		evtdev->mult = itd->real_mult;
		evtdev->shift = itd->real_shift;
		itd->slave = NULL;
	}
}

#endif /* CONFIG_RTX_DOMAIN_HRT && CONFIG_GENERIC_CLOCKEVENTS */

#ifdef CONFIG_RTX_DOMAIN_INTEGRITY_CHECK
void rtx_check_context(char *file, int line, const char *func)
{
	struct rtx_pcd *pcd;
	unsigned long flags;

	local_irq_save_hw(flags);
	pcd = &__raw_get_cpu_var(rtx_percpu_data);
	if (pcd->rtx_curr_dom == RT_DOMAIN) {
		rtx_trace_panic();
		panic("RTX domain integrity check failed at %s:%d:%s(): pid=%d cpu=%d rt_state=%#lx rt_state2=%#lx\n", file, line, func, current->pid, rtx_processor_id(), current->rt_state, current->rt_state2);
	}
	local_irq_restore_hw(flags);
}
EXPORT_SYMBOL(rtx_check_context);
#endif
