/* -*- linux-c -*-
 * include/linux/rtx_tickdev.h
 *
 * Copyright (C) 2010 Siemens AG.
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

#ifndef __LINUX_RTX_TICKDEV_H
#define __LINUX_RTX_TICKDEV_H

#if defined(CONFIG_RTX_DOMAIN) && defined(CONFIG_GENERIC_CLOCKEVENTS)

#include <linux/clockchips.h>

struct tick_device;

struct rtx_tick_device {

	void (*emul_set_mode)(enum clock_event_mode,
			      struct clock_event_device *cdev);
	int (*emul_set_tick)(unsigned long delta,
			     struct clock_event_device *cdev);
	void (*real_set_mode)(enum clock_event_mode mode,
			      struct clock_event_device *cdev);
	int (*real_set_tick)(unsigned long delta,
			     struct clock_event_device *cdev);
	struct tick_device *slave;
	unsigned long real_max_delta_ns;
	unsigned long real_mult;
	int real_shift;
};

int rtx_request_tickdev(const char *devname,
			  void (*emumode)(enum clock_event_mode mode,
					  struct clock_event_device *cdev),
			  int (*emutick)(unsigned long evt,
					 struct clock_event_device *cdev),
			  unsigned long *tmfreq);

void rtx_release_tickdev(void);

#endif /* CONFIG_RTX_DOMAIN && CONFIG_GENERIC_CLOCKEVENTS */

#endif /* !__LINUX_RTX_TICKDEV_H */
