/*
 * linux/aud/rt_kernel.h
 *
 * 2007-19-01:  Manfred.Neugebauer@siemens.com
 * Copyright (c) 2007 Siemens AG
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
 * Definitions for the kernel (realtime) driver interface to support
 * the AuD API driver related functions (realtime, synchronous timer, events).
 * 
 */
#ifndef _LINUX_AUD_RT_KERNEL_H
#define _LINUX_AUD_RT_KERNEL_H

/* we have common defines with rtime.h */
#define __SIGRTMIN SIGRTMIN /* taken from the general definitions */
#include <linux/aud/rt_internal.h>
/* we have common defines with rtime_lib.h */
#include <linux/aud/rt_internal2.h>

/* 
 * rt_driver.h contains the definitions a driver writer needs when exploiting the additional
 * aud features register_clock(), event_create() provided by the Audis driver.
 */
#include <linux/aud/rt_driver.h>

#endif /* _LINUX_AUD_RT_KERNEL_H */

