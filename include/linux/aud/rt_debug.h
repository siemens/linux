/*
 * linux/aud/rt_debug.h
 *
 * Copyright (C) 2012 Siemens AG
 * Contributed by manfred.neugebauer@siemens.com, 2012.
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
 * This file contains the definitions for extended debugging support
 * for realtime threads
 *
 */
#ifndef _LINUX_AUD_RT_DEBUG_H
#define _LINUX_AUD_RT_DEBUG_H

// support for extended realtime debug action
int rt_debug_action(int cmd, unsigned long arg);
// support for realtime watchdog activities
int rt_watchdog_action(int cmd, unsigned long arg);

// stop all realtime threads immediately
int rtx_stopAllRtThreads(struct task_struct *breakpointThread);
int rtx_stopAllRtThreadsFromLx(struct task_struct *breakpointThread);

// migration to / from lx extension
int testForExtendedDebugWaitToRt(void);
int testForExtendedDebugWaitToLx(void);


#endif /* _LINUX_AUD_RT_DEBUG_H */
