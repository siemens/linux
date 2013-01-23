/* -*- linux-c -*-
 * include/linux/aud/rt_trace.h
 *
 * Copyright (C) 2005 Luotao Fu.
 *               2005-2007 Jan Kiszka.
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
 * Copyright (C) 2010 Wolfgang Hartmann
 *
 * This file is based on the adeos-ipipe patch (include/linux/ipipe_trace.h).
 */

#ifndef _LINUX_AUD_RT_TRACE_H
#define _LINUX_AUD_RT_TRACE_H

#ifdef CONFIG_RTX_TRACE

#include <linux/types.h>

void rt_trace_begin(unsigned long v);
void rt_trace_end(unsigned long v);
void rt_trace_freeze(unsigned long v);
void rt_trace_special(unsigned char special_id, unsigned long v);
void rt_trace_pid(pid_t pid, short prio);
void rt_trace_event(unsigned char id, unsigned long delay_tsc);
int rt_trace_max_reset(void);
int rt_trace_frozen_reset(void);

#else /* !CONFIG_RTX_TRACE */

#define rt_trace_begin(v)			do { (void)(v); } while(0)
#define rt_trace_end(v)			do { (void)(v); } while(0)
#define rt_trace_freeze(v)			do { (void)(v); } while(0)
#define rt_trace_special(id, v)		do { (void)(id); (void)(v); } while(0)
#define rt_trace_pid(pid, prio)		do { (void)(pid); (void)(prio); } while(0)
#define rt_trace_event(id, delay_tsc)	do { (void)(id); (void)(delay_tsc); } while(0)
#define rt_trace_max_reset()			do { } while(0)
#define rt_trace_froze_reset()		do { } while(0)

#endif /* !CONFIG_RTX_TRACE */

#ifdef CONFIG_RTX_TRACE_PANIC
void rt_trace_panic_freeze(void);
void rt_trace_panic_dump(void);
#endif

#endif	/* !__LINUX_AUD_RT_TRACE_H */
