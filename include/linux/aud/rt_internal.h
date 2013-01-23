/*
 * linux/aud/rt_internal.h
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
 * Internal definitions to work with a realtime subsystem.
 * 
 */
#ifndef _LINUX_AUD_RT_INTERNAL_H
#define _LINUX_AUD_RT_INTERNAL_H
#include <linux/version.h>

/* Hard RT priorities */
#define RT_P0			0				// SCHED_OTHER
#define RT_P1			68				// RT-prio min
#define RT_P2			(RT_P1 + 1)
#define RT_P3			(RT_P1 + 2)
#define RT_P4			(RT_P1 + 3)
#define RT_P5			(RT_P1 + 4)
#define RT_P6			(RT_P1 + 5)
#define RT_P7			(RT_P1 + 6)
#define RT_P8			(RT_P1 + 7)
#define RT_P9			(RT_P1 + 8)
#define RT_P10			(RT_P1 + 9)
#define RT_P11			(RT_P1 + 10)
#define RT_P12			(RT_P1 + 11)
#define RT_P13			(RT_P1 + 12)
#define RT_P14			(RT_P1 + 13)
#define RT_P15			(RT_P1 + 14)
#define RT_P16			(RT_P1 + 15)
#define RT_P17			(RT_P1 + 16)
#define RT_P18			(RT_P1 + 17)
#define RT_P19			(RT_P1 + 18)
#define RT_P20			(RT_P1 + 19)
#define RT_P21			(RT_P1 + 20)
#define RT_P22			(RT_P1 + 21)
#define RT_P23			(RT_P1 + 22)
#define RT_P24			(RT_P1 + 23)
#define RT_P25			(RT_P1 + 24)
#define RT_P26			(RT_P1 + 25)
#define RT_P27			(RT_P1 + 26)
#define RT_P28			(RT_P1 + 27)
#define RT_P29			(RT_P1 + 28)
#define RT_P30			(RT_P1 + 29)
#define RT_P31			(RT_P1 + 30)		// RT-prio max

#define RT_PMIN		RT_P1
#define RT_PMAX		RT_P31
#define RT_PRIORITY_MIN		RT_PMIN			// Only for compatibility (e.g. glibc)

/* Soft RT priorities */
#define SRT_P0			0					// SCHED_OTHER
#define SRT_P1			53					// SRT-prio min
#define SRT_P2			(SRT_P1 + 1)
#define SRT_P3			(SRT_P1 + 2)
#define SRT_P4			(SRT_P1 + 3)
#define SRT_P5			(SRT_P1 + 4)
#define SRT_P6			(SRT_P1 + 5)
#define SRT_P7			(SRT_P1 + 6)
#define SRT_P8			(SRT_P1 + 7)
#define SRT_P9			(SRT_P1 + 8)
#define SRT_P10			(SRT_P1 + 9)
#define SRT_P11			(SRT_P1 + 10)
#define SRT_P12			(SRT_P1 + 11)
#define SRT_P13			(SRT_P1 + 12)
#define SRT_P14			(SRT_P1 + 13)
#define SRT_P15			(SRT_P1 + 14)		// SRT-prio max

#define SRT_PMIN		SRT_P1
#define SRT_PMAX		SRT_P15

/* Priority interrupt routine */
#define RT_PIR			(RT_P31 + 1)

/* Placeholder value for priority 0.
 * It must be a negative value of an
 * unsupported priority value. */
#define RT_NORM_PRIO	-5555

#define AUDIS_DRIVER_PATH		"/dev/audis"
#define AUDIS_DRIVER_NAME		"audis"

/* 
 * The flags RT_SOFT and RT_HARD are used in ioctl and rt_proc_hdr.state.
 * The remaining flags are defined in rt_system_state only.
 */
#define RT_SOFT 				0x0010
#define RT_HARD					0x0020
#define RT_TIMER_INITIALIZED	0x0040
#define RT_SHUTDOWN				0x0100
#define RT_SHUTDOWN_PROCESS		0x0200
#define RT_ATTACH_PROCESS		0x0400
#define RT_DETACH_PROCESS		0x0800
#define RT_EXEC_MASK			(RT_SOFT | RT_HARD)
/*
 * special flag to wait for end of a real process within create_process_image()
 */
#define RT_WAIT_FOR_EXIT		0x4000

/* Flags for audis_return_to_linux() */
#define R2L_FORCE				0x1			// return to Linux in any case
#define R2L_THREAD				0x2			// return to Linux if no pending signal for the current thread
#define R2L_PROCESS				0x4			// return to Linux if no pending signal for the RT-process

/* Return values for audis_return_to_linux() */
#define R2L_RET_SIG_PENDING_THREAD		0x1		// not returned to LX, pending signals for this thread
#define R2L_RET_SIG_PENDING_PROCESS		0x2		// not returned to LX, pending signals for other threads of the RT-process
#define R2L_RET_SIG_WAIT_THREAD			0x4		// returned from LX after waiting, a signal is sent to this thread
#define R2L_RET_SIG_WAIT_PROCESS		0x8		// returned from LX after waiting, a signal is sent to a thread of the RT-process
#define R2L_RET_SIG_WAIT_INTERRUPTED	0x10	// returned from LX after waiting, some RT event continued the thread

#define RT_SIG_WAIT_R2L					0x1		// RT thread voluntarily gives up control to LX
#define RT_SIG_WAIT_R2L_IN_PROGRESS		0x2	

#define R2L_CHECK_SIGNAL_DELIVERY(p)	(p->rt_proc->r2l & RT_SIG_WAIT_R2L)

/*
 * Preferred definitions of a set of realtime signals.
 * Note: The RT-signals __SIGRTMIN, __SIGRTMIN+1, __SIGRTMIN+2
 * are reserved (see rt_internal2.h).
 */
#define SIGRT0			(__SIGRTMIN + 3)
#define SIGRT1			(__SIGRTMIN + 4)
#define SIGRT2			(__SIGRTMIN + 5)
#define SIGRT3			(__SIGRTMIN + 6)
#define SIGRT4			(__SIGRTMIN + 7)
#define SIGRT5			(__SIGRTMIN + 8)
#define SIGRT6			(__SIGRTMIN + 9)
#define SIGRT7			(__SIGRTMIN + 10)
#define SIGRT8			(__SIGRTMIN + 11)
#define SIGRT9			(__SIGRTMIN + 12)
#define SIGRT10			(__SIGRTMIN + 13)
#define SIGRT11			(__SIGRTMIN + 14)
#define SIGRT12			(__SIGRTMIN + 15)
#define SIGRT13			(__SIGRTMIN + 16)
#define SIGRT14			(__SIGRTMIN + 17)
#define SIGRT15			(__SIGRTMIN + 18)

#define EXEC_T_STRUCT_VERSION	0x01072006

typedef struct rt_exec_t_struct {
	unsigned version;
	unsigned exec_flags;
} rt_exec_t;

#define CLOCK_SYNC	0x0008

/* Used for the event_create(), event_delete() functions. */
typedef unsigned int eventid_t;

/* Thread group specific */
typedef unsigned int tgroup_t;

#define TGROUP_PROCESSING_TIME_QUOTA		1
#define PROCESSING_TIME_QUOTA_MIN		0		/* per cent */
#define PROCESSING_TIME_QUOTA_MAX		90		/* per cent */

typedef struct tgroup_descr {
	tgroup_t *pgroup;
	tgroup_t group;
	int aspect;
	pid_t pid;
	int *pquota;
	int quota;
	int *pquota_peak;
	int quota_peak;
} tgroup_descr_t;

/* RT tracer ioctl arguments */
typedef struct trace_descr {
	unsigned long long ll_val;
	unsigned long l_val;
	int once;
	unsigned char id;
} trace_descr_t;

/* Signal/event stuff: 
 * SI_USER (0) and SI_TIMER (-2) are already defined by the kernel.
 */
#define SI_IOEVENT	(-8)		/* check if '-8' is an unused value (see asm-generic/siginfo.h) */

#ifdef __KERNEL__
/* siginfo_t structure:
 * kernel use: from "asm-generic/siginfo.h"
 */
#define si_timerid		_sifields._timer._tid
#define si_eventid		_sifields._timer._tid
#define si_threadid		_sifields._kill._tid
#else
/* application use: from "sysdeps/unix/sysv/linux/bits/siginfo.h"
 * si_timerid is already defined 
 */
#define si_eventid		_sifields._timer.si_tid
#define si_threadid		_sifields._kill.si_threadid
#endif

/* This definition is used by the kernel and the glibc (sigevent_set_notification(), sigevent_set_monitor()).
 * Note: Using _pad[1,2,3] prevents us from changing the sigevent structure (kernel/glibc).
 * But we have to take care that these fields are not used otherwise. */
#define si_cycle	_sigev_un._pad[1]
#define si_ovtid	_sigev_un._pad[2]
#define si_ovsig	_sigev_un._pad[3]

#define USE_CREATOR		0xDEAFFEED
#define INVALID_CREATOR	0xDEADBEEF

/* Add on to siginfo.h for AuD API	*/
#define SIGEV_SIGNAL_SV			(SIGEV_THREAD_ID | 0x10)


#endif /* _LINUX_AUD_RT_INTERNAL_H */

