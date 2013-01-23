/*
 * linux/aud/rt_internal2.h
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
 * Common definitions for the interface between kernel / driver and library
 * to support the AuD API driver related functions
 * (realtime, synchronous timer, events)
 *
 */
#ifndef _LINUX_AUD_RT_INTERNAL2_H
#define _LINUX_AUD_RT_INTERNAL2_H

/*
 * Note: __SIGRTMIN (SIGCANCEL) and __SIGRTMIN+1 (SIGSETXID) 
 * are reserved by glibc.
 * The Audis fast timer handler has problems (debugging)
 * to use the identical definitions for SIGCANCEL and SIGTIMER
 * as it is used with glibc (see nptl/pthreadP.h).
 * Therefore, we use a separate signal number for SIGTIMER:
 * The first valid RT-signal is __SIGRTMIN+2.
 */
#ifdef SIGTIMER
#undef SIGTIMER	
#endif
#define SIGTIMER			__SIGRTMIN+2
#ifdef __KERNEL__
/* the Audis kernel also needs the SIGCANCEL definition (identical to glibc) */
#define SIGCANCEL			__SIGRTMIN
#endif

#define AuD_Dev_IOCTL_BASE			0x8000			// base for device specific ioctls
#define AuD_API_IOCTL_BASE			0x44410000		// base for ioctls common to all devices

/* ioctls for the Audis driver */
#define AuD_REGISTER_PROCESS			(AuD_Dev_IOCTL_BASE + 0x00)
#define AuD_UNREGISTER_PROCESS			(AuD_Dev_IOCTL_BASE + 0x01)
#define AuD_PROBE_REALTIME				(AuD_Dev_IOCTL_BASE + 0x02)
#define AuD_IS_REALTIME					(AuD_Dev_IOCTL_BASE + 0x03)
#define AuD_DEV_RT_PARAM				(AuD_Dev_IOCTL_BASE + 0x04)
#define AuD_REGISTER_AND_PROBE_REALTIME	(AuD_Dev_IOCTL_BASE + 0x05)
#define AuD_TEST_RT						(AuD_Dev_IOCTL_BASE + 0x06)
#define AuD_SET_DIAGNOSIS				(AuD_Dev_IOCTL_BASE + 0x07)
#define AuD_GET_VERSION					(AuD_Dev_IOCTL_BASE + 0x08)

/* Using the Audis driver for test purposes and for time measurement */
#define AuD_DEV_SEND_EVENT				(AuD_Dev_IOCTL_BASE + 0x09)
#define AuD_DEV_GET_TIME				(AuD_Dev_IOCTL_BASE + 0x0a)
#define AuD_DEV_GET_TIME_STAMP			(AuD_Dev_IOCTL_BASE + 0x0b)
#define AuD_DEV_GET_CPU					(AuD_Dev_IOCTL_BASE + 0x0c)

#define AuD_REGISTER_PROCESS_ATTACH		(AuD_Dev_IOCTL_BASE + 0x0d)
#define AuD_UNREGISTER_PROCESS_DETACH	(AuD_Dev_IOCTL_BASE + 0x0e)


/* Using the Audis driver to transfer a thread name to/from the kernel */
#define AuD_DEV_SET_THRD_NAME			(AuD_Dev_IOCTL_BASE + 0x20)
#define AuD_DEV_GET_THRD_NAME			(AuD_Dev_IOCTL_BASE + 0x21)

/* A RT-thread voluntarily returns to Linux */
#define AuD_DEV_RETURN_TO_LINUX			(AuD_Dev_IOCTL_BASE + 0x22)

/* Get the amount of time a RT-thread is running */
#define AuD_DEV_GET_THREAD_RUNTIME		(AuD_Dev_IOCTL_BASE + 0x23)

/* Allow sched_setaffinity()/sched_setscheduler() calls for (N)RT-threads */
#define AuD_DEV_ALLOW_SETAFFINITY		(AuD_Dev_IOCTL_BASE + 0x24)
#define AuD_DEV_ALLOW_SETSCHEDULER		(AuD_Dev_IOCTL_BASE + 0x25)
#define AuD_DEV_RESET_SCHED_FLAGS		(AuD_Dev_IOCTL_BASE + 0x26)

/* Store the user-defined thread-id for pthread_kill. */
#define AuD_DEV_STORE_TID				(AuD_Dev_IOCTL_BASE + 0x27)

/* Thread group commands */
#define AuD_TGROUP_CREATE		  		(AuD_Dev_IOCTL_BASE + 0x30)
#define AuD_TGROUP_DELETE		  		(AuD_Dev_IOCTL_BASE + 0x31)
#define AuD_TGROUP_THREAD_ADD			(AuD_Dev_IOCTL_BASE + 0x32)
#define AuD_TGROUP_THREAD_REMOVE		(AuD_Dev_IOCTL_BASE + 0x33)
#define AuD_TGROUP_SETQUOTA		  		(AuD_Dev_IOCTL_BASE + 0x34)
#define AuD_TGROUP_GETQUOTA		  		(AuD_Dev_IOCTL_BASE + 0x35)

/* RT tracer commands */
#define AuD_TRACE_MAX_BEGIN		  		(AuD_Dev_IOCTL_BASE + 0x40)
#define AuD_TRACE_MAX_END		  		(AuD_Dev_IOCTL_BASE + 0x41)
#define AuD_TRACE_MAX_RESET		  		(AuD_Dev_IOCTL_BASE + 0x42)
#define AuD_TRACE_USER_START			(AuD_Dev_IOCTL_BASE + 0x43)
#define AuD_TRACE_USER_STOP				(AuD_Dev_IOCTL_BASE + 0x44)
#define AuD_TRACE_USER_FREEZE			(AuD_Dev_IOCTL_BASE + 0x45)
#define AuD_TRACE_SPECIAL				(AuD_Dev_IOCTL_BASE + 0x46)
#define AuD_TRACE_SPECIAL_U64			(AuD_Dev_IOCTL_BASE + 0x47)

/* RT extended debug support commands */
#define AuD_DEV_ATTACH_DEBUG		  		(AuD_Dev_IOCTL_BASE + 0x50)
#define AuD_DEV_DETACH_DEBUG		  		(AuD_Dev_IOCTL_BASE + 0x51)
#define AuD_DEV_WAIT_FOR_DEBUG_EVENT		  	(AuD_Dev_IOCTL_BASE + 0x52)
#define AuD_DEV_HANDLE_DEBUG_EVENT		  	(AuD_Dev_IOCTL_BASE + 0x53)
#define AuD_DEV_TEST_WATCHDOG_EVENT		  	(AuD_Dev_IOCTL_BASE + 0x54)
#define AuD_DEV_FORCE_USER_BREAK		  	(AuD_Dev_IOCTL_BASE + 0x55)
#define AuD_DEV_TEST_WATCHDOG_IN_DEBUG		  	(AuD_Dev_IOCTL_BASE + 0x56)

/* Definitions of ioctl to handle device based API extensions (sync timer, events) */
#define AuD_REGISTER_CLOCK			(AuD_API_IOCTL_BASE + 0x08)
#define AuD_EVENT_CREATE			(AuD_API_IOCTL_BASE + 0x09)
#define AuD_WAIT_FOR_EVENT		  	(AuD_API_IOCTL_BASE + 0x0a)
#define AuD_UNREGISTER_CLOCK		(AuD_API_IOCTL_BASE + 0x0b)

struct rt_clock_desc {
	int clock_type;
	struct timespec clock_spec;
};

struct rt_ev_desc {
	unsigned event;
	sigevent_t sigevent;
};

/* new probe interface to pass the write buffer to the kernel */
#define PROBE_DESCR_VERSION 0x20080108

struct probe_descr {
	unsigned version;
	char *write_buff;
	int write_len;
};

#define SIGEV_SIGNAL_SV_MAGIC		0x5f6e7d00

struct rt_config {
	int no_of_timers;
	int no_of_sigq;
	int no_of_headers;
	int no_of_fu_req;	// for delegating futex_wake requests to Linux;
	int buf_size;		// buffer size for delegated write calls
	int td_prio;		// prio of the timer daemon for RT
	int wd_prio;		// prio of the write daemon for Linux
};

struct exec_rt_kernel {
	unsigned version;
	unsigned exec_flags;
	int	cpu;
};

struct aud_version {
	unsigned version_rt;
	unsigned version_dev;
};

struct aud_diag {
	unsigned diag_rt;
	unsigned diag_dev;
};

struct write_buf {
	char *begin;
	int  len;
};

struct thrd_name_buf {
	int thrdPid;
	char *thrdName;
};

struct thread_runtime_par {
	int pid;
	unsigned long long *time;			// accumulated runtime
};

struct allow_set_par {
	int policy;
	int prio;
};

/* diagnostic flags for the Audis device and the realtime driver */
#define RT_FUTEX_VERBOSE_BIT			0x00000001
#define RT_SIGNAL_VERBOSE_BIT			0x00000002
#define RT_START_VERBOSE_BIT			0x00000004
#define RT_INIT_VERBOSE_BIT			0x00000008
#define RT_SHUTDOWN_VERBOSE_BIT			0x00000010
#define RT_CHECK_VERBOSE_BIT			0x00000020
#define RT_THREAD_VERBOSE_BIT			0x00000040
#define RT_TIMER_VERBOSE_BIT			0x00000080
#define RT_EVENT_VERBOSE_BIT			0x00000100
#define RT_MIGRATE_VERBOSE_BIT			0x00000200
#define RT_FAULT_VERBOSE_BIT			0x00000400
#define RT_SYSCALL_IN_LX_VERBOSE_BIT		0x00000800
#define RT_SYSCALL_NOT_IMPL_VERBOSE_BIT		0x00001000
#define AUD_DEV_VERBOSE_BIT			0x00002000
#define RT_EXCEPTION_VERBOSE_BIT  		0x00004000
#define RT_WARN_VERBOSE_BIT			0x00008000
#define RT_PRINT_STDOUT_BIT			0x00010000
#define RT_PRINT_STDERR_BIT			0x00020000
#define RT_FP_EXCEPTION_VERBOSE_BIT		0x00040000
#define RT_DEBUG_VERBOSE_BIT			0x00080000
#define RT_UNALIGNMENT_VERBOSE_BIT		0x00100000

#define RT_ALL_EXCEPTION_BITS	(RT_EXCEPTION_VERBOSE_BIT | RT_FP_EXCEPTION_VERBOSE_BIT | RT_DEBUG_VERBOSE_BIT | RT_UNALIGNMENT_VERBOSE_BIT)

#endif /* _LINUX_AUD_RT_INTERNAL2_H */

