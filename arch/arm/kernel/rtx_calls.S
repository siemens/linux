/*
 *  linux/arch/arm/kernel/rtx_calls.S
 *
 *  Copyright (C) 2008 Manfred.Neugebauer@siemens.com
 *  derived from calls.S
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  This file is included twice in entry-common.S
 */

/* m.n. we have problems to include it in entry-common.S the same way as calls.S */
/*so we handle it separately and include it at the start of entry-common.S once  */

#define NR_rtx_syscalls 364

#ifndef NR_rtx_syscalls
#define NR_rtx_syscalls 364
#else

/* excludes unsupported system calls in uClinux */
#ifndef CONFIG_MMU
#define sys_rt_fork_wrapper	sys_rt_ni_syscall
#define sys_rt_msync		sys_rt_ni_syscall
#define sys_rt_mlock		sys_rt_ni_syscall
#define sys_rt_munlock		sys_rt_ni_syscall
#define sys_rt_mincore		sys_rt_ni_syscall
#define sys_rt_madvise		sys_rt_ni_syscall
#define sys_rt_remap_file_pages	sys_rt_ni_syscall
#endif

200:
/* 0 */		.long	sys_rt_ni_syscall
		.long	sys_rt_exit
		.long	sys_rt_ni_syscall
		.long	sys_rt_read
		.long	sys_rt_write
/* 5 */		.long	execute_in_lx			/* 5 open */
		.long	execute_in_lx			/* close */
		.long	sys_rt_ni_syscall		/* was sys_waitpid */
		.long	sys_rt_ni_syscall
		.long	execute_in_lx			/* 9: link */
/* 10 */	.long	execute_in_lx			/* 10: unlink */
		.long	sys_rt_ni_syscall
		.long	execute_in_lx			/* chdir */
		.long	sys_rt_time			/* 13: time */
		.long	sys_rt_ni_syscall
/* 15 */	.long	execute_in_lx			/* chmod */
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall		/* was sys_break */
		.long	sys_rt_ni_syscall		/* was sys_stat */
		.long	execute_in_lx			/* 19: lseek */
/* 20 */	.long	sys_rt_getpid
		.long	execute_in_lx			/* mount */
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
/* 25 */	.long	execute_in_lx			/* 25: sys_stime */
		.long	sys_rt_ni_syscall
		.long	execute_in_lx			/* 27: alarm */
		.long	sys_rt_ni_syscall
		.long	execute_in_lx			/* 29: pause */
/* 30 */	.long	execute_in_lx			/* 30: utime */
		.long	sys_rt_ni_syscall		/* was sys_stty */
		.long	sys_rt_ni_syscall		/* was sys_getty */
		.long 	execute_in_lx			/* 33: access */
		.long	sys_rt_ni_syscall
/* 35 */	.long	sys_rt_ni_syscall		/* was sys_ftime */
		.long	execute_in_lx                   /* 36:   sys_sync */
		.long	execute_in_lx			/* 37: kill */
		.long	execute_in_lx			/* 38: rename */
		.long	execute_in_lx			/* 39: mkdir */
/* 40 */	.long	execute_in_lx			/* 40: rmdir */
		.long	execute_in_lx		    	/* 41: dup */
		.long	execute_in_lx			/* 42: pipe */
		.long	execute_in_lx			/* 43: times */
		.long	sys_rt_ni_syscall
/* 45 */	.long	execute_in_lx			/* brk */
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	execute_in_lx			/* 49: geteuid16 */
/* 50 */	.long	execute_in_lx			/* 50 getegid16 */
		.long	sys_rt_ni_syscall
		.long	execute_in_lx			/* 52: umount */
		.long	sys_rt_ni_syscall		/* was sys_lock */
		.long	sys_rt_ioctl
/* 55 */	.long	execute_in_lx			/* 55: fcntl */
		.long	sys_rt_ni_syscall		/* was sys_mpx */
		.long	sys_rt_ni_syscall	/* 57: setpgid - do not change (glibc expects ENOSYS to detect rt threads) */
		.long	sys_rt_ni_syscall		/* was sys_ulimit */
		.long	sys_rt_ni_syscall		/* was sys_olduname */
/* 60 */	.long	execute_in_lx			/* 60 umask */
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	execute_in_lx			/* dup2 */
		.long	sys_rt_ni_syscall
/* 65 */	.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	execute_in_lx			/* sigaction */
		.long	sys_rt_ni_syscall		/* was sys_sgetmask */
		.long	sys_rt_ni_syscall		/* was sys_ssetmask */
/* 70 */	.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
/* 75 */	.long	execute_in_lx			/* 75: setrlimit */
		.long	execute_in_lx			/* 76: old_getrlimit */
		.long	sys_rt_ni_syscall
		.long	sys_rt_gettimeofday
		.long	 execute_in_lx			/* settimeofday */
/* 80 */	.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall		/* was sys_lstat */
/* 85 */	.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	execute_in_lx			/* sys_reboot */
		.long	sys_rt_ni_syscall
/* 90 */	.long	execute_in_lx			/* 90: old mmap*/
		.long	execute_in_lx			/* munmap */
		.long	execute_in_lx			/* truncate */
		.long	execute_in_lx			/* 93: ftruncate */
		.long	sys_rt_ni_syscall
/* 95 */	.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall		/* was sys_profil */
		.long	execute_in_lx			/* 99: sys_statfs */
/* 100 */	.long	execute_in_lx			/* 100: fstatfs */
		.long	execute_in_lx			/* ioperm */
		.long	execute_in_lx			/* 102: socket functions - socketcall*/
		.long	execute_in_lx			/* 103: syslog */
		.long	sys_rt_ni_syscall
/* 105 */	.long	sys_rt_ni_syscall
		.long	execute_in_lx			/* 106: newstat */
		.long	execute_in_lx			/* 107: newlstat */
		.long	execute_in_lx			/* 108: newfstat */
		.long	execute_in_lx			/* 109: uname */
/* 110 */	.long	execute_in_lx			/* 110 iopl */
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall		/* call a syscall */
		.long	sys_rt_ni_syscall
/* 115 */	.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	execute_in_lx			/* sys_fsync */
		.long	execute_in_lx			/* sigreturn */
/* 120 */	.long	sys_rt_clone			/* 120 clone */
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
/* 125 */	.long	execute_in_lx			/* 125: mprotect */
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall		/* was sys_create_module */
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
/* 130 */	.long	sys_rt_ni_syscall		/* was sys_get_kernel_syms */
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
/* 135 */	.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall		/* .long	_sys_afs_syscall */
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
/* 140 */	.long	execute_in_lx			/* 140: llseek */
		.long	execute_in_lx			/* 141: getdents */
		.long	execute_in_lx			/* 142: select */
		.long	sys_rt_ni_syscall
		.long	execute_in_lx			/* 144: msync */
/* 145 */	.long	execute_in_lx			/* 145: readv */
		.long	execute_in_lx			/* 146: writev */
		.long	sys_rt_ni_syscall
		.long	execute_in_lx			/* 148: fdatasync */
		.long	sys_rt_ni_syscall
/* 150 */	.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	execute_in_lx			/* 152: mlockall */
		.long	sys_rt_ni_syscall
		.long	sys_rt_sched_setparam
/* 155 */	.long	sys_rt_sched_getparam
		.long	sys_rt_sched_setscheduler
		.long	sys_rt_sched_getscheduler
		.long	sys_rt_sched_yield
		.long	execute_in_lx			/* sched_get_prio_max */
/* 160 */	.long	execute_in_lx			/* 160 sched_get_prio_min */
		.long	sys_rt_ni_syscall
		.long	sys_rt_nanosleep
		.long	execute_in_lx			/* mremap */
		.long	sys_rt_ni_syscall
/* 165 */	.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall		/* was sys_query_module */
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
/* 170 */	.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	execute_in_lx			/* sys_prctl */
		.long	sys_rt_ni_syscall
		.long	execute_in_lx			/* sigaction */
/* 175 */	.long	sys_rt_ni_syscall
		.long	sys_rt_sigpending_rt
		.long	sys_rt_sigtimedwait_rt
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
/* 180 */	.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	execute_in_lx			/* 183: getcwd */
		.long	sys_rt_ni_syscall
/* 185 */	.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
/* 190 */	.long	sys_rt_ni_syscall
		.long	execute_in_lx			/* 191: getrlimit */
		.long	execute_in_lx			/* mmap2 */
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
/* 195 */	.long	execute_in_lx			/* 195: stat64 */
		.long	execute_in_lx			/* 196: lstat64 */
		.long	execute_in_lx			/* fstat64 */
		.long	sys_rt_ni_syscall
		.long	sys_rt_getuid			/* getuid */
/* 200 */	.long	execute_in_lx			/* 200; getgid */
		.long	execute_in_lx			/* 201: geteuid */
		.long	execute_in_lx			/* 202: getegid */
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
/* 205 */	.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
/* 210 */	.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
/* 215 */	.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
/* 220 */	.long	sys_rt_ni_syscall
		.long	execute_in_lx 		/* fcntl64 */
		.long	sys_rt_ni_syscall /* TUX */
		.long	sys_rt_ni_syscall
		.long	sys_rt_gettid			/* 224 */
/* 225 */	.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
/* 230 */	.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
/* 235 */	.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_tkill
		.long	sys_rt_ni_syscall
/* 240 */	.long	sys_rt_futex
		.long	execute_in_lx		/* sched_setaffinity */
		.long	execute_in_lx		/* sched_getaffinity */
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
/* 245 */	.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_exit_group	    /* exit_group */
		.long	sys_rt_ni_syscall
/* 250 */	.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
	 	.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall	/* sys_set_thread_area */
/* 255 */	.long	sys_rt_ni_syscall	/* sys_get_thread_area */
 		.long	sys_rt_ni_syscall
		.long	sys_rt_timer_create
		.long	sys_rt_timer_settime
		.long   sys_rt_timer_gettime
/* 260 */	.long	sys_rt_ni_syscall
		.long	sys_rt_timer_delete
		.long	sys_rt_clock_settime
		.long	sys_rt_clock_gettime
		.long	sys_rt_clock_getres
/* 265 */	.long	sys_rt_clock_nanosleep
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_tgkill
		.long	sys_rt_ni_syscall
/* 270 */	.long	sys_rt_ni_syscall
		.long	execute_in_lx
		.long	execute_in_lx
		.long	execute_in_lx
		.long	execute_in_lx			/* mq_open */
/* 275 */	.long	execute_in_lx			/* mq_unlink */
		.long	sys_rt_mq_timedsend
		.long	sys_rt_mq_timedreceive
		.long	sys_rt_ni_syscall		/* mq_notify */
		.long	execute_in_lx			/* mq_getsetattr */
/* 280 */	.long	execute_in_lx
		.long	execute_in_lx			/* sys_socket */
		.long	execute_in_lx			/* sys_bind */
		.long	execute_in_lx			/* sys_connect */
		.long	execute_in_lx			/* sys_listen */
/* 285 */	.long	execute_in_lx			/* sys_accept */
		.long	execute_in_lx			/* sys_getsockname */
		.long	execute_in_lx			/* sys_getpeername */
		.long	execute_in_lx			/* sys_socketpair */
		.long	execute_in_lx			/* sys_send */
/* 290 */	.long	execute_in_lx			/* sys_sendto */
		.long	execute_in_lx			/* sys_recv */
		.long	execute_in_lx			/* sys_recvfrom */
		.long	execute_in_lx			/* sys_shutdown */
		.long	execute_in_lx			/* sys_setsockopt */
/* 295 */	.long	execute_in_lx			/* sys_getsockopt */
		.long	execute_in_lx			/* sys_sendmsg */
		.long	execute_in_lx			/* sys_recvmsg */
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
/* 300 */	.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
/* 305 */	.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
/* 310 */	.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
/* 315 */	.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
/* 320 */	.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
/* 325 */	.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
/* 330 */	.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
/* 335 */	.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_ni_syscall
		.long	sys_rt_set_robust_list
		.long	sys_rt_ni_syscall
/* 340 */	.long	sys_rt_ni_syscall
		.rept	NR_rtx_syscalls - (. - 200b) / 4
			.long	sys_rt_ni_syscall
		.endr
#endif
