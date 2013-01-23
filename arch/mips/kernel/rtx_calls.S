/*
 * syscall table for realtime domain
 * manfred.neugebauer@siemens.com
 * derived from original table and
 * realtime work from other architectures
 */

	.macro	rt_syscalltable
#if defined(CONFIG_BINFMT_IRIX)
	mille	sys_rt_ni_syscall		0	/*    0 -  999 SVR4 flavour */
	mille	sys_rt_ni_syscall		0	/* 1000 - 1999 32-bit IRIX */
	mille	sys_rt_ni_syscall		0	/* 2000 - 2999 BSD43 flavour */
	mille	sys_rt_ni_syscall		0	/* 3000 - 3999 POSIX flavour */
#endif

        	/* we use the same routine as with linux and separate later */
	sys	sys_syscall			8	/* 4000 */
	sys	sys_rt_exit			1
	sys	sys_rt_ni_syscall		0
	sys	sys_rt_read			3
	sys	sys_rt_write			3
	sys	execute_in_lx			3	/* 4005: open */
	sys	execute_in_lx			1	/* 4006: close */
	sys	sys_rt_ni_syscall		3
	sys	sys_rt_ni_syscall		2
	sys	execute_in_lx			2	/* 4009: link */
	sys	execute_in_lx			1	/* 4010: unlink */
	sys	sys_rt_ni_syscall		0
	sys	execute_in_lx			1	/* 4012: chdir */
	sys	sys_rt_time			1	/* 4013: time */
	sys	sys_rt_ni_syscall		3
	sys	execute_in_lx			2	/* 4015: chmod */
	sys	sys_rt_ni_syscall		3
	sys	sys_rt_ni_syscall		0
	sys	sys_rt_ni_syscall		0	/* was sys_stat */
	sys	execute_in_lx			3	/* 4019: lseek */
	sys	sys_rt_getpid			0	/* 4020 */
		/* nr of arguments was 5, but we don't do stack modifications */
		/* when propagating to linux                                  */
	sys	execute_in_lx			3	/* 4021: mount */
	sys	sys_rt_ni_syscall		1
	sys	sys_rt_ni_syscall		1
	sys	sys_rt_ni_syscall		0
	sys	execute_in_lx			1	/* 4025: sys_stime */
	sys	sys_rt_ni_syscall		4
	sys	execute_in_lx			1	/* 4027: alarm */
	sys	sys_rt_ni_syscall		0	/* was sys_fstat */
	sys	execute_in_lx  			0	/* 4029: pause */
	sys	execute_in_lx  			2	/* 4030: utime */
	sys	sys_rt_ni_syscall		0
	sys	sys_rt_ni_syscall		0
	sys	execute_in_lx  			2	/* 4033: access */
	sys	sys_rt_ni_syscall		1
	sys	sys_rt_ni_syscall		0	/* 4035 */
	sys	execute_in_lx			0	/* 4036: sync */
	sys	execute_in_lx			2	/* 4037: kill */
	sys	execute_in_lx			2	/* 4038: rename */
	sys	execute_in_lx			2	/* 4039: mkdir */
	sys	execute_in_lx			1	/* 4040: rmdir */
	sys	execute_in_lx			1	/* 4041: dup */
	sys	execute_in_lx			0	/* 4042: pipe */
	sys	execute_in_lx			1	/* 4043: times */
	sys	sys_rt_ni_syscall		0
	sys	execute_in_lx			1	/* 4045: brk */
	sys	sys_rt_ni_syscall		1
	sys	sys_rt_ni_syscall		0	/* 4047: getgid */
	sys	sys_rt_ni_syscall		0	/* was signal(2) */
	sys	execute_in_lx			0	/* 4049: geteuid16 */
	sys	execute_in_lx			0	/* 4050: getegid16 */
	sys	sys_rt_ni_syscall		1
	sys	execute_in_lx			2	/* 4052: umount */
	sys	sys_rt_ni_syscall		0
	sys	sys_rt_ioctl			3
	sys	execute_in_lx			3	/* 4055: fcntl */
	sys	sys_rt_ni_syscall		2
	sys	sys_rt_ni_syscall		2	/* 4057: setpgid - do not change (glibc expects ENOSYS to detect rt threads) */
	sys	sys_rt_ni_syscall		0
	sys	sys_rt_ni_syscall		1
	sys	execute_in_lx			1	/* 4060: umask */
	sys	sys_rt_ni_syscall		1
	sys	sys_rt_ni_syscall		2
	sys	execute_in_lx			2	/* 4063: dup2 */
	sys	sys_rt_ni_syscall		0
	sys	sys_rt_ni_syscall		0	/* 4065 */
	sys	sys_rt_ni_syscall		0
	sys	execute_in_lx			3	/* 4067: sigaction */
	sys	sys_rt_ni_syscall		0
	sys	sys_rt_ni_syscall		1
	sys	sys_rt_ni_syscall		2	/* 4070 */
	sys	sys_rt_ni_syscall		2
	sys	sys_rt_ni_syscall		0
	sys	sys_rt_ni_syscall		1
	sys	sys_rt_ni_syscall		2
	sys	execute_in_lx			2	/* 4075 setrtlimit */
	sys	execute_in_lx			2	/* 4076: old_getrlimit */
	sys	sys_rt_ni_syscall		2
	sys	sys_rt_gettimeofday		2
	sys	execute_in_lx			2	/* settimeofday */
	sys	sys_rt_ni_syscall		2	/* 4080 */
	sys	sys_rt_ni_syscall		2
	sys	sys_rt_ni_syscall		0	/* old_select */
	sys	sys_rt_ni_syscall		2
	sys	sys_rt_ni_syscall		0	/* was sys_lstat */
	sys	sys_rt_ni_syscall		3	/* 4085 */
	sys	sys_rt_ni_syscall		1
	sys	sys_rt_ni_syscall		2
	sys	execute_in_lx			3	/* sys_reboot */
	sys	sys_rt_ni_syscall		3
		/* nr of arguments was 6, but we don't do stack modifications */
		/* when propagating to linux                                  */
	sys	execute_in_lx			3	/* 4090 old mmap */
	sys	execute_in_lx			2	/* munmap */
	sys	execute_in_lx			2	/* truncate */
	sys	execute_in_lx			2	/* 4093: ftruncate */
	sys	sys_rt_ni_syscall		2
	sys	sys_rt_ni_syscall		3	/* 4095 */
	sys	sys_rt_ni_syscall		2
	sys	sys_rt_ni_syscall		3
	sys	sys_rt_ni_syscall		0
	sys	execute_in_lx			2	/* 4099: sys_statfs */
	sys	execute_in_lx			2	/* 4100: fstatfs */
	sys	execute_in_lx			0	/* was ioperm(2) */
	sys	execute_in_lx			2	/* 4102: socketcall */
	sys	execute_in_lx			0	/* 4103: sys_syslog */
	sys	sys_rt_ni_syscall		3
	sys	sys_rt_ni_syscall		2	/* 4105 */
	sys	execute_in_lx			2	/* 4106: newstat */
	sys	execute_in_lx			2	/* 4107: newlstat */
	sys	execute_in_lx			2	/* 4108: newfstat */
	sys	execute_in_lx			1	/* 4109: uname */
	sys	execute_in_lx			0	/* 4110 was iopl(2) */
	sys	sys_rt_ni_syscall		0
	sys	sys_rt_ni_syscall		0	/* was sys_idle() */
	sys	sys_rt_ni_syscall		0	/* was sys_vm86 */
	sys	sys_rt_ni_syscall		4
	sys	sys_rt_ni_syscall		1	/* 4115 */
	sys	sys_rt_ni_syscall		1
	sys	sys_rt_ni_syscall		6
	sys	execute_in_lx			1	/* sys_fsync */
	sys	execute_in_lx			0	/* sigreturn */
	sys	sys_rt_clone			0	/* 4120 clone */
	sys	sys_rt_ni_syscall		2
	sys	sys_rt_ni_syscall		1
	sys	sys_rt_ni_syscall		0	/* sys_modify_ldt */
	sys	sys_rt_ni_syscall		1
	sys	execute_in_lx			3	/* 4125 mprotect */
	sys	sys_rt_ni_syscall		3
	sys	sys_rt_ni_syscall		0	/* was create_module */
	sys	sys_rt_ni_syscall		5
	sys	sys_rt_ni_syscall		1
	sys	sys_rt_ni_syscall		0	/* 4130	was get_kernel_syms */
	sys	sys_rt_ni_syscall		4
	sys	sys_rt_ni_syscall		1
	sys	sys_rt_ni_syscall		1
	sys	sys_rt_ni_syscall		2
	sys	sys_rt_ni_syscall		3	/* 4135 */
	sys	sys_rt_ni_syscall		1
	sys	sys_rt_ni_syscall		0	/* for afs_syscall */
	sys	sys_rt_ni_syscall		1
	sys	sys_rt_ni_syscall		1
		/* nr of arguments was 5, but we don't do stack modifications */
		/* when propagating to linux                                  */
	sys	execute_in_lx			3	/* 4140 llseek */
	sys	execute_in_lx			3	/* 4141 getdents */
		/* nr of arguments was 5, but we don't do stack modifications */
		/* when propagating to linux                                  */
	sys	execute_in_lx			3	/* 4142 select */
	sys	sys_rt_ni_syscall		2
	sys	execute_in_lx			3	/* 4144: msync */
	sys	execute_in_lx			3	/* 4145: readv */
	sys	execute_in_lx			3	/* 4146: writev */
	sys	sys_rt_ni_syscall		3
	sys	sys_rt_ni_syscall		3
	sys	sys_rt_sysmips			4
	sys	sys_rt_ni_syscall		0	/* 4150 */
	sys	sys_rt_ni_syscall		1
	sys	execute_in_lx			1	/* 4152: fdatasync */
	sys	sys_rt_ni_syscall		1
	sys	sys_rt_ni_syscall		2
	sys	sys_rt_ni_syscall		2	/* 4155 */
	sys	execute_in_lx			1	/* 4156: mlockall */
	sys	sys_rt_ni_syscall		0
	sys	sys_rt_sched_setparam		2
	sys	sys_rt_sched_getparam		2
	sys	sys_rt_sched_setscheduler	3	/* 4160 */
	sys	sys_rt_sched_getscheduler	1
	sys	sys_rt_sched_yield		0
	sys	sys_sched_get_priority_max	1	/* sched_get_priority_max */
	sys	sys_sched_get_priority_min	1	/* sched_get_priority_min */
	sys	sys_rt_ni_syscall 		2	/* 4165 */
	sys	sys_rt_nanosleep		2
		/* nr of arguments was 4, but we don't do stack modifications */
		/* when propagating to linux                                  */
	sys	execute_in_lx,			3	/* mremap */
	sys	sys_rt_ni_syscall		3
	sys	sys_rt_ni_syscall		3	/* sys_bind */
	sys	sys_rt_ni_syscall		3	/* 4170 */
	sys	sys_rt_ni_syscall		3
	sys	sys_rt_ni_syscall		3
	sys	sys_rt_ni_syscall		5
	sys	sys_rt_ni_syscall		2
	sys	sys_rt_ni_syscall		4	/* 4175 */
	sys	sys_rt_ni_syscall		6
	sys	sys_rt_ni_syscall		3
	sys	sys_rt_ni_syscall		4
	sys	sys_rt_ni_syscall		3
	sys	sys_rt_ni_syscall		6	/* 4180 */
	sys	sys_rt_ni_syscall		5
	sys	sys_rt_ni_syscall		2
	sys	execute_in_lx			3	/* sys_socket */
	sys	sys_rt_ni_syscall		4
	sys	sys_rt_ni_syscall		3	/* 4185 */
	sys	sys_rt_ni_syscall		3
	sys	sys_rt_ni_syscall		0	/* was sys_query_module */
	sys	sys_rt_ni_syscall		3
	sys	sys_rt_ni_syscall		3
	sys	sys_rt_ni_syscall		3	/* 4190 */
	sys	sys_rt_ni_syscall		3
		/* nr of arguments was 5, but we don't do stack modifications */
		/* when propagating to linux                                  */
	sys	execute_in_lx			3	/* sys_prctl */
	sys	sys_rt_ni_syscall		0
		/* nr of arguments was 4, but we don't do stack modifications */
		/* when propagating to linux                                  */
	sys	execute_in_lx			3	/* sigaction */
	sys	sys_rt_ni_syscall		4	/* 4195 */
	sys	sys_rt_sigpending_rt		2
	sys	sys_rt_sigtimedwait_rt		4
	sys	sys_rt_ni_syscall		3
	sys	sys_rt_ni_syscall		0
	sys	sys_rt_ni_syscall		6	/* 4200 */
	sys	sys_rt_ni_syscall		6
	sys	sys_rt_ni_syscall		3
	sys	execute_in_lx			2	/* 4213: getcwd */
	sys	sys_rt_ni_syscall		2
	sys	sys_rt_ni_syscall		2	/* 4205 */
	sys	sys_rt_ni_syscall		0
	sys	sys_rt_ni_syscall		4
	sys	sys_rt_ni_syscall		0
	sys	sys_rt_ni_syscall		0
		/* nr of arguments was 6, but we don't do stack modifications */
		/* when propagating to linux                                  */
	sys	execute_in_lx			3	/* 4210 mmap2 */
	sys	sys_rt_ni_syscall		4
	sys	sys_rt_ni_syscall		4
	sys	execute_in_lx			2	/* 4213: stat64 */
	sys	execute_in_lx			2	/* 4214: lstat64 */
	sys	execute_in_lx			2	/* 4215 fstat64 */
	sys	sys_rt_ni_syscall		2
	sys	sys_rt_ni_syscall		3
	sys	sys_rt_ni_syscall		3
	sys	sys_rt_ni_syscall		3
	sys	execute_in_lx			3	/* 4220 fcntl64 */
	sys	sys_rt_ni_syscall		0
	sys	sys_rt_gettid			0
	sys	sys_rt_ni_syscall		5
	sys	sys_rt_ni_syscall		5
	sys	sys_rt_ni_syscall		5	/* 4225 */
	sys	sys_rt_ni_syscall		5
	sys	sys_rt_ni_syscall		4
	sys	sys_rt_ni_syscall		4
	sys	sys_rt_ni_syscall		4
	sys	sys_rt_ni_syscall		3	/* 4230 */
	sys	sys_rt_ni_syscall		3
	sys	sys_rt_ni_syscall		3
	sys	sys_rt_ni_syscall		2
	sys	sys_rt_ni_syscall		2
	sys	sys_rt_ni_syscall		2	/* 4235 */
	sys	sys_rt_tkill			2
	sys	sys_rt_ni_syscall		5
	sys	sys_rt_futex			6
	sys	execute_in_lx			3	/* 4239: sched_setaffinity */
	sys	execute_in_lx			3	/* 4240: sched_getaffinity */
	sys	sys_rt_ni_syscall		2
	sys	sys_rt_ni_syscall		1
	sys	sys_rt_ni_syscall		5
	sys	sys_rt_ni_syscall		3
	sys	sys_rt_ni_syscall		3	/* 4245 */
	sys	sys_rt_exit_group		1
	sys	sys_rt_ni_syscall		4
	sys	sys_rt_ni_syscall		1
	sys	sys_rt_ni_syscall		4
	sys	sys_rt_ni_syscall		3	/* 4250 */
	sys	sys_rt_ni_syscall		5
	sys	sys_rt_ni_syscall		1
	sys	sys_rt_ni_syscall		0
	sys	sys_rt_ni_syscall		7
	sys	sys_rt_ni_syscall		3	/* 4255 */
	sys	sys_rt_ni_syscall		2
	sys	sys_rt_timer_create		3
	sys	sys_rt_timer_settime		4
	sys	sys_rt_timer_gettime		2
	sys	sys_rt_ni_syscall		1	/* 4260 */
	sys	sys_rt_timer_delete		1
	sys	sys_rt_clock_settime		2
	sys	sys_rt_clock_gettime		2
	sys	sys_rt_clock_getres		2
	sys	sys_rt_clock_nanosleep		4	/* 4265 */
	sys	sys_rt_tgkill			3
	sys	sys_rt_ni_syscall		2
	sys	sys_rt_ni_syscall		4
	sys	sys_rt_ni_syscall		0	/* sys_get_mempolicy */
	sys	sys_rt_ni_syscall		0	/* 4270 sys_set_mempolicy */
		/* nr of arguments was 4, but we don't do stack modifications */
		/* when propagating to linux                                  */
	sys	execute_in_lx			3	/* mq_open */
	sys	execute_in_lx			1	/* mq_unlink */
	sys	sys_rt_mq_timedsend		5
	sys	sys_rt_mq_timedreceive		5
	sys	sys_rt_ni_syscall		2	/* 4275 */
	sys	execute_in_lx			3	/* mq_getsetattr */
	sys	sys_rt_ni_syscall		0	/* sys_vserver */
		/* nr of arguments was 5, but we don't do stack modifications */
		/* when propagating to linux                                  */
	sys	execute_in_lx			3	/* sys_waitid */
	sys	sys_rt_ni_syscall		0	/* available, was setaltroot */
	sys	sys_rt_ni_syscall		5	/* 4280 */
	sys	sys_rt_ni_syscall		4
	sys	sys_rt_ni_syscall		5
	sys	sys_rt_ni_syscall		1
	sys	sys_rt_ni_syscall		0
	sys	sys_rt_ni_syscall		3	/* 4285 */
	sys	sys_rt_ni_syscall		2
	sys	sys_rt_ni_syscall		4
	sys	sys_rt_ni_syscall		4
	sys	sys_rt_ni_syscall		3
	sys	sys_rt_ni_syscall		4	/* 4290 */
	sys	sys_rt_ni_syscall		5
	sys	sys_rt_ni_syscall		3
	sys	sys_rt_ni_syscall		4
	sys	sys_rt_ni_syscall		3
	sys	sys_rt_ni_syscall		4	/* 4295 */
	sys	sys_rt_ni_syscall		5
	sys	sys_rt_ni_syscall		3
	sys	sys_rt_ni_syscall		4
	sys	sys_rt_ni_syscall		3
	sys	sys_rt_ni_syscall		3	/* 4300 */
	sys	sys_rt_ni_syscall		6
	sys	sys_rt_ni_syscall		5
	sys	sys_rt_ni_syscall		1
	sys	sys_rt_ni_syscall		6
	sys	sys_rt_ni_syscall		7	/* 4305 */
	sys	sys_rt_ni_syscall		4
	sys	sys_rt_ni_syscall		4
	sys	sys_rt_ni_syscall		6
	sys	sys_rt_set_robust_list		2
	sys	sys_rt_ni_syscall		3	/* 4310 */
	sys	sys_rt_ni_syscall		4
	sys	sys_rt_ni_syscall		3
	sys	sys_rt_ni_syscall		6
	sys	sys_rt_ni_syscall		3
	sys	sys_rt_ni_syscall		2	/* 4315 */
	.endm

	.align	3
	.type	rt_sys_call_table,@object
EXPORT(rt_sys_call_table)
	rt_syscalltable
	.size	rt_sys_call_table, . - rt_sys_call_table
