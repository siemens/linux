#ifdef CONFIG_RTX_DOMAIN
ENTRY(rtx_sys_call_table)
.long sys_rt_ni_syscall
.long sys_rt_exit        	/* sys_exit */
.long sys_rt_ni_syscall
.long sys_rt_read
.long sys_rt_write
.long execute_in_lx			/* 5 open */
.long execute_in_lx			/* close */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long execute_in_lx			/* 9: link */
.long execute_in_lx			/* 10: unlink */
.long sys_rt_ni_syscall
.long execute_in_lx			/* chdir */
.long sys_rt_time			/* 13: time */
.long sys_rt_ni_syscall
.long execute_in_lx			/* chmod */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long execute_in_lx			/* 19: lseek */
.long sys_rt_getpid			/* 20 */
.long execute_in_lx			/* mount */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long execute_in_lx			/* sys_stime */
.long sys_rt_ni_syscall
.long execute_in_lx			/* alarm */
.long sys_rt_ni_syscall
.long execute_in_lx			/* pause */
.long execute_in_lx			/* 30: utime */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long execute_in_lx			/* access */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long execute_in_lx			/* sync */
.long execute_in_lx			/* kill */
.long execute_in_lx			/* rename */
.long execute_in_lx			/* mkdir */
.long execute_in_lx			/* 40: rmdir */
.long execute_in_lx		    /* dup */
.long execute_in_lx			/* pipe */
.long execute_in_lx			/* times */
.long sys_rt_ni_syscall
.long execute_in_lx			/* brk */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long execute_in_lx			/* geteuid16 */
.long execute_in_lx			/* 50 getegid16 */
.long sys_rt_ni_syscall
.long execute_in_lx			/* umount */
.long sys_rt_ni_syscall
.long sys_rt_ioctl
.long execute_in_lx			/* 55: fcntl */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall     /* 57: setpgid - do not change (glibc expects ENOSYS) */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long execute_in_lx			/* 60 umask */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long execute_in_lx			/* dup2 */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long execute_in_lx			/* sigaction */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall		/* 70 */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long execute_in_lx			/* 75: setrtlimit */
.long execute_in_lx			/* 76: old_getrlimit */
.long sys_rt_ni_syscall
.long sys_rt_gettimeofday
.long execute_in_lx			/* settimeofday */
.long sys_rt_ni_syscall		/* 80 */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long execute_in_lx			/* 90: old mmap*/
.long execute_in_lx			/* munmap */
.long execute_in_lx			/* truncate */
.long execute_in_lx			/* ftruncate */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall		/* 95 */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long execute_in_lx			/* sys_statfs */
.long execute_in_lx			/* 100: fstatfs */
.long execute_in_lx			/* ioperm */
.long execute_in_lx			/* socket functions */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall		/* 105 */
.long execute_in_lx			/* newstat */
.long execute_in_lx			/* newlstat */
.long execute_in_lx			/* newfstat */
.long execute_in_lx			/* uname */
.long execute_in_lx			/* 110 iopl */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall		/* 115 */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long execute_in_lx			/* sys_fsync */
.long execute_in_lx			/* sigreturn */
.long sys_rt_clone			/* 120 clone */
.long sys_rt_ni_syscall
.long execute_in_lx			/* sys_newuname */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long execute_in_lx			/* 125: mprotect */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall		/* 130 */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long execute_in_lx			/* 140: llseek */
.long execute_in_lx			/* getdents */
.long execute_in_lx			/* select */
.long sys_rt_ni_syscall
.long execute_in_lx			/* msync */
.long execute_in_lx			/* 145: readv */
.long execute_in_lx			/* writev */
.long sys_rt_ni_syscall
.long execute_in_lx			/* fdatasync */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall		/* 150 */
.long sys_rt_ni_syscall
.long execute_in_lx			/* mlockall */
.long sys_rt_ni_syscall
.long sys_rt_sched_setparam
.long sys_rt_sched_getparam  /*155 */
.long sys_rt_sched_setscheduler
.long sys_rt_sched_getscheduler
.long sys_rt_sched_yield
.long sys_sched_get_priority_max
.long sys_sched_get_priority_min	/* 160 */
.long sys_rt_ni_syscall
.long sys_rt_nanosleep
.long execute_in_lx			/* mremap */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall		/* 165 */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall		/* 170 */
.long sys_rt_ni_syscall
.long execute_in_lx			/* prctl */
.long sys_rt_ni_syscall
.long execute_in_lx			/* rt_sigaction */
.long sys_rt_ni_syscall		/* 175 */
.long sys_rt_sigpending_rt
.long sys_rt_sigtimedwait_rt
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall		/* 180 */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long execute_in_lx			/* getcwd */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall		/* 185 */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall		/* 190 */
.long execute_in_lx			/* getrlimit */
.long execute_in_lx			/* mmap2 */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long execute_in_lx			/* 195: stat64 */
.long execute_in_lx			/* lstat64 */
.long execute_in_lx			/* fstat64 */
.long sys_rt_ni_syscall
.long sys_rt_getuid			/* getuid */
.long execute_in_lx			/* 200; getgid */
.long execute_in_lx			/* geteuid */
.long execute_in_lx			/* getegid */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall		/* 210 */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long execute_in_lx			/* sys_madvise */
.long sys_rt_ni_syscall		/* 220 */
.long execute_in_lx 		/* fcntl64 */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_gettid
.long sys_rt_ni_syscall		/* 225 */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall		/* 230 */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_tkill
.long sys_rt_ni_syscall
.long sys_rt_futex			/* 240 */
.long execute_in_lx			/* sched_setaffinity */
.long execute_in_lx			/* sched_getaffinity */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall		/* 250 */
.long sys_rt_ni_syscall
.long sys_rt_exit_group	    /* exit_group */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_timer_create
.long sys_rt_timer_settime		/* 260 */
.long sys_rt_timer_gettime
.long sys_rt_ni_syscall
.long sys_rt_timer_delete
.long sys_rt_clock_settime
.long sys_rt_clock_gettime		/* 265 */
.long sys_rt_clock_getres
.long sys_rt_clock_nanosleep
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_tgkill				/* 270 */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall			/* 275 */
.long sys_rt_ni_syscall
.long execute_in_lx				/* mq_open */
.long execute_in_lx				/* mq_unlink */
.long sys_rt_mq_timedsend
.long sys_rt_mq_timedreceive	/* 280  */
.long sys_rt_ni_syscall			/* mq_notify */
.long execute_in_lx				/* mq_getsetattr */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall			/* 285 */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall			/* 290 */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall			/* 295 */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall			/* 300 */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall			/* 305 */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall			/* 310 */
.long sys_rt_set_robust_list
#endif
