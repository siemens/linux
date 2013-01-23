#ifdef CONFIG_RTX_DOMAIN
.align 4

_GLOBAL(rtx_sys_call_table)
.long sys_rt_ni_syscall
.long sys_rt_exit        	/* sys_exit */
.long execute_in_lx             /* ppc_fork */
.long sys_rt_read
.long sys_rt_write
.long execute_in_lx			/* 5 open */
.long execute_in_lx			/* close */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long execute_in_lx			/* 9: link */
.long execute_in_lx			/* 10: unlink */
.long sys_rt_ni_syscall
.long execute_in_lx			/* 12: chdir */
.long execute_in_lx			/* 13: time */
.long sys_rt_ni_syscall
.long execute_in_lx			/* 15: chmod */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long execute_in_lx			/* 19: lseek */
.long sys_rt_getpid			/* 20 */
.long execute_in_lx			/* 21: mount */
.long execute_in_lx			/* 22: mount */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall			/* 24 getuid (sys_rt_getuid) */
.long execute_in_lx			/* 25 sys_stime */
.long sys_rt_ni_syscall
.long execute_in_lx			/* 27: alarm */
.long sys_rt_ni_syscall
.long execute_in_lx             /* 29: pause */
.long execute_in_lx		/* 30: utime */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long execute_in_lx			/* 33: access */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long execute_in_lx 		/* 36: sys_sync */
.long execute_in_lx			/* kill */
.long execute_in_lx			/* 38: rename */
.long execute_in_lx			/* 39: mkdir */
.long execute_in_lx		/* 40: rmdir */
.long execute_in_lx		    /* dup */
.long execute_in_lx			/* pipe */
.long execute_in_lx			/* 43: times */
.long sys_rt_ni_syscall
.long execute_in_lx			/* 45 brk */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall			/* 47: getgid */
.long sys_rt_ni_syscall
.long execute_in_lx			/* 49: geteuid */
.long execute_in_lx			/* 50 getegid */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ioctl
.long execute_in_lx			/* 55: fcntl */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall		/* 57: setpgid - do not change (glibc expects ENOSYS to detect rt threads) */
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
.long execute_in_lx			/* 82: select */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long execute_in_lx			/* 90: old mmap */
.long execute_in_lx			/* munmap */
.long execute_in_lx			/* truncate */
.long execute_in_lx			/* 93: ftruncate */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long execute_in_lx			/* 99: sys_statfs */ /* ppc ni_syscall ? */
.long execute_in_lx		/* 100:  fstatfs */
.long execute_in_lx			/* ioperm */
.long execute_in_lx			/* 102:  socket functions */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long execute_in_lx			/* 106: newstat */
.long execute_in_lx			/* 107: newlstat */
.long execute_in_lx			/* 108: newfstat */
.long sys_rt_ni_syscall
.long execute_in_lx			/* 110 iopl */ /* ppc ni_syscall ? */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long execute_in_lx			/* sys_fsync */
.long execute_in_lx			/* sigreturn */
.long sys_rt_clone			/* 120 ppc_clone */
.long sys_rt_ni_syscall
.long execute_in_lx			/* 122: uname */
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
.long execute_in_lx			/* 141: getdents */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long execute_in_lx			/* 144: msync */
.long execute_in_lx			/* 145: readv */
.long execute_in_lx			/* 146: writev */
.long sys_rt_ni_syscall
.long execute_in_lx			/* 148: fdatasync */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall		/* 150 */
.long sys_rt_ni_syscall
.long execute_in_lx			/* 152: mlockall */
.long sys_rt_ni_syscall
.long sys_rt_sched_setparam	 
.long sys_rt_sched_getparam  /*155 */
.long sys_rt_sched_setscheduler 
.long sys_rt_sched_getscheduler 
.long sys_rt_sched_yield 
.long sys_sched_get_priority_max	/* sched_get_priority_max */
.long sys_sched_get_priority_min	/* 160 sched_get_priority_min */
.long sys_rt_ni_syscall
.long sys_rt_nanosleep
.long execute_in_lx			/* mremap */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall		/* 165 */
.long sys_rt_ni_syscall         /* watch: here starts the difference to x86 table sys_vm86 ! */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall		/* 170 */
.long execute_in_lx     	/* sys_prctl */ 
.long sys_rt_ni_syscall         /* sys_rt_sigreturn */
.long execute_in_lx			/* sigaction */
.long sys_rt_ni_syscall		/* 174 */
.long sys_rt_sigpending_rt      /* 175 */
.long sys_rt_sigtimedwait_rt
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long execute_in_lx		/* sys_pread64 */
.long execute_in_lx		/* 180 */ /* sys_pwrite64 */
.long sys_rt_ni_syscall
.long execute_in_lx			/* 182: getcwd */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall		/* 185 */  /* sys_sigaltstack */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall         /* ppc_vfork */
.long execute_in_lx			/* 190: getrlimit */
.long sys_rt_ni_syscall         /* sys_readahead */
.long execute_in_lx			/* mmap2 */ /* now the sequence is identical to x86 */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long execute_in_lx			/* 195: stat64 */
.long execute_in_lx			/* 196: lstat64 */
.long execute_in_lx			/* fstat64 */
.long sys_rt_ni_syscall			/* sys_pciconfig_read */
.long sys_rt_ni_syscall			/* sys_pciconfig_write */
.long sys_rt_ni_syscall			/* sys_pciconfig_iobase  */	/* 200 */
.long sys_rt_ni_syscall			/* 201 - reserved - MacOnLinux - new */
.long sys_rt_ni_syscall			/* sys_getdents64 */
.long sys_rt_ni_syscall			/* sys_pivot_root */
.long execute_in_lx 		/* 204 fcntl64 */
.long sys_rt_ni_syscall			/* sys_madvise */	/* 205 */
.long sys_rt_ni_syscall			/* sys_mincore */
.long sys_rt_gettid			/* 207 */
.long sys_rt_tkill                      /* 208 */
.long sys_rt_ni_syscall			/* sys_setxattr */
.long sys_rt_ni_syscall			/* sys_lsetxattr */	/* 210 */
.long sys_rt_ni_syscall			/* sys_fsetxattr */
.long sys_rt_ni_syscall			/* sys_getxattr */
.long sys_rt_ni_syscall			/* sys_lgetxattr */
.long sys_rt_ni_syscall			/* sys_fgetxattr */
.long sys_rt_ni_syscall			/* sys_listxattr */	/* 215 */
.long sys_rt_ni_syscall			/* sys_llistxattr */
.long sys_rt_ni_syscall			/* sys_flistxattr */
.long sys_rt_ni_syscall			/* sys_removexattr */
.long sys_rt_ni_syscall			/* sys_lremovexattr */
.long sys_rt_ni_syscall			/* sys_fremovexattr */	/* 220 */
.long sys_rt_futex			/* 221 */
.long sys_rt_ni_syscall			/* sys_sched_setaffinity */
.long sys_rt_ni_syscall			/* sys_sched_getaffinity */
.long sys_rt_ni_syscall			
.long sys_rt_ni_syscall			/* 225 - reserved for Tux */
.long sys_rt_ni_syscall			/* sys_sendfile64 */
.long sys_rt_ni_syscall			/* sys_io_setup */
.long sys_rt_ni_syscall			/* sys_io_destroy */
.long sys_rt_ni_syscall			/* sys_io_getevents */
.long sys_rt_ni_syscall			/* sys_io_submit	/* 230 */
.long sys_rt_ni_syscall			/* sys_io_cancel */
.long sys_rt_ni_syscall			/* sys_set_tid_address */
.long sys_rt_ni_syscall			/* sys_fadvise64 */
.long sys_rt_exit_group	    /* 234 exit_group */
.long sys_rt_ni_syscall			/* sys_lookup_dcookie /* 235 */
.long sys_rt_ni_syscall			/* sys_epoll_create */
.long sys_rt_ni_syscall			/* sys_epoll_ctl */
.long sys_rt_ni_syscall			/* sys_epoll_wait */
.long sys_rt_ni_syscall			/* sys_remap_file_pages */
.long sys_rt_timer_create        /* 240 */
.long sys_rt_timer_settime	 /* 241 */
.long sys_rt_timer_gettime       /* 242 */
.long sys_rt_ni_syscall          /* sys_timer_getoverrun */
.long sys_rt_timer_delete
.long sys_rt_clock_settime       /* 245 */
.long sys_rt_clock_gettime	 /* 246 */
.long sys_rt_clock_getres        /* 247 */
.long sys_rt_ni_syscall          /* sys_clock_nanosleep */
.long sys_rt_ni_syscall			/* ppc_swapcontext */
.long sys_rt_tgkill     	/* 250 */
.long sys_rt_ni_syscall			/* sys_utimes */
.long sys_rt_ni_syscall			/* sys_statfs64 */
.long sys_rt_ni_syscall			/* sys_fstatfs64 */
.long sys_rt_ni_syscall			/* ppc_fadvise64_64 */
.long sys_rt_ni_syscall			/* 255 - rtas (used on ppc64) */
.long sys_rt_ni_syscall			/*  sys_debug_setcontext */
.long sys_rt_ni_syscall			/* 257 reserved for vserver */
.long sys_rt_ni_syscall			/* 258 reserved for new sys_remap_file_pages */
.long sys_rt_ni_syscall			/* 259 reserved for new sys_mbind */
.long sys_rt_ni_syscall			/* 260 reserved for new sys_get_mempolicy */
.long sys_rt_ni_syscall			/* 261 reserved for new sys_set_mempolicy */
.long execute_in_lx			/* mq_open */
.long execute_in_lx			/* mq_unlink */
.long sys_rt_mq_timedsend
.long sys_rt_mq_timedreceive		/* 265  */
.long sys_rt_ni_syscall			/* mq_notify */
.long execute_in_lx			/* mq_getsetattr */
.long sys_rt_ni_syscall			/* sys_kexec_load */
.long sys_rt_ni_syscall			/* sys_add_key */
.long sys_rt_ni_syscall			/* sys_request_key */		/* 270 */
.long sys_rt_ni_syscall			/* sys_keyctl */
.long sys_rt_ni_syscall			/* sys_waitid */
.long sys_rt_ni_syscall			/* sys_ioprio_set */
.long sys_rt_ni_syscall			/* sys_ioprio_get */
.long sys_rt_ni_syscall			/* sys_inotify_init	 */	/* 275 */
.long sys_rt_ni_syscall			/* sys_inotify_add_watch */
.long sys_rt_ni_syscall			/* sys_inotify_rm_watch */
#ifdef CONFIG_AUD_SPIN_LOCK_WORKAROUND
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall			/* 280 */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
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
.long sys_rt_set_robust_list		/* 300 */
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
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall			/* 315 */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall			/* 320 */
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall
.long ppc_aud_spinlock_rt_cli_sti
.long sys_rt_ni_syscall
.long sys_rt_ni_syscall			/* 325 */

#endif

#endif
