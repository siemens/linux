/*
 * linux/aud/rt_base.h
 *
 * 2007-19-01:  Karl-Heinz.Krause@siemens.com
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
 */
#ifndef _LINUX_AUD_RT_BASE_H
#define _LINUX_AUD_RT_BASE_H

#define INLINE inline
#define _INT64 long long

#include <linux/kernel.h>
#include <asm-generic/errno-base.h>
#include <linux/list.h>
#include <linux/pid.h>
#include <asm/processor.h>
#include <linux/sched.h>
#include <asm/mmu_context.h>
#include <asm/bitops.h>
#include <linux/types.h>	
#include <linux/module.h>
#include <linux/spinlock.h>  
#include <asm/uaccess.h>
#include <linux/signal.h>
#include <asm/signal.h>
#include <linux/spinlock.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/version.h>
#include <linux/semaphore.h>
#include <linux/time.h>
#include <linux/futex.h>
#include <linux/linkage.h>
#include <linux/mm.h>
#include <linux/mqueue.h>
#include <asm/uaccess.h>
#include <asm/pgtable.h>

#ifdef CONFIG_I386
#include <asm/ldt.h>
#endif
#include <asm/processor.h>
#ifdef CONFIG_I386
#include <asm/i387.h>
#include <asm/desc.h>
#endif		
					
#include <linux/aud/rt_semaphore.h>
#include <linux/aud/rt_kernel.h>
#include <linux/aud/rt_futex.h>

DECLARE_PER_CPU(struct task_struct *, interdomain_daemon);
DECLARE_PER_CPU(int, rtx_mode);

extern int __rtx_timer_irq;
extern cpumask_t rtx_cpu_onlinemask;

/* Audis version (should match clearcase version) */
#define RT_VERSION_STRING		"V02.11"
#define RT_DEV_VERSION			0x0000020B

#if NR_CPUS <= 16
#define RT_MAX_CPUS		NR_CPUS
#else
#define RT_MAX_CPUS		16
#endif

#define RT_MAX_INT		0x7fffffff
#define MAX_PID_BUF		10			// max size for the pid buffer
#define MAX_CPU_BUF		6			// max size for the cpu buffer

struct buf_desc {
	int state;						// 0: buffer empty <0: buffer requested >0: length of data in bytes
	struct type_info *type;
	char str_pid[MAX_PID_BUF];		// pid buffer
	int len_pid;
#ifdef CONFIG_SMP
	char str_cpu[MAX_CPU_BUF];		// cpu buffer
	int len_cpu;
#endif
};

struct buf_head {
	struct buf_head *link;
	struct buf_desc desc;
	char data[1];
};

struct com_buf {
	struct buf_head *pbegin;		// points to the begin of the print buffer
	char *pbuf;						// the print buffer (dynamically allocated)
	struct buf_head *pend;			// points to the end of the print buffer
    struct semaphore com_sem;
};

/* definitions for 'slot_state' */
#define RT_PROC_SLOT_ALLOC		0x1

#define RT_PROC_R2L				1		/* RT thread voluntarily returned to LX */

/* process-specific data structures */
struct rt_proc_hdr {
	int slot_state;					/* allocated or free */
	int cpu;						/* CPU the RT process is assigned to */
	cpumask_t cpu_mask;
	atomic_t thread_cnt;
	unsigned long state;
	pid_t aud_lx_tgid;
	pid_t curr_tgid;
	int slot;

	/* write daemon specific */
	struct task_struct *write_daemon;
	char *wr_buf;
	int wr_buf_len;
	unsigned volatile msg_count;
	struct proc_wait_queue wr_req;
	struct com_buf buffer;
	struct buf_head *rpr;
	struct buf_head *old_wpr;
	struct buf_head *wpr;

	struct task_struct *main;
	struct task_struct *rt_check_daemon;
	struct futex_hash_bucket futex_queues[1<<FUTEX_HASHBITS];
	struct list_head to_lx_list;
#ifdef CONFIG_RTX_RETURN_TO_LINUX
	int r2l;					/* if set, a RT-thread returned to linux */
#endif
};

#define WRITE_DAEMON_ID			1
#define CHECK_DAEMON_ID			2

#define RT_DOMAIN				1	/* RT-process is active: hard mode */
#define LX_DOMAIN_SOFT			-1	/* RT-process is active: soft mode */
#define LX_DOMAIN_NATIVE		0	/* no RT-process is active */
  
/* printkGen() and write_daemon specific definitions */
#define BUFLEN			(CONFIG_RTX_PRINT_BUF_LEN_KB * 1024)		/* configured size of print buffer */
#define MAX_PRINT_BUFF	128			/* size of the printkGen() message buffer */
#define MAX_PREFIX		6			/* an additional message prefix */

#define LX_PRINTK_MSG	"LXK: "		/* prefix if printkGen() is mapped to printk() */
#define RT_PRINTK_MSG	"RTK: "

#define LX_WD_MSG		"LXD: "		/* prefix if printkGen() is mapped to write daemon */
#define RT_WD_MSG		"RTD: "

#define LX_OUT_MSG		"LX1: "		/* stdout: prefix if printf() is invoked by a NRT-thread */
#define LX_ERR_MSG		"LX2: "		/* stderr */

#define RT_OUT_MSG		"RT1: "		/* stdout: prefix if printf() is invoked by a RT-thread */
#define RT_ERR_MSG		"RT2: "		/* stderr */

#define WRITE_DAEMON_IS_ACTIVE(p)	((p->rt_proc != NULL) && (p->rt_proc->write_daemon != NULL))
		
#define NO_OF_FU_REQUESTS		(32 + RT_MAX_CPUS*32)
#define NO_OF_SIGQUEUE_EL		(16 + RT_MAX_CPUS*16)

typedef struct runqueue runqueue_t;

struct rq_head {
		struct list_head qhead;
		int idx;
};

struct runqueue {
	unsigned intFlag;
	unsigned long nr_running;
	unsigned long bitmapPrio;
	struct rq_head queue[32];
};

typedef struct migqueue migqueue_t;

struct migqueue {
	unsigned long bitmapPrio;
	struct rq_head queue[32];
};

struct sync_param {
	struct task_struct *source;			/* initiator, e.g. thread creator */
	struct task_struct *target;			/* target, e.g. created thread */
	struct task_struct *first_waiter;	/* source or target	*/
};

/*
 * This structure holds all the setup parameters for the realtime system
 * It gets initialized with default values, but may be overriden later via 
 * an ioctl(,AuD_DEV_RT_PARAM, )
 */
struct rt_config_par {
	int max;
	int val;
	int min;
};

typedef struct rt_config_par confpar_t;

struct rt_config_cntrl {
	confpar_t no_of_timers;
	confpar_t no_of_sigq;
	confpar_t no_of_headers;
	confpar_t no_of_fu_req;				// for delegating futex_wake requests to Linux;
	confpar_t buf_size;					// buffer size for delegated write calls
	confpar_t td_prio;					// prio of the timer daemon for RT
	confpar_t wd_prio;					// prio of the write daemon for Linux
};

#define RT_FUTEX_VERBOSE 		(aud_diagnostic & RT_FUTEX_VERBOSE_BIT)
#define RT_SIGNAL_VERBOSE	 	(aud_diagnostic & RT_SIGNAL_VERBOSE_BIT)
#define RT_START_VERBOSE		(aud_diagnostic & RT_START_VERBOSE_BIT)
#define RT_INIT_VERBOSE	 		(aud_diagnostic & RT_INIT_VERBOSE_BIT)
#define RT_SHUTDOWN_VERBOSE	 	(aud_diagnostic & RT_SHUTDOWN_VERBOSE_BIT)
#define RT_CHECK_VERBOSE	 	(aud_diagnostic & RT_CHECK_VERBOSE_BIT)
#define RT_THREAD_VERBOSE	 	(aud_diagnostic & RT_THREAD_VERBOSE_BIT)
#define RT_TIMER_VERBOSE	 	(aud_diagnostic & RT_TIMER_VERBOSE_BIT)
#define RT_EVENT_VERBOSE	 	(aud_diagnostic & RT_EVENT_VERBOSE_BIT)
#define RT_MIGRATE_VERBOSE	 	(aud_diagnostic & RT_MIGRATE_VERBOSE_BIT)
#define RT_FAULT_VERBOSE		(aud_diagnostic & RT_FAULT_VERBOSE_BIT)
#define RT_SYSCALL_IN_LX_VERBOSE	(aud_diagnostic & RT_SYSCALL_IN_LX_VERBOSE_BIT)
#define RT_SYSCALL_NOT_IMPL_VERBOSE	(aud_diagnostic & RT_SYSCALL_NOT_IMPL_VERBOSE_BIT)
#define AUD_DEV_VERBOSE			(aud_diagnostic & AUD_DEV_VERBOSE_BIT)
#define RT_EXCEPTION_VERBOSE		(aud_diagnostic & RT_EXCEPTION_VERBOSE_BIT)
#define RT_WARN_VERBOSE			(aud_diagnostic & RT_WARN_VERBOSE_BIT)
#define RT_PRINT_STDOUT			(aud_diagnostic & RT_PRINT_STDOUT_BIT)
#define RT_PRINT_STDERR			(aud_diagnostic & RT_PRINT_STDERR_BIT)
#define RT_FP_EXCEPTION_VERBOSE		(aud_diagnostic & RT_FP_EXCEPTION_VERBOSE_BIT)
#define RT_DEBUG_VERBOSE		(aud_diagnostic & RT_DEBUG_VERBOSE_BIT)
#define RT_UNALIGNMENT_VERBOSE		(aud_diagnostic & RT_UNALIGNMENT_VERBOSE_BIT)

#define TIMER_DAEMON_NAME		"rt_timerd"
#define INTERDOMAIN_DAEMON_NAME		"lx_interd"
#define CHECK_DAEMON_NAME		"rt_checkd"
#define LTT_DAEMON_NAME			"lx_lttd"
#define WRITE_DAEMON_NAME		"lx_writed"
#define CLOCK_PROXY_NAME		"lx_clockp"
#define EVENT_PROXY_NAME		"lx_eventp"

/* audis realtime: generic definitions for a rt timer */
typedef int (*aud_register)(void);
typedef int (*aud_start)(void);
typedef int (*aud_unregister)(void);

struct aud_timer_ops {
    aud_register    rt_register;    // register the timer device
    aud_start       rt_start;       // start interrupt notification
    aud_unregister  rt_unregister;  // unregister timer device
};

struct periodic_req {
	long period;
	int (*timer_action) (void *);
	void *action_par;
};

/* mode definitions */
#define RT_PERIODIC_LX_TICK		0x1		// RT activates a periodic request

struct aud_timer {
    char name[20];              // timer name
    int irq;                    // interrupt request
    unsigned long mode;			// timer mode
    unsigned long res_nsec;		// resolution in nsec
    unsigned long delta;        // compensation for a timer read 
	unsigned long gravity;		// gravity interval
    void __iomem *readp;        // timer read address
    void __iomem *writep;       // timer write address
    struct aud_timer_ops ops;   // function pointers
    struct periodic_req preq;	// parameters for periodic request
    void*	dev_id;				// device pointer for satisfying Linux request_irq
};

/*
 * ISR-safe version of copy_from_user (without might_sleep()).
 */
static inline int rt_copy_from_user(void *to, const void __user *from, unsigned long n)
{
	if (!access_ok(VERIFY_READ, from, n))
		return -1;
	return __copy_from_user_inatomic(to, from, n);
}

/*
 * ISR-safe version of copy_to_user (without might_sleep()).
 */
static inline int rt_copy_to_user(void __user *to, const void *from, unsigned long n)
{
	if (!access_ok(VERIFY_WRITE, to, n))
		return -1;
	return __copy_to_user_inatomic(to, from, n);
}

#endif /* _LINUX_AUD_RT_BASE_H */

