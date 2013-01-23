/* Copyright (C) 2003 Krzysztof Benedyczak & Michal Wronski

   This program is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   It is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with this software; if not, write to the Free
   Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
   02111-1307 USA.  */

#ifndef _LINUX_MQUEUE_H
#define _LINUX_MQUEUE_H

#ifdef CONFIG_RTX_DOMAIN
#include <asm/siginfo.h>

#define MQ_TIMEOUT_TYPE		long long
#define MQ_PRIO_MAX 		32

#define MQ_OCCUPIED	0
#define MQ_FREE		1
#else
#define MQ_TIMEOUT_TYPE		long
#define MQ_PRIO_MAX 		32768
#endif

/* per-uid limit of kernel memory used by mqueue, in bytes */
#define MQ_BYTES_MAX	819200

struct mq_attr {
	long	mq_flags;	/* message queue flags			*/
	long	mq_maxmsg;	/* maximum number of messages		*/
	long	mq_msgsize;	/* maximum message size			*/
	long	mq_curmsgs;	/* number of messages currently queued	*/
#ifndef CONFIG_RTX_DOMAIN
	long	__reserved[4];	/* ignored for input, zeroed for output */
#endif
};

#ifdef CONFIG_RTX_DOMAIN
/*
 * The spin_(un)lock_mq() functions are used to protect common code which may
 * be called in the LX-domain and in the RT-domain.
 * Therefore these functions must not call the LX macros local_irq_enable()
 * and local_irq_disable().
 */
#ifndef CONFIG_SMP
#define spin_lock_mq(l)			do { local_irq_disable_hw(); } while (0)
#define spin_unlock_mq(l)		do { local_irq_enable_hw(); } while (0)
#else
extern void __lockfunc _spin_lock_irq_hw(rtx_spinlock_t_rtp *lock);
extern void __lockfunc _spin_unlock_irq_hw(rtx_spinlock_t_rtp *lock);

#define spin_lock_mq(l)			do { _spin_lock_irq_hw(l); } while (0)
#define spin_unlock_mq(l)		do { _spin_unlock_irq_hw(l); } while (0)
#endif
#else
#define spin_lock_mq(l)			do { spin_lock(l); } while (0)
#define spin_unlock_mq(l)		do { spin_unlock(l); } while (0)
#endif

#ifdef CONFIG_RTX_DOMAIN
struct mq_list {
	struct list_head list;
	size_t msgsize;		/* size of message (chars)		*/
	unsigned int msgprio; /* message priority (shortcut transfer) */
	char msg[1];		/* -- the message --			*/
};

struct proc_wait_queue {	/* queue of sleeping tasks 		*/
	struct list_head list;
	struct task_struct *task;
	void (*do_wakeup) (struct proc_wait_queue *wait);
};

struct mqueue_inode_info {
	unsigned long bitmap;
	struct list_head send_wait;
	struct list_head recv_wait;
	struct list_head free_list;
	struct list_head queues[MQ_PRIO_MAX];

	rtx_spinlock_t_rtp lock;
	struct inode vfs_inode;
	wait_queue_head_t wait_q;

	struct mq_attr attr;

	struct sigevent notify;
	struct pid *notify_owner;
	struct user_struct *user;	/* user who created, for accounting */
	struct sock *notify_sock;
	struct sk_buff *notify_cookie;
};

int timedsend_common(struct file *filp, const char __user *u_msg_ptr, size_t msg_len, unsigned int msg_prio,
	struct timespec *p_abs_timeout, MQ_TIMEOUT_TYPE (* sleep) (struct mqueue_inode_info *info, struct list_head *wait_list,
	MQ_TIMEOUT_TYPE timeout), MQ_TIMEOUT_TYPE (* to_jiffies) (struct timespec *ts), int rel_flag);

ssize_t timedreceive_common(struct file *filp, char __user *u_msg_ptr, size_t msg_len, unsigned int __user *u_msg_prio,
	struct timespec *p_abs_timeout, MQ_TIMEOUT_TYPE (* sleep) (struct mqueue_inode_info *info, struct list_head *wait_list,
	MQ_TIMEOUT_TYPE timeout), MQ_TIMEOUT_TYPE (* to_jiffies) (struct timespec *ts), int rel_flag);
#endif

/*
 * SIGEV_THREAD implementation:
 * SIGEV_THREAD must be implemented in user space. If SIGEV_THREAD is passed
 * to mq_notify, then
 * - sigev_signo must be the file descriptor of an AF_NETLINK socket. It's not
 *   necessary that the socket is bound.
 * - sigev_value.sival_ptr must point to a cookie that is NOTIFY_COOKIE_LEN
 *   bytes long.
 * If the notification is triggered, then the cookie is sent to the netlink
 * socket. The last byte of the cookie is replaced with the NOTIFY_?? codes:
 * NOTIFY_WOKENUP if the notification got triggered, NOTIFY_REMOVED if it was
 * removed, either due to a close() on the message queue fd or due to a
 * mq_notify() that removed the notification.
 */
#define NOTIFY_NONE	0
#define NOTIFY_WOKENUP	1
#define NOTIFY_REMOVED	2

#define NOTIFY_COOKIE_LEN	32

#endif
