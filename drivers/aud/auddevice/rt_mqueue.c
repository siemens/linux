/*
 * drivers/aud/auddevice/rt_mqueue.c
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
 * This file contains the basic send/receive frontend functions of
 * the POSIX message queues needed for the realtime domain.
 */
#include <linux/aud/rt_timer.h>

extern struct file* rt_fcheck_file_fd(int fd, int flags);

#ifdef CONFIG_POSIX_MQUEUE
static void rt_do_wakeup(struct proc_wait_queue *wait)
{
	if (IS_REALTIME) {
		rt_wake_up_thread(wait->task);		
	 	rt_schedule(0);
	}
	else {
		lx_wake_up_rt_thread(wait->task);
	}
}

/*
 * rt_to_jiffies
 * Compute RT jiffies from timespec.
 */
static MQ_TIMEOUT_TYPE rt_to_jiffies(struct timespec *ts)
{
	MQ_TIMEOUT_TYPE tval = 0;

	convert_to_internal(&tval, ts);
	return tval;
}

/*
 * This function is called under lock and left under lock.
 */
static MQ_TIMEOUT_TYPE rt_mq_sleep(struct mqueue_inode_info *info, 
		struct list_head *wait_list, MQ_TIMEOUT_TYPE timeout)
{
	int flag;
	MQ_TIMEOUT_TYPE time;
	struct proc_wait_queue wait;

	flag = 0;
	time = 0;

	wait.do_wakeup = rt_do_wakeup;
	wait.task = current;
	list_add_tail(&wait.list, wait_list);	
	__set_current_rt_state(RT_TASK_UNINTERRUPTIBLE);
	spin_unlock_mq(&info->lock);				// see linux/mqueue.h

	if (timeout == MAX_SCHEDULE_TIMEOUT)
	{
		rt_schedule(0);
		spin_lock_mq(&info->lock);
		if (!list_empty(&wait.list)) {
			// We got woken up by sig (debugging).
			list_del(&wait.list);
			return -ERESTARTNOINTR;
		}
		return timeout;
	}

	time = rt_schedule_timeout(&__raw_get_cpu_var(isr_list_hdr), timeout);
	spin_lock_mq(&info->lock);

	if (!list_empty(&wait.list)) {
		list_del(&wait.list);
		// We got woken up by time.
		if (time == 0) {
			return  -ETIMEDOUT;
		}
		// We got woken up by sig (debugging).
		return -ERESTARTNOINTR;		
	}
	return time;
}
#endif /* CONFIG_POSIX_MQUEUE */

asmlinkage long sys_rt_mq_timedsend(mqd_t mqdes, const char __user *u_msg_ptr,
	size_t msg_len, unsigned int msg_prio,
	const struct timespec __user *u_abs_timeout)
{
#ifdef CONFIG_POSIX_MQUEUE
	int ret;
	struct file *filp;
	struct timespec ts, *p = NULL;
	int rel_flag = 0;
	unsigned long ptmp;

	if (u_abs_timeout) {
		/* 
		 * The glibc gives a hint for relative times: 
		 * Bit 0 of the pointer is set in case of a relative time.
		 */
		if ((rel_flag = (unsigned long)u_abs_timeout & 1)) {
			ptmp = (unsigned long)u_abs_timeout;
			ptmp &= ~1;
			u_abs_timeout = (const struct timespec __user *)ptmp;
		}
		if (rt_copy_from_user(&ts, u_abs_timeout, sizeof(struct timespec)))
			return -EFAULT;
		p = &ts;
	}

	filp = rt_fcheck_file_fd(mqdes, 0);
	if (unlikely(!filp)) {
		ret = -EBADF;
		goto out;
	}
	ret = timedsend_common(filp, u_msg_ptr, msg_len, msg_prio, 
			       p, rt_mq_sleep, rt_to_jiffies, rel_flag);
out:
	return ret;
#else
	if (RT_FAULT_VERBOSE || RT_SYSCALL_NOT_IMPL_VERBOSE)
		printkGen(NULL, "%s: syscall not configured\n", __func__);
	return -ENOSYS;
#endif
}

asmlinkage ssize_t sys_rt_mq_timedreceive(mqd_t mqdes, char __user *u_msg_ptr,
	size_t msg_len, unsigned int __user *u_msg_prio,
	const struct timespec __user *u_abs_timeout) 
{
#ifdef CONFIG_POSIX_MQUEUE
	ssize_t ret;
	struct file *filp;
	struct timespec ts, *p = NULL;
	int rel_flag = 0;
	unsigned long ptmp;

	if (u_abs_timeout) {
		/* 
		 * The glibc gives a hint for relative times: 
		 * Bit 0 of the pointer is set in case of a relative time.
		 */
		if ((rel_flag = (unsigned long)u_abs_timeout & 1)) {
			ptmp = (unsigned long)u_abs_timeout;
			ptmp &= ~1;
			u_abs_timeout = (const struct timespec __user *)ptmp;
		}
		if (rt_copy_from_user(&ts, u_abs_timeout, sizeof(struct timespec)))
			return -EFAULT;
		p = &ts;
	}

	filp = rt_fcheck_file_fd(mqdes, 0);
	if (unlikely(!filp)) {
		ret = -EBADF;
		goto out;
	}
	ret = timedreceive_common(filp, u_msg_ptr, msg_len, u_msg_prio, 
			       p, rt_mq_sleep, rt_to_jiffies, rel_flag);
out:
	return ret;
#else
	if (RT_FAULT_VERBOSE || RT_SYSCALL_NOT_IMPL_VERBOSE)
		printkGen(NULL, "%s: syscall not configured\n", __func__);
	return -ENOSYS;
#endif
}
