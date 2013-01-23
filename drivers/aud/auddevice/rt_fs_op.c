/*
 * drivers/aud/auddevice/rt_fs_op.c
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
 * This file contains the routines for handling the file system operations allowed in the realtime domain.
 * This include the basic entry routines for read(), write() and ioctl() and the write buffer management for fd 1 and 2
 * (stdout, stderr). The write() calls in the realtime domain fill the buffer, the corresponding routines for emptying the 
 * buffer are executed in the Linux domain
 */
#include <linux/syscalls.h>             // because of sys_write()
#include <linux/version.h>
#include <linux/fdtable.h>
#include <linux/aud/rt_timer.h>

#define BUF_OVERRUN				0x4000

DECLARE_PER_CPU(int, rt_dev_open_cnt);

#ifndef CONFIG_RTX_DISABLE_WRITE_DAEMON
static struct type_info stdout = {1,RT_OUT_MSG, 5};
static struct type_info stderr = {2,RT_ERR_MSG, 5};
static char ov_prefix[] = {63,32};	         // "? "
#endif

#define RT_IO_ENTRIES			(12 * RT_MAX_CPUS)

struct rt_io_acc {
    int allowed_calls;
    struct file *filp;
#ifndef RTX_OPTIMIZE_FOR_RELEASE
    pid_t tgid;
#endif
};

struct rt_io_acc_list {
	struct semaphore sem;
	struct rt_io_acc acc_entries[RT_IO_ENTRIES];
};
 
static struct rt_io_acc_list io_acc;

void init_io_management(void)
{
	int i;
	
	sema_init(&io_acc.sem, 1); 
	for (i = 0; i < RT_IO_ENTRIES; i++)
	{
		io_acc.acc_entries[i].filp = NULL;
		io_acc.acc_entries[i].allowed_calls = 0;
#ifndef RTX_OPTIMIZE_FOR_RELEASE
		io_acc.acc_entries[i].tgid = 0;
#endif
	}
}

/* 
 * This routine is executed from the realtime domain.
 * Currently it is assumed that  for the realtime process the fdtable doesn't need to be relocated.
 * The RCU lock free stuff may get inserted later 
 * as long as all the other routines maintain the right sequence between changing .filp and 
 * .allowed_calls there should be no race problems
 */
struct file* rt_fcheck_files(struct file *filp, int flags)
{
	int i;

	for (i = 0; i < RT_IO_ENTRIES; i++)	
		if ((io_acc.acc_entries[i].filp == filp) && (io_acc.acc_entries[i].allowed_calls & flags))
			return(filp);
	return (NULL);
}

struct file* rt_fcheck_file_fd(unsigned int fd, int flags)
{
	struct file *filp;	
	struct files_struct *fls = current->files;
	struct fdtable *fdt; 	

	do {	
		fdt = files_fdtable(fls);
		if (fd >= fdt->max_fds)						// check for invalid fd
			return NULL;							
		filp = fdt->fd[fd];
	} while (fdt != files_fdtable(fls)); 				// to circumvent the RCU stuff
	if (flags)
		return rt_fcheck_files(filp, flags);
	return filp;
}

#ifndef CONFIG_RTX_DISABLE_WRITE_DAEMON
/*
 * For realtime capable writes to stdout and stderr in the RT domain these writes are buffered and
 * the low priority LX write daemon reads the buffer and writes out. During shutdown another thread may
 * flush out the messages.
 */
int lx_write_daemon_thread(void *arg)
{
    struct sched_param sched_par;
    struct rt_proc_hdr *pproc;
	
    strcpy(current->comm, WRITE_DAEMON_NAME);
    sched_par.sched_priority = aud_config.wd_prio.val;	
	current->rt_state3 |= RT_TASK_ALLOW_SETSCHEDULER;
    sched_setscheduler(current, SCHED_FIFO, &sched_par);		
    current->rt_state = LX_DAEMON_STATE;
    pproc = current->rt_proc = get_rt_proc(WRITE_DAEMON_ID);
    if (RT_INIT_VERBOSE)
        printkGen("(wd)", "write daemon installed prio=%d:%d\n", current->rt_priority, current->prio);
	while (!(rtx_get_cpu_var(rt_system_state) & RT_SHUTDOWN))
    {
		set_current_state(TASK_UNINTERRUPTIBLE);   	
		if (pproc->msg_count > 0)
		{
			set_current_state(TASK_RUNNING);			
			write_out_buffer();
			continue;
		}
		schedule();
		/* Necessary for debugging. */	
 		clear_tsk_thread_flag(current, TIF_SIGPENDING);
   }
	sched_par.sched_priority = 0;
	current->rt_state3 |= RT_TASK_ALLOW_SETSCHEDULER;
	sched_setscheduler(current, SCHED_NORMAL, &sched_par);

    // Waiting for the timer and the interdomain daemon to be shut down. 
    while (rtx_get_cpu_var(rt_timer_daemon) != NULL || rtx_get_cpu_var(interdomain_daemon)) {
        set_current_state(TASK_UNINTERRUPTIBLE);
        schedule_timeout(SLOW_POLL_TIME);
    }
      if (RT_SHUTDOWN_VERBOSE)
  		printkGen("(wd)", "write daemon shutting down rt_system_state=%#x\n", (int)rtx_get_cpu_var(rt_system_state));

	// Flush the remaining pending messages.
	clear_tsk_thread_flag(current, TIF_SIGPENDING);
	while (pproc->msg_count > 0)
		write_out_buffer();

	pproc->write_daemon = NULL;
	// Free print buffer memory
	KFREE((char *)pproc->buffer.pbuf);
    return 0;
}

static int print_buf_len = BUFLEN;		// default 16 KB

/*
* The buffer size (in KB)  may be specified by the
* boot parameter "audis_print_buf_len" which overwrites
* the configured value (RTX_PRINT_BUF_LEN_KB).
*/
static int __init print_buf_setup(char *str)
{
	if (!str)
		return 0;
	// The size is specified in KB.
	print_buf_len = simple_strtoul(str, &str, 0) * 1024;
	return 1;
}

__setup("audis_print_buf_len=", print_buf_setup);

/*
* Initialize the print service for the current RT process.
*/
int init_write_service(struct probe_descr *wbuf)
{
	pid_t pid;
	struct rt_proc_hdr *pproc = current->rt_proc;

	pproc->msg_count = 0;
	// Should have at least the size of the user buffer.
	if (print_buf_len < wbuf->write_len) {
		printk(KERN_ALERT "%s: invalid print buffer size=%d (requested size=%d at least)\n", __func__, print_buf_len, wbuf->write_len);
		return -1;
	}
	memset(&pproc->buffer, 0, sizeof(pproc->buffer));
	if ((pproc->buffer.pbuf = (char *)KMALLOC(print_buf_len, GFP_KERNEL)) == NULL) {
		printk(KERN_ALERT "%s: no memory available for print buffer (size=%d)\n", __func__, print_buf_len);
		return -1;
	}
	memset(pproc->buffer.pbuf, 0, print_buf_len);
    sema_init(&pproc->buffer.com_sem, 1);

	if (wbuf->write_len < 80)
		return -1;

	// Initialize the user buffer with zeroes.
	if (copy_to_user(wbuf->write_buff, pproc->buffer.pbuf, wbuf->write_len))
		return -1;
	pproc->buffer.pbegin = (struct buf_head *)pproc->buffer.pbuf;
	pproc->buffer.pbegin->link = (struct buf_head *)pproc->buffer.pbuf;
	pproc->buffer.pend = (struct buf_head *)(pproc->buffer.pbuf + (print_buf_len - ((sizeof(struct buf_head) + 3) & ~0x3)));
	pproc->buffer.pend->link = (struct buf_head *)pproc->buffer.pbuf;
	pproc->wpr = (struct buf_head *)pproc->buffer.pbuf;

	pproc->old_wpr = pproc->wpr;
	pproc->rpr = pproc->wpr;
	pproc->wr_buf = wbuf->write_buff;
	pproc->wr_buf_len = wbuf->write_len;
	if ((pid = kernel_thread(lx_write_daemon_thread, 0, CLONE_KERNEL | CLONE_THREAD))<= 0)
		return -1;
	while(!rt_is_running(WRITE_DAEMON_ID))
	{
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(SLOW_POLL_TIME);
	}
	INIT_LIST_HEAD(&pproc->wr_req.list);
	pproc->wr_req.task = pproc->write_daemon;
	return 0;
}

struct buf_head *get_write_buffer(int wrlen)
{
	struct buf_head *req_wpr = NULL;
	int len;
	unsigned long flags;
	struct rt_proc_hdr *pproc = current->rt_proc;

	len = (wrlen + sizeof(struct buf_head) + 3) & ~0x3;   // size rounded to the next 32-Bit boundary
	if (len >= print_buf_len/2)			// a pragmatic check to allow for a simple buffer management
		return(NULL);
	local_irq_save_hw(flags); 
	if (pproc->rpr <= pproc->wpr)
	{
		// Wrap around of free area.
		if (((int)pproc->buffer.pend - (int)pproc->wpr) >= (len + (int)sizeof(struct buf_head)))
			goto out_ok;
		if (((int)pproc->rpr - (int)pproc->buffer.pbegin) >= (len + (int)sizeof(struct buf_head)))
		{
			pproc->wpr->link = pproc->buffer.pbegin;
			pproc->wpr = pproc->buffer.pbegin;
			goto out_ok;
		}
		goto out_not_ok;
	}
	else
		if (((int)pproc->rpr - (int)pproc->wpr) < (len + (int)sizeof(struct buf_head))) {
			goto out_not_ok;
		}
out_ok:
	pproc->old_wpr = pproc->wpr;
	req_wpr = pproc->wpr;
	req_wpr->desc.state = -1;
	pproc->wpr = (struct buf_head *)((int)pproc->wpr + len);
	req_wpr->link = pproc->wpr;
	pproc->wpr->link = pproc->rpr;				// remaining free area element
	pproc->wpr->desc.state = 0;
    local_irq_restore_hw(flags);
	return(req_wpr);
out_not_ok:	
	pproc->old_wpr->desc.state |= BUF_OVERRUN;
    local_irq_restore_hw(flags);
	return(req_wpr);
};

/* 
 *  Reads an element of the comm buffer, copies the prefix string into the user buffer
 *  and writes out the user buffer with a sys_write call.
 *  The read pointer rpr needs no locking, since there is only one instance which modifies it
 *  If redirected to a file the library buffer may be larger than the printf buffer. 
 */
void write_out_buffer(void)
{	
	char *dptr, *addr;
	int retval = 0; 
	unsigned long flags;
	int pref_len, msg_len, len, wr_len;
	int flag_len = 0;
	struct rt_proc_hdr *pproc = current->rt_proc;

    down(&pproc->buffer.com_sem);
	while (pproc->rpr->desc.state <= 0) {
		if ((pproc->rpr->link == pproc->buffer.pbegin) && (pproc->rpr != pproc->buffer.pbegin))
		{
			pproc->rpr = pproc->buffer.pbegin;
			continue;
		}
        up(&pproc->buffer.com_sem);
		return;
	}

	msg_len = pproc->rpr->desc.state;
	addr = pproc->wr_buf;
	len = pproc->wr_buf_len;
	if (msg_len & BUF_OVERRUN)
	{
		msg_len &= ~BUF_OVERRUN;
		retval = copy_to_user(addr, &ov_prefix, 2);	
		addr += 2;
		len -= 2;
		flag_len = 2;
	}
	pref_len = pproc->rpr->desc.type->pref_len + pproc->rpr->desc.len_pid;
#ifdef CONFIG_SMP
	pref_len += pproc->rpr->desc.len_cpu;
#endif
	wr_len = 	pproc->rpr->desc.type->pref_len;
	dptr = pproc->rpr->desc.type->prefix;
	retval |= copy_to_user(addr, dptr, wr_len);
	addr += wr_len;
	len -= wr_len;

	/* Copy pid info. */
	wr_len = pproc->rpr->desc.len_pid;
	dptr = &pproc->rpr->desc.str_pid[0];
	retval |= copy_to_user(addr, dptr, wr_len);
	addr += wr_len;
	len -= wr_len;

#ifdef CONFIG_SMP
	/* Copy CPU info. */
	wr_len = pproc->rpr->desc.len_cpu;
	dptr = &pproc->rpr->desc.str_cpu[0];
	retval |= copy_to_user(addr, dptr, wr_len);
	addr += wr_len;
	len -= wr_len;
#endif
	dptr = &pproc->rpr->data[0];

	do{
		wr_len = len;		
		if (msg_len <= len)
			wr_len = msg_len;										// do it in parts
		retval |= copy_to_user(addr, dptr, wr_len);
		sys_write(pproc->rpr->desc.type->fd, pproc->wr_buf, wr_len + pref_len + flag_len);
		dptr += wr_len;
		msg_len -= wr_len;
		addr = pproc->wr_buf;
		len = pproc->wr_buf_len;
		pref_len = 0;
		flag_len = 0;
	} while(msg_len);
	local_irq_save_hw(flags); 
	pproc->rpr =  pproc->rpr->link;
	pproc->wpr->link = pproc->rpr;
	pproc->msg_count--;
    local_irq_restore_hw(flags);
	if (retval)
		printk("pid %5d:      write_out_buffer failed\n", current->pid);
    up(&pproc->buffer.com_sem);
    return;
}

int rt_write_buffered(int flag, struct type_info *info, void *buff, unsigned len)
{
	struct buf_head *wpr;
	char *dptr;
	struct rt_proc_hdr *pproc = current->rt_proc;

#ifdef CONFIG_ARCH_MAP1_DIRECT_PRINTF_IN_RT
	int ii;
	for (ii = 0; ii < len; ii++) {
		serial_out_char_virt_ng(*((char*)buff + ii));
	}
	serial_out_char_virt_ng(0xD); //we need a carriage return here
	return(len);
#endif

	wpr = get_write_buffer(len);		
	if (wpr == NULL) {
		return len;	
	}
	dptr = &wpr->data[0];
	wpr->desc.type = info;	
	wpr->desc.len_pid = sprintf(&wpr->desc.str_pid[0], "P%5d: ", current->pid);
#ifdef CONFIG_SMP
	wpr->desc.len_cpu = sprintf(&wpr->desc.str_cpu[0], "C%2d: ", rtx_processor_id());
#endif
	if (flag == 1)
	{
		if (rt_copy_from_user(dptr, (void __user *)buff, len))
			return -EFAULT;
	}
	else
		strcpy(dptr, buff);
	local_irq_disable_hw();
	wpr->desc.state = len;								//  buffer is marked as filled	
	pproc->msg_count++;
	local_irq_enable_hw();
	if (pproc->msg_count == 1) {
		if (IS_REALTIME)
			__rtx_wake_lx_handling(&pproc->wr_req);
		else
			wake_up_process(pproc->write_daemon);
	}
	return len;
}
#endif /* CONFIG_RTX_DISABLE_WRITE_DAEMON */

asmlinkage long sys_rt_ioctl(unsigned int fd, unsigned int cmd, unsigned long arg)
{	
	struct file * filp;
	
	filp = rt_fcheck_file_fd(fd, RT_IO_IOCTL);
	if (filp && filp->f_op)
	{
		if (filp->f_op->unlocked_ioctl)
			return filp->f_op->unlocked_ioctl(filp, cmd, arg);	
		else
			return -EINVAL;    			// plain/old ioctl() not allowed for RT-access
	}	
	rtx_migrate_to_lx(RTLX_TASK, NULL);
	return(0);	
}


asmlinkage ssize_t sys_rt_read(unsigned int fd, char __user * buf, size_t count)
{
	struct file *filp;
	
	filp = rt_fcheck_file_fd(fd, RT_IO_READ);
	if (filp && filp->f_op && filp->f_op->read)
		return filp->f_op->read(filp, buf, count, NULL);
	rtx_migrate_to_lx(RTLX_TASK, NULL);
	return(0);		
}

/*
 * For following the rerouting via dup we tie the decision where to send stdout and stderr to the fd. 
 */
asmlinkage ssize_t sys_rt_write(unsigned int fd, const char __user *buf, size_t count)
{
	struct file *filp;

	switch (fd){
		case 0:
			return -EBADF;
		case 1:
#ifdef CONFIG_RTX_DISABLE_WRITE_DAEMON
			rtx_migrate_to_lx(RTLX_TASK, NULL);
			return 0;
#else
			return rt_write_buffered(1, &stdout, (void *)buf, count);
#endif
		case 2:
#ifdef CONFIG_RTX_DISABLE_WRITE_DAEMON
			rtx_migrate_to_lx(RTLX_TASK, NULL);
			return 0;
#else
			return rt_write_buffered(1, &stderr, (void *)buf, count);
#endif
		default:
			filp = rt_fcheck_file_fd(fd, RT_IO_WRITE);
			if (filp && filp->f_op && filp->f_op->write)	
				return filp->f_op->write(filp, buf, count, NULL);
	}
	rtx_migrate_to_lx(RTLX_TASK, NULL);
	return(0);
}

/*
 * Allow multiple calls with the same fp and the same flags.
 */
int rt_allow_access (struct file *fp, int flags)
{
	int i;
	int error = 0;
 
	if (!flags || (flags & ~(RT_IO_ALLOWED)) || (fp == NULL))
		return -EINVAL;
	if (!current->rt_state)
		return 0;
	down(&io_acc.sem);	
	for (i = 0; i < RT_IO_ENTRIES; i++) {			// look for an existing entry
		if 	(io_acc.acc_entries[i].filp == fp)
		{
#ifndef RTX_OPTIMIZE_FOR_RELEASE
			if (io_acc.acc_entries[i].tgid != current->tgid)
				printkGen(KERN_ALERT, "WARNING: %s - pid=%d detected a lost entry rt_dev_open_cnt=%d tgid=%d:%d\n", __func__, current->pid, rtx_get_cpu_var(rt_dev_open_cnt), io_acc.acc_entries[i].tgid, current->tgid);
#endif
			if (flags != io_acc.acc_entries[i].allowed_calls)
				error = -EINVAL;
			up(&io_acc.sem);	
			return (error);
		}
	}

	// Look for an empty entry and set it up.
	for (i = 0; i < RT_IO_ENTRIES; i++)
	{
		if 	(io_acc.acc_entries[i].filp == NULL) {
			io_acc.acc_entries[i].allowed_calls = flags;
			io_acc.acc_entries[i].filp = fp;
#ifndef RTX_OPTIMIZE_FOR_RELEASE
			io_acc.acc_entries[i].tgid = current->tgid;
#endif
			rtx_add_cpu_var(rt_dev_open_cnt, 1);
			up(&io_acc.sem);			
			return (0);
		}
		continue;
	}
	up(&io_acc.sem);			
	return -ENOMEM;
}

/*
 * May be called with a non-existent fp.
 */
void rt_remove_access(struct file *fp)
{
	int i;

	if (!current->rt_state)
		return;
	
	down(&io_acc.sem);
	for (i = 0; i < RT_IO_ENTRIES; i++) {
		if 	(io_acc.acc_entries[i].filp == fp)
		{
			rtx_add_cpu_var(rt_dev_open_cnt, -1);
#ifndef RTX_OPTIMIZE_FOR_RELEASE
			if (io_acc.acc_entries[i].tgid != current->tgid)
				printkGen(KERN_ALERT, "WARNING: %s - pid=%d release a lost entry rt_dev_open_cnt=%d tgid=%d:%d\n", __func__, current->pid, rtx_get_cpu_var(rt_dev_open_cnt), io_acc.acc_entries[i].tgid, current->tgid);
		    io_acc.acc_entries[i].tgid = 0;
#endif
			io_acc.acc_entries[i].allowed_calls = 0;
			io_acc.acc_entries[i].filp = NULL;
		}
	}
	up(&io_acc.sem);
}

EXPORT_SYMBOL(rt_allow_access);
EXPORT_SYMBOL(rt_remove_access);
