/****************************************************************************
 *
 *   kernel driver to jump into firmware
 *
 *   fe, 29-NOV-10
 *   ak, 06-March-2012 ported to 2.6.31
 *
 */

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/kmod.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/preempt.h>
#include <linux/proc_fs.h>
#include <linux/ptrace.h>
#include <linux/sched.h>
#include <linux/version.h>
#include <linux/vmalloc.h>
#include <asm/uaccess.h>

#if (LINUX_VERSION_CODE != KERNEL_VERSION(2,6,31))
#error incompatible kernel version
#endif

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("firmware runner");
MODULE_AUTHOR("fe");


static unsigned int jumpaddr = 0;
module_param (jumpaddr, uint, S_IRUGO);


/****************************************************************************
 *
 * runfw - driver installation routine jumping to the given address
 *
 */
static int runfw (void)
{
    typedef void (* PJUMP) (void);
    PJUMP p;

	printk (KERN_NOTICE " runfw driver is installed\n");

    
    if (jumpaddr != 0)
    {
        local_irq_disable_hw ();
        p = (PJUMP) jumpaddr;
        p ();
        return 0;
    }
    else
    {
        printk (KERN_ERR "no valid jumpaddr parameter given\n");
        return -EINVAL;
    }
}


module_init(runfw);
