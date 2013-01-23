/* 
 * simple driver for audis event support
 *
 * copyright manfred.neugebauer@siemens.com
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

#include <linux/fs.h> 
#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>           

#include <linux/errno.h> 
#include <asm/uaccess.h>
#include <asm/io.h>                                
#include <linux/ioport.h>       // for the region stuff                  
#include <linux/interrupt.h>
#include <linux/aud/rt_driver.h>
#include <linux/aud/rt_base.h>

#ifdef MODULE
MODULE_AUTHOR("Manfred Neugebauer");
MODULE_DESCRIPTION( "simple audis event driver" );
MODULE_LICENSE("GPL");
#endif

#define EVENT_IRQ 99

static int evt_drv_major = 222;		// major number
static char* evt_drv_name = "evtdrv";		// device name               

static long evt_drv_ioctl(struct file *filp,
		  unsigned int cmd, unsigned long arg);
static int evt_drv_open(struct inode *inode, struct file *filp);
static int evt_drv_release(struct inode *inode, struct file *filp);

static struct file_operations evt_drv_fops =
{ 
	.unlocked_ioctl = evt_drv_ioctl,
	.open = evt_drv_open,
	.release = evt_drv_release,
};

static long evt_drv_ioctl(struct file *filp,
		  unsigned int cmd, unsigned long arg)
{	
	printk("in evt_drv_ioctl fp=%p cmd=%#x arg=%#lx\n",
		filp, cmd, arg);

	switch(cmd) {
	case AuD_EVENT_CREATE:
		//rt_register_event(ev_area_id, &event);
		break;
	case AuD_WAIT_FOR_EVENT: // for carrier threads
		//rt_wait_for_event();
		break;

	default:
		break;

	}
        
   	return (-EINVAL);
}

static int evt_drv_open (struct inode *inode, struct file *filp)
{	
	printk("in evt_drv_open fp=%p\n", filp);
   	return (rt_allow_access(filp, RT_IO_IOCTL));
}


static int evt_drv_release (struct inode *inode, struct file *filp)
{
	printk("in evt_drv_release!\n");
	rt_allow_access(filp, 0);	 // remove entry

	return 0;
}

static irqreturn_t evt_drv_isr_lx(int irq, void *dev_id)
{
	int ev_stat;

	printkGen(NULL, "in evt_drv_isr_lx!\n");

	return IRQ_HANDLED;
}

static irqreturn_t evt_drv_isr_rt(int irq, void *dev_id)
{
	int ev_stat;

	printkGen(NULL, "in evt_drv_isr_rt!\n");

	// we may do some more interrupt handling within the lx domain
	execute_nonrt_handler(0, irq);

	return IRQ_HANDLED;
}

static int __init evt_drv_init(void)
{
	int err;

	if ((err = register_chrdev(evt_drv_major, evt_drv_name, &evt_drv_fops)) < 0) 
	{
		printk("register_chrdev failed major: %d err:%x .\n", evt_drv_major, (unsigned)err);
		return err;
	} 

	if (rt_request_irq(EVENT_IRQ, evt_drv_isr_rt, 0, evt_drv_name, NULL, evt_drv_isr_lx) < 0)
	{	
		printk("Could not register evt_isr\n");
		return -1;
	}

	return 0;		
}
 
/* 
 * cleanup before unloading the module
 */ 

static void __exit evt_drv_exit(void)
{
	rt_free_irq(EVENT_IRQ, evt_drv_name);
	unregister_chrdev(evt_drv_major, evt_drv_name);	
}

module_init(evt_drv_init);
module_exit(evt_drv_exit);

