/* 
 * simple driver for audis synchronous timer support
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
MODULE_DESCRIPTION( "simple audis sync timer driver" );
MODULE_LICENSE("GPL");
#endif

#define SYNC_IRQ 99

// for simulation of an irq
#define SIMULATE_SYNC_IRQ_IOCTL 0x23101947

static int sync_drv_major = 221;		// major number
static char* sync_drv_name = "syncdrv";		// device name               

static void (*sync_drv_clock_callback)(void);

static long sync_drv_ioctl(struct file *filp,
		  unsigned int cmd, unsigned long arg);
static int sync_drv_open(struct inode *inode, struct file *filp);
static int sync_drv_release(struct inode *inode, struct file *filp);
static irqreturn_t sync_drv_isr_rt(int irq, void *dev_id);

static struct file_operations sync_drv_fops =
{ 
	.unlocked_ioctl = sync_drv_ioctl,
	.open = sync_drv_open,
	.release = sync_drv_release,
};

static struct timespec period;

static long sync_drv_ioctl(struct file *filp,
		  unsigned int cmd, unsigned long arg)
{	
	struct rt_clock_desc reg_clock_info;
	clockid_t clockid;
	long retval = 0;
	int ret;

	printkGen(NULL, "in sync_drv_ioctl fp=%p cmd=%#x arg=%#lx\n",
		filp, cmd, arg);

   	switch (cmd) {
	case AuD_REGISTER_CLOCK:
		if (current->rt_state == LX_TASK)
			return -EPERM;
		if ((retval = copy_from_user(&reg_clock_info, (void *) arg,
				sizeof(struct rt_clock_desc))))
			return retval;
		// we may do some consistency tests
		if (reg_clock_info.clock_spec.tv_nsec != period.tv_nsec) {

		}
		retval = rt_register_sync_clock(filp, &reg_clock_info, CLOCK_SYNC_HARD,
			&sync_drv_clock_callback);
		if (retval < 0)
			return retval;

#ifdef CONFIG_SMP
		// we need to pin the rt irq to the cpu of the requestor
		// this stuff is currently under discussion
		// see Wolfgang for the actual state
		ret = rt_enable_irq_affinity(SYNC_IRQ);
		if (ret < 0) {
			rt_unregister_sync_clock(filp, retval);
			return ret;
		}
#endif

		// now it's time to init our timer base

		break;

	case AuD_UNREGISTER_CLOCK:
		clockid = (clockid_t) arg;

		if (current->rt_state == LX_TASK)
			return -EPERM;

		// first stop our timer base

		sync_drv_clock_callback = NULL;
		ret = rt_disable_irq_affinity(SYNC_IRQ);
		return rt_unregister_sync_clock(filp, clockid);

		break;


	case SIMULATE_SYNC_IRQ_IOCTL:
		// simulate a sync clock source interrupt
		if (current->rt_state == LX_TASK)
			return -EPERM;
		sync_drv_isr_rt(SYNC_IRQ, NULL);
		break;

	default:
		return (-EINVAL);
	}
	return retval;
}

static int sync_drv_open (struct inode *inode, struct file *filp)
{	
	printkGen(NULL, "in sync_drv_open fp=%p\n", filp);
   	return (rt_allow_access(filp, RT_IO_IOCTL));
}


static int sync_drv_release (struct inode *inode, struct file *filp)
{
	printkGen(NULL, "in sync_drv_release!\n");
	rt_allow_access(filp, 0);	 // remove entry

	// we may need to disable our clock source
	// and unregister the clock from the audis driver
	return 0;
}

static irqreturn_t sync_drv_isr_lx(int irq, void *dev_id)
{

	printkGen(NULL, "in sync_drv_isr_lx!\n");

	return IRQ_HANDLED;
}

static irqreturn_t sync_drv_isr_rt(int irq, void *dev_id)
{

	printkGen(NULL, "in sync_drv_isr_rt!\n");

	// call the audis driver to do the remaining stuff
	// (sending the event, handling timers with offset timing values)
	if (sync_drv_clock_callback) {
		sync_drv_clock_callback();
	}

	return IRQ_HANDLED;
}

static int __init sync_drv_init(void)
{
	int err;

	period.tv_sec = 0;
	period.tv_nsec = 25000000;  // cycle 25 ms

	if ((err = register_chrdev(sync_drv_major, sync_drv_name, &sync_drv_fops)) < 0) 
	{
		printk("register_chrdev failed major: %d err:%x .\n", sync_drv_major, (unsigned)err);
		return err;
	} 

	if (rt_request_irq(SYNC_IRQ, sync_drv_isr_rt, 0, sync_drv_name, NULL, sync_drv_isr_lx) < 0)
	{	
		printk("Could not register sync_isr\n");
		return -1;
	}
	sync_drv_clock_callback = NULL;

	return 0;		
}
 
/* 
 * cleanup before unloading the module
 */ 

static void __exit sync_drv_exit(void)
{
	rt_free_irq(SYNC_IRQ, sync_drv_name);
	unregister_chrdev(sync_drv_major, sync_drv_name);	
}

module_init(sync_drv_init);
module_exit(sync_drv_exit);

