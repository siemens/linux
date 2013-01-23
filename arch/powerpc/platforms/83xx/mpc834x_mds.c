/*
 * arch/powerpc/platforms/83xx/mpc834x_mds.c
 *
 * MPC834x MDS board specific routines
 *
 * Maintainer: Kumar Gala <galak@kernel.crashing.org>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/stddef.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/reboot.h>
#include <linux/pci.h>
#include <linux/kdev_t.h>
#include <linux/major.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/seq_file.h>
#include <linux/root_dev.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/mtd/mtd.h>

#include <asm/system.h>
#include <asm/atomic.h>
#include <asm/time.h>
#include <asm/io.h>
#include <asm/machdep.h>
#include <asm/ipic.h>
#include <asm/irq.h>
#include <asm/prom.h>
#include <asm/udbg.h>
#include <sysdev/fsl_soc.h>
#include <sysdev/fsl_pci.h>
#include "mpc83xx.h"

#define BCSR5_INT_USB		0x02
#ifdef CONFIG_USB_SUPPORT
static int mpc834xemds_usb_cfg(void)
{
	struct device_node *np;
	void __iomem *bcsr_regs = NULL;
	u8 bcsr5;

	mpc834x_usb_cfg();
	/* Map BCSR area */
	np = of_find_node_by_name(NULL, "bcsr");
	if (np) {
		struct resource res;

		of_address_to_resource(np, 0, &res);
		bcsr_regs = ioremap(res.start, res.end - res.start + 1);
		of_node_put(np);
	}
	if (!bcsr_regs)
		return -1;

	/*
	 * if Processor Board is plugged into PIB board,
	 * force to use the PHY on Processor Board
	 */
	bcsr5 = in_8(bcsr_regs + 5);
	if (!(bcsr5 & BCSR5_INT_USB))
		out_8(bcsr_regs + 5, (bcsr5 | BCSR5_INT_USB));
	iounmap(bcsr_regs);
	return 0;
}
#endif


//#define VIRT_IMMRBAR		((uint)0xfe000000)
#define VIRT_IMMRBAR get_immrbase()
char *virtImmrPtrAudis;

#ifdef CONFIG_AUD_LOW_LEVEL_KERNEL_DUMP 
/* LOW LEVEL SERIAL OUTPUT FUNCTIONS (from M. Neugebauer) */

#define MPC8349_IMM_OFFSET	0xe0000000
#define UART_1_OFFSET		0x4500

void ng_loop(int volatile count)
{
	while (--count) ;
}

int ppc_serial_out_char_virt(char myChar)
{
	volatile char *helpPtr;
	volatile char *statePtr;

	helpPtr = virtImmrPtrAudis + UART_1_OFFSET;
	statePtr = virtImmrPtrAudis + UART_1_OFFSET + 0x5;
	
	ng_loop(10000);
	
	*helpPtr = myChar;

	return(myChar);
}

int serial_out_char_virt_ng(char myChar)
{
	return(ppc_serial_out_char_virt(myChar));
}

int ppc_serial_out_char(char myChar)
{
	volatile char *helpPtr;
	volatile char *statePtr;
	
	helpPtr = (char *)  MPC8349_IMM_OFFSET + UART_1_OFFSET;
	statePtr = (char *)  MPC8349_IMM_OFFSET + UART_1_OFFSET + 0x5;
	*helpPtr = myChar;

	return(myChar);
}

int ppc_serial_out_state(unsigned state)
{
	volatile char *helpPtr;
	volatile char *statePtr;
	
	helpPtr = (char *)  MPC8349_IMM_OFFSET + UART_1_OFFSET;
	statePtr = (char *)  MPC8349_IMM_OFFSET + UART_1_OFFSET + 0x5;

	if (state > 15) 
	{
		*helpPtr = (0x40 + (state & 0x0f));
	}
	else 
	{
		*helpPtr = ('0' + state);
	}

	return(state);
}

int ppc_serial_out_hex(unsigned state)
{
	int ii;
	unsigned help = state;

	for (ii = 0; ii < 8; ii++, help = help >> 4) 
	{
		ppc_serial_out_state(help & 0x0f);
	}
	
	return(state);
}

int ppc_serial_out_hex_virt(unsigned state)
{
	int ii;
	unsigned help = state;

	for (ii = 0; ii < 8; ii++, help = help >> 4)
	{
		ppc_serial_out_char_virt((help & 0x0f) | 0x30);
	}
	
	return(state);
}

#endif // CONFIG_AUD_LOW_LEVEL_KERNEL_DUMP 

// called from printk.c: currently we don't have a solution which works
// during startup
void do_putchar(char myChar)
{
    //ppc_serial_out_char_virt(myChar);
    //ppc_serial_out_char(myChar);
}

/*
 * m.n. we have a problem with data lock:
 * it seems that u-boot locks the data cache
 * we use this stage to disable this lock
 * remark:the location to do this is sensitive
 */
int clear_cache_lock(int x)
{
	unsigned flag;

	flag = mfspr(SPRN_HID0);
	// printk("cache state=%#x\n", flag);
	flag &= ~HID0_DLOCK;
	mtspr(SPRN_HID0, flag);

	return(0);
}


/* ************************************************************************
 *
 * Setup the architecture
 *
 */
static void __init mpc834x_mds_setup_arch(void)
{
#ifdef CONFIG_PCI
	struct device_node *np;
#endif
	if (!virtImmrPtrAudis)
		virtImmrPtrAudis = ioremap(VIRT_IMMRBAR, 0x100000);
              
	if (ppc_md.progress)
		ppc_md.progress("mpc834x_mds_setup_arch()", 0);

#ifdef CONFIG_SIMATIC_NET_SCALANCE_W
	// scalance W -> enable the cache
	clear_cache_lock(1);
#endif

#ifdef CONFIG_PCI
	for_each_compatible_node(np, "pci", "fsl,mpc8349-pci")
		mpc83xx_add_bridge(np);
#endif

#ifdef CONFIG_USB_SUPPORT
	mpc834xemds_usb_cfg();
#endif
}

static void __init mpc834x_mds_init_IRQ(void)
{
	struct device_node *np;

	np = of_find_node_by_type(NULL, "ipic");
	if (!np)
		return;

	ipic_init(np, 0);

	/* Initialize the default interrupt mapping priorities,
	 * in case the boot rom changed something on us.
	 */
	ipic_set_default_priority();
}

static struct of_device_id mpc834x_ids[] = {
	{ .type = "soc", },
	{ .compatible = "soc", },
	{ .compatible = "simple-bus", },
	{ .compatible = "gianfar", },
	{},
};

static int __init mpc834x_declare_of_platform_devices(void)
{
	of_platform_bus_probe(NULL, mpc834x_ids, NULL);
	return 0;
}
machine_device_initcall(mpc834x_mds, mpc834x_declare_of_platform_devices);

/*
 * Called very early, MMU is off, device-tree isn't unflattened
 */
static int __init mpc834x_mds_probe(void)
{
	unsigned long root = of_get_flat_dt_root();

	return of_flat_dt_is_compatible(root, "MPC834xMDS");
}

define_machine(mpc834x_mds) {
	.name			= "MPC834x MDS",
	.probe			= mpc834x_mds_probe,
	.setup_arch		= mpc834x_mds_setup_arch,
	.init_IRQ		= mpc834x_mds_init_IRQ,
	.get_irq		= ipic_get_irq,
	.restart		= mpc83xx_restart,
	.time_init		= mpc83xx_time_init,
	.calibrate_decr		= generic_calibrate_decr,
	.progress		= udbg_progress,
};

#ifdef CONFIG_SIMATIC_CP15431

#include <asm/uaccess.h>

#define CP_MLFB_LEN      20
#define CP_SVP_LEN       32
#define CP_FW_VERS_LEN    9

static unsigned int wd_enable_gpio = 0, wd_trig_gpio = 0, phy_reset_gpio=0;
static unsigned int hwrev_0_gpio = 0, hwrev_1_gpio = 0, hwrev_2_gpio = 0, hwrev_3_gpio = 0;
static unsigned char hw_revision = 0;
static unsigned char odis = 1; /* output disable */
static unsigned char rackslot = 0;
static char sw_revision[CP_FW_VERS_LEN+3] = "";
static char mlfb[CP_MLFB_LEN+4] = "                    ";
static char serial_num[CP_SVP_LEN+4] = "                                ";


static int proc_read_hw_rev(char *buf, char **start, off_t fpos, int length, int *eof, void *data)
{
	int len = snprintf(buf, length, "%d", hw_revision);

	len -= fpos;
	if (len < 0)
		len = 0;

	*eof = (length >= 1);
	*start = buf + fpos;
	return len;
}

static int proc_read_sw_rev(char *buf, char **start, off_t fpos, int length, int *eof, void *data)
{
	int len = snprintf(buf, length, "%s", sw_revision);

	len -= fpos;
	if (len < 0)
		len = 0;

	*eof = (length >= strlen(sw_revision));
	*start = buf + fpos;
	return len;
}

static int proc_write_sw_rev(struct file *file, const char __user * buffer, unsigned long count, void *data)
{
	if (count > CP_FW_VERS_LEN)
		count = CP_FW_VERS_LEN;

	if (copy_from_user(sw_revision, buffer, count))
		return -EFAULT;
	sw_revision[count] = 0;

	return count;
}

static int proc_read_mlfb(char *buf, char **start, off_t fpos, int length, int *eof, void *data)
{
	int len = snprintf(buf, length, "%s", mlfb);

	len -= fpos;
	if (len < 0)
		len = 0;

	*eof = (length >= strlen(mlfb));
	*start = buf + fpos;
	return len;
}

static int proc_write_mlfb(struct file *file, const char __user * buffer, unsigned long count, void *data)
{
	if (count > CP_MLFB_LEN)
		count = CP_MLFB_LEN;

	if (copy_from_user(mlfb, buffer, count))
		return -EFAULT;
	mlfb[count] = 0;

	return count;
}

static int proc_read_serial_num(char *buf, char **start, off_t fpos, int length, int *eof, void *data)
{
	int len = snprintf(buf, length, "%s", serial_num);

	len -= fpos;
	if (len < 0)
		len = 0;

	*eof = (length >= strlen(serial_num));
	*start = buf + fpos;
	return len;
}

static int proc_write_serial_num(struct file *file, const char __user * buffer, unsigned long count, void *data)
{
	if (count > CP_SVP_LEN)
		count = CP_SVP_LEN;

	if (copy_from_user(serial_num, buffer, count))
		return -EFAULT;
	serial_num[count] = 0;

	return count;
}

static int proc_read_odis(char *buf, char **start, off_t fpos, int length, int *eof, void *data)
{
	int len = snprintf(buf, length, "%d", odis);

	len -= fpos;
	if (len < 0)
		len = 0;

	*eof = (length >= 1);
	*start = buf + fpos;
	return len;
}

static int proc_write_odis(struct file *file, const char __user * buffer, unsigned long count, void *data)
{
	char buf[2];
	int val;

	if (count > 1)
		count = 1;

	if (copy_from_user(buf, buffer, count))
		return -EFAULT;

	buf[1] = 0;
	sscanf(buf, "%d", &val);
	if ((val == 0) || (val == 1))
		odis = val;
	else return -EFAULT;

	return count;
}

static int proc_read_rackslot(char *buf, char **start, off_t fpos, int length, int *eof, void *data)
{
	int len = snprintf(buf, length, "%d", rackslot);

	len -= fpos;
	if (len < 0)
		len = 0;

	*eof = (length >= 1);
	*start = buf + fpos;
	return len;
}

static int proc_write_rackslot(struct file *file, const char __user * buffer, unsigned long count, void *data)
{
	char buf[4];
	int val;

	if (count > 3)
		count = 3;

	if (copy_from_user(buf, buffer, count))
		return -EFAULT;

	buf[count] = 0;
	sscanf(buf, "%d", &val);
	rackslot = val;

	return count;
}

static void __init cp15431_store_hwrev_in_proc_fs(void)
{
	struct proc_dir_entry *ent;
	unsigned char rev = 0;

	// calculate HW revnum
	if(gpio_get_value(hwrev_3_gpio))
		rev += 8;
	if(gpio_get_value(hwrev_2_gpio))
		rev += 4;
	if(gpio_get_value(hwrev_1_gpio))
		rev += 2;
	if(gpio_get_value(hwrev_0_gpio))
		rev += 1;
	hw_revision = rev;

	create_proc_read_entry("hwrev", S_IRUGO, 0, proc_read_hw_rev, 0);

	if ((ent = create_proc_entry("swrev", S_IFREG | S_IRUGO | S_IWUSR, (struct proc_dir_entry *)NULL)))
	{
		ent->read_proc = proc_read_sw_rev;
		ent->write_proc = proc_write_sw_rev;
		ent->data = NULL;
	}

	if ((ent = create_proc_entry("mlfb", S_IFREG | S_IRUGO | S_IWUSR, (struct proc_dir_entry *)NULL)))
	{
		ent->read_proc = proc_read_mlfb;
		ent->write_proc = proc_write_mlfb;
		ent->data = NULL;
	}

	if ((ent = create_proc_entry("serial_number", S_IFREG | S_IRUGO | S_IWUSR, (struct proc_dir_entry *)NULL)))
	{
		ent->read_proc = proc_read_serial_num;
		ent->write_proc = proc_write_serial_num;
		ent->data = NULL;
	}

	if ((ent = create_proc_entry("odis", S_IFREG | S_IRUGO | S_IWUSR, (struct proc_dir_entry *)NULL)))
	{
		ent->read_proc = proc_read_odis;
		ent->write_proc = proc_write_odis;
		ent->data = NULL;
	}

    if ((ent = create_proc_entry("rackslot", S_IFREG | S_IRUGO | S_IWUSR, (struct proc_dir_entry *)NULL)))
	{
		ent->read_proc = proc_read_rackslot;
		ent->write_proc = proc_write_rackslot;
		ent->data = NULL;
	}
}


static int __init cp15431_init(void)
{
	struct device_node *np, *child;

	np = of_find_compatible_node(NULL, NULL, "gpio-misc");
	if (!np) {
		printk(KERN_WARNING "********* Unable to find GPIOs\n");
		return 1;
	}

	for_each_child_of_node(np, child)
	{
		if (strcmp(child->name, "phy_reset") == 0) {
			phy_reset_gpio = of_get_gpio(child, 0);
		}
		else if (strcmp(child->name, "hw_rev0") == 0) {
			hwrev_0_gpio = of_get_gpio(child, 0);
		}
		else if (strcmp(child->name, "hw_rev1") == 0) {
			hwrev_1_gpio = of_get_gpio(child, 0);
		}
		else if (strcmp(child->name, "hw_rev2") == 0) {
			hwrev_2_gpio = of_get_gpio(child, 0);
		}
		else if (strcmp(child->name, "hw_rev3") == 0) {
			hwrev_3_gpio = of_get_gpio(child, 0);
		}
		else if (strcmp(child->name, "wd_enable") == 0) {
			wd_enable_gpio = of_get_gpio(child, 0);
		}
		else if (strcmp(child->name, "wd_trig") == 0) {
			wd_trig_gpio = of_get_gpio(child, 0);
		}
	}
	of_node_put(np);

	cp15431_store_hwrev_in_proc_fs();
	return 0;
}

late_initcall(cp15431_init);
#endif

#ifdef CONFIG_SIMATIC_NET_SCALANCE_W

/************************ added for Scalance W786 ****************************/

static unsigned int device_type_gpio, cplug_present_gpio, phy_reset_gpio;

#define WLAN_EAP_BOARD_TYPE_OPTICAL 0
#define WLAN_EAP_BOARD_TYPE_COPPER  1

static int wlan_eap_board_type; //!< EAP board type (fibre optic or RJ45)
#define CONFIG_PHY_RESET_TIME_IN_US     50000

int wlan_eap_proc_entries(void);
int wlan_eap_get_board_type(void);

static int __init wlan_eap_init(void)
{
	struct device_node *np, *child;

	np = of_find_compatible_node(NULL, NULL, "gpio-misc");
	if (!np) {
		printk(KERN_WARNING "********* Unable to find GPIOs\n");
		return 1;
	}

	for_each_child_of_node(np, child)
		if (strcmp(child->name, "device_type") == 0) {
			device_type_gpio = of_get_gpio(child, 0);
//            gpio_direction_input(device_type_gpio);
		}
		else if (strcmp(child->name, "cplug_present") == 0) {
			cplug_present_gpio = of_get_gpio(child, 0);
//			gpio_direction_input(cplug_present_gpio);
		}
		else if (strcmp(child->name, "phy_reset") == 0) {
			phy_reset_gpio = of_get_gpio(child, 0);
//			 gpio_direction_output(phy_reset_gpio, 0);
		}
	of_node_put(np);

	wlan_eap_get_board_type();

	wlan_eap_proc_entries();
	return 0;
}


/** Auxiliary function to get the board type (fibre optic or RJ45). 
 * 
 * \return zero, if this device is a fibre optic device.
 * \return one, if this device is not equiped with a fibre optic unit. 
 */
int wlan_eap_get_board_type(void) 
{
	static unsigned char initOK = 0;
	
	if(initOK) return wlan_eap_board_type;

	gpio_set_value(phy_reset_gpio, 0);

	/* configure phy reset function -> phy reset on */
	//regWriteBitsLocked(M83XX_GP1DAT(CCSBAR), 0, GPIO1_PHY_RESET);
	//regWriteBitsLocked(M83XX_GP1DIR(CCSBAR), DIR_OUT, GPIO1_PHY_RESET);

	// wait for some µsec
	udelay(CONFIG_PHY_RESET_TIME_IN_US);

	// check for fibre optic
	if(gpio_get_value(device_type_gpio))
	{
		// this device is a board with fibre optic support
		wlan_eap_board_type = WLAN_EAP_BOARD_TYPE_OPTICAL;
		printk("Found EAP with optical port !\n");
	} 
	else 
	{
		// this board is a RJ45 device
		wlan_eap_board_type = WLAN_EAP_BOARD_TYPE_COPPER;
		printk("Found EAP with RJ45 port !\n");
	}

	gpio_set_value(phy_reset_gpio, 1);

	initOK = 1;

	return wlan_eap_board_type;
}
EXPORT_SYMBOL(wlan_eap_get_board_type);


int wlan_eap_get_cplug_present(void) 
{
	// check for C-PLUG
	if(gpio_get_value(cplug_present_gpio))
	{
		return 0;
	}
	return 1;
}
EXPORT_SYMBOL(wlan_eap_get_cplug_present);

#define EPLD_BASE_ADRS 0x78400000
static unsigned char *hw_epld_ptr = 0;

static int
proc_read_hw_rev(char *buf, char **start, off_t fpos, int length, int *eof, void *data)
{
  int len=0;
  if(length > 50)
	  len += sprintf(buf+len, "REV: %d\n", (*(hw_epld_ptr+0x05) & 0x0f) + 1);
  *eof = 1;
  return len;
}

#define NAND_FLASH_BASE_ADRS 0x78400000

static int
proc_read_hw_cplug(char *buf, char **start, off_t fpos, int length, int *eof, void *data)
{
  int len=0;
  if(length > 50) {
		  len += sprintf(buf+len, "PRESENT: %d\n", wlan_eap_get_cplug_present());
		  len += sprintf(buf+len, "ADDR: 0x%08x\n", NAND_FLASH_BASE_ADRS);
  }
  *eof = 1;
  return len;
}

#define HWINFO_MTD_NAME "HWINFO"

static int
proc_read_hw_info(char *buf, char **start, off_t fpos, int length, int *eof, void *data)
{
	struct mtd_info *mymtd;
	size_t lenwritten=0;
	int i;

	for(i=0;i<MAX_MTD_DEVICES;i++){
		mymtd = get_mtd_device(NULL,i);
		if(!mymtd||!strcmp(mymtd->name,HWINFO_MTD_NAME))
			break;
		put_mtd_device(mymtd);
	}
	if(!mymtd || !mymtd->read){
		printk(KERN_WARNING "hwinfo procread: Failed to read MTD: %s\n", HWINFO_MTD_NAME);
		return 0;
	}
	if(mymtd->read(mymtd, (loff_t)fpos, (size_t)length, &lenwritten, (u_char*)buf)) {
		lenwritten = 0;
	}
	put_mtd_device(mymtd);
	if(lenwritten < length) {
		*eof = 1;
		*start = 0;
	} else {
		*eof = 0;
		*start = (char*)lenwritten;
	}
	return lenwritten;
}

int wlan_eap_proc_entries(void)
{
	struct proc_dir_entry *ent;

	hw_epld_ptr = ioremap(EPLD_BASE_ADRS, 32);

	ent = create_proc_read_entry("hwrev", S_IRUGO, 0, proc_read_hw_rev, 0);
	if (!ent)
		return 1;

	ent = create_proc_read_entry("hwinfo", S_IRUGO, 0, proc_read_hw_info, 0);
	if (!ent)
		return 1;

	ent = create_proc_read_entry("cplug", S_IRUGO, 0, proc_read_hw_cplug, 0);
	if (!ent)
		return 1;

	return 0;
}

module_init(wlan_eap_init);

#endif /* #ifdef CONFIG_SIMATIC_NET_SCALANCE_W */

