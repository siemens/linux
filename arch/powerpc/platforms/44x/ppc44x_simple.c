/*
 * Generic PowerPC 44x platform support
 *
 * Copyright 2008 IBM Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; version 2 of the License.
 *
 * This implements simple platform support for PowerPC 44x chips.  This is
 * mostly used for eval boards or other simple and "generic" 44x boards.  If
 * your board has custom functions or hardware, then you will likely want to
 * implement your own board.c file to accommodate it.
 */

#include <asm/machdep.h>
#include <asm/pci-bridge.h>
#include <asm/ppc4xx.h>
#include <asm/prom.h>
#include <asm/time.h>
#include <asm/udbg.h>
#include <asm/uic.h>
#include <asm/uaccess.h>

#include <linux/init.h>
#include <linux/of_platform.h>

static __initdata struct of_device_id ppc44x_of_bus[] = {
	{ .compatible = "ibm,plb4", },
	{ .compatible = "ibm,opb", },
	{ .compatible = "ibm,ebc", },
	{ .compatible = "simple-bus", },
	{},
};

static int __init ppc44x_device_probe(void)
{
	of_platform_bus_probe(NULL, ppc44x_of_bus, NULL);

	return 0;
}
machine_device_initcall(ppc44x_simple, ppc44x_device_probe);

/* This is the list of boards that can be supported by this simple
 * platform code.  This does _not_ mean the boards are compatible,
 * as they most certainly are not from a device tree perspective.
 * However, their differences are handled by the device tree and the
 * drivers and therefore they don't need custom board support files.
 *
 * Again, if your board needs to do things differently then create a
 * board.c file for it rather than adding it to this list.
 */
static char *board[] __initdata = {
	"amcc,arches",
	"amcc,bamboo",
	"amcc,canyonlands",
	"amcc,glacier",
	"ibm,ebony",
	"amcc,katmai",
	"amcc,rainier",
	"amcc,redwood",
	"amcc,sequoia",
	"amcc,taishan",
	"amcc,yosemite"
};

static int __init ppc44x_probe(void)
{
	unsigned long root = of_get_flat_dt_root();
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(board); i++) {
		if (of_flat_dt_is_compatible(root, board[i])) {
			ppc_pci_set_flags(PPC_PCI_REASSIGN_ALL_RSRC);
			return 1;
		}
	}

	return 0;
}

define_machine(ppc44x_simple) {
	.name = "PowerPC 44x Platform",
	.probe = ppc44x_probe,
	.progress = udbg_progress,
	.init_IRQ = uic_init_tree,
	.get_irq = uic_get_irq,
	.restart = ppc4xx_reset_system,
	.calibrate_decr = generic_calibrate_decr,
};


#ifdef CONFIG_SIMATIC_NET_SCALANCE_W

/************************ added for Scalance W ****************************/
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/mtd/mtd.h>

#include <asm/wlan/led.h>

static unsigned int cplug_present_gpio=0, cplug_production_gpio=0, cplug_wp_gpio, phy_reset_gpio=0;
static unsigned int hwrev_0_gpio=0, hwrev_1_gpio=0, hwrev_2_gpio=0, hwrev_3_gpio=0;
static unsigned int hwtype_0_gpio=0, hwtype_1_gpio=0, hwtype_2_gpio=0, hwtype_3_gpio=0;
static unsigned int digital_in_gpio=0, digital_out_gpio=0, button_mode_gpio=0, button_gpio=0;
static unsigned int power_ok_poe_gpio=0, power_ok_ext1_gpio=0, power_ok_ext2_gpio=0;

struct scw_board_data_t
{
	unsigned char hw_rev;
	unsigned char hw_type;
	short sfp_speed[2];
	/* we don't need the full array for SCW_PPC460 - 32 leds are more
	   than enough for us. */
	unsigned long leds_startup_state;

};
/* store board related information */
static struct scw_board_data_t scw_board_data;

int wlan_scw_proc_entries(void);


static int wlan_scw_get_power_ok_poe(void)
{
#ifdef CANYONLANDS_EVAL_BOARD
	return 0;
#else
	if(gpio_get_value(power_ok_poe_gpio))
	{
		return 1;
	}
	return 0;
#endif
}


static int wlan_scw_get_power_ok_ext1(void)
{
#ifdef CANYONLANDS_EVAL_BOARD
	return 0;
#else
	if(gpio_get_value(power_ok_ext1_gpio))
	{
		return 1;
	}
	return 0;
#endif
}


static int wlan_scw_get_power_ok_ext2(void)
{
#ifdef CANYONLANDS_EVAL_BOARD
	return 0;
#else
	if(gpio_get_value(power_ok_ext2_gpio))
	{
		return 1;
	}
	return 0;
#endif
}


static void __init scw_store_hwrev(void)
{
#ifdef CANYONLANDS_EVAL_BOARD
	scw_board_data.hw_rev = 0;
#else
	unsigned char rev = 0;
	// calculate HW revnum
	if(gpio_get_value(hwrev_3_gpio))
		rev += 1;
	if(gpio_get_value(hwrev_2_gpio))
		rev += 2;
	if(gpio_get_value(hwrev_1_gpio))
		rev += 4;
	if(gpio_get_value(hwrev_0_gpio))
		rev += 8;
	rev+=1; /* add 1 since we start on the hw with 0 */

	scw_board_data.hw_rev = rev;
#endif
}


static void __init scw_store_hwtype(void)
{
#ifdef CANYONLANDS_EVAL_BOARD
	scw_board_data.hw_type = 0;
#else
	unsigned char type = 0;
	// calculate HW type
	if(gpio_get_value(hwtype_3_gpio))
		type += 1;
	if(gpio_get_value(hwtype_2_gpio))
		type += 2;
	if(gpio_get_value(hwtype_1_gpio))
		type += 4;
	if(gpio_get_value(hwtype_0_gpio))
		type += 8;

	scw_board_data.hw_type = type;
#endif
}


static int scw_is_sfp_present(int sfp)
{
	int gpio=0;

	switch(sfp) {
	case 0:
		gpio = power_ok_ext2_gpio;
		break;
	case 1:
		gpio = digital_in_gpio;
		break;
	default:
		printk("Invalid SFP\n");
		return 0;
	}

	return !gpio_get_value(gpio);
}


static int __init scw_calc_sfp_speed(int sfp)
{
	return 1000;
}


static void __init scw_store_sfp_speed(void)
{
	int i;

	if (scw_board_data.hw_type != 8) {
		scw_board_data.sfp_speed[0] = scw_board_data.sfp_speed[1] = 0;
		return;
	}
	/* DD: SFP Board - calculate speed */
	for (i=0; i<2; i++) {
		if (scw_is_sfp_present(i))
			scw_board_data.sfp_speed[i] = scw_calc_sfp_speed(i);
		else
			scw_board_data.sfp_speed[i] = -1;
	}
}


void inline scw_set_startup_led_state(unsigned long led, int state)
{
	if (likely (state == 1))
		scw_board_data.leds_startup_state |= led;
	else
		scw_board_data.leds_startup_state &= ~(led);
}
EXPORT_SYMBOL(scw_set_startup_led_state);


static int wlan_scw_get_digital_in(void) 
{
#ifdef CANYONLANDS_EVAL_BOARD
	return 0;
#else
	if(gpio_get_value(digital_in_gpio))
	{
		return 1;
	}
	return 0;
#endif
}


static int wlan_scw_get_digital_out(void) 
{
#ifdef CANYONLANDS_EVAL_BOARD
	return 0;
#else
	if(gpio_get_value(digital_out_gpio))
	{
		return 1;
	}
	return 0;
#endif
}


static void wlan_scw_set_digital_out(unsigned long value) 
{
#ifdef CANYONLANDS_EVAL_BOARD
	return;
#else
	gpio_set_value(digital_out_gpio, (value ? 1 : 0));
	return;
#endif
}


static void wlan_scw_set_phy_reset(unsigned long value) 
{
#ifdef CANYONLANDS_EVAL_BOARD
	return;
#else
	gpio_set_value(phy_reset_gpio, (value ? 1 : 0));
	return;
#endif
}


static int wlan_scw_get_button_mode(void) 
{
#ifdef CANYONLANDS_EVAL_BOARD
	return 0;
#else
	if(gpio_get_value(button_mode_gpio))
	{
		return 1;
	}
	return 0;
#endif
}


static int wlan_scw_get_button(void) 
{
#ifdef CANYONLANDS_EVAL_BOARD
	return 0;
#else
	if(gpio_get_value(button_gpio))
	{
		return 0;
	}
	return 1;
#endif
}


static void wlan_scw_set_button_mode(unsigned long value) 
{
#ifdef CANYONLANDS_EVAL_BOARD
	return;
#else
	gpio_set_value(button_mode_gpio, (value ? 1 : 0));
	return;
#endif
}


static int wlan_scw_get_cplug_wp(void)
{
#ifdef CANYONLANDS_EVAL_BOARD
	return 0;
#else
	/* High is write enabled, Low is write protected */
	if(gpio_get_value(cplug_wp_gpio))
		return 0;
	else
		return 1;
#endif
}


static void wlan_scw_set_cplug_wp(unsigned int value) 
{
#ifdef CANYONLANDS_EVAL_BOARD
	return;
#else
	gpio_set_value(cplug_wp_gpio, value);
	return;
#endif
}


static int wlan_scw_get_cplug_present(void) 
{
#ifdef CANYONLANDS_EVAL_BOARD
	return 0;
#else
	// check for C-PLUG
	if(gpio_get_value(cplug_present_gpio))
	{
		return 0;
	}
	return 1;
#endif
}


static int wlan_scw_get_cplug_production(void) 
{
#ifdef CANYONLANDS_EVAL_BOARD
	return 0;
#else
	// check for C-PLUG
	if(gpio_get_value(cplug_production_gpio))
	{
		return 0;
	}
	return 1;
#endif
}


int scw_get_hwrev(void)
{
	return scw_board_data.hw_rev;
}
EXPORT_SYMBOL(scw_get_hwrev);


int scw_get_hwtype(void)
{
	return scw_board_data.hw_type;
}
EXPORT_SYMBOL(scw_get_hwtype);


int scw_get_sfp_speed(int sfp)
{
	return scw_board_data.sfp_speed[sfp];
}
EXPORT_SYMBOL(scw_get_sfp_speed);


unsigned long scw_get_led_startup_state(void)
{
	return scw_board_data.leds_startup_state;
}
EXPORT_SYMBOL(scw_get_led_startup_state);


static void __init init_board_data(void)
{
	memset(&scw_board_data, 0, sizeof(scw_board_data));

	scw_store_hwrev();
	scw_store_hwtype();
	scw_store_sfp_speed();

	if (wlan_scw_get_power_ok_ext1())
		scw_set_startup_led_state(LED_POWER1_ON, LED_ON);

	/* Only RAP has second power line */
	if ((scw_get_hwtype() < 5) && wlan_scw_get_power_ok_ext2())
		scw_set_startup_led_state(LED_POWER2_ON, LED_ON);

	/* SFP Variant of EAP does not support POE */
	if ((scw_get_hwtype() != 8) && wlan_scw_get_power_ok_poe())
		scw_set_startup_led_state(LED_POWER_POE_ON, LED_ON);
}


static int __init wlan_scw_init(void)
{
	struct device_node *np, *child;

	np = of_find_compatible_node(NULL, NULL, "gpio-misc");
	if (!np) {
		printk(KERN_WARNING "********* Unable to find GPIOs\n");
		return 1;
	}

	for_each_child_of_node(np, child)
		if (strcmp(child->name, "cplug_present") == 0) {
			cplug_present_gpio = of_get_gpio(child, 0);
		}
		else if (strcmp(child->name, "prod_test") == 0) {
			cplug_production_gpio = of_get_gpio(child, 0);
		}
		else if (strcmp(child->name, "cplug_protect") == 0) {
			cplug_wp_gpio = of_get_gpio(child, 0);
		}
		else if (strcmp(child->name, "phy_reset") == 0) {
			phy_reset_gpio = of_get_gpio(child, 0);
//			 gpio_direction_output(phy_reset_gpio, 0);
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
		else if (strcmp(child->name, "hw_type0") == 0) {
			hwtype_0_gpio = of_get_gpio(child, 0);
		}
		else if (strcmp(child->name, "hw_type1") == 0) {
			hwtype_1_gpio = of_get_gpio(child, 0);
		}
		else if (strcmp(child->name, "hw_type2") == 0) {
			hwtype_2_gpio = of_get_gpio(child, 0);
		}
		else if (strcmp(child->name, "hw_type3") == 0) {
			hwtype_3_gpio = of_get_gpio(child, 0);
		}
		else if (strcmp(child->name, "digit_output0") == 0) {
			digital_out_gpio = of_get_gpio(child, 0);
		}
		else if (strcmp(child->name, "digit_input0") == 0) {
			digital_in_gpio = of_get_gpio(child, 0);
		}
		else if (strcmp(child->name, "button_mode") == 0) {
			button_mode_gpio = of_get_gpio(child, 0);
		}
		else if (strcmp(child->name, "button_irq") == 0) {
			button_gpio = of_get_gpio(child, 0);
		}
		else if (strcmp(child->name, "power_ok_poe") == 0) {
			power_ok_poe_gpio = of_get_gpio(child, 0);
		}
		else if (strcmp(child->name, "power_ok_ext1") == 0) {
			power_ok_ext1_gpio = of_get_gpio(child, 0);
		}
		else if (strcmp(child->name, "power_ok_ext2") == 0) {
			power_ok_ext2_gpio = of_get_gpio(child, 0);
		}
	of_node_put(np);

	init_board_data();
	wlan_scw_proc_entries();
	return 0;
}




static int
proc_read_hw_rev(char *buf, char **start, off_t fpos, int length, int *eof, void *data)
{
#ifdef CANYONLANDS_EVAL_BOARD
	int len=0;
	if(length > 50) {
		len += sprintf(buf+len, "REV: %d\n", scw_get_hwrev());
		len += sprintf(buf+len, "TYPE: %d\n", scw_get_hwtype());
	}
	*eof = 1;
	return len;
#else
	int len=0;
	if(length > 50) {
		  len += sprintf(buf+len, "REV: %d\n", scw_get_hwrev());
		  len += sprintf(buf+len, "TYPE: %d\n", scw_get_hwtype());
	}
	*eof = 1;
	return len;
#endif
}


static int
proc_read_digital_in(char *buf, char **start, off_t fpos, int length, int *eof, void *data)
{
  int len=0;
  if(length > 50) {
		  len += sprintf(buf+len, "INPUT: %d\n", wlan_scw_get_digital_in());
  }
  *eof = 1;
  return len;
}


static int proc_read_digital_out(char *buf, char **start, off_t fpos, int length, int *eof, void *data)
{
  int len=0;
  if(length > 50) {
		  len += sprintf(buf+len, "OUTPUT: %d\n", wlan_scw_get_digital_out());
  }
  *eof = 1;
  return len;
}


static int proc_write_digital_out(struct file *file, const char __user * buffer,
			     unsigned long count, void *data)
{
	char buf[] = "0x00000000\n";
	unsigned long len = min((unsigned long)sizeof(buf) - 1, count);
	unsigned long val;

	if (copy_from_user(buf, buffer, len))
		return count;
	buf[len] = 0;
	if (sscanf(buf, "%li", &val) != 1) {
			printk(": %s is not in hex or decimal form.\n", buf);
	}
	else
	{
		wlan_scw_set_digital_out(val);
	}

	return strnlen(buf, len);
}


static int proc_read_button(char *buf, char **start, off_t fpos, int length, int *eof, void *data)
{
  int len=0;
  if(length > 50) {
	  len += sprintf(buf+len, "BUTTON: %d\n", wlan_scw_get_button());
		  len += sprintf(buf+len, "MODE: %d\n", wlan_scw_get_button_mode());
  }
  *eof = 1;
  return len;
}

static int proc_read_power_states(char *buf, char **start, off_t fpos, int length, int *eof, void *data)
{
  int len=0;
  if(length > 50) {
	  len += sprintf(buf+len, "POE: %d\n", wlan_scw_get_power_ok_poe());
	  len += sprintf(buf+len, "EXT1: %d\n", wlan_scw_get_power_ok_ext1());
	  len += sprintf(buf+len, "EXT2: %d\n", wlan_scw_get_power_ok_ext2());
  }
  *eof = 1;
  return len;
}

static int proc_write_button_mode(struct file *file, const char __user * buffer,
			     unsigned long count, void *data)
{
	char buf[] = "0x00000000\n";
	unsigned long len = min((unsigned long)sizeof(buf) - 1, count);
	unsigned long val;

	if (copy_from_user(buf, buffer, len))
		return count;
	buf[len] = 0;
	if (sscanf(buf, "%li", &val) != 1) {
		printk(": %s is not in hex or decimal form.\n", buf);
	}
	else
	{
		wlan_scw_set_button_mode(val);
	}

	return strnlen(buf, len);
}


static int proc_write_phy_reset(struct file *file, const char __user * buffer,
			     unsigned long count, void *data)
{
	char buf[] = "0x00000000\n";
	unsigned long len = min((unsigned long)sizeof(buf) - 1, count);
	unsigned long val;

	if (copy_from_user(buf, buffer, len))
		return count;
	buf[len] = 0;
	if (sscanf(buf, "%li", &val) != 1) {
		printk(": %s is not in hex or decimal form.\n", buf);
	}
	else
	{
		wlan_scw_set_phy_reset(val);
	}

	return strnlen(buf, len);
}


static int proc_read_cplug_wp(char *buf, char **start, off_t fpos, int length, int *eof, void *data)
{
	int len=0;
	if(length >= 10 ) {
		len += sprintf(buf, "WRITEPROTECT: %d\n", wlan_scw_get_cplug_wp());
	}
	*eof = 1;
	return len;
}


static int proc_write_cplug_wp(struct file *file, const char __user * buffer,
			     unsigned long count, void *data)
{
	char buf[] = "000000000";
	unsigned long len = min((unsigned long)sizeof(buf) - 1, count);
	int val = -1;

	if (copy_from_user(buf, buffer, len))
		return -EFAULT;

	buf[len] = '\0';
	sscanf(buf, "%d", &val);
	if ((val == 0) || (val == 1))
		wlan_scw_set_cplug_wp(val);
	else
		printk(KERN_NOTICE "Invalid input: %s. Cannot set cplug_wp\n", buf);

	return strnlen(buf, len); /* return no of bytes written */
}


#define NAND_FLASH_BASE_ADRS 0x78400000


static int
proc_read_hw_cplug(char *buf, char **start, off_t fpos, int length, int *eof, void *data)
{
  int len=0;
  if(length > 50) {
		  len += sprintf(buf+len, "PRESENT: %d\n", wlan_scw_get_cplug_present());
		  len += sprintf(buf+len, "PRODPLUG: %d\n", wlan_scw_get_cplug_production());
		  len += sprintf(buf+len, "WRITEPROTECT: %d\n", wlan_scw_get_cplug_production());
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

	mymtd = get_mtd_device_nm(HWINFO_MTD_NAME);
	if (IS_ERR(mymtd)) {
		printk(KERN_WARNING "hwinfo procread: MTD partition %s not found\n", HWINFO_MTD_NAME);
		return 0;
	}
	if (!mymtd->read) {
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

static struct proc_dir_entry *digital_out_proc = NULL;
static struct proc_dir_entry *button_mode_proc = NULL;
static struct proc_dir_entry *cplug_wp_proc = NULL;
static struct proc_dir_entry *phy_reset_proc = NULL;

int wlan_scw_proc_entries(void)
{
	struct proc_dir_entry *ent;

	ent = create_proc_read_entry("hwrev", S_IRUGO, 0, proc_read_hw_rev, 0);
	if (!ent)
		return 1;

	ent = create_proc_read_entry("hwinfo", S_IRUGO, 0, proc_read_hw_info, 0);
	if (!ent)
		return 1;

	ent = create_proc_read_entry("cplug", S_IRUGO, 0, proc_read_hw_cplug, 0);
	if (!ent)
		return 1;

	ent = create_proc_entry("cplug_wp", S_IFREG | S_IRUGO | S_IWUSR, cplug_wp_proc);
	if (!ent)
		return 1;

	ent->read_proc = proc_read_cplug_wp;
	ent->write_proc = proc_write_cplug_wp;
	ent->data = NULL;

	ent = create_proc_read_entry("digital_in", S_IRUGO, 0, proc_read_digital_in, 0);
	if (!ent)
		return 1;

	ent = create_proc_entry("digital_out", S_IFREG | S_IRUGO | S_IWUSR, digital_out_proc);
	if (!ent)
		return 1;

	ent->read_proc = proc_read_digital_out;
	ent->write_proc = proc_write_digital_out;
	ent->data = NULL;

	ent = create_proc_entry("button_mode", S_IFREG | S_IRUGO | S_IWUSR, button_mode_proc);
	if (!ent)
		return 1;

	ent->read_proc = proc_read_button;
	ent->write_proc = proc_write_button_mode;
	ent->data = NULL;

	ent = create_proc_read_entry("button", S_IRUGO, 0, proc_read_button, 0);
	if (!ent)
		return 1;

	ent = create_proc_read_entry("power_states", S_IRUGO, 0, proc_read_power_states, 0);
	if (!ent)
		return 1;

	/* test only */
#if 1
	ent = create_proc_entry("phy_reset", S_IFREG | S_IRUGO | S_IWUSR, phy_reset_proc);
	if (!ent)
		return 1;

	ent->read_proc = NULL;
	ent->write_proc = proc_write_phy_reset;
	ent->data = NULL;
#endif
	return 0;
}

late_initcall(wlan_scw_init);


#ifdef CONFIG_AUD_LOW_LEVEL_PRINTK

/*
 * to catch problems during early kernel startup
 * we may use the serial line address provided by u-boot
 *
 * To use this, we need to prevent that the tlb entry for
 * this address range is deleted. See documentation in
 * arch/powerpc/kernel/head_44x.S to do this.
 * you need to set up the define  FORCE_LOW_LEVEL_SERIAL_OUT
 * (see below) and ENABLE_LOW_LEVEL_SERIAL_OUT_TLB within head_44x.S
 */

//#define FORCE_LOW_LEVEL_SERIAL_OUT
#define PHYS_ADDR_SERIAL_OUT		0xef600300

static unsigned volatile myLoopCnt;

void serial_out_lowlevel_phys(char c)
{
#ifdef FORCE_LOW_LEVEL_SERIAL_OUT
	*((char *) (PHYS_ADDR_SERIAL_OUT)) = c;
#endif
}

void serial_out_lowlevel_phys_d(char c)
{
#ifdef FORCE_LOW_LEVEL_SERIAL_OUT
	int ii;
	for (ii = 0; ii < 10000; ii++)
		myLoopCnt += ii;
	*((char *) (PHYS_ADDR_SERIAL_OUT)) = c;
#endif
}

// called from printk.c for low level output during startup
void do_putchar(char myChar)
{
	int ii;

	if (myChar == '\n')
		do_putchar('\r');
	for (ii = 0; ii < 10000; ii++) // delay loop
		myLoopCnt += ii; 

	serial_out_lowlevel_phys(myChar);
}

/*
 * you may also use the ppc44x CONFIG_PPC_EARLY_DEBUG_44x option
 * (see configuration option "early debugging" within kernel hacking)
 * you need to set up the following physical addresses:
 * low 32 bit: PHYS_ADDR_SERIAL_OUT (see above)
 * high EPRN: 0x4
 * the virtual address will end up with 0xf0000000
 *                             (offset 0x300 should be the serial line)
 * the code for this serial line handling ist in
 * arch/powerpc/kernel/udbg_16550.c,
 * the translation initialization in arch/powerpc/kernel/head_44x.S
 */
#endif /* CONFIG_AUD_LOW_LEVEL_PRINTK */

#endif /* #ifdef CONFIG_SIMATIC_NET_SCALANCE_W */

#ifdef CONFIG_RTX_DOMAIN

#ifndef CONFIG_RTX_DOMAIN_HRT

#include <linux/aud/rt_timer.h>

/*
 * normally we use high res timer for realtime timing support
 * if you need a tick based solution, you have to implement
 * your own realtime timer device.
 */
#error "ppc 44x rt with no highres timer not yet implemented"

/* TBD: handle timer stuff with this powerpc platform */
int ack_ppc_timer(void)
{
	return(0);
}

int rtx_timer_init(void)
{
	//aud_rt_timer = &default_aud_timer;

	return(0);
}

int rtxTimerStart(void)
{
	printkGen(NULL, "%s:%s\n", __func__, "decr");
	return(0);
}

int rtxTimerStop(void)
{
	printkGen(NULL, "%s:%s\n", __func__, "decr");
	return(0);
}

#endif
#endif
