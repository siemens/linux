/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 1997, 1998, 2001, 2003 by Ralf Baechle
 * (from ../sgi-ip22/ip22-reset.c)
 * adjusted for MIPS Siemens SOC1 by manfred.neugebauer@siemens.com
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/pm.h>

#include <asm/system.h>
#include <asm/reboot.h>

#include <socGen.h>

#define OFFSET_RESET_REGISTER    0x314
#define SOFT_RESET_BIT           0x2
#define OFFSET_RESET_PULSE_WIDTH 0x318
#define RESET_PULSE_WIDTH        0x10

static void soc_machine_restart(char *command)
{
	printk("enter soc_machine_restart (%s)\n", command);
	IO_WORD(SOC_SCRP_START, OFFSET_RESET_REGISTER) = SOFT_RESET_BIT;
	while (1);
}

static void soc_machine_halt(void)
{
        printk("enter soc_machine_halt\n");
	while (1);
}

static void soc_machine_power_off(void)
{
        printk("simulate soc_machine_poweroff\n");
	while (1);
}

static int __init soc_reset_setup(void)
{
	IO_WORD(SOC_SCRP_START,OFFSET_RESET_PULSE_WIDTH) = RESET_PULSE_WIDTH;

	_machine_restart = soc_machine_restart;
	_machine_halt = soc_machine_halt;
	pm_power_off = soc_machine_power_off;

	return 0;
}

subsys_initcall(soc_reset_setup);

