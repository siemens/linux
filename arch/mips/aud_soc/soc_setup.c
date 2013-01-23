/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2011 Manfred.Neugebauer@siemens.com
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/ioport.h>

#include <linux/serial.h>
#include <linux/serial_core.h>

#include "soc_uart.h"
#include <socGen.h>

void __init mips_pcibios_init(void);
void __init soc_be_init(void);

void __init plat_mem_setup(void)
{
	printk("AuD Soc plat_mem_setup\n");

	soc_be_init();
#ifdef CONFIG_PCI
	mips_pcibios_init();
#endif

#if (0)
	/* Request I/O space for devices used on the Malta board. */
	for (i = 0; i < ARRAY_SIZE(standard_io_resources); i++)
		request_resource(&ioport_resource, standard_io_resources+i);


#endif


}


const char *get_system_type(void)
{
	char *helpStr = "AuD_Soc1";
    
	printk("AuD Soc get_system_type:%s\n", helpStr);

	return(helpStr);
}


