/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2011 Manfred.Neugebauer@siemens.com
 */

#include <linux/kernel.h>
#include <linux/module.h>

#include <linux/init.h>
#include <linux/mm.h>
#include <linux/bootmem.h>
#include <linux/pfn.h>

#include <linux/platform_device.h>

#include <asm/bootinfo.h>
#include <asm/page.h>
#include <asm/sections.h>

#include <asm/mips-boards/prom.h>

#ifdef CONFIG_MTD_NAND_SOC1
static struct platform_device soc_nand_dev = {
	.name		= "soc-nand",
	.id		= 1,
	.dev.platform_data = NULL,
	.num_resources	= 1,
	.resource	= (struct resource[]) {
		{
			.start	= 0xB8000000,
			.end	= 0xB80FFFFF,
			.flags	= IORESOURCE_MEM,
		},
	},
};
static struct platform_device *nand_devices[] __initdata = {
	&soc_nand_dev
};

int add_soc_nand_device(void)
{
	int retVal=0;

	printk("CONFIG_MTD_NAND_SOC1 platform_add_devices \n");
        printk("     correct startup problem!!\n");
	retVal = platform_add_devices(nand_devices, 1);

	return retVal;
}
#endif

void __init prom_meminit(void)
{
	unsigned memsize = 0x2000000; // default 32 MB
	unsigned base;
	unsigned size;
	unsigned type;
	char *ptr;
	char cmdline[CL_SIZE];
	int len;

	len = strlen(arcs_cmdline) + 1;
	printk("AuD Soc prom_meminit (cmdlen=%d)\n", len);
	if (len > CL_SIZE) {
		printk("problem prom_meminit: cmdline too long (max=%d)\n",
			CL_SIZE);
		len = CL_SIZE;
	}

	/* Check the command line for a memsize directive that overrides
	   the physical/default amount */
	strncpy(cmdline, arcs_cmdline, len);
	cmdline[CL_SIZE - 1] = '\0';
	ptr = strstr(cmdline, "memsize=");
	if (ptr && (ptr != cmdline) && (*(ptr - 1) != ' '))
		ptr = strstr(ptr, " memsize=");

	if (ptr)
		memsize = memparse(ptr + 8, &ptr);

	printk("    memsize=%#x\n", memsize);
    
	base = 0;
	size = 0x1000;
	type = BOOT_MEM_RESERVED;  // BOOT_MEM_ROM_DATA
	printk("    add mem base=%#x size=%#x type=%#x\n",
		base, size, type);
	add_memory_region(base, size, type);

	// TBD : reserve for PCI memory???

	base = CPHYSADDR(PFN_ALIGN(&_end));
        size = memsize - base;
	type = BOOT_MEM_RAM;
	printk("    add mem base=%#x size=%#x type=%#x\n",
		base, size, type);
	add_memory_region(base, size, type);

}

void __init prom_free_prom_memory(void)
{
	printk("AuD Soc prom_free_prom_memory\n");
}

