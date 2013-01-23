
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

#include <asm/bootinfo.h>
#include <asm/page.h>
#include <asm/sections.h>

#include <asm/mips-boards/prom.h>

#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>
#include <linux/platform_device.h>
#include <asm/mach-aud_soc/nand.h>

int add_soc_uart_devices(void);
int add_soc_spi_devices(void);
int initSoftwareHandler(void);

#ifdef CONFIG_AUD_SOC1_CP443_5x
// partitions currently passed via kernel command line
static struct mtd_partition cp443_mtd_partitions[] = {
	{
		.name =		"boot",
		.offset =	0x0,
		.size =		0xa0000,
	}, {
		.name =		"cmdl",
		.offset = 	0xa0000,
		.size =		0x20000
	}, {
		.name =		"ovs",
		.offset =	0xc0000,
		.size =		0x40000,
	}, {
		.name =		"kernel",
		.offset =	0x100000,
		.size =		0x400000,
	}, {
		.name =		"root",
		.offset =	0x500000,
		.size =		0x800000,
	}, {
		.name =		"appl",
		.offset =	0xd00000,
		.size =		0x2c0000,
	}, {
		.name =		"aux1",
		.offset = 	0xfc0000,
		.size =		0x20000
	}, {
		.name =		"aux2",
		.offset = 	0xfe0000,
		.size =		0x20000
	}
};

static struct physmap_flash_data cp443_flash_data = {
	.width		= 4,
	.nr_parts	= ARRAY_SIZE(cp443_mtd_partitions),
	.parts		= cp443_mtd_partitions
};
static struct resource cp443_flash_resource = {
	.start		= 0x19000000,
	.end		= 0x19ffffff,
	.flags		= IORESOURCE_MEM
};

static struct platform_device cp443_flash_device = {
	.name		= "physmap-flash",
	.id		= 0,
	.dev		= {
        .platform_data	= &cp443_flash_data,
	},
	.num_resources	= 1,
	.resource	= &cp443_flash_resource,
};

static struct platform_device *cp443_flash_devices[] __initdata = {
	&cp443_flash_device
};

#endif

#ifdef CONFIG_AUD_SOC1_CP1500

static struct mtd_partition cp1500_mtd_partitions[] = {
	{ .name = "boot",                /* mtd0 size=0x200000*/
		  .offset = 0,
		  .size = 2*1024*1024 },   
	{ .name = "empty1",              /* mtd1 size=0x300000*/
		  .offset = 0x00200000,     
		  .size = 3*1024*1024 },                            
	{ .name = "kernel1",             /* mtd2 size=0xB00000*/
		  .offset = 0x00500000,
		  .size = 11*1024*1024 },  
	{ .name = "ramdisk1",            /* mtd3 size=0xC00000*/
		  .offset = 0x01000000,
		  .size = 12*1024*1024 },                       
	{ .name = "kernel2",             /* mtd4 size=0xB00000*/
		  .offset = 0x01C00000, 
		  .size = 11*1024*1024 },
	{ .name = "ramdisk2",            /* mtd5 size=0xC00000*/
		  .offset = 0x02700000,
		  .size = 12*1024*1024 },
	{ .name = "jffs2",               /* mtd6 size=0x880000*/
		  .offset = 0x03300000,
		  .size = 8704*1024 },
	{ .name = "runvars1",            /* mtd7 size=0xA0000*/
		  .offset = 0x03B80000,
		  .size = 640*1024 }, 
	{ .name = "runvars2",            /* mtd8 size=0xA0000*/
		  .offset = 0x03C20000,
		  .size = 640*1024 },
	{ .name = "bootvars",            /* mtd9 size=0xA0000*/
		  .offset = 0x03CC0000,
		  .size = 640*1024 },
	{ .name = "empty2",              /* mtd10 size=0x4100000*/
		  .offset = 0x03D60000,
		  .size = 65*1024*1024 },
	{ .name = "env",                 /* mtd11 size=0xA0000*/
		  .offset = 0x07E60000,
		  .size = 640*1024 },
};

static struct soc_nand_set cp1500_set_info[] = {
    {
	.nr_chips = 1,
	.nr_partitions = ARRAY_SIZE(cp1500_mtd_partitions),
	.name = "cp1500_nand",
	.nr_map = NULL,
	.partitions = cp1500_mtd_partitions,
    }
};

static struct soc_platform_nand cp1500_platform_info = {
	.nr_sets = 1,
	.sets = cp1500_set_info,
};

static struct resource cp1500_flash_resource = {
	.start		= 0xb8000000,
	.end		= 0xb80fffff,
	.flags		= IORESOURCE_MEM
};

static struct platform_device cp1500_flash_device = {
	.name		= "soc-nand",
	.id		= 0,
	.dev		= {
        .platform_data	= &cp1500_platform_info,
	},
	.num_resources	= 1,
	.resource	= &cp1500_flash_resource,
};

static struct platform_device *cp1500_flash_devices[] __initdata = {
	&cp1500_flash_device
};

#endif

#ifdef CONFIG_AUD_SOC1_CP1626

static struct mtd_partition cp1626_mtd_partitions[] = {
	{ .name = "boot",			/* mtd0 size=0x100000*/
		  .offset = 0,
		  .size = 1*1024*1024 },
	{ .name = "kernel",			/* mtd1 size=0x500000*/
		  .offset = 0x00100000,
		  .size = 5*1024*1024 },
	{ .name = "ubi",			/* mtd2 size=0x2000000*/
		  .offset = 0x00600000,
		  .size = 16*1024*1024,
		  .mask_flags =	MTD_WRITEABLE },
	{ .name = "empty",			/* mtd3 size=0x800000*/
		  .offset = 0x01600000,
		  .size = 88*1024*1024 },
};

static struct soc_nand_set cp1626_set_info[] = {
    {
	.nr_chips = 1,
	.nr_partitions = ARRAY_SIZE(cp1626_mtd_partitions),
	.name = "cp1626_nand",
	.nr_map = NULL,
	.partitions = cp1626_mtd_partitions,
    }
};

static struct soc_platform_nand cp1626_platform_info = {
	.nr_sets = 1,
	.sets = cp1626_set_info,
};

static struct resource cp1626_flash_resource = {
	.start		= 0xb8000000,
	.end		= 0xb80fffff,
	.flags		= IORESOURCE_MEM
};

static struct platform_device cp1626_flash_device = {
	.name		= "soc-nand",
	.id		= 0,
	.dev		= {
        .platform_data	= &cp1626_platform_info,
	},
	.num_resources	= 1,
	.resource	= &cp1626_flash_resource,
};

static struct platform_device *cp1626_flash_devices[] __initdata = {
	&cp1626_flash_device
};

#endif

static int __init add_soc_devices(void)
{
	int retVal = 0;

	printk("AuD Soc add_soc_devices\n");

        // we need to delay this up to now
        add_soc_uart_devices();

#ifdef CONFIG_AUD_SOC1_CP1500
	retVal = platform_add_devices(cp1500_flash_devices, 1);
	printk("CONFIG_MTD_SOC1_cp1500 platform_add_devices(ret=%d)\n",
               retVal);
#endif
#ifdef CONFIG_AUD_SOC1_CP1626
	retVal = platform_add_devices(cp1626_flash_devices, 1);
	printk("CONFIG_MTD_SOC1_cp1626 platform_add_devices(ret=%d)\n",
               retVal);
#endif
#ifdef CONFIG_AUD_SOC1_CP342_5
	printk("cp342 platform: no board specific mtd device added(%d)\n",
		retVal);
#endif
#ifdef CONFIG_AUD_SOC1_CP443_5
	//retVal = platform_add_devices(cp443_flash_devices, 1);
	printk("cp443 platform: no board specific mtd device added(%d)\n",
		retVal);
#endif
#ifdef CONFIG_SPI
	add_soc_spi_devices();
#endif
	initSoftwareHandler();

	return(retVal);
}

device_initcall(add_soc_devices);
