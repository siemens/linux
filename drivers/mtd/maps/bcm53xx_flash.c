
/*
 * drivers/mtd/maps/bcm53xx.c
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <asm/io.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/errno.h>


#define FLASH_BASE_ADDR 0x1FC00000
#define FLASH_BANK_SIZE (4*1024*1024)

MODULE_AUTHOR("anninh.nguyen@siemens.com");
MODULE_DESCRIPTION("User-programmable flash device on the bcm53114 board");
MODULE_LICENSE("GPL");

static struct map_info bcm53xx_map = {
	.name		= "bcm53xx",
	.bankwidth	= 2,
	.size		= FLASH_BANK_SIZE
};

#define BOOTLOADER_PART_SIZE 		(400 * 1024)
#define ENV_PART_SIZE				(32*1024)
#define NVRAM_PART_SIZE				(32*1024)
#define DIM_PART_SIZE				(48*1024)
#define FREESPACE_PART_SIZE			(16*1024)
#define KERNEL_PART_SIZE  			(1024 * 1024)
#define ROOTFS_PART_SIZE  			(1664 * 1024)
#define CONFIG_PART_SIZE  			(256 * 1024)

static struct mtd_partition bcm53xx_partitions[] = {
	{
		name:       "CFE bootloader",
		size:       BOOTLOADER_PART_SIZE,
		offset:     0,
		mask_flags: MTD_WRITEABLE // read-only
	},
	{
		name:       "environment variable for CFE",
		size:       ENV_PART_SIZE,
		offset:     MTDPART_OFS_APPEND,
		mask_flags: MTD_WRITEABLE // read-only
	},
	{
		name:       "nvram",
		size:       NVRAM_PART_SIZE,
		offset:     MTDPART_OFS_APPEND,
		mask_flags: MTD_WRITEABLE // read-only
	},
	{
		name:       "dim",
		size:       DIM_PART_SIZE,
		offset:     MTDPART_OFS_APPEND,
		mask_flags: MTD_WRITEABLE // read-only
	},
//	{
//		name:       "freespace",
//		size:       FREESPACE_PART_SIZE,
//		offset:     MTDPART_OFS_NXTBLK
//	},
//	{
//		name:       "kernel",
//		size:       KERNEL_PART_SIZE,
//		offset:     MTDPART_OFS_APPEND,
//	},
	{
		name:       "rootfs",
		size:       ROOTFS_PART_SIZE,
		offset:     MTDPART_OFS_APPEND
	},
	{
		name:       "CFE config",
		size:       CONFIG_PART_SIZE,
		offset:     MTDPART_OFS_APPEND,
		mask_flags: MTD_WRITEABLE // read-only
	}
};

static struct mtd_info *this_mtd;

static int __init init_bcm53xx(void)
{
	struct mtd_partition *partitions;
	int num_parts = ARRAY_SIZE(bcm53xx_partitions);

	partitions = bcm53xx_partitions;

	bcm53xx_map.virt = ioremap(FLASH_BASE_ADDR, bcm53xx_map.size);

	if (bcm53xx_map.virt == 0) {
		printk("Failed to ioremap FLASH memory area.\n");
		return -EIO;
	}

	simple_map_init(&bcm53xx_map);

	this_mtd = do_map_probe("cfi_probe", &bcm53xx_map);
	if (!this_mtd)
	{
		iounmap((void *)bcm53xx_map.virt);
		return -ENXIO;
	}

	printk(KERN_NOTICE "BCM53XX flash device: %dMiB at 0x%08x\n",
		   this_mtd->size >> 20, FLASH_BASE_ADDR);

	this_mtd->owner = THIS_MODULE;
	add_mtd_partitions(this_mtd, partitions, num_parts);

	return 0;
}

static void __exit cleanup_bcm53xx(void)
{
	if (this_mtd)
	{
		del_mtd_partitions(this_mtd);
		map_destroy(this_mtd);
	}

	if (bcm53xx_map.virt)
	{
		iounmap((void *)bcm53xx_map.virt);
		bcm53xx_map.virt = 0;
	}

	return;
}

module_init(init_bcm53xx);
module_exit(cleanup_bcm53xx);
