/* linux/drivers/mtd/nand/soc1Nand.c
 *
 * Herbert.Bernecker@siemens.com
 * derived from linux/drivers/mtd/nand/ertecNand.c
 *
 * SOC1 NAND driver 
 * based on SAMSUNG NAND components
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/err.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>

#include <asm/io.h>
#include <asm/mach-aud_soc/nand.h>
#include <asm/mach-aud_soc/gpio.h>

#ifdef CONFIG_AUD_SOC1_CP342_5
   // GPIO 153
   #define NAND_READY_BUSY_BIT		P25
#endif

#ifdef CONFIG_AUD_SOC1_CP1500
   // GPIO 187
   #define NAND_READY_BUSY_BIT		P27
#endif

#ifdef CONFIG_AUD_SOC1_CP1626
   // GPIO 187
   #define NAND_READY_BUSY_BIT		P27
#endif

#define CLE (unsigned long *)(SOC1_NAND_BASE_ADDR + CLE_ADDRESS)
#define ALE (unsigned long *)(SOC1_NAND_BASE_ADDR + ALE_ADDRESS)

#ifdef CONFIG_MTD_PARTITIONS
const char *part_probes[] = { "cmdlinepart", NULL };
#endif

static int soc_nand_dev_ready_fast(struct mtd_info *mtd)
{
    unsigned long i;
    for ( i = 0 ; i < 1000 ; i++ ) {
       udelay(10);

       /* see, if the gpio is now inactive */
       #ifdef CONFIG_AUD_SOC1_EVALBOARD
       if (IO_GPIO_IN_4 & NAND_READY_BUSY_BIT) return 1;
       #endif

       #ifdef CONFIG_AUD_SOC1_CP342_5
       if (IO_GPIO_IN_4 & NAND_READY_BUSY_BIT) return 1;
       #endif

       #ifdef CONFIG_AUD_SOC1_CP1500
       if (IO_GPIO_IN_5 & NAND_READY_BUSY_BIT) return 1;
       #endif

       #ifdef CONFIG_AUD_SOC1_CP1626
       if (IO_GPIO_IN_5 & NAND_READY_BUSY_BIT) return 1;
       #endif
    }
    return 0;
}

static int soc_nand_dev_ready(struct mtd_info *mtd)
{
#if 1
    unsigned long i;
    for ( i = 0 ; i < 100 ; i++ ) {
       udelay(100);

       /* see, if the gpio is now inactive */
       #ifdef CONFIG_AuD_SOC1_EVALBOARD
       if (IO_GPIO_IN_4 & NAND_READY_BUSY_BIT) return 1;
       #endif

       #ifdef CONFIG_AuD_SOC1_CP342_5
       if (IO_GPIO_IN_4 & NAND_READY_BUSY_BIT) return 1;
       #endif

       #ifdef CONFIG_AUD_SOC1_CP1500
       if (IO_GPIO_IN_5 & NAND_READY_BUSY_BIT) return 1;
       #endif

       #ifdef CONFIG_AUD_SOC1_CP1626
       if (IO_GPIO_IN_5 & NAND_READY_BUSY_BIT) return 1;
       #endif
    }
    return 0;
#else
    /* read the chip's status register */
    struct nand_chip *this = mtd->priv;

    /* see if the status is still busy */
    this->cmdfunc (mtd, NAND_CMD_STATUS, -1, -1);
    unsigned char sts = this->read_byte(mtd);

    if(this->read_byte(mtd) & NAND_STATUS_READY) return 1;
    else				         return 0;
#endif
}

static u_char soc_nand_read_byte(struct mtd_info *mtd)
{
	struct nand_chip *this = mtd->priv;
	return (readb(this->IO_ADDR_R));
}

static unsigned short soc_nand_read_word(struct mtd_info *mtd)
{
	unsigned short value;
	struct nand_chip *this = mtd->priv;
	value = readb(this->IO_ADDR_R);
	value &= (readb(this->IO_ADDR_R) << 8);
	return (value);
}

static void soc_nand_read_buf(struct mtd_info *mtd, u_char *buf, int len)
{
	int i;
	struct nand_chip *this = mtd->priv;

	while (!soc_nand_dev_ready_fast(mtd));
	for (i=0; i<len; i++) {
		buf[i] = readb(this->IO_ADDR_R);
	}
}


static void soc_nand_write_buf(struct mtd_info *mtd, const u_char *buf, int len)
{
	int i;
	struct nand_chip *this = mtd->priv;

	while (!this->dev_ready(mtd));
	for (i=0; i<len; i++)
		writeb(buf[i], this->IO_ADDR_W);
}

static int soc_nand_verify_buf(struct mtd_info *mtd, const u_char *buf, int len)
{
	int i;
	struct nand_chip *this = mtd->priv;

	while (!soc_nand_dev_ready_fast(mtd));
	for (i=0; i<len; i++)
		if (buf[i] != readb(this->IO_ADDR_R))
			return -EFAULT;

	return 0;
}

static void soc_nand_hwcontrol(struct mtd_info *mtd, int cmd,
				   unsigned int ctrl)
{
	if (cmd == NAND_CMD_NONE)
		return;

	if (ctrl & NAND_CLE)
		writeb(cmd, CLE);
	else if (ctrl & NAND_ALE)
		writeb(cmd, ALE);
	else
		printk("-- %s ctrl=0x%x --\n",__FUNCTION__, ctrl);
}

static void soc_select_chip(struct mtd_info *mtd, int chip)
{
//	puts("soc_select_chip");
}

/* device management functions */

static struct soc_nand_info *to_nand_info(struct platform_device *dev)
{
	return platform_get_drvdata(dev);
}

static struct soc_platform_nand *to_nand_plat(struct platform_device *dev)
{
	return dev->dev.platform_data;
}

static int soc_nand_remove(struct platform_device *dev)
{
	struct soc_nand_info *info = to_nand_info(dev);

	printk("soc_nand_remove(%p)\n", dev);

	platform_set_drvdata(dev, NULL);

	if (info == NULL) return 0;

	/* first thing we need to do is release all our mtds
	 * and their partitions, then go through freeing the
	 * resources used
	 */
	if (info->mtds != NULL) {
		struct soc_nand_mtd *ptr = info->mtds;
		int mtdno;

		for (mtdno = 0; mtdno < info->mtd_count; mtdno++, ptr++) {
			pr_debug("releasing mtd %d (%p)\n", mtdno, ptr);
			nand_release(&ptr->mtd);
		}

		kfree(info->mtds);
	}

	/* free the common resources */

	if (info->regs != NULL) {
		iounmap(info->regs);
		info->regs = NULL;
	}

	if (info->area != NULL) {
		release_resource(info->area);
		kfree(info->area);
		info->area = NULL;
	}

	kfree(info);

	return 0;
}


#ifdef CONFIG_MTD_PARTITIONS
static int soc_nand_add_partition(struct soc_nand_info *info,
				      struct soc_nand_mtd *mtd,
				      struct soc_nand_set *set)
{
	int mtd_parts_nb = 0;
	struct mtd_partition *mtd_parts = 0;
	const char *part_type = 0;

        if(strlen(CONFIG_MTD_NAND_SOC1_FLASH_NAME) > 0)
          mtd->mtd.name = CONFIG_MTD_NAND_SOC1_FLASH_NAME;

	printk("soc_nand_add_partition mtd->mtd.name = %s \n",mtd->mtd.name);
	mtd_parts_nb = parse_mtd_partitions(&mtd->mtd, part_probes,
					    &mtd_parts, 0);
	if (mtd_parts_nb > 0)
		part_type = "command line";
	else {
            mtd_parts_nb = 0;
            if (set) {
		mtd_parts = set->partitions;//partition_info;
                    mtd_parts_nb = set->nr_partitions;//NUM_PARTITIONS;
                    }
		part_type = "static";
	}

	/* Register the partitions */
	printk(KERN_NOTICE "Using %s partition definition\n", part_type);
	add_mtd_partitions(&mtd->mtd, mtd_parts, mtd_parts_nb);

	return add_mtd_device(&mtd->mtd);
}
#else
static int soc_nand_add_partition(struct soc_nand_info *info,
				      struct soc_nand_mtd *mtd,
				      struct soc_nand_set *set)
{
	return add_mtd_device(&mtd->mtd);
}
#endif

/* soc_nand_init_chip
 *
 * init a single instance of an chip 
*/

static void soc_nand_init_chip(	struct soc_nand_info *info,
				struct soc_nand_mtd  *nmtd,
				struct soc_nand_set   *set)
{
	struct nand_chip *chip = &nmtd->chip;

	chip->IO_ADDR_R	   = (void  __iomem *)SOC1_NAND_BASE_ADDR;
	chip->IO_ADDR_W    = (void  __iomem *)SOC1_NAND_BASE_ADDR;
	chip->dev_ready    = soc_nand_dev_ready;
	chip->read_byte    = soc_nand_read_byte;
	chip->read_word    = soc_nand_read_word;
	chip->write_buf    = soc_nand_write_buf;
	chip->read_buf     = soc_nand_read_buf;
	chip->verify_buf   = soc_nand_verify_buf;
	chip->cmd_ctrl     = soc_nand_hwcontrol;
	chip->select_chip  = soc_select_chip;
	chip->chip_delay   = 50;
	chip->priv	   = nmtd;
    #ifdef CONFIG_MTD_NAND_SOC1_FLASH_BBT
	chip->options	   = NAND_USE_FLASH_BBT;
    #endif 
	chip->controller   = &info->controller;
	chip->ecc.mode	   = NAND_ECC_SOFT;

	nmtd->info	   = info;
	nmtd->mtd.priv	   = chip;
	nmtd->set	   = set;

    #ifdef CONFIG_AUD_SOC1_EVALBOARD
        IO_GPIO_IOCTRL_4 |= 0x02000000; // GPIO 153 -> INPUT; NAND_RDY_BSY
    #endif

#ifdef CONFIG_AUD_SOC1_CP1500
	IO_GPIO_IOCTRL_5 |= 0x08000000; // GPIO 187 -> INPUT; NAND_RDY_BSY
#endif

    #ifdef CONFIG_AUD_SOC1_CP1626
	IO_GPIO_IOCTRL_5 |= 0x08000000; // GPIO 187 -> INPUT; NAND_RDY_BSY
    #endif
}

/* soc_nand_probe
 *
 * called by device layer when it finds a device matching
 * one our driver can handled. This code checks to see if
 * it can allocate all necessary resources then calls the
 * nand layer to look for devices
*/
static int soc_nand_probe(struct platform_device *dev)
{
	struct soc_platform_nand *plat = to_nand_plat(dev);
	struct soc_nand_info *info;
	struct soc_nand_mtd *nmtd;
	struct soc_nand_set *sets;
	struct resource *res;
	int err = 0;
	int size;
	int nr_sets;
	int setno;

	printk("soc_nand_probe(%p)\n", dev);

	info = kmalloc(sizeof(*info), GFP_KERNEL);
	if (info == NULL) {
		printk("no memory for flash info\n");
		err = -ENOMEM;
		goto exit_error;
	}

	memset((char *) info, 0, sizeof(*info));
	platform_set_drvdata(dev, info);

	spin_lock_init(&info->controller.lock);
	init_waitqueue_head(&info->controller.wq);

	/* allocate and map the resource */

	/* currently we assume we have the one resource */
	res  = dev->resource;
	size = res->end - res->start + 1;

	info->area = request_mem_region(res->start, size, dev->name);

	if (info->area == NULL) {
		printk("cannot reserve register region\n");
		err = -ENOENT;
		goto exit_error;
	}

	info->device     = &dev->dev;
	info->platform   = plat;
	info->regs       = ioremap(res->start, size);

	if (info->regs == NULL) {
		printk("cannot reserve register region\n");
		err = -EIO;
		goto exit_error;
	}

	printk("mapped registers at %p\n", info->regs);

	sets = (plat != NULL) ? plat->sets : NULL;
	nr_sets = (plat != NULL) ? plat->nr_sets : 1;
	printk("plat = %p   sets = %p   nr_sets = %x \n",plat,sets,nr_sets);

	info->mtd_count = nr_sets;

	/* allocate our information */

	size = nr_sets * sizeof(*info->mtds);
	info->mtds = kmalloc(size, GFP_KERNEL);
	if (info->mtds == NULL) {
		printk("failed to allocate mtd storage\n");
		err = -ENOMEM;
		goto exit_error;
	}

	memset((char *) info->mtds, 0, size);

	/* initialise all possible chips */

	nmtd = info->mtds;

	for (setno = 0; setno < nr_sets; setno++, nmtd++) {
		printk("initialising set %d (%p, info %p, sets %p)\n",
			 setno, nmtd, info, sets);

		soc_nand_init_chip(info, nmtd, sets);

		nmtd->scan_res = nand_scan(&nmtd->mtd,
					   (sets) ? sets->nr_chips : 1);

		soc_nand_add_partition(info, nmtd, sets);

		if (sets != NULL)
			sets++;
	}

	printk("initialised ok\n");
	return 0;

 exit_error:
	soc_nand_remove(dev);

	if (err == 0)
		err = -EINVAL;
	return err;
}

static struct platform_driver soc_nand_driver = {
	.probe		= soc_nand_probe,
	.remove		= soc_nand_remove,
	.driver		= {
		.name	= "soc-nand",
		.owner	= THIS_MODULE,
	},
};

static int __init soc_nand_init(void)
{
	int ret;
	printk("SOC NAND Driver, (c) 2006-12 Siemens AD ATS 11\n");
	ret = platform_driver_register(&soc_nand_driver);

	return ret;
}

static void __exit soc_nand_exit(void)
{
	platform_driver_unregister(&soc_nand_driver);
}

module_init(soc_nand_init);
module_exit(soc_nand_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("<Siemens AD ATS 11>");
MODULE_DESCRIPTION("SOC MTD NAND driver");
