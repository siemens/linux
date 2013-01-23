/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2007 Aurelien Jarno <aurelien@aurel32.net>
 */

#include <linux/ssb/ssb.h>
#include <linux/ssb/ssb_driver_chipcommon.h>
#include <linux/ssb/ssb_driver_extif.h>
#include <asm/mach-bcm53xx/bcm53xx.h>
#include <asm/mach-bcm53xx/gpio.h>

#if (BCM53XX_CHIPCO_GPIO_LINES > BCM53XX_EXTIF_GPIO_LINES)
static DECLARE_BITMAP(gpio_in_use, BCM53XX_CHIPCO_GPIO_LINES);
#else
static DECLARE_BITMAP(gpio_in_use, BCM53XX_EXTIF_GPIO_LINES);
#endif

int gpio_request(unsigned gpio, const char *tag)
{
	if (ssb_chipco_available(&ssb_bcm53xx.chipco) &&
	    ((unsigned)gpio >= BCM53XX_CHIPCO_GPIO_LINES))
		return -EINVAL;

	if (ssb_extif_available(&ssb_bcm53xx.extif) &&
	    ((unsigned)gpio >= BCM53XX_EXTIF_GPIO_LINES))
		return -EINVAL;

	if (test_and_set_bit(gpio, gpio_in_use))
		return -EBUSY;

	return 0;
}
EXPORT_SYMBOL(gpio_request);

void gpio_free(unsigned gpio)
{
	if (ssb_chipco_available(&ssb_bcm53xx.chipco) &&
	    ((unsigned)gpio >= BCM53XX_CHIPCO_GPIO_LINES))
		return;

	if (ssb_extif_available(&ssb_bcm53xx.extif) &&
	    ((unsigned)gpio >= BCM53XX_EXTIF_GPIO_LINES))
		return;

	clear_bit(gpio, gpio_in_use);
}
EXPORT_SYMBOL(gpio_free);

int gpio_to_irq(unsigned gpio)
{
	if (ssb_chipco_available(&ssb_bcm53xx.chipco))
		return ssb_mips_irq(ssb_bcm53xx.chipco.dev) + 2;
	else if (ssb_extif_available(&ssb_bcm53xx.extif))
		return ssb_mips_irq(ssb_bcm53xx.extif.dev) + 2;
	else
		return -EINVAL;
}
EXPORT_SYMBOL_GPL(gpio_to_irq);

