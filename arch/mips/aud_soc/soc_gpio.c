/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2012 Manfred.Neugebauer@siemens.com
 */

#include <linux/kernel.h>
#include <linux/module.h>

#include <asm/mach-aud_soc/soc_gpio.h>

static struct soc_gpio_descr *socGpioRef;

// enable a specific gpio bit
int aud_soc_enable_gpio_bit(int gpio_nr)
{
	int index, bitnr;

	index = gpio_nr / SOC_GPIO_BITS_PER_WORD;
	bitnr = gpio_nr % SOC_GPIO_BITS_PER_WORD;

        socGpioRef->gpioWordInfo[index].gpio_out_set = 1 << bitnr;

	return(0);
}

// disable a specific gpio bit
int aud_soc_disable_gpio_bit(int gpio_nr)
{
	int index, bitnr;

	index = gpio_nr / SOC_GPIO_BITS_PER_WORD;
	bitnr = gpio_nr % SOC_GPIO_BITS_PER_WORD;
    
        socGpioRef->gpioWordInfo[index].gpio_out_clear = 1 << bitnr;

	return(0);
}

// use gpio bit for output
int aud_soc_set_gpio_outMode(int gpio_nr)
{
	int index, bitnr;

	index = gpio_nr / SOC_GPIO_BITS_PER_WORD;
	bitnr = gpio_nr % SOC_GPIO_BITS_PER_WORD;
    
        socGpioRef->gpioWordInfo[index].gpio_ioctrl &= ~(1 << bitnr);

	return(0);
}

// use gpio bit for input
int aud_soc_set_gpio_inMode(int gpio_nr)
{
	int index, bitnr;

	index = gpio_nr / SOC_GPIO_BITS_PER_WORD;
	bitnr = gpio_nr % SOC_GPIO_BITS_PER_WORD;
    
        socGpioRef->gpioWordInfo[index].gpio_ioctrl |= (1 << bitnr);

	return(0);
}

// select a gpio function
int aud_soc_set_gpio_function(int gpio_nr, int function)
{
	int index, bitnr;

	index = gpio_nr / SOC_GPIO_BITS_PER_WORD;
	bitnr = gpio_nr % SOC_GPIO_BITS_PER_WORD;
	function &= 0x3;

	if (bitnr >= SOC_GPIO_BITS_PER_WORD / 2) {
		bitnr -= SOC_GPIO_BITS_PER_WORD / 2;
		socGpioRef->gpioWordInfo[index].gpio_port_mode_h
			&= ~(0x3 << 2*bitnr);
		socGpioRef->gpioWordInfo[index].gpio_port_mode_h
			|= (function << 2*bitnr);
        }
	else {
		socGpioRef->gpioWordInfo[index].gpio_port_mode_l
			&= ~(0x3 << 2*bitnr);
		socGpioRef->gpioWordInfo[index].gpio_port_mode_l
			|= (function << 2*bitnr);
	}

	return(0);
}

int aud_soc_init_gpio(void)
{
	socGpioRef = ioremap(SOC_GPIO_START_ADDR, SOC_GPIO_LEN);
	printk("aud_soc_init_gpio ref=%p(%#x:%#x)\n",
		socGpioRef, SOC_GPIO_START_ADDR, SOC_GPIO_LEN);

	return(0);
}

