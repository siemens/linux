/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2012 Manfred.Neugebauer@siemens.com
 */

#define SOC_GPIO_BITS_PER_WORD		32

#define SOC_GPIO_START_ADDR		0x1fa00000
#define SOC_GPIO_LEN			0x0fffff

#define SOC_GPIO_NR			6 

struct soc_gpio_info {
	unsigned gpio_ioctrl;
	unsigned gpio_out;
	unsigned gpio_out_set;
	unsigned gpio_out_clear;
	unsigned gpio_res_dis;
	unsigned gpio_in;
	unsigned gpio_port_mode_l;
	unsigned gpio_port_mode_h;
};

struct soc_gpio_descr {
    struct soc_gpio_info gpioWordInfo[SOC_GPIO_NR];
};

int aud_soc_init_gpio(void);
int aud_soc_enable_gpio_bit(int gpio_nr);
int aud_soc_disable_gpio_bit(int gpio_nr);
int aud_soc_set_gpio_outMode(int gpio_nr);
int aud_soc_set_gpio_inMode(int gpio_nr);
int aud_soc_set_gpio_function(int gpio_nr, int function);
