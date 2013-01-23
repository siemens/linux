/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2012 Manfred.Neugebauer@siemens.com
 * Copyright (C) 2012 Alexander.Kubicki@siemens.com
 */

#include <linux/kernel.h>
#include <linux/module.h>

#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <asm/mach-aud_soc/soc_gpio.h>
#include <asm/mach-aud_soc/soc_spi.h>
#include <asm/mach-aud_soc/irq.h>

#include <asm/mach-aud_soc/soc_pullcontrol.h>
#include <linux/delay.h>

//#define DEBUG

#ifdef CONFIG_SPI

/*
 * aud soc SPI interface
 */
static u64 soc_spi_dmamask = DMA_BIT_MASK(32);

static struct resource soc_spi_resources[] = {
	[0] = {
		.start	= 0x1F600000,
		.end	= 0x1F60FFFF,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= 117,
		.end	= 117,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource soc_pullcontrol_resources[] = {
	[0] = {
		.start	= 0x1FB00A20,  	//the SystemControlRegisterBlock starts at 0x1FB00000, 
					// but pullcontrol is at offset 0xA20 to 0xA38
		.end	= 0x1FBFFFFF,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device aud_soc_spi_bus_device = {
	.name		= "aud_soc_spi",
	.id		= 1,
	.dev		= {
				.dma_mask		= &soc_spi_dmamask,
				.coherent_dma_mask	= DMA_BIT_MASK(32),
	},
	.resource	= soc_spi_resources,
	.num_resources	= ARRAY_SIZE(soc_spi_resources),
};

static struct platform_device *aud_soc_spi_bus_devs[1];

/*
 * SPI devices.
 */
#define SOC_SPI_CLK				58
#define SOC_SPI_MOSI				59
#define SOC_SPI_MISO				60

// **** SOC-GPIO 61 = SOC-Interrupt 111 = Linux-Interrupt 143 ****
#define SOC_SPI_DEV_IRQ				IRQ_SPI_ETHERNET
#define SOC_SPI_DEV_IRQ_GPIO			61

#define SOC_SPI_DEV_CS				89
#define SOC_SPI_DEV_RESET			113



static struct spi_board_info aud_soc_spi_devices[] = {
	{	/* SPI Ethernet extension */
		.modalias	= "ks8851",
		.chip_select	= 0,
		.max_speed_hz	= 15 * 1000 * 1000,
		.bus_num	= 1,
                .irq		= SOC_SPI_DEV_IRQ,
                .platform_data	= NULL,
                .mode		= 0
	},
};

int aud_soc_init_spi(void)
{
	struct soc_spi_info *socSpiRef;
	struct soc_pullcontrol_info *socPullcontrolRef;
	unsigned pc2, pcc = 0x88888888;

	socSpiRef = ioremap(soc_spi_resources[0].start,
		soc_spi_resources[0].end - soc_spi_resources[0].start + 1 );

	printk("aud_soc_init_spi (%p)\n", socSpiRef);

	//memory region for pullcontrol
	socPullcontrolRef = ioremap(soc_pullcontrol_resources[0].start,
		soc_pullcontrol_resources[0].end - soc_pullcontrol_resources[0].start + 1 );
#ifdef DEBUG
	printk("aud_soc_init_spi [pullcontrol](%p)\n", socPullcontrolRef);
#endif

	//readout actual values of pullcontrol_2 and pullcontrol_configuration
	pc2 = socPullcontrolRef->soc_pullcontrol_2;
	pcc = socPullcontrolRef->soc_pullcontrol_configuration;

	//set GPIO_PULL_CONTROL_2, Bit29
	socPullcontrolRef->soc_pullcontrol_2 = (pc2 | 0x20000000 );
	//set GPIO_PULL_CONFIGURATION, Bit2
	socPullcontrolRef->soc_pullcontrol_configuration = (pcc | 0x00000004 );

	//the SOC-documentation requests a certain time period waiting between writing and reading back of registers 
	udelay(10);

	pc2 = socPullcontrolRef->soc_pullcontrol_2;
	pcc = socPullcontrolRef->soc_pullcontrol_configuration;

#ifdef DEBUG
	printk("aud_soc_init_spi [pullcontrol, spi with pullup] pc2: <0x%08x>  pcc: <0x%08x>\n", pc2, pcc);
#endif

	// handle board devices
	// set gpio directions for cs pin
	// T B D
	/* pass chip-select pin to driver */
	aud_soc_spi_devices[0].controller_data =
		(void *) SOC_SPI_DEV_CS;
        // select gpio pin direction for interrupt
        aud_soc_set_gpio_inMode(SOC_SPI_DEV_IRQ_GPIO);
	set_irq_icu_edge(SOC_SPI_DEV_IRQ, 1);



	// select gpio pin direction for chipselect
	aud_soc_set_gpio_outMode(SOC_SPI_DEV_CS);

	// select gpio pin direction for reset
	aud_soc_set_gpio_outMode(SOC_SPI_DEV_RESET);

	// handle spi support for aud soc
	// (re)set spi interface for master mode
	socSpiRef->soc_spi_sspcr1 = 0;
        // select function of gpio pins for SPI support
        aud_soc_set_gpio_function(SOC_SPI_CLK, 1);
        aud_soc_set_gpio_function(SOC_SPI_MOSI, 1);
        aud_soc_set_gpio_function(SOC_SPI_MISO, 1);

	aud_soc_spi_bus_devs[0] = &aud_soc_spi_bus_device;


	return(0);
}

// delay platform registration until memory allocation is available
int add_soc_spi_devices(void)
{

	printk("register board spi bus and devices\n");
	// board devices must be first to be scanned
	// by spi master driver registration
	spi_register_board_info(aud_soc_spi_devices, 1);

        platform_add_devices(aud_soc_spi_bus_devs, 1);

	return(0);
}

#endif
