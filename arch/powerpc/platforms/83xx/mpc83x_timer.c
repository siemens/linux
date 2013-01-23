/*
 * MPC83xx global timer module support
 *
 * Copyright (c) 2010 manfred.neugebauer@siemens.com
 * Copyright (c) 2007 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 */

/* this is a preliminary implementation: the following things are missing
 *
 * allocate memory for the device description to handle more than one timer module
 * change code to be able to run the two timer which are part of one control word in parallel.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/sysfs.h>
#include <linux/of_platform.h>

#include <asm/io.h>
#include <asm/irq.h>

#define PRIVATE_DRIVER_NAME "mpc83x_timer"

#define GTM_MS_REF 2640

struct gtm_regs {
       u8    cfr1; /* Timer1/2 Configuration  */
       #define CFR1_PCAS 0x80 /* Pair Cascade mode  */
       #define CFR1_BCM  0x40  /* Backward compatible mode  */
       #define CFR1_STP2 0x20 /* Stop timer  */
       #define CFR1_RST2 0x10 /* Reset timer  */
       #define CFR1_GM2  0x08 /* Gate mode for pin 2  */
       #define CFR1_GM1  0x04 /* Gate mode for pin 1  */
       #define CFR1_STP1 0x02 /* Stop timer  */
       #define CFR1_RST1 0x01 /* Reset timer  */
       #define CFR1_RES ~(CFR1_PCAS | CFR1_STP2 | CFR1_RST2 | CFR1_GM2 |\
               CFR1_GM1 | CFR1_STP1 | CFR1_RST1)

       u8    res0[3];
       u8    cfr2; /* Timer3/4 Configuration  */
       #define CFR2_PCAS 0x80 /* Pair Cascade mode  */
       #define CFR2_SCAS 0x40 /* Super Cascade mode  */
       #define CFR2_STP4 0x20 /* Stop timer  */
       #define CFR2_RST4 0x10 /* Reset timer  */
       #define CFR2_GM4  0x08 /* Gate mode for pin 4  */
       #define CFR2_GM3  0x04 /* Gate mode for pin 3  */
       #define CFR2_STP3 0x02 /* Stop timer  */
       #define CFR2_RST3 0x01 /* Reset timer  */

       u8    res1[11];
       u16   mdr1; /* Timer1 Mode Register  */
       #define MDR_SPS  0xff00 /* Secondary Prescaler value (256) */
       #define MDR_CE   0x00c0 /* Capture edge and enable interrupt  */
       #define MDR_OM   0x0020 /* Output mode  */
       #define MDR_ORI  0x0010 /* Output reference interrupt enable  */
       #define MDR_FRR  0x0008 /* Free run/restart  */
       #define MDR_ICLK 0x0002 /* Input clock source for the timer */
       #define MDR_ICLK_DIV16 0x0004
       #define MDR_ITIN 0x0006 /* TIN Input as clock source for the timer */
       #define MDR_GE   0x0001 /* Gate enable  */

       u16   mdr2; /* Timer2 Mode Register  */
       u16   rfr1; /* Timer1 Reference Register  */
       u16   rfr2; /* Timer2 Reference Register  */
       u16   cpr1; /* Timer1 Capture Register  */
       u16   cpr2; /* Timer2 Capture Register  */
       u16   cnr1; /* Timer1 Counter Register  */
       u16   cnr2; /* Timer2 Counter Register  */
       u16   mdr3; /* Timer3 Mode Register  */
       u16   mdr4; /* Timer4 Mode Register  */
       u16   rfr3; /* Timer3 Reference Register  */
       u16   rfr4; /* Timer4 Reference Register  */
       u16   cpr3; /* Timer3 Capture Register  */
       u16   cpr4; /* Timer4 Capture Register  */
       u16   cnr3; /* Timer3 Counter Register  */
       u16   cnr4; /* Timer4 Counter Register  */
       u16   evr1; /* Timer1 Event Register  */
       u16   evr2; /* Timer2 Event Register  */
       u16   evr3; /* Timer3 Event Register  */
       u16   evr4; /* Timer4 Event Register  */
       #define GTEVR_REF 0x0002 /* Output reference event  */
       #define GTEVR_CAP 0x0001 /* Counter Capture event   */
       #define GTEVR_RES ~(EVR_CAP|EVR_REF)

       u16   psr1; /* Timer1 Prescaler Register  */
       u16   psr2; /* Timer2 Prescaler Register  */
       u16   psr3; /* Timer3 Prescaler Register  */
       u16   psr4; /* Timer4 Prescaler Register  */
       #define GTPSR_PPS  0x00FF /* Primary Prescaler Bits (256). */
       #define GTPSR_RES  ~(GTPSR_PPS)
};

struct mpc83x_timer_dev_priv {
       struct of_device *ofdev;
       void *node;
} mpc83x_timer_priv;

static int gtm_irq1, gtm_irq2, gtm_irq3, gtm_irq4;
volatile struct gtm_regs *myTimerRef;
static int mpc83xTimerVerbose = 0;

#ifdef CONFIG_RTX_DOMAIN
#include <linux/aud/rt_timer.h>


extern int __rtx_timer_irq;

#ifdef CONFIG_RTX_DOMAIN_HRT

#if (0)
// we used this when starting with high res timer within realtime:
// we had a separate timer source
// we save this code to be used fo a 64 bit high res timer
static int initTimer1_2(void)
{
        myTimerRef->cfr1 = 0x0; /* reset timer 1 and 2 */
	printk("    cfr:%#x psr=%#x ref=%#x\n", myTimerRef->cfr1, myTimerRef->psr2, myTimerRef->rfr2);
	myTimerRef->cfr1 = CFR1_GM2 /* timer 2 is controlled by TGATE2 (is also GPIO pin!) */
		| CFR1_PCAS /* set to 32 bit mode */
		| CFR1_STP1 /* stop counter clock */
		| CFR1_RST1 /* disable reset */
		| CFR1_STP2 /* stop counter clock */
		| CFR1_RST2; /* disable reset */
	*((unsigned *)&myTimerRef->cnr1) = 0; /* clear counter and prescaler */
	//myTimerRef->cnr2 = 0; /* clear counter and prescaler */
	/* set prescaler */
	/* we have a frequency of 264 Mhz = 25 * 4 * 2640 */
	myTimerRef->psr2 = 24; /* first prescaler: 25 */
	printk("    psr2:%#x mdr2:%#x\n", myTimerRef->psr2, myTimerRef->mdr2);
	myTimerRef->mdr2 = (3 << 8) /* 2nd prescaler: 4 */
		| MDR_ORI /* interrupt */
            /* | MDR_FRR no restart, but continuous operation */
		| MDR_ICLK; /* use system clock */
	printk("    psr2:%#x mdr2:%#x\n", myTimerRef->psr2, myTimerRef->mdr2);
	*((unsigned *)&myTimerRef->rfr1) = 0xffffffff; /* reference counter */
	myTimerRef->evr2 = GTEVR_REF | GTEVR_CAP;  /* clear events */

        return(0);
}

#endif

#else

static int initTimer1(void)
{
	myTimerRef->cfr1 = 0x0; /* reset timer 1 and 2 */
	if (mpc83xTimerVerbose)
		printk("    cfr:%#x psr=%#x ref=%#x\n",
			myTimerRef->cfr1, myTimerRef->psr1, myTimerRef->rfr1);
	myTimerRef->cfr1 = CFR1_GM1 /* timer 1 is controlled by TGATE1 (is also GPIO pin!) */
		| CFR1_STP1 /* stop counter clock */
		| CFR1_RST1; /* disable reset */
	myTimerRef->cnr1 = 0; /* clear counter and prescaler */
	/* set prescaler */
	/* we have a frequency of 264 Mhz = 25 * 4 * 2640 */
	myTimerRef->psr1 = 24; /* first prescaler: 25 */
	if (mpc83xTimerVerbose)
		printk("    psr1:%#x mdr1:%#x\n", myTimerRef->psr1, myTimerRef->mdr1);
	myTimerRef->mdr1 = (3 << 8) /* 2nd prescaler: 4 */
		| MDR_ORI /* interrupt */
		| MDR_FRR /* restart */
		| MDR_ICLK; /* use system clock */
	if (mpc83xTimerVerbose)
		printk("    psr1:%#x mdr1:%#x\n", myTimerRef->psr1, myTimerRef->mdr1);
	myTimerRef->rfr1 = GTM_MS_REF; /* reference counter */
	myTimerRef->evr1 = GTEVR_REF | GTEVR_CAP;  /* clear events */

        return(0);
}

#endif

#ifdef CONFIG_RTX_DOMAIN_HRT

#else

// this is our implementation of a tick based realtime subsystem
// we use a separate clock source
// since this may run asynchronously to the linux clock
// you should not use this for production systems
// use the high res clock!
static int tick_register(void);
static int tick_start(void);
static int tick_unregister(void);

static struct  aud_timer default_aud_timer = {
	.name = "LEGACY",
	.ops = {
		.rt_register 	= tick_register,
		.rt_start	= tick_start,
		.rt_unregister	= tick_unregister,
	},
};

static int tick_start(void)
{
	if (mpc83xTimerVerbose)
		printkGen(NULL, "%s: tick_start\n", PRIVATE_DRIVER_NAME);

	if (myTimerRef == NULL) {
		printk("%s: problem tick_start: timer_ref not set\n",
			PRIVATE_DRIVER_NAME);
		return(-EINVAL);
	}

        initTimer1();
	return 0;
}

int rtxTimerStart(void)
{
	if (mpc83xTimerVerbose)
		printkGen(NULL, "%s: tick_timerStart\n", PRIVATE_DRIVER_NAME);
	/* start timer 1 by release counter clock */
	myTimerRef->cfr1 = CFR1_GM1 | CFR1_RST1;

	while (0) {
		/* test timer */
            printk("gtm_timer val=%d irq=%d\n",
                   myTimerRef->cnr1, myTimerRef->evr1);
		if (myTimerRef->evr1)
			break;
	}

	return(0);

}

int rtxTimerStop(void)
{
	if (mpc83xTimerVerbose)
		printkGen(NULL, "%s: tick_timerStop\n", PRIVATE_DRIVER_NAME);
	/* reset timer 1/2 */
	myTimerRef->cfr1 = 0x0;

	return(0);
}

static int tick_unregister(void)
{
	if (mpc83xTimerVerbose)
		printk("%s: tick_unregister (%d:%#x)\n", PRIVATE_DRIVER_NAME,
			myTimerRef->cnr1, myTimerRef->evr1);
	return 0;
}

static int tick_register(void)
{
	if (mpc83xTimerVerbose)
		printk("%s: tick_register\n", PRIVATE_DRIVER_NAME);
	// Set up for a tick based timing
	default_aud_timer.irq = __rtx_timer_irq = gtm_irq1;
	default_aud_timer.res_nsec = clock_getres_int(CLOCK_REALTIME);
	return 0;
}

int ack_ppc_timer(void)
{
	myTimerRef->evr1 = GTEVR_REF;

	return(0);
}

int rtx_timer_init(void)
{
	aud_rt_timer = &default_aud_timer;

	return(0);
}

#endif // not CONFIG_RTX_DOMAIN_HRT

#endif // CONFIG_RTX_DOMAIN


/* get the dts description of our device */
/* TBD: do a dynamic allocation of memory to use both timer modules */
static int mpc83x_timer_of_probe(struct of_device *ofdev,
		const struct of_device_id *match)
{
	struct resource res;
        char *helpPtr;

	if (mpc83xTimerVerbose)
		printk("%s: of_probe\n",  PRIVATE_DRIVER_NAME);

	mpc83x_timer_priv.ofdev = ofdev;
	mpc83x_timer_priv.node = ofdev->node;

	if (of_address_to_resource(mpc83x_timer_priv.node, 0, &res)) {
		printk("%s: of_probe problem parse address\n", PRIVATE_DRIVER_NAME);
		return(-EINVAL);
	}
	printk("%s: used resource %#x to %#x\n", PRIVATE_DRIVER_NAME, res.start, res.end);

	helpPtr = ioremap(res.start & 0xfffff000, 0x1000);
        myTimerRef = (void *)(helpPtr + (res.start & 0x00000fff));
        printk("%s: used ptr %p\n", PRIVATE_DRIVER_NAME, myTimerRef);

        gtm_irq1 = irq_of_parse_and_map(mpc83x_timer_priv.node, 0);
        gtm_irq2 = irq_of_parse_and_map(mpc83x_timer_priv.node, 1);
        gtm_irq3 = irq_of_parse_and_map(mpc83x_timer_priv.node, 2);
        gtm_irq4 = irq_of_parse_and_map(mpc83x_timer_priv.node, 3);
	if ((gtm_irq1 == NO_IRQ) || (gtm_irq2 == NO_IRQ)) {
		printk("%s: of_probe problem parse irq\n",  PRIVATE_DRIVER_NAME);
		return(-EINVAL);
	}
	if (mpc83xTimerVerbose)
		printk("%s: used interrupt: %d:%d\n", PRIVATE_DRIVER_NAME,
			gtm_irq1, gtm_irq2);

	return 0;
}

static int mpc83x_timer_of_remove(struct of_device *ofdev)
{
	if (mpc83xTimerVerbose)
		printk("%s: of_remove\n",  PRIVATE_DRIVER_NAME);
	return 0;
}

static struct of_device_id mpc83x_timer_of_match[] =
{
	{
		.compatible = "global-timer-module",
	},
	{},
};

/* Structure for a of device driver */
static struct of_platform_driver mpc83x_timer_of_driver = {
	.name = "mpc83x_timer",
	.match_table = mpc83x_timer_of_match,
	.probe = mpc83x_timer_of_probe,
	.remove = mpc83x_timer_of_remove,
};

static int mpc83x_timer_init(void)
{
	int result = 0;

	printk("%s: init...\n", PRIVATE_DRIVER_NAME);

	of_register_platform_driver(&mpc83x_timer_of_driver);

        return(result);
}


static void mpc83x_timer_exit(void)
{
	printk("%s: exit\n", PRIVATE_DRIVER_NAME);
    
	of_unregister_platform_driver(&mpc83x_timer_of_driver);

}

module_init(mpc83x_timer_init);
module_exit(mpc83x_timer_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("mpc83x_timer driver");

