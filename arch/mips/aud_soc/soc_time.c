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
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <asm/io.h>
#include <asm/time.h>

#include <socGen.h>

#ifdef CONFIG_RTX_DOMAIN

#include <linux/aud/rt_timer.h>

#define PRIVATE_DRIVER_NAME "soc_timer"

extern int __rtx_timer_in_user_code;

#endif

static volatile unsigned myAuDsocJiffy = 0;
static volatile unsigned last_jiffy_val;

volatile struct timertopSocDescr *myTimertopInfo
			= (struct timertopSocDescr *) SOC_TIMERTOP_START;

int initTimer(int Timer, unsigned int Reload, unsigned int Prescaler, unsigned int Mode)
{
	unsigned int tmp;
	unsigned long flags;

	if ((unsigned) Timer >= TIMER_SOC_MAX ) { // unsigned allows faster test
		printk("initTimer(%d): limit of %d soc timers exceeded\n", Timer, TIMER_SOC_MAX);
		return(-EINVAL);
	}
#ifdef DEBUG_SOC_TIMER
	printk("initTimer(%d) (%p:%#x) mode=%#x reload=%d count=%#x\n", Timer,
		&myTimertopInfo->gateTrigCtrl, myTimertopInfo->gateTrigCtrl,
		myTimertopInfo->timer[Timer].mode,
		myTimertopInfo->timer[Timer].loadValue,
		myTimertopInfo->timer[Timer].countValue);
	printk("initTimer(%d)_new: mode=%#x reload=%d pre=%d\n", Timer,
		Mode, Reload, Prescaler);
#endif
	switch (Mode) {
          case TIMER_PERIODIC:
#ifdef CONFIG_RTX_DOMAIN
		local_irq_save_hw(flags);
#else
		local_irq_save(flags);
#endif
		myTimertopInfo->gateTrigCtrl &=  // disable
                    ~(TIMER_CLOCK_ENABLE_BASE << Timer);
		myTimertopInfo->timer[Timer].prescale = Prescaler;
		myTimertopInfo->timer[Timer].loadValue = Reload;
		tmp = myTimertopInfo->timer[Timer].mode = TIMER_MODE_INIT_BIT;
		myTimertopInfo->gateTrigCtrl |= // enable
			((TIMER_CLOCK_ENABLE_BASE << Timer)
			| (TIMER_GATE_ENABLE_BASE << Timer));
#ifdef CONFIG_RTX_DOMAIN
		local_irq_restore_hw(flags);
#else
		local_irq_restore(flags);
#endif
		break;

	case TIMER_ONESHOT:
#ifdef CONFIG_RTX_DOMAIN
		local_irq_save_hw(flags);
#else
		local_irq_save(flags);
#endif
		myTimertopInfo->gateTrigCtrl &=  // disable
		                    ~(TIMER_CLOCK_ENABLE_BASE << Timer);
		myTimertopInfo->timer[Timer].loadValue = Reload;
		myTimertopInfo->timer[Timer].prescale = Prescaler;
		tmp = myTimertopInfo->timer[Timer].mode = (TIMER_MODE_RELOAD_DISABLE | TIMER_MODE_INIT_BIT);
		myTimertopInfo->gateTrigCtrl |= // enable
			((TIMER_CLOCK_ENABLE_BASE << Timer)
			| (TIMER_GATE_ENABLE_BASE << Timer));
#ifdef CONFIG_RTX_DOMAIN
		local_irq_restore_hw(flags);
#else
		local_irq_restore(flags);
#endif
		break;

	default:
		printk("initTimer(%d): invalid mode=%#x\n", Timer, Mode);
		return(-EINVAL);
	}

#ifdef DEBUG_SOC_TIMER
	printk("initTimer(%d)_done: (%p:%#x) mode=%#x reload=%d count=%#x\n", Timer,
		&myTimertopInfo->gateTrigCtrl, myTimertopInfo->gateTrigCtrl,
		myTimertopInfo->timer[Timer].mode,
		myTimertopInfo->timer[Timer].loadValue,
		myTimertopInfo->timer[Timer].countValue);
#endif
	return(0);
}

unsigned getSockTimertopTime(int timerid)
{
	if ((unsigned) timerid >= TIMER_SOC_MAX)  // unsigned: faster test
		return(0);
	return(myTimertopInfo->timer[timerid].countValue);
}

unsigned getActClockTimeForStatistic(void)
{
	return(GET_SOC_TIMERTOP_1_TIME);
}


// SOC1 timer "user" interface

int initTimer_USP(int timerid, unsigned int Reload, unsigned int Prescaler, unsigned int Mode)
{
	// test for useable timer id
	if ((unsigned) timerid < TIMER_SOC_USER_MIN)
		return(-EINVAL);
	return initTimer(timerid, Reload, Prescaler, Mode);
}

unsigned getSockTimertopTime_USP(int timerid)
{
	// test for useable timer id
	if ((unsigned) timerid < TIMER_SOC_USER_MIN)
		return(0);
	return getSockTimertopTime(timerid);
}

int stopTimer_USP(int timerid, int *remaining)
{
	unsigned long flags;

	// test for useable timer id
	if ((unsigned) timerid < TIMER_SOC_USER_MIN)
		return(-EINVAL);
	if ((unsigned) timerid >= TIMER_SOC_MAX)  // unsigned: faster test
		return(-EINVAL);

#ifdef CONFIG_RTX_DOMAIN
	local_irq_save_hw(flags);
#else
	local_irq_save(flags);
#endif
	if ((myTimertopInfo->gateTrigCtrl &
			((TIMER_CLOCK_ENABLE_BASE << timerid) | (TIMER_GATE_ENABLE_BASE << timerid)))
			== 0) {
#ifdef CONFIG_RTX_DOMAIN
		local_irq_restore_hw(flags);
#else
		local_irq_restore(flags);
#endif

#ifdef DEBUG_SOC_TIMER
		printk("stopTimer_USP(%d): timer already stopped mode=%#x\n",
			timerid, myTimertopInfo->timer[timerid].mode);
#endif
		if (remaining)
			*remaining = 0;
		return(-EINVAL);
	}

	myTimertopInfo->gateTrigCtrl &=  // stop the timer
			                    ~((TIMER_CLOCK_ENABLE_BASE << timerid) |
			                      (TIMER_GATE_ENABLE_BASE << timerid));
#ifdef CONFIG_RTX_DOMAIN
	local_irq_restore_hw(flags);
#else
	local_irq_restore(flags);
#endif

	//save old value
	if (remaining) {
		*remaining = myTimertopInfo->timer[timerid].countValue;
#ifdef DEBUG_SOC_TIMER
		printk("stopTimer_USP(%d): stopping timer rem:%d\n",timerid, *remaining);
#endif
	}
#ifdef DEBUG_SOC_TIMER
	else {
		printk("stopTimer_USP(%d): stopping timer\n",timerid);
	}
#endif

	return (1);
}

EXPORT_SYMBOL(initTimer_USP);
EXPORT_SYMBOL(getSockTimertopTime_USP);
EXPORT_SYMBOL(stopTimer_USP);

/*
 * issue 1: hrtimer use xtime_lock so we can't allow this lock
 *          when calling update_process_times();
 * issue 2: posix timers need a interrupt lock else we have a bug
 */
static irqreturn_t mips_sock_timer_interrupt(int irq, void *dev_id)
{
    unsigned long flags1, flags2;
	irqreturn_t r = IRQ_HANDLED;

	myAuDsocJiffy++;
        if (0) {
	// if (myAuDsocJiffy == 1000) {
	// if ((myAuDsocJiffy % 5000) == 1) {
		printk("enter mips_sock_timer irq(%u:%lu)\n", myAuDsocJiffy, jiffies);
	}

#ifndef CONFIG_SELECT_MIPS_COMPARE_TIMEBASE
	local_irq_save(flags1);  // for posix_timer handling
        // we need this sequence lock, to enable task to get a time value
        // within one sequence
        // without having a interrupt lock.
	write_seqlock_irqsave(&xtime_lock, flags2);
	// do_timer() is the generic time accounting subroutine
        // in linux/kernel/timer.c
	do_timer(1); // number of ticks
	write_sequnlock_irqrestore(&xtime_lock, flags2);

	// update_process_times() is the generic timer action subroutine
        // in linux/kernel/timer.c
	update_process_times(__rtx_timer_in_user_code);
	local_irq_restore(flags1);
#endif
        
	return r;
}
    
static void __init mips_soc_timer_setup(struct irqaction *irq)
{
	unsigned myTimeValue;
	int retVal;
	unsigned maxCount = 1000, actCount = 0;

	printk("mips_soc_timer_setup\n");

	if ((retVal = initTimer(SOC_RT_CLOCK_NR, JIFFY_DISTANCE, 0, TIMER_PERIODIC)) != 0)
		printk("mips_soc: problem with timer0 %d\n", retVal);
	/* test for working timer */
	myTimeValue = GET_SOC_TIMERTOP_0_TIME;
	printk("mips_soc: timer0 base value =%#x\n", myTimeValue);
	while (--maxCount) {
		if (myTimeValue != GET_SOC_TIMERTOP_0_TIME) {
			myTimeValue = GET_SOC_TIMERTOP_0_TIME;
			printk("mips_soc: timer0 changed to %#x\n", myTimeValue);
			break;
		}
	}
	if (!maxCount) {
		printk("mips_soc timer_init: timer0 is not running (%#x)\n",
				GET_SOC_TIMERTOP_0_TIME);
	}

	irq->handler = mips_sock_timer_interrupt;	/* we use our own handler */
	setup_irq(IRQ_TIMERTOP_0, irq);

#ifdef CONFIG_RTX_DOMAIN
#ifndef CONFIG_SELECT_MIPS_COMPARE_TIMEBASE
	/* Use Timer1 as a free running counter (without interrupt) */
	if ((retVal = initTimer(SOC_RT_COUNTER_NR, 0xffffffff, 0, TIMER_PERIODIC)) != 0)
		printk("mips_soc: problem with timer1 %d\n", retVal);

	myTimeValue = GET_SOC_TIMERTOP_1_TIME;
	maxCount = 1000;
	while (--maxCount) {
		if (myTimeValue != GET_SOC_TIMERTOP_1_TIME) {
			myTimeValue = GET_SOC_TIMERTOP_1_TIME;
			printk("mips_soc: timer1 value changed to %#x\n", myTimeValue);
			break;
		}
	}
	if (!maxCount) {
		printk("mips_soc: timer0 is not running (%#x)\n", GET_SOC_TIMERTOP_1_TIME);
	}
	last_jiffy_val = myTimeValue;
#endif
#endif
	// try to calculate the cpu frequency if not already done in time_init()
	//if(!cpu_khz) {
	if (1) {
		unsigned tsc_start, tsc_end;
		unsigned myOldTimeValue;

restart:
                actCount++;
		// test for restart of timer value
		myOldTimeValue = GET_SOC_TIMERTOP_0_TIME;
		while ((myTimeValue = GET_SOC_TIMERTOP_0_TIME) < myOldTimeValue)
			myOldTimeValue = myTimeValue;
		tsc_start = read_c0_count();
		// test for next tick
		myOldTimeValue = GET_SOC_TIMERTOP_0_TIME;
		while ((myTimeValue = GET_SOC_TIMERTOP_0_TIME) < myOldTimeValue)
			myOldTimeValue = myTimeValue;
		tsc_end = read_c0_count();
                if (tsc_end < tsc_start)
                    goto restart;

		// the counter is increased every other clock
		// hence the cpu clock is double of the calculated value
		// however, ltt likes this value
                // but we show the correct value in the print statement
		mips_hpt_frequency = (tsc_end - tsc_start) * HZ / 1000;
	}
        printk("calculated cpu frequency %d khz (cnt=%d)\n",
               2 * mips_hpt_frequency, actCount);
#ifdef CONFIG_SELECT_MIPS_COMPARE_TIMEBASE
	// we don't need this timer interrupt when using
        // the compare timebase
        // TBD: free this interrupt
#endif
}

static struct irqaction timer_irqaction = {
	.handler = mips_sock_timer_interrupt,
	.flags = 0 /*SA_INTERRUPT*/,
	.name = "timertop_soc",
};

// comes with time_init() from main
void __init plat_time_init(void)
{
	printk("AuD Soc plat_time_init\n");
/*
 * Here we need to calibrate the cycle counter to at least be close.
 */
	printk(KERN_INFO "Calibrating system timer... ");

        mips_soc_timer_setup(&timer_irqaction);

}


#ifdef CONFIG_RTX_DOMAIN

extern int __rtx_timer_irq;

#ifndef CONFIG_RTX_DOMAIN_HRT

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
	printk("%s: tick_start\n", PRIVATE_DRIVER_NAME);
	return 0;
}

static int tick_unregister(void)
{
	printk("%s: tick_unregister\n", PRIVATE_DRIVER_NAME);
	return 0;
}

static int tick_register(void)
{
	printk("%s: tick_register\n", PRIVATE_DRIVER_NAME);
	// Set up for a tick based timing
	default_aud_timer.irq = __rtx_timer_irq = IRQ_TIMERTOP_0;
	default_aud_timer.res_nsec = clock_getres_int(CLOCK_REALTIME);
	return 0;
}


int rtx_timer_init(void)
{
	aud_rt_timer = &default_aud_timer;

	return(0);
}

#endif // not CONFIG_RTX_DOMAIN_HRT

#endif
