/*
 * asm-arm/rtx_timer.h
 *
 * Copyright (c) 2011 Siemens AG
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, Inc., 675 Mass Ave, Cambridge MA 02139,
 * USA; either version 2 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 */
#ifndef _RTX_TIMER_H_
#define _RTX_TIMER_H_

#ifndef __ASSEMBLY__

/* Use optimized scaled math */
#define RTX_HAVE_LLMULSHFT		0
#define RTX_HAVE_NODIV_LLIMD	0
#include <linux/rtx_time_conv.h>

#define RTX_TIMER_TYPE             	unsigned long long
#define RTX_TIMER_TYPE_S            long long

extern unsigned int cpu_khz;

#if defined (CONFIG_ARCH_AT91SAM9G45)
/* The pit is no high resolution timer (clocksource rating = 175) ! */
extern unsigned long rtx_pit_clk_res; // resolution in ns (120 ns)
#endif

#if defined (CONFIG_ARCH_MX6Q)
extern int mx3_set_next_event(	unsigned long evt,
				struct clock_event_device *unused);
extern unsigned long read_mxc_clksrc(void);
extern struct clocksource clocksource_mxc;
extern struct clock_event_device clockevent_mxc;
#define RTX_HIGH_RES_NSEC		(1000000/cpu_khz)
#elif defined (CONFIG_ARCH_AT91) && defined (CONFIG_ATMEL_TCB_CLKSRC)
extern unsigned long rtx_tcb_clk_res; // resolution in ns (60 ns)
extern unsigned long read_tcb_clksrc(void);
extern void set_tcb_next_event(unsigned long delta);
#define RTX_HIGH_RES_NSEC		rtx_tcb_clk_res
#elif defined (CONFIG_ARCH_FEROCEON_KW)
extern struct clock_event_device kw_clkevt;
extern unsigned long get_kw_clksrc(void);
extern int kw_clkevt_next_event(unsigned long delta, struct clock_event_device *dev);
#define RTX_HIGH_RES_NSEC		(1000000/cpu_khz)
#else
#define RTX_HIGH_RES_NSEC		(1000000/cpu_khz)
#endif

#ifdef CONFIG_RTX_DOMAIN_HRT
#define RTX_NSEC_TO_RT_JIFFIES(val)		(val)

#if defined (CONFIG_ARCH_MX6Q)
RTX_TIMER_TYPE hres_getCntValue(struct clocksource *clks);
int hres_setRefValue(unsigned nsDelay, struct clock_event_device *dev);
// ns pro cycle = 121,21212100982666015625
#define ARCH_TIMER_SHIFT 20
#define ARCH_TIMER_MULT  127100121
#define ARCH_TIMER_SHIFT_REV 22
#define ARCH_TIMER_MULT_REV  34603
#define ARCH_TIMER_GET			(read_mxc_clksrc())
#define ARCH_TIMER_SET(delay)	(mx3_set_next_event(delay, NULL))
#define RTX_TIMER_SET(ns_delay) (hres_setRefValue(ns_delay, &clockevent_mxc))
#define RTX_TIMER_GET           (hres_getCntValue(&clocksource_mxc))
//#define RTX_NS2CYC(ns_delay) rtx_imuldiv_ceil(ns_delay, rtx_timer_freq, 1000000000)
#elif defined (CONFIG_ARCH_AT91)
#define RTX_TIMER_GET			((RTX_TIMER_TYPE)(read_tcb_clksrc() * RTX_HIGH_RES_NSEC))
#elif defined (CONFIG_ARCH_FEROCEON_KW)
#define RTX_TIMER_GET			((RTX_TIMER_TYPE)(get_kw_clksrc() * RTX_HIGH_RES_NSEC))
#else
#define RTX_TIMER_GET (1)
#endif

#include <linux/tick.h>

static inline void rtx_timer_program_shot(unsigned long delay)
{
#if defined (CONFIG_ARCH_MX6Q)
	mx3_set_next_event(delay, NULL);
#elif defined (CONFIG_ARCH_AT91)
	set_tcb_next_event(delay);
#elif defined (CONFIG_ARCH_FEROCEON_KW)
	kw_clkevt_next_event(delay,&kw_clkevt);
#else
#endif
}

#ifndef CONFIG_ARCH_MX6Q
#define RTX_TIMER_SET(ns_delay) 							\
({ 											\
	int ret = 1; 									\
	rtx_timer_program_shot(rtx_imuldiv_ceil(ns_delay, rtx_timer_freq, 1000000000)); \
	ret; 										\
})
#endif /* #ifndef CONFIG_ARCH_MX6Q */

/* Gravity interval in nsec. It is used as threshold for the HW-timer to be armed.
 * This value needs to be adjusted according to the performance of the processor
 * which cannot be derived by the clock speed only.
 */
#define RTX_GRAVITY_INTERVAL	(0)

/*
* Convert seconds/nsecs to RT jiffies (nsec).
* jiffies: pointer to jiffies
* tpr: timespec pointer
*/
#define RTX_CONVERT_TO_INTERNAL(jiffies, tpr) \
({ \
	unsigned long long ns; \
	ns = (tpr->tv_sec * 1000000000ULL) + tpr->tv_nsec; \
	*jiffies = ns; \
})

/*
* Convert the internal RT representation (nsec) to seconds/nsecs.
* timeval: internal value (nsec)
* tpr: timespec pointer
*/
#define RTX_CONVERT_TO_EXTERNAL(timeval, tpr) \
({ \
	unsigned long long ns; \
	unsigned long rem; \
	ns = timeval; \
	tpr->tv_sec = rtx_divrem_billion(ns, &rem); \
	tpr->tv_nsec = rem; \
})

#else /* !CONFIG_RTX_DOMAIN_HRT */
#define RTX_TIMER_GET                   (0)
#define RTX_TIMER_SET(V)                (1)
#define RTX_GRAVITY_INTERVAL		0
#endif


DECLARE_PER_CPU(RTX_TIMER_TYPE, rt_time_last);

/* Read time stamp counter */
static inline uint64_t rtx_gettime(void)
{
	uint32_t lo, hi;

	hi = 0;
#if defined (CONFIG_ARCH_MX6Q)
	lo = read_mxc_clksrc();
#elif defined (CONFIG_ATMEL_TCB_CLKSRC)
	lo = read_tcb_clksrc();
#elif defined (CONFIG_ARCH_FEROCEON_KW)
	lo = get_kw_clksrc();
#else
	lo = 10000;
#endif

	return (uint64_t)hi << 32 | lo;
}

/* Convert time stamp counter to usecs */
#define rtx_tsc2us(t) \
({ \
    unsigned long long delta = (t) * rt_timer_res_nsec; \
    do_div(delta, 1000); \
    (unsigned long)delta; \
})

/* Convert time stamp counter to nsecs */
#define rtx_tsc2ns(t) \
({ \
    unsigned long long delta = (t) * rt_timer_res_nsec; \
        (unsigned long)delta; \
})

#endif /* __ASSEMBLY__ */

#endif /* _RTX_TIMER_H_ */
