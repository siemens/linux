/*
 * arch/x86/include/asm/rtx_timer.h
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
 * Parts of this file are based on the adeos-ipipe/Xenomai patch.
 * Adaption to Audis by <wolfgang.hartmann@siemens.com>
 *
 */
#ifndef _X86_RTX_TIMER_H_
#define _X86_RTX_TIMER_H_


#ifndef __ASSEMBLY__

/* Use optimized scaled math */
#define RTX_HAVE_LLMULSHFT		1
#define RTX_HAVE_NODIV_LLIMD	1
#include <linux/rtx_time_conv.h>
#include <linux/hrtimer.h>
#include <asm/apic.h>

/* Provide an architecture dependent time stamp.
 * Note: For x86 the rdtsc may not be synchronized
 * for Multicore CPUs. So make sure that the software
 * using this function gets exclusive ownership of the CPU.
 * Note: Because of the out-of-order execution of Intel
 * CPUs we serialize the rdtsc, but this may take an
 * additional overhead which has to be considered.  */
static notrace inline uint64_t rtx_gettime_serialized(void)
{
	uint64_t x;

	__asm__ __volatile__ ("xorl %%eax,%%eax\n\t"
						"cpuid\n\t"
						::: "%eax", "%ebx", "%ecx", "%edx");
	x = __native_read_tsc();
	return x;
}

/* Provide an architecture dependent time stamp.
 * Note: This is the raw version for x86 (not serialized).
*/
static notrace inline uint64_t rtx_gettime(void)
{
	uint64_t x;

	x = __native_read_tsc();
	return x;
}

#ifdef CONFIG_X86_LOCAL_APIC
/* Convert time stamp counter to usecs for the rt_tracer. */
#define rtx_tsc2us(t)  (((long)rtx_tsc_to_ns(t))/1000)

/* Convert time stamp counter to nsecs */
#define rtx_tsc2ns(t) rtx_tsc_to_ns(t)
#else
/* Convert time stamp counter to usecs for the rt_tracer. */
#define rtx_tsc2us(t)  \
({ \
	unsigned long long delta = (t); \
	do_div(delta, cpu_khz/1000+1); \
	(unsigned long)delta; \
})

/* Convert time stamp counter to nsecs */
#define rtx_tsc2ns(t) \
({ \
	unsigned long long delta = (t)*1000; \
	do_div(delta, cpu_khz/1000+1); \
	(unsigned long)delta; \
})
#endif


/*
 * Audis high resolution timer specific access routines:
 * - The clock source is always based on time stamp counter.
 * - The clock event is the LAPIC timer.
 *
 * RTX_TIMER_GET: Assumption is a single up-/down-counter (32- or 64-bit) which increases/decreases monotonically.
 * RTX_TIMER_SET: Must be a timer or comparator which causes an interrupt when the timer event fires.
 */
#ifdef CONFIG_RTX_DOMAIN_HRT
extern struct aud_timer *aud_rt_timer;
#define RTX_TIMER_TYPE             	unsigned long long
#define RTX_TIMER_TYPE_S			long long


#define RTX_NSEC_TO_RT_JIFFIES(val)		(val)

#define RTX_HIGH_RES_NSEC				1			// take the value from Linux (see HIGH_RES_NSEC)

/*
 * Note: reading before writing just to work around the Pentium
 * APIC double write bug. apic_read() expands to nil whenever
 * CONFIG_X86_GOOD_APIC is set.
 * Note: Even though we may use a 64 bits delay here,
 * we voluntarily limit to 32 bits, 4 billion ticks should
 * be enough for now.
 */
static inline void rtx_timer_program_shot(unsigned long apic_delay)
{
	apic_read(APIC_TMICT);
	apic_write(APIC_TMICT, apic_delay);
}

/* The clock source is the time stamp counter. 
 * Note: This macro must return a time value in nsec. */
#define RTX_TIMER_GET	          	(rtx_tsc_to_ns(rtx_gettime()))

/* The clock event device is the local apic timer. 
 * Note: The time value to be set must be specified in nsec. */
#define RTX_TIMER_SET(ns_delay) \
({ \
	int ret = 1; \
\
	rtx_timer_program_shot(rtx_imuldiv_ceil(ns_delay, rtx_timer_freq, 1000000000)); \
	ret; \
})

/* Gravity interval in nsec. It is used as threshold for the HW-timer to be armed.
 * Since we are using a one-shot timer for each timer request to be armed we don't
 * need a threshold value.
 */
#define RTX_GRAVITY_INTERVAL	(0)

/*
* Convert seconds/nsecs to the internal RT representation (nsec).
* pnsec: pointer to 64-bit entity (nsec)
* tpr: timespec pointer
*/
#define RTX_CONVERT_TO_INTERNAL(pnsec, tpr) \
({ \
	unsigned long long ns; \
	ns = (tpr->tv_sec * 1000000000ULL) + tpr->tv_nsec; \
	*pnsec = ns; \
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
#define RTX_TIMER_TYPE             	unsigned long
#define RTX_TIMER_TYPE_S			long

#define RTX_TIMER_GET				(0)
#define RTX_TIMER_SET(V)			(1)
#define RTX_GRAVITY_INTERVAL		0
#endif

DECLARE_PER_CPU(RTX_TIMER_TYPE, rt_time_last);

#endif /* __ASSEMBLY__ */

#endif /* _X86_RTX_TIMER_H_ */
