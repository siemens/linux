/*
 * asm-powerpc/rtx_timer.h
 *
 * Copyright (c) 2007 Siemens AG
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
#ifndef _ASM_POWERPC_RTX_TIMER_H_
#define _ASM_POWERPC_RTX_TIMER_H_

#ifndef __ASSEMBLY__

#include <asm/rtx_base.h>
#include <linux/rtx_time_conv.h>

#ifdef CONFIG_RTX_DOMAIN_HRT

#define RTX_TIMER_TYPE             	unsigned long long
#define HRES_LX_DEVICE_NAME "decrementer"

RTX_TIMER_TYPE hres_getCntValue(void);
void hres_setRefValue(unsigned newValue);

/* Read time stamp counter */
static inline uint64_t rtx_gettime(void)
{
    return(hres_getCntValue());
}

/* Gravity interval in nsec. It is used as threshold for the HW-timer to be armed.
 * This value needs to be adjusted according to the performance of the processor
 * which cannot be derived by the clock speed only.
 */
#define RTX_GRAVITY_INTERVAL	(0)

#define RTX_TIMER_GET           (hres_getCntValue())

#define RTX_TIMER_SET(ns_delay) \
({ \
    int ret = 1;                                                \
    hres_setRefValue(ns_delay); \
	ret; \
})

// RT_JIFFIES are nsec
#define RTX_NSEC_TO_RT_JIFFIES(val)	(val)

/*
* Convert seconds/nsecs to RT jiffies (nsec).
* jiffies: pointer to jiffies
* tpr: timespec pointer
*/
#define RTX_CONVERT_TO_INTERNAL(jiffies, tpr) \
({ \
	unsigned long long ns; \
	ns = (tpr->tv_sec * 1000000000ULL) + tpr->tv_nsec;      \
	*jiffies = ns; \
})


/*
* Convert  RT jiffies (nsec) to seconds/nsecs.
* jiffies: TSC value
* tpr: timespec pointer
*/
#define RTX_CONVERT_TO_EXTERNAL(jiffies, tpr) \
({ \
	unsigned long long ns; \
	unsigned long rem; \
	ns = jiffies; \
	tpr->tv_sec = rtx_divrem_billion(ns, &rem); \
	tpr->tv_nsec = rem; \
})

DECLARE_PER_CPU(RTX_TIMER_TYPE, rt_time_last);

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

#else
#define RTX_TIMER_TYPE             	unsigned long
#define RTX_TIMER_GET				(0)
#define RTX_TIMER_SET(V)			(1)
#define RTX_GRAVITY_INTERVAL			0 /* ms tick -> no gravity */

/* Read time stamp counter */
static inline uint64_t rtx_gettime(void)
{
    return((uint64_t) 1000000 );
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

#endif /* CONFIG_RTX_DOMAIN_HRT */


#else /* __ASSEMBLY__ */

#endif /* __ASSEMBLY__ */

#endif /* _ASM_POWERPC_RTX_TIMER_H_ */
          
