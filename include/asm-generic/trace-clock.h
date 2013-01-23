#ifndef _ASM_GENERIC_TRACE_CLOCK_H
#define _ASM_GENERIC_TRACE_CLOCK_H

/*
 * include/asm-generic/trace-clock.h
 *
 * Copyright (C) 2007 - Mathieu Desnoyers (mathieu.desnoyers@polymtl.ca)
 *
 * Generic tracing clock for architectures without TSC.
 */

#include <linux/param.h>	/* For HZ */
#include <asm/atomic.h>

#define TRACE_CLOCK_SHIFT 13
#ifdef CONFIG_ARCH_FEROCEON_KW
#define XCAT_MEM_READ_LE(addr)  ((*((volatile unsigned int*)(addr))))

#define SOCTIMER1_VAL_REG 0xF102031C
#define TIMER1_VAL ~(XCAT_MEM_READ_LE(SOCTIMER1_VAL_REG))

/* i should call the mvXXXX function for Mhz*/
#define TCLK_200MHZ	200000000
#endif

/*
 * Number of hardware clock bits. The higher order bits are expected to be 0.
 * If the hardware clock source has more than 32 bits, the bits higher than the
 * 32nd will be truncated by a cast to a 32 bits unsigned. Range : 1 - 32.
 * (too few bits would be unrealistic though, since we depend on the timer to
 * detect the overflows).
 */
#define TC_HW_BITS			32

/* Expected maximum interrupt latency in ms : 15ms, *2 for security */
#define TC_EXPECTED_INTERRUPT_LATENCY	30

extern atomic_long_t trace_clock_var;

static inline u32 trace_clock_read32(void)
{
#ifdef CONFIG_ARCH_FEROCEON_KW
	return TIMER1_VAL;
#else
	return (u32)atomic_long_add_return(1, &trace_clock_var);
#endif
}

#ifdef CONFIG_HAVE_TRACE_CLOCK_32_TO_64
extern u64 trace_clock_read_synthetic_tsc(void);
extern void get_synthetic_tsc(void);
extern void put_synthetic_tsc(void);

static inline u64 trace_clock_read64(void)
{
	return trace_clock_read_synthetic_tsc();
}
#else
static inline void get_synthetic_tsc(void)
{
}

static inline void put_synthetic_tsc(void)
{
}

static inline u64 trace_clock_read64(void)
{
	return atomic_long_add_return(1, &trace_clock_var);
}
#endif

static inline unsigned int trace_clock_frequency(void)
{
#ifdef CONFIG_ARCH_FEROCEON_KW
	return TCLK_200MHZ;	//number of timer ticks in a second.
#else
	return HZ << TRACE_CLOCK_SHIFT;
#endif
}

static inline u32 trace_clock_freq_scale(void)
{
	return 1;
}

extern void get_trace_clock(void);
extern void put_trace_clock(void);

static inline void set_trace_clock_is_sync(int state)
{
}
#endif /* _ASM_GENERIC_TRACE_CLOCK_H */
