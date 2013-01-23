/*
 * linux/rtx_time_conv.h
 *
 * Copyright (c) 2009 Siemens AG
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
#ifndef _RTX_TIME_CONV_H_
#define _RTX_TIME_CONV_H_
#include <asm/rtx_arith_32.h>

extern unsigned long long clockfreq;

static inline void rtx_init_timeconv(unsigned long long freq)
{
	clockfreq = freq;
#ifdef RTX_HAVE_LLMULSHFT
	rtx_init_llmulshft(1000000000, freq, &tsc_scale, &tsc_shift);
#ifdef RTX_HAVE_NODIV_LLIMD
	rtx_init_u32frac(&tsc_frac, 1 << tsc_shift, tsc_scale);
	rtx_init_u32frac(&bln_frac, 1, 1000000000);
#endif
#endif
}

#ifdef RTX_HAVE_NODIV_LLIMD
static inline unsigned long long rtx_divrem_billion(unsigned long long value,
					 unsigned long *rem)
{
	unsigned long long q;
	unsigned r;

	q = rtx_nodiv_ullimd(value, bln_frac.frac, bln_frac.integ);
	r = value - q * 1000000000;
	if (r >= 1000000000) {
		++q;
		r -= 1000000000;
	}
	*rem = r;
	return q;
}
#else
static inline unsigned long long rtx_divrem_billion(unsigned long long value,
					 unsigned long *rem)
{
	return rtx_ulldiv(value, 1000000000, rem);

}
#endif

static inline long long rtx_tsc_to_ns(long long ticks)
{
#ifdef RTX_HAVE_LLMULSHFT
	return rtx_llmulshft(ticks, tsc_scale, tsc_shift);
#else
	return rtx_llimd(ticks, 1000000000, clockfreq);
#endif
}

static inline long long rtx_ns_to_tsc(long long ns)
{
#ifdef RTX_HAVE_LLMULSHFT
#ifdef RTX_HAVE_NODIV_LLIMD
	return rtx_nodiv_llimd(ns, tsc_frac.frac, tsc_frac.integ);
#else
	return rtx_llimd(ns, 1 << tsc_shift, tsc_scale);
#endif
#else
	return rtx_llimd(ns, (unsigned)clockfreq, 1000000000);
#endif
}

#endif /* _RTX_TIME_CONV_H_ */
