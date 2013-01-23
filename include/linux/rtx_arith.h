/**
 *   @ingroup hal
 *   @file
 *
 *   Generic arithmetic/conversion routines.
 *   Copyright &copy; 2005 Stelian Pop.
 *   Copyright &copy; 2005 Gilles Chanteperdrix.
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, Inc., 675 Mass Ave, Cambridge MA 02139,
 *   USA; either version 2 of the License, or (at your option) any later
 *   version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 * 	 This file is based on the Xenomai patch.
 *   Adaption to Audis by <wolfgang.hartmann@siemens.com>
 *
 */

#ifndef _RTX_ARITH_H
#define _RTX_ARITH_H

#ifdef __KERNEL__
#include <asm/byteorder.h>
#include <asm/div64.h>

#ifdef __BIG_ENDIAN
#define endianstruct struct { unsigned _h; unsigned _l; } _s
#else /* __LITTLE_ENDIAN */
#define endianstruct struct { unsigned _l; unsigned _h; } _s
#endif

#else /* !__KERNEL__ */
#include <stddef.h>
#include <endian.h>

#if __BYTE_ORDER == __BIG_ENDIAN
#define endianstruct struct { unsigned _h; unsigned _l; } _s
#else /* __BYTE_ORDER == __LITTLE_ENDIAN */
#define endianstruct struct { unsigned _l; unsigned _h; } _s
#endif /* __BYTE_ORDER == __LITTLE_ENDIAN */

static inline unsigned __rtx_do_div(unsigned long long *a, unsigned d)
{
	unsigned r = *a % d;
	*a /= d;
	return r;
}

#define do_div(a, d) __rtx_do_div(&(a), (d))

#endif /* !__KERNEL__ */

#ifdef RTX_HAVE_NODIV_LLIMD

/* Representation of a 32 bits fraction. */
typedef struct {
	unsigned long long frac;
	unsigned integ;
} rtx_u32frac_t;

extern unsigned int tsc_scale, tsc_shift;
extern rtx_u32frac_t tsc_frac;
extern rtx_u32frac_t bln_frac;
#endif
extern unsigned long rtx_timer_freq;

#ifndef __rtx_u64tou32
#define __rtx_u64tou32(ull, h, l) ({          \
    union { unsigned long long _ull;            \
    endianstruct;                               \
    } _u;                                       \
    _u._ull = (ull);                            \
    (h) = _u._s._h;                             \
    (l) = _u._s._l;                             \
})
#endif /* !__rtx_u64tou32 */

#ifndef __rtx_u64fromu32
#define __rtx_u64fromu32(h, l) ({             \
    union { unsigned long long _ull;            \
    endianstruct;                               \
    } _u;                                       \
    _u._s._h = (h);                             \
    _u._s._l = (l);                             \
    _u._ull;                                    \
})
#endif /* !__rtx_u64fromu32 */

#ifndef rtx_ullmul
static inline __attribute__((__const__)) unsigned long long
__rtx_generic_ullmul(const unsigned m0, const unsigned m1)
{
    return (unsigned long long) m0 * m1;
}
#define rtx_ullmul(m0,m1) __rtx_generic_ullmul((m0),(m1))
#endif /* !rtx_ullmul */

#ifndef rtx_ulldiv
static inline unsigned long long __rtx_generic_ulldiv (unsigned long long ull,
                                                         const unsigned uld,
                                                         unsigned long *const rp)
{
    const unsigned r = do_div(ull, uld);

    if (rp)
        *rp = r;

    return ull;
}
#define rtx_ulldiv(ull,uld,rp) __rtx_generic_ulldiv((ull),(uld),(rp))
#endif /* !rtx_ulldiv */

#ifndef rtx_uldivrem
#define rtx_uldivrem(ull,ul,rp) ((unsigned) rtx_ulldiv((ull),(ul),(rp)))
#endif /* !rtx_uldivrem */


#ifndef rtx_imuldiv_ceil
static inline __attribute__((__const__)) int __rtx_generic_imuldiv_ceil(int i,
									  int mult,
									  int div)
{
	/* Same as __rtx_generic_imuldiv, rounding up. */
	const unsigned long long ull = rtx_ullmul(i, mult);
	return rtx_uldivrem(ull + (unsigned)div - 1, div, NULL);
}
#define rtx_imuldiv_ceil(i,m,d) __rtx_generic_imuldiv_ceil((i),(m),(d))
#endif /* !rtx_imuldiv_ceil */

/* Division of an unsigned 96 bits ((h << 32) + l) by an unsigned 32 bits. 
   Building block for llimd. Without const qualifiers, gcc reload registers
   after each call to uldivrem. */
static inline unsigned long long
__rtx_generic_div96by32 (const unsigned long long h,
                           const unsigned l,
                           const unsigned d,
                           unsigned long *const rp)
{
    unsigned long rh;
    const unsigned qh = rtx_uldivrem(h, d, &rh);
    const unsigned long long t = __rtx_u64fromu32(rh, l);
    const unsigned ql = rtx_uldivrem(t, d, rp);

    return __rtx_u64fromu32(qh, ql);
}

#ifndef rtx_llimd
static inline __attribute__((__const__))
unsigned long long __rtx_generic_ullimd (const unsigned long long op,
                                           const unsigned m,
                                           const unsigned d)
{
    unsigned oph, opl, tlh, tll;
    unsigned long long th, tl;
    
    __rtx_u64tou32(op, oph, opl);
    tl = rtx_ullmul(opl, m);
    __rtx_u64tou32(tl, tlh, tll);
    th = rtx_ullmul(oph, m);
    th += tlh;

    return __rtx_generic_div96by32(th, tll, d, NULL);
}

static inline __attribute__((__const__)) long long
__rtx_generic_llimd (long long op, unsigned m, unsigned d)
{
	long long ret;
	int sign = 0;

	if(op < 0LL) {
		sign = 1;
		op = -op;
	}
        ret = __rtx_generic_ullimd(op, m, d);

	return sign ? -ret : ret;
}
#define rtx_llimd(ll,m,d) __rtx_generic_llimd((ll),(m),(d))
#endif /* !rtx_llimd */

#ifndef __rtx_u96shift
#define __rtx_u96shift(h, m, l, s) ({		\
	unsigned _l = (l);			\
	unsigned _m = (m);			\
	unsigned _s = (s);			\
	_l >>= _s;				\
	_l |= (_m << (32 - _s));		\
	_m >>= _s;				\
	_m |= ((h) << (32 - _s));		\
        __rtx_u64fromu32(_m, _l);		\
})
#endif /* !__rtx_u96shift */

static inline long long rtx_llmi(int i, int j)
{
	/* Signed fast 32x32->64 multiplication */
	return (long long) i * j;
}

#ifndef rtx_llmulshft
/* Fast scaled-math-based replacement for long long multiply-divide */
static inline long long
__rtx_generic_llmulshft(const long long op,
			  const unsigned m,
			  const unsigned s)
{
	unsigned oph, opl, tlh, tll, thh, thl;
	unsigned long long th, tl;

	__rtx_u64tou32(op, oph, opl);
	tl = rtx_ullmul(opl, m);
	__rtx_u64tou32(tl, tlh, tll);
	th = rtx_llmi(oph, m);
	th += tlh;
	__rtx_u64tou32(th, thh, thl);

	return __rtx_u96shift(thh, thl, tll, s);
}
#define rtx_llmulshft(ll, m, s) __rtx_generic_llmulshft((ll), (m), (s))
#endif /* !rtx_llmulshft */

#ifdef RTX_HAVE_NODIV_LLIMD

static inline void rtx_init_u32frac(rtx_u32frac_t *const f,
				       const unsigned m,
				       const unsigned d)
{
	/* Avoid clever compiler optimizations to occur when d is
	   known at compile-time. The performance of this function is
	   not critical since it is only called at init time. */
	volatile unsigned vol_d = d;
	f->integ = m / d;
	f->frac = __rtx_generic_div96by32
		(__rtx_u64fromu32(m % d, 0), 0, vol_d, NULL);
}

#ifndef rtx_nodiv_ullimd

#ifndef __rtx_add96and64
#error "__rtx_add96and64 must be implemented."
#endif

static inline __attribute__((__const__)) unsigned long long
__rtx_mul64by64_high(const unsigned long long op, const unsigned long long m)
{
    /* Compute high 64 bits of multiplication 64 bits x 64 bits. */
    register unsigned long long t0, t1, t2, t3;
    register unsigned oph, opl, mh, ml, t0h, t0l, t1h, t1l, t2h, t2l, t3h, t3l;

    __rtx_u64tou32(op, oph, opl);
    __rtx_u64tou32(m, mh, ml);
    t0 = rtx_ullmul(opl, ml);
    __rtx_u64tou32(t0, t0h, t0l);
    t3 = rtx_ullmul(oph, mh);
    __rtx_u64tou32(t3, t3h, t3l);
    __rtx_add96and64(t3h, t3l, t0h, 0, t0l >> 31);
    t1 = rtx_ullmul(oph, ml);
    __rtx_u64tou32(t1, t1h, t1l);
    __rtx_add96and64(t3h, t3l, t0h, t1h, t1l);
    t2 = rtx_ullmul(opl, mh);
    __rtx_u64tou32(t2, t2h, t2l);
    __rtx_add96and64(t3h, t3l, t0h, t2h, t2l);

    return __rtx_u64fromu32(t3h, t3l);
}


static inline unsigned long long
__rtx_generic_nodiv_ullimd(const unsigned long long op,
			     const unsigned long long frac,
			     unsigned integ)
{
	return __rtx_mul64by64_high(op, frac) + integ * op;
}
#define rtx_nodiv_ullimd(op, f, i)  __rtx_generic_nodiv_ullimd((op),(f), (i))
#endif /* !rtx_nodiv_ullimd */

#ifndef rtx_nodiv_llimd
static inline __attribute__((__const__)) long long
__rtx_generic_nodiv_llimd (long long op, unsigned long long frac, unsigned integ)
{
	long long ret;
	int sign = 0;

	if(op < 0LL) {
		sign = 1;
		op = -op;
	}
        ret = rtx_nodiv_ullimd(op, frac, integ);

	return sign ? -ret : ret;
}
#define rtx_nodiv_llimd(ll,frac,integ) __rtx_generic_nodiv_llimd((ll),(frac),(integ))
#endif /* !rtx_nodiv_llimd */

#endif /* RTX_HAVE_NODIV_LLIMD */

static inline void rtx_init_llmulshft(const unsigned m_in,
					 const unsigned d_in,
					 unsigned *m_out,
					 unsigned *s_out)
{
	/* Avoid clever compiler optimizations to occur when d is
	   known at compile-time. The performance of this function is
	   not critical since it is only called at init time. */
	volatile unsigned vol_d = d_in;
	unsigned long long mult;

	*s_out = 31;
	while (1) {
		mult = ((unsigned long long)m_in) << *s_out;
		do_div(mult, vol_d);
		if (mult <= 0x7FFFFFFF)
			break;
		(*s_out)--;
	}
	*m_out = (unsigned)mult;
}

#endif /* _XENO_ASM_GENERIC_ARITH_H */
