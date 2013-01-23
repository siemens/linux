/**
 *   @ingroup hal
 *   @file
 *
 *   Arithmetic/conversion routines for x86.
 *
 *   Original RTAI/x86 HAL services from: \n
 *   Copyright &copy; 2000 Paolo Mantegazza, \n
 *   Copyright &copy; 2000 Steve Papacharalambous, \n
 *   Copyright &copy; 2000 Stuart Hughes, \n
 *   and others.
 *
 *   RTAI/x86 rewrite over Adeos: \n
 *   Copyright &copy; 2002,2003 Philippe Gerum.
 *   Major refactoring for Xenomai: \n
 *   Copyright &copy; 2004,2005 Philippe Gerum.
 *   Arithmetic/conversion routines: \n
 *   Copyright &copy; 2005 Gilles Chanteperdrix.
 *
 *   Xenomai is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU General Public License as
 *   published by the Free Software Foundation, Inc., 675 Mass Ave,
 *   Cambridge MA 02139, USA; either version 2 of the License, or (at
 *   your option) any later version.
 *
 *   Xenomai is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *   General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with Xenomai; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
 *   02111-1307, USA.
 *
 * 	 This file is based on the Xenomai patch.
 *   Adaption to Audis by <wolfgang.hartmann@siemens.com>
 *   adaption to mips by manfred.neugebauer@siemens.com
 */

#ifndef _MIPS_INCLUDE_ASM_RTX_ARITH_32_H
#define _MIPS_INCLUDE_ASM_RTX_ARITH_32_H

#if (0)

/*
 * rtx_divrem_billion is defined in <linux/rtx_time_conv.h>
 * and calls rtx_ulldiv() which calls rtx_generic_ulldiv()
 * (in <linux/rtx_arith.h>)
 * 
 * we may find an optimized routine to provide a faster division
 * by 1000000000 (1 million)
 * for now we use the subroutine defined in <linux/rtx_arith.h>
 * __rtx_generic_ulldiv()
 */
static inline unsigned long long
__rtx_mips_ulldiv (const unsigned long long ull,
                     const unsigned d,
                     unsigned long *const rp)
{
    unsigned long h, l;
    //__rtx_u64tou32(ull, h, l);
    //return __rtx_div96by32(h, l, d, rp);
    return 1;
}
#define rtx_ulldiv(ull,d,rp) __rtx_mips_ulldiv((ull),(d),(rp))

#endif

#include <linux/rtx_arith.h>

#endif /* _MIPS_INCLUDE_ASM_RTX_ARITH_32_H */
