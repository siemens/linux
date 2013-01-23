/*
 * arch/x86/include/asm/rtx_percpu.h
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
 * Parts of this file are based on the adeos-ipipe patch.
 * Adaption to Audis by <wolfgang.hartmann@siemens.com>
 */
#ifndef _X86_RTX_PERCPU_H_
#define _X86_RTX_PERCPU_H_
#include <linux/rtx_percpu.h>
#include <asm/alternative.h>

/* Note: Assumption is that the stalled bit is defined in the first
 * element of the structure rtx_percpu_data. */

#ifdef CONFIG_SMP
#define GET_IRQ_CONTROL_ADDR \
	"pushfl; cli;" \
	"movl %%fs:per_cpu__this_cpu_off, %%eax;" \
	"lea per_cpu__rtx_percpu_data(%%eax), %%eax;"

#define PUT_IRQ_CONTROL_ADDR 	"popfl;"

static inline void __rtx_set_stalled_bit(void)
{
	__asm__ __volatile__(GET_IRQ_CONTROL_ADDR
						LOCK_PREFIX
						"btsl $0, (%%eax);"
						PUT_IRQ_CONTROL_ADDR
						: : : "eax", "memory");
}

static inline unsigned long __rtx_test_and_set_stalled_bit(void)
{
	int oldbit;

	__asm__ __volatile__(GET_IRQ_CONTROL_ADDR
						LOCK_PREFIX
						"btsl $0,(%%eax);"
						"sbbl %0,%0;"
						PUT_IRQ_CONTROL_ADDR
						:"=r" (oldbit)
						: : "eax", "memory");
	return oldbit;
}

static inline unsigned long __rtx_test_stalled_bit(void)
{
	int oldbit;

	__asm__ __volatile__(GET_IRQ_CONTROL_ADDR
						"btl $0,(%%eax);"
			     	 	"sbbl %0,%0;"
						PUT_IRQ_CONTROL_ADDR
						:"=r" (oldbit)
						: : "eax");
	return oldbit;
}
#else /* !CONFIG_SMP */
static inline void __rtx_set_stalled_bit(void)
{
	volatile struct rtx_pcd *p = &__raw_get_cpu_var(rtx_percpu_data);
	__asm__ __volatile__("btsl $0,%0;"
			     :"+m" (*p) : : "memory");
}

static inline unsigned long __rtx_test_and_set_stalled_bit(void)
{
	volatile struct rtx_pcd *p = &__raw_get_cpu_var(rtx_percpu_data);
	int oldbit;

	__asm__ __volatile__("btsl $0,%1;"
			     "sbbl %0,%0;"
			     :"=r" (oldbit), "+m" (*p)
			     : : "memory");
	return oldbit;
}

static inline unsigned long __rtx_test_stalled_bit(void)
{
	volatile struct rtx_pcd *p = &__raw_get_cpu_var(rtx_percpu_data);
	int oldbit;

	__asm__ __volatile__("btl $0,%1;"
			     "sbbl %0,%0;"
			     :"=r" (oldbit)
			     :"m" (*p));
	return oldbit;
}
#endif
#endif /* _X86_RTX_PERCPU_H_ */
