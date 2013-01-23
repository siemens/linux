/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2003 by Ralf Baechle
 *
 * adjusted by Manfred.Neugebauer@siemens.com for use with soc_1
 * adjusted by Alexander.Kubicki@siemens.com for use with soc_1
 */
#ifndef __ASM_MACH_AUD_SOC_IRQ_H
#define __ASM_MACH_AUD_SOC_IRQ_H

#define SOC1_AUD_INTR_BASE		0
#define NR_DIRECT_IRQ			8

#define MIPS_CPU_IRQ_BASE		SOC1_AUD_INTR_BASE

#define SOC1_IRQ_FAST_DIRECT_BASE	(SOC1_AUD_INTR_BASE + NR_DIRECT_IRQ)
#define NR_FAST_DIRECT_IRQ		NR_DIRECT_IRQ

#define SOC1_IRQ_RESERVE_BASE_1		(SOC1_IRQ_FAST_DIRECT_BASE + NR_FAST_DIRECT_IRQ)
#define NR_IRQ_RESERVE_1		16

// should be 32
#define SOC1_ICU_BASE			(SOC1_IRQ_RESERVE_BASE_1 + NR_IRQ_RESERVE_1)

#define NR_IRQ_ICU			160
#define IRQ_SOFTWARE_0			(SOC1_ICU_BASE + 2)
#define IRQ_SOFTWARE_1			(SOC1_ICU_BASE + 3)
#define IRQ_CPU_COMPARE			(SOC1_ICU_BASE + 4)
#define IRQ0_SP				(SOC1_ICU_BASE + 8)
#define IRQ1_SP				(SOC1_ICU_BASE + 9)
#define IRQ_PROFIBUS_PB1		(SOC1_ICU_BASE + 17)
#define IRQ_PROFIBUS_PB2		(SOC1_ICU_BASE + 18)
#define IRQ_PROFIBUS_PLL		(SOC1_ICU_BASE + 19)
#define IRQ_PROFIBUS_BREAK_K1		(SOC1_ICU_BASE + 20)
#define IRQ_PROFIBUS_BREAK_K2		(SOC1_ICU_BASE + 21)
#define IRQ_TIMERTOP_0			(SOC1_ICU_BASE + 22)
#define IRQ_SERIAL_0			(SOC1_ICU_BASE + 30)
#define IRQ_SERIAL_1			(SOC1_ICU_BASE + 31)
#define IRQ_SERIAL_ERR_0		(SOC1_ICU_BASE + 32)
#define IRQ_SERIAL_ERR_1		(SOC1_ICU_BASE + 33)

#define IRQ_PCI_INTA			(SOC1_ICU_BASE + 91)
#define IRQ_PCI_INTB			(SOC1_ICU_BASE + 92)

#define IRQ_SPI_ETHERNET		(SOC1_ICU_BASE + 111)

#define IRQ_PBUS_PLUS_INTH		(SOC1_ICU_BASE + 115)
#define IRQ_PROFIBUS_X1_BUS3		(SOC1_ICU_BASE + 115)
#define IRQ_PBUS_PLUS_INTL		(SOC1_ICU_BASE + 116)
#define IRQ_PROFIBUS_SYSINIT		(SOC1_ICU_BASE + 116)
#define IRQ_GPIO158			(SOC1_ICU_BASE + 121)

// default icu internal priorities for specific interrupts
#define IRQ_PROFIBUS_BREAK_K1_ICU_PRIO		20
#define IRQ_PROFIBUS_BREAK_K2_ICU_PRIO		20

#define IRQ_PROFIBUS_X1_BUS3_ICU_PRIO		30
#define IRQ_PROFIBUS_SYSINIT_ICU_PRIO		30

#define IRQ_PBUS_PLUS_INTH_ICU_PRIO		40
#define IRQ_PROFIBUS_PB1_ICU_PRIO		40
#define IRQ_PROFIBUS_PB2_ICU_PRIO		40
#define IRQ_PROFIBUS_PLL_ICU_PRIO		40

#define IRQ_TIMERTOP_0_ICU_PRIO			55

#define IRQ_PBUS_PLUS_INTL_ICU_PRIO		70

#define IRQ_SP0_ICU_PRIO			80
#define IRQ_SP1_ICU_PRIO			80

#define IRQ_SERIAL_0_ICU_PRIO			90
#define IRQ_SERIAL_1_ICU_PRIO			90

// should be 192
#define SOC1_IRQ_RESERVE_BASE_2		(SOC1_ICU_BASE + NR_IRQ_ICU)
#define NR_IRQ_RESERVE_2		16

#define NR_IRQS				(NR_DIRECT_IRQ + NR_FAST_DIRECT_IRQ \
						+ NR_IRQ_RESERVE_1 \
						+ NR_IRQ_ICU \
						 + NR_IRQ_RESERVE_2)


int set_irq_icu_edge(int irq_nr, int edge);


#endif /* __ASM_MACH_AUD_SOC_IRQ_H */
