/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2008 Herbert.Bernecker@siemens.com
 * Copyright (C) 2011 Manfred.Neugebauer@siemens.com
 * Copyright (C) 2012 Alexander.Kubicki@siemens.com
 * file "socGen.h"
 *
 * general defines for use with SOC IO
 */

#include <param.h>

#ifndef __ASM_MACH_SOC_SOCGEN_H
#define __ASM_MACH_SOC_SOCGEN_H

/******************************************************************************/
#define SOC_GPIO_START          0xBFA00000  /* General Purpose I/O Base Addr */
#define SOC_GPIO_LENGTH         0x00100000  /* 1 MB */
/******************************************************************************/
#define SOC_SCRP_START          0xBFB00000  /* System Control Register Base Addr */
#define SOC_SCRP_LENGTH         0x00100000  /* 1 MB */
/******************************************************************************/
#define SOC_UART1_START         0xBF480000  /* UART1 Base Addr */
#define SOC_UART1_LENGTH        0x00080000  /* 512 KB */
/******************************************************************************/
#define SOC_TIMERTOP_START      0xBF900000  /* TIMERTOP Base Addr */
#define SOC_TIMERTOP_LENGTH     0x00100000  /* 1 MB */
/******************************************************************************/
#define SOC_ICU_START      	0xBE200000  /* ICU Base Addr */
#define SOC_ICU_LENGTH     	0x00100000  /* 1 MB */
/******************************************************************************/
#define SOC_LOCALBUS_START         0xB8000000  /* LOCALBUS Base Addr */
#define SOC_LOCALBUS_LENGTH        0x05100000  /* 81 MB */
/******************************************************************************/

#define IO_WORD(start, offset)	(*(volatile unsigned long *)(((start)+(offset))))

/*---- Timer register ----------------------------------------------------*/

#define TIMER_ONESHOT				1
#define TIMER_PERIODIC				2

#define TIMER_INIT				0x1

#define TIMER_ONEMILLISEC			50000

#define TIMER_SOC_MAX				6
#define SOC_RT_CLOCK_NR				0
#define SOC_RT_COUNTER_NR			1

#define	TIMER_MODE_INIT_BIT			0x01
#define TIMER_MODE_RELOAD_DISABLE		0x20
#define TIMER_CLOCK_ENABLE_BASE			0x40
#define TIMER_GATE_ENABLE_BASE			0x01

#define MSEC_FACTOR				1000/HZ
#define JIFFY_DISTANCE         		(TIMER_ONEMILLISEC * MSEC_FACTOR)

#define TIM_1_COUNT_REG				0x0000002C

#define GET_SOC_TIMERTOP_0_TIME			getSockTimertopTime(SOC_RT_CLOCK_NR)
#define GET_SOC_TIMERTOP_1_TIME			getSockTimertopTime(SOC_RT_COUNTER_NR)
#define IO_TIMER_1_COUNT			IO_WORD(SOC_TIMERTOP_START, TIM_1_COUNT_REG)

// Timer 0 and 1 are used kernel internal (linux timer, rt)
// we can use timers 2 to TIMER_SOC_MAX for use in other drivers
#define TIMER_SOC_USER_MIN	2

struct timertopSocTimer {
	unsigned mode;
	unsigned prescale;
	unsigned loadValue;
	unsigned countValue;
	unsigned internalEvt;
	unsigned externalEvt0;
	unsigned externalEvt1;
	unsigned dummy1;
};
struct timertopSocDescr {
	struct timertopSocTimer timer[TIMER_SOC_MAX];
	unsigned gateTrigCtrl;
	unsigned clockDivider;
	unsigned extGateMux;
	unsigned extEvt1Mux;
	unsigned extEvt2Mux;
	unsigned swEvtTrigger;
};

extern int soc1_aud_timer_init(void);

/*---- Configuration HighResolutionTimer ----------------------------------------------------*/

#ifdef CONFIG_AUD_SOC1_CP342_5

// frequency 150 MHz / 2
// values for division

#define HRT_MULT 4915 
#define HRT_SHIFT 16 

// values for multiply
#define HRT_MULT_REV 873813 
#define HRT_SHIFT_REV 16

#else

// frequency 450 MHz / 2 (e.g. cp1626)
// values for division

#define HRT_MULT 1843 
#define HRT_SHIFT 13 

// values for multiply
#define HRT_MULT_REV 36409 
#define HRT_SHIFT_REV 13 

#endif



/*---- Local bus register ----------------------------------------------------*/
#define REVISION_CODE     0x05000000
#define IO_REVISION_CODE		IO_WORD(SOC_LOCALBUS_START, REVISION_CODE)

#define WAIT_CYCLE_CONFIG 0x05000004
#define IO_WAIT_CYCLE_CONFIG		IO_WORD(SOC_LOCALBUS_START, WAIT_CYCLE_CONFIG)

#define BANK_0_CONFIG     0x05000008
#define IO_BANK_0_CONFIG		IO_WORD(SOC_LOCALBUS_START, BANK_0_CONFIG)

/*---- UART1 ---------------------------------------------------------------- */
/* UART Data register ----------------------- */
#define IO_UARTDR1			IO_WORD(SOC_UART1_START, UARTDR)
/* UART Receive status register */
#define IO_UARTSR1			IO_WORD(SOC_UART1_START, UARTRSR_ECR)
/* UART line control register h-byte -------- */
#define IO_UARTLCR_H1			IO_WORD(SOC_UART1_START, UARTLCR_H)
/* UART line control register m-byte -------- */
#define IO_UARTLCR_M1			IO_WORD(SOC_UART1_START, UARTLCR_M)
/* UART line control register l-byte -------- */
#define IO_UARTLCR_L1			IO_WORD(SOC_UART1_START, UARTLCR_L)
/* UART control register -------------------- */
#define IO_UARTCR1			IO_WORD(SOC_UART1_START, UARTCR)
/* UART flag register ------------------------*/
#define IO_UARTFR1			IO_WORD(SOC_UART1_START, UARTFR)

/*---- GPIO ----------------------------------------------------------------- */
/* Port	Data Direction register ------------- */
#define IO_GPIO_IOCTRL_0		IO_WORD(SOC_GPIO_START, GPIO_IOCTRL_0)
#define IO_GPIO_IOCTRL_1		IO_WORD(SOC_GPIO_START, GPIO_IOCTRL_1)
#define IO_GPIO_IOCTRL_2		IO_WORD(SOC_GPIO_START, GPIO_IOCTRL_2)
#define IO_GPIO_IOCTRL_3		IO_WORD(SOC_GPIO_START, GPIO_IOCTRL_3)
#define IO_GPIO_IOCTRL_4		IO_WORD(SOC_GPIO_START, GPIO_IOCTRL_4)
#define IO_GPIO_IOCTRL_5		IO_WORD(SOC_GPIO_START, GPIO_IOCTRL_5)

/* Port	Data Out register ------------------- */
#define IO_GPIO_OUT_0			IO_WORD(SOC_GPIO_START, GPIO_OUT_0)
#define IO_GPIO_OUT_1			IO_WORD(SOC_GPIO_START, GPIO_OUT_1)
#define IO_GPIO_OUT_2			IO_WORD(SOC_GPIO_START, GPIO_OUT_2)
#define IO_GPIO_OUT_3			IO_WORD(SOC_GPIO_START, GPIO_OUT_3)
#define IO_GPIO_OUT_4			IO_WORD(SOC_GPIO_START, GPIO_OUT_4)
#define IO_GPIO_OUT_5			IO_WORD(SOC_GPIO_START, GPIO_OUT_5)

/* bit selective output register ------------------- */
#define IO_GPIO_OUT_SET_0		IO_WORD(SOC_GPIO_START, GPIO_OUT_SET_0)
#define IO_GPIO_OUT_SET_1		IO_WORD(SOC_GPIO_START, GPIO_OUT_SET_1)
#define IO_GPIO_OUT_SET_2		IO_WORD(SOC_GPIO_START, GPIO_OUT_SET_2)
#define IO_GPIO_OUT_SET_3		IO_WORD(SOC_GPIO_START, GPIO_OUT_SET_3)
#define IO_GPIO_OUT_SET_4		IO_WORD(SOC_GPIO_START, GPIO_OUT_SET_4)
#define IO_GPIO_OUT_SET_5		IO_WORD(SOC_GPIO_START, GPIO_OUT_SET_5)

/* bit selective clear register ------------------- */
#define IO_GPIO_OUT_CLEAR_0		IO_WORD(SOC_GPIO_START, GPIO_OUT_CLEAR_0)
#define IO_GPIO_OUT_CLEAR_1		IO_WORD(SOC_GPIO_START, GPIO_OUT_CLEAR_1)
#define IO_GPIO_OUT_CLEAR_2		IO_WORD(SOC_GPIO_START, GPIO_OUT_CLEAR_2)
#define IO_GPIO_OUT_CLEAR_3		IO_WORD(SOC_GPIO_START, GPIO_OUT_CLEAR_3)
#define IO_GPIO_OUT_CLEAR_4		IO_WORD(SOC_GPIO_START, GPIO_OUT_CLEAR_4)
#define IO_GPIO_OUT_CLEAR_5		IO_WORD(SOC_GPIO_START, GPIO_OUT_CLEAR_5)

/* Port	Data IN register -------------------- */
#define IO_GPIO_IN_0			IO_WORD(SOC_GPIO_START, GPIO_IN_0)
#define IO_GPIO_IN_1			IO_WORD(SOC_GPIO_START, GPIO_IN_1)
#define IO_GPIO_IN_2			IO_WORD(SOC_GPIO_START, GPIO_IN_2)
#define IO_GPIO_IN_3			IO_WORD(SOC_GPIO_START, GPIO_IN_3)
#define IO_GPIO_IN_4			IO_WORD(SOC_GPIO_START, GPIO_IN_4)
#define IO_GPIO_IN_5			IO_WORD(SOC_GPIO_START, GPIO_IN_5)

/* Port l-mode register --------------------- */
#define IO_GPIO_PORT_MODE_0_L		IO_WORD(SOC_GPIO_START, GPIO_PORT_MODE_0_L)
#define IO_GPIO_PORT_MODE_1_L		IO_WORD(SOC_GPIO_START, GPIO_PORT_MODE_1_L)
#define IO_GPIO_PORT_MODE_2_L		IO_WORD(SOC_GPIO_START, GPIO_PORT_MODE_2_L)
#define IO_GPIO_PORT_MODE_3_L		IO_WORD(SOC_GPIO_START, GPIO_PORT_MODE_3_L)
#define IO_GPIO_PORT_MODE_4_L		IO_WORD(SOC_GPIO_START, GPIO_PORT_MODE_4_L)
#define IO_GPIO_PORT_MODE_5_L		IO_WORD(SOC_GPIO_START, GPIO_PORT_MODE_5_L)

/* Port h-mode register --------------------- */
#define IO_GPIO_PORT_MODE_0_H		IO_WORD(SOC_GPIO_START, GPIO_PORT_MODE_0_H)
#define IO_GPIO_PORT_MODE_1_H		IO_WORD(SOC_GPIO_START, GPIO_PORT_MODE_1_H)
#define IO_GPIO_PORT_MODE_2_H		IO_WORD(SOC_GPIO_START, GPIO_PORT_MODE_2_H)
#define IO_GPIO_PORT_MODE_3_H		IO_WORD(SOC_GPIO_START, GPIO_PORT_MODE_3_H)
#define IO_GPIO_PORT_MODE_4_H		IO_WORD(SOC_GPIO_START, GPIO_PORT_MODE_4_H)
#define IO_GPIO_PORT_MODE_5_H		IO_WORD(SOC_GPIO_START, GPIO_PORT_MODE_5_H)

/*--------------------------------------------------------------------------- */

#endif /* __ASM_MACH_SOC_SOCGEN_H */
