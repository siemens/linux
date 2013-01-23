/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) Siemens AG A&D ATS 13 Franz Goetz
 *
 * file "soc_uart.h"
 */

#ifndef __ASM_MACH_SOC_SOC_UART_H
#define __ASM_MACH_SOC_SOC_UART_H

/*--- #define-Konstanten und Makros ------------------------------------------*/

/******************************************************************************/
#define UARTDR			0x00		/* UART Data register ----------------------- */
#define RX_TX_DATA		0x000000ff  	/* Rx / Tx data */

/******************************************************************************/
#define UARTRSR_ECR		0x04		/* UART Receive status / error clear regsiter */
#define FRMERR			0x00000001  	/* framing error */
#define PARERR			0x00000002  	/* parity error */
#define BKERR			0x00000004  	/* Break error */
#define OVERR			0x00000008  	/* FIFO overrun error */
#define CLRERR			0x000000ff	/* clear error */

/******************************************************************************/
#define UARTLCR_H		0x08		/* UART line control register h-byte -------- */
#define BREAK			0x00000001  	/* set Tx high */
#define PRTEN			0x00000002  	/* parity enable */
#define EVENPRT 		0x00000004  	/* even parity */
#define XSTOP			0x00000008  	/* two stop bit */
#define FIFOEN  		0x00000010  	/* enable FIFO */
#define WRDLEN			0x00000060  	/* Word length */
#define WRDLEN_SHIFT		5
#define WL_5	    		0x0	    	/*   5 bits */
#define WL_6	    		0x1	    	/*   6 bits */
#define WL_7	    		0x2	    	/*   7 bits */
#define WL_8	    		0x3	    	/*   8 bits */

/******************************************************************************/
#define UARTLCR_M		0x0c		/* UART line control register m-byte -------- */
#define BRDIV			0x000000ff  	/* Bit rate divisor */

/******************************************************************************/
#define UARTLCR_L		0x10		/* UART line control register l-byte -------- */
#define BR_115200    		0x01		/* uartclk 3.6864 MHz */
#define BR_57600     		0x03
#define BR_38400     		0x05
#define BR_19200     		0x0b
#define BR_9600      		0x17
#define BR_2400      		0x5f
#define BR_1200      		0x6e

/******************************************************************************/
#define UARTCR			0x14	/* UART control register -------------------- */
#define UARTEN			0x00000001  /* UART enable */
#define SIREN			0x00000002 	/* IRDA SIR endec */
#define SIRLP			0x00000004	/* IRDA low power mode */
#define MSIE			0x00000008	/* modem status interrupt */
#define RIE				0x00000010	/* receive interrupt */
#define TIE				0x00000020	/* transmit interrupt */
#define RTIE			0x00000040	/* receive timiout interrupt */
#define LBE				0x00000080	/* loop back enable */

/******************************************************************************/
#define UARTFR			0x18	/* UART flag register ------------------------*/
#define CTS				0x00000001  /* clear to send */
#define DSR				0x00000002  /* data set ready */
#define DCD				0x00000004  /* data carrier detect */
#define UBUSY			0x00000008  /* UART busy */
#define URXFE			0x00000010  /* Rx FIFO empty */
#define UTXFF			0x00000020  /* Tx FIFO full */
#define URXFF			0x00000040  /* Rx FIFO full */
#define UTXFE			0x00000080  /* Tx FIFO empty */

/******************************************************************************/
#define	UARTIIR_ICR		0x1c	/* UART interrupt ident / clear register ---- */
#define MIS				0x00000001	/* modem status interrupt */
#define RIS				0x00000002	/* receive interrupt status */
#define	TIS				0x00000004	/* trasmit interrupt status */
#define CLRMSI			0x000000ff	/* clear modem status interrupt */

/******************************************************************************/
#define UARTILPR		0x20	/* UART IrDA low-power counter register ----- */
#define ILPDIV			0x000000ff  /* IRDA low power divisor */

#endif /* __ASM_MACH_SOC_SOC_UART_H */

