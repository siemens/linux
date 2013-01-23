/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * provide a simple serial line functionality
 * file "soc_serial.c"
 * m.n. h.b. Siemens AG, AD ATS 11
 * 30-June-2008
 */

#include <linux/kernel.h>
#include "soc_uart.h"
#include <socGen.h>

// dummy when using soc 1/2
int setGPIOmodeUART(void)
{
	return(0);
}

static unsigned lowCtrl;
static unsigned medCtrl;
static unsigned highCtrl;
static unsigned genCtrl;

#define USE_BAUD_INDEX 0
//#define USE_BAUD_INDEX 4
static unsigned baudDiv[] = {
	26,	// 115200 bd
	40,	// 76800 bd
	53,	// 57600 bd
	80,	// 38400 bd
	162,	// 19200 bd
	216,	// 14400 bd
	325,	// 9600 bd
	1301,	// 2400 bd
	2603	// 1200 bd
};

int initUart(void)
{
	setGPIOmodeUART();
	
	IO_UARTCR1 = 0; // disable UART
	IO_UARTLCR_L1 = lowCtrl = baudDiv[USE_BAUD_INDEX] & 0x0ffff; // BR_115200; // was 26;
	IO_UARTLCR_M1 = medCtrl = 0; // BRDIV; // was 0;
//	IO_UARTLCR_H1 = WL_8 << WRDLEN_SHIFT;  // 8 Bit, 1 stop, no parity, FiFO off
	IO_UARTLCR_H1 = highCtrl = FIFOEN | (WL_8 << WRDLEN_SHIFT);  // 8 Bit, 1 stop, no parity, FiFO on
	IO_UARTCR1 = genCtrl = UARTEN /*| RIE | TIE */;  /* uart enable + enable interrupts */

	return(0);
}

// we have different locations where we try to init
// the serial line; so use a flag to prevent multiple init calls
static int uartInitDone = 0;
int initUartDyn(void)
{
    if (uartInitDone)
      return(0);

    initUart();
    uartInitDone = 1;
    
    return(0);
}

int soc_clear_rcv_intr(void)
{
	IO_UARTSR1 = CLRERR;
	return(0);
}

int read_char_state2_soc(void)
{
	int commState = IO_UARTFR1;
	return(commState);
}

int read_char_state_soc(void)
{
	int commState = IO_UARTSR1;
	return(commState);
}

int soc_enable_rcv_intr(void)
{

	IO_UARTCR1 = genCtrl = UARTEN | RIE | RTIE;
	return(0);
}


static void drainSoc(void)
{
	while (IO_UARTFR1 & UTXFF)
		;
}

static volatile int myCnt;
static inline int myloop(int loops)
{
    int ii;
    for (ii = 0; ii < loops; ii++)
      myCnt++;

    return(0);
}

int put_char_soc(char Char)
{
    	drainSoc();

	IO_UARTDR1 = Char;
        myloop(1000);
	return(Char);
}

int get_char_soc(void)
{
	while( IO_UARTFR1 & URXFE )
		;

	return IO_UARTDR1 & RX_TX_DATA;
}

int read_char_soc(void)
{
	return IO_UARTDR1 & RX_TX_DATA;
}

int is_char_soc_available(void)
{
	return(!( IO_UARTFR1 & URXFE ));
}

int printkSoc(const char *format, ...)
{
	int myChar;
	int ii;


	for (ii	= 0; (myChar = *(format + ii)) != 0; ii++) {
		put_char_soc(myChar);
	}
	return(ii);
}

#ifdef CONFIG_KGDB

/* used for kernel debugging with gdb */

int putDebugChar(unsigned char Char)
{
	drainSoc();

#ifdef CONFIG_ARCH_AuD_SOC1_KDBG_SPLIT
        // use bit 7 to separate between normal and kgdb communication
        Char |= 0x80;
#endif
        
	IO_UARTDR1 = Char;

	return(Char);
}

int getDebugChar(void)
{
        int myChar;
    
	while( IO_UARTFR1 & URXFE )
		;

        myChar = IO_UARTDR1 & RX_TX_DATA;
        
#ifdef CONFIG_ARCH_AuD_SOC1_KDBG_SPLIT
        // use bit 7 to separate between normal and kgdb communication
        myChar &= 0x7F;
#endif
	return myChar;
}

#endif  // CONFIG_KGDB

/*
 * low level output during first startups
 */
static volatile unsigned myCount;
static void myWait(int cnt)
{
    int ii;
    for (ii = 0; ii < cnt; ii++)
      myCount++;
}

int serial_out_char_virt_ng(char myChar)
{
        initUartDyn();
	put_char_soc(myChar);
        myWait(1000);
	return(0);
}

void do_putchar(char myChar)
{
        initUartDyn();
	put_char_soc(myChar);
        myWait(1000);
}


#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <asm/serial.h>
#include <asm/time.h>

#include <asm/mips-boards/generic.h>
#include <asm/mips-boards/prom.h>
#include <asm/bootinfo.h>
#include <asm/cpu-info.h>
#include <socGen.h>

/* Serial port registrations */

// currently a dummy
static struct sockUart {
	unsigned flag;
} uart_cfg;

struct platform_device *s_soc_uart_devs[1];

/* is in reality the second UART of the SOC1 MIPS */
static struct resource soc_uart0_resource[] = {
    [0] = {
		.name = "uart_addr",
		.start = SOC_UART1_START,
		.end   = (SOC_UART1_START + SOC_UART1_LENGTH - 1),
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.name = "uart_irq",
		.start = IRQ_SERIAL_1,
		.end   = IRQ_SERIAL_1,
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.name = "uart_err",
		.start = IRQ_SERIAL_ERR_1,
		.end   = IRQ_SERIAL_ERR_1,
		.flags = IORESOURCE_IRQ,
	}
};

static struct platform_device soc_uart0 = {
	.name		  = "s-soc-uart",
	.id		  = 0,
	.num_resources	  = ARRAY_SIZE(soc_uart0_resource),
	.resource	  = soc_uart0_resource,
};

static struct platform_device *uart_devices[] __initdata = {
	&soc_uart0
};

static int s_soc_uart_count = 0;

int aud_soc_init_uarts(void)
{
	struct platform_device *platdev;

	printk("aud_soc_init_uarts\n");
	platdev = uart_devices[0]; // cfg->hwport

	s_soc_uart_devs[0] = uart_devices[0];
	platdev->dev.platform_data = &uart_cfg;

	s_soc_uart_count = 1;


    return(0);
}

int add_soc_uart_devices(void)
{
    int retVal;

    retVal = platform_add_devices(s_soc_uart_devs, s_soc_uart_count);

    printk("add_soc_uart_dev  (ret=%d)\n", retVal);

        return(0);
}

