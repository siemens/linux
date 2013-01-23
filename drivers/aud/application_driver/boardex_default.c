/****************************************************************************
 *
 *   board level exception handler example for linux
 *
 *   fe, 03-NOV-08
 *   ak, 07-March-12, updated for kernel 2.6.31
 *
 */
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kmod.h>
#include <linux/module.h>
#include <linux/preempt.h>
#include <linux/proc_fs.h>
#include <linux/reboot.h>
#include <linux/sched.h>


MODULE_LICENSE("GPL");


// CONFIG_AUD_LOW_LEVEL_KERNEL_DUMP set by kernel option Kernel hacking->Audis settings->support low level kernel dump messages
#ifdef CONFIG_AUD_LOW_LEVEL_KERNEL_DUMP
extern void show_trace_direct (struct pt_regs *regs);
#endif

#define BUS3_STATUS_REGADDR      0xBA000001
#define BUS3_STATUS_SWITCH_RUN   0x04

#define GPIO2_REGADDR            0xBFA00044
#define LED_BITMASK              0x003F0000
#define PN_5V_BITMASK            0x00400000

#define KRISCRESET_REGADDR       0xBEF00040
#define RESET_PB_CHANNEL_1       0x00000001
#define RESET_PB_CHANNEL_2       0x00000002

#define UART_BASE                0xBF480000   /* UART1 Base Adr und Datenregister   */
#define UART_DR                  (UART_BASE + 0x00)
#define UART_LSR                 (UART_BASE + 0x18)

#define LINKCHG_REGADDR          0xBD21500C

#define UART1_LSR_BUSY           0x08         /* Busy Flag in LSR register          */      
#define UART1_LSR_RXFE           0x10         /* FIFO Empty Receive in LSR register */
#define UART1_LSR_TXFF           0x20         /* FIFO Full Transmit in LSR register */


/****************************************************************************
 *
 * boardex_set_all_leds - switch all board leds on/off
 *
 */
static void boardex_set_all_leds (int status)
{
    volatile unsigned int *pGpio2 = (volatile unsigned int *) GPIO2_REGADDR;
        
    if (status)
    {
        *pGpio2 &= ~LED_BITMASK;
    }
    else
    {
        *pGpio2 |= LED_BITMASK;
    }
}


/****************************************************************************
 *
 * boardex_get_switch - get the board's current switch status
 *
 */
static int boardex_get_switch (void)
{
    volatile unsigned char *pBus3Status = (volatile unsigned char *) BUS3_STATUS_REGADDR;
    
    return ((*pBus3Status & BUS3_STATUS_SWITCH_RUN) == BUS3_STATUS_SWITCH_RUN);
}


/****************************************************************************
 *
 * boardex_delay - delay for a selectable number of milliseconds
 *
 */
static void boardex_delay (int msec)
{
    volatile unsigned long long *pTtuRegBase = (volatile unsigned long long *) 0xB6000000;
    unsigned long long endTime = *(pTtuRegBase + 3) + (unsigned long long) msec * 1000LL;

    while (*(pTtuRegBase + 3) < endTime)
    {
        /* do nothing */
    }
}


/****************************************************************************
 *
 * boardex_get_key - returns the value of a character coming in from the
 *                   serial interface or 0 if no key is pressed at all
 *
 */
static int boardex_get_key (void)
{
    volatile unsigned char *pUartLsr = (volatile unsigned char *) UART_LSR;
    volatile unsigned char *pUartDr  = (volatile unsigned char *) UART_DR;

    if (*pUartLsr & UART1_LSR_RXFE)
    {
        return 0;
    }
    else
    {
        return *pUartDr;
    }
}


/****************************************************************************
 *
 * boardex_write_char - transmit one character on the serial interface
 *
 */
static void boardex_write_char (const char ch)
{
    volatile unsigned char *pUartLsr = (volatile unsigned char *) UART_LSR;
    volatile unsigned char *pUartDr  = (volatile unsigned char *) UART_DR;

    /* wait til there is a free entry in the transmitter fifo */
    while (*pUartLsr & UART1_LSR_TXFF);
           
    *pUartDr = ch;
}


/****************************************************************************
 *
 * boardex_write_string - transmit a zero-terminated string
 *                        on the serial interface
 *
 * this polling-driven approach is preferred over printk,
 * because printk obviously re-enables interrupts
 * which we don't want in our exception loop
 *
 */
static void boardex_write_string (const char *str)
{
    while (*str)
    {
        switch (*str)
        {
        case '\n':
            boardex_write_char ('\r');
        default:
            boardex_write_char (*str++);
            break;
        }
    }
}


/****************************************************************************
 *
 * boardex_disable_output - disable the output to external interfaces
 *
 */
static void boardex_disable_output (void)
{
    volatile unsigned int *pKriscReset = (volatile unsigned int *) KRISCRESET_REGADDR;
        
    boardex_write_string ("\nresetting pb and kbus\n\n");

    /* reset both channels of the krisc
       channel 1 : profibus
       channel 2 : kbus */
    *pKriscReset = (RESET_PB_CHANNEL_1 | RESET_PB_CHANNEL_2);
}


/****************************************************************************
 *
 * boardex_reset - reset the board
 *
 */
static void boardex_reset (void)
{
    boardex_write_string ("\nattempting to reset the board ...\n\n");
    
    emergency_restart ();    
}


/****************************************************************************
 *
 * boardex_linkchange_reset - reset link change detection bit
 *
 */
static void boardex_linkchange_reset (void)
{
    volatile unsigned int *pLinkChgReg = (volatile unsigned int *) LINKCHG_REGADDR;
        
    *pLinkChgReg = 0;
}


/****************************************************************************
 *
 * boardex_linkchange_get - evaluate link change detection bit
 *
 */
static int boardex_linkchange_get (void)
{
    volatile unsigned int *pLinkChgReg = (volatile unsigned int *) LINKCHG_REGADDR;
        
    return ((*pLinkChgReg & 0x00000002) == 0x00000002) ? 1 : 0;
}


/****************************************************************************
 *
 * board_exception_notify - this function must be installed by board specific
 *                          sw to handle the cyclical activities, like
 *                          blinking and pausing.
 *                          returning the value 0 here means, that the exception
 *                          loop shall terminate and that a gdb backend is to be
 *                          activated for debugging.
 *
 */
int board_exception_notify (char *str)
{
    static int lastrunstop = 2;
    static int ledstatus   = 0;
    static int firstcall   = 1;

    int runstop, lastkey;

    /* when entered for the first time, shut down external interfaces */
    if (firstcall)
    {
        boardex_disable_output ();
    }

    /* react on key input from the terminal if available */
    lastkey = boardex_get_key ();
    if ((lastkey == 'd') || (lastkey == 'D'))
    {
        boardex_set_all_leds (1);
        lastrunstop = 2;

        /* leave the cyclic loop of kernex */
        return 0;
    }
    else if ((lastkey == 's') || (lastkey == 'S'))
    {
        
	// CONFIG_AUD_LOW_LEVEL_KERNEL_DUMP set by kernel option Kernel hacking->Audis settings->support low level kernel dump messages
	#ifdef CONFIG_AUD_LOW_LEVEL_KERNEL_DUMP
	show_trace_direct (NULL);
	#endif

    }
    else if ((lastkey != 0) || (firstcall))
    {
        /* repeat error string */
        boardex_write_string (str);
        boardex_write_string ("\n- press 'D' or plug in network cable to continue");
        boardex_write_string ("\n- move the board's switch from STOP to RUN to force a reset\n\n");
    }        
         
    /* react on manipulations of the frontpanel switch */
    runstop = boardex_get_switch ();
    if (runstop != lastrunstop)
    {
        /* a transition from stop to run shall cause a reboot */
        if ((runstop == 1) && (lastrunstop == 0))
        {
            boardex_set_all_leds (1);
            lastrunstop = 2;
            boardex_reset ();
        }
        lastrunstop = runstop;
    }

    /* toggle leds and wait a little */
    boardex_set_all_leds (ledstatus);
    ledstatus = !ledstatus;
    boardex_linkchange_reset ();
    boardex_delay (250);

    /* react on plug of network cable */
    if (boardex_linkchange_get ())
    {
        boardex_set_all_leds (1);
        lastrunstop = 2;

        /* leave the cyclic loop of kernex */
        return 0;
    }

    /* remain in the cyclic loop of kernex */
    firstcall = 0;
    return 1;
}


/****************************************************************************
 *
 * boardex_init - driver installation routine
 *
 */
static int boardex_init (void)
{
    printk (KERN_NOTICE " boardex driver is installed\n");

    return 0;
}


/****************************************************************************
 *
 * boardex_exit - driver uninstallation routine
 *
 */
static void boardex_exit (void)
{
}


EXPORT_SYMBOL(board_exception_notify);

module_init(boardex_init);
module_exit(boardex_exit);
