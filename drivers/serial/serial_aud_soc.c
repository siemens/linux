/*
 * linux/drivers/serial/serial_aud_soc.c
 *
 * Driver for onboard UARTs on the Siemens AuD SOC board
 *
 * Based on drivers/char/serial.c, drivers/char/21285.c. s3c2410.c
 *
 * Manfred.Neugebauer@siemens.com
 *
 * Changelog:
 *
 * 24-June-2008 m.n.  initial version
 * 24-Mar-2011 m.n. add rt domain interrupt support
 * 03-Aug-2011 m.n. move to kernel 2.6.31.12
 */

#include <linux/autoconf.h>

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/sysrq.h>
#include <linux/console.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/delay.h>

#include <asm/io.h>
#include <asm/irq.h>

//#include <asm/hardware.h>
//#include <asm/hardware/clock.h>

#include <asm/mach-aud_soc/regs-serial.h>
//#include <asm/arch/regs-gpio.h>

/* structures */

//#define CONFIG_SERIAL_ENABLE_RT_HOT_KEY 1
//#define CONFIG_SERIAL_RT_HOT_KEY_STRING "witch"

/* UART name and device definitions */

#define S_SOC_SERIAL_NAME	"ttyS"
#define S_SOC_SERIAL_DEVFS	"tts/"
#define S_SOC_SERIAL_MAJOR	TTY_MAJOR
#define S_SOC_SERIAL_MINOR	64

/* take Samsung S3C2400 SoC as tty port id */
#define PORT_AUD_SOC PORT_S3C2400

unsigned s_soc_serial_debug = 0; // 1;
#define S_SOC_DEBUG_TX	0x010

int initUartDyn(void);

#ifdef CONFIG_SERIAL_AUD_SOC_CONSOLE
static void
s_soc_serial_console_write(struct console *co, const char *s,
			     unsigned int count);
static int __init
s_soc_serial_console_setup(struct console *co, char *options);
static struct console s_soc_serial_console =
{
    .name		= S_SOC_SERIAL_NAME,
	.device		= uart_console_device,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.write		= s_soc_serial_console_write,
	.setup		= s_soc_serial_console_setup
};
#else
static struct console s_soc_serial_console;
#endif
int soc_clear_rcv_intr(void);
int soc_enable_rcv_intr(void);
int read_char_soc(void);
int read_char_state_soc(void);
int read_char_state2_soc(void);

int get_char_soc(void);
int put_char_soc(int myChar);
int is_char_soc_available(void);

#ifdef CONFIG_SERIAL_SOC_INTR_MODE_REALTIME

// should be part of include/interrupt.h
#define IRQF_RTIRQ_BALANCED            0x00010000

#include <linux/aud/rt_driver.h>

#define MAX_RT_BUFFER_LENGTH	256

static int verbose_rt = 0;

static irqreturn_t
s_soc_uart_interrupt_rt(int irq, void *dev_id);
static irqreturn_t
s_soc_uart_interrupt_lx(int irq, void *dev_id);

#ifdef CONFIG_SERIAL_ENABLE_RT_HOT_KEY
int hotKeyAction(void);
#define MAX_NR_LAST_CHAR	5
static unsigned hotKeyOffset;
static char lastCharString[MAX_NR_LAST_CHAR];
static char hotCharString[MAX_NR_LAST_CHAR];
static unsigned hotKeyFlag;
#endif
#else

#ifndef CONFIG_SERIAL_AUD_SOC_WITH_INTR
static int receiveLoop(void *ref);
#endif
static irqreturn_t s_soc_serial_handle_rx(struct uart_port *port);
#endif

/* we currently only support one UART */

#define NR_PORTS (1)

/* port irq numbers */

#define TX_IRQ(port) ((port)->irq)
#define RX_IRQ(port) ((port)->irq)

/* register access controls */

#define portaddr(port, reg) ((port)->membase + (reg))

static struct uart_driver s_soc_uart_drv = {
	.owner		= THIS_MODULE,
	.dev_name	= "s_soc_serial",
	.nr		= 1,
	.cons		= &s_soc_serial_console,
	.driver_name	= S_SOC_SERIAL_NAME,
	.major		= S_SOC_SERIAL_MAJOR,
	.minor		= S_SOC_SERIAL_MINOR,
};

struct s_soc_uart_port {
	unsigned flag;
	struct uart_port port;
	unsigned rtIrqHandlerActive;
#ifdef CONFIG_SERIAL_SOC_INTR_MODE_REALTIME
	unsigned serialIrqAction;
	int actRtReadIndex;
	int actRtWriteIndex;
	struct realtimeBuffer {
		unsigned int lineState;
		unsigned char myChar;
		unsigned char myState;
	} rtCharInfo[MAX_RT_BUFFER_LENGTH];
#else
	volatile int serialKernelThrdActive;
#endif
};


static inline struct s_soc_uart_port *to_ourport(struct uart_port *port)
{
	return container_of(port, struct s_soc_uart_port, port);
}

/* power power management control */

static void s_soc_serial_pm(struct uart_port *port, unsigned int level,
			      unsigned int old)
{

}

static unsigned int s_soc_serial_tx_empty(struct uart_port *port)
{
	if (s_soc_serial_debug)
		printk(" s_soc_serial_tx_empty\n");
	return(1);
}

/* no modem control lines */
static unsigned int s_soc_serial_get_mctrl(struct uart_port *port)
{
	printk("s_soc_serial_get_mctrl\n");
	return(0);
}

static void s_soc_serial_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	/* todo */
	if (s_soc_serial_debug)
		printk("s_soc_serial_set_mctrl (%#x)\n", mctrl);
}

static void s_soc_serial_set_termios(struct uart_port *port,
				       struct ktermios *termios,
				       struct ktermios *old)
{
	struct s_soc_uart_port *myPort;
#ifdef CONFIG_SERIAL_SOC_INTR_MODE_REALTIME
	unsigned long flags;
#endif

	myPort = to_ourport(port);

	if (s_soc_serial_debug)
		printk("s_soc_serial_set_termios iflag=%#x oflag=%#x cflag=%#x lflag=%#x c_line=%#x vmin=%d:%d kill=%d ctrl=%d\n",
		(unsigned) termios->c_iflag, (unsigned) termios->c_oflag,
		(unsigned) termios->c_cflag, (unsigned) termios->c_lflag,
		termios->c_line, termios->c_cc[VMIN], VMIN,
		termios->c_cc[VKILL],
		termios->c_cc[NCCS]);		/* control characters */

	// flush
	while (is_char_soc_available())
		get_char_soc();
#ifdef CONFIG_SERIAL_SOC_INTR_MODE_REALTIME
	local_irq_save_hw(flags); 
	while (is_char_soc_available()) /* normally the buffer should already be empty now */ 
		get_char_soc();
	myPort->actRtReadIndex = myPort->actRtWriteIndex = 0;
	local_irq_restore_hw(flags);
#endif

	// m.n. do we really need this copy back?
	if (old) {
		*old = *termios;
		printk("termios old %d\n", old->c_cc[VMIN]);
	}
}

#ifdef CONFIG_SERIAL_SOC_INTR_MODE_REALTIME


#else

#ifdef CONFIG_SERIAL_AUD_SOC_WITH_INTR
static irqreturn_t
s_soc_uart_interrupt(int irq, void *dev_id)
{
	struct uart_port *port = (struct uart_port *) dev_id;
	
	s_soc_serial_handle_rx(port);
	soc_clear_rcv_intr();
	return IRQ_HANDLED;
}

#endif
#endif

#ifdef CONFIG_SERIAL_AUD_SOC_WITH_INTR
static struct irqaction s_soc_uart_irq = {
	.name		= "aud_soc UART irq",
	.flags		= 0,
};
#endif

static int s_soc_serial_startup(struct uart_port *port)
{
#ifdef CONFIG_SERIAL_AUD_SOC_WITH_INTR
	int ret = 0;
#else
	unsigned loopCnt = 0;
#endif
	struct s_soc_uart_port *myPort;

	myPort = to_ourport(port);
	
#ifdef CONFIG_SERIAL_AUD_SOC_WITH_INTR
	printk("s_soc_serial_startup with interrupts\n");
	s_soc_uart_irq.dev_id = port;
	if (!myPort->rtIrqHandlerActive) {
#ifdef CONFIG_SERIAL_SOC_INTR_MODE_REALTIME
		ret = rt_request_irq(RX_IRQ(port), s_soc_uart_interrupt_rt,
				IRQF_RTIRQ_BALANCED, // not cpu related
				s_soc_uart_irq.name, port,
				s_soc_uart_interrupt_lx);
		if (!ret) ret = rt_enable_irq_affinity(RX_IRQ(port));
#else
		s_soc_uart_irq.handler = s_soc_uart_interrupt;
		ret = setup_irq(RX_IRQ(port), &s_soc_uart_irq);
#endif
	}
	soc_enable_rcv_intr();
	if (ret < 0) {
		printk(KERN_ERR "cannot get irq %d\n", IRQ_SERIAL_1);
		return ret;
	}
	myPort->rtIrqHandlerActive = 1;

#else /* polling mode */
	if (!myPort->serialKernelThrdActive)
		printk("s_soc_serial_startup with kernel thread\n");
	else if (s_soc_serial_debug)
		printk("s_soc_serial_startup with kernel thread\n");
	if (!myPort->serialKernelThrdActive) { // we need only one thread
		kernel_thread(receiveLoop, (void *) port, 0);
		// test for running thread
		while (!myPort->serialKernelThrdActive) {
			loopCnt++;
			__set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(20);
			if ((loopCnt % 300) == 1) {
				printk("waiting for serial line kernel thread(%d)\n", loopCnt);
			}
		}
	}
#endif	
	return(0);
}

static void s_soc_serial_shutdown(struct uart_port *port)
{
	if (s_soc_serial_debug)
		printk("s_soc_serial_shutdown\n");
}

static void s_soc_serial_break_ctl(struct uart_port *port, int break_state)
{
	printk("s_soc_serial_break_ctl (state=%#x)\n", break_state);
}

static void s_soc_serial_stop_tx(struct uart_port *port)
{
	if (s_soc_serial_debug & S_SOC_DEBUG_TX)
		printk("s_soc_serial_stop_tx\n");
}

static void s_soc_serial_start_tx(struct uart_port *port)
{
	struct circ_buf *xmit = &port->info->xmit;
	int count = 256;
	char myChar;

	if (s_soc_serial_debug & S_SOC_DEBUG_TX)
		printk("s_soc_serial_start_tx\n");

	while (!uart_circ_empty(xmit) && count-- > 0) {
		myChar = xmit->buf[xmit->tail];
		put_char_soc(myChar);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		port->icount.tx++;
	}

	if (uart_circ_empty(xmit))
		s_soc_serial_stop_tx(port);
}

#ifndef CONFIG_SERIAL_AUD_SOC_WITH_INTR
static unsigned actCount = 0;
static unsigned stateComm = 0;
#endif
#ifndef CONFIG_SERIAL_SOC_INTR_MODE_REALTIME
static unsigned portCount = 0;
#endif

//		/* If we are full, just stop reading */
//		if (tty->flip.count >= TTY_FLIPBUF_SIZE)
//			break;


#ifdef CONFIG_SERIAL_SOC_INTR_MODE_REALTIME

#define INC_INDEX(a, b) \
	a = b + 1; \
	if (a >= MAX_RT_BUFFER_LENGTH) \
		a = 0;
void printkDirect(char *ptr);
static irqreturn_t
s_soc_uart_interrupt_rt(int irq, void *dev_id)
{
	int tmpIndex;
	int max_count = 64, serialState;
	unsigned char myChar;
	unsigned long flags;
	struct uart_port *port = (struct uart_port *) dev_id;
	struct s_soc_uart_port *myPort;

	myPort = to_ourport(port);
    
	if (!(is_char_soc_available()))
		return IRQ_HANDLED;
        
startagain:
	while (max_count-- > 0) {
		serialState = read_char_state_soc();
		myChar = read_char_soc();
		local_irq_save_hw(flags); 
		INC_INDEX(tmpIndex, myPort->actRtWriteIndex);
		if (tmpIndex == myPort->actRtReadIndex) {
			local_irq_restore_hw(flags);
			// buffer full, signal overflow
			printkGen(NULL, "rt-serial-rcv: buffer overflow\n"); 
			goto contReceive;
                }
		myPort->rtCharInfo[tmpIndex].myChar = myChar;
                if (!serialState)
			myPort->rtCharInfo[tmpIndex].myState = TTY_NORMAL;
                else {
			myPort->rtCharInfo[tmpIndex].myState = TTY_OVERRUN;
			printkGen(NULL, "state error %#x\n", serialState);
			soc_clear_rcv_intr();
                }
		myPort->rtCharInfo[tmpIndex].lineState = 0;
		myPort->actRtWriteIndex = tmpIndex;
#ifdef CONFIG_SERIAL_ENABLE_RT_HOT_KEY
			if (hotKeyFlag) {
			if (hotKeyOffset < MAX_NR_LAST_CHAR) {
				lastCharString[hotKeyOffset] = myChar;
				hotKeyOffset++;
			}
			else {
				int ii;
				for (ii = 0; ii < MAX_NR_LAST_CHAR - 1; ii++)
					lastCharString[ii] = lastCharString[ii +1];
				lastCharString[ii] = myChar; 
			}
			if (!memcmp(lastCharString, hotCharString, MAX_NR_LAST_CHAR)) {
                            int ii;
                            printkDirect("hot key was activated\r\n");
                            hotKeyAction();
                            printkDirect(">> actual buffer:");
                            printkDirectHex(myPort->actRtWriteIndex);
                            printkDirect(":");
                            printkDirectHex(myPort->actRtReadIndex);
                            printkDirect(" <<\r\n");
                                for (ii = 0; ii < MAX_RT_BUFFER_LENGTH; ii++)
		put_char_soc(myPort->rtCharInfo[ii].myChar);
                            printkDirect("\r\n");
			}
		}
#endif
		local_irq_restore_hw(flags);
		if (!(is_char_soc_available()))
			break;
	}
	if (!max_count)
		printkGen(NULL, "rt-serial-rcv:too many data\n");
	else if ((is_char_soc_available()))
		goto startagain;
        
contReceive:	
	execute_nonrt_handler(0, irq);

	return IRQ_HANDLED;
}

static irqreturn_t
s_soc_uart_handle_lx_rx(struct uart_port *port)
{
	int tmpIndex;
	irqreturn_t ret = IRQ_HANDLED;
	unsigned long flags;
	unsigned char myChar, flag;
	unsigned int line_state;
	struct s_soc_uart_port *myPort;
	struct tty_struct *tty = port->info->port.tty;

	myPort = to_ourport(port);

	while (1) {
		local_irq_save_hw(flags); 
		if (myPort->actRtReadIndex == myPort->actRtWriteIndex) {
			// we are done
			local_irq_restore_hw(flags);
			break;
		}
		INC_INDEX(tmpIndex, myPort->actRtReadIndex);
		myChar = myPort->rtCharInfo[tmpIndex].myChar;
		flag = myPort->rtCharInfo[tmpIndex].myState;
		line_state = myPort->rtCharInfo[tmpIndex].lineState;
		myPort->actRtReadIndex = tmpIndex;
		local_irq_restore_hw(flags);
		if (verbose_rt)
			printkGen(NULL, "rcv_lx:%#x tty=%p flag=%#x state=%#x\n",
				myChar, tty, flag, line_state);
		if (tty) { /* upper levels may not yet be ready */
			if (myChar == 3) {
				printk("uart_handle_break\n");
				//port->icount.brk++;
				//uart_handle_break(port);
				//flag = TTY_BREAK;
				do_SAK(tty);
			}
			else
				tty_insert_flip_char(tty, myChar, flag);
                }
	}
	// m.n. we do this here, the code takes care when this is
	// can't be done immediately.
	if (tty) /* upper levels may not yet be ready */
		tty_flip_buffer_push(tty);
	return ret;
}

static irqreturn_t
s_soc_uart_interrupt_lx(int irq, void *dev_id)
{
	struct uart_port *port = (struct uart_port *) dev_id;
	
	s_soc_uart_handle_lx_rx(port);

	return IRQ_HANDLED;
}

#else

static irqreturn_t s_soc_serial_handle_rx(struct uart_port *port)
{
	int max_count = 64;
	unsigned int myChar, flag;
	struct tty_struct *tty = port->info->port.tty;
	//unsigned long flags;

	if (!(is_char_soc_available()))
		return IRQ_HANDLED;
startagain:
	while (max_count-- > 0) {
		myChar = read_char_soc();
//		stateComm |= read_char_state_sock();
		if (!tty) /* upper levels may not yet be ready */
			goto loop_end;

		flag = TTY_NORMAL;
		portCount = port->icount.rx++;
		//ttyCount = tty->flip.count;
//printk("intr rcv:%#x\n", myChar);
//		spin_lock_irqsave(&tty->read_lock, flags);
//		if ((status & port->ignore_status_mask & ~overrun) == 0)
		if (tty) /* upper levels may not yet be ready */
			tty_insert_flip_char(tty, myChar, flag);
//		spin_unlock_irqrestore(&tty->read_lock, flags);

		/*
		 * Overrun is special.  Since it's reported immediately,
		 * it doesn't affect the current character.
		 */
//		if (status & ~port->ignore_status_mask & overrun)
//			tty_insert_flip_char(tty, 0, TTY_OVERRUN);
loop_end:
		if (!(is_char_soc_available()))
			break;
	}
	soc_clear_rcv_intr();
	if (!max_count)
		printk("serial-rcv:too many data\n");
	else if ((is_char_soc_available()))
		goto startagain;
#ifndef CONFIG_SERIAL_AUD_SOC_WITH_INTR
	actCount += 64 - max_count;
#endif
	// m.n. we do this here, the code takes care when this is
	// can't be done immediately.
	if (tty) /* upper levels may not yet be ready */
		tty_flip_buffer_push(tty);

	return IRQ_HANDLED;
}

#endif /* CONFIG_SERIAL_SOC_INTR_MODE_REALTIME */

#ifndef CONFIG_SERIAL_AUD_SOC_WITH_INTR

static int receiveLoop(void *ref)
{
	struct uart_port *port = ref;
	unsigned loop = 0;
	struct s_soc_uart_port *myPort;
	myPort = to_ourport(port);

	printk("enter receiveLoop...\n");
	myPort->serialKernelThrdActive = 1;

	while (1) {
		int loccnt;
		int locState;
		__set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(1);  // was 40
#if (1)
		loop++;
		if ((loop % 4000) == 4001) {
			printk("ser state: %#x:%#x\n",
				read_char_state_soc(), read_char_state2_soc());
		}
		if (read_char_state2_soc() & 0x40) {
			printk("force reread overflow\n");
			s_soc_serial_handle_rx(port);
		}
		if (!(read_char_state2_soc() & 0x10)) {
//			printk("force reread buffer\n");
			s_soc_serial_handle_rx(port);
		}
#endif
		loccnt = actCount;
		locState = stateComm;
//		if (loccnt > 10) {
		if (loccnt) {
			stateComm = 0;
			actCount = 0;
		}
	}
	
	return(5);
}

#endif

static void s_soc_serial_stop_rx(struct uart_port *port)
{
	if (s_soc_serial_debug)
		printk("s_soc_serial_stop_rx\n");
}

static void s_soc_serial_enable_ms(struct uart_port *port)
{
	printk("s_soc_serial_enable_ms\n");
}

static const char *s_soc_serial_type(struct uart_port *port)
{
	char *myType;

	switch (port->type) {
	case PORT_AUD_SOC:
		myType = "s_soc_serial port 0";
		break;
	default:
		myType = "unknown";
		break;
	}
	printk("s_soc_serial_type=%s\n", myType);

	return(myType);
}

static void s_soc_serial_release_port(struct uart_port *port)
{
	printk("s_soc_serial_release_port\n");
//	release_mem_region(port->mapbase, MAP_SIZE);
}

static int s_soc_serial_request_port(struct uart_port *port)
{
	printk("s_soc_serial_request_port\n");
//	const char *name = s3c24xx_serial_portname(port);
//	return request_mem_region(port->mapbase, MAP_SIZE, name) ? 0 : -EBUSY;
	
	return(0);
}

static void s_soc_serial_config_port(struct uart_port *port, int flags)
{
	printk("s_soc_serial_config_port flag=%#x\n", flags);
	if (flags & UART_CONFIG_TYPE)
		port->type = PORT_AUD_SOC;

}

/*
 * verify the new serial_struct (for TIOCSSERIAL).
 */
static int s_soc_serial_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	printk("s_soc_serial_verify_port\n");
	
	return(0);
}

static struct uart_ops s_soc_serial_ops = {
	.pm		= s_soc_serial_pm,
	.tx_empty	= s_soc_serial_tx_empty,
	.get_mctrl	= s_soc_serial_get_mctrl,
	.set_mctrl	= s_soc_serial_set_mctrl,
	.stop_tx	= s_soc_serial_stop_tx,
	.start_tx	= s_soc_serial_start_tx,
	.stop_rx	= s_soc_serial_stop_rx,
	.enable_ms	= s_soc_serial_enable_ms,
	.break_ctl	= s_soc_serial_break_ctl,
	.startup	= s_soc_serial_startup,
	.shutdown	= s_soc_serial_shutdown,
	.set_termios	= s_soc_serial_set_termios,
	.type		= s_soc_serial_type,
	.release_port	= s_soc_serial_release_port,
	.request_port	= s_soc_serial_request_port,
	.config_port	= s_soc_serial_config_port,
	.verify_port	= s_soc_serial_verify_port,
};

static struct s_soc_uart_port s_soc_serial_ports[NR_PORTS] = {
	[0] = {
		.port = {
			.lock		= SPIN_LOCK_UNLOCKED,
			.iotype		= UPIO_MEM,
			.irq		= IRQ_SERIAL_1,
			.uartclk	= 0,
			.fifosize	= 16,
			.ops		= &s_soc_serial_ops,
			.flags		= UPF_BOOT_AUTOCONF,
			.line		= 0,
		}
	}
};

/* Device driver serial port probe */

static int probe_index = 0;

static int s_soc_serial_probe(struct device *_dev)
{
	int ret;
	struct s_soc_uart_port *ourport;

	struct platform_device *dev = to_platform_device(_dev);

	printk("s_soc_serial_probe (0x%p:0x%p)\n", dev, _dev);

	ourport = &s_soc_serial_ports[probe_index];

	// T B D: init port
	ourport->port.type = PORT_AUD_SOC; // initialization somewhere else??

	ret = uart_add_one_port(&s_soc_uart_drv, &ourport->port);
	printk("after uart_add_one_port (ret=%d)\n", ret);
	dev_set_drvdata(_dev, &ourport->port);
        initUartDyn();

#ifdef CONFIG_SERIAL_ENABLE_RT_HOT_KEY
	if (strlen(CONFIG_SERIAL_RT_HOT_KEY_STRING) == MAX_NR_LAST_CHAR) {
		memcpy(hotCharString, CONFIG_SERIAL_RT_HOT_KEY_STRING, MAX_NR_LAST_CHAR);
		hotKeyFlag = 1;
	}
	else {
		printk(KERN_ERR "s_soc_serial_probe: no correct hot key string: %s\n",
			CONFIG_SERIAL_RT_HOT_KEY_STRING);
	}
#endif
	return 0;
}

static int s_soc_serial_remove(struct device *_dev)
{
	printk("s_soc_serial_remove\n");
//	struct uart_port *port = s3c24xx_dev_to_port(_dev);

//	if (port)
//		uart_remove_one_port(&s3c24xx_uart_drv, port);

	return 0;
}

static int s_soc_serial_suspend(struct device *dev,  pm_message_t state)
{
	printk("s_soc_serial_suspend\n");

	return 0;
}

static int s_soc_serial_resume(struct device *dev)
{
	printk("s_soc_serial_resume\n");

	return 0;
}

static struct device_driver s_soc_serial_drv = {
	.name		= "s-soc-uart",
	.owner		= THIS_MODULE,
	.bus		= &platform_bus_type,
	.probe		= s_soc_serial_probe,
	.remove		= s_soc_serial_remove,
	.suspend	= s_soc_serial_suspend,
	.resume		= s_soc_serial_resume,
};

static inline int s_soc_serial_init(void)
{
	int ret;
	
	ret = driver_register(&s_soc_serial_drv);
	printk("s_soc_serial_init done(ret=%d)\n", ret);

	return(ret);
}

static inline void s_soc_serial_exit(void)
{
	driver_unregister(&s_soc_serial_drv);
}

/* module initialisation code */

static int __init s_soc_serial_modinit(void)
{
	int ret;

	printk("s_soc_serial_modinit\n");

	ret = uart_register_driver(&s_soc_uart_drv);
	if (ret < 0) {
            printk("failed to register UART driver(%d)\n", ret);
            //	return -1;
	}

	s_soc_serial_init();

	return 0;
}

static void __exit s_soc_serial_modexit(void)
{

	s_soc_serial_exit();

	uart_unregister_driver(&s_soc_uart_drv);
}

module_init(s_soc_serial_modinit);
module_exit(s_soc_serial_modexit);

/* Console code */

#ifdef CONFIG_SERIAL_AUD_SOC_CONSOLE

static void
s_soc_serial_console_write(struct console *co, const char *s,
			     unsigned int count)
{
	int ii;
//	printk("serial cons write (%c len=%d)\n", *s, count);
	for (ii = 0; ii < count; ii++, s++) {
		put_char_soc(*s);
//		if (*s == '\n')
//		put_char_soc('\r');
	}
}

static int __init
s_soc_serial_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
	int baud = 9600;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	// m.n. now the serial line is active as linux console output
	printk("s_soc_serial_console_setup: co=%p (%d), %s\n",
	    co, co->index, options);

	return(0);
}

/* s_soc_serial_initconsole
 *
 * initialise the console from one of the uart drivers
*/

static int s_soc_serial_initconsole_done = 0;
static int s_soc_serial_initconsole(void)
{
	struct platform_device *dev = s_soc_uart_devs[0];

	printk("s_soc_serial_initconsole (%d)\n", s_soc_serial_initconsole_done);
	if (s_soc_serial_initconsole_done)
		return(0);

	if (dev == NULL) {
		printk("s_soc_serial_initconsole: no board device available\n");
		return 0;
	}

	if (strcmp(dev->name, "s-soc-uart")) {
		printk("s_soc_serial_initconsole: uncorrect device name\n");
		return 0;
	}
	
	// assumption: the UART was already initialized during board setup

	s_soc_serial_console.data = &s_soc_uart_drv;
	register_console(&s_soc_serial_console);
        initUartDyn();
	s_soc_serial_initconsole_done = 1;
	
	return 0;
}

console_initcall(s_soc_serial_initconsole);

int __init early_serial_console_initx(char **cmdline)
{
	printk("early_serial_console_init:%s\n", *cmdline);

//	s_soc_serial_initconsole();

	return(0);
}

#include <asm/setup.h>
//???__early_param("console=", early_serial_console_initx);

#endif /* SERIAL_AuD_SOCK_CONSOLE */

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Manfred.Neugebauer@siemens.com");
MODULE_DESCRIPTION("Siemens AuD Soc Serial port driver");

