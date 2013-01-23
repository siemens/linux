/* drivers/spi/aud_soc_spi.c
 *
 * 
 * Copyright (C) 2012 SiemensAG
 * <alexander.kubicki.@siemens.com>
 * <manfred.neugebauer@siemens.com>
 *
 * - Low-level SPI-driver for Siemens SOC1 to connect SPI-Ethernet driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */




#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/ioport.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>

#include <linux/workqueue.h>
 	#include <linux/amba/pl022.h>
 #include <linux/io.h>

#include <asm/mach-aud_soc/soc_spi.h>
#include <asm/mach-aud_soc/soc_gpio.h>



#define BUFFER_SIZE		PAGE_SIZE

//#define AUD_SOC_SPI_DEBUG
//#define READWRITER_DEBUG

/*
 * Queue State
 */
#define QUEUE_RUNNING                   (0)
#define QUEUE_STOPPED                   (1)

/*
 * SSP State - Whether Enabled or Disabled
 */
#define SSP_DISABLED 			(0)
#define SSP_ENABLED 			(1)

/*
 * SSP DMA State - Whether DMA Enabled or Disabled
 */
#define SSP_DMA_DISABLED 		(0)
#define SSP_DMA_ENABLED 		(1)

/*
 * SSP DMA Control Register - SSP_DMACR
 */
/* Receive DMA Enable bit */
#define SSP_DMACR_MASK_RXDMAE		(0x1UL << 0)
/* Transmit DMA Enable bit */
#define SSP_DMACR_MASK_TXDMAE		(0x1UL << 1)

/*
 * Macros to access SSP Registers with their offsets
 */
//Registers adapted from sp022 to sp021
#define SSP_CR0(r)	(r + 0x000)
#define SSP_CR1(r)	(r + 0x004)
#define SSP_DR(r)	(r + 0x008)
#define SSP_SR(r)	(r + 0x00C)
#define SSP_CPSR(r)	(r + 0x010)
#define SSP_ICR(r)	(r + 0x014)

/*
 * SSP Clock Prescale Register  - SSP_CPSR
 */
//Register CPSDVSR adapted from sp022 to sp021 (=same)
#define SSP_CPSR_MASK_CPSDVSR	(0xFFUL << 0)

/*
 * SSP Clock Defaults
 */
#define NMDK_SSP_DEFAULT_CLKRATE 0x2
#define NMDK_SSP_DEFAULT_PRESCALE 0x40

/*
 * Message State
 * we use the spi_message.state (void *) pointer to
 * hold a single state value, that's why all this
 * (void *) casting is done here.
 */
#define STATE_START                     ((void *) 0)
#define STATE_RUNNING                   ((void *) 1)
#define STATE_DONE                      ((void *) 2)
#define STATE_ERROR                     ((void *) -1)

/*
 * SSP DMA Control Register - SSP_DMACR
 */
/* Receive DMA Enable bit */
#define SSP_DMACR_MASK_RXDMAE		(0x1UL << 0)
/* Transmit DMA Enable bit */
#define SSP_DMACR_MASK_TXDMAE		(0x1UL << 1)


/*
 * Macros to access SSP Registers with their offsets
 */
//in SOC12: adress 14h = SSPIIR_SSPICR (r/w)
#define SSP_IMSC(r)	(r + 0x014) 

/*
 * SSP Interrupt related Macros
 */
//in SOC12: adress 0h = ???
#define DEFAULT_SSP_REG_IMSC  0x0UL
#define DISABLE_ALL_INTERRUPTS DEFAULT_SSP_REG_IMSC
#define ENABLE_ALL_INTERRUPTS (~DEFAULT_SSP_REG_IMSC)


/*
 * Macros to access SSP Registers with their offsets
 */
//in SOC12: same adress range as pl022
#define SSP_CR0(r)	(r + 0x000)
#define SSP_CR1(r)	(r + 0x004)
#define SSP_DR(r)	(r + 0x008)
#define SSP_SR(r)	(r + 0x00C)

/*
 * SSP Control Register 0  - SSP_CR0
 */
//Register CR0 adapted from sp022 to sp021
#define SSP_CR0_MASK_DSS	(0x1FUL << 0)
#define SSP_CR0_MASK_FRF	(0x3UL << 4)
#define SSP_CR0_MASK_SPO	(0x1UL << 6)
#define SSP_CR0_MASK_SPH	(0x1UL << 7)
#define SSP_CR0_MASK_SCR	(0xFFUL << 8)
/*
#define SSP_CR0_MASK_HALFDUP	(0x1UL << 5)
#define SSP_CR0_MASK_CSS	(0x1FUL << 16)
*/

/*
 * SSP Control Register 0  - SSP_CR1
 */
//Register CR1 adapted from sp022 to sp021
#define SSP_CR1_MASK_LBM	(0x1UL << 3)
#define SSP_CR1_MASK_SSE	(0x1UL << 4)
#define SSP_CR1_MASK_MS		(0x1UL << 5)
#define SSP_CR1_MASK_SOD	(0x1UL << 6)

/*
 * SSP Status Register - SSP_SR
 */
#define SSP_SR_MASK_TFE		(0x1UL << 0) /* Transmit FIFO empty */
#define SSP_SR_MASK_TNF		(0x1UL << 1) /* Transmit FIFO not full */
#define SSP_SR_MASK_RNE		(0x1UL << 2) /* Receive FIFO not empty */
#define SSP_SR_MASK_RFF 	(0x1UL << 3) /* Receive FIFO full */
#define SSP_SR_MASK_BSY		(0x1UL << 4) /* Busy Flag */


/*
 * This macro is also used to define some default values.
 * It will just shift val by sb steps to the left and mask
 * the result with mask.
 */
#define GEN_MASK_BITS(val, mask, sb) \
 (((val)<<(sb)) & (mask))

#define DRIVE_TX		0
#define DO_NOT_DRIVE_TX		1

#define DO_NOT_QUEUE_DMA	0
#define QUEUE_DMA		1

#define RX_TRANSFER		1
#define TX_TRANSFER		2

/*
 * Default SSP Register Values
 */
//CR0 adapted from sp022 to sp021
#define DEFAULT_SSP_REG_CR0 ( \
	GEN_MASK_BITS(SSP_DATA_BITS_12, SSP_CR0_MASK_DSS, 0)	| \
	GEN_MASK_BITS(SSP_INTERFACE_MOTOROLA_SPI, SSP_CR0_MASK_FRF, 4) \
	GEN_MASK_BITS(SSP_CLK_POL_IDLE_LOW, SSP_CR0_MASK_SPO, 6) | \
	GEN_MASK_BITS(SSP_CLK_FALLING_EDGE, SSP_CR0_MASK_SPH, 7) | \
	GEN_MASK_BITS(NMDK_SSP_DEFAULT_CLKRATE, SSP_CR0_MASK_SCR, 8) | \
)

//CR0 adapted from sp022 to sp021, but without FIFO interrupts
#define DEFAULT_SSP_REG_CR1 ( \
	GEN_MASK_BITS(LOOPBACK_DISABLED, SSP_CR1_MASK_LBM, 3) | \
	GEN_MASK_BITS(SSP_DISABLED, SSP_CR1_MASK_SSE, 4) | \
	GEN_MASK_BITS(SSP_MASTER, SSP_CR1_MASK_MS, 5) | \
	GEN_MASK_BITS(DO_NOT_DRIVE_TX, SSP_CR1_MASK_SOD, 6) | \
)

//CR0 adapted from sp022 to sp021 (=same clock prescale divisor)
#define DEFAULT_SSP_REG_CPSR ( \
	GEN_MASK_BITS(NMDK_SSP_DEFAULT_PRESCALE, SSP_CPSR_MASK_CPSDVSR, 0) \
)

//CR0 adapted from sp022 to sp021 (=no DMA at the moment)
#define DEFAULT_SSP_REG_DMACR (\
)

/*
 * SSP Interrupt related Macros
 */
#define DEFAULT_SSP_REG_IMSC  0x0UL
#define DISABLE_ALL_INTERRUPTS DEFAULT_SSP_REG_IMSC
#define ENABLE_ALL_INTERRUPTS (~DEFAULT_SSP_REG_IMSC)

#define CLEAR_ALL_INTERRUPTS  0x3



/**
 * ks8851_cs_control 
 * @command: select/delect the ks8851-chip
 *
 */
static void ks8851_cs_control(u32 command)
{
#define SOC_SPI_DEV_CS		89
	
	switch (command) {
		case SSP_CHIP_SELECT:
			#ifdef AUD_SOC_SPI_DEBUG
			printk("[aud_soc_spi] disable gpio-pin %d \n",SOC_SPI_DEV_CS);
			#endif
			aud_soc_disable_gpio_bit(SOC_SPI_DEV_CS);
			break;
		case SSP_CHIP_DESELECT:
			#ifdef AUD_SOC_SPI_DEBUG
			printk("[aud_soc_spi] enable gpio-pin %d \n",SOC_SPI_DEV_CS);
			#endif
			aud_soc_enable_gpio_bit(SOC_SPI_DEV_CS);
			break;
	}

}

/**
 * ks8851_hard_reset 
 * reset the ks8851 via SOC-gpio 113 (low active)
 */
static void ks8851_hard_reset(void)
{
#define SOC_SPI_DEV_RESET	113

	aud_soc_enable_gpio_bit(SOC_SPI_DEV_RESET);
	udelay(100);
	aud_soc_disable_gpio_bit(SOC_SPI_DEV_RESET);
	udelay(20);
	aud_soc_enable_gpio_bit(SOC_SPI_DEV_RESET);
	

}



 /*
 * The type of reading going on on this chip
 */
enum ssp_reading 
{
	READING_NULL,
	READING_U8,
	READING_U16,
	READING_U32
};

/*
 * The type of writing going on on this chip
 */
enum ssp_writing 
{
	WRITING_NULL,
	WRITING_U8,
	WRITING_U16,
	WRITING_U32
};


/**
 * struct vendor_data - vendor-specific config parameters
 * for PL022 derivates
 * @fifodepth: depth of FIFOs (both)
 * @max_bpw: maximum number of bits per word
 * @unidir: supports unidirection transfers
 */
struct vendor_data {
	int fifodepth;
	int max_bpw;
	bool unidir;
};


struct pl021 {				  //---v new elements from pl022
	struct amba_device		*adev;
	struct vendor_data		*vendor;
	unsigned		flag;
	spinlock_t		lock;
	int			irq;
	struct clk		*clk;
	struct spi_master		*master;
	struct pl022_ssp_controller	*master_info;
	void __iomem			*virtbase;
	// Driver message queue //
	struct workqueue_struct		*workqueue;
	struct work_struct		pump_messages;
	spinlock_t			queue_lock;
	struct list_head	queue;
	int				busy;
	int				run;
	char 			*buffer;
	dma_addr_t 		buffer_dma;
	void __iomem 		*hwPtr;
	struct platform_device 	*pdev;
	// Message transfer pump //
	struct tasklet_struct		pump_transfers;
	struct spi_message		*cur_msg;
	struct spi_transfer		*cur_transfer;
	struct chip_data		*cur_chip;
	void				*tx;
	void				*tx_end;
	void				*rx;
	void				*rx_end;
	enum ssp_reading		read;
	enum ssp_writing		write;
};

/**
 * struct chip_data - To maintain runtime state of SSP for each client chip
 * @cr0: Value of control register CR0 of SSP
 * @cr1: Value of control register CR1 of SSP
 * @dmacr: Value of DMA control Register of SSP
 * @cpsr: Value of Clock prescale register
 * @n_bytes: how many bytes(power of 2) reqd for a given data width of client
 * @enable_dma: Whether to enable DMA or not
 * @write: function ptr to be used to write when doing xfer for this chip
 * @read: function ptr to be used to read when doing xfer for this chip
 * @cs_control: chip select callback provided by chip
 * @xfer_type: polling/interrupt/DMA
 *
 * Runtime state of the SSP controller, maintained per chip,
 * This would be set according to the current message that would be served
 */
struct chip_data {
	u16 		cr0;
	u16 		cr1;
	u16 		dmacr;
	u16 		cpsr;
	u8 		n_bytes;
	u8 		enable_dma:1;
	enum 		ssp_reading read;
	enum 		ssp_writing write;
	void (*cs_control) (u32 command);
	int 		xfer_type;
};




static void giveback(struct pl021 *pl021_info);




static int aud_soc_spi_setup(struct spi_device *spi)
{

	struct soc_spi_info	*soc_spi_register;
	struct resource		*regs;

	struct pl021			*pl021_info_local;

	printk("[aud_soc_spi] aud_soc_spi_setup\n");

	pl021_info_local = spi_master_get_devdata(spi->master);

	ks8851_hard_reset();
	

	regs = platform_get_resource(pl021_info_local->pdev, IORESOURCE_MEM, 0);
	if (!regs) 
	{
		printk("[aud_soc_spi] setup: no resource found !\n");
		return -ENXIO;
	}

	#ifdef AUD_SOC_SPI_DEBUG
	printk("[aud_soc_spi] setup: resource=<%#x:%#x>\n",regs->start, regs->end);
	#endif

	soc_spi_register = ioremap(regs->start , (regs->end - regs->start) + 1 );

	#ifdef AUD_SOC_SPI_DEBUG
	printk("[aud_soc_spi] aud_soc_spi_setup: ioremap=<%p>\n", soc_spi_register);
	#endif

	//register CR0:
	//bit 0-3 : DSS=0111 (8-Bit-data) 
	//bit 4-5 : FRF=00 (Motorola Format)
	//bit 6	  : SPO=0 (SCLKOUT polarity for Motorola SPI)
	//bit 7	  : SPH=0 (SCLKOUT phase for Motorola SPI)	
	//bit 8-15: SCR=0 (25 Mhz with CPSDVSR=2)
	//soc_spi_register->soc_spi_sspcr0 = 0x0107;
	
	soc_spi_register->soc_spi_sspcr0 = 0x0007;

	//register CR1:
	//bit 0 : RIE=0 (Receive FIFO interrupt enable)
	//bit 1 : TIE=0 (Transmit FIFO interrupt enable)
	//bit 2 : RORIE=0 (Receive FIFO overrun interrupt enable))
	//bit 3 : LBM=0 (Loop back mode)
	//bit 4 : SSE=0 ( Synchronous serial port enable)
	//bit 5 : MS=0 (Master/Slave mode select, modify only if SSE=0)
	//bit 6 : SOD=0 (Slave mode output disable, relevant if MS=1)
	//soc_spi_register->soc_spi_sspcr1 = 0x0008;

	soc_spi_register->soc_spi_sspcr1 = 0x00001;
	
	//register CPSR:
	//bit 0-7 : CPSDVSR=0x02 (Clock Prescale Devisor CPSDVSR)

	soc_spi_register->soc_spi_sspcpsr = 0x02;

	printk("[aud_soc_spi] aud_soc_spi_setup: OK and exit\n");
 
	return 0;
}



/**
 * flush - flush the FIFO to reach a clean state
 * @pl022: SSP driver private data structure
 */
static int flush(struct pl021 *pl021_info)
{
	unsigned long limit = loops_per_jiffy << 1;
#ifdef AUD_SOC_SPI_DEBUG
	dev_dbg(&pl021_info->adev->dev, "flush\n");
#endif
	do {
		while (readw(SSP_SR(pl021_info->virtbase)) & SSP_SR_MASK_RNE)
			readw(SSP_DR(pl021_info->virtbase));
	} while ((readw(SSP_SR(pl021_info->virtbase)) & SSP_SR_MASK_BSY) && limit--);
	return limit;
}

/**
 * This sets up the pointers to memory for the next message to
 * send out on the SPI bus.
 */
static int set_up_next_transfer(struct pl021 *pl021_info,
				struct spi_transfer *transfer)
{

	pl021_info->tx = (void *)transfer->tx_buf;
	pl021_info->tx_end = pl021_info->tx + pl021_info->cur_transfer->len;
	pl021_info->rx = (void *)transfer->rx_buf;
	pl021_info->rx_end = pl021_info->rx + pl021_info->cur_transfer->len;

	pl021_info->write = pl021_info->tx ? WRITING_U8 : WRITING_NULL;
	pl021_info->read = pl021_info->rx ? READING_U8 : READING_NULL;  

#ifdef AUD_SOC_SPI_DEBUG
	printk("aud_soc_spi] set_up_next_transfer : tx(%d)=%p:%p rx(%d)=%p:%p\n",
			pl021_info->write, pl021_info->tx, pl021_info->tx_end, 
			pl021_info->read, pl021_info->rx, pl021_info->rx_end); 
#endif
	return 0;
}


/**
 * next_transfer - Move to the Next transfer in the current spi message
 * @pl021: SSP driver private data structure
 *
 * This function moves though the linked list of spi transfers in the
 * current spi message and returns with the state of current spi
 * message i.e whether its last transfer is done(STATE_DONE) or
 * Next transfer is ready(STATE_RUNNING)
 */
static void *next_transfer(struct pl021 *pl021_info)
{
	struct spi_message *msg = pl021_info->cur_msg;
	struct spi_transfer *trans = pl021_info->cur_transfer;

	/* Move to next transfer */
	if (trans->transfer_list.next != &msg->transfers) 
	{
		pl021_info->cur_transfer =  list_entry(trans->transfer_list.next, struct spi_transfer, transfer_list);
		return STATE_RUNNING;
	}
	return STATE_DONE;
}


/**
 * This will write to TX and read from RX according to the parameters
 * set in pl021.
 */
static void readwriter(struct pl021 *pl021_info)
{

#ifdef READWRITER_DEBUG
printk("[aud_soc_spi] readwriter entry\n");
#endif
	/*
	 * The FIFO depth is different inbetween primecell variants.
	 * I believe filling in too much in the FIFO might cause
	 * errons in 8bit wide transfers on ARM variants (just 8 words
	 * FIFO, means only 8x8 = 64 bits in FIFO) at least.
	 *
	 * FIXME: currently we have no logic to account for this.
	 * perhaps there is even something broken in HW regarding
	 * 8bit transfers (it doesn't fail on 16bit) so this needs
	 * more investigation...
	 */
#ifdef READWRITER_DEBUG
	printk("[aud_soc_spi] readwriter: %s, rx: %p, rxend: %p, tx: %p, txend: %p\n",__func__, pl021_info->rx, pl021_info->rx_end, pl021_info->tx, pl021_info->tx_end);
	dev_dbg(&pl021_info->adev->dev, "%s, rx: %p, rxend: %p, tx: %p, txend: %p\n",__func__, pl021_info->rx, pl021_info->rx_end, pl021_info->tx, pl021_info->tx_end);
#endif

	/* Read as much as you can */
	while ((readw(SSP_SR(pl021_info->virtbase)) & SSP_SR_MASK_RNE) && (pl021_info->rx < pl021_info->rx_end)) 
	{
		switch (pl021_info->read) {
		case READING_NULL:
			readw(SSP_DR(pl021_info->virtbase));
			break;

		case READING_U8:
			*(u8 *) (pl021_info->rx) = readw(SSP_DR(pl021_info->virtbase)) & 0xFFU;
			break;

		case READING_U16:
			*(u16 *) (pl021_info->rx) = (u16) readw(SSP_DR(pl021_info->virtbase));
			break;

		case READING_U32:
			*(u32 *) (pl021_info->rx) = readl(SSP_DR(pl021_info->virtbase));
			break;
		}

		
#ifdef READWRITER_DEBUG
		if ((pl021_info->read) != READING_NULL) printk("readwriter: read_1 of %#x\n", *(u8 *) (pl021_info->rx));
#endif
		pl021_info->rx += 1; 
	}
	/*
	 * Write as much as you can, while keeping an eye on the RX FIFO!
	 */
	while ((readw(SSP_SR(pl021_info->virtbase)) & SSP_SR_MASK_TNF) && (pl021_info->tx < pl021_info->tx_end)) 
	{

		switch (pl021_info->write) {
		case WRITING_NULL:
			writew(0x0, SSP_DR(pl021_info->virtbase));
			break;

		case WRITING_U8:
			writew(*(u8 *) (pl021_info->tx), SSP_DR(pl021_info->virtbase));
			break;
	
		case WRITING_U16:
			writew((*(u16 *) (pl021_info->tx)), SSP_DR(pl021_info->virtbase));
			break;
		
		case WRITING_U32:
			writel(*(u32 *) (pl021_info->tx), SSP_DR(pl021_info->virtbase));
			break;
		}

#ifdef READWRITER_DEBUG
		if ((pl021_info->write) != WRITING_NULL) printk("readwriter: write of %#x\n", *(u8 *) (pl021_info->tx));
#endif
		pl021_info->tx += 1;
		/*
		 * This inner reader takes care of things appearing in the RX
		 * FIFO as we're transmitting. This will happen a lot since the
		 * clock starts running when you put things into the TX FIFO,
		 * and then things are continously clocked into the RX FIFO.
		 */
		while ((readw(SSP_SR(pl021_info->virtbase)) & SSP_SR_MASK_RNE) && (pl021_info->rx < pl021_info->rx_end)) 
		{
			switch (pl021_info->read)
			{
			case READING_NULL:
				readw(SSP_DR(pl021_info->virtbase));
				break;

			case READING_U8:
				*(u8 *) (pl021_info->rx) = readw(SSP_DR(pl021_info->virtbase)) & 0xFFU;
				break;

			case READING_U16:
				*(u16 *) (pl021_info->rx) = (u16) readw(SSP_DR(pl021_info->virtbase));
				break;

			case READING_U32:
				*(u32 *) (pl021_info->rx) = readl(SSP_DR(pl021_info->virtbase));
				break;
			}
			
		  
#ifdef READWRITER_DEBUG
		if ((pl021_info->read) != READING_NULL) printk("readwriter: read_2 of %#x\n", *(u8 *) (pl021_info->rx));
#endif
			pl021_info->rx += 1; 
		}
	}
	/*
	 * When we exit here the TX FIFO should be full and the RX FIFO
	 * should be empty
	 */
#ifdef READWRITER_DEBUG
printk("readwriter done\n");	
#endif

}



static void do_polling_transfer(void *data)
{
	struct pl021 *pl021_info = data;
	struct spi_message *message = NULL;
	struct spi_transfer *transfer = NULL;
	struct spi_transfer *previous = NULL;

	struct chip_data *chip;

#ifdef AUD_SOC_SPI_DEBUG
	printk("[aud_soc_spi] do_polling_transfer\n");
#endif
	chip = pl021_info->cur_chip;

	message = pl021_info->cur_msg;


	while (message->state != STATE_DONE) 
	{
		/* Handle for abort */

		if (message->state == STATE_ERROR) break;

		transfer = pl021_info->cur_transfer;
	
		/* Delay if requested at end of transfer */
		/**/

		if (message->state == STATE_RUNNING) 
		{
			previous = list_entry(transfer->transfer_list.prev, struct spi_transfer, transfer_list);

			if (previous->delay_usecs) udelay(previous->delay_usecs);

			if (previous->cs_change) ks8851_cs_control(SSP_CHIP_SELECT);

		} else {

			//STATE_START //
			message->state = STATE_RUNNING;

			ks8851_cs_control(SSP_CHIP_SELECT);

		}
	
		// Configuration Changing Per Transfer //
		if (set_up_next_transfer(pl021_info, transfer)) {
			// Error path //	
			message->state = STATE_ERROR;
			break;
		}
	
		/* Flush FIFOs and enable SSP */
		flush(pl021_info);
	
#ifdef AUD_SOC_SPI_DEBUG
		printk("[aud_soc_spi] do_polling_transfer: flush fifo\n");
#endif
		writew((readw(SSP_CR1(pl021_info->virtbase)) | SSP_CR1_MASK_SSE), SSP_CR1(pl021_info->virtbase));
	
#ifdef AUD_SOC_SPI_DEBUG
		printk("[aud_soc_spi] do_polling_transfer: POLLING TRANSFER ONGOING .... \n");
		dev_dbg(&pl021_info->adev->dev, "POLLING TRANSFER ONGOING ... \n");
#endif

		/* FIXME: insert a timeout so we don't hang here indefinately */
		/* resolve issue with recv-fifo overflow:
		 * first approach: disable linux rescheduling
		 * may be better: don't use complete write fifo
		 * TBD: add a max count to prevent indefinite loops
		 */
		preempt_disable();
		while (pl021_info->tx < pl021_info->tx_end || pl021_info->rx < pl021_info->rx_end) 
		{
		readwriter(pl021_info);
		}
		preempt_enable();

		/* Update total byte transfered */
		message->actual_length += pl021_info->cur_transfer->len;

		if (pl021_info->cur_transfer->cs_change) ks8851_cs_control(SSP_CHIP_DESELECT);
	
		/* Move to next transfer */
		message->state = next_transfer(pl021_info);

	}
	/* Handle end of message */
	if (message->state == STATE_DONE)
		message->status = 0;
	else
		message->status = -EIO;

	giveback(pl021_info);

#ifdef AUD_SOC_SPI_DEBUG
	printk("do_polling_transfer done \n");
#endif
	return;
}



/**
 * pump_messages - Workqueue function which processes spi message queue
 * @data: pointer to private data of SSP driver
 *
 * This function checks if there is any spi message in the queue that
 * needs processing and delegate control to appropriate function
 * do_polling_transfer()/do_interrupt_transfer()/do_dma_transfer()
 * based on the kind of the transfer
 *
 */
static void pump_messages(struct work_struct *work)
{
	struct pl021 *pl021_info = container_of(work, struct pl021, pump_messages);
	unsigned long flags;

#ifdef AUD_SOC_SPI_DEBUG
	printk("[aud_soc_spi] pump_messages\n");
#endif

	/* Lock queue and check for queue work */
	spin_lock_irqsave(&pl021_info->queue_lock, flags);
	
#ifdef AUD_SOC_SPI_DEBUG
	printk("[aud_soc_spi] pump_messages: list_empty=<%u>, queue_status=<%u>\n",list_empty(&pl021_info->queue), pl021_info->run );
#endif

	if (list_empty(&pl021_info->queue) || pl021_info->run == QUEUE_STOPPED) 
	{
		pl021_info->busy = 0;
		spin_unlock_irqrestore(&pl021_info->queue_lock, flags);
		return;
	}
	/* Make sure we are not already running a message */
	if (pl021_info->cur_msg) 
	{
#ifdef AUD_SOC_SPI_DEBUG
		printk("[aud_soc_spi] pump_messages: already running a message\n");
#endif
		spin_unlock_irqrestore(&pl021_info->queue_lock, flags);
		return;
	}
		
	/* Extract head of queue */
	pl021_info->cur_msg = list_entry(pl021_info->queue.next, struct spi_message, queue);

	list_del_init(&pl021_info->cur_msg->queue);

	pl021_info->busy = 1;
	spin_unlock_irqrestore(&pl021_info->queue_lock, flags);

	/* Initial message state */
	pl021_info->cur_msg->state = STATE_START;
	pl021_info->cur_transfer = list_entry(pl021_info->cur_msg->transfers.next, struct spi_transfer, transfer_list);

	flush(pl021_info);

#ifdef AUD_SOC_SPI_DEBUG
		printk("[aud_soc_spi] pump_messages: calling do_polling_transfer\n");
#endif
		do_polling_transfer(pl021_info);


#ifdef AUD_SOC_SPI_DEBUG
	printk("pump_messages done\n");
#endif

}


static int start_queue(struct pl021 *pl021_info)
{
	unsigned long flags;

	spin_lock_irqsave(&pl021_info->queue_lock, flags);

#ifdef AUD_SOC_SPI_DEBUG
	printk("[aud_soc_spi] start_queue: entry\n");
#endif

	if (pl021_info->run == QUEUE_RUNNING || pl021_info->busy) 
	{
		spin_unlock_irqrestore(&pl021_info->queue_lock, flags);
		return -EBUSY;
	}

	pl021_info->run = QUEUE_RUNNING;
	pl021_info->cur_msg = NULL;
	pl021_info->cur_transfer = NULL;
	pl021_info->cur_chip = NULL;

#ifdef AUD_SOC_SPI_DEBUG
	printk("[aud_soc_spi] start_queue: set pl021_info \n");
#endif

	spin_unlock_irqrestore(&pl021_info->queue_lock, flags);

	queue_work(pl021_info->workqueue, &pl021_info->pump_messages);
	
#ifdef AUD_SOC_SPI_DEBUG	
	printk("[aud_soc_spi] start_queue: ok and exit\n");
#endif
	return 0;
}



static int stop_queue(struct pl021 *pl021_info)
{
	unsigned long flags;
	unsigned limit = 500;
	int status = 0;

#ifdef AUD_SOC_SPI_DEBUG
	printk("[aud_soc_spi] stop_queue: entry\n");
#endif

	spin_lock_irqsave(&pl021_info->queue_lock, flags);

	/* This is a bit lame, but is optimized for the common execution path.
	 * A wait_queue on the pl021_info->busy could be used, but then the common
	 * execution path (pump_messages) would be required to call wake_up or
	 * friends on every SPI message. Do this instead */
	pl021_info->run = QUEUE_STOPPED;
	while (!list_empty(&pl021_info->queue) && pl021_info->busy && limit--) 
	{
		spin_unlock_irqrestore(&pl021_info->queue_lock, flags);
		msleep(10);
		spin_lock_irqsave(&pl021_info->queue_lock, flags);
	}

	if (!list_empty(&pl021_info->queue) || pl021_info->busy)
		status = -EBUSY;

	spin_unlock_irqrestore(&pl021_info->queue_lock, flags);

#ifdef AUD_SOC_SPI_DEBUG
	printk("[aud_soc_spi] stop_queue: ok and exit\n");
#endif

	return status;
}

static int destroy_queue(struct pl021 *pl021_info)
{
	int status;

	status = stop_queue(pl021_info);
	/* we are unloading the module or failing to load (only two calls
	 * to this routine), and neither call can handle a return value.
	 * However, destroy_workqueue calls flush_workqueue, and that will
	 * block until all work is done.  If the reason that stop_queue
	 * timed out is that the work will never finish, then it does no
	 * good to call destroy_workqueue, so return anyway. */
	if (status != 0) return status;

	destroy_workqueue(pl021_info->workqueue);

	return 0;
}


/**
 * giveback - current spi_message is over, schedule next message and call
 * callback of this message. Assumes that caller already
 * set message->status; dma and pio irqs are blocked
 * @pl021: SSP driver private data structure
 */
static void giveback(struct pl021 *pl021_info)
{
	struct spi_transfer *last_transfer;
	unsigned long flags;
	struct spi_message *msg;
	struct spi_message *next_msg;

#ifdef AUD_SOC_SPI_DEBUG
	printk("[aud_soc_spi] giveback: entry\n");
#endif

	spin_lock_irqsave(&pl021_info->queue_lock, flags);

	msg = pl021_info->cur_msg;
	pl021_info->cur_msg = NULL;
	pl021_info->cur_transfer = NULL;
	pl021_info->cur_chip = NULL;

	queue_work(pl021_info->workqueue, &pl021_info->pump_messages);

	spin_unlock_irqrestore(&pl021_info->queue_lock, flags);

	last_transfer = list_entry(msg->transfers.prev,
					struct spi_transfer,
					transfer_list);


	/* Delay if requested before any change in chip select */
	if (last_transfer->delay_usecs)
		/*
		 * FIXME: This runs in interrupt context.
		 * Is this really smart?
		 */
		udelay(last_transfer->delay_usecs);

	/*
	 * Drop chip select UNLESS cs_change is true or we are returning
	 * a message with an error, or next message is for another chip
	 */
	
	if (!last_transfer->cs_change)
	{
			ks8851_cs_control(SSP_CHIP_DESELECT);
	}
	else 
	{

		/* Holding of cs was hinted, but we need to make sure
		 * the next message is for the same chip.  Don't waste
		 * time with the following tests unless this was hinted.
		 *
		 * We cannot postpone this until pump_messages, because
		 * after calling msg->complete (below) the driver that
		 * sent the current message could be unloaded, which
		 * could invalidate the cs_control() callback...
		 */

		/* get a pointer to the next message, if any */
		spin_lock_irqsave(&pl021_info->queue_lock, flags);

		if (list_empty(&pl021_info->queue))
			next_msg = NULL;
		else
			next_msg = list_entry(pl021_info->queue.next, struct spi_message, queue);

		spin_unlock_irqrestore(&pl021_info->queue_lock, flags);
		/* see if the next and current messages point
		 * to the same chip
		 */
		if (next_msg && next_msg->spi != msg->spi) next_msg = NULL;

	if (!next_msg || msg->state == STATE_ERROR) ks8851_cs_control(SSP_CHIP_DESELECT);

	}


	if (msg->complete) msg->complete(msg->context); 

	/* This message is completed, so let's turn off the clock! */
	
#ifdef AUD_SOC_SPI_DEBUG
	printk("[aud_soc_spi] giveback: exit\n");
#endif
}

/**
 * pump_transfers - Tasklet function which schedules next interrupt transfer
 * when running in interrupt transfer mode.
 * @data: SSP driver private data structure
 *
 */
static void pump_transfers(unsigned long data)
{
	struct pl021 *pl021_info = (struct pl021 *) data;
	struct spi_message *message = NULL;
	struct spi_transfer *transfer = NULL;
	struct spi_transfer *previous = NULL;

	/* Get current state information */
	message = pl021_info->cur_msg;
	transfer = pl021_info->cur_transfer;

	/* Handle for abort */
	if (message->state == STATE_ERROR) {
		message->status = -EIO;
		giveback(pl021_info);
		return;
	}

	/* Handle end of message */
	if (message->state == STATE_DONE) {
		message->status = 0;
		giveback(pl021_info);
		return;
	}

	/* Delay if requested at end of transfer before CS change */
	if (message->state == STATE_RUNNING) {
		previous = list_entry(transfer->transfer_list.prev, struct spi_transfer, transfer_list);
		if (previous->delay_usecs)
			/*
			 * FIXME: This runs in interrupt context.
			 * Is this really smart?
			 */
			udelay(previous->delay_usecs);

		/* Drop chip select only if cs_change is requested */
			if (previous->cs_change)ks8851_cs_control(SSP_CHIP_SELECT);

	} else {
		/* STATE_START */
		message->state = STATE_RUNNING;
	}

	if (set_up_next_transfer(pl021_info, transfer)) 
	{
		message->state = STATE_ERROR;
		message->status = -EIO;
		giveback(pl021_info);
		return;
	}

	/* Flush the FIFOs and let's go! */
	flush(pl021_info);
	writew((u16)ENABLE_ALL_INTERRUPTS, SSP_IMSC(pl021_info->virtbase));
}

static int __init init_queue(struct pl021 *pl021_info)
{
#ifdef AUD_SOC_SPI_DEBUG
	printk("[aud_soc_spi] init_queue: entry\n");
#endif

	INIT_LIST_HEAD(&pl021_info->queue);
	spin_lock_init(&pl021_info->queue_lock);

	pl021_info->run = QUEUE_STOPPED;
	pl021_info->busy = 0;

	tasklet_init(&pl021_info->pump_transfers, pump_transfers, (unsigned long)pl021_info);

	INIT_WORK(&pl021_info->pump_messages, pump_messages);

	pl021_info->workqueue = create_singlethread_workqueue(dev_name(pl021_info->master->dev.parent));

	if (pl021_info->workqueue == NULL)
	{
		printk("[aud_soc_spi] init_queue: setup workqueue failed\n");
		return -EBUSY;
	}
#ifdef AUD_SOC_SPI_DEBUG
	printk("[aud_soc_spi] init_queue: ok and exit\n");
#endif
	
	return 0;
}



static int aud_soc_spi_transfer(struct spi_device *spi, struct spi_message *msg)
{
	struct pl021 		*pl021_info;
	unsigned long 		flags;

	pl021_info = spi_master_get_devdata(spi->master);

#ifdef AUD_SOC_SPI_DEBUG
        printk("[aud_soc_spi] aud_soc_spi_transfer: msg=<%p>, queue=<%u>\n", msg, (pl021_info->run) );
#endif

	spin_lock_irqsave(&pl021_info->queue_lock, flags);

	if (pl021_info->run == QUEUE_STOPPED) {
		spin_unlock_irqrestore(&pl021_info->queue_lock, flags);
		return -ESHUTDOWN;
	}

	msg->actual_length = 0; 
	msg->status = -EINPROGRESS; 
	msg->state = STATE_START; 


	// ------------- for filling the queue and polling ------------
	list_add_tail(&msg->queue, &pl021_info->queue);
	
	queue_work(pl021_info->workqueue, &pl021_info->pump_messages);


	spin_unlock_irqrestore(&pl021_info->queue_lock, flags);

#ifdef AUD_SOC_SPI_DEBUG
printk("[aud_soc_spi] aud_soc_spi_transfer done\n");
#endif
	return 0;
}

static void aud_soc_spi_cleanup(struct spi_device *spi)
{
	struct pl021_spi	*pl021_info;

	pl021_info = spi_master_get_devdata(spi->master);

#ifdef AUD_SOC_SPI_DEBUG
        printk("[aud_soc_spi] aud_soc_spi_cleanup\n");
#endif
}

static irqreturn_t aud_soc_spi_interrupt(int irq, void *dev_id)
{
	int ret;
    
#ifdef AUD_SOC_SPI_DEBUG
	printk("[aud_soc_spi] aud_soc_spi_interrupt\n");
#endif
	ret = IRQ_HANDLED;
	return(ret);
}



static int __init aud_soc_spi_probe(struct platform_device *pdev)
{
	struct resource		*regs;
	int			irq;
	int			ret;
	struct spi_master	*master;
	struct pl021		*pl021_info;
	int 			status = 0;


#ifdef AUD_SOC_SPI_DEBUG
	printk("[aud_soc_spi] aud_soc_spi_probe\n");
#endif
	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs)
		return -ENXIO;
#ifdef AUD_SOC_SPI_DEBUG
	printk("[aud_soc_spi] aud_soc_spi_probe: resource=<%#x:%#x>\n",
		regs->start, regs->end);
#endif

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
#ifdef AUD_SOC_SPI_DEBUG
	printk("[aud_soc_spi] aud_soc_spi_probe: irq=<%d>\n", irq);
#endif
	master = spi_alloc_master(&pdev->dev, sizeof(struct pl021));
#ifdef AUD_SOC_SPI_DEBUG	
	printk("[aud_soc_spi] aud_soc_spi_probe: master=<%p>\n", master);
#endif
	if (!master)
		goto out_free;

	// the spi->mode bits understood by this driver: //
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH;

	master->bus_num = pdev->id;
	master->num_chipselect = 1;  // max numbers of chipselects
	master->setup = aud_soc_spi_setup;
	master->transfer = aud_soc_spi_transfer;
	master->cleanup = aud_soc_spi_cleanup;
	platform_set_drvdata(pdev, master);

        pl021_info = spi_master_get_devdata(master);
	pl021_info->master = master;

	//
	//  Scratch buffer is used for throwaway rx and tx data.
	//  It's coherent to minimize dcache pollution.
	//

	pl021_info->buffer = dma_alloc_coherent(&pdev->dev, BUFFER_SIZE,
					&pl021_info->buffer_dma, GFP_KERNEL);
#ifdef AUD_SOC_SPI_DEBUG
	printk("[aud_soc_spi] aud_soc_spi_probe: dma=<%p(%u)>\n", pl021_info->buffer,
            pl021_info->buffer_dma);
#endif
	if (!pl021_info->buffer) {
#ifdef AUD_SOC_SPI_DEBUG
		printk("[aud_soc_spi] aud_soc_spi_probe: dma_alloc_problem\n");
#endif
		goto out_free;
        }

	spin_lock_init(&pl021_info->lock);
	INIT_LIST_HEAD(&pl021_info->queue);
	pl021_info->pdev = pdev;

	pl021_info->hwPtr = ioremap(regs->start, (regs->end - regs->start) + 1);
#ifdef AUD_SOC_SPI_DEBUG	
	printk("[aud_soc_spi] aud_soc_spi_probe: ioremap=<%p>\n", pl021_info->hwPtr);
#endif

	pl021_info->virtbase = pl021_info->hwPtr;
#ifdef AUD_SOC_SPI_DEBUG
	printk("[aud_soc_spi] aud_soc_spi_probe: set pl021_info->virtbase=<%p>\n", pl021_info->hwPtr);
#endif	

	if (IS_ERR(pl021_info->clk)) 
	{
	status = PTR_ERR(pl021_info->clk);
#ifdef AUD_SOC_SPI_DEBUG	
	printk("[aud_soc_spi] aud_soc_spi_probe: could not retrieve SSP/SPI bus clock\n");
		dev_err(&pdev->dev, "could not retrieve SSP/SPI bus clock\n");
#endif
		
	}
#ifdef AUD_SOC_SPI_DEBUG
	printk("[aud_soc_spi] aud_soc_spi_probe: pl022->clk=<%p>\n", pl021_info->clk);
#endif
	

	if (!pl021_info->hwPtr) goto out_free_buffer;
	pl021_info->irq = irq;
	
	ret = request_irq(irq, aud_soc_spi_interrupt, 0,dev_name(&pdev->dev), master);

#ifdef AUD_SOC_SPI_DEBUG
               printk("[aud_soc_spi] aud_soc_spi_probe: req_irq<(%d)>\n", ret);
#endif

	if (ret)goto out_unmap_regs;


	// Initialize and start queue 
	status = init_queue(pl021_info);
	if (status != 0) {
#ifdef AUD_SOC_SPI_DEBUG
		printk("[aud_soc_spi] probe: problem initializing queue\n");
#endif
		goto err_init_queue;
	}


	status = start_queue(pl021_info);
	if (status != 0) {
#ifdef AUD_SOC_SPI_DEBUG
		printk("[aud_soc_spi] probe: problem starting queue\n");
#endif
		goto err_start_queue;
	}


	ret = spi_register_master(master);
#ifdef AUD_SOC_SPI_DEBUG
	printk("[aud_soc_spi] aud_soc_spi_probe: spi_register<(%d)>\n", ret);
#endif
	if (ret) goto out_reset_hw;

#ifdef AUD_SOC_SPI_DEBUG
        printk("[aud_soc_spi] aud_soc_spi_probe: successful\n");
#endif
	return 0;


err_start_queue:
err_init_queue:
	destroy_queue(pl021_info);


out_unmap_regs:
out_free_buffer:
out_reset_hw:
out_free:

#ifdef AUD_SOC_SPI_DEBUG
	printk("[aud_soc_spi] aud_soc_spi_probe: problem\n");
#endif
	return(-ENXIO);
}


static int __exit aud_soc_spi_remove(struct platform_device *pdev)
{
#ifdef AUD_SOC_SPI_DEBUG
        printk("[aud_soc_spi] aud_soc_spi_probe: remove\n");
#endif

	return 0;

}

static int aud_soc_spi_suspend(struct platform_device *pdev, pm_message_t mesg)
{
#ifdef AUD_SOC_SPI_DEBUG
        printk("[aud_soc_spi] aud_soc_spi_probe: suspend\n");
#endif

	return 0;
}

static int aud_soc_spi_resume(struct platform_device *pdev)
{
#ifdef AUD_SOC_SPI_DEBUG
        printk("[aud_soc_spi] aud_soc_spi_probe: resume\n");
#endif
	return 0;
}


static struct platform_driver aud_soc_spi_driver = {
	.driver		= {
		.name	= "aud_soc_spi",
		.owner	= THIS_MODULE,
		},
	.probe		= aud_soc_spi_probe, 
	.suspend	= aud_soc_spi_suspend,
	.resume		= aud_soc_spi_resume,
	.remove		= __exit_p(aud_soc_spi_remove),
};


/*
 *
 * aud_soc_init_spi - driver installation routine
 *
 */
static int __init aud_soc_spi_init (void)
{
    int retVal;

#ifdef AUD_SOC_SPI_DEBUG
    printk("[aud_soc_spi] aud_soc_spi_probe: init\n");
#endif

    retVal = platform_driver_register(&aud_soc_spi_driver);
#ifdef AUD_SOC_SPI_DEBUG
    printk("[aud_soc_spi] aud_soc_spi_probe: leave init (%d)\n", retVal);
#endif

	return 0;
}

/*
 *
 * aud_soc_exit_spi - driver uninstallation routine
 *
 */
static void __exit aud_soc_spi_exit (void)
{
#ifdef AUD_SOC_SPI_DEBUG	
    printk("[aud_soc_spi] aud_soc_spi_probe: driver unloaded (exit)\n");
#endif
}

module_init(aud_soc_spi_init);
module_exit(aud_soc_spi_exit);
 
 
 
