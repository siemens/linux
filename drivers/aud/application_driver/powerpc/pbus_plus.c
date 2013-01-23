/*
 *  linux/drivers/aud/application_driver/powerpc/pbus_plus.c
 *
 *  Copyright (C) 2008-2010 Manfred Neugebauer <manfred.neugebauer@siemens.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Driver for ppc based pbus plus driver for Siemens boards
 *
 * some parts of this code is taken from
 * Copyright (C) 2001 Alessandro Rubini and Jonathan Corbet
 * Copyright (C) 2001 O'Reilly & Associates
 *
 * The source code in this file can be freely used, adapted,
 * and redistributed in source or binary form, so long as an
 * acknowledgment appears in derived source files.  The citation
 * should list that the code comes from the book "Linux Device
 * Drivers" by Alessandro Rubini and Jonathan Corbet, published
 * by O'Reilly & Associates.   No warranty is attached;
 * we cannot take responsibility for errors or fitness for use.
 *
 */

#include <linux/fs.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/slab.h>	
#include <linux/mm.h>
#include <linux/errno.h>
#include <linux/pci.h>    /* Zugriff auf PCI Konfiguration */
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/sem.h>

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/bcd.h>
#include <linux/list.h>

#include <linux/aud/rt_driver.h>
#include <linux/aud/rt_base.h>

#define MAJOR_NUMBER	208
#define PRIVATE_DRIVER_NAME "ppc_pbus_plus"

#ifdef MODULE
MODULE_AUTHOR("Manfred Neugebauer");
MODULE_DESCRIPTION( "Linux driver for ppc based pbus_plus device" );
MODULE_LICENSE("GPL");	     /* "Proprietary" returns warning during loading */
#endif

//#define DEBUG_DEVICE_PCI_ACCESS    1

int set_irq_icu_prio(int irq_nr, int prio);

#ifndef __LITTLE_ENDIAN
#define __LITTLE_ENDIAN		0x1234
#endif
#ifndef __BIG_ENDIAN
#define __BIG_ENDIAN		0x4321
#endif
#define __BYTE_ORDER		__BIG_ENDIAN

#if __BYTE_ORDER == __LITTLE_ENDIAN
// we may get a separate rt/endian.h
static inline int32_t btoh_int32(int32_t biglong) {
	int32_t littlelong;
	char *inp = (char *)&biglong;
	char *outp = (char *)&littlelong;

	outp[0] = inp[3];
	outp[1] = inp[2];
	outp[2] = inp[1];
	outp[3] = inp[0];
	return littlelong;
}
static inline uint32_t btoh_uint32(uint32_t biglong) {
	uint32_t littlelong;
	char *inp = (char *)&biglong;
	char *outp = (char *)&littlelong;

	outp[0] = inp[3];
	outp[1] = inp[2];
	outp[2] = inp[1];
	outp[3] = inp[0];
	return littlelong;
}
static inline int32_t htob_int32(int32_t hostlong) {
	int32_t biglong;
	char *inp = (char *)&hostlong;
	char *outp = (char *)&biglong;

	outp[0] = inp[3];
	outp[1] = inp[2];
	outp[2] = inp[1];
	outp[3] = inp[0];
	return biglong;
}
static inline uint32_t htob_uint32(uint32_t hostlong) {
	uint32_t biglong;
	char *inp = (char *)&hostlong;
	char *outp = (char *)&biglong;

	outp[0] = inp[3];
	outp[1] = inp[2];
	outp[2] = inp[1];
	outp[3] = inp[0];
	return biglong;
}
#elif __BYTE_ORDER == __BIG_ENDIAN

#define btoh_int32(x) (x)
#define btoh_uint32(x) (x)
#define htob_int32(x) (x)
#define htob_uint32(x) (x)

#endif

// from csir/interrupt_priorities.h
#define PBP_CSIR_INTERRUPT_PRIORITY_HIGH	(uint8_t)0
#define PBP_CSIR_INTERRUPT_PRIORITY_LOW		(uint8_t)1


// issues:
// how do we get a target independant definition of events
// eigentlich sollten die Events durch die Anwendung gepraegt sein und 
// nicht durch ein (erstes) Target
// this stuff should be moved to a header file common with the application

// event definitions
#define ADN_INT_CORE_OFFSET	0
#define ADN_IRQ_CORE_IRQ_NR     8     ///< Number of interrupt inputs on the mips core
#define ADN_IRQ_ICU_IRQ_NR      160     ///< Number of interrupt inputs on the icu unit
#define ADN_IRQ_ICU_FIQ_NR	8	///< Number of interrupt inputs on the FIQ unit

#define ADN_INT_ICU_IRQ_OFFSET  ADN_INT_CORE_OFFSET + ADN_IRQ_CORE_IRQ_NR
#define ADN_INT_ICU_FIQ_OFFSET  ADN_INT_ICU_IRQ_OFFSET + ADN_IRQ_ICU_IRQ_NR
#define ADN_INT_CSIR_OFFSET	(ADN_INT_ICU_FIQ_OFFSET + ADN_IRQ_ICU_FIQ_NR)
#define ADN_INT_CSIR_S1_OFFSET	ADN_INT_CSIR_OFFSET
#define ADN_INT_CSIR_S2_OFFSET	(ADN_INT_CSIR_S1_OFFSET + 32)

/**************************************************************************
 * bit positions of the CSIR slave interrupt flags	               	  *
 * represents the bit position within the interrupt registers		  *
 * 31 means leftmost bit and 0 means rightmost bit			  *
 *************************************************************************/
// S1 bits
#define PBP_IRQ_S1_INT_S2_REG_MOREFOLLOW		 0
#define PBP_IRQ_S1_INT_SEA_DOUT_TRIG_2			 1
#define PBP_IRQ_S1_INT_SEA_DOUT_TRIG_1			 2
#define PBP_IRQ_S1_INT_CLK_5				 3
#define PBP_IRQ_S1_INT_CLK_4				 4
#define PBP_IRQ_S1_INT_CLK_3				 5
#define PBP_IRQ_S1_INT_CLK_2				 6
#define PBP_IRQ_S1_INT_CLK_1				 7
#define PBP_IRQ_S1_INT_CLK_0				 8
#define PBP_IRQ_S1_INT_DIFF_CYCLE_TIMER			 9
#define PBP_IRQ_S1_INT_NEW_CYCLE			10
#define PBP_IRQ_S1_INT_S_DATACLOCK_SYNC			11
#define PBP_IRQ_S1_INT_S_PHASE_VIOLATION		12
#define PBP_IRQ_S1_INT_JITTER_RANGE_VIOLATION		13
#define PBP_IRQ_S1_INT_CLOCK_FRAME_REC			14
#define PBP_IRQ_S1_INT_TIME_FRAME_REC			15
#define PBP_IRQ_S1_AZK_REC_1_INT			16
#define PBP_IRQ_S1_AZK_REC_0_INT			17
#define PBP_IRQ_S1_AZK_REC_CMD_INT			18
#define PBP_IRQ_S1_AZK_SND_INT				19
#define PBP_IRQ_S1_AZK_SND_CMD_INT			20
#define PBP_IRQ_S1_ALARMCREATETYPEINT			21
#define PBP_IRQ_S1_LALARMCREATEBUSYINT			22
#define PBP_IRQ_S1_HALARMCREATEBUSYINT			23
#define PBP_IRQ_S1_NEWSUBMODODIS			24
#define PBP_IRQ_S1_NEWGLOBODIS				25
#define PBP_IRQ_S1_SEA_SYSEDATA2INT			26
#define PBP_IRQ_S1_SEA_SYSEDATA1INT			27
#define PBP_IRQ_S1_SEA_EDATAINT				28
#define PBP_IRQ_S1_SEA_SYSADATA2INT			29
#define PBP_IRQ_S1_SEA_SYSADATA1INT			30
#define PBP_IRQ_S1_SEA_ADATAINT				31

#define MAX_NR_OF_S1_INTR		(PBP_IRQ_S1_SEA_ADATAINT +1)

// S2 bits
// bits 0...9 are unused
#define PBP_IRQ_S2_KOBUF_DMA_XFR_END			10
#define PBP_IRQ_S2_APP_IR_READY_INT			11
// bits 12...15 are unused
#define PBP_IRQ_S2_INT_MBF_INT_PBUSP_RAM		16
#define PBP_IRQ_S2_INT_OBF_INT_PBUSP_RAM		17
#define PBP_IRQ_S2_INT_TIME_SYNC_ERROR			18
#define PBP_IRQ_S2_INT_TIME_LENGTH_ERR			19
#define PBP_IRQ_S2_INT_CLOCK_SYNC_ERROR			20
#define PBP_IRQ_S2_INT_CLOCK_LENGTH_ERR			21
// bits 22...26 are unused
#define PBP_IRQ_S2_BUS_ACT_STATE_INT			27
#define PBP_IRQ_S2_PORT_STAT_CHANGE_INT			28
#define PBP_IRQ_S2_NEWMODADDR				29
#define PBP_IRQ_S2_INT_SEA_DOUT_TRIG_4			30
#define PBP_IRQ_S2_INT_SEA_DOUT_TRIG_3			31

#define MAX_NR_OF_S2_INTR		(PBP_IRQ_S2_INT_SEA_DOUT_TRIG_3 + 1)

#define MAX_NR_OF_CSIR_INTR		(MAX_NR_OF_S1_INTR + MAX_NR_OF_S2_INTR)


#define PBP_INT_CSIR_S_ALARMCREATETYPEINT	(ADN_INT_CSIR_S1_OFFSET + PBP_IRQ_S1_ALARMCREATETYPEINT) ///< Identifier of AlarmTypeFlag reset because of AlarmQuit
#define PBP_INT_CSIR_S_HALARMCREATEBUSYINT	(ADN_INT_CSIR_S1_OFFSET + PBP_IRQ_S1_HALARMCREATEBUSYINT) ///< Identifier of BufferBusy reset because of AlarmInfoRdHigh
#define PBP_INT_CSIR_S_LALARMCREATEBUSYINT	(ADN_INT_CSIR_S1_OFFSET + PBP_IRQ_S1_LALARMCREATEBUSYINT) ///< Identifier of BufferBusy reset because of AlarmInfoRdLow
#define PBP_INT_CSIR_S_SEA_ADATAINT		(ADN_INT_CSIR_S1_OFFSET + PBP_IRQ_S1_SEA_ADATAINT) ///< Identifier of interrupt when system output range has been written
#define PBP_INT_CSIR_S_INT_SEA_DOUT_TRIG_2	(ADN_INT_CSIR_S1_OFFSET + PBP_IRQ_S1_INT_SEA_DOUT_TRIG_2)  ///< Identifier of data trigger 2 interrupt
#define PBP_INT_CSIR_S_INT_SEA_DOUT_TRIG_1	(ADN_INT_CSIR_S1_OFFSET + PBP_IRQ_S1_INT_SEA_DOUT_TRIG_1)  ///< Identifier of data trigger 1 interrupt
#define PBP_INT_CSIR_S_INT_SEA_DOUT_TRIG_4 	(ADN_INT_CSIR_S2_OFFSET + PBP_IRQ_S2_INT_SEA_DOUT_TRIG_4) ///< Identifier of data trigger 4 interrupt
#define PBP_INT_CSIR_S_INT_SEA_DOUT_TRIG_3	(ADN_INT_CSIR_S2_OFFSET + PBP_IRQ_S2_INT_SEA_DOUT_TRIG_3) ///< Identifier of data trigger 3 interrupt
#define PBP_INT_CSIR_S_INT_JITTER_RANGE_VIOLATION (ADN_INT_CSIR_S1_OFFSET + PBP_IRQ_S1_INT_JITTER_RANGE_VIOLATION) ///< Identifier of interrupt when jitter is to big
#define PBP_INT_CSIR_S_INT_CLOCK_FRAME_REC	(ADN_INT_CSIR_S1_OFFSET + PBP_IRQ_S1_INT_CLOCK_FRAME_REC) ///< Identifier of clock frame received interrupt
#define PBP_INT_CSIR_S_INT_CLOCK_SYNC_ERROR	(ADN_INT_CSIR_S2_OFFSET + PBP_IRQ_S2_INT_CLOCK_SYNC_ERROR) ///< Identifier of interrupt when no correct clock frame received while Clock_Sync_Time
#define PBP_INT_CSIR_S_INT_CLOCK_LENGTH_ERR	(ADN_INT_CSIR_S2_OFFSET + PBP_IRQ_S2_INT_CLOCK_LENGTH_ERR) ///< Identifier of interrupt when received clock frame couldnt be storage because of to less RAM
#define PBP_INT_CSIR_S_INT_S_DATACLOCK_SYNC	(ADN_INT_CSIR_S1_OFFSET + PBP_IRQ_S1_INT_S_DATACLOCK_SYNC)
#define PBP_INT_CSIR_S_INT_S_PHASE_VIOLATION	(ADN_INT_CSIR_S1_OFFSET + PBP_IRQ_S1_INT_S_PHASE_VIOLATION) 
#define PBP_INT_CSIR_S_BUS_ACT_STATE_INT	(ADN_INT_CSIR_S2_OFFSET + PBP_IRQ_S2_BUS_ACT_STATE_INT) ///< Identifier of interrupt when activity on bus
#define PBP_INT_CSIR_S_NEWSUBMODODIS		(ADN_INT_CSIR_S1_OFFSET + PBP_IRQ_S1_NEWSUBMODODIS) ///< Identifier of submodule granular ODIS telegram received
#define PBP_INT_CSIR_S_NEWGLOBODIS		(ADN_INT_CSIR_S1_OFFSET + PBP_IRQ_S1_NEWGLOBODIS) ///< Identifier of global ODIS telegram received
#define PBP_INT_CSIR_S_AZK_REC_0_INT		(ADN_INT_CSIR_S1_OFFSET + PBP_IRQ_S1_AZK_REC_0_INT)
#define PBP_INT_CSIR_S_AZK_REC_1_INT		(ADN_INT_CSIR_S1_OFFSET + PBP_IRQ_S1_AZK_REC_1_INT)
#define PBP_INT_CSIR_S_AZK_SND_INT		(ADN_INT_CSIR_S1_OFFSET + PBP_IRQ_S1_AZK_SND_INT)
#define PBP_INT_CSIR_S_PORT_STAT_CHANGE_INT	(ADN_INT_CSIR_S2_OFFSET + PBP_IRQ_S2_PORT_STAT_CHANGE_INT) ///< Identifier of interrupt when changing port state
#define PBP_INT_CSIR_S_NEWMODADDR		(ADN_INT_CSIR_S2_OFFSET + PBP_IRQ_S2_NEWMODADDR) ///< Identifier of interrupt when module address changed
#define PBP_INT_CSIR_S_INT_TIME_FRAME_REC	(ADN_INT_CSIR_S1_OFFSET + PBP_IRQ_S1_INT_TIME_FRAME_REC) ///< Identifier of time frame received interrupt
#define PBP_INT_CSIR_S_INT_TIME_SYNC_ERROR	(ADN_INT_CSIR_S2_OFFSET + PBP_IRQ_S2_INT_TIME_SYNC_ERROR) ///< Identifier of interrupt when no correct time frame received while Time_Sync_Time
#define PBP_INT_CSIR_S_INT_TIME_LENGTH_ERR	(ADN_INT_CSIR_S2_OFFSET + PBP_IRQ_S2_INT_TIME_LENGTH_ERR) ///< Identifier of interrupt when received time frame couldnt be storage because of to less RAM
#define PBP_INT_CSIR_S_KOBUF_DMA_XFR_END	(ADN_INT_CSIR_S2_OFFSET + PBP_IRQ_S2_KOBUF_DMA_XFR_END) ///< Identifier of transfer end interrupt from DMA
#define PBP_INT_CSIR_S_APP_IR_READY_INT		(ADN_INT_CSIR_S2_OFFSET + PBP_IRQ_S2_APP_IR_READY_INT)
#define PBP_INT_CSIR_S_INT_MBF_INT_PBUSP_RAM	(ADN_INT_CSIR_S2_OFFSET + PBP_IRQ_S2_INT_MBF_INT_PBUSP_RAM)


#define ADN_ADDITIONAL_IDENTIFIER_OFFSET	(ADN_INT_CSIR_S2_OFFSET + 32) //Offset of non-interrupt-identifiers handled as event  
#define PBP_LOW_ALARM_ACKNOWLEDGEMENT_RECEIVED       (ADN_ADDITIONAL_IDENTIFIER_OFFSET + 0)
#define MAX_NR_OF_SOFT_INTR                 (PBP_LOW_ALARM_ACKNOWLEDGEMENT_RECEIVED - ADN_ADDITIONAL_IDENTIFIER_OFFSET + 1)

// idea: use some type of magic information to prevent an accidential access of our driver
#define PBP_INT_ENABLE   		1
#define PBP_INT_DISABLE  		2
#define PBP_INT_CLEAR    		3
#define PBP_INT_EOI      		4
#define PBP_CONFIG_PRIO  		5
#define PBP_HANDLE_PCI_DEV_ADDR 	6
#define PBP_HANDLE_PCI_DEV_ADDR_RELEASE 7
#define PBP_HANDLE_PCI_DEV_ADDR_ACCESS	99

// description to request a kernel / user memory accessible from a pci target (ms-asic)
struct pbp_handle_pci_dev_info {
    unsigned pciLen;
    unsigned pciAddress;
    unsigned ahbAddress;
    int deviceIndex;
    unsigned fileOffset;
};

#define WORK_WITH_RT		1
/*
 * the standard powerpc interrupt handling of external interrupts doesn't
 * recognize low-to-high edge changes as an interrupt.
 * Therefore, we must use the gpio based interrupt handling.
 * However, this handling signals every edge change and both Pbus-Plus
 * interrupts (high and low) are signalled with the same interrupt number.
 *
 * for better understanding
 * pbus_high interrupt uses either external interrupt irq[4] or gpio2[16]
 * pbus_low interrupt uses either external interrupt irq[5] or gpio2[17]
 * irq[4] has global interrupt number 20
 * irq[5] has gloabl interrupt number 21
 * gpio2 interrupt number is 75
 */
#define WORK_WITH_GPIO_EVENTS   1
#ifdef WORK_WITH_GPIO_EVENTS
static int gpio_low_bit, gpio_high_bit;
#endif
#define NO_OF_EVENTS		(MAX_NR_OF_CSIR_INTR + MAX_NR_OF_SOFT_INTR)

// this object must be alive for the complete time
// events are used.
// entries according the interrupt numbers of the pbusplus interrupt controller
static struct rt_event ppdrv_ev_arr[NO_OF_EVENTS] = {
	{ .ev_id = 0 },						// index S1-0
	{ .ev_id = PBP_INT_CSIR_S_INT_SEA_DOUT_TRIG_2 },
	{ .ev_id = PBP_INT_CSIR_S_INT_SEA_DOUT_TRIG_1 },
	{ .ev_id = 0 },
	{ .ev_id = 0 },
	{ .ev_id = 0 },
	{ .ev_id = 0 },
	{ .ev_id = 0 },
	{ .ev_id = 0 },						// index S1-8
	{ .ev_id = 0 },
	{ .ev_id = 0 },
	{ .ev_id = PBP_INT_CSIR_S_INT_S_DATACLOCK_SYNC },
	{ .ev_id = PBP_INT_CSIR_S_INT_S_PHASE_VIOLATION },
	{ .ev_id = PBP_INT_CSIR_S_INT_JITTER_RANGE_VIOLATION },
	{ .ev_id = PBP_INT_CSIR_S_INT_CLOCK_FRAME_REC },
	{ .ev_id = PBP_INT_CSIR_S_INT_TIME_FRAME_REC },
	{ .ev_id = PBP_INT_CSIR_S_AZK_REC_1_INT },		// index S1-16
	{ .ev_id = PBP_INT_CSIR_S_AZK_REC_0_INT },
	{ .ev_id = 0 },
	{ .ev_id = PBP_INT_CSIR_S_AZK_SND_INT },
	{ .ev_id = 0 },
	{ .ev_id = PBP_INT_CSIR_S_ALARMCREATETYPEINT },
	{ .ev_id = PBP_INT_CSIR_S_LALARMCREATEBUSYINT },
	{ .ev_id = PBP_INT_CSIR_S_HALARMCREATEBUSYINT },
	{ .ev_id = PBP_INT_CSIR_S_NEWSUBMODODIS },		// index S2-24
	{ .ev_id = PBP_INT_CSIR_S_NEWGLOBODIS },
	{ .ev_id = 0 },
	{ .ev_id = 0 },
	{ .ev_id = 0 },
	{ .ev_id = 0 },
	{ .ev_id = 0 },
	{ .ev_id = PBP_INT_CSIR_S_SEA_ADATAINT },

	{ .ev_id = 0 },						// index S2-0
	{ .ev_id = 0 },
	{ .ev_id = 0 },
	{ .ev_id = 0 },
	{ .ev_id = 0 },
	{ .ev_id = 0 },
	{ .ev_id = 0 },
	{ .ev_id = 0 },
	{ .ev_id = 0 },						// index S2-8
	{ .ev_id = 0 },
	{ .ev_id = PBP_INT_CSIR_S_KOBUF_DMA_XFR_END },
	{ .ev_id = PBP_INT_CSIR_S_APP_IR_READY_INT },
	{ .ev_id = 0 },
	{ .ev_id = 0 },
	{ .ev_id = 0 },
	{ .ev_id = 0 },
	{ .ev_id = PBP_INT_CSIR_S_INT_MBF_INT_PBUSP_RAM },	// index S2-16
	{ .ev_id = 0 },
	{ .ev_id = PBP_INT_CSIR_S_INT_TIME_SYNC_ERROR },
	{ .ev_id = PBP_INT_CSIR_S_INT_TIME_LENGTH_ERR },
	{ .ev_id = PBP_INT_CSIR_S_INT_CLOCK_SYNC_ERROR },
	{ .ev_id = PBP_INT_CSIR_S_INT_CLOCK_LENGTH_ERR },
	{ .ev_id = 0 },
	{ .ev_id = 0 },
	{ .ev_id = 0 },						// index S2-24
	{ .ev_id = 0 },
	{ .ev_id = 0 },
	{ .ev_id = PBP_INT_CSIR_S_BUS_ACT_STATE_INT },
	{ .ev_id = PBP_INT_CSIR_S_PORT_STAT_CHANGE_INT },
	{ .ev_id = PBP_INT_CSIR_S_NEWMODADDR },
	{ .ev_id = PBP_INT_CSIR_S_INT_SEA_DOUT_TRIG_4 },
	{ .ev_id = PBP_INT_CSIR_S_INT_SEA_DOUT_TRIG_3 },	// index S2-31
	{ .ev_id = PBP_LOW_ALARM_ACKNOWLEDGEMENT_RECEIVED },	// index 32; special software-event
};

static int eventActiveFlag[NO_OF_EVENTS];
static int ev_area_id = -EINVAL;

#define MAX_PCI_BAR	6

static struct board_mem {
	int usedIrq;
	int NrOfMemSlots;
	struct mem_info {
		unsigned start;
		unsigned len;
	} MemInfo[MAX_PCI_BAR];
} pci_mem;

// description for the ahb-to-pci mapping of the ms-asic
#define MAX_DEVICE_SLOT_NR 5
static struct device_slot_descr {
    int busy;
    struct file *fileptr;
    unsigned ahbAddr;
    unsigned actSlotLen;
    unsigned actPciAddr;
    void *kernelAddr;
    unsigned fileOffset;
} ms_asic_pci_slot[MAX_DEVICE_SLOT_NR];

#define SHOW_VMA_ACTION 0x10 // extend verbose information for vma handling
static int verbose = 0;

/* defines for vendorID and deviceID */
#define VENDOR_ID          		0x110a	/* pci vendor id of Siemens AG	*/

#define DEVICE_ID_PBUS_PLUS		0x4041	/* DEVICE_ID    */

static int pbus_plus_probe(struct pci_dev *pdev, const struct pci_device_id *ent);
static void pbus_plus_remove( struct pci_dev *pdev );

static struct pci_device_id pbus_plus_tbl[] __devinitdata = {
         { VENDOR_ID, DEVICE_ID_PBUS_PLUS, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
         { 0, }	/* This has to be the last entry */
         };

static struct pci_driver pbus_plus_driver = {
	.name		= PRIVATE_DRIVER_NAME,
	.id_table	= pbus_plus_tbl,
	.probe		= pbus_plus_probe,
	.remove		= pbus_plus_remove,
};

static long pbus_plus_ioctl(struct file *filp,
		  unsigned int cmd, unsigned long arg);
static int pbus_plus_read(struct file *filp, char *pC, size_t a,
	loff_t *f_pos);
static int pbus_plus_write(struct file *filp, const char *src, size_t len,
	loff_t *ppos);
static int pbus_plus_open(struct inode *inode, struct file *filp);
static int pbus_plus_mmap(struct file *Filep, struct vm_area_struct *vma);
static int pbus_plus_flush(struct file *filp, fl_owner_t id);
static int pbus_plus_release(struct inode *inode, struct file *filp);
static unsigned long pbus_plus_get_unmapped_area( struct file *file, unsigned long addr, unsigned long len, unsigned long pgoff, unsigned long flags);

static struct file_operations pbus_plus_fops =
{ 
    .unlocked_ioctl = pbus_plus_ioctl,
    .mmap = pbus_plus_mmap,
    .read = pbus_plus_read,
    .write = pbus_plus_write,
    .open = pbus_plus_open,
    .release = pbus_plus_release,
    .flush = pbus_plus_flush,
    .get_unmapped_area = pbus_plus_get_unmapped_area,
};

static uint8_t usr_irq_prio[MAX_NR_OF_CSIR_INTR];

/* defines for mmap BAR OFFSETS */
/* 
 * pbus-plus: we have a max. window of 1 MB
 * we provide an additional non-useable memory range of 1 MB 
 * which results in 2 MB offset for each window
 */
#define OFFSET_BAR0		0x00000000
#define OFFSET_BAR1		0x00200000
#define OFFSET_BAR2		0x00400000
#define OFFSET_BAR3		0x00600000
#define OFFSET_BAR4		0x00800000
/* not used for pbus-plus
#define OFFSET_BAR5		0x00a00000
*/
#define INDEX_BAR0		0
#define INDEX_BAR1		1
#define INDEX_BAR2		2
#define INDEX_BAR3		3
/* not used for pbus-plus
#define INDEX_BAR4		4
#define INDEX_BAR5		5
*/
#define INDEX_BAR_MAX		(INDEX_BAR2 + 1)
#define INDEX_BAR_PBUSFULL	INDEX_BAR2

static char *pbp_virtual_base_addr[INDEX_BAR_MAX];
static struct vm_area_struct *vma_ref[INDEX_BAR_MAX];
static struct pci_dev *pbus_plus_dev;
struct pbus_plus_dev_priv {
    struct of_device *ofdev;
    struct device_node *node_pbus_plus;
} pbus_plus_priv;


// profibus_plus address ofsets
#define PBUSP_S_OFFSET  			0x40000	
#define CSIR_INTCTRL_INTEOICTRLH_EXT		(PBUSP_S_OFFSET+0x0410)
#define CSIR_INTCTRL_INTEOICTRLL_EXT		(PBUSP_S_OFFSET+0x0414)
#define CSIR_INTS_INTMASKH_EXT_S1_ADDR          (PBUSP_S_OFFSET+0x049C)
#define CSIR_INTS_INTMASKH_EXT_S2_ADDR          (PBUSP_S_OFFSET+0x04A0)
#define CSIR_INTS_INTMASKL_EXT_S1_ADDR          (PBUSP_S_OFFSET+0x04A4)
#define CSIR_INTS_INTMASKL_EXT_S2_ADDR          (PBUSP_S_OFFSET+0x04A8)
#define	CSIR_INTS_INTSTATH_EXT_S1_ADDR          (PBUSP_S_OFFSET+0x04BC)
#define	CSIR_INTS_INTSTATH_EXT_S2_ADDR          (PBUSP_S_OFFSET+0x04C0)
#define	CSIR_INTS_INTSTATL_EXT_S1_ADDR          (PBUSP_S_OFFSET+0x04C4)
#define	CSIR_INTS_INTSTATL_EXT_S2_ADDR          (PBUSP_S_OFFSET+0x04C8)
#define CSIR_INTS_INTCLEAR_EXT_S1_ADDR          (PBUSP_S_OFFSET+0x04D4)
#define CSIR_INTS_INTCLEAR_EXT_S2_ADDR          (PBUSP_S_OFFSET+0x04D8)

#define GPIO2_OFFSET 0x30880
#define GPIO2_OUT (GPIO2_OFFSET+0x0)
#define GPIO2_IN  (GPIO2_OFFSET+0x4)
#define GPIO2_SET (GPIO2_OFFSET+0x8)
#define GPIO2_CLEAR (GPIO2_OFFSET+0xc)
#define GPIO2_DIR (GPIO2_OFFSET+0x10)


// pause time between two interrupts
#define PBP_CSIR_EOI_PAUSETIME	10000	/// 10000 * 10 ns = 100us

// intrIndex is in the range of 0 to 63 (0 to 31 is S1, 32 to 63 is S2)
static inline void enableCSIRExtInterrupt(unsigned intrIndex)
{
	volatile unsigned* pMaskRegister = NULL;
	const uint8_t interruptPriority = usr_irq_prio[intrIndex];

	if (intrIndex >= MAX_NR_OF_S1_INTR) {
		if (interruptPriority == PBP_CSIR_INTERRUPT_PRIORITY_HIGH)
		{
                    pMaskRegister = (uint32_t *)(pbp_virtual_base_addr[INDEX_BAR_PBUSFULL]
                                                 + (uint32_t)CSIR_INTS_INTMASKH_EXT_S2_ADDR);
		}
		else if (interruptPriority == PBP_CSIR_INTERRUPT_PRIORITY_LOW)
		{
			pMaskRegister = (uint32_t *)(pbp_virtual_base_addr[INDEX_BAR_PBUSFULL]
                                                 + (uint32_t)CSIR_INTS_INTMASKL_EXT_S2_ADDR);
		}

		intrIndex -= MAX_NR_OF_S1_INTR;
	}
	else {
		if (interruptPriority == PBP_CSIR_INTERRUPT_PRIORITY_HIGH)
		{
			pMaskRegister = (uint32_t *)(pbp_virtual_base_addr[INDEX_BAR_PBUSFULL]
                                                 + (uint32_t)CSIR_INTS_INTMASKH_EXT_S1_ADDR);
		}
		else if (interruptPriority == PBP_CSIR_INTERRUPT_PRIORITY_LOW)
		{
			pMaskRegister = (uint32_t *)(pbp_virtual_base_addr[INDEX_BAR_PBUSFULL]
                                                 + (uint32_t)CSIR_INTS_INTMASKL_EXT_S1_ADDR);
		}
	}

	// clear interrupt mask bit in order to enable the interrupt
	*pMaskRegister = htob_uint32((btoh_uint32(*pMaskRegister)) & ~(1 << intrIndex));

	return;
}

static inline void disableCSIRExtInterrupt(unsigned intrIndex)
{
	volatile unsigned* pMaskRegister = NULL;
	const uint8_t interruptPriority = usr_irq_prio[intrIndex];

	if (intrIndex >= MAX_NR_OF_S1_INTR) {
		if (interruptPriority == PBP_CSIR_INTERRUPT_PRIORITY_HIGH)
		{
			pMaskRegister = (uint32_t *)(pbp_virtual_base_addr[INDEX_BAR_PBUSFULL]
                                                 + (uint32_t)CSIR_INTS_INTMASKH_EXT_S2_ADDR);
		}
		else if (interruptPriority == PBP_CSIR_INTERRUPT_PRIORITY_LOW)
		{
			pMaskRegister = (uint32_t *)(pbp_virtual_base_addr[INDEX_BAR_PBUSFULL]
                                                 + (uint32_t)CSIR_INTS_INTMASKL_EXT_S2_ADDR);
		}

		intrIndex -= MAX_NR_OF_S1_INTR;
	}
	else {
		if (interruptPriority == PBP_CSIR_INTERRUPT_PRIORITY_HIGH)
		{
			pMaskRegister = (uint32_t *)(pbp_virtual_base_addr[INDEX_BAR_PBUSFULL]
                                                 + (uint32_t)CSIR_INTS_INTMASKH_EXT_S1_ADDR);
		}
		else if (interruptPriority == PBP_CSIR_INTERRUPT_PRIORITY_LOW)
		{
			pMaskRegister = (uint32_t *)(pbp_virtual_base_addr[INDEX_BAR_PBUSFULL]
                                                 + (uint32_t)CSIR_INTS_INTMASKL_EXT_S1_ADDR);
		}
	}

	// set the interrupt mask bit in order to disable the interrupt
	*pMaskRegister = htob_uint32((btoh_uint32(*pMaskRegister)) | (1 << intrIndex));	

	return;
}

// call back functions from the aud driver
// we may be called back when a thread or application is forced to terminate
// we are also called in the case of rt_register_event() (SIGEV_NONE => unregister)
static int ev_disable_callback(void *arg, struct rt_event *actEvt)
{
	int index = (int) arg;

	eventActiveFlag[index] = 0;

	return(0);
}
// we are called in the case of rt_register_event() (no SIGEV_NONE)
static int ev_enable_callback(void *arg, struct rt_event *actEvt)
{
	int index = (int) arg;

	eventActiveFlag[index] = 1;

	return(0);
}

/*
 * we search for a free slot in the ahb-pci bar mapping of the ms-asic device
 * we may pass a specific ahb address or we want that this subroutine provides one for us.
 */
static int findFreeDeviceSlot(unsigned addressLen, unsigned ahbAddress)
{
    int ii;
    unsigned deviceAhbAddressStart = 0x60000000;
    unsigned deviceAhbAddressActual = deviceAhbAddressStart;
    unsigned deviceAhbAddressEnd = 0x80000000;
    int order;

    if (addressLen > 0x04000000)  // not more than 64 MB
      return(-EINVAL);
    if (addressLen < 0x10000)  // not less than 64 KB
      return(-EINVAL);
    order = get_order(addressLen) + 12; // get_order is for pages
    if (addressLen != (0x1 << order)) 
      return(-EINVAL);  // we only accept power of two as length argument

    if (ahbAddress && (ahbAddress & (addressLen - 1))) {
            // uncorrect mask
      return(-EINVAL);
    }
    // make sure we use the correct address range
    if (ahbAddress && (ahbAddress < deviceAhbAddressStart))
      return(-EINVAL);
    if (ahbAddress && (((ahbAddress + addressLen) >= deviceAhbAddressEnd)))
      return(-EINVAL);

    // find a useable ahb address range
    for (ii = 0; ii < MAX_DEVICE_SLOT_NR; ii++) {
        if (ms_asic_pci_slot[ii].busy) {
            if (ms_asic_pci_slot[ii].ahbAddr < deviceAhbAddressStart)
              continue; // not in our range
            // simple approach take address range behind this one
            deviceAhbAddressActual = ms_asic_pci_slot[ii].ahbAddr + ms_asic_pci_slot[ii].actSlotLen;
        }
    }
    // test if address fits
    if (!ahbAddress) {
        while (deviceAhbAddressActual & (addressLen - 1)) {
            // uncorrect mask: find the correct start
            deviceAhbAddressActual += sizeof(unsigned);
        }
            if (deviceAhbAddressActual >= deviceAhbAddressEnd)
              return(-EAGAIN);
            if ((deviceAhbAddressActual + addressLen) >= deviceAhbAddressEnd)
              return(-EAGAIN);
    }
    

    for (ii = 0; ii < MAX_DEVICE_SLOT_NR; ii++) {
        if (!ms_asic_pci_slot[ii].busy) {
            if (ahbAddress) {
                ms_asic_pci_slot[ii].ahbAddr = ahbAddress;
            }
            else {
                ms_asic_pci_slot[ii].ahbAddr = deviceAhbAddressActual;
            }
            ms_asic_pci_slot[ii].busy = 1;
            ms_asic_pci_slot[ii].actSlotLen = addressLen;
            return(ii);
        }
    }
    return(-EAGAIN);
}

/* implemented in arch/powerpc/sysdev/indirect_pci.c */
int setPciHostMapping(unsigned *pciHandle, unsigned kernelMemHandle, unsigned len);
int resetPciHostMapping(void);

#define ADN_IRQ_ENABLE(a)	enableCSIRExtInterrupt(a)
#define ADN_IRQ_DISABLE(a)	disableCSIRExtInterrupt(a)

#define PCI_CNF_COMMAND_OFFSET        0x04
#define PCI_CNF_PCI_ARB_CONF_OFFSET   0x80
#define PCI_CNF_AHB_BAR0_OFFSET       0x90
#define PCI_CNF_AHB_BAMR0_OFFSET      0xa4
#define PCI_CNF_AHB_BATR0_OFFSET      0xb8
#define PCI_CNF_AHB_STAT_REG_OFFSET   0xcc

static long pbus_plus_ioctl(struct file *filp,
		  unsigned int cmd, unsigned long arg)
{
	long ret = 0;
	unsigned long myArgs[2];
	int intrNr;
  	struct rt_ev_desc event; 
	int ii;
        unsigned mask, ahb_address;
        struct pbp_handle_pci_dev_info myPciDevInfo;
        unsigned a, b, c;
        int myDeviceSlotIndex;
	dma_addr_t myKernelMemHandle;
	unsigned myTargetPciHandle;
        unsigned myTargetPciLen;
        void *myPtrForTargetPci;

	if (verbose)
		printkGen(NULL, "%s: soc_pbus_plus_ioctl cmd=%#x arg=%#lx\n",
			PRIVATE_DRIVER_NAME, cmd, arg);

	switch(cmd) {
	case PBP_INT_ENABLE:
		if (copy_from_user(myArgs, (void *) arg, sizeof(unsigned long))) {
			ret = -EINVAL;
			break;
		}
		if (verbose)
			printkGen(NULL, "IRQ ENABLE: %ld\n", myArgs[0]);
 		// test interrupt range
		intrNr = myArgs[0];
		if ((intrNr < ADN_INT_CSIR_S1_OFFSET)
				|| (intrNr >= (ADN_INT_CSIR_S2_OFFSET + MAX_NR_OF_S2_INTR))) {
			printkGen(NULL, "   unvalid intrNr\n");
			ret = -EINVAL;
			break;
		}
		intrNr -= ADN_INT_CSIR_S1_OFFSET;
		ADN_IRQ_ENABLE(intrNr);
        	break;

	case PBP_INT_DISABLE:
 		if (copy_from_user(myArgs, (void *) arg, sizeof(unsigned long))) {
			ret = -EINVAL;
			break;
		}
		if (verbose)
			printkGen(NULL, "IRQ DISABLE: %ld\n", myArgs[0]);
 		// test interrupt range
		intrNr = myArgs[0];
		if ((intrNr < ADN_INT_CSIR_S1_OFFSET)
				|| (intrNr >= (ADN_INT_CSIR_S2_OFFSET + MAX_NR_OF_S2_INTR))) {
			printkGen(NULL, "   unvalid intrNr\n");
			ret = -EINVAL;
			break;
		}
		intrNr -= ADN_INT_CSIR_S1_OFFSET;
		ADN_IRQ_DISABLE(intrNr);
        	break;

	case PBP_CONFIG_PRIO:
		if (copy_from_user(myArgs, (void *) arg, 2 * sizeof(unsigned long))) {
			ret = -EINVAL;
			break;
		}
		if (verbose)
			printkGen(NULL, "CONFIG PRIO: %ld %ld\n", myArgs[0], myArgs[1]);
		// test interrupt range
		intrNr = myArgs[0];
		if ((intrNr < ADN_INT_CSIR_S1_OFFSET)
				|| (intrNr >= (ADN_INT_CSIR_S2_OFFSET + MAX_NR_OF_S2_INTR))) {
			printk("   unvalid intrNr\n");
			ret = -EINVAL;
			break;
		}
		intrNr -= ADN_INT_CSIR_S1_OFFSET;
		if ((myArgs[1] != PBP_CSIR_INTERRUPT_PRIORITY_HIGH)
				&& (myArgs[1] != PBP_CSIR_INTERRUPT_PRIORITY_LOW)) {
			printk("   unvalid prio value\n");
			ret = -EINVAL;
			break;
		}
		usr_irq_prio[intrNr] = myArgs[1];
        	break;

	case AuD_EVENT_CREATE:
		if (verbose)
			printkGen(NULL, "AuD_EVENT_CREATE task=%d\n", current->pid);
		if ((ret = copy_from_user(&event, (struct rt_ev_desc *)arg,
				sizeof(event))))
			return (ret);
		if (ev_area_id < 0) {
			printkGen(NULL, "event area not yet initialized\n");
			return(ev_area_id);
		}
		for (ii = 0; ii < NO_OF_EVENTS; ii++) {
			if (event.event && (ppdrv_ev_arr[ii].ev_id == event.event)) {
				if ((ret = rt_register_event(ev_area_id, &event)) == 0)	{
					if (event.sigevent.sigev_notify == SIGEV_NONE) {
						// we do our work in the callback routine
					}
					else {
						// we do our work in the callback routine
					}
					if (verbose)
						printkGen(NULL, 
				"register event %#x (slot %d flag=%d) for pid %d done (%d)\n",
						event.event, ii, eventActiveFlag[ii], 
						event.sigevent._sigev_un._tid, ret);
					return 0;
				}
				printkGen(NULL, "problem register event %#x (ret=%d)\n",
					event.event, ret);
				return ret;
			}	
		}
		printkGen(NULL, "problem with event %#x\n", event.event);
		return -EINVAL;

          case PBP_HANDLE_PCI_DEV_ADDR:
		/*
		 * get a slot on the ahb-pci bridge of the ms-asic device
		 * and a corresponding kernel memory
		 */
		copy_from_user(&myPciDevInfo, (void *)arg, sizeof(myPciDevInfo));
		myTargetPciLen =  myPciDevInfo.pciLen;

		printk("try to get a pci dev slot for ahb=%#x len=%#x\n",
			myPciDevInfo.ahbAddress, myTargetPciLen);

                myDeviceSlotIndex = findFreeDeviceSlot(myTargetPciLen, myPciDevInfo.ahbAddress);
		if (myDeviceSlotIndex < 0)
			return(myDeviceSlotIndex);
    
		/* we really don't have a pci allocation: it's only a dma allocation */
		myPtrForTargetPci = pci_alloc_consistent(pbus_plus_dev, myTargetPciLen,
			&myKernelMemHandle);
		if (!myPtrForTargetPci) {
			// free slot again
			ms_asic_pci_slot[myDeviceSlotIndex].busy = 0;
			return -ENOMEM;
		}

		// map the PCI bus address to the kernel physical memory address
		// on the pci host controller
		// reset mytargetPciHandle info so that we get the actual pci address
		myTargetPciHandle = 0; 
		if (setPciHostMapping(&myTargetPciHandle, myKernelMemHandle, myTargetPciLen)) {
			pci_free_consistent(pbus_plus_dev, myTargetPciLen,
				myPtrForTargetPci, myKernelMemHandle);
			// free slot again
			ms_asic_pci_slot[myDeviceSlotIndex].busy = 0;
			return -EINVAL;
		}
		printk("%s: alloc pci memory and window %p (mem=%#x pci=%#x)\n", PRIVATE_DRIVER_NAME,
			myPtrForTargetPci, myKernelMemHandle, myTargetPciHandle);

#ifdef DEBUG_DEVICE_PCI_ACCESS
		strcpy(myPtrForTargetPci, "this is manfred's PCI mapping RAM\n");
		strcpy(((char *)myPtrForTargetPci + 0x1000), "xxxxxxxx we are on the second page\n");
#endif
                
		/* set informations on ms-asic ahb to pci register set */
		ahb_address = ms_asic_pci_slot[myDeviceSlotIndex].ahbAddr;
                pbus_plus_dev->bus->ops->write(pbus_plus_dev->bus,
                        pbus_plus_dev->devfn, PCI_CNF_AHB_STAT_REG_OFFSET, sizeof(unsigned), 0);

                mask = 0xffffffff - (myTargetPciLen - 1);
                /* use mapping information 0x2 PCI adress range 0xc000000 enable translation */
                pbus_plus_dev->bus->ops->write(pbus_plus_dev->bus,
                           pbus_plus_dev->devfn,
                           PCI_CNF_AHB_BAMR0_OFFSET + myDeviceSlotIndex * sizeof(unsigned), sizeof(unsigned),
                           0xc0000000 | (mask & 0x3fff0000) | 0x2);
                pbus_plus_dev->bus->ops->write(pbus_plus_dev->bus,
                           pbus_plus_dev->devfn,
                           PCI_CNF_AHB_BAR0_OFFSET + myDeviceSlotIndex * sizeof(unsigned),
                           sizeof(unsigned), ahb_address);
                /* use pci address which we received for our buffer */
                pbus_plus_dev->bus->ops->write(pbus_plus_dev->bus,
                           pbus_plus_dev->devfn,
                           PCI_CNF_AHB_BATR0_OFFSET + myDeviceSlotIndex * sizeof(unsigned),
                           sizeof(unsigned), myTargetPciHandle);

                pbus_plus_dev->bus->ops->read(pbus_plus_dev->bus, pbus_plus_dev->devfn,
                           PCI_CNF_COMMAND_OFFSET, 4, &a);
                a |= 0x0107;
                pbus_plus_dev->bus->ops->write(pbus_plus_dev->bus,
                           pbus_plus_dev->devfn, PCI_CNF_COMMAND_OFFSET, sizeof(unsigned), a);
                /* enable arbiter */
                pbus_plus_dev->bus->ops->write(pbus_plus_dev->bus,
                           pbus_plus_dev->devfn, PCI_CNF_PCI_ARB_CONF_OFFSET, sizeof(unsigned), 0x2);
                pbus_plus_dev->bus->ops->write(pbus_plus_dev->bus,
                        pbus_plus_dev->devfn, PCI_CNF_AHB_STAT_REG_OFFSET, sizeof(unsigned), 0x2);


		myPciDevInfo.pciAddress = myTargetPciHandle;
                myPciDevInfo.ahbAddress = ahb_address;
                myPciDevInfo.deviceIndex = myDeviceSlotIndex;
                ms_asic_pci_slot[myDeviceSlotIndex].actPciAddr = myTargetPciHandle;
                ms_asic_pci_slot[myDeviceSlotIndex].fileptr = filp;
                ms_asic_pci_slot[myDeviceSlotIndex].kernelAddr = myPtrForTargetPci;
                ms_asic_pci_slot[myDeviceSlotIndex].fileOffset = myPciDevInfo.fileOffset;
		copy_to_user((void *)arg, &myPciDevInfo, sizeof(myPciDevInfo));

#ifdef DEBUG_DEVICE_PCI_ACCESS
                /*
                 * write a value to the pci bus which in turn should change the kernel memory
                 * this test doesn't work on powerpc, but it worked on SOC1
                 */
                {  
                    unsigned *myTestAddr = ioremap(myTargetPciHandle, 0x10000);
                    printk("test for address %p (%#x)\n", myTestAddr, myTargetPciHandle);
                    *(myTestAddr + 3) = 0;
                }
#endif

		return(0);

          case PBP_HANDLE_PCI_DEV_ADDR_ACCESS:
            /* for test purposes of the pci host bridge */
                pbus_plus_dev->bus->ops->read(pbus_plus_dev->bus, pbus_plus_dev->devfn, 0xcc, 4, &a);
                pbus_plus_dev->bus->ops->read(pbus_plus_dev->bus, pbus_plus_dev->devfn, 0xd0, 4, &b);
                pbus_plus_dev->bus->ops->read(pbus_plus_dev->bus, pbus_plus_dev->devfn, 0xd4, 4, &c);
                printk("ahb state_b %#x:%#x:%#x\n", a, b, c);
		return(0);

          case PBP_HANDLE_PCI_DEV_ADDR_RELEASE:
		/* release the ms asic slot for pci memory and the corresponding kernel memory */

		copy_from_user(&myPciDevInfo, (void *)arg, sizeof(myPciDevInfo));

                myDeviceSlotIndex = myPciDevInfo.deviceIndex;
                if ((myDeviceSlotIndex < 0) || (myDeviceSlotIndex >= MAX_DEVICE_SLOT_NR)) {
                  return(-EINVAL);
                }
                if (!ms_asic_pci_slot[myDeviceSlotIndex].busy) {
                  return(-EINVAL);
                }
                if (ms_asic_pci_slot[myDeviceSlotIndex].actPciAddr != myPciDevInfo.pciAddress) {
                  return(-EINVAL);
                }
                if (ms_asic_pci_slot[myDeviceSlotIndex].fileptr != filp) {
                  return(-EINVAL);
                }

		pci_free_consistent(pbus_plus_dev, myPciDevInfo.pciLen,
			ms_asic_pci_slot[myDeviceSlotIndex].kernelAddr, myPciDevInfo.pciAddress);
		resetPciHostMapping();

                ms_asic_pci_slot[myDeviceSlotIndex].busy = 0;

		return(0);

	default:
		printkGen(NULL, "%s: ioctl unknown cmd=%#x\n",
			PRIVATE_DRIVER_NAME, cmd);
		return(-ENOSYS);
	}
	return(ret);
}

static int pbus_plus_read(struct file *filp, char *pC, size_t len,
	loff_t *f_pos)
{
	printkGen(NULL, "%s: soc_pbus_plus_read buf=%p len=%#x\n", PRIVATE_DRIVER_NAME,
		pC, len);

	return(len);
}

static int pbus_plus_write(struct file *filp, const char *src, size_t len,
	loff_t *ppos)
{
	printkGen(NULL, "%s: soc_pbus_plus_write buf=%p len=%#x\n", PRIVATE_DRIVER_NAME,
		src, len);

	return(len );
}



/*
 * vma operations for mmap on kernel memory.
 *
 * remark: we have a correlation between the ioctl PBP_HANDLE_PCI_DEV_ADDR and the mmap to the offset
 * OFFSET_BAR3. the mmap offset is also part of the ioctl parameter and stored in the device mapping.
 * The vma calls (see next) use this entry to find the address of the allocated kernel memory.
 * This will also work, when we have several device areas with separate kernel memory areas.
 */

static void pbus_plus_mmap_kernel_vma_open(struct vm_area_struct *vma)
{
	if (verbose)
        	printk("%s: mmap_kernel_vma_open virt %lx, phys %lx\n", PRIVATE_DRIVER_NAME,
                       vma->vm_start, vma->vm_pgoff << PAGE_SHIFT);
}

static void pbus_plus_mmap_kernel_vma_close(struct vm_area_struct *vma)
{
	unsigned long vsize;
 	unsigned kernelAddr;
	int deviceSlot;
	unsigned fileOffset = vma->vm_pgoff << PAGE_SHIFT;
	unsigned long offsetInVma;
	unsigned long offsetInMemory;

	vsize = vma->vm_end - vma->vm_start;

	// find the corresponding device slot
	for (deviceSlot = 0; deviceSlot < MAX_DEVICE_SLOT_NR; deviceSlot++) {
		if (ms_asic_pci_slot[deviceSlot].busy) {
			if ((fileOffset >= ms_asic_pci_slot[deviceSlot].fileOffset) &&
					((fileOffset + vsize) <= (ms_asic_pci_slot[deviceSlot].fileOffset +
					ms_asic_pci_slot[deviceSlot].actSlotLen)))
				break;
		}
	}
	if (deviceSlot >= MAX_DEVICE_SLOT_NR) {
		// no device slot found for this file offset
		printk("%s: problem mmap_kernel_vma_close vma_start %lx offset=%#x\n",
			PRIVATE_DRIVER_NAME,
			vma->vm_start, fileOffset);
		return;
	}

	offsetInMemory = fileOffset - ms_asic_pci_slot[deviceSlot].fileOffset;
	kernelAddr = (unsigned) (ms_asic_pci_slot[deviceSlot].kernelAddr + offsetInMemory);

	if (verbose)
		printk("%s: mmap_kernel_vma_close virt %lx off=%lx kernelAddr=%#x\n",
			PRIVATE_DRIVER_NAME,
			vma->vm_start, vma->vm_pgoff << PAGE_SHIFT, kernelAddr);

	/* decrement the count */
	/* the loop is not quite correct, since we may not increment all pages */
	/* should we do the get_page function in pbus_plus_mmap_kernel_vma_open? */
	for (offsetInVma = 0; offsetInVma < vsize; offsetInVma += 0x1000) {
		put_page(virt_to_page(kernelAddr + offsetInVma));
	}
}

int pbus_plus_mmap_kernel_vma_fault(struct vm_area_struct *vma,
                                    struct vm_fault *vmf)
{
	struct page *pageptr;
	char *kernelptr;
	unsigned long vsize;
	int deviceSlot;
	unsigned fileOffset = vma->vm_pgoff << PAGE_SHIFT;
	unsigned long offsetInVma = (unsigned long) vmf->virtual_address - vma->vm_start;
	unsigned long offsetInMemory;

	vsize = vma->vm_end - vma->vm_start;

	if (offsetInVma > vsize) {
		printk("%s: problem mmap_kernel_vma_fault 1 vma_start %lx addr=%p(offset=%#lx)\n",
			PRIVATE_DRIVER_NAME,
			vma->vm_start, vmf->virtual_address, offsetInVma);
		return(VM_FAULT_NOPAGE);
	}
        
	// find the corresponding device slot
	for (deviceSlot = 0; deviceSlot < MAX_DEVICE_SLOT_NR; deviceSlot++) {
		if (ms_asic_pci_slot[deviceSlot].busy) {
			if ((fileOffset >= ms_asic_pci_slot[deviceSlot].fileOffset) &&
					((fileOffset + vsize) <= (ms_asic_pci_slot[deviceSlot].fileOffset +
					ms_asic_pci_slot[deviceSlot].actSlotLen)))
				break;
		}
	}
	if (deviceSlot >= MAX_DEVICE_SLOT_NR) {
		// no device slot found for this file offset
		printk("%s: problem mmap_kernel_vma_fault 2 vma_start %lx addr=%p(offset=%#lx)\n",
			PRIVATE_DRIVER_NAME,
			vma->vm_start, vmf->virtual_address, offsetInVma);
		return(VM_FAULT_NOPAGE);
	}

	offsetInMemory = fileOffset - ms_asic_pci_slot[deviceSlot].fileOffset;
	kernelptr = ((char *)ms_asic_pci_slot[deviceSlot].kernelAddr + offsetInVma + offsetInMemory);
	pageptr = virt_to_page(kernelptr);

	/* increment the count */
	get_page(pageptr);
	vmf->page = pageptr;

	if (verbose & SHOW_VMA_ACTION)
		printk("%s: mmap_kernel_vma_fault kernptr=%p(%p) vma_start=%lx off=%lx addr=%p(%#lx:%#lx)\n",
			PRIVATE_DRIVER_NAME, kernelptr, pageptr, 
				vma->vm_start, vma->vm_pgoff << PAGE_SHIFT, vmf->virtual_address,
				offsetInVma, offsetInMemory);

	return(VM_FAULT_LOCKED);
}

int pbus_plus_mmap_kernel_vma_page_mkwrite(struct vm_area_struct *vma,
                                           struct vm_fault *vmf)
{
	unsigned long vsize;
	unsigned fileOffset = vma->vm_pgoff << PAGE_SHIFT;

	vsize = vma->vm_end - vma->vm_start;

	if (verbose & SHOW_VMA_ACTION)
		printkGen(NULL, "%s: mmap_kernel_vma_mkwrite mem=%#x len=%#x offs=%#x flt flags=%#x off=%#x addr=%p\n",
			PRIVATE_DRIVER_NAME,
			vma->vm_start, vsize, fileOffset,
			vmf->flags, vmf->pgoff, vmf->virtual_address);

	return(VM_FAULT_LOCKED);
}

int pbus_plus_mmap_kernel_vma_access(struct vm_area_struct *vma, unsigned long addr,
                                     void *buf, int len, int write)
{
	printkGen(NULL, "%s: _mmap_kernel_vma_access\n", PRIVATE_DRIVER_NAME);
	return(len);
}

static struct vm_operations_struct soc_pbus_plus_mmap_kernel_vm_ops = {
        .open =   pbus_plus_mmap_kernel_vma_open,
        .close =  pbus_plus_mmap_kernel_vma_close,
        .fault = pbus_plus_mmap_kernel_vma_fault,
        .page_mkwrite = pbus_plus_mmap_kernel_vma_page_mkwrite,
        .access = pbus_plus_mmap_kernel_vma_access,
};


static int pbus_plus_mmap_kernel_ram(struct vm_area_struct *vma)
{
	unsigned long vsize;
	int deviceSlot;
	unsigned fileOffset = vma->vm_pgoff << PAGE_SHIFT;
    
	vsize = vma->vm_end - vma->vm_start;
	if (verbose)
		printk("%s: mmap_kernel_ram size=%#lx offset=%#x\n", PRIVATE_DRIVER_NAME,
			vsize, fileOffset);

	// find the corresponding device slot
	for (deviceSlot = 0; deviceSlot < MAX_DEVICE_SLOT_NR; deviceSlot++) {
		if (ms_asic_pci_slot[deviceSlot].busy) {
			if ((fileOffset >= ms_asic_pci_slot[deviceSlot].fileOffset) &&
					((fileOffset + vsize) <= (ms_asic_pci_slot[deviceSlot].fileOffset +
					ms_asic_pci_slot[deviceSlot].actSlotLen)))
				break;
		}
	}
	if (deviceSlot >= MAX_DEVICE_SLOT_NR) {
		// no device slot found for this file offset
		printk("%s: problem mmap_kernel_ram vma_start %lx offset=%#x\n",
			PRIVATE_DRIVER_NAME, vma->vm_start, fileOffset);
		return(-EINVAL);
	}

        // TBD: do we want some memory parts cached?
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

        vma->vm_flags |= VM_RESERVED;
        vma->vm_ops = &soc_pbus_plus_mmap_kernel_vm_ops;
        pbus_plus_mmap_kernel_vma_open(vma);
        
        return(0);
}

int pbus_plus_mmap_io_vma_access(struct vm_area_struct *vma, unsigned long addr,
                                     void *buf, int len, int write)
{
	unsigned long off, vsize;
	int ii;
	char *ioAddr = NULL;
        
	off = (vma->vm_pgoff << PAGE_SHIFT); /* after mmap now the physical address ? */
	vsize = vma->vm_end - vma->vm_start;
        
	if (verbose) {
		printkGen(NULL, "%s: _mmap_io_vma_access addr=%#x buf=%p len=%d write=%d\n",
			PRIVATE_DRIVER_NAME, addr, buf, len, write);
 		printkGen(NULL, "    vma_start=%#lx vma_off=%#lx vma_size=%#lx\n",
			vma->vm_start, off, vsize);
	}
	if (len != 4)
		 return -EFAULT;

        /*
         * simple approach: we have a parallel vma array to the kernel virtual address references
         * there should be better ways (use an array of physical addresses or is there any
         * information inside vma?
         */
	for (ii = 0; ii < INDEX_BAR_MAX; ii++) {
		/* find vma */
		if (vma_ref[ii] != vma)
			continue;
		/* test range */
		if ((addr < vma->vm_start) || ((addr + len) > vma->vm_end))
			continue;
		ioAddr = pbp_virtual_base_addr[ii] + (addr - vma->vm_start);
	}

	if (!ioAddr) {
		return -EFAULT;
	}

	if (write) {
		*((unsigned *) ioAddr) = *((unsigned *) buf);
	}
	else {
		*((unsigned *) buf) = *((unsigned *) ioAddr);
	}
	if (verbose) {
		printk("info=%#x:%#x ioAddr=%p base=%p\n",
			*((unsigned *) buf), *((unsigned *) ioAddr), ioAddr, pbp_virtual_base_addr[ii]);
	}

	return(len); 
}

static struct vm_operations_struct pbus_plus_mmap_io_vm_ops = {
        .access = pbus_plus_mmap_io_vma_access,
};
static int pbus_plus_mmap(struct file *Filep, struct vm_area_struct *vma)
{
	unsigned long off, vsize, bar_size;
	unsigned PhysAddr; 	// IO-mem-Adresse
	int bar;
	int ret;
	char *bereich_name;

	off = (vma->vm_pgoff << PAGE_SHIFT);
	vsize = vma->vm_end - vma->vm_start;

	if (verbose)
		printk("%s: mmap offset=%#lx len=%#lx start=%#lx\n", PRIVATE_DRIVER_NAME,
                       off, vsize, vma->vm_start);

	if (off >= OFFSET_BAR4) {
        	return -EINVAL;
	}
	else if (off >= OFFSET_BAR3) {
        	/* memory which is used by PCI device to get info from DRAM */
        	return(pbus_plus_mmap_kernel_ram(vma));
	}
	else if (off >= OFFSET_BAR2) {
        	off -= OFFSET_BAR2;
		bar = INDEX_BAR2;
		bar_size = pci_mem.MemInfo[bar].len;	/* Laenge Bereich am Board */
		PhysAddr = pci_mem.MemInfo[bar].start;
		bereich_name = "PBUSFULL";
	}
	else if (off >= OFFSET_BAR1) {
        	off -= OFFSET_BAR1;
		bar = INDEX_BAR1;
		bar_size = pci_mem.MemInfo[bar].len;	/* Laenge Bereich am Board */
		PhysAddr = pci_mem.MemInfo[bar].start;
		bereich_name = "PBUSP   ";
	}
	else if (off >= OFFSET_BAR0) {
        	off -= OFFSET_BAR0;
		bar = INDEX_BAR0;
		bar_size = pci_mem.MemInfo[bar].len;	/* Laenge Bereich am Board */
		PhysAddr = pci_mem.MemInfo[bar].start;
		bereich_name = "XRAM    ";
	}
	else {
        	return -EINVAL;
	}

	if ((vsize + off) > bar_size) {
        	printk("[%s]: mmap size problem bar=%d vsize=%#lx off=%#lx len=%#lx\n",
			PRIVATE_DRIVER_NAME, bar, vsize, off, bar_size);

       		return -EINVAL;
	}
	if (verbose)
        	printk("[%s]: mmap bar=%d phys=%#x len=%#lx\n",
			PRIVATE_DRIVER_NAME, bar, PhysAddr, bar_size);

        // TBD: do we want some memory parts cached?
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	vma->vm_flags |= (VM_IO | VM_RESERVED);
		if ((ret = remap_pfn_range(vma, vma->vm_start,
				(PhysAddr + off) >> PAGE_SHIFT,
				vsize, vma->vm_page_prot))) {
        	printk("[%s]: problem mmap remap ret=%d\n",
			PRIVATE_DRIVER_NAME, ret);
		return -EAGAIN;
	}

        vma->vm_ops = &pbus_plus_mmap_io_vm_ops;
        vma_ref[bar] = vma;
        
	return 0;
}

unsigned long pbus_plus_get_unmapped_area( struct file *file, unsigned long addr,
	unsigned long len, unsigned long pgoff, unsigned long flags)
{
	unsigned long mapped_addr = -ENOMEM;

	mapped_addr = get_unmapped_area(NULL, addr, len, pgoff, flags);

	if (verbose)
		printk("%s: get_unmapped_area addr=%#lx:%lx len=%lx off=%#lx flag=%#lx\n",
			PRIVATE_DRIVER_NAME, addr, mapped_addr, len, pgoff, flags);

	return(mapped_addr);
}

static int pbus_plus_flush(struct file *filp, fl_owner_t id)
{
	if (verbose)
		printk("%s: flush filp=%p\n", PRIVATE_DRIVER_NAME,
			filp);

	return(0);
}

static int pbus_plus_open(struct inode *inode, struct file *filp)
{
	if (verbose)
		printk("%s: open filp=%p\n", PRIVATE_DRIVER_NAME,
			filp);
	rt_allow_access(filp, RT_IO_IOCTL);

	return(0);
}

static int pbus_plus_release(struct inode *inode, struct file *filp)
{
	int ii;

	if (verbose)
		printk("%s: release filp=%p\n", PRIVATE_DRIVER_NAME,
			filp);
        
	rt_remove_access(filp);	 				// remove entry

	for (ii = 0; ii < MAX_DEVICE_SLOT_NR; ii++) {
        	if (ms_asic_pci_slot[ii].busy) {
            		if (ms_asic_pci_slot[ii].fileptr == filp) {
                		pci_free_consistent(pbus_plus_dev, ms_asic_pci_slot[ii].actSlotLen,
                                         ms_asic_pci_slot[ii].kernelAddr,
                                         ms_asic_pci_slot[ii].actPciAddr);
				resetPciHostMapping();
                		ms_asic_pci_slot[ii].busy = 0;
            		}
        	}
    	}
	return(0);
}

static irqreturn_t pbus_plus_isr_low(int irq, void *dev_id);

static irqreturn_t pbus_plus_isr_high(int irq, void *dev_id)
{
	int evtId1 = 0, evtId2 = 0;
	int ev_stat;
	volatile unsigned* pMaskRegister = NULL;
	unsigned statS1, statS2;
#ifdef WORK_WITH_GPIO_EVENTS
	unsigned ierValue;
#endif

        if (0) {
            volatile unsigned *gpioPtr;
            unsigned in, out, dir;

            gpioPtr = (uint32_t *)(pbp_virtual_base_addr[INDEX_BAR_PBUSFULL]
                                   + (uint32_t) GPIO2_OUT);
            out = *gpioPtr;
            gpioPtr = (uint32_t *)(pbp_virtual_base_addr[INDEX_BAR_PBUSFULL]
                                   + (uint32_t) GPIO2_IN);
            in = *gpioPtr;
            gpioPtr = (uint32_t *)(pbp_virtual_base_addr[INDEX_BAR_PBUSFULL]
                                   + (uint32_t) GPIO2_DIR);
            dir = *gpioPtr;
            printkGen(NULL, "isr_high gpio2_%p in=%#x out=%#x dir=%#x\n",
                      gpioPtr, in, out, dir);
        }

#ifdef WORK_WITH_GPIO_EVENTS

	ierValue = __gpio_get_event(gpio_high_bit);
	if (0) printkGen(NULL, "isr_high ier_value=%#x io=%#x\n",
                      ierValue, __gpio_get_value(gpio_high_bit)); 

        if (ierValue) {

#endif
	pMaskRegister = (uint32_t *)(pbp_virtual_base_addr[INDEX_BAR_PBUSFULL]
                                                 + (uint32_t)CSIR_INTS_INTSTATH_EXT_S1_ADDR);
	statS1 = btoh_uint32(*pMaskRegister);
	pMaskRegister = (uint32_t *)(pbp_virtual_base_addr[INDEX_BAR_PBUSFULL]
                                                 + (uint32_t)CSIR_INTS_INTSTATH_EXT_S2_ADDR);
	statS2 =  btoh_uint32(*pMaskRegister);

	if (statS1) {
		evtId1 = __ffs(statS1);	// search for the first bit, starting at top
                if (verbose) {
			printkGen(NULL, "isr_H stat1:%X %X evtId1:%X\n", statS1, statS2, evtId1);
		}
	}
	else if (statS2) {
		evtId2 = __ffs(statS2);	// search for the first bit, starting at top
		// here we use the second part of event descriptions
		evtId1 = evtId2 + MAX_NR_OF_S1_INTR;
		if (verbose) {
			printkGen(NULL, "isr_H stat2:%X %X evtId2:%X evtId1:%X\n",
				statS1, statS2, evtId2, evtId1);
		}
	}
	else {
		// nothing to do
#ifdef WORK_WITH_GPIO_EVENTS
            if (__gpio_get_value(gpio_high_bit)) /* ignore wrong edge */
#endif
		printkGen(NULL, 
			"%s:unvalid isr_high irq=%d dev=%p\n", PRIVATE_DRIVER_NAME, irq, dev_id);
		goto irqHighEnd;
	}

	ADN_IRQ_DISABLE(evtId1);

	if (statS1) {
		// clear interrupt flag
		pMaskRegister = (uint32_t *)(pbp_virtual_base_addr[INDEX_BAR_PBUSFULL]
                                                 + (uint32_t)CSIR_INTS_INTCLEAR_EXT_S1_ADDR);
		*pMaskRegister = htob_uint32(1 << evtId1);
	}
	else if (statS2) {
		// clear interrupt flag
		pMaskRegister = (uint32_t *)(pbp_virtual_base_addr[INDEX_BAR_PBUSFULL]
                                                 + (uint32_t)CSIR_INTS_INTCLEAR_EXT_S2_ADDR);
		*pMaskRegister = htob_uint32(1 << evtId2);
	}

	if (verbose)
		printkGen(NULL,
			"%s: enter isr_high irq=%d dev=%p evt_id=%d/%X (flag=%d)\n", PRIVATE_DRIVER_NAME,
			irq, dev_id, evtId1, ppdrv_ev_arr[evtId1].ev_id, eventActiveFlag[evtId1]);

	if (eventActiveFlag[evtId1]) {
		ev_stat = rt_send_event(&ppdrv_ev_arr[evtId1]);

		if (ppdrv_ev_arr[evtId1].ev_id == PBP_INT_CSIR_S_ALARMCREATETYPEINT) {
			/* special handling: send additional software interrupt */
			if (verbose)
				printkGen(NULL, "SPECIAL rt_send_event PBP_LOW_ALARM_ACKNOWLEDGEMENT_RECEIVED:%X %X\n", 
					PBP_LOW_ALARM_ACKNOWLEDGEMENT_RECEIVED, ppdrv_ev_arr[MAX_NR_OF_CSIR_INTR].ev_id);
			ev_stat = rt_send_event(&ppdrv_ev_arr[MAX_NR_OF_CSIR_INTR]);
			ADN_IRQ_ENABLE(PBP_IRQ_S1_ALARMCREATETYPEINT);
		}
	}

irqHighEnd:
	pMaskRegister = (uint32_t *)(pbp_virtual_base_addr[INDEX_BAR_PBUSFULL]
                                                 + (uint32_t)CSIR_INTCTRL_INTEOICTRLH_EXT);
	*pMaskRegister = PBP_CSIR_EOI_PAUSETIME;

#ifdef WORK_WITH_GPIO_EVENTS
        }

        /* test for low priority interrupts */
	ierValue = __gpio_get_event(gpio_low_bit);
	if (0) printkGen(NULL, "isr_low ier_value=%#x io=%#x\n",
                      ierValue, __gpio_get_value(gpio_low_bit)); 

	if (ierValue) {
		pbus_plus_isr_low(irq, dev_id);
	}

#endif

	return IRQ_HANDLED;
}

static irqreturn_t pbus_plus_isr_low(int irq, void *dev_id)
{
	int evtId1 = 0, evtId2 = 0;
	int ev_stat;
	volatile uint32_t* pMaskRegister;
	unsigned statS1, statS2;

	pMaskRegister = (uint32_t *)(pbp_virtual_base_addr[INDEX_BAR_PBUSFULL]
                                                 + (uint32_t)CSIR_INTS_INTSTATL_EXT_S1_ADDR);
	statS1 = btoh_uint32(*pMaskRegister);
	pMaskRegister = (uint32_t *)(pbp_virtual_base_addr[INDEX_BAR_PBUSFULL]
                                                 + (uint32_t)CSIR_INTS_INTSTATL_EXT_S2_ADDR);
	statS2 =  btoh_uint32(*pMaskRegister);

	if (statS1) {
		evtId1 = __ffs(statS1);	// search for the first bit, starting at top
		if (verbose) {
			printkGen(NULL, "isr_L stat1:%X %X evtId1:%X\n", statS1, statS2, evtId1);
		}
	}
	else if (statS2) {
		evtId2 = __ffs(statS2);	// search for the first bit, starting at top
		// here we use the second part of event descriptions
		evtId1 = evtId2 + MAX_NR_OF_S1_INTR;
		if (verbose) {
			printkGen(NULL, "isr_L stat2:%X %X evtId2:%X evtId1:%X\n",
				statS1, statS2, evtId2, evtId1);
		}
	}
	else {
		// nothing to do
#ifdef WORK_WITH_GPIO_EVENTS
            if (__gpio_get_value(gpio_low_bit)) /* ignore wrong edge */
#endif
		printkGen(NULL, "%s: unvalid soc_pbus_plus_isr_low irq=%d dev=%p\n",PRIVATE_DRIVER_NAME,
			irq, dev_id);
		goto irqLowEnd;
	}

	ADN_IRQ_DISABLE(evtId1);

	if (statS1) {
		// clear interrupt flag
		pMaskRegister = (uint32_t *)(pbp_virtual_base_addr[INDEX_BAR_PBUSFULL]
                                                 + (uint32_t)CSIR_INTS_INTCLEAR_EXT_S1_ADDR);
		*pMaskRegister = htob_uint32(1 << evtId1);
	}
	else if (statS2) {
		// clear interrupt flag
		pMaskRegister = (uint32_t *)(pbp_virtual_base_addr[INDEX_BAR_PBUSFULL]
                                                 + (uint32_t)CSIR_INTS_INTCLEAR_EXT_S2_ADDR);
		*pMaskRegister = htob_uint32(1 << evtId2);
	}

	if (verbose)
		printkGen(NULL, 
			"%s: enter isr_low irq=%d dev=%p evt_id=%d/%X (flag=%d)\n",PRIVATE_DRIVER_NAME,
			irq, dev_id, evtId1, ppdrv_ev_arr[evtId1].ev_id, eventActiveFlag[evtId1]);

	if (eventActiveFlag[evtId1]) {
		ev_stat = rt_send_event(&ppdrv_ev_arr[evtId1]);
	}

irqLowEnd:
	// end of interrupt
	pMaskRegister = (uint32_t *)(pbp_virtual_base_addr[INDEX_BAR_PBUSFULL]
                                                 + (uint32_t)CSIR_INTCTRL_INTEOICTRLL_EXT);
	*pMaskRegister = PBP_CSIR_EOI_PAUSETIME;

	return IRQ_HANDLED;
}

#ifdef WORK_WITH_RT
#define REQUEST_IRQ(irqNr, irqHdlr, flags, name, dev, non_rt_drv) \
	rt_request_irq(irqNr, irqHdlr, flags, name, dev, irqHdlr)
#else
#define REQUEST_IRQ(irqNr, irqHdlr, flags, name, dev, non_rt_drv) \
	request_irq(irqNr, irqHdlr, 0, name, dev)
#endif

void setExtIrq(unsigned irqInfo);
int __gpio_to_irq(unsigned gpio);
int get_immrbase(void);
static int pbus_plus_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	int ret, ii, err;
	int irq1, irq2;
#ifdef WORK_WITH_GPIO_EVENTS
	char *immrPtr;
	unsigned *configPtr;
#endif
	printk("%s: probe dev=%#x vendor=%#x \n", PRIVATE_DRIVER_NAME,
		pdev->device, pdev->vendor);

	switch(pdev->device) {
	case DEVICE_ID_PBUS_PLUS:
		break;

	default:            /* Alle anderen DEVICE_IDs sind ungueltig             */
        	printk("[%s]:   probe pci: ---> device %#x:%#x not useable !\n",
			PRIVATE_DRIVER_NAME, pdev->device, pdev->vendor);
		ret = -EINVAL;
		return(ret);
	}

        if (!pbus_plus_priv.node_pbus_plus) {
		printk("%s: probe pci: missing of node\n", PRIVATE_DRIVER_NAME);
		ret = -EINVAL;
		return(ret);
        }
        
	pci_mem.usedIrq = pdev->irq;

#ifdef WORK_WITH_GPIO_EVENTS

	gpio_high_bit = of_get_gpio_flags(pbus_plus_priv.node_pbus_plus, 0, NULL);
	gpio_low_bit = of_get_gpio_flags(pbus_plus_priv.node_pbus_plus, 1, NULL);
	if ((gpio_high_bit < 0) || (gpio_low_bit < 0)) {
		printk("%s: probe pci: unvalid gpio numbers %d:%d\n", PRIVATE_DRIVER_NAME,
			gpio_high_bit, gpio_low_bit);
		ret = -EINVAL;
		return(ret);
	}
	printk("       gpio_bits high=%d low=%d\n",
		gpio_high_bit, gpio_low_bit);

	immrPtr = ioremap(get_immrbase(), 0x1000);
        /*
         * we use gpio2 bit 16 and 17 for high and low pbus interrupts
         * set SICRH config register for this useage
         */
	configPtr = (unsigned *) (immrPtr + 0x118);
	*configPtr = *configPtr | 0x0600;
        
#else
        
        irq1 = irq_of_parse_and_map(pbus_plus_priv.node_pbus_plus, 0);
        irq2 = irq_of_parse_and_map(pbus_plus_priv.node_pbus_plus, 1);
	printk("       pci_irq=%d of_irq=%d:%d\n",
		pci_mem.usedIrq, irq1, irq2);

	if ((irq1 <= 0) || (irq2 <= 0)) {
		printk("%s: probe pci: unvalid irq numbers %d:%d\n", PRIVATE_DRIVER_NAME, irq1, irq2);
		ret = -EINVAL;
		return(ret);
	}
#endif

	/* get memory addresses of board */
	pci_mem.NrOfMemSlots = 0;
	for (ii = 0; ii < MAX_PCI_BAR; ii++) {
		pci_mem.MemInfo[ii].start = pci_resource_start(pdev, ii);
		pci_mem.MemInfo[ii].len = pci_resource_len(pdev, ii);
		if (pci_mem.MemInfo[ii].len) {
			printk("       memInfo bar %d: addr=%#x len=%#x\n",
				ii, pci_mem.MemInfo[ii].start, pci_mem.MemInfo[ii].len);
		}
	}

	pbp_virtual_base_addr[INDEX_BAR0] = ioremap(pci_mem.MemInfo[INDEX_BAR0].start,
		pci_mem.MemInfo[INDEX_BAR0].len);
	pbp_virtual_base_addr[INDEX_BAR1] = ioremap(pci_mem.MemInfo[INDEX_BAR1].start,
		pci_mem.MemInfo[INDEX_BAR1].len);
	pbp_virtual_base_addr[INDEX_BAR2] = ioremap(pci_mem.MemInfo[INDEX_BAR2].start,
		pci_mem.MemInfo[INDEX_BAR2].len);

	for (ii = 0; ii < NO_OF_EVENTS; ii++) {
		// set callback routines for valid event ids
		if (ppdrv_ev_arr[ii].ev_id != 0) {
			ppdrv_ev_arr[ii].ev_disable = ev_disable_callback;
			ppdrv_ev_arr[ii].ev_enable = ev_enable_callback;
			ppdrv_ev_arr[ii].endisable_par = (void *) ii;
		}
	}

	if ((ev_area_id = rt_init_event_area(ppdrv_ev_arr, NO_OF_EVENTS)) < 0)
	{
		printk("could not initialize event area of %s\n", PRIVATE_DRIVER_NAME);
		return(ev_area_id);
	}

        // preset the ms asic view of PCI slots
        // the first two slots are used internally
        ms_asic_pci_slot[0].busy = 1;
        ms_asic_pci_slot[0].ahbAddr = 0x10000; // dummy value
        ms_asic_pci_slot[0].fileOffset = (unsigned)(-1);
        ms_asic_pci_slot[1].busy = 1;
        ms_asic_pci_slot[1].ahbAddr = 0x20000; // dummy value
        ms_asic_pci_slot[1].fileOffset = (unsigned)(-1);
        // the remaining three slots can be used for PCI accesses
        ms_asic_pci_slot[2].busy = 0;
        ms_asic_pci_slot[2].ahbAddr = 0;
        ms_asic_pci_slot[3].busy = 0;
        ms_asic_pci_slot[3].ahbAddr = 0;
        ms_asic_pci_slot[4].busy = 0;
        ms_asic_pci_slot[4].ahbAddr = 0;

	/* enable device */
        /* this must be done before using the pci addresses */
	if ((err = pci_enable_device(pdev)) < 0) {
		printk("[%s] pci_enable_device failed err=%d\n", PRIVATE_DRIVER_NAME, err);
		return err;
	}

	/* disable low and high priority interrupts on the ms asic controller */
	for (ii = 0; ii < MAX_NR_OF_CSIR_INTR; ii++) {
		usr_irq_prio[ii] = PBP_CSIR_INTERRUPT_PRIORITY_HIGH;
		ADN_IRQ_DISABLE(ii);
		usr_irq_prio[ii] = PBP_CSIR_INTERRUPT_PRIORITY_LOW;
		ADN_IRQ_DISABLE(ii);
        }
	/*
         * setup low and high priority interrupts
	 * PCI interrupts are not yet initialized
	 * and are not used with Pbus-Plus PCI component
	 */
        if (0) {
            volatile unsigned *gpioPtr;
            gpioPtr = (uint32_t *)(pbp_virtual_base_addr[INDEX_BAR_PBUSFULL]
                                   + (uint32_t) GPIO2_SET);
            *gpioPtr = 0x30;
            
            gpioPtr = (uint32_t *)(pbp_virtual_base_addr[INDEX_BAR_PBUSFULL]
                                   + (uint32_t) GPIO2_DIR);
            *gpioPtr = *gpioPtr & 0xffcf;
            __gpio_get_value(209);
            __gpio_get_value(254);

        }

#ifdef WORK_WITH_GPIO_EVENTS
        /* set event mode for the gpios 208 and 209 */
        irq1 = __gpio_to_irq(gpio_high_bit);
        irq2 = __gpio_to_irq(gpio_low_bit);
        printkGen(NULL, "to_irq %d:%d\n", irq1, irq2);
#endif

#if (0)
        if (set_irq_icu_prio(irq1, IRQ_PBUS_PLUS_INTH_ICU_PRIO))
		printk("[%s] warning: can't set icu prio for high interrupt\n",
                       PRIVATE_DRIVER_NAME);
#endif
	if ((err = REQUEST_IRQ(irq1, pbus_plus_isr_high, 0, "pbus_plus_high",
                               pdev, pbus_plus_isr_high)) < 0) {
		printk("[%s] can't request irq %d err=%d\n", PRIVATE_DRIVER_NAME,
			irq1, err);
		return err;
        }
#if (0)
        if (set_irq_icu_prio(irq2, IRQ_PBUS_PLUS_INTL_ICU_PRIO))
		printk("[%s] warning: can't set icu prio for low interrupt\n",
			PRIVATE_DRIVER_NAME);
#endif
#ifndef WORK_WITH_GPIO_EVENTS
	if ((err = REQUEST_IRQ(irq2, pbus_plus_isr_low, 0, "pbus_plus_low",
                               pdev, pbus_plus_isr_low)) < 0) {
		printk("[%s] can't request irq %d err=%d\n", PRIVATE_DRIVER_NAME,
			irq2, err);
		return err;
        }
#endif
        pbus_plus_dev = pdev;

	return(0);
}

static void pbus_plus_remove( struct pci_dev *pdev )
{
	printk("%s: remove %p\n", PRIVATE_DRIVER_NAME, pdev);
}

/* get the dts description of our device */
static int pbus_plus_of_probe(struct of_device *ofdev,
		const struct of_device_id *match)
{
	printk("%s: of_probe\n",  PRIVATE_DRIVER_NAME);

	pbus_plus_priv.ofdev = ofdev;
	pbus_plus_priv.node_pbus_plus = ofdev->node;

	return 0;
}

static int pbus_plus_of_remove(struct of_device *ofdev)
{
	printk("%s: of_remove\n",  PRIVATE_DRIVER_NAME);
	return 0;
}

static struct of_device_id pbus_plus_of_match[] =
{
	{
		.compatible = "pbus-plus",
	},
	{},
};

/* Structure for a of device driver */
static struct of_platform_driver pbus_plus_of_driver = {
	.name = "pbus-plus",
	.match_table = pbus_plus_of_match,
	.probe = pbus_plus_of_probe,
	.remove = pbus_plus_of_remove,
};

static int pbus_plus_init(void)
{
	int result;

	printk("%s: init...\n", PRIVATE_DRIVER_NAME);

	// register device id
	if ((result = register_chrdev(MAJOR_NUMBER, PRIVATE_DRIVER_NAME,
			&pbus_plus_fops))==-1) {
		printk("[%s]: problem pbus_plus_init register_chrdev\n",
			PRIVATE_DRIVER_NAME);
		return -EBUSY;
	} 

	if ((result = of_register_platform_driver(&pbus_plus_of_driver)) < 0) {
            printk("%s: error: init: of_register:%d\n", PRIVATE_DRIVER_NAME, result);
        }

	// register within pci subsystem
	if ((result = pci_register_driver(&pbus_plus_driver)) < 0) {
        	printk("[%s]: error pci_module_init can't register PCI driver\n",
			PRIVATE_DRIVER_NAME);
		unregister_chrdev(MAJOR_NUMBER, PRIVATE_DRIVER_NAME);
        	return(ENODEV);
	}

	return 0;
}


static void pbus_plus_exit(void)
{
	printk("%s: exit\n", PRIVATE_DRIVER_NAME);
	// unregister within pci subsystem
	pci_unregister_driver(&pbus_plus_driver);
	of_unregister_platform_driver(&pbus_plus_of_driver);
	// unregister device id
        unregister_chrdev(MAJOR_NUMBER, PRIVATE_DRIVER_NAME);
}

module_init(pbus_plus_init);
module_exit(pbus_plus_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("PBUS-plus driver");
