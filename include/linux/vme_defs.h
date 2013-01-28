#ifndef VME_DEFS_H
#define VME_DEFS_H

#include <linux/types.h>

/* VME definitions shared between kernel and userland */

/* Resource Type */
enum vme_resource_type {
	VME_MASTER,
	VME_SLAVE,
	VME_DMA,
	VME_LM
};

enum vme_dwb_dhb {
	VME_DWB,	/* Device wants bus */
	VME_DHB		/* Device has bus */
};

struct vme_window {
	__u8  type;			/* enum vme_resource_type*/
	__u32 aspace;			/* Address Space */
	__u32 cycle;			/* Cycle properties */
	__u32 dwidth;			/* Maximum Data Width */
};

/*
 * VMEbus Master Window Configuration Structure
 */
struct vme_master {
	int enable;			/* State of Window */
	unsigned long long vme_addr;	/* Starting Address on the VMEbus */
	unsigned long long size;	/* Window Size */
	struct vme_window win;
#if 0
	char prefetchEnable;		/* Prefetch Read Enable State */
	int prefetchSize;		/* Prefetch Read Size (Cache Lines) */
	char wrPostEnable;		/* Write Post State */
#endif
};


/* VMEbus Slave Window Configuration Structure */
struct vme_slave {
	int enable;			/* State of Window */
	unsigned long long vme_addr;	/* Starting Address on the VMEbus */
	unsigned long long size;	/* Window Size */
	struct vme_window win;
#if 0
	char wrPostEnable;		/* Write Post State */
	char rmwLock;			/* Lock PCI during RMW Cycles */
	char data64BitCapable;		/* non-VMEbus capable of 64-bit Data */
#endif
};

/* VME Address Spaces */
#define VME_A16		0x1
#define VME_A24		0x2
#define	VME_A32		0x4
#define VME_A64		0x8
#define VME_CRCSR	0x10
#define VME_USER1	0x20
#define VME_USER2	0x40
#define VME_USER3	0x80
#define VME_USER4	0x100

#define VME_ASPACE_MASK	(VME_A16 | VME_A24 | VME_A32 | VME_A64 |	\
			 VME_CRCSR | VME_USER1 | VME_USER2 | 		\
			 VME_USER3 | VME_USER4)
#define VME_A16_MAX	0x10000ULL
#define VME_A24_MAX	0x1000000ULL
#define VME_A32_MAX	0x100000000ULL
#define VME_A64_MAX	0x10000000000000000ULL
#define VME_CRCSR_MAX	0x1000000ULL


/* VME Cycle Types */
#define VME_SCT		0x1
#define VME_BLT		0x2
#define VME_MBLT	0x4
#define VME_2eVME	0x8
#define VME_2eSST	0x10
#define VME_2eSSTB	0x20

#define VME_2eSST160	0x100
#define VME_2eSST267	0x200
#define VME_2eSST320	0x400

#define	VME_SUPER	0x1000
#define	VME_USER	0x2000
#define	VME_PROG	0x4000
#define	VME_DATA	0x8000

#define VME_CYCLE_MASK	(VME_SCT | VME_BLT | VME_MBLT | VME_2eVME |	\
			 VME_2eSST | VME_2eSSTB | VME_2eSST160 |	\
			 VME_2eSST267 | VME_2eSST320 | VME_SUPER |	\
			 VME_USER | VME_PROG | VME_DATA)

/* VME Data Widths */
#define VME_D8		0x1
#define VME_D16		0x2
#define VME_D32		0x4
#define VME_D64		0x8

#define VME_DATA_MASK	(VME_D8 | VME_D16 | VME_D32 | VME_D64)

/* Arbitration Scheduling Modes */
#define VME_R_ROBIN_MODE	0x1
#define VME_PRIORITY_MODE	0x2

/* Generic status bits from the VME bridge */
struct vme_status {
	unsigned short sysfail:1;
	unsigned short acfail:1;
	unsigned short scons:1;

	// TODO: Whatever is of interest here
};

#endif
