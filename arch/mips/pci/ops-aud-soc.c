/*
 * file "ops-aud-soc.c"
 * support for Siemens AuD SOC PCI
 * Copyright (C) manfred.neugebauer@siemens.com
 * 22-July-2008
 *
 * derived from ops-gt64120.c
 * Copyright (C) 1999, 2000, 2004  MIPS Technologies, Inc.
 *	All rights reserved.
 *	Authors: Carsten Langgaard <carstenl@mips.com>
 *		 Maciej W. Rozycki <macro@mips.com>
 *
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 */
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/kernel.h>

//#define DEBUG_PCI_BIOS	1

#define SOC1_PCI_HOST_BRIDGE_VENDOR 0x110A
#define SOC1_PCI_HOST_BRIDGE_DEVICE 0x4026

// address range within AHB
// configuration space 0xc0000000	64 MB
// currently no reservation for this address space
#define CONFIG_SPACE_START_ADDR		0xc0000000
#define CONFIG_SPACE_LEN		0x04000000
#define AHB_PCI_BASE_PHYS               0x1d500000
#define AHB_PCI_BASE_PHYS_LEN           0x00100000

// system interface id should be 0, but we have a SOC1 problem
// so we simulate device 0xf8 (0x1f) as system interface
#define PCI_SYS_CTRL_ID		0x1f 

// general memory space for PCI
// we use the same address range on the AHB and on the PCI bus side
// open issue: how do we use the prefetch bit? (currently not set)
//         but we will get it from the PCI config space, who uses it?
#define SOC_PCI_MEM_ADDR_START	0xe0000000
#define SOC_PCI_MEM_ADDR_LEN	0x20000000
// io address range: we use the 64 MB after CONFIG_SPACE ADDRESS RANGE
// remark: ioport_resource address range is below iomem_resource
// currently we don't see problems
#define SOC_PCI_IO_ADDR_START	0xc4000000
#define SOC_PCI_IO_ADDR_LEN	0x04000000

#define ERROR_TRANSFER_STATE1	0xf9000000
#define ERROR_TRANSFER_STATE2	0x003f0000

static volatile char *ahb_pci_base_ptr;
static volatile char *ahb_pci_configspace_base_ptr;
static unsigned readTransferState1(void);
static unsigned readTransferState2(void);
static unsigned resetTransferState(void);


#define PCI_ACCESS_READ  0
#define PCI_ACCESS_WRITE 1

/*
 *  PCI configuration cycle AD bus definition
 */
#define SOC_PCI0_CFGADDR_BUSNUM_SHF	16
#define SOC_PCI0_CFGADDR_FUNCTNUM_SHF	8
#define SOC_PCI0_CFGADDR_REGNUM_SHF	2

// workaround: will be set/reset while accessing pci configuration range
// to prevent segmentation faults; see sock_berr.c
extern int pci_test_enabled;

/*
 * m.n. currently we see pbus-plus PCI device nr 16 as device 0
 * we don't see soc1 (device 15) and mac/mpc (soc1 eval-board) device 14
 */
static int aud_soc_pcibios_config_access(unsigned char access_type,
	struct pci_bus *bus, unsigned int devfn, int where, u32 * data)
{
	unsigned char busnum = bus->number;
	u32 intr;
	unsigned configAddr;

	// we can't do this earlier since ioremap will not yet work
	if (!ahb_pci_base_ptr) {
		ahb_pci_configspace_base_ptr = ioremap(CONFIG_SPACE_START_ADDR,
			CONFIG_SPACE_LEN);
		ahb_pci_base_ptr = ioremap(AHB_PCI_BASE_PHYS,
			AHB_PCI_BASE_PHYS_LEN);
#ifdef DEBUG_PCI_BIOS
		printk("pcibios base_ptr=%p config_access_ptr=%p\n",
			ahb_pci_base_ptr, ahb_pci_configspace_base_ptr);
#endif
		// we did the initialization of this config space window
		// already before
		// so make sure we have the same address
		// (may be better: do the window initialization now
		// see os_init_pci_regs())
		if (ahb_pci_configspace_base_ptr !=
				(char *) CONFIG_SPACE_START_ADDR) {
			printk("pcibios config_space: problem with ioremap\n");
		}
	}

	/* Clear error register bits */
	resetTransferState();
        pci_test_enabled = 1;

	/* Setup address */
	configAddr = 
		 (busnum << SOC_PCI0_CFGADDR_BUSNUM_SHF) |
		 (devfn << SOC_PCI0_CFGADDR_FUNCTNUM_SHF) |
		 ((where / 4) << SOC_PCI0_CFGADDR_REGNUM_SHF);

	if (access_type == PCI_ACCESS_WRITE) {
#ifdef DEBUG_PCI_BIOS
		printk("soc_pcibios_config_write_access bus=%d dev=%#x where=%#x data=%#x\n",
			busnum, devfn, where, *data);
#endif
		if (busnum == 0 && PCI_SLOT(devfn) == PCI_SYS_CTRL_ID) {
			/*
			 * different access for system controller
			 */
			configAddr |= (unsigned) ahb_pci_base_ptr;
			// we don't allow writes up to now
			//return 0;
		} else {
			configAddr |= (unsigned) ahb_pci_configspace_base_ptr;
		}
		*((unsigned *)configAddr) = *data;
	} else {
		if (busnum == 0 && PCI_SLOT(devfn) == PCI_SYS_CTRL_ID) {
			/*
			 * different access for system controller
			 */
                    configAddr |= (unsigned) ahb_pci_base_ptr;
		} else {
			configAddr |= (unsigned) ahb_pci_configspace_base_ptr;
		}
#ifdef DEBUG_PCI_BIOS
		printk("soc_pcibios_config_read_access %#x:%p bus=%d dev=%#x where=%#x\n",
			configAddr, data, busnum, devfn, where);
#endif
		*data = *((unsigned *)configAddr);
	}

        pci_test_enabled = 0;
	/* Check for master or target abort */
	intr = readTransferState1() & ERROR_TRANSFER_STATE1;
	if (intr) {
#ifdef DEBUG_PCI_BIOS
		printk("problem soc_pci_bios_config state1=%#x\n", intr);
#endif
		resetTransferState();
		return -1;
	}
	intr = readTransferState2() & ERROR_TRANSFER_STATE2;
	if (intr) {
#ifdef DEBUG_PCI_BIOS
		printk("problem soc_pci_bios_config state2=%#x\n", intr);
#endif
		resetTransferState();
		return -1;
	}

	return 0;
}


/*
 * We can't address 8 and 16 bit words directly.  Instead we have to
 * read/write a 32bit word and mask/modify the data we actually want.
 */
static int aud_soc_pcibios_read(struct pci_bus *bus, unsigned int devfn,
                                int where, int size, u32 * val)
{
	u32 data = 0;

	if (aud_soc_pcibios_config_access(PCI_ACCESS_READ, bus, devfn, where,
				          &data))
		return PCIBIOS_DEVICE_NOT_FOUND;
#ifdef DEBUG_PCI_BIOS
	printk("       data=%#x\n", data);
#endif
	if (size == 1)
		*val = (data >> ((where & 3) << 3)) & 0xff;
	else if (size == 2)
		*val = (data >> ((where & 3) << 3)) & 0xffff;
	else
		*val = data;

	return PCIBIOS_SUCCESSFUL;
}

static int aud_soc_pcibios_write(struct pci_bus *bus, unsigned int devfn,
			      int where, int size, u32 val)
{
	u32 data = 0;

	if (size == 4)
		data = val;
	else {
		if (aud_soc_pcibios_config_access(PCI_ACCESS_READ, bus, devfn,
		                                  where, &data))
			return PCIBIOS_DEVICE_NOT_FOUND;

		if (size == 1)
			data = (data & ~(0xff << ((where & 3) << 3))) |
				(val << ((where & 3) << 3));
		else if (size == 2)
			data = (data & ~(0xffff << ((where & 3) << 3))) |
				(val << ((where & 3) << 3));
	}

	if (aud_soc_pcibios_config_access(PCI_ACCESS_WRITE, bus, devfn, where,
				       &data))
		return PCIBIOS_DEVICE_NOT_FOUND;
	return PCIBIOS_SUCCESSFUL;
}

struct pci_ops aud_soc_pci_ops = {
	.read = aud_soc_pcibios_read,
	.write = aud_soc_pcibios_write
};

static volatile unsigned myDelay;

int os_delay(unsigned value)
{
	int ii;

	value *= 1024;
	for (ii = 0; ii < value; ii++) {
		myDelay = ii;
	}
	return(0);
}

/****************************/
/*   GPIO                   */
/****************************/
#define GPIO_BASE                   0xbfa00000

#define GPIO_IOCTRL_0       (GPIO_BASE+0x0)
#define GPIO_OUT_0          (GPIO_BASE+0x4)
#define GPIO_OUT_SET_0      (GPIO_BASE+0x8)
#define GPIO_OUT_CLEAR_0    (GPIO_BASE+0xC)
#define GPIO_IN_0           (GPIO_BASE+0x10)
#define GPIO_RES_DIS_0      (GPIO_BASE+0x14)
#define GPIO_PORT_MODE_0_L  (GPIO_BASE+0x18)
#define GPIO_PORT_MODE_0_H  (GPIO_BASE+0x1C)
#define GPIO_IOCTRL_1       (GPIO_BASE+0x20)
#define GPIO_OUT_1          (GPIO_BASE+0x24)
#define GPIO_OUT_SET_1      (GPIO_BASE+0x28)
#define GPIO_OUT_CLEAR_1    (GPIO_BASE+0x2C)
#define GPIO_RES_DIS_1      (GPIO_BASE+0x30)
#define GPIO_IN_1           (GPIO_BASE+0x34)
#define GPIO_PORT_MODE_1_L  (GPIO_BASE+0x38)
#define GPIO_PORT_MODE_1_H  (GPIO_BASE+0x3C)
#define GPIO_IOCTRL_2       (GPIO_BASE+0x40)
#define GPIO_OUT_2          (GPIO_BASE+0x44)
#define GPIO_OUT_SET_2      (GPIO_BASE+0x48)
#define GPIO_OUT_CLEAR_2    (GPIO_BASE+0x4C)
#define GPIO_RES_DIS_2      (GPIO_BASE+0x50)
#define GPIO_IN_2           (GPIO_BASE+0x54)
#define GPIO_PORT_MODE_2_L  (GPIO_BASE+0x58)
#define GPIO_PORT_MODE_2_H  (GPIO_BASE+0x5C)
#define GPIO_IOCTRL_3       (GPIO_BASE+0x60)
#define GPIO_OUT_3          (GPIO_BASE+0x64)
#define GPIO_OUT_SET_3      (GPIO_BASE+0x68)
#define GPIO_OUT_CLEAR_3    (GPIO_BASE+0x6C)
#define GPIO_RES_DIS_3      (GPIO_BASE+0x70)
#define GPIO_IN_3           (GPIO_BASE+0x74)
#define GPIO_PORT_MODE_3_L  (GPIO_BASE+0x78)
#define GPIO_PORT_MODE_3_H  (GPIO_BASE+0x7C)
#define GPIO_IOCTRL_4       (GPIO_BASE+0x80)
#define GPIO_OUT_4          (GPIO_BASE+0x84)
#define GPIO_OUT_SET_4      (GPIO_BASE+0x88)
#define GPIO_OUT_CLEAR_4    (GPIO_BASE+0x8C)
#define GPIO_RES_DIS_4      (GPIO_BASE+0x90)
#define GPIO_IN_4           (GPIO_BASE+0x94)
#define GPIO_PORT_MODE_4_L  (GPIO_BASE+0x98)
#define GPIO_PORT_MODE_4_H  (GPIO_BASE+0x9C)
#define GPIO_IOCTRL_5       (GPIO_BASE+0xA0)
#define GPIO_OUT_5          (GPIO_BASE+0xA4)
#define GPIO_OUT_SET_5      (GPIO_BASE+0xA8)
#define GPIO_OUT_CLEAR_5    (GPIO_BASE+0xAC)
#define GPIO_RES_DIS_5      (GPIO_BASE+0xB0)
#define GPIO_IN_5           (GPIO_BASE+0xB4)
#define GPIO_PORT_MODE_5_L  (GPIO_BASE+0xB8)
#define GPIO_PORT_MODE_5_H  (GPIO_BASE+0xBC)

/****************************/
/*   PCI                    */
/****************************/
#define PCIICU_BASE                 0xbe300000

#define AHB_PCI_BASE                0xbd500000

#define PCI_CNF_V_ID                (AHB_PCI_BASE + 0x00000000)
#define PCI_CNF_COMMAND             (AHB_PCI_BASE + 0x00000004)
#define PCI_CNF_REV_ID              (AHB_PCI_BASE + 0x00000008)
#define PCI_CNF_CACHE_L_S           (AHB_PCI_BASE + 0x0000000C)
#define PCI_CNF_BAR0                (AHB_PCI_BASE + 0x00000010)
#define PCI_CNF_BAR1                (AHB_PCI_BASE + 0x00000014)
#define PCI_CNF_BAR2                (AHB_PCI_BASE + 0x00000018)
#define PCI_CNF_BAR3                (AHB_PCI_BASE + 0x0000001C)
#define PCI_CNF_BAR4                (AHB_PCI_BASE + 0x00000020)
#define PCI_CNF_BAR5                (AHB_PCI_BASE + 0x00000024)
#define PCI_CNF_CARDBUS             (AHB_PCI_BASE + 0x00000028)
#define PCI_CNF_SUB_V_ID            (AHB_PCI_BASE + 0x0000002C)
#define PCI_CNF_EXP_ROM_BA          (AHB_PCI_BASE + 0x00000030)
#define PCI_CNF_CAPABILITY_PTR      (AHB_PCI_BASE + 0x00000034)
#define PCI_CNF_INT_LINE            (AHB_PCI_BASE + 0x0000003C)
#define PCI_CNF_V_ID_CP             (AHB_PCI_BASE + 0x00000040)
#define PCI_CNF_SUB_V_ID_CP         (AHB_PCI_BASE + 0x00000044)
#define PCI_CNF_PM1                 (AHB_PCI_BASE + 0x00000048)
#define PCI_CNF_PM2                 (AHB_PCI_BASE + 0x0000004C)
#define PCI_CNF_BAMR0               (AHB_PCI_BASE + 0x00000050)
#define PCI_CNF_BAMR1               (AHB_PCI_BASE + 0x00000054)
#define PCI_CNF_BAMR2               (AHB_PCI_BASE + 0x00000058)
#define PCI_CNF_BAMR3               (AHB_PCI_BASE + 0x0000005C)
#define PCI_CNF_BAMR4               (AHB_PCI_BASE + 0x00000060)
#define PCI_CNF_BAMR5               (AHB_PCI_BASE + 0x00000064)
#define PCI_CNF_BATR0               (AHB_PCI_BASE + 0x00000068)
#define PCI_CNF_BATR1               (AHB_PCI_BASE + 0x0000006C)
#define PCI_CNF_BATR2               (AHB_PCI_BASE + 0x00000070)
#define PCI_CNF_BATR3               (AHB_PCI_BASE + 0x00000074)
#define PCI_CNF_BATR4               (AHB_PCI_BASE + 0x00000078)
#define PCI_CNF_BATR5               (AHB_PCI_BASE + 0x0000007C)
#define PCI_CNF_PCI_ARB_CONF        (AHB_PCI_BASE + 0x00000080)
#define PCI_CNF_INT_PIN_DWADDR      (AHB_PCI_BASE + 0x00000084)
#define PCI_CNF_PM2_CP_DWADDR       (AHB_PCI_BASE + 0x00000088)
#define PCI_CNF_REV_ID_CP           (AHB_PCI_BASE + 0x0000008C)
#define PCI_CNF_AHB_BAR0            (AHB_PCI_BASE + 0x00000090)
#define PCI_CNF_AHB_BAR1            (AHB_PCI_BASE + 0x00000094)
#define PCI_CNF_AHB_BAR2            (AHB_PCI_BASE + 0x00000098)
#define PCI_CNF_AHB_BAR3            (AHB_PCI_BASE + 0x0000009C)
#define PCI_CNF_AHB_BAR4            (AHB_PCI_BASE + 0x000000A0)
#define PCI_CNF_AHB_BAMR0           (AHB_PCI_BASE + 0x000000A4)
#define PCI_CNF_AHB_BAMR1           (AHB_PCI_BASE + 0x000000A8)
#define PCI_CNF_AHB_BAMR2           (AHB_PCI_BASE + 0x000000AC)
#define PCI_CNF_AHB_BAMR3           (AHB_PCI_BASE + 0x000000B0)
#define PCI_CNF_AHB_BAMR4           (AHB_PCI_BASE + 0x000000B4)
#define PCI_CNF_AHB_BATR2           (AHB_PCI_BASE + 0x000000C0)
#define PCI_CNF_AHB_BATR3           (AHB_PCI_BASE + 0x000000C4)
#define PCI_CNF_AHB_BATR4           (AHB_PCI_BASE + 0x000000C8)
#define PCI_CNF_AHB_STAT_REG        (AHB_PCI_BASE + 0x000000CC)
#define PCI_CNF_WAIT_STATES         (AHB_PCI_BASE + 0x000000D0)
#define PCI_CNF_BRIDGE_INT_STAT     (AHB_PCI_BASE + 0x000000D4)
#define PCI_CNF_AHB_INT_EN          (AHB_PCI_BASE + 0x000000D8)
#define PCI_CNF_PCI_INT_EN          (AHB_PCI_BASE + 0x000000DC)
#define PCI_CNF_SERR_GEN_SW         (AHB_PCI_BASE + 0x000000F8)
#define PCI_CNF_EN_CONF_PCI         (AHB_PCI_BASE + 0x000000FC)

/****************************/
/*   SCRB                   */
/****************************/
#define SCRB_BASE                   0xbfb00000

#define SCRB_PCI_SOFT_RES                (SCRB_BASE+0x400)
#define SCRB_PCI_RES_STATE               (SCRB_BASE+0x404)
#define SCRB_PCI_PM_STATE_REQ            (SCRB_BASE+0x408)
#define SCRB_PCI_PM_STATE_ACK            (SCRB_BASE+0x40C)
#define SCRB_PCI_PME                     (SCRB_BASE+0x410)
#define SCRB_PCI_SOFT_RES_REQ            (SCRB_BASE+0x414)
#define SCRB_PCI_SOFT_RES_ACK            (SCRB_BASE+0x418)
#define SCRB_PCI_IRTE_CTRL               (SCRB_BASE+0x41C)
#define SCRB_PCI_XSERR_INT_CTRL          (SCRB_BASE+0x420)
#define SCRB_PCI_DIS_BUS                 (SCRB_BASE+0x424)
#define SCRB_PCI_CLK_CONFIG              (SCRB_BASE+0x428)
#define SCRB_PCI_XINTA_CTRL_REG          (SCRB_BASE+0x42C)
#define SCRB_PCI_VTP_SOFT                (SCRB_BASE+0x430)
#define SCRB_PCI_VTP_STATUS              (SCRB_BASE+0x434)
#define SCRB_PCI_VTP_MD                  (SCRB_BASE+0x438)
#define SCRB_PCI_VTP_REG                 (SCRB_BASE+0x43C)

#define REG32(a) (*((unsigned *) (a)))

// configuration flags for bus address mapping
#define	SOC_PCI_CNF_IO		0x1
#define SOC_PCI_CNF_EXT		0x2
#define SOC_PCI_CNF_CONF	0x6
#define SOC_PCI_CNF_PREFETCH	0x8

static unsigned readTransferState1(void)
{
	return(REG32(PCI_CNF_COMMAND));
}

static unsigned readTransferState2(void)
{
	return(REG32(PCI_CNF_AHB_STAT_REG));
}

static unsigned resetTransferState(void)
{
	REG32(PCI_CNF_COMMAND) = REG32(PCI_CNF_COMMAND) | ERROR_TRANSFER_STATE1;
	REG32(PCI_CNF_AHB_STAT_REG) = REG32(PCI_CNF_AHB_STAT_REG)
		| ERROR_TRANSFER_STATE2;
	return(0);
}

// address range within AHB
// configuration space 0xc0000000	64 MB

// some register entries are available multiple time
// e.g. one for read the other for read/write
// e.g., vendor / device id
// it is sufficient to write this information once
void os_init_pci_regs (void)
{

#if (defined CONFIG_AUD_SOC1_CP1500 && (CONFIG_AUD_SOC1_CP1500_PROTO_VERSION == 3))

	REG32(GPIO_PORT_MODE_5_H) |= 0x00000100; // GPIO 180 PCI-CLCK
	REG32(GPIO_PORT_MODE_5_L) |= 0x00000100; // GPIO 164 PCI-CLCK
  
	// GPIO 186 SOC/MSASIC-Reset
	REG32(GPIO_IOCTRL_5)     &= 0xFBFFFFFF; 
	os_delay(1000);
	// Reset bei 0 -> lowaktiv
	REG32(GPIO_OUT_5)        &= 0xFBFFFFFF;
	os_delay(1000);

	// Reset bei 0 -> lowaktiv
	REG32(GPIO_OUT_5)        |= 0x04000000;
	os_delay(1000);

	// 0x10 enable, Divider 14 = 0x0e
	REG32(SCRB_PCI_CLK_CONFIG) = 0x1e;

#else

	// 0x10 enable, Divider 14 = 0x0e
	REG32(SCRB_PCI_CLK_CONFIG) = 0x1e;
	// GPIO 186     -> Output Enable PCI MS Reset
	REG32(GPIO_IOCTRL_5)     &= 0xFBFFFFFF; 
	// GPIO 157+154 -> Output Enable MS Reset
	REG32(GPIO_IOCTRL_4)     &= 0xDBFFFFFF;
	os_delay(1000);

	// Reset bei 0 -> lowaktiv
	REG32(GPIO_OUT_5)        &= 0xFBFFFFFF;
	REG32(GPIO_OUT_4)        &= 0xDBFFFFFF;
	os_delay(1000);

	// Reset bei 0 -> lowaktiv
	REG32(GPIO_OUT_5)        |= 0x04000000;
	REG32(GPIO_OUT_4)        |= 0x24000000;
	os_delay(1000);

	// GPIO 186     -> Alternate Function 0 186 - PCI-Reset
	REG32(GPIO_PORT_MODE_5_H) |= 0x00100000;
	os_delay(1000);
#endif

	REG32(SCRB_PCI_SOFT_RES_REQ) = 0x1;
	REG32(SCRB_PCI_SOFT_RES) = 0x1;

	os_delay(1000);

	REG32(SCRB_PCI_SOFT_RES) = 0x0;
	os_delay(1000);
	// immer auf 0, Reset kommt von extern
	REG32(SCRB_PCI_SOFT_RES_REQ) = 0x0;

	os_delay(1000);

#if (defined CONFIG_AUD_SOC1_CP1500 && (CONFIG_AUD_SOC1_CP1500_PROTO_VERSION == 3))

#else

	// remark IDSEL are somehow correlated to device selection
	// when reading from config space
	// GPIO 183 -> Output Enable IDSEL
	REG32(GPIO_IOCTRL_5)     &= 0xFF7FFFFB;
	// GPIO 162 -> Output Enable IDSEL
	// ???

#endif

	// 0xFC   !!! Enable access thru the bridge from AHB side
	//        (set register valid bit)
	REG32(PCI_CNF_EN_CONF_PCI)   = 0x0;
	// 0xCC AHB status reg(31..16) | AHB function reg(15..0) (REGs_valid=0x2)
	REG32(PCI_CNF_AHB_STAT_REG)  = 0x00;
	//WAIT_SYSTEM_TIMER_IN_US(50); // Stable Frequenz UP
	os_delay(50); 

	// 0x40 Device ID | Vendor ID
	REG32(PCI_CNF_V_ID_CP)     = (SOC1_PCI_HOST_BRIDGE_DEVICE << 16)
            | SOC1_PCI_HOST_BRIDGE_VENDOR;  
	//REG32(PCI_CNF_V_ID_CP)     = 0x4026110A;  
        
	// Enable arbiter (Master 0 on high priority)
	REG32(PCI_CNF_PCI_ARB_CONF)   = 0x00000001;

        // set mapping address for bridge from PCI side
        // for a first approach we use three bars with
        // different sizes (64kB, 1 MB, 16 MB)
	REG32(PCI_CNF_BAR3)  = 0;
	REG32(PCI_CNF_BAMR3)  = 0xFF000000;
	REG32(PCI_CNF_BAR4)  = 0;
	REG32(PCI_CNF_BAMR4)  = 0xFFFF0000;
	REG32(PCI_CNF_BAR5)  = 0;
	REG32(PCI_CNF_BAMR5)  = 0xFFF00000;


	// set mapping addresses from the cpu / AHB side
	// 0x90 AHB Base Address Register 0  AHB2PCI-Bridge internal registers
	REG32(PCI_CNF_AHB_BAR0)  = 0x1D500001;
	// 0x94 AHB Base Address Register 1  PCI Configuration space,
	// 16 MByte,
	REG32(PCI_CNF_AHB_BAR1)  = CONFIG_SPACE_START_ADDR;
	// 0x98 AHB Base Address Register 2  PCI1,
	REG32(PCI_CNF_AHB_BAR2)  = SOC_PCI_MEM_ADDR_START;
	// 0x9C AHB Base Address Register 3  PCI2,
	REG32(PCI_CNF_AHB_BAR3)  = SOC_PCI_IO_ADDR_START;
	// 0xA0 AHB Base Address Register 4  PCI3,
	// m.n. not used
	REG32(PCI_CNF_AHB_BAR4)  = 0x0; 

	// 0xA8 AHB Base Address Mask Register 0 64k
	REG32(PCI_CNF_AHB_BAMR0) = 0xbFFF0001;
	// 0xA8 AHB Base Address Mask Register 1 16M
	REG32(PCI_CNF_AHB_BAMR1) = (~(CONFIG_SPACE_LEN - 1)
			| SOC_PCI_CNF_CONF | SOC_PCI_CNF_IO)
		& 0xBFFFFFFF; // disable translation
	// 0xAC AHB Base Address Mask Register 2 
	REG32(PCI_CNF_AHB_BAMR2) = ~(SOC_PCI_MEM_ADDR_LEN - 1)
			| SOC_PCI_CNF_EXT;
	// 0xB0 AHB Base Address Mask Register 3 
	REG32(PCI_CNF_AHB_BAMR3) = ~(SOC_PCI_IO_ADDR_LEN - 1)
			| SOC_PCI_CNF_EXT | SOC_PCI_CNF_IO;
	// 0xB4 AHB Base Address Mask Register 4 
	// m.n. not used
	REG32(PCI_CNF_AHB_BAMR4) = 0x0; 

	// 0xC0 AHB Base Address Translation Register 2
	REG32(PCI_CNF_AHB_BATR2) = SOC_PCI_MEM_ADDR_START;
	// 0xC4 AHB Base Address Translation Register 3
	REG32(PCI_CNF_AHB_BATR3) = SOC_PCI_IO_ADDR_START;
	// 0xC8 AHB Base Address Translation Register 4
	// m.n. not used
	REG32(PCI_CNF_AHB_BATR4) = 0x0;

	// !!! If Bus master enable bit is disabled all accesses from the AHB side
	// to the PCI side that are stored
	// !!! as transactions are removed! (Chap. 10.2.3)
	// 0x04 Status | Command PCI-Clock = 33 MHZ, Mem space en = 1, Bus master en = 1
	REG32(PCI_CNF_COMMAND)     = 0x02100107;
	// 0x0C BIST | Header Type | Latency Timer | Cache line size
	REG32(PCI_CNF_CACHE_L_S)   = 0x0;

	// 0x44 Subsystem Device ID | Subsystem Vendor ID
	REG32(PCI_CNF_SUB_V_ID_CP) = 0x00010001;
	// 0x4C PM_Data | PM_CSR_BSE | PM_Control_Status
	REG32(PCI_CNF_PM2)         = 0x00000100;

	// 0x8C Class Code | Revision ID
	// m.n. we have a host (bridge) controller
	REG32(PCI_CNF_REV_ID_CP)      =
            ((PCI_CLASS_BRIDGE_HOST << 8) << 8) | 0x01;
        //0x06000001;
	// 0x84 Max_Lat | Min_Gnt | Interrupt Pin | Interrupt line
	REG32(PCI_CNF_INT_PIN_DWADDR) = 0x1f000100;
	// 0x88 PM_Capabilities | reserved
	REG32(PCI_CNF_PM2_CP_DWADDR)  = 0x00020000; // m.n. 0x7E0A0000;

	// 0xD0 Waitstates
	REG32(PCI_CNF_WAIT_STATES) = 0x07070707;
	// 0xD8 AHB Interrupt enable register keinen zulassen
	REG32(PCI_CNF_AHB_INT_EN)  = 0x00000000;
	// 0xDC PCI Interrupt enable register
	REG32(PCI_CNF_PCI_INT_EN)  = 0x00000000;

	// !!! Enable access thru the bridge from PCI side and enable access
	// to internal configuration registers from PCI side (see pcimaster.cmds)
	// !!! Enable access thru the bridge from AHB side (set register valid bit)
	// 0xCC AHB status reg(31..16) | AHB function reg(15..0) (REGs_valid=0x2)
	REG32(PCI_CNF_AHB_STAT_REG)  = 0x00000003;
	REG32(PCI_CNF_EN_CONF_PCI)   = 0x1;         // 0xFC

#if (defined CONFIG_AUD_SOC1_CP1500 && (CONFIG_AUD_SOC1_CP1500_PROTO_VERSION == 3))

	REG32(PCI_CNF_AHB_STAT_REG)  = 0x00040003;

#endif
}

// from mips-baords/generic/pci.c

static struct resource aud_soc_mem_resource = {
	.name	= "AuD SOC PCI MEM",
	.flags	= IORESOURCE_MEM,
};

static struct resource aud_soc_io_resource = {
	.name	= "AuD SOC PCI I/O",
	.flags	= IORESOURCE_IO,
};

static struct pci_controller aud_soc_controller = {
	.pci_ops	= &aud_soc_pci_ops,
	.io_resource	= &aud_soc_io_resource,
	.mem_resource	= &aud_soc_mem_resource,
};

// m.n. search for gt64120_pci_ops to see more work

void __init mips_pcibios_init(void)
{
	struct pci_controller *controller;
	unsigned start;
	unsigned end;

	controller = &aud_soc_controller;

#ifdef DEBUG_PCI_BIOS
	printk("enter pcibios_init\n");
#endif
	// map soc register
	os_init_pci_regs();

	start = SOC_PCI_MEM_ADDR_START; 
	end =   start + SOC_PCI_MEM_ADDR_LEN - 1;
	aud_soc_mem_resource.start = start;
	iomem_resource.end = aud_soc_mem_resource.end = end;

	start = SOC_PCI_IO_ADDR_START;
	end = start + SOC_PCI_IO_ADDR_LEN - 1;
	aud_soc_io_resource.start = start;
	ioport_resource.end = aud_soc_io_resource.end = end;

	register_pci_controller (controller);

#ifdef DEBUG_PCI_BIOS
	printk("pcibios_init done\n");
#endif
}

/* include the bars from the bridge in the general pci address distribution */
int testForHostBridge(struct pci_dev *dev)
{
    struct resource *res;
    int ii, retVal;
    
    if (!((dev->vendor == SOC1_PCI_HOST_BRIDGE_VENDOR)
          && (dev->device == SOC1_PCI_HOST_BRIDGE_DEVICE)))
      return(0);

    printk("testForHostBridge found dev=%p vendor=%#x device=%#x\n",
           dev, dev->vendor, dev->device);

    res = dev->resource;
    for (ii = 0; ii < 6; ii++) {
        if (!(res + ii)->end)
                continue;
        retVal = pci_assign_resource(dev, ii);
        printk("    start=%#x end=%#x (%d)\n",
               (res + ii)->start, (res + ii)->end, ii);
    }

    return(1);
}


