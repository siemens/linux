/*
 * Support for indirect PCI bridges.
 *
 * Copyright (C) 1998 Gabriel Paubert.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/init.h>

#include <asm/io.h>
#include <asm/prom.h>
#include <asm/pci-bridge.h>
#include <asm/machdep.h>

static int
indirect_read_config(struct pci_bus *bus, unsigned int devfn, int offset,
		     int len, u32 *val)
{
	struct pci_controller *hose = pci_bus_to_host(bus);
	volatile void __iomem *cfg_data;
	u8 cfg_type = 0;
	u32 bus_no, reg;

	if (hose->indirect_type & PPC_INDIRECT_TYPE_NO_PCIE_LINK) {
		if (bus->number != hose->first_busno)
			return PCIBIOS_DEVICE_NOT_FOUND;
		if (devfn != 0)
			return PCIBIOS_DEVICE_NOT_FOUND;
	}

	if (ppc_md.pci_exclude_device)
		if (ppc_md.pci_exclude_device(hose, bus->number, devfn))
			return PCIBIOS_DEVICE_NOT_FOUND;

	if (hose->indirect_type & PPC_INDIRECT_TYPE_SET_CFG_TYPE)
		if (bus->number != hose->first_busno)
			cfg_type = 1;

	bus_no = (bus->number == hose->first_busno) ?
			hose->self_busno : bus->number;

	if (hose->indirect_type & PPC_INDIRECT_TYPE_EXT_REG)
		reg = ((offset & 0xf00) << 16) | (offset & 0xfc);
	else
		reg = offset & 0xfc;

	if (hose->indirect_type & PPC_INDIRECT_TYPE_BIG_ENDIAN)
		out_be32(hose->cfg_addr, (0x80000000 | (bus_no << 16) |
			 (devfn << 8) | reg | cfg_type));
	else
		out_le32(hose->cfg_addr, (0x80000000 | (bus_no << 16) |
			 (devfn << 8) | reg | cfg_type));

	/*
	 * Note: the caller has already checked that offset is
	 * suitably aligned and that len is 1, 2 or 4.
	 */
	cfg_data = hose->cfg_data + (offset & 3);
	switch (len) {
	case 1:
		*val = in_8(cfg_data);
		break;
	case 2:
		*val = in_le16(cfg_data);
		break;
	default:
		*val = in_le32(cfg_data);
		break;
	}
	return PCIBIOS_SUCCESSFUL;
}

static int
indirect_write_config(struct pci_bus *bus, unsigned int devfn, int offset,
		      int len, u32 val)
{
	struct pci_controller *hose = pci_bus_to_host(bus);
	volatile void __iomem *cfg_data;
	u8 cfg_type = 0;
	u32 bus_no, reg;

	if (hose->indirect_type & PPC_INDIRECT_TYPE_NO_PCIE_LINK) {
		if (bus->number != hose->first_busno)
			return PCIBIOS_DEVICE_NOT_FOUND;
		if (devfn != 0)
			return PCIBIOS_DEVICE_NOT_FOUND;
	}

	if (ppc_md.pci_exclude_device)
		if (ppc_md.pci_exclude_device(hose, bus->number, devfn))
			return PCIBIOS_DEVICE_NOT_FOUND;

	if (hose->indirect_type & PPC_INDIRECT_TYPE_SET_CFG_TYPE)
		if (bus->number != hose->first_busno)
			cfg_type = 1;

	bus_no = (bus->number == hose->first_busno) ?
			hose->self_busno : bus->number;

	if (hose->indirect_type & PPC_INDIRECT_TYPE_EXT_REG)
		reg = ((offset & 0xf00) << 16) | (offset & 0xfc);
	else
		reg = offset & 0xfc;

	if (hose->indirect_type & PPC_INDIRECT_TYPE_BIG_ENDIAN)
		out_be32(hose->cfg_addr, (0x80000000 | (bus_no << 16) |
			 (devfn << 8) | reg | cfg_type));
	else
		out_le32(hose->cfg_addr, (0x80000000 | (bus_no << 16) |
			 (devfn << 8) | reg | cfg_type));

	/* surpress setting of PCI_PRIMARY_BUS */
	if (hose->indirect_type & PPC_INDIRECT_TYPE_SURPRESS_PRIMARY_BUS)
		if ((offset == PCI_PRIMARY_BUS) &&
			(bus->number == hose->first_busno))
		val &= 0xffffff00;

	/* Workaround for PCI_28 Errata in 440EPx/GRx */
	if ((hose->indirect_type & PPC_INDIRECT_TYPE_BROKEN_MRM) &&
			offset == PCI_CACHE_LINE_SIZE) {
		val = 0;
	}

	/*
	 * Note: the caller has already checked that offset is
	 * suitably aligned and that len is 1, 2 or 4.
	 */
	cfg_data = hose->cfg_data + (offset & 3);
	switch (len) {
	case 1:
		out_8(cfg_data, val);
		break;
	case 2:
		out_le16(cfg_data, val);
		break;
	default:
		out_le32(cfg_data, val);
		break;
	}
	return PCIBIOS_SUCCESSFUL;
}

static struct pci_ops indirect_pci_ops =
{
	.read = indirect_read_config,
	.write = indirect_write_config,
};

#ifdef CONFIG_PCI_USE_HOST_INBOUND_WINDOW
struct ppcPciHostMapDescr {
	unsigned pitar;
	unsigned dummy1;
	unsigned pibar;
	unsigned piebar;
	unsigned piwar;
	unsigned dummy2;
};
#define PCI_INBOUND_WINDOW_ENABLE		0x80000000
#define PCI_INBOUND_PREFETCH_WINDOW		0x20000000
#define PCI_INBOUND_NO_READ_SNOOPING		0x00040000
#define PCI_INBOUND_READ_SNOOPING		0x00050000
#define PCI_INBOUND_NO_WRITE_SNOOPING		0x00004000
#define PCI_INBOUND_WRITE_SNOOPING		0x00005000

static char *pciBase = NULL;

int setPciHostMapping(unsigned *pciHandle, unsigned kernelMemHandle, unsigned len)
{
	struct ppcPciHostMapDescr *myHostRef = (struct ppcPciHostMapDescr *)
		(pciBase + 0x38);
	unsigned order = get_order(len) + 12;
	int index = CONFIG_PCI_INBOUND_WINDOW_INDEX; // select host bar 2

	if (!pciBase)
		return(-ENOMEM);
	if (len > CONFIG_PCI_INBOUND_MAX_WINDOW_LEN) {
		return(-EINVAL);
	}

	/*
	 * the kernel pci setup may move the pci address around
         * Therefore, we use the information stored in the bar register
         * The first call to this subroutine from below during
         * primary setup is before the kernel pci setup
         * and can force a first pci address
	 */
	if (!*pciHandle) 
		*pciHandle = (myHostRef + index)->pibar << 12;
	(myHostRef + index)->pitar = kernelMemHandle >> 12;
	(myHostRef + index)->pibar = *pciHandle >> 12;
	(myHostRef + index)->piebar = 0;
	(myHostRef + index)->piwar = 0x0 //
		| (order - 1)
		| PCI_INBOUND_READ_SNOOPING // RTT read transaction type no snoop
		| PCI_INBOUND_WRITE_SNOOPING // WTT write transaction type no snoop
//            | PCI_INBOUND_PREFETCH_WINDOW // prefetch
		| PCI_INBOUND_WINDOW_ENABLE; // enable window
        
	printk("setPciHostMapping(%p) pitar=%#x pibar=%#x:%#x piwar=%#x\n",
		(myHostRef + index),
		(myHostRef + index)->pitar, (myHostRef + index)->pibar,
		(myHostRef + index)->piebar, (myHostRef + index)->piwar);

	return(0);
}

int resetPciHostMapping(dma_addr_t pci_handle)
{
	struct ppcPciHostMapDescr *myHostRef = (struct ppcPciHostMapDescr *)
		(pciBase + 0x38);
	int index = CONFIG_PCI_INBOUND_WINDOW_INDEX; // select host bar 2

	(myHostRef + index)->pitar = CONFIG_PCI_INBOUND_DEFAULT_MEM_ADDR >> 12;
	(myHostRef + index)->piwar = 0x0;  // disable window

	return(0);
}

/*
 * search for a pci bus with this configAddr
 * setup a selected inbound window 
 */
static int addPciInboundWindow(unsigned configAddr)
{
	struct device_node *np;
	struct resource rsrc_reg;
        unsigned pciAddress;

	if (pciBase)
		return(0); /* we already found one pci slot */

	/* search for a pci node with the configaddr 'configAddr' */
	/* take the register base address of this node */
	for_each_compatible_node(np, NULL, "fsl,mpc8349-pci") {
		if (of_address_to_resource(np, 1, &rsrc_reg)) {
			break;
		}
		if (rsrc_reg.start != configAddr)
			continue;
		if (of_address_to_resource(np, 0, &rsrc_reg)) {
			break;
		}
		pciBase = ioremap(rsrc_reg.start, 0x1000);
		break; 
	}
        
	if (!pciBase) {
		printk(KERN_WARNING "no config for PciInboundWindow!\n");
		return -ENOMEM;
	}
	/*
	 * preset a selected host inbound window
	 * use a dummy memory address to block accidential pci accesses
	 * will later be set to a useable memory address
	 */
	/*
	 * first setPciHostMapping is before kernel pci setup and
	 * loads the bar with an init value; this value may be
	 * modified by the kernel pci setup.
	 */
	pciAddress = CONFIG_PCI_INBOUND_BASE_ADDR;
	setPciHostMapping(&pciAddress, CONFIG_PCI_INBOUND_DEFAULT_MEM_ADDR,
		CONFIG_PCI_INBOUND_MAX_WINDOW_LEN);

	return(0);
}
#endif

void __init
setup_indirect_pci(struct pci_controller* hose,
		   resource_size_t cfg_addr,
		   resource_size_t cfg_data, u32 flags)
{
	resource_size_t base = cfg_addr & PAGE_MASK;
	void __iomem *mbase;

	mbase = ioremap(base, PAGE_SIZE);
	hose->cfg_addr = mbase + (cfg_addr & ~PAGE_MASK);
	if ((cfg_data & PAGE_MASK) != base)
		mbase = ioremap(cfg_data & PAGE_MASK, PAGE_SIZE);
	hose->cfg_data = mbase + (cfg_data & ~PAGE_MASK);
	hose->ops = &indirect_pci_ops;
	hose->indirect_type = flags;
#ifdef CONFIG_PCI_USE_HOST_INBOUND_WINDOW
	addPciInboundWindow(cfg_addr);
#endif
}
