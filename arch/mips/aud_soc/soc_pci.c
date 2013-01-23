/*
 * soc specific support for pci
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2012 manfred.neugebauer@siemens.com
 */

#include <linux/types.h>
#include <linux/pci.h>
#include <linux/kernel.h>

#ifdef CONFIG_PCI

// currently we may get requests for 32 slots
// we support two interrupts
// m.n. change array from 'char' to 'unsigned char' else we get negative irq
static unsigned char pci_irq[32] __initdata = {IRQ_PCI_INTA, IRQ_PCI_INTB, 0};

/* Do platform specific device initialization at pci_enable_device() time */
int pcibios_plat_dev_init(struct pci_dev *dev)
{
	printk("pcibios_plat_dev_init\n");
	return 0;
}

int testForHostBridge(struct pci_dev *dev);

int pcibios_map_irq(const struct pci_dev *dev, u8 slot, u8 pin)
{

	testForHostBridge((struct pci_dev *)dev);
    
	printk("pcibios_map_irq dev=%p slot=%#x pin=%d irq=%d\n",
               dev, slot, pin, pci_irq[slot]);
#if (0)
        /* we may exclude some slots: not yet necessary */
	if ((slot < 0) || (slot > 31)) {
		return(0);
	}
#endif
	return pci_irq[slot];
}

/* we setup the host bridge with the first call to pci_alloc_consistent */
static struct pci_dev *HostBridgeDev;

#define PRIVATE_DRIVER_NAME  "soc_host_bridge_driver"
#define SOC_PCI_HOST_BRIDGE_VENDOR 0x110A
#define SOC_PCI_HOST_BRIDGE_DEVICE 0x4026

static int soc_host_bridge_probe(struct pci_dev *pdev, const struct pci_device_id *ent);
static void soc_host_bridge_remove( struct pci_dev *pdev );

static struct pci_device_id soc_host_bridge_tbl[] __devinitdata = {
         { SOC_PCI_HOST_BRIDGE_VENDOR, SOC_PCI_HOST_BRIDGE_DEVICE, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
         { 0, }	/* This has to be the last entry */
         };

static struct pci_driver soc_host_bridge_driver = {
	.name		= PRIVATE_DRIVER_NAME,
	.id_table	= soc_host_bridge_tbl,
	.probe		= soc_host_bridge_probe,
	.remove		= soc_host_bridge_remove,
};

#define MAX_PCI_BAR	6

static struct board_mem {
	int usedIrq;
	struct mem_info {
		unsigned start;
        	unsigned len;
        	unsigned memAddress;
        	void *kernelAddr;
	} MemInfo[MAX_PCI_BAR];
} pci_bridge;

static int soc_host_bridge_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	int ret;
    
	switch(pdev->device) {
          case SOC_PCI_HOST_BRIDGE_DEVICE:
            break;

	default:            
          printk("probe bridge unvalid device dev=%#x vendor=%#x \n",
		pdev->device, pdev->vendor);
		ret = -EINVAL;
		return(ret);
	}
	HostBridgeDev = pdev;
    
	printk("probe bridge dev=%#x vendor=%#x \n",
		pdev->device, pdev->vendor);

        return(0);
}

static void soc_host_bridge_remove( struct pci_dev *pdev )
{
	printk("%s: remove %p\n", PRIVATE_DRIVER_NAME, pdev);
	HostBridgeDev = NULL;
}

// TBD: block for parallel access
static int getHostBridgeBarAndSize(size_t *size, unsigned *pciAddress)
{
	int ii, result;
	size_t mySize = *size;
	int lastSlot = -1;
	int lastLen = 0x7fffffff;

	if (!HostBridgeDev) {
		// register within pci subsystem
            if ((result = pci_register_driver(&soc_host_bridge_driver)) < 0) {
                printk("getHostBridgeBarAndSize no host pci interface\n");
              return(-1);
            }
        }
            if (!HostBridgeDev)
              return(-1);

		/* get memory addresses of bridge */
		for (ii = 0; ii < MAX_PCI_BAR; ii++) {
			pci_bridge.MemInfo[ii].start = pci_resource_start(HostBridgeDev, ii);
			pci_bridge.MemInfo[ii].len = pci_resource_len(HostBridgeDev, ii);
			pci_bridge.MemInfo[ii].memAddress = 0;
			if (pci_bridge.MemInfo[ii].len) {
				printk("       bridgeInfo bar %d: addr=%#x len=%#x\n",
					ii, pci_bridge.MemInfo[ii].start, pci_bridge.MemInfo[ii].len);
			}
		}

        // search for a free slot
        for (ii = 0; ii < MAX_PCI_BAR; ii++) {
        	if ((pci_bridge.MemInfo[ii].len >= mySize) && !pci_bridge.MemInfo[ii].memAddress) {
                        if (lastLen > pci_bridge.MemInfo[ii].len) {
                          lastLen = pci_bridge.MemInfo[ii].len;
                          lastSlot = ii;
                        }
                }
        }
        if (lastSlot >= 0) {
        	*size = pci_bridge.MemInfo[lastSlot].len;
        	*pciAddress = pci_bridge.MemInfo[lastSlot].start;
        }

        return(lastSlot);
}

static int searchSlot(unsigned pciAddress)
{
    int ii;

        for (ii = 0; ii < MAX_PCI_BAR; ii++) {
                if (pciAddress == pci_bridge.MemInfo[ii].start) {
                        if (pci_bridge.MemInfo[ii].memAddress) {
                                return(ii);
                        }
                        break; // not used: no hit
                }
        }
        
        return(-1);
}


#define PCI_CNF_BAR0_OFFSET              0x10
#define PCI_CNF_BAMR0_OFFSET             0x50
#define PCI_CNF_BATR0_OFFSET             0x68

/* m.n. siemens soc1/2 use a different address calculation for the dma devices */
#define GET_DMA_BUS_ADDRESS(a) (((unsigned) a & 0x3fffffff) | 0x40000000)
#define GET_DMA_BUS_ADDRESS_PHYS(a) (((unsigned) a & 0x3fffffff) | 0x40000000)

// update host bridge with the new translation (dma) address
static int updateHostBridge(int slot, unsigned dmaHandle, int enableFlag)
{
    unsigned barMaskInfo;
    unsigned barTranslationInfo;
    unsigned barBaseInfo;

    if (!HostBridgeDev)
      return(-1);
    
    HostBridgeDev->bus->ops->read(HostBridgeDev->bus,
                           HostBridgeDev->devfn, PCI_CNF_BAMR0_OFFSET + slot * sizeof(unsigned), sizeof(unsigned), &barMaskInfo);
    HostBridgeDev->bus->ops->read(HostBridgeDev->bus,
                           HostBridgeDev->devfn, PCI_CNF_BATR0_OFFSET + slot * sizeof(unsigned), sizeof(unsigned), &barTranslationInfo);

        printk("updateHostBridgeTranslation slot=%d bar=%#x trans=%#x\n", slot, barMaskInfo, barTranslationInfo);

    // enable host address mapping
    HostBridgeDev->bus->ops->write(HostBridgeDev->bus,
                                   HostBridgeDev->devfn, PCI_CNF_BATR0_OFFSET + slot * sizeof(unsigned), sizeof(unsigned), dmaHandle);

    if (enableFlag)
      barMaskInfo |= 0xC0000000;
    else
      barMaskInfo &= ~0xC0000000;
      
    HostBridgeDev->bus->ops->write(HostBridgeDev->bus,
                           HostBridgeDev->devfn, PCI_CNF_BAMR0_OFFSET + slot * sizeof(unsigned), sizeof(unsigned), barMaskInfo);
                
    HostBridgeDev->bus->ops->read(HostBridgeDev->bus,
                           HostBridgeDev->devfn,  PCI_CNF_BAR0_OFFSET + slot * sizeof(unsigned), sizeof(unsigned), &barBaseInfo);
    HostBridgeDev->bus->ops->read(HostBridgeDev->bus,
                           HostBridgeDev->devfn, PCI_CNF_BAMR0_OFFSET + slot * sizeof(unsigned), sizeof(unsigned), &barMaskInfo);
    HostBridgeDev->bus->ops->read(HostBridgeDev->bus,
                           HostBridgeDev->devfn, PCI_CNF_BATR0_OFFSET + slot * sizeof(unsigned), sizeof(unsigned), &barTranslationInfo);
    printk("updateHostBridgeTranslation slot=%d base=%#x mask=%#x trans=%#x\n",
           slot, barBaseInfo, barMaskInfo, barTranslationInfo);
    
    return(0);
}

// get a pci slot and connect it to the kernelMemHandle
// return the pci bus address in pciHandle

int setPciHostMapping(unsigned *pciHandle, unsigned kernelMemHandle,
	unsigned size)
{
        size_t realSize = size;
        int actualSlot;
        unsigned pciAddress = 0;
        unsigned dmaHandle = 0;

        actualSlot = getHostBridgeBarAndSize(&realSize, &pciAddress);
        
        printk("setPciHostMapping size=%#x:%#x (slot=%d pci=%#x)\n",
               size, realSize, actualSlot, pciAddress);
        if (actualSlot < 0)
          return(-ENOMEM);

        //pci_bridge.MemInfo[actualSlot].kernelAddr = virtualAddress;
        *pciHandle = pciAddress;
        dmaHandle = GET_DMA_BUS_ADDRESS(kernelMemHandle);
        printk("mem=%#x:%#x\n", dmaHandle, kernelMemHandle);
        //dma_cache_wback_inv((unsigned long) ret, realSize);
        //ret = UNCAC_ADDR(ret);

                //pci_bridge.MemInfo[actualSlot].memAddress = dmaHandle;
                // write address to the host bridge registers
                updateHostBridge(actualSlot, dmaHandle, 1);


	printk("setPciHostMapping pci=%#x:dma=%#x size=%#x:%#x\n",
               *pciHandle, dmaHandle, size, realSize);

	return(0);
}

int resetPciHostMapping(dma_addr_t pci_handle)
{
    int index;
    
    //#ifdef DEBUG_PHYS_DMA
        printk("resetPciHostMapping handle=%#x\n",pci_handle);
    //#endif
    index = searchSlot(pci_handle);
    if (index < 0) {
	printk("problem resetPciHostMapping\n");
	return(-EINVAL);
    }

    updateHostBridge(index, 0, 0); // disable pci address translation 
    pci_bridge.MemInfo[index].memAddress = 0;
    pci_bridge.MemInfo[index].kernelAddr = 0;

	return(0);
}



#endif
