/*******************************************************************************
Copyright (C) Marvell International Ltd. and its affiliates
*******************************************************************************
Marvell BSD License Option

If you received this File from Marvell, you may opt to use, redistribute and/or 
modify this File under the following licensing terms. 
Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

*   Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer. 

      *   Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution. 

        *   Neither the name of Marvell nor the names of its contributors may be 
        used to endorse or promote products derived from this software without 
        specific prior written permission. 
    
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR 
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON 
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#include <linux/kernel.h>
#include <linux/reboot.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/pci.h>

#include "mvTypes.h"
#include "mvOs.h"
#include "pssBspApis.h"
#include "miiInf.h"
#include "eth-phy/mvEthPhy.h"
#include "eth/mvEth.h"

#define STUB_FAIL printk("stub function %s returning MV_NOT_SUPPORTED\n", \
                         __FUNCTION__);  return MV_NOT_SUPPORTED

#define STUB_TBD printk("stub function TBD %s returning MV_FAIL\n", __FUNCTION__); \
   return MV_FAIL

#define  SMI_WRITE_ADDRESS_MSB_REGISTER   (0x00)
#define  SMI_WRITE_ADDRESS_LSB_REGISTER   (0x01)
#define  SMI_WRITE_DATA_MSB_REGISTER      (0x02)
#define  SMI_WRITE_DATA_LSB_REGISTER      (0x03)

#define  SMI_READ_ADDRESS_MSB_REGISTER    (0x04)
#define  SMI_READ_ADDRESS_LSB_REGISTER    (0x05)
#define  SMI_READ_DATA_MSB_REGISTER       (0x06)
#define  SMI_READ_DATA_LSB_REGISTER       (0x07)

#define  SMI_STATUS_REGISTER              (0x1f)

#define SMI_STATUS_WRITE_DONE             (0x02)
#define SMI_STATUS_READ_READY             (0x01)

/* #define SMI_WAIT_FOR_STATUS_DONE */
#define SMI_TIMEOUT_COUNTER  1000

static inline u16 mv_swab16(u16 w)
{
  return (w << 8) | ((w >> 8) & 0xff);
}

static inline u32 mv_swab32(u32 w)
{
  return ((w & 0xff000000) >> 24) |
    ((w & 0x00ff0000) >> 8) |
    ((w & 0x0000ff00) << 8)  |
    ((w & 0x000000ff) << 24);
}

/* interrupt routine pointer */
static MV_VOIDFUNCPTR   bspIsrRoutine = NULL;
static MV_U32           bspIsrParameter = 0;
static MV_U32           bspTxModeSetOn  = 0;
bspEthTxMode_ENT        bspEthTxMode    = bspEthTxMode_asynch_E;

typedef struct
{
  MV_U32 data;
  MV_U32 mask;
}PEX_HEADER_DATA;

#define DB_XCAT_INTERNAL_PP_INT_GPP_PIN    49
#define XCAT_INTERNAL_PP_BASE_ADDR 0xF4000000
ulong   XCAT_INTERNAL_PP_BASE_ADDR_extern = XCAT_INTERNAL_PP_BASE_ADDR;

static MV_U16 gPPDevId = 0xFFFF;
static MV_U16 gtPPrevision = 0;
PEX_HEADER_DATA pp_configHdr[16] =
  {
    {0x000011ab, 0x00000000}, /* 0x00 */
    {0x00100006, 0x00000000}, /* 0x04 */
    {0x05800000, 0x00000000}, /* 0x08 */
    {0x00000008, 0x00000000}, /* 0x0C */
    {0xF000000c, 0x00000000}, /* 0x10 */
    {0x00000000, 0x00000000}, /* 0x14 */
    {XCAT_INTERNAL_PP_BASE_ADDR | 0xC, 0x00000000}, /* 0x18 */
    {0x00000000, 0x00000000}, /* 0x1C */
    {0x00000000, 0x00000000}, /* 0x20 */
    {0x00000000, 0x00000000}, /* 0x24 */
    {0x00000000, 0x00000000}, /* 0x28 */
    {0x11ab11ab, 0x00000000}, /* 0x2C */
    {0x00000000, 0x00000000}, /* 0x30 */
    {0x00000040, 0x00000000}, /* 0x34 */
    {0x00000000, 0x00000000}, /* 0x38 */
    {0x00000100, 0x00000000}  /* 0x3C */
  };
#define HEADER_WRITE(data, offset) pp_configHdr[offset/4].data = ((pp_configHdr[offset/4].data & ~pp_configHdr[offset/4].mask) | \
                                                                  (data & pp_configHdr[offset/4].mask))
#define HEADER_READ(offset) pp_configHdr[offset/4].data
#define PRESTERA_DEV_ID_REG_OFFSET 0x4C

int PRESTERA_DEV_ID_REG_OFFSET_extern = PRESTERA_DEV_ID_REG_OFFSET;

DEFINE_SPINLOCK(mii_rx_lock);
int mii_rx_work_in_progress = 0;

#if 0
#define MV_DEBUG 
#endif
#ifdef MV_DEBUG
#define DB(x) x
#else
#define DB(x)
#endif

static struct tasklet_struct rx_tasklet;
static struct tasklet_struct tx_tasklet;

static void BSP_traffic_dispatch_task(unsigned long ignored);
static void BSP_traffic_dispatch_tx(unsigned long ignored);
static irqreturn_t bspIsr(int irq, void *dev_id, struct pt_regs  *regs);
static void     mgiInfTxDoneJob (void);
static void     mgiInfRxReadyJob(void);

extern int      mgiInfTxDone(void);
extern int      mgiInfRxReady(void);
extern void     mvEthInit(void);

#define MV_ETH_RX_READY_ISR_MASK                        \
  (((1<<MV_ETH_RX_Q_NUM)-1)<<ETH_CAUSE_RX_READY_OFFSET)

#define MV_ETH_TX_DONE_ISR_MASK                       \
  (((1<<MV_ETH_TX_Q_NUM)-1)<<ETH_CAUSE_TX_BUF_OFFSET)

#define MGI_RX_READY_MASK           MV_ETH_RX_READY_ISR_MASK
#define MGI_TX_DONE_MASK            MV_ETH_TX_DONE_ISR_MASK 
#define MGI_MISC_MASK               ((1<<(ETH_CAUSE_LINK_STATE_CHANGE_BIT)) | \
                                     (1<<(ETH_CAUSE_PHY_STATUS_CHANGE_BIT)))

#define ENABLE_MGI_TX_DONE_INTERRUPT(x) \
  MV_REG_WRITE(ETH_INTR_MASK_EXT_REG(x),  \
                (MGI_TX_DONE_MASK | MGI_MISC_MASK))

#define DISABLE_MGI_TX_DONE_INTERRUPT(x) \
  MV_REG_WRITE(ETH_INTR_MASK_EXT_REG(x), 0)

#define ENABLE_MGI_RX_ALL_INTERRUPTS   \
  MV_REG_WRITE(ETH_INTR_MASK_REG(EGIGA_CPU_PORT), 0x00003FC);

#define DISABLE_MGI_RX_ALL_INTERRUPTS   \
  MV_REG_WRITE(ETH_INTR_MASK_REG(EGIGA_CPU_PORT), 0);

/*** reset ***/
/*******************************************************************************
 * bspResetInit
 *
 * DESCRIPTION:
 *       This routine calls in init to do system init config for reset.
 *
 * INPUTS:
 *       none.
 *
 * OUTPUTS:
 *       none.
 *
 * RETURNS:
 *       MV_OK      - on success.
 *       MV_FAIL    - otherwise.
 *
 * COMMENTS:
 *       None.
 *
 *******************************************************************************/
MV_STATUS bspResetInit
(
 MV_VOID
 )
{
  return MV_OK;
}


/*******************************************************************************
 * bspReset
 *
 * DESCRIPTION:
 *       This routine calls to reset of CPU.
 *
 * INPUTS:
 *       none.
 *
 * OUTPUTS:
 *       none.
 *
 * RETURNS:
 *       MV_OK      - on success.
 *       MV_FAIL    - otherwise.
 *
 * COMMENTS:
 *       None.
 *
 *******************************************************************************/

MV_STATUS bspReset
(
 MV_VOID
 )
{
  kernel_restart(NULL);
  return  MV_OK;
}

/*** cache ***/
/*******************************************************************************
 * bspCacheFlush
 *
 * DESCRIPTION:
 *       Flush to RAM content of cache
 *
 * INPUTS:
 *       type        - type of cache memory data/intraction
 *       address_PTR - starting address of memory block to flush
 *       size        - size of memory block
 *
 * OUTPUTS:
 *       None.
 *
 * RETURNS:
 *       MV_OK   - on success,
 *       MV_FAIL - othersise.
 *
 * COMMENTS:
 *
 *******************************************************************************/
MV_STATUS bspCacheFlush
(
 IN bspCacheType_ENT         cacheType, 
 IN void                     *address_PTR, 
 IN size_t                   size
 )
{
  switch (cacheType)
  {
  case bspCacheType_InstructionCache_E:
    return MV_BAD_PARAM; /* only data cache supported */

  case bspCacheType_DataCache_E:
    break;

  default:
    return MV_BAD_PARAM;
  }

  //
  // our area doesn't need cache flush/invalidate
  //
  return  MV_OK;
}

/*******************************************************************************
 * bspCacheInvalidate
 *
 * DESCRIPTION:
 *       Invalidate current content of cache
 *
 * INPUTS:
 *       type        - type of cache memory data/intraction
 *       address_PTR - starting address of memory block to flush
 *       size        - size of memory block
 *
 * OUTPUTS:
 *       None.
 *
 * RETURNS:
 *       MV_OK   - on success,
 *       MV_FAIL - othersise.
 *
 * COMMENTS:
 *
 *******************************************************************************/
MV_STATUS bspCacheInvalidate 
(
 IN bspCacheType_ENT         cacheType, 
 IN void                     *address_PTR, 
 IN size_t                   size
 )
{
  switch (cacheType)
  {
  case bspCacheType_InstructionCache_E:
    return MV_BAD_PARAM; /* only data cache supported */
    
  case bspCacheType_DataCache_E:
    break;
    
  default:
    return MV_BAD_PARAM;
  }

  //
  // our area doesn't need cache flush/invalidate
  //
  return MV_OK;
}

/*** DMA ***/
/*******************************************************************************
 * bspDmaWrite
 *
 * DESCRIPTION:
 *       Write a given buffer to the given address using the Dma.
 *
 * INPUTS:
 *       address     - The destination address to write to.
 *       buffer      - The buffer to be written.
 *       length      - Length of buffer in words.
 *       burstLimit  - Number of words to be written on each burst.
 *
 * OUTPUTS:
 *       None.
 *
 * RETURNS:
 *       MV_OK   - on success,
 *       MV_FAIL - othersise.
 *
 * COMMENTS:
 *       1.  The given buffer is allways 4 bytes aligned, any further allignment
 *           requirements should be handled internally by this function.
 *       2.  The given buffer may be allocated from an uncached memory space, and
 *           it's to the function to handle the cache flushing.
 *       3.  The Prestera Driver assumes that the implementation of the DMA is
 *           blocking, otherwise the Driver functionality might be damaged.
 *
 *******************************************************************************/
MV_STATUS bspDmaWrite
(
 IN  MV_U32  address,
 IN  MV_U32  *buffer,
 IN  MV_U32  length,
 IN  MV_U32  burstLimit
 )
{
  STUB_FAIL;
}

/*******************************************************************************
 * bspDmaRead
 *
 * DESCRIPTION:
 *       Read a memory block from a given address.
 *
 * INPUTS:
 *       address     - The address to read from.
 *       length      - Length of the memory block to read (in words).
 *       burstLimit  - Number of words to be read on each burst.
 *
 * OUTPUTS:
 *       buffer  - The read data.
 *
 * RETURNS:
 *       MV_OK   - on success,
 *       MV_FAIL - othersise.
 *
 * COMMENTS:
 *       1.  The given buffer is allways 4 bytes aligned, any further allignment
 *           requirements should be handled internally by this function.
 *       2.  The given buffer may be allocated from an uncached memory space, and
 *           it's to the function to handle the cache flushing.
 *       3.  The Prestera Driver assumes that the implementation of the DMA is
 *           blocking, otherwise the Driver functionality might be damaged.
 *
 *******************************************************************************/
MV_STATUS bspDmaRead
(
 IN  MV_U32  address,
 IN  MV_U32  length,
 IN  MV_U32  burstLimit,
 OUT MV_U32  *buffer
 )
{
  STUB_FAIL;
}

void  *mv_base_p = NULL;
void  *mv_base_v;
void  *mv_top_v;
void  *mv_free_v;
void  *mv_base_descr_v;

/*******************************************************************************
 * bspCacheDmaMalloc
 *
 * DESCRIPTION:
 *       Allocate a cache free area for DMA devices.
 *
 * INPUTS:
 *       size_t bytes - number of bytes to allocate
 *
 * OUTPUTS:
 *       None.
 *
 * RETURNS:
 *       virtual address of area
 *       NULL - per failure to allocate space
 *
 * COMMENTS:
 *       None
 *
 *******************************************************************************/
void *bspCacheDmaMalloc(IN size_t bytes)
{
  void *ptr;
  
  if (!mv_base_p) // first time ?
  {
    mv_base_p = (void *)mv_high_memory_paddr;
    mv_base_v = (void *)mv_high_memory_vaddr; 
    mv_top_v = (void *)((MV_U32)mv_base_v + mv_high_memory_len);
    mv_free_v = mv_base_v;
    mv_base_descr_v = mv_top_v;
  }
  
  ptr = mv_free_v;
  mv_free_v = (void *)((MV_U32)mv_free_v + bytes);
  
  if ((MV_U32)mv_base_descr_v <= (MV_U32)mv_free_v)
    panic(">>> mv area exahasted\n");

  return ptr;
  
}

/*******************************************************************************
 * bspCacheDmaMallocDescriptors
 *
 * DESCRIPTION:
 *       Allocate a cache free area for descriptors
 *
 * INPUTS:
 *       size_t bytes - number of bytes to allocate
 *
 * OUTPUTS:
 *       None.
 *
 * RETURNS:
 *       virtual address of area
 *       NULL - per failure to allocate space
 *
 * COMMENTS:
 *       None
 *
 *******************************************************************************/
void *bspCacheDmaMallocDescriptors
(
    size_t bytes
)
{
  void *ptr;

  // allocate from mv_top_v downwards. 

  if (!mv_base_p) // first time ?
  {
    mv_base_p = (void *)mv_high_memory_paddr;
    mv_base_v = (void *)mv_high_memory_vaddr; 
    mv_top_v = (void *)((MV_U32)mv_base_v + mv_high_memory_len);
    mv_free_v = mv_base_v;
    mv_base_descr_v = mv_top_v;
  }
  
  ptr = (void *)((MV_U32)mv_base_descr_v - bytes);
  mv_base_descr_v = ptr;

  if ((MV_U32)mv_base_descr_v <= (MV_U32)mv_free_v)
    panic("Error: mv area exhausted\n");
  
  return ptr;  
}

/*******************************************************************************
 * bspCacheDmaFree
 *
 * DESCRIPTION:
 *       free a cache free area back to pool.
 *
 * INPUTS:
 *       size_t bytes - number of bytes to allocate
 *
 * OUTPUTS:
 *       None.
 *
 * RETURNS:
 *       MV_OK   - on success
 *       MV_FAIL - on error
 *
 * COMMENTS:
 *       None
 *
 *******************************************************************************/
MV_STATUS bspCacheDmaFree(void * pBuf)
{
  /* a quick solution to free DMA area,
   * usable for cleanup only */
  if ((MV_U32)mv_free_v >= (MV_U32)pBuf)
  {
    /* it seems memory already freed */
    return MV_FAIL;
  }
  mv_free_v = pBuf;
  return MV_OK;
}

/*** PCI ***/
/*******************************************************************************
 * bspPciConfigWriteReg
 *
 * DESCRIPTION:
 *       This routine write register to the PCI configuration space.
 *
 * INPUTS:
 *       busNo    - PCI bus number.
 *       devSel   - the device devSel.
 *       funcNo   - function number.
 *       regAddr  - Register offset in the configuration space.
 *       data     - data to write.
 *
 * OUTPUTS:
 *       None.
 *
 * RETURNS:
 *       MV_OK   - on success,
 *       MV_FAIL - othersise.
 *
 * COMMENTS:
 *
 *******************************************************************************/
MV_STATUS bspPciConfigWriteReg
(
 IN  MV_U32  busNo,
 IN  MV_U32  devSel,
 IN  MV_U32  funcNo,
 IN  MV_U32  regAddr,
 IN  MV_U32  data
 )
{
  struct pci_dev *dev;

  /* Emulate internal PP on PEX */
  if((0xFF == busNo) && (0xFF == devSel))
  {
    HEADER_WRITE(data, regAddr);
    return MV_OK;
  }
  /* Emulate end*/

  dev = pci_get_bus_and_slot(busNo, PCI_DEVFN(devSel,funcNo));
  if (dev)
  {
    pci_write_config_dword(dev, regAddr, data);
    return MV_OK;
  }
  else
    return MV_FAIL;
}

/*******************************************************************************
 * bspPciConfigReadReg
 *
 * DESCRIPTION:
 *       This routine read register from the PCI configuration space.
 *
 * INPUTS:
 *       busNo    - PCI bus number.
 *       devSel   - the device devSel.
 *       funcNo   - function number.
 *       regAddr  - Register offset in the configuration space.
 *
 * OUTPUTS:
 *       data     - the read data.
 *
 * RETURNS:
 *       MV_OK   - on success,
 *       MV_FAIL - othersise.
 *
 * COMMENTS:
 *
 *******************************************************************************/
MV_STATUS bspPciConfigReadReg
(
 IN  MV_U32  busNo,
 IN  MV_U32  devSel,
 IN  MV_U32  funcNo,
 IN  MV_U32  regAddr,
 OUT MV_U32  *data
 )
{
  struct pci_dev *dev;


  /* Emulate internal PP on PEX */
  if((0xFF == busNo) && (0xFF == devSel))
  {
    if ( ((regAddr & 0x00000003) != 0) || regAddr > 0x40)
      return MV_FAIL;

    if(0 == regAddr)
    {
      *data = 0x000011ab | (gPPDevId << 16);
    }
    else if (8 == regAddr)
    {
      *data = HEADER_READ(regAddr) | gtPPrevision;
    }
    else
    {
      *data = HEADER_READ(regAddr);
    }

    return MV_OK;
  }
  /* Emulate end*/

  dev = pci_get_bus_and_slot(busNo, PCI_DEVFN(devSel,funcNo));
  if (dev)
  {
    pci_read_config_dword(dev, regAddr, data);
    return MV_OK;
  }
  else
    return MV_FAIL;
}

/*******************************************************************************
 * bspReadRegisterInternal
 *
 * DESCRIPTION:
 *       This routine read register from given address.
 *
 * INPUTS:
 *       address  - Register address.
 *
 * OUTPUTS:
 *
 * RETURNS:
 *       data     - the read data.
 *
 * COMMENTS:
 *
 *******************************************************************************/
MV_U32 bspReadRegisterInternal
(
    IN MV_U32  address
)
{
  /* Endianess. */
#if defined(MV_CPU_BE)
  /* need to swap the bytes */
  MV_U8   *bytesPtr;
  MV_U32  registerValue = *((MV_U32*)address);

  bytesPtr = (MV_U8*)&registerValue;

  return ((MV_U32)(bytesPtr[3] << 24)) |
         ((MV_U32)(bytesPtr[2] << 16)) |
         ((MV_U32)(bytesPtr[1] << 8 )) |
         ((MV_U32)(bytesPtr[0]      )) ;
#else
  /* direct access - no swap needed */
  return *((MV_U32*)address);
#endif  /* MV_CPU_BE */ 
}

/*******************************************************************************
 * bspPciFindDev
 *
 * DESCRIPTION:
 *       This routine returns the next instance of the given device (defined by
 *       vendorId & devId).
 *
 * INPUTS:
 *       vendorId - The device vendor Id.
 *       devId    - The device Id.
 *       instance - The requested device instance.
 *
 * OUTPUTS:
 *       busNo    - PCI bus number.
 *       devSel   - the device devSel.
 *       funcNo   - function number.
 *
 * RETURNS:
 *       MV_OK   - on success,
 *       MV_FAIL - othersise.
 *
 * COMMENTS:
 *
 *******************************************************************************/
MV_STATUS bspPciFindDev
(
 IN  MV_U16  vendorId,
 IN  MV_U16  devId,
 IN  MV_U32  instance,
 OUT MV_U32  *busNo,
 OUT MV_U32  *devSel,
 OUT MV_U32  *funcNo
 )
{
  struct pci_dev *dev = NULL;

  MV_U32 regValue;

  /* Emulate internal PP on PEX */
  /* If internal PP not found - read device ID from PP internal register space */


  if(0xFFFF == gPPDevId)
  {

    regValue = bspReadRegisterInternal(PRESTERA_DEV_ID_REG_OFFSET + XCAT_INTERNAL_PP_BASE_ADDR);
    gPPDevId = (MV_U16)((regValue >> 4) & 0xFFFF);
    gtPPrevision = (MV_U16)(regValue & 0xF);

    
    if(gPPDevId == devId)
    {
      *busNo  = 0xFF;
      *devSel = 0xFF;
      *funcNo = 0xFF;
      return MV_OK;
    }
    else
    {
      gPPDevId = 0xFFFF;
    }    
  }
  /* Emulate - end */

  *busNo = *devSel = *funcNo = 0;

  while ((dev = pci_get_device(PCI_ANY_ID, PCI_ANY_ID, dev)) != NULL) 
  {
    if ((dev->vendor == vendorId) && 
        (dev->device == devId))
    {
      if (instance > 0)
      {
        instance --;
      }
      else
      {
        *busNo = dev->bus->number;
        *devSel = PCI_SLOT(dev->devfn);
        *funcNo = PCI_FUNC(dev->devfn);
    
        return MV_OK;
      } 
    }
  }

  return MV_FAIL;
}

/*******************************************************************************
 * bspPciFindDevReset
 *
 * DESCRIPTION:
 *       Reset gPPDevId to make chance to find internal PP again
 *
 * INPUTS:
 *       None
 *
 * OUTPUTS:
 *       None
 *
 * RETURNS:
 *       MV_OK   - on success,
 *
 * COMMENTS:
 *
 *******************************************************************************/
MV_STATUS bspPciFindDevReset(void)
{
  gPPDevId = 0xFFFF;
  gtPPrevision = 0;
  return MV_OK;
}

/*******************************************************************************
 * bspPciGetIntVec
 *
 * DESCRIPTION:
 *       This routine return the PCI interrupt vector.
 *
 * INPUTS:
 *       pciInt - PCI interrupt number.
 *
 * OUTPUTS:
 *       intVec - PCI interrupt vector.
 *
 * RETURNS:
 *       MV_OK      - on success.
 *       MV_FAIL    - otherwise.
 *
 * COMMENTS:
 *       None.
 *
 *******************************************************************************/
MV_STATUS bspPciGetIntVec
(
 IN  bspPciInt_PCI_INT  pciInt,
 OUT void               **intVec
 )
{
  /* check parameters */
  if(intVec == NULL)
  {
    return MV_BAD_PARAM;
  }
  /* get the PCI interrupt vector */
  if ((pciInt == bspPciInt_PCI_INT_B) || (pciInt == bspPciInt_PCI_INT_D))
  {
    /* The internal PP interrupt is connected to GPIO 49 of the CPU */
    *intVec = (void *)IRQ_GPP_49;
  }
  else
  {
    /* The external PP interrupt is PEX0INT of the CPU */
    *intVec = (void *)IRQ_PEX0_INT;
  }
  
  return MV_OK;
}

/*******************************************************************************
 * bspPciGetIntMask
 *
 * DESCRIPTION:
 *       This routine return the PCI interrupt vector.
 *
 * INPUTS:
 *       pciInt - PCI interrupt number.
 *
 * OUTPUTS:
 *       intMask - PCI interrupt mask.
 *
 * RETURNS:
 *       MV_OK      - on success.
 *       MV_FAIL    - otherwise.
 *
 * COMMENTS:
 *       PCI interrupt mask should be used for interrupt disable/enable.
 *
 *******************************************************************************/
MV_STATUS bspPciGetIntMask
(
 IN  bspPciInt_PCI_INT  pciInt,
 OUT MV_U32             *intMask
 )
{
  STUB_FAIL;
}

/*******************************************************************************
 * bspPciEnableCombinedAccess
 *
 * DESCRIPTION:
 *       This function enables / disables the Pci writes / reads combining
 *       feature.
 *       Some system controllers support combining memory writes / reads. When a
 *       long burst write / read is required and combining is enabled, the master
 *       combines consecutive write / read transactions, if possible, and
 *       performs one burst on the Pci instead of two. (see comments)
 *
 * INPUTS:
 *       enWrCombine - MV_TRUE enables write requests combining.
 *       enRdCombine - MV_TRUE enables read requests combining.
 *
 * OUTPUTS:
 *       None.
 *
 * RETURNS:
 *       MV_OK               - on sucess,
 *       MV_NOT_SUPPORTED    - if the controller does not support this feature,
 *       MV_FAIL             - otherwise.
 *
 * COMMENTS:
 *       1.  Example for combined write scenario:
 *           The controller is required to write a 32-bit data to address 0x8000,
 *           while this transaction is still in progress, a request for a write
 *           operation to address 0x8004 arrives, in this case the two writes are
 *           combined into a single burst of 8-bytes.
 *
 *******************************************************************************/
MV_STATUS bspPciEnableCombinedAccess
(
 IN  MV_BOOL     enWrCombine,
 IN  MV_BOOL     enRdCombine
 )
{
  STUB_FAIL;
}

/*
  service routines 
  On Xcat with smi connection to prestera only one device is connected and 
  it's on smi address 0x0. Since we may be invoked from other functions that 
  assume that first phy is on 0x10 we adjust it here.
*/

#define FIX_XCAT_SMI_ADDRESS(x)  if (x >=0x10) x-= 0x10

static inline MV_STATUS ethPhyRegRead(MV_U32 phyAddr, MV_U32 regOffs, MV_U16 *data)
{
  return mvEthPhyRegRead(phyAddr, regOffs, data);
}

static inline MV_STATUS ethPhyRegWrite(MV_U32 phyAddr, MV_U32 regOffs, MV_U16 data)
{
  return mvEthPhyRegWrite(phyAddr, regOffs, data);
}

static inline MV_STATUS smiReadReg(MV_U32 devSlvId, MV_U32  regAddr, MV_U32 *value)
{
  /* perform direct smi read */
  MV_STATUS ret;
  MV_U16    temp1;
  
  FIX_XCAT_SMI_ADDRESS(devSlvId);

  ret = ethPhyRegRead(devSlvId, regAddr, &temp1);
  *value = temp1;
  return (MV_OK == ret)? MV_OK : MV_FAIL;  
}

static inline MV_STATUS smiWriteReg(MV_U32 devSlvId, MV_U32 regAddr, MV_U32 value)
{
  /* Perform direct smi write reg */
  MV_STATUS ret;

  FIX_XCAT_SMI_ADDRESS(devSlvId);

  ret = ethPhyRegWrite(devSlvId, regAddr, value);
  return (MV_OK == ret)? MV_OK : MV_FAIL;  
}

static inline void smiWaitForStatus(MV_U32 devSlvId)
{
#ifdef SMI_WAIT_FOR_STATUS_DONE
  MV_U32 stat;
  unsigned int timeOut;
  int rc;
  
  /* wait for write done */
  timeOut = SMI_TIMEOUT_COUNTER;
  do
  {
    rc = smiReadReg(devSlvId, SMI_STATUS_REGISTER,&stat);
    if (rc != MV_OK) 
      return;
    if (--timeOut < 1)
      return;
  } while ((stat & SMI_STATUS_WRITE_DONE) == 0);
#endif
}

/*******************************************************************************
 * bspSmiInitDriver
 *
 * DESCRIPTION:
 *       Init the TWSI interface 
 *
 * INPUTS:
 *       None.
 *
 * OUTPUTS:
 *       smiAccessMode - direct/indirect mode
 *
 * RETURNS:
 *       MV_OK               - on success
 *       MV_FAIL   - on hardware error
 *
 * COMMENTS:
 *
 *******************************************************************************/
MV_STATUS bspSmiInitDriver
(
 bspSmiAccessMode_ENT  *smiAccessMode
 )
{
  /* Set SMI access speed to be faster than default */
  MV_REG_WRITE(ETH_PHY_SMI_ACCEL_REG, 1<<ETH_PHY_SMI_ACCEL_8_OFFS);
  *smiAccessMode = bspSmiAccessMode_inDirect_E;
  return MV_OK;  
}

/*******************************************************************************
 * bspSmiReadReg
 *
 * DESCRIPTION:
 *       Reads a register from SMI slave.
 *
 * INPUTS:
 *       devSlvId - Slave Device ID
 *       actSmiAddr - actual smi addr to use (relevant for SX PPs)
 *       regAddr - Register address to read from.
 *
 * OUTPUTS:
 *       valuePtr     - Data read from register.
 *
 * RETURNS:
 *       MV_OK               - on success
 *       MV_ERROR   - on hardware error
 *
 * COMMENTS:
 *
 *******************************************************************************/
MV_STATUS bspSmiReadReg
(
 IN  MV_U32  devSlvId,
 IN  MV_U32  actSmiAddr /* not used in xcat */,
 IN  MV_U32  regAddr,
 OUT MV_U32 *valuePtr
 )
{
  /* Perform indirect smi read reg */
  int           rc;
  MV_U32        msb;
  MV_U32        lsb;
  static int first_time=1;

  if (first_time)
  {
    bspSmiAccessMode_ENT  smiAccessMode;
    first_time = 0;
    bspSmiInitDriver(&smiAccessMode);
  }

  /* write addr to read */
  msb = regAddr >> 16;
  lsb = regAddr & 0xFFFF;
  rc = smiWriteReg(devSlvId, SMI_READ_ADDRESS_MSB_REGISTER,msb);
  if (rc != MV_OK)
    return rc;
  
  rc = smiWriteReg(devSlvId, SMI_READ_ADDRESS_LSB_REGISTER,lsb);
  if (rc != MV_OK)
    return rc;

  smiWaitForStatus(devSlvId);

  /* read data */
  rc = smiReadReg(devSlvId, SMI_READ_DATA_MSB_REGISTER,&msb);
  if (rc != MV_OK)
    return rc;
  
  rc = smiReadReg(devSlvId, SMI_READ_DATA_LSB_REGISTER,&lsb);
  if (rc != MV_OK)
    return rc;
  
  *valuePtr = ((msb & 0xFFFF) << 16) | (lsb & 0xFFFF);  
  return 0;
}

/*******************************************************************************
 * bspSmiWriteReg
 *
 * DESCRIPTION:
 *       Writes a register to an SMI slave.
 *
 * INPUTS:
 *       devSlvId - Slave Device ID
 *       actSmiAddr - actual smi addr to use (relevant for SX PPs)
 *       regAddr - Register address to read from.
 *       value   - data to be written.
 *
 * OUTPUTS:
 *        None,
 *
 * RETURNS:
 *       MV_OK               - on success
 *       MV_ERROR   - on hardware error
 *
 * COMMENTS:
 *
 *******************************************************************************/
MV_STATUS bspSmiWriteReg
(
 IN MV_U32 devSlvId,
 IN MV_U32 actSmiAddr /* not used in xcat */,
 IN MV_U32 regAddr,
 IN MV_U32 value
 )
{
  /* Perform indirect smi write reg */
  int           rc;
  MV_U32        msb;
  MV_U32        lsb;
  
  /* write addr to read */
  msb = regAddr >> 16;
  lsb = regAddr & 0xFFFF;
  rc = smiWriteReg(devSlvId, SMI_READ_ADDRESS_MSB_REGISTER,msb);
  if (rc != 0)
    return rc;
  
  rc = smiWriteReg(devSlvId, SMI_READ_ADDRESS_LSB_REGISTER,lsb);
  if (rc != 0)
    return rc;

  /* write data to write */
  msb = value >> 16;
  lsb = value & 0xFFFF;
  rc = smiWriteReg(devSlvId, SMI_WRITE_DATA_MSB_REGISTER,msb);
  if (rc != MV_OK)
    return rc;

  rc = smiWriteReg(devSlvId, SMI_WRITE_DATA_LSB_REGISTER,lsb);
  if (rc != MV_OK) 
    return rc;
  
  smiWaitForStatus(devSlvId);

  return MV_OK;
}

/*** Ethernet Driver ***/
/*******************************************************************************
 * bspEthInit
 *
 * DESCRIPTION: Init the ethernet HW and HAL
 *
 * INPUTS:
 *       port   - eth port number
 *
 * OUTPUTS:
 *       None.
 *
 * RETURNS:
 *       MV_OK if successful, or
 *       MV_FAIL otherwise.
 *
 * COMMENTS:
 *       None.
 *
 *******************************************************************************/
MV_VOID bspEthInit
(
 MV_U8 port
 )
{
  MV_U32 reg;
  reg = MV_REG_READ ( POWER_MNG_CTRL_REG );

  if ( MV_FALSE == mvCtrlPwrClckGet ( ETH_GIG_UNIT_ID, port ) )
  {
    mvCtrlPwrClckSet ( ETH_GIG_UNIT_ID, port, MV_TRUE );
  }

  reg = MV_REG_READ ( POWER_MNG_CTRL_REG );

  mvCtrlPwrMemSet ( ETH_GIG_UNIT_ID, port, MV_TRUE );

  mvEthInit();

  mvEthPortPowerUp ( port );
}

/*** TWSI ***/
/*******************************************************************************
 * bspTwsiInitDriver
 *
 * DESCRIPTION:
 *       Init the TWSI interface 
 *
 * INPUTS:
 *       None.
 *
 * OUTPUTS:
 *       None.
 *
 * RETURNS:
 *       MV_OK               - on success
 *       MV_ERROR   - on hardware error
 *
 * COMMENTS:
 *
 *******************************************************************************/
MV_STATUS bspTwsiInitDriver
(
 MV_VOID
 )
{
  STUB_FAIL;
}

/*******************************************************************************
 * bspTwsiWaitNotBusy
 *
 * DESCRIPTION:
 *       Wait for TWSI interface not BUSY
 *
 * INPUTS:
 *       None.
 *
 * OUTPUTS:
 *       None.
 *
 * RETURNS:
 *       MV_OK               - on success
 *       MV_ERROR   - on hardware error
 *
 * COMMENTS:
 *
 *******************************************************************************/
MV_STATUS bspTwsiWaitNotBusy
(
 MV_VOID
 )
{
  STUB_FAIL;
}

/*******************************************************************************
 * bspTwsiMasterReadTrans
 *
 * DESCRIPTION:
 *       do TWSI interface Transaction 
 *
 * INPUTS:
 *    devId - I2c slave ID                               
 *    pData - Pointer to array of chars (address / data)
 *    len   - pData array size (in chars).              
 *    stop  - Indicates if stop bit is needed.
 *
 * OUTPUTS:
 *       None.
 *
 * RETURNS:
 *       MV_OK               - on success
 *       MV_ERROR   - on hardware error
 *
 * COMMENTS:
 *
 *******************************************************************************/
MV_STATUS bspTwsiMasterReadTrans
(
 IN MV_U8           devId,       /* I2c slave ID                              */ 
 IN MV_U8           *pData,      /* Pointer to array of chars (address / data)*/
 IN MV_U8           len,         /* pData array size (in chars).              */
 IN MV_BOOL         stop         /* Indicates if stop bit is needed in the end  */
 )
{
  STUB_FAIL;
}

/*******************************************************************************
 * bspTwsiMasterWriteTrans
 *
 * DESCRIPTION:
 *       do TWSI interface Transaction 
 *
 * INPUTS:
 *    devId - I2c slave ID                               
 *    pData - Pointer to array of chars (address / data)
 *    len   - pData array size (in chars).              
 *    stop  - Indicates if stop bit is needed.
 *
 * OUTPUTS:
 *       None.
 *
 * RETURNS:
 *       MV_OK               - on success
 *       MV_ERROR   - on hardware error
 *
 * COMMENTS:
 *
 *******************************************************************************/
MV_STATUS bspTwsiMasterWriteTrans
(
 IN MV_U8           devId,       /* I2c slave ID                              */ 
 IN MV_U8           *pData,      /* Pointer to array of chars (address / data)*/
 IN MV_U8           len,         /* pData array size (in chars).              */
 IN MV_BOOL         stop         /* Indicates if stop bit is needed in the end  */
 )
{
  STUB_FAIL;
}

/*******************************************************************************
 * bspIntConnect
 *
 * DESCRIPTION:
 *       Connect a specified C routine to a specified interrupt vector.
 *
 * INPUTS:
 *       vector    - interrupt vector number to attach to
 *       routine   - routine to be called
 *       parameter - parameter to be passed to routine
 *
 * OUTPUTS:
 *       None
 *
 * RETURNS:
 *       MV_OK   - on success
 *       MV_FAIL - on error
 *
 * COMMENTS:
 *       None
 *
 *******************************************************************************/
MV_STATUS bspIntConnect
(
 IN  MV_U32           vector,
 IN  MV_VOIDFUNCPTR   routine,
 IN  MV_U32           parameter
 )
{
  int rc;

  bspIsrParameter = parameter;
  bspIsrRoutine   = routine;

  rc = request_irq(vector, 
                   (irq_handler_t)bspIsr, 
                   IRQF_DISABLED, "PP_interrupt", (void *)&bspIsrParameter);

  return (0 == rc) ? MV_OK : MV_FAIL;

}

/*** Ethernet access MII with the Packet Processor ***/
/*******************************************************************************
 * bspEthPortRxInit
 *
 * DESCRIPTION: Init the ethernet port Rx interface
 *
 * INPUTS:
 *       rxBufPoolSize   - buffer pool size
 *       rxBufPool_PTR   - the address of the pool
 *       rxBufSize       - the buffer requested size
 *       numOfRxBufs_PTR - number of requested buffers, and actual buffers created
 *       headerOffset    - packet header offset size
 *
 * OUTPUTS:
 *       None.
 *
 * RETURNS:
 *       MV_OK if successful, or
 *       MV_FAIL otherwise.
 *
 * COMMENTS:
 *       None.
 *
 *******************************************************************************/
MV_STATUS bspEthPortRxInit
(
 IN MV_U32           rxBufPoolSize,
 IN MV_U8*           rxBufPool_PTR,
 IN MV_U32           rxBufSize,
 INOUT MV_U32        *numOfRxBufs_PTR,
 IN MV_U32           headerOffset,
 IN MV_U32           rxQNum,
 IN MV_U32           rxQbufPercentage[]
 )
{
  DB(printk ( "\nbspEthPortRxInit:\n"));
  DB(printk ( "rxBufPoolSize = 0x%x, rxBufPool_PTR = 0x%x, "
              "rxBufSize = 0x%x, rxQNum = %d.\n",
              rxBufPoolSize, ( unsigned int ) rxBufPool_PTR, rxBufSize,rxQNum ));
  
  return gtEthernetInterfaceRxInit ( rxBufPoolSize, rxBufPool_PTR, rxBufSize,
                                     numOfRxBufs_PTR, headerOffset,
                                     rxQNum, rxQbufPercentage );
}


/*******************************************************************************
 * bspEthPortTxInit
 *
 * DESCRIPTION: Init the ethernet port Tx interface
 *
 * INPUTS:
 *       numOfTxBufs - number of requested buffers
 *
 * OUTPUTS:
 *       None.
 *
 * RETURNS:
 *       MV_OK if successful, or
 *       MV_FAIL otherwise.
 *
 * COMMENTS:
 *       None.
 *
 *******************************************************************************/
MV_STATUS bspEthPortTxInit
(
 IN MV_U32           numOfTxBufs
 )
{
  bspTxModeSetOn = 1;
  return gtEthernetInterfaceTxInit ( numOfTxBufs );
}

/*******************************************************************************
 * bspEthPortEnable
 *
 * DESCRIPTION: Enable the ethernet port interface
 *
 * INPUTS:
 *       None.
 *
 * OUTPUTS:
 *       None.
 *
 * RETURNS:
 *       MV_OK if successful, or
 *       MV_FAIL otherwise.
 *
 * COMMENTS:
 *       None.
 *
 *******************************************************************************/
MV_STATUS bspEthPortEnable
(
 MV_VOID
 )
{
  bspTxModeSetOn = 0;
  return gtEthernetInterfaceEnable();
}

/*******************************************************************************
 * bspEthPortDisable
 *
 * DESCRIPTION: Disable the ethernet port interface
 *
 * INPUTS:
 *       None.
 *
 * OUTPUTS:
 *       None.
 *
 * RETURNS:
 *       MV_OK if successful, or
 *       MV_FAIL otherwise.
 *
 * COMMENTS:
 *       None.
 *
 *******************************************************************************/
MV_STATUS bspEthPortDisable
(
 MV_VOID
 )
{
  return gtEthernetInterfaceDisable();
}

/*******************************************************************************
 * bspEthInputHookAdd
 *
 * DESCRIPTION:
 *       This bind the user Rx callback
 *
 * INPUTS:
 *       userRxFunc - the user Rx callback function
 *
 * OUTPUTS:
 *       None.
 *
 * RETURNS:
 *       MV_OK if successful, or
 *       MV_FAIL otherwise.
 *
 * COMMENTS:
 *       None.
 *
 *******************************************************************************/
MV_STATUS bspEthInputHookAdd
(
 IN BSP_RX_CALLBACK_FUNCPTR    userRxFunc
 )
{
  return gtEthernetRxCallbackBind ( ( MV_Rx_FUNCPTR ) userRxFunc );
}

/*******************************************************************************
 * bspEthPortTxModeSet
 *
 * DESCRIPTION: Set the ethernet port tx mode
 *
 * INPUTS:
 *       if txMode == bspEthTxMode_asynch_E -- don't wait for TX done - free packet when interrupt received
 *       if txMode == bspEthTxMode_synch_E  -- wait to TX done and free packet immediately
 *
 * OUTPUTS:
 *       None.
 *
 * RETURNS:
 *       MV_OK if successful
 *       MV_NOT_SUPPORTED if input is wrong
 *       MV_FAIL if bspTxModeSetOn is zero
 *
 * COMMENTS:
 *       None.
 *
 *******************************************************************************/
MV_STATUS bspEthPortTxModeSet
(
 bspEthTxMode_ENT    txMode
 )
{
  if ( bspTxModeSetOn == 0 )
    return MV_FAIL;

  if ( ( txMode == bspEthTxMode_asynch_E ) || ( txMode == bspEthTxMode_synch_E ) )
    bspEthTxMode = txMode;
  else
    return MV_NOT_SUPPORTED;
  
  if (bspEthTxMode == bspEthTxMode_synch_E)
  {
    DISABLE_MGI_TX_DONE_INTERRUPT(EGIGA_CPU_PORT);
  }
  else
  {
    ENABLE_MGI_TX_DONE_INTERRUPT(EGIGA_CPU_PORT);
  }

  return MV_OK;
}

/*******************************************************************************
 * bspEthPortTx
 *
 * DESCRIPTION:
 *       This function is called after a TxEnd event has been received, it passes
 *       the needed information to the Tapi part.
 *
 * INPUTS:
 *       segmentsList     - A list of pointers to the packets segments.
 *       segmentsLen      - A list of segment length.
 *       numOfSegments   - The number of segment in segment list.
 *
 * OUTPUTS:
 *       None.
 *
 * RETURNS:
 *       MV_OK if successful, or
 *       MV_FAIL otherwise.
 *
 * COMMENTS:
 *       None.
 *
 *******************************************************************************/
MV_STATUS bspEthPortTx
(
 IN MV_U8*           segmentList[],
 IN MV_U32           segmentLen[],   
 IN MV_U32           numOfSegments
 )
{
  int i;
  MV_U8* segmentList1[100];

  for (i = 0; i < numOfSegments; i++)
    segmentList1[i] = (MV_U8* )bspPhys2Virt((MV_U32)(segmentList[i]));
  
  return gtEthernetPacketSend(segmentList1, segmentLen, numOfSegments,MII_DEF_TX_Q);
}

/*******************************************************************************
* bspEthPortRx
*
* DESCRIPTION:
*       This function is called when a packet has received.
*
* INPUTS:
*       rxQueue     	- RX Queue.
*
* OUTPUTS:
*       None.
*
* RETURNS:
*       MV_OK if successful, or
*       MV_FAIL otherwise.
*
* COMMENTS:
*       None.
*
*******************************************************************************/
MV_PKT_INFO* bspEthPortRx
(
    IN MV_U32           rxQueue
)
{
   return gtEthernetPacketReceive(rxQueue);
}

/*******************************************************************************
* bspEthPortTxQueue
*
* DESCRIPTION:
*       This function is called after a TxEnd event has been received, it passes
*       the needed information to the Tapi part.
*
* INPUTS:
*       segmentList     - A list of pointers to the packets segments.
*       segmentLen      - A list of segment length.
*       numOfSegments   - The number of segment in segment list.
*       txQueue         - The TX queue.
*
* OUTPUTS:
*       None.
*
* RETURNS:
*       MV_OK if successful, or
*       MV_FAIL otherwise.
*
* COMMENTS:
*       None.
*
*******************************************************************************/
MV_STATUS bspEthPortTxQueue
(
    IN MV_U8*           segmentList[],
    IN MV_U32           segmentLen[],   
    IN MV_U32           numOfSegments,
    IN MV_U32           txQueue
)
{
  int i;
  MV_U8* segmentList1[100];
  
  for (i = 0; i < numOfSegments; i++)
    segmentList1[i] = (MV_U8* )bspPhys2Virt((MV_U32)(segmentList[i]));
  
  return gtEthernetPacketSend(segmentList1, segmentLen, numOfSegments, txQueue);
}

/*******************************************************************************
 * bspEthTxCompleteHookAdd
 *
 * DESCRIPTION:
 *       This bind the user Tx complete callback
 *
 * INPUTS:
 *       userTxFunc - the user Tx callback function
 *
 * OUTPUTS:
 *       None.
 *
 * RETURNS:
 *       MV_OK if successful, or
 *       MV_FAIL otherwise.
 *
 * COMMENTS:
 *       None.
 *
 *******************************************************************************/
MV_STATUS bspEthTxCompleteHookAdd
(
 IN BSP_TX_COMPLETE_CALLBACK_FUNCPTR userTxFunc
 )
{
  return gtEthernetTxCompleteCallbackBind ( ( MV_Tx_COMPLETE_FUNCPTR ) userTxFunc );
}

/*******************************************************************************
 * bspEthRxPacketFree
 *
 * DESCRIPTION:
 *       This routine frees the received Rx buffer. 
 *
 * INPUTS:
 *       segmentsList     - A list of pointers to the packets segments.
 *       numOfSegments   - The number of segment in segment list.
 *       queueNum        - Receive queue number
 *
 * OUTPUTS:
 *       None.
 *
 * RETURNS:
 *       MV_OK if successful, or
 *       MV_FAIL otherwise.
 *
 * COMMENTS:
 *       None.
 *
 *******************************************************************************/
MV_STATUS bspEthRxPacketFree
(
 IN MV_U8*           segmentList[],
 IN MV_U32           numOfSegments,
 IN MV_U32           queueNum
 )
{
  return gtEthernetRxPacketFree ( segmentList, numOfSegments, queueNum );
}

/*******************************************************************************
* bspEthCpuCodeToQueue
*
* DESCRIPTION:
*       Binds DSA CPU code to RX queue.
*
* INPUTS:
*       dsaCpuCode - DSA CPU code
*       rxQueue    -  rx queue
*
* OUTPUTS:
*       None.
*
* RETURNS:
*       MV_OK if successful, or
*       MV_FAIL otherwise.
*
* COMMENTS:
*       None.
*
*******************************************************************************/
MV_STATUS bspEthCpuCodeToQueue
(
    IN MV_U32 dsaCpuCode,
    IN MV_U8  rxQueue
)
{
    return miiCfgDFSMTForCpuCodes(dsaCpuCode,rxQueue);
}

/********************************* Internal functions ********************************/

static void BSP_traffic_dispatch_task(unsigned long ignored)
{
  DB(printk("Started BSP_traffic_dispatch_task"));
  mgiInfRxReadyJob();
}

static void BSP_traffic_dispatch_tx(unsigned long ignored)
{
  DB(printk("Started BSP_traffic_dispatch_tx"));
  mgiInfTxDoneJob();
}

static void mgiInfTxDoneJob(void)
{
  if(bspEthTxMode == bspEthTxMode_asynch_E)
  {
    mgiInfTxDone();
  
    /* Re-enable TxDone interrupt !!! Remember to renable MISC interrupts too*/
    ENABLE_MGI_TX_DONE_INTERRUPT(EGIGA_CPU_PORT);
  }
  
  return;
}

static void mgiInfRxReadyJob(void)
{
  unsigned long flags;

  spin_lock_irqsave(&mii_rx_lock, flags);
  mii_rx_work_in_progress = 0;
  spin_unlock_irqrestore(&mii_rx_lock, flags);

  mgiInfRxReady();

  /* re-unmask mgi1 specific interrupts */
  ENABLE_MGI_RX_ALL_INTERRUPTS;
  return;
}

static irqreturn_t mgiInfRxReadyIsr(int irq , void *dev_id)
{
  unsigned long flags;

  /* Disable RX interrupts */
  DISABLE_MGI_RX_ALL_INTERRUPTS;

  /* Clear bits for RX */
  MV_REG_WRITE(ETH_INTR_CAUSE_REG(EGIGA_CPU_PORT), 0/*~MGI_RX_READY_MASK*/);

  /* signal Rx task to start working */

  spin_lock_irqsave(&mii_rx_lock, flags);
  if (mii_rx_work_in_progress == 1)
  {
    spin_unlock_irqrestore(&mii_rx_lock, flags);
    return IRQ_HANDLED;    
  }
  
  mii_rx_work_in_progress = 1;
  tasklet_hi_schedule(&rx_tasklet);
  spin_unlock_irqrestore(&mii_rx_lock, flags);
  
  return IRQ_HANDLED;    
}

static irqreturn_t mgiInfTxDoneIsr(int irq , void *dev_id)
{
    /* Disable TxDone interrupt (all interrupts in Isr Ext register) */
    DISABLE_MGI_TX_DONE_INTERRUPT(EGIGA_CPU_PORT);

    if(bspEthTxMode == bspEthTxMode_asynch_E)
    {
      /* signal Tx task to start working */
      tasklet_hi_schedule(&tx_tasklet);
      
      /* Acknowledge mgi-tx-done by Clearing bits for TX_DONE */
      MV_REG_WRITE(ETH_INTR_CAUSE_EXT_REG(EGIGA_CPU_PORT), ~MGI_TX_DONE_MASK);
    }
    else
    {
      // leave disabled for sync mode
    }
    return IRQ_HANDLED; 
}

static irqreturn_t bspIsr
(
 int             irq,
 void            *dev_id,
 struct pt_regs  *regs
 )
{
  if (bspIsrRoutine)
    bspIsrRoutine(dev_id);  

  return IRQ_HANDLED;
}

void mgiInfTxRxIsrConnect(void)
{

  /* unmask mgi1 specific interrupts */
  ENABLE_MGI_RX_ALL_INTERRUPTS;
  ENABLE_MGI_TX_DONE_INTERRUPT(EGIGA_CPU_PORT);

  DB(mvOsPrintf("mgiInfTxRxIsrConnect: enabled   Rx & Tx ok\n"));

  /* connect to port interrupt line(s) */
  if( request_irq( 16, mgiInfRxReadyIsr,
                   (IRQF_DISABLED | IRQF_SAMPLE_RANDOM) , "miiRxReady", NULL ) ) 
  {
    printk( KERN_ERR "IRQ: cannot assign irq%d\n", ETH_PORT_IRQ_NUM(1) );
  }

  if( request_irq( 17, mgiInfTxDoneIsr,
                   (IRQF_DISABLED | IRQF_SAMPLE_RANDOM) , "miiTxDone", NULL ) ) 
  {
    printk( KERN_ERR "IRQ: cannot assign irq%d\n", 17 );
  }

  /*Start interrupr handler tasklets*/
  tasklet_init(&rx_tasklet, BSP_traffic_dispatch_task, (unsigned int) 0);
  tasklet_init(&tx_tasklet, BSP_traffic_dispatch_tx, (unsigned int) 0);
}


int bspSmiScan(int instance)
{
  int found1 = 0;
  int found2 = 0;
  int i;
  MV_U32 data;
  
  /* scan for SMI devices */
  for (i = 0; i < 32;  i++)
  {
    bspSmiReadReg(i, 0, 3, &data);
    if (data != 0xffffffff && data != 0xffff)
    {
      bspSmiReadReg(i, 0, 0x50, &data);
      if (data == 0x000011ab  || data == 0xab110000)
      {
        if (instance == found1++)
        {
          printk("Smi Scan found Marvell device instance %d on smi addr 0x%x\n",
                 instance, i);
          found2 = 1;
          break;
        }
      }
    }
  }

  if (!found2)
  {
    printk("Smi scan found no device\n");
    return -1;
  }
  
  return  i;
}

/* EXPORTS */
EXPORT_SYMBOL(bspCacheDmaFree);
EXPORT_SYMBOL(bspCacheDmaMalloc);
EXPORT_SYMBOL(bspCacheFlush);
EXPORT_SYMBOL(bspCacheInvalidate);
EXPORT_SYMBOL(bspDmaRead);
EXPORT_SYMBOL(bspDmaWrite);
EXPORT_SYMBOL(bspEthInputHookAdd);
EXPORT_SYMBOL(bspEthPortDisable);
EXPORT_SYMBOL(bspEthPortEnable);
EXPORT_SYMBOL(bspEthPortRxInit);
EXPORT_SYMBOL(bspEthPortTx);
EXPORT_SYMBOL(bspEthPortRx);
EXPORT_SYMBOL(bspEthPortTxQueue);
EXPORT_SYMBOL(bspEthPortTxInit);
EXPORT_SYMBOL(bspEthInit);
EXPORT_SYMBOL(bspEthRxPacketFree);
EXPORT_SYMBOL(bspEthTxCompleteHookAdd);
EXPORT_SYMBOL(bspIntConnect);
EXPORT_SYMBOL(bspPciConfigReadReg);
EXPORT_SYMBOL(bspPciConfigWriteReg);
EXPORT_SYMBOL(bspPciEnableCombinedAccess);
EXPORT_SYMBOL(bspPciFindDev);
EXPORT_SYMBOL(bspPciFindDevReset);
EXPORT_SYMBOL(bspPciGetIntMask);
EXPORT_SYMBOL(bspPciGetIntVec);
EXPORT_SYMBOL(bspReset);
EXPORT_SYMBOL(bspResetInit);
EXPORT_SYMBOL(bspSmiInitDriver);
EXPORT_SYMBOL(bspSmiReadReg);
EXPORT_SYMBOL(bspSmiWriteReg);
EXPORT_SYMBOL(bspTwsiInitDriver);
EXPORT_SYMBOL(bspTwsiMasterReadTrans);
EXPORT_SYMBOL(bspTwsiMasterWriteTrans);
EXPORT_SYMBOL(bspTwsiWaitNotBusy);
EXPORT_SYMBOL(bspEthPortTxModeSet);
EXPORT_SYMBOL(bspEthCpuCodeToQueue);
EXPORT_SYMBOL(bspSmiScan);
