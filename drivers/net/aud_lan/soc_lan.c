/*
 * (C) Copyright 2008
 * Siemens AG
 * ATS 11
 * Herbert Bernecker <HerbertBernecker@siemens.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <linux/autoconf.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <asm/setup.h>
#include <asm/delay.h>
#include <asm/dma-mapping.h>
#include <linux/fs.h>

#include <asm/mach-aud_soc/gpio.h>
#include <socGen.h>

#include "irq.h"
#include "irteRev7.h"

#define PRIVATE_DRIVER_NAME        "soc_lan"

// ------ Interrupt ---

#define IRQ_CHB0_RCV_DONE   0x00000020

// ------ SCRB System Control Register Block ---

#define SOC1_SCRB_START     0xBFB00000
#define IO_SCR(offset)      IO_WORD(SOC1_SCRB_START, offset)
#define IRTE_SCRB_CTRL      0x0150

// ------------- KRAM -----------------------

#define KRAM_SIZE           0x2C000
#define KRAM_START_OFFSET   0x100000

#define FREE_CTRL_BASE      KRAM_START_OFFSET + 0x10
#define NUMBER_OF_NRT_FCWs  0x80
#define SIZE_OF_NRT_FCW     0x10
#define NUMBER_OF_NRT_DBs   NUMBER_OF_NRT_FCWs
#define SIZE_OF_NRT_DB      0x80

#define FIRST_FREE_NRT_FCW KRAM_START_OFFSET + 0x8400
#define LAST_FREE_NRT_FCW  (FIRST_FREE_NRT_FCW + (NUMBER_OF_NRT_FCWs -1)*SIZE_OF_NRT_FCW)
#define FIRST_FREE_NRT_DB  (FIRST_FREE_NRT_FCW + NUMBER_OF_NRT_FCWs*SIZE_OF_NRT_FCW)
#define LAST_FREE_NRT_DB   (FIRST_FREE_NRT_FCW +  \
            NUMBER_OF_NRT_FCWs*SIZE_OF_NRT_FCW +  \
            (NUMBER_OF_NRT_DBs -1)*SIZE_OF_NRT_DB)

#define NRT_CTRL_BASE        KRAM_START_OFFSET + 0x8200
#define NRT_CTRL_CHA_BASE    NRT_CTRL_BASE + 6*4
#define NRT_CTRL_CHB_BASE    NRT_CTRL_CHA_BASE +  20*4
#define NRT_CTRL_PORT0_BASE  NRT_CTRL_CHA_BASE +  40*4
#define NRT_CTRL_PORT1_BASE  NRT_CTRL_CHA_BASE +  60*4
#define NRT_CTRL_PORT2_BASE  NRT_CTRL_CHA_BASE +  80*4
#define NRT_CTRL_PORT3_BASE  NRT_CTRL_CHA_BASE + 100*4

#define UC_TABLE_BASE        KRAM_START_OFFSET + 0x28
#define UC_TABLE_LENGTH      0xFFF

#define STATISTC_CTRL_BASE   KRAM_START_OFFSET + 0x006C00

#define INVALID_POINTER      0x1FFFFF

// ------------- DMACW -----------------------
#define SIZE_OF_DMACW         16
#define NUMBER_OF_RX_DMACWs   10
#define NUMBER_OF_TX_DMACWs   10
#define DMACW_ALIGNMENT        8
#define DMACW_ALIGNMENT_MASK   0xFFFFFFF8

#define FRAME_BUFFER_LENGTH      1536  /* max. length without CRC        */

/*-----------------------------------------------*/
/*              structures                       */
/*-----------------------------------------------*/
struct irte_rtx_dsc
{
   unsigned long   *pDMACW;
   unsigned long   *pPhyDMACW;

   unsigned char   *pBuf;
   unsigned char   *pPhyBuf;

   struct irte_rtx_dsc   *pNext;
   struct irte_rtx_dsc   *pPrev;
};

struct irte_private {
   unsigned char   *pDMACWArray;
   unsigned char   *pPhyDMACWArray;
   unsigned int    SizeOfDMACWArray;

   struct irte_rtx_dsc   *pRecvTopDsc;
   struct irte_rtx_dsc   *pRecvDoneDsc;

   struct irte_rtx_dsc   *pSendTopDsc;
   struct irte_rtx_dsc   *pSendDsc;
};

/*-----------------------------------------------*/
/*              fuinction definitions            */
/*-----------------------------------------------*/
static void init_dmacw(void);
static void init_nrt_fcw_ring (unsigned long StartAddress,unsigned int count);
static void init_nrt_db_ring (unsigned long StartAddress, unsigned int count);
static void init_nrt_send_list_headers (unsigned long StartAddress);
static void init_nrt_ctrl_base (unsigned long StartAddress);
static void init_uc_table (unsigned long StartAddress, unsigned int length);
static  int irte_init (struct net_device *dev);
static  int irte_start_xmit(struct sk_buff *skb, struct net_device *dev);
static  int irte_open(struct net_device *dev);
static irqreturn_t irte_interrupt(int irq_in, void *dev_id);
static int irte_halt(struct net_device *dev);
static u32 irte_get_link_status(struct net_device *dev);
static void irte_get_drvinfo(struct net_device *dev, struct ethtool_drvinfo *drvinfo);

/*-----------------------------------------------*/
/*              globals                          */
/*-----------------------------------------------*/
int irte_major = 205;
char* irte_name = "soc_irte";
unsigned char InitDone = 0;
static int verbose = 0;

static struct ethtool_ops ethtool_ops = {
	.get_link = irte_get_link_status,
	.get_drvinfo = irte_get_drvinfo
};

static struct file_operations irte_fops =
{
    .unlocked_ioctl = NULL,
    .read = NULL,
    .open = NULL,
    .release = NULL,
};

struct net_device *irte_dev;
struct irte_private *irte_priv;

static struct irqaction irte_lan_irq = {
   .name     = "audirte LAN irq",
   .flags    = 0,
   .handler  = irte_interrupt,
};

unsigned char MyMACAddr[ETH_ALEN] = {0x04, 0x22, 0x00, 0x00, 0x00, 0x08}; 
static spinlock_t irte_lock;


/*-----------------------------------------------*/
/*              memory                           */
/*-----------------------------------------------*/

void DMA_ALLOC_COHERENT_MEM( void**  virt_ptr, void**  phy_ptr, unsigned long length )
{
    *virt_ptr = dma_alloc_coherent(NULL, length, (dma_addr_t*)(phy_ptr), 0);
    *phy_ptr = (void *)(((unsigned)*phy_ptr) | 0x40000000);
}

void DMA_FREE_COHERENT_MEM( void* virt_ptr, void* phy_ptr, unsigned long length )
{
    phy_ptr = (void *)(((unsigned) phy_ptr) & 0x0fffffff);
   dma_free_coherent(NULL, length, virt_ptr, (dma_addr_t)phy_ptr);
}

/*-----------------------------------------------*/
/*              functions                        */
/*-----------------------------------------------*/

/*-------------------------- DMACW -------------------------------*/
static void free_dmacw(void)
{
   struct irte_private *priv = irte_priv;
   struct irte_rtx_dsc *pTmp,*pEnd;

   pTmp = priv->pSendTopDsc;
   pEnd = priv->pSendTopDsc;

   if(pTmp != NULL) {
      do {
         if(pTmp->pBuf != NULL && pTmp->pPhyBuf != NULL) {
            DMA_FREE_COHERENT_MEM((void*)pTmp->pBuf, (void*)pTmp->pPhyBuf, FRAME_BUFFER_LENGTH);
            pTmp = pTmp->pNext;
         } else pTmp = pEnd;
      } while(pTmp != pEnd);
   }

   pTmp = priv->pRecvTopDsc;
   pEnd = priv->pRecvTopDsc;

   if(pTmp != NULL) {
      do {
         if(pTmp->pBuf != NULL && pTmp->pPhyBuf != NULL) {
            DMA_FREE_COHERENT_MEM((void*)pTmp->pBuf, (void*)pTmp->pPhyBuf, FRAME_BUFFER_LENGTH);
            pTmp = pTmp->pNext;
         } else pTmp = pEnd;
      } while(pTmp != pEnd);
   }

   DMA_FREE_COHERENT_MEM( (void*)priv->pDMACWArray,
                          (void*)priv->pPhyDMACWArray,
                          priv->SizeOfDMACWArray );
}

static void init_dmacw(void)
{
   unsigned char *pDMACWArray;
   unsigned char *pPhyDMACWArray;
   unsigned char *pBuffer;
   unsigned char *pPhyBuffer;
   unsigned long *pDMACW;
   unsigned long *pPhyDMACW;
   unsigned long length = ((FRAME_BUFFER_LENGTH / 4) << 16) & 0x03FF0000;
   unsigned int i,size;

   struct irte_rtx_dsc *pTmp, *pPrevDsc=NULL;
   struct irte_private *priv = irte_priv;

   size = ((NUMBER_OF_RX_DMACWs + NUMBER_OF_TX_DMACWs) * SIZE_OF_DMACW) + DMACW_ALIGNMENT;
   DMA_ALLOC_COHERENT_MEM((void**)&pDMACWArray, (void**)&pPhyDMACWArray, size);
   memset(pDMACWArray, 0, size);

   priv->pDMACWArray      = (unsigned char *)pDMACWArray;
   priv->pPhyDMACWArray   = (unsigned char *)pPhyDMACWArray;
   priv->SizeOfDMACWArray = size;

   pDMACW    = (unsigned long *)( ((unsigned long)(pDMACWArray + DMACW_ALIGNMENT)) &  DMACW_ALIGNMENT_MASK );
   pPhyDMACW = (unsigned long *)( ((unsigned long)(pPhyDMACWArray + DMACW_ALIGNMENT)) &  DMACW_ALIGNMENT_MASK );

   IO_SWITCH(NRT_Send_Descriptor_CHB0)   = (unsigned long)pPhyDMACW;

   for(i=0; i<NUMBER_OF_TX_DMACWs; i++) {

      DMA_ALLOC_COHERENT_MEM((void**)&pBuffer, (void**)&pPhyBuffer, FRAME_BUFFER_LENGTH);

      if(!pBuffer) {
          printk("[%s] init_dmacw - failed to alloc frame buffer \n", PRIVATE_DRIVER_NAME);
         return;
      }
      else *(pDMACW + 3) = (unsigned long)pPhyBuffer;

      memset(pBuffer, 0, FRAME_BUFFER_LENGTH);

      pTmp = (struct irte_rtx_dsc *) kmalloc (sizeof(struct irte_rtx_dsc), GFP_KERNEL);
      if(!pTmp) {
          printk("[%s]: init_dmacw - Failed to allocate descriptor irte_rtx_dsc \n",
              PRIVATE_DRIVER_NAME);
         return;
      }

      pTmp->pDMACW    = pDMACW;
      pTmp->pPhyDMACW = pPhyDMACW;
      pTmp->pBuf      = pBuffer;
      pTmp->pPhyBuf   = pPhyBuffer;
      pTmp->pNext     = NULL;
      if(pPrevDsc == NULL) priv->pSendTopDsc = pTmp;
      else                 pPrevDsc->pNext = pTmp;

      pTmp->pPrev = pPrevDsc;
      pPrevDsc = pTmp;

      // buffer length
      *pDMACW = *pDMACW | length;

      // nextDMACW
      if( i<(NUMBER_OF_TX_DMACWs -1)) *(pDMACW + 1) = (unsigned long)(pPhyDMACW + 4);
      else             *(pDMACW + 1) = IO_SWITCH(NRT_Send_Descriptor_CHB0);

      pDMACW = pDMACW + 4;
      pPhyDMACW = pPhyDMACW + 4;
   }

   pTmp->pNext              = priv->pSendTopDsc;
   priv->pSendTopDsc->pPrev = pTmp;
   priv->pSendDsc           = priv->pSendTopDsc;
   pPrevDsc                 = NULL;

   IO_SWITCH(NRT_Receive_Descriptor_CHB0)   = (unsigned long)pPhyDMACW;

   for(i=0; i<NUMBER_OF_RX_DMACWs; i++) {

      DMA_ALLOC_COHERENT_MEM((void**)&pBuffer, (void**)&pPhyBuffer, FRAME_BUFFER_LENGTH);

      if(!pBuffer) {
          printk("[%s] init_dmacw - failed to alloc frame buffer \n", PRIVATE_DRIVER_NAME);
         return;
      }
      else *(pDMACW + 3) = (unsigned long)pPhyBuffer;

      memset(pBuffer, 0, FRAME_BUFFER_LENGTH);

      pTmp = (struct irte_rtx_dsc *) kmalloc (sizeof(struct irte_rtx_dsc), GFP_KERNEL);   
      if(!pTmp) {
          printk("[%s]: init_dmacw - Failed to allocate descriptor irte_rtx_dsc \n",
              PRIVATE_DRIVER_NAME);
         return;
      }

      pTmp->pDMACW    = pDMACW;
      pTmp->pPhyDMACW = pPhyDMACW;
      pTmp->pBuf      = pBuffer;
      pTmp->pPhyBuf   = pPhyBuffer;
      pTmp->pNext     = NULL;
      if(pPrevDsc == NULL)  priv->pRecvTopDsc = pTmp;
      else                  pPrevDsc->pNext = pTmp;

      pTmp->pPrev = pPrevDsc;
      pPrevDsc = pTmp;

      // buffer length
      *pDMACW = *pDMACW | length;

      // owner == Hardware
      *pDMACW = *pDMACW | ((1<<29));

      // nextDMACW
      if( i<(NUMBER_OF_TX_DMACWs -1)) *(pDMACW + 1) = (unsigned long)(pPhyDMACW + 4);
      else             *(pDMACW + 1) = IO_SWITCH(NRT_Receive_Descriptor_CHB0);

      pDMACW = pDMACW + 4;
      pPhyDMACW = pPhyDMACW + 4;
   }

   pTmp->pNext              = priv->pRecvTopDsc;
   priv->pRecvTopDsc->pPrev = pTmp;
   priv->pRecvDoneDsc       = priv->pRecvTopDsc;

}


/*-------------------------- KRAM -------------------------------*/

static void clean_kram(void)
{
   unsigned long i;
   for(i = 0; i < KRAM_SIZE; i += 4) IO_SWITCH(KRAM_START_OFFSET + i) = 0x00000000L;
}

static void init_nrt_fcw_ring (unsigned long StartAddress,unsigned int count)
{
   unsigned int i;
   unsigned long Address = StartAddress;

   for(i=0; i<(count-1); i++)   {
      IO_SWITCH(Address + i*SIZE_OF_NRT_FCW +  0x0) = 0x00000002;   // opcode
      IO_SWITCH(Address + i*SIZE_OF_NRT_FCW +  0x4) = (Address + (i+1)*SIZE_OF_NRT_FCW) << 11;
      IO_SWITCH(Address + i*SIZE_OF_NRT_FCW +  0x8) = 0;
      IO_SWITCH(Address + i*SIZE_OF_NRT_FCW +  0xC) = 0;
   }

   IO_SWITCH(Address + (count-1)*SIZE_OF_NRT_FCW +  0x0) = 0x00000002;   // opcode
   IO_SWITCH(Address + (count-1)*SIZE_OF_NRT_FCW +  0x4) = 0x1FFFFF << 11; // Address << 11;
   IO_SWITCH(Address + (count-1)*SIZE_OF_NRT_FCW +  0x8) = 0;
   IO_SWITCH(Address + (count-1)*SIZE_OF_NRT_FCW +  0xC) = 0;
}

static void init_nrt_db_ring (unsigned long StartAddress, unsigned int count)
{
   unsigned int i;
   unsigned long Address = StartAddress;

   for(i=0; i<(count-1); i++)   {
      IO_SWITCH(Address + i*SIZE_OF_NRT_DB) = (Address + (i+1)*SIZE_OF_NRT_DB) << 8;   
   }

   IO_SWITCH(Address + (count-1)*SIZE_OF_NRT_DB) = 0x1FFFFF00; //Address << 8;
}

static void init_nrt_send_list_headers (unsigned long StartAddress)
{
   unsigned int i;
   unsigned long Address = StartAddress;

   for(i=0; i<20; i++)   {
      IO_SWITCH(Address + i*4) = 0x1FFFFF;
   }
}

static void init_nrt_ctrl_base (unsigned long StartAddress)
{
   IO_SWITCH(StartAddress) = NRT_CTRL_CHA_BASE;
   init_nrt_send_list_headers(NRT_CTRL_CHA_BASE);

   IO_SWITCH(StartAddress + 0x4) = NRT_CTRL_CHB_BASE;
   init_nrt_send_list_headers(NRT_CTRL_CHB_BASE);

   IO_SWITCH(StartAddress + 0x8) = NRT_CTRL_PORT0_BASE;
   init_nrt_send_list_headers(NRT_CTRL_PORT0_BASE);

   IO_SWITCH(StartAddress + 0xC) = NRT_CTRL_PORT1_BASE;
   init_nrt_send_list_headers(NRT_CTRL_PORT1_BASE);

   IO_SWITCH(StartAddress + 0x10) = NRT_CTRL_PORT2_BASE;
   init_nrt_send_list_headers(NRT_CTRL_PORT2_BASE);

   IO_SWITCH(StartAddress + 0x14) = NRT_CTRL_PORT3_BASE;
   init_nrt_send_list_headers(NRT_CTRL_PORT3_BASE);
}

static void init_uc_table (unsigned long StartAddress, unsigned int length) 
{
   unsigned int i;
   unsigned long Address = StartAddress;
   unsigned int len = 2 * length; // 64-Bit Entry !!!

   for(i=0; i<len; i++) {
      IO_SWITCH(Address + i*0x4) = 0x00000000;
   }
}

static void irte_interrupt_enable(void)
{
   IO_SWITCH(SP_IRQ_MODE)      = 0x0000000CL;
   IO_SWITCH(SP_IRQ_ACTIVATE)  = 0x00000001L;
   IO_SWITCH(SP_IRQ1_MASK_IRT) = 0x00000000L;
   IO_SWITCH(SP_IRQ1_MASK_NRT) = 0x00000020L;
}

static void irte_interrupt_disable(void)
{
   IO_SWITCH(SP_IRQ1_MASK_IRT) = 0x00000000L;
   IO_SWITCH(SP_IRQ1_MASK_NRT) = 0x00000000L;
   IO_SWITCH(SP_IRQ_MODE)      = 0x0000000FL;
}

static int switch_status_wait(unsigned long status, unsigned long mask)
{
   unsigned long count;

   for(count=0; count<0x00FF; count++) {
      if( (IO_SWITCH(Switch_Status) & mask) == status ) return 0;
      udelay(1000); 
   }

   printk("[%s]: Failed to get switch status = %lx - Switch_Status = %lx \n",
                        PRIVATE_DRIVER_NAME,
      status, IO_SWITCH(Switch_Status) & mask);
   return 1;
}

/*===========================================================================*/
/*                              PHY Functions                                */
/*===========================================================================*/
/* Basic Mode Control Register */
#define BMCR      0x00
/* Basic Mode Status Register */
#define BMSR      0x01
/* PHY Identifier Register #1 */
#define PHYIDR1   0x02
/* PHY Identifier Register #2 */
#define PHYIDR2   0x03
/* PHY Status Register */
#define PHYSTS    0x10

int phy_mdio_read( unsigned char  addr,
                   unsigned char  reg,
                   unsigned short *value )
{
   unsigned short cmd = 0x0800 | (addr << 5) | reg;

   IO_SWITCH(MD_CA) = cmd;
   udelay(10);
   IO_SWITCH(MD_CA) = 0x0000;
   udelay(10);
   *value = IO_SWITCH(MD_Data);
   return 0;
}

int phy_mdio_write( unsigned char addr,
                    unsigned char reg,
                    unsigned short value )
{
   unsigned short cmd = 0x0C00 | (addr << 5) | reg;

   IO_SWITCH(MD_Data) = value;
   udelay(10);
   IO_SWITCH(MD_CA) = cmd;
   udelay(10);
   IO_SWITCH(MD_CA) = 0x0000;

   return 0;
}

#ifdef CONFIG_AUD_SOC1_CP1626
void phy_init(void)
{
	//IO_GPIO_PORT_MODE_0_L &=  0x00000000; // default function (GPIO)
	IO_GPIO_PORT_MODE_0_L |=  0x000A00AA; // alternate function 1	

	//IO_GPIO_PORT_MODE_0_H &=  0x00000000; // default function (GPIO)
	IO_GPIO_PORT_MODE_0_H |=  0x000A00AA; // alternate function 1

	//IO_GPIO_PORT_MODE_1_L &=  0x00000000; // default function (GPIO)
	IO_GPIO_PORT_MODE_1_L |=  0x000A00AA; // alternate function 1

	//IO_GPIO_PORT_MODE_1_H &=  0xFFFFFFC0; // default function (GPIO)
	IO_GPIO_PORT_MODE_1_H |=  0x00000015; // alternate function 0
}
#else
void phy_init(void)
{
   // Set function: MII1 (GPIO 15 - 0) 
   //              15 14  13 12  11 10   9  8   7  6   5  4   3  2   1  0
   // GPIO15 - 0 : 00 00  00 00  00 00  00 00  00 00  00 00  00 00  00 00 
   IO_GPIO_PORT_MODE_0_L &=  0x00000000; // default function (GPIO)
   IO_GPIO_PORT_MODE_0_L |=  0xAAAAAAAA; // alternate function 1 

   // Set function: MII2 (GPIO 25 - 16) 
   //              31 30  29 28  27 26  25 24  23 22  21 20  19 18  17 16
   // GPIO31 -16 : 00 00  00 00  00 00  00 00  00 00  00 00  00 00  00 00 
   IO_GPIO_PORT_MODE_0_H &=  0x00000000; // default function (GPIO)
   IO_GPIO_PORT_MODE_0_H |=  0xAAAAAAAA; // alternate function 1

   // Set direction: 1 == input
   IO_GPIO_IOCTRL_0 |= 0xFCF0FCF0;

   // Set function: MDIO (GPIO 48) / MDC (GPIO 49) / RES_PN_N (GPIO 50)
   //              63 62  61 60  59 58  57 56  55 54  53 52  51 50  49 48
   // GPIO63 -48 : xx xx  xx xx  xx xx  xx xx  xx xx  xx xx  xx 00  00 00 
   IO_GPIO_PORT_MODE_1_H &=  0xFFFFFFC0; // default function (GPIO)
   IO_GPIO_PORT_MODE_1_H |=  0x00000015; // alternate function 0

#if (defined CONFIG_AUD_SOC1_CP1500 && (CONFIG_AUD_SOC1_CP1500_PROTO_VERSION == 3))
   //new CP1500 HW - Proto Model 3

   // Set function: MII1_MDINT_N (GPIO 84) / MII2_MDINT_N (GPIO 90) 
   //                 95  94   93  92   91  90   89  88   87  86   85  84   83  82   81  80
   // GPIO 80 - 95 :  xx  xx   xx  xx   xx  00   xx  xx   xx  xx   xx  00   xx  xx   xx  xx
   IO_GPIO_PORT_MODE_2_H &=  0xFFCFFCFF; // default function (GPIO)

   // Set direction: PN1_INTA (GPIO 84) / PN1_INTB (GPIO 90)
   IO_GPIO_IOCTRL_2 |= 0x04100000; // GPIO 84/90 is input

#else

   // Set function: PN1_INTA (GPIO 180) / PN1_INTB (GPIO 188)
   //                191 190  189 188  187 186  185 184  183 182  181 180  179 178  177 176
   // GPIO191 -176 :  xx  xx   xx  00   xx  xx   xx  xx   xx  xx   xx  00   xx  xx   xx  xx
   IO_GPIO_PORT_MODE_5_H &=  0xFCFFFCFF; // default function (GPIO)

   // Set direction: PN1_INTA (GPIO 180) / PN1_INTB (GPIO 188)
   IO_GPIO_IOCTRL_5 |= 0x10100000; // GPIO 188/180 is input

#endif

   // Set function: RXDO_A (GPIO 4) / RXD1_A (GPIO 5) / RXD2_A (GPIO 6) / RXD3_A (GPIO 7)
   //             15 14  13 12  11 10   9  8   7  6   5  4   3  2   1  0
   // GPIO15 -0 : xx xx  xx xx  xx xx  xx xx  00 00  00 00  xx xx  xx xx 
   IO_GPIO_PORT_MODE_0_L &=  0xFFFF00FF; // default function (GPIO)

   // Set function: RXDO_B (GPIO 20) / RXD1_B (GPIO 21) / RXD2_B (GPIO 22) / RXD3_B (GPIO 23)
   //              31 30  29 28  27 26  25 24  23 22  21 20  19 18  17 16
   // GPIO31 -16 : xx xx  xx xx  xx xx  xx xx  00 00  00 00  xx xx  xx xx 
   IO_GPIO_PORT_MODE_0_H &=  0xFFFF00FF; // default function (GPIO)   
}
#endif

/*===========================================================================*/
/*                boot arguments                                            */
/*==========================================================================*/
void get_bootargs_mac(void)
{
   char tmp_cmdline[COMMAND_LINE_SIZE];
   strlcpy(tmp_cmdline, saved_command_line, COMMAND_LINE_SIZE);

   if(strstr(tmp_cmdline,"mac=")==NULL){
       printk("[%s] Use default mac-address %02X:%02X:%02X:%02X:%02X:%02X \n",
              PRIVATE_DRIVER_NAME,
         MyMACAddr[0],MyMACAddr[1],MyMACAddr[2],MyMACAddr[3],
         MyMACAddr[4],MyMACAddr[5] );
   } else {
      char *single_arg,*zeile;

      zeile = tmp_cmdline;
      while((single_arg = strsep(&zeile," "))!=NULL) {
         if(strstr(single_arg,"mac=")!=NULL) {
            strsep(&single_arg,"=: \0");

            MyMACAddr[0] = simple_strtol(strsep(&single_arg,"=: \0"),NULL,16);
            MyMACAddr[1] = simple_strtol(strsep(&single_arg,"=: \0"),NULL,16);
            MyMACAddr[2] = simple_strtol(strsep(&single_arg,"=: \0"),NULL,16);
            MyMACAddr[3] = simple_strtol(strsep(&single_arg,"=: \0"),NULL,16);
            MyMACAddr[4] = simple_strtol(strsep(&single_arg,"=: \0"),NULL,16);
            MyMACAddr[5] = simple_strtol(strsep(&single_arg,"=: \0"),NULL,16);

            printk("[%s] Use bootargs mac-address %02X:%02X:%02X:%02X:%02X:%02X \n",
                   PRIVATE_DRIVER_NAME, 
                    MyMACAddr[0],MyMACAddr[1],MyMACAddr[2],
                    MyMACAddr[3],MyMACAddr[4],MyMACAddr[5] );

         }
      }
   }
}

/*===========================================================================*/
/*                              Driver Functions                             */
/*===========================================================================*/

static int irte_init (struct net_device *dev)
{

   if(InitDone) goto Start;

   printk("[%s]: irte_init \n", PRIVATE_DRIVER_NAME);
   IO_WRITE(Port_Control, 0x333);   // disable port 0/1/2

   IO_SCR(IRTE_SCRB_CTRL) = 0x7; // MII

   IO_WRITE(Switch_Control, 0x2);   // Soft-Reset; Konfigurationsmodus
   udelay(10);
   switch_status_wait(0x7FFF, 0x7FFF);

        // ---------- PHY/SMI -------------------
   phy_init();

   IO_WRITE(PHY_Cmd_P0, 0x20);
   IO_WRITE(PHY_Cmd_P1, 0x21);

   IO_WRITE(MAC_Control_P0, 0x48);      // Force MII; Full Duplex
   IO_WRITE(Transmit_Control_P0, 0x5);   // Suppress Padding / Transmit Enable
   IO_WRITE(Receive_Control_P0, 0x1D);   // Strip CRC; Short-/Long-/Receive Enable

   IO_WRITE(MAC_Control_P1, 0x48);      // Force MII; Full Duplex
   IO_WRITE(Transmit_Control_P1, 0x5);   // Suppress Padding / Transmit Enable
   IO_WRITE(Receive_Control_P1, 0x1D);   // Strip CRC; Short-/Long-/Receive Enable

   // ---------- Global --------------------
   IO_WRITE(SA_31_0, 0x00000008);
   IO_WRITE(SA_47_32, 0x0522);
   // NRT_Ctrl_Base
   IO_WRITE(NRT_Ctrl_Base, NRT_CTRL_BASE);
   init_nrt_ctrl_base(NRT_CTRL_BASE);
   // Free_Ctrl_Base
   IO_WRITE(Free_Ctrl_Base, FREE_CTRL_BASE);
   IO_WRITE(FREE_CTRL_BASE, FIRST_FREE_NRT_FCW);
   IO_WRITE(FREE_CTRL_BASE + 0x4, LAST_FREE_NRT_FCW);
   IO_WRITE(FREE_CTRL_BASE + 0x8, FIRST_FREE_NRT_DB);
   IO_WRITE(FREE_CTRL_BASE + 0xC, LAST_FREE_NRT_DB);
   IO_WRITE(FREE_CTRL_BASE + 0x10, 0xFFFFFFFF);   // FIRST_FREE_SRT_FCW
   IO_WRITE(FREE_CTRL_BASE + 0x14, 0xFFFFFFFF);   // LAST_FREE_SRT_FCW
   init_nrt_fcw_ring(FIRST_FREE_NRT_FCW, NUMBER_OF_NRT_FCWs);
   init_nrt_db_ring(FIRST_FREE_NRT_DB, NUMBER_OF_NRT_DBs);
   IO_WRITE(Min_Idle_Time, 0x1D0196);
   IO_WRITE(MAC_Empty_Count, 0x1010);
   IO_WRITE(NRT_Transfer_Control, 0);
   IO_WRITE(NRT_SafetyMargin, 0xA50640);
   IO_WRITE(Switch_Setup, 0x3FE);
   IO_WRITE(Statistic_Ctrl_Base, STATISTC_CTRL_BASE);
   memset((void*)(SOC1_IRTE_START_ADDR + STATISTC_CTRL_BASE), 0, 512);

   // ---------- Cycle --------------------
   IO_WRITE(Cycle_Length, 0xFFFFF);
   IO_WRITE(Cycle_Cnt_Entity, 0x1);

   // ---------- KRAM ----------------------
   IO_WRITE(NRT_FCW_Upper_Limit, 0xFFF);
   IO_WRITE(NRT_FCW_Lower_Limit, 0x7);
   IO_WRITE(NRT_DB_Upper_Limit, 0xFFF);
   IO_WRITE(NRT_DB_Lower_Limit, 0x28);

   IO_WRITE(HOL_Upper_Limit_CH, 0x27);
   IO_WRITE(HOL_Lower_Limit_CH, 0xFFF);
   IO_WRITE(HOL_Upper_Limit_Port, 0x68);
   IO_WRITE(HOL_Lower_Limit_Port, 0xFFF);

   IO_WRITE(UC_Table_Base, UC_TABLE_BASE);
   IO_WRITE(UC_Table_Length, UC_TABLE_LENGTH);
   init_uc_table(UC_TABLE_BASE, UC_TABLE_LENGTH);
   IO_WRITE(UC_Default_Ctrl, 0xE);

   IO_WRITE(MC_Table_Base, 0x1FFFFF);
   IO_WRITE(MC_Table_Length, 0x0);
   IO_WRITE(MC_Default_Ctrl, 0xE);

   IO_WRITE(UCMC_LFSR_Ctrl, 0x4A4);
   IO_WRITE(UC_Table_Range, 0x7);

   IO_WRITE(VLAN_Table_Base, 0x1FFFFF);
   IO_WRITE(SS_Queue_Disable, 0x17);

   IO_WRITE(NRT_FCW_Count, NUMBER_OF_NRT_FCWs);
   IO_WRITE(NRT_DB_Count, NUMBER_OF_NRT_DBs);

   IO_WRITE(Group_Number, 0x00);
   IO_WRITE(FC_MASK, 0x3FF);
   IO_WRITE(HOL_MASK_P0, 0x3FF);
   IO_WRITE(HOL_MASK_P1, 0x3FF);

   // ---------- NRT -----------------------
   // CHA
   IO_WRITE(NRT_Receive_MapCHA, 0x3FF);
   IO_WRITE(NRT_Enable_CHA0, 0x6);
   IO_WRITE(NRT_Send_Descriptor_CHA0, 0x0);
   IO_WRITE(NRT_Receive_Descriptor_CHA0, 0x0);
   IO_WRITE(NRT_Enable_CHA1, 0x6);
   IO_WRITE(NRT_Send_Descriptor_CHA1, 0x0);
   IO_WRITE(NRT_Receive_Descriptor_CHA1, 0x0);

   // CHB
   init_dmacw();
   IO_WRITE(NRT_Receive_MapCHB, 0x3FF);
   IO_WRITE(NRT_Enable_CHB0, 0x2);
   IO_WRITE(NRT_Enable_CHB0, 0x5);
   IO_WRITE(NRT_Enable_CHB1, 0x6);
   IO_WRITE(NRT_Send_Descriptor_CHB1, 0x0);
   IO_WRITE(NRT_Receive_Descriptor_CHB1, 0x0);

   // No  ARP/DCP Table
   IO_WRITE(ARP_Table_Base, INVALID_POINTER);
   IO_WRITE(DCP_Table_Base, INVALID_POINTER);

   // ---------- Port 0 --------------------
   IO_WRITE(Min_Preamble_Count_P0, 0x0030);
   IO_WRITE(NRT_Control_P0, 0x106E);   // 100 MBit; Vollduplex; short enable
   IO_WRITE(Default_VLAN_Tag_P0, 0x0001);

   // ---------- Port 1 --------------------
   IO_WRITE(Min_Preamble_Count_P1, 0x30);
   IO_WRITE(NRT_Control_P1, 0x107E);   // 100 MBit; Vollduplex; short enable
   IO_WRITE(Default_VLAN_Tag_P1, 0x0001);

  Start:
   // ---------- Start the controller ------
#ifdef CONFIG_AUD_SOC1_CP1626
   IO_WRITE(Port_Control, 0x300);   // enable port 0 / port 1
#else
   IO_WRITE(Port_Control, 0x330);   // enable port 0
#endif
   IO_WRITE(Switch_Control, 0x3F2D);   // Betriebsmodus; stat / smi/kram/nrt/int/pot0/port1
   udelay(10);
   if(!switch_status_wait(0x40FA, 0x71FF)) {
       InitDone = 1;
   }
   else {
       printk("soc_lan: problem switch_status_wait\n");
       InitDone = 1;
   }

return 0;
}

static int irte_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
   unsigned long flags;
   unsigned long L0_Reg = 0;
   unsigned long *pDMACW;
   unsigned short length = skb->len;
   unsigned char *buffer = skb->data;
   struct irte_private *priv = irte_priv;
   //printk("irte start_xmit irq=%#x\n", IO_SWITCH(SP_IRQ1_NRT));
   //irte_get_link_status(NULL);
   spin_lock_irqsave(&irte_lock, flags);

   if(length > FRAME_BUFFER_LENGTH) goto Done;
   dev->trans_start = jiffies;

   if (length <= 0) {
      printk ("[%s]: %s bad packet size: %d\n", PRIVATE_DRIVER_NAME, dev->name, length);
      goto Done;
   }

   pDMACW = priv->pSendDsc->pDMACW;
   if(pDMACW == NULL) {
      printk ("[%s]: pDMACW = %x\n", PRIVATE_DRIVER_NAME, (int) pDMACW);
      goto Done;
   }

   L0_Reg = *pDMACW;

   if(L0_Reg & (1<<29))   {
       printk("[%s]: dev busy  L0_Reg = %x (priv)%x->(pSendDesc)%x->(pDMACW)%x\n",
              PRIVATE_DRIVER_NAME, 
          (int)L0_Reg, (int)priv, (int)(priv->pSendDsc), (int)(priv->pSendDsc->pDMACW) );
      goto Done;
   }

   L0_Reg &= 0x03FF0000;
   memcpy(priv->pSendDsc->pBuf, buffer, length);

   L0_Reg = (length&0x07FF) | L0_Reg;   
   L0_Reg = (1<<29) | L0_Reg;

   *pDMACW = L0_Reg;

   // next descriptor
   priv->pSendDsc = priv->pSendDsc->pNext;

   // enable tx
   IO_SWITCH(NRT_Enable_CHB0) = 0x7;

   ((struct net_device_stats *)netdev_priv(dev))->tx_packets++;
   ((struct net_device_stats *)netdev_priv(dev))->tx_bytes += (length);

  Done:
   dev_kfree_skb (skb);
   spin_unlock_irqrestore(&irte_lock, flags);

   return 0;
}

static irqreturn_t irte_interrupt( int irq_in,
                                   void *dev_id )
{
   struct net_device *dev = irte_dev;
   struct sk_buff *skb;
   struct irte_private *priv = irte_priv;
   struct irte_rtx_dsc *pFirstDsc = priv->pRecvDoneDsc;
   unsigned long Reg;  //32 Bit Reg vom DMACW
   unsigned long *pFrameBuffer;
   unsigned long  FrameLength;
   unsigned char *pBuffer;
   unsigned long IrqNrt;

   spin_lock(&irte_lock);

   IrqNrt = IO_SWITCH(SP_IRQ1_NRT);
   IO_SWITCH(SP_INT_ACK_NRT) = IrqNrt;

   if( !(IrqNrt & IRQ_CHB0_RCV_DONE) ) goto Done;
   for (;;) {
      Reg = *((unsigned long *)(priv->pRecvDoneDsc->pDMACW));
      if( (Reg & (1<<29)) == 0 ) { 
         FrameLength = *(priv->pRecvDoneDsc->pDMACW) & 0x07FF;
         pFrameBuffer = (unsigned long *)(priv->pRecvDoneDsc->pBuf);

         skb = NULL;
         skb = dev_alloc_skb(FrameLength +2);
         if (skb == NULL) {
             printk(KERN_WARNING "%s]: Couldn't allocate a sk_buff of size %d.\n",
                    PRIVATE_DRIVER_NAME, (int) FrameLength);
            ((struct net_device_stats *)netdev_priv(dev))->rx_dropped++;
         }
         else {
            skb_reserve(skb,2);

            skb->dev = dev;
            pBuffer = skb_put(skb,FrameLength);
            memcpy(pBuffer, pFrameBuffer, FrameLength);
            skb->protocol = eth_type_trans(skb, dev);

            netif_rx(skb);

            ((struct net_device_stats *)netdev_priv(dev))->rx_packets++;
            ((struct net_device_stats *)netdev_priv(dev))->rx_bytes += FrameLength;
         }

         Reg = Reg | (1<<29);
         Reg = Reg & 0xFFFFF800;
         *(priv->pRecvDoneDsc->pDMACW) = Reg;
      }

      if(priv->pRecvDoneDsc->pNext == pFirstDsc)
         break;
      else
         priv->pRecvDoneDsc = priv->pRecvDoneDsc->pNext;
   }

   IO_WRITE(NRT_Enable_CHB0, 0x5);

  Done:
   IO_SWITCH(SP_EOI_IRQ1) = 0x00000007;
   spin_unlock(&irte_lock);   

   return IRQ_HANDLED;
}

static int irte_open(struct net_device *dev)
{
   int ret = 0;
   static int firstrun = 0;

   printk("[%s]: irte_open dev->irq=%d dev->name=%s(%#x) \n", PRIVATE_DRIVER_NAME, 
          dev->irq, dev->name, *((unsigned *)(SOC1_IRTE_START_ADDR + 0x19400)));

   if (firstrun == 0) {
       ret = setup_irq(IRQ1_SP, &irte_lan_irq);
       get_bootargs_mac();
       memcpy(dev->dev_addr, MyMACAddr, ETH_ALEN);
       firstrun = 1;
   }
   else {
	irte_init(dev);
   }
   
   if(irte_get_link_status(irte_dev))
     printk("[%s]: irte init P0 link is UP\n", PRIVATE_DRIVER_NAME);
   netif_start_queue(dev);
   irte_interrupt_enable();
   IO_WRITE(NRT_Enable_CHB0, 0x5);

   return (ret);
}

static int irte_halt(struct net_device *dev)
{
   unsigned long i,status;

   printk("[%s]: irte_halt \n", PRIVATE_DRIVER_NAME);

   irte_interrupt_disable();
   netif_stop_queue(dev);

   if(InitDone) {
      //Alle Ports schliessen
      IO_WRITE(Port_Control, 0x33);

      //* IRT_Control alle ports disabled */
      IO_WRITE(IRT_Control, 0x0);

       //* 20 ms */
      udelay(20);

      //* irt/nrt/ports disabled */
      IO_SWITCH(Switch_Control) = IO_SWITCH(Switch_Control) & 0x00000E7F;

      //* 10 ms */
      udelay(10);

      //* Status */
      for(i=0; i<0xFFFF; i++) {
         status = IO_SWITCH(Switch_Status) & 0x3000;
         if( status == 0x3000 || status == 0x2000 || status == 0x1000) return 0;
         udelay(10);
      }

      InitDone = 0;
      printk("Failed to reset irte! \n");
   }
   return -1;
}

static void irte_timeout(struct net_device *dev)
{
    printk("[%s]: timeout\n", PRIVATE_DRIVER_NAME);

   /* Restart the adapter. */
   irte_halt(irte_dev);
   irte_init(irte_dev);
   netif_wake_queue(dev);
}

static struct net_device_stats *irte_stats(struct net_device *dev)
{
   return(netdev_priv(dev));
}

static void irte_get_drvinfo(struct net_device *dev,
	struct ethtool_drvinfo *drvinfo)
{
    
    printk("[%s]: driverinfo called\n", PRIVATE_DRIVER_NAME);

	strncpy(drvinfo->driver, "soc_lan", ARRAY_SIZE(drvinfo->driver));
	strncpy(drvinfo->version, "N/A",    ARRAY_SIZE(drvinfo->version));
	strncpy(drvinfo->fw_version, "N/A", ARRAY_SIZE(drvinfo->fw_version));
	strncpy(drvinfo->bus_info, "N/A",	ARRAY_SIZE(drvinfo->bus_info));

}

static u32 irte_get_link_status(struct net_device *dev)
{
	unsigned long link_status;

	// we have to read link change byte
	// before reading PHY_Stat_P0 to get
	// the correct value
	IO_READ(Link_Change);

//	IO_READ(PHY_Cmd_P0);
	link_status = IO_READ(PHY_Stat_P1);

	if (verbose)
		printk("[%s]: Link status called: %lu : %i\n",
			PRIVATE_DRIVER_NAME, link_status, LinkStatusMask);
	if (link_status & LinkStatusMask)
		return 1;
	else
		return 0;
}

static const struct net_device_ops irte_netdev_ops =
{
    .ndo_get_stats        = irte_stats,
    .ndo_open             = irte_open,
    .ndo_stop             = irte_halt,
    .ndo_start_xmit       = irte_start_xmit,
    .ndo_tx_timeout       = irte_timeout,
};
   
static int __init irte_drv_init (void)
{
   struct net_device_stats *priv = NULL;
   int ret;

   printk("[%s]: irte_drv_init aud_lan \n", PRIVATE_DRIVER_NAME);

   if((ret = register_chrdev(irte_major, irte_name, &irte_fops))   ==   -1) 
      {
          printk("[%s]: device already in use.\n", PRIVATE_DRIVER_NAME);
      return ret;
   }

   irte_dev = alloc_etherdev(sizeof(struct net_device_stats));   
   if(irte_dev == NULL) {
       printk("[%s]: Failed to alloc etherdev! \n", PRIVATE_DRIVER_NAME);
      return (-ENODEV);
   }

   priv = (struct net_device_stats *)netdev_priv(irte_dev);
   if(priv == NULL) {
       printk("[%s]: Failed to alloc priv! \n", PRIVATE_DRIVER_NAME);
      return (-ENODEV);
   }
   memset(priv, 0, sizeof(struct net_device_stats));

   irte_priv = kmalloc(sizeof(struct irte_private), GFP_KERNEL);
   if(irte_priv == NULL) {
       printk("%s]: Failed to alloc irte_priv! \n", PRIVATE_DRIVER_NAME);
      return (-ENODEV);
   }
   memset(irte_priv, 0, sizeof(struct irte_private));

   ether_setup(irte_dev);

   irte_dev->base_addr        = 0x5;
   irte_dev->irq              = IRQ1_SP;

   irte_dev->netdev_ops       = &irte_netdev_ops;
   irte_dev->watchdog_timeo   = 100; 

   spin_lock_init(&irte_lock);

   irte_init(irte_dev);
   ret = register_netdev(irte_dev);

   SET_ETHTOOL_OPS(irte_dev,&ethtool_ops);

   if(irte_get_link_status(irte_dev))
     printk("[%s]: irte init P0 link is UP\n", PRIVATE_DRIVER_NAME);
   if (IO_READ(PHY_Stat_P0) & AutoNegComplMask)
     printk("[%s]: irte init P0 Auto negotiation complete\n", PRIVATE_DRIVER_NAME);

   return ret;
}

static void __exit irte_drv_cleanup (void)
{
    printk("[%s]: irte_drv_cleanup \n", PRIVATE_DRIVER_NAME);

   unregister_netdev(irte_dev);
   free_irq(IRQ1_SP, NULL);
   disable_irq(IRQ1_SP);
   free_dmacw();
   clean_kram();
   free_netdev(irte_dev);
   kfree(irte_priv);
   unregister_chrdev(irte_major, irte_name);

}

module_init(irte_drv_init);
module_exit(irte_drv_cleanup);


/*****************************************************************************/
/*  end of file soc_lan.c                                                   */
/*****************************************************************************/

