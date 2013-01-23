/*******************************************************************************
Copyright (C) Marvell International Ltd. and its affiliates
	
********************************************************************************
Marvell GPL License Option

If you received this File from Marvell, you may opt to use, redistribute and/or 
modify this File in accordance with the terms and conditions of the General 
Public License Version 2, June 1991 (the "GPL License"), a copy of which is 
available along with the File in the license.txt file or by writing to the Free 
Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 or 
on the worldwide web at http://www.gnu.org/licenses/gpl.txt. 
	
THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED 
WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY 
DISCLAIMED.  The GPL License provides additional details about this warranty 
disclaimer.
	
*******************************************************************************/
#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/mm.h>
#include <linux/in.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/rtnetlink.h>
#include <linux/notifier.h>
#include <linux/etherdevice.h>
#include <linux/if_vlan.h>
#include <linux/if_bridge.h>
	
#if defined (MV_PRESTERA_SWITCH)
#include "mvOs.h"
#include "hwIf/gtHwIf.h"
#include "util/gtUtils.h"
#include "util/gtNetTransmit.h"
#include "util/gtNetReceive.h"
#include "vlan/gtVlan.h"
#include "sdma/gtBridge.h"
//#include "ethfp/gbe/mvEthGbe.h"
//#include "ethfp/gbe/mvEthRegs.h"
	
#if 0
#define MV_DEBUG 
#endif
#ifdef MV_DEBUG
#define DB(x) x
#else
#define DB(x)
#endif

#define printf printk
	
/****************************************************** 
* driver internal definitions --                     *
******************************************************/ 
	
#define SWITCH_MTU            1512
	
/* rings length */
#define PRESTERA_TXQ_LEN   2
#define PRESTERA_RXQ_LEN   2

#define PRESTERA_Q_NUM		8
#define PRESTERA_PORT_NUM	24

#define PRESTERA_PORT_STATUS_REG 0x0A800010

/* MG - Address Decoding registers */
#define PRESTERA_AD_BASE_ADDR(index) (0x30C + (index*8))
#define PRESTERA_AD_SIZE(index)      (0x310 + (index*8))
#define PRESTERA_AD_BARE              0x34C                /* Base address enable */

#define PRESTERA_AD_BARE_BITS_SET(value, index)    value &= ~(1<<index)
#define PRESTERA_AD_BASE_ADDR_BITS_SET(base, attr, target) ((base & 0xFFFF0000) | (attr << 8) | target)
#define PRESTERA_AD_DRAM_TARGET_ID 0

static MV_U8 decodingTargetAttribures[] = {0xE, 0xD, 0xB, 0x7};

#define ETH_CSUM_MIN_BYTE_COUNT     72
#define ETH_RX_IP_FRAME_TYPE_BIT            24
#define ETH_RX_IP_FRAME_TYPE_MASK           (1<<ETH_RX_IP_FRAME_TYPE_BIT)
#define ETH_RX_IP_HEADER_OK_BIT             25
#define ETH_RX_IP_HEADER_OK_MASK            (1<<ETH_RX_IP_HEADER_OK_BIT)
#define ETH_RX_L4_UDP_TYPE                  (1<<ETH_RX_L4_TYPE_OFFSET)
#define ETH_RX_L4_OTHER_TYPE                (2<<ETH_RX_L4_TYPE_OFFSET)
#define ETH_RX_L4_TYPE_OFFSET               21
#define ETH_RX_L4_CHECKSUM_OK_BIT           30
#define ETH_RX_L4_CHECKSUM_OK_MASK          (1<<ETH_RX_L4_CHECKSUM_OK_BIT)
#define ETH_RX_L4_CHECKSUM_OFFSET           3
#define ETH_RX_L4_CHECKSUM_MASK             (0xffff<<ETH_RX_L4_CHECKSUM_OFFSET)

typedef struct _switchPriv
{
	int port;
	MV_VOID *halPriv;
	MV_U32 rxqCount;
	MV_U32 txqCount;
	MV_BOOL devInit;
	
	struct timer_list   rx_timer;
	struct timer_list   tx_timer;

	struct napi_struct napi;
	
} switchPriv;
	
typedef struct _SwitchDevConfig
{
	MV_U8                   devNum;
	MV_U8                   queueIdx;
	MV_U8                   qWeight;
}SWITCH_DEV_CONFIG, *SWITCH_DEV_CONFIG_PTR;
	
	
MV_U8	*txPacketData;
MV_U32  txPacketPhyData;  /* TX buffer physical addr ptr */
MV_U32  txMemHandle;	  /* for mvOsIoUncachedMalloc func */
MV_U8	rxPacketData[SWITCH_MTU];
MV_PKT_INFO 	pktInfo;
MV_BUF_INFO 	bufInfo;
	
/****************************************************** 
* functions prototype --                             *
******************************************************/
#ifdef CONFIG_MV_PRESTERA_SWITCH_ACTIVATE
static int mvSwitchLoad( int port, char *name, char *enet_addr );
#endif
static int mvSwitchInit( struct net_device *dev );
static int mvSwitchHalt( struct net_device *dev );
int mvSwitchTx( struct sk_buff *skb , struct net_device *dev );
static int mvSwitchRx( struct net_device *dev, int *quota);
void mv_rx_timer_callback(  unsigned long data, void *dev_id );
MV_BOOL mvSwitchCheckPortStatus(int portNum);
int mv_set_mac_addr( struct net_device *dev, void *addr );

extern MV_32  mvBoardSwitchGpioPinGet(MV_VOID);

/*********************************************************** 
* mv_prestera_initialize --                               *
*   main driver initialization. loading the interfaces.   *
***********************************************************/
int mv_prestera_initialize(void) 
{
#ifndef CONFIG_MV_PRESTERA_SWITCH_ACTIVATE
  return 0;
#else
	int dev,port;
	MV_8 name[IFNAMSIZ+1];
	MV_8 enet_addr[6];
	
	/* SDMA - disable retransmit on resource error */
	mvSwitchReadModWriteReg(0, 0x2800, 0xff00, 0xff00);

	/* Open Window to xBar */
	mvSwitchWriteReg(0, 0x30C, 0xE00);          /* Define window to DRAM
									Target ID 0, attribute 0xE */
	mvSwitchWriteReg(0, 0x310, 0xFFFF0000);     /* Set window size */
	mvSwitchReadModWriteReg(0, 0x34C, 0x1,0x0); /* Enable window 0 */

	if(mvSwitchGetDevicesNum() > 1)
	{
		/* Open PEX Window for second device */	
		mvSwitchWriteReg(1, 0x30C, 0xF);  	    /* Define window to PEX
						       	       Target ID 0xF, attribute 0x0 */
		mvSwitchWriteReg(1, 0x310, 0xFFFF0000);     /* Set window size */
		mvSwitchReadModWriteReg(1, 0x34C, 0x1,0x0); /* Enable window 0 */
	}
	
	for(dev=0; dev < mvSwitchGetDevicesNum(); dev++)
	{
		if(setCpuAsVLANMember(dev, 1 /*VLAN Num*/)!=MV_OK)
		{
			printf( "\nError: (Prestera) Unable to set CPU as VLAN member (for device %d)\n",dev);
			return MV_FAIL;
		}	
	}

	/* load interface(s) */
	for( port=0; port < PRESTERA_PORT_NUM; port++ )
	{
		/* interface name */
		sprintf( name, "port%d", port );
		/* interface MAC addr extract */
		enet_addr[0] = 0;
		enet_addr[1]=enet_addr[2]=enet_addr[3]=enet_addr[4]=enet_addr[5]=0x11;

		if(setCPUAddressInMACTAble(0 /* devNum */, 
					"00:11:11:11:11:11" /* macAddr */,
					1 /* vid */)!=MV_OK)
		{
			printk( "\nError: (Prestera) Unable to teach CPU MAC address\n");
			return -1;
		}
			
		mvSwitchLoad( port, name, enet_addr );
	}
		
//	txPacketData = kmalloc(SWITCH_MTU,GFP_KERNEL);
	txPacketData = (char*)mvOsIoUncachedMalloc(NULL,SWITCH_MTU,&txPacketPhyData,&txMemHandle); 

	if( !txPacketData ) {
		DB( printk( "%s: %s falied to alloc TX buffer (error)\n", __FUNCTION__, name ) );
		return 1;
	}

	return 0;
#endif
}

#ifdef CONFIG_MV_PRESTERA_SWITCH_ACTIVATE	
/*********************************************************** 
* mvSwitchLoad --                                          *
*   load a network interface into uboot network core.     *
*   initialize sw structures e.g. private, rings, etc.    *
***********************************************************/
static int mvSwitchLoad( int port, char *name, char *enet_addr ) 
{
	struct net_device   *dev = NULL;
	int                 ret = 0;
	switchPriv *priv = NULL;
	
	DB( printk( "%s: %s load - ", __FUNCTION__, name ) );
	
	dev = alloc_etherdev(sizeof(switchPriv));
	
	if( !dev ) {
		ret = -ENOMEM;
		goto error;
	}
	
	priv = (switchPriv *)dev->priv;
	if( !priv ) {
		DB( printk( "%s: %s falied to alloc egiga_priv (error)\n", __FUNCTION__, name ) );
		goto error;
	}
	memset( priv, 0, sizeof(switchPriv) );
	
	/* init device methods */
	strcpy( dev->name, name );
	dev->base_addr = 0;
	if( port == 0) dev->irq = mvBoardSwitchGpioPinGet();
	dev->open = mvSwitchInit;
	dev->stop = mvSwitchHalt;
	dev->hard_start_xmit = mvSwitchTx;
	dev->watchdog_timeo = 5*HZ;
	dev->tx_queue_len = PRESTERA_RXQ_LEN;
	dev->poll = &mvSwitchRx;
	dev->priv = priv;
	priv->port = port;
        dev->dev_addr[0]= enet_addr[0];
	dev->dev_addr[1]= enet_addr[1];
	dev->dev_addr[2]= enet_addr[2];
	dev->dev_addr[3]= enet_addr[3];
	dev->dev_addr[4]= enet_addr[4];
	dev->dev_addr[5]= enet_addr[5];
    	dev->set_mac_address = mv_set_mac_addr;

	memset( priv, 0, sizeof(switchPriv) );
	
	dev->mtu = 1500;
	dev->features = NETIF_F_SG | NETIF_F_HW_CSUM;
	
	if(register_netdev(dev)) {
		printk( KERN_ERR "%s: register failed\n", dev->name );
		ret = -ENODEV;
		goto error;
	}
	
	DB( printk( "%s: %s load ok\n", __FUNCTION__, name ) );
	
	return 0;
	
	error:
	printk( "%s: %s load failed\n", __FUNCTION__, name );
	unregister_netdevice(dev);
	if( priv ) kfree( dev->priv );
	if( dev ) kfree( dev );
	return -1;
}
#endif
	
unsigned int switch_init=0;
unsigned int entryId=0;
static irqreturn_t mv_interrupt_handler(int irq , void *dev_id)
{
    	MV_STATUS   rc;
	MV_U32      regAddr, regData;
	struct net_device *dev = dev_id;

	regAddr = 0x280C;
    	rc = mvSwitchReadReg(0,regAddr,&regData);
    	if (rc != MV_OK)
    	{
        	printk("failed to read 0x%X", regAddr);
		return rc;
    	}
	regAddr = 0x2810;
    	rc = mvSwitchReadReg(0,regAddr,&regData);
    	if (rc != MV_OK)
    	{
        	printk("failed to read 0x%X", regAddr);
		return rc;
    	}
	regAddr = 0x30;
    	rc = mvSwitchReadReg(0,regAddr,&regData);
    	if (rc != MV_OK)
    	{
        	printk("failed to read 0x%X", regAddr);
		return rc;
    	}
//mvPresteraReadPortMibCounters(0);
	mvSwitchRx( __dev_get_by_name(dev_net(dev),"port0") , NULL);
	
	return IRQ_HANDLED;
}
	
	
static int mvSwitchInit( struct net_device *dev)
{
	switchPriv *priv = (switchPriv *)netdev_priv(dev);

	DB( printk( "%s: %s init - ", __FUNCTION__, dev->name ) );

	/* egiga not ready */
	DB( printk ("mvBoardPhyAddrGet()=0x%x , priv->port =0x%x\n",mvBoardPhyAddrGet(priv->port),priv->port));
	
	/* in default link is down */
	netif_carrier_off( dev );
	
	/* Stop the TX queue - it will be enabled upon PHY status change after link-up interrupt/timer */
	netif_stop_queue( dev );
	
	/* enable polling on the port, must be used after netif_poll_disable */
	napi_enable(&priv->napi);
	
	/* Check Link */
	if( mvSwitchCheckPortStatus(priv->port) == MV_FALSE ) {
		printk( "%s no link\n", dev->name );
		return 0;
	}
	
	DB( printk( "link up\n" ) );
	
	if (switch_init==0)
	{
		/* init the hal -- create internal port control structure and descriptor rings */
		if(gtInitSdmaNetIfDev(0,20,
				PRESTERA_Q_NUM*PRESTERA_RXQ_LEN,
				PRESTERA_Q_NUM*PRESTERA_TXQ_LEN,
				SWITCH_MTU)!=MV_OK)
		{
			printk( "Error: (Prestera) Unable to initialize SDMA\n");
			goto error;
		}	

		priv->devInit = MV_TRUE;
		switch_init = 1;
	}
	
	/* connect to port interrupt line */
	if(dev->name[4] == '0')
	{

		dev->irq  = 113;
		printk("IRQ:\n");
		if( request_irq( dev->irq, mv_interrupt_handler,
			(IRQF_DISABLED | IRQF_SAMPLE_RANDOM) , dev->name, dev ) ) {
			printk( KERN_ERR "IRQ: cannot assign irq%d to %s port%d\n", dev->irq, dev->name, priv->port );
			dev->irq = 0;
			goto error;
		}
	}
	
	/*Do Linux Stuff for full enable*/
    	napi_enable(&priv->napi);
        netif_carrier_on( dev );
        netif_wake_queue( dev );            

   	if(!netif_running(dev)) 
	{
		printk("Device not running\n");
		goto error;
	}
	
	DB( printk( "%s: %s complete ok\n", __FUNCTION__, dev->name ) );
	return 0;

	error:
	if( priv->devInit )
		mvSwitchHalt( dev );
	printk( "%s: %s failed\n", __FUNCTION__, dev->name );
	return 1;
}
	
static int mvSwitchHalt( struct net_device *dev )
{
	switchPriv *priv = (switchPriv *)netdev_priv(dev);
	
	DB( printk( "%s: %s halt - ", __FUNCTION__, dev->name ) );
	
	if( priv->devInit == MV_TRUE ) {
		priv->devInit = MV_FALSE;
	}
	
	/* switch_init = 0; */
	DB( printk( "%s: %s complete\n", __FUNCTION__, dev->name ) );
	return 0;
}
	
int mvSwitchTx( struct sk_buff *skb , struct net_device *dev )
{
	switchPriv *priv = (switchPriv *)netdev_priv(dev);
	MV_PKT_DESC 	packetDesc;
	MV_STATUS 	status;
	
	/* if no link exist */
	if(!switch_init) return 0;

	/* build the packet */
	memset( &packetDesc, 0, sizeof(MV_PKT_DESC) );
	packetDesc.pcktData[0] = txPacketData;
	memset( txPacketData, 0, SWITCH_MTU );
	packetDesc.pcktData[0] = txPacketData;		/* Packet's buffer virtual address */
	packetDesc.pcktPhyData[0] = (MV_U8 *)(txPacketPhyData);	/* Packet's buffer physical address */

	packetDesc.pcktDataLen[0] = skb->len;

	netif_stop_queue( dev );

	status = mvSwitchBuildPacket(0, /*IgorP*/
				priv->port /* port num */,
				entryId /* entryId */ ,
				1 /* appendCrc */ ,
				1 /* pcktsNum */ ,
				0 /* gap */ ,
				(MV_U8*)skb->data /* pcktData */,
				packetDesc.pcktDataLen[0] /* pcktSize*/,
				&packetDesc);
	
	if( status != MV_OK ) {
		if( status == MV_NO_RESOURCE ) 
			DB( printk( "can't build packet. out of memory (error)\n" ) ) ;
		else 
			DB( printk( "unrecognize status (error) mvSwitchBuildPacket\n" ) );
		goto error;
	} 
	else {
		DB( printk( "packet build ok\n" ) );
	}

	/* send the packet */
	if(mvSwitchTxStart(&packetDesc)!=MV_OK)
	{
		DB( printk( "Unable to Start Tx\n"));
		return MV_FAIL;
	}

	if(netif_queue_stopped(dev))
		netif_wake_queue( dev );

	priv->txqCount++;
	
	DB( printk( "%s: %s complete ok\n", __FUNCTION__, dev->name ) );
	
	return 0;
	
	error:
	DB( printk( "%s: %s failed\n", __FUNCTION__, dev->name ) );
	return 1;
}
	
	
static int mvSwitchRx( struct net_device *dev , int *quota)
{
	MV_STATUS   	status = GT_REDO;
	MV_PKT_INFO 	pktInfo;
	MV_BUF_INFO 	bufInfo;
	MV_U8		queueIdx = 0;
	MV_U8		devNum = 0;
	MV_U32		buf_size;
        struct sk_buff  *skb;
    	MV_U32	    	rx_status; 
	switchPriv *priv = (switchPriv *)netdev_priv(dev);
	
	/* if no link exist */
	if(!switch_init) return 0;
	
	memset( &pktInfo, 0, sizeof(MV_PKT_INFO) );
	pktInfo.pFrags = &bufInfo; 
	
	while(status == GT_REDO)
	{
		memset( &bufInfo, 0, sizeof(MV_BUF_INFO) );

		status = mvSwitchRxStart(devNum, queueIdx, &pktInfo);
		priv->rxqCount--;
		
		if( status==MV_OK){
			/* no more rx packets ready */
			return MV_OK;
		}		
		else if(status!=GT_REDO) {
		DB( printk( "Rx error\n") );
			goto error;
		}
		else {
			/* good rx - push the packet up (skip on two first empty bytes) */
		
			/*Allocate skb*/
        		buf_size = dev->mtu + 20 +
                        	CPU_D_CACHE_LINE_SIZE /* 32(extra for cache prefetch) */ + 
                        	8 /* +8 to align on 8B */;
        		skb = dev_alloc_skb( buf_size ); 

			/* remove 8 bytes of DSA Tag , set MAC SA & DA */
			memcpy( skb->data, (void*)pktInfo.pFrags->bufVirtPtr , 12 );
			/* Set rest of packet (without 8 bytes */
			memcpy( skb->data + 12, 
				(void*)pktInfo.pFrags->bufVirtPtr + 20 , 
				pktInfo.pFrags->bufSize-16 );

			memset(skb->data + pktInfo.pFrags->bufSize - 16, 0,12);

			skb_put(skb, pktInfo.pFrags->bufSize - /*ETH_MV_HEADER_SIZE*/2 - 10);
			skb->dev = dev;
        		skb->protocol = eth_type_trans(skb, dev);
			skb->ip_summed = CHECKSUM_COMPLETE;

			rx_status = pktInfo.status;
	    		if (((pktInfo.pFrags->dataSize + 4) > ETH_CSUM_MIN_BYTE_COUNT)  && 
				(rx_status & ETH_RX_IP_FRAME_TYPE_MASK) && 
				(rx_status & ETH_RX_IP_HEADER_OK_MASK)) {  
					if (!(pktInfo.fragIP)		&&
			   		(!(rx_status & ETH_RX_L4_OTHER_TYPE)) &&
			   		(rx_status & ETH_RX_L4_CHECKSUM_OK_MASK)) { 
						skb->csum = 0;
						skb->ip_summed = CHECKSUM_UNNECESSARY;	
				}
				else if (pktInfo.fragIP && (rx_status & ETH_RX_L4_UDP_TYPE)) {	/* TBD: UDP only */
				skb->csum = ntohl(0xFFFF ^ ((rx_status & ETH_RX_L4_CHECKSUM_MASK) >> ETH_RX_L4_CHECKSUM_OFFSET));
				skb->ip_summed = CHECKSUM_COMPLETE;
				}
	    		}

        		status = netif_receive_skb(skb);
			dev->stats.rx_packets++;
	
			gtFreeRxBuf(&pktInfo.pFrags->bufVirtPtr,1,
					devNum,queueIdx);
		}
	}
	
	DB( printk( "%s: %s complete ok\n", __FUNCTION__, dev->name ) );
	return 0;
	
	error:
	DB( printk( "%s: %s failed\n", __FUNCTION__, dev->name ) );
	return 1;
}
	
MV_BOOL mvSwitchCheckPortStatus(int portNum)
{
	unsigned long regVal = 0;
	if( mvSwitchReadReg(
			0, 
			PRESTERA_PORT_STATUS_REG + portNum * 0x400, 
			(MV_U32*)&regVal) != MV_OK)
	{
		return MV_FALSE;
	}
	return (regVal & 0x1) ? MV_TRUE : MV_FALSE;
}
	
void mv_rx_timer_callback(  unsigned long data, void *dev_id )
{
	struct net_device *dev = dev_id;
	mvSwitchRx( __dev_get_by_name(dev_net(dev),"port0") , NULL);
	
	return;
}
	
static int mv_set_mac_addr_internals(struct net_device *dev, void *addr )
{
    u8* mac = &(((u8*)addr)[2]);  /* skip on first 2B (ether HW addr type) */
    int i;

    /* set new addr in hw */
    if( setCPUAddressInMACTAble( 0, mac, 1) != MV_OK ) {
        printk( KERN_ERR "%s: ethSetMacAddr failed\n", dev->name );
    return -1;
    }

    /* set addr in the device */ 
    for( i = 0; i < 6; i++ )
        dev->dev_addr[i] = mac[i];

    printk( KERN_NOTICE "%s: mac address changed\n", dev->name );

    return 0;
}

int mv_set_mac_addr( struct net_device *dev, void *addr )
{
   if(!netif_running(dev)) {
        if(mv_set_mac_addr_internals(dev, addr) == -1)
            goto error;
        return 0;
    }

    if( mvSwitchHalt( dev )) {
        printk( KERN_ERR "%s: stop interface failed\n", dev->name );
        goto error;
    }

    if(mv_set_mac_addr_internals(dev, addr) == -1)
        goto error;

    if(mvSwitchInit( dev )) {
        printk( KERN_ERR "%s: start interface failed\n", dev->name );
    goto error;
    } 

    return 0;

 error:
    printk( "%s: set mac addr failed\n", dev->name );
    return -1;
}
/*******************************************************************************
* mvSwitchDecodingInit
*
* DESCRIPTION:
*       Initialization of decoding windows for memory access
*       Purpose of this function is to open window to xBar for all enabled SDRAM ChipSelects
*
* INPUTS:
*       baseAddr  - Device Base address
*
* OUTPUTS:
*       None.
*
* RETURNS:
*       MV_OK      - on success
*       MV_FAIL      - on error
*
* COMMENTS:
*
*******************************************************************************/
MV_STATUS __init mvSwitchDecodingInit(void) 
{
    MV_U32 baseReg=0,sizeReg=0;
    MV_U32 baseToReg=0 , sizeToReg=0;
    MV_U32 target, decodingIndex = 0;
    MV_U32 initBare = 0x3F; /* init value of BaseAddressRegisterEnable - 0x3F - all off */
    MV_U32 baseAddr = 0;
	
    /* Check all SDRAM chip selects (0 to 3) and process enabled CS 
            We have two indexes - "target" for RAM CS and "decodingIndex" for decoding window number .*/
    for (target = 0; target < 4; target ++)
    {
        /* read size register */
        sizeReg = MV_REG_READ((0x1504 + (target * 8))/*SDRAM_SIZE_REG(target)*/);

        /* check if window enabled */
        if (!(sizeReg & BIT0/*SCSR_WIN_EN*/))
            continue;

        baseReg = MV_REG_READ((0x1500 + (target * 8))/*SDRAM_BASE_ADDR_REG(target)*/); /* read base register */

        baseToReg = PRESTERA_AD_BASE_ADDR_BITS_SET(baseReg, decodingTargetAttribures[target], PRESTERA_AD_DRAM_TARGET_ID);

        mvSwitchWriteReg(baseAddr, PRESTERA_AD_BASE_ADDR(decodingIndex), baseToReg); /* Define window */

        sizeToReg = ((sizeReg & /*SCSR_SIZE_MASK*/(0xff << 24)) | 0xFF0000); /* Prestera size decoding fielld is 16 bit [32-16] and CPU size decoding  field is 8 bit [32-24] */

        mvSwitchWriteReg(baseAddr, PRESTERA_AD_SIZE(decodingIndex), sizeToReg); /* Set window size */		
    
        PRESTERA_AD_BARE_BITS_SET(initBare, decodingIndex); /* set corresponding bit to 0 - enable windows */

        decodingIndex++;
    }
	
    mvSwitchWriteReg(baseAddr, PRESTERA_AD_BARE, initBare);/* Enable relevant windows */
 
    printk("Switch decoding windows init is done.\n");

    return MV_OK;
}

	
#endif /* #if defined (MV_PRESTERA_SWITCH) */
