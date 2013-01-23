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
#include <linux/ip.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/rtnetlink.h>
#include <linux/notifier.h>
#include <linux/etherdevice.h>
#include <linux/if_vlan.h>
#include <linux/if_bridge.h>
	
#if defined (MV_PRESTERA_SWITCH)
#include "mvOs.h"
#include "hwIf/mvHwIf.h"
#include "util/mvUtils.h"
#include "util/mvNetTransmit.h"
#include "util/mvNetReceive.h"
#include "common/macros/mvCommonDefs.h"
#include "mvPresteraRegs.h"
#include "vlan/mvVlan.h"
#include "sdma/mvBridge.h"

#include "eth/mvEth.h"
#include "eth/nfp/mvNfp.h"
#include "../mv_drivers_lsp/mv_network/nfp_mgr/mv_nfp_mgr.h"
	
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

// MII definitions
MV_U8 rxBuffPool[RX_BUFFER_DEFAULT_SIZE*RX_BUFFERS_ARRAY_SIZE];

/* MG - Address Decoding registers */
#define PRESTERA_AD_BASE_ADDR(index) (0x30C + (index*8))
#define PRESTERA_AD_SIZE(index)      (0x310 + (index*8))
#define PRESTERA_AD_BARE              0x34C                /* Base address enable */

#define PRESTERA_AD_BARE_BITS_SET(value, index)    value &= ~(1<<index)
#define PRESTERA_AD_BASE_ADDR_BITS_SET(base, attr, target) ((base & 0xFFFF0000) | (attr << 8) | target)
#define PRESTERA_AD_DRAM_TARGET_ID 0

static MV_U8 decodingTargetAttribures[] = {0xE, 0xD, 0xB, 0x7};

#define ETH_CSUM_MIN_BYTE_COUNT             72
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

#define PRESTERA_FP_FOUND               0
#define PRESTERA_FP_NOT_FOUND           1

typedef struct _switchPriv
{
        int port;
        int ifindex;
	MV_VOID *halPriv;
	MV_U32 rxqCount;
	MV_U32 txqCount;
	MV_BOOL devInit;

        MV_FP_STATS pFpStats;
	
	struct timer_list   rx_timer;
	struct timer_list   tx_timer;
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
static int mvSwitchLoad( int port, char *name, char *enet_addr );
static int mvSwitchInit( struct net_device *dev );
static int mvSwitchHalt( struct net_device *dev );
static int mvSwitchRx( struct net_device *dev );
#ifndef MV_PRESTERA_USE_MII_DRIVER
static int mvSwitchTx( struct sk_buff *skb , struct net_device *dev );
#else
static int mvSwitchSendMii( struct sk_buff *skb , struct net_device *dev );
#endif

static int mvPortLoad( int port, char *name, char *enet_addr );
static int mvPortInit( struct net_device *dev );
static int mvPortHalt( struct net_device *dev );
static int mvPortTx( struct sk_buff *skb, struct net_device *dev );
static int mvPortForward( struct sk_buff *skb, struct net_device *dev );

MV_BOOL mvSwitchCheckPortStatus(int portNum);
static int mv_set_mac_addr( struct net_device *dev, void *addr );
int mv_netif_rx_nfp(struct sk_buff *skb);

/*********************************************************** 
* mv_prestera_initialize --                               *
*   main driver initialization. loading the interfaces.   *
***********************************************************/
int mv_prestera_initialize() 
{
	int dev,port;
	MV_8 macaddr[18];
        MV_8 name[IFNAMSIZ+1];
	MV_8 enet_addr[6];
        int NumberOfDevices, portsNumber;
        
	/* SDMA - disable retransmit on resource error */
	mvSwitchReadModWriteReg(0, 0x2800, 0xff00, 0xff00);

	/* Open Window to xBar */
	mvSwitchWriteReg(0, 0x30C, 0xE00);          /* Define window to DRAM
									Target ID 0, attribute 0xE */
	mvSwitchWriteReg(0, 0x310, 0xFFFF0000);     /* Set window size */
	mvSwitchReadModWriteReg(0, 0x34C, 0x1,0x0); /* Enable window 0 */

        NumberOfDevices = mvSwitchGetDevicesNum(); 

	if(NumberOfDevices > 1)
	{
		/* Open PEX Window for second device */	
		mvSwitchWriteReg(1, 0x30C, 0x80F);  	    /* Define window to PEX
						       	       Target ID 0xF, attribute 0x0 */
		mvSwitchWriteReg(1, 0x310, 0xFFFF0000);     /* Set window size */
		mvSwitchReadModWriteReg(1, 0x34C, 0x1,0x0); /* Enable window 0 */

                for(dev=0; dev < mvSwitchGetDevicesNum(); dev++)
                {
                        MV_U32 mask = dev * (BIT_24|BIT_25) | (1 - dev) * (BIT_26|BIT_27);
                        mvSwitchReadModWriteReg(dev, CASCADE_AND_HEADER_CONFIG_REG, mask,mask);

                        /* Enable the device if not enabled already */
                        mvSwitchReadModWriteReg(dev, 0x58, BIT_0, BIT_0);       
                }
	}
           
	for(dev=0; dev < NumberOfDevices; dev++)
	{
		if(setCpuAsVLANMember(dev, 1 /*VLAN Num*/)!=MV_OK)
		{
			printf("\nError: (Prestera) Unable to set CPU as VLAN member(for device %d)\n",
                                dev);
			return MV_FAIL;
		}	
	}

        portsNumber = NumberOfDevices * PRESTERA_PORT_NUM;

	/* load SDMA interface */
	for( dev=0; dev < NumberOfDevices; dev++ )
	{
		/* interface name */
		sprintf( name, "port%d", dev );

		mvSwitchLoad( dev, name, enet_addr );
	} 

        /* load port logical interfaces */
        for( port=0; port < portsNumber; port++ )
        {
                /* interface name */
                sprintf( name, "p%d", port );
                /* interface MAC addr extract */
                enet_addr[0] = 0;
                enet_addr[1]=enet_addr[2]=enet_addr[3]=enet_addr[4] = (port/PRESTERA_PORT_NUM);
                enet_addr[5]=port+1;

                sprintf(macaddr,"%02x:%02x:%02x:%02x:%02x:%02x", 
                        enet_addr[0],enet_addr[1],enet_addr[2],
                        enet_addr[3],enet_addr[4],enet_addr[5]);
                if(setCPUAddressInMACTAble(port/PRESTERA_PORT_NUM /*devNum*/, macaddr /*macAddr*/,1 /*vid*/) != MV_OK)
                {
                        printk( "\nError: (Prestera) Unable to teach CPU MAC address\n");
                        return -1;
                }

                mvPortLoad( port, name, enet_addr );
        } 
		
#ifdef MV_PRESTERA_USE_MII_DRIVER
        txPacketData = (char*)mvOsIoCachedMalloc(NULL,SWITCH_MTU,&txPacketPhyData,&txMemHandle);  
#else
        txPacketData = (char*)mvOsIoUncachedMalloc(NULL,SWITCH_MTU,&txPacketPhyData,&txMemHandle);  
#endif

	if( !txPacketData ) {
		DB( printk( "%s: %s falied to alloc TX buffer (error)\n", __FUNCTION__, name ) );
		return 1;
	}
	return 0;
}
	
/**************************************************************** 
* mvSwitchLoad --                                               *
*   load a SDMA network interface into Linux network core.      *
*   initialize sw structures e.g. private, rings, etc.          *
*****************************************************************/
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
        dev->irq = mvBoardSwitchGpioPinGet(port);
	dev->open = mvSwitchInit;
	dev->stop = mvSwitchHalt;
#ifndef MV_PRESTERA_USE_MII_DRIVER
	dev->hard_start_xmit = mvSwitchTx;
#else
        dev->hard_start_xmit = mvSwitchSendMii;
#endif
	dev->watchdog_timeo = 5*HZ;
	dev->tx_queue_len = PRESTERA_RXQ_LEN;
	dev->poll = &mvSwitchRx;
	dev->priv = priv;
	priv->port = port;
        memcpy(dev->dev_addr, enet_addr,6);
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

/**************************************************************** 
* mvPortLoad --                                                 *
*   load a port network interface into Linux network core.      *
*****************************************************************/
static int mvPortLoad( int port, char *name, char *enet_addr ) 
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
        dev->open = mvPortInit;
        dev->stop = mvPortHalt;
        dev->hard_start_xmit = mvPortTx;
        dev->watchdog_timeo = 5*HZ;
        dev->tx_queue_len = PRESTERA_RXQ_LEN;
        dev->poll = &mvPortForward;
        dev->priv = priv;
        priv->port = port;
        memcpy(dev->dev_addr, enet_addr, 6);
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
	
unsigned int entryId=0;
#ifndef MV_PRESTERA_USE_MII_DRIVER
/**************************************************************** 
* mv_interrupt_handler --                                       *
*   Interrupt handler starting the RX process for SDMA mode     *
*****************************************************************/
static irqreturn_t mv_interrupt_handler(int irq , void *dev_id)
{
    	MV_STATUS   rc;
	MV_U32      regAddr, regData;
        struct net_device *newDev;

        if( irq == mvBoardSwitchGpioPinGet(0) )
        {
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
	        mvSwitchRx( __dev_get_by_name("port0") );
        }
        else
        {
                regAddr = 0x280C;
                rc = mvSwitchReadReg(1,regAddr,&regData);
                if (rc != MV_OK)
                {
                        printk("failed to read 0x%X", regAddr);
                        return rc;
                }
                regAddr = 0x2810;
                rc = mvSwitchReadReg(1,regAddr,&regData);
                if (rc != MV_OK)
                {
                        printk("failed to read 0x%X", regAddr);
                        return rc;
                }
                regAddr = 0x30;
                rc = mvSwitchReadReg(1,regAddr,&regData);
                if (rc != MV_OK)
                {
                        printk("failed to read 0x%X", regAddr);
                        return rc;
                }

                newDev = __dev_get_by_name("port1");
                newDev->poll(newDev, 0);
        }

	return IRQ_HANDLED;
}
#else	
/**************************************************************** 
* mv_interrupt_frame_receive --                              *
*   Interrupt handler callback the RX process for MII  mode     *
*****************************************************************/
static int mv_interrupt_frame_receive
(    
        IN MV_U8     * segmentsList[],
        IN MV_U32      segmentLen[],
        IN MV_U32      segmentNum,
        IN MV_U32      rxQueueNum
)
{
        struct net_device *dev;
        switchPriv*     priv;
        MV_STATUS       status = MV_PRESTERA_REDO;
        MV_U8           queueIdx = 0;
        MV_U32          buf_size;
        struct sk_buff  *skb;
        MV_U32          hwDsa[2]; 
        MV_8 name[IFNAMSIZ+1];
        MV_U32 port, devNum = 0;
        MV_U8     *bufPtr = segmentsList[0] + ETH_MV_HEADER_SIZE;

        sprintf( name, "port%d", devNum );
        dev = __dev_get_by_name(name);
        priv = dev->priv;

        /*Copmute port number from DSA tag*/
        hwDsa[1] = bufPtr[16] | (bufPtr[15]<<8) | 
                        (bufPtr[14]<<16) | (bufPtr[13]<<24);
        port = (hwDsa[1] >> 27) + devNum * PRESTERA_PORT_NUM;

        /*Allocate skb*/
        buf_size = dev->mtu + PRESTERA_MAC_HEADER_SIZE + PRESTERA_DSA_TAG_SIZE +
                   CPU_D_CACHE_LINE_SIZE /* 32(extra for cache prefetch) */ + 
                   8 /* +8 to align on 8B */;
        skb = dev_alloc_skb( buf_size ); 

        /* remove 8 bytes of DSA Tag , set MAC SA & DA */
        memcpy( skb->data, (void*)bufPtr , PRESTERA_MAC_HEADER_SIZE );
        /* Set rest of packet (without 8 bytes */
        memcpy( skb->data + PRESTERA_MAC_HEADER_SIZE, 
                (void*)bufPtr + PRESTERA_MAC_HEADER_SIZE + PRESTERA_DSA_TAG_SIZE, 
                segmentLen[0] - PRESTERA_MAC_HEADER_SIZE - PRESTERA_DSA_TAG_SIZE );

        skb_put(skb, segmentLen[0] - PRESTERA_MAC_HEADER_SIZE);

        //Find port device by source port number
        sprintf( name, "p%d", port );
        skb->dev = __dev_get_by_name(name);

        skb->protocol = eth_type_trans(skb, skb->dev);
        skb->ip_summed = CHECKSUM_PARTIAL;
        
        /*Length of the packet*/
        skb->len = segmentLen[0] - PRESTERA_MAC_HEADER_SIZE - 
                PRESTERA_DSA_TAG_SIZE - ETH_MV_HEADER_SIZE;

        //Send the packet to port device
        status = mvPortForward(skb,skb->dev);
        dev->stats.rx_packets++;
        
        bspEthRxPacketFree(&segmentsList[0], 1, devNum, queueIdx);
        return 0;
}
#endif	

/**************************************************************** 
* mvSwitchInit --                                               *
*   load a SDMA network interface into Linux network core.      *
*   initialize sw structures e.g. private, rings, etc.          *
*   connect to interrupt, enable traffic                        *
*****************************************************************/
static int mvSwitchInit( struct net_device *dev)
{
	switchPriv *priv = dev->priv;
        MV_U32  devNum = dev->name[4] - '0';

	DB( printk( "%s: %s init - ", __FUNCTION__, dev->name ) );

	/* egiga not ready */
	DB( printk ("mvBoardPhyAddrGet()=0x%x , priv->port =0x%x\n",
                     mvBoardPhyAddrGet(priv->port),priv->port));
	
	/* in default link is down */
	netif_carrier_off( dev );
	
/* Stop the TX queue - it will be enabled upon PHY status change after link-up interrupt/timer */
	netif_stop_queue( dev );
	
	/* enable polling on the port, must be used after netif_poll_disable */
	netif_poll_enable(dev);
	
#ifdef MV_PRESTERA_USE_MII_DRIVER
        { /*MII port init*/
                MV_U32 qPercentage[] = {100,0,0,0,0,0,0,0};
                int reqBufNum = RX_BUFFERS_ARRAY_SIZE;

                mvSwitchCpuPortConfig(PRESTERA_DEFAULT_DEV);

                /******         Controller required initializations     ******/
                bspEthInit(EGIGA_CPU_PORT);
        
                if (bspEthPortRxInit(RX_BUFFER_DEFAULT_SIZE * RX_BUFFERS_ARRAY_SIZE,
                                rxBuffPool,
                                RX_BUFFER_DEFAULT_SIZE,
                                &reqBufNum,
                                0              /*headerOffset*/,
                                1              /*rxQNum*/,
                                &qPercentage) != MV_OK)
                {
                        printf("bspEthPortRxInit failed\n");
                        return 1;
                }

                /* Connect Egiga to CPU Port 
                * This is only controller configuration
                * Switch configuration should be done separately */
                miiInfCpuPortConfig();
        
                if (bspEthPortTxInit(RX_BUFFERS_ARRAY_SIZE)!=MV_OK)
                {
                        printf("bspEthPortTxInit failed\n");
                        return 1;
                }
        
                if (bspEthPortEnable()!=MV_OK)
                {
                        printf("bspEthPortEnable failed\n");
                        return 1;
                }

                {/*Learn MAC for CPU port*/
                        ETH_PORT_CTRL dummy_port_handle;
                        MV_8 enet_addr[6];

                        dummy_port_handle.portNo = EGIGA_CPU_PORT;
                        mvEthMacAddrSet( &dummy_port_handle, enet_addr, ETH_DEF_RXQ);
                }

                gtEthernetRxCallbackBind(mv_interrupt_frame_receive);
        }
#else
/* init the hal -- create internal port control structure and descriptor rings */
        if(mvInitSdmaNetIfDev(devNum, 20,
                        PRESTERA_Q_NUM * PRESTERA_RXQ_LEN,
                        PRESTERA_Q_NUM * PRESTERA_TXQ_LEN,
                        SWITCH_MTU) !=MV_OK)
        {
                printk( "Error: (Prestera) Unable to initialize SDMA\n");
                goto error;
        }       

	/* connect to port interrupt line */
	if( request_irq( dev->irq, mv_interrupt_handler,
		(IRQF_DISABLED | IRQF_SAMPLE_RANDOM) , dev->name, dev ) ) {
		printk( KERN_ERR "IRQ: cannot assign irq%d to %s port%d\n", 
                        dev->irq, dev->name, priv->port );
		dev->irq = 0;
		goto error;
	}
#endif

        priv->devInit = MV_TRUE;
	
	/*Do Linux Stuff for full enable*/
    	netif_poll_enable(dev);
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

/**************************************************************** 
* mvPortInit --                                                 *
*   connect to interrupt, enable traffic on physycal port device*
*****************************************************************/
static int mvPortInit( struct net_device *dev)
{
        switchPriv *priv = dev->priv;

        DB( printk( "%s: %s init - ", __FUNCTION__, dev->name ) );

        /* in default link is down */
        netif_carrier_off( dev );
        
/* Stop the TX queue - it will be enabled upon PHY status change after link-up interrupt/timer */
        netif_stop_queue( dev );
        
        /* enable polling on the port, must be used after netif_poll_disable */
        netif_poll_enable(dev);

        priv->port = 0;
        if(dev->name[2] == '\0')
                priv->port = dev->name[1]-'0';
        else 
                priv->port += (dev->name[1]-'0')*10 + (dev->name[2]-'0');

        {//Register the device in Fast Path
                int rc = fp_mgr_if_register(dev->ifindex,MV_FP_IF_EXT,dev);
                if (rc) 
                        printk(KERN_ERR"%s: nfp register interface failed, index = %d rc=%d\n",
                                dev->name,dev->ifindex, rc);
        }

        /*Do Linux Stuff for full enable*/
        netif_poll_enable(dev);
        netif_carrier_on( dev );
        netif_wake_queue( dev );            

        if(!netif_running(dev)) 
        {
                printk("Device not running\n");
                printk( "%s: %s failed\n", __FUNCTION__, dev->name );
                return 1;
        }
        
        DB( printk( "%s: %s complete ok\n", __FUNCTION__, dev->name ) );
        return 0;
}
	
/**************************************************************** 
* mvSwitchHalt --                                               *
*   Stop SDMA port device                                       *
*****************************************************************/
static int mvSwitchHalt( struct net_device *dev )
{
	switchPriv *priv = dev->priv;
	
	DB( printk( "%s: %s halt - ", __FUNCTION__, dev->name ) );
	
	if( priv->devInit == MV_TRUE ) {
		priv->devInit = MV_FALSE;
	}
	
	DB( printk( "%s: %s complete\n", __FUNCTION__, dev->name ) );
	return 0;
}

/**************************************************************** 
* mvPortHalt --                                                 *
*   Stop physycal port device                                   *
*****************************************************************/
static int mvPortHalt( struct net_device *dev )
{
        fp_mgr_if_unregister(dev->ifindex);

        DB( printk( "%s: %s complete(Do nothing)\n", __FUNCTION__, dev->name ) );
        return 0;
}

#ifndef MV_PRESTERA_USE_MII_DRIVER
/**************************************************************** 
* mvSwitchTx --                                                 *
*   Perform SDMA TX                                             *
*****************************************************************/
static int mvSwitchTx( struct sk_buff *skb , struct net_device *dev )
{
	switchPriv 	*priv = dev->priv;
	MV_PKT_DESC 	packetDesc;
	MV_STATUS 	status;
        MV_U32  devNum = dev->name[4] - '0';

	/* build the packet */
	memset( &packetDesc, 0, sizeof(MV_PKT_DESC) );
	packetDesc.pcktData[0] = txPacketData;
	memset( txPacketData, 0, SWITCH_MTU );
	packetDesc.pcktData[0] = txPacketData;		/* Packet's buffer virtual address */
	packetDesc.pcktPhyData[0] = txPacketPhyData;	/* Packet's buffer physical address */

	packetDesc.pcktDataLen[0] = skb->len;

	netif_stop_queue( dev );

	status = mvSwitchBuildPacket(devNum,
				(priv->port)%24 /* port num */,
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

	dev->stats.tx_packets++;

	DB( printk( "%s: %s complete ok\n", __FUNCTION__, dev->name ) );

	return 0;
	
	error:
	DB( printk( "%s: %s failed\n", __FUNCTION__, dev->name ) );
	return 1;
}
#else
/**************************************************************** 
* mvSwitchTx --                                                 *
*   Perform MII  TX                                             *
*****************************************************************/
static int mvSwitchSendMii( struct sk_buff *skb , struct net_device *dev )
{
        switchPriv      *priv = dev->priv;
        MV_PKT_DESC     packetDesc;
        MV_STATUS       status;
        MV_U32  devNum = dev->name[4] - '0';

        /* build the packet */
        memset( &packetDesc, 0, sizeof(MV_PKT_DESC) );
        packetDesc.pcktData[0] = txPacketData;
        memset( txPacketData, 0, SWITCH_MTU );
        packetDesc.pcktData[0] = txPacketData;          /* Packet's buffer virtual address */
        packetDesc.pcktPhyData[0] = txPacketPhyData;    /* Packet's buffer physical address */

        packetDesc.pcktDataLen[0] = skb->len;

        netif_stop_queue( dev );

        DB(mvOsPrintf("ENTER %s: %s\n", __func__, dev->name));

        status = mvSwitchBuildPacket(devNum,
                                (priv->port)%24 /* port num */,
                                entryId /* entryId */ ,
                                1 /* appendCrc */ ,
                                1 /* pcktsNum */ ,
                                0 /* gap */ ,
                                (MV_U8*)skb->data /* pcktData */,
                                packetDesc.pcktDataLen[0] /* pcktSize*/,
                                &packetDesc);
        
        if( status != MV_OK ) {
                if( status == MV_NO_RESOURCE ) 
                        printk( "can't build packet. out of memory (error)\n" );
                else 
                        printk( "unrecognize status (error) mvSwitchBuildPacket\n");
                goto error;
        } 
        else {
                DB( printk( "packet build ok\n" ) );
        }

        /* send the packet */
        status = bspEthPortTx(&packetDesc.pcktData[0], &packetDesc.pcktDataLen[0], 1);

        if(netif_queue_stopped(dev))
                netif_wake_queue( dev );

        dev->stats.tx_packets++;

        DB( printk( "%s: %s complete ok\n", __FUNCTION__, dev->name ) );
        return 0;
        
        error:
        DB( printk( "%s: %s failed\n", __FUNCTION__, dev->name ) );
        return 1;
}
#endif

/**************************************************************** 
* mvPortTx --                                                   *
*   Physycal port TX, extract port num and call SDMA TX         *
*****************************************************************/
static int mvPortTx( struct sk_buff *skb , struct net_device *dev )
{
        switchPriv      *priv;
        struct net_device *newDev;
        int port=0;
        MV_8 name[IFNAMSIZ+1];
        
        //Extract port number from device name
        if(dev->name[2] == '\0')port = dev->name[1]-'0';
        else port += (dev->name[1]-'0')*10 + (dev->name[2]-'0');

        sprintf( name, "port%d", port/PRESTERA_PORT_NUM );
        newDev = __dev_get_by_name(name);

        priv = newDev->priv;
        priv->port = port;
        newDev->hard_start_xmit(skb, newDev);

        dev->stats.tx_packets++;

        DB( printk( "%s: %s complete ok\n", __FUNCTION__, dev->name ) );
        return 0;
}
	
/************************************************************************
* mvSwitchRx --                                                         *
*   SDMA RX - exstract the packet from device and route to port device  *
*************************************************************************/
static int mvSwitchRx( struct net_device *dev )
{
	switchPriv*  	priv = dev->priv;
	MV_STATUS   	status = MV_PRESTERA_REDO;
	MV_PKT_INFO 	pktInfo;
	MV_BUF_INFO 	bufInfo;
	MV_U8		queueIdx = 0;
	MV_U32		buf_size;
        struct sk_buff  *skb;
    	MV_U32	    	rx_status, hwDsa[2]; 
        MV_8 name[IFNAMSIZ+1];
        MV_U32 port, devNum = dev->name[4] - '0';
	
	memset( &pktInfo, 0, sizeof(MV_PKT_INFO) );
	pktInfo.pFrags = &bufInfo; 
	
	while(status == MV_PRESTERA_REDO)
	{
		memset( &bufInfo, 0, sizeof(MV_BUF_INFO) );

		status = mvSwitchRxStart(devNum, queueIdx, &pktInfo);
		priv->rxqCount--;
		
		if( status==MV_OK){
			/* no more rx packets ready */
			return MV_OK;
		}		
		else if(status!=MV_PRESTERA_REDO) {
		DB( printk( "Rx error\n") );
			goto error;
		}
		else {
/* good rx - push the packet up (skip on two first empty bytes) */
                        hwDsa[1] = pktInfo.pFrags->bufVirtPtr[16] |
                                  (pktInfo.pFrags->bufVirtPtr[15]<<8) |
                                  (pktInfo.pFrags->bufVirtPtr[14]<<16) |
                                  (pktInfo.pFrags->bufVirtPtr[13]<<24);

                        port = (hwDsa[1] >> 27) + devNum*PRESTERA_PORT_NUM;

			/*Allocate skb*/
        		buf_size = dev->mtu + 20 +
                        	CPU_D_CACHE_LINE_SIZE /* 32(extra for cache prefetch) */ + 
                        	8 /* +8 to align on 8B */;
        		skb = dev_alloc_skb( buf_size ); 

			/* remove 8 bytes of DSA Tag , set MAC SA & DA */
			memcpy( skb->data, (void*)pktInfo.pFrags->bufVirtPtr , 12 );
			/* Set rest of packet (without 8 bytes */
			memcpy( skb->data + 12, 
				(void*)pktInfo.pFrags->bufVirtPtr + 20/*FP + 2*/, 
				pktInfo.pFrags->bufSize-16 );

			skb_put(skb, pktInfo.pFrags->bufSize - /*ETH_MV_HEADER_SIZE*/2 - 10);

                        //Find port device by source port number
                        sprintf( name, "p%d", port );
                        skb->dev = __dev_get_by_name(name);

        		skb->protocol = eth_type_trans(skb, skb->dev);
			skb->ip_summed = CHECKSUM_PARTIAL;
                        
                        // FastPath process and send if matched rule
                        if (PRESTERA_FP_FOUND == mv_netif_rx_nfp(skb))
                        {
                                mvFreeRxBuf(&pktInfo.pFrags->bufVirtPtr,1,
                                        devNum,queueIdx);
                                kfree_skb(skb);
                                
                                dev->stats.rx_packets++;
                                return MV_OK;//Stop threat if matched
                        }

			rx_status = pktInfo.status;
	    		if (((pktInfo.pFrags->dataSize + 4) > ETH_CSUM_MIN_BYTE_COUNT)  && 
				(rx_status & ETH_RX_IP_FRAME_TYPE_MASK) && 
				(rx_status & ETH_RX_IP_HEADER_OK_MASK)) {  
					if (!(pktInfo.fragIP)		&&
			   		(!(rx_status & ETH_RX_L4_OTHER_TYPE))&&
			   		(rx_status & ETH_RX_L4_CHECKSUM_OK_MASK)) { 
						skb->csum = 0;
						skb->ip_summed = CHECKSUM_UNNECESSARY;	
				}
				else if (pktInfo.fragIP && (rx_status & ETH_RX_L4_UDP_TYPE)) {	
				skb->csum = ntohl(0xFFFF ^ ((rx_status & ETH_RX_L4_CHECKSUM_MASK) >> ETH_RX_L4_CHECKSUM_OFFSET));
				skb->ip_summed = CHECKSUM_COMPLETE;
				}
	    		}

                        //Send the packet to port device
                        status = mvPortForward(skb,skb->dev);
			dev->stats.rx_packets++;
	
			mvFreeRxBuf(&pktInfo.pFrags->bufVirtPtr,1,
					devNum,queueIdx);
		}
	}
	
	DB( printk( "%s: %s complete ok\n", __FUNCTION__, dev->name ) );
	return 0;
	
	error:
	DB( printk( "%s: %s failed\n", __FUNCTION__, dev->name ) );
	return 1;
}

/************************************************************************
* mvPortForward --                                                           *
*   Send the received packet to Linux stack                             *
*************************************************************************/
static int mvPortForward( struct sk_buff *skb, struct net_device *dev )
{
        dev->stats.rx_packets++;
        netif_receive_skb(skb);
        
        DB( printk( "%s: %s complete ok\n", __FUNCTION__, dev->name ) );
        return 0;
}
	
/************************************************************************
* mvSwitchCheckPortStatus --                                            *
*   Check if the physycal port link up                                  *
*************************************************************************/
MV_BOOL mvSwitchCheckPortStatus(int portNum)
{
	unsigned long regVal = 0;

	if( mvSwitchReadReg( portNum/PRESTERA_PORT_NUM, 
                PRESTERA_PORT_STATUS_REG + (portNum%PRESTERA_PORT_NUM) * 0x400, 
		(MV_U32*)&regVal) != MV_OK)
	{
		return MV_FALSE;
	}
	return (regVal & 0x1) ? MV_TRUE : MV_FALSE;
}

/************************************************************************
* mv_set_mac_addr_internals --                                          *
*   Set port mac address in PP                                          *
*************************************************************************/
static int mv_set_mac_addr_internals(struct net_device *dev, void *addr )
{
        /* skip on first 2B (ether HW addr type) */
        u8* mac = &(((u8*)addr)[2]);  
        int i;
        MV_U32  devNum = dev->name[4] - '0';

        /* set new addr in hw */
        if( setCPUAddressInMACTAble( devNum, mac, 1) != MV_OK ) {
                printk( KERN_ERR "%s: ethSetMacAddr failed\n", dev->name );
        return -1;
        }

        /* set addr in the device */ 
        for( i = 0; i < 6; i++ )
                dev->dev_addr[i] = mac[i];

        printk( KERN_NOTICE "%s: mac address changed\n", dev->name );
        return 0;
}

/*******************************************************************************
* mv_set_mac_addr --                                                           *
*   Change MAC address for network device                                      *
*******************************************************************************/
static int mv_set_mac_addr( struct net_device *dev, void *addr )
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
// 
        mvSwitchWriteReg(baseAddr, PRESTERA_AD_SIZE(decodingIndex), sizeToReg); /* Set window size */		
    
        PRESTERA_AD_BARE_BITS_SET(initBare, decodingIndex); /* set corresponding bit to 0 - enable windows */

        decodingIndex++;
    }
	
    mvSwitchWriteReg(baseAddr, PRESTERA_AD_BARE, initBare);/* Enable relevant windows */
 
    printk("Switch decoding windows init is done.\n");

    return MV_OK;
}

MV_IP_HEADER mvFpProcessedHeader;
int mvFpProcessActiveFlag = 1;
int mv_netif_rx_nfp(struct sk_buff *skb)
{
        int     out_ifindex;
        struct  net_device* out_dev;
        MV_FP_STATS pFpStats;
        MV_8    name[IFNAMSIZ+1];

        MV_8 *dataPtr = skb->data;
        
        /*Parse the IP header*/
        mvFpProcessedHeader.version = (dataPtr[0] & 0xf0)>>4;
        mvFpProcessedHeader.tos = dataPtr[1];
        mvFpProcessedHeader.totalLength = (dataPtr[2]<<8) | dataPtr[3];
        mvFpProcessedHeader.identifier  = (dataPtr[4]<<8) | dataPtr[5];
        mvFpProcessedHeader.fragmentCtrl  = (dataPtr[6]<<8) | dataPtr[7];
        mvFpProcessedHeader.ttl  = dataPtr[8];
        mvFpProcessedHeader.protocol = (dataPtr[9]);
        mvFpProcessedHeader.checksum  = (dataPtr[10]<<8) | dataPtr[11];
        mvFpProcessedHeader.srcIP =  (dataPtr[15]<<24) | (dataPtr[14]<<16) |
                                     (dataPtr[13]<<8) | dataPtr[12];
        mvFpProcessedHeader.dstIP =  (dataPtr[19]<<24) | (dataPtr[18]<<16) |
                                     (dataPtr[17]<<8) | dataPtr[16];
  
        /*Find the output device*/
        out_ifindex = mvFpProcess(0, skb->data+2, 
                &mvFpProcessedHeader, &pFpStats);

        if (out_ifindex > 0)
        {//Send to output device
                if( out_ifindex == PRESTERA_FP_DROP_INTERFACE)
                        return PRESTERA_FP_FOUND;

                /*Mirror the packet to out_ifindex port*/
                sprintf( name, "p%d", out_ifindex );
                out_dev = __dev_get_by_name(name);
                BUG_ON(!out_dev);

                skb_push(skb, ETH_HLEN);

                skb->ip_summed = CHECKSUM_PARTIAL;
                skb->dev = out_dev;
                
                mvFpProcessActiveFlag = 1;
                out_dev->hard_start_xmit(skb, out_dev);
                mvFpProcessActiveFlag = 0;
                
                return PRESTERA_FP_FOUND;
        }
        
        return PRESTERA_FP_NOT_FOUND;
}
#endif /* #if defined (MV_PRESTERA_SWITCH) */
