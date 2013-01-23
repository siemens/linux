/*******************************************************************************
Copyright (C) Marvell International Ltd. and its affiliates

This software file (the "File") is owned and distributed by Marvell
International Ltd. and/or its affiliates ("Marvell") under the following
alternative licensing terms.  Once you have made an election to distribute the
File under one of the following license alternatives, please (i) delete this
introductory statement regarding license alternatives, (ii) delete the two
license alternatives that you have not elected to use and (iii) preserve the
Marvell copyright notice above.


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
********************************************************************************
* miiInf.h
*
* DESCRIPTION:
*       Interface for ethernet communication to MII-interface based ASICS.
*
* DEPENDENCIES:
*       None
*
* FILE REVISION NUMBER:
*       $Revision: 1 $
*******************************************************************************/

#ifndef __INCmiiInfh
#define __INCmiiInfh

#include "mvTypes.h"
#include <linux/netdevice.h>

#define DFSMT_RX_Q_EN_BIT       (1)
#define DFSMT_RX_Q_NUM_OFFSET   (1)
#define DFSMT_TBL_OFFSET        (0x77400)
#define DFSMT_ENTRY_NUM         (64)
#define DFSMT_TBL_ADDR          (INTER_REGS_BASE + DFSMT_TBL_OFFSET)
#define MII_ETH_PORT_MRVL_HEADER_REG (INTER_REGS_BASE + 0x76454)

/* constants */
#define MII_DEF_TX_Q    0
#define MII_DEF_RX_Q    0
#define MAC_SRC_ADDR_SIZE       6
#define RX_BUFFER_DEFAULT_SIZE  0x600     /* MGI_MTU */
#define MINIMAL_PACKET_LENGTH   60
#define MAX_SEGMENTS_PER_PACKET 10

typedef enum {
        HOSTG_MII_CLASSIFIER_ACTION_FORWARD_E = 0,
        HOSTG_MII_CLASSIFIER_ACTION_DISCARD_E
} HOSTG_MII_CLASSIFIER_ACTION_ENT;

typedef  struct hostg_mii_classifier
{
  MV_U32 queueNum;    /*    The queue to put a matched packet into */
  MV_U32 offset;        /*  Offset from beginning of packet           */
  MV_U32 length;        /*  Number of bytes to compare with pattern */
  MV_U8    pattern[48]; /*   The pattern to compare packet with      */
  MV_U8    mask[48];    /* Mask for comparison - compare bit only if it is binary 1. If binary 0, ignore it */
  HOSTG_MII_CLASSIFIER_ACTION_ENT        action;
} HOSTG_MII_CLASSIFIER_STC;

void miiCpuCode2RxQEnable(void);
void miiUp2RxQEnable(void);
MV_STATUS miiCfgDFSMTForCpuCodes(MV_U32 cpuCode, MV_U32 rxQ);
MV_STATUS miiCfgDFSMTForAllCpuCodes(MV_U32 rxQ);
MV_VOID miiClearDFSMT(MV_U32 rxQ);
MV_VOID miiCfgRgmiiRxQs(MV_U32 rxQ);


/*******************************************************************************
* MV_Rx_FUNCPTR
*
* DESCRIPTION:
*       The prototype of the routine to be called after a packet was received
*
* INPUTS:
*       packet_PTR    - The recieved packet.
*       packetLen     - The recived packet len
*
* OUTPUTS:
*       None.
*
* RETURNS:
*       MV_TRUE if it has handled the input packet and no further action should
*               be taken with it, or
*       MV_FALSE if it has not handled the input packet and normal processing.
*
* COMMENTS:
*       None.
*
*******************************************************************************/
typedef MV_STATUS (*MV_Rx_FUNCPTR)
(
    IN MV_U8     * segmentsList[],
    IN MV_U32      segmentLen[],
    IN MV_U32      segmentNum,
    IN MV_U32      rxQueueNum
);

typedef MV_STATUS (*MV_Tx_COMPLETE_FUNCPTR)
(
    IN MV_U8     * segmentsList[],
    IN MV_U32      segmentNum
);


/*******************************************************************************
* gtEthernetPacketSend
*
* DESCRIPTION:
*       This function transmits an Ethenet packet to the Packet processor
*
* INPUTS:
*       segmentList    - A list of pointers to the packets segments.
*       segmentLen     - A list of segment length.
*       numOfSegments  - The number of segment in segment list.
*       txQ            - TX queue
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
MV_STATUS gtEthernetPacketSend
(
    IN MV_U8     * segmentsList[],
    IN MV_U32      segmentLen[],
    IN MV_U32      segmentNum,
    IN MV_U32      txQ
);

MV_PKT_INFO* gtEthernetPacketReceive
(
    IN MV_U32 rxQueue
);

/*******************************************************************************
* gtEthernetRxCallbackBind
*
* DESCRIPTION:
*       This bind the user Rx callback
*
* INPUTS:
*       userRxFunc - the user Rx callbak function
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
MV_STATUS gtEthernetRxCallbackBind
(
    IN MV_Rx_FUNCPTR userRxFunc
);

/*******************************************************************************
* gtEthernetTxCompleteCallbackBind
*
* DESCRIPTION:
*       Tx callback function
*
* INPUTS:
*       bufAddr - Pointer to the packet sent.
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
MV_STATUS gtEthernetTxCompleteCallbackBind
(
    IN MV_Tx_COMPLETE_FUNCPTR userTxCompleteFunc
);

/*******************************************************************************
* miiInfBufferRxDone
*
* DESCRIPTION:
*       Rx callback function
*
* INPUTS:
*       bufAddr - Pointer to the packde to tbe sent.
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
MV_STATUS miiInfBufferRxDone
(
    MV_PKT_INFO *pktInfo,
    IN MV_U32   queueNum
);

/*******************************************************************************
* gtEthernetRxPacketFree
*
* DESCRIPTION:
*       This function free the recievd Rx buffer - if needed
*       ( This BSP does this action automatically -
*         This API function will be called from the PSS only for comptabilty reason )
*
* INPUTS:
*       segmentsList  - pointer the the 14 bytes packet header.
*       segmentNum  - length ot the payload.
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
MV_STATUS gtEthernetRxPacketFree
(
    IN MV_U8     * segmentsList[],
    IN MV_U32      segmentNum,
    IN MV_U32      QueueNum
);

/*******************************************************************************
* gtEthernetInterfaceEnable
*
* DESCRIPTION:
*
* INPUTS:
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
MV_STATUS gtEthernetInterfaceEnable(void);

/*******************************************************************************
* gtEthernetInterfaceDisable
*
* DESCRIPTION:
*
* INPUTS:
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
MV_STATUS gtEthernetInterfaceDisable(void);

/*******************************************************************************
* gtEthernetInterfaceInit
*
* DESCRIPTION:
*
* INPUTS:
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
MV_STATUS gtEthernetInterfaceRxInit(
IN MV_U32     rxBufPoolSize,    /* buffer pool size  */
IN MV_U8     *rxBufBlock,   /* the address of the pool */
IN MV_U32     rxBufBlockSize,   /* the buffer requested size */
INOUT MV_U32 *numOfRxBufs,  /* number of requested buffers, and actual buffers created */
IN MV_U32     headerOffset, /* packet header offset size */
IN MV_U32     rxQNum,
IN MV_U32     rxQbufPercentage[]
);

/*******************************************************************************
* gtEthernetInterfaceTxInit
*
* DESCRIPTION:
*
* INPUTS:
*       numOfTxDescr - Total number of TX descriptors to be allocated for
*                      all queues.
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
MV_STATUS gtEthernetInterfaceTxInit(IN MV_U32 numOfTxDescr);


void miiInfCpuPortConfig(void);

#endif /* __INCmiiInfh */


