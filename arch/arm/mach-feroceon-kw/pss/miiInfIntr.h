/*******************************************************************************
*              Copyright 2003, RADLAN Computer Communications, LTD.
*
* THIS CODE CONTAINS CONFIDENTIAL INFORMATION OF MARVELL. NO RIGHTS ARE GRANTED
* HEREIN UNDER ANY PATENT, MASK WORK RIGHT OR COPYRIGHT OF MARVELL OR ANY THIRD
* PARTY. MARVELL RESERVES THE RIGHT AT ITS SOLE DISCRETION TO REQUEST THAT THIS
* CODE BE IMMEDIATELY RETURNED TO MARVELL. THIS CODE IS PROVIDED "AS IS".
* MARVELL MAKES NO WARRANTIES, EXPRESS, IMPLIED OR OTHERWISE, REGARDING ITS
* ACCURACY, COMPLETENESS OR PERFORMANCE. MARVELL COMPRISES MARVELL TECHNOLOGY
* GROUP LTD. (MTGL) AND ITS SUBSIDIARIES, MARVELL INTERNATIONAL LTD. (MIL),
* MARVELL TECHNOLOGY, INC. (MTI), MARVELL SEMICONDUCTOR, INC. (MSI), MARVELL
* ASIA PTE LTD. (MAPL), MARVELL JAPAN K.K. (MJKK), GALILEO TECHNOLOGY LTD. (GTL)
* GALILEO TECHNOLOGY, INC. (GTI) AND RADLAN Computer Communications, LTD.
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

#ifndef __INCmiiInfInternalh
#define __INCmiiInfInternalh

/*#include "mvEth.h"*/
#include "eth/mvEth.h"

/* Macros */
#define ETH_PORT_RX_QUEUE_NUM   8
#define MII_MAX_RX_QUEUE_NUM	ETH_PORT_RX_QUEUE_NUM
#define MII_MAX_TX_QUEUE_NUM    8
#define GLB_MII_RX_QUEUE_ENABLED_MAC(queue) 	(MV_TRUE == glbMiiEthPortRxQueueEnabled[queue])
#define ETH_CAUSE_TX_BUFFER_BIT(queue)	queue 

/* Global variables */

/* mvEth driver port handle */
extern void    *glbMiiPortHandle;

/* en/dis queue status */
extern MV_BOOL  glbMiiEthPortRxQueueEnabled[MII_MAX_RX_QUEUE_NUM];

/* define for debug output */
#undef DEBUG_MII_PATH

#ifdef DEBUG_MII_PATH
  extern int mii_print_on;
  #define MII_PATH_PRINT(x) if(mii_print_on) HOSTC_term_printf x
#else
  #define MII_PATH_PRINT(x)
#endif   
  

/*******************************************************************************
* miiInfGetEthPortInit
*
* DESCRIPTION:
*       Return eth port initialization data.
*
* INPUTS:
*	portInit	- HAL structure for port initialization
*
* OUTPUTS:
*       None.
*
* RETURNS:
*       ETH_PORT_INIT - eth port initialization data
*
* COMMENTS:
*       None.
*
*******************************************************************************/
void miiInfGetEthPortInit(MV_ETH_PORT_INIT* portInit);

/*******************************************************************************
* miiInfBufferTxDone
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
*       GT_OK if successful, or
*       GT_FAIL otherwise.
*
* COMMENTS:
*       None.
*
*******************************************************************************/
int miiInfBufferTxDone
(
    IN MV_PKT_INFO	*pktInfo,
    IN MV_U32        txQ
);

/*******************************************************************************
* miiInfBufferRxSendToAppStack
*
* DESCRIPTION:
*       This function is the internal Rx callback
*
* INPUTS:
*       pIf          - interface packet was received on
*       buffer_PTR   - received packet
*       bufferLen    - length of received packet
*
* OUTPUTS:
*       None.
*
* RETURNS:
*       TRUE if successful, or
*       FALSE otherwise.
*
* COMMENTS:
*       None.
*
*******************************************************************************/
MV_BOOL miiInfBufferRxSendToAppStack
(
    IN char         *buffer_PTR,
    IN int          bufferLen,
    IN MV_U32       QueueNum
);



#endif /* __INCmiiInfInternalh */
