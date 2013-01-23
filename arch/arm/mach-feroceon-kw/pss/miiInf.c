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
 * miiInf.c
 *
 * DESCRIPTION:
 *       Interface for ethernet communication to MII-interface based ASICS.
 *
 * DEPENDENCIES:
 *       None
 *
 * FILE REVISION NUMBER:
 *       $Revision: 2 $
 *
 *******************************************************************************/

#include "eth/mvEth.h"
#include "eth/gbe/mvEthGbe.h"
#include "pssBspApis.h"
#include "mvOsPrestera.h"
#include "miiInf.h"
#include "miiInfIntr.h"

/* definitions */

//
// if y is in marvell virtual himem then translate it to marvell physical himem.
// else
// if y happens to be in marvell physical himem then do nothing
// else
// call mvOsIoVirtToPhy (x, y)
//
#define  _mvOsIoVirtToPhy(x, y) \
  (bspVirtIsInMvHiMem((MV_U32)(y))) ? bspVirt2Phys((MV_U32)(y)) : \
    (bspPhysIsInMvHiMem((MV_U32)(y))) ? (MV_U32)y : mvOsIoVirtToPhy (x, y)

#define MII_USE_CODE_2_RX_Q_MAPPING

/* DEBUG */
#if 0
#define DEBUG_MII_INF
#define DEBUG_PRINT
#endif
#if 0
#define MV_DEBUG 
#endif
#ifdef MV_DEBUG
#define DB(x) x
#else
#define DB(x)
#endif


typedef  struct mii_buff_manag
{
  MV_U32 rx_queue_bufs_base[MII_MAX_RX_QUEUE_NUM];    /* base address of Rx buffers in queue */
  MV_U32 num_of_rx_queue_bufs[MII_MAX_RX_QUEUE_NUM];  /* number of Rx buffers in queue */
  MV_U32 buffer_size;                                 /* the buffer size (defined by application) */
  MV_U32 header_offset;                           /* packet header offset size */
  MV_U32 rx_desc_base;
  MV_U32 num_of_rx_desc[MII_MAX_RX_QUEUE_NUM];
  MV_U32 num_of_tx_desc[MII_MAX_TX_QUEUE_NUM];
  MV_U32 num_of_free_rx[MII_MAX_RX_QUEUE_NUM];
  MV_U32 num_of_free_tx[MII_MAX_TX_QUEUE_NUM];
  MV_U32 totalNumOfTxDesc;
  MV_U32 rx_counter[MII_MAX_RX_QUEUE_NUM];
  MV_U32 tx_counter;
  MV_U32 tx_comp_counter;
  MV_PKT_INFO**   txPktInfoArr;
  MV_U32          txPktInfoIdx;
} MII_BUFF_MNG;


/* eth drvr handle - initialized by mgiEnd */
void    *glbMiiPortHandle = NULL;
MV_BOOL  glbMiiEthPortRxQueueEnabled[MII_MAX_RX_QUEUE_NUM];

/* function pointer binded by the PSS in order to release the TX descriptor */
MV_Tx_COMPLETE_FUNCPTR txCompleteFunc = ( MV_Tx_COMPLETE_FUNCPTR ) NULL;

/* The user internal Rx callback routine */
static MV_Rx_FUNCPTR  userRxCallback = NULL;

MII_BUFF_MNG mii_managment;

extern bspEthTxMode_ENT    bspEthTxMode;

static MV_PKT_INFO** rxPktInfoArr[MII_MAX_RX_QUEUE_NUM];
static MV_U32 curr_pkt_info_idx[MII_MAX_RX_QUEUE_NUM] = {0};

MV_STATUS miiInfRxBufferInit(void);
MV_STATUS miiInfTxInit(void);
void miiInfRxDescInit ( char *bufferPtr, int dataSize, int queueNum );
MV_STATUS miiInfQueueEnable ( IN MV_U32 queueNum );
MV_STATUS miiInfQueueDisable ( IN MV_U32 queueNum );
MV_PKT_INFO* miiInfRxFill ( MV_U8 *buf );
void miiInfCpuPortConfig ( void );

void gc_dump ( char *buf, int len )
{
#define isprint(c)       ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9'))
  
  int offs, i;
  int j;
  
  for ( offs = 0; offs < len; offs += 16 )
  {
    j = 1;
    mvOsPrintf ( "%08x   ", offs );
    for ( i = 0; i < 16 && offs + i < len; i++ )
    {
      mvOsPrintf ( "%02x", ( unsigned char ) buf[offs + i] );
      if ( ! ( ( j++ ) %4 ) && ( j < 16 ) )
        mvOsPrintf ( "," );
    }
    for ( ; i < 16; i++ )
      mvOsPrintf ( "   " );
    mvOsPrintf ( "  " );
    for ( i = 0; i < 16 && offs + i < len; i++ )
      mvOsPrintf ( "%c",
                   isprint ( buf[offs + i] ) ? buf[offs + i] : '.' );
    mvOsPrintf ( "\n" );
  }
  mvOsPrintf ( "\n" );
}

/*
 * assumes 8 Rx queues are configured
 */
void miiCpuCode2RxQEnable(void)
{
  /*
   * Enalbes mapping of CpuCode in DSA tag of incoming packet
   * to RxQ of RGMII-1 using DFSMT table
   */
  *(MV_U32 *)MII_ETH_PORT_MRVL_HEADER_REG = 0x804;
}

/*
 * assumes 8 Rx queues are configured
 */
void miiUp2RxQEnable(void)
{
  /*
   * Enalbes mapping of UP in DSA tag of incoming packet
   * to RxQ of RGMII-1;
   * RxQ = UP
   */
  *(MV_U32 *)MII_ETH_PORT_MRVL_HEADER_REG = 0x802;
}

MV_STATUS miiCfgDFSMTForCpuCodes(MV_U32 cpuCode, MV_U32 rxQ)
{
  unsigned int entryValue;
  unsigned int tblOffset;
  unsigned int regOffset;

  DB(msOsPrintf("miiCfgDFSMTForCpuCodes called, cpuCode=0x%08x, rxQ=0x%08x\n", 
            cpuCode, rxQ));

  if (rxQ >= MII_MAX_RX_QUEUE_NUM || cpuCode >= DFSMT_ENTRY_NUM*4)
  {
    DB ( mvOsPrintf("%s: wrong params rxQ (%d) cpuCode (%d).\n", __func__, rxQ, cpuCode); )
      return MV_FAIL;
  }
  
  /* Locate the DFSMT table entry */
  tblOffset = (cpuCode / 4);     /* Register offset from DFSMT table base    */
  regOffset = cpuCode % 4;       /* Entry offset within the above register */
  
  entryValue = MV_REG_READ(DFSMT_TBL_ADDR + tblOffset*4);
  
  entryValue &= ~(0xFF << (8 * regOffset));
  entryValue |= ((0x01 | (rxQ <<1)) << (8 * regOffset));
  
  MV_REG_WRITE((DFSMT_TBL_ADDR + tblOffset*4), entryValue);
  return MV_OK;
}

MV_STATUS miiCfgDFSMTForAllCpuCodes(MV_U32 rxQ)
{
  MV_U32 i, ptr = DFSMT_TBL_ADDR;
  MV_U32 rxQcfg;
  
  if (rxQ >= MII_MAX_RX_QUEUE_NUM)
  {
    mvOsPrintf("%s: wrong rxQ (%d).\n", __func__, rxQ);
    return MV_FAIL;
  }
  
  for (i = 0; i < DFSMT_ENTRY_NUM; i++)
  {
    rxQcfg = (rxQ << DFSMT_RX_Q_NUM_OFFSET) | DFSMT_RX_Q_EN_BIT;
    *((MV_U32 *)ptr + i) = rxQcfg | (rxQcfg << 8)
      | (rxQcfg << 16)
      | (rxQcfg << 24);
  }
  
  return MV_OK;
}

MV_VOID miiClearDFSMT(MV_U32 rxQ)
{
  MV_U32 i, ptr = DFSMT_TBL_ADDR;
  
  for (i = 0; i < 64; i++)
  {
    *((MV_U32 *)ptr + i) = 0x0;
  }
}

MV_VOID miiCfgRgmiiRxQs(MV_U32 rxQ)
{
  *(MV_U32 *)0xf1076454 = 0x804;
  miiCfgDFSMTForAllCpuCodes(rxQ);
}

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
 )
{
  DB(mvOsPrintf ( "gtEthernetRxCallbackBind: bind to 0x%x.\n", ( MV_U32 ) userRxFunc ));

  userRxCallback = userRxFunc;
  return MV_OK;
}

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
 )
{
  DB(mvOsPrintf ( "gtEthernetTxCompleteCallbackBind: bind to 0x%x.\n", 
                  ( MV_U32 ) userTxCompleteFunc ));

  /* Binding the TX complete function from the PSS */
  txCompleteFunc = userTxCompleteFunc;

  return MV_OK;
}

/*******************************************************************************
 * miiInfBufferRxDone
 *
 * DESCRIPTION:
 *       Rx callback function.
 *       Give the buffer back to HAL
 *
 * INPUTS:
 *       pktInfo         - pointer to PacketInfo structure.
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
MV_STATUS miiInfBufferRxDone
(
 MV_PKT_INFO *pktInfo,
 IN MV_U32   queueNum
 )
{
  MV_STATUS status;

  /* give the buffer back to hal (re-init the buffer address) */
  pktInfo->pktSize = mii_managment.buffer_size/*RX_BUFFER_DEFAULT_SIZE*/; /* how much to invalidate */

  status = mvEthPortRxDone ( glbMiiPortHandle, queueNum, pktInfo );

  if ( status==MV_OK || status==MV_FULL )
  {
    mii_managment.num_of_free_rx[queueNum]++;
    return MV_OK;
  }

  return status;
}

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
 *
 *******************************************************************************/
MV_STATUS gtEthernetPacketSend
(
 IN MV_U8     * segmentList[],
 IN MV_U32      segmentLen[],
 IN MV_U32      segmentNum,
 IN MV_U32      txQ
 )
{
  MV_STATUS        status;
  
  MV_U32           overallPacketLen = 0;
  int              i;
  MV_PKT_INFO      *txPktInfoPtr = NULL;
  MV_BUF_INFO      *bufInfoPtr = NULL;

  MV_U32       cause;
  
  if ( mii_managment.num_of_free_tx[txQ] < segmentNum)
  {
    return ( MV_FAIL );
  }
  
  if ( segmentNum > MAX_SEGMENTS_PER_PACKET )
  {
    return MV_FAIL;
  }
  
  txPktInfoPtr = mii_managment.txPktInfoArr[mii_managment.txPktInfoIdx++];
  if(mii_managment.txPktInfoIdx == mii_managment.totalNumOfTxDesc)
    mii_managment.txPktInfoIdx = 0;
  
  bufInfoPtr = txPktInfoPtr->pFrags;
  
  /* Sending the segments sent from the PSS */
  for ( i=0; i<segmentNum; i++ )
  {
#ifdef DEBUG_PRINT
    {
      int len = segmentLen[i];
      
      mvOsPrintf ( "gtEthernetPacketSend: Segment %d - 0x%x\n",i,
                   ( unsigned int ) segmentList[i] );
      mvOsPrintf ( "len=%d, buffer at 0x%p = \n", len, segmentList[i] );
      gc_dump((char *)segmentList[i], len);
    }
#endif
    
    
    /* Sum of the packet length in order to check padding */
    overallPacketLen += segmentLen[i];
    bufInfoPtr[i].bufVirtPtr = (MV_U8*)segmentList[i];
    bufInfoPtr[i].bufPhysAddr = mvOsIoVirtToPhy(NULL, segmentList[i]);
    
    /* At the last segment ... */
    if ( i == ( segmentNum-1 ) )
    {
      if ( overallPacketLen < MINIMAL_PACKET_LENGTH )
      {
        bufInfoPtr[i].dataSize = MINIMAL_PACKET_LENGTH - overallPacketLen
          + segmentLen[i];
        overallPacketLen += (MINIMAL_PACKET_LENGTH-overallPacketLen);
      }
      else
        bufInfoPtr[i].dataSize   = segmentLen[i];
    }
    else
    {
      bufInfoPtr[i].dataSize       = segmentLen[i];
    }
    
#ifdef DEBUG_MII_INF
    mvOsPrintf(" bufInfo[%d].bufVirtPtr: 0x%x\n", i, bufInfoPtr[i].bufVirtPtr);
    mvOsPrintf(" bufInfo[%d].bufPhysAddr: 0x%x\n", i, bufInfoPtr[i].bufPhysAddr);
    mvOsPrintf(" bufInfo[%d].dataSize: 0x%x %d\n", i, bufInfoPtr[i].dataSize, bufInfoPtr[i].dataSize);
#endif
  } /* build bufInfo list */
  
  txPktInfoPtr->osInfo = (MV_ULONG)0x44CAFE44;
  txPktInfoPtr->pktSize = overallPacketLen;
  txPktInfoPtr->status = 0;
  txPktInfoPtr->numFrags = segmentNum;
  
#ifdef DEBUG_MII_INF
  mvOsPrintf(" txPktInfo.pktSize: 0x%x %d\n", txPktInfoPtr->pktSize, txPktInfoPtr->pktSize);
#endif
  
  /* Sending the data packet sent by the calling function */
  status = mvEthPortSgTx( glbMiiPortHandle, txQ, txPktInfoPtr );
  
  if (status != MV_OK)
  {
    if(mii_managment.txPktInfoIdx == 0)
    {
      mii_managment.txPktInfoIdx = mii_managment.totalNumOfTxDesc - 1;
    }
    else
    {
      mii_managment.txPktInfoIdx--;
    }
    
    mvOsPrintf ( "mvEthPortTx Failed\n" );
    return MV_FAIL;
  }
  
  /* If we in synch mode we don't wait for Tx done interrupt.
     We clean the descriptor just after Tx DMA is done. */
  
  if ( bspEthTxMode == bspEthTxMode_synch_E )
  {
    MV_PKT_INFO *pPktInfo;
    
    DB(mvOsPrintf("in gtEthernetPacketS - we are now in sync mode\n"));
    /*
      This procedure is used for sync mode only. 
      We are waiting on the TxEnd bit for EGIGA_CPU_PORT (eg - TxEnd0).
      See FS_88F6281_6192_80_Internal.pdf pages 661-662.
      
      Not to be confused with the mgi-tx-interrupt 
      (see our fellow pssBspApis.c - txBuffer[0] bit, in same manual)
      which we use for async mode.
      the mgi-tx-interrupt must be disabled for this to work !. gc */
    
    do /* wait till Tx DMA is done */
    {
      cause = MV_REG_READ(ETH_INTR_CAUSE_EXT_REG(EGIGA_CPU_PORT));
    }
    while ( ! (cause & (1 << ETH_CAUSE_TX_BUFFER_BIT(txQ))));

    /* clear bits for TX_DONE */
    MV_REG_WRITE(ETH_INTR_CAUSE_EXT_REG(EGIGA_CPU_PORT),
                 ~(1<<ETH_CAUSE_TX_BUFFER_BIT(txQ)));

    /* clear descriptor */
    pPktInfo = mvEthPortTxDone( glbMiiPortHandle, txQ);
    //    printk(">>>> after mvEthPortTxDone\n"); while(1);
    
    if ( pPktInfo != NULL )
    {
      
      /* validate skb */
      if( (pPktInfo != txPktInfoPtr) || (pPktInfo->osInfo != 0x44CAFE44 ) )
      {
        mvOsPrintf ( "error\n" );
        return MV_FAIL;
      }
      
      /* handle tx error */
      if ( pPktInfo->status & ( ETH_ERROR_SUMMARY_BIT ) )
      {
        mvOsPrintf ( "bad status (error)\n" );
        return MV_FAIL;
      }
    }
    
    /* call mii tx complete handler */
    /* assumption by miiInf: only one desc is used per packet
     * since mvEth driver only returns the last desc and miiInf
     * doesn't hold tx buff chain !!!!! */
    miiInfBufferTxDone((MV_PKT_INFO*)pPktInfo,txQ);
  }
  else
  {
    DB(mvOsPrintf("in gtEthernetPacketS - we are now in async mode\n"));
  }
  
  mii_managment.num_of_free_tx[txQ] -= segmentNum;
  mii_managment.tx_counter++;
  
  return MV_OK;
}

/*******************************************************************************
 * gtEthernetPacketReceive
 *
 * DESCRIPTION:
 *       This function receives an Ethenet packet from the Packet processor
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
 *
 *******************************************************************************/

MV_PKT_INFO* gtEthernetPacketReceive
(
    IN MV_U32           rxQueue
 )
{
    return mvEthPortRx( glbMiiPortHandle, rxQueue);
}



/*******************************************************************************
 * miiInfBufferTxDone
 *
 * DESCRIPTION:
 *       Calling the PSS callback function to release the TX buffer
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

int miiInfBufferTxDone
(
 IN MV_PKT_INFO	*pktInfo,
 IN MV_U32       txQ
)
{
    static IN MV_U8     * segmentList[MAX_SEGMENTS_PER_PACKET];
    static IN MV_U32      segmentNum;
	int i;

    /* setting the descriptor information to the segment list */
	segmentNum = pktInfo->numFrags;
    for(i=0; i<segmentNum; i++)
	{
        segmentList[i] = (MV_U8 *)pktInfo->pFrags[i].bufVirtPtr;
	}

    if ( txCompleteFunc != NULL )
        txCompleteFunc(segmentList, segmentNum);

    mii_managment.num_of_free_tx[txQ] += segmentNum;
    mii_managment.tx_comp_counter += segmentNum;

    segmentNum = 0;

    return MV_TRUE;
}

/*******************************************************************************
 * gtEthernetRxPacketFree
 *
 * DESCRIPTION:
 *       This function free the recievd Rx buffer - if needed
 *       ( This BSP does this action automatically -
 *         This API function will be called from the PSS only for comptabilty reason )
 *
 * INPUTS:
 *       segmentList  - pointer the the 14 bytes packet header.
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
 IN MV_U8     * segmentList[],
 IN MV_U32      segmentNum,
 IN MV_U32      queueNum
 )
{
  MV_STATUS status;
  MV_PKT_INFO* pktInfo = rxPktInfoArr[queueNum][curr_pkt_info_idx[queueNum]];

  //__asm__ __volatile__("19:\n" "b 19b\n" :::"memory");

  curr_pkt_info_idx[queueNum] = ( curr_pkt_info_idx[queueNum]+1 ) %mii_managment.num_of_rx_desc[queueNum];

  /* Return buffer pointer to original -> before IP header alignment removal */
  pktInfo->pFrags->bufVirtPtr = segmentList[0] - 2;
  pktInfo->pFrags->bufPhysAddr = _mvOsIoVirtToPhy ( NULL, pktInfo->pFrags->bufVirtPtr );

  status = miiInfBufferRxDone ( ( MV_PKT_INFO* ) pktInfo, queueNum );

  if ( status==MV_OK && !glbMiiEthPortRxQueueEnabled[queueNum] )
  {
    miiInfQueueEnable ( queueNum );
  }

  return status;
}


MV_BOOL miiInfPortInit ( void )
{

  MV_ETH_PORT_INIT   portInit;

  /* get port init from miiInf */
  miiInfGetEthPortInit ( &portInit );

  /* Callout to perform init */
  /* init the hal -- create internal port control structure and descriptor rings, */
  /* open address decode windows, disable rx and tx operations. mask interrupts.  */

  glbMiiPortHandle = mvEthPortInit ( EGIGA_CPU_PORT, &portInit );

  if ( glbMiiPortHandle == NULL )
  {
    mvOsPrintf ( "mgiInfPortInit failed\n" );
    return ( MV_FAIL );
  }

  return MV_TRUE;
}


extern void mgiInfTxRxIsrConnect (void);

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
MV_STATUS gtEthernetInterfaceEnable(void)
{
  static int first_time = 1;
  unsigned long i;
    MV_STATUS status = MV_FAIL;

  if ( first_time )
  {
#ifdef DEBUG_MII_INF
    mvOsPrintf ( "%s: init process - miiInfStartEndDriver start.\n",__FUNCTION__ );
#endif
    if ( MV_FALSE == mvCtrlPwrClckGet ( ETH_GIG_UNIT_ID, EGIGA_CPU_PORT ) )
    {
#ifdef DEBUG_MII_INF
      mvOsPrintf ( "%s: Egiga%d clock was not enabled. Enabling Clock\n",__FUNCTION__,EGIGA_CPU_PORT );
#endif
      mvCtrlPwrClckSet ( ETH_GIG_UNIT_ID, EGIGA_CPU_PORT, MV_TRUE );
    }

    if ( miiInfPortInit() != MV_TRUE )
    {
      return MV_FAIL;
    }

    /* attaching rx buffers to rx descs */
    if ( miiInfRxBufferInit() !=MV_OK )
    {
      mvOsPrintf ( "miiInfRxBufferInit Failed\n" );
      return MV_FAIL;
    }

        if (miiInfTxInit() != MV_OK)
        {
            mvOsPrintf("miiInfTxInit Failed\n");
            return MV_FAIL;
        }

    /* re-start Ethernet port operation */
    mgiInfTxRxIsrConnect();

    MV_REG_WRITE ( ETH_TX_QUEUE_COMMAND1_REG ( EGIGA_CPU_PORT ), 0x8 );

    /* start the hal - rx/tx activity */
    /* Check if link is up for 2 Sec */
    for ( i = 1; i < 100 ; i ++ )
    {
      status = mvEthPortEnable ( glbMiiPortHandle );
      if ( status == MV_OK )
        break;
      mvOsDelay ( 20 );
    }

    if ( status != MV_OK )
    {
      mvOsPrintf ( "%s: mvEthPortEnable failed (error)\n", __FUNCTION__ );
      return MV_FAIL;
    }

#ifdef DEBUG_MII_INF
    mvOsPrintf ( "%s: init process - miiInfStartEndDriver end.\n",__FUNCTION__ );
#endif

    
    /* Connect Egiga to CPU Port 
     * This is only controller configuration
     * Switch configuration should be done separately */
    miiInfCpuPortConfig();
    
    first_time = 0;
  }

  /*
   * should be last
   */
#ifdef MII_USE_CODE_2_RX_Q_MAPPING
  miiCpuCode2RxQEnable();
  miiCfgDFSMTForAllCpuCodes(0 /* rxQ */);
#elif defined MII_USE_UP_2_RX_Q_MAPPING
  miiUp2RxQEnable();
#else
  /* Zero RxQ only is used */
#endif
  
  return MV_OK;
}

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
MV_STATUS gtEthernetInterfaceDisable ( void )
{
  int i;

  /* Diable all queues */
  for ( i=0; i<MII_MAX_RX_QUEUE_NUM; i++ )
    glbMiiEthPortRxQueueEnabled[i] = MV_FALSE;

  return MV_OK;
}


/*******************************************************************************
 * gtEthernetInterfaceRxInit
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
MV_STATUS gtEthernetInterfaceRxInit (
 IN MV_U32     rxBufPoolSize,        /* buffer pool size  */
 IN MV_U8      *rxBufBlock,          /* the address of the pool */
 IN MV_U32     rxBufBlockSize,       /* the buffer requested size */
 INOUT MV_U32 *numOfRxBufs,          /* number of requested buffers, and actual buffers created */
 IN MV_U32     headerOffset,         /* packet header offset size */
 IN MV_U32     rxQNum,
 IN MV_U32     rxQbufPercentage[]
 )
{
  int i, delta;
  int overall_num_of_bufs;
  int percentage_sum = 0;

  /* check input parameters validation */
  if ( ( rxBufBlock == NULL )                      ||
       ( rxBufBlockSize < RX_BUFFER_DEFAULT_SIZE ) ||
       ( rxBufPoolSize < rxBufBlockSize )          ||
       ( headerOffset > rxBufBlockSize ) )
  {
    return MV_FAIL;
  }

  delta = ( ( int ) rxBufBlock ) %8;
  rxBufBlock += delta;


#ifdef DEBUG_MII_INF
  mvOsPrintf ( "gtEthernetInterfaceRxInit: rxBufBlock phys address 0x%x\n", ( unsigned int ) rxBufBlock );
#endif

  /* rxBufBlockSize += headerOffset;  We removed this line. We leave some space at the start of the pool for headers.
     This header shift should only be at the start of the whole pool, not before each block. */
  rxBufBlockSize += ( rxBufBlockSize%8 );

  /* KW support 8 queues */
  if ( ( rxQNum > MII_MAX_RX_QUEUE_NUM ) && 
       ( rxQbufPercentage[MII_MAX_RX_QUEUE_NUM] != 0 ) )
    return MV_FAIL;

  overall_num_of_bufs = rxBufPoolSize/rxBufBlockSize;

  mii_managment.header_offset = headerOffset;
  mii_managment.buffer_size   = rxBufBlockSize;

  *numOfRxBufs = 0;

  /* distribute buffers amoung the queues */
  for ( i=0; i<MII_MAX_RX_QUEUE_NUM; i++ )
  {
    mii_managment.num_of_rx_queue_bufs[i] = ( ( overall_num_of_bufs * rxQbufPercentage[i] ) / 100 );
    percentage_sum+=rxQbufPercentage[i];
    if ( percentage_sum > 100 )
    {
      mvOsPrintf ( "%s:Error -Percentage summary exceeds 100%s (%d)\n",
                   __FUNCTION__, "%", percentage_sum );
      return MV_FAIL;
    }

    if ( mii_managment.num_of_rx_queue_bufs[i] )
      mii_managment.rx_queue_bufs_base[i] = ( ( MV_U32 ) rxBufBlock + ( *numOfRxBufs * rxBufBlockSize ) );
    else
      mii_managment.rx_queue_bufs_base[i] = 0;

    /* # of descs is identical to the # of bufs */
    mii_managment.num_of_rx_desc[i] = mii_managment.num_of_rx_queue_bufs[i];

    *numOfRxBufs += mii_managment.num_of_rx_queue_bufs[i];
  }

  return MV_OK;
}


/*******************************************************************************
 * miiInfBufferRxInit
 *
 * DESCRIPTION:
 *
 * INPUTS:
 *
 * OUTPUTS:
 *       None.
 *
 * RETURNS:
 *       MV_TRUE if successful, or
 *       MV_FALSE otherwise.
 *
 * COMMENTS:
 *       None.
 *
 *******************************************************************************/
MV_STATUS miiInfRxBufferInit(void)
{
  MV_U8     *pBufferPtr;
  int       ix, queue_num;             /* A counter */
  MV_U8*  temp_buf_ptr;

  /* attach new buffer for each descriptor */
  for ( queue_num = 0; queue_num < MII_MAX_RX_QUEUE_NUM; queue_num++ )
  {
#ifdef DEBUG_MII_INF
    mvOsPrintf ( "num_of_rx_queue_bufs[%d] = %d\n",queue_num,mii_managment.num_of_rx_queue_bufs[queue_num] );
#endif
    if ( mii_managment.num_of_rx_queue_bufs[queue_num]==0 )
    {
      glbMiiEthPortRxQueueEnabled[queue_num] = MV_FALSE;
      continue;
    }

    glbMiiEthPortRxQueueEnabled[queue_num] = MV_TRUE;

#ifdef DEBUG_MII_INF
    mvOsPrintf ( "Setting ring for Q %d\n",queue_num );
#endif
    pBufferPtr = ( MV_U8* ) mii_managment.rx_queue_bufs_base[queue_num];

    mii_managment.num_of_free_rx[queue_num] = mii_managment.num_of_rx_desc[queue_num];

    rxPktInfoArr[queue_num] = ( MV_PKT_INFO** ) osMalloc ( sizeof ( MV_PKT_INFO* ) * mii_managment.num_of_rx_desc[queue_num] );

    if ( rxPktInfoArr[queue_num] == NULL )
    {
      mvOsPrintf ( "%s: error - Failed to alloc rxPktInfoArr[%d]!\n",__FUNCTION__, queue_num );
      return MV_FALSE;
    }

    /* # of descs is identical to # of bufs. */
    for ( ix = 0; ix < mii_managment.num_of_rx_desc[queue_num]; ix++ )
    {
      if ( pBufferPtr == NULL )
      {
#ifdef DEBUG_MII_INF
        mvOsPrintf ( "miiInfRxBufferInit: error - NULL buffer pointer.!\n" );
#endif
        return MV_FALSE;

      }
#ifdef DEBUG_MII_INF
      mvOsPrintf ( "miiInfRxBufferInit: buffer 0x%x.\n", ( u32 ) pBufferPtr );
#endif

      /* pass rx buff to eth drv */
      temp_buf_ptr  = pBufferPtr + mii_managment.header_offset;
      temp_buf_ptr  += ( ( int ) temp_buf_ptr ) %8;

      miiInfRxDescInit ( ( MV_8* ) temp_buf_ptr, mii_managment.buffer_size/*RX_BUFFER_DEFAULT_SIZE*/, queue_num );

      /* next buf */
      pBufferPtr += mii_managment.buffer_size;
    } /* for: # of bufs per q */

    /* after filling rxPktInfoArr[queue_num] we can reset the pointer */
    curr_pkt_info_idx[queue_num] = 0;

  } /* for: # of queues */

  return MV_OK;
}

/*******************************************************************************
* miiInfTxInit
*
* DESCRIPTION:
*
* INPUTS:
*       None
* OUTPUTS:
*       None.
*
* RETURNS:
*       MV_TRUE if successful, or
*       MV_FALSE otherwise.
*
* COMMENTS:
*       None.
*
*******************************************************************************/
MV_STATUS miiInfTxInit(void)
{
    MV_U32  i;  /* iterator */

    mii_managment.txPktInfoArr = 
        mvOsMalloc(sizeof(MV_PKT_INFO*) * mii_managment.totalNumOfTxDesc);

    if (mii_managment.txPktInfoArr == NULL)
    {
        return MV_FAIL;
    }

    for (i = 0; i < mii_managment.totalNumOfTxDesc; i++)
    {
        mii_managment.txPktInfoArr[i] = mvOsMalloc(sizeof(MV_PKT_INFO));
        if(mii_managment.txPktInfoArr[i] == NULL)
        {
            return MV_FAIL;
        }

        memset(mii_managment.txPktInfoArr[i], 0, sizeof(MV_PKT_INFO));
        mii_managment.txPktInfoArr[i]->srcIdx = (char)-1;
        mii_managment.txPktInfoArr[i]->pFrags = 
            mvOsMalloc(sizeof(MV_BUF_INFO)* MAX_SEGMENTS_PER_PACKET);

        if(mii_managment.txPktInfoArr[i]->pFrags == NULL)
        {
           return MV_FAIL;
        }

        memset(mii_managment.txPktInfoArr[i]->pFrags, 0, sizeof(MV_BUF_INFO)* MAX_SEGMENTS_PER_PACKET);
    }

    mii_managment.txPktInfoIdx = 0;

    return MV_OK;
}

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
MV_STATUS gtEthernetInterfaceTxInit(IN MV_U32 numOfTxDescr)
{
    MV_U32 txDescrPerQueue; /* number of tx descriptor per queue */
    MV_U32 i;               /* iterator */

    /* TODO: check if mvEthPortInit allready called */

    /* distribute the descriptors evenly between all queues */
    txDescrPerQueue = numOfTxDescr / MII_MAX_TX_QUEUE_NUM;

    for(i=0; i<MII_MAX_TX_QUEUE_NUM; i++)
    {
        mii_managment.num_of_tx_desc[i] = txDescrPerQueue;
        mii_managment.num_of_free_tx[i] = txDescrPerQueue;
    }

    /* add the remaining descriptors to default tx queue */
    mii_managment.num_of_tx_desc[MII_DEF_TX_Q] += numOfTxDescr % MII_MAX_TX_QUEUE_NUM;
    mii_managment.num_of_free_tx[MII_DEF_TX_Q] += numOfTxDescr % MII_MAX_TX_QUEUE_NUM;
    mii_managment.totalNumOfTxDesc = numOfTxDescr;

    return MV_OK;
}


/*******************************************************************************
 * miiInfRxDescInit
 *
 * DESCRIPTION:
 *
 *
 * INPUT:
 *
 * OUTPUT:
 *
 * RETURN:
 *       None.
 *
 *******************************************************************************/
void miiInfRxDescInit ( char *bufferPtr, int dataSize, int queueNum )
{
  MV_PKT_INFO *pPktInfo;
  MV_STATUS status;

  pPktInfo = miiInfRxFill ( ( MV_U8* ) bufferPtr );
  rxPktInfoArr[queueNum][curr_pkt_info_idx[queueNum]++] = pPktInfo;

  status = mvEthPortRxDone ( glbMiiPortHandle, queueNum, pPktInfo );
  if ( status!=MV_OK && status!=MV_FULL )
  {
    mvOsPrintf ( "mvEthPortRxDone error\n" );
    return;
  }

  return;
}

/*******************************************************************************
 * miiInfQueueEnable
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
MV_STATUS miiInfQueueEnable ( IN MV_U32 queueNum )
{
  /* Enable one queue */
  glbMiiEthPortRxQueueEnabled[queueNum] = MV_TRUE;

  return MV_OK;
}

/*******************************************************************************
 * miiInfQueueDisable
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
MV_STATUS miiInfQueueDisable ( IN MV_U32 queueNum )
{
  glbMiiEthPortRxQueueEnabled[queueNum] = MV_FALSE;

  return MV_OK;
}


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
void miiInfGetEthPortInit(MV_ETH_PORT_INIT* portInitPtr)
{
    int i;

    portInitPtr->maxRxPktSize = mii_managment.buffer_size/*RX_BUFFER_DEFAULT_SIZE*/;
    portInitPtr->rxDefQ = MII_DEF_RX_Q;

    for (i = 0; i < MII_MAX_TX_QUEUE_NUM; i++)
        portInitPtr->txDescrNum[i]    = mii_managment.num_of_tx_desc[i];

    for (i = 0; i < MII_MAX_RX_QUEUE_NUM; i++)
        portInitPtr->rxDescrNum[i]    = mii_managment.num_of_rx_desc[i];

    portInitPtr->osHandle = NULL;
}

MV_PKT_INFO* miiInfRxFill ( MV_U8 *buf )
{
  MV_PKT_INFO *pPktInfo;
  MV_BUF_INFO *pBufInfo;

  pPktInfo = osMalloc ( sizeof ( MV_PKT_INFO ) );
  if ( pPktInfo == NULL )
  {
    mvOsPrintf ( "Error: cannot allocate memory for pktInfo\n" );
    osFree ( buf );
    return NULL;
  }

  pBufInfo = osMalloc ( sizeof ( MV_BUF_INFO ) );
  if ( pBufInfo == NULL )
  {
    mvOsPrintf ( "Error: cannot allocate memory for pBufInfo\n" );
    return NULL;
  }


  pBufInfo->bufPhysAddr = _mvOsIoVirtToPhy ( NULL, buf );
  pBufInfo->bufVirtPtr = buf;
  pBufInfo->bufSize = mii_managment.buffer_size/*RX_BUFFER_DEFAULT_SIZE*/;
  pBufInfo->dataSize = 0;
  pPktInfo->osInfo = ( MV_ULONG ) buf;
  pPktInfo->pFrags = pBufInfo;
  pPktInfo->pktSize = mii_managment.buffer_size/*RX_BUFFER_DEFAULT_SIZE*/; /* how much to invalidate */
  pPktInfo->numFrags = 1;
  pPktInfo->status = 0;
  pPktInfo->srcIdx = -1;

  return pPktInfo;
}

void miiInfCpuPortConfig ( void )
{
  /* set KW MPP  */
  /* KW_MPP[45:34] is GPIO (by default - 0x0) */
  /* MPP Control2 register in KW */
  MV_REG_WRITE ( 0x10008,0x33331100 );
  MV_REG_WRITE ( 0x1000C,0x33333333 );
  MV_REG_WRITE ( 0x10010,0x33 );

  /* Set GPIO_OE[46] = 1 */
  MV_REG_WRITE ( 0x10144,0x4000 );

  /* Unicast Promiscuous mode - to enable the recieving of unknown unicast for GBE 1 */
  MV_REG_BIT_SET(0x76400, 0x1);

  /* Disable Marvell header */
  MV_REG_WRITE ( 0x76454,0x0 );

  /* Config DFOMT, DFSMT and DFUT to accept mode */
  mvEthRxFilterModeSet(glbMiiPortHandle,MV_TRUE);
}

MV_STATUS dump_segments(
               IN MV_U8     * segmentList[],
               IN MV_U32      segmentLen[],
               IN MV_U32      segmentNum,
               IN MV_U32      rxQueueNum)
{  
  int i;

  for (i = 0; i < segmentNum; i++)
  {
    printk(">>> seg[%d]= 0x%x (%d), data= ... \n", (int)i,
           (int)segmentList[i], (int)segmentLen[i]);
    gc_dump((char *)segmentList[i], segmentLen[i]);
  }
  printk("\n");
  return MV_OK;
}

int mgiInfRxReady(void)
{
  MV_PKT_INFO *pktInfo;
  int         pktCounter = 0;
  int         rxQueue;
  /* int      int_save; */
  int         something_to_process = 1;

  /* While there are RFDs to process */
  while (something_to_process)
  {
    something_to_process = 0;

    for (rxQueue = 0; rxQueue < ETH_PORT_RX_QUEUE_NUM; rxQueue++)
    {
      /* check which queues are waiting for handling */
      if (!glbMiiEthPortRxQueueEnabled[rxQueue])
        continue;

      /* Get the packet from device */
      /* int_save = intLock(); */
      pktInfo = mvEthPortRx(glbMiiPortHandle, rxQueue);
      /* intUnlock(int_save); */
      if (pktInfo == NULL)
      {
        DB(mvOsPrintf( "no more work\n" ));
        continue;
      }

      something_to_process = 1;

      /* check rx error status */
      if (pktInfo->status & (ETH_ERROR_SUMMARY_MASK))
      {
        MV_U32 err = pktInfo->status & ETH_RX_ERROR_CODE_MASK;
        /*DB( printf( "bad rx status %08x, ", (MV_U32)pktInfo->cmdSts ) );*/
        if (err == ETH_RX_RESOURCE_ERROR)
					DB( mvOsPrintf( "(resource error)\n" ) );
        else if (err == ETH_RX_MAX_FRAME_LEN_ERROR)
					DB( mvOsPrintf( "(max frame length error)\n" ) );
        else if (err == ETH_RX_OVERRUN_ERROR)
					DB( mvOsPrintf( "(overrun error)\n" ) );
        else if (err == ETH_RX_CRC_ERROR)
					DB( mvOsPrintf( "(crc error)\n" ) );
        else
        {
					DB( mvOsPrintf( "(unknown error)\n" ) );
        }
        continue;
      }
      else
      {
				DB( printf( "%s: calling NetRecieve pkInfo = 0x%x\n", __FUNCTION__, pktInfo) );
				DB( printf( "%s: calling NetRecieve osInfo = 0x%x\n", __FUNCTION__, pktInfo->osInfo) );
				DB( printf( "%s: calling NetRecieve pktSize = 0x%x\n", __FUNCTION__, pktInfo->pFrags->dataSize) );

        /* TODO support more than one segment */
        if (pktInfo->numFrags > 1)
        {
          mvOsPrintf("Only one fragment packets are suppoerted\n");   
        }

        DB(mvOsPrintf("userRxCallback != NULL\n"));

        if (userRxCallback != NULL)
        {
          //          printk(">>>> calling userRxCallBack, rxQueue=%d\n", rxQueue);
          
#ifdef DEBUG_PRINT
          dump_segments(&pktInfo->pFrags->bufVirtPtr,
                        &pktInfo->pFrags->dataSize,
                        pktInfo->numFrags,rxQueue);
#endif
          /* remove IP header alignment */
          pktInfo->pFrags->bufVirtPtr += 2;
          pktInfo->pFrags->dataSize -= 2;
          userRxCallback(&pktInfo->pFrags->bufVirtPtr, &pktInfo->pFrags->dataSize,
                         pktInfo->numFrags,rxQueue);
          pktCounter++;
        }
      }
    }                                   /* rxQueue */
  }                                       /* for: while(1) */

  return pktCounter;
}

/* Free all already sent buffers */
int mgiInfTxDone(void)
{
  MV_PKT_INFO *pktInfo;
  MV_U32      pktCounter;
  MV_U32      txQ;
  
  pktCounter = 0;
  for (txQ = 0; txQ < MII_MAX_TX_QUEUE_NUM; txQ++)
  {
    while (MV_TRUE)
    {
      /* clear descriptor */
      pktInfo = mvEthPortTxDone( glbMiiPortHandle, txQ);
      if (pktInfo != NULL)
      {
        DB( mvOsPrintf("mgiInfTxDone free packet, ptr: 0x%x, size: 0x%x\n", 
                       pktInfo->pFrags->bufVirtPtr, 
                       pktInfo->pFrags->dataSize));
        
        /* validate skb */
        if (pktInfo->osInfo != 0x44CAFE44)
        {
          mvOsPrintf( "error\n" );
          return MV_FAIL;
        }
        
        /* handle tx error */
        if (pktInfo->status & (ETH_ERROR_SUMMARY_BIT))
        {
          mvOsPrintf( "bad status (error)\n" );
          return MV_FAIL;
        }
      }
      else
        break;
      
      /* call mii tx complete handler */
      /* assumption by miiInf: only one desc is used per packet 
       * since mvEth driver only returns the last desc and miiInf 
       * doesn't hold tx buff chain !!!!! */
      miiInfBufferTxDone(pktInfo,txQ);
      
      pktCounter++;
    }
  }
  return pktCounter;
}
  

/* GLOBAL FUNCTIONS EXPORT */
EXPORT_SYMBOL ( gtEthernetPacketSend );
EXPORT_SYMBOL ( gtEthernetRxCallbackBind );
EXPORT_SYMBOL ( gtEthernetTxCompleteCallbackBind );
EXPORT_SYMBOL ( miiInfBufferRxDone );
EXPORT_SYMBOL ( gtEthernetRxPacketFree );
EXPORT_SYMBOL ( gtEthernetInterfaceEnable );
EXPORT_SYMBOL ( gtEthernetInterfaceDisable );
EXPORT_SYMBOL ( gtEthernetInterfaceRxInit );
EXPORT_SYMBOL ( gtEthernetInterfaceTxInit );
EXPORT_SYMBOL ( miiInfCpuPortConfig );
EXPORT_SYMBOL ( gc_dump );
