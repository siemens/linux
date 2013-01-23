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
Marvell Commercial License Option

If you received this File from Marvell and you have entered into a commercial
license agreement (a "Commercial License") with Marvell, the File is licensed
to you under the terms of the applicable Commercial License.

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

#include "sysConf/gtSysConf.h"
#include "sdma/gtNetIf.h"
#include "sdma/interNetIfTypes.h"
#include "hwIf/gtHwIf.h"
#include "common/macros/gtPresteraDefs.h"
#include "mvOsPrestera.h"
#include "mvOs.h"

#define MV_SYNC

/* Debug defines */
#if 0
#define MV_DEBUG
#endif
#ifdef MV_DEBUG
#define DB(x) x
#else
#define DB(x)
#endif


/*static MV_BOOL          enableAuProcces = MV_TRUE;*/
static MV_BOOL          enableRxProcces = MV_TRUE;

static MV_U32             numRxDataBufs;   /* These vars. are used to be   */
static MV_U32             numRxDescriptors;/* sent to Tx /Rx initialization*/
static MV_U32             numTxDescriptors;/* functions. and hold the      */
                                            /* number of the different descs*/
                                            /* and buffers allocated.       */

NET_CACHE_DMA_BLOCKS_INFO   netCacheDmaBlocks[SYS_CONF_MAX_DEV];

SDMA_INTERRUPT_CTRL         sdmaInterCtrl[SYS_CONF_MAX_DEV];

/****************************************************************************
 * Forward function declarations                                            *
 ****************************************************************************/

static MV_STATUS coreInitTx
(
    IN MV_U8  devNum,
    IN MV_U8  *descBlock,
    IN MV_U32 descBlockSize,
    OUT MV_U32 *numOfDescs
);

static MV_STATUS coreInitRx
(
    IN MV_U8  devNum,
    IN MV_U8  *rxDescBlock,
    IN MV_U32 rxDescBlockSize,
    IN MV_U8  *rxBufBlock,
    IN MV_U32 rxBufBlockSize,
    IN MV_U32 buffSize,
    OUT MV_U32 *numOfDescs,
    OUT MV_U32 *numOfBufs
);

/*******************************************************************************
* gtFreeRxBuf
*
* DESCRIPTION:
*       Frees a list of buffers, that where previously passed to the upper layer
*       in an Rx event.
*
* INPUTS:
*       rxBuffList  - List of Rx buffers to be freed.
*       buffListLen - Length of rxBufList.
*       devNum      - The device number throw which these buffers where
*                     received.
*       rxQueue     - The Rx queue number throw which these buffers where
*                     received.
*
* OUTPUTS:
*       None.
*
* RETURNS:
*       MV_OK on success, or
*       MV_FAIL otherwise.
*
* COMMENTS:
*       None.
*
*******************************************************************************/
MV_STATUS gtFreeRxBuf
(
    IN MV_U8    *rxBuffList[],
    IN MV_U32   buffListLen,
    IN MV_U8    devNum,
    IN MV_U8    rxQueue
)
{
    RX_DESC_LIST    *rxDescList;
    STRUCT_RX_DESC  *rxDesc;
    MV_U32          tmpData;
    MV_U32          i;

    rxDescList = &(sdmaInterCtrl[devNum].rxDescList[rxQueue]);

    /*
    osSemWait(rxDescList->rxListSem,OS_WAIT_FOREVER);
    */

    for(i = 0; i < buffListLen; i++)
    {
        rxDescList->freeDescNum++;
/* Haim osVirt2Phy((MV_U32)(rxBuffList[i]),&tmpData);*/
		
		/* Calc Physical address */ 
		tmpData = (MV_U32)rxBuffList[i] - 
			      (MV_U32)netCacheDmaBlocks[devNum].rxBufBlock +
				  (MV_U32)netCacheDmaBlocks[devNum].rxBufBlockPhy;
		
		
        tmpData = hwByteSwap(tmpData);
        rxDesc = rxDescList->next2Return->rxDesc;
        rxDesc->buffPointer = tmpData;

        MV_SYNC;

        RX_DESC_RESET(rxDescList->next2Return->rxDesc);

        MV_SYNC;

        /*osPrintf("Freed 0x%x.\n",rxDescList->next2Return->rxDesc);*/
        rxDescList->next2Return = rxDescList->next2Return->swNextDesc;
    }

    if(rxDescList->forbidQEn == MV_FALSE)
    {
        /* Enable the Rx SDMA   */
        CHECK_STATUS(hwIfGetReg(devNum, 0x2680, &tmpData));
        tmpData |= (1 << rxQueue);
        tmpData &= ~(1 << (rxQueue+8));
        CHECK_STATUS(hwIfSetReg(devNum, 0x2680, tmpData));
    }

    /*
    osSemSignal(rxDescList->rxListSem);
    */
    return MV_OK;
}

/*******************************************************************************
* gtInitSdmaNetIfDev
*
* DESCRIPTION:
*       Initialize the network interface structures, Rx descriptors & buffers
*       and Tx descriptors (For a single device).
*
* INPUTS:
*       devNum  - The device to initialize.
*
* OUTPUTS:
*       None.
*
* RETURNS:
*       MV_OK   - on success,
*       MV_FAIL - otherwise.
*
* COMMENTS:
*       None.
*
*******************************************************************************/
MV_STATUS gtInitSdmaNetIfDev
(
    IN  MV_U8       devNum,
    IN  MV_U32      auDescNum,
    IN  MV_U32      txDescNum,
    IN  MV_U32      rxDescNum,
    IN  MV_U32      rxBufSize
)
{
    static MV_BOOL     initDone[SYS_CONF_MAX_DEV] = {MV_FALSE};
    MV_U32 *handle;
    MV_ULONG *ptr;

    /* Allocate space for the Address update, Rx & Tx descriptors,  */
    /* and for the Rx buffers.                                      */

    if (initDone[devNum] == MV_TRUE) 
    {
        return MV_OK;
    }
    netCacheDmaBlocks[devNum].devNum = devNum;

    /* Au blocks size calc & malloc  */
    netCacheDmaBlocks[devNum].auDescBlockSize = sizeof(STRUCT_AU_DESC) * auDescNum;
    netCacheDmaBlocks[devNum].auDescBlock = 
        /*osCacheDmaMalloc(netCacheDmaBlocks[devNum].auDescBlockSize);*/
	osCacheDmaMalloc(NULL, 	netCacheDmaBlocks[devNum].auDescBlockSize, 
			       	(MV_ULONG*)(&netCacheDmaBlocks[devNum].auDescBlockPhy),
				(MV_U32 *)(&netCacheDmaBlocks[devNum].auDescMemHandle)); /*Haim*/
    if(netCacheDmaBlocks[devNum].auDescBlock == NULL)
        return MV_FAIL;
    osMemSet(netCacheDmaBlocks[devNum].auDescBlock,0,
             netCacheDmaBlocks[devNum].auDescBlockSize);
    
    netCacheDmaBlocks[devNum].auDescBlockSecond = 
        /*osCacheDmaMalloc(netCacheDmaBlocks[devNum].auDescBlockSize);*/
	osCacheDmaMalloc(NULL, 	netCacheDmaBlocks[devNum].auDescBlockSize, 
			       	(MV_ULONG*)(&netCacheDmaBlocks[devNum].auDescBlockSecondPhy),
				(MV_U32 *)(&netCacheDmaBlocks[devNum].auDescMemSecondHandle)); /*Haim*/
    if(netCacheDmaBlocks[devNum].auDescBlockSecond == NULL)
        return MV_FAIL;
    osMemSet(netCacheDmaBlocks[devNum].auDescBlockSecond,0,
             netCacheDmaBlocks[devNum].auDescBlockSize);

    /* FDB upload blocks size calc & malloc  */
    netCacheDmaBlocks[devNum].fuDescBlockSize = sizeof(STRUCT_AU_DESC) * auDescNum;
    netCacheDmaBlocks[devNum].fuDescBlock = 
        /*osCacheDmaMalloc(netCacheDmaBlocks[devNum].fuDescBlockSize);*/
	osCacheDmaMalloc(NULL, 	netCacheDmaBlocks[devNum].fuDescBlockSize, 
			       	(MV_ULONG*)(&netCacheDmaBlocks[devNum].fuDescBlockPhy),
				(MV_U32 *)(&netCacheDmaBlocks[devNum].fuDescMemHandle)); /*Haim*/
    if(netCacheDmaBlocks[devNum].fuDescBlock == NULL)
        return MV_FAIL;
    osMemSet(netCacheDmaBlocks[devNum].fuDescBlock,0,
             netCacheDmaBlocks[devNum].fuDescBlockSize);

    netCacheDmaBlocks[devNum].fuDescBlockSecond = 0;
 /*       osCacheDmaMalloc(netCacheDmaBlocks[devNum].fuDescBlockSize);
    if(netCacheDmaBlocks[devNum].fuDescBlockSecond == NULL)
        return MV_FAIL;
    osMemSet(netCacheDmaBlocks[devNum].fuDescBlockSecond,0,
             netCacheDmaBlocks[devNum].fuDescBlockSize);*/

    /* Tx block size calc & malloc  */
    netCacheDmaBlocks[devNum].txDescBlockSize = TX_DESC_SIZE * txDescNum;
    handle = (MV_U32 *)&(netCacheDmaBlocks[devNum].txDescMemHandle);
    ptr = (MV_ULONG*)(netCacheDmaBlocks[devNum].txDescBlockPhy);
    netCacheDmaBlocks[devNum].txDescBlock = 
        /* osCacheDmaMalloc(netCacheDmaBlocks[devNum].txDescBlockSize);*/
	(MV_U32 *)osCacheDmaMalloc(NULL, 	netCacheDmaBlocks[devNum].txDescBlockSize, ptr, handle); /*Haim*/
    if(netCacheDmaBlocks[devNum].txDescBlock == NULL)
        return MV_FAIL;

    osMemSet(netCacheDmaBlocks[devNum].txDescBlock,0,
             netCacheDmaBlocks[devNum].txDescBlockSize);

    /* Rx block size calc & malloc  */
    netCacheDmaBlocks[devNum].rxDescBlockSize = RX_DESC_SIZE * rxDescNum;
    handle = (MV_U32 *)&(netCacheDmaBlocks[devNum].rxDescMemHandle);
    ptr = (MV_ULONG*)(netCacheDmaBlocks[devNum].rxDescBlockPhy);
    netCacheDmaBlocks[devNum].rxDescBlock = 
        /*osCacheDmaMalloc(netCacheDmaBlocks[devNum].rxDescBlockSize);*/
	(MV_U32 *)osCacheDmaMalloc(NULL, 	netCacheDmaBlocks[devNum].rxDescBlockSize, ptr, handle); /*Haim*/
    if(netCacheDmaBlocks[devNum].rxDescBlock == NULL)
    {
        return MV_FAIL;
    }
    osMemSet(netCacheDmaBlocks[devNum].rxDescBlock,0,
             netCacheDmaBlocks[devNum].rxDescBlockSize);

	DB( printf("*** RX Desc Block alocation: Virtual: 0x%x	Physical: 0x%x	Size: 0x%x\n" ,
				 netCacheDmaBlocks[devNum].rxDescBlock,
				 netCacheDmaBlocks[devNum].rxDescBlockPhy,
				 netCacheDmaBlocks[devNum].rxDescBlockSize) );

    /* Rx buffer malloc */
    netCacheDmaBlocks[devNum].rxBufBlockSize = rxBufSize * rxDescNum;
    handle = (MV_U32 *)&(netCacheDmaBlocks[devNum].rxBufMemHandle);
    ptr = (MV_ULONG*)(netCacheDmaBlocks[devNum].rxBufBlockPhy);
    netCacheDmaBlocks[devNum].rxBufBlock = 
        /*osCacheDmaMalloc(rxBufSize * rxDescNum);*/
	(MV_U32 *)osCacheDmaMalloc(NULL, 	netCacheDmaBlocks[devNum].rxBufBlockSize, ptr, handle); /*Haim*/
    if(netCacheDmaBlocks[devNum].rxBufBlock == NULL)
    {
        return MV_FAIL;
    }   

	DB( printf("*** RX Buf  Block alocation: Virtual: 0x%x	Physical: 0x%x	Size: 0x%x\n" ,
				 netCacheDmaBlocks[devNum].rxBufBlock,
				 netCacheDmaBlocks[devNum].rxBufBlockPhy,
				 netCacheDmaBlocks[devNum].rxBufBlockSize) );
	

    /* init RX */
    CHECK_STATUS(coreInitRx(devNum,
                        (MV_U8*)netCacheDmaBlocks[devNum].rxDescBlock,
                        netCacheDmaBlocks[devNum].rxDescBlockSize,
                        (MV_U8*)netCacheDmaBlocks[devNum].rxBufBlock,
                        netCacheDmaBlocks[devNum].rxBufBlockSize,
                        rxBufSize, &numRxDescriptors, &numRxDataBufs));
#if 0
	printf("**** RX Desc Virtual  Base: 0x%x\n",netCacheDmaBlocks[devNum].rxDescBlock);
	printf("**** RX Desc Physical Base: 0x%x\n",netCacheDmaBlocks[devNum].rxDescBlockPhy);
	printf("**** RX Buf  Virtual  Base: 0x%x\n",netCacheDmaBlocks[devNum].rxBufBlock);
	printf("**** RX Buf  Physical Base: 0x%x\n",netCacheDmaBlocks[devNum].rxBufBlockPhy);
	printf("**** TX Desc Virtual  Base: 0x%x\n",netCacheDmaBlocks[devNum].txDescBlock);
	printf("**** TX Desc Physical Base: 0x%x\n",netCacheDmaBlocks[devNum].txDescBlockPhy);
#endif
    /* init TX */
    CHECK_STATUS(coreInitTx(devNum, 
                        (MV_U8*)netCacheDmaBlocks[devNum].txDescBlock,
                        netCacheDmaBlocks[devNum].txDescBlockSize,
                        &numTxDescriptors));

    initDone[devNum] = MV_TRUE;
    
    return MV_OK;
}

/*******************************************************************************
* coreTxRegConfig
*
* DESCRIPTION:
*       Set the needed values for SDMA registers to enable Tx activity.
*
* INPUTS:
*       devNum  - The Pp device number.
*
* OUTPUTS:
*       None.
*
* RETURNS:
*       MV_OK   - on success,
*       MV_FAIL - otherwise.
*
* COMMENTS:
*       None.
*
*******************************************************************************/
static MV_STATUS coreTxRegConfig
(
    IN MV_U8 devNum
)
{
    MV_U8 i;

    /* transmit queue WRR */
    for(i = 0; i < 8; i++)
    {
        hwIfSetReg(devNum, 0x2708 + i*0x10, 0);
        hwIfSetReg(devNum, 0x2700 + i*0x10, 0);
        hwIfSetReg(devNum, 0x2704 + i*0x10, 0xfffffcff);
    }

    /*********************/
    hwIfSetReg(devNum, 0x2874, 0xffffffc1);

    /*********************/
    hwIfSetReg(devNum, 0x2870, 0);

    return MV_OK;
}

/*******************************************************************************
* coreInitTx
*
* DESCRIPTION:
*       This function initializes the Core Tx module, by allocating the cyclic
*       Tx descriptors list, and the tx Headers buffers.
*
* INPUTS:
*       devNum      - The device number to init the Tx unit for.
*       descBlock   - A block to allocate the descriptors from.
*       descBlockSize - Size in bytes of descBlock.
*
* OUTPUTS:
*       numOfDescs  - Number of allocated Tx descriptors.
*
* RETURNS:
*       MV_OK on success, or
*       MV_FAIL otherwise.
*
* COMMENTS:
*       None.
*
*******************************************************************************/
static MV_STATUS coreInitTx
(
    IN MV_U8  devNum,
    IN MV_U8  *descBlock,
    IN MV_U32 descBlockSize,
    OUT MV_U32 *numOfDescs
)
{
    /*MV_8 semName[30];*/
    TX_DESC_LIST *txDescList;   /* Points to the relevant Tx desc. list */

    STRUCT_TX_DESC *firstTxDesc;/* Points to the first Tx desc in list. */
    MV_U8 txQueue;              /* Index of the Tx Queue.               */
    MV_U32 numOfTxDesc;         /* Number of Tx desc. that may be       */
                                /* allocated from the given block.      */
    MV_U32 sizeOfDesc;          /* The ammount of memory (in bytes) that*/
                                /* a single desc. will occupy, including*/
                                /* the alignment.                       */
    MV_U32 descPerQueue;        /* Number of descriptors per Tx Queue.  */
    MV_U32 i;

    STRUCT_SW_TX_DESC *swTxDesc = NULL;/* Points to the Sw Tx desc to   */
                                /* init.                                */
    STRUCT_SW_TX_DESC *firstSwTxDesc;/* Points to the first Sw Tx desc  */
                                /* in list.                             */
    MV_U32  phyNext2Feed;       /* The physical address of the next2Feed*/
                                /* field.                               */

    txDescList = sdmaInterCtrl[devNum].txDescList;

    /* Tx Descriptor list initialization.   */
    sizeOfDesc  = sizeof(STRUCT_TX_DESC);
    numOfTxDesc = descBlockSize / sizeOfDesc;
    if((sizeOfDesc % TX_DESC_ALIGN) != 0)
    {
        sizeOfDesc += (TX_DESC_ALIGN-(sizeof(STRUCT_TX_DESC) % TX_DESC_ALIGN));
        numOfTxDesc = (descBlockSize -
                       (TX_DESC_ALIGN -
                        (((MV_U32)descBlock) % TX_DESC_ALIGN))) / sizeOfDesc;
    }
	/* Set the descBlock to point to an alligned start address. */
	if(((MV_U32)descBlock % TX_DESC_ALIGN) != 0)
	{
		descBlock =
			(MV_U8*)((MV_U32)descBlock +
					 (TX_DESC_ALIGN - (((MV_U32)descBlock) % TX_DESC_ALIGN)));
	}

    descPerQueue = numOfTxDesc / NUM_OF_TX_QUEUES;
    /* Number of descriptors must devide by 2.  */
    descPerQueue -= (descPerQueue & 0x1);
    txDescList->sizeOfList = descPerQueue * sizeOfDesc;

    *numOfDescs = descPerQueue * NUM_OF_TX_QUEUES;

    for(txQueue = 0; txQueue < NUM_OF_TX_QUEUES; txQueue++)
    {
	/*
        osSprintf(semName,"txDescList%d-%d",devNum,txQueue);
        osSemBinCreate(semName,1,&(txDescList[txQueue].txListSem));
	*/

        txDescList[txQueue].freeDescNum = descPerQueue;

        firstTxDesc = (STRUCT_TX_DESC*)descBlock;

        firstSwTxDesc = (STRUCT_SW_TX_DESC*)osMalloc(sizeof(STRUCT_SW_TX_DESC) *
                                                     descPerQueue);

        if(firstSwTxDesc == NULL)
            return MV_FAIL;

        for(i = 0; i < descPerQueue; i++)
        {
            swTxDesc = firstSwTxDesc + i;

            swTxDesc->txDesc = (STRUCT_TX_DESC*)descBlock;
            descBlock   = (MV_U8*)((MV_U32)descBlock + sizeOfDesc);

            TX_DESC_RESET(swTxDesc->txDesc);

            if((descPerQueue - 1) != i)
            {
                /* Next descriptor should not be configured for the last one.*/
                swTxDesc->swNextDesc  = firstSwTxDesc + i + 1;
/* Haim         osVirt2Phy((MV_U32)descBlock,
                           ((MV_U32*)&(swTxDesc->txDesc->nextDescPointer)));*/
			
				/* Calc Physical address */ 
				swTxDesc->txDesc->nextDescPointer = (MV_U32)descBlock - 
					      (MV_U32)netCacheDmaBlocks[devNum].txDescBlock +
						  (MV_U32)netCacheDmaBlocks[devNum].txDescBlockPhy;

                swTxDesc->txDesc->nextDescPointer =
                    hwByteSwap(swTxDesc->txDesc->nextDescPointer);
            }

            /* Initilaize the aligment */
            swTxDesc->txBufferAligment = 0;
        }

        /* Close the cyclic desc. list. */
        swTxDesc->swNextDesc = firstSwTxDesc;
/* Haim osVirt2Phy((MV_U32)firstTxDesc,
                   ((MV_U32*)&(swTxDesc->txDesc->nextDescPointer)));*/

		/* Calc Phyiscal address */ 
		swTxDesc->txDesc->nextDescPointer = (MV_U32)firstTxDesc - 
					      (MV_U32)netCacheDmaBlocks[devNum].txDescBlock +
						  (MV_U32)netCacheDmaBlocks[devNum].txDescBlockPhy;

        swTxDesc->txDesc->nextDescPointer =
            hwByteSwap(swTxDesc->txDesc->nextDescPointer);

        txDescList[txQueue].next2Feed   = firstSwTxDesc;
        txDescList[txQueue].next2Free   = firstSwTxDesc;
    }

    for(i = 0; i < NUM_OF_TX_QUEUES; i++)
    {
/* Haim  osVirt2Phy((MV_U32)(txDescList[i].next2Feed->txDesc),&phyNext2Feed); */
		
		/* Calc Phyiscal address */ 
		phyNext2Feed = (MV_U32)txDescList[i].next2Feed->txDesc - 
					   (MV_U32)netCacheDmaBlocks[devNum].txDescBlock +
					   (MV_U32)netCacheDmaBlocks[devNum].txDescBlockPhy;


        CHECK_STATUS(hwIfSetReg(devNum, 0x26c0 + i*0x4, phyNext2Feed));
    }

    /* Posibile HW BTS: TxWordSwap and TxByteSwap are DISABLED  */
    CHECK_STATUS(hwIfSetMaskReg(devNum, 0x2800, (3 << 23), 0));

    coreTxRegConfig(devNum);

    return MV_OK;
}


/*******************************************************************************
* coreInitRx
*
* DESCRIPTION:
*       This function initializes the Core Rx module, by allocating the cyclic
*       Rx descriptors list, and the rx buffers.
*
* INPUTS:
*       devNum      - The device number to init the Rx unit for.
*       descBlock   - A block to allocate the descriptors from.
*       descBlockSize - Size in bytes of descBlock.
*       buffBlock   - A block to allocate the Rx buffers from.
*       buffBlockSize - Size in bytes of buffBlock.
*       headerOffset - The application required header offset to be kept before
*                      the Rx buffer.
*       buffSize    - Size of a single Rx buffer in list.
*
* OUTPUTS:
*       numOFDescs  - Number of Rx descriptors allocated.
*       numOfBufs   - Number of Rx buffers allocated.
*
* RETURNS:
*       MV_OK on success, or
*       MV_FAIL otherwise.
*
* COMMENTS:
*       None.
*
*******************************************************************************/
static MV_STATUS coreInitRx
(
    IN MV_U8    devNum,
    IN MV_U8   *descBlock,
    IN MV_U32   descBlockSize,
    IN MV_U8   *rxBufBlock,
    IN MV_U32   rxBufBlockSize,
    IN MV_U32   buffSize,
    OUT MV_U32 *numOfDescs,
    OUT MV_U32 *numOfBufs
)
{
    /*MV_8 semName[30];*/
    RX_DESC_LIST *rxDescList;   /* Points to the relevant Rx desc. list */
    STRUCT_RX_DESC *rxDesc = NULL; /* Points to the Rx desc to init.    */
    STRUCT_RX_DESC *firstRxDesc;/* Points to the first Rx desc in list. */
    MV_U8 rxQueue;              /* Index of the Rx Queue.               */
    MV_U32 numOfRxDesc;         /* Number of Rx desc. that may be       */
                                /* allocated from the given block.      */
    MV_U32 sizeOfDesc;          /* The ammount of memory (in bytes) that*/
                                /* a single desc. will occupy, including*/
                                /* the alignment.                       */
    MV_U32 descPerQueue;        /* Number of descriptors per Rx Queue.  */
    MV_U32 actualBuffSize;      /* Size of a single buffer, after taking*/
                                /* the required allignment into account.*/
    STRUCT_SW_RX_DESC *swRxDesc = NULL;/* Points to the Sw Rx desc to   */
                                /* init.                                */
    STRUCT_SW_RX_DESC *firstSwRxDesc;/* Points to the first Sw Rx desc  */
                                /* in list.                             */
    MV_U32 virtBuffAddr;        /* The virtual address of the Rx buffer */
                                /* To be enqueued into the current Rx   */
                                /* Descriptor.                          */
	MV_U32 phyBuffAddr;			/* The physical address of the Rx buffer*/
                                /* To be enqueued into the current Rx   */
                                /* Descriptor. 							*/
    MV_U32 phyAddr;             /* Physical Rx descriptor's address.    */

    MV_U32 i;

    rxDescList  = sdmaInterCtrl[devNum].rxDescList;

    /* Rx Descriptor list initialization.   */
    sizeOfDesc = sizeof(STRUCT_RX_DESC);

    if((sizeOfDesc % RX_DESC_ALIGN) != 0)
    {
        sizeOfDesc += (RX_DESC_ALIGN -(sizeof(STRUCT_RX_DESC) % RX_DESC_ALIGN));
    }

    /* The buffer size must be a multiple of RX_BUFF_SIZE_MULT  */
    actualBuffSize = buffSize - (buffSize % RX_BUFF_SIZE_MULT);

    /* Number of Rx descriptors is calculated according to the  */
    /* size of the given Rx Buffers block.                      */
    /* Take the "dead" block in head of the buffers block as a  */
    /* result of the allignment.                                */
    if(((MV_U32)descBlock % RX_DESC_ALIGN) != 0)
    {
        numOfRxDesc = (rxBufBlockSize -
                       ((MV_U32)descBlock %RX_DESC_ALIGN)) / actualBuffSize;
    }
    else
    {
        numOfRxDesc = rxBufBlockSize / actualBuffSize;
    }

    /* Set numOfRxDesc according to the number of descriptors that  */
    /* may be allocated from descBlock and the number of buffers    */
    /* that may be allocated from rxBufBlock.                        */
    if((descBlockSize / sizeOfDesc) < numOfRxDesc)
        numOfRxDesc = descBlockSize / sizeOfDesc;

    /* Set the descBlock to point to an alligned start address. */
    if(((MV_U32)descBlock % RX_DESC_ALIGN) != 0)
    {
        descBlock =
            (MV_U8*)((MV_U32)descBlock - (((MV_U32)descBlock) % RX_DESC_ALIGN));
    }

    descPerQueue = numOfRxDesc / NUM_OF_RX_QUEUES;

    *numOfDescs = descPerQueue * NUM_OF_RX_QUEUES;

    for(rxQueue = 0; rxQueue < NUM_OF_RX_QUEUES; rxQueue++)
    {
 	/* 
        osSprintf(semName,"rxDescList%d-%d",devNum,rxQueue);
        osSemBinCreate(semName,1,&(rxDescList[rxQueue].rxListSem));
	*/

        rxDescList[rxQueue].freeDescNum = descPerQueue;
        rxDescList[rxQueue].forbidQEn = MV_FALSE;

        /* store pointer to first RX hw desc */
        firstRxDesc = (STRUCT_RX_DESC*)descBlock;

        /* allocate block for sw rx descriptors */
        firstSwRxDesc = (STRUCT_SW_RX_DESC*)osMalloc(sizeof(STRUCT_SW_RX_DESC) *
                                                     descPerQueue);
        osMemSet(firstSwRxDesc,0,sizeof(STRUCT_SW_RX_DESC) * descPerQueue);

        if(firstSwRxDesc == NULL)
            return MV_FAIL;

        for(i = 0; i < descPerQueue; i++)
        {
            swRxDesc = firstSwRxDesc + i;

            rxDesc      = (STRUCT_RX_DESC*)descBlock;
            descBlock   = (MV_U8*)((MV_U32)descBlock + sizeOfDesc);
            swRxDesc->rxDesc = rxDesc;
            RX_DESC_RESET(swRxDesc->rxDesc);

            if((descPerQueue - 1) != i)
            {
                /* Next descriptor should not be configured for the last one.*/
                swRxDesc->swNextDesc  = (firstSwRxDesc + i + 1);

                /* config HW desc link list */
/* Haim         osVirt2Phy((MV_U32)descBlock,
                           ((MV_U32*)(&(swRxDesc->rxDesc->nextDescPointer))));*/

				/* Calc Phyiscal address */ 
				swRxDesc->rxDesc->nextDescPointer = (MV_U32)descBlock - 
			      (MV_U32)netCacheDmaBlocks[devNum].rxDescBlock +
				  (MV_U32)netCacheDmaBlocks[devNum].rxDescBlockPhy;

				DB( printf("*** RX Ring build: Virtual: 0x%x	Physical: 0x%x		\
							(Virtual base: 0x%x		Physical base: 0x%x)\n" ,
				 descBlock,
				 swRxDesc->rxDesc->nextDescPointer,
				 netCacheDmaBlocks[devNum].rxDescBlock,
				 netCacheDmaBlocks[devNum].rxDescBlockPhy) );


                swRxDesc->rxDesc->nextDescPointer =
                    hwByteSwap(swRxDesc->rxDesc->nextDescPointer);
            }
        }

        /* Close the cyclic desc. list. */
        swRxDesc->swNextDesc = firstSwRxDesc;
/* Haim osVirt2Phy((MV_U32)firstRxDesc,
                   ((MV_U32*)(&(swRxDesc->rxDesc->nextDescPointer))));*/

		/* Calc Phyiscal address */ 
		swRxDesc->rxDesc->nextDescPointer = (MV_U32)firstRxDesc - 
		      (MV_U32)netCacheDmaBlocks[devNum].rxDescBlock +
			  (MV_U32)netCacheDmaBlocks[devNum].rxDescBlockPhy;

        swRxDesc->rxDesc->nextDescPointer =
            hwByteSwap(swRxDesc->rxDesc->nextDescPointer);

        rxDescList[rxQueue].next2Receive    = firstSwRxDesc;
        rxDescList[rxQueue].next2Return     = firstSwRxDesc;
    }


    /* Rx Buffers initialization.           */

    /* Set the buffers block to point to a properly alligned block. */
    if(((MV_U32)rxBufBlock % RX_BUFF_ALIGN) != 0)
    {
        rxBufBlockSize = (rxBufBlockSize -
                         (RX_BUFF_ALIGN -
                          ((MV_U32)rxBufBlock % RX_BUFF_ALIGN)));

        rxBufBlock =
            (MV_U8*)((MV_U32)rxBufBlock +
                     (RX_BUFF_ALIGN - ((MV_U32)rxBufBlock % RX_BUFF_ALIGN)));
    }

    /* Check if the given buffers block, is large enough to be cut  */
    /* into the needed number of buffers.                           */
    if((rxBufBlockSize / (descPerQueue * NUM_OF_RX_QUEUES)) <
       RX_BUFF_SIZE_MULT)
        return MV_FAIL;

    *numOfBufs = descPerQueue * NUM_OF_RX_QUEUES;
    for(rxQueue = 0; rxQueue < NUM_OF_RX_QUEUES; rxQueue++)
    {
        swRxDesc = rxDescList[rxQueue].next2Receive;

        for(i = 0; i < descPerQueue; i++)
        {
            RX_DESC_SET_BUFF_SIZE_FIELD(swRxDesc->rxDesc, actualBuffSize);

            /* Set the Rx desc. buff pointer field. */
            virtBuffAddr = (MV_U32)rxBufBlock;
			phyBuffAddr = (MV_U32)rxBufBlock - 
						  (MV_U32)netCacheDmaBlocks[devNum].rxBufBlock +
						  (MV_U32)netCacheDmaBlocks[devNum].rxBufBlockPhy;
			
/* Haim 	osVirt2Phy(virtBuffAddr,
                       ((MV_U32*)(&(swRxDesc->rxDesc->buffPointer))));*/

			/* Calc Physical address */ 
			swRxDesc->rxDesc->buffPointer = (MV_U32)rxBufBlock - 
				(MV_U32)netCacheDmaBlocks[devNum].rxBufBlock +
				(MV_U32)netCacheDmaBlocks[devNum].rxBufBlockPhy;

			swRxDesc->rxDesc->buffPointer =
                hwByteSwap(swRxDesc->rxDesc->buffPointer);

			/* Haim - save the virtual address */
			swRxDesc->buffVirtPointer = virtBuffAddr;//hwByteSwap(virtBuffAddr);

            rxBufBlock = (MV_U8*)(((MV_U32)rxBufBlock) + actualBuffSize);

            /* Set the buffers block to point to a properly alligned block*/
            if(((MV_U32)rxBufBlock % RX_BUFF_ALIGN) != 0)
            {
                rxBufBlock =
                    (MV_U8*)((MV_U32)rxBufBlock +
                             (RX_BUFF_ALIGN -
                              ((MV_U32)rxBufBlock % RX_BUFF_ALIGN)));
            }

            swRxDesc = swRxDesc->swNextDesc;
        }
    }


    for(i = 0; i < NUM_OF_RX_QUEUES; i++)
    {
        MV_U32  tmpData;
/* Haim osVirt2Phy((MV_U32)(rxDescList[i].next2Receive->rxDesc),
                   &phyAddr);*/
			
		/* Calc Physical address */ 
		phyAddr = (MV_U32)rxDescList[i].next2Receive->rxDesc - 
				  (MV_U32)netCacheDmaBlocks[devNum].rxDescBlock +
				  (MV_U32)netCacheDmaBlocks[devNum].rxDescBlockPhy;

        CHECK_STATUS(hwIfSetReg(devNum, 0x260c + i*0x10, phyAddr));
        
        /* Enable Rx DMA    */
        CHECK_STATUS(hwIfGetReg(devNum, 0x2680, &tmpData));
        tmpData |= (1 << i);
        tmpData &= ~(1 << (i + 8));
        CHECK_STATUS(hwIfSetReg(devNum, 0x2680, tmpData));
    }

    /* Set the Receive Interrupt Frame Boundaries   */
    CHECK_STATUS(hwIfSetMaskReg(devNum, 0x2800, 1, 1));

    /* Posibile HW BTS: RxWordSwap and RxByteSwap are ENABLED  */
    CHECK_STATUS(hwIfSetMaskReg(devNum, 0x2800, 0xC0, 0xC0));
    
    return MV_OK;
}

/*******************************************************************************
* gtNetEnableRxProcess
*
* DESCRIPTION: 
*   Enable the RX process. 
*
* INPUTS:
*       None.
*
* OUTPUTS:
*       None.
*
* RETURNS :
*       MV_OK        - Operation succeede
*
* COMMENTS:
*
*
*******************************************************************************/
MV_STATUS gtNetEnableRxProcess
(
    MV_VOID
)
{
    enableRxProcces = MV_TRUE;

    return (MV_OK);
}

/*******************************************************************************
* gtNetGetEnableRxProcess
*
* DESCRIPTION: 
*   Get the status of Enable the RX process. 
*
* INPUTS:
*       None.
*
* OUTPUTS:
*       MV_TRUE  - Enable RX process
*       MV_FALSE - Disable RX process 
*
* RETURNS :
*       None
*
* COMMENTS:
*
*
*******************************************************************************/
MV_BOOL gtNetGetEnableRxProcess
(
    MV_VOID
)
{
    return (enableRxProcces);
}
