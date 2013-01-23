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

#include "sdma/gtNetIf.h"
#include "sdma/interNetIfTypes.h"
#include "mvOsPrestera.h"
#include "mvOs.h"

#define MV_SYNC

/*******************************************************************************
* External usage environment parameters
*******************************************************************************/
extern SDMA_INTERRUPT_CTRL         sdmaInterCtrl[32];

/*******************************************************************************
* Internal usage environment parameters
*******************************************************************************/
static MV_BOOL sdmaTxActicvate = MV_TRUE;


#if 0
#define MV_DEBUG 
#endif
#ifdef MV_DEBUG
#define DB(x) x
#else
#define DB(x)
#endif


/*******************************************************************************
* mvSwitchTxPcktCfg
*
* DESCRIPTION:
*       This function receives packet buffers & parameters and prepares them 
*       for the transmit operation, and
*       enqueues the prepared descriptors to the PP's tx queues.
*
* INPUTS:
* 	devNum	    - Device number
*	txQueue     - TX Q ID
*       recalcCrc   - Whether to calculate CRC.
*       buffList    - The packet data buffers list.
*       phyBuffList - The packet physical data buffers list.
*       buffLen     - A list of the buffers len in buffList.
*       numOfBufs   - Length of buffList.
*       getTxSem    - Whether to get the Tx semaphore or not.
*
* OUTPUTS:
*       None.
*
* RETURNS:
*       GT_OK on success, or
*       GT_NO_RESOURCE if there is not enough desc. for enqueuing the packet, or
*       GT_FAIL otherwise.
*
* COMMENTS:
*
*******************************************************************************/

MV_STATUS mvSwitchTxPcktCfg
(
    IN MV_U8                devNum,
    IN MV_U8                txQueue,
    IN MV_BOOL              recalcCrc,
    IN MV_U8                *buffList[],
    IN MV_U8                *phyBuffList[],
    IN MV_U32               *buffLen,
    IN MV_U32               numOfBufs,
    IN MV_BOOL              getTxSem
)
{
    TX_DESC_LIST *txDescList;   /* The Tx desc. list control structure. */
    STRUCT_SW_TX_DESC *currSwDesc;  /* The current handled descriptor of*/
                                /* the currently transmitted packet.    */
    STRUCT_TX_DESC  tmpDesc;  /* Temporary Tx descriptor              */
    STRUCT_TX_DESC  tmpFirstTxDesc={0}; /* Temporary Tx descriptor used for */
                                /* preparing the real first descriptor  */
                                /* data.                                */
    STRUCT_TX_DESC *firstDesc;  /* The first descriptor of the          */
                                /* currently transmitted packet.        */
    
    MV_U32  descWord1;          /* The first word of the Tx descriptor. */
    /*MV_U32  tmpBuffPtr;*/         /* Holds the real buffer pointer.       */
    MV_U32          i;


   	DB( printf( "%s - Start\n", __FUNCTION__) );
    
	if (txQueue > 7)
        return (MV_BAD_PARAM);

    txDescList  = &(sdmaInterCtrl[devNum].txDescList[txQueue]);
    descWord1 = 0;

    /*
    if(getTxSem == MV_TRUE)
    {
        osSemWait(txDescList->txListSem,OS_WAIT_FOREVER);
    }
    */
   
    if(numOfBufs  > txDescList->freeDescNum)
    {
	/*
        if(getTxSem == MV_TRUE)
        {
            osSemSignal(txDescList->txListSem);
        }
	*/
        return MV_NO_RESOURCE;
    }

    if (recalcCrc == MV_TRUE)
    {
        descWord1 |= (1 << 12);
    }

    currSwDesc = txDescList->next2Feed;

    if (currSwDesc->txDesc->nextDescPointer == (MV_U32)txDescList->next2Free->txDesc)
    {
        return MV_NO_RESOURCE;
    }
    firstDesc  = currSwDesc->txDesc;

    /*cookie->numOfBuf = numOfBufs;*/
    
    /* build descriotors list */
    for(i = 0; i < numOfBufs; i++)
    {
        /* Check if the buffers length is larger than (TX_SHORT_BUFF_SIZE)  */
        if(buffLen[i] <= TX_SHORT_BUFF_SIZE)
        {
            return MV_FAIL;
        }
        /* get next descriptor */
        /*currSwDesc->userData = cookie;*/
        
        TX_DESC_RESET(&tmpDesc);
        tmpDesc.word1 = descWord1;

/*Haim  osVirt2Phy((MV_U32)(buffList[i]),&tmpBuffPtr);
        tmpDesc.buffPointer = hwByteSwap(tmpBuffPtr); */
        tmpDesc.buffPointer = hwByteSwap((MV_U32)phyBuffList[i]);
		
        TX_DESC_SET_BYTE_CNT(&tmpDesc,buffLen[i]);
        
        /* in case first or last descriptor don't swap */
        if (i == 0)
        {
            /* store first descriptor */
            TX_DESC_COPY(&tmpFirstTxDesc, &tmpDesc);
        }
        else if (i != (numOfBufs - 1))
        {
            TX_DESC_SET_OWN_BIT(&tmpDesc,MV_OWNERSHIP_DMA);
            tmpDesc.word1 = hwByteSwap(tmpDesc.word1);
            tmpDesc.word2 = hwByteSwap(tmpDesc.word2);
            TX_DESC_COPY(currSwDesc->txDesc,&tmpDesc);
            currSwDesc = currSwDesc->swNextDesc;
        }
    }

    /* Set the LAST desc params.        */
    if (currSwDesc->txDesc != firstDesc)
    {
        TX_DESC_SET_LAST_BIT(&tmpDesc,1);
        TX_DESC_SET_INT_BIT(&tmpDesc,1);
        tmpDesc.word1 = hwByteSwap(tmpDesc.word1);
        tmpDesc.word2 = hwByteSwap(tmpDesc.word2);
        TX_DESC_COPY(currSwDesc->txDesc,&tmpDesc);
    }
    else
    {
        TX_DESC_SET_LAST_BIT(&tmpFirstTxDesc,1);
        TX_DESC_SET_INT_BIT(&tmpFirstTxDesc,1);
    }

    txDescList->freeDescNum -= numOfBufs;
    txDescList->next2Feed    = currSwDesc->swNextDesc;

    /* Make sure that all previous operations where */
    /* executed before changing the own bit of the  */
    /* first desc.                                  */
    MV_SYNC;

    /* Set the FIRST descriptor own bit to start transmitting.  */
    TX_DESC_SET_FIRST_BIT(&tmpFirstTxDesc,1);
    TX_DESC_SET_OWN_BIT(&tmpFirstTxDesc,MV_OWNERSHIP_DMA);
    tmpFirstTxDesc.word1    = hwByteSwap(tmpFirstTxDesc.word1);
    tmpFirstTxDesc.word2    = hwByteSwap(tmpFirstTxDesc.word2);
    TX_DESC_COPY(firstDesc,&tmpFirstTxDesc);
    
	/* The Enable DMA operation should be done only */
    /* AFTER all desc. operations where completed.  */
    MV_SYNC;

    if(sdmaTxActicvate == MV_TRUE)
    {
        /* Enable the Tx DMA.   */
        CHECK_STATUS(dbTxDmaSetMode(devNum, txQueue, MV_TRUE));      
    }

    /*
    if(getTxSem == MV_TRUE)
    {
        osSemSignal(txDescList->txListSem);
    }
    */
    
	return MV_OK;
}


/*******************************************************************************
* mvSwitchTxDone
*
* DESCRIPTION:
*       This function resets TX descriptor and updates the 
*       TX descriptors linked list.
*
* INPUTS:
*       devNum      - The device number from which the packet was sent.
*       txQueue     - The Tx queue index from which the packet was sent.
*       descNum     - Number of descriptors (buffers) this packet occupies.
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
MV_STATUS mvSwitchTxDone
(
	IN MV_U8                devNum,
    	IN MV_U8                txQueue,
    	IN MV_U32               descNum
)
{
    MV_STATUS status = MV_OK;
    TX_DESC_LIST *txDescList;
    STRUCT_SW_TX_DESC  *swTxDesc;
    MV_U32  freeDescNum;        /* Counts the number of descriptors     */
                                /* already freed.                       */
    /*MV_PTR  cookie;*/             /* Holds the cookie to be returned to   */
                                /* the application throw Tapi.          */
    /*MV_INT_CAUSE    intCause;*/ 
    MV_U8 ownerBit, count = 0;

    freeDescNum = descNum;

    txDescList = &(sdmaInterCtrl[devNum].txDescList[txQueue]);

    swTxDesc = txDescList->next2Free;

   	DB( printf( "%s - Start\n", __FUNCTION__) );

    /* osSemWait(txDescList->txListSem,OS_WAIT_FOREVER); */

    /*cookie = swTxDesc->userData;*/

    /* check if ownership was changed to CPU */
    /* wait for 10 mSec */
    while (1)
    { 
		udelay(1000);
		ownerBit = TX_DESC_GET_OWN_BIT(swTxDesc->txDesc);
		if(ownerBit == 0)
		{
			break;
		}
		
		if(count == 10)
		{
			status = MV_TIMEOUT;
			/* Disable the Tx DMA.   */
			dbTxDmaSetMode(devNum, txQueue, MV_FALSE); 
			break;
		}
		count++;
    }

    while(freeDescNum > 0)
    {
	/*ownerBit = TX_DESC_GET_OWN_BIT(swTxDesc->txDesc);
	if(ownerBit) 
	{
		return MV_FAIL;
	}*/
        swTxDesc->txDesc->word1 = 0x0;
        /*TX_DESC_SET_OWN_BIT(swTxDesc->txDesc,TX_DESC_CPU_OWN);*/

        swTxDesc = swTxDesc->swNextDesc;
        freeDescNum--;
    }

    txDescList->freeDescNum += descNum;
    txDescList->next2Free = swTxDesc;

    /* osSemSignal(txDescList->txListSem); */

    return status;
}


