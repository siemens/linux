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
                                             

/********* Include files ******************************************************/
#include "mvNetReceive.h"
#include "common/mvGenTypes.h"
#include "common/macros/mvCommonFuncs.h"
#include "sdma/interNetIfTypes.h"
#include "sdma/mvNetIf.h"
#include "hwIf/mvHwIf.h"
#include "mvOsPrestera.h"

MV_BOOL     syncAuqPointer = MV_FALSE;  /* Indicates whether a              */
                                        /* syncronization for the AUQ       */
                                        /* pointer should be done on the    */
                                        /* first AU_PENDING interrupt.      */

extern SDMA_INTERRUPT_CTRL         sdmaInterCtrl[32];

MV_U32  numOfRxPckts = 0;
MV_U32  numOfAuMsg = 0;

#define MV_SYNC

#if 0
#define MV_DEBUG 
#endif
#ifdef MV_DEBUG
#define DB(x) x
#else
#define DB(x)
#endif


/*******************************************************************************
* mvSwitchRxStart
*
* DESCRIPTION:
*       This function handles packet receive interrupts.
*
* INPUTS:
*       devNum	    - Device number	
*       queueIdx    - Q ID	
*
* OUTPUTS:
*       pktInfo     - Packet Data
*
* RETURNS:
*       MV_REDO - if more interrupts of this type need to be handled,
*       MV_OK   - Otherwise.
*
* COMMENTS:
*       None.
*
*******************************************************************************/
MV_STATUS mvSwitchRxStart
(
    IN MV_U8        devNum,
    IN MV_U8        queueIdx,
    MV_PKT_INFO 	*pktInfo /* MV_RX_PKTS_INFO *rxPcktsInfo */
)
{
    RX_DESC_LIST        *rxDescList;
    STRUCT_SW_RX_DESC   *descPtr;
    MV_BOOL             enableProcessRxMsg;
    STRUCT_SW_RX_DESC   *firstRxDesc = NULL;/* Points to the first desc to*/
                                    /* be sent to the callback function.*/
    MV_U32 descNum;                 /* Number of descriptors this packet*/
                                    /* occupies to be sent to the       */
                                    /* callback function.               */


    rxDescList = &(sdmaInterCtrl[devNum].rxDescList[queueIdx]);

/*
    osSemWait(rxDescList->rxListSem,OS_WAIT_FOREVER);
*/
   	DB( printf( "%s - Start\n", __FUNCTION__) );
   
    /* Check if process of RX messages is enabled.  */
    enableProcessRxMsg = mvNetGetEnableRxProcess();

    if(rxDescList->freeDescNum == 0)
    {
        /* osSemSignal(rxDescList->rxListSem); */
        return MV_OK;
    }

    descPtr = rxDescList->next2Receive;

    descPtr->shadowRxDesc.word1 = hwByteSwap(descPtr->rxDesc->word1);
    descPtr->shadowRxDesc.word2 = hwByteSwap(descPtr->rxDesc->word2);
    
	/* No more Packets to process, return   */
    if(RX_DESC_GET_OWN_BIT(&(descPtr->shadowRxDesc)) != MV_OWNERSHIP_CPU)
    {
        /* osSemSignal(rxDescList->rxListSem); */
		PRESTERA_DESCR_INV(NULL, (descPtr->rxDesc));
        return MV_OK;
    }

    if(RX_DESC_GET_FIRST_BIT(&(descPtr->shadowRxDesc)) == 0x0)
    {
        /* osSemSignal(rxDescList->rxListSem);*/
        return MV_ERROR;
    }

    descNum     = 1;
    firstRxDesc = descPtr;

    if(RX_DESC_GET_REC_ERR_BIT(&(descPtr->shadowRxDesc)) == 1)
    {
        /* osSemSignal(rxDescList->rxListSem); */
        return MV_OK;
    }

    /* Get the packet's descriptors.        */
    while(RX_DESC_GET_LAST_BIT(&(descPtr->shadowRxDesc)) == 0)
    {
        descPtr     = descPtr->swNextDesc;
        descPtr->shadowRxDesc.word1 = hwByteSwap(descPtr->rxDesc->word1);
        descPtr->shadowRxDesc.word2 = hwByteSwap(descPtr->rxDesc->word2);
        descNum++;

	    /* If enable RX process if disabled then the descriptor is cleared */
        if (enableProcessRxMsg == MV_FALSE)
        {
            MV_SYNC;
            RX_DESC_RESET(descPtr->rxDesc);
            MV_SYNC;
        }

        if(RX_DESC_GET_REC_ERR_BIT(&(descPtr->shadowRxDesc)) == 1)
        {
            /* osSemSignal(rxDescList->rxListSem); */
            return MV_OK;
        }
    }

    /* If enable RX process if disabled then the descriptor is cleared */
    if (enableProcessRxMsg == MV_FALSE)
    {
        MV_SYNC;
        RX_DESC_RESET(rxDescList->next2Return->rxDesc);
        MV_SYNC;

        /* Updating the next to free pointer */
        rxDescList->next2Return = descPtr->swNextDesc;
    }

    rxDescList->next2Receive = descPtr->swNextDesc;

    if (enableProcessRxMsg == MV_TRUE)
    {
        rxDescList->freeDescNum  -= descNum;
    }

    /* osSemSignal(rxDescList->rxListSem); */

    if( (enableProcessRxMsg == MV_TRUE))
    {
        mvReceivePacket(devNum,queueIdx,firstRxDesc,descNum,pktInfo/*rxPcktsInfo*/);
    }

    return MV_PRESTERA_REDO;
}

/*******************************************************************************
* mvReceivePacket
*
* DESCRIPTION:
*       This function receives a packet descriptor from the Rx interrupt handler
*       and passes it to the Tapi Rx handler (tapiIntRxHandler()).
*
* INPUTS:
*       devNum	    - Device number	
*       queueIdx    - Q ID	
*       swRxDesc    - A pointer to the first Sw Rx desc. of the packet.
*       descNum     - Number of descriptors (buffers) this packet occupies.
*
* OUTPUTS:
*       pktInfo     - Packet Data
*
* RETURNS:
*       MV_OK if successful, or
*       MV_FAIL otherwise.
*
* COMMENTS:
*       None.
*
*******************************************************************************/
MV_STATUS mvReceivePacket
(
    IN MV_U8                devNum,
    IN MV_U8                queueIdx,
    IN STRUCT_SW_RX_DESC    *swRxDesc,
    IN MV_U32               descNum,
    MV_PKT_INFO 	    	*pktInfo /* MV_RX_PKTS_INFO *rxPcktsInfo */
)
{
    /*MV_U8 *buffList[RX_MAX_PACKET_DESC];*//*The received packet buffer*/
                                            /* list.                    */
    MV_U32  buffLen[RX_MAX_PACKET_DESC];    /* List of buffers length   */
                                            /* for buffList.            */
    MV_U32  packetLen;                      /* Length of packet in bytes*/
    MV_U32  bufferSize;                     /* Size of a single buffer. */
    MV_U32  numOfBufs;                      /* Real number of buffers   */
    MV_BOOL isLast;                         /* Last descriptor          */
    /*MV_INT_CAUSE    intCause;*/
    /* MV_U8   *tmpBuffPtr; */
    /* MV_U32  temp; */
    MV_U32  i;
    MV_RX_DESC_DATA rxDesc;      /* RX descriptor data          */
    MV_BOOL         rxExtDataAdded;
	MV_U8	*lastBuf;

   	DB( printf( "%s - Start\n", __FUNCTION__) );

    /* Filling the unit and device the interrupt came on  */
    rxDesc.intDev  = devNum;
    
    /* Filling the descriptor parameters that are vaild only in the first     */
    /* descriptor in the packet                                               */
    rxDesc.word1 = swRxDesc->shadowRxDesc.word1;
    rxDesc.word2 = swRxDesc->shadowRxDesc.word2;

    rxDesc.rxDesc.busError = RX_DESC_GET_BUS_ERR_BIT(&(swRxDesc->shadowRxDesc));
    rxDesc.rxDesc.byteCount = 
                        RX_DESC_GET_BYTE_COUNT_FIELD(&(swRxDesc->shadowRxDesc)); 
    rxDesc.rxDesc.resourceError = 
                        RX_DESC_GET_REC_ERR_BIT(&(swRxDesc->shadowRxDesc));
    rxDesc.rxDesc.validCrc = 
    (RX_DESC_GET_VALID_CRC_BIT(&(swRxDesc->shadowRxDesc)) ? MV_FALSE : MV_TRUE);
      
    /* Clearing the number of buffers in the packet */
    numOfBufs   = 0;

    /* Check that the number of descriptors in this     */
    /* packet is not greater than RX_MAX_PACKET_DESC.   */
    if(descNum > RX_MAX_PACKET_DESC)
    {
        /* Free all buffers */
        for(i = 0; i < descNum; i++)
        {
            /* osPhy2Virt(hwByteSwap(swRxDesc->rxDesc->buffPointer),
                       (MV_U32*)(&tmpBuffPtr)); 
            mvFreeRxBuf(&(tmpBuffPtr),1,rxDesc.intDev,queueIdx); */
            mvFreeRxBuf((MV_U8**)&swRxDesc->buffVirtPointer,1,rxDesc.intDev,queueIdx);

            swRxDesc  = swRxDesc->swNextDesc;
        }
        return MV_OK;
    }

    packetLen  = rxDesc.rxDesc.byteCount;
    
    /* Setting the extended data to be invalid */
    /*rxDesc.extDataValid = MV_FALSE;*/

    /* Get the SDMA mode. If there were added 2 words of extended RX data */
    /*rc = coreGetRxSdmaMode(&rxExtDataAdded);*/
    rxExtDataAdded = MV_FALSE;
    
    /* Gathering all the data from all the descriptors */
    for(i = 0; i < descNum; i++)
    {
        /* Get the buffer size of the current descriptor */
        bufferSize = RX_DESC_GET_BUFF_SIZE_FIELD_NO_SWAP(&(swRxDesc->shadowRxDesc));

        /* Checking if this is the last descriptor */
        isLast = ((i == (descNum - 1)) ? MV_TRUE : MV_FALSE);

        /* Set the length of each buffer */
        if(packetLen > bufferSize)
        {
            buffLen[i] = bufferSize;
        }
        else
        {
            buffLen[i] = packetLen;
        }
        
        /* Descrementing the packet's length */
        packetLen -= buffLen[i];

        /* Setting the data of the buffer */
/*      temp = swRxDesc->rxDesc->buffPointer;
        osPhy2Virt(hwByteSwap(temp),(MV_U32*)(&(buffList[i])));
		buffList[i] =  swRxDesc->buffVirtPointer; */

		/* Update packet structure */
		pktInfo->pFrags->bufPhysAddr = hwByteSwap(swRxDesc->rxDesc->buffPointer);
		pktInfo->pFrags[i].bufVirtPtr = (MV_U8*)swRxDesc->buffVirtPointer;
		pktInfo->pFrags[i].bufSize = buffLen[i];

		lastBuf = (MV_U8*)swRxDesc->buffVirtPointer;

       /* If this is the last buffer then we get the extended data  */        
        
        numOfBufs++;
        swRxDesc  = swRxDesc->swNextDesc;
    }

    /* If the buffer length of the last descriptor in 0 then it is freed */
 	if(buffLen[descNum - 1] == 0)
    {
		if (mvFreeRxBuf(&lastBuf/*&(buffList[descNum - 1])*/,1,rxDesc.intDev,
                    queueIdx) != MV_OK);
		{
            return MV_FAIL;
        }

		numOfBufs--;
	}
	
	return MV_OK;
}

