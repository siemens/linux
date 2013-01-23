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

#include "mvCommon.h"
#include "common/mvSysConf.h"
#include "sdma/interNetIfTypes.h"
#include "sdma/mvNetIf.h"
#include "util/mvUtils.h"
#include "util/mvNetTransmit.h"
#include "hwIf/mvHwIf.h"
#include "mvOsPrestera.h"



/******************************** Globals *************************************/

#if 0
#define MV_DEBUG 
#endif
#ifdef MV_DEBUG
#define DB(x) x
#else
#define DB(x)
#endif


/********* Externals **********************************************************/
extern MV_STATUS mvSwitchTxPcktCfg
(
    IN MV_U8                devNum,
    IN MV_U8                txQueue,
    IN MV_BOOL              recalcCrc,
    IN MV_U8                *buffList[],
    IN MV_U8                *phyBuffList[],
    IN MV_U32               *buffLen,
    IN MV_U32               numOfBufs,
    IN MV_BOOL              getTxSem
);

extern MV_STATUS mvSwitchTxDone
(
	IN MV_U8                devNum,
    	IN MV_U8                txQueue,
    	IN MV_U32               descNum
);

/********* Internal functions *************************************************/

static MV_U32 calcFromCpuDsaTagWord0(MV_U8 devNum, MV_U32 portNum);
static MV_U32 calcFromCpuDsaTagWord1(MV_U32 portNum);

/*******************************************************************************
* mvSwitchTxStart
*
* DESCRIPTION:
*       Starts transmition of packets
*
* INPUTS:
*       None
*
* OUTPUTS:
*       None
*
* RETURNS:
*       MV_OK   - on success
*       MV_FAIL - on error
*
* COMMENTS:
*
* GalTis:
*       Command     - cmdTxStart
*
*******************************************************************************/
MV_STATUS mvSwitchTxStart(MV_PKT_DESC *packetDesc)
{
   	MV_STATUS status;

   	DB( printf( "%s - Start\n", __FUNCTION__) );

	status = mvSwitchTxPcktCfg(
                    packetDesc->mrvlTag.swDsa.srcDev,
                    packetDesc->mrvlTag.swDsa.tc,
                    packetDesc->appendCrc,
                    packetDesc->pcktData, 
                    packetDesc->pcktPhyData, 
                    packetDesc->pcktDataLen,
                    1/*numOfBufs */, MV_FALSE);

        if(status != MV_OK)
        {
		printf("mvSwitchTxPcktCfg: Problem Sending Packet (error: %d)\n",status);
        }

	/* reset descriptor */
	status = mvSwitchTxDone(
			packetDesc->mrvlTag.swDsa.srcDev,
               		packetDesc->mrvlTag.swDsa.tc,
			1);
	if(status != MV_OK)
	{
		printf("mvSwitchTxDone: Problem Sending Packet (error: %d)\n",status);
	}

    return status;
}

MV_STATUS mvSwitchBuildPacket
(
	MV_U8  devNum,
	MV_U32 portNum,
    MV_U32 entryId,
	MV_U8  appendCrc,
	MV_U32 pcktsNum,
	MV_U8  gap,
	MV_U8* pcktData,
	MV_U32 pcktSize,
	MV_PKT_DESC *netTxPacketDesc 
)
{
    MV_U32	    dsaTag[2],bufferIndex;
    
    if (netTxPacketDesc->pcktData[0] == NULL)
        return MV_NO_RESOURCE;

    netTxPacketDesc->entryId = entryId;
    netTxPacketDesc->appendCrc = 1;
    netTxPacketDesc->pcktsNum = pcktsNum;
    netTxPacketDesc->gap = gap;
    netTxPacketDesc->waitTime = 0;


#ifdef PRESTERA_NO_CPU_PORT_DSA
    /* packet size should be at least MIN_PCKT_SIZE bytes */
    if (pcktSize >= MIN_PCKT_SIZE - CRC_SIZE)
        netTxPacketDesc->pcktDataLen[0] = pcktSize + CRC_SIZE;
    else
        netTxPacketDesc->pcktDataLen[0] = MIN_PCKT_SIZE;
#else
    /* packet size should be at least MIN_PCKT_SIZE bytes including DSA tag) */
	if (pcktSize >= MIN_PCKT_SIZE - DSA_TAG_SIZE - CRC_SIZE)
        netTxPacketDesc->pcktDataLen[0] = pcktSize + DSA_TAG_SIZE + CRC_SIZE;
    else
        netTxPacketDesc->pcktDataLen[0] = MIN_PCKT_SIZE;

    dsaTag[0] = calcFromCpuDsaTagWord0(devNum, portNum);
    dsaTag[1] = calcFromCpuDsaTagWord1(portNum);

    dsaTag[0] = MV_BYTE_SWAP_32BIT(dsaTag[0]);
    dsaTag[1] = MV_BYTE_SWAP_32BIT(dsaTag[1]);

    /* Set SA & DA */
    bufferIndex = 0; 
    osMemCpy(netTxPacketDesc->pcktData[0] + bufferIndex, pcktData, MAC_SA_AND_DA_SIZE);

    /* Set DSA Tag (2 words) */
    bufferIndex += MAC_SA_AND_DA_SIZE; 
    osMemCpy(netTxPacketDesc->pcktData[0] + bufferIndex, 
         &dsaTag[0], DSA_TAG_SIZE/2);
    bufferIndex += DSA_TAG_SIZE/2;
    osMemCpy(netTxPacketDesc->pcktData[0] + bufferIndex, 
         &dsaTag[1], DSA_TAG_SIZE/2);

    /* Set the rest of the packet */
    bufferIndex += DSA_TAG_SIZE/2;
    osMemCpy(netTxPacketDesc->pcktData[0] + bufferIndex, 
         pcktData + MAC_SA_AND_DA_SIZE, pcktSize - MAC_SA_AND_DA_SIZE);
#endif /* PRESTERA_NO_CPU_PORT_DSA */
    
	netTxPacketDesc->pcktDataLen[1] = 0;
    netTxPacketDesc->pcktDataAlign[0] = 0 ;
    netTxPacketDesc->mrvlTag.swDsa.srcDev = devNum;
    netTxPacketDesc->mrvlTag.swDsa.tc = 0;

    return MV_OK;
}


MV_STATUS mvSwitchReleasePacket
(
	MV_PKT_DESC *packetDesc 
)
{
	if(packetDesc->pcktData[0])
	{
		osFree(packetDesc->pcktData[0]);
	}

	return MV_OK;
}


static MV_U32 calcFromCpuDsaTagWord0(MV_U8 devNum, MV_U32 portNum)
{
	MV_U32 baseTag = 0x40001001; /* To CPU, Extended, VID=1 */
	/* port bits [4:0] are in this tag */
	/* port bits [5:5] are in extended tag */
	
	return hwByteSwap(baseTag | ((devNum  & 0x0F) << MRVL_TAG_DEV_BIT) | 
				    ((portNum & 0x1F) << MRVL_TAG_PORT_BIT));
}

static MV_U32 calcFromCpuDsaTagWord1(MV_U32 portNum)
{
	/* port bits [4:0] are in word 0 */
	/* port bits [5:5] are in extended tag */
	
	return hwByteSwap((((portNum & 0x20) >> 5) << MRVL_TAG_EXT_PORT_BIT));
}

void readPort0MIBCounters(int devNum)
{
	MV_U32 countersBaseAddr = 0x04010000, offset, regVal;
	for(offset = countersBaseAddr; offset < countersBaseAddr+0x80; offset+=4)
	{
		mvSwitchReadReg(devNum, offset, &regVal);
		printf("Address: 0x%08x \tValue: 0x%08x\n",offset,regVal);
	}
}

MV_STATUS mvSwitchAddDsaTag
(
	MV_U8  devNum,
	MV_U32 swPortNum,
	MV_U8* buf,
	MV_U32 len,
	MV_BUF_INFO* bufInfo 
)
{
    MV_U32	    dsaTag[2],bufferIndex;
    
    if (bufInfo == NULL || bufInfo->bufVirtPtr == NULL)
        return MV_NO_RESOURCE;

    if(len > (SWITCH_MTU - DSA_TAG_SIZE - CRC_SIZE))
	return MV_BAD_PARAM;

    /* packet size should be at least MIN_PCKT_SIZE bytes including DSA tag) */
    if (len >= MIN_PCKT_SIZE - DSA_TAG_SIZE - CRC_SIZE)
        bufInfo->dataSize = len + DSA_TAG_SIZE + CRC_SIZE;
    else
        bufInfo->dataSize = MIN_PCKT_SIZE;

    dsaTag[0] = calcFromCpuDsaTagWord0(devNum, swPortNum);
    dsaTag[1] = calcFromCpuDsaTagWord1(swPortNum);

    dsaTag[0] = MV_BYTE_SWAP_32BIT(dsaTag[0]);
    dsaTag[1] = MV_BYTE_SWAP_32BIT(dsaTag[1]);

    /* Set SA & DA */
    bufferIndex = 0; 
    osMemCpy(bufInfo->bufVirtPtr + bufferIndex, buf, MAC_SA_AND_DA_SIZE);

    /* Set DSA Tag (2 words) */
    bufferIndex += MAC_SA_AND_DA_SIZE; 
    osMemCpy(bufInfo->bufVirtPtr + bufferIndex, 
         &dsaTag[0], DSA_TAG_SIZE/2);
    bufferIndex += DSA_TAG_SIZE/2;
    osMemCpy(bufInfo->bufVirtPtr + bufferIndex, 
         &dsaTag[1], DSA_TAG_SIZE/2);

    /* Set the rest of the packet */
    bufferIndex += DSA_TAG_SIZE/2;
    osMemCpy(bufInfo->bufVirtPtr + bufferIndex, 
         buf + MAC_SA_AND_DA_SIZE, len - MAC_SA_AND_DA_SIZE);
    
    return MV_OK;
}


