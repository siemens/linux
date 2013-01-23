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

#ifndef __mvNetTransmit_h__
#define __mvNetTransmit_h__

/*#include "common/mvEnvDep.h"*/
#include "mvTypes.h"
#include "sdma/mvNetIf.h"
#include "sdma/interNetIfTypes.h"


/********* Defines ************************************************************/
#define TX_STACK_SIZE 0x2000


#define MRVL_TAG_PORT_BIT	19
#define MRVL_TAG_DEV_BIT	24
#define MRVL_TAG_EXT_PORT_BIT	10

#define MAX_NUM_OF_SDMA_BUFFERS_PER_CHAIN   64
#define MIN_PCKT_SIZE		64

/* Size of block to aligned to. Must be power of 2, 2 is minimum */
#define ALIGN_ADDR_CHUNK 64
/* This macro used to get alighnment address from allocated */
#define ALIGN_ADR_FROM_MEM_BLOCK(adr,align)\
                   (( ((MV_U32)( ((MV_U8*)(adr)) + (ALIGN_ADDR_CHUNK) - 1)) &\
                   ((0xFFFFFFFF - (ALIGN_ADDR_CHUNK) + 1) )) + (align))

/*
 * Typedef: enum MV_TX_CMD
 *
 * Description: Enumeration of Transmit command Types
 *
 * Enumerations:
 *      TX_BY_VLAN  - Send packets using "netSendPktByVid" function and
 *                    defines to use spesific parameters of this function.
 *      TX_BY_LPORT - Send packets using "netSendPktByLport" function and
 *                    defines to use spesific parameters of this function.
 */
typedef enum
{
    TX_BY_VLAN = 0,
    TX_BY_LPORT
} MV_TX_CMD;


/*
 * Typedef: struct MV_PKT_DESC
 *
 * Description: Packet Description structure
 *
 *      entryID         - Entry ID number for Delete/Get/ChangeAllign operations
 *      appendCrc       - (MV_TRUE) Add Crc to the transmitted packet,
 *                        (MV_FALSE) Leave the packet as is.
 *      tagged          - does the packet already include tag.
 *      pcktsNum        - Number of packets to send.
 *      gap             - The time is calculated in multiples of 64 clock cycles
 *                        Valid values are 0 (no delay between packets to
 *                        CPU), through 0x3FFE (1,048,544 Clk cycles).
 *                        0x3FFF - has same effect as 0.
 *      waitTime        - The wait time before moving to the next entry.
 *      pcktData        - Array of pointers to packet binary data.
 *      pcktDataLen     - Array of lengths of pcktData pointers.
 *      pcktDataAllign  - Alignments of buffers for pcktData pointers.
 *      numSentPackets  - Number of sent packets.
 *      cmdType         - Defines type of transmition (VLAN, If, LPort).
 *      cmdParams       - Defines fields for transmition type (VLAN, If, LPort)
 *
 */
typedef struct
{
    MV_U32          entryId;
    MV_BOOL         appendCrc;
    MV_U32          pcktsNum;
    MV_U32          gap;
    MV_U32          waitTime;
    MV_U8           *pcktData[MAX_NUM_OF_SDMA_BUFFERS_PER_CHAIN];
    MV_U8           *pcktPhyData[MAX_NUM_OF_SDMA_BUFFERS_PER_CHAIN];
    MV_U32          pcktDataLen[MAX_NUM_OF_SDMA_BUFFERS_PER_CHAIN];
    MV_U8           pcktDataAlign[MAX_NUM_OF_SDMA_BUFFERS_PER_CHAIN];
    MV_U32          numSentPackets;
    MV_TX_DSA       mrvlTag;
    MV_TX_CMD       cmdType;
} MV_PKT_DESC;


/*******************************************************************************
* mvSwitchTxStart
*
* DESCRIPTION:
*       Starts transmition of packets
*
* INPUTS:
*       packetDesc - pointer to packet desciptor struct
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
*       Command   - cmdTxStart
*
* Toolkit:
*       Interface - <prestera/tapi/networkif/commands.api>
*
*******************************************************************************/
MV_STATUS mvSwitchTxStart(MV_PKT_DESC *packetDesc);

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
	MV_PKT_DESC *packetDesc
);

MV_STATUS mvSwitchReleasePacket
(
	MV_PKT_DESC *packetDesc 
);

MV_STATUS mvSwitchPacketTransmit(MV_PKT_DESC *packetDesc);




void readPort0MIBCounters(int devNum);

#endif /*__mvNetTransmit_h__*/


