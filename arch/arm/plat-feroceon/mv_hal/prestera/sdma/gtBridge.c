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

/************ Includes ********************************************************/
#include "gtBridge.h"
#include "common/macros/gtCommonFuncs.h"
#include "util/gtUtils.h"
#include "hwIf/gtHwIf.h"
#include "prestera/mvOsPrestera.h"

/************ Global Variables *************************************************/
/* Indicate if system is in Controlled Learning mode */
MV_BOOL brgControlledLearning = MV_FALSE;

extern SDMA_INTERRUPT_CTRL         sdmaInterCtrl[32];
extern MV_U32 numOfAddrUpQFull;


/*******************************************************************************
* gtBrgTeachNewAddress
*
* DESCRIPTION:
*
* INPUTS:
*       devNum      - Device number      
*       updateMsg   - an update message
*
* OUTPUTS:
*       None.
*
* RETURNS:
*       None.
*
* COMMENTS:
*
*******************************************************************************/
MV_STATUS gtBrgTeachNewAddress
(
    IN MV_U8                    devNum,
    IN  MAC_TBL_UPDATE_MSG      *updateMsg
)
{
    MV_U32      regAddr;
    MV_U32      data[5];
    MV_U32      timeout;
    int i;

    /* clear address update registers before build new message */
    regAddr = 0x6000040;
    osMemSet(data, 0, sizeof(data));

    if (updateMsg->macEntryData.entryType == UPDATE_MSG_ENTRY_MAC)
    {
        /* set mac address */
        data[0] |= (GT_HW_MAC_LOW16(&(updateMsg->macEntryData.macAddr)) << 16);
        data[1] |= GT_HW_MAC_HIGH32(&(updateMsg->macEntryData.macAddr));

        /* set devNum */
        data[3] |= ((updateMsg->macEntryData.ownDevNum & 0x1F) << 7);

        /* Multipple = 1, or Mac[40] = 1 */
        if ((updateMsg->macEntryData.multiple == 1) || 
            ((updateMsg->macEntryData.macAddr.arEther[0] & 1) == 1))
        {
            /* set vidx */
            data[2] |= (updateMsg->macEntryData.vidx << 17);
        }
        else
        {
            /* Trunk = 1 */
            if (updateMsg->macEntryData.trunk == 1)
            {
                /* set trunk */
                data[2] |= (updateMsg->macEntryData.trunkNum << 18);
            }
            else
            {
                /* set regular feature */
                data[2] |= (updateMsg->macEntryData.portNum << 18);
            }

            /* set IsTrunk */
            data[2] |= ((updateMsg->macEntryData.trunk & 1) << 17);
        }
        /* set SrcID */
        data[3] |= ((updateMsg->macEntryData.srcId & 0x1F) << 2);

        /* set UserDefined */
        data[2] |= (updateMsg->macEntryData.userDefined << 25);
    }
    else
    {
        /* set dip & sip */
        data[0] |= (updateMsg->macEntryData.dipAddr.arIP[0] << 16);
        data[0] |= (updateMsg->macEntryData.dipAddr.arIP[1] << 24);
        data[1] |= updateMsg->macEntryData.dipAddr.arIP[2];
        data[1] |= (updateMsg->macEntryData.dipAddr.arIP[3] << 8);

        data[1] |= (updateMsg->macEntryData.sipAddr.arIP[0] << 16);
        data[1] |= (updateMsg->macEntryData.sipAddr.arIP[1] << 24);
        data[3] |= updateMsg->macEntryData.sipAddr.arIP[2];
        data[3] |= ((updateMsg->macEntryData.sipAddr.arIP[3] & 0xf) << 8);
        data[3] |= ((updateMsg->macEntryData.sipAddr.arIP[3] >> 4) << 27);

        /* set vidx */
        data[2] |= (updateMsg->macEntryData.vidx << 17);

        /* set search type */
        data[2] |= (updateMsg->searchType << 16);
    }

    /* set multiple */
    data[2] |= (updateMsg->macEntryData.multiple << 15);
    /* set SPUnknown must be set to 0 by CPU */
    data[2] |= (updateMsg->macEntryData.spUnknown << 14);
    /* set Age */
    data[2] |= (updateMsg->macEntryData.age << 13);
    /* set Skip */
    data[2] |= (updateMsg->macEntryData.skip << 12);
    /* set regular feature */
    data[2] |= updateMsg->macEntryData.vid;

    /* set mirror2anal */
    data[3] |= (updateMsg->macEntryData.mirrorToAnalyzer << 31);
    /* set SA_CMD */
    data[3] |= (updateMsg->macEntryData.saCmd << 24);
    /* set DA_CMD */
    data[3] |= (updateMsg->macEntryData.daCmd << 21);

    /* set EntryType */
    data[3] |= (updateMsg->macEntryData.entryType << 19);
    /* set Static */
    data[3] |= (updateMsg->macEntryData.isStatic << 18);

    /* SASecurityLevel */
    data[0] |= (updateMsg->macEntryData.saSecurityLevel << 12);

    /* DASecurityLevel */
    data[0] |= (updateMsg->macEntryData.daSecurityLevel << 1);
       
    /* set DARoute */
    data[2] |= (updateMsg->daRoute << 30);
    
    /* set AppSpecificCPUCodeEn */
    data[2] |= (updateMsg->macEntryData.appSpecCpuCodeEn << 29);
        
    /* set SaQosProfile */
    data[3] |= (updateMsg->macEntryData.saQosProfileIndex << 12);
    /* set DaQosProfile */
    data[3] |= (updateMsg->macEntryData.daQosProfileIndex << 15);


    /* WRITE TO hw */
    for (i = 0 ; i <= 4; i++) 
    {
        CHECK_STATUS(hwIfSetReg(devNum, regAddr + i*0x4, data[i]));
    }

    /* trigeer operation in FDB CPU Update Message Control */
    CHECK_STATUS(hwIfSetReg(devNum, 0x06000050, 1));
    
    /* wait to operation done */
    timeout = 0;
    do
    {
        CHECK_STATUS(hwIfGetReg(devNum, 0x06000050, &(data[0])));
        if ((data[0] & 1) == 0)
        {
            break;
        }
        timeout++;
    }while(timeout < 10);

	if (timeout == 10) 
	{
		osPrintf("timeout in teach MAC address.\n");
		return MV_FAIL;
	}
    return MV_OK;
}

/*******************************************************************************
* setCPUAddressInMACTAble
*
* DESCRIPTION:
*
* INPUTS:
*       devId           - Device Id
*       macAddr		- MAC address
*	vid		- VLAN ID
*  
* OUTPUTS:
*       None
*
* RETURNS:
*       CMD_OK            - on success.
*       CMD_AGENT_ERROR   - on failure.
*       CMD_FIELD_UNDERFLOW - not enough field arguments.
*       CMD_FIELD_OVERFLOW  - too many field arguments.
*       CMD_SYNTAX_ERROR    - on fail
* 
* COMMENTS:
*       None
*
*******************************************************************************/
MV_STATUS setCPUAddressInMACTAble
(
   	MV_U8	devNum,
	MV_U8*	macAddr,
	MV_U32	vid
)
{
	MAC_TBL_UPDATE_MSG updMsg;

	osMemSet(&updMsg, 0, sizeof(updMsg));

	updMsg.macEntryData.entryType = UPDATE_MSG_ENTRY_MAC;

	/* set mac address */
	createMacAddr(&updMsg.macEntryData.macAddr, macAddr);

	/* set devNum */
	updMsg.macEntryData.ownDevNum = devNum;

        /* set regular feature */
        updMsg.macEntryData.portNum = 63; /* CPU Port */

        /* set regular feature */
        updMsg.macEntryData.vid = vid;

        /* set SA_CMD */
        updMsg.macEntryData.saCmd = 0; /* Forward */
        
	/* set DA_CMD */
        updMsg.macEntryData.daCmd = 2; /* Trap to CPU */

        /* set Static */
        updMsg.macEntryData.isStatic = 1;
    
    	/* set UserDefined */
	updMsg.macEntryData.userDefined = 0;

	/* set SaQosProfile */
	updMsg.macEntryData.saQosProfileIndex = 0;
	/* set DaQosProfile */
	updMsg.macEntryData.daQosProfileIndex = 0;
    
	CHECK_STATUS(gtBrgTeachNewAddress(devNum, &updMsg));

	return MV_OK;
}


