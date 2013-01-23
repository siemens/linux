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

#ifndef __gtBridge_H
#define __gtBridge_H

/************ Includes ********************************************************/
#include "common/os/gtTypes.h"
#include "gtOs/gtGenTypes.h"
#include "gtOs/gtEnvDep.h"
#include "sdma/gtNetIf.h"

/************ Define **********************************************************/
#define MACTBL_MESSAGE_SIZE 4


/************ Global Variables *************************************************/
/* Indicate if system is in Controlled Learning mode */

/************ Typedefs ********************************************************/
/************* Structures for Mac Table Update Message ***************************/


/******************************************************************************/
/*
 * typedef: enum MAC_TBL_UPDATE_MSG_TYPE
 *
 * Description:  
 *      Update message type
 *
 * Fields:
 *      UPDATE_MSG_NA - 
 *      UPDATE_MSG_QA - 
 *      UPDATE_MSG_QR - 
 *      UPDATE_MSG_AA - 
 *      UPDATE_MSG_TA - 
 *
 * Comment: 
 */
typedef enum
{
    UPDATE_MSG_NA = 0,
    UPDATE_MSG_QA = 1,
    UPDATE_MSG_QR = 2,
    UPDATE_MSG_AA = 3,
    UPDATE_MSG_TA = 4,
    UPDATE_MSG_FU = 5,
    
    UPDATE_MSG_LAST

}MAC_TBL_UPDATE_MSG_TYPE;

/******************************************************************************/
/*
 * typedef: enum MAC_TBL_UPDATE_ENTRY_TYPE
 *
 * Description:  
 *      Update message entry type
 *
 * Fields:
 *      UPDATE_MSG_ENTRY_MAC  - 
 *      UPDATE_MSG_ENTRY_IPV4 - 
 *      UPDATE_MSG_ENTRY_IPV6 - 
 *
 * Comment: 
 */
typedef enum
{
    UPDATE_MSG_ENTRY_MAC = 0,
    UPDATE_MSG_ENTRY_IPV4 = 1,
    UPDATE_MSG_ENTRY_IPV6 = 2,
    
    UPDATE_MSG_ENTRY_LAST

}MAC_TBL_UPDATE_ENTRY_TYPE;

/*
 * Typedef struct: MAC_TBL_HW_ENTRY
 *
 * Description:
 *      Define HW entry in the MAC TABLE
 *
 * Fields:
 *
 *   macAddr     -  
 *   vid         -  
 *   trunk       -  
 *   aging       -  
 *   skip        -  
 *   valid       -  
 *   rxSniff     -  
 *   saCmd       -  
 *   daCmd       -  
 *   forceL3Cos  -  
 *   multiple    -   
 *   isStatic    -  
 *   daPrio      -  
 *   saPrio      -  
 *   vidx        -  
 *   portTrnk    -  
 *                  
 */
typedef struct
{
    GT_ETHERADDR                    macAddr;
    GT_IPADDR                       dipAddr;
    GT_IPADDR                       sipAddr;
    MV_U32                          vidx;
    MV_U32                          portNum;
    MV_U32                          trunkNum;
    MV_BOOL                         trunk;
    MV_BOOL                         multiple;
    MV_BOOL                         spUnknown;
    MV_BOOL                         age;
    MV_BOOL                         skip;
    MV_U32                          vid;
    MV_BOOL                         mirrorToAnalyzer;
    MV_U32                          saCmd;
    MV_U32                          daCmd;
    MAC_TBL_UPDATE_ENTRY_TYPE       entryType;
    MV_BOOL                         isStatic;
    MV_U32                          daQosProfileIndex;
    MV_U32                          saQosProfileIndex;
    MV_U32                          ownDevNum;
    MV_U32                          srcId;
    MV_BOOL                         valid;
    MV_U32                          saSecurityLevel;
    MV_U32                          daSecurityLevel;
    MV_BOOL                         appSpecCpuCodeEn;
    MV_U32                          userDefined;
    GT_IPV6ADDR                     ipv6Sip;
    GT_IPV6ADDR                     ipv6Dip;

}MAC_TBL_HW_ENTRY;


/*
 * typedef: struct MAC_TBL_UPDATE_MSG
 *
 * Description:  
 *      Defines the update message format.
 *
 * Fields:
 *      msgType      - message type.
 *      entryFound   - Valid Only for query reply message (QR) from the 
 *                     98DX240/160 to the Host CPU and for Index query reply 
 *                     message (QI) from the 98DX240/160 to the Host CPU and 
 *                     MV_TRUE  - Entry was found. 
 *                     MV_FALSE - Entry was not found.  
 *      ownDevNum    - The 98DX240/160 Device number.
 *      macAddrIndex - If entryFound = MV_TRUE, this fields contains the entry 
 *                      Index in the FDB.
 *      macEntryData - Other mac entry data.
 *
 * Comment: 
 */
typedef struct
{
    MV_BOOL                     entryFound;
    MV_U32                      macAddrOffset;
    MV_U32                      macAddrIndex;
    MAC_TBL_UPDATE_MSG_TYPE     msgType;
    MV_U32                      msgId;
    MV_BOOL                     chainTooLong;
    MV_BOOL                     daRoute;
    MV_BOOL                     searchType;
    MAC_TBL_HW_ENTRY            macEntryData;
        
}MAC_TBL_UPDATE_MSG;

/************* Functions Prototype ********************************************/


/*******************************************************************************
* gtBrgTeachNewAddress
*
* DESCRIPTION:
*       Building the message for learning the cheetah new address on controlled
*       learning mode.
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
);

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
);


#endif /* __gtBridge_H */
