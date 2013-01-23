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

#ifndef __mvPrvInterrupt_H
#define __mvPrvInterrupt_H

/********* Include files ******************************************************/
#include "common/mvGenTypes.h"
#include "sdma/mvNetIf.h"
#include "sdma/interNetIfTypes.h"
#include "mvTypes.h"



/********* Defines ************************************************************/

/* Maximum interrupts number in system per PP   */
#define     PP_INTERRUPT_MAX_NUM        MV_NUM_OF_INT_CAUSE

/* Number of inerrupt queues                    */
#define     NUM_OF_INT_QUEUES           8

/* The interrupt queue handling policy for all  */
/* queues.                                      */
#define     INT_QUEUES_POLICY           MV_STRICT_PRIO
#define     INT_QUEUES_WEIGHT           10
    
#define     INT_NOT_EXIST   0x0
    
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
    IN MV_U8            devNum,
    IN MV_U8            queueIdx,
    MV_PKT_INFO 	*pktInfo    /*MV_RX_PKTS_INFO	*rxPcktsInfo*/
);

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

    IN MV_U8             devNum,
    IN MV_U8             queueIdx,
    STRUCT_SW_RX_DESC    *swRxDesc,
    MV_U32               descNum,
    MV_PKT_INFO 	 *pktInfo /*   MV_RX_PKTS_INFO	*rxPcktsInfo*/
);


#endif /* mvPrvInterrupt_H */

