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

/***** Include files ***************************************************/

#include "mvVlan.h"
#include "prestera/mvOsPrestera.h"
#include "prestera/hwIf/mvHwIf.h"
#include "mvCommon.h"

#define toupper(c)	c

static const char hexcode[] = "0123456789ABCDEF";

#define VLAN_READ_TIMEOUT 1000

/****************************************
 * 				  	*
 * VLAN Table handeling functions 	*
 *				  	*
 ****************************************/

#define PRESTERA_VLAN_DIRECT

#if defined PRESTERA_VLAN_INDIRECT
	#define VLT_CTRL_REG_OFFSET	0xA000118
	#define VLT_DATA_REG_OFFSET	0xA000000
	#define VLAN_ENTRY_SIZE		0x10
#elif defined PRESTERA_VLAN_DIRECT
	#define VLT_CTRL_REG_OFFSET	0xA000118 /*not in use in direct access*/
	#define VLT_DATA_REG_OFFSET	0xA200000
	#define VLAN_ENTRY_SIZE		0x20
#else
	#error "Prestera Hal: VLAN table access isn't defined!\n"
#endif

#define VLT_ENTRY_BIT		0
#define VLT_TRIGGER_BIT		15
#define VLT_OPERATION_BIT	12

#define CPU_VLAN_MEMBER_BIT	2
#define PORT24_VLAN_MEMBER_BIT	72
#define PORT25_VLAN_MEMBER_BIT	74
#define PORT26_VLAN_MEMBER_BIT	76
#define PORT27_VLAN_MEMBER_BIT	104
#define UNKNOWN_UNICATS_CMD_BIT 12

#if 0
#define MV_DEBUG 
#endif
#ifdef MV_DEBUG
#define DB(x) x
#else
#define DB(x)
#endif


/*******************************************************************************
* setVLANTableCtrlReg
*
* DESCRIPTION:
*       	Sets the access control register in order to
* 		read/write VLAN Table entries
*
* INPUTS:
*       entryNum - VLAN Table entry number
*	operation- 0 - Read 1 - Write
*
* RETURNS:
*       
*
*******************************************************************************/
MV_STATUS setVLANTableCtrlReg(int devNum, int entryNum, int operation)
{
	MV_U32 timeout = 0;
	MV_U32 ctrlRegVal = entryNum << VLT_ENTRY_BIT | 
			    operation << VLT_OPERATION_BIT | 
			    1 << VLT_TRIGGER_BIT;

	DB( printf( "%s: \n", __FUNCTION__) );
	/* Write VLT Table Access Control register */
	if (mvSwitchWriteReg(devNum, VLT_CTRL_REG_OFFSET, ctrlRegVal)!=MV_OK) {
		DB( printf( "%s: Error: Problem writing register 0x%08x\n", 
					__FUNCTION__, VLT_CTRL_REG_OFFSET) );
		return MV_FAIL;
	}

	while((ctrlRegVal & (1 << VLT_TRIGGER_BIT)) && timeout < VLAN_READ_TIMEOUT)
	{
		if (mvSwitchReadReg(devNum,VLT_CTRL_REG_OFFSET, &ctrlRegVal)!=MV_OK) {
			DB( printf( "%s: Error: Problem reading register 0x%08x\n", 
					__FUNCTION__, VLT_CTRL_REG_OFFSET) );
			return MV_FAIL;
		}
		timeout++;
	}

	return (timeout==VLAN_READ_TIMEOUT) ? MV_FAIL : MV_OK;
}


MV_STATUS readVLANEntry(int devNum, int entryNum, STRUCT_VLAN_ENTRY* vlanTableEntry)
{
	MV_U32 dataRegVal, i=0;
	DB( printf( "%s: \n", __FUNCTION__) );

#if defined PRESTERA_VLAN_INDIRECT
	if(setVLANTableCtrlReg(devNum, entryNum, 0 /*Read*/)!=MV_OK)
	{
		DB( printf( "%s: Error: Problem setting VLAN control register\n", __FUNCTION__) );
		return MV_FAIL;
	}
#endif	
	for(i=0; i<4; i++)
	{
		if (mvSwitchReadReg(devNum,VLT_DATA_REG_OFFSET + entryNum*VLAN_ENTRY_SIZE + i*4, 
					&dataRegVal)!=MV_OK) {
			DB( printf( "%s: Error: Problem reading register 0x%08x\n", 
						__FUNCTION__ , VLT_DATA_REG_OFFSET + i*4) );
			return MV_FAIL;
		}
	 	vlanTableEntry->VLTData[3-i] = dataRegVal; /* Word 0 is in reg 3 and  */
							   /* Word 3 is in reg 0, etc */
		DB( printf("vlanTableEntry->VLTData[%d] = 0x%x\n",i,vlanTableEntry->VLTData[3-i]));
	}

	

	return MV_OK;
}

MV_STATUS setVLANEntry(int devNum, int entryNum,STRUCT_VLAN_ENTRY* vlanTableEntry)
{
	MV_U32 i=0;
	DB( printf( "%s: \n", __FUNCTION__) );

	for(i=0; i<4; i++)
	{
		DB( printf("vlanTableEntry->VLTData[%d] = 0x%x\n",i,vlanTableEntry->VLTData[3-i]));

		/* Word 0 is in reg 3 and  Word 3 is in reg 0, etc*/
		if (mvSwitchWriteReg(devNum,VLT_DATA_REG_OFFSET + entryNum*VLAN_ENTRY_SIZE + i*4,
			vlanTableEntry->VLTData[3-i])!=MV_OK) {
			DB( printf( "%s: Error: Problem reading register 0x%08x\n",
					 __FUNCTION__ , VLT_DATA_REG_OFFSET + i*4) );
			return MV_FAIL;
		}
	}

#if defined PRESTERA_VLAN_INDIRECT
	if(setVLANTableCtrlReg(devNum, entryNum, 1 /*Write*/)!=MV_OK)
	{
		DB( printf( "%s: Error: Problem setting VLAN control register\n", __FUNCTION__) );
		return MV_FAIL;
	}
#endif
	return MV_OK;
}

MV_STATUS setCpuAsVLANMember(int devNum, int vlanNum)
{
	STRUCT_VLAN_ENTRY vlanTableEntry;
	DB( printf( "%s: \n", __FUNCTION__) );
	if (readVLANEntry(devNum, vlanNum, &vlanTableEntry)!=MV_OK)
	{
		return MV_FAIL;
	}

	vlanTableEntry.VLTData[3] =  vlanTableEntry.VLTData[3] | (1 << CPU_VLAN_MEMBER_BIT);

	return setVLANEntry(devNum, vlanNum, &vlanTableEntry);
}


