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

#include "mvHwIf.h"
#include "mvPresteraRegs.h"
#include "common/macros/mvPresteraDefs.h"
#include "common/macros/mvCommonDefs.h"
#include "mvOsPrestera.h"

/*
#define DEBUG
*/

#ifdef  DEBUG
#define debug(fmt,args...)      printf (fmt ,##args)
#else
#define debug(fmt,args...)
#endif  /* DEBUG */

#define DEFAULT_REGION	2

MV_U32 _prestera_dev_base_addr[] = MV_PRESTERA_DEV_BASE_TBL;
/* Number of devices in the system */
static int devicesNum = -1;

/*******************************************************************************
* mvSwitchAddrComp
*
* DESCRIPTION:
*       Writes the unmasked bits of a register using Pci.
*
* INPUTS:
*       baseAddr    - Device base address
*       regAddr     - Register address to perform address completion to.
*
* OUTPUTS:
*       None.
*
* RETURNS:
*       MV_OK       - on success
*       MV_FAIL      - on error
*
* COMMENTS:
*
*******************************************************************************/
static MV_STATUS mvSwitchAddrComp(MV_U32 baseAddr, MV_U32 regAddr, MV_U32 *pciAddr)
{
	MV_U32      addressCompletion;
	MV_U32      region, regionValue, regionMask;

	/* read the contents of the address completion register */
	addressCompletion = (MV_U32)MV_32BIT_LE_FAST(*((volatile MV_U32 *)baseAddr));
	/*debug("DEBUG - Old Address Completion: 0x%08x\n", addressCompletion);*/

	/* check if the region need to be changed */
	/* calculate the region (bits 24,25) */
	region = (regAddr & 0x3000000) >> 24;
	/* the region value is the 8 MSB of the register address */
	regionValue = (regAddr & 0xFF000000) >> 24;
	
	/*debug("DEBUG - Region: %d Region Value: 0x%02x\n", region, regionValue);*/

	/* region 0 value is always 0 therefore we need to
 	   check that the regionValue is also 0 */
	if (region == 0 && regionValue !=0 )	
	{
		/* change to a region that can be changed */
		region = DEFAULT_REGION;
	}
	
	/* the region mask for read modify write */
	regionMask = ~(0xFF << (region * 8));

	/* check if update of region is neccessary */
	if (regionValue != (addressCompletion & regionMask) >> (region * 8))
	{
		/* unset all the specific region bits */
		addressCompletion &= regionMask;
		/* write the new region value to the right place */
		addressCompletion |= regionValue << (region * 8);

		/* write back the updated address completion register */
		*((volatile MV_U32 *)baseAddr) = (MV_U32)MV_32BIT_LE_FAST(addressCompletion);
	}

	/*debug("DEBUG - New Address Completion: 0x%08x\n", addressCompletion);*/

	/* Calculate the PCI address to return */
	/* Remove 8 MSB and add region ID + base address */
        *pciAddr = baseAddr | (region << 24) | (regAddr & 0x00FFFFFF);
	/*debug("DEBUG - Address: 0x%08x\n", *pciAddr);*/

	return MV_OK;
}

/*******************************************************************************
* mvSwitchWriteReg
*
* DESCRIPTION:
*       Writes the unmasked bits to the switch internal register.
*
* INPUTS:
*       devNum    - Device Number
*       regAddr  - Register address to write to.
*       value    - Data to be written to register.
*
* OUTPUTS:
*       None.
*
* RETURNS:
*       MV_OK     - on success
*       MV_FAIL   - on error
*
* COMMENTS:
*
*******************************************************************************/
MV_STATUS mvSwitchWriteReg(MV_U8 devNum, MV_U32 regAddr, MV_U32 value)
{
	MV_U32 address;
	
	if(devNum >= mvSwitchGetDevicesNum())
    	{
		return MV_BAD_PARAM; 
    	}	 
	
	debug("mvSwitchWriteReg: devNum: %d  regAddr: 0x%x  value: 0x%x\n",
		devNum,regAddr,value);

	if (mvSwitchAddrComp(_prestera_dev_base_addr[devNum], regAddr, &address)!=MV_OK)
	{
		return MV_FAIL;
	}

	*((volatile MV_U32 *)address) = (MV_U32)MV_32BIT_LE_FAST(value);

	return MV_OK;
}

/*******************************************************************************
* mvSwitchReadReg
*
* DESCRIPTION:
*       Reads the unmasked bits of the switch internal register.
*
* INPUTS:
*       devNum    - Device Number
*       regAddr   - Register address to read.
*       pValue    - Data to read from the register.
*
* OUTPUTS:
*       None.
*
* RETURNS:
*       MV_OK      - on success
*       MV_FAIL      - on error
*
* COMMENTS:
*
*******************************************************************************/
MV_STATUS mvSwitchReadReg(MV_U8   devNum, MV_U32   regAddr, MV_U32  *pValue)
{
	MV_U32	address;
        MV_U32 tmp;

	if(devNum >= mvSwitchGetDevicesNum())
    	{
		return MV_BAD_PARAM; 
	}	 
	
	debug("mvSwitchReadReg: devNum: %d  regAddr: 0x%x\n",
		devNum,regAddr);

	if ((mvSwitchAddrComp(_prestera_dev_base_addr[devNum], regAddr, &address))!=MV_OK)
	{
		return MV_FAIL;
	}

	tmp = (MV_U32)(*((volatile MV_U32 *)address));
        *pValue = MV_32BIT_LE_FAST(tmp);
	
	return MV_OK;
}



/*******************************************************************************
* mvSwitchReadModWriteReg
*
* DESCRIPTION:
*       Writes the masked bits to the switch internal register.
*
* INPUTS:
*       devNum    - Device Number
*       regAddr   - Register address to write.
*       mask      - Bit mask value selecting the bits needed to be written.
*       value     - Data to write to the register.
*
* OUTPUTS:
*       None.
*
* RETURNS:
*       MV_OK      - on success
*       MV_FAIL      - on error
*
* COMMENTS:
*
*******************************************************************************/

MV_STATUS mvSwitchReadModWriteReg(MV_U8 devNum, MV_U32 regAddr, MV_U32 mask, MV_U32 value)
{
    MV_U32      regData;
    MV_STATUS   rc;

    if(devNum >= mvSwitchGetDevicesNum())
    {
	return MV_BAD_PARAM; 
    }	 

    rc = mvSwitchReadReg(devNum,regAddr,&regData);

    if (rc != MV_OK)
    {
        return rc;
    }

    /* Update the relevant bits at the register data */
    regData = (regData & ~mask) | (value & mask);

    rc = mvSwitchWriteReg(devNum, regAddr, regData);

    return rc;
}

/*******************************************************************************
* mvSwitchBitSet
* DESCRIPTION: function is self-descriptive
*******************************************************************************/
MV_STATUS mvSwitchBitSet(MV_U8 devNum, MV_U32 regAddr, MV_U32 bitMask)
{
    return mvSwitchReadModWriteReg(devNum, regAddr, bitMask, bitMask);
}

/*******************************************************************************
* mvSwitchBitReset
* DESCRIPTION: function is self-descriptive
*******************************************************************************/
MV_STATUS mvSwitchBitReset(MV_U8 devNum, MV_U32 regAddr, MV_U32 bitMask)
{
    return mvSwitchReadModWriteReg(devNum, regAddr, bitMask, 0);
}

/*******************************************************************************
* mvSwitchGetDevicesNum
*
* DESCRIPTION:
*       returns the number of devices on the system.
*
* INPUTS:
*       None.
*
* OUTPUTS:
*       none.
*
* RETURNS:
*        number of devices
*
* COMMENTS:
*
*******************************************************************************/

MV_U8 mvSwitchGetDevicesNum(void)
{
    #if defined (MV_XCAT_INTERPOSER)
    devicesNum = 1;
    #endif

#if 0
    MV_U32      vendorId;

	if (devicesNum == -1)
	{
    		/* read the vendor ID and see if we recongnize it on the other device */
    		vendorId = (MV_U32)MV_32BIT_LE_FAST(*((volatile MV_U32 *)
			(_prestera_dev_base_addr[1/*devNum*/] + PRESTERA_VENDOR_ID_REG)));

		devicesNum = (vendorId==MV_VENDOR_ID) ?
				2 : 1;
	}
	
#else
    if (devicesNum == -1)
    {
        int dev;
        MV_U32 pexData;
        devicesNum = 1;

	for (dev=0;dev<32;dev++)
	{
		pexData = mvPexConfigRead(0/*pexIf*/, 0/*bus*/, dev/*dev*/, 
                                          0/*func*/,0x0/*VENDOR_ID_REG*/);
		debug("dev %d, pexData: 0x%x\n",dev,pexData);

		if ((pexData&0xFFFF) == 0x11AB && (pexData&0xF0000000) == 0xD0000000)
		{
			pexData = mvPexConfigRead(0/*pexIf*/, 0/*bus*/, 
                                                 dev/*dev*/ , 0/*func*/,0x18);
			debug("----- Found: 0x%x\n",pexData);
			devicesNum ++;
		}
	}
    }

#endif

    return devicesNum;
}

/*******************************************************************************
* mvSwitchGetPortsNum
*
* DESCRIPTION:
*       returns the number of ports on the system.
*
* INPUTS:
*       None.
*
* OUTPUTS:
*       none.
*
* RETURNS:
*        number of ports
*
* COMMENTS:
*
*******************************************************************************/

MV_U32 mvSwitchGetPortsNum(void)
{
	return mvSwitchGetDevicesNum() * PRESTERA_DEV_PORT_NUM;
}

#include "common/macros/mvCommonFuncs.h"
//#define PRESTERA_NO_CPU_PORT_DSA

void mvSwitchCpuPortConn(MV_U8 devNum)
{
	miiInfCpuPortConfig(devNum);
	
	mvSwitchCpuPortConfig(devNum);
}

void mvSwitchCpuPortConfig(MV_U8 devNum)
{
    	/* Mg Global Control: set SelPortSDMA = 0 and PowerSave = 0 */
	mvSwitchReadModWriteReg(devNum, 0x58, BIT_19|BIT_20, 0);

    	/* Set ref_clk_125_sel  to PLL */
	mvSwitchReadModWriteReg(devNum, 0x5C, BIT_10, BIT_10);

	/* Set CPUPortActive = 1, CPUPortIFType = 2, MIBCountMode = 1 */
	mvSwitchWriteReg(devNum, 0xA0, 0xd);

	/* Set R0_Active = 0, RGPP_TEST = 0, GPP_Active = 1 */
	mvSwitchReadModWriteReg(devNum, 0x28, BIT_18|BIT_19|BIT_20, BIT_18);

	mvSwitchReadModWriteReg(devNum, 0x2C, BIT_10|BIT_11|BIT_12, BIT_10|BIT_11);

	/* Set PortMacControl fot port 63: */
	/* PcsEn = 0, UseIntClkforEn = 1, PortMACReset = 0, CollisionOnBackPressureCntEn=1 */
	mvSwitchWriteReg(devNum, 0xa80fc48, 0x300);
	mvSwitchWriteReg(devNum, 0xa80fc08, 0x4010);
	mvSwitchWriteReg(devNum, 0xa80fc04, 0x1f87);
	mvSwitchWriteReg(devNum, 0xa80fc00, 0x8be5);

#ifndef PRESTERA_NO_CPU_PORT_DSA
	/* Add DSA tag from CPU port: CPUPortDSATagEn = 1 */
	mvSwitchBitSet(devNum, 0x0f000004, BIT_31);
#else
	/* Add DSA tag from CPU port: CPUPortDSATagEn = 1 */
	mvSwitchBitReset(devNum, 0x0f000004, BIT_31);
#endif
}

