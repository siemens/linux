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

#ifdef MV_INCLUDE_DRAGONITE
#include "mvDragonite.h"
#include "hwIf/gtHwIf.h"
#include "common/macros/gtCommonDefs.h"
#include "common/macros/gtPresteraDefs.h"
#include "mvOs.h"
#include "mvSysHwConfig.h"
#include "ctrlEnv/mvCtrlEnvRegs.h"
#include "ctrlEnv/sys/mvCpuIf.h"
#include "mvCommon.h"

/*
 * Internal defines; should not be exposed.
 */

#define DRAGONITE_CTRL_REG        	(0x88)
#define DRAGONITE_EN_BIT          	BIT_0
#define DRAGONITE_CPU_EN_BIT      	BIT_1
#define DRAGONITE_CPU_GPP0FUNC    BIT_2
#define DRAGONITE_CPU_INITRAM_BIT 	BIT_3

#define DTCM_BRIDGE_OFFSET	0x4000000

#define ITCM_START (INTER_REGS_BASE + 0x80000)
#define ITCM_SIZE (_64KB)
static MV_VOID * const itcmP = (MV_VOID * const)ITCM_START;
static const MV_U32 itcmSize = ITCM_SIZE;

#define DTCM_START 	0xF2000000
#define DTCM_SIZE 	(_32KB)

static MV_VOID * const dtcmP = (MV_VOID * const)DTCM_START;
static const MV_U32 dtcmSize = DTCM_SIZE;

#define DTCM_STACK_START  (DTCM_START + DTCM_SIZE)
#define DTCM_STACK_SIZE   (_16KB)
static MV_VOID * const dtcmStackP = (MV_VOID * const)DTCM_STACK_START;
static const MV_U32 dtcmStackSize = DTCM_STACK_SIZE;

#define DTCM_SHARED_START (DTCM_START)
#define DTCM_SHARED_SIZE  (_8KB)
static MV_VOID * const dtcmSharedP = (MV_VOID * const)DTCM_SHARED_START;
static const MV_U32 dtcmSharedSize = DTCM_SHARED_SIZE;

#define DTCM_OTHER_START  (DTCM_SHARED_START + DTCM_SHARED_SIZE)
#define DTCM_OTHER_SIZE   (_8KB)
static MV_VOID * const dtcmOtherP = (MV_VOID * const)DTCM_OTHER_START;
static const MV_U32 dtcmOtherSize = DTCM_OTHER_SIZE;

/* CPU address decode table. */
#ifndef MV_LINUX
extern MV_CPU_DEC_WIN mvCpuAddrWinMap[];
#endif
extern MV_CPU_DEC_WIN* mv_sys_map(void);

/******************************************************************************
ToDo:

0.  Should I use in real C code SendVerilogLine()-like function?
1.  IRQ_polarity.

******************************************************************************/
MV_STATUS mvDragoniteInit(MV_VOID)
{
	MV_ADDR_WIN addrWin;
    MV_CPU_DEC_WIN *win;

#ifndef MV_LINUX
        win = mvCpuAddrWinMap;
#else
        win = mv_sys_map();
#endif
	
	addrWin.baseLow = DTCM_BRIDGE_OFFSET;
	addrWin.baseHigh = 0x0;
	if (0xffffffff == mvAhbToMbusWinRemap(win[DRAGONITE_UNIT].winNum ,&addrWin))
	{
		printk("mvDragoniteInit:WARN. mvAhbToMbusWinRemap can't remap winNum=%d\n",
					  win[DRAGONITE_UNIT].winNum);
	}

   	mvSwitchBitSet(PRESTERA_DEFAULT_DEV, DRAGONITE_CTRL_REG, DRAGONITE_EN_BIT | DRAGONITE_CPU_GPP0FUNC | DRAGONITE_CPU_INITRAM_BIT);
    	
	if (memset(dtcmP, 0xFF, dtcmSize) != dtcmP)
        	return MV_FAIL;

   	return MV_OK;
}

static MV_U16 slowCrc16(MV_U16 sum, MV_U8 *p, MV_U32 len)
{
    MV_U32 i;
    MV_U8 byte;
    MV_U16 osum;

    while (len--)
    {
        byte = *p++;
        
        for (i = 0; i < 8; ++i)
        {
            osum = sum;
            sum <<= 1;

            if (byte & 0x80)
                sum |= 1 ;

            if (osum & 0x8000)
                sum ^= 0x1021;

            byte <<= 1;
        }
    }

    return sum;
}

static MV_STATUS dragoniteFwCrcCheck(MV_VOID)
{
#define __checksum_begin (ITCM_START)
#define __checksum_end (ITCM_START + ITCM_SIZE - 5)
#define __checksum (ITCM_START + ITCM_SIZE - 2)

    MV_U16 sum=0;
    const MV_U32 zero=0;

    sum = slowCrc16(sum, (MV_U8 *)__checksum_begin, (MV_U32)__checksum_end - (MV_U32)__checksum_begin + 1);
    sum = slowCrc16(sum, (MV_U8 *)&zero, 2);

    if(MV_16BIT_LE(sum) != (*(MV_U16 *)__checksum))
    {
        return MV_FAIL;
    }

    return MV_OK;
}


MV_STATUS mvDragoniteSWDownload(const MV_VOID *src, MV_U32 size)
{
    const MV_U8 *srcPtr = (MV_U8*)src;
    MV_U8 *dstPtr = (MV_U8*)itcmP;
    MV_U32 i;

    if (size == 0 || size > itcmSize)
        return MV_FAIL;

    /* copy by bytes to support firmware download on both 
     * big- and little-endian
     */
    for(i = 0; i < size; i++)
        *(dstPtr+i) = *(srcPtr+i);

    return dragoniteFwCrcCheck();
}

MV_STATUS mvDragoniteEnableSet(MV_BOOL enable)
{
    if (enable)
        mvSwitchBitSet(PRESTERA_DEFAULT_DEV, DRAGONITE_CTRL_REG, DRAGONITE_CPU_EN_BIT);
    else
        mvSwitchBitReset(PRESTERA_DEFAULT_DEV, DRAGONITE_CTRL_REG, DRAGONITE_CPU_EN_BIT);
    return MV_OK;
}

MV_STATUS mvDragoniteSharedMemWrite(MV_U32 offset, const MV_VOID *buffer, MV_U32 length)
{
    if (length > dtcmSharedSize - offset)
        return MV_BAD_PARAM;

    memcpy(dtcmSharedP + offset, buffer, length);
    return MV_OK;
}

MV_STATUS mvDragoniteSharedMemRead(MV_U32 offset, MV_U32 length, MV_VOID *buffer)
{
    if (length > dtcmSharedSize - offset)
        return MV_BAD_PARAM;

    memcpy(buffer, dtcmSharedP + offset, length);
    return MV_OK;
}

MV_STATUS mvDragoniteSharedMemBaseAddrGet(MV_U32* sharedMemP)
{
    *sharedMemP = (MV_U32)dtcmSharedP;
    return MV_OK;
}

#endif /* MV_INCLUDE_DRAGONITE */

