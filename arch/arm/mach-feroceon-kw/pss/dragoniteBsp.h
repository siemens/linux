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

/*
 * Copy-paste from pssBspApis.h
 */
#undef IN
#define IN
#undef OUT
#define OUT
#undef INOUT
#define INOUT

/*******************************************************************************
* bspDragoniteSWDownload
*
* DESCRIPTION:
*       Download new version of Dragonite firmware to Dragonite MCU
*
* INPUTS:
*       sourcePtr      - Pointer to memory where new version of Dragonite firmware resides.
*       size           - size of firmware to download to ITCM.
*
* OUTPUTS:
*       None.
*
* RETURNS:
*       MV_OK   - on success,
*       MV_FAIL - otherwise.
*
* COMMENTS:
*       None.
*
*******************************************************************************/
MV_STATUS bspDragoniteSWDownload
(
    IN  const MV_VOID *sourcePtr,
    IN  MV_U32         size
);

/*******************************************************************************
* bspDragoniteEnableSet
*
* DESCRIPTION: Enable/Disable DRAGONITE module
*
* INPUTS:
*       enable – MV_TRUE  – DRAGONITE MCU starts work with parameters set by application
*                MV_FALSE – DRAGONITE MCU stops function
*
* OUTPUTS:
*       None.
*
* RETURNS:
*       MV_OK if successful, or
*       MV_FAIL otherwise.
*
* COMMENTS:
*       call after SW download
*
*******************************************************************************/
MV_STATUS bspDragoniteEnableSet
(
    IN  MV_BOOL enable
);


/*******************************************************************************
* bspDragoniteInit
*
* DESCRIPTION: Initialize DRAGONITE module
*
* INPUTS:
*       None.
*
* OUTPUTS:
*       None.
*
* RETURNS:
*       MV_OK if successful, or
*       MV_FAIL otherwise.
*
* COMMENTS:
*       Application will call this before firmware download
*
*******************************************************************************/
MV_STATUS bspDragoniteInit
(
    MV_VOID
);

/*******************************************************************************
* bspDragoniteSharedMemWrite
*
* DESCRIPTION:
*       Write a given buffer to the given offset in shared memory of DRAGONITE 
*        microcontroller.
*
* INPUTS:
*       offset  - Offset from beginning of shared memory
*       buffer  - The buffer to be written.
*       length  - Length of buffer in bytes.
*
* OUTPUTS:
*       None.
*
* RETURNS:
*       MV_OK   - on success.
*       MV_BAD_PARAM - out-of-boundary memory access
*       MV_FAIL - otherwise.
*
* COMMENTS:
*       Only DTCM is reachable
*
*******************************************************************************/
MV_STATUS bspDragoniteSharedMemWrite
(
    IN  MV_U32         offset,
    IN  const MV_VOID *buffer,
    IN  MV_U32         length
);

/*******************************************************************************
* bspDragoniteSharedMemRead
*
* DESCRIPTION:
*       Read a memory block from a given offset in shared memory of DRAGONITE
*        microcontroller.
*
* INPUTS:
*       offset  - Offset from beginning of shared memory
*       length  - Length of the memory block to read (in bytes).
*
* OUTPUTS:
*       buffer  - The read data.
*
* RETURNS:
*       MV_OK   - on success.
*       MV_BAD_PARAM - out-of-boundary memory access.
*       MV_FAIL - otherwise.
*
* COMMENTS:
*       Only DTCM is reachanble
*
*******************************************************************************/
MV_STATUS bspDragoniteSharedMemRead
(
    IN  MV_U32   offset,
    OUT MV_VOID *buffer,
    IN  MV_U32   length
);

/*******************************************************************************
* bspDragoniteSharedMemoryBaseAddrGet
*
* DESCRIPTION:
*       Get start address of DTCM
*
* INPUTS:
*       dtcmPtr - Pointer to beginning of DTCM where communication structures 
*                 must be placed
*
* OUTPUTS:
*       None.
*
* RETURNS:
*       MV_OK   - on success,
*       MV_FAIL - otherwise.
*
* COMMENTS:
*       None.
*
*******************************************************************************/
MV_STATUS bspDragoniteSharedMemoryBaseAddrGet
(
    OUT MV_U32 *dtcmPtr
);

/*******************************************************************************
* bspDragoniteGetIntVec
*
* DESCRIPTION:
*       This routine returns the DRAGONITE interrupt vector.
*
* INPUTS:
*       None
*
* OUTPUTS:
*       intVec - DRAGONITE interrupt vector.
*
* RETURNS:
*       MV_OK      - on success.
*       MV_FAIL    - otherwise.
*
* COMMENTS:
*       None.
*
*******************************************************************************/
MV_STATUS bspDragoniteGetIntVec
(
    OUT MV_U32 *intVec
);


