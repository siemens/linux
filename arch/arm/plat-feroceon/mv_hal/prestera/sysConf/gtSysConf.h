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

#ifndef __gtSysConf_
#define __gtSysConf_


/************ Includes ********************************************************/

/* General H Files */
#include <gtOs/gtGenTypes.h>
#include <common/os/gtTypes.h>

/************ Define **********************************************************/

#define SYS_CONF_MAX_DEV    4
#define SYS_CPU_PORT_NUM    63

/************ Typedefs ********************************************************/

/*
 * Typedef enum SYS_CONF_SW_VER
 *
 * Description:
 *      define the software version as read from the hardware device id
 *
 */
typedef enum
{
    SYS_CONF_SW_VER_1,
    SYS_CONF_SW_VER_2
}SYS_CONF_SW_VER;

#define GT_VENDOR_ID 0x11AB

/*
 * typedef: struct GT_PCI_DEV_VENDOR_ID
 *
 * Description: PCI device and vendor ID struct
 *
 * Fields:
 *   vendorId - The Prestera PCI vendor Id.
 *   devId    - The different Prestera PCI device Id.
 */
typedef struct
{
    GT_U16  vendorId;
    GT_U16  devId;
} GT_DEV_VENDOR_ID;

/* Cheetah devices id enum */
typedef enum
{
    /* EX devices Cheetah J*/
    GT_98DX5058_GE = 0xDB0011AB,
    GT_98DX5058_XG = 0xDB0111AB,
    GT_98DX5128_GE = 0xDB1011AB,
    GT_98DX8110_XG = 0xDB1111AB,
    GT_98DX5129_GE = 0xDB9011AB,
    /* XCAT */
    GT_98DX2_GE = 0xDC0211AB,
    GT_98DX3_XG = 0xDC0311AB,

}GT_EX_DEVICE;

/*
 * typedef: struct GT_DEV_INFO
 *
 * Description: PCI device information
 *
 * Fields:
 *      baseAddr        - base address of prestera device on management.
 *      internalBase    - Base address to which the internal management registers
 *                        are mapped to.
 *
 */
typedef struct
{
    GT_U32                  baseAddr;
    GT_U32                  internalBase;
} GT_DEV_INFO;

#define MARVELL_PHY_OUI     (0x0141)
#define DEFAULT_MARVELL_PHY (0x01410C10)

#define MARVELL_PHY_88E1145 (0x01410CD0)
#define MARVELL_PHY_88X2010 (0x01410D20)
#define MARVELL_PHY_88X2011 (0x01410D30)
#define MARVELL_PHY_88X2010_X (0x01410D00)


#define IS_MARVELL_PHY(phyId)                                   \
        (((phyId) >> 10) == 0x5043)

#define GET_PHY_ID(phyId)                                       \
        ((phyId) >> 4)


#endif  /* __gtSysConf_ */
/* Do Not Add Anything Below This Line */
