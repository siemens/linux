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
#ifndef __mvCommonDefsh
#define __mvCommonDefsh

#include "common/mvPresteraTypes.h"
#include "common/macros/mvPresteraDefs.h"

/* Define the Ip protocol ether type.   */
#define IP_PROTOCOL_ETHER_TYPE      (0x800)


/* Macro to check for a IP unicast address   */
/* MV_U32 ipAddr                               */
#define IS_IP_UNICAST(ipAddr) (((ipAddr) & 0xE0000000) != 0xE0000000)

/* Macro to check for a IP multicast address   */
/* MV_U32 ipAddr                               */
#define IS_IP_MULTICAST(ipAddr) (((ipAddr) & 0xE0000000) == 0xE0000000)


/*
 * Typedef: enum MV_GET_METHOD
 *
 * Description: Defines the different get methods, from internal data-structures
 *
 * Fields:
 *      MV_GET_FIRST    - Get the first entry from data-base.
 *      MV_GET_NEXT     - Get the next entry from data-base.
 *      MV_GET_ENTRY    - Get the specified entry from data-base.
 *
 */
typedef enum
{
    MV_GET_FIRST = 0,
    MV_GET_NEXT,
    MV_GET_ENTRY
}MV_GET_METHOD;

/*
 * typedef: struct MV_CORE_LPORT
 *
 * Description:  Defines the core logical port, unit device and port
 *
 * Fields:
 *      unitNum - unit number.
 *      devNum  - Device number.
 *      portNum - physical port number.
 *
 * Comment:
 */
typedef struct
{
    MV_U8 unitNum;
    MV_U8 devNum;
    MV_U8 portNum;

}MV_CORE_LPORT;


/*
 * Typedef:     struct MV_DEV_LIST
 *
 * Description: Datatype that is passed from the TAPI to CORE (via T2C) used
 *              to pass an array of device IDs to a unit and to retrieve a
 *              result code for each device from that unit.
 *
 * Fields:
 *              listLen       - length of devNumList and  retStatusList
 *              devNumList    - array of device IDs
 *              retStatusList - array of result Status list for each device
 *
 */
typedef struct
{
    MV_U8       listLen;
    MV_U8       devNumList[MAX_PP_DEVICES];
    MV_STATUS   retStatusList[MAX_PP_DEVICES];

}MV_DEV_LIST;


/*
 * typedef: enum MV_INT_TYPE
 *
 * Description: MV device interrupt connection type
 *
 * Enumerations:
 *    MV_INT_2_PP_GPP0 - interrupt line is connected to packet
 *                            processor GPP pin 0.
 *    MV_INT_2_PP_GPP1 - interrupt line is connected to packet
 *                            processor GPP pin 1.
 *    MV_INT_2_PP_GPP2 - interrupt line is connected to packet
 *                            processor GPP pin 2.
 *    MV_INT_2_UPLINK_GPP - interrupt line is connected to packet
 *                            processor uplink GPP.
 *    MV_INT_2_PCI_A   - interrupt is directly connected to PCI A
 *                            interrupt
 *    MV_INT_2_PCI_B   - interrupt is directly connected to PCI B
 *                            interrupt
 *    MV_INT_2_PCI_C   - interrupt is directly connected to PCI C
 *                            interrupt
 *    MV_INT_2_PCI_D   - interrupt is directly connected to PCI D
 *                            interrupt
 *    MV_INT_OTHER     - interrupt is connected in some other manner.
 *
 */
typedef enum
{
    MV_INT_2_PP_GPP0 = 0,
    MV_INT_2_PP_GPP1,
    MV_INT_2_PP_GPP2,
    MV_INT_2_UPLINK_GPP,
    MV_INT_2_PCI_A,
    MV_INT_2_PCI_B,
    MV_INT_2_PCI_C,
    MV_INT_2_PCI_D,
    MV_INT_OTHER
}MV_INT_TYPE;


/*
 * typedef: struct MV_INT
 *
 * Description:
 *     MV device Interrupt definition
 *
 * Fields:
 *  intType   - interrupt connection type.
 *  intVecNum - The interrupt vector number this device is connected to in
 *              case of PCI or other interrupt, or the packet processor device
 *              number connected to by its GPP pin.
 *    intMask - The interrupt mask to enable MV interrupts (used in Mips)
 */
typedef struct
{
    /*
        <CONFI_DESCRIPTION>
            <MIN_VAL>0</MIN_VAL>
            <MAX_VAL>7</MAX_VAL>
            <DESCRIPTION>
            interrupt connection type.
            </DESCRIPTION>
            <COMMENTS>Dont care - SW use Pci scan info</COMMENTS>
            </CONFI_DESCRIPTION>             
    */
    MV_INT_TYPE          intType;
    /*
        <CONFI_DESCRIPTION>
            <MIN_VAL>0</MIN_VAL>
            <MAX_VAL>0xFFFFFFFF</MAX_VAL>
            <DESCRIPTION>
            The interrupt vector number this device is connected to in
            case of PCI or other interrupt, or the packet processor device
            number connected to by its GPP pin.         
            </DESCRIPTION>
            <COMMENTS>Dont care - SW use Pci scan info</COMMENTS>
            </CONFI_DESCRIPTION>             
    */
    MV_U32               intVecNum;
    /*
        <CONFI_DESCRIPTION>
            <MIN_VAL>0</MIN_VAL>
            <MAX_VAL>0xFFFFFFFF</MAX_VAL>
            <DESCRIPTION>
            The interrupt mask to enable MV interrupts (used in Mips).
            </DESCRIPTION>
            <COMMENTS>Dont care - SW use Pci scan info</COMMENTS>
            </CONFI_DESCRIPTION>             
    */
    MV_U32               intMask;
}MV_INT;


/* Define the different memory sizes    */
#define _1KB            (0x400)
#define _2KB            (0x800)
#define _4KB            (0x1000)
#define _8KB            (0x2000)
#define _10KB           (0x2800)
#define _16KB           (0x4000)
#define _32KB           (0x8000)
#define _64KB           (0x10000)
#define _128KB          (0x20000)
#define _256KB          (0x40000)
#define _512KB          (0x80000)
#define _1MB            (0x100000)
#define _2MB            (0x200000)
#define _4MB            (0x400000)
#define _8MB            (0x800000)
#define _16MB           (0x1000000)
#define _32MB           (0x2000000)
#define _64MB           (0x4000000)
#define _128MB          (0x8000000)
#define _256MB          (0x10000000)


/* Define single bit masks.             */
#define BIT_0           (0x1)
#define BIT_1           (0x2)
#define BIT_2           (0x4)
#define BIT_3           (0x8)
#define BIT_4           (0x10)
#define BIT_5           (0x20)
#define BIT_6           (0x40)
#define BIT_7           (0x80)
#define BIT_8           (0x100)
#define BIT_9           (0x200)
#define BIT_10          (0x400)
#define BIT_11          (0x800)
#define BIT_12          (0x1000)
#define BIT_13          (0x2000)
#define BIT_14          (0x4000)
#define BIT_15          (0x8000)
#define BIT_16          (0x10000)
#define BIT_17          (0x20000)
#define BIT_18          (0x40000)
#define BIT_19          (0x80000)
#define BIT_20          (0x100000)
#define BIT_21          (0x200000)
#define BIT_22          (0x400000)
#define BIT_23          (0x800000)
#define BIT_24          (0x1000000)
#define BIT_25          (0x2000000)
#define BIT_26          (0x4000000)
#define BIT_27          (0x8000000)
#define BIT_28          (0x10000000)
#define BIT_29          (0x20000000)
#define BIT_30          (0x40000000)
#define BIT_31          (0x80000000)

// MII definitions
#define EGIGA_CPU_PORT          1
#define RX_BUFFER_DEFAULT_SIZE  0x600
#define RX_BUFFERS_ARRAY_SIZE   0x20

/* SDMA  definitions*/
#define PRESTERA_TXQ_LEN                2
#define PRESTERA_RXQ_LEN                2
#define PRESTERA_Q_NUM                  8
#define PRESTERA_PORT_NUM               24
#define PRESTERA_DSA_TAG_SIZE           8
#define PRESTERA_MAC_HEADER_SIZE        12

#endif /* __mvCommonDefsh */

