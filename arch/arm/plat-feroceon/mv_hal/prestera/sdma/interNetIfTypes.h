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

#ifndef __interNetIfTypesh
#define __interNetIfTypesh

  
/********* Include files ******************************************************/
#include <common/os/gtOs.h>
#include <sdma/gtNetIf.h>
#include <common/macros/gtCommonFuncs.h>

#define NUM_OF_RX_QUEUES    (8)
#define NUM_OF_TX_QUEUES    (8)

#define TX_DESC_ALIGN       (16)  /* In bytes */
#define TX_SHORT_BUFF_SIZE  (8)   /* Bytes    */
#define TX_HEADER_SIZE      (16)  /* Bytes    */
#define TX_DESC_SIZE        (16)  /* In Bytes */

/* Resets the Tx descriptor's word1 & word2.                        */
#define TX_DESC_RESET(txDesc)                                           \
            (txDesc)->word1 = 0x0;                                      \
            (txDesc)->word2 = 0x0

/* Copy a tx descriptor from one struct to another      */
#define TX_DESC_COPY(dstDesc,srcDesc)                                   \
            (dstDesc)->buffPointer      = (srcDesc)->buffPointer;       \
            (dstDesc)->word2            = (srcDesc)->word2;             \
            (dstDesc)->word1            = (srcDesc)->word1;

/* Get / Set the Own bit field of the tx descriptor.    */
#define TX_DESC_GET_OWN_BIT(txDesc)                         \
            ( (hwByteSwap((txDesc)->word1) >> 31) & 0x1)
#define TX_DESC_SET_OWN_BIT(txDesc,val)                     \
            (U32_SET_FIELD((txDesc)->word1,31,1,(val & 1)))

/* Get / Set the AM bit field of the tx descriptor.    */
#define TX_DESC_GET_AM_BIT(txDesc)                         \
            ( (hwByteSwap((txDesc)->word1) >> 30) & 0x1 )
#define TX_DESC_SET_AM_BIT(txDesc,val)                     \
            (U32_SET_FIELD((txDesc)->word1,30,1,(val & 1)))
            

/* Get / Set the Int bit field of the tx descriptor.    */
#define TX_DESC_GET_INT_BIT(txDesc)                          \
        ( (hwByteSwap( (txDesc)->word1 ) >> 23) & 0x1)
#define TX_DESC_SET_INT_BIT(txDesc,val)                      \
            (U32_SET_FIELD((txDesc)->word1,23,1,(val & 1)))


/* Get / Set the First bit field of the tx descriptor.  */
#define TX_DESC_GET_FIRST_BIT(txDesc)                       \
            ((hwByteSwap((txDesc)->word1) >> 21) & 0x1)
#define TX_DESC_SET_FIRST_BIT(txDesc,val)                   \
            (U32_SET_FIELD((txDesc)->word1,21,1,(val & 1)))

/* Get / Set the Last bit field of the tx descriptor.   */
#define TX_DESC_GET_LAST_BIT(txDesc)                        \
            ((hwByteSwap((txDesc)->word1)>> 20) & 0x1)
#define TX_DESC_SET_LAST_BIT(txDesc,val)                    \
            (U32_SET_FIELD((txDesc)->word1,20,1,(val & 1)))

/* Get / Set the recalc crc bit field of the tx descriptor.   */
#define TX_DESC_GET_RECALC_CRC_BIT(txDesc)                        \
            ((hwByteSwap((txDesc)->word1)>> 12) & 0x1)
#define TX_DESC_SET_RECALC_CRC_BIT(txDesc,val)                    \
            (U32_SET_FIELD((txDesc)->word1,12,1,(val & 1)))

/* Get / Set the Byte Count field in the tx descriptor.       */
#define TX_DESC_GET_BYTE_CNT_FIELD(txDesc)                       \
        (U32_GET_FIELD(hwByteSwap((txDesc)->word2),16,14) )
#define TX_DESC_SET_BYTE_CNT(txDesc,val)                         \
            (U32_SET_FIELD((txDesc)->word2,16,14,(val & 0x3fff)))


/************************   RX Related Definitions  ***************************/

#define RX_DESC_ALIGN       (16)  /* In bytes */
#define RX_BUFF_ALIGN       (8)   /* In bytes */
#define RX_BUFF_SIZE_MULT   (8)   /* In bytes */
#define RX_DESC_SIZE        (16)  /* In Bytes */


#define RX_MAX_PACKET_DESC  (1000)    /* Max number of descriptors    */
                                    /* per Rx packet.               */

/* Resets the Rx descriptor's word1 & word2.                        */
#define RX_DESC_RESET(rxDesc) ((rxDesc)->word1 = hwByteSwap(0xA0000000))


/* Returns the Own bit field of the rx descriptor.           */
#define RX_DESC_GET_OWN_BIT(rxDesc) (((rxDesc)->word1) >> 31)

/* Sets the Own bit field of the rx descriptor.           */
#define RX_DESC_SET_OWN_BIT(rxDesc,val)   \
         (rxDesc)->word1 = hwByteSwap( (rxDesc)->word1); \
         U32_SET_FIELD( (rxDesc)->word1,31,1,(val & 1));      \
         (rxDesc)->word1 = hwByteSwap((rxDesc)->word1)

/* Returns the First bit field of the rx descriptor.                */
#define RX_DESC_GET_FIRST_BIT(rxDesc) ((((rxDesc)->word1) >> 27) & 0x1)

/* Returns the Last bit field of the rx descriptor.                 */
#define RX_DESC_GET_LAST_BIT(rxDesc) ((((rxDesc)->word1) >> 26) & 0x1)

/* Returns the Resource error bit field of the rx descriptor.       */
#define RX_DESC_GET_REC_ERR_BIT(rxDesc) ((((rxDesc)->word1) >> 28) & 0x1)

/* Returns the bus error bit field of the rx descriptor.       */
#define RX_DESC_GET_BUS_ERR_BIT(rxDesc) ((((rxDesc)->word1) >> 30) & 0x1)

/* Returns the enable interrupt bit field of the rx descriptor.       */
#define RX_DESC_GET_EI_BIT(rxDesc) ((((rxDesc)->word1) >> 29) & 0x1)

/* Sets the packet encapsulation field of the rx descriptor.       */
#define RX_DESC_SET_EI_BIT(rxDesc,val)                     \
         (rxDesc)->word1 = hwByteSwap( (rxDesc)->word1);   \
         U32_SET_FIELD( (rxDesc)->word1,29,1,(val & 1));      \
         (rxDesc)->word1 = hwByteSwap((rxDesc)->word1)

/* Return the valid crc bit from the second word of an Rx desc.     */
#define RX_DESC_GET_VALID_CRC_BIT(rxDesc)   \
            ((MV_BOOL)((((rxDesc)->word2) >> 30) & 0x1))

/* Return the buffer size field from the second word of an Rx desc. */
/* Make sure to set the lower 3 bits to 0.                          */
#define RX_DESC_GET_BUFF_SIZE_FIELD(rxDesc)             \
            (((hwByteSwap((rxDesc)->word2) >> 3) & 0x7FF) << 3)

#define RX_DESC_SET_BUFF_SIZE_FIELD(rxDesc,val)         \
            (rxDesc)->word2 = hwByteSwap((rxDesc)->word2);  \
            U32_SET_FIELD((rxDesc)->word2,3,11,(val & 0x7ff) );      \
            (rxDesc)->word2 = hwByteSwap((rxDesc)->word2)

/* Return the byte count field from the second word of an Rx desc.  */
/* Make sure to set the lower 3 bits to 0.                          */
#define RX_DESC_GET_BYTE_COUNT_FIELD(rxDesc)        \
            ((((rxDesc)->word2) >> 16) & 0x3FFF)

/****************** Marvell tag ***********************************************/
/* Return the Tag command field from the first word of a DSA          */
#define DSA_GET_TAG_CMD_FIELD(hwDsa)  (((hwDsa).word0 >> 30) & 0x3)

/* Return the SrcTagged/TrgTagged field from the first word of a DSA          */
#define DSA_GET_SRC_TRG_TAGGED_FIELD(hwDsa) (((hwDsa).word0 >> 29) & 0x1)

/* Return the SrcDev/TrgDev field from the first word of a DSA          */
#define RX_DSA_GET_SRC_TRG_DEV_FIELD(hwDsa) ((((hwDsa).word0) >> 24) & 0x1F)

/* Return the SrcPort/TrgPort field from the first and seconds words of a DSA */
#define DSA_GET_SRC_TRG_PORT_FIELD(hwDsa)             \
    ((((((hwDsa).word1) >> 10) & 0x3) << 5) | ((((hwDsa).word0) >> 19) & 0x1F))
    
/* Return the UP field from the first word of a DSA          */
#define DSA_GET_UP_FIELD(hwDsa)  ((((hwDsa).word0) >> 13) & 0x7)

/* Return the CPU code field from the first word of a DSA          */
#define RX_DSA_GET_CPU_CODE_FIELD(hwDsa)             \
      ((((((hwDsa).word0) >> 16) & 0x7) << 1) | ((((hwDsa).word0) >> 12) & 0x1))

/* Return the VID field from the first word of a DSA          */
#define DSA_GET_VID_FIELD(hwDsa)  (((hwDsa).word0) & 0xFFF)

/* Return the Extend field from the second word of a DSA          */
#define RX_DSA_GET_EXTEND_FIELD(hwDsa)  ((((hwDsa).word1) >> 31) & 0x1)

/* Return the CFI field from the second word of a DSA          */
#define RX_DSA_GET_CFI_FIELD(hwDsa)  ((((hwDsa).word1) >> 30) & 0x1)

/* Return the Truncated field from the second word of a DSA          */
#define RX_DSA_GET_TRUNCATED_FIELD(hwDsa)  ((((hwDsa).word1) >> 26) & 0x1)

/* Return the Packet original byte count field from the second word of a DSA  */
#define RX_DSA_GET_PCKT_ORIG_BYTE_COUNT_FIELD(hwDsa)  \
            ((((hwDsa).word1) >> 12) & 0x3FFF)

/* Return the SrcTrg field from the second word of a DSA  */
#define RX_DSA_GET_SRC_TRG_FIELD(hwDsa)  ((((hwDsa).word1) >> 8) & 0x1)
    
/* Return the Long CPU code field from the second word of a DSA  */
#define RX_DSA_GET_LONG_CPU_CODE_FIELD(hwDsa)  (((hwDsa).word1) & 0xFF)


/* Set the Tag command field from the first word of a DSA          */
#define DSA_SET_TAG_CMD_FIELD(hwDsa, val)  \
    (U32_SET_FIELD((hwDsa).word0,30,1,(val & 1)))

/* Set the VID field from the first word of a DSA          */
#define DSA_SET_VID_FIELD(hwDsa, val)  \
    (U32_SET_FIELD((hwDsa).word0,0,12,(val & 0xfff)))

/* Set the SrcTagged/TrgTagged field from the first word of a DSA          */
#define DSA_SET_SRC_TRG_TAGGED_FIELD(hwDsa, val)   \
    (U32_SET_FIELD((hwDsa).word0,29,1,(val & 1)))

/* Set the SrcDev/TrgDev field from the first word of a DSA          */
#define RX_DSA_SET_SRC_TRG_DEV_FIELD(hwDsa, val) \
    (U32_SET_FIELD((hwDsa).word0,24,5,(val & 0x1f))); 

/* Set the SrcPort/TrgPort field from the first and seconds words of a DSA */
#define DSA_SET_SRC_TRG_PORT_FIELD(hwDsa, val)        \
    (U32_SET_FIELD((hwDsa).word0,19,5,(val & 0x1f)));          \
    (U32_SET_FIELD((hwDsa).word1,10,3,(val >> 5)))
        
/* Set the UP field from the first word of a DSA          */
#define DSA_SET_UP_FIELD(hwDsa, val)  \
     (U32_SET_FIELD((hwDsa).word0,13,3,(val & 7)))

/* Set the DropOnSource field from Rx DSA          */
#define RX_DSA_SET_DROP_ON_SRC_FIELD(hwDsa, val)   \
    (U32_SET_FIELD((hwDsa).word1,29,1,(val & 1)))
/* Return the DropOnSource field from the DSA  */
#define RX_DSA_GET_DROP_ON_SRC_FIELD(hwDsa)  ((((hwDsa).word1) >> 29) & 0x1)

/* Set the PacketIsLooped field from Rx DSA          */
#define RX_DSA_SET_PKT_IS_LOOPED_SRC_FIELD(hwDsa, val)   \
    (U32_SET_FIELD((hwDsa).word1,28,1,(val & 1)))
/* Return the PacketIsLooped field from the DSA  */
#define RX_DSA_GET_PKT_IS_LOOPED_FIELD(hwDsa)  ((((hwDsa).word1) >> 28) & 0x1)

/* Set the OrigIsTrunk field from Rx DSA          */
#define RX_DSA_SET_ORIG_IS_TRUNK_SRC_FIELD(hwDsa, val)   \
    (U32_SET_FIELD((hwDsa).word1,27,1,(val & 1)))
/* Return the OrigIsTrunk field from the DSA  */
#define RX_DSA_GET_ORIG_IS_TRUNK_FIELD(hwDsa)  ((((hwDsa).word1) >> 27) & 0x1)

/************************ From CPU DSA tag fields *****************************/

/* Return the VIDX code field from the first & second word of a DSA          */
#define TX_DSA_GET_VIDX_FIELD(hwDsa)             \
   ((((((hwDsa).word1) >> 12) & 0x3) << 10) | ((((hwDsa).word0) >> 19) & 0x3FF))

/* Set the VIDX field from the first & second word of a DSA          */
#define TX_DSA_SET_VIDX_FIELD(hwDsa,val)             \
      (U32_SET_FIELD((hwDsa).word0,19,10,(val & 0x3ff))); \
      (U32_SET_FIELD((hwDsa).word1,12,2,(val >> 10)))

/* Set the USE_VIDX field from the first word of a DSA          */
#define TX_DSA_SET_USE_VIDX_FIELD(hwDsa,val)             \
      (U32_SET_FIELD((hwDsa).word0,18,1,(val & 1)))
    
/* Set the TC field from the first & second word of a DSA          */
#define TX_DSA_SET_TC_FIELD(hwDsa,val)             \
      (U32_SET_FIELD((hwDsa).word0,17,1,(val & 1))); \
      (U32_SET_FIELD((hwDsa).word1,14,1,((val >> 1) & 1))); \
      (U32_SET_FIELD((hwDsa).word1,27,1,((val >> 2) & 1)))
      
/* Set the CFI field from the first word of a DSA          */
#define TX_DSA_SET_CFI_FIELD(hwDsa,val)             \
      (U32_SET_FIELD((hwDsa).word0,16,1,(val & 1)))

/* Set the Extend field from the first word of a DSA          */
#define TX_DSA_SET_EXTEND0_FIELD(hwDsa,val)             \
      (U32_SET_FIELD((hwDsa).word0,12,1,(val & 1)))

/* Set the Extend field from the second word of a DSA          */
#define TX_DSA_SET_EXTEND1_FIELD(hwDsa,val)             \
      (U32_SET_FIELD((hwDsa).word1,31,1,(val & 1)))

/* Set the EgressFilterEn field from the second word of a DSA          */
#define TX_DSA_SET_EGRESS_FLTR_EN_FIELD(hwDsa,val)             \
      (U32_SET_FIELD((hwDsa).word1,30,1,(val & 1)))

/* Set the Cascade Control field from the second word of a DSA          */
#define TX_DSA_SET_CASCADE_CNTRL_FIELD(hwDsa,val)             \
      (U32_SET_FIELD((hwDsa).word1,29,1,(val & 1)))

/* Set the EgressFilterReg field from the second word of a DSA          */
#define TX_DSA_SET_EGRESS_FLTR_REG_FIELD(hwDsa,val)             \
      (U32_SET_FIELD((hwDsa).word1,28,1,(val & 1)))

/* Set the PacketIsLooped field from the second word of a DSA          */
#define TX_DSA_SET_PKT_IS_LOOPED_FIELD(hwDsa,val)             \
      (U32_SET_FIELD((hwDsa).word1,25,1,(val & 1)))

/* Set the DropOnSource field from the second word of a DSA          */
#define TX_DSA_SET_DROP_ON_SRC_FIELD(hwDsa,val)             \
      (U32_SET_FIELD((hwDsa).word1,26,1,(val & 1)))

    /* Set the Src ID field from the second word of a DSA          */
#define TX_DSA_SET_SRC_ID_FIELD(hwDsa,val)             \
      (U32_SET_FIELD((hwDsa).word1,20,5,(val & 0x1f)))

/* Set SrcDev field from the first word of a DSA          */
#define TX_DSA_SET_SRC_DEV_FIELD(hwDsa)     \
    (U32_SET_FIELD((hwDsa).word1,15,5,(val & 0x1f)));
/* Set TrgDev field from the first word of a DSA          */
#define TX_DSA_SET_TRG_DEV_FIELD(hwDsa)     \
    (U32_SET_FIELD((hwDsa).word0,24,5,(val & 0x1f)));

/* Set the ExcludeIsTrunk field from the second word of a DSA          */
#define TX_DSA_SET_EXCL_IS_TRUNK_FIELD(hwDsa,val)             \
      (U32_SET_FIELD((hwDsa).word1,11,1,(val & 1)))

/* Set the MirrotTo All CPUs field from the second word of a DSA          */
#define TX_DSA_SET_MIRROR_TO_ALL_CPU_FIELD(hwDsa,val)             \
      (U32_SET_FIELD((hwDsa).word1,10,1,(val & 1)))

/* Set the ExcludedTrunk field from the second word of a DSA          */
#define TX_DSA_SET_EXCL_TRUNK_FIELD(hwDsa,val)             \
      (U32_SET_FIELD((hwDsa).word1,0,7,(val & 0x7f)))

/* Set the ExcludedDev field from the second word of a DSA          */
#define TX_DSA_SET_EXCL_DEV_FIELD(hwDsa,val)             \
      (U32_SET_FIELD((hwDsa).word1,0,5,(val & 0x1f)));

/* Set the ExcludedPort field from the second word of a DSA          */
#define TX_DSA_SET_EXCL_PORT_FIELD(hwDsa,val)             \
      (U32_SET_FIELD((hwDsa).word1,5,6,(val & 0x3f)))


/********* Typedefs ***********************************************************/
/*
 * typedef: enum MV_NET_DIRECTION
 *
 * Description: transfer direction: RX or TX
 *
 * Enumerations:
 *       MV_NET_RX  - incomming packets.
 *       MV_NET_TX  - outgoing packets.
 */
typedef enum
{
    MV_NET_RX=0,
    MV_NET_TX
} MV_NET_DIRECTION;

/*
 * Typedef: MV_RX_CPU_CODE
 *
 * Description: Defines the different CPU codes that indicate the reason for
 *              sending a packet to the CPU.
 *
 * Fields:
 *
 *  BPDU TRAP               - BPDU packet trapped to the host CPU.
 *  FDB ENTEY TRAP/MIRROR   - Packet trapped or mirrored to the host CPU due
 *                            to FDB Entry command set to Trap or Mirror on 
 *                            the packet's MAC SA or MAC DA.
 *  MAC RANGE TRAP          - 98DX240 Code for MAC Range Trapping.
 *  ARP BROADCAST TRAP/MIRROR - ARP Broadcast packet trapped or mirrored to the 
 *                            host CPU.
 *  IPv4 IGMP TRAP/MIRROR   - IPv4 IGMP packet trapped or mirrored to the host 
 *                            CPU.
 *  MAC ADDR INTERVENTION   - 98DX240 Code for FDB command set to intervention.
 *  LEARN DISABLE UNKNOWN   - Packet with unknown Source Address received on a 
 *  SOURCE MAC ADDR TRAP      learning disabled port with learning disable 
 *                            command set to 'Trap packets with unknown Source 
 *                            Address'.
 *  MAC RANGE TRAP          - 98DX240 Code for MAC Range Mirroring
 *  LEARN DISABLE UNKNOWN   - Packet with unknown Source Address received on a
 *  SOURCE MAC ADDR MIRROR    learning disabled port with learning disable 
 *                            command set to 'Mirror packets with unknown Source 
 *                            Address'.
 *  BRIDGED UNKNOWN UNICAST - 98DX240 Code: Packet with unknown Unicast
 *  TRAP/MIRROR               Destination Address assigned to a VLAN with 
 *                            Unknown Unicast Command set to Trap or Mirror.
 *  BRIDGED UNREGISTERED    - 8DX240 Code: packet with unknown Destination
 *  MULTICAST TRAP/MIRROR     Address assigned to a VLAN with Unknown Unicast 
 *                            Command set to Trap or Mirror
 *  IEEE RESERVED MULTICAST - Packet with MAC DA in the IEEE Reserved Multicast
 *  ADDR TRAP/MIRROR          range trapped or mirrored to the host CPU.
 *  IPv6 ICMP TRAP/MIRROR   - IPv6 ICMP packet trapped or mirrored to the host.
 *  IPv4/IPv6 LINK-LOCAL    - IPv4 or IPv6 packets with DIP in the Link Local 
 *  MULTICAST DIP TRAP/MIRROR Range trapped or mirrored to the host CPU.
 *  IPv4 RIPv1 MIRROR       - IPv4 RIPv1 packet mirrored to the host CPU.
 *  IPv6 NEIGHBOR SOLICITATION - IPv6 neighbor solicitation packet trapped or
 *  TRAP/MIRROR                  mirrored to the host CPU.
 *  IPv4 BROADCAST TRAP/MIRROR - IPv4 Broadcast packet assigned to a VLAN with 
 *                               IPv4 Broadcast Command set to Trap or Mirror.
 *  NON IPv4 BROADCAST      - Non IPv4 Broadcast packet assigned to a VLAN with
 *  TRAP/MIRROR               non IPv4 Broadcast Command set to Trap or Mirror.
 *  CISCO CONTROL MULTICAST - Multicast packet with a MAC DA in the CISCO AUI 
 *  MAC ADDR TRAP/MIRROR      range trapped or mirrored to the host CPU.
 *  BRIDGED NON IPv4/IPv6   - Non IPv4/IPv6 Unregistered Multicast packet  
 *  UNREGISTERED              assigned to a VLAN with non IPv4/IPv6 Unregistered
 *  MULTICAST TRAP/MIRROR     Multicast Command set to Trap or Mirror.
 *  BRIDGED IPv4 UNREGISTERED - IPv4 Unregistered Multicast packet assigned to 
 *  MULTICAST TRAP/MIRROR       a VLAN with IPv4 Unregistered Multicast Command 
 *                              set to Trap or Mirror.
 *  BRIDGED IPv6 UNREGISTERED - IPv6 Unregistered Multicast packet assigned to 
 *  MULTICAST TRAP/MIRROR       a VLAN with IPv6 Unregistered Multicast Command 
 *                              set to Trap or Mirror.
 *  ROUTED PACKET FORWARD   - Packet forwarded to the host CPU by the Router.
 *  BRIDGED PACKET FORWARD  - Packet forwarded to the host CPU by one of the 
 *                            following engines in the device:
 *                           . Redirected by the PCL engine to the CPU port.
 *                           . MAC DA is associated with the CPU port.
 *                           . Private VLAN edge target set to CPU port.
 *  INGRESS MIRRORED TO     - Ingress mirrored packets to the analyzer port, 
 *  ANALYAER                  when the ingress analyzer port number is set to 
 *                            the CPU port number.
 *  EGRESS MIRRORED TO      - Egress mirrored packets to the analyzer port, 
 *  ANALYZER                  when the egress analyzer port number is set to the
 *                            CPU port number.
 *  MAIL FROM NEIGHBOR CPU  - A packet sent to the local host CPU as a mail from 
 *                            the neighboring CPU. 
 *  CPU TO CPU              - A packet sent to the local host CPU, from one of 
 *                            the other host CPUs in the system.
 *  EGRESS SAMPLED          - Packet mirrored to the host CPU by the egress STC 
 *                            mechanism.
 *  INGRESS SAMPLED         - Packet mirrored to the host CPU by the ingress STC 
 *                            mechanism.
 *  INVALID USER DEFINED    - Packet trapped/mirrored to the host CPU by the
 *  SELECTED BYTES ON PCL     Policy Engine due to the following:
 *  KEY TRAP/MIRROR             User-defined bytes in the key could not be
 *                              parsed due to packet header length or its format.
 */
typedef enum
{
    RESERVED0                                       = 0,
    RESERVED1                                       = 1,
    BPDU_TRAP                                       = 2,
    FDB_ENTRY_TRAP_MIRROR                           = 3,
    MAC_RANGE_TRAP                                  = 4,
    ARP_BROADCAST_TRAP_MIRROR                       = 5,
    IPv4_IGMP_TRAP_MIRROR                           = 6,
    MAC_ADDR_INTERVENTION                           = 7,
    LEARN_DISABLE_UNKNOWN_SOURCE_MAC_ADDR_TRAP      = 8,
    MAC_RANGE_MIRROR                                = 9,
    LEARN_DISABLE_UNKNOWN_SOURCE_MAC_ADDR_MIRROR    = 10,
    BRIDGED_UNKNOWN_UNICAST_TRAP_MIRROR             = 11,
    BRIDGED_UNREGISTERED_MULTICAST_TRAP_MIRROR      = 12,
    IEEE_RESERVED_MULTICAST_ADDR_TRAP_MIRROR        = 13,
    IPv6_ICMP_TRAP_MIRROR                           = 14,
    RESERVED15,
    IPv4_IPv6_LINK_LOCAL_MULTICAST_DIP_TRAP_MIRROR  = 16,
    IPv4_RIPv1_MIRROR                               = 17,
    IPv6_NEIGHBOR_SOLICITATION_TRAP_MIRROR          = 18,
    IPv4_BROADCAST_TRAP_MIRROR                      = 19,
    NON_IPv4_BROADCAST_TRAP_MIRROR                  = 20,
    CISCO_CONTROL_MULTICAST_MAC_ADDR_TRAP_MIRROR    = 21,
    BRIDGED_NON_IPv4_IPv6_UNREGISTERED_MULTICAST_TRAP_MIRROR   = 22,
    BRIDGED_IPv4_UNREGISTERED_MULTICAST_TRAP_MIRROR = 23,
    BRIDGED_IPv6_UNREGISTERED_MULTICAST_TRAP_MIRROR = 24,
    UNKNOWN_UNICAST_TRAP_TO_CPU                     = 25,
    IEEE_RES_MC_ADDR_TRAP_MIRROR1                   = 26,
    IEEE_RES_MC_ADDR_TRAP_MIRROR2                   = 27,
    IEEE_RES_MC_ADDR_TRAP_MIRROR3                   = 28,
    IPV4_6_LINK_LOCAL_MC_ADDR_TRAP_MIRROR1          = 29,
    IPV4_6_LINK_LOCAL_MC_ADDR_TRAP_MIRROR2          = 30,
    IPV4_6_LINK_LOCAL_MC_ADDR_TRAP_MIRROR3          = 31,
    UDP_BC_TRAP_MIRROR_0                            = 32,
    UDP_BC_TRAP_MIRROR_1                            = 33,
    UDP_BC_TRAP_MIRROR_2                            = 34,
    UDP_BC_TRAP_MIRROR_3                            = 35,
    SEC_AUTO_LEARN_UNK_SRC_TRAP                     = 36,
    ROUTED_PACKET_FORWARD                           = 64,
    BRIDGED_PACKET_FORWARD                          = 65, 
    INGRESS_MIRRORED_TO_ANALYZER                    = 66,               
    EGRESS_MIRRORED_TO_ANALYZER                     = 67,
    MAIL_FROM_NEIGHBOR_CPU                          = 68,
    CPU_TO_CPU                                      = 69,
    EGRESS_SAMPLED                                  = 70,
    INGRESS_SAMPLED                                 = 71,
    INVALID_USER_DEF_SELECTED_BYTES_ON_PCL_KEY_TRAP_MIRROR = 74,
    IPV4_TT_HEADER_ERROR                            = 75,
    IPV4_TT_OPTION_FRAG_ERROR                       = 76,
    IPV4_TT_UNSUP_GRE_ERROR                         = 77,
    RESERVED_78                                     = 78,
    RESERVED_79                                     = 79,
    RESERVED_80                                     = 80,
    RESERVED_81                                     = 81,
    RESERVED_82                                     = 82,
    RESERVED_83                                     = 83,
    RESERVED_84                                     = 84,
    RESERVED_85                                     = 85,
    RESERVED_86                                     = 86,
    RESERVED_87                                     = 87,
    RESERVED_88                                     = 88,
    RESERVED_89                                     = 89,
    RESERVED_90                                     = 90,
    RESERVED_91                                     = 91,
    RESERVED_92                                     = 92,
    RESERVED_93                                     = 93,
    RESERVED_94                                     = 94,
    RESERVED_95                                     = 95,
    RESERVED_96                                     = 96,
    RESERVED_97                                     = 97,
    RESERVED_98                                     = 98,
    RESERVED_99                                     = 99,
    RESERVED_100                                    = 100,
    RESERVED_101                                    = 101,
    RESERVED_102                                    = 102,
    RESERVED_103                                    = 103,
    RESERVED_104                                    = 104,
    RESERVED_105                                    = 105,
    RESERVED_106                                    = 106,
    RESERVED_107                                    = 107,
    RESERVED_108                                    = 108,
    RESERVED_109                                    = 109,
    RESERVED_110                                    = 110,
    RESERVED_111                                    = 111,
    RESERVED_112                                    = 112,
    RESERVED_113                                    = 113,
    RESERVED_114                                    = 114,
    RESERVED_115                                    = 115,
    RESERVED_116                                    = 116,
    RESERVED_117                                    = 117,
    RESERVED_118                                    = 118,
    RESERVED_119                                    = 119,
    RESERVED_120                                    = 120,
    RESERVED_121                                    = 121,
    RESERVED_122                                    = 122,
    RESERVED_123                                    = 123,
    RESERVED_124                                    = 124,
    RESERVED_125                                    = 125,
    RESERVED_126                                    = 126,
    RESERVED_127                                    = 127,
    RESERVED_128                                    = 128,
    RESERVED_129                                    = 129,
    RESERVED_130                                    = 130,
    RESERVED_131                                    = 131,
    RESERVED_132                                    = 132,
    IPV4_TTL_EXCEEDED                               = 133,
    IPV4_6_MTU_EXCEEDED                             = 134,
    IPV6_HOPLIMIT_EXCEEDED                          = 135,
    IPV4_6_ILLEGAL_ADDR_ERROR                       = 136,
    IPV4_HEADER_ERROR                               = 137,
    IPV4_6_DIP_DA_MISMATCH                          = 138,
    IPV6_HEADER_ERROR                               = 139,
    IPV4_6_UC_SIP_SA_MISMATCH                       = 140,
    IPV4_OPTIONS                                    = 141,
    IPV6_NON_HBH_OPTION                             = 142,
    IPV6_HBH_OPTIONS                                = 143,
    RESERVED_144                                    = 144,
    RESERVED_145                                    = 145,
    RESERVED_146                                    = 146,
    RESERVED_147                                    = 147,
    RESERVED_148                                    = 148,
    RESERVED_149                                    = 149,
    RESERVED_150                                    = 150,
    RESERVED_151                                    = 151,
    RESERVED_152                                    = 152,
    RESERVED_153                                    = 153,
    RESERVED_154                                    = 154,
    RESERVED_155                                    = 155,
    RESERVED_156                                    = 156,
    RESERVED_157                                    = 157,
    RESERVED_158                                    = 158,
    IPV6_SCOPE                                      = 159,
    IPV4_UC_ROUTE_ENTRY_0                           = 160,
    IPV4_UC_ROUTE_ENTRY_1                           = 161,
    IPV4_UC_ROUTE_ENTRY_2                           = 162,
    IPV4_UC_ROUTE_ENTRY_3                           = 163,
    IPV4_MC_ROUTE_ENTRY_0                           = 164,
    IPV4_MC_ROUTE_ENTRY_1                           = 165,
    IPV4_MC_ROUTE_ENTRY_2                           = 166,
    IPV4_MC_ROUTE_ENTRY_3                           = 167,
    IPV6_UC_ROUTE_ENTRY_0                           = 168,
    IPV6_UC_ROUTE_ENTRY_1                           = 169,
    IPV6_UC_ROUTE_ENTRY_2                           = 170,
    IPV6_UC_ROUTE_ENTRY_3                           = 171,
    IPV6_MC_ROUTE_ENTRY_0                           = 172,
    IPV6_MC_ROUTE_ENTRY_1                           = 173,
    IPV6_MC_ROUTE_ENTRY_2                           = 174,
    IPV6_MC_ROUTE_ENTRY_3                           = 175,
    IPV4_6_UC_RPF_FAIL                              = 176,
    IPV4_6_MC_ROUTE_RPF_FAIL                        = 177,
    IPV4_6_MC_MLL_RPF_FAIL                          = 178,
    ARP_BC_TO_ME                                    = 179,
    IPV4_UC_ICMP_REDIRECT                           = 180,
    IPV6_UC_ICMP_REDIRECT                           = 181,
    RESERVED_182                                    = 182,
    RESERVED_183                                    = 183,
    RESERVED_184                                    = 184,
    RESERVED_185                                    = 185,
    RESERVED_186                                    = 186,
    RESERVED_187                                    = 187,
    ARP_REPLY_TO_ME                                 = 188,
    CPU_TO_ALL_CPUS                                 = 189,
    TCP_SYN_TO_CPU                                  = 190,
    PACKET_TO_VIRTUAL_ROUTER_PORT                   = 191,
    USER_DEFINED_192                                = 192,
    USER_DEFINED_193                                = 193,
    USER_DEFINED_194                                = 194,
    USER_DEFINED_195                                = 195,
    USER_DEFINED_196                                = 196,
    USER_DEFINED_197                                = 197,
    USER_DEFINED_198                                = 198,
    USER_DEFINED_199                                = 199,
    USER_DEFINED_200                                = 200,
    USER_DEFINED_201                                = 201,
    USER_DEFINED_202                                = 202,
    USER_DEFINED_203                                = 203,
    USER_DEFINED_204                                = 204,
    USER_DEFINED_205                                = 205,
    USER_DEFINED_206                                = 206,
    USER_DEFINED_207                                = 207,
    USER_DEFINED_208                                = 208,
    USER_DEFINED_209                                = 209,
    USER_DEFINED_210                                = 210,
    USER_DEFINED_211                                = 211,
    USER_DEFINED_212                                = 212,
    USER_DEFINED_213                                = 213,
    USER_DEFINED_214                                = 214,
    USER_DEFINED_215                                = 215,
    USER_DEFINED_216                                = 216,
    USER_DEFINED_217                                = 217,
    USER_DEFINED_218                                = 218,
    USER_DEFINED_219                                = 219,
    USER_DEFINED_220                                = 220,
    USER_DEFINED_221                                = 221,
    USER_DEFINED_222                                = 222,
    USER_DEFINED_223                                = 223,
    USER_DEFINED_224                                = 224,
    USER_DEFINED_225                                = 225,
    USER_DEFINED_226                                = 226,
    USER_DEFINED_227                                = 227,
    USER_DEFINED_228                                = 228,
    USER_DEFINED_229                                = 229,
    USER_DEFINED_230                                = 230,
    USER_DEFINED_231                                = 231,
    USER_DEFINED_232                                = 232,
    USER_DEFINED_233                                = 233,
    USER_DEFINED_234                                = 234,
    USER_DEFINED_235                                = 235,
    USER_DEFINED_236                                = 236,
    USER_DEFINED_237                                = 237,
    USER_DEFINED_238                                = 238,
    USER_DEFINED_239                                = 239,
    USER_DEFINED_240                                = 240,
    USER_DEFINED_241                                = 241,
    USER_DEFINED_242                                = 242,
    USER_DEFINED_243                                = 243,
    USER_DEFINED_244                                = 244,
    USER_DEFINED_245                                = 245,
    USER_DEFINED_246                                = 246,
    USER_DEFINED_247                                = 247,
    USER_DEFINED_248                                = 248,
    USER_DEFINED_249                                = 249,
    USER_DEFINED_250                                = 250,
    USER_DEFINED_251                                = 251,
    USER_DEFINED_252                                = 252,
    USER_DEFINED_253                                = 253,
    USER_DEFINED_254                                = 254,
    USER_DEFINED_255                                = 255,
    
    LAST_CODE

}MV_RX_CPU_CODE;


typedef enum
{
    DSA_TO_CPU,
    DSA_FROM_CPU
}MV_DSA_TYPE;

typedef enum
{
    INGRESS_PIPE,
    EGRESS_PIPE
}MV_PIPE_TYPE;

/*
 * typedef: struct MV_MTAG_TO_CPU
 *
 * Description:  
 *          Marvell (DSA) tag to CPU structure
 * Fields:
 *      TagCommand   - 0 - To CPU, 
 *                     1 - From CPU
 *      SrcTagged    - 0 - Packet was received from a network port untagged.
 *                     1 - Packet was received from a network port tagged.
 *      SrcDev       - number of the Source Device on which the packet was 
 *                     received.
 *      SrcPort      - number of the Source Port on which the packet was 
 *                     received.
 *      CPUCode      - must be set to 0xF to indicate an Extended DSA tag.
 *      UP           - The 802.1p User Priority field assigned to the packet.
 *      VID          - The VID assigned to the packet.
 *      Extend       - This must be set to 0.
 *      CFI          - When SrcTagged = 1, this is the VLAN Tag CFI bit with 
 *                          which the packet was received from the network port.
 *                     When SrcTagged = 0, the packet was received untagged and 
 *                          the CFI bit is assigned by the device to '0'.
 *      Truncated    - Packet sent to CPU is truncated.
 *      flowId      - The packet Flow ID.
 *      SrcTrg       - indicates the type of data forwarded to the CPU.
 *                     0 - The packet was forwarded to the CPU by the Ingress 
 *                         pipe and this tag contains the packets source info.
 *                     1 - The packet was forwarded to the CPU by the Egress 
 *                         pipe and this tag contains the pckt destination info.
 *      LongCPUCode  - 8 bits CPU code
 */
typedef struct
{
    struct 
    {
        MV_DSA_TYPE     tagCmd ;
        MV_BOOL         srcTagged;
        MV_U8           srcDev;
        MV_U8           srcPort;
        MV_U8           cpuCode;
        MV_U8           up;         
        MV_U16          vid;        
        MV_BOOL         extend;
        MV_U8           cfi;    
        MV_BOOL         packetIsLooped;  
        MV_BOOL         dropOnSource;  
        MV_BOOL         isTrunk;
        MV_BOOL         truncated;
        MV_PIPE_TYPE    srcTrg;
        MV_U8           longCpuCode;
        MV_U16          pktOrigBc;
    }swDsa;
    struct
    {
        MV_U32 word0;
        MV_U32 word1;
    }hwDsa;
}MV_RX_DSA;

/*
 * typedef: struct MV_MTAG_FROM_CPU
 *
 * Description:  
 *          Marvell (DSA) tag from CPU structure
 * Fields:
 *      TagCommand   - 0 - To CPU, 
 *                     1 - From CPU
 *      TrgTagged    - 0 - Packet is sent via network port untagged.
 *                     1 - Packet is sent via network port tagged.
 *      mllPtr       - mll pointer
 *      TrgDev       - Target Device to which the packet is forwarded. 
 *      TrgPort      - Target Port to which the packet is forwarded.
 *      VIDx         - vidx according to which the packet is to be forwarded.
 *      Use_VIDx     - 0 - Packet from the CPU is a Unicast packet forwarded to 
 *                         a target specified in this tag.
 *                     1 - Packet from the CPU is a Multicast packet forwarded 
 *                         to the VLAN-ID and Multicast group index in this tag.
 *      UP           - The 802.1p User Priority field assigned to the packet.
 *      TC           - The packet Traffic Class.
 *      Extend0      - Extended bit in word 0 must be set to 1.
 *      CFI          - When TrgTagged = 1, this is the VLAN Tag CFI bit with 
 *                          which the packet was received from the network port.
 *                     When TrgTagged = 0, the packet was received untagged and 
 *                          the CFI bit is assigned by the device to '0'.
 *      VID          - VID assigned to the packet.
 *      Extend1      - Extended bit in word 1 must be set to 0.
 *      EgressFltrEn - Egress filtering enable.
 *      CascadeCntr  - This bit indicates which TC is assigned to the packet 
 *                     when it is forwarded over a cascading/stacking port.
 *                     0 - If the packet is queued on a port that is enabled for
 *                         Data QoS mapping (typically a cascade port), the 
 *                         packet is queued according to the data traffic {TC, 
 *                         DP} mapping table, which maps the DSA tag TC and DP 
 *                         to a cascade port TC and DP. On a port that is not 
 *                         enabled for Data QoS mapping (a non-cascade ports), 
 *                         the packet is queued according to the DSA tag info. 
 *                      1 - If the packet is queued on a port that is enabled 
 *                          for Control QoS mapping (typically a cascade port), 
 *                          the packet is queued according to the configured 
 *                          Control TC and DP. On a port that is not enabled for 
 *                          Control QoS mapping (non-cascade ports), the packet 
 *                          is queued according to the DSA tag TC and DP.
 *      EgressFltrReg- Indicates that the packet is Egress filtered as a 
 *                     registered packet.
 *      DP           - Packet's drop precedence. 
 *      Src_ID       - Packet's Source ID.
 *      SrcDev       - Packet's Source Device Number.
 *      ExcIsTrunk   - A Trunk is excluded from the Multicast group.
 *      ExcTrunk     - The excluded trunk number.
 *      ExcPort      - Together with ExcludedDev, specifies the port that is 
 *                     excluded from the Multicast group.
 *      ExcDev       - Together with ExcludedPort, specifies the port that is 
 *                     excluded from the Multicast group.
 *      MailBoxToNeighborCPU - Mail box to Neighbor CPU, for CPU to CPU mail box 
 *                             communication. 
 */
typedef struct
{
    struct 
    {
        MV_DSA_TYPE     tagCmd;
        MV_BOOL         trgTagged;
        MV_BOOL         useVidx;
        MV_U8           trgDev;
        MV_U8           trgPort; 
        MV_U16          vidx;
        MV_U8           up;
        MV_U8           tc;
        MV_U16          vid;
        MV_BOOL         extend0;
        MV_BOOL         extend1;
        MV_U8           cfi;                             
        MV_BOOL         egressFltrEn;                       
        MV_BOOL         cascadeCntr;  
        MV_BOOL         egressFltrReg;
        MV_BOOL         dropOnSrc;           
        MV_BOOL         pktIsLooped;           
        MV_U8           srcId;       
        MV_U8           srcDev;
        MV_BOOL         excludeIsTrunk;   
        MV_U8           excludeTrunk;
        MV_U8           excludePort;
        MV_U8           excludeDev;
        MV_BOOL         mirrorToAllCpus;
        MV_BOOL         mailBoxToNeigCpu;
    }swDsa;
    struct
    {
        MV_U32 word0;
        MV_U32 word1;
    }hwDsa;

}MV_TX_DSA;                                          
                                                     


/*******************************************************************************
* coreSetTxDesc
*
* DESCRIPTION: 
*       Set the TX descriptor parameters
*
* INPUTS:
*       devNum     - Device number.
*       queue      - Queue number.
*       descNum    - Descriptor number. The first descriptor is number 1.
*       txDesc     - TX descriptor parameters.
*       bufferPtr  - Buffer pointer.
*   
* OUTPUTS:
*       None.
*
* RETURNS :
*       MV_OK        - Operation succeede
*       MV_BAD_PARAM - Illegal parameter in function called
*
* COMMENTS:
*
*
*******************************************************************************/
MV_STATUS coreSetTxDesc
(
    IN    MV_U8         devNum,
    IN    MV_U8         queue,
    IN    MV_U32        descNum,
    IN    MV_TX_DESC    *txDesc,
    IN    MV_U8         *bufferPtr
);

/*******************************************************************************
* dbRxDmaSetMode
*
* DESCRIPTION:
*       Enable/Disable Rx on specified TX DMA Queue
*
* INPUTS:
*        devNum    - device number
*        queue     - tx queue
*        enable    - enable/disable
*
* OUTPUTS:
*       None.
*
* RETURNS:
*       MV_OK   - on success
*       MV_FAIL - on error
*
* COMMENTS:
*
* GalTis:
*       Command - netQueueEnable
*
*******************************************************************************/
MV_STATUS dbRxDmaSetMode
(
    IN  MV_U8    devNum,
    IN  MV_U8    rxQueue,
    IN  MV_BOOL  enable
);

/*******************************************************************************
* dbTxDmaSetMode
*
* DESCRIPTION:
*       Enable/Disable Tx on specified TX DMA Queue
*
* INPUTS:
*        devNum    - device number
*        queue     - tx queue
*        enable    - enable/disable
*
* OUTPUTS:
*       None.
*
* RETURNS:
*       MV_OK   - on success
*       MV_FAIL - on error
*
* COMMENTS:
*
* GalTis:
*       Command - netQueueEnable
*
*******************************************************************************/
MV_STATUS dbTxDmaSetMode
(
    IN  MV_U8    devNum,
    IN  MV_U8    txQueue,
    IN  MV_BOOL  enable
);

/*******************************************************************************
* coreTxSetAm
*
* DESCRIPTION: 
*       Set all the descriptors for a specific TC to AM
*
* INPUTS:
*       devNum - Device number.
*       queue  - Queue number.
*       am     - MV_TRUE - Auto mode
*
* OUTPUTS:
*       None.
*
* RETURNS :
*       MV_OK        - Operation succeede
*       MV_BAD_PARAM - Illegal parameter in function called
*
* COMMENTS:
*
*
*******************************************************************************/
MV_STATUS coreTxSetAm
(
    IN  MV_U8    devNum,
    IN  MV_U8    queue,
    IN  MV_BOOL  am
);

#endif /* __tapiNetTypesh */                         
                                                     
                                                     
