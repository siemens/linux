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

#ifndef __gtNetIf_h__
#define __gtNetIf_h__

#include <common/os/gtOs.h>

#define NUM_OF_RX_QUEUES    (8)
#define NUM_OF_TX_QUEUES    (8)

/********* Typedefs ***********************************************************/

/*
 * Typedef: struct STRUCT_TX_DESC
 *
 * Description: Includes the PP Tx descriptor fields, to be used for handling
 *              packet transmits.
 *
 * Fields:
 *      word1           - First word of the Tx Descriptor.
 *      word2           - Second word of the Tx Descriptor.
 *      buffPointer     - The physical data-buffer address.
 *      virtBuffPointer - The virtual data-buffer address.
 *      nextDescPointer - The physical address of the next Tx descriptor.
 *
 */
typedef struct _txDesc
{
    volatile MV_U32         word1;
    volatile MV_U32         word2;

    volatile MV_U32         buffPointer;
    volatile MV_U32         nextDescPointer;
}STRUCT_TX_DESC;



/*
 * typedef: struct STRUCT_SW_TC_DESC
 *
 * Description: Sw management Tx descirptor.
 *
 * Fields:
 *      txDesc          - Points to the Tx descriptor representing this Sw Tx
 *                        desc.
 *      swNextDesc      - A pointer to the next descriptor in the linked-list.
 *      userData        - A data to be stored in gtBuf on packet transmit
 *                        request, and returned to user on txEnd.
 *
 */
typedef struct _swTxDesc
{
    STRUCT_TX_DESC      *txDesc;
    MV_U8               txBufferAligment;
    struct _swTxDesc    *swNextDesc;
    volatile MV_U32     virtBuffPointer; /* Haim */
    MV_PTR              userData;
}STRUCT_SW_TX_DESC;



/*
 * Typedef: struct TX_DESC_LIST
 *
 * Description: The control block of a single Tx descriptors list.
 *
 * Fields:
 *
 *      next2Free       - Points to the descriptor from which the next
 *                        transmitted buffer should be freed, When receiving
 *                        a TxEnd interrupt.
 *      next2Feed       - Points to the descriptor to which the next transmitted
 *                        packet should be attached.
 *                        (This actually points to the first desc. of the packet
 *                        in case of a multi desc. packet).
 *      freeDescNum     - Number of free descriptors in list.
 *      txListSem       - Semaphore for mutual exclusion on the access to the Tx
 *                        descriptors list.
 *      freeCoreBuff    - Whether to free the tx core buffer,  this is true
 *                        whenever a packet is trasfered to the core with the
 *                        bufCopy on (parameter of coreGetTxPacket()).
 */
typedef struct
{
    STRUCT_SW_TX_DESC   *next2Free;
    STRUCT_SW_TX_DESC   *next2Feed;

    MV_U32              freeDescNum;
    MV_SEM              txListSem;

    MV_BOOL             freeCoreBuff;
    MV_U32              sizeOfList;
}TX_DESC_LIST;


/*
 * typedef: struct NET_CACHE_DMA_BLOCKS_INFO
 *
 * Description:
 *
 * Fields:
 *    devNum          - The device number to assign to this PP.
 *    txDescBlock     - Pointer to a block of host memory to be used
 *                       for allocating Tx packet descriptor structures.
 *    txDescBlockSize - The raw size in bytes of txDescBlock memory.
 *    rxDescBlock     - Pointer to a block memory to be used for
 *                       allocating Rx description structures.
 *    rxDescBlockSize - The raw size in byte of rxDescBlock.
 *    rxBufInfo       - Rx buffers allocation information.
 *    auDescBlock     - The block of memory used for the Address Update Queue.
 *                       The packet processor writes AU messages to this queue.
 *    auDescBlockSize - Size of auDescBlock (in Bytes).
 *
 *
 */
typedef struct
{
    MV_U8   devNum;
    MV_U32  *txDescBlock;
    MV_U32  *txDescBlockPhy; /*Haim*/
    MV_U32  txDescBlockSize;
    MV_U32  txDescMemHandle; /*Haim*/

    MV_U32  *rxDescBlock;
    MV_U32  *rxDescBlockPhy; /*Haim*/
    MV_U32  rxDescBlockSize;
    MV_U32  rxDescMemHandle; /*Haim*/
    MV_U32  *rxBufBlock;
    MV_U32  *rxBufBlockPhy; /*Haim*/
    MV_U32  rxBufBlockSize;
    MV_U32  rxBufMemHandle; /*Haim*/

    MV_U8   *auDescBlock;
    MV_U8   *auDescBlockSecond;
    MV_U32  *auDescBlockPhy; /*Haim*/
    MV_U32  *auDescBlockSecondPhy; /*Haim*/
    MV_U32  auDescBlockSize;
    MV_U32  auDescMemHandle; /*Haim*/
    MV_U32  auDescMemSecondHandle; /*Haim*/

    MV_U8   *fuDescBlock;
    MV_U8   *fuDescBlockSecond;
    MV_U32  *fuDescBlockPhy; /*Haim*/
    MV_U32  *fuDescBlockSecondPhy; /*Haim*/
    MV_U32  fuDescBlockSize;
    MV_U32  fuDescMemHandle; /*Haim*/
    MV_U32  fuDescMemSecondHandle; /*Haim*/

}NET_CACHE_DMA_BLOCKS_INFO;

/*
 * Typedef: struct STRUCT_RX_DESC
 *
 * Description: Includes the PP Rx descriptor fields, to be used for handling
 *              recieved packets.
 *
 * Fields:
 *      word1           - First word of the Rx Descriptor.
 *      word2           - Second word of the Rx Descriptor.
 *      buffPointer     - The physical data-buffer address.
 *      nextDescPointer - The physical address of the next Rx descriptor.
 *
 */
typedef struct _rxDesc
{
    volatile MV_U32         word1;
    volatile MV_U32         word2;

    volatile MV_U32         buffPointer;
    volatile MV_U32         nextDescPointer;

}STRUCT_RX_DESC;



/*
 * typedef: struct STRUCT_SW_TC_DESC
 *
 * Description: Sw management Tx descirptor.
 *
 * Fields:
 *      rxDesc          - Points to the Rx descriptor representing this Sw Rx
 *                        desc.
 *      swNextDesc      - A pointer to the next descriptor in the linked-list.
 *      buffVirtPointer - The virtual data-buffer address
 *      shadowRxDesc    - A shadow struct to hold the real descriptor data
 *                        after byte swapping to save non-cachable memory
 *                        access.
 *
 */
typedef struct _swRxDesc
{
    STRUCT_RX_DESC      *rxDesc;
    struct _swRxDesc    *swNextDesc;
    
	volatile MV_U32     buffVirtPointer; /* Haim */


    STRUCT_RX_DESC      shadowRxDesc;
}STRUCT_SW_RX_DESC;


/*
 * Typedef: struct RX_DESC_LIST
 *
 * Description: The control block of a single Rx descriptors list.
 *
 * Fields:
 *
 *      next2Return     - Points to the descriptor to which the next returned
 *                        buffer should be attached.
 *      next2Receive    - Points to the descriptor from which the next packet
 *                        will be fetched when an Rx interrupt is received.
 *                        (This actually points to the first desc. of the packet
 *                        in case of a multi desc. packet).
 *      freeDescNum     - Number of free descriptors in list.
 *      rxListSem       - Semaphore for mutual exclusion on the access to the rx
 *                        descriptors list.
 *      headerOffset    - Number of reserved bytes before each buffer, to be
 *                        kept for application and internal use.
 *      forbidQEn       - When set to MV_TRUE enabling the Rx SDMA on buffer
 *                        release is forbidden.
 *
 */
typedef struct
{
    STRUCT_SW_RX_DESC   *next2Return;
    STRUCT_SW_RX_DESC   *next2Receive;

    MV_U32              freeDescNum;
    MV_SEM              rxListSem;
    MV_BOOL             forbidQEn;
}RX_DESC_LIST;

/*
 * typedef: struct AU_DESC_CTRL
 *
 * Description: Address Update descriptors block control struct.
 *
 * Fields:
 *      blockAddr   - Address of the descs. block (Virtual Address)
 *      blockSize   - Size (in descs.) of the descs. block.
 *      currDescIdx - Index of the next desc. to handle.
 *      auCtrlSem   - Semaphore for mutual exclusion.
 *
 */
typedef struct
{
    MV_U32  blockAddr;
    MV_U32  blockAddrSecond;
    MV_U32  blockSize;
    MV_U32  currDescIdx;
    MV_SEM  auCtrlSem;
}AU_DESC_CTRL;

/************* AUQ structures ********************************************/

/*
 * Typedef: struct AU_MESSAGE
 *
 * Description: Includes fields definitions of the Address Update messages sent
 *              to the CPU.
 *
 * Fields:
 */
typedef struct
{
    MV_U32  word0;
    MV_U32  word1;
    MV_U32  word2;
    MV_U32  word3;

}STRUCT_AU_DESC;

/*
 * Typedef: struct SDMA_INTERRUPT_CTRL
 *
 * Description: Includes all needed definitions for interrupts handling in core
 *              level. (Rx, Tx, Address Updates,...).
 *
 * Fields:
 *      rxDescList  - A list of Rx_Descriptor_List control structs, one for each
 *                    Rx queue.
 *      txDescList  - A list of Tx_Descriptor_List control structs, one for each
 *                    Tx queue.
 *      auDescCtrl  - Control block of the AU desc.
 *
 */
typedef struct
{
    RX_DESC_LIST    rxDescList[NUM_OF_RX_QUEUES];
    TX_DESC_LIST    txDescList[NUM_OF_TX_QUEUES];
    AU_DESC_CTRL    auDescCtrl;
    AU_DESC_CTRL    fuDescCtrl;

}SDMA_INTERRUPT_CTRL;

/*
 * typedef: enum MV_DESC_OWNERSHIP
 *
 * Description:
 *      Descriptor ownership values
 *
 * Fields:
 *      MV_OWNERSHIP_CPU - CPU ownership
 *      MV_OWNERSHIP_DMA - DMA ownership
 *
 * Comment:
 */
typedef enum
{
    MV_OWNERSHIP_CPU = 0,
    MV_OWNERSHIP_DMA = 1

}MV_DESC_OWNERSHIP;

/*
 * typedef: struct MV_RX_DESC
 *
 * Description:
 *      Contains the RX descriptor parameters
 *
 * Fields:
 *      os              - ownership bit
 *      busError        - bus error occured.
 *      enableInterrupt - enable interrupt
 *      resourceError   - Resource error
 *      first           - First buffer of the packet.
 *      last            - Last buffer of the packet.
 *      srcDev          - The device the packet was recieved on.
 *      srcPort         - The port the packet was recieved on.
 *      cpuCode         - CPU code.
 *      validCrc        - Valid or invalid CRC.
 *      byteCount       - Total byts count of the packet.
 *      bufferSize      - Current buffer size.
 *
 * Comment:
 */
typedef struct
{
    MV_DESC_OWNERSHIP os;
    MV_BOOL           busError;
    MV_BOOL           enableInterrupt;
    MV_BOOL           resourceError;
    MV_BOOL           first;
    MV_BOOL           last;
    MV_BOOL           validCrc;
    MV_U32            byteCount;
    MV_U32            bufferSize;

}MV_RX_DESC;


/*
 * typedef: struct MV_RX_DESC_DATA
 *
 * Description:
 *      Contains the over all data in the RX descriptor
 *
 * Fields:
 *      rxDesc       - RX descriptor
 *      intDev       - The device the interrupt was received on.
 *
 * Comment:
 */
typedef struct
{
    MV_RX_DESC     rxDesc;
    MV_U8          intDev;
    MV_U32         word1;
    MV_U32         word2;

}MV_RX_DESC_DATA;


/*
 * typedef: struct MV_TX_DESC
 *
 * Description:
 *      Contains the TX descriptor parameters
 *
 * Fields:
 *      os               - ownership bit
 *      autoMode         - Auto mode.
 *      enableInterrupt  - enable interrupt
 *      first            - First buffer of the packet.
 *      last             - Last buffer of the packet.
 *      recalcCrc        - Recalculate crc.
 *      byteCount        - Current buffer size.
 * Comment:
 */
typedef struct
{
    MV_DESC_OWNERSHIP os;
    MV_BOOL           autoMode;
    MV_BOOL           enableInterrupt;
    MV_BOOL           first;
    MV_BOOL           last;
    MV_BOOL           recalcCrc;
    MV_U32            byteCount;

}MV_TX_DESC;

/*
 * typedef: enum MV_TX_COOKIE_TYPE
 *
 * Description:
 *      Type of cookie placed in the descriptor
 *
 * Fields:
 *      COOKIE_MV_BUFF            - The cookie contains a pointer to the gtBuf.
 *      COOKIE_MV_BUFF_RX_2_TX    - The cookie contains a pointer to the
 *                                  gtBuf and the packet was sent because of
 *                                  rx2Tx mode.
 *      COOKIE_NO_MV_BUFF         - The packet was sent directly from the core.
 *                                  This means there is no gtBuf pointer.
 *      COOKIE_NO_MV_BUFF_RX_2_TX - The packet was sent directly from the core.
 *                                  this means there is no gtBuff.the packet was
 *                                  sent because of rx2Tx mode
 * Comment:
 */
typedef enum
{
    COOKIE_MV_BUFF             = 0,
    COOKIE_MV_BUFF_RX_2_TX     = 1,
    COOKIE_NO_MV_BUFF          = 2,
    COOKIE_NO_MV_BUFF_RX_2_TX  = 3

}MV_TX_COOKIE_TYPE;


/*
 * typedef: struct MV_TX_COOKIE
 *
 * Description:
 *
 * Fields:
 *      cookieType - cookieType
 *      gtBuf      - Pointer to the gtBuf. This parameter is valid only in
 *                   packet types COOKIE_MV_BUFF and COOKIE_MV_BUFF_RX_2_TX.
 *      tc         - The Rx queue in which the packet was received befor the.
 *                   rx 2 tx.
 *      port       - The port in which the packet was received befor the rx 2
 *                   tx.
 *      bufOwDevNum - device number of buffer owner (for Rx 2 Tx support on
 *                    Mini SV platform)
 *
 * Comment:
 */
typedef struct
{
    MV_TX_COOKIE_TYPE cookieType;
    MV_U8             **gtBuf;
    MV_U32            numOfBuf;
    MV_U8             tc;
    MV_U8             bufOwDevNum;

}MV_TX_COOKIE;

/*
 * typedef: struct MV_NET_RX_2_TX_PARAMS
 *
 * Description:
 *      Rx To TX parameters.
 *
 * Fields:
 *      valid    - Entry valid
 *      start    - MV_TRUE - enable RX to TX.
 *      outPort  - Out port.
 *      outTc    - Out queue.
 *      packetsCnt  - Number of received packets.
 *
 * Comment:
 */
typedef struct
{
    MV_BOOL  valid;
    MV_BOOL  start;
    MV_U8    outPort;
    MV_U8    outDev;
    MV_U8    outTc;
    MV_U32   packetsCnt;

}MV_NET_RX_2_TX_PARAMS;



/*******************************************************************************
* gtFreeRxBuf
*
* DESCRIPTION:
*       Frees a list of buffers, that where previously passed to the upper layer
*       in an Rx event.
*
* INPUTS:
*       rxBuffList  - List of Rx buffers to be freed.
*       buffListLen - Length of rxBufList.
*       devNum      - The device number throw which these buffers where
*                     received.
*       rxQueue     - The Rx queue number throw which these buffers where
*                     received.
*
* OUTPUTS:
*       None.
*
* RETURNS:
*       MV_OK on success, or
*       MV_FAIL otherwise.
*
* COMMENTS:
*       None.
*
*******************************************************************************/
MV_STATUS gtFreeRxBuf
(
    IN MV_U8    *rxBuffList[],
    IN MV_U32   buffListLen,
    IN MV_U8    devNum,
    IN MV_U8    rxQueue
);


/*******************************************************************************
* gtInitSdmaNetIfDev
*
* DESCRIPTION:
*       Initialize the network interface structures, Rx descriptors & buffers
*       and Tx descriptors (For a single device).
*
* INPUTS:
*       devNum  - The device to initialize.
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
MV_STATUS gtInitSdmaNetIfDev
(
    IN  MV_U8       devNum,
    IN  MV_U32      auDescNum,
    IN  MV_U32      txDescNum,
    IN  MV_U32      rxDescNum,
    IN  MV_U32      rxBufSize
);

/*******************************************************************************
* gtNetDisableRxProcess
*
* DESCRIPTION:
*   Disables the RX process. By calling this function when the interrupt accrues
*   the CPU will change only the ownership bit to DMA.
*
* INPUTS:
*       None.
*
* OUTPUTS:
*       None.
*
* RETURNS :
*       MV_OK        - Operation succeede
*
* COMMENTS:
*
*
*******************************************************************************/
MV_STATUS gtNetDisableRxProcess
(
    MV_VOID
);

/*******************************************************************************
* gtNetEnableRxProcess
*
* DESCRIPTION:
*   Enable the RX process.
*
* INPUTS:
*       None.
*
* OUTPUTS:
*       None.
*
* RETURNS :
*       MV_OK        - Operation succeede
*
* COMMENTS:
*
*
*******************************************************************************/
MV_STATUS gtNetEnableRxProcess
(
    MV_VOID
);

/*******************************************************************************
* gtNetGetEnableRxProcess
*
* DESCRIPTION:
*   Get the status of Enable the RX process.
*
* INPUTS:
*       None.
*
* OUTPUTS:
*       MV_TRUE  - Enable RX process
*       MV_FALSE - Disable RX process
*
* RETURNS :
*       None
*
* COMMENTS:
*
*
*******************************************************************************/
MV_BOOL gtNetGetEnableRxProcess
(
    MV_VOID
);


#endif /* __gtNetIf_h__ */
