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

#include "mvOsPrestera.h"
#include "mvUtils.h"
#include "mvPresteraRegs.h"
#include "hwIf/mvHwIf.h"

#define toupper(c)	c

static const char hexcode[] = "0123456789ABCDEF";

/*******************************************************************************
* createMacAddr
*
* DESCRIPTION:
*       Create Ethernet MAC Address from hexadecimal coded string
*       6 elements, string size = 17 bytes
*       MAC address is on te format of  XX:XX:XX:XX:XX:XX
*
* INPUTS:
*       source - hexadecimal coded string reference
*
* OUTPUTS:
*       dest   - pointer to MV_ETHERADDR structure
*
* RETURNS:
*       none
*
* COMMENTS:
*       no assertion is performed on validity of coded string
*
*******************************************************************************/
MV_VOID createMacAddr
(
    OUT MV_ETHERADDR *dest,
    IN  MV_U8        *source
)
{
	int i,j;

	/* Remove ':' from MAC affress */
	MV_8 enet_addr[6*2+1];

	for( i = 0, j = 0;i < 6*2+1 ; j++ ){ 
		if( source[j] != ':' ) {
			enet_addr[i] = source[j];
			i++;
		}
	}

	mvStrToMac(enet_addr, (MV_8*)dest);
}



/*********************************************************** 
 * string helpers for mac address setting                  *
 ***********************************************************/
MV_VOID mvStrToMac( MV_8 *source , MV_8 *dest ) 
{
	dest[0] = (mvStrToHex( source[0] ) << 4) + mvStrToHex( source[1] );
	dest[1] = (mvStrToHex( source[2] ) << 4) + mvStrToHex( source[3] );
	dest[2] = (mvStrToHex( source[4] ) << 4) + mvStrToHex( source[5] );
	dest[3] = (mvStrToHex( source[6] ) << 4) + mvStrToHex( source[7] );
	dest[4] = (mvStrToHex( source[8] ) << 4) + mvStrToHex( source[9] );
	dest[5] = (mvStrToHex( source[10] ) << 4) + mvStrToHex( source[11] );
}

MV_U32 mvStrToHex( MV_8 ch ) 
{
	if( (ch >= '0') && (ch <= '9') ) return( ch - '0' );
	if( (ch >= 'a') && (ch <= 'f') ) return( ch - 'a' + 10 );
	if( (ch >= 'A') && (ch <= 'F') ) return( ch - 'A' + 10 );

	return 0;
}

/*******************************************************************************
* mvPresteraMibCounterRead - Read a MIB counter
*
* DESCRIPTION:
*       This function reads a MIB counter of a specific ethernet port.
*       NOTE - Read from PRESTERA_MIB_GOOD_OCTETS_RECEIVED_LOW or 
*              PRESTERA_MIB_GOOD_OCTETS_SENT_LOW counters will return 64 bits value,
*              so pHigh32 pointer should not be NULL in this case.
*
* INPUT:
*       int           port  	  - Ethernet Port number.
*       unsigned int  mibOffset   - MIB counter offset.
*
* OUTPUT:
*       MV_U32*       pHigh32 - pointer to place where 32 most significant bits
*                             of the counter will be stored.
*
* RETURN:
*       32 low sgnificant bits of MIB counter value.
*
*******************************************************************************/
MV_U32  mvPresteraMibCounterRead(int devNum, int portNum, unsigned int mibOffset, 
                            MV_U32* pHigh32)
{
    MV_U32          valLow32, valHigh32;

    if( mvSwitchReadReg(devNum, PRESTERA_MIB_REG_BASE(portNum) + mibOffset, 
			&valLow32) != MV_OK)
   {
		return MV_FALSE;
   }
    
    /* Implement FEr ETH. Erroneous Value when Reading the Upper 32-bits    */
    /* of a 64-bit MIB Counter.                                             */
    if( (mibOffset == PRESTERA_MIB_GOOD_OCTETS_RECEIVED_LOW) || 
        (mibOffset == PRESTERA_MIB_GOOD_OCTETS_SENT_LOW) )
    {
	if( mvSwitchReadReg(devNum, PRESTERA_MIB_REG_BASE(portNum) + mibOffset, 
			&valHigh32) != MV_OK)
   	{
		return MV_FALSE;
   	}

        if(pHigh32 != NULL)
            *pHigh32 = valHigh32;
    }
    return valLow32;
}


/* Print counters of the Ethernet port */
void    mvPresteraReadPortMibCounters(int port)
{
    MV_U32  regValue, regValHigh;
    MV_U8   devNum = PORT_TO_DEV(port);
    MV_U32  portNum = PORT_TO_DEV_PORT(port);

    printf("\n\t Port #%d MIB Counters (Port %d Device %d)\n\n",port, portNum, devNum);
    printf("Port MIB base address: 0x%08x\n",PRESTERA_MIB_REG_BASE(portNum));

    printf("GoodFramesReceived          = %u\n", 
              mvPresteraMibCounterRead(devNum, portNum, PRESTERA_MIB_GOOD_FRAMES_RECEIVED, NULL));
    printf("BroadcastFramesReceived     = %u\n", 
              mvPresteraMibCounterRead(devNum, portNum, PRESTERA_MIB_BROADCAST_FRAMES_RECEIVED, NULL));
    printf("MulticastFramesReceived     = %u\n", 
              mvPresteraMibCounterRead(devNum, portNum, PRESTERA_MIB_MULTICAST_FRAMES_RECEIVED, NULL));

    regValue = mvPresteraMibCounterRead(devNum, portNum, PRESTERA_MIB_GOOD_OCTETS_RECEIVED_LOW, 
                                 &regValHigh);
    printf("GoodOctetsReceived          = 0x%08x%08x\n", 
               regValHigh, regValue);

    printf("\n");
    printf("GoodFramesSent              = %u\n", 
              mvPresteraMibCounterRead(devNum, portNum, PRESTERA_MIB_GOOD_FRAMES_SENT, NULL));
    printf("BroadcastFramesSent         = %u\n", 
              mvPresteraMibCounterRead(devNum, portNum, PRESTERA_MIB_BROADCAST_FRAMES_SENT, NULL));
    printf("MulticastFramesSent         = %u\n", 
              mvPresteraMibCounterRead(devNum, portNum, PRESTERA_MIB_MULTICAST_FRAMES_SENT, NULL));

    regValue = mvPresteraMibCounterRead(devNum, portNum, PRESTERA_MIB_GOOD_OCTETS_SENT_LOW, 
                                 &regValHigh);
    printf("GoodOctetsSent              = 0x%08x%08x\n", regValHigh, regValue);

    regValue = mvPresteraMibCounterRead(devNum, portNum, PRESTERA_MIB_SENT_MULTIPLE, NULL);
    printf("SentMultiple                = %u\n", regValue);

    printf("SentDeferred                = %u\n", 
              mvPresteraMibCounterRead(devNum, portNum, PRESTERA_MIB_SENT_DEFERRED, NULL));
    
    printf("\n\t FC Control Counters\n");

    regValue = mvPresteraMibCounterRead(devNum, portNum, PRESTERA_MIB_GOOD_FC_RECEIVED, NULL);
    printf("GoodFCFramesReceived        = %u\n", regValue);

    regValue = mvPresteraMibCounterRead(devNum, portNum, PRESTERA_MIB_RECEIVED_FIFO_OVERRUN, NULL);
    printf("ReceivedFifoOverrun         = %u\n", regValue);

    regValue = mvPresteraMibCounterRead(devNum, portNum, PRESTERA_MIB_FC_SENT, NULL);
    printf("FCFramesSent                = %u\n", regValue);


    printf("\n\t RX Errors\n");

    regValue = mvPresteraMibCounterRead(devNum, portNum, PRESTERA_MIB_BAD_OCTETS_RECEIVED, NULL);
    printf("BadOctetsReceived           = %u\n", regValue);

    regValue = mvPresteraMibCounterRead(devNum, portNum, PRESTERA_MIB_UNDERSIZE_RECEIVED, NULL);
    printf("UndersizeFramesReceived     = %u\n", regValue);

    regValue = mvPresteraMibCounterRead(devNum, portNum, PRESTERA_MIB_FRAGMENTS_RECEIVED, NULL);
    printf("FragmentsReceived           = %u\n", regValue);

    regValue = mvPresteraMibCounterRead(devNum, portNum, PRESTERA_MIB_OVERSIZE_RECEIVED, NULL);
    printf("OversizeFramesReceived      = %u\n", regValue);
    
    regValue = mvPresteraMibCounterRead(devNum, portNum, PRESTERA_MIB_JABBER_RECEIVED, NULL);
    printf("JabbersReceived             = %u\n", regValue);

    regValue = mvPresteraMibCounterRead(devNum, portNum, PRESTERA_MIB_RX_ERROR_FRAME_RECEIVED, NULL);
    printf("RxErrorFrameReceived        = %u\n", regValue);

    /*regValue = mvPresteraMibCounterRead(devNum, portNum, PRESTERA_MIB_BAD_CRC_EVENT, NULL);
    printf("BadCrcReceived              = %u\n", regValue);*/

    printf("\n\t TX Errors\n");

    regValue = mvPresteraMibCounterRead(devNum, portNum, PRESTERA_MIB_TX_FIFO_UNDERRUN_AND_CRC, NULL);
    printf("TxFifoUnderrunAndCRC        = %u\n", regValue);

    regValue = mvPresteraMibCounterRead(devNum, portNum, PRESTERA_MIB_EXCESSIVE_COLLISION, NULL);
    printf("TxExcessiveCollisions       = %u\n", regValue);

    /*regValue = mvPresteraMibCounterRead(devNum, portNum, PRESTERA_MIB_COLLISION, NULL);
    printf("TxCollisions                = %u\n", regValue);

    regValue = mvPresteraMibCounterRead(devNum, portNum, PRESTERA_MIB_LATE_COLLISION, NULL);
    printf("TxLateCollisions            = %u\n", regValue);*/


    printf("\n");
    /*regValue = MV_REG_READ( PRESTERA_RX_DISCARD_PKTS_CNTR_REG(port));
    printf("Rx Discard packets counter    = %u\n", regValue);

    regValue = MV_REG_READ(PRESTERA_RX_OVERRUN_PKTS_CNTR_REG(port));
    printf("Rx Overrun packets counter  = %u\n", regValue);*/
}






