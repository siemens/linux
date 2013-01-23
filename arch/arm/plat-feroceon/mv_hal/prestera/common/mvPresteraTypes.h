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

#ifndef __mvPresteraTypesh
#define __mvPresteraTypesh

#include "common/mvGenTypes.h"


#define MARVELL_VENDOR_ID 0x11ab


typedef struct
{
    MV_U8   arIP[4];
}MV_IPADDR;

/* convert MV_IPADDR structure to word */
#define MV_IPADDR_TO_U32(ipAddr) \
       (((ipAddr).arIP[3])         |  \
        ((ipAddr).arIP[2] << 8)     |  \
        ((ipAddr).arIP[1] << 16)    |  \
        ((ipAddr).arIP[0] << 24))
        
/* convert U32 to MV_IPADDR structure */
#define MV_U32_TO_IPADDR(data, ipAddr) \
        (ipAddr).arIP[3] = (MV_U8) (((data)) & 0xFF);       \
        (ipAddr).arIP[2] = (MV_U8) (((data) >> 8) & 0xFF);  \
        (ipAddr).arIP[1] = (MV_U8) (((data) >> 16) & 0xFF); \
        (ipAddr).arIP[0] = (MV_U8) (((data) >> 24) & 0xFF)

typedef struct
{
    MV_U8       arEther[6];
}MV_ETHERADDR;

typedef struct
{
    MV_U32       lowWord;
    MV_U32       highHalfWord;

}MV_ETHERADDR_WORD;

typedef struct
{
    MV_U8 addr[16];

}MV_IPV6ADDR;

typedef struct
{
    MV_U32 word0; /* low bits 31:0 */
    MV_U32 word1;
    MV_U32 word2;
    MV_U32 word3;

}MV_IPV6ADDR_WORDS;

typedef struct 
{
    unsigned long word[3];
}MV_U96;


#define MV_HW_MAC_LOW32(macAddr)                \
           (MV_U32)((macAddr)->arEther[5] |          \
                ((macAddr)->arEther[4] << 8) |    \
                ((macAddr)->arEther[3] << 16) |   \
                ((macAddr)->arEther[2] << 24))

#define MV_HW_MAC_HIGH16(macAddr)           \
        (MV_U32)((macAddr)->arEther[1] | ((macAddr)->arEther[0] << 8))

#define MV_HW_MAC_LOW16(macAddr)            \
        (MV_U32)((macAddr)->arEther[5] | ((macAddr)->arEther[4] << 8))

#define MV_HW_MAC_HIGH32(macAddr)               \
                (MV_U32)((macAddr)->arEther[3] |          \
                ((macAddr)->arEther[2] << 8) |    \
                ((macAddr)->arEther[1] << 16) |   \
                ((macAddr)->arEther[0] << 24))

/* convert MV_ETHERADDR structure to word */
#define MV_ETHERADDR_TO_U32(arAddr, wordAddr) \
       wordAddr.lowWord = MV_HW_MAC_LOW32(arAddr); \
       wordAddr.highHalfWord = MV_HW_MAC_HIGH16(arAddr)

/* convert U32 to MV_ETHERADDR structure */
#define MV_U32_TO_ETHERADDR(arAddr, wordAddr) \
    arAddr.arEther[0] = (MV_U8) ((wordAddr).highHalfWord >> 8) & 0xFF; \
    arAddr.arEther[1] = (MV_U8) ((wordAddr.highHalfWord) & 0xFF); \
    arAddr.arEther[2] = (MV_U8) ((wordAddr.lowWord >> 24) & 0xFF); \
    arAddr.arEther[3] = (MV_U8) ((wordAddr.lowWord >> 16) & 0xFF); \
    arAddr.arEther[4] = (MV_U8) ((wordAddr.lowWord >> 8) & 0xFF); \
    arAddr.arEther[5] = (MV_U8) ((wordAddr.lowWord) & 0xFF); 
        

/*
 * typedef: enum MV_PRTCL
 *
 * Description: Enumeration of protocols codes.
 *
 * Enumerations:
 *    IPV4       - IPv4.
 *    IPV6       - IPv6.
 *    MPLS       - MPLS.
 *    USER_DEF_0 - User Defined Protocol 0.
 *    USER_DEF_1 - User Defined Protocol 1.
 *    L2CE_ETHER - An Ethernet Layer-2 Circuit Emulation (An Ethernet CRC must
 *                 be generated).
 *    L2CE_OTHER - Non Ethernet Layer-2 Circuit Emulation.
 */
typedef enum
{
    IPV4 = 0,
    IPV6,
    MPLS,
    USER_DEF_0,
    USER_DEF_1,
    L2CE_ETHER = 6,
    L2CE_OTHER
}MV_PRTCL;

/*
 * Typedef enum SYS_CONF_HW_MNG_PRTCL
 *
 * Description:
 *      define the connection type used to manage the HW
 *
 */
typedef enum
{
    SYS_NONE_PRTCL,
    SYS_TWSI_PRTCL,
    SYS_SMI_PRTCL,
    SYS_PCI_PRTCL

}HW_MNG_PRTCL;



/*
 * typedef: MV_LPORT.
 *
 * Description: represents either a Port or Trunk.
 *
 *
 * comments:
 *    A logical port index has two possible representations:
 *         If a trunk group,
 *            Bit  31 is set
 *            Bits 15:0 contain the trunk group
 *         If a single port,
 *            Bit  31 is clear (not trunk)
 *            Bits 7:0 contain the local port index on the device
 *            Bits 15:8 contain the device ID
 *            Bits 23:16 contain the unit number
 *
 *   Logical Port value based on a Port
 *   3 3 2 2 2 2 2 2 2 2 2 2 1 1 1 1 1 1 1 1 1 1
 *   1 0 9 8 7 6 5 4 3 2 1 0 9 8 7 6 5 4 3 2 1 0 9 8 7 6 5 4 3 2 1 0
 *  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *  |0|  reserved   |     unit      |  device ID    |    port       |
 *  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *
 *  Logical Port value based on a Trunk Group
 *   3 3 2 2 2 2 2 2 2 2 2 2 1 1 1 1 1 1 1 1 1 1
 *   1 0 9 8 7 6 5 4 3 2 1 0 9 8 7 6 5 4 3 2 1 0 9 8 7 6 5 4 3 2 1 0
 *  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *  |1|        reserved             |           Trunk ID            |
 *  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *
 *   The following macros should be used to extract specific info
 *   from a Logical Port index
 */
typedef MV_U32 MV_LPORT;

#define MV_LPORT_2_PORT(lport)      (MV_U8)((lport) & 0xff)
#define MV_LPORT_2_DEV(lport)       (MV_U8)(((lport) >> 8) & 0xff)
#define MV_LPORT_2_UNIT(lport)      (MV_U8)(((lport) >> 16) & 0xff)
#define MV_LPORT_IS_TRUNK(lport)    (MV_BOOL)((((lport) >> 31) & 0x1)==1? \
                                    MV_TRUE:MV_FALSE)
#define MV_LPORT_2_TRUNK_ID(lport)  (MV_U16)((lport) & 0xffff)

#define MV_PORT_DEV_2_LPORT(dev, port, unit)    \
        ((((unit) & 0xff) << 16) | (((dev) & 0xff) << 8) | ((port) & 0xff))
#define MV_TRUNK_2_LPORT(trunk)         (0x80000000 | (trunk))

/* CPU PORT NUMBER Definition */
#define CPU_PORT_NUM    63
#define MV_CPU_LPORT(dev, unit)    \
                ((((unit) & 0xff) << 16) | (((dev) & 0xff) << 8) | \
                (CPU_PORT_NUM & 0xff))

/* NULL Port */
#define MV_NULL_PORT_NUM   61

/* this macro should be used for regular Port validation */
/* CPU port does not allowed                             */
#define MV_PORT_NOT_EXIST(lPort)( (MV_LPORT_IS_TRUNK(lPort)) || \
                                (tapiPpDevs[MV_LPORT_2_DEV(lPort)] == NULL)||\
                                (MV_LPORT_2_PORT(lPort) >=                   \
                                tapiPpDevs[MV_LPORT_2_DEV(lPort)]->numOfPorts))

/* this macro should be used for regular Port validation */
/* CPU port does not allowed                             */
#define MV_PORT_EXIST(lPort) (!( (MV_LPORT_IS_TRUNK(lPort)) || \
                                (tapiPpDevs[MV_LPORT_2_DEV(lPort)] == NULL)||\
                                (MV_LPORT_2_PORT(lPort) >=                   \
                                tapiPpDevs[MV_LPORT_2_DEV(lPort)]->numOfPorts)))


/* this macro should be used for physical regular Port validation */
/* CPU port does not allowed                             */
#define MV_PHYSICAL_PORT_NOT_EXIST(lPort)( (MV_LPORT_IS_TRUNK(lPort)) || \
                                (tapiPpDevs[MV_LPORT_2_DEV(lPort)] == NULL)||\
                                (MV_LPORT_2_PORT(lPort) >=                   \
                       tapiPpDevs[MV_LPORT_2_DEV(lPort)]->numOfPhysicalPorts))

/* this macro should be used for physical regular Port validation */
/* CPU port does not allowed                             */
#define MV_PHYSICAL_PORT_EXIST(lPort) (!( (MV_LPORT_IS_TRUNK(lPort)) || \
                                (tapiPpDevs[MV_LPORT_2_DEV(lPort)] == NULL)||\
                                (MV_LPORT_2_PORT(lPort) >=                   \
                                tapiPpDevs[MV_LPORT_2_DEV(lPort)]->numOfPhysicalPorts)))


/* this macro should be used for CPU Port validation */
#define MV_CPU_PORT_NOT_EXIST(lPort)( (MV_LPORT_IS_TRUNK(lPort)) || \
                                (tapiPpDevs[MV_LPORT_2_DEV(lPort)] == NULL)||\
                                (MV_LPORT_2_PORT(lPort) != CPU_PORT_NUM))



/*
 * Typedef: enumeration MV_DEVICE
 *
 * Description: Defines the different device type that may exist in system.
 *
 * Fields:
 *  Tango group:
 *      MV_98MX620  -   10 * 1G
 *      MV_98MX630  -   1 * 10G
 *      MV_98EX120  -   10 * 1G
 *      MV_98EX130  -   1 * 10G
 *
 *  Twist group:
 *      MV_98MX610B     - 48 + 4 DSSMII full feature
 *      MV_98MX620B     - 10 * 1G full feature
 *      MV_98MX610BS    - 48 + 4 SSMII full feature
 *      MV_98EX110BS    - 48 + 4 SSMII no external rams
 *      MV_98EX111BS    - 48 + 4 SSMII no ns, ws
 *      MV_98EX112BS    - 48 + 4 SSMII no ws, no df
 *      MV_98EX110B     - 48 + 4 DSSMII no external rams
 *      MV_98EX111B     - 48 + 4 DSSMII no ns, ws
 *      MV_98EX112B     - 48 + 4 DSSMII no ws, no df
 *      MV_98EX120B     - 12G no external rams
 *      MV_98EX120B_    - 12G no external rams (TC enabled)
 *      MV_98EX121B     - 12G no ns, no ws
 *      MV_98EX122B     - 12G no ws, no df
 *      MV_98EX128B     - 12G  with extrenal memories
 *      MV_98EX129B     - 12G  with ws, and fd
 *
 *  Twist D group:
 *      MV_98EX110D  - 48+4 SSMII
 *      MV_98EX115D  - 48+4 SSMII with external narrow sram
 *      MV_98EX110DS - 48+4 DSSMII
 *      MV_98EX115DS - 48+4 DSSMII with external narrow sram
 *      MV_98EX120D  - 12G
 *      MV_98EX125D  - 12G with external narrow sram
 *      MV_98EX130D  - 1 * 10G (XG)
 *      MV_98EX135D  - 1 * 10G (XG) with external narrow sram
 *
 *  Samba group:
 *    
 *      MV_98MX625A  - 12G MX full feature
 *      MV_98MX625AB - 12G MX full feature - 0x197 - hardware device ID.
 *      MV_98MX615A  - 48+4 MX device with all external interfcaes - DSSMII
 *      MV_98MX615AS - 48+4 MX device with all external interfcaes - SSMII
 *      MV_98MX635A  - 1XGE MX 
 *
 *  Tiger group:
 *
 *      MV_98EX116 - 48F+4G Tiger
 *      MV_98EX106 - 24F+4G Tiger
 *      MV_98EX108 - 48F+4G Tiger
 *      MV_98EX126 - 12G Tiger
 *      MV_98EX126_VB - 12G Tiger value blade configuration
 *      MV_98EX126_VB_PHY - 12G Tiger value blade configuration with physical
 *                          connection to to external boards.       
 *      MV_98EX136 - 1XG Tiger
 *      MV_98EX136_VB - 1XG Tiger value blade configuration
 *      MV_98EX136_VB_PHY - 1XG Tiger value blade configuration with physical
 *                          connection to to external boards.       
 */
typedef enum
{
    MV_98MX620A = 0x000011AB,
    MV_98MX630A = 0x001011AB,
    MV_98EX120A = 0x000B11AB,
    MV_98EX130A = 0x001B11AB,

    MV_98MX610B  = 0x007011AB,
    MV_98MX620B  = 0x004011AB,
    MV_98MX610BS = 0x006011AB,
    MV_98EX110BS = 0x006711AB,
    MV_98EX111BS = 0x006311AB,
    MV_98EX112BS = 0x006511AB,
    MV_98EX110B  = 0x007711AB,
    MV_98EX111B  = 0x007311AB,
    MV_98EX112B  = 0x007511AB,
    MV_98EX120B  = 0x005711AB,
    MV_98EX120B_ = 0x005611AB,
    MV_98EX121B  = 0x005311AB,
    MV_98EX122B  = 0x005511AB,
    MV_98EX128B  = 0x005011AB,
    MV_98EX129B  = 0x005211AB,

    MV_98EX110D  = 0x00E711AB,
    MV_98EX115D  = 0x00E111AB,
    MV_98EX110DS = 0x00F711AB,
    MV_98EX115DS = 0x00F111AB,
    MV_98EX120D  = 0x00D711AB,
    MV_98EX125D  = 0x00D111AB,
    MV_98EX130D  = 0x01D711AB,
    MV_98EX135D  = 0x01D111AB,

    MV_98MX625A  = 0x018011AB,
    MV_98MX625AB = 0x019711AB,
    MV_98MX615A  = 0x01B011AB,
    MV_98MX615AS = 0x01A011AB,
    MV_98MX635A  = 0x01D011AB,

    MV_98EX116   = 0x012011AB, /* 0x012x11AB */
    MV_98EX106   = 0x012A11AB,
    MV_98EX108   = 0x012B11AB,
    MV_98EX126   = 0x011011AB, /* 0x011x11AB */
    MV_98EX126_VB = 0x111011AB, 
    MV_98EX126_VB_PHY = 0x111111AB,
    MV_98EX136   = 0x015011AB,  /* 0x015x11AB */
    MV_98EX136_VB = 0x115011AB, 
    MV_98EX136_VB_PHY = 0x115111AB

}MV_DEVICE;

/*
 * Typedef: enumeration MV_FA_XBAR_DEVICE
 *
 * Description: Defines the different fabric adaptor and crossbar device types
 *
 * Fields:
 *      FX900A  - Mesh adaptor with 4 hyper Glinks and crossbar  -
 *                  4 x 4 serdes lanes.
 *      FX902A  - Stacking adaptor with 3 hyper Glinks and crossbar  -
 *                  2 x 4 serdes lanes + 1 x 6 serdes lanes
 *      FX910A  - Fabric adaptor with universal configuration -
 *                  2 x 6 serdes lanes, 3 x 6 serdes lanes,
 *                  2 x 4 serdes lanes + 1 x 6 serdes lanes,
 *                  4 x 4 serdes lanes or 3 x 6 serdes lanes.
 *      FX9010A - four port crossbar.
 *      FX9110  - Nine port crossbar.
 *      FX9210  - twelve port crossbar.
 *
 *      FX915   - Leopard
 */
typedef enum
{
    MV_98FX900A  = 0,
    MV_98FX902A ,
    MV_98FX910A ,
    MV_98FX9010A,
    MV_98FX9110,
    MV_98FX9210,
    MV_98FX915
}MV_FA_XBAR_DEVICE;

/*
 * Typedef: enumeration MV_XBAR_DEVICE
 *
 * Description: Defines the different cossbar device types
 *
 * Fields:
 *      FX900A  - Mesh adaptor with 4 hyper Glinks and crossbar  -
 *                  4 x 4 serdes lanes.
 *      FX902A  - Stacking adaptor with 3 hyper Glinks and crossbar  -
 *                  2 x 4 serdes lanes + 1 x 6 serdes lanes
 *      FX910A  - Fabric adaptor with universal configuration -
 *                  2 x 6 serdes lanes, 3 x 6 serdes lanes,
 *                  2 x 4 serdes lanes + 1 x 6 serdes lanes,
 *                  4 x 4 serdes lanes or 3 x 6 serdes lanes.
 *      FX9010A - four port crossbar.
 *      FX9110  - Nine port crossbar.
 *      FX9210  - twelve port crossbar.
 *
 *      FX915   - Leoprad
 */
typedef enum
{
    MV_XBAR_98FX900A  = 0,
    MV_XBAR_98FX902A ,
    MV_XBAR_98FX910A ,
    MV_XBAR_98FX9010A,
    MV_XBAR_98FX9110,
    MV_XBAR_98FX9210,
    MV_XBAR_98FX915
}MV_XBAR_DEVICE;

/*
 * Typedef: enumeration MV_FA_DEVICE
 *
 * Description: Defines the different fabric adaptor device types
 *
 * Fields:
 *      FX900A  - Mesh adaptor with 4 hyper Glinks and crossbar  -
 *                  4 x 4 serdes lanes.
 *      FX902A  - Stacking adaptor with 3 hyper Glinks and crossbar  -
 *                  2 x 4 serdes lanes + 1 x 6 serdes lanes
 *      FX910A  - Fabric adaptor with universal configuration -
 *                  2 x 6 serdes lanes, 3 x 6 serdes lanes,
 *                  2 x 4 serdes lanes + 1 x 6 serdes lanes,
 *                  4 x 4 serdes lanes or 3 x 6 serdes lanes.
 *      FX915   - Leopard
 */
typedef enum
{
    MV_FA_98FX900A  = 0,
    MV_FA_98FX902A ,
    MV_FA_98FX910A,
    MV_FA_98FX915
}MV_FA_DEVICE;




/*
 * Typedef: enumeration MV_DEV_FAMILY
 *
 * Description: Defines the different device families that may exist in system.
 *
 * Fields:
 *      MV_TANGO_FAMILY - includes following devices: MV_98MX620, MV_98MX630,
 *                        MV_98EX120, MV_98EX130;
 *
 *      MV_TWIST_FAMILY - includes following devices:;
 *      MV_TWISTD_FAMILY - Twist-D family devices.
 *
 *      MV_SAMBA_FAMILY     - Samba family devices.
 *      MV_TIGER_FAMILY     - Tiger family devices.
 */
typedef enum
{
    MV_TANGO_FAMILY,
    MV_TWIST_FAMILY,
    MV_TWISTD_FAMILY,
    MV_SAMBA_FAMILY,
    MV_TIGER_FAMILY
}MV_DEV_FAMILY;

/*
 * Typedef: enumeration MV_FA_DEV_FAMILY
 *
 * Description: Defines the different device families that may exist in system.
 *
 * Fields:
 *      MV_FABRIC_FAMILY - includes following devices: MV_98FX900;
 *
 *      MV_XBAR_FAMILY - includes following devices:MV_98FX900;
 */
typedef enum
{
    MV_FABRIC_FAMILY,
    MV_XBAR_FAMILY
}MV_FA_XBAR_DEV_FAMILY;

/*
 * Typedef: enumeration MV_XBAR_DEV_FAMILY
 *
 * Description: Defines the different device families that may exist in system.
 *
 * Fields:
 *      MV_XBAR_FOX_FAMILY - includes following devices:MV_98FX900;
 *      MV_XBAR_LEO_FAMILY - MV_98FX915
 */
typedef enum
{
    MV_XBAR_FOX_FAMILY,
    MV_XBAR_CAP_FAMILY,
    MV_XBAR_LEO_FAMILY
}MV_XBAR_DEV_FAMILY;

/*
 * Typedef: enumeration MV_FA_DEV_FAMILY
 *
 * Description: Defines the different device families that may exist in system.
 *
 * Fields:
 *      MV_FA_FOX_FAMILY - includes following devices: MV_98FX900;
 *      MV_FA_LEO_FAMILY - includes following devices: MV_98FX915;
 *
 */
typedef enum
{
    MV_FA_FOX_FAMILY,
    MV_FA_LEO_FAMILY
}MV_FA_DEV_FAMILY;




/* specific error codes */
#define MV_PRESTERA_ERROR_BASE  0x10000

/* Hardware error*/
#define MV_PRESTERA_HW_ERROR         (MV_PRESTERA_ERROR_BASE | (0x17))
/* Transmit operation not succeeded */
#define MV_PRESTERA_TX_ERROR         (MV_PRESTERA_ERROR_BASE | (0x18))
/* Recieve operation not succeeded  */
#define MV_PRESTERA_RCV_ERROR        (MV_PRESTERA_ERROR_BASE | (0x19))
/* Re-perfrom the interrupt handling */
#define MV_PRESTERA_REDO             (MV_PRESTERA_ERROR_BASE | (0x1E))
/* PP memory allocation failed.          */
#define MV_PRESTERA_OUT_OF_PP_MEM    (MV_PRESTERA_ERROR_BASE | (0x1D))


#endif   /* __mvPresteraTypesh */


