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

#ifndef __gtTypesh
#define __gtTypesh

#include <gtOs/gtGenTypes.h>


#define MARVELL_VENDOR_ID 0x11ab


typedef struct
{
    GT_U8   arIP[4];
}GT_IPADDR;

/* convert GT_IPADDR structure to word */
#define GT_IPADDR_TO_U32(ipAddr) \
       (((ipAddr).arIP[3])         |  \
        ((ipAddr).arIP[2] << 8)     |  \
        ((ipAddr).arIP[1] << 16)    |  \
        ((ipAddr).arIP[0] << 24))
        
/* convert U32 to GT_IPADDR structure */
#define GT_U32_TO_IPADDR(data, ipAddr) \
        (ipAddr).arIP[3] = (GT_U8) (((data)) & 0xFF);       \
        (ipAddr).arIP[2] = (GT_U8) (((data) >> 8) & 0xFF);  \
        (ipAddr).arIP[1] = (GT_U8) (((data) >> 16) & 0xFF); \
        (ipAddr).arIP[0] = (GT_U8) (((data) >> 24) & 0xFF)

typedef struct
{
    GT_U8       arEther[6];
}GT_ETHERADDR;

typedef struct
{
    GT_U32       lowWord;
    GT_U32       highHalfWord;

}GT_ETHERADDR_WORD;

typedef struct
{
    GT_U8 addr[16];

}GT_IPV6ADDR;

typedef struct
{
    GT_U32 word0; /* low bits 31:0 */
    GT_U32 word1;
    GT_U32 word2;
    GT_U32 word3;

}GT_IPV6ADDR_WORDS;

typedef struct 
{
    unsigned long word[3];
}GT_U96;


#define GT_HW_MAC_LOW32(macAddr)                \
           (GT_U32)((macAddr)->arEther[5] |          \
                ((macAddr)->arEther[4] << 8) |    \
                ((macAddr)->arEther[3] << 16) |   \
                ((macAddr)->arEther[2] << 24))

#define GT_HW_MAC_HIGH16(macAddr)           \
        (GT_U32)((macAddr)->arEther[1] | ((macAddr)->arEther[0] << 8))

#define GT_HW_MAC_LOW16(macAddr)            \
        (GT_U32)((macAddr)->arEther[5] | ((macAddr)->arEther[4] << 8))

#define GT_HW_MAC_HIGH32(macAddr)               \
                (GT_U32)((macAddr)->arEther[3] |          \
                ((macAddr)->arEther[2] << 8) |    \
                ((macAddr)->arEther[1] << 16) |   \
                ((macAddr)->arEther[0] << 24))

/* convert GT_ETHERADDR structure to word */
#define GT_ETHERADDR_TO_U32(arAddr, wordAddr) \
       wordAddr.lowWord = GT_HW_MAC_LOW32(arAddr); \
       wordAddr.highHalfWord = GT_HW_MAC_HIGH16(arAddr)

/* convert U32 to GT_ETHERADDR structure */
#define GT_U32_TO_ETHERADDR(arAddr, wordAddr) \
    arAddr.arEther[0] = (GT_U8) ((wordAddr).highHalfWord >> 8) & 0xFF; \
    arAddr.arEther[1] = (GT_U8) ((wordAddr.highHalfWord) & 0xFF); \
    arAddr.arEther[2] = (GT_U8) ((wordAddr.lowWord >> 24) & 0xFF); \
    arAddr.arEther[3] = (GT_U8) ((wordAddr.lowWord >> 16) & 0xFF); \
    arAddr.arEther[4] = (GT_U8) ((wordAddr.lowWord >> 8) & 0xFF); \
    arAddr.arEther[5] = (GT_U8) ((wordAddr.lowWord) & 0xFF); 
        

/*
 * typedef: enum GT_PRTCL
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
}GT_PRTCL;

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
 * typedef: GT_LPORT.
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
typedef GT_U32 GT_LPORT;

#define GT_LPORT_2_PORT(lport)      (GT_U8)((lport) & 0xff)
#define GT_LPORT_2_DEV(lport)       (GT_U8)(((lport) >> 8) & 0xff)
#define GT_LPORT_2_UNIT(lport)      (GT_U8)(((lport) >> 16) & 0xff)
#define GT_LPORT_IS_TRUNK(lport)    (GT_BOOL)((((lport) >> 31) & 0x1)==1? \
                                    GT_TRUE:GT_FALSE)
#define GT_LPORT_2_TRUNK_ID(lport)  (GT_U16)((lport) & 0xffff)

#define GT_PORT_DEV_2_LPORT(dev, port, unit)    \
        ((((unit) & 0xff) << 16) | (((dev) & 0xff) << 8) | ((port) & 0xff))
#define GT_TRUNK_2_LPORT(trunk)         (0x80000000 | (trunk))

/* CPU PORT NUMBER Definition */
#define CPU_PORT_NUM    63
#define GT_CPU_LPORT(dev, unit)    \
                ((((unit) & 0xff) << 16) | (((dev) & 0xff) << 8) | \
                (CPU_PORT_NUM & 0xff))

/* NULL Port */
#define GT_NULL_PORT_NUM   61

/* this macro should be used for regular Port validation */
/* CPU port does not allowed                             */
#define GT_PORT_NOT_EXIST(lPort)( (GT_LPORT_IS_TRUNK(lPort)) || \
                                (tapiPpDevs[GT_LPORT_2_DEV(lPort)] == NULL)||\
                                (GT_LPORT_2_PORT(lPort) >=                   \
                                tapiPpDevs[GT_LPORT_2_DEV(lPort)]->numOfPorts))

/* this macro should be used for regular Port validation */
/* CPU port does not allowed                             */
#define GT_PORT_EXIST(lPort) (!( (GT_LPORT_IS_TRUNK(lPort)) || \
                                (tapiPpDevs[GT_LPORT_2_DEV(lPort)] == NULL)||\
                                (GT_LPORT_2_PORT(lPort) >=                   \
                                tapiPpDevs[GT_LPORT_2_DEV(lPort)]->numOfPorts)))


/* this macro should be used for physical regular Port validation */
/* CPU port does not allowed                             */
#define GT_PHYSICAL_PORT_NOT_EXIST(lPort)( (GT_LPORT_IS_TRUNK(lPort)) || \
                                (tapiPpDevs[GT_LPORT_2_DEV(lPort)] == NULL)||\
                                (GT_LPORT_2_PORT(lPort) >=                   \
                       tapiPpDevs[GT_LPORT_2_DEV(lPort)]->numOfPhysicalPorts))

/* this macro should be used for physical regular Port validation */
/* CPU port does not allowed                             */
#define GT_PHYSICAL_PORT_EXIST(lPort) (!( (GT_LPORT_IS_TRUNK(lPort)) || \
                                (tapiPpDevs[GT_LPORT_2_DEV(lPort)] == NULL)||\
                                (GT_LPORT_2_PORT(lPort) >=                   \
                                tapiPpDevs[GT_LPORT_2_DEV(lPort)]->numOfPhysicalPorts)))


/* this macro should be used for CPU Port validation */
#define GT_CPU_PORT_NOT_EXIST(lPort)( (GT_LPORT_IS_TRUNK(lPort)) || \
                                (tapiPpDevs[GT_LPORT_2_DEV(lPort)] == NULL)||\
                                (GT_LPORT_2_PORT(lPort) != CPU_PORT_NUM))



/*
 * Typedef: enumeration GT_DEVICE
 *
 * Description: Defines the different device type that may exist in system.
 *
 * Fields:
 *  Tango group:
 *      GT_98MX620  -   10 * 1G
 *      GT_98MX630  -   1 * 10G
 *      GT_98EX120  -   10 * 1G
 *      GT_98EX130  -   1 * 10G
 *
 *  Twist group:
 *      GT_98MX610B     - 48 + 4 DSSMII full feature
 *      GT_98MX620B     - 10 * 1G full feature
 *      GT_98MX610BS    - 48 + 4 SSMII full feature
 *      GT_98EX110BS    - 48 + 4 SSMII no external rams
 *      GT_98EX111BS    - 48 + 4 SSMII no ns, ws
 *      GT_98EX112BS    - 48 + 4 SSMII no ws, no df
 *      GT_98EX110B     - 48 + 4 DSSMII no external rams
 *      GT_98EX111B     - 48 + 4 DSSMII no ns, ws
 *      GT_98EX112B     - 48 + 4 DSSMII no ws, no df
 *      GT_98EX120B     - 12G no external rams
 *      GT_98EX120B_    - 12G no external rams (TC enabled)
 *      GT_98EX121B     - 12G no ns, no ws
 *      GT_98EX122B     - 12G no ws, no df
 *      GT_98EX128B     - 12G  with extrenal memories
 *      GT_98EX129B     - 12G  with ws, and fd
 *
 *  Twist D group:
 *      GT_98EX110D  - 48+4 SSMII
 *      GT_98EX115D  - 48+4 SSMII with external narrow sram
 *      GT_98EX110DS - 48+4 DSSMII
 *      GT_98EX115DS - 48+4 DSSMII with external narrow sram
 *      GT_98EX120D  - 12G
 *      GT_98EX125D  - 12G with external narrow sram
 *      GT_98EX130D  - 1 * 10G (XG)
 *      GT_98EX135D  - 1 * 10G (XG) with external narrow sram
 *
 *  Samba group:
 *    
 *      GT_98MX625A  - 12G MX full feature
 *      GT_98MX625AB - 12G MX full feature - 0x197 - hardware device ID.
 *      GT_98MX615A  - 48+4 MX device with all external interfcaes - DSSMII
 *      GT_98MX615AS - 48+4 MX device with all external interfcaes - SSMII
 *      GT_98MX635A  - 1XGE MX 
 *
 *  Tiger group:
 *
 *      GT_98EX116 - 48F+4G Tiger
 *      GT_98EX106 - 24F+4G Tiger
 *      GT_98EX108 - 48F+4G Tiger
 *      GT_98EX126 - 12G Tiger
 *      GT_98EX126_VB - 12G Tiger value blade configuration
 *      GT_98EX126_VB_PHY - 12G Tiger value blade configuration with physical
 *                          connection to to external boards.       
 *      GT_98EX136 - 1XG Tiger
 *      GT_98EX136_VB - 1XG Tiger value blade configuration
 *      GT_98EX136_VB_PHY - 1XG Tiger value blade configuration with physical
 *                          connection to to external boards.       
 */
typedef enum
{
    GT_98MX620A = 0x000011AB,
    GT_98MX630A = 0x001011AB,
    GT_98EX120A = 0x000B11AB,
    GT_98EX130A = 0x001B11AB,

    GT_98MX610B  = 0x007011AB,
    GT_98MX620B  = 0x004011AB,
    GT_98MX610BS = 0x006011AB,
    GT_98EX110BS = 0x006711AB,
    GT_98EX111BS = 0x006311AB,
    GT_98EX112BS = 0x006511AB,
    GT_98EX110B  = 0x007711AB,
    GT_98EX111B  = 0x007311AB,
    GT_98EX112B  = 0x007511AB,
    GT_98EX120B  = 0x005711AB,
    GT_98EX120B_ = 0x005611AB,
    GT_98EX121B  = 0x005311AB,
    GT_98EX122B  = 0x005511AB,
    GT_98EX128B  = 0x005011AB,
    GT_98EX129B  = 0x005211AB,

    GT_98EX110D  = 0x00E711AB,
    GT_98EX115D  = 0x00E111AB,
    GT_98EX110DS = 0x00F711AB,
    GT_98EX115DS = 0x00F111AB,
    GT_98EX120D  = 0x00D711AB,
    GT_98EX125D  = 0x00D111AB,
    GT_98EX130D  = 0x01D711AB,
    GT_98EX135D  = 0x01D111AB,

    GT_98MX625A  = 0x018011AB,
    GT_98MX625AB = 0x019711AB,
    GT_98MX615A  = 0x01B011AB,
    GT_98MX615AS = 0x01A011AB,
    GT_98MX635A  = 0x01D011AB,

    GT_98EX116   = 0x012011AB, /* 0x012x11AB */
    GT_98EX106   = 0x012A11AB,
    GT_98EX108   = 0x012B11AB,
    GT_98EX126   = 0x011011AB, /* 0x011x11AB */
    GT_98EX126_VB = 0x111011AB, 
    GT_98EX126_VB_PHY = 0x111111AB,
    GT_98EX136   = 0x015011AB,  /* 0x015x11AB */
    GT_98EX136_VB = 0x115011AB, 
    GT_98EX136_VB_PHY = 0x115111AB

}GT_DEVICE;

/*
 * Typedef: enumeration GT_FA_XBAR_DEVICE
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
    GT_98FX900A  = 0,
    GT_98FX902A ,
    GT_98FX910A ,
    GT_98FX9010A,
    GT_98FX9110,
    GT_98FX9210,
    GT_98FX915
}GT_FA_XBAR_DEVICE;

/*
 * Typedef: enumeration GT_XBAR_DEVICE
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
    GT_XBAR_98FX900A  = 0,
    GT_XBAR_98FX902A ,
    GT_XBAR_98FX910A ,
    GT_XBAR_98FX9010A,
    GT_XBAR_98FX9110,
    GT_XBAR_98FX9210,
    GT_XBAR_98FX915
}GT_XBAR_DEVICE;

/*
 * Typedef: enumeration GT_FA_DEVICE
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
    GT_FA_98FX900A  = 0,
    GT_FA_98FX902A ,
    GT_FA_98FX910A,
    GT_FA_98FX915
}GT_FA_DEVICE;




/*
 * Typedef: enumeration GT_DEV_FAMILY
 *
 * Description: Defines the different device families that may exist in system.
 *
 * Fields:
 *      GT_TANGO_FAMILY - includes following devices: GT_98MX620, GT_98MX630,
 *                        GT_98EX120, GT_98EX130;
 *
 *      GT_TWIST_FAMILY - includes following devices:;
 *      GT_TWISTD_FAMILY - Twist-D family devices.
 *
 *      GT_SAMBA_FAMILY     - Samba family devices.
 *      GT_TIGER_FAMILY     - Tiger family devices.
 */
typedef enum
{
    GT_TANGO_FAMILY,
    GT_TWIST_FAMILY,
    GT_TWISTD_FAMILY,
    GT_SAMBA_FAMILY,
    GT_TIGER_FAMILY
}GT_DEV_FAMILY;

/*
 * Typedef: enumeration GT_FA_DEV_FAMILY
 *
 * Description: Defines the different device families that may exist in system.
 *
 * Fields:
 *      GT_FABRIC_FAMILY - includes following devices: GT_98FX900;
 *
 *      GT_XBAR_FAMILY - includes following devices:GT_98FX900;
 */
typedef enum
{
    GT_FABRIC_FAMILY,
    GT_XBAR_FAMILY
}GT_FA_XBAR_DEV_FAMILY;

/*
 * Typedef: enumeration GT_XBAR_DEV_FAMILY
 *
 * Description: Defines the different device families that may exist in system.
 *
 * Fields:
 *      GT_XBAR_FOX_FAMILY - includes following devices:GT_98FX900;
 *      GT_XBAR_LEO_FAMILY - GT_98FX915
 */
typedef enum
{
    GT_XBAR_FOX_FAMILY,
    GT_XBAR_CAP_FAMILY,
    GT_XBAR_LEO_FAMILY
}GT_XBAR_DEV_FAMILY;

/*
 * Typedef: enumeration GT_FA_DEV_FAMILY
 *
 * Description: Defines the different device families that may exist in system.
 *
 * Fields:
 *      GT_FA_FOX_FAMILY - includes following devices: GT_98FX900;
 *      GT_FA_LEO_FAMILY - includes following devices: GT_98FX915;
 *
 */
typedef enum
{
    GT_FA_FOX_FAMILY,
    GT_FA_LEO_FAMILY
}GT_FA_DEV_FAMILY;




/* specific error codes */
#define GT_PRESTERA_ERROR_BASE  0x10000

#define GT_HW_ERROR         (GT_PRESTERA_ERROR_BASE | (0x17))/* Hardware error*/
/* Transmit operation not succeeded */
#define GT_TX_ERROR         (GT_PRESTERA_ERROR_BASE | (0x18))
/* Recieve operation not succeeded  */
#define GT_RCV_ERROR        (GT_PRESTERA_ERROR_BASE | (0x19))
/* Re-perfrom the interrupt handling */
#define GT_REDO             (GT_PRESTERA_ERROR_BASE | (0x1E))
/* PP memory allocation failed.          */
#define GT_OUT_OF_PP_MEM    (GT_PRESTERA_ERROR_BASE | (0x1D))


#endif   /* __gtTypesh */


