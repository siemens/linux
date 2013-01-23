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

#include "mvOs.h"
#include "mv_switch.h"
#include "eth-phy/mvEthPhy.h"
#include "mv_switchRegs.h"
#include "ctrlEnv/mvCtrlEnvLib.h"


static void switchVlanInit(MV_U32 ethPortNum,
						   MV_U32 switchCpuPort,
					   MV_U32 switchMaxPortsNum,
					   MV_U32 switchPortsOffset,
					   MV_U32 switchEnabledPortsMask);


/*******************************************************************************
* mvEthE6065_61PhyBasicInit - 
*
* DESCRIPTION:
*	Do a basic Init to the Phy , including reset
*       
* INPUT:
*       ethPortNum - Ethernet port number
*
* OUTPUT:
*       None.
*
* RETURN:   None
*
*******************************************************************************/
MV_VOID		mvEthE6065_61SwitchBasicInit(MV_U32 ethPortNum)
{
	switchVlanInit(ethPortNum,
			   MV_E6065_CPU_PORT,
			   MV_E6065_MAX_PORTS_NUM,
			   MV_E6065_PORTS_OFFSET,
			   MV_E6065_ENABLED_PORTS);
}

/*******************************************************************************
* mvEthE6063PhyBasicInit - 
*
* DESCRIPTION:
*	Do a basic Init to the Phy , including reset
*       
* INPUT:
*       ethPortNum - Ethernet port number
*
* OUTPUT:
*       None.
*
* RETURN:   None
*
*******************************************************************************/
MV_VOID		mvEthE6063SwitchBasicInit(MV_U32 ethPortNum)
{
	switchVlanInit(ethPortNum,
			   MV_E6063_CPU_PORT,
			   MV_E6063_MAX_PORTS_NUM,
			   MV_E6063_PORTS_OFFSET,
			   MV_E6063_ENABLED_PORTS);
}

/*******************************************************************************
* mvEthE6131PhyBasicInit - 
*
* DESCRIPTION:
*	Do a basic Init to the Phy , including reset
*       
* INPUT:
*       ethPortNum - Ethernet port number
*
* OUTPUT:
*       None.
*
* RETURN:   None
*
*******************************************************************************/
MV_VOID		mvEthE6131SwitchBasicInit(MV_U32 ethPortNum)
{

	MV_U16 reg;

	/*Enable Phy power up*/
	mvEthPhyRegWrite (0,0,0x9140);
	mvEthPhyRegWrite (1,0,0x9140);
	mvEthPhyRegWrite (2,0,0x9140);


	/*Enable PPU*/
	mvEthPhyRegWrite (0x1b,4,0x4080);


	/*Enable Phy detection*/
	mvEthPhyRegRead (0x13,0,&reg);
	reg &= ~(1<<12);
	mvEthPhyRegWrite (0x13,0,reg);

	mvOsDelay(100);
	mvEthPhyRegWrite (0x13,1,0x33);


	switchVlanInit(ethPortNum,
			    MV_E6131_CPU_PORT,
			   MV_E6131_MAX_PORTS_NUM,
			   MV_E6131_PORTS_OFFSET,
			   MV_E6131_ENABLED_PORTS);

}


/*******************************************************************************
* mvEthE6161PhyBasicInit - 
*
* DESCRIPTION:
*	Do a basic Init to the Phy , including reset
*       
* INPUT:
*       ethPortNum - Ethernet port number
*
* OUTPUT:
*       None.
*
* RETURN:   None
*
*******************************************************************************/
MV_VOID		mvEthE6161SwitchBasicInit(void)
{

    MV_U32 prt;
    MV_U16 reg;
    volatile MV_U32 timeout; 

    /* The 6161/5 needs a delay */
    mvOsDelay(100);

    /* Init vlan */
    switchVlanInit(0,
		    MV_E6161_CPU_PORT,
		    MV_E6161_MAX_PORTS_NUM,
		    MV_E6161_PORTS_OFFSET,
		    MV_E6161_ENABLED_PORTS);

    /* Enable RGMII delay on Tx and Rx for CPU port */
    mvEthPhyRegWrite (0x14,0x1a,0x81e7);
    mvEthPhyRegRead (0x15,0x1a,&reg);
    mvEthPhyRegWrite (0x15,0x1a,0x18);
    mvEthPhyRegWrite (0x14,0x1a,0xc1e7);

    for(prt=0; prt < MV_E6161_MAX_PORTS_NUM; prt++)
    {
	if (prt != MV_E6161_CPU_PORT)
	{
	    /*Enable Phy power up*/
	    mvEthPhyRegWrite (MV_E6161_GLOBAL_2_REG_DEV_ADDR, 
				MV_E6161_SMI_PHY_DATA, 0x3360);
	    mvEthPhyRegWrite (MV_E6161_GLOBAL_2_REG_DEV_ADDR, 
				MV_E6161_SMI_PHY_COMMAND, (0x9410 | (prt << 5)));

	    /*Make sure SMIBusy bit cleared before another SMI operation can take place*/
	    timeout = E6161_PHY_TIMEOUT;
	    do
            {
	     	mvEthPhyRegRead(MV_E6161_GLOBAL_2_REG_DEV_ADDR,
				MV_E6161_SMI_PHY_COMMAND,&reg);
		if(timeout-- == 0)
		{
		 	mvOsPrintf("mvEthPhyRegRead: SMI busy timeout\n");
			return;
		}
	    }while (reg & E6161_PHY_SMI_BUSY_MASK);   

	    mvEthPhyRegWrite (MV_E6161_GLOBAL_2_REG_DEV_ADDR,
				MV_E6161_SMI_PHY_DATA,0x1140);
	    mvEthPhyRegWrite (MV_E6161_GLOBAL_2_REG_DEV_ADDR,
				MV_E6161_SMI_PHY_COMMAND,(0x9400 | (prt << 5)));

	    /*Make sure SMIBusy bit cleared before another SMI operation can take place*/
	    timeout = E6161_PHY_TIMEOUT;
	    do
            {
	     	mvEthPhyRegRead(MV_E6161_GLOBAL_2_REG_DEV_ADDR,
				MV_E6161_SMI_PHY_COMMAND,&reg);
		if(timeout-- == 0)
		{
		 	mvOsPrintf("mvEthPhyRegRead: SMI busy timeout\n");
			return;
		}
	    }while (reg & E6161_PHY_SMI_BUSY_MASK);   

	}

	/*Enable port*/
	mvEthPhyRegWrite (MV_E6161_PORTS_OFFSET + prt, 4, 0x7f);
    }

    /*Force CPU port to RGMII FDX 1000Base*/
    mvEthPhyRegWrite (MV_E6161_PORTS_OFFSET + MV_E6161_CPU_PORT, 1, 0x3e);
}

static void switchVlanInit(MV_U32 ethPortNum,
						   MV_U32 switchCpuPort,
					   MV_U32 switchMaxPortsNum,
					   MV_U32 switchPortsOffset,
					   MV_U32 switchEnabledPortsMask)
{
    MV_U32 prt;
	MV_U16 reg;

	/* be sure all ports are disabled */
    	for(prt=0; prt < switchMaxPortsNum; prt++)
	{
		mvEthPhyRegRead (mvBoardPhyAddrGet(ethPortNum)+ MV_SWITCH_PORT_OFFSET(prt),
							  MV_SWITCH_PORT_CONTROL_REG,&reg);
		reg &= ~0x3;
		mvEthPhyRegWrite (mvBoardPhyAddrGet(ethPortNum)+ MV_SWITCH_PORT_OFFSET(prt),
							  MV_SWITCH_PORT_CONTROL_REG,reg);

	}

	/* Set CPU port VID to 0x1 */
	mvEthPhyRegRead (mvBoardPhyAddrGet(ethPortNum)+ MV_SWITCH_PORT_OFFSET(switchCpuPort),
						  MV_SWITCH_PORT_VID_REG,&reg);
	reg &= ~0xfff;
	reg |= 0x1;
	mvEthPhyRegWrite (mvBoardPhyAddrGet(ethPortNum)+ MV_SWITCH_PORT_OFFSET(switchCpuPort),
						  MV_SWITCH_PORT_VID_REG,reg);


	/* Setting  Port default priority for all ports to zero */
    	for(prt=0; prt < switchMaxPortsNum; prt++)
	{

		mvEthPhyRegRead (mvBoardPhyAddrGet(ethPortNum)+ MV_SWITCH_PORT_OFFSET(prt),
							  MV_SWITCH_PORT_VID_REG,&reg);
		reg &= ~0xc000;
		mvEthPhyRegWrite (mvBoardPhyAddrGet(ethPortNum)+ MV_SWITCH_PORT_OFFSET(prt),
							  MV_SWITCH_PORT_VID_REG,reg);
	}

	/* Setting VID and VID map for all ports except CPU port */
    	for(prt=0; prt < switchMaxPortsNum; prt++)
	{

		/* only for enabled ports */
        	if ((1 << prt)& switchEnabledPortsMask)
		{
			/* skip CPU port */
			if (prt== switchCpuPort) continue;

			/* 
			*  set Ports VLAN Mapping.
			*	port prt <--> MV_SWITCH_CPU_PORT VLAN #prt+1.
			*/
	
			mvEthPhyRegRead (mvBoardPhyAddrGet(ethPortNum)+ MV_SWITCH_PORT_OFFSET(prt),
								  MV_SWITCH_PORT_VID_REG,&reg);
			reg &= ~0x0fff;
			reg |= (prt+1);
			mvEthPhyRegWrite (mvBoardPhyAddrGet(ethPortNum)+ MV_SWITCH_PORT_OFFSET(prt),
								  MV_SWITCH_PORT_VID_REG,reg);
	
	
			/* Set Vlan map table for all ports to send only to MV_SWITCH_CPU_PORT */
			mvEthPhyRegRead (mvBoardPhyAddrGet(ethPortNum)+ MV_SWITCH_PORT_OFFSET(prt),
								  MV_SWITCH_PORT_VMAP_REG,&reg);
			reg &= ~((1 << switchMaxPortsNum) - 1);
			reg |= (1 << switchCpuPort);
			mvEthPhyRegWrite (mvBoardPhyAddrGet(ethPortNum)+ MV_SWITCH_PORT_OFFSET(prt),
								  MV_SWITCH_PORT_VMAP_REG,reg);
		}

	}


	/* Set Vlan map table for MV_SWITCH_CPU_PORT to see all ports*/
	mvEthPhyRegRead (mvBoardPhyAddrGet(ethPortNum)+ MV_SWITCH_PORT_OFFSET(switchCpuPort),
						  MV_SWITCH_PORT_VMAP_REG,&reg);
	reg &= ~((1 << switchMaxPortsNum) - 1);
	reg |= switchEnabledPortsMask & ~(1 << switchCpuPort);
	mvEthPhyRegWrite (mvBoardPhyAddrGet(ethPortNum)+ MV_SWITCH_PORT_OFFSET(switchCpuPort),
						  MV_SWITCH_PORT_VMAP_REG,reg);


    	/*enable only appropriate ports to forwarding mode - and disable the others*/
    	for(prt=0; prt < switchMaxPortsNum; prt++)
	{

        	if ((1 << prt)& switchEnabledPortsMask)
		{
			mvEthPhyRegRead (mvBoardPhyAddrGet(ethPortNum)+ MV_SWITCH_PORT_OFFSET(prt),
								  MV_SWITCH_PORT_CONTROL_REG,&reg);
			reg |= 0x3;
			mvEthPhyRegWrite (mvBoardPhyAddrGet(ethPortNum)+ MV_SWITCH_PORT_OFFSET(prt),
								  MV_SWITCH_PORT_CONTROL_REG,reg);

		}
		else
		{
			/* Disable port */
			mvEthPhyRegRead (mvBoardPhyAddrGet(ethPortNum)+ MV_SWITCH_PORT_OFFSET(prt),
								  MV_SWITCH_PORT_CONTROL_REG,&reg);
			reg &= ~0x3;
			mvEthPhyRegWrite (mvBoardPhyAddrGet(ethPortNum)+ MV_SWITCH_PORT_OFFSET(prt),
								  MV_SWITCH_PORT_CONTROL_REG,reg);

		}
	}


	return;

}






