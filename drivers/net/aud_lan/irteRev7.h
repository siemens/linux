//
// general defines for use with SOC IRTE 
//
// file "irteSOC.h"
// h.b. AD ATS 11 
// 21-July-2008
//


#ifndef SOCIRTE_H_
#define SOCIRTE_H_

#define SOC1_IRTE_START_ADDR	0xBD200000
#define IO_WORD(start, offset)	(*(volatile unsigned long *)(((start)+(offset))))
#define IO_SWITCH(offset)	IO_WORD(SOC1_IRTE_START_ADDR, offset)

#define IO_WRITE(offset,value)		\
{					\
	IO_SWITCH(offset) = value;	\
}

#define IO_READ(offset) \
		IO_SWITCH(offset)

#define LinkStatusMask 0x4
#define AutoNegComplMask 0x20


#define Min_Preamble_Count_P0		0x00004
#define NRT_Control_P0			0x00010
#define Default_VLAN_Tag_P0		0x00014
#define Min_Preamble_Count_P1		0x01004
#define NRT_Control_P1			0x01010
#define Default_VLAN_Tag_P1		0x01014
#define Cycle_Length			0x1102C
#define Cycle_Cnt_Entity		0x1103C
#define Clock_Cnt_Val			0x11418
#define NRT_Receive_MapCHA		0x12000
#define NRT_Receive_MapCHB		0x12004
#define ARP_Table_Base			0x1200C
#define ARP_Table_Length		0x12010
#define DCP_Table_Base			0x12014
#define NRT_Enable_CHA0			0x12400
#define NRT_Send_Descriptor_CHA0	0x12404
#define NRT_Receive_Descriptor_CHA0	0x12408
#define NRT_Enable_CHA1			0x1240C
#define NRT_Send_Descriptor_CHA1	0x12410
#define NRT_Receive_Descriptor_CHA1	0x12414
#define NRT_Enable_CHB0			0x12418
#define NRT_Send_Descriptor_CHB0	0x1241C
#define NRT_Receive_Descriptor_CHB0	0x12420
#define NRT_Enable_CHB1			0x12424
#define NRT_Send_Descriptor_CHB1	0x12428
#define NRT_Receive_Descriptor_CHB1	0x1242C
#define IRT_Control			0x13000
#define MD_Data				0x15000
#define MD_CA				0x15004
#define SMI_Config			0x15008
#define Link_Change         		0x1500C
#define PHY_Cmd_P0			0x15010
#define PHY_Stat_P0         		0x15014
#define PHY_Cmd_P1			0x15018
#define PHY_Stat_P1         		0x1501C
#define MAC_Control_P0			0x15440
#define Transmit_Control_P0		0x15448
#define Receive_Control_P0		0x15450
#define MAC_Control_P1			0x154C0
#define Transmit_Control_P1		0x154C8
#define Receive_Control_P1		0x154D0
#define NRT_FCW_Upper_Limit		0x16000
#define NRT_FCW_Lower_Limit		0x16004
#define NRT_DB_Upper_Limit		0x16008
#define NRT_DB_Lower_Limit		0x1600C
#define HOL_Upper_Limit_CH		0x16010
#define HOL_Lower_Limit_CH		0x16014
#define HOL_Upper_Limit_Port		0x16018
#define HOL_Lower_Limit_Port		0x1601C
#define UC_Table_Base			0x16020
#define UC_Table_Length			0x16024
#define UC_Default_Ctrl			0x16028
#define MC_Table_Base			0x1602C
#define MC_Table_Length			0x16030
#define MC_Default_Ctrl			0x16034
#define UCMC_LFSR_Ctrl			0x16038
#define UC_Table_Range			0x1603C
#define VLAN_Table_Base			0x16040
#define Group_Number			0x1604C
#define FC_MASK				0x16068
#define HOL_MASK_P0			0x1606C
#define HOL_MASK_P1			0x16070
#define Port_Control			0x1640C
#define SS_Queue_Disable		0x16418
#define Low_Water_Status		0x16420
#define NRT_FCW_Count			0x16424
#define NRT_DB_Count			0x16428
#define SP_IRQ_MODE      		0x17014
#define SP_IRQ1_MASK_IRT    		0x17020
#define SP_IRQ1_MASK_NRT    		0x17024
#define SP_IRQ1_NRT       		0x17434
#define SP_INT_ACK_NRT			0x1743C
#define SP_EOI_IRQ1       		0x1744C
#define SP_IRQ_ACTIVATE			0x17454
#define NRT_Fatal_Error			0x1745C
#define SA_31_0				0x1901C
#define SA_47_32			0x19020
#define NRT_Ctrl_Base			0x1904C
#define Free_Ctrl_Base			0x19050
#define Switch_Control			0x1905C
#define Min_Idle_Time			0x19060
#define MAC_Empty_Count			0x19064
#define NRT_Transfer_Control		0x19068
#define NRT_SafetyMargin		0x1906C
#define Statistic_Ctrl_Base		0x19074
#define Switch_Setup			0x19078
#define Version_Number			0x19400
#define Switch_Status			0x19404


#endif /*SOCIRTE_H_*/
