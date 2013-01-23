/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */


#include "mvSysHwConfig.h"
#include "ctrlEnv/sys/mvCpuIf.h"
#include <asm/mach/map.h>

/* for putstr */
/* #include <asm/arch/uncompress.h> */

MV_CPU_DEC_WIN* mv_sys_map(void);

#if defined(CONFIG_MV_INCLUDE_CESA)
u32 mv_crypto_base_get(void);
#endif

#define DRAGONITE_UNIT_BASE 0xF2000000
#define DRAGONITE_UNIT_SIZE _64K
struct map_desc  MEM_TABLE[] =	{
   // the first entry is for Marvell dma remap. See mv_map_io below.
  { 0x00000000, __phys_to_pfn(0x000000000), 0, MT_DEVICE},

	/* no use for pex mem remap */  
	//  { PEX0_MEM_BASE,              __phys_to_pfn(PEX0_MEM_BASE),           PEX0_MEM_SIZE,          MT_DEVICE},
  { INTER_REGS_BASE, __phys_to_pfn(INTER_REGS_BASE), 	SZ_1M, MT_DEVICE},

	//  { PEX0_IO_BASE,   		__phys_to_pfn(PEX0_IO_BASE),   	 	PEX0_IO_SIZE,  		MT_DEVICE},
	//  { NFLASH_CS_BASE, 		__phys_to_pfn(NFLASH_CS_BASE), 		NFLASH_CS_SIZE, 	MT_DEVICE},


  { SPI_CS_BASE, __phys_to_pfn(SPI_CS_BASE), SPI_CS_SIZE, 		MT_DEVICE},
#ifdef MV_INCLUDE_SAGE
  { SAGE_UNIT_BASE, __phys_to_pfn(SAGE_UNIT_BASE), SAGE_UNIT_SIZE, MT_DEVICE},
#endif
  { CRYPT_ENG_BASE, __phys_to_pfn(CRYPT_ENG_BASE), CRYPT_ENG_SIZE, MT_DEVICE},
  { DRAGONITE_UNIT_BASE, __phys_to_pfn(DRAGONITE_UNIT_BASE), DRAGONITE_UNIT_SIZE, MT_DEVICE}
};


MV_CPU_DEC_WIN SYSMAP_88F6281[] = {
  	 /* base low        base high    size       	WinNum     enable */
	{{SDRAM_CS0_BASE ,    0,      SDRAM_CS0_SIZE } ,0xFFFFFFFF,DIS},
	{{SDRAM_CS1_BASE ,    0,      SDRAM_CS1_SIZE } ,0xFFFFFFFF,DIS},
	{{SDRAM_CS2_BASE ,    0,      SDRAM_CS2_SIZE } ,0xFFFFFFFF,DIS},
	{{SDRAM_CS3_BASE ,    0,      SDRAM_CS3_SIZE } ,0xFFFFFFFF,DIS},
	{{PEX0_MEM_BASE  ,    0,      PEX0_MEM_SIZE  } ,0x1       ,EN},
	{{PEX0_IO_BASE   ,    0,      PEX0_IO_SIZE   } ,0x3       ,EN},
	{{INTER_REGS_BASE,    0,      INTER_REGS_SIZE} ,0x8       ,EN},
	{{NFLASH_CS_BASE,     0,      NFLASH_CS_SIZE}  ,0x2	      ,EN}, 
	{{SPI_CS_BASE,        0,      SPI_CS_SIZE    } ,0x5       ,EN},
	{{DEVICE_CS2_BASE,    0,      DEVICE_CS2_SIZE}, 0x6	      ,DIS},
 	{{BOOTDEV_CS_BASE,    0,      BOOTDEV_CS_SIZE}, 0x4	      ,DIS},
	{{CRYPT_ENG_BASE,     0,      CRYPT_ENG_SIZE}  ,0x7  	  ,EN},
#ifdef MV_INCLUDE_SAGE
	{{SAGE_UNIT_BASE, 	  0,      SAGE_UNIT_SIZE}  ,0x6    	  ,EN},    
#endif
#ifdef MV_INCLUDE_DRAGONITE
    {{DRAGONITE_UNIT_BASE, 0,     DRAGONITE_UNIT_SIZE},0x0    ,EN},
#endif 
	{{TBL_TERM,TBL_TERM, TBL_TERM} ,TBL_TERM  ,TBL_TERM}		
};

MV_CPU_DEC_WIN SYSMAP_88F6180[] = {
  	 /* base low        base high    size       	WinNum     enable */
	{{SDRAM_CS0_BASE ,    0,      SDRAM_CS0_SIZE } ,0xFFFFFFFF,DIS},
	{{SDRAM_CS1_BASE ,    0,      SDRAM_CS1_SIZE } ,0xFFFFFFFF,DIS},
	{{SDRAM_CS2_BASE ,    0,      SDRAM_CS2_SIZE } ,0xFFFFFFFF,DIS},
	{{SDRAM_CS3_BASE ,    0,      SDRAM_CS3_SIZE } ,0xFFFFFFFF,DIS},
	{{PEX0_MEM_BASE  ,    0,      PEX0_MEM_SIZE  } ,0x1       ,EN},
	{{PEX0_IO_BASE   ,    0,      PEX0_IO_SIZE   } ,0x0       ,EN},
	{{INTER_REGS_BASE,    0,      INTER_REGS_SIZE} ,0x8       ,EN},
	{{NFLASH_CS_BASE,     0,      NFLASH_CS_SIZE } ,0x2	  ,EN}, 
	{{SPI_CS_BASE,        0,      SPI_CS_SIZE}     ,0x5       ,EN}, 
	{{DEVICE_CS2_BASE,    0,      DEVICE_CS2_SIZE}, 0x6	  ,DIS},
 	{{BOOTDEV_CS_BASE,    0,      BOOTDEV_CS_SIZE}, 0x4	  ,DIS},
	{{CRYPT_ENG_BASE,     0,      CRYPT_ENG_SIZE}  ,0x7  	  ,EN},
	{{TBL_TERM,TBL_TERM, TBL_TERM} ,TBL_TERM  ,TBL_TERM}		
};

MV_CPU_DEC_WIN SYSMAP_88F6192[] = {
  	 /* base low        base high    size       	WinNum     enable */
	{{SDRAM_CS0_BASE ,    0,      SDRAM_CS0_SIZE } ,0xFFFFFFFF,DIS},
	{{SDRAM_CS1_BASE ,    0,      SDRAM_CS1_SIZE } ,0xFFFFFFFF,DIS},
	{{SDRAM_CS2_BASE ,    0,      SDRAM_CS2_SIZE } ,0xFFFFFFFF,DIS},
	{{SDRAM_CS3_BASE ,    0,      SDRAM_CS3_SIZE } ,0xFFFFFFFF,DIS},
	{{PEX0_MEM_BASE  ,    0,      PEX0_MEM_SIZE  } ,0x1       ,EN},
	{{PEX0_IO_BASE   ,    0,      PEX0_IO_SIZE   } ,0x3       ,EN},
	{{INTER_REGS_BASE,    0,      INTER_REGS_SIZE} ,0x8       ,EN},
	{{NFLASH_CS_BASE,     0,      NFLASH_CS_SIZE}  ,0x2	  ,EN}, 
	{{SPI_CS_BASE,        0,      SPI_CS_SIZE}     ,0x5       ,EN},
	{{DEVICE_CS2_BASE,    0,      DEVICE_CS2_SIZE}, 0x6	  ,DIS},
 	{{BOOTDEV_CS_BASE,    0,      BOOTDEV_CS_SIZE}, 0x4	  ,DIS},
	{{CRYPT_ENG_BASE,     0,      CRYPT_ENG_SIZE}  ,0x7  	  ,EN},
#ifdef MV_INCLUDE_SAGE
	{{SAGE_UNIT_BASE, 	  0,      SAGE_UNIT_SIZE}  ,0x6    	  ,EN},    
#endif
#ifdef MV_INCLUDE_DRAGONITE
    {{DRAGONITE_UNIT_BASE, 0,     DRAGONITE_UNIT_SIZE},0x0    ,EN},
#endif 
	{{TBL_TERM,TBL_TERM, TBL_TERM} ,TBL_TERM  ,TBL_TERM}		
};


MV_CPU_DEC_WIN* mv_sys_map(void)
{
	switch(mvBoardIdGet()) {
	case DB_88F6281A_BP_ID:
	case RD_88F6281A_ID: 
		return SYSMAP_88F6281;
	case DB_88F6192A_BP_ID:
	case RD_88F6192A_ID:
	case DB_88F6190A_BP_ID:
	case RD_88F6190A_ID:
	case RD_98DX3121_ID:
	case DB_98DX4122_ID:
	case DB_X500PROTOTYPE_ID:
		return SYSMAP_88F6192;
	case DB_88F6180A_BP_ID:
		return SYSMAP_88F6180;
	default:
		printk("ERROR: can't find system address map\n");
		return NULL;
	}
}


#if defined(CONFIG_MV_INCLUDE_CESA)
u32 mv_crypto_base_get(void)
{
	return CRYPT_ENG_BASE;
}
#endif

int xcat_version;

void __init mv_map_io(void)
{
  
  extern unsigned long mv_high_memory_paddr;
  extern unsigned long mv_high_memory_vaddr;
  extern unsigned long mv_high_memory_len;
	extern unsigned long PRESTERA_DEV_ID_REG_OFFSET_extern;
	extern unsigned long XCAT_INTERNAL_PP_BASE_ADDR_extern;
	extern void fix_xcat_version(void);
	extern MV_U32 bspReadRegisterInternal(IN MV_U32  address);

  int i;
	unsigned long prestera_device_id;

  MEM_TABLE[0].virtual = mv_high_memory_vaddr;
  MEM_TABLE[0].pfn     = __phys_to_pfn(mv_high_memory_paddr);
  MEM_TABLE[0].length  = mv_high_memory_len;

  printk("calling iotable_init. MEM_TABLE=\n");
  for (i = 0; i < ARRAY_SIZE(MEM_TABLE); i++)
    printk("  virt=0x%08lx, phys=0x%08lx, lengh=0x%08lx\n",
           MEM_TABLE[i].virtual,
           (MEM_TABLE[i].pfn << PAGE_SHIFT),
           MEM_TABLE[i].length);    
  iotable_init(MEM_TABLE, ARRAY_SIZE(MEM_TABLE));

	//
	// at this point we can access the internal xcat prestera id register ...
	//
	prestera_device_id = bspReadRegisterInternal(PRESTERA_DEV_ID_REG_OFFSET_extern + 
										 XCAT_INTERNAL_PP_BASE_ADDR_extern);	
	printk("Internal prestera id register = 0x%08lx\n", prestera_device_id);
	xcat_version = prestera_device_id & 0x0000000f; // first 4 bits 
	fix_xcat_version();
}

