/*******************************************************************************
Copyright (C) Marvell International Ltd. and its affiliates
*******************************************************************************
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

*******************************************************************************
* pssBspApis.h - bsp APIs
*
* DESCRIPTION:
*       Inline and macro functions are placed here for performance
*
* DEPENDENCIES:
*       None.
*       
* FILE REVISION NUMBER:
*       $Revision: 1 $
*
*******************************************************************************/

#ifndef __pssBspApisInlineH
#define __pssBspApisInlineH
#ifndef __ASSEMBLY__

extern unsigned long mv_high_memory_paddr;
extern unsigned long mv_high_memory_paddr_top;
extern unsigned long mv_high_memory_vaddr;
extern unsigned long mv_high_memory_vaddr_top;
extern unsigned long mv_high_memory_vaddr_minus_paddr;
extern unsigned long mv_high_memory_paddr_minus_vaddr;
extern unsigned long mv_high_memory_len;

#define bspVirtIsInMvHiMem(vAddr) \
  ((((unsigned long)vAddr >= mv_high_memory_vaddr) && \
    ((unsigned long)vAddr <= mv_high_memory_vaddr_top))? 1 : 0)

#define bspPhysIsInMvHiMem(pAddr) \
  ((((unsigned long)pAddr >= mv_high_memory_paddr) &&                            \
    ((unsigned long)pAddr <= mv_high_memory_paddr_top))? 1 : 0)

#define bspVirt2Phys(vAddr) \
  ((((unsigned long)vAddr >= mv_high_memory_vaddr) && \
    ((unsigned long)vAddr <= mv_high_memory_vaddr_top)) ? \
   ((unsigned long)((unsigned long)vAddr - mv_high_memory_vaddr_minus_paddr)) : \
   ((unsigned long)__pa(vAddr)))

#define bspPhys2Virt(pAddr) \
  ((((unsigned long)pAddr >= mv_high_memory_paddr) && \
    ((unsigned long)pAddr <= mv_high_memory_paddr_top)) ?               \
   ((unsigned long)((unsigned long)pAddr - mv_high_memory_paddr_minus_vaddr)) : \
   0)

#define bspIntEnable(intMask) (enable_irq(intMask) & 0)

#define bspIntDisable(intMask) (disable_irq(intMask) & 0)

// for debugging
#define GC_LOOP __asm__ __volatile__("19:\n" "b 19b\n" :::"memory")

#endif /*  __ASSEMBLY__ */
#endif /* __pssBspApisInlineH */

