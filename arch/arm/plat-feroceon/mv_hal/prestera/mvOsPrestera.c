#include "mvOsPrestera.h"

/*******************************************************************************
* osCacheDmaMalloc - Allocated memory for RX and TX descriptors.
*
* DESCRIPTION:
*       This function allocates memory for RX and TX descriptors.

* INPUT:
*       int size - size of memory should be allocated.
*
* RETURN: None
*
*******************************************************************************/
MV_U8*  osCacheDmaMalloc(void* osHandle, int descSize,
				   MV_ULONG* pPhysAddr, MV_U32 *memHandle)
{
    MV_U8*  pVirt;

    pVirt = (char*)mvOsIoUncachedMalloc(osHandle, descSize, 
					    pPhysAddr,memHandle);

/*    pVirt = (char*)mvOsIoCachedMalloc(osHandle, descSize, 
					  pPhysAddr,memHandle);*/

    memset(pVirt, 0, descSize);

    return pVirt;
}

void mvOsVirt2Phy(MV_U32 virtAddr, MV_U32* pPhyAddr)
{
  
    *pPhyAddr = mvOsIoVirtToPhy(NULL, &virtAddr);	

	return;
}

void mvOsPhy2Virt(MV_U32 phyAddr, MV_U32* pVirtAddr)
{
    return;
}



