// SPDX-License-Identifier: GPL-2.0
/*
 * TI Peripheral Virtualization Unit driver for static DMA isolation
 *
 * Copyright (c) 2024, Siemens AG
 */

#ifndef _LINUX_TI_PVU_H
#define _LINUX_TI_PVU_H

#include <linux/ioport.h>

int ti_pvu_create_region(unsigned int virt_id, const struct resource *region);
int ti_pvu_remove_region(unsigned int virt_id, const struct resource *region);

#endif /* _LINUX_TI_PVU_H */
