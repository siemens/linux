/* arch/mips/include/asm/mach-aud_soc1/regs-serial.h
 *
 *  From linux/include/asm-arm/hardware/serial_s3c2410.h
 *  From linux/include/asm-arm/arch-s3c2410/regs-serial.h
 *
 *  Copyright (C) 2008 Manfred.Neugebauer@siemens.com
 *
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
 *
 * Modifications:
 */

#ifndef __ASM_MIPS_MACH_AUD_SOC1_REGS_SERIAL_H
#define __ASM_MIPS_MACH_AUD_SOC1_REGS_SERIAL_H

/* s_soc_uart_devs
 *
 * this is exported from the core as we cannot use driver_register(),
 * or platform_add_device() before the console_initcall()
*/

extern struct platform_device *s_soc_uart_devs[];

#endif /* __ASM_MIPS_MACH_AUD_SOC1_REGS_SERIAL_H */

