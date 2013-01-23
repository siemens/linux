/*
 *
 * error handler enhanced for invalid pci accesses
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2012 manfred.neugebauer@siemens.com
 */

#include <linux/kernel.h>
#include <linux/sched.h>
#include <asm/traps.h>
#include <asm/uaccess.h>

// workaround: will be set/reset while accessing pci configuration range
// to prevent segmentation faults
int pci_test_enabled;

int soc_be_handler(struct pt_regs *regs, int is_fixup)
{

	int data = regs->cp0_cause & 4;

	printk("soc1 bus error handler called (fix=%d)\n", is_fixup);

	if (is_fixup)
		return MIPS_BE_FIXUP;

	printk("Got %cbe at 0x%lx\n", data ? 'd' : 'i', regs->cp0_epc);

	if (pci_test_enabled) {
		// m.n. for now take next instruction
		regs->cp0_epc += 4;
		return(MIPS_BE_DISCARD);
	}

#if (0)
	show_regs(regs);
	dump_tlb_all();
	while(1);
	force_sig(SIGBUS, current);
#endif

	return MIPS_BE_FATAL;
}

void __init soc_be_init(void)
{
	board_be_handler = soc_be_handler;
}
