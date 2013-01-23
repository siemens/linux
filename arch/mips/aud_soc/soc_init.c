/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2011 Manfred.Neugebauer@siemens.com
 */

#include <linux/kernel.h>
#include <linux/module.h>

#include <asm/bootinfo.h>

void prom_meminit(void);
int aud_soc_init_uarts(void);
#ifdef CONFIG_SPI
int aud_soc_init_gpio(void);
int aud_soc_init_spi(void);
#endif

/* may be used for a default (PROM) command line */
void  __init prom_init_cmdline(void)
{
	int boot_argc;
	unsigned long *boot_argv, *boot_envp;
	char *cp;
	int actr;

	boot_argc = fw_arg0;
	boot_argv = (unsigned long *) fw_arg1;
	boot_envp = (unsigned long *) fw_arg2;

	actr = 1; /* Always ignore argv[0] */

	cp = arcs_cmdline;

	while (actr < boot_argc) {
		strcpy(cp, (unsigned char *)boot_argv[actr]);
		cp += strlen((unsigned char *)boot_argv[actr]);
		*cp++ = ' ';

		actr++;
	}

	if (cp != arcs_cmdline)		/* get rid of trailing space */
		--cp;
	*cp = '\0';

#ifdef DEBUG_CMDLINE
	printk(KERN_DEBUG "init cmdline: %s\n", arcs_cmdline);
#endif
}

void __init prom_init(void)
{
	printk("AuD Soc prom_init\n");

	prom_init_cmdline();
	prom_meminit();

	aud_soc_init_uarts();

#ifdef CONFIG_SPI
        aud_soc_init_gpio();
	aud_soc_init_spi();
#endif
#ifdef CONFIG_SERIAL_8250_CONSOLExxxx
	console_config();
#endif

}


