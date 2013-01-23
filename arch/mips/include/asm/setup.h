#ifndef _MIPS_SETUP_H
#define _MIPS_SETUP_H

#ifdef CONFIG_AUD_EXTEND_COMMAND_LINE
#define COMMAND_LINE_SIZE	CONFIG_AUD_EXTEND_COMMAND_LINE
#else
#define COMMAND_LINE_SIZE	512
#endif

#ifdef  __KERNEL__
extern void setup_early_printk(void);
#endif /* __KERNEL__ */

#endif /* __SETUP_H */
