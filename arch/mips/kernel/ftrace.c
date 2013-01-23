/*
 * this file provides the interfaces to handle ftrace syscall interface
 * for mips architecture.
 *
 * Copyright (C) 2012 manfred.neugebauer@siemens.com
 * 
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */


#include <linux/spinlock.h>
#include <linux/hardirq.h>
#include <linux/uaccess.h>
#include <linux/ftrace.h>
#include <linux/percpu.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/list.h>

#include <trace/syscall.h>
#include <asm/ftrace.h>

#ifdef CONFIG_FTRACE_SYSCALLS

extern unsigned long __start_syscalls_metadata[];
extern unsigned long __stop_syscalls_metadata[];
extern unsigned long *sys_call_table;

static struct syscall_metadata **syscalls_metadata;

static struct syscall_metadata *find_syscall_meta(unsigned long *syscall)
{
	struct syscall_metadata *start;
	struct syscall_metadata *stop;
	char str[KSYM_SYMBOL_LEN];


	start = (struct syscall_metadata *)__start_syscalls_metadata;
	stop = (struct syscall_metadata *)__stop_syscalls_metadata;
	kallsyms_lookup((unsigned long) syscall, NULL, NULL, NULL, str);

	for ( ; start < stop; start++) {
		if (start->name && !strcmp(start->name, str))
			return start;
	}
	return NULL;
}

struct syscall_metadata *syscall_nr_to_meta(int nr)
{ 
	nr -= 4000; // mips: we have a special start number
	if (!syscalls_metadata || nr >= FTRACE_SYSCALL_MAX || nr < 0)
		return NULL;

	return syscalls_metadata[nr];
}

void arch_init_ftrace_syscalls(void)
{
	int i;
	struct syscall_metadata *meta;
	unsigned long **psys_syscall_table = &sys_call_table;
	static atomic_t refs;

	if (atomic_inc_return(&refs) != 1)
		goto end;

	syscalls_metadata = kzalloc(sizeof(*syscalls_metadata) *
					FTRACE_SYSCALL_MAX, GFP_KERNEL);
	if (!syscalls_metadata) {
		WARN_ON(1);
		return;
	}

	for (i = 0; i < FTRACE_SYSCALL_MAX; i++) {
		meta = find_syscall_meta(psys_syscall_table[2 * i]);
		syscalls_metadata[i] = meta;
	}

	return;

	/* Paranoid: avoid overflow */
end:
	atomic_dec(&refs);
}

#endif
