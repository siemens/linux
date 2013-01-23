/*
 * Access to user system call parameters and results
 *
 * Copyright (C) 2012 manfred.neugebauer@siemens.com
 *
 * This copyrighted material is made available to anyone wishing to use,
 * modify, copy, or redistribute it subject to the terms and conditions
 * of the GNU General Public License v.2.
 *
 * All of these functions expect to be called with no locks,
 * and only when the caller is sure that the task of interest
 * cannot return to user mode while we are looking at it.
 *
 * See asm-generic/syscall.h for descriptions of what we must do here.
 */

#ifndef _ASM_MIPS_SYSCALL_H
#define _ASM_MIPS_SYSCALL_H

/**
 * syscall_get_nr - find what system call a task is executing
 * @task:	task of interest, must be blocked
 * @regs:	task_pt_regs() of @task
 *
 * If @task is executing a system call or is at system call
 * tracing about to attempt one, returns the system call number.
 * If @task is not executing a system call, i.e. it's blocked
 * inside the kernel for a fault or signal, returns -1.
 *
 * It's only valid to call this when @task is known to be blocked.
 */
static inline long syscall_get_nr(struct task_struct *task,
				  struct pt_regs *regs)
{
	/*
	 * We always sign-extend a -1 value being set here,
	 * so this is always either -1L or a syscall number.
	 */
	return regs->regs[2];
}

/**
 * syscall_get_return_value - get the return value of a traced system call
 * @task:	task of interest, must be blocked
 * @regs:	task_pt_regs() of @task
 *
 * Returns the return value of the successful system call.
 * This value is meaningless if syscall_get_error() returned nonzero.
 *
 * It's only valid to call this when @task is stopped for tracing on exit
 * from a system call, due to %TIF_SYSCALL_TRACE or %TIF_SYSCALL_AUDIT.
 */
static inline long syscall_get_return_value(struct task_struct *task,
					    struct pt_regs *regs)
{
	/*
	 * mips: system call id and return value uses the same register (v0)
	 * and stack memory; we overload a0 when calling syscall exit tracing
	 * to get the return value
	 */
	return regs->regs[4];
}

/**
 * syscall_get_arguments - extract system call parameter values
 * @task:	task of interest, must be blocked
 * @regs:	task_pt_regs() of @task
 * @i:		argument index [0,5]
 * @n:		number of arguments; n+i must be [1,6].
 * @args:	array filled with argument values
 *
 * Fetches @n arguments to the system call starting with the @i'th argument
 * (from 0 through 5).  Argument @i is stored in @args[0], and so on.
 * An arch inline version is probably optimal when @i and @n are constants.
 *
 * It's only valid to call this when @task is stopped for tracing on
 * entry to a system call, due to %TIF_SYSCALL_TRACE or %TIF_SYSCALL_AUDIT.
 * It's invalid to call this with @i + @n > 6; we only support system calls
 * taking up to 6 arguments.
 */
static inline void syscall_get_arguments(struct task_struct *task,
					 struct pt_regs *regs,
					 unsigned int i, unsigned int n,
					 unsigned long *args)
{
	BUG_ON(i + n > 6);
	memcpy(args, &regs->regs[4] + i, n * sizeof(args[0]));
}


#endif	/* _ASM_MIPS_SYSCALL_H */
