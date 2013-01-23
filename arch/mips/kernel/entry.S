/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 1994 - 2000, 2001, 2003 Ralf Baechle
 * Copyright (C) 1999, 2000 Silicon Graphics, Inc.
 * Copyright (C) 2001 MIPS Technologies, Inc.
 */

#include <asm/asm.h>
#include <asm/asmmacro.h>
#include <asm/regdef.h>
#include <asm/mipsregs.h>
#include <asm/stackframe.h>
#include <asm/isadep.h>
#include <asm/thread_info.h>
#include <asm/war.h>
#ifdef CONFIG_MIPS_MT_SMTC
#include <asm/mipsmtregs.h>
#endif

#ifdef CONFIG_RTX_DOMAIN
#include <linux/aud/rt_sched.h>
#endif

#ifndef CONFIG_PREEMPT
#define resume_kernel	restore_all
#else
#ifndef CONFIG_RTX_DOMAIN
#define __ret_from_irq	ret_from_exception
#endif
#endif

	.text
	.align	5
#ifdef CONFIG_RTX_DOMAIN
FEXPORT(ret_from_irq)
	LONG_S	s0, TI_REGS($28)
	beqz v0, restore_all			# if zero, take fast return path
	j  rtx_slow_path				#skip down to common code

FEXPORT(ret_from_exception)
	// Check whether the exception was handled in RT (return
	// fast path) or not (return slow path).
	lw	a2, TI_TASK($28)	# current->task ref
	lw	a2, TS_rt_state(a2)
	li	t0, RT_TASK_RUNNING
	and	t0, a2
	beqz	t0, rtx_slow_path
	nop
	j	restore_all
	nop
rtx_slow_path:
	LONG_L	t0, PT_STATUS(sp)		# returning to kernel mode?
	andi	t0, t0, KU_USER
	beqz	t0, resume_kernel
#else
#ifndef CONFIG_PREEMPT
FEXPORT(ret_from_exception)
	local_irq_disable			# preempt stop
	b	__ret_from_irq
#endif
FEXPORT(ret_from_irq)
	LONG_S	s0, TI_REGS($28)
FEXPORT(__ret_from_irq)
	LONG_L	t0, PT_STATUS(sp)		# returning to kernel mode?
	andi	t0, t0, KU_USER
	beqz	t0, resume_kernel
#endif

resume_userspace:
#ifdef CONFIG_RTX_DOMAIN
	jal	__rtx_local_irq_disable
	nop
#else
	local_irq_disable		# make sure we dont miss an
					# interrupt setting need_resched
					# between sampling and return
#endif
	LONG_L	a2, TI_FLAGS($28)	# current->work
	andi	t0, a2, _TIF_WORK_MASK	# (ignoring syscall_trace)
	bnez	t0, work_pending
#ifdef CONFIG_RTX_DOMAIN
	lw	a2, TI_TASK($28)	# current->task ref
	lw	t2, TS_rt_state(a2)
        # the test for LX_MIGRATE_TO_RT was at ret_from_fork before.
        # we moved the test for LX_MIGRATE_TO_RT to here
        # since we need to allow Linux to test for (debugging) signals
        # when a new thread is created
	li	t0, LX_MIGRATE_TO_RT | LXRT_TASK_LEAVE_PENDING
	and	t0, t2				# migration sequence?
	bnez	t0, 902f
	lw	t2, TS_rt_state3(a2)
	li	t0, RT_TASK_LX_DEBUG_PENDING
	and	t0, t2				# debugging sequence?
	beqz	t0, 901f
        nop
902:
        la	t2, rtx_migrate_to_rt	# migrate back to RT
	jalr	t2
	nop
	// return value is in v0: 
	// debugging: we may be moved back to lx
        beqz     v0, resume_userspace
901:
	jal	__rtx_local_irq_enable
	nop
#endif
	j	restore_all

#ifdef CONFIG_PREEMPT
resume_kernel:
#ifdef CONFIG_RTX_DOMAIN
	jal	__rtx_local_irq_disable
	nop
	lw	t0, TI_PRE_COUNT($28)
	bnez	t0, restore_all_rtx
need_resched:
	LONG_L	t0, TI_FLAGS($28)
	andi	t1, t0, _TIF_NEED_RESCHED
	beqz	t1, restore_all_rtx
	nop
	LONG_L	t0, PT_STATUS(sp)		# Interrupts off?
	andi	t0, 1
	beqz	t0, restore_all_rtx
	nop
	jal	preempt_schedule_irq
	nop
	b	need_resched
	nop
#else
	local_irq_disable
	lw	t0, TI_PRE_COUNT($28)
	bnez	t0, restore_all
need_resched:
	LONG_L	t0, TI_FLAGS($28)
	andi	t1, t0, _TIF_NEED_RESCHED
	beqz	t1, restore_all
	LONG_L	t0, PT_STATUS(sp)		# Interrupts off?
	andi	t0, 1
	beqz	t0, restore_all
	jal	preempt_schedule_irq
	b	need_resched
#endif
#endif

#ifdef CONFIG_RTX_DOMAIN
        restore_all_rtx:
	jal	__rtx_local_irq_enable
	nop
	b restore_all
	nop
#endif

FEXPORT(ret_from_fork)
	jal	schedule_tail		# a0 = struct task_struct *prev
#ifdef CONFIG_RTX_DOMAIN
	# realtime threads: check for migration to RT
        # we have two situations:
        # (a) debugging and others: we allow linux to handle these events
        #     before we migrate to rt (in this case the migration will
        #     be done later as part of resume_userspace
        # (b) no work bits are set: migrate immediately 
	//local_irq_disable		# make sure need_resched and
					# signals dont change between
					# sampling and return
	LONG_L	a2, TI_FLAGS($28)	# current->work
	li	t0, _TIF_SIGPENDING     # did we catch debug signals?
	and	t0, a2, t0
	bnez	t0, 905f
	lw	a2, TI_TASK($28)	# current->task ref
	lw	a2, TS_rt_state(a2)
	li	t0, LXRT_TASK_LEAVE_PENDING	# clone of a realtime thread?
	and	t0, a2
	beqz	t0, 905f
	//jal	schedule		# doesn't seem to be necessary
	nop
        li      a0, 0
	la	t2, rtx_migrate_to_rt	# migrate to RT
	jalr	t2
	nop
	// return value is in v0: 0 == LX, 1 == RT
	beqz	v0, 905f
        li      v0, 0
        b	restore_all
	nop
905:
#endif


FEXPORT(syscall_exit)
#ifdef CONFIG_RTX_DOMAIN
	jal	__rtx_local_irq_disable
	nop
#else
	local_irq_disable		# make sure need_resched and
					# signals dont change between
					# sampling and return
#endif
	LONG_L	a2, TI_FLAGS($28)	# current->work
	li	t0, _TIF_ALLWORK_MASK
	and	t0, a2, t0
	bnez	t0, syscall_exit_work

FEXPORT(restore_all)			# restore full frame
#ifdef CONFIG_MIPS_MT_SMTC
#ifdef CONFIG_MIPS_MT_SMTC_IM_BACKSTOP
/* Re-arm any temporarily masked interrupts not explicitly "acked" */
	mfc0	v0, CP0_TCSTATUS
	ori	v1, v0, TCSTATUS_IXMT
	mtc0	v1, CP0_TCSTATUS
	andi	v0, TCSTATUS_IXMT
	_ehb
	mfc0	t0, CP0_TCCONTEXT
	DMT	9				# dmt t1
	jal	mips_ihb
	mfc0	t2, CP0_STATUS
	andi	t3, t0, 0xff00
	or	t2, t2, t3
	mtc0	t2, CP0_STATUS
	_ehb
	andi	t1, t1, VPECONTROL_TE
	beqz	t1, 1f
	EMT
1:
	mfc0	v1, CP0_TCSTATUS
	/* We set IXMT above, XOR should clear it here */
	xori	v1, v1, TCSTATUS_IXMT
	or	v1, v0, v1
	mtc0	v1, CP0_TCSTATUS
	_ehb
	xor	t0, t0, t3
	mtc0	t0, CP0_TCCONTEXT
#endif /* CONFIG_MIPS_MT_SMTC_IM_BACKSTOP */
/* Detect and execute deferred IPI "interrupts" */
	LONG_L	s0, TI_REGS($28)
	LONG_S	sp, TI_REGS($28)
	jal	deferred_smtc_ipi
	LONG_S	s0, TI_REGS($28)
#endif /* CONFIG_MIPS_MT_SMTC */
	.set	noat
	RESTORE_TEMP
	RESTORE_AT
	RESTORE_STATIC
FEXPORT(restore_partial)		# restore partial frame
#ifdef CONFIG_TRACE_IRQFLAGS
	SAVE_STATIC
	SAVE_AT
	SAVE_TEMP
	LONG_L	v0, PT_STATUS(sp)
#if defined(CONFIG_CPU_R3000) || defined(CONFIG_CPU_TX39XX)
	and	v0, ST0_IEP
#else
	and	v0, ST0_IE
#endif
	beqz	v0, 1f
	jal	trace_hardirqs_on
	b	2f
1:	jal	trace_hardirqs_off
2:
	RESTORE_TEMP
	RESTORE_AT
	RESTORE_STATIC
#endif
	RESTORE_SOME
	RESTORE_SP_AND_RET
	.set	at

work_pending:
	andi	t0, a2, _TIF_NEED_RESCHED # a2 is preloaded with TI_FLAGS
	beqz	t0, work_notifysig
work_resched:
	jal	schedule

#ifdef CONFIG_RTX_DOMAIN
	jal	__rtx_local_irq_disable
	nop
#else
	local_irq_disable		# make sure need_resched and
					# signals dont change between
					# sampling and return
#endif
	LONG_L	a2, TI_FLAGS($28)
	andi	t0, a2, _TIF_WORK_MASK	# is there any work to be done
					# other than syscall tracing?
#ifdef CONFIG_RTX_DOMAIN
	beqz	t0, restore_all_rtx
#else
	beqz	t0, restore_all
#endif
	andi	t0, a2, _TIF_NEED_RESCHED
	bnez	t0, work_resched

work_notifysig:				# deal with pending signals and
					# notify-resume requests
	move	a0, sp
	li	a1, 0
	jal	do_notify_resume	# a2 already loaded
	j	resume_userspace

FEXPORT(syscall_exit_work_partial)
	SAVE_STATIC
syscall_exit_work:
	li	t0,  _TIF_SYSCALL_TRACE | _TIF_SYSCALL_FTRACE | _TIF_SYSCALL_AUDIT | _TIF_KERNEL_TRACE
	and	t0, a2			# a2 is preloaded with TI_FLAGS
	beqz	t0, work_pending	# trace bit set?
#ifdef CONFIG_RTX_DOMAIN
	jal	__rtx_local_irq_enable
	nop
#else
	local_irq_enable		# could let do_syscall_trace()
					# call schedule() instead
#endif
#ifdef CONFIG_RTX_DOMAIN
	// we do it earlier - see scall32-o32.S
#else
	move	a0, sp
	li	a1, 1
	jal	do_syscall_trace
#endif
	b	resume_userspace

#if defined(CONFIG_CPU_MIPSR2) || defined(CONFIG_MIPS_MT)

/*
 * MIPS32R2 Instruction Hazard Barrier - must be called
 *
 * For C code use the inline version named instruction_hazard().
 */
LEAF(mips_ihb)
	.set	mips32r2
	jr.hb	ra
	nop
	END(mips_ihb)

#endif /* CONFIG_CPU_MIPSR2 or CONFIG_MIPS_MT */
