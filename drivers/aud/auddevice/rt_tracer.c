/* -*- linux-c -*-
 * drivers/aud/auddevice/rt_tracer.c
 *
 * Copyright (C) 2005 Luotao Fu.
 *               2005-2008 Jan Kiszka.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, Inc., 675 Mass Ave, Cambridge MA 02139,
 * USA; either version 2 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 * Copyright (C) 2010 Wolfgang Hartmann
 *
 * This file is based on the adeos-ipipe patch (kernel/ipipe/tracer.c).
 * It has been adapted to a dual-domain environment (Linux/RT-kernel).
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/kallsyms.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/ctype.h>
#include <linux/vmalloc.h>
#include <linux/pid.h>
#include <linux/utsrelease.h>
#include <linux/sched.h>
#include <linux/ftrace.h>
#include <asm/uaccess.h>

#include <linux/aud/rt_base.h>
#include <asm/rtx_timer.h>

#ifndef BROKEN_BUILTIN_RETURN_ADDRESS
#define __BUILTIN_RETURN_ADDRESS0 ((unsigned long)__builtin_return_address(0))
#define __BUILTIN_RETURN_ADDRESS1 ((unsigned long)__builtin_return_address(1))
#endif /* !BUILTIN_RETURN_ADDRESS */

#define RT_TRACE_PATHS           4 /* <!> Do not lower below 3 */
#define RT_DEFAULT_ACTIVE        0
#define RT_DEFAULT_MAX           1
#define RT_DEFAULT_FROZEN        2

#define RT_TRACE_POINTS          (1 << CONFIG_RTX_TRACE_SHIFT)
#define WRAP_POINT_NO(point)     ((point) & (RT_TRACE_POINTS-1))

#define RT_DEFAULT_PRE_TRACE     10
#define RT_DEFAULT_POST_TRACE    10
#define RT_DEFAULT_BACK_TRACE    100

#define RT_DELAY_NOTE            1000  /* in nanoseconds */
#define RT_DELAY_WARN            10000 /* in nanoseconds */

#define RT_TFLG_NMI_LOCK         0x0001
#define RT_TFLG_NMI_HIT          0x0002
#define RT_TFLG_NMI_FREEZE_REQ   0x0004

#define RT_TFLG_HWIRQ_OFF        0x0100
#define RT_TFLG_FREEZING         0x0200
#define RT_TFLG_CURRDOM_SHIFT    10   /* bits 10..11: current domain */
#define RT_TFLG_CURRDOM_MASK     0x0C00
#define RT_TFLG_DOMSTATE_SHIFT   12   /* bit 12: LX domain stalled? 13..15 unused */
#define RT_TFLG_DOMSTATE_BITS    1

#define RT_TFLG_DOMAIN_STALLED(point, n) \
	(point->flags & (1 << (n + RT_TFLG_DOMSTATE_SHIFT)))
#define RT_TFLG_CURRENT_DOMAIN(point) \
	((point->flags & RT_TFLG_CURRDOM_MASK) >> RT_TFLG_CURRDOM_SHIFT)

struct rt_trace_point {
	short type;
	short flags;
	unsigned long eip;
	unsigned long parent_eip;
	unsigned long v;
	unsigned long long timestamp;
};

struct rt_trace_path {
	volatile int flags;
	int dump_lock; /* separated from flags due to cross-cpu access */
	int trace_pos; /* next point to fill */
	int begin, end; /* finalised path begin and end */
	int post_trace; /* non-zero when in post-trace phase */
	unsigned long long length; /* max path length in cycles */
	unsigned long nmi_saved_eip; /* for deferred requests from NMIs */
	unsigned long nmi_saved_parent_eip;
	unsigned long nmi_saved_v;
	struct rt_trace_point point[RT_TRACE_POINTS];
} ____cacheline_aligned_in_smp;

enum rt_trace_type
{
	RT_TRACE_FUNC = 0,
	RT_TRACE_BEGIN,
	RT_TRACE_END,
	RT_TRACE_FREEZE,
	RT_TRACE_SPECIAL,
	RT_TRACE_PID,
	RT_TRACE_EVENT,
};

#define RT_TYPE_MASK             0x0007
#define RT_TYPE_BITS             3

#ifdef CONFIG_RTX_TRACE_VMALLOC
static DEFINE_PER_CPU(struct rt_trace_path *, trace_path);
#else /* !CONFIG_RTX_TRACE_VMALLOC */
static DEFINE_PER_CPU(struct rt_trace_path, trace_path[RT_TRACE_PATHS]) =
	{ [0 ... RT_TRACE_PATHS-1] = { .begin = -1, .end = -1 } };
#endif /* CONFIG_RTX_TRACE_VMALLOC */

int rt_trace_enable = 0;

static DEFINE_PER_CPU(int, active_path) = { RT_DEFAULT_ACTIVE };
static DEFINE_PER_CPU(int, max_path) = { RT_DEFAULT_MAX };
static DEFINE_PER_CPU(int, frozen_path) = { RT_DEFAULT_FROZEN };
static RTX_DEFINE_SPINLOCK(global_path_lock);
static int pre_trace = RT_DEFAULT_PRE_TRACE;
static int post_trace = RT_DEFAULT_POST_TRACE;
static int back_trace = RT_DEFAULT_BACK_TRACE;
static int verbose_trace = 1;
static unsigned long trace_overhead;

static unsigned long trigger_begin;
static unsigned long trigger_end;

static DEFINE_MUTEX(out_mutex);
static struct rt_trace_path *print_path;
#ifdef CONFIG_RTX_TRACE_PANIC
static struct rt_trace_path *panic_path;
#endif /* CONFIG_RTX_TRACE_PANIC */
static int print_pre_trace;
static int print_post_trace;


static long __rt_signed_tsc2us(long long tsc);
static void __rt_trace_point_type(char *buf, struct rt_trace_point *point);
static void __rt_print_symname(struct seq_file *m, unsigned long eip);

extern void __rtx_spin_unlock_irqbegin(rtx_spinlock_t *lock);
extern void __rtx_spin_unlock_irqcomplete(unsigned long x);

#ifdef CONFIG_PROC_FS
struct proc_dir_entry *rt_proc_root;
#endif

static notrace void
__rt_store_domain_states(struct rt_trace_point *point)
{
	struct rtx_pcd *pcd = &__raw_get_cpu_var(rtx_percpu_data);

	/* Set the stalled bit marker for the LX-domain. */
	if (test_bit(LX_IRQS_STALLED, &pcd->rtx_irq_control))
		point->flags |= 1 << (0 + RT_TFLG_DOMSTATE_SHIFT);
	/* Set the domain marker for the RT-domain (LX-domain ist default). */
	if (pcd->rtx_curr_dom == RT_DOMAIN) {
		point->flags |= 1 << RT_TFLG_CURRDOM_SHIFT;
	}
}

static notrace int __rt_get_free_trace_path(int old, int cpu)
{
	int new_active = old;
	struct rt_trace_path *tp;

	do {
		if (++new_active == RT_TRACE_PATHS)
			new_active = 0;
		tp = &per_cpu(trace_path, cpu)[new_active];
	} while (new_active == per_cpu(max_path, cpu) ||
	         new_active == per_cpu(frozen_path, cpu) ||
	         tp->dump_lock);

	return new_active;
}

static notrace void
__rt_migrate_pre_trace(struct rt_trace_path *new_tp,
                          struct rt_trace_path *old_tp, int old_pos)
{
	int i;

	new_tp->trace_pos = pre_trace+1;

	for (i = new_tp->trace_pos; i > 0; i--)
		memcpy(&new_tp->point[WRAP_POINT_NO(new_tp->trace_pos-i)],
		       &old_tp->point[WRAP_POINT_NO(old_pos-i)],
		       sizeof(struct rt_trace_point));

	/* mark the end (i.e. the point before point[0]) invalid */
	new_tp->point[RT_TRACE_POINTS-1].eip = 0;
}

static notrace struct rt_trace_path *
__rt_trace_end(int cpu, struct rt_trace_path *tp, int pos)
{
	struct rt_trace_path *old_tp = tp;
	long active = per_cpu(active_path, cpu);
	unsigned long long length;

	/* do we have a new worst case? */
	length = tp->point[tp->end].timestamp -
	         tp->point[tp->begin].timestamp;
	if (length > per_cpu(trace_path, cpu)[per_cpu(max_path, cpu)].length) {
		/* we need protection here against other cpus trying
		   to start a proc dump */
		rtx_spin_lock(&global_path_lock);

		/* active path holds new worst case */
		tp->length = length;
		per_cpu(max_path, cpu) = active;

		/* find next unused trace path */
		active = __rt_get_free_trace_path(active, cpu);

		rtx_spin_unlock(&global_path_lock);

		tp = &per_cpu(trace_path, cpu)[active];

		/* migrate last entries for pre-tracing */
		__rt_migrate_pre_trace(tp, old_tp, pos);
	}

	return tp;
}

static notrace struct rt_trace_path *
__rt_trace_freeze(int cpu, struct rt_trace_path *tp, int pos)
{
	struct rt_trace_path *old_tp = tp;
	long active = per_cpu(active_path, cpu);
	int n;

	/* frozen paths have no core (begin=end) */
	tp->begin = tp->end;

	/* we need protection here against other cpus trying
	 * to set their frozen path or to start a proc dump */
	rtx_spin_lock(&global_path_lock);

	per_cpu(frozen_path, cpu) = active;

	/* find next unused trace path */
	active = __rt_get_free_trace_path(active, cpu);

	/* check if this is the first frozen path */
	for_each_possible_cpu(n) {
		if (n != cpu &&
		    per_cpu(trace_path, n)[per_cpu(frozen_path, n)].end >= 0)
			tp->end = -1;
	}

	rtx_spin_unlock(&global_path_lock);

	tp = &per_cpu(trace_path, cpu)[active];

	/* migrate last entries for pre-tracing */
	__rt_migrate_pre_trace(tp, old_tp, pos);

	return tp;
}

void notrace
__rt_trace(enum rt_trace_type type, unsigned long eip,
              unsigned long parent_eip, unsigned long v)
{
	struct rt_trace_path *tp, *old_tp;
	int pos, next_pos, begin;
	struct rt_trace_point *point;
	unsigned long flags;
	int cpu;

	local_irq_save_hw_notrace(flags);

	cpu = rtx_processor_id();
 restart:
	tp = old_tp = &per_cpu(trace_path, cpu)[per_cpu(active_path, cpu)];

	/* here starts a race window with NMIs - catched below */

	/* check for NMI recursion */
	if (unlikely(tp->flags & RT_TFLG_NMI_LOCK)) {
		tp->flags |= RT_TFLG_NMI_HIT;

		/* first freeze request from NMI context? */
		if ((type == RT_TRACE_FREEZE) &&
		    !(tp->flags & RT_TFLG_NMI_FREEZE_REQ)) {
			/* save arguments and mark deferred freezing */
			tp->flags |= RT_TFLG_NMI_FREEZE_REQ;
			tp->nmi_saved_eip = eip;
			tp->nmi_saved_parent_eip = parent_eip;
			tp->nmi_saved_v = v;
		}
		return; /* no need for restoring flags inside IRQ */
	}

	/* clear NMI events and set lock (atomically per cpu) */
	tp->flags = (tp->flags & ~(RT_TFLG_NMI_HIT |
	                           RT_TFLG_NMI_FREEZE_REQ))
	                       | RT_TFLG_NMI_LOCK;

	/* check active_path again - some nasty NMI may have switched
	 * it meanwhile */
	if (unlikely(tp !=
		     &per_cpu(trace_path, cpu)[per_cpu(active_path, cpu)])) {
		/* release lock on wrong path and restart */
		tp->flags &= ~RT_TFLG_NMI_LOCK;

		/* there is no chance that the NMI got deferred
		 * => no need to check for pending freeze requests */
		goto restart;
	}

	/* get the point buffer */
	pos = tp->trace_pos;
	point = &tp->point[pos];

	/* store all trace point data */
	point->type = type;
	point->flags = raw_irqs_disabled_flags(flags) ? RT_TFLG_HWIRQ_OFF : 0;
	point->eip = eip;
	point->parent_eip = parent_eip;
	point->v = v;
	point->timestamp = rtx_gettime();

	__rt_store_domain_states(point);

	/* forward to next point buffer */
	next_pos = WRAP_POINT_NO(pos+1);
	tp->trace_pos = next_pos;

	/* only mark beginning if we haven't started yet */
	begin = tp->begin;
	if (unlikely(type == RT_TRACE_BEGIN) && (begin < 0))
		tp->begin = pos;

	/* end of critical path, start post-trace if not already started */
	if (unlikely(type == RT_TRACE_END) &&
	    (begin >= 0) && !tp->post_trace)
		tp->post_trace = post_trace + 1;

	/* freeze only if the slot is free and we are not already freezing */
	if ((unlikely(type == RT_TRACE_FREEZE) ||
	     (unlikely(eip >= trigger_begin && eip <= trigger_end) &&
	     type == RT_TRACE_FUNC)) &&
	    per_cpu(trace_path, cpu)[per_cpu(frozen_path, cpu)].begin < 0 &&
	    !(tp->flags & RT_TFLG_FREEZING)) {
		tp->post_trace = post_trace + 1;
		tp->flags |= RT_TFLG_FREEZING;
	}

	/* enforce end of trace in case of overflow */
	if (unlikely(WRAP_POINT_NO(next_pos + 1) == begin)) {
		tp->end = pos;
		goto enforce_end;
	}

	/* stop tracing this path if we are in post-trace and
	 *  a) that phase is over now or
	 *  b) a new TRACE_BEGIN came in but we are not freezing this path */
	if (unlikely((tp->post_trace > 0) && ((--tp->post_trace == 0) ||
	             ((type == RT_TRACE_BEGIN) &&
	              !(tp->flags & RT_TFLG_FREEZING))))) {
		/* store the path's end (i.e. excluding post-trace) */
		tp->end = WRAP_POINT_NO(pos - post_trace + tp->post_trace);

 enforce_end:
		if (tp->flags & RT_TFLG_FREEZING)
			tp = __rt_trace_freeze(cpu, tp, pos);
		else
			tp = __rt_trace_end(cpu, tp, pos);

		/* reset the active path, maybe already start a new one */
		tp->begin = (type == RT_TRACE_BEGIN) ?
			WRAP_POINT_NO(tp->trace_pos - 1) : -1;
		tp->end = -1;
		tp->post_trace = 0;
		tp->flags = 0;

		/* update active_path not earlier to avoid races with NMIs */
		per_cpu(active_path, cpu) = tp - per_cpu(trace_path, cpu);
	}

	/* we still have old_tp and point,
	 * let's reset NMI lock and check for catches */
	old_tp->flags &= ~RT_TFLG_NMI_LOCK;
	if (unlikely(old_tp->flags & RT_TFLG_NMI_HIT)) {
		/* well, this late tagging may not immediately be visible for
		 * other cpus already dumping this path - a minor issue */
		point->flags |= RT_TFLG_NMI_HIT;

		/* handle deferred freezing from NMI context */
		if (old_tp->flags & RT_TFLG_NMI_FREEZE_REQ)
			__rt_trace(RT_TRACE_FREEZE, old_tp->nmi_saved_eip,
			              old_tp->nmi_saved_parent_eip,
			              old_tp->nmi_saved_v);
	}

	local_irq_restore_hw_notrace(flags);
}

static unsigned long __rt_global_path_lock(void)
{
	unsigned long flags;
	int cpu;
	struct rt_trace_path *tp;

	rtx_spin_lock_irqsave(&global_path_lock, flags);

	cpu = rtx_processor_id();
 restart:
	tp = &per_cpu(trace_path, cpu)[per_cpu(active_path, cpu)];

	/* here is small race window with NMIs - catched below */

	/* clear NMI events and set lock (atomically per cpu) */
	tp->flags = (tp->flags & ~(RT_TFLG_NMI_HIT |
	                           RT_TFLG_NMI_FREEZE_REQ))
	                       | RT_TFLG_NMI_LOCK;

	/* check active_path again - some nasty NMI may have switched
	 * it meanwhile */
	if (tp != &per_cpu(trace_path, cpu)[per_cpu(active_path, cpu)]) {
		/* release lock on wrong path and restart */
		tp->flags &= ~RT_TFLG_NMI_LOCK;

		/* there is no chance that the NMI got deferred
		 * => no need to check for pending freeze requests */
		goto restart;
	}

	return flags;
}

static void __rt_global_path_unlock(unsigned long flags)
{
	int cpu;
	struct rt_trace_path *tp;

	/* release spinlock first - it's not involved in the NMI issue */
	__rtx_spin_unlock_irqbegin(&global_path_lock);

	cpu = rtx_processor_id();
	tp = &per_cpu(trace_path, cpu)[per_cpu(active_path, cpu)];

	tp->flags &= ~RT_TFLG_NMI_LOCK;

	/* handle deferred freezing from NMI context */
	if (tp->flags & RT_TFLG_NMI_FREEZE_REQ)
		__rt_trace(RT_TRACE_FREEZE, tp->nmi_saved_eip,
		              tp->nmi_saved_parent_eip, tp->nmi_saved_v);

	/* See __rtx_spin_lock_irqsave() and friends. */
	__rtx_spin_unlock_irqcomplete(flags);
}

void notrace rt_trace_begin(unsigned long v)
{
	if (!rt_trace_enable)
		return;
	__rt_trace(RT_TRACE_BEGIN, __BUILTIN_RETURN_ADDRESS0,
	              __BUILTIN_RETURN_ADDRESS1, v);
}
EXPORT_SYMBOL(rt_trace_begin);

void notrace rt_trace_end(unsigned long v)
{
	if (!rt_trace_enable)
		return;
	__rt_trace(RT_TRACE_END, __BUILTIN_RETURN_ADDRESS0,
	              __BUILTIN_RETURN_ADDRESS1, v);
}
EXPORT_SYMBOL(rt_trace_end);

void notrace rt_trace_freeze(unsigned long v)
{
	if (!rt_trace_enable)
		return;
	__rt_trace(RT_TRACE_FREEZE, __BUILTIN_RETURN_ADDRESS0,
	              __BUILTIN_RETURN_ADDRESS1, v);
}
EXPORT_SYMBOL(rt_trace_freeze);

void notrace rt_trace_special(unsigned char id, unsigned long v)
{
	if (!rt_trace_enable)
		return;
	__rt_trace(RT_TRACE_SPECIAL | (id << RT_TYPE_BITS),
	              __BUILTIN_RETURN_ADDRESS0,
	              __BUILTIN_RETURN_ADDRESS1, v);
}
EXPORT_SYMBOL(rt_trace_special);

void notrace rt_trace_pid(pid_t pid, short prio)
{
	if (!rt_trace_enable)
		return;
	__rt_trace(RT_TRACE_PID | (prio << RT_TYPE_BITS),
	              __BUILTIN_RETURN_ADDRESS0,
	              __BUILTIN_RETURN_ADDRESS1, pid);
}
EXPORT_SYMBOL(rt_trace_pid);

void notrace rt_trace_event(unsigned char id, unsigned long delay_tsc)
{
	if (!rt_trace_enable)
		return;
	__rt_trace(RT_TRACE_EVENT | (id << RT_TYPE_BITS),
	              __BUILTIN_RETURN_ADDRESS0,
	              __BUILTIN_RETURN_ADDRESS1, delay_tsc);
}
EXPORT_SYMBOL(rt_trace_event);

int rt_trace_max_reset(void)
{
	int cpu;
	unsigned long flags;
	struct rt_trace_path *path;
	int ret = 0;

	flags = __rt_global_path_lock();

	for_each_possible_cpu(cpu) {
		path = &per_cpu(trace_path, cpu)[per_cpu(max_path, cpu)];

		if (path->dump_lock) {
			ret = -EBUSY;
			break;
		}

		path->begin     = -1;
		path->end       = -1;
		path->trace_pos = 0;
		path->length    = 0;
	}

	__rt_global_path_unlock(flags);

	return ret;
}
EXPORT_SYMBOL(rt_trace_max_reset);

int rt_trace_frozen_reset(void)
{
	int cpu;
	unsigned long flags;
	struct rt_trace_path *path;
	int ret = 0;

	flags = __rt_global_path_lock();

	for_each_online_cpu(cpu) {
		path = &per_cpu(trace_path, cpu)[per_cpu(frozen_path, cpu)];

		if (path->dump_lock) {
			ret = -EBUSY;
			break;
		}

		path->begin = -1;
		path->end = -1;
		path->trace_pos = 0;
		path->length    = 0;
	}

	__rt_global_path_unlock(flags);

	return ret;
}
EXPORT_SYMBOL(rt_trace_frozen_reset);

static void
__rt_get_task_info(char *task_info, struct rt_trace_point *point,
                      int trylock)
{
	struct task_struct *task = NULL;
	char buf[8];
	int i;
	int locked = 1;

	if (trylock) {
		if (!read_trylock(&tasklist_lock))
			locked = 0;
	} else
		read_lock(&tasklist_lock);

	if (locked)
		task = find_task_by_pid_ns((pid_t)point->v, &init_pid_ns);

	if (task)
		strncpy(task_info, task->comm, 11);
	else
		strcpy(task_info, "-<?>-");

	if (locked)
		read_unlock(&tasklist_lock);

	for (i = strlen(task_info); i < 11; i++)
		task_info[i] = ' ';

	sprintf(buf, " %d ", point->type >> RT_TYPE_BITS);
	strcpy(task_info + (11 - strlen(buf)), buf);
}

static void
__rt_get_event_date(char *buf,struct rt_trace_path *path,
		       struct rt_trace_point *point)
{
	long time;
	int type;

	time = __rt_signed_tsc2us(point->timestamp -
				     path->point[path->begin].timestamp + point->v);
	type = point->type >> RT_TYPE_BITS;

	if (type == 0)
		/*
		 * Event type #0 is predefined, stands for the next
		 * timer tick.
		 */
		sprintf(buf, "tick@%-6ld", time);
	else
		sprintf(buf, "%3d@%-7ld", type, time);
}

#ifdef CONFIG_RTX_TRACE_PANIC
void rt_trace_panic_freeze(void)
{
	unsigned long flags;
	int cpu;

	if (!rt_trace_enable)
		return;

	rt_trace_enable = 0;
	local_irq_save_hw_notrace(flags);

	cpu = rtx_processor_id();

	panic_path = &per_cpu(trace_path, cpu)[per_cpu(active_path, cpu)];

	local_irq_restore_hw(flags);
}
EXPORT_SYMBOL(rt_trace_panic_freeze);

void rt_trace_panic_dump(void)
{
	int cnt = back_trace;
	int start, pos;
	char buf[16];

	if (!panic_path)
		return;


	printk("Audis tracer log (%d points):\n", cnt);

	start = pos = WRAP_POINT_NO(panic_path->trace_pos-1);

	while (cnt-- > 0) {
		struct rt_trace_point *point = &panic_path->point[pos];
		long time;
		char info[16];
		int i;

		printk(" %c",
		       (point->flags & RT_TFLG_HWIRQ_OFF) ? '|' : ' ');

		for (i = RT_TFLG_DOMSTATE_BITS; i >= 0; i--)
			printk("%c",
			       (RT_TFLG_CURRENT_DOMAIN(point) == i) ?
				(RT_TFLG_DOMAIN_STALLED(point, i) ?
					'#' : '+') :
				(RT_TFLG_DOMAIN_STALLED(point, i) ?
					'*' : ' '));

		if (!point->eip)
			printk("-<invalid>-\n");
		else {
			__rt_trace_point_type(buf, point);
			printk("%s", buf);

			switch (point->type & RT_TYPE_MASK) {
				case RT_TRACE_FUNC:
					printk("           ");
					break;

				case RT_TRACE_PID:
					__rt_get_task_info(info,
							      point, 1);
					printk("%s", info);
					break;

				case RT_TRACE_EVENT:
					__rt_get_event_date(info,
							       panic_path, point);
					printk("%s", info);
					break;

				default:
					printk("0x%08lx ", point->v);
			}

			time = __rt_signed_tsc2us(point->timestamp -
				panic_path->point[start].timestamp);
			printk(" %5ld ", time);

			__rt_print_symname(NULL, point->eip);
			printk(" (");
			__rt_print_symname(NULL, point->parent_eip);
			printk(")\n");
		}
		pos = WRAP_POINT_NO(pos - 1);
	}

	panic_path = NULL;
}
EXPORT_SYMBOL(rt_trace_panic_dump);
#endif /* CONFIG_RTX_TRACE_PANIC */


/* --- /proc output --- */

static notrace int __rt_in_critical_trpath(long point_no)
{
	return ((WRAP_POINT_NO(point_no-print_path->begin) <
	         WRAP_POINT_NO(print_path->end-print_path->begin)) ||
	        ((print_path->end == print_path->begin) &&
	         (WRAP_POINT_NO(point_no-print_path->end) >
	          print_post_trace)));
}

static long __rt_signed_tsc2us(long long tsc)
{
        unsigned long long abs_tsc;
        long us;

	/* rtx_tsc2us works on unsigned => handle sign separately */
        abs_tsc = (tsc >= 0) ? tsc : -tsc;
        us = rtx_tsc2us(abs_tsc);
        if (tsc < 0)
                return -us;
        else
                return us;
}

static void
__rt_trace_point_type(char *buf, struct rt_trace_point *point)
{
	switch (point->type & RT_TYPE_MASK) {
		case RT_TRACE_FUNC:
			strcpy(buf, "func    ");
			break;

		case RT_TRACE_BEGIN:
			strcpy(buf, "begin   ");
			break;

		case RT_TRACE_END:
			strcpy(buf, "end     ");
			break;

		case RT_TRACE_FREEZE:
			strcpy(buf, "freeze  ");
			break;

		case RT_TRACE_SPECIAL:
			sprintf(buf, "(0x%02x)  ",
				point->type >> RT_TYPE_BITS);
			break;

		case RT_TRACE_PID:
			sprintf(buf, "[%5d] ", (pid_t)point->v);
			break;

		case RT_TRACE_EVENT:
			sprintf(buf, "event   ");
			break;
	}
}

static void
__rt_print_pathmark(struct seq_file *m, struct rt_trace_point *point)
{
	char mark = ' ';
	int point_no = point - print_path->point;
	int i;

	if (print_path->end == point_no)
		mark = '<';
	else if (print_path->begin == point_no)
		mark = '>';
	else if (__rt_in_critical_trpath(point_no))
		mark = ':';
	seq_printf(m, "%c%c", mark,
	           (point->flags & RT_TFLG_HWIRQ_OFF) ? '|' : ' ');

	if (!verbose_trace)
		return;

	for (i = RT_TFLG_DOMSTATE_BITS; i >= 0; i--)
		seq_printf(m, "%c",
			(RT_TFLG_CURRENT_DOMAIN(point) == i) ?
			    (RT_TFLG_DOMAIN_STALLED(point, i) ?
				'#' : '+') :
			(RT_TFLG_DOMAIN_STALLED(point, i) ? '*' : ' '));
}

static void
__rt_print_delay(struct seq_file *m, struct rt_trace_point *point)
{
	unsigned long delay = 0;
	int next;
	char *mark = "  ";

	next = WRAP_POINT_NO(point+1 - print_path->point);

	if (next != print_path->trace_pos)
		delay = rtx_tsc2ns(print_path->point[next].timestamp -
		                     point->timestamp);

	if (__rt_in_critical_trpath(point - print_path->point)) {
		if (delay > RT_DELAY_WARN)
			mark = "! ";
		else if (delay > RT_DELAY_NOTE)
			mark = "+ ";
	}
	seq_puts(m, mark);

	if (verbose_trace)
		seq_printf(m, "%3lu.%03lu%c ", delay/1000, delay%1000,
		           (point->flags & RT_TFLG_NMI_HIT) ? 'N' : ' ');
	else
		seq_puts(m, " ");
}

static void __rt_print_symname(struct seq_file *m, unsigned long eip)
{
	char namebuf[KSYM_NAME_LEN+1];
	unsigned long size, offset;
	const char *sym_name;
	char *modname;

	sym_name = kallsyms_lookup(eip, &size, &offset, &modname, namebuf);

#ifdef CONFIG_RTX_TRACE_PANIC
	if (!m) {
		/* panic dump */
		if (sym_name) {
			printk("%s+0x%lx", sym_name, offset);
			if (modname)
				printk(" [%s]", modname);
		}
	} else
#endif /* CONFIG_RTX_TRACE_PANIC */
	{
		if (sym_name) {
			if (verbose_trace) {
				seq_printf(m, "%s+0x%lx", sym_name, offset);
				if (modname)
					seq_printf(m, " [%s]", modname);
			} else
				seq_puts(m, sym_name);
		} else
			seq_printf(m, "<%08lx>", eip);
	}
}

static void __rt_print_headline(struct seq_file *m)
{
	seq_printf(m, "Calibrated minimum trace-point overhead: %lu.%03lu "
		   "us\n\n", trace_overhead/1000, trace_overhead%1000);

	if (verbose_trace) {
		const char *name[2] = { [0 ... 1] = "<unused>" };

		name[0] = "LX-domain";
		name[1] = "RT-domain";

		seq_printf(m,
		           " +----- Hard IRQs ('|': locked)\n"
		           " |+---- %s\n"
		           " ||+--- %s%s\n"
		           " |||                        +---------- "
		               "Delay flag ('+': > %d us, '!': > %d us)\n"
		           " |||                        |        +- "
		               "NMI noise ('N')\n"
		           " |||                        |        |\n"
		           "    Type    User Val.   Time    Delay  Function "
		               "(Parent)\n",
		           name[1], name[0],
		           name[0] ? " ('*': domain stalled, '+': current, "
		               "'#': current+stalled)" : "",
		           RT_DELAY_NOTE/1000, RT_DELAY_WARN/1000);
	} else
		seq_printf(m,
		           " +--------------- Hard IRQs ('|': locked)\n"
		           " |             +- Delay flag "
		               "('+': > %d us, '!': > %d us)\n"
		           " |             |\n"
		           "  Type     Time   Function (Parent)\n",
		           RT_DELAY_NOTE/1000, RT_DELAY_WARN/1000);
}

static void *__rt_max_prtrace_start(struct seq_file *m, loff_t *pos)
{
	loff_t n = *pos;

	mutex_lock(&out_mutex);

	if (!n) {
		struct rt_trace_path *tp;
		unsigned long length_usecs;
		int points, cpu;
		unsigned long flags;

		/* protect against max_path/frozen_path updates while we
		 * haven't locked our target path, also avoid recursively
		 * taking global_path_lock from NMI context */
		flags = __rt_global_path_lock();

		/* find the longest of all per-cpu paths */
		print_path = NULL;
		for_each_online_cpu(cpu) {
			tp = &per_cpu(trace_path, cpu)[per_cpu(max_path, cpu)];
			if ((print_path == NULL) ||
			    (tp->length > print_path->length)) {
				print_path = tp;
				break;
			}
		}
		print_path->dump_lock = 1;

		__rt_global_path_unlock(flags);

		/* does this path actually contain data? */
		if (print_path->end == print_path->begin)
			return NULL;

		/* number of points inside the critical path */
		points = WRAP_POINT_NO(print_path->end-print_path->begin+1);

		/* pre- and post-tracing length, post-trace length was frozen
		   in __rt_trace, pre-trace may have to be reduced due to
		   buffer overrun */
		print_pre_trace  = pre_trace;
		print_post_trace = WRAP_POINT_NO(print_path->trace_pos -
		                                 print_path->end - 1);
		if (points+pre_trace+print_post_trace > RT_TRACE_POINTS - 1)
			print_pre_trace = RT_TRACE_POINTS - 1 - points -
				print_post_trace;

		length_usecs = rtx_tsc2us(print_path->length);

		seq_printf(m, "Audis worst-case tracing service on %s/audis-%s\n"
			"------------------------------------------------------------\n",
			UTS_RELEASE, RT_VERSION_STRING);
		seq_printf(m, "CPU: %d, Begin: %lld cycles, Trace Points: "
			"%d (-%d/+%d), Length: %lu us\n",
			cpu, print_path->point[print_path->begin].timestamp,
			points, print_pre_trace, print_post_trace, length_usecs);
		__rt_print_headline(m);
	}

	/* check if we are inside the trace range */
	if (n >= WRAP_POINT_NO(print_path->end - print_path->begin + 1 +
	                       print_pre_trace + print_post_trace))
		return NULL;

	/* return the next point to be shown */
	return &print_path->point[WRAP_POINT_NO(print_path->begin -
	                                        print_pre_trace + n)];
}

static void *__rt_prtrace_next(struct seq_file *m, void *p, loff_t *pos)
{
	loff_t n = ++*pos;

	/* check if we are inside the trace range with the next entry */
	if (n >= WRAP_POINT_NO(print_path->end - print_path->begin + 1 +
	                       print_pre_trace + print_post_trace))
		return NULL;

	/* return the next point to be shown */
	return &print_path->point[WRAP_POINT_NO(print_path->begin -
	                                        print_pre_trace + *pos)];
}

static void __rt_prtrace_stop(struct seq_file *m, void *p)
{
	if (print_path)
		print_path->dump_lock = 0;
	mutex_unlock(&out_mutex);
}

static int __rt_prtrace_show(struct seq_file *m, void *p)
{
	long time;
	struct rt_trace_point *point = p;
	char buf[16];

	if (!point->eip) {
		seq_puts(m, "-<invalid>-\n");
		return 0;
	}

	__rt_print_pathmark(m, point);
	__rt_trace_point_type(buf, point);
	seq_puts(m, buf);
	if (verbose_trace)
		switch (point->type & RT_TYPE_MASK) {
			case RT_TRACE_FUNC:
				seq_puts(m, "           ");
				break;

			case RT_TRACE_PID:
				__rt_get_task_info(buf, point, 0);
				seq_puts(m, buf);
				break;

			case RT_TRACE_EVENT:
				__rt_get_event_date(buf, print_path, point);
				seq_puts(m, buf);
				break;

			default:
				seq_printf(m, "0x%08lx ", point->v);
		}

	time = __rt_signed_tsc2us(point->timestamp -
		print_path->point[print_path->begin].timestamp);
	seq_printf(m, "%5ld", time);

	__rt_print_delay(m, point);
	__rt_print_symname(m, point->eip);
	seq_puts(m, " (");
	__rt_print_symname(m, point->parent_eip);
	seq_puts(m, ")\n");

	return 0;
}

static struct seq_operations __rt_max_ptrace_ops = {
	.start = __rt_max_prtrace_start,
	.next  = __rt_prtrace_next,
	.stop  = __rt_prtrace_stop,
	.show  = __rt_prtrace_show
};

static int __rt_max_prtrace_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &__rt_max_ptrace_ops);
}

static ssize_t
__rt_max_reset(struct file *file, const char __user *pbuffer,
                  size_t count, loff_t *data)
{
	mutex_lock(&out_mutex);
	rt_trace_max_reset();
	mutex_unlock(&out_mutex);

	return count;
}

struct file_operations __rt_max_prtrace_fops = {
	.open       = __rt_max_prtrace_open,
	.read       = seq_read,
	.write      = __rt_max_reset,
	.llseek     = seq_lseek,
	.release    = seq_release,
};

static void *__rt_frozen_prtrace_start(struct seq_file *m, loff_t *pos)
{
	loff_t n = *pos;

	mutex_lock(&out_mutex);

	if (!n) {
		struct rt_trace_path *tp;
		int cpu;
		unsigned long flags;

		/* protect against max_path/frozen_path updates while we
		 * haven't locked our target path, also avoid recursively
		 * taking global_path_lock from NMI context */
		flags = __rt_global_path_lock();

		/* find the first of all per-cpu frozen paths */
		print_path = NULL;
		for_each_online_cpu(cpu) {
			tp = &per_cpu(trace_path, cpu)[per_cpu(frozen_path, cpu)];
			if (tp->end >= 0) {
				print_path = tp;
				break;
			}
		}
		if (print_path)
			print_path->dump_lock = 1;

		__rt_global_path_unlock(flags);

		if (!print_path)
			return NULL;

		/* back- and post-tracing length, post-trace length was frozen
		   in __rt_trace, back-trace may have to be reduced due to
		   buffer overrun */
		print_pre_trace  = back_trace-1; /* substract freeze point */
		print_post_trace = WRAP_POINT_NO(print_path->trace_pos -
		                                 print_path->end - 1);
		if (1+pre_trace+print_post_trace > RT_TRACE_POINTS - 1)
			print_pre_trace = RT_TRACE_POINTS - 2 -
				print_post_trace;

		seq_printf(m, "Audis frozen back-tracing service on %s/audis-%s\n"
			"------------------------------------------------------"
			"------\n",
			UTS_RELEASE, RT_VERSION_STRING);
		seq_printf(m, "CPU: %d, Freeze: %lld cycles, Trace Points: %d (+%d)\n",
			cpu, print_path->point[print_path->begin].timestamp,
			print_pre_trace+1, print_post_trace);
		__rt_print_headline(m);
	}

	/* check if we are inside the trace range */
	if (n >= print_pre_trace + 1 + print_post_trace)
		return NULL;

	/* return the next point to be shown */
	return &print_path->point[WRAP_POINT_NO(print_path->begin-
	                                        print_pre_trace+n)];
}

static struct seq_operations __rt_frozen_ptrace_ops = {
	.start = __rt_frozen_prtrace_start,
	.next  = __rt_prtrace_next,
	.stop  = __rt_prtrace_stop,
	.show  = __rt_prtrace_show
};

static int __rt_frozen_prtrace_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &__rt_frozen_ptrace_ops);
}

static ssize_t
__rt_frozen_ctrl(struct file *file, const char __user *pbuffer,
                    size_t count, loff_t *data)
{
	char *end, buf[16];
	int val;
	int n;

	n = (count > sizeof(buf) - 1) ? sizeof(buf) - 1 : count;

	if (copy_from_user(buf, pbuffer, n))
		return -EFAULT;

	buf[n] = '\0';
	val = simple_strtol(buf, &end, 0);

	if (((*end != '\0') && !isspace(*end)) || (val < 0))
		return -EINVAL;

	mutex_lock(&out_mutex);
	rt_trace_frozen_reset();
	if (val > 0)
		rt_trace_freeze(-1);
	mutex_unlock(&out_mutex);

	return count;
}

struct file_operations __rt_frozen_prtrace_fops = {
	.open       = __rt_frozen_prtrace_open,
	.read       = seq_read,
	.write      = __rt_frozen_ctrl,
	.llseek     = seq_lseek,
	.release    = seq_release,
};

static int __rt_rd_proc_val(char *page, char **start, off_t off,
                               int count, int *eof, void *data)
{
	int len;

	len = sprintf(page, "%u\n", *(int *)data);
	len -= off;
	if (len <= off + count)
		*eof = 1;
	*start = page + off;
	if (len > count)
		len = count;
	if (len < 0)
		len = 0;

	return len;
}

static int __rt_wr_proc_val(struct file *file, const char __user *buffer,
                               unsigned long count, void *data)
{
	char *end, buf[16];
	int val;
	int n;

	n = (count > sizeof(buf) - 1) ? sizeof(buf) - 1 : count;

	if (copy_from_user(buf, buffer, n))
		return -EFAULT;

	buf[n] = '\0';
	val = simple_strtol(buf, &end, 0);

	if (((*end != '\0') && !isspace(*end)) || (val < 0))
		return -EINVAL;

	mutex_lock(&out_mutex);
	*(int *)data = val;
	mutex_unlock(&out_mutex);

	return count;
}

static int __rt_rd_trigger(char *page, char **start, off_t off, int count,
			      int *eof, void *data)
{
	int len;

	if (!trigger_begin)
		return 0;

	len = sprint_symbol(page, trigger_begin);
	page[len++] = '\n';

	len -= off;
	if (len <= off + count)
		*eof = 1;
	*start = page + off;
	if (len > count)
		len = count;
	if (len < 0)
		len = 0;

	return len;
}

static int __rt_wr_trigger(struct file *file, const char __user *buffer,
			      unsigned long count, void *data)
{
	char buf[KSYM_SYMBOL_LEN];
	unsigned long begin, end;

	if (count > sizeof(buf) - 1)
		count = sizeof(buf) - 1;
	if (copy_from_user(buf, buffer, count))
		return -EFAULT;
	buf[count] = 0;
	if (buf[count-1] == '\n')
		buf[count-1] = 0;

	/* Reset trigger value. */
	if (count == 1 && buf[0] == 0) {
		mutex_lock(&out_mutex);
		/* invalidate the current range before setting a new one */
		trigger_end = 0;
		wmb();
		/* reset new range */
		trigger_begin = 0;
		mutex_unlock(&out_mutex);
		return count;
	}

	begin = kallsyms_lookup_name(buf);
	if (!begin || !kallsyms_lookup_size_offset(begin, &end, NULL))
		return -ENOENT;
	end += begin - 1;

	mutex_lock(&out_mutex);
	/* invalidate the current range before setting a new one */
	trigger_end = 0;
	wmb();
	rt_trace_frozen_reset();

	/* set new range */
	trigger_begin = begin;
	wmb();
	trigger_end = end;
	mutex_unlock(&out_mutex);

	return count;
}

#ifdef CONFIG_RTX_TRACE_MCOUNT
static void notrace
rt_trace_function(unsigned long ip, unsigned long parent_ip)
{
	if (!rt_trace_enable)
		return;
	__rt_trace(RT_TRACE_FUNC, ip, parent_ip, 0);
}

static struct ftrace_ops rt_trace_ops = {
	.func = rt_trace_function,
	.rtx_safe = true
};

static int __rt_wr_enable(struct file *file, const char __user *buffer,
			     unsigned long count, void *data)
{
	char *end, buf[16];
	int val;
	int n;

	n = (count > sizeof(buf) - 1) ? sizeof(buf) - 1 : count;

	if (copy_from_user(buf, buffer, n))
		return -EFAULT;

	buf[n] = '\0';
	val = simple_strtol(buf, &end, 0);

	if (((*end != '\0') && !isspace(*end)) || (val < 0))
		return -EINVAL;

	mutex_lock(&out_mutex);

	if (rt_trace_enable) {
		if (!val)
			unregister_ftrace_function(&rt_trace_ops);
	} else if (val)
		register_ftrace_function(&rt_trace_ops);

	rt_trace_enable = val;

	mutex_unlock(&out_mutex);

	return count;
}
#endif /* CONFIG_RTX_TRACE_MCOUNT */

static struct proc_dir_entry * __init
__rt_create_trace_proc_val(struct proc_dir_entry *trace_dir,
                              const char *name, int *value_ptr)
{
	struct proc_dir_entry *entry;

	entry = create_proc_entry(name, 0644, trace_dir);
	if (entry) {
		entry->data = value_ptr;
		entry->read_proc = __rt_rd_proc_val;
		entry->write_proc = __rt_wr_proc_val;
	}
	return entry;
}

void __init __rt_init_tracer(void)
{
	struct proc_dir_entry *trace_dir;
	struct proc_dir_entry *entry;
	unsigned long long start, end, min = ULLONG_MAX;
	int i;
#ifdef CONFIG_RTX_TRACE_VMALLOC
	int cpu, path;

	for_each_possible_cpu(cpu) {
		struct rt_trace_path *tp_buf;

		tp_buf = vmalloc_node(sizeof(struct rt_trace_path) *
				      RT_TRACE_PATHS, cpu_to_node(cpu));
		if (!tp_buf) {
			printk(KERN_ERR "Audis: "
			       "insufficient memory for trace buffer.\n");
			return;
		}
		memset(tp_buf, 0,
		       sizeof(struct rt_trace_path) * RT_TRACE_PATHS);
		for (path = 0; path < RT_TRACE_PATHS; path++) {
			tp_buf[path].begin = -1;
			tp_buf[path].end   = -1;
		}
		per_cpu(trace_path, cpu) = tp_buf;
	}
#endif /* CONFIG_RTX_TRACE_VMALLOC */

	/* Calculate minimum overhead of __rt_trace() */
	local_irq_disable_hw();
	for (i = 0; i < 100; i++) {
		start = rtx_gettime();
		__rt_trace(RT_TRACE_FUNC, __BUILTIN_RETURN_ADDRESS0,
			      __BUILTIN_RETURN_ADDRESS1, 0);
		end = rtx_gettime();

		end -= start;
		if (end < min)
			min = end;
	}
	local_irq_enable_hw();
	trace_overhead = rtx_tsc2ns(min);

#ifdef CONFIG_RTX_TRACE_ENABLE
	rt_trace_enable = 1;
#ifdef CONFIG_RTX_TRACE_MCOUNT
	register_ftrace_function(&rt_trace_ops);
#endif /* CONFIG_RTX_TRACE_MCOUNT */
#endif /* CONFIG_RTX_TRACE_ENABLE */

	trace_dir = create_proc_entry("trace", S_IFDIR, rt_proc_root);

	entry = create_proc_entry("max", 0644, trace_dir);
	if (entry)
		entry->proc_fops = &__rt_max_prtrace_fops;

	entry = create_proc_entry("frozen", 0644, trace_dir);
	if (entry)
		entry->proc_fops = &__rt_frozen_prtrace_fops;

	entry = create_proc_entry("trigger", 0644, trace_dir);
	if (entry) {
		entry->read_proc = __rt_rd_trigger;
		entry->write_proc = __rt_wr_trigger;
	}

	__rt_create_trace_proc_val(trace_dir, "pre_trace_points",
	                              &pre_trace);
	__rt_create_trace_proc_val(trace_dir, "post_trace_points",
	                              &post_trace);
	__rt_create_trace_proc_val(trace_dir, "back_trace_points",
	                              &back_trace);
	__rt_create_trace_proc_val(trace_dir, "verbose",
	                              &verbose_trace);
	entry = __rt_create_trace_proc_val(trace_dir, "enable",
					      &rt_trace_enable);
#ifdef CONFIG_RTX_TRACE_MCOUNT
	if (entry)
		entry->write_proc = __rt_wr_enable;
#endif /* CONFIG_RTX_TRACE_MCOUNT */
}

#ifdef CONFIG_PROC_FS
static int __rt_version_info_proc(char *page,
				     char **start,
				     off_t off, int count, int *eof, void *data)
{
	int len = sprintf(page, "%s\n", RT_VERSION_STRING);

	len -= off;

	if (len <= off + count)
		*eof = 1;

	*start = page + off;

	if(len > count)
		len = count;

	if(len < 0)
		len = 0;

	return len;
}


void __init rtx_init_proc(void)
{
	rt_proc_root = create_proc_entry("audis",S_IFDIR, 0);
	create_proc_read_entry("version",0444,rt_proc_root,&__rt_version_info_proc,NULL);

	__rt_init_tracer();
}

#endif	/* CONFIG_PROC_FS */

/* Common trace function, called from ioctl. */
int notrace rt_trace_common(unsigned int cmd, trace_descr_t *trace_descr)
{
	int err = 0;

	switch (cmd) {
    case AuD_TRACE_MAX_BEGIN:
    	rt_trace_begin(trace_descr->l_val);
    	return 0;
    case AuD_TRACE_MAX_END:
    	rt_trace_end(trace_descr->l_val);
    	return 0;
    case AuD_TRACE_MAX_RESET:
    	return rt_trace_max_reset();
    case AuD_TRACE_USER_START:
    	return rt_trace_frozen_reset();
    case AuD_TRACE_USER_STOP:
    	rt_trace_freeze(trace_descr->l_val);
    	return 0;
    case AuD_TRACE_USER_FREEZE:
    	if (!trace_descr->once)
    		err = rt_trace_frozen_reset();
    	rt_trace_freeze(trace_descr->l_val);
    	return err;
    case AuD_TRACE_SPECIAL:
    	rt_trace_special(trace_descr->id, trace_descr->l_val);
    	return 0;
    case AuD_TRACE_SPECIAL_U64:
    	rt_trace_special(trace_descr->id, (unsigned long)(trace_descr->ll_val >> 32));
       	rt_trace_special(trace_descr->id, (unsigned long)(trace_descr->ll_val & 0xFFFFFFFF));
    	return 0;
	}
	return -ENOSYS;
}
EXPORT_SYMBOL(rt_trace_common);

