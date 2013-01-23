/*
 * linux/aud/rt_link.h
 *
 * 2007-19-01:  Karl-Heinz.Krause@siemens.com
 * Copyright (c) 2007 Siemens AG
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
 */
#ifndef _LINUX_AUD_RT_LINK_H
#define _LINUX_AUD_RT_LINK_H

// Request types for the interdomain daemon
#define	EXEC_FUTEX_LX		1			// a futex operation must get executed by the daemon
#define GET_LX_TOD			2			// do a do_gettimeofday for keeping the CLOCK_REALTIME synchromnized
#define MIGRATE_TO_RT		3
#define FUNCTION_IN_LX		4

struct reg_proc {
		struct list_head p_link;
		unsigned long tgid;
};

struct type_info {						// used for specifying the message in the write buffer
	int fd;
	char *prefix;
	int pref_len;
};

struct lx_cmd {							// command structure for delegation requests to Linux
		struct list_head req_link;
		int		type;					// so far EXEC_FUTEX_LX and CHECK_SETUP are defined
		union {
				struct task_struct *_proc;
				struct fu {
					unsigned long uaddr;
					int op;
					int	val;
					unsigned long uaddr2;
					int	val2;
					int val3;
				}_fuop;
#ifdef CONFIG_LTT
				struct fct {
					void *functionPtr;
					unsigned par1;
					unsigned par2;
					unsigned par3;
				}_fctop;
#endif
		}_req;		
};

struct lx_cmd* get_lx_request(void);

struct dom_hdr  { 
		struct semaphore dev_sem;			// for serialization of auddevice functions
#ifdef CONFIG_LTT
		struct task_struct *ltt_daemon;		//FIXME
#endif
		struct list_head free_list;			// free list header for futex wake operations to be delegated
		struct list_head reg_list;			// threads which registered with RT_SOFT only will be queued temporarily
											// reason: mlockall() doesn't survive an exec()
};

DECLARE_PER_CPU(struct semaphore, to_rt_op);
DECLARE_PER_CPU(struct rt_semaphore, rt_tl_sem);
DECLARE_PER_CPU(unsigned long, rt_system_state);
DECLARE_PER_CPU(struct task_struct *, rt_timer_daemon);
DECLARE_PER_CPU(int, td_error);
DECLARE_PER_CPU(_INT64, rt_new_offset);
DECLARE_PER_CPU(long, mn_rt_state);
DECLARE_PER_CPU(int, cpu_received_sig);
DECLARE_PER_CPU(int, sig_to_deliver);
DECLARE_PER_CPU(struct sync_param, rt_sync_migration);
DECLARE_PER_CPU(struct itimer, rr_timer);
DECLARE_PER_CPU(migqueue_t, rt_migqueue);
DECLARE_PER_CPU(runqueue_t, rt_runqueue);
DECLARE_PER_CPU(runqueue_t *, prt_runqueue);

extern struct semaphore scl_sem;

extern int rt_sched_return_to_lx(int);
extern int rt_tgroup_common(unsigned int, tgroup_descr_t *);
extern void rt_tgroup_init(void);
extern void rt_tgroup_destroy(void);
extern int rt_trace_common(unsigned int, trace_descr_t *);

extern struct rt_proc_hdr rt_proc[];
extern struct dom_hdr aud_rt_hdr;
extern unsigned long aud_diagnostic;
extern struct rt_link_desc *rtlpr;
extern struct rtg_clock *rt_clock_arr[];
extern struct rt_config_cntrl aud_config;
struct rt_proc_hdr *get_rt_proc(int);
int rt_is_running(int);

void rt_emulate_lx_tick(_INT64, int (*)(void *));
void rt_release_emulated_lx_tick(void);
void rt_set_policy(int policy, struct task_struct *p);
asmlinkage long execute_in_lx(struct pt_regs);
void wake_up_interdomain_daemon(void);

int rt_send_sigqueue(struct notify *);
int rt_send_alt_sigqueue(struct notify *);
void lx_wake_up_rt_thread(struct task_struct *);
void lx_wake_up_rt_thread_id(struct task_struct *);
int rt_wake_up_thread(void *);
int rt_wake_up_thread_id(void *);
int add_interdomain_request(void*);
struct task_struct *rt_schedule(int);
void setup_internal_timer(struct rt_list_hdr *, struct itimer *, _INT64, int (*)(void *));
int lx_notify_thread(void *);
int rt_dequeue_sigq(struct sigqueue *); 
struct task_struct * rt_find_task_by_pid(int); 	
_INT64 rt_schedule_timeout(struct rt_list_hdr *, _INT64); 
void rt_enter_schedule(void);
int rt_task_leave(void);	
void rt_mark_exit_pending(struct task_struct *);	
int rt_timer_daemon_thread(void*);
int rt_check_daemon_thread(void*);
int interdomain_daemon_thread(void*);
#ifdef CONFIG_LTT
int ltt_enable_daemon(void *arg);
#endif
int rt_init_sigqueue_pool(int );
void rt_free_sigqueue_pool(void);
int setup_rr_timer(struct task_struct *);
int dequeue_timer(struct itimer *);
int enqueue_timer(struct itimer *);
void write_out_buffer(void);
int rt_write_buffered(int flag, struct type_info *, void *, unsigned);
void write_out_messages(void);
int setup_clock_periodic_timer(void);

void rt_delete_event_notification(struct task_struct *);
void rt_delete_timer_notification(struct task_struct *);
void clock_sync_delete_timer_notification(struct task_struct *);
int check_sync_timer_notification(struct task_struct *);
int rt_event_notification(struct task_struct *);
void init_rt_notify(struct notify *, int);

int probe_realtime(struct probe_descr *);
void do_pinning(void);
int init_timer_management(int, int);
void rtx_init_isr_management(void);
void init_io_management(void);
int rt_init_realtime(struct probe_descr *);

struct task_struct *rt_check_sigevent(int, pid_t, int);			
int rt_setup_notification(void **, int,  struct sigevent *, struct notify *, int); 
void init_rt_list(struct rt_list_hdr *);
struct posix_timer *get_tmr(void);
void put_tmr(struct posix_timer *tmr);
void rt_sched_rr(void);
void rt_init_sched(void);
void rt_init_futex(void);
void rt_init_futex_queue(void);
void rt_init_nfb_pool(void);
int sched_setscheduler(struct task_struct*, int, struct sched_param *);
int init_write_service(struct probe_descr *);
long copy_conv_to_internal(long long *, struct timespec __user *);
_INT64 delete_internal_timer(struct itimer *);
struct itimer *get_timer_elapsed(struct rt_list_hdr *);
struct itimer *get_sync_timer_elapsed(struct rt_list_hdr *);
int rt_reschedule(struct task_struct *);
int rt_notification(struct task_struct *);
void rtx_reenter_lx(struct task_struct *);
void rtx_adjust_rq_cnt(void);
int setup_aud_rt_domain(void);
void destroy_aud_rt_domain(void);

void rt_sched_rr_delay(void);
int rt_sched_has_delay(void);
void rt_dequeue_migration_thread(migqueue_t *, struct task_struct *);

#ifndef CONFIG_RTX_DOMAIN_HRT
unsigned long clock_getres_int(int);
#endif
struct rt_list_hdr *get_hdr(void);

#ifdef CONFIG_SMP
int rtx_pin_irq(int, cpumask_t);
int rtx_unpin_irq(int);
#endif

#endif /* _LINUX_AUD_RT_LINK_H_ */

