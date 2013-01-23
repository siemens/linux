/*
 * linux/aud/rt_driver.h
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
 * Timer specific definitions and common functions.
 */
#ifndef _LINUX_AUD_RT_DRIVER_H
#define _LINUX_AUD_RT_DRIVER_H
#include <linux/aud/rt_internal.h>
#include <linux/aud/rt_internal2.h>
#include <linux/rtx_system.h>

/*
 * All definition needed when writing a dual domain driver
 */
#define	RT_IO_READ				1
#define	RT_IO_WRITE				2
#define	RT_IO_IOCTL				4
#define	RT_IO_ALLOWED			(RT_IO_READ | RT_IO_WRITE | RT_IO_IOCTL)

int rt_allow_access(struct file *, int);
void rt_remove_access(struct file *);					// only when shutting down the realtime process

int rt_request_irq(unsigned int, irqreturn_t (*)(int, void *), unsigned long, const char *, void *, irqreturn_t (*)(int, void *));
int execute_nonrt_handler(int, int);
void rt_free_irq(int, void *);
int rt_enable_irq_affinity(int);
int rt_disable_irq_affinity(int);

/*
 * All definitions needed for a driver which needs to do IO-event signalling
 */
#define EV_EMPTY		0
#define EV_BUSY			1
#define	EV_REGISTERED	2

#define RT_EVAR_MAX		(RT_MAX_CPUS + 8)	// number of event areas
#define CARRIER_WAITS	1
#define MASTER_EVENT	2
#define SLAVE_EVENT		4

struct notify	{
	int ev_notify;						// either counter for signal sending (Linux propagation to thread level)
										// or 0x1: carrier waits, 0x2: master event, 0x4: slave event for ioctl(..WAIT..)
	int cycle;							// for timeliness control
	int sigev_notify;					// notify word of sigevent struct
	struct rt_event *ev_master;			// reference to rt_event element the carrier thread refers to
	struct task_struct *thread;
	struct task_struct *alt_thread;		// monitor thread the signal has to be sent to
	struct sigqueue sigq;
	int shared;							// used by send_sigqueue
	struct pid *ppid;					// used by send_sigqueue
};
struct rt_event {
	eventid_t ev_id;					// visible for device driver
	int (*ev_disable) (void *, struct rt_event*);			// action for disabling the event source
	int (*ev_enable) (void *, struct rt_event*);			// action for enabling the event source	
	void *endisable_par;				// first parameter for both calls
	atomic_t state;						// busy, registered, empty
	struct rt_evar_hdr *hdpl;			// pointer to header in evar_list
	int (*ev_action) (void *);
	struct list_head ev_link;			// in case additional rt_event elements share the same carrier
	struct notify rt_notify;
};


struct rt_evar_hdr {
	int	ev_no;							// no of elements in the area
	int no_of_proxy_user;				// for killing a proxy
	int sum_cnt;						// sum of all ev_cnt (for optimization)
	struct task_struct *ev_proxy;		// proxy thread to be notified when signalling should be done from LINUX ISR
	struct semaphore proxy_sem;
	struct semaphore sync_sem;
	struct rt_event *evpr;				// pointer to first element of a rt_event_list
};

/*
 * All definitions needed for a driver which does event signalling
 */
int rt_init_event_area (struct rt_event *, int);
int rt_register_event(int, struct rt_ev_desc *);
int rt_send_event(struct rt_event *);
int rt_destroy_event_area(int);
int rt_wait_for_event(void);


/*
 * All definitions needed for a driver which needs to register a sync clock
 */
#define CLOCK_SYNC_HARD			1
#define CLOCK_SYNC_TIMER		2

int rt_register_sync_clock(struct file*, struct rt_clock_desc *, int, void(**)(void));
int rt_unregister_sync_clock(struct file*, clockid_t);

int printkGen(char *pidstr, char *format, ...);

#include <linux/aud/rt_sched.h>
#endif /* _LINUX_AUD_RT_DRIVER_H */


