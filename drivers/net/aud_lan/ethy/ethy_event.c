/*
 * (C) Copyright 2012
 * Siemens AG
 * ATS 11
 * Herbert Bernecker <Herbert.Bernecker@siemens.com>
 *
 * First version 2012-14-02: Herbert Bernecker
 * Linux IRTE base driver for SOC1-Boards
 *   - address mapping for register and kram
 *   - interrupt propagation to the ethernet device driver (EDD) in user space
 *   - adaption of the linux TCP/IP-Stack
 *         - get instructions  from the EDD to forward ip-frames (no copy) to the linux TCP/IP-Stack
 *         - get ip-frames from the linux TCP/IP-Stack and instruct the EDD to send it (no copy)
 *
 * ethy_event.c:
 *       - initialize the event area
 *       - register rt-events (ioctl)
 *
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include "ethy.h"

// this object must be alive for the complete time
// events are used.
// entries according the interrupt numbers of the soc1 interrupt controller
struct rt_event ethy_ev_arr[NO_OF_EVENTS] = {
   { .ev_id = EVENT_ID_INTERRUPT },
   { .ev_id = EVENT_ID_TIMEOUT }
};

int eventActiveFlag[NO_OF_EVENTS];
static int ev_area_id = -EINVAL;
static unsigned short affinity_state[NO_OF_EVENTS];


// call back functions from the aud driver
// we may be called back when a thread or application is forced to terminate
// we are also called in the case of rt_register_event() (SIGEV_NONE => unregister)
static int ev_disable_callback(void *arg, struct rt_event *actEvt)
{
	int index = (int) arg;

	eventActiveFlag[index] = 0;

	if(affinity_state[index] == 1) {
		if(index == EVENT_INDEX_INTERRUPT)
			rt_disable_irq_affinity(ethy_net_dev->irq);
		affinity_state[index] = 0;
	}

	return(0);
}
// we are called in the case of rt_register_event() (no SIGEV_NONE)
static int ev_enable_callback(void *arg, struct rt_event *actEvt)
{
	int index = (int) arg;

	eventActiveFlag[index] = 1;

	if(affinity_state[index] == 0) {
		if(index == EVENT_INDEX_INTERRUPT)
			rt_enable_irq_affinity(ethy_net_dev->irq);
		affinity_state[index] = 1;
	}

	return(0);
}

int ethy_init_event_area()
{
   int ii;

   for (ii = 0; ii < NO_OF_EVENTS; ii++) {
      // set callback routines for valid event ids
      if (ethy_ev_arr[ii].ev_id != 0) {
         ethy_ev_arr[ii].ev_disable = ev_disable_callback;
         ethy_ev_arr[ii].ev_enable = ev_enable_callback;
         ethy_ev_arr[ii].endisable_par = (void *) ii;
      }
   }

   if ((ev_area_id = rt_init_event_area(ethy_ev_arr, NO_OF_EVENTS)) < 0)
   {
      PRINTK("could not initialize event area of %s\n", ETHY_NAME);
      return(ev_area_id);
   }

   return ETHY_OK;
}

int ethy_destroy_event_area()
{
   if (ev_area_id > 0) {
      rt_destroy_event_area(ev_area_id);
   }

   return ETHY_OK;
}

int ethy_ioctl_event_create(unsigned long arg)
{
   int ret=0;
   int ii;
   struct rt_ev_desc event;

   PRINTK(KERN_INFO "[%s]: AuD_EVENT_CREATE task=%d \n", ETHY_NAME, current->pid);

   if ((ret = copy_from_user(&event, (struct rt_ev_desc *)arg,
            sizeof(event))))
         return (ret);

   if (ev_area_id < 0) {
      PRINTK("event area not yet initialized\n");
      return(ev_area_id);
   }

   for (ii = 0; ii < NO_OF_EVENTS; ii++) {
      if (event.event && (ethy_ev_arr[ii].ev_id == event.event)) {
         if ((ret = rt_register_event(ev_area_id, &event)) == 0) {
            if (event.sigevent.sigev_notify == SIGEV_NONE) {
               // we do our work in the callback routine
            }
            else {
               // we do our work in the callback routine
            }

            PRINTK( "register event %#x (slot %d flag=%d) for pid %d done (%d)\n",
                event.event, ii, eventActiveFlag[ii], 
                  event.sigevent._sigev_un._tid, ret);

            return 0;
         }

         PRINTK("problem register event %#x (ret=%d)\n",
               event.event, ret);

         return ret;
      }   
   }

   PRINTK("problem with event %#x\n", event.event);
   return -EINVAL;
}

/*****************************************************************************/
/*  end of file ethy_event.c                                                 */
/*****************************************************************************/ 
