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
 * ethy_net.c: ethernet driver
 *                 - send interrupt event to user-space-driver (EDD)
 *                 - forward ip-frames from EDD to TCP/IP-Stack
 *                 - forward ip-frames from TCP/IP-Stack to EDD
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
#include <linux/aud/rt_sched.h>
#include <linux/aud/rt_link.h>
#include <linux/ip.h>
#include <linux/icmp.h>

static int net_open = 0;
unsigned long ethy_stop_snd = 0;
struct net_device *ethy_net_dev = NULL;
unsigned char mac_default[ETH_ALEN] = {0x04, 0x22, 0x00, 0x00, 0x00, 0x08};
struct edd_frame edd_snd_frame;
struct sk_buff   *skb_snd = NULL;
struct sk_buff   *skb_snd_done = NULL;
struct task_struct *snd_task;
spinlock_t snd_lock;

struct edd_frame edd_rcv_frame = {NULL, 0};


void*   phy_dma_ptr;
void*   virt_dma_ptr;
size_t dma_size;

static irqreturn_t ethy_net_interrupt( int irq_in, void *dev_id );
static struct net_device_stats *ethy_get_stats(struct net_device *dev);
static int ethy_net_open(struct net_device *dev);
static int ethy_net_start_xmit(struct sk_buff *skb, struct net_device *dev);
static void ethy_net_timeout(struct net_device *dev);

extern int rtx_raise_soft_irq(int irqnr);

static const struct net_device_ops ethy_netdev_ops = {
   .ndo_open   = ethy_net_open,
   .ndo_stop   = ethy_net_stop,
   .ndo_start_xmit  = ethy_net_start_xmit,
   .ndo_get_stats   = ethy_get_stats,
   .ndo_tx_timeout		= ethy_net_timeout,
};

static struct irqaction ethy_irq = {
   .name     = ETHY_NAME,
   .flags    = IRQF_SHARED,
   .handler  = ethy_net_interrupt,
};


static irqreturn_t ethy_net_interrupt( int irq_in,
                                   void *dev_id )
{
   int evtId1 = EVENT_ID_INTERRUPT -1;
   int ev_stat;

   // send event to uedd
   if (eventActiveFlag[evtId1]) {
      ev_stat = rt_send_event(&ethy_ev_arr[evtId1]);
   }

   PR_DEBUG(" %#x %#x \n", eventActiveFlag[evtId1]);

   return IRQ_HANDLED;
}

static struct net_device_stats *ethy_get_stats(struct net_device *dev)
{
   struct ethy_priv *priv = netdev_priv(dev);
   return &priv->stats;
}

static int ethy_net_open(struct net_device *dev)
{
   int ret;
   struct ethy_priv *priv = netdev_priv(dev);

   if(!net_open) {
      net_open = 1;
      PRINTK(KERN_INFO "[%s]: %s \n", ETHY_NAME,__FUNCTION__);

      memset (&priv->stats, 0, sizeof (priv->stats));

#ifdef ETHY_REALTIME
      ret = rt_request_irq(ethy_net_dev->irq, 
                           ethy_irq.handler, 
                           ethy_irq.flags, 
                           ethy_irq.name, 
                           ethy_net_dev,
                           NULL);
#else
      ret = request_irq(ethy_net_dev->irq, 
                     ethy_irq.handler, 
                     ethy_irq.flags, 
                     ethy_irq.name, 
                     ethy_net_dev);
#endif

      if (ret < 0) {
         PRINTK(KERN_WARNING "[%s]: %s failed to request irq!\n", 
                               ETHY_NAME,__FUNCTION__);

         return ret;
      }

      netif_start_queue(dev);

   } else PRINTK(KERN_WARNING "[%s]: %s error - only one instance!\n", 
                               ETHY_NAME,__FUNCTION__);

   return (0);
}

int ethy_net_stop(struct net_device *dev)
{
   if(net_open) {
      PRINTK(" net stop rt_state = %#x \n", current->rt_state);
      netif_stop_queue(dev);
      netif_carrier_off(dev);
#ifdef ETHY_REALTIME
      rt_free_irq(dev->irq, dev);
#else
      free_irq(dev->irq, dev);
#endif
      net_open = 0;
   }
   return (0);
}

int ethy_net_halt(struct net_device *dev)
{
   if(net_open) {
      PRINTK(" net halt \n");
      netif_stop_queue(dev);
   }
   return (0);
}

int ethy_net_queue(unsigned long arg)
{
   PR_DEBUG("[%s]: %s \n", ETHY_NAME,__FUNCTION__);
   if(ethy_net_dev) {
      switch(arg) {
         case NETIF_START: netif_start_queue(ethy_net_dev); break;
         case NETIF_STOP:  netif_stop_queue(ethy_net_dev);  break;
         case NETIF_WAKE:  netif_wake_queue(ethy_net_dev);  break;
      }
   } else return 0;

   return 1;
}

int ethy_net_carrier(unsigned long arg)
{
   PR_DEBUG("[%s]: %s \n", ETHY_NAME,__FUNCTION__);
   if(ethy_net_dev) {
      switch(arg) {
         case NETIF_CARRIER_OFF: netif_carrier_off(ethy_net_dev); break;
         case NETIF_CARRIER_ON:  netif_carrier_on(ethy_net_dev);  break;
         default: return 0;
      }
   } else return 0;

   return 1;
}

int ethy_net_get_mac(unsigned long arg)
{
   struct ethy_priv *priv = netdev_priv(ethy_net_dev);
   if (copy_to_user((char*)arg, (char*)(priv->mac), ETH_ALEN))
      return (-EFAULT);
   else return 0;
}

static int ethy_net_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
   PR_DEBUG("[%s]: %s \n", ETHY_NAME,__FUNCTION__);

   local_irq_disable_hw();

   if(skb_snd) {
      // already busy: forget it
      // TBD something better
      local_irq_enable_hw();
      PR_DEBUG("busy \n");
      return NETDEV_TX_BUSY;
   }

   if(skb_snd_done) {
      dev_kfree_skb( skb_snd_done );
      skb_snd_done = NULL;
   }

   if(snd_task) {
      if(THREAD_IS_REALTIME(snd_task)) {
         skb_snd = skb;
         local_irq_enable_hw();
         lx_wake_up_rt_thread(snd_task);
         PR_DEBUG("after rt wake up\n");
      } else {
         local_irq_enable_hw();
         spin_lock(&snd_lock);
         skb_snd = skb;
         spin_unlock(&snd_lock);
         wake_up_process(snd_task);
         PR_DEBUG("after wake_up_process\n");
      }
   } else {
      local_irq_enable_hw();
      PR_DEBUG("no send task\n");
      return NETDEV_TX_BUSY;
   }

   return NETDEV_TX_OK;
}

void ethy_net_snd_stop()
{
   netif_stop_queue(ethy_net_dev);
   ethy_stop_snd = 1;

   if(snd_task) {
      if(THREAD_IS_REALTIME(snd_task)) {
         lx_wake_up_rt_thread(snd_task);
      } else {
         wake_up_process(snd_task);
      }
   }

	if(skb_snd_done) dev_kfree_skb( skb_snd_done );
	if(skb_snd) dev_kfree_skb( skb_snd );

}

void ethy_net_rcv_stop()
{
   ethy_net_stop(ethy_net_dev);

   // send one event to stop the rcv-thread
   if (eventActiveFlag[0])
      rt_send_event(&ethy_ev_arr[0]);

   // send one event to stop the timeout-thread
   if (eventActiveFlag[1])
      rt_send_event(&ethy_ev_arr[1]);
}

int ethy_net_read(unsigned char* buf, unsigned long *len)
{
   // alternative to ioctl; not implemented
   PRINTK(" not implemented \n");
   return 0;
}

int ethy_net_write(const char * buf, unsigned long len)
{
   // alternative to ioctl; not implemented
   PRINTK(" not implemented \n");
   return 0;
}

int ethy_io_net_snd(unsigned long arg)
{
   PR_DEBUG(" \n");

   if (__copy_from_user(&edd_snd_frame, (char*)arg, sizeof(struct edd_frame))) {
      PRINTK(" copy_from_user fault \n");
      return (-EFAULT);
   }

   if(IS_REALTIME) {
      local_irq_disable_hw();
      while(1) {
         if(ethy_stop_snd) {
            local_irq_enable_hw();
            return 0;
         }

         if(skb_snd) {
            // use it
            edd_snd_frame.len = skb_snd->len;
            memcpy( (void *)edd_snd_frame.buf, skb_snd->data, skb_snd->len);
            __copy_to_user((char*)arg, &edd_snd_frame, sizeof(struct edd_frame));
            ((struct net_device_stats *)netdev_priv(ethy_net_dev))->tx_packets++;
            ((struct net_device_stats *)netdev_priv(ethy_net_dev))->tx_bytes += skb_snd->len;
            skb_snd_done = skb_snd;
            skb_snd = NULL;
            local_irq_enable_hw();
            return 0;
         }

         // wait for request
         snd_task = current;
         __set_current_rt_state(RT_TASK_UNINTERRUPTIBLE);
         local_irq_enable_hw();
         rt_schedule(0);
         local_irq_disable_hw();
         snd_task = NULL;
      }
   } else {
      PR_DEBUG("[%s]: %s linux\n", ETHY_NAME,__FUNCTION__);
      spin_lock(&snd_lock);
      while(1) {
         if(ethy_stop_snd) {
            spin_unlock(&snd_lock);
            return 0;
         }

         if(skb_snd) {
            // use it
            edd_snd_frame.len = skb_snd->len;
            memcpy( (void *)edd_snd_frame.buf, skb_snd->data, skb_snd->len);
            __copy_to_user((char*)arg, &edd_snd_frame, sizeof(struct edd_frame));
            ((struct net_device_stats *)netdev_priv(ethy_net_dev))->tx_packets++;
            ((struct net_device_stats *)netdev_priv(ethy_net_dev))->tx_bytes += skb_snd->len;
            //dev_kfree_skb( skb_snd );
            skb_snd_done = skb_snd;
            skb_snd = NULL;
            spin_unlock(&snd_lock);
            PR_DEBUG("[%s]: %s send\n", ETHY_NAME,__FUNCTION__);
            return 0;
         }

         // wait for request
         snd_task = current;
         set_current_state(TASK_UNINTERRUPTIBLE);
         spin_unlock(&snd_lock);
         schedule();
         spin_lock(&snd_lock);
         snd_task = NULL;
      }
   }

   return 0;
}

int ethy_io_net_rcv(unsigned long arg)
{
   struct sk_buff *skb = NULL;
   unsigned char *buf;

   PR_DEBUG("net rcv \n");

   if (copy_from_user(&edd_rcv_frame, (char*)arg, sizeof(struct edd_frame)))
      return (-EFAULT);

   skb = dev_alloc_skb(edd_rcv_frame.len +2);
   if (skb == NULL) {
      PRINTK(KERN_WARNING "[%s]: Failed to allocate sk_buff (%d).\n",
                          ETHY_NAME, (int) edd_rcv_frame.len);
      ((struct net_device_stats *)netdev_priv(ethy_net_dev))->rx_dropped++;
   } else {
      skb_reserve(skb,2);
      skb->dev = ethy_net_dev;
      buf = skb_put(skb,edd_rcv_frame.len);
      memcpy(buf, edd_rcv_frame.buf, edd_rcv_frame.len);
      skb->protocol = eth_type_trans(skb, ethy_net_dev);

      netif_rx(skb);
      rtx_raise_soft_irq(IRQ_SOFTWARE_0);

      ((struct net_device_stats *)netdev_priv(ethy_net_dev))->rx_packets++;
      ((struct net_device_stats *)netdev_priv(ethy_net_dev))->rx_bytes += edd_rcv_frame.len;
   }

   return 0;
}

static void ethy_net_timeout(struct net_device *dev)
{
   int evtId = EVENT_ID_TIMEOUT -1;

   PRINTK(KERN_INFO " timeout\n");

   // send event to uedd
   if (eventActiveFlag[evtId]) {
      rt_send_event(&ethy_ev_arr[evtId]);
   }
}

void ethy_net_mac(unsigned char *mac)
{
   int i;
   char tmp_cmdline[COMMAND_LINE_SIZE];

   PRINTK(KERN_INFO "[%s]: %s \n", ETHY_NAME,__FUNCTION__);
   strlcpy(tmp_cmdline, saved_command_line, COMMAND_LINE_SIZE);

   if(strstr(tmp_cmdline,"mac=")==NULL){
      for(i=0; i<ETH_ALEN; i++) *(mac+i) = mac_default[i];
   } else {
      char *single_arg,*zeile;

      zeile = tmp_cmdline;
      while((single_arg = strsep(&zeile," "))!=NULL) {
         if(strstr(single_arg,"mac=")!=NULL) {
            strsep(&single_arg,"=: \0");
            for(i=0; i<ETH_ALEN; i++) 
               *(mac+i) = simple_strtol(strsep(&single_arg,"=: \0"),NULL,16);
         }
      }
   }

   PRINTK("[%s] Use bootargs mac-address %02X:%02X:%02X:%02X:%02X:%02X \n",
           ETHY_NAME, *mac,*(mac+1),*(mac+2),*(mac+2),*(mac+3),*(mac+5) );
}

void ethy_net_setup( struct net_device *dev )
{
   struct ethy_priv *priv = netdev_priv(dev);

   PRINTK(KERN_INFO "[%s]: %s \n", ETHY_NAME,__FUNCTION__);
   ether_setup(dev);

   netif_carrier_off(dev);

   dev->netdev_ops = &ethy_netdev_ops;
   dev->destructor = free_netdev;
   dev->irq = ETHY_INT_NR;
   dev->watchdog_timeo = 1000;

   ethy_net_mac((unsigned char *)(&priv->mac));
   memcpy(dev->dev_addr, priv->mac, ETH_ALEN);

   spin_lock_init(&snd_lock);
}

/*****************************************************************************/
/*  end of file ethy_net.c                                                   */
/*****************************************************************************/
