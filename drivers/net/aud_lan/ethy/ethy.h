#ifndef ETHY_H_
#define ETHY_H_

#define __NO_VERSION_
#ifndef __KERNEL__
#define __KERNEL__
#endif

#include <linux/fs.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>	
#include <linux/mm.h>

#include <linux/autoconf.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>

#include <linux/dma-mapping.h>
#include <asm/setup.h>
#include <asm/delay.h>
#include <asm/dma-mapping.h>

#include <asm/mach-aud_soc/irq.h>

#include <linux/aud/rt_driver.h>
#include <linux/aud/rt_base.h>

#define ETHY_NAME             "ethy"
#define ETHY_INT_NR  IRQ1_SP

// --- irte
#define SOC1_IRTE_START_ADDR	0x1D200000
#define SOC1_GPIO_START_ADDR    0x1FA00000 

// --- ioctl's
#define ETHY_IOCTL_BASE 'Y'
#define ETHY_IOCTL_REGISTER       _IO  (ETHY_IOCTL_BASE, 0)
#define ETHY_IOCTL_UNREGISTER     _IO  (ETHY_IOCTL_BASE, 1)
#define ETHY_IOCTL_FRAME_RCV      _IOWR(ETHY_IOCTL_BASE, 2, struct edd_frame*)
#define ETHY_IOCTL_FRAME_RCV_STOP _IO  (ETHY_IOCTL_BASE, 3)
#define ETHY_IOCTL_FRAME_SND      _IOWR(ETHY_IOCTL_BASE, 4, struct edd_frame*)
#define ETHY_IOCTL_FRAME_SND_STOP _IO  (ETHY_IOCTL_BASE, 5)
#define ETHY_IOCTL_NETIF_QUEUE    _IOW (ETHY_IOCTL_BASE, 6, unsigned long)
#define ETHY_IOCTL_PHY_DMA_PTR    _IO  (ETHY_IOCTL_BASE, 7)
#define ETHY_IOCTL_VIRT_DMA_PTR   _IO  (ETHY_IOCTL_BASE, 8)
#define ETHY_IOCTL_GET_MAC_ADDR   _IOR (ETHY_IOCTL_BASE, 9, char*)
#define ETHY_IOCTL_NETIF_CARRIER  _IOW (ETHY_IOCTL_BASE, 10, unsigned long)

#define NETIF_START 1
#define NETIF_STOP  2
#define NETIF_WAKE  3

#define NETIF_CARRIER_OFF 1
#define NETIF_CARRIER_ON  2

// --- events
#define NO_OF_EVENTS       2
#define EVENT_ID_INTERRUPT 1
#define EVENT_ID_TIMEOUT   2

#define EVENT_INDEX_INTERRUPT (EVENT_ID_INTERRUPT -1)
#define EVENT_INDEX_TIMEOUT   (EVENT_ID_TIMEOUT -1)

//---- printk
#define PRINTK(fmt, args...) printkGen(NULL, "%s: " fmt, __FUNCTION__ , ## args)
#undef ETHY_PRINTK_DEBUG
#ifdef ETHY_PRINTK_DEBUG
#  define PR_DEBUG(fmt, args...) printkGen(NULL, "%s: " fmt, __FUNCTION__ , ## args)
#else
#  define PR_DEBUG(fmt, args...)
#endif

//---- bool
#define ETHY_OK     1
#define ETHY_FAILED 0

//--- realtime
#define ETHY_REALTIME

//--- send frames
#define MAX_SND_FRAMES 10

// --- externals
extern void*   phy_dma_ptr;
extern void*   virt_dma_ptr;
extern size_t  dma_size;
extern struct net_device *ethy_net_dev;
extern int eventActiveFlag[NO_OF_EVENTS];
extern struct rt_event ethy_ev_arr[NO_OF_EVENTS];

struct edd_frame
{
   unsigned char   *buf;
   unsigned long   len;
};


struct ethy_priv {
   struct net_device_stats stats;
   unsigned char mac[ETH_ALEN];
   spinlock_t lock;
};

/* --------------------------------------------------
 * ethy net operations
 */
void ethy_net_setup( struct net_device *dev );
int ethy_io_net_rcv(unsigned long arg);
void ethy_net_rcv_stop(void);
int ethy_io_net_snd(unsigned long arg);
void ethy_net_snd_stop(void);
int ethy_net_queue(unsigned long arg);
int ethy_net_get_mac(unsigned long arg);
int ethy_net_carrier(unsigned long arg);
int ethy_net_write(const char* buf, unsigned long len);
int ethy_net_read(unsigned char* buf, unsigned long *len);
int ethy_net_stop(struct net_device *dev);
int ethy_net_halt(struct net_device *dev);


/* --------------------------------------------------
 * ethy event operations
 */
int ethy_ioctl_event_create(unsigned long arg);
int ethy_init_event_area(void);
int ethy_destroy_event_area(void);


#endif /*ETHY_H_*/
