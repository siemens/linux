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
 * ethy_char.c: character driver as base driver for
 *                 - ioctl's
 *                 - start/stop the ethernet driver
 *                 - init the event area and create rt-events
 *                 - memory mapping
 *                 - allocate dma memory
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

static int ethy_major = 0;
static char* ethy_name = ETHY_NAME;
static struct class *ethy_class;
static struct device *ethy_dev;

static int ethy_open(struct inode *node, struct file *file);
static int ethy_close(struct inode *node, struct file *file);
static ssize_t ethy_write(struct file *file, const char *buf, size_t count, loff_t *offset);
static ssize_t ethy_read(struct file *file, char *buf, size_t count, loff_t *offset);
static int ethy_phys_mmap (struct file *filp, struct vm_area_struct *vma);
static unsigned long ethy_unmapped_area( struct file *file, unsigned long addr,
                                  unsigned long len, unsigned long pgoff,
                                  unsigned long flags );
static int ethy_register_net_dev(void);
static void ethy_unregister_net_dev(void);
static long ethy_ioctl (struct file *file, unsigned int cmd, unsigned long arg);

extern unsigned long ethy_stop_snd;

static struct file_operations ethy_fops =
{
   owner:THIS_MODULE,
   open:ethy_open,
   release:ethy_close,
   read:ethy_read,
   write:ethy_write,
   unlocked_ioctl:ethy_ioctl,
   mmap:ethy_phys_mmap,
   get_unmapped_area:ethy_unmapped_area
};

static int ethy_open(struct inode *node, struct file *file)
{
   PRINTK(" \n");
   rt_allow_access(file, RT_IO_ALLOWED);
   return 0;
}

static int ethy_close(struct inode *node, struct file *file)
{
   PRINTK(" \n");
   rt_remove_access(file);
   ethy_net_halt(ethy_net_dev);
   ethy_stop_snd = 0;
   return 0;
}

static ssize_t ethy_write(struct file *file, const char *buf, size_t count, loff_t *offset)
{
   ethy_net_write(buf, count);
   return (count);
}

static ssize_t ethy_read(struct file *file, char *buf, size_t count, loff_t *offset)
{
   unsigned long read;
   ethy_net_read(buf, &read);
   return (read);
}

static long ethy_ioctl (struct file *file, unsigned int cmd, unsigned long arg)
{
   int ret=0;

   PR_DEBUG("[%s]: ioctl 0x%4.4x %p \n", ETHY_NAME,  cmd, (void *)arg);

   switch( cmd ) {
      case ETHY_IOCTL_REGISTER:     ethy_register_net_dev();            break;
      case ETHY_IOCTL_UNREGISTER:   ethy_unregister_net_dev();          break;
      case ETHY_IOCTL_FRAME_RCV:    ethy_io_net_rcv(arg);               break;
      case ETHY_IOCTL_FRAME_RCV_STOP:   ethy_net_rcv_stop();            break;
      case ETHY_IOCTL_FRAME_SND:    ethy_io_net_snd(arg);               break;
      case ETHY_IOCTL_FRAME_SND_STOP:   ethy_net_snd_stop();            break;
      case ETHY_IOCTL_NETIF_QUEUE:  ethy_net_queue(arg);                break;
      case ETHY_IOCTL_PHY_DMA_PTR:  ret = (ssize_t)(phy_dma_ptr);       break;
      case ETHY_IOCTL_VIRT_DMA_PTR: ret = (ssize_t)(virt_dma_ptr);      break;
      case ETHY_IOCTL_GET_MAC_ADDR: ethy_net_get_mac(arg);              break;
      case ETHY_IOCTL_NETIF_CARRIER: ethy_net_carrier(arg);             break;
      case AuD_EVENT_CREATE:        ret = ethy_ioctl_event_create(arg); break;
      default:
         PRINTK(KERN_WARNING "[%s]: ioctl unknown 0x%x\n", ETHY_NAME,cmd); break;
  }
      return (ret);
}

static unsigned long ethy_unmapped_area( struct file *file, unsigned long addr,
                                  unsigned long len, unsigned long pgoff,
                                  unsigned long flags )
{
   unsigned long mapped_addr = -ENOMEM;

   mapped_addr = get_unmapped_area(NULL, addr, len, pgoff, flags);

   PR_DEBUG("[%s]: get_unmapped_area addr=%#lx:%lx len=%lx off=%#lx flag=%#lx\n",
          ETHY_NAME, addr, mapped_addr, len, pgoff, flags);

   return(mapped_addr);
}

static int ethy_phys_mmap (struct file *filp, struct vm_area_struct *vma) 
{
   int ret = -EIO;
   size_t size = vma->vm_end - vma->vm_start;
   unsigned long addr;

   PR_DEBUG("[%s]: %s \n", ETHY_NAME,__FUNCTION__);

   /* map irte register */
   if (vma->vm_pgoff == 0) {
      addr = SOC1_IRTE_START_ADDR;
      if (addr & (PAGE_SIZE - 1)) return -ENOSYS;

      vma->vm_flags |= VM_IO | VM_RESERVED;
      vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

      ret = remap_pfn_range(vma, vma->vm_start,
                            addr >> PAGE_SHIFT,
                            size, vma->vm_page_prot);
      return (ret);
   }

   /* map dma buffer */
   if (vma->vm_pgoff == 0x10) {
      if ((unsigned long)phy_dma_ptr & (PAGE_SIZE - 1)) return -ENOSYS;

      vma->vm_flags |= VM_IO | VM_RESERVED;
      vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

      ret = remap_pfn_range(vma, vma->vm_start,
                            (unsigned long)phy_dma_ptr >> PAGE_SHIFT,
                            size, vma->vm_page_prot);
      return (ret);
   }

   /* map irte kram */
   if (vma->vm_pgoff == 0x100) {
      addr = SOC1_IRTE_START_ADDR + (vma->vm_pgoff * PAGE_SIZE);
      if (addr & (PAGE_SIZE - 1)) return -ENOSYS;

      vma->vm_flags |= VM_IO | VM_RESERVED;
      vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

      ret = remap_pfn_range(vma, vma->vm_start,
                            addr >> PAGE_SHIFT,
                            size, vma->vm_page_prot);
      return (ret);
   }

   /* map gpio's */
   if (vma->vm_pgoff == 0x800) {
      addr = SOC1_GPIO_START_ADDR;
      if (addr & (PAGE_SIZE - 1)) return -ENOSYS;

      vma->vm_flags |= VM_IO | VM_RESERVED;
      vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

      ret = remap_pfn_range(vma, vma->vm_start,
                            addr >> PAGE_SHIFT,
                            size, vma->vm_page_prot);
      return (ret);
   }

   /* map the system control register block */
   if (vma->vm_pgoff == 0x2900) {
      addr = SOC1_IRTE_START_ADDR + (vma->vm_pgoff * PAGE_SIZE);
      if (addr & (PAGE_SIZE - 1)) return -ENOSYS;

      vma->vm_flags |= VM_IO | VM_RESERVED;
      vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

      ret = remap_pfn_range(vma, vma->vm_start,
                            addr >> PAGE_SHIFT,
                            size, vma->vm_page_prot);
      return (ret);
   }

   PRINTK(KERN_WARNING "[%s]: error \n", ETHY_NAME);
   return (ret);
}

static int ethy_register_net_dev()
{
   int ret=0;

   PR_DEBUG(" \n");
   if( (ethy_net_dev = alloc_netdev(sizeof(struct ethy_priv),"ethy%d",ethy_net_setup))==NULL )
      return -ENOMEM; 

   ret = register_netdev(ethy_net_dev);

   /* Allocate and map DMA-capable buffer */
   dma_size = (size_t)0x10000;
   PR_DEBUG("[%s]: dma_alloc_coherent size=0x%x\n", ETHY_NAME,dma_size);
   virt_dma_ptr = 
          (dma_alloc_coherent(NULL, dma_size, (dma_addr_t*)(&phy_dma_ptr), 0));
   phy_dma_ptr = (void *)(((unsigned)phy_dma_ptr) | 0x40000000);
   PRINTK("[%s]: dma alloc virt_dma_ptr = %#x phy_dma_ptr = %#x \n",ETHY_NAME,virt_dma_ptr,phy_dma_ptr);

   if(!virt_dma_ptr) {
      PRINTK(KERN_WARNING "[%s]: dma_alloc_coherent failed \n", ETHY_NAME);
      dma_free_coherent(NULL, dma_size, virt_dma_ptr, (dma_addr_t)phy_dma_ptr);
      return -ENOMEM;
   } else {
      PR_DEBUG("[%s]: dma_alloc_coherent  virt=0x%lx phy=0x%lx \n", 
                        ETHY_NAME, 
                        (unsigned long)virt_dma_ptr, 
                        (unsigned long)phy_dma_ptr);
   }

   return (ret);
}

static void ethy_unregister_net_dev()
{
   PRINTK(KERN_INFO "[%s]: %s \n", ETHY_NAME,__FUNCTION__);
   if(virt_dma_ptr) {
      phy_dma_ptr = (void *)(((unsigned) phy_dma_ptr) & 0x0fffffff);
      dma_free_coherent(NULL, dma_size, virt_dma_ptr, (dma_addr_t)phy_dma_ptr);
   }
   unregister_netdev(ethy_net_dev);
   free_irq(ETHY_INT_NR, NULL);
   free_netdev(ethy_net_dev);
}

/* --------------------------------------------------
 * ethy init / exit
 */

static int __init ethy_drv_init (void)
{
   int ret=0;
   int major=0;
   void *ptr_err;

   PRINTK(KERN_INFO "[%s]: %s \n", ETHY_NAME,__FUNCTION__);

   if( (ethy_major = register_chrdev (major, ethy_name, &ethy_fops)) < 0 ) {
      PRINTK(KERN_WARNING "[%s]: device already in use.\n", ETHY_NAME);
      return -EBUSY;
   } else PR_DEBUG("[%s]: name=%s major=%d \n", ETHY_NAME, ethy_name, ethy_major);

   /* Create the driver device node dynamically. */
   ethy_class = class_create(THIS_MODULE, ethy_name);
   if (IS_ERR(ptr_err = ethy_class)) {
      PRINTK(KERN_WARNING "[%s]: class_create failed (%d) major=%d\n", ETHY_NAME, (int)PTR_ERR(ptr_err), ethy_major);
      goto err;
   }

   ethy_dev = device_create(ethy_class, NULL, MKDEV(ethy_major, 0), NULL, ethy_name);
   if (IS_ERR(ptr_err = ethy_dev)) {
      PRINTK(KERN_WARNING "[%s]: device_create failed (%d) major=%d\n", ETHY_NAME, (int)PTR_ERR(ptr_err), ethy_major);
      class_destroy(ethy_class);
      goto err;
   }

   // initialize the event area
   ethy_init_event_area();

   // register ethy0
   ethy_register_net_dev();

   return ret;

err:
   unregister_chrdev(ethy_major, ethy_name);
   ethy_major = 0;
   return PTR_ERR(ptr_err);
}
module_init(ethy_drv_init);

#ifdef MODULE
static void __exit ethy_drv_exit (void)
{
   PRINTK(KERN_INFO "[%s]: exit \n", ETHY_NAME);
   if (ethy_major > 0)
   {
      // unregister ethy0
      ethy_unregister_net_dev();

      // Delete event area
      ethy_destroy_event_area();

      // Remove driver
      device_destroy(ethy_class, MKDEV(ethy_major, 0));
      class_destroy(ethy_class);
      unregister_chrdev(ethy_major, ETHY_NAME);   // unregister the driver
   }
}
module_exit(ethy_drv_exit);
#endif

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Herbert Bernecker <Herbert.Bernecker@siemens.com>");

/*****************************************************************************/
/*  end of file ethy_char.c                                                  */
/*****************************************************************************/ 
