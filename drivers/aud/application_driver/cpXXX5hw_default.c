/*
 *  Copyright (C) 2008 Manfred Neugebauer <manfred.neugebauer@siemens.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  Hardwaredriver for SOC1 based CP boards using AUDIS
 *  
 *  Updated for kernel 2.6.31 by Alexander Kubicki 
 */


#include <asm/siginfo.h>
#include <linux/fs.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/bcd.h>
#include <linux/list.h>
#include <linux/irq.h>

#include <linux/aud/rt_driver.h>
#include <linux/aud/rt_base.h>


#define MAJOR_NUMBER        209

#define CPXXX5HW_DEBUG

#ifdef MODULE
MODULE_AUTHOR ("M.Neugebauer / F.Ensslin / A.Kubicki");
MODULE_LICENSE ("GPL");
MODULE_DESCRIPTION ("SOC1 HW driver");
#endif


#define PROFIBUS_MAX_INTR_NR            7
#define SOC_PROFIBUS_WAIT_MAGIC         0x12120800
#define SOC_PROFIBUS_WAITFOR_PB1        (SOC_PROFIBUS_WAIT_MAGIC | 0)
#define SOC_PROFIBUS_WAITFOR_PB2        (SOC_PROFIBUS_WAIT_MAGIC | 1)
#define SOC_PROFIBUS_WAITFOR_PB_PLL     (SOC_PROFIBUS_WAIT_MAGIC | 2)
#define SOC_PROFIBUS_WAITFOR_BREAK_K1   (SOC_PROFIBUS_WAIT_MAGIC | 3)
#define SOC_PROFIBUS_WAITFOR_BREAK_K2   (SOC_PROFIBUS_WAIT_MAGIC | 4)
#define SOC_PROFIBUS_WAITFOR_INT_BUS_3  (SOC_PROFIBUS_WAIT_MAGIC | 5)
#define SOC_PROFIBUS_WAITFOR_SYS_INIT   (SOC_PROFIBUS_WAIT_MAGIC | 6)

#define NO_OF_EVENTS                    7

#define MMIO_BASE_ADDR                  0x15E00000
#define MMIO_END_ADDR                   0x20000000
#define MMIO_SIZE                       (MMIO_END_ADDR - MMIO_BASE_ADDR)

#define PSEUDO_FILENAME  "application-interrupts"
#define PSEUDO_FILENAME2 "cptype"

#define CPXXX5HW_MAGIC 'k'
#define CPXXX5HW_AuD_SIGNAL_0 _IO (CPXXX5HW_MAGIC, 0)
#define CPXXX5HW_AuD_SIGNAL_1 _IO (CPXXX5HW_MAGIC, 1)


static int ev_area_id = -EINVAL;
/* this object must be alive for the complete time
 * events are used */
static struct rt_event ppdrv_ev_arr [NO_OF_EVENTS] = {
    { .ev_id = SOC_PROFIBUS_WAITFOR_PB1 },
    { .ev_id = SOC_PROFIBUS_WAITFOR_PB2 },
    { .ev_id = SOC_PROFIBUS_WAITFOR_PB_PLL },
    { .ev_id = SOC_PROFIBUS_WAITFOR_BREAK_K1 },
    { .ev_id = SOC_PROFIBUS_WAITFOR_BREAK_K2 },
    { .ev_id = SOC_PROFIBUS_WAITFOR_INT_BUS_3 },
    { .ev_id = SOC_PROFIBUS_WAITFOR_SYS_INIT },
};
static int eventActive [NO_OF_EVENTS];

static char driver_name [256];

static unsigned countBus3;
static unsigned countpb1;
static unsigned countpb2;
static unsigned countInit;


typedef struct soc1hw_info {
    unsigned flag;
} soc1hw_info_t ;

static soc1hw_info_t *private_dev;

static int  soc1hw_open    (struct inode *inode, struct file *filp);
static int  soc1hw_release (struct inode *inode, struct file *filp);
static int  soc1hw_read    (struct file *filp, char *pC, size_t a, loff_t *f_pos);
static int  soc1hw_write   (struct file *filp, const char *pC, size_t a, loff_t *f_pos);
static long soc1hw_ioctl   (struct file *filp, unsigned int cmd, unsigned long arg);
static int  soc1hw_mmap    (struct file *filp, struct vm_area_struct *memArea);
static unsigned long soc1hw_get_unmapped_area (struct file *filp,
                                                 unsigned long a, unsigned long b, unsigned long c,
                                                 unsigned long d);
//TBD
//int export_map_to_debug (struct file *filp, struct vm_area_struct *vma, void *ioAddr);

//TBD
//int unexport_map_to_debug (struct file *filp);

static struct file_operations soc1hw_fops =
{
    .release           = soc1hw_release,
    .open              = soc1hw_open,
    .unlocked_ioctl    = soc1hw_ioctl,
    .write             = soc1hw_write,
    .read              = soc1hw_read,
    .mmap              = soc1hw_mmap,
    .get_unmapped_area = soc1hw_get_unmapped_area,
};

/* the 32 bit value at this address is a microsecond counter of the SOC1 hw */
volatile unsigned int *pUsec    = (unsigned int *) 0xB6000018; 


/****************************************************************************
 *
 * ev_disable_callback - call back functions from the aud driver
 *                       may be called back when a thread or application is
 *                       forced to terminate
 *                       also called in the case of rt_register_event()
 *                       (SIGEV_NONE => unregister)
 *
 */
static int ev_disable_callback (void *arg, struct rt_event *actEvt)
{
    int index = (int) arg;

    #ifdef CPXXX5HW_DEBUG	
    printkGen (NULL, KERN_NOTICE "----cpXXX5hw: in ev_disable_callback ----\n");
    #endif


    eventActive [index] = 0;

    return(0);
}


/****************************************************************************
 *
 * ev_enable_callback - called in the case of rt_register_event()
 *
 */
static int ev_enable_callback (void *arg, struct rt_event *actEvt)
{
    
    int index = (int) arg;

    #ifdef CPXXX5HW_DEBUG	
    printkGen (NULL, KERN_NOTICE "----cpXXX5hw: in ev_enable_callback ----\n");
    #endif

    eventActive [index] = 1;

    return(0);
}


/****************************************************************************
 *
 * soc1hw_read - service currently not supported
 *
 */
static int soc1hw_read (struct file *filp, char *pC, size_t a, loff_t *f_pos)
{
    #ifdef CPXXX5HW_DEBUG	
    printkGen (NULL, KERN_NOTICE "----cpXXX5hw: in soc1hw_read ----\n");
    #endif

    return (-EINVAL);
}


/****************************************************************************
 *
 * soc1hw_write - service currently not supported
 *
 */
static int soc1hw_write (struct file *filp, const char *pC, size_t a, loff_t *f_pos)
{
    #ifdef CPXXX5HW_DEBUG	
    printkGen (NULL, KERN_NOTICE "----cpXXX5hw: in soc1hw_write ----\n");
    #endif
    return (-EINVAL);
}


/****************************************************************************
 *
 * soc1hw_ioctl - ioctl service to await interrupt events
 *
 */
static long soc1hw_ioctl (struct file *filp, unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    int ii;
    struct rt_ev_desc event;

    #ifdef CPXXX5HW_DEBUG	
    printkGen (NULL, KERN_NOTICE "----cpXXX5hw: in soc1hw_ioctl----\n");
    #endif

    switch (cmd)
    {




    case AuD_EVENT_CREATE:
        printkGen (NULL, KERN_NOTICE "[%s] ioctl AuD_EVENT_CREATE filp=%p arg=%#lx task=%d prio=%d:%u\n",
                driver_name,
                filp,
                arg,
                current->pid,
                current->prio,
                current->rt_priority);
        if ((ret = copy_from_user (&event, (struct rt_ev_desc *)arg,
                                   sizeof (event))))
            return (ret);
        if (ev_area_id < 0) {
            printkGen (NULL, KERN_NOTICE "[%s] event area not yet initialized\n", driver_name);
            return (ev_area_id);
        }
        for (ii = 0; ii < NO_OF_EVENTS; ii++) {
            if (event.event && (ppdrv_ev_arr [ii].ev_id == event.event)) {
                if ((ret = rt_register_event (ev_area_id, &event)) == 0) {
                    if (event.sigevent.sigev_notify != SIGEV_NONE)
                        ; // we do our work in the callback routine 
                    else
                        ; // we do our work in the callback routine 
                    return 0;
                }
                return ret;
            }
        }
        return -EINVAL;
    break;	


   case CPXXX5HW_AuD_SIGNAL_0:

    if (eventActive [SOC_PROFIBUS_WAITFOR_PB1 & 0x0ff])
    {
    #ifdef CPXXX5HW_DEBUG	
    printkGen (NULL, KERN_NOTICE "----cpXXX5hw: returning signal 0 ----\n");
    #endif
        rt_send_event (&ppdrv_ev_arr [SOC_PROFIBUS_WAITFOR_PB1 & 0x0ff]);
    }

    return -EINVAL;

    break;


   case CPXXX5HW_AuD_SIGNAL_1:

    if (eventActive [SOC_PROFIBUS_WAITFOR_PB2 & 0x0ff])
    {
    #ifdef CPXXX5HW_DEBUG	
    printkGen (NULL, KERN_NOTICE "----cpXXX5hw: returning signal 1 ----\n");
    #endif
        rt_send_event (&ppdrv_ev_arr [SOC_PROFIBUS_WAITFOR_PB2 & 0x0ff]);
    }

    return -EINVAL;

    break;

    default:
        printkGen (NULL, KERN_NOTICE "[%s] invalid cmd (%#x)\n", driver_name, cmd);
        ret = -EINVAL;
        break;
    }

    return (ret);
}


/****************************************************************************
 *
 * soc1hw_mmap - mmap service to map hw address space to userland
 *
 */
static int soc1hw_mmap (struct file *filp, struct vm_area_struct *vma)
{
    unsigned long off, vsize;
    int ret = 0;
    void *ioremapAddr;

    #ifdef CPXXX5HW_DEBUG	
    printkGen (NULL, KERN_NOTICE  "----cpXXX5hw: in soc1hw_mmap ----\n");
    #endif

    off = (vma->vm_pgoff << PAGE_SHIFT);
    vsize = vma->vm_end - vma->vm_start;

    if ((off < MMIO_BASE_ADDR) || ((off + vsize) > MMIO_END_ADDR)) {
        printkGen (NULL, KERN_NOTICE "[%s] mmap out of range (%#lx - %#lx))\n", driver_name, off, off + vsize);
        return -EINVAL;
    }

    vma->vm_page_prot = pgprot_noncached (vma->vm_page_prot);
    vma->vm_flags |= (VM_IO | VM_RESERVED);
    if ((ret = remap_pfn_range (vma,
                                vma->vm_start,
                                off >> PAGE_SHIFT,
                                vsize,
                                vma->vm_page_prot))) {
        printkGen (NULL, KERN_NOTICE "[%s] mmap problem remap ret=%d\n",
                driver_name, ret);
        return -EAGAIN;
    }

    ioremapAddr = ioremap (off, vsize);
    printkGen (NULL, KERN_NOTICE "[%s] mmap offset=%#lx len=%#08lx to %#lx filp=%#x vma=%#x remap=%#x\n",
            driver_name,
            off,
            vsize,
            vma->vm_start,
            (unsigned int) filp,
            (unsigned int) vma,
            (unsigned int) ioremapAddr);
    //export_map_to_debug(filp, vma, ioremapAddr);

    return (ret);
}


/****************************************************************************
 *
 * soc1hw_get_unmapped_area - service to get unmapped area
 *
 */
static unsigned long soc1hw_get_unmapped_area (struct file *filp,
                                                 unsigned long addr,
                                                 unsigned long len,
                                                 unsigned long pgoff,
                                                 unsigned long flags)
{
    unsigned long mapped_addr = -ENOMEM;

    #ifdef CPXXX5HW_DEBUG	
    printkGen (NULL, KERN_NOTICE "----cpXXX5hw: in soc1hw_get_unmapped_area ----\n");
    #endif

    mapped_addr = get_unmapped_area (NULL, addr, len, pgoff, flags);

    printk (KERN_NOTICE "[%s] get_unmapped_area addr=%#lx:%lx len=%lx off=%#lx flag=%#lx\n",
            driver_name,
            addr,
            mapped_addr,
            len,
            pgoff,
            flags);

    return (mapped_addr);
}


/****************************************************************************
 *
 * soc1hw_open - open service to open a soc1hw device
 *
 */
static int soc1hw_open (struct inode *inode, struct file *filp)
{
    int ret = 0;

    #ifdef CPXXX5HW_DEBUG	
   printkGen (NULL, KERN_NOTICE "----cpXXX5hw: in soc1hw_open ----\n");
    #endif

    filp->private_data = private_dev;
    rt_allow_access (filp, RT_IO_IOCTL);

    printk (KERN_NOTICE "[%s] open filp=%p\n", driver_name, filp);

    return (ret);
}


/****************************************************************************
 *
 * soc1hw_release - release service to close a soc1hw device
 *
 */
static int soc1hw_release (struct inode *inode, struct file *filp)
{
    int ret = 0;

    #ifdef CPXXX5HW_DEBUG	
    printkGen (NULL, KERN_NOTICE "----cpXXX5hw: in soc1hw_release ----\n");
    #endif

    //unexport_map_to_debug (filp);
    rt_remove_access (filp);                                     /* remove entry */
    printk (KERN_NOTICE "[%s] release filp=%p\n", driver_name, filp);

    return (ret);
}


/****************************************************************************
 *
 * soc1hw_pb1_interrupt - interrupt service routine
 *
 */
static irqreturn_t soc1hw_pb1_interrupt (int irq, void *dev_id,
                                           struct pt_regs *regs)
{
    #ifdef CPXXX5HW_DEBUG	
    printkGen (NULL, KERN_NOTICE "----cpXXX5hw: in soc1hw_bp1_interrupt ----\n");
    #endif

    if (++countpb1 == 1)
        printkGen (NULL, KERN_NOTICE "[%s] pb1 interrupt for intr=%d\n", driver_name, irq);

    if (eventActive [SOC_PROFIBUS_WAITFOR_PB1 & 0x0ff])
        rt_send_event (&ppdrv_ev_arr [SOC_PROFIBUS_WAITFOR_PB1 & 0x0ff]);

    return IRQ_HANDLED;
}


/****************************************************************************
 *
 * soc1hw_pb2_interrupt - interrupt service routine
 *
 */
static irqreturn_t soc1hw_pb2_interrupt (int irq, void *dev_id,
                                           struct pt_regs *regs)
{
    #ifdef CPXXX5HW_DEBUG	
    printkGen (NULL, KERN_NOTICE "----cpXXX5hw: in soc1hw_bp2_interrupt ----\n");
    #endif

    if (++countpb2 == 1)
        printkGen (NULL, KERN_NOTICE "[%s] pb2 interrupt for intr=%d\n", driver_name, irq);

    if (eventActive [SOC_PROFIBUS_WAITFOR_PB2 & 0x0ff])
        rt_send_event (&ppdrv_ev_arr [SOC_PROFIBUS_WAITFOR_PB2 & 0x0ff]);

    return IRQ_HANDLED;
}


/****************************************************************************
 *
 * enable the interrupt's source
 *
 */
static void EnableEpldPbusInt (void)
{
unsigned char *  const  pPanelRegBase = (unsigned char *) 0xBA000000;
    unsigned char           SysMsk        = pPanelRegBase [0];

    #ifdef CPXXX5HW_DEBUG	
    printkGen (NULL, KERN_NOTICE "----cpXXX5hw: in EnableEpldPbusInt ----\n");
    #endif    

    SysMsk |= 1 /* BUS3 */;
    pPanelRegBase [0] = SysMsk;
}


/****************************************************************************
 *
 * disable the interrupt's source to avoid subsequent occurences
 *
 */
static void DisableEpldPbusInt (void)
{
    volatile unsigned char *  const  pPanelRegBase = (unsigned char *) 0xBA000000;
    volatile unsigned char           SysMsk        = pPanelRegBase [0];

    #ifdef CPXXX5HW_DEBUG	
    printkGen (NULL, KERN_NOTICE "----cpXXX5hw: in DisableEpldPbusInt ----\n");
    #endif    

    SysMsk &= ~(1 /* BUS3 */);
    pPanelRegBase [0] = SysMsk;
}


/****************************************************************************
 *
 * soc1hw_int_bus3_interrupt - bus 3 interrupt service routine
 *
 */
static irqreturn_t soc1hw_int_bus3_interrupt (int irq, void *dev_id,
                                                struct pt_regs *regs)
{
    
    #ifdef CPXXX5HW_DEBUG	
    printkGen (NULL, KERN_NOTICE "----cpXXX5hw: in soc1hw_int_bus3_interrupt ----\n");
    #endif    

    DisableEpldPbusInt ();

    if (++countBus3 == 1)
        printkGen (NULL, KERN_NOTICE "[%s] bus3 interrupt for intr=%d\n", driver_name, irq);

    if (eventActive [SOC_PROFIBUS_WAITFOR_INT_BUS_3 & 0x0ff])
        rt_send_event (&ppdrv_ev_arr [SOC_PROFIBUS_WAITFOR_INT_BUS_3 & 0x0ff]);

    return IRQ_HANDLED;
}


/****************************************************************************
 *
 * disable the interrupt's source to avoid subsequent occurences
 *
 */
static void DisableEpldPbusResetInt (void)
{

    volatile unsigned char *  const  pPanelRegBase = (unsigned char *) 0xBA000000;
    volatile unsigned char           SysMsk        = pPanelRegBase [3];

    #ifdef CPXXX5HW_DEBUG	
    printkGen (NULL, KERN_NOTICE "----cpXXX5hw: in DisableEpldPbusResetInt ----\n");
    #endif    

    SysMsk &= ~((1 << 1 /* RES_BUS3 */));
    pPanelRegBase [3] = SysMsk;
}


/****************************************************************************
 *
 * soc1hw_sys_init_interrupt - interrupt service routine
 *
 */
static irqreturn_t soc1hw_sys_init_interrupt (int irq, void *dev_id,
                                                struct pt_regs *regs)
{

    #ifdef CPXXX5HW_DEBUG	
    printkGen (NULL, KERN_NOTICE "----cpXXX5hw: in soc1hw_sys_init_interrupt ----\n");
    #endif    

    if (++countInit == 1)
        printkGen (NULL, KERN_NOTICE "[%s] sys init interrupt for intr=%d\n", driver_name, irq);

    DisableEpldPbusResetInt ();

    if (eventActive [SOC_PROFIBUS_WAITFOR_SYS_INIT & 0x0ff])
        rt_send_event (&ppdrv_ev_arr [SOC_PROFIBUS_WAITFOR_SYS_INIT & 0x0ff]);
    
    return IRQ_HANDLED;
}


/* entries for pseudo-files in the /proc directory */
static struct proc_dir_entry *  soc1_ints_file;
static struct proc_dir_entry *  soc1_cptype_file;

/* buffer holding the last user error details */
static char soc1_ints_string   [2048];
static char soc1_cptype_string [2048];


/****************************************************************************
 *
 * soc1_cptype - read the cp type number from hw coding
 *
 */
static int soc1_cptype (void)
{
    
    unsigned int gpio1in = *((volatile unsigned int *) 0xBFA00034);
    unsigned int gpio2in = *((volatile unsigned int *) 0xBFA00054);
    unsigned int cpcode = ((gpio1in & 0xE0000000) >> 29) | ((gpio2in & 0x00000001) << 3);

    #ifdef CPXXX5HW_DEBUG	
    printkGen (NULL, KERN_NOTICE "----cpXXX5hw: in soc1_cptype ----\n");
    #endif    


    switch (cpcode)
    {
    case 0xF:
        return 342;
    case 0xE:
        return 443;
    default:
        return cpcode;
    }
}


/****************************************************************************
 *
 * soc1_ints_read -  handle read requests from /proc/application-interrupts
 *
 */
static int soc1_ints_read (char *buffer, char **start, off_t off, int count, int *eof, void *data)
{
    #ifdef CPXXX5HW_DEBUG	
    printkGen (NULL, KERN_NOTICE "----cpXXX5hw: in soc1_ints_read ----\n");
    #endif    
    

    sprintf (soc1_ints_string, "%s%12u\n%s%12u\n%s%12u\n%s%12u\n",
             "Profibus: ", countpb1,
             "Kbus/MPI: ", countpb2,
             "PBUS:     ", countBus3,
             "PBUS-RES: ", countInit
            );

    if (count > sizeof (soc1_ints_string)) count = sizeof (soc1_ints_string);

    strncpy (buffer, soc1_ints_string, count);

    *eof = 1;
    buffer [count - 1] = '\0';

    return strlen (buffer);
}


/****************************************************************************
 *
 * soc1_cptype_read -  handle read requests from /proc/cptype
 *
 */
static int soc1_cptype_read (char *buffer, char **start, off_t off, int count, int *eof, void *data)
{
    #ifdef CPXXX5HW_DEBUG	
    printkGen (NULL, KERN_NOTICE "----cpXXX5hw: in soc1_cptype_read ----\n");
    #endif    

    sprintf (soc1_cptype_string, "CP%d-5\n", soc1_cptype ());

    if (count > sizeof (soc1_cptype_string)) count = sizeof (soc1_cptype_string);

    strncpy (buffer, soc1_cptype_string, count);

    *eof = 1;
    buffer [count - 1] = '\0';

    return strlen (buffer);
}


/****************************************************************************
 *
 * soc1hw_init - setup the resources for memory mapping and
 *               interrupt attaching
 *               (and install an entry for a pseudo-file in the /proc directory)
 */
static int __devinit soc1hw_init (void)
{
    int ii;

    uint32_t *pIcuTrig = (uint32_t *) 0xBE200054;
    uint32_t *pIcuPrio = (uint32_t *) 0xBE200090;

    /* set up level triggered interrupts */
    *(pIcuTrig + 0) |= 0x00000020;                   /* irq 5   (localbustimeout) */
    *(pIcuTrig + 3) |= 0x00080000;                   /* irq 115 (pbus)            */

    /* set up interrupt priorities */
    *(pIcuPrio +   5) = 9;                           /* irq 5   (localbustimeout) */
    *(pIcuPrio +  17) = IRQ_TIMERTOP_0_ICU_PRIO;     /* irq 17  (krisc)           */
    *(pIcuPrio +  18) = IRQ_TIMERTOP_0_ICU_PRIO;     /* irq 18  (krisc)           */
    *(pIcuPrio + 115) = IRQ_TIMERTOP_0_ICU_PRIO - 1; /* irq 115 (pbus)            */
    *(pIcuPrio + 116) = IRQ_TIMERTOP_0_ICU_PRIO - 2; /* irq 116 (pbus reset)      */
       
    #ifdef CPXXX5HW_DEBUG	
    printkGen (NULL, KERN_NOTICE " cpXXX5hw driver is installed (in soc1hw_init )\n");
    #endif
 
    sprintf (driver_name, "CP%d-5", soc1_cptype ());
    
    printk (KERN_NOTICE "[%s] driver for is running\n", driver_name);

    if (register_chrdev (MAJOR_NUMBER, driver_name,
                         &soc1hw_fops) < 0) {
        printk (KERN_ALERT "[%s] unable to get major number\n", driver_name);
        return -1;
    }


    if (rt_request_irq (IRQ_PROFIBUS_PB1,
			(irqreturn_t (*)(int,void *)) soc1hw_pb1_interrupt,
                        //&soc1hw_pb1_interrupt,
                        0,
                        driver_name,
                        private_dev,
                        (irqreturn_t (*)(int,void *)) soc1hw_pb1_interrupt
			//&soc1hw_pb1_interrupt
			)) {
        printk (KERN_ALERT "[%s] could not allocate pb1 irq %d\n",
                driver_name, IRQ_PROFIBUS_PB1);
        return -1;
    }
    printk (KERN_NOTICE "[%s] attached pb1 irq %d\n",
            driver_name, IRQ_PROFIBUS_PB1);

    if (rt_request_irq (IRQ_PROFIBUS_PB2,
                        (irqreturn_t (*)(int,void *)) soc1hw_pb2_interrupt,
			//&soc1hw_pb2_interrupt,
                        0,
                        driver_name,
                        private_dev,
                        (irqreturn_t (*)(int,void *)) soc1hw_pb2_interrupt
			//&soc1hw_pb2_interrupt
			)) {
        printk (KERN_ALERT "[%s] could not allocate pb2 irq %d\n",
                driver_name, IRQ_PROFIBUS_PB2);
        return -1;
    }
    printk (KERN_NOTICE "[%s] attached pb2 irq %d\n",
            driver_name, IRQ_PROFIBUS_PB2);

    if (rt_request_irq (IRQ_PROFIBUS_X1_BUS3,
			(irqreturn_t (*)(int,void *)) soc1hw_int_bus3_interrupt,
                        //&soc1hw_int_bus3_interrupt,
                        0,
                        driver_name,
                        private_dev,
                        (irqreturn_t (*)(int,void *)) soc1hw_int_bus3_interrupt
			//&soc1hw_int_bus3_interrupt
			)) {
        printk (KERN_ALERT "[%s] could not allocate int_bus3 irq %d\n",
                driver_name, IRQ_PROFIBUS_X1_BUS3);
        return -1;
    }
    printk (KERN_NOTICE "[%s] attached int_bus3 irq %d\n",
            driver_name, IRQ_PROFIBUS_X1_BUS3);
    
    if (rt_request_irq (IRQ_PROFIBUS_SYSINIT,
                        (irqreturn_t (*)(int,void *)) soc1hw_sys_init_interrupt,
			//&soc1hw_sys_init_interrupt,
                        0,
                        driver_name,
                        private_dev,
                        (irqreturn_t (*)(int,void *)) soc1hw_sys_init_interrupt
			//&soc1hw_sys_init_interrupt
			)) {
        printk (KERN_ALERT "[%s] could not allocate sys_init irq %d\n",
                driver_name, IRQ_PROFIBUS_SYSINIT);
        return -1;
    }
    printk (KERN_NOTICE "[%s] attached sys_init irq %d\n",
            driver_name, IRQ_PROFIBUS_SYSINIT);

    for (ii = 0; ii < NO_OF_EVENTS; ii++) {
        /* set callback routines for valid event ids */
        if (ppdrv_ev_arr [ii].ev_id != 0) {
            ppdrv_ev_arr [ii].ev_disable = ev_disable_callback;
            ppdrv_ev_arr [ii].ev_enable = ev_enable_callback;
            ppdrv_ev_arr [ii].endisable_par = (void *) ii;
        }
    }

    if ((ev_area_id = rt_init_event_area (ppdrv_ev_arr, NO_OF_EVENTS)) < 0)
    {
        printk (KERN_ALERT "[%s] could not initialize event area\n", driver_name);
        return -1;
    }

    memset (soc1_ints_string, 0, sizeof (soc1_ints_string));
    soc1_ints_file = create_proc_entry (PSEUDO_FILENAME, 0644, NULL);

    if (soc1_ints_file != NULL)
    {
        soc1_ints_file->read_proc  = soc1_ints_read;
        soc1_ints_file->write_proc = NULL;
        soc1_ints_file->data       = NULL;
    }
    else
    {
        printk (KERN_ALERT "[%s] unable to install /proc/%s\n", driver_name, PSEUDO_FILENAME);
    }

    memset (soc1_cptype_string, 0, sizeof (soc1_cptype_string));
    soc1_cptype_file = create_proc_entry (PSEUDO_FILENAME2, 0644, NULL);

    if (soc1_cptype_file != NULL)
    {
        soc1_cptype_file->read_proc  = soc1_cptype_read;
        soc1_cptype_file->write_proc = NULL;
        soc1_cptype_file->data       = NULL;
    }
    else
    {
        printk (KERN_ALERT "[%s] unable to install /proc/%s\n", driver_name, PSEUDO_FILENAME2);
    }

    EnableEpldPbusInt ();
    
    return 0;
}


/****************************************************************************
 *
 * soc1hw_exit - driver uninstallation routine
 *
 */
static void __devexit soc1hw_exit (void)
{
    #ifdef CPXXX5HW_DEBUG	
    printkGen (NULL, KERN_NOTICE "----cpXXX5hw: in soc1hw_exit ----\n");
    #endif

    printk (KERN_NOTICE "[%s] driver removed\n", driver_name);
}


module_init (soc1hw_init);
module_exit (soc1hw_exit);
