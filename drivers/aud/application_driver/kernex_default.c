/****************************************************************************
 *
 *   system level exception handler for linux
 *
 *   fe, 31-OCT-08
 *   ak, 07-March-12, updated for kernel 2.6.31
 *
 */



#include <linux/delay.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kmod.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/preempt.h>
#include <linux/proc_fs.h>
#include <linux/ptrace.h>
#include <linux/sched.h>
#include <linux/version.h>
#include <linux/vmalloc.h>
#include <asm/uaccess.h>
#if defined (__PPC__)
#include <linux/utsname.h>
#include <asm/io.h>
#endif

#if (LINUX_VERSION_CODE != KERNEL_VERSION(2,6,31))
#error incompatible kernel version
#endif


MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("kernel exception handler");
MODULE_AUTHOR("fe");

#define FNAME    "kernex"
#define CNAME    "kernex.core"
#define BUFSIZ   (1024 * 16)

#if LINUX_VERSION_CODE != KERNEL_VERSION(2,6,15)

  // CONFIG_KGDB is set by kernel option kernel hacking->KGDB
  #ifdef CONFIG_KGDB 
  extern void kgdb_breakpoint (void);
  #endif


#else
extern void set_debug_traps (void);
extern void breakpoint (void);
#endif

extern int  early_attach_other_threads (struct task_struct *primThrd);

/* this functionpointer must be installed in the kernel to get
   called in case of kernel problems */
extern void (*kernex_wait_hook)(const char *, struct pt_regs *);

/* this function must be installed by board specific sw
   to handle the cyclical activities, like blinking or pausing */
extern int board_exception_notify (char *str);

/* entries for pseudo-files in the /proc directory */
static struct proc_dir_entry *kernex_file;
static struct proc_dir_entry *kernex_core_file;

/* buffer holding the last user error details */
static char kernex_string [BUFSIZ];

static const char *delimiter  = "--------------------------------------------------------------------------------\n";

static int maxcoresize = 8000;


module_param (maxcoresize, int, S_IRUGO);

static char *p_core_begin = NULL;
static char *p_core_used  = NULL;
static char *p_core_end   = NULL;

#if defined (__PPC__)

#ifdef CONFIG_PPC64
#define REG		"%016lx"
#define REGS_PER_LINE	4
#define LAST_VOLATILE	13
#else
#define REG		"%08lx"
#define REGS_PER_LINE	8
#define LAST_VOLATILE	12
#endif

static struct regbit {
    unsigned long bit;
    const char *name;
} msr_bits[] = {
    {MSR_EE,	"EE"},
    {MSR_PR,	"PR"},
    {MSR_FP,	"FP"},
    {MSR_VEC,	"VEC"},
    {MSR_VSX,	"VSX"},
    {MSR_ME,	"ME"},
    {MSR_CE,	"CE"},
    {MSR_DE,	"DE"},
    {MSR_IR,	"IR"},
    {MSR_DR,	"DR"},
    {0,		NULL}
};

static int instructions_to_print = 16;


static int show_instructions(char *buf, int len, struct pt_regs *regs)
{
    char *bufstart = buf, *bufend = buf + len;
    unsigned long pc = regs->nip - (instructions_to_print * 3 / 4 * sizeof(int));
    int i;

    buf += snprintf (buf, bufend - buf, "Instruction dump:");

    for (i = 0; i < instructions_to_print; i++) {
        int instr;

        if (!(i % 8))
            buf += snprintf (buf, bufend - buf, "\n");

#if !defined(CONFIG_BOOKE)
        /* If executing with the IMMU off, adjust pc rather
         * than print XXXXXXXX.
         */
        if (!(regs->msr & MSR_IR))
            pc = (unsigned long)phys_to_virt(pc);
#endif

        /* We use __get_user here *only* to avoid an OOPS on a
         * bad address because the pc *should* only be a
         * kernel address.
         */
        if (!__kernel_text_address(pc) ||
            __get_user(instr, (unsigned int __user *)pc)) {
            buf += snprintf (buf, bufend - buf, "XXXXXXXX ");
        } else {
            if (regs->nip == pc)
                buf += snprintf (buf, bufend - buf, "<%08x> ", instr);
            else
                buf += snprintf (buf, bufend - buf, "%08x ", instr);
        }

        pc += sizeof(int);
    }

    buf += snprintf (buf, bufend - buf, "\n");

    return (int)(buf - bufstart);
}


static int printbits(char *buf, int len, unsigned long val, struct regbit *bits)
{
    char *bufstart = buf, *bufend = buf + len;
    const char *sep = "";

    buf += snprintf (buf, bufend - buf, "<");
    for (; bits->bit; ++bits)
        if (val & bits->bit) {
            buf += snprintf (buf, bufend - buf, "%s%s", sep, bits->name);
            sep = ",";
        }
    buf += snprintf (buf, bufend - buf, ">");

    return (int)(buf - bufstart);
}

static int kstack_depth_to_print = CONFIG_PRINT_STACK_DEPTH;


static int snprintf_stack(char *buf, int len, struct task_struct *tsk, unsigned long *stack)
{
    char *bufstart = buf, *bufend = buf + len;
    unsigned long sp, ip, lr, newsp;
    int count = 0;
    int firstframe = 1;
#ifdef CONFIG_FUNCTION_GRAPH_TRACER
    int curr_frame = current->curr_ret_stack;
    extern void return_to_handler(void);
    unsigned long addr = (unsigned long)return_to_handler;
#ifdef CONFIG_PPC64
    addr = *(unsigned long*)addr;
#endif
#endif

    sp = (unsigned long) stack;
    if (tsk == NULL)
        tsk = current;
    if (sp == 0) {
        if (tsk == current)
            asm("mr %0,1" : "=r" (sp));
        else
            sp = tsk->thread.ksp;
    }

    lr = 0;
    buf += snprintf (buf, bufend - buf, "Call Trace:\n");
    do {
        if (!validate_sp(sp, tsk, STACK_FRAME_OVERHEAD))
            return (int)(buf - bufstart);

        stack = (unsigned long *) sp;
        newsp = stack[0];
        ip = stack[STACK_FRAME_LR_SAVE];
        if (!firstframe || ip != lr) {
            buf += snprintf (buf, bufend - buf, "["REG"] ["REG"] %pS", sp, ip, (void *)ip);
#ifdef CONFIG_FUNCTION_GRAPH_TRACER
            if (ip == addr && curr_frame >= 0) {
                buf += snprintf (buf, bufend - buf, " (%pS)",
                                 (void *)current->ret_stack[curr_frame].ret);
                curr_frame--;
            }
#endif
            if (firstframe)
                buf += snprintf (buf, bufend - buf, " (unreliable)");
            buf += snprintf (buf, bufend - buf, "\n");
        }
        firstframe = 0;

        /*
         * See if this is an exception frame.
         * We look for the "regshere" marker in the current frame.
         */
        if (validate_sp(sp, tsk, STACK_INT_FRAME_SIZE)
            && stack[STACK_FRAME_MARKER] == STACK_FRAME_REGS_MARKER) {
            struct pt_regs *regs = (struct pt_regs *)
                (sp + STACK_FRAME_OVERHEAD);
            lr = regs->link;
            buf += snprintf (buf, bufend - buf, "--- Exception: %lx at %pS\n    LR = %pS\n",
                             regs->trap, (void *)regs->nip, (void *)lr);
            firstframe = 1;
        }

        sp = newsp;
    } while (count++ < kstack_depth_to_print);

    return (int)(buf - bufstart);
}
#endif


/****************************************************************************
 *
 * print_regs - print human readable string containing proc register values
 *
 */
void print_regs (struct pt_regs *regs, char *buf, int len)
{
#if defined (__mips__)
#define ROW_COUNT       8 /* MIPS has 32 registers */

#elif defined (__arm__)
#define ROW_COUNT       4 /* ARM has 16 registers */
#endif

    char *bufend = buf + len;
#if defined (__arm__) || defined (__mips__)
    int   row, col;
#endif

    buf += snprintf (buf, bufend - buf, "\n\n");
    if (buf > bufend) buf = bufend; 

#if defined (__arm__) || defined (__mips__)
    if (regs != NULL)
    {
        for (row = 0; row < ROW_COUNT; row++)
        {
            for (col = 0; col < 4; col++)
            {
                int reg = row * 4 + col;
#if defined (__arm__)
                buf += snprintf (buf, bufend - buf, "r%02d: %08lX  ", reg, regs->uregs [reg]);
#elif defined (__mips__)
                buf += snprintf (buf, bufend - buf, "r%02d: %08lX  ", reg, regs->regs [reg]);
#endif
                if (buf > bufend) buf = bufend; 
            }
            buf += snprintf (buf, bufend - buf, "\n");
            if (buf > bufend) buf = bufend; 
        }
        
        buf += snprintf (buf, bufend - buf, "\n");

#if defined (__arm__)
        buf += snprintf (buf, bufend - buf, "cpsr  : %08lx\n", regs->ARM_cpsr);
#elif defined (__mips__)
        buf += snprintf (buf, bufend - buf, "epc   : %08lx\n", regs->cp0_epc);
        buf += snprintf (buf, bufend - buf, "status: %08lx\n", regs->cp0_status);
        buf += snprintf (buf, bufend - buf, "cause : %08lx\n", regs->cp0_cause);
        buf += snprintf (buf, bufend - buf, "badva : %08lx\n", regs->cp0_badvaddr);
        buf += snprintf (buf, bufend - buf, "hi    : %08lx\n", regs->hi);
        buf += snprintf (buf, bufend - buf, "lo    : %08lx\n", regs->lo);
#endif
        buf += snprintf (buf, bufend - buf, "\n");
    }
    else
    {
       buf += snprintf (buf, bufend - buf, "register contents cannot be displayed\n");
        if (buf > bufend) buf = bufend; 
    }
#elif defined (__PPC__)
    {
        int i, trap;

#ifdef CONFIG_PREEMPT
        buf += snprintf (buf, bufend - buf, "PREEMPT ");
#endif
#ifdef CONFIG_SMP
        buf += snprintf (buf, bufend - buf, "SMP NR_CPUS=%d ", NR_CPUS);
#endif
#ifdef CONFIG_DEBUG_PAGEALLOC
        buf += snprintf (buf, bufend - buf, "DEBUG_PAGEALLOC ");
#endif
#ifdef CONFIG_NUMA
        buf += snprintf (buf, bufend - buf, "NUMA ");
#endif

        if (regs != NULL)
        {
            // taken from show_regs
            buf += snprintf (buf, bufend - buf, "NIP: "REG" LR: "REG" CTR: "REG"\n",
                             regs->nip, regs->link, regs->ctr);
            buf += snprintf (buf, bufend - buf, "REGS: %p TRAP: %04lx   %s  (%s)\n",
                             regs, regs->trap, print_tainted(), init_utsname()->release);
            buf += snprintf (buf, bufend - buf, "MSR: "REG" ", regs->msr);
            buf += printbits(buf, bufend - buf, regs->msr, msr_bits);
            buf += snprintf (buf, bufend - buf, "  CR: %08lx  XER: %08lx\n", regs->ccr, regs->xer);
            trap = TRAP(regs);
            if (trap == 0x300 || trap == 0x600)
#if defined(CONFIG_4xx) || defined(CONFIG_BOOKE)
                buf += snprintf (buf, bufend - buf, "DEAR: "REG", ESR: "REG"\n", regs->dar, regs->dsisr);
#else
            buf += snprintf (buf, bufend - buf, "DAR: "REG", DSISR: "REG"\n", regs->dar, regs->dsisr);
#endif
            buf += snprintf (buf, bufend - buf, "TASK = %p[%d] '%s' THREAD: %p",
                             current, task_pid_nr(current), current->comm, task_thread_info(current));

#ifdef CONFIG_SMP
            buf += snprintf (buf, bufend - buf, " CPU: %d", raw_smp_processor_id());
#endif /* CONFIG_SMP */

            for (i = 0;  i < 32;  i++) {
                if ((i % REGS_PER_LINE) == 0)
                    buf += snprintf (buf, bufend - buf, "\nGPR%02d: ", i);
                buf += snprintf (buf, bufend - buf, REG " ", regs->gpr[i]);
                if (i == LAST_VOLATILE && !FULL_REGS(regs))
                    break;
            }
            buf += snprintf (buf, bufend - buf, "\n");
#ifdef CONFIG_KALLSYMS
            /*
             * Lookup NIP late so we have the best change of getting the
             * above info out without failing
             */
            buf += snprintf (buf, bufend - buf, "NIP ["REG"] %pS\n", regs->nip, (void *)regs->nip);
            buf += snprintf (buf, bufend - buf, "LR ["REG"] %pS\n", regs->link, (void *)regs->link);
#endif
            /* go and print the stack */
            buf += snprintf_stack(buf, bufend - buf, current, (unsigned long *) regs->gpr[1]);
            if (!user_mode(regs))
                buf += show_instructions(buf, bufend - buf, regs);
        }
        else
        {
            buf += snprintf (buf, bufend - buf, "register contents cannot be displayed\n");
            if (buf > bufend) buf = bufend; 
        }
    }
#else
    buf += snprintf (buf, bufend - buf, "register contents cannot be displayed\n");
    if (buf > bufend) buf = bufend; 
#endif

    buf += snprintf (buf, bufend - buf, "\n\n");
    if (buf > bufend) buf = bufend; 
}


/****************************************************************************
 *
 * kernex_wait_loop
 *  - loop in exception handling, until the user wants us to
 *                   continue
 *
 */

 void kernex_wait_loop (void)
{


    
    while (board_exception_notify (kernex_string))
    {
/*

#ifdef CONFIG_DETECT_SOFTLOCKUP
        touch_softlockup_watchdog
 ();
#endif
*/
	;	

    }

;
}


/****************************************************************************
 *
 * kernex_handle_user_event - handling for user-triggered exceptions
 *
 */
static void kernex_handle_user_event (int threadattach)
{
    unsigned long flags;

    if (threadattach)
    {
        /* put all other threads in a sleeping state waiting for the debugger */
        early_attach_other_threads (current);
    }
    
    /* save and disable interrupts */

#if CONFIG_RTX_DOMAIN
    local_irq_save_hw (flags);
#else
    local_irq_save (flags);
#endif

    /* loop till somebody wants to debug */
    kernex_wait_loop ();

    /* restore interrupt status */
#if CONFIG_RTX_DOMAIN
    local_irq_restore_hw (flags);
#else
    local_irq_restore (flags);
#endif

}


extern void show_regs_direct(struct pt_regs *regs);
extern void show_trace_direct(struct pt_regs *regs);


/****************************************************************************
 *
 * kernex_handle_kernel_event - handling for kernel-triggered exceptions
 *
 */
static void kernex_handle_kernel_event (const char *str, struct pt_regs *regs)
{
    strncpy (kernex_string, str, sizeof (kernex_string));

    print_regs (regs,
                kernex_string + strlen (kernex_string),
                sizeof (kernex_string) - strlen (kernex_string));

    kernex_wait_loop ();

    /* now wait for a gdb frontend to be attached */
    printk (KERN_ALERT "starting KGDB ...\n");

#if LINUX_VERSION_CODE != KERNEL_VERSION(2,6,15)

  // CONFIG_KGDB is set by kernel option kernel hacking->KGDB
  #ifdef CONFIG_KGDB 
  kgdb_breakpoint ();
  #endif
    
#else
    set_debug_traps ();
    breakpoint ();
#endif
  
    /* remove the hook, to avoid getting trapped again due to the same problem */

  kernex_wait_hook = NULL;

}


/****************************************************************************
 *
 * kernex_read - handle read requests from /proc/kernex
 *
 */
static int kernex_read (char *buffer, char **start, off_t off, int count, int *eof, void *data)
{
    if (count > sizeof (kernex_string)) count = sizeof (kernex_string);

    strncpy (buffer, kernex_string, count);

    *eof = 1;
    buffer [count - 1] = '\0';
    
    return strlen (buffer);
}


/****************************************************************************
 *
 * kernex_write - handle write requests to /proc/kernex
 *
 */
static int kernex_write (struct file *file, const char *buffer, unsigned long count, void *data)
{
    if (count > sizeof (kernex_string)) count = sizeof (kernex_string);

    if (copy_from_user (kernex_string, buffer, count))
    {
        /* if the string cannot be accessed, continue with a default string to
           keep the exception processing going on */
        strcpy (kernex_string, "<exception description not available>\n");
    }

    /* when kernex is written, this is a signal for an exception in user space so we
       start reporting this event cyclically while the kernel is locked */
    kernex_handle_user_event (1);
    return count;
}


/****************************************************************************
 *
 * kernex_core_read - handle read requests from /proc/kernex.core
 *
 */
static int kernex_core_read (char *buffer, char **start, off_t off, int count, int *eof, void *data)
{
    char *p_core_read = p_core_begin + off;
    
    if (count > (p_core_used - p_core_read)) count = (p_core_used - p_core_read);
    memcpy (buffer, p_core_read, count);

    *eof = (count == 0);
    *start = buffer;
    
    return count;
}


/****************************************************************************
 *
 * kernex_core_write - handle write requests to /proc/kernex.core
 *
 */
static int kernex_core_write (struct file *file, const char *buffer, unsigned long count, void *data)
{
    char *p_core_write = p_core_begin + file->f_pos;

    /* the first write access to the corefile triggers kernel exception handling */
    if (p_core_write == p_core_begin)
    {
        snprintf (kernex_string,
                  sizeof (kernex_string),
                  "%s"
                  "  application corefile dumped\n"
                  "%s"
                  "  process : %d (%s)\n"
                  "%s"
                  "  coredump available in /proc/%s\n"
                  "%s",
                  delimiter,
                  delimiter,
                  current->pid,
                  current->comm,
                  delimiter,
                  CNAME,
                  delimiter);

        kernex_handle_user_event (0);

        memset (p_core_begin, 0, p_core_end - p_core_begin);
    }

    if (count > (p_core_end - p_core_write)) count = (p_core_end - p_core_write);
    if (copy_from_user (p_core_write, buffer, count))
    {
        return -EFAULT;
    }

    file->f_pos  += count;
    p_core_write += count; 
    if (p_core_write > p_core_used) p_core_used = p_core_write;
    
    return count;
}


/****************************************************************************
 *
 * kernex_init - driver installation routine
 *
 */
static int kernex_init(void)
{

	printk (KERN_NOTICE " kernex driver is installed\n");


    kernex_wait_hook = kernex_handle_kernel_event;

    memset (kernex_string, 0, sizeof (kernex_string));
    

#ifndef CONFIG_PROC_FS
    printk (KERN_ALERT "procfs not available in kernel\n");

    return (-1);
#endif

    kernex_file = create_proc_entry (FNAME, 0644, NULL);
    if (!kernex_file)
    {
        printk (KERN_ALERT "unable to install %s\n", FNAME);
        remove_proc_entry (FNAME, NULL);
        return (-ENOMEM);
    }
        
    kernex_core_file = create_proc_entry (CNAME, 0644, NULL);
    if (!kernex_core_file)
    {
        printk (KERN_ALERT "unable to install %s\n", CNAME);
        remove_proc_entry (FNAME, NULL);
        remove_proc_entry (CNAME, NULL);
        return (-ENOMEM);
    }

    kernex_file->read_proc  = kernex_read;
    kernex_file->write_proc = kernex_write;
    kernex_file->data  = NULL;
#if (LINUX_VERSION_CODE == KERNEL_VERSION(2,6,15))
    kernex_file->owner = THIS_MODULE;
#endif
    
    kernex_core_file->read_proc  = kernex_core_read;
    kernex_core_file->write_proc = kernex_core_write;
    kernex_core_file->data  = NULL;
#if (LINUX_VERSION_CODE == KERNEL_VERSION(2,6,15))
    kernex_core_file->owner = THIS_MODULE;
#endif

    p_core_begin = vmalloc (maxcoresize);
    if (!p_core_begin)
    {
        printk (KERN_ALERT "cannot allocate coredump buffer\n");
        remove_proc_entry (FNAME, NULL);
        remove_proc_entry (CNAME, NULL);
        return (-ENOMEM);
    }
    p_core_used = p_core_begin;
    p_core_end  = p_core_begin + maxcoresize;
    
    printk (KERN_NOTICE "kernex driver is running\n");
    
    return 0;
}


/****************************************************************************
 *
 * kernex_exit - driver uninstallation routine
 *
 */
static void kernex_exit(void)
{
    vfree (p_core_begin);
    
    remove_proc_entry (FNAME, NULL);
    remove_proc_entry (CNAME, NULL);


  kernex_wait_hook = NULL;


    printk (KERN_ALERT "kernex driver removed\n");
}


module_init(kernex_init);
module_exit(kernex_exit);
