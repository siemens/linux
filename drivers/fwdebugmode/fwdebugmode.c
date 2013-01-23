#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/stat.h>
#include <linux/moduleparam.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>

#define FWDEBUG_FILE "fwdebugmode"



MODULE_LICENSE("GPL");


static int fwdebugmode = 0;
static struct proc_dir_entry *fwdebugfile;



static int fwdebugmode_read(char *buffer, char **start, off_t off, int count, int *eof, void *data)
{
    if (!off && count > 0)
    {
        *buffer = '0' + fwdebugmode;
        *eof = 1;
        return 1;
    }

    *eof = 0;

    return 0;
}



static int fwdebugmode_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
    char buf;

    if (count > 0)
    {
        if (copy_from_user(&buf, buffer, sizeof(buf)))
            return 0;

        if (buf == '1')
        {
            fwdebugmode = 1;
            console_loglevel = 7;
        }
        else
        {
            fwdebugmode = 0;
            console_loglevel = 4;
        }
    }

    return count;
}



static int __init fwdebugmode_kernel(char *str)
{
    fwdebugmode = 1;
    console_loglevel = 7;
    return 0;
}



early_param("fwdebugmode", fwdebugmode_kernel);



int getFWDebugMode(void)
{
    return fwdebugmode;
}



static int kernelcrash_init(void)
{
    fwdebugfile = create_proc_entry (FWDEBUG_FILE, 0600, NULL);

    if (!fwdebugfile)
    {
        printk(KERN_ALERT "Cannot install %s\n", FWDEBUG_FILE);
        remove_proc_entry(FWDEBUG_FILE, NULL);
        return -ENOMEM;
    }

    fwdebugfile->read_proc = fwdebugmode_read;
    fwdebugfile->write_proc = fwdebugmode_write;
    fwdebugfile->data  = NULL;

	return 0;
}



static void kernelcrash_exit(void)
{
    remove_proc_entry(FWDEBUG_FILE, NULL);
}

EXPORT_SYMBOL(getFWDebugMode);

module_init(kernelcrash_init);
module_exit(kernelcrash_exit);

