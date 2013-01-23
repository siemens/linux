#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/proc_fs.h>

/* Defines the license for this LKM */
MODULE_LICENSE("MRVL");

#include "../arch/arm/plat-feroceon/mv_hal/pex/mvPex.c"
#include "../arch/arm/plat-feroceon/mv_hal/prestera/hwIf/gtHwIf.c"
//#include "../arch/arm/mach-feroceon-kw/kw_family/ctrlEnv/mvCtrlEnvLib.c"

static const char proc_dir_name[] = "ppdrv";
static const char proc_write_name[] = "write";
static const char proc_read_name[] = "read";
static const char proc_test_name[] = "test";

//#define SWITCH_INTERNAL_REG_BASE 0xE8000000
#define SWITCH_INTERNAL_REG_BASE 0xF4000000

MV_U32 mvCtrlPexMaxIfGet(MV_VOID){return 1;}
MV_VOID   mvCtrlPwrClckSet(MV_UNIT_ID unitId, MV_U32 index, MV_BOOL enable) {return;}

static int convStr2Num(char* buf, int index, unsigned int *Num)
{
	char* kbuf = &buf[index];
	int  i, ii;

	Num[0] = i = ii = 0;
	while(kbuf[i] == ' ')i++;
	while(kbuf[i] != ' ')	
	{
		switch (kbuf[i]) {
		case '0'...'9':
			Num[0]  = 16 * Num[0] + (kbuf[i]-'0');
			break;
		case 'A'...'F':
			Num[0]  = 16 * Num[0] + (kbuf[i]-'A') + 10;
			break;
		    default:
			break;
		}
		i++;
		ii++;
		if( ii == 8)break;
	}
	return (index + i);
}

int read_setAddr_func(struct file *file, const char __user *buffer,
			   unsigned int count, void *data)
{
	unsigned int ppdrvAddr;
	unsigned int regVal;
	unsigned int status;
	char kbuf[8];
	int  i;

	if ( copy_from_user(kbuf, buffer, count))
	{
		printk(" failed on copy from user\n");
		return -EFAULT;
	}

	convStr2Num(kbuf, 0, &ppdrvAddr);

	status = mvSwitchReadReg(SWITCH_INTERNAL_REG_BASE, ppdrvAddr, &regVal);
	if ( status != MV_OK) 
	{
		printk( "Error: Problem reading register 0x%X, status 0x%X\n", 
			ppdrvAddr, status);
		return 0;
	}
	
	printk("Read 0x%X - 0x%X\n", ppdrvAddr, regVal);
	return count;
}

int write_SetAddrData_func(struct file *file, const char __user *buffer,
			   unsigned int count, void *data)
{
	unsigned int ppdrvAddr = 0;
	unsigned int ppdrvData = 0;
	unsigned int status;
	char kbuf[128];
	int  i= 0;

	if ( copy_from_user(kbuf, buffer, count))
	{
		printk(" failed on copy from user\n");
		return -EFAULT;
	}

	i = convStr2Num(kbuf, 0, &ppdrvAddr);
	convStr2Num(kbuf, i, &ppdrvData);

	status = mvSwitchWriteReg(SWITCH_INTERNAL_REG_BASE, ppdrvAddr, ppdrvData);
	if (status != MV_OK) 
	{
		printk( "Error: Problem writing register 0x%X, status 0x%X\n", 
			ppdrvAddr, status);
		return 0;
	}
	
	printk("Write 0x%X - 0x%X\n", ppdrvAddr, ppdrvData);
	return count;
}

int write_getStatus_func(struct file *file, const char __user *buffer,
			   unsigned int count, void *data)
{
	printk("Do Nothing - write performed on Address and Data Set\n");
	return count;
}
int read_data_func(struct file *file, const char __user *buffer,
			   unsigned int count, void *data)
{
	printk("Do Nothing - read performed on Address Set\n");
	return count;
}


int test_read_func(struct file *file, const char __user *buffer,
			   unsigned int count, void *data)
{
	unsigned int ppdrvAddr = 0;
	unsigned int ppdrvData = 0;
	unsigned int status, i;
	unsigned int regVal;
	
	ppdrvAddr = 0x50;
	status = mvSwitchReadReg(SWITCH_INTERNAL_REG_BASE, ppdrvAddr, &regVal);
	if ( status != MV_OK) 
	{
		printk( "Error: Problem reading register 0x%X, status 0x%X\n", 
			ppdrvAddr, status);
		return 0;
	}
	
	if( (regVal != 0x11ab) && (regVal != 0xab110000) )
	{
		printk( "Error: Problem reading register 0x%X, expected 0x11ab, read 0x%X\n", 
			ppdrvAddr, regVal);
		return 0;
	}


	ppdrvAddr = 0x34;
	for( i = 0; i< 100; i++)
	{
		status = mvSwitchWriteReg(SWITCH_INTERNAL_REG_BASE, ppdrvAddr, ppdrvData);
		if (status != MV_OK) 
		{
			printk( "Error: Problem writing register 0x%X, status 0x%X\n", 
				ppdrvAddr, status);
			return 0;
		}

		status = mvSwitchReadReg(SWITCH_INTERNAL_REG_BASE, ppdrvAddr, &regVal);
		if ( status != MV_OK) 
		{
			printk( "Error: Problem reading register 0x%X, status 0x%X\n", 
				ppdrvAddr, status);
			return 0;
		}
		
		if(regVal != ppdrvData)
		{
			printk( "Error: Problem reading register 0x%X, expected 0x%X, read 0x%X\n", 
				ppdrvAddr,ppdrvData, regVal);
			return 0;
		}

		ppdrvData = ppdrvData + 0x11111111;
	}	
	
	printk("Test Passed.\n");
	
	return count;
}

int test_write_func(struct file *file, const char __user *buffer,
			   unsigned int count, void *data)
{
	printk("Do Nothing - test done on entry read\n");
	return count;
}

/* Init function called on module entry */
struct proc_dir_entry *procDir;
struct proc_dir_entry *readEnt;
struct proc_dir_entry *writeEnt;
struct proc_dir_entry *testEnt;

int PPdrv_module_init( void )
{
	procDir = proc_mkdir( proc_dir_name, NULL);
	if(procDir == NULL)
	{ 
		printk(KERN_INFO "ppdrv create FAILED\n");
	}
	
	readEnt = create_proc_entry( proc_read_name,S_IFREG|S_IRUSR|S_IWUSR, procDir);
	if(procDir == NULL)
	{ 
		printk(KERN_INFO "read create FAILED\n");
	}

	writeEnt = create_proc_entry(proc_write_name,S_IFREG|S_IRUSR|S_IWUSR, procDir);
	if(procDir == NULL)
	{ 
		printk(KERN_INFO "write create FAILED\n");
	}

	testEnt = create_proc_entry(proc_test_name,S_IFREG|S_IRUSR|S_IWUSR, procDir);
	if(procDir == NULL)
	{ 
		printk(KERN_INFO "write create FAILED\n");
	}

	readEnt->read_proc = read_data_func;
	readEnt->write_proc = read_setAddr_func;

      	writeEnt->write_proc = write_SetAddrData_func;
      	writeEnt->read_proc = write_getStatus_func;

      	testEnt->write_proc = test_write_func;
      	testEnt->read_proc = test_read_func;


      	readEnt->owner = THIS_MODULE;
      	writeEnt->owner = THIS_MODULE;

 	printk(KERN_INFO "PPdrv_module_init called.  Module is now loaded.\n");
  	return 0;
}



/* Cleanup function called on module exit */

void PPdrv_module_cleanup( void )

{
	remove_proc_entry( "test", procDir );
	remove_proc_entry( "read", procDir );
	remove_proc_entry( "write", procDir );
	remove_proc_entry( "ppdrv", NULL );

  	printk(KERN_INFO "PPdrv_module_cleanup called.  Module is now unloaded.\n");
  	return;
}


/* Declare entry and exit functions */

module_init( PPdrv_module_init );
module_exit( PPdrv_module_cleanup );
