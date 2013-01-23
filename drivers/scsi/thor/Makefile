# Makefile for Marvell driver - non raid
#
# V0.0.0.5 Ver.
# A.C. <ake at marvell dot com>

# default to build for the running kernel
ifeq ("x", "x$(KERNEL_SRC)")
	KERNEL_SRC=/lib/modules/$(shell uname -r)/build
endif

# use KERNEL_SRC if not called by Mahoro
ifeq ("x", "x$(KERNEL_SOURCE_DIR)")
        KERNEL_SRC_DIR=$(KERNEL_SRC)
else
	KERNEL_SRC_DIR=$(KERNEL_SOURCE_DIR)
endif

ifeq ("x", "x$(KERNEL_BUILD_DIR)")
	KERNEL_BLD_DIR=$(KERNEL_SRC_DIR)
else
        KERNEL_BLD_DIR=$(KERNEL_BUILD_DIR)
endif

CONFIG_AMD64=$(shell [ -f $(KERNEL_BLD_DIR)/.config ] && cat $(KERNEL_BLD_DIR)/.config | grep -m 1 CONFIG_X86_64 | awk -F= '{print $$2}')

MV_CC     =  $(CROSS_COMPILE)$(CC)
MV_LD     =  $(CROSS_COMPILE)$(LD)

ifeq ($(ARCH), )
ifeq ($(strip $(CONFIG_AMD64)),y)
	ARCH_TYPE=x86_64
	RAID_LIB_NAME=libmv$(LIB_TYPE)64
else	
	ARCH_TYPE=i386 
	ifeq ($(strip $(CONFIG_REGPARM)),y)
		RAID_LIB_NAME=libmv$(LIB_TYPE)32
	else
		RAID_LIB_NAME=libmv$(LIB_TYPE)32_noregparm
	endif
endif
else
ARCH_TYPE=$(ARCH)
endif

CORE_OBJS       := core/core_exp.o core/core_xor.o\
		   core/core_api.o core/core_init.o core/scsi2sata.o

COMM_OBJS       := common/com_util.o \
		   common/com_u64.o common/com_scsi.o common/com_tag.o

OSDEP_OBJS      := linux/linux_sense.o  linux/linux_helper.o   \
		   linux/hba_mod.o      linux/hba_timer.o \
		   linux/hba_exp.o      linux/linux_iface.o \
		   linux/linux_main.o

OSD_OBJS        := $(OSDEP_OBJS) $(CORE_OBJS) $(COMM_OBJS) 

INCLUDE_DIR     = -I$(KERNEL_BLD_DIR)/include \
		  -I$(KERNEL_BLD_DIR)/include/scsi \
		  -I$(KERNEL_BLD_DIR)/drivers/scsi \
	 	  -I$(KERNEL_SRC_DIR)/include \
	 	  -I$(KERNEL_SRC_DIR)/include/scsi \
	 	  -I$(KERNEL_SRC_DIR)/drivers/scsi


ifneq ($(KERNELRELEASE),)
obj-m         :=   mv61xx.o
mv61xx-objs   :=   $(OSD_OBJS) 
clean-files   +=   Modules.symvers
clean-files   +=   Module.symvers
EXTRA_CFLAGS  :=   -I$(src)/common -I$(src)/core \
		   -I$(src)/linux  -I$(src)/. 

EXTRA_CFLAGS  +=   $(INCLUDE_DIR)

EXTRA_CFLAGS  +=   -include $(src)/mv_config.h

#EXTRA_CFLAGS  +=   -D__MV_DEBUG__

EXTRA_CFLAGS  += -D__AC_DBG__

else

ifeq ($(SUPPORT_RAID), y)
CFLAGS += -DRAID_DRIVER=1
endif

all:
	$(MAKE) ARCH=$(ARCH_TYPE) CC=$(MV_CC) LD=$(MV_LD) CROSS_COMPILE=$(CROSS_COMPILE) V=$(V) -C $(KERNEL_BLD_DIR) M=`pwd` modules

kbuild:
	-@cat patch.kbuild | patch -p1 -N -d $(KERNEL_SRC_DIR)/drivers/scsi
	@./patch.sh $(KERNEL_SRC_DIR)/drivers/scsi a
	@mkdir -p $(KERNEL_SRC_DIR)/drivers/scsi/mv
	@cp -r * $(KERNEL_SRC_DIR)/drivers/scsi/mv
	@cp -f Makefile.kbuild $(KERNEL_SRC_DIR)/drivers/scsi/mv/Makefile
	@echo done.

ukbuild:
	-@cat patch.kbuild | patch -p1 -N -R -d $(KERNEL_SRC_DIR)/drivers/scsi
	@./patch.sh $(KERNEL_SRC_DIR)/drivers/scsi
	@rm -rf $(KERNEL_SRC_DIR)/drivers/scsi/mv
	@echo Patch has been successfully rewinded.
	
clean:  
	$(MAKE) V=$(V) -C $(KERNEL_BLD_DIR) M=`pwd` clean
	@rm -rf *.o *.obj
endif
