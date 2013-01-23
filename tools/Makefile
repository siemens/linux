# kernel sources 
TOPDIR  := $(shell /bin/pwd)

ifndef KSRC
KSRC  := $(TOPDIR)/..
endif

include $(KSRC)/.config

CFLAGS  := $(EXTRA_CFLAGS)
CFLAGS  += -I$(KSRC)/arch/arm/plat-feroceon/mv_drivers_lsp/mv_cesa/ \
	   -I$(KSRC)/arch/arm/plat-feroceon/mv_drivers_lsp/mv_network/mv_ethernet/

ifdef CONFIG_CPU_BIG_ENDIAN
CC := armeb-none-linux-gnueabi-gcc
LD := armeb-none-linux-gnueabi-ld
CFLAGS  += -DCONFIG_CPU_BIG_ENDIAN
else
CC := arm-none-linux-gnueabi-gcc
LD := arm-none-linux-gnueabi-ld
endif


SUBDIRS += tsutool

TARGETS=mv_shell mv_eth_tool

ifeq ($(CONFIG_MV_GATEWAY),y)
CFLAGS += -DCONFIG_MV_GATEWAY
endif

ifeq ($(CONFIG_MV_ETHERNET),y)
CFLAGS += -DCONFIG_MV_ETHERNET
endif

ifeq ($(CONFIG_MV_CESA),y)
TARGETS += mv_cesa_tool
endif

ifeq ($(CONFIG_MV_PHONE),y)
TARGETS += mv_voip_tool
endif

ifeq ($(CONFIG_MV_INCLUDE_USB),y)
TARGETS += testusb
endif



all : Makefile $(TARGETS)
	@for subdir in $(SUBDIRS); do \
                (cd $$subdir && $(MAKE) $@) || exit 1; \
        done
	
mv_shell : MV_shell.c 
	$(CC) $(CFLAGS) MV_shell.c -o mv_shell

mv_eth_tool: mv_eth_tool.c
	$(CC) $(CFLAGS) mv_eth_tool.c -o mv_eth_tool

mv_set_coal : mv_set_coal.c
	$(CC) $(CFLAGS) mv_set_coal.c -o mv_set_coal

mv_voip_tool: mv_voip_tool.c
	$(CC) $(CFLAGS) mv_voip_tool.c -o mv_voip_tool

mv_cesa_tool:mv_cesa_tool.c
	$(CC) $(CFLAGS) mv_cesa_tool.c -o mv_cesa_tool

testusb: testusb.c
	$(CC) $(CFLAGS) testusb.c -o testusb -lpthread

clean:
	rm -f $(TARGETS) *.o
	@for subdir in $(SUBDIRS); do \
                (cd $$subdir && $(MAKE) $@) || exit 1; \
        done

