# kernel sources 
TOPDIR  := $(shell /bin/pwd)

ifndef KSRC
KSRC  := $(TOPDIR)/../..
endif

include $(KSRC)/.config

CC := arm-none-linux-gnueabi-gcc
LD := arm-none-linux-gnueabi-ld

CFLAGS  := $(EXTRA_CFLAGS) -static
CFLAGS  += -I$(KSRC)/arch/arm/plat-feroceon/mv_drivers_lsp/mv_tsu/

SRCS := mv_tsu_tool.c
OBJS := mv_tsu_tool.o

TARGET := mvTsuTool

default :
	$(CC) $(CFLAGS) $(SRCS) -o $(TARGET)

clean:
	rm -f $(TARGET) $(OBJS)


all : clean default

