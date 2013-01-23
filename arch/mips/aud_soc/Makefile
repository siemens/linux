#
# Makefile for the AuD SOC specific kernel interface routines
# under Linux.
#

obj-y	+= soc_irq.o soc_init.o soc_memory.o soc_time.o soc_setup.o \
		soc_serial.o soc_berr.o soc_board_devices.o soc_reset.o

obj-$(CONFIG_SPI)	+= soc_spi.o soc_gpio.o
obj-$(CONFIG_PCI)	+= soc_pci.o
