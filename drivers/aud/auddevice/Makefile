#
# Makefile for the AuD realtime device driver
#
#
obj-$(CONFIG_RTX_DOMAIN)	:= auddriver.o
auddriver-y := rt_device.o rt_io_event.o rt_sync_timer.o rt_fs_op.o rt_base.o rt_timer.o rt_semaphore.o \
		rt_sched.o rt_signal.o rt_thread.o rt_futex.o rt_mqueue.o rt_tgroup.o \
		rt_debug.o
obj-$(CONFIG_RTX_TRACE)  += rt_tracer.o
