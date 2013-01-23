
contents
========

major modifications
how to use




list of major changes
=====================

01-Oct-2012
c.w. set Label AUDIS_(ARCH/RT/FULL/NRK)-V02.11

28-Sep-2012
m.n. add ftrace hooks for mips systemcalls in lx and rt
(arch/mips/kernel/entry.S and scall32-o32.S)
m.n. enhance general ftrace code to support rt system call tracing
(kernel/trace/trace.c; trace_syscalls.c)

28-Sep-2012
a.k. system call "sys_socket" in arch/mips/kernel/rtx_calls.S now with execute_in_lx

27-Sept-2012
h.b. drivers/mtd/chips/cfi_cmdset_0002.c: take over the task_list initialization from 2.6.15 (m.n.)

27-Sep-2012
h.b. take over from 2.6.15
h.b drivers/mtd/chips/cfi_cmdset_0002.c: it's unsafe to
use jiffies for timeouts, if Realtime is configured
(Buffer I/O error on device mtdblock)

26-Sep-2012
w.h. arch/mips/include/asm/siginfo.h, kernel/signal.c, linux/sched.h, 
asm-generic/siginfo.h, rt_internal.h rt_internal2.h, rt_device.c, 
rt_io_event.c, rt_signal.c: Bugfixes for timer/events and pthread_kill(). 
The si_ptr member (struct sigval) was not set in the case of SIGEV_SIGNAL_SV. 
For pthread_kill() si_threadid was not defined (Note: glibc must also be adjusted).

25-Sept-2012
a.k. HR-Timer for mips: arch/mips/kernel/rtx_adapt_32.c cevt-r4k.c: use values for
mips soc1 boards from arch/mips/include/asm/mach-aud_soc/socGen.h

24-Sept-2012
a.k. - binutils, gcc-4.2, glibc, linux-kernel and gdb can be build with multicore-support, see how to use
- new config target cm1542
- reboot for SOC1 (arch/mips/aud_soc/soc-reset)
- SOC1 internal pullups/pulldown set for spi lan connection with ks8851
(arch/mips/include/asm/mach-aud_soc/soc_pullcontrol.h, arch/mips/aud_soc/soc_spi.c)

24-Sep-2012
a.k. new config target cm1542
a.k. reboot for SOC1 (arch/mips/aud_soc/soc-reset)
a.k. SOC1 internal pullups/pulldown set for spi lan connection with ks8851
(arch/mips/include/asm/mach-aud_soc/soc_pullcontrol.h, arch/mips/aud_soc/soc_spi.c)

21-Sep-2012
w.h. kernel/sched.c, rt_sched.h, rt_internal2.h, rt_base.c, rt_device.c,
rt_fs_op.c, rt_io_event.c, rt_sync_timer.c, rt_timer.c: Adjustments for the
restricted system-calls sched_setscheduler() and sched_setaffinity().

19-Sep-2012
w.h. rt_internal.h, rt_sched.c: In some special cases the audis_return_to_linux()
function may return the value 0 (if the thread has been continued by some RT event).
This case now returns R2L_RET_SIG_WAIT_INTERRUPTED instead of 0.

rt_base.h, rt_device.c, rt_fs_op.c, fs/read_write.c: Now printf() messages of
RT- and NRT-threads are also prefixed by pid and cpu (in case of SMP). 
The format of printkGen() kernel messages has been changed accordingly.
The first prefix of kernel messages (printkGen) is:
LXK resp. RTK: The printkGen() was mapped to printk()
LXD resp. RTD: The printkGen() was mapped to the write daemon
The first prefix of user messages (printf) which are mapped to the write daemon is:
LX1 resp. LX2: NRT-thread is calling a printf() to stdout resp. stderr
RT1 resp. RT2: RT-thread is calling a printf() to stdout resp. stderr

19-Sep-2012
m.n. mips: resolve issue with Rt interrupt handler (not running in RT): when
starting realtime process run affinity code for all RT handler (in 
arch/mips/kernel/rtx_adapt_32.c)

17-Sep-2012
m.n. drivers/aud/rt_sched.c: sched_yield: stop small timeouts immediately
when no rt thread runs in lx

13-Sep-2012
w.h. rt_timer.h: IT_INTERVAL_MIN is used to check the reload value of a
timer. Its value has been changed to 10 usec.

12-Sep-2012
w.h. rt_timer.h, rt_base.c, rt_device.c, rt_timer.c, rt_sync_timer.c:
Type of it_period changed from 32-bit to 64-bit. This bugfix now prevents
invalid reload values for CLOCK_REALTIME/MONOTONIC timers (it_interval) 
rt_sync_timer.c: Initialize action_par in get_tmr(). This bugfix prevents
a wrong action_par if the timer has been used by register_clock() and
later on it will be re-used by a CLOCK_REALTIME/MONOTONIC timer.

rt_sched.h, rt_futex.h, rt_futex.c, rt_thread.c: Rename the function
rt_fixup_owner_priority() (=> rt_fixup_owner_prio()). For PI-mutexes
a deadlock detection checks the priority boosting of a PI-mutex lock. In case
of a deadlock it returns EBUSY. For this fix the glibc (pthread_mutex_lock())
also has to be changed. Bugfix for PI-mutex unlock. Now a possible
error will be returned to the application.

12-Sep-2012
m.n. arch/mips/kernel/scall32-o32.S and entry.S: we missed some code to
enable linux interrupts
drivers/aud/auddevice/rt_debug.c: we have a new subroutine
forceUserBreakFromKernel to allow a debug stop within the kernel

07-Sep-2012
m.n. arch/mips/kernel/scall32-o32.S: enable linux interrupts when doing 
linux system calls.

06-Sep-2012
set Label AUDIS_(ARCH/RT/FULL/NRK)-V02.10.01

05-Sep-2012
m.n. powerpc high res timer: correct problem when the 64-bit time counter
overflows (arch/powerpc/kernel/rtx_adapt_32.c)
m.n. drivers/aud/application_driver/pbus_plus.c: consider different endian
setup in the various architectures.

05-Sep-2012
w.h. rt_internal.h, rt_signal.c, rt_sched.c: The return values for the 
audis_return_to_linux() have been changed and are now bit-based.

04-Sep-2012
m.n. mips: allow CONFIG_PREEMPT configuration option, update default 
configurations

31-Aug-2012
m.n. resolve speed problem with ks8851 spi lan driver
(arch/mips/include/asm/mach-aud_soc/soc_spi.h, drivers/spi/aud_soc_spi.c,
drivers/net/ks8851.c)
arch/powerpc/kernel/rtx_adapt_32.c: call_rt_domain: disable interrupts when
leaving the rt handler; set_affinity: enable interrupts for error return

30-Aug-2012
m.n. kernel/hung_task.c: prevent hung task message also with rt threads
in the running state

23-Aug-2012
m.n. arch/(mips - powerpc)/kernel/rtx_adapt_32.c: __rtx_wake_lx_handling():
restore old interrupt flags when leaving

22-Aug-2012
m.n. update of drivers/aud/application_driver/pbus_plus.c

22-Aug-2012
m.n.merge error correction for thread groups in kernel/fork.c from RT branch
to FULL branch (was forgotten and caused problems with SC CI applications).

21-Aug-2012
m.n. arch/mips/kernel/rtx_adapt_32.c cevt-r4k.c: use separate values for
mips soc1 boards

21-Aug-2012
h.b. arch/arm/kernel/rtx_calls.S: redirect the systemcalls 281-297 (socket)
to linux
h.b. arch/arm/kernel/entry-common.S: test after systemcall, whether a
realtime task is forced to migrate to linux (e.g., prio change)

09-Aug-2012
w.h. include/trace/events/sched.h: Bugfix - This file could not be
compiled without setting CONFIG_RTX_DOMAIN.

06-Aug-2012
set Label AUDIS_(ARCH/RT/FULL/NRK)-V02.10

06-Aug-2012
m.n. update default configurations for mips-cp1500, mips-cp443 and
x86-embedded-ctrl for tracepoint support

04-Aug-2012
m.n. add realtime tracepoint support: include/trace/events/irq.h, sched.h
kernel/trace/trace_sched_switch.c, trace_clock.d
drivers/aud/auddevice/rt_sched.c,
architecture specific files kernel/rtx_adapt_32.c

drivers/aud/auddevice/rt_debug.c: some enhancements

architecture specific files mm/fault.c: pagefault: 
prevent migration back to rt when forced to be debugged.

02-Aug-2012
w.h. rt_tgroup.h, rt_sched.h, rt_base.c, rt_device.c, rt_sched.c,
rt_tgroup.c, fork.c: Bugfix (thread groups) - When returning to LX,
the interval timer was not set in any case. As a consequence the group
timer fired in the context of the LX-domain and the system crashed 
because of a BUG_ON(). A second problem was resolved: When a thread
migrates initially from LX to RT then do not initialize the thread
group variables (rt_thread_group, rt_thread_list, rt_blocked_list)
in any case. This initialization is only requested in the context
of a fork().

rt_timer.c: Bugfix (timer_gettime) - For expired timers the returned
value was not correct in case of timers handled by a carrier thread.

30-Jul-2012
w.h. rt_base.c, rt_sched.c, rt_thread.c: Bugfix (shutdown) - Do not 
wakeup threads which are blocked at rt_tl_sem in rt_task_leave() resp. 
sys_rt_exit_group(). Otherwise a deadlock may occur concerning the
rt_tl_sem semaphore.

26-Jul-2012
m.n. arch/mips/kernel/rtx_adapt_32.c: add code for rtx_spin_lock_irqsave /
restore
arch/powerpc/kernel/rtx_adapt_32.c: add a 64-bit-overflow counter for high
res timer.
add code to support PCI memory access from PCI device to cpu memory for
MIPS SOC: arch/mips/aud_soc/soc_pci.c
get common code for mips and powerpc:
    drivers/aud/application_driver/pbus_plus.c 
adjust for powerpc: arch/powerpc/sysdev/indirect_pci.c

25-Jul-2012
a.k. added support for GDB-server hardware-watchpoints for PowerPC
in arch/powerpc/kernel/ptrace.c

13-Jul-2012
w.h. rt_semaphore.c: Bugfix (RT semaphore) - prevent loss of wakeups 
for down_rt().

10-Jul-2012
m.n. set Label AUDIS_(ARCH/RT/FULL/NRK)-V02.09

10-Jul-2012
h.b. add support for freescale cortex-a9 evaluation board (cortex)
     realtime and high resolution timers

10-Jul-2012
h.b. add support for freescale cortex-a9 evaluation board (cortex)

10-Jul-2012
m.n. update mips default configurations for high res support

09-Jul-2012
w.h. kernel/rtx_adapt.c: Now the definition RTX_MAX_DELTA_NS (to be defined
in arch/<architecture>/include/asm/rtx_base.h) is used to set an
architecture-depend value for the maximum delta of the oneshot timer in
case of high resolution timers.

arch/<architecture>/kernel/rtx_adapt_32.c: Bugfix in case of a SMP-system
when playing the Linux interrupts (__rtx_handle_nonrt_pending()). Delete
the bit in the high level bit-mask only if the low level bitmask is empty.

04-Jul-2012
a.k. added support for GDB-server hardware-watchpoints for MIPS SOC
in arch/mips/kernel/ptrace.c
in arch/mips/kernel/watch.c

27-Jun-2012
m.n. drivers/aud/auddevice: provide better integration between extended
debug and watchdog support (rt_debug.c)

18-Jun-2012
m.n.add high res timer support for MIPS SOC based on compare register:
introduce SOC MIPS Kconfig options for high res timer (arch/mips/Kconfig
and arch/mips/aud_soc/Kconfig)
in arch/mips/kernel: rtx_adapt_32.c, traps.c, cevt-r4k.c
in arch/mips/include/asm: irq_regs.h, rtx_timer.h, rtx_adapt.h,
   mach-aud_soc/irq.h
in include/linux/aud: rt_timer.h
in arch/mips/aud_soc: soc_irq.c soc_time.c

m.n. overtake changes "illegal exception" in arch/mips/include/asm/mm_context.h

31-May-2012
w.h. kernel/signal.c: Bugfix for ALLOW_POSIX_TIMER_OVERRUN and SIGEV_SIGNAL.
A cyclic timer didn't work correctly in this case (it fired only once).

drivers/aud/Kconfig.debug, rt_base.h, rt_fs_op.c: The size of the print buffer
is now handled dynamically. There is a config option RTX_PRINT_BUF_LEN_KB which
specifies the size of the print buffer in KB (default is 16 KB). Additionally
this option can be overwritten by the boot parameter "audis_print_buf_len="
which also specifies the size of the print buffer in KB.

Major fixes for sync-timers: rt_timer.h, rt_base.h, rt_driver.h, rt_link.h,
rt_signal.c, rt_device.c, rt_timer.c, rt_sync_timer.c.
Carrier thread handling added. Disarm a sync-timer while the sync clock is active. 
The CLOCK_SYNC_SOFT functionaltity removed which is not part of the A&D-API. 
Some minor A&D-API adaptions.

kernel/posix-timers.c: Bugfix - in soft mode the timer_delete_hook() function
may return an error for a sync-timer. 

30-May-2012
m.n. drivers/aud/auddevice/rt_debug.c and rt_sched.c: changes to support 
multicore extended debug 

21-May-2012
m.n. set Label AUDIS_(ARCH/RT/FULL/NRK)-V02.08

18-May-2012
a.k. add SPI support for MIPS SOC (drivers/spi/aud_soc_spi.c)
integrate ks8851 to cp1626 board (drivers/net/ks8851.c)
provide device extensions for spi and spi device (arch/mips/aud_soc/soc_spi.c)

15-May-2012
h.b. add support for cp1243-1 (ARM based)

14-May-2012
m.n. error corrections for drivers/aud/auddevice/rt_debug.c
extend functions used with execute_in_lx from rt (mips and powerpc)

11-May-2012
w.h. rt_sched.h, arch/x86/kernel/rtx_adapt_32.c, arch/x86/mm/fault.c:
Bugfix for exception handling in LX. We must not migrate back to RT
if the exception was initiated in LX (e.g. a RT-thread migrates
temporarily to LX and causes an exception in LX).
This was also updated in the architecture specific files on arm, mips
and powerpc

09-May-2012
m.n. powerpc: arch/powerpc/kernel/vdso.c and vdso32/gettimeofday.S:
do a direct system call with RTX_DOMAIN else we may starve in RT 
polling for a thread blocking these library functions in lx

07-May-2012
w.h. rt_timer.c: Bugfix - In case of an error timer_delete() now returns 
this error.
rt_sync_timer.c: Bugfix - If the timer group is active timer_delete() 
now returns EBUSY. clock_settime() now checks whether the time value is
equal to the period of the timer group.

05-May-2012
m.n. add scripts to build a mips-v42eb kernel

04-May-2012
m.n. fs/proc/array.c: add rt_state3 as additional output for task state

04-May-2012
w.h. rt_futex.c, rt_timer.c, rt_mqueue.c, rt_signal.c, rt_link.h:
Timeouts are now handled at ISR-level instead of the timer daemon
to avoid priority inversion.

arch/x86/kernel/rtx_syscall_table_32.S: Systemcalls which are defined
by the A&D-API are now executed in Linux (previously not implemented):
chdir, chmod, mount, utime, access, rename, mkdir, rmdir, times, umount,
fstatfs, uname, getdents, select, readv, writev, fdatasync, getcwd.

03-May-2012
m.n. arch/powerpc/kernel/entry_32.S, drivers/aud/auddevice/rt_debug.c
improve code to support non-extended debugging
drivers/aud/application_driver/pbus_plus.c: include add-ons from KA
for powerpc and mips; update cp15431 default configuration

23-Apr-2012
m.n. drivers/aud/application_driver/Kconfig: pbus_plus driver depends on PCI

17-Apr-2012
w.h. rt_timer.c: Bugfix in sys_rt_timer_create(). This function returned 0
if clock->timer_create() returned an error.

12-Apr-2012
m.n. set Label AUDIS_(ARCH/RT/FULL/NRK)-V02.07

10-Apr-2012
w.h. kernel/sched.c, include/linux/aud/rt_sched.h: Now provides a better
time accounting for RT-threads.
m.n.arch/mips: use a flag to pass the state of a task (user/kernel mode) from
rtx_handle_irq to the linux timer interrupt routine.
h.b. add support for bad block table in flash
(soc1Nand.c / CONFIG_MTD_NAND_SOC1_FLASH_BBT)
add config-option for flash name (mtd-id / CONFIG_MTD_NAND_SOC1_FLASH_NAME)
m.n. provide default flash partition configuration for cp1626 eval board
and cp1500 board

03-Apr-2012
h.b. add the ethy driver drivers/net/aud_lan/ethy/

02-Apr-2012
m.n. use CONFIG_EXTENDED_COMMAND_LINE length option in architecture 
specific setup.h file

30-Mrch-2012
m.n. drivers/aud/auddevice/rt_sched.c: mark_debug: consider smp when
calling stopAllRtThreads.

29-Mrch-2012
m.n. enhance soc1 setup code to detect and work with the SOC1 pci interface.

28-Mrch-2012
m.n. enhance drivers/serial/serial_aud_soc.c for (rt) interrupts

23-Mrch-2012
m.n. set Label AUDIS_(ARCH/RT/FULL/NRK)-V02.06

m.n. mips: add NAND flash driver for soc
m.n. provide an extended debug and watchdog handling for RT
introduce rt_state3 and special flags (changes in architecture specific
files rtx_adapt_32.c, entry_32.S); changes in rt_base.c and rt_sched.c
add file drivers/aud/auddevice/rt_debug.c

m.n. merge kernel/ptrace.c and include/linux/ptrace.h to the rt-branch level.
else these changes are not seen by the NRK releases.

16-Mar-2012
a.k. included driver kernex, boardex,runfw,cpXXX5hw for cp342 and cp443 and
updated to 2.6.31

13-Mrch-2012
m.n. arch/x86/configs: nrk def config: add rootfs and thread_group support

12-Mrch-2012
m.n. kernel/hung_task.c: don't test realtime threads.

09-Mar-2012
w.h. rt_sched.c, rt_signal.c, rt_device.c, rt_base.h, rt_internal.h, rt_sched.h: 
rt_sched_return_to_lx() now provides a parameter 'flag' with parameters 
(R2L_FORCE, R2L_THREAD, R2L_PROCESS) specifying the conditions for a voluntary
return to Linux. The funtion now returns R2L_RET_PENDING_THIS, 
R2L_RET_PENDING_OTHER, R2L_RET_THIS or R2L_RET_OTHER to indicate the reason this
function returned. CTRL+C handling added.

06-Mar-2012
w.h. Bugfix return-to-linux (rt_base.h, rt_sched.h, rt_signal.c, rt_sched.c):
If a thread returned voluntarily to linux, each signal sent to a thread of
the RT-process makes the RT-domain running again.

06-Mrch-2012
m.n. arch/powerpc/configs/cp15431_defconfig:
set default stacksize to 0x80000 to allow application from Khe to run

01-Mar-2012
w.h. Bugfix rt_timer.c: The function timer_settime() didn't work correctly
for CLOCK_MONOTONIC timers.

29-Feb-2012
m.n. add low level PCI support for MIPS SOC
define a more generic pbus-plus-driver in drivers/aud/application_driver

27-Feb-2012
h.b. LTTng flight-recorder mode: send the streams to the customer deamon
     only on trace stop (-> overwrite buffers)
     - ltt/ltt-relay-vfs.c 

22-Feb-2012
w.h. Bugfix kernel/trace/trace.c, trace_functions.c: Exchange declaration
and definition of the core local variable rt_domain_trace. Otherwise, a specific
combination of tracer options may cause the kernel linking to fail.

21-Feb-2012
w.h. Bugfix arch/x86/mm/fault.c: Ipipe patch adopted to prevent
booting problems with grub2.

20-Feb-2012
m.n. arch/mips/aud_soc/soc_irq.c: irqdispatch(): path actual interrupted
register set to rtx_handle_irq().

16-Feb-2012
m.n. set Label AUDIS_(ARCH/RT/FULL/NRK)-V02.05

15-Feb-2012

h.b. add support for cp1626 (mips; soc1)

m.n. update files from Lars arch/powerpc/boot/dts/cp15431.dts,
arch/powerpc/configs/cp15431_defconfig, arch/powerpc/platforms/83xx/834x_mds.c

14-Feb-2012

m.n. arch/mips/kernel entry.S and scall32-o32.S: improve ret_from_fork
and system call entry to pass test suite

a.k. set label x686_secle_audis

extension of kernel-OCF-framework: 
(sources by A.Meisinger, 'secle_audis')
files added in /audis_dev/src/pkgs/linux/crypto/ocf
patch of /audis_dev/src/pkgs/linux/fs/fcntl.c: export 'sys_dup'
patch of /audis_dev/src/pkgs/linux/kernel/pid.c: export 'find_task_by_vpid'
kernel-config x686_secle_audis_defconfig

linux-kernel-build for non-casesensitive filesystems:
add shellscripts for searching, renaming and un-renaming critical filenames
renaming some filenames to lowercase and change filecontents
kompiled with kernel-config x686_secle_audis_defconfig
(see /audis_dev/src/pkgs/linux/addInfo/non-casesensitive)
	
14-Feb-2012
w.h. rt_io_event.c, rt_sync_timer.c, rt_signal.c: A better handling
of the cycle parameter (sigevent_set_notification()) is now provided.

rt_tgroup.c: The quota value 0 for tgroup_setquota() is no longer 
handled as a special value. 

14-Feb-2012
w.h. rt_io_event.c, rt_sync_timer.c, rt_signal.c: A better handling
of the cycle parameter (sigevent_set_notification()) is now provided.

rt_tgroup.c: The quota value 0 for tgroup_setquota() is no longer 
handled as a special value. 

09-Feb-2012
w.h. rt_link.h, rt_device.c, rt_sched.c: rt_sched_return_to_lx() now
checks for pending signals. If the current thread has pending signals
this function does not temporarily return to Linux (a value of 0 is 
returned). Otherwise, a value of 1 is returned.

09-Feb-2012
m.n. lib/Kconfig.audis and kernel/fork.c: add configuration option to limit
stack size of (user) threads.

08-Feb-2012
m.n. arch/mips/kernel/rtx_adapt_32.c: RTX SOFT: correct problem with 
Linux timer.

03-Feb-2012
h.b set MainHighSum bit (marvell interrupt controller) in the macro get_irqnr_and_base
    neccessary to avoid interrupt storms and deadlocks 
    arch/arm/mach-feroceon-kw/irq.c
    arch/arm/mach-feroceon-kw/include/mach/entry-macro.S

01-Feb-2012
w.h. Now the ftrace function tracer can be used for tracing Linux
functions as long as AuDis tracing is disabled. But tracing 
of RT-functions will be skipped if the function call is invoked in 
the RT-domain.

The RT system-call sys_rt_sigprocmask_rt() has been removed, since 
it is not supported by the A&D API.

31-Jan-2012
w.h. rt_base.c, rtx_adapt.h (x86, arm, powerpc, mips): 
Define RTX_SYSCALL_SETPGID architecture dependent (bugfix).

Kconfig.debug: A new kernel option RTX_THREAD_RUNTIME allows
for runtime accounting of RT-threads. The accumulated amount of time
a thread has been in running state also comprises the amount of
time the thread has been interrupted.

30-Jan-2012
m.n. arch/powerpc/include/asm/mmu_context.h: hard interrupt blocking for
linux mmu switch
arch/powerpc/include/asm/rtx_adapt.h: add define prepare_arch_switch to
prevent hard interrupt interruption during linux task switch.

24-Jan-2012
h.b realtime and preemption (CONFIG_PREEMPT) for arm architecture
    arch/arm/include/asm/irqflags.h
    arch/arm/include/asm/system.h
    arch/arm/kernel/entry-armv.S
    arch/arm/kernel/rtx_adapt_32.c

17-Jan-2011
m.n. add first level of implementation for user spinlock for mips
(arch/mips/kernel/syscall.c)

16-Jan-2012
m.n. add first step for mips realtime:
arch/mips/Kconfig, arch/mips/uad_soc/soc_irq.c soc_time.c
arch/mips/include/asm/irqflags.h, mmu_context.h, rtx_adapt.h, rtx_base.h
               rtx_per_cpu.h, system.h
arch/mips/kernel/asm-offsets.c, entry.S, Makefile, rtx_adapt_32.c,
               rtx_calls.S, scall32-o32.S, syscall.c, unaligned.c
arch/mips/kernel/mm/fault.c
upgrade default configurations for cp342-5 and cp1500

16-Jan-2012
w.h. Bugfix in rt_thread.c/rt_do_setscheduler(): If policy is -1 we also have
to check for a valid priority value.

13-Dec-2011
w.h. Support for new API-calls sigevent_set_monitor() and unregister_clock()
added. The following files have been changed: rt_timer.c, rt_io_event.c,
rt_device.c, rt_sync_timer.c, rt_signal.c, rt_driver.h, rt_link.h, rt_internal.h,
rt_internal2.h, include/asm-generic/siginfo.h.

08-Dec-2011
w.h. kernel/rtx_adapt.c, include/linux/irqflags.h: The signature of
rtx_check_context() has been changed. Now it provides file, line, func
from the caller context.

07-Dec-2011
m.n. arch/powerpc/kernel/rt_adapt_32.c: correct non-SMP handling for
rt_request/free_irq().

05-Dec-2011
m.n. enhance arch/powerpc/kernel/rtx_adapt_32.c and
arch/powerpc/sysdev/uic.c for use of realtime
high resolution timer with the 44x cpu type
update powerpc default configurations

28-Nov-2011
w.h. Bugfix for ioctl(..AuD_DEV_SEND_EVENT..): rt_device.c
The domain selection has been corrected.

25-Nov-2011
m.n.rt system call entries: add execute_in_lx for socket calls (#102)
(required by cp15431)

24-Nov-2011
w.h. Bugfix for x86-SMP: Apply a previous version of the stall-bit
access macros in arch/x86/include/asm/rtx_percpu.h. Now the kernel
config option "Compile the kernel with frame pointers" can be 
disabled without crashing the kernel.

w.h. Bugfix for SMP-debugging: rt_sched.c/rtx_mark_debug_pending().
Since the gdbserver may be running on an arbitrary CPU the invocation 
of rtx_mark_debug_pending() has to be delegated to the CPU the RT-process
is running on. 

24-Nov-2011
m.n. overtake changes vom KA: drivers/net/ibm_newemac/*
update arch/powerpc/platforms/4xx/ppc44x_simple.c and extend low level
debug features
arch/powerpc/kernel/head_44x.S: add low level debug features
drivers/Makefile and lib/Kconfig.audis: add configuration option for an
audis fwdebugmode driver which allows to change the kernel console level
add arch/powerpc/include/asm/wlan/led.h to get rid of an external header file

22-Nov-2011
w.h. rt_timer.h, rt_timer.c, rt_device.c: Now in a SMP-system the 
time accounting for RT is administrated only by one CPU (rt_timer_cpu). 
This prevents RT-processes on different CPUs from getting a time offset 
when reading the RT time by clock_gettime() at the same time.

21-Nov-2011
m.n. overtake changes from KA:
enhance fs/squashfs for additional (de)compress modes.
lib/decompress*.c and linux/decompress/*.h: adjust for use with squashfs-files.

18-Nov-2011
m.n. drivers/aud/auddevice/rt_sched.c: add a verbose message within the
subroutine rtx_mark_debug_pending

17-Nov-2011
w.h. Audis exclusive mode added for x86 SMP (drivers/aud/Kconfig.debug,
rt_device.c, arch/x86/include/asm/hw_irq.h, irq_vectors.h,
arch/x86/kernel/rtx_adapt_32.c, entry_32.S, irqinit.c). Whenever
a realtime process is running in the RT-domain the remaining cores
of the system are forced to run in an idle loop until the realtime
process leaves the RT-domain. This mode allows only one realtime
process to be running.

11-Nov-2011
m.n. set Label AUDIS_(ARCH/RT/FULL/NRK)-V02.04.01
(Linux kernel only)
h.b. enhancements for UBI in
       - drivers/mtd/chips/cfi_cmdset_0002.c
       - drivers/mtd/mtdconcat.c
       - drivers/mtd/mtdpart.c
       - drivers/mtd/nand/nand_base.c
     modifications from SC IC (Mr. Reuter) in file 
       - arch/arm/mach-feroceon-kw/kw_family/boardEnv/mvBoardEnvSpec.c

04-Nov-2011
m.n. arch/powerpc/kernel/head_booke.h: correct definition for DECREMENTER
Exception when not RTX_DOMAIN
arch/powerpc/kernel/process.c: spinlock workaround: don't use en/disable_hw
when not running with RTX_DOMAIN

09-Nov-2011
w.h. rt_io_event.c: Bugfix for rt_wait_for_event(), si_sys_cycle
replaced by si_int (previous value).

04-Nov-2011
w.h. rt_signal.c, rt_sched.c: Bugfix for audis_return_to_linux(). 
kernel/signal.c, rt_device.c, rt_internal.h rt_internal2.h, rt_sched.h:
Bugfix for shutdown of a RT process which has been attached dynamically
by audis_attachToRtDomain().

31-Oct-2011
m.n. set Label AUDIS_(ARCH/RT/FULL/NRK)-V02.04

28-0ct-2011
h.b Add UBI, UBIFS, HIGH_RES_TIMERS and RTX_DOMAIN_HRT to
    Marvell (db_98dx4122_spi_2_6_31_12_defconfig) and 
    X500 (X500PROTOTYPE_defconfig) configuration.
    Bugfix from Mr. Reuter: 
       arch\arm\plat-feroceon\mv_drivers_lsp\mv_mtd\sflash.c

27-Oct-2011
w.h. rt_internal2.h, rt_device.c: ioctl AuD_DEV_GET_THRD_NAME added.
rt_sched.c: Bugfix for audis_return_to_linux().
rt_internal.h, rt_io_event.c, rt_signal.c: Bugfix - the cycle value
(sigevent_set_notification) is now stored in a separate field in the
structures sigevent and siginfo.

24-Oct-2011
m.n. powerpc: update default configurations
add configuration option for special cp15432 watchdog handling
update pbusplus driver

20-Oct-2011
w.h. The first release for thread group support.
arch/xxx/include/asm: Split rtx_adapt.h and put the timer specific
definitions to rtx_timer.h (new).
rt_sync_timer.c: clock_sync_getres(), clock_sync_gettime() may be 
invoked from Linux.
arch/xxx/kernel: RT system call tables provide lseek system call 
via execute_in_lx.

17-Oct-2011
m.n. add high-resolution timer support for realtime domain with the
powerpc architecture

13-Oct-2011
m.n. set Label AUDIS_(ARCH/RT/FULL/NRK)-V02.03

13-Oct-2011
w.h. Adjust v2.6.31.12-audis-rt21-sinumerik.patch for a new Release.

10-Oct-2011
h.b. Update UBI/UBIFS to current version 2011-06-07
     http://git.infradead.org/users/dedekind/ubifs-v2.6.31.git
       ->  UBIFS: fix-up free space earlier   ->   snapshot

10-Oct-2011
w.h/m.n. arch/x86/kernel/rtx_adapt_32.c, entry_32.S: We added the hooks
for RT debugging. The macros RTX_SYSCALL_ENTER, LX_SYSCALL_LEAVE have 
been optimized.

07-Oct-2011
w.h. rt_sched.h, rt_base.c, entry_32.S: Remove the attribute 'asmlinkage'
from rtx_migrate_to_rt() which has the consequence that the parameter
is handed over in a register and not any longer on the stack.

05-Oct-2011
w.h. rt_internal2.h, rt_sched.h, exit.c, signal.c, rt_base.c, rt_device.c:
rtx_shutdown_rt_process() has now a new input parameter 'flag' which is 
usually set to 0. Only in the case of dynamically detaching a process from
the RT-domain 'flag' is set to DETACH_PROCESS, which decrements the thread
count. Otherwise, the slot for the calling process remains allocated and there
would be no way to attach the current process again.

29-Sep-2011
m.n. drivers/aud/auddevice/rt_signal.c: sys_rt_sigprocmask_rt: ignore SIGSTOP
when blocking signals

27-Sep-2011
m.n. arch/powerpc/kernel/entry_32.S: use rt_task_leave when migrating to lx
if bit RT_LEAVE_PENDING_TASK is set.

14-Sep-2011
w.h. rt_sched.c, rt_device.c, rt_link.h: add rt_sched_return_to_lx()
which voluntarily leaves the RT-domain and returns to Linux.

rt_sync_timer.c, rt_timer.h: Bugfixes sync timer 
- sync_timer_settime(): it_value may be zero, it's a legal value.
- clock_sync_gettime(): wrong time in the last interval of a cycle.

rt_internal2.h, rt_base.h, rt_device.c, rt_base.c, rt_mqueue.c, rt_futex.c:
VERBOSE bits: RT_SYSCALL_VERBOSE split into RT_SYSCALL_IN_LX_VERBOSE and
RT_SYSCALL_NOT_IMPL_VERBOSE to differentiate between RT system calls executed
in Linux and not implemented system calls in the RT-domain.

rt_base.c, rt_device.c, rt_tracer.c, asm/rtx_adapt.h: rtx_rdtsc() renamed to
rtx_gettime(). The name is now architecture independent.

arch/x86/mm/fault.c: Bugfix - In case of a page fault (which migrates to LX)
we have to migrate back to the RT-domain when the cause of the page fault is fixed.

rt_timer.c, rt_device.c: remove shutdown_in_progress (see rt_system_state). 
Set highest prio for the timer daemon in case of shutdown (the last wake-up).

rtx_adapt_32.c: __rtx_flush_printk depends on CONFIG_PRINTK.



13-Aug-2011
m.n. set Label AUDIS_(ARCH/RT/FULL/NRK)-V02.02
remark: this is the first issued label with the new clearcase

12-Aug-2011
m.n. arch/powerpc/kernel/rtx_adapt_32.c: add low level debug messages within
realtime timer interrupt.
m.n. arch/powerpc/kernel/process.c: spinlock_workaround: lock: lock
interrupts immediately. Start to implement an alternative solution based
on masking interrupts within the interrupt controller
add low level kernel dump messages
lib/Kconfig.audis: add option for low level kernel dump messages
arch/mips/kernel/traps.c: add low level kernel dump messages
arch/mips/aud_soc/soc_memory.c: test for memsize command line option

08-Aug-2011
m.n. drivers/aud/auddevice/rt_base.c: rtx_migrate_to_rt(): delay dequeuing
of migration threads.

08-Aug-2011
m.n. add support for mips soc (cp1500 and cp342-5)

29-Jul-2011
w.h./m.n. drivers/aud/auddevice/rt_futex.c: boost_own_priority: init list
header earlier to prevent kernel crash

27-Jul-2011
m.n. arch/powerpc/kernel/entry_32.S rtx_adapt.c: changes for realtime
debugging

21-Jul-2011
m.n. arch/powerpc/kernel/rtx_syscall_table.S: add execute_in_lx for
sys_pread64 and sys_pwrite64

/******************************************************************/
/************  we now have a new clearcase organisation  **********/
/******************************************************************/


21-Jul-2011
m.n. set Labels
LINUXPKG_LINUX-PATCH-2.6.31.12-AUDIS(-ARCH)(-RT)(-DRV)-V01.04
remark: this label is only used to have a merging point for our
move to a new clearcase structure

20-Jul-2011
m.n. arch/x86/kernel/trampoline.c: introduce init sections to prevent
section warnings during kernel linking.

w.h. rt_sched.c: Bugfix - replace percpu_write() by rtx_set_cpu_var()
to avoid a kernel crash for architectures other than x86.

19-Jul-2011
m.n. arch/x86/kvm/x86.c: resolve linking issue, when x86 configured without
profiling.

18-Jul-2011
m.n. arch/x86/kernel/tsc-sync.c: disable this code, when LTT is configured
(LTT use separate implementation in kernel/time/tsc_sync.c
arch/x86/Kconfig and ltt/Kconfig: select UNSYNCHRONIZE_TSC within ltt

14-Jul-2011
w.h. arch/x86/kernel/traps.c: Bugfix for shared linked apps. In this
case the 1st FP instruction in the RT-domain will initialize the FPU.

m.n. arch/powerpc/kernel/entry_32.S: enter systemcall: test for
migration_to_rt; leave rt systemcall: test for migration to lx
rtx_adapt_32.c: rtx_handle_irq: threads looping in user mode: test for 
migration to rt or to lx (see also 07-Jul-2011, 2nd entry).

08-Jul-2011
m.n. net/802/Makefile: built depending on board selection
arch/powerpc/platforms/83xx/Kconfig: use separate names for SCALANCE and
CP15431 boards.

06-Jul-2011
h.b. Add Realtime High Resolution Timer for ARM (X500_PROTOTYPE)
CONFIG_RTX_DOMAIN_HRT 

07-Jul-2011
w.h. Audis-driver: Kconfig.debug: RTX_DISABLE_WRITE_DAEMON new.
rt_base.c, rt_device.c, rt_fs_op.c, rt_futex.c, rt_sched.c,
rt_semaphore.c, rt_sync_timer.c, rt_thread.c, rt_timer.c:
A new kernel option allows to disable the write daemon for test
purposes (this may corrupt RT behaviour). The priority of the
write daemon can be adjusted at start of a RT-process.

Pending migrations back to LX (rt_task_leave()) are only handled
at systemcall exit of the RT-domain. Therefore a new state (rt_state2)
was introduced for the corresponding flags.

Changing the priority of a RT-thread (which is currently executing
in LX) is deferred, until the thread migrates back to RT. 
But there are some issues (policy change, priority change
of a SCHED_RR thread) which cannot be handled.

rt_base.h: V01.04, wd_prio new
rt_internal2.h rt_link.h, rt_sched.h: 
New macros IS_EXIT_PENDING, IS_REAL_TIME_PRIO, 
IS_REAL_TIME_PRIO_NORMALIZED. New flags for rt_state2. 

w.h Kernel-Hooks: linux/sched.h, fs/proc/array.c, lib/bust_spinlocks.c
arch/x86/kernel/entry_32.S, rtx_adapt_32.c, rtx_syscall_table_32.S,
asm_offsets_32.c:
Now sys_sched_get_priority_min/max called directly in RT-domain.
Handle migration if a RT-thread is running in user-mode.
A new task state rt_state2 handles the task leave pending request.

27-Jun-2011
m.n. arch/powerpc/boot/dts/scalance_w_old.dtc: add configuration for
timer support (non-high-resolution) for realtime domain

20-Jun-2011
m.n. kernel/signal.c: do_send_specific(): test for SIGINT to migrate a 
running RT main thread back to Linux for debugging.

15-Jun-2011
m.n. update arch/powerpc/boot/dts/cp15431.dts (flash configuration) and
drivers/aud/application_driver/powerpc/pbus_plus.c (new event)
m.n. within drivers/aud/auddevice: rt_base.c: set UNINTERRUPTIBLE for 
ltt daemon when sleeping (prevent permanent running when debugging).
rt_sched.c: rtx_is_rt_kernel_thread: use p->rt_proc not rt_proc of current
thread (may be the debugger); add test for ltt daemon
kernel/signal.c: complete_signal(): call rtx_mark_debug_pending()

08-Jun-2011
m.n. arch/powerpc/kernel/rtx_syscall_table.S: add missing rt syscall
rt_set_robust_list()

07-Jun-2011
m.n. set Labels
LINUXPKG_LINUX-PATCH-2.6.31.12-AUDIS(-ARCH)(-RT)(-DRV)-V01.03

w.h. v2.6.31.12-audis-rt21_sinumerik.patch: Bugfix in rt_base.c, 
invoke __schedule() instead of schedule().

30-May-2011
w.h. Audis-driver: Directory drivers/aud/audrealtime removed. The kernel
option CONFIG_RTX_DOMAIN_HARD is no longer supported.
drivers/aud/auddevice/rt_sync_timer.c: rt_enable_irq_affinity() removed.
drivers/aud/auddevice/rt_device.c: rt_disable_irq_affinity() removed.
arch/x86/kernel/rtx_adapt_32.c: rt_enable_irq_affinity() and 
rt_disable_irq_affinity() can be invoked implicitly if rt_request_irq()
and rt_free_irq() are called in the context of an Audis RT-process.
m.n. kernel/signal.c: do_send_specific: add call to rtx_mark_debug_pending().
arch/arm/Kconfig and arch/powerpc/Kconfig: remove hard realtime configuration
option.

26-May-2011
h.b. Add source for the X500-Prototype

25-May-2011
m.n. fs/proc/array.c: extend status message for tasks with an rt_state info.

20-May-2011
w.h. ipc/mqueue.c, rt_base.c: Bugfix message queues - crash because of poisened
list element.

rt_sched.c, rt_timer.h: YIELD_MAX_DELAY_OF_SLEEP now limits the time to wait.

16-May-2011
w.h. include/linux/aud/rt_base.h: New Version number V01.03
Note: This version of the kernel (V01.03) requires glibc version V00.08.
User applications must be rebuilt.

arch/x86/Kconfig.debug: Add "source lib/Kconfig.audis" to the Hooks31-12 branch.
lib/Kconfig.audis: Merge from the addArch branch to the Hooks31-12 branch. 

drivers/aud/auddevice:
rt_device.c: New global state rtx_mode_rt, deactivate RR-Timer on shutdown.
rt_futex.c: Bugfix of a PI-Mutex error (prio boosting). Return ERESTARTNOINTR
for debugging. New kernel interface for relative timeout values (FUTEX_WAIT_BITSET_REL,
FUTEXLOCK_PI_REL).
rt_mqueue.c: New kernel interface for relative timeout values.
rt_sched.c: Bugfix for setting the Linux mm (when returning from RT). Only relevant for
architectures which are using the previous mm.
rt_timer.c: Bugfix for RR-scheduling (race condition).
rt_tracer.c: Now it is possible to reset the trigger value (echo "" >trigger).
rt_fs_op.c: Added tgid-info for some kernel messages.
rt_io_event.c: Only some code clarification.

ipc/mqueue.c: New kernel interface for relative timeout values.
kernel/futex.c: New kernel interface for relative timeout values (FUTEX_WAIT_BITSET_REL,
FUTEXLOCK_PI_REL).
kernel/sched.c: LX_PENDING_SYNC removed (now obsolete).

kernel/trace:
ftrace.c: Now rtx_safe also depends on RTX_TRACE. The consequence is that Linux function
tracing can be used in an Audis-kernel. This feature was disabled in an Audis-kernel by now.
trace_functions.c: Stop the Linux function tracer in an Audis-kernel whenever a hard 
RT-process is running on an arbitrary CPU.
trace_selftest.c: Self testing also depends on RTX_TRACE.

include/linux:
rtx_system.h: Add the global state rtx_mode_rt.
mqueue.h: Remove RELTIME_MAX constant (now obsolete).
futex.h: Add FUTEX_WAIT_BITSET_REL, FUTEXLOCK_PI_REL.

arch/x86/kernel/rtx_adapt_32.c: Remove the LX_PENDING_SYNC flag. Adjust __rtx_handle_irq()
and __rtx_handle_nonrt_pending().
arch/x86/include/asm/rtx_base.h: Remove the LX_PENDING_SYNC flag.

lib/bust_spinlocks.c: In case of a Linux panic, the kernel message depends on 
RTX_DOMAIN_STOP_ON_PANIC.

13-May-2011
m.n. arch/powerpc/kernel/rtx_adapt_32.c: rtx_handle_irq(): some refinements
m.n. kernel/exit.c: correct missing merge to ltt branch

09-May-2011
m.n. arch/powerpc/kernel/traps.c: add call to rtx_trap_handling within
program_check_exception (for realtime debugging)
arch/powerpc/kernel/rtx_adapt_32.c: rtx_trap_handling: include debug handling
rtx_handle_irq() and handle_nonrt_pending(): add ipipe optimizations (no
pending sync flag) and our experiences.
arch/powerpc/kernel/entry_32.S: enhance for (realtime) debug stuff
arch/powerpc/include/asm/rtx_base.h: remove pending sync flag

03-May-2011
m.n. kernel/signal.c and lib/Kconfig.audis: add configuration option
CONFIG_ALLOW_POSIX_TIMER_OVERRUN; this allows to override the linux
default handling to restart posix timers only when the signal was dequeued
by the receiving thread. Therefore no overrun counts will happen and a
thread will not recognize late scheduling.

29-Apr-2011
m.n. drivers/net/Makefile: correct merge problem from Marvell
m.n. arch/powerpc/kernel/rtx_syscall_table.S: do a direct call to the 
Linux code from the rt-domain to get prio_min / prio_max

28-Apr-2011
m.n. arch/powerpc/configs/cp15431_defconfig: enable shmem (for named sema)
arch/powerpc/kernel/entry_32.S: rt thread executing in lx: pass return
value to rtx_migrate_to_rt()

27-Apr-2011
m.n. arch/powerpc/kernel/rtx_syscall_table.S: correct entry for clone

18-Apr-2011
m.n. set Labels
LINUXPKG_LINUX-PATCH-2.6.31.12-AUDIS(-ARCH)(-RT)(-DRV)-V01.02

18-Apr-2011
w.h. context switch (LX- and RT-domain) tracing added for RT-tracer.
drivers/aud/Kconfig.trace, kernel/sched.c, drivers/aud/auddevice/rt_sched.c.

15-Apr-2011
w.h. fs/aio.c, fs/exec.c: adapted to the latest I-pipe patch 
rt_device.c: rtx_is_rt_capable() - replace RT_PMAX by RT_PIR.
v2.6.31.12-audis-rt21_sinumerik.patch: Corrected (irq/chip.c,
arch/x86/kernel/cpu: mtrr/generic.c, cpufreq/acpi_cpufreq.c). 
arch/x86/kernel/rtx_adapt_32.c: rtx_handle_nonrt_pending: run
soft realtime handler again, after leaving a linux interrupt handler.
rt_base.h.: Version number 01.02 for next label.

w.h. user interface (ioctl) for RT-tracer added:
drivers/aud/auddevice: rt_device.c, rt_tracer.c
include/linux/aud: rt_internal2.h, rt_internal.h, rt_link.h

m.n. arch/powerpc/kernel/rtx_adapt_32.c: rtx_handle_nonrt_pending: run
soft realtime handler again, after leaving a linux interrupt handler

14-Apr-2011
m.n. arch/powerpc/include/asm/rtx_adapt.h and arch/powerpc/kernel/rtx_adapt.32.c
update for use with the rt-driver enhancements for multicore
(Remark: powerpc still works single core only!)

07-Apr-2011
w.h. rt_base.h: Version number corrected.
arch/x86/kernel/rtx_adapt_32.c: Missing initialization (rtx_init_timeconv()) added.

05-Apr-2011
m.n. spinlock workaround with powerpc: uncorrect config option in
arch/powerpc/include/asm/unistd.h and systbl.h

01-Apr-2011
m.n. set Labels
LINUXPKG_LINUX-PATCH-2.6.31.12-AUDIS(-ARCH)(-RT)(-DRV)-V01.01
Remark: this label works only with x86!

##############################################################
#
# with the move to multicore realtime support we set our major
# version number to 01!
#
##############################################################

08-Apr-2011
h.b. the interrupts have to be disabled, if the arm goes idle (low-power standby mode)
for real-time this is corrected in arch/arm/kernel/process.c


01-Apr-2011
w.h. Minor corrections rt_device.c, rtx_adapt.h, rtx_adapt_32.c
include/linux/rtx_arith.h

31-Mar-2011
h.b. solve the lttng net-extended problem with
lttng-trace-format-allow-large-alignment-fix.patch
file ltt-type-serializer.h; function ltt_specialized_trace; remov: largest_align = min_t(...

29-Mar-2011
w.h. First Multicore Release (one RT-process per cpu)
arch/x86/Kconfig
arch/x86/include/asm: hpet.c, irqflags.h, rtx_adapt.h, rtx_base.h, rtx_percpu.h
arch/x86/kernel: entry_32.S, irq.c, rtx_adapt_32.c, rtx_syscall_table_32.S
drivers/aud: Kconfig.debug, Kconfig.trace, Makefile
drivers/aud/auddevice: rt_base.c, rt_device.c, rt_fs_op.c, rt_futex.c, rt_io_event.c, 
rt_mqueue.c, rt_sched.c, rt_semaphore.c, rt_signal.c, rt_sync_timer.c, rt_tgroup.c,
rt_thread.c, rt_timer.c, rt_tracer.c
include/linux: irqflags.h, limits.h, rtx_percpu.h, rtx_system.h, sched.h
include/linux/aud: rt_base.h, rt_driver.h, rt_futex.h, rt_internal2.h, rt_internal.h,
rt_kernel.h, rt_link.h, rt_sched.h, rt_timer.h
ipc/mqueue.c
kernel: exit.c, fork.c, printk.c, sched.c, signal.c, spinlock.c
kernel/irq/manage.c
kernel/time/timekeeping.c
lib: bust_spinlocks.c
v2.6.31.12-audis-rt21_sinumerik.patch: The corresponding PREEMPT_RT-patch.

28-Mar-2011
m.n. set Labels 
LINUXPKG_LINUX-PATCH-2.6.31.12-AUDIS(-ARCH)(-RT)(-DRV)-V00.12

25-Mar-2011
w.h. error correction: rt_io_event.c/proxy_send_event() without
LXRT_STATE (LX-only thread).

15-Mar-2011
w.h. error correction in cancel_sync_migration():
rt_base.c, rt_semaphore.c, rt_semaphore.h
The function call up_rt() was replaced by up_rt_from_virq(). 
up_rt() may cause a system crash in case of calling rt_schedule(0).

14-Mar-2011
m.n. set Labels 
LINUXPKG_LINUX-PATCH-2.6.31.12-AUDIS(-ARCH)(-RT)(-DRV)-V00.11

14-Mar-2011
h.b. add ltt support and extend default configurations

11-Mar-2011
m.n. powerpc: extend system call entries for realtime domain
arch/powerpc/kernel/rts_syscall_table.S, rtx_adapt_32.c, entry_32.S,
asm-offset.c, process.c, traps.c
arch/powerpc/mm/fault.c and arch/powerpc/include/asm/rtx_adapt.h


04-Mar-2011
w.h. Error correction: sysenter system-calls.
arch/x86/kernel/entry_32.S, rtx_adapt_32.c:
For sysenter RT system-calls the return value must be set in EAX
before jumping to sysenter_exit.
Change the function interface of __rtx_syscall_migrated_to_lx() to
get a valid pt_regs pointer.

03-Mrch-2011
m.n. PCI host inbound window: provide a more generic approach to be used
with pbus-plus.c. Changed files arch/powerpc/Kconfig,
arch/powerpc/kernel/pci_32.c, arch/powerpc/sysdev/indirect_pci.c
drivers/aud/application_driver/powerpc/pbus_plus.c

28-Feb-2011
m.n. add scripts for use of a i686 tool chain

21-Feb-2011
m.n. one more change in the previous label: add a definition for OPEN_MAX
to provide a good number to be used within the glibc. Somehow this 
definition is no longer part of the kernel release.

21-Feb-2011
m.n. set Labels 
LINUXPKG_LINUX-PATCH-2.6.31.12-AUDIS(-ARCH)(-RT)-V00.10

21-Feb-2011
m.n. merge thread group support source / header file from 2.6.15 to 2.6.31
(drivers/aud/auddevice/rt_tgroup.c and include/linux/aud/rt_tgroup.h)

03-Feb-2011
w.h. Error correction: Synchronized migration of a source with a target
thread (e.g. thread create parent and child thread). 
Audis-driver: rt_semaphore.c, rt_device.c, rt_futex.c, rt_sched.c, rt_base.c,
rt_signal.c, rt_sync_timer.c, rt_timer.c, rt_thread.c, rt_fs_op.c.
rt_semaphore.h, rt_sched.h, rt_base.h, rt_link.h.
Architecture-independent: kernel/sched.c, include/linux/rtx_system.h
Architecture-dependent: rtx_adapt_32.c (x86, arm, powerpc), entry_32.S (x86),
arch/arm/mm/fault.c (arm).
Note: The calling interface of rtx_migrate_to_lx(), rtx_migrate_to_rt()
has been changed.
adjustment from entry-common.S (arm) (h.b.).
<ToDo>: entry_32.S (powerpc)

02-Feb-2011
m.n. set Labels 
LINUXPKG_LINUX-PATCH-2.6.31.12-AUDIS(-ARCH)(-RT)-V00.09

01-Feb-2011
m.n. add spinlock workaround system call functionality:
arch/powerpc/include/asm/unistd.h systbl.h
arch/powerpc/kernel/entry.S process.c
pbus-plus driver: change interrupt handling to gpio event driven mode.
this required a event mode for gpio (when an interrupt occurs). Changes
were done in
drivers/gpio/gpiolib.c include/asm-generic/gpio.h
arch/powerpc/sysdev/mpc8xxx_gpio.h
arch/powerpc/kernel/rtx_adapt_32.c: take care of edge triggered interrupts

24-Jan-2011
w.h. include/linux/aud/internal.h, internal2.h: Thread group specific
definitions added.

20-Jan-2011
m.n. arch/arm/common/Makefile: for rtctime.c do built only depending on
specific boards

17-Jan-2011
m.n. arch/powerpc/kernel/rtx_adapt.c: use mask_ack when entering realtime
interrupt routine.
drivers/aud/application_driver/powerpc/pbus_plus.c: for Linux only 
interrupt-handler: use flag value 0.

12-Jan-2011
m.n. powerpc scalance_w_old default config: add soft realtime option
arch/powerpc/include/asm/hw_irq.h: add raw_local_save_flags() for non-rt

23-Dec-2010
m.n. set Labels 
LINUXPKG_LINUX-PATCH-2.6.31.12-AUDIS(-ARCH)(-RT)-V00.08

first level of realtime extensions for powerpc
files in arch/powerpc:
sysdev/ipic.c: change to rtx_spin_(un)lock
Kconfig: realtime configuration options
kernel/Makefile: add make of rtx_adapt_32.o
kernel/cpu_setup_6xx.S: setup_common_caches: redo a change
kernel/head_32.S: add special calls for interrupts when running with realtime
kernel/irq.c: do_IRQ() is changed to do_IRQ_from_rt() to add the irq number
as second parameter.
kernel/ppc_ksyms.c: add do_IRQ_from_rt()
kernel/process.c: add / change for (realtime) task switch
kernel/time.c: add diagnostic info for lost interrupts
kernel/rtx_adapt_32.c: all the arch specific realtime stuff
platforms/83xx/Kconfig Makefile: extend for mpc83xx global timer build
platforms/83xx/mpc83x_timer.c: use global timer module for realtime timer
mm/fault.c: dummy call rtx_pin_range_globally; implementation TBD
boot/dts/cp15431.dts
include/asm/irq_internal.h, rtx_adapt.h, rtx_base.h rtx_percpu.h,
system.h, mmu_context.h, irqflags.h, irq.h, hw_irq.h
include/asm/unistd.h systbl.h, kernel/process.c: add system call to simulate
cli / sti user level handling (implementation: TBD)


20-Dec-2010
w.h.
rt_sync_timer.c: get_tmr()/put_tmr() may be called in environments
where the IRQs are already off, so we have to save the previous state.

17-Dec-2010
w.h.
AddArch31-12: lib/Kconfig.debug (options AUD_EXCEPTION_HOOK and 
AuD_PTRACE removed. lib/Kconfig.audis (NEW) with options 
AUD_EXCEPTION_HOOK and AUD_PTRACE. This file will be used in the
future to define Arch-specific options which do not depend on
the Audis driver.
arch/x86(arm, mips, powerpc)/Kconfig.debug: The menu "Audis 
settings" was added as a sub-menu to "Kernel hacking". So now
we have a defined place in menuconfig where we can find additional
options for the Audis driver for arbitrary architectures.
kernel/ptrace.c, include/linux/ptrace.h: Replace AuD_PTRACE by
AUD_PTRACE.

Hooks31-12:
arch/x86(arm, mips, powerpc)/Kconfig.debug: Add the Audis driver
menu options (drivers/aud/Kconfig.debug, Kconfig.trace) to the 
menu "Audis settings". 

Audis driver:
drivers/aud/Kconfig.debug, auddevice/rt_device.c: The menuconfig
allows to chose between setting the major number for the Audis
device node "/dev/audis" statically (with a given major number)
or dynamically. 

10-Dec-2010
m.n.
set Labels 
LINUXPKG_LINUX-PATCH-2.6.31.12-AUDIS(-ARCH)(-RT)-V00.07
add pbus_plus-driver for cp15431 (powerpc)

03-Dec-2010
h.b.
drivers/serial/Kconfig: CONFIG_SERIAL_KDBG_SPLIT
drivers/serial/kgdboc.c: Use CONFIG_SERIAL_KDBG_SPLIT
kernel/kgdb.c: Return an empty packet for a unimplemented command.

03-Dec-2010
w.h.
include/linux/irqflags.h: Replace the CONFIG_X86 comment by 
CONFIG_TRACE_IRQFLAGS_SUPPORT (back-port from a newer kernel).

01-Dec-2010
m.n.; h.b.
Workaround for real-time applications with shared libraries
arch/arm/kernel/rtx_adapt_32.c: Print the error "fault 0 in RT domain ..." once only
arch/arm/mm/fault.c: In normal case migrate to RT

30-Nov-2010
m.n.;h.b.
kernel/ptrace.c: ptrace_attach_self() - corresponding to ptrace_attach()
                 early_attach_other_threads - includes the main thread

30-Nov-2010
w.h.
kernel/sched.c: sched_setaffinity() - do not allow threads of the RT-process
to migrate to another CPU in case of a Multicore system.

25-Nov-2010
arch/powerpc/kernel/cpu_setup_6xx.S: setup_common_cache(): solve 
cache startup problem
move the marvell specific documentation to addInfo/Marvell
m.n. overtake some more changes from Joerg Mueller (Khe)
for SIMATIC_NET_SCALANCE_W
arch/powerpc/kernel/cputable.c, pci-common.c, time,c
arch/powerpc/platforms/44x/Kconfig, ppc44x_simple.c,
arch/powerpc/platforms/83xx/Kconfig, mpc834x_mds.c
arch/powerpc/sysdev/ppc4xx_pci.c
net/core/dev.c

23-Nov-2010
m.n.
set Labels 
LINUXPKG_LINUX-PATCH-2.6.31.12-AUDIS(-ARCH)(-RT)-V00.06

23-Nov-2010
w.h.
drivers/aud/auddevice/rt_sync_timer.c: RTX_SYS_CLEAR_INTR() deleted
include/linux/rtx_system.h: RTX_SYS_CLEAR_INTR() deleted 

23-Nov-2010
h.b. add realtime functionality for the marvell switch (DB-xCat-24GE-4GB, arch=ARM)
arch/arm/kernel/rtx_*
arch/arm/kernel/entry_armv.S
arch/arm/kernel/entry_common.S
arch/arm/include/asm/rtx_*
arch/arm/include/asm/irqflags.h
etc.

22-Nov-2010
m.n. add default config x86 with high resolution timer for microbox and
embedded controller;
use a common bit number for MIGRATION2 realtime state flag in 2.6.15 and .31

17-Nov-2010
w.h.
arch/x86/kernel/rtx_adapt_32.c
drivers/aud/Kconfig.debug
New option RTX_DOMAIN_STOP_ON_PANIC added.

16-Nov-2010
w.h.
arch/x86/include/asm/irqflags.h
arch/x86/include/asm/rtx_base.h
arch/x86/kernel/rtx_adapt_32.c
Corrected: safe_halt() doesn't halt the CPU if there are 
pending LX interrupts to be handled (Jan Kiszka).

rt_tracer.c: Replace spin_(un)lock by rtx_spin_(un)lock (Jan Kiszka).

12-Nov-2010
m.n.
set Labels 
LINUXPKG_LINUX-PATCH-2.6.31.12-AUDIS(-ARCH)(-RT)-V00.05
overtake *.dts and defconfig-files for scalance-w old and product

11-Nov-2010
m.n.
drivers/mtd/maps/Makefile: change script that arm-marvell stuff is separated
overtake performance optimization stuff from amcc (from kernel 2.6.32)
according to stefan keller:
arch/powerpc/include/asm/dc_regs.h ppc4xx_ocm.h
arch/powerpc/Kconfig
arch/powerpc/platforms/Kconfig.cputype
arch/powerpc/platforms/44x/Kconfig
arch/powerpc/sysdev/Kconfig, Makefile, ppc4xx_soc.c ppc4xx_ocm.c

10-Nov-2010
m.n.
net/llc/Kconfig: add configuration option for cp15431 download
drivers/net/Makefile: some of the stuff overtaken from Marvell (kernel 2.6.22)
is no longer valid on 2.6.31, take the newer one
(remark: we may still have too much stuff from 2.6.22 marvell)

10-Nov-2010
m.n.
these changes were already part of the previous label:
- add configuration for cp15431
- arch/powerpc/platforms/83xx/mpc834x_mds.c: add debugging stuff for our board
- arch/powerpc/kernel/traps.c: add exception hook for die()
- kernel/panic.c: export symbol print_tainted
- kernel/extable.c: add export __kernel_text_address
- lib/Kconfig.debug: add AuD debugging features
- net/802/Makefile: add llc compilation
- net/llc/llc_input.c llc_output.c: a: add code for cp15431 download

08-Nov-2010
m.n. set Labels 
LINUXPKG_LINUX-PATCH-2.6.31.12-AUDIS(-ARCH)(-RT)-V00.04
kernel/hrtimer.c: nanosleep(): ARM: we have problems with debugging;
workaround: ignore restart, but we need more investigations

29-Oct-2010
m.n. kernel/ptrace.c: attach of an early attached thread set ret correctly.

15-Oct-2010
w.h. Release with LAPIC-Timer instead of HPET. /dev/auddriver node
is now created dynamically and it is renamed to /dev/audis. This
has implications on Startup-Code and glibc, because the macro
name AuD_DRIVER_NAME also changed to AUDIS_DRIVER_PATH.

arch/x86/Kconfig: Audis HRT cannot be set explicitly any longer.
The decision nonHRT/HRT depends on LAPIC, NO_HZ, HIGH_RES_TIMERS.

rtx_adapt.h, rtx_arith_32.h (new), hpet.c, rtx_adapt_32.c, rtx_arith.h (new)
rtx_tickdev.h (new), rtx_time_conv.h (new), rtx_adapt.c (new), time/Kconfig,
kernel/Makefile:
Audis timer mangement - replace HPET by LAPIC timer (core-specific).

asm/rtx_percpu.h, linux/interrupt.h, irqflags.h, ioremap.c, fault.c, vmalloc.c:
Only minor comment changes.

i8259.c, time_32.c, io_apic.c, pci/htirq.c, fs/aio.c, exec.c, linux/mqueue.h, 
preempt.h, sched.h, spinlock.h spinlock_types.h, mqueue.c, spinlock.c:
Changes due to the adaption of the Audis patch to PREEMPT_RT.

rtx_system.h: FUTEX_PI_MODE removed.
rtx_syscall_table_32.S: Cut the not implemented LX syscalls at the end of the file.
sched.c: Handle the runqueue uninterruptible count correctly.
asm-generic/bug.h: Merged from Arch31 branch.

rt_base.c, rt_device.c, rt_fs_op.c, rt_futex.c, rt_io_event.c, rt_mqueue.c,
rt_sched.c, rt_signal.c, rt_sync_timer.c, rt_thread.c, rt_timer.c,
rt_base.h, rt_internal.h rt_internal2.h, rt_link.h, rt_timer.h:
copy_to/from_user() replaced by rt_copy_to/from_user() (ISR-safe).
Timer management adapted to LAPIC timer.
PI-futexes: Enhanced prio boosting when the RT-thread is currently running in LX.
printkGen: Kernel log level added.
Adaption to PREEMPT_RT.
	
07-Oct-2010
w.h. kernel/panic.c: Calling printk after exception hook.

02-Oct-2010
m.n. set Labels 
LINUXPKG_LINUX-PATCH-2.6.31.12-AUDIS(-ARCH)(-RT)-V00.03
include/linux/if.h add defines to allow parallel use of net/if.h and this file

01-Oct-2010
w.h. linux/if_tunnel.h: include asm/byteorder.h added.

22-September-2010
w.h. rtx_adapt_32.c: __rtx_handle_irq() is now calling __rtx_handle_nonrt_pending().
Remove __set_bit(IRQ_STALLED). Both changes are initiated by Jan Kiszka. 

17-September-2010
w.h. rtx_adapt_32.c: IRQ mode string "+/-" replaced by "*/+" (MC requirement).

01-September-2010
w.h. arch/x86/Kconfig: Enable CONFIG_HOTPLUG for RTX_DOMAIN (MC requirement),
RTX_DOMAIN depends on X86_32.

31-August-2010
w.h. rtx_adapt_32.c: Add IRQ mode (/proc/interrupts) for lapic timer.
Replace IRQ mode string "RT/LX" by "+/-". In case of the lapic timer
"desc" is NULL (do not access).

30-August-2010
w.h. Use __rtx_pin_range_globally() unconditionally to avoid page faults 
in case of an unmapped page table directory.

13-August-2010
w.h. rtx_adapt_32.c: Support for /proc/interrupt. Count interrupts for
RT-handlers. Display IRQ-Mode "RT/LX" for RT-IRQs.

12-August-2010
w.h. arch/x86/include/asm/rtx_base.h, arch/x86/mm/fault.c
mm/vmalloc.c, lib/ioremap.c: Use __rtx_pin_range_globally() for the RT-domain
to prevent unexpected page faults.
 
10-August-2010
w.h. arch/x86/kernel/irq.c, include/linux/interrupt.h, rtx_adapt-32.c:
Interrupt count added for RT-interrupts, IRQ-mode ("RT" or "LX") added
for RT-interrupts (see /proc/interrupts).

04-August-2010
w.h. rtx_adapt.h - remove 'cli' in __rtx_call_lx_(v)irq_handler 
because there is already local_irq_disable_hw() after calling this
macro.

rtx_adapt_32.c: Remove the setting of rtx_mode in set_execution_environment(),
it is done explicitly in rtx_shutdown_rt_process.

fork.c: sys_futex(FUTEX_WAKE) needs FUTEX_PRIVATE_FLAG but only for threads
of the RT-process..

futex.c: PI-mutex stuff added. The RT-domain only supports private futexes.
We do not allow for the LX-domain to access a futex which already has RT
PI waiters and vice versa.Robust mutexes added though they are not part of
the AuD-API.

futex.h: Add special flags FUTEX_WAITERS_PI_RT (..._PI_LX) to signal
whether a thread is waiting for a PI-mutex in the RT- or the LX-domain.

panic.c: Changed - display the "kernel panic" message before calling the
system exception handler.

sched.c: Do not allow to change the prio of a LX thread (causing this thread
to migrate to RT) when this thread still has PI-waiters or is blocked on a
PI-mutex).
Use the macro IS_REALTIME_PROCESS instead of querying the rt_state.
Call check_class_changed() only if LEAVE_PENDING is not set.  

spinlock.c: When calling _spin_(un)lock_irq_hw in the RT-domain do not
call preempt_enable/disable().


how to use
==========

   - increments for marvell DB-xCat-24GE-4GB switch
   - use build_marvell_linux to compile
   - to build binutils, gcc-4.2, glibc, linux-kernel and gdb with multicore-support:
     do a "source mcbuild.sh" in /audis_dev/src/ownsrc/aud_other/multicore in the same shell before building

