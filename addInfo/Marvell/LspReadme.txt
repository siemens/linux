
General LSP information for KW platforms
=================================================

Contents:
---------
  1.  Default kernel configuration
  2.  Marvell LSP File locations
  3.  Procedure for Porting a new Customer Board
  4.  MTD (Memory Technology Devices) Support
  5.  mv_ethernet interface name configuration
  6.  mv_gateway driver 
  7.  mv_phone driver
  8.  SATA
  9.  USB in HOST mode
  10.  USB in Device mode
  11.  Real Time Clock
  12. CESA
  13. SD\MMC\SDIO
  14. Audio
  15. Kernel configuration
    15.1 General Configuration
    15.2 Run-Time Configuration
    15.3 Compile-Time Configuration 
  16. Debugging  Tools
  17. Upgrading the U-Boot


1.  Default kernel configuration
---------------------------------

	Board			Default Configuration
====================================================================================
	DB-88F6281-BP 	        mv88f6281_eth_defconfig, mv88f6281_eth_be_defconfig	
	DB-88F6192-BP		mv88f6281_eth_defconfig, mv88f6281_eth_be_defconfig
	RD-88F6192		mv88f6281_eth_defconfig, mv88f6281_eth_be_defconfig
	RD-88F6281		mv88f6281_gw_defconfig,  mv88f6281_gw_be_defconfig



2.  Marvell LSP File locations
-------------------------------
    o  KW core directory: 
       - /arch/arm/mach-feroceon-kw/...
       - /include/asm-arm/arch-feroceon-kw/...

    o  KW drivers:  
       - /arch/arm/plat-feroceon/...


3.  Procedure for Porting a new Customer Board
-----------------------------------------------
The following are the steps for porting a new customer board to the Marvell LSP:

    o Add the Board Specific configuration definitions:
	File location: ~/arch/arm/mach-feroceon-kw/kw_family/boardEnv/mvBoardEnv.h

	- MPP pin configuration. Each pin is represented by a nible. Refer the
	  SoC Datasheet for detailed information about the options and values
	  per pin.

	- MPP pin direction (input or output). Each MPP pin is represented
	  with a single bit (1 for input and 0 for output).

	- MPP pin level (default level, high or low) if the MPP pin is a GPIO
	  and configured to output.

	- Specify the Board ID. This is need to identify the board. This is
	  supposed to be synchronized with the board ID passed by the UBoot.

    o Add the Board Specific configuration tables:
	File location: ~/arch/arm/mach-feroceon-kw/kw_family/boardEnv/mvBoardEnv.c

	The following configuration options are listed in the order they are
	present in the "MV_BOARD_INFO" structure.

	- boardName: Set the board name string. This is displayed by both Uboot and Linux
	  during the boot process.

	- pBoardMppConfigValue (MV_BOARD_MPP_INFO): This structure arranges the MPP pins
	  configuration. This is usually not modified.

	- intsGppMask: Select MPP pins that are supposed to operate as
	  interrupt lines.

	- pDevCsInfo (MV_DEV_CS_INFO):Specify the devices connected on the device bus 
	  with the Chip select configuration.

	- pBoardPciIf (MV_BOARD_PCI_IF): This is the PCI Interface table with the PCI 
	  device number and MPP pins assigned for each of the 4 interrupts A, B, C and D.

	- pBoardTwsiDev (): List of I2C devices connected on the TWSI
	  interface with the device ID Addressing mode (10 or 7 bit).

	- pBoardMacInfo (MV_BOARD_MAC_INFO): Specifies the MAC speed and the Phy address
  	  per Ethernet interface.

	- pBoardGppInfo (): List of MPP pins configured as GPIO pins with special functionality.

	- pLedGppPin (MV_U8): array of the MPP pins connected to LEDs.

	- ledsPolarity: Bitmap specifying the MPP pins to be configured with
	  reverse polarity.

	- gppOutEnVal: This is usually defined in the mvCustomerBoardEnv.h
	  specifying the direction of all MPP pins.

	- gppPolarityVal: Not used.

	Finally update all of the configuration table sizes (xxxxxxxxx_NUM definitions)
  	according to the number of entries in the relevant table.

    o  Specify the memory map of your new board. 
	File location: ~/arch/arm/mach-feroceon-kw/sysmap.c

	The following configurations should be done:
	- Look for the section in the file related to the SoC device you are using.

	- Add a new table with Address Decoding information (MV_CPU_DEC_WIN) for your board.
	  (Usually existing address decoding tables are compatible with most boards, the 
	  changes might be only in the Device Chip selects only).

	- In the function "mv_sys_map()", add a new "case:" statement (under the appropriate 
	  SoC type) with the your newly added board ID mapping it to the appropriate Address 
	  Decoding configuration table.


4.  MTD (Memory Technology Devices) Support
--------------------------------------------

A new MTD map driver has been added, this driver automatically detects the existing Flash devices
and mapps it into the Linux MTD subsystem. This new driver affect NOR flashes (CFI, SPI and Marvell). 
NAND flashes are supported separately and not not part of this driver.

The detection of MTD devices depends on the Linux kernel configuration options set (using the 
"make menuconfig" or "make xconfig" tools).
To have basic MTD Support the following options should be selected:
	-> Device Drivers                                                                                                   
          -> Memory Technology Devices (MTD)                                                                                
            -> Memory Technology Device (MTD) support (MTD [=y])                                                            

For CFI Flashes the following options should be selected
	-> Device Drivers                                                                                                   
          -> Memory Technology Devices (MTD)                                                                                
            -> Memory Technology Device (MTD) support (MTD [=y])                                                            
              -> RAM/ROM/Flash chip drivers 
		-> Detect flash chips by Common Flash Interface (CFI) probe

For Intel (and Intel compatible) Flashes the following options should be selected
	-> Device Drivers                                                                                                   
          -> Memory Technology Devices (MTD)                                                                                
            -> Memory Technology Device (MTD) support (MTD [=y])                                                            
              -> RAM/ROM/Flash chip drivers 
		-> Support for Intel/Sharp flash chips

For AMD (and AMD compatible) Flashes the following options should be selected
	-> Device Drivers                                                                                                   
          -> Memory Technology Devices (MTD)                                                                                
            -> Memory Technology Device (MTD) support (MTD [=y])                                                            
              -> RAM/ROM/Flash chip drivers 
		->  Support for AMD/Fujitsu flash chips

By default, the map driver maps the whole flash device as single mtd device (/dev/mtd0, /dev/mtd1, ..)
unless differently specified from the UBoot using the partitioning mechanism.
To use the flash partitioning you need to have this option selected in the kernel. To do this
you will need the following option selected:
 	-> Device Drivers                                                                                                   
          -> Memory Technology Devices (MTD)                                                                                
            -> Memory Technology Device (MTD) support (MTD [=y])   
	      -> MTD concatenating support

The exact partitioning is specified from the UBoot arguments passed to the kernel. The following 
is the syntax of the string to be added to the UBoot "booatargs" environment variable:
    
       'mtdparts=<mtd-id>:7m@0(rootfs),1m@7(uboot)ro' 
       where <mtd-id> can be one of options: 
       1) M-Flash => "marvell_flash"
       2) SPI-Flash => "spi_flash"
       3) NOR-Flash => "cfi_flash"

The latest release of the mtd-utils can be downloaded from http://www.linux-mtd.infradead.org.
(The main page has a link to the latest release of the mtd-utils package).
This package provides a set of sources that can be compiled and used to manage and debug MTD devices. 
These tools can be used to erase, read and write MTD devices and to retrieve some basic information.

The following is a list of useful commands:
To see a list of MTD devices detect by the kernel: "cat /proc/mtd"
To erase the whole MTD device: "./flash_eraseall /dev/mtd0"
To erase the whole MTD device and format it with jffs2: "./flash_eraseall -j /dev/mtd1"
To get device info (sectors size and count): "./flash_info /dev/mtd1"
To create jffs2 image for NAND flash(with eraseblock size 0x20000): 
           ./mkfs.jffs2 -l -e 0x20000 -n -d <path_to_fs> -o <output_file>
for NOR flash only:
===================
To protect all sectors: "./flash_lock /dev/mtd1 0x0 -1"
To unprotect all sectors: "./flash_unlock /dev/mtd1"


5.  mv_ethernet interface name configuration
--------------------------------------------

The name of the Ethernet interface/s can be changed at compilation time. This is configured using 
the kernel configuration tools. The following is the parameter defining the name:
	-> System Type                                                                                                      
          -> Feroceon SoC options                                                                                           
            -> SoC Networking support
	      -> Marvell network driver name

Changing the interface name in the Linux might also require changing the name in the UBoot. This 
is essential if the root filesystem is loaded over NFS. The name is part of the UBoot "bootargs_end" 
environment variable.

The Ethernet driver is found under ~/arch/arm/plat-feroceon/mv_drivers_lsp/mv_network/mv_ethernet_fp/


6.  mv_gateway driver
---------------------

    o  Supported SoC: 88F6281. 
       Supported switch: 88E6165. 
       Used for platforms with switch device on board (RD platforms).

    o  Interface name - "eth<port>"

    o  Multiple VLANs/network-interface management.
       Configuration in kerenl command line -
       Sysntax: mv_net_config=(<if-name>,<mac-addr>,<port-list>)(...)... 
       e.g. mv_net_config=(eth0,00:aa:bb:cc:dd:ee,0)(eth1,00:11:22:33:44:55,1:2:3:4)

    o  IP ToS based QoS
       -  VoIP QoS
       -  Routing

    o  L2 IGMP snooping support

    o  Packets transfers between CPU and Switch are controlled with -
       -  VLAN-tag (GbE port)

    o  Link status indication implemented in one of two ways - 
       -  ISR connected to switch interrupt line 
       -  When option above is not applicable, using a timer

    o  Switch LEDs: Link/Speed/Activity per port indication.

    o  See ~/arch/arm/plat-feroceon/mv_drivers_lsp/mv_network/mv_gateway/

Audio: TBD
----------

7.  mv_phone driver
-------------------

    o  Used only on DB-88F6281-BP with attached TDM-FXS-FXO module, 
       or on RD-88F6281 with on board FXS/FXO. 
       Default settings are: dev0 --> FXS, dev1 --> FXO

    o  Configuration in kernel command line - 
       Syntax: mv_phone_config=dev0:<fxs/fxo>,dev1:<fxs/fxo> 

    o  See ~/arch/arm/plat-feroceon/mv_drivers_lsp/mv_phone/



8. SATA 
---------

The LSP includes a full driver for Marvell's SATA controllers, the following is a list of the 
devices supported:
	- Integrated Sata Controller (in 88F5182, 88F6082, 88F6082L, 88F5082)
	- 88SX5041
	- 88SX5080
	- 88SX5081
	- 88SX6081
	- 88SX6041
	- 88SX6042
	- 88SX7042

The driver HAL APIs are found under: ~/arch/arm/mach-feroceon/Board/SATA/
The Linux driver is found under: ~/arch/arm/plat-feroceon/mv_drivers_lsp/mv_sata/
To enable supporting Optical disk drives (CD-ROM/DVD-ROM), this option should be selected:
 	-> System Type                                                                                                       
          -> Feroceon SoC options                                                                                            
            -> Support for Marvell Sata Adapters (SCSI_MVSATA [=y])                                                          
              -> Sata options                                                                                                
                -> Support ATAPI (CD-ROM/DVD-ROM) devices

The SATA driver has basic debugging capabilities. Using the kernel configuration tools, the user
can select 1 of 2 debugging options:
	- Display log messages on error conditions.
	- Display complete debugging log.


The SATA kernel configuration options are found under:
 	-> System Type                                                                                                       
          -> Feroceon SoC options                                                                                            
            -> Support for Marvell Sata Adapters (SCSI_MVSATA [=y])                                                          
              -> Sata options                                                                                                
                -> Debug level (<choice> [=y]) 

Besides, the SATA driver provides a runtime mechanism using the /proc filesystem to display 
all information about detected controllers and Disks.
The command "cat /proc/scsi/mvSata/0" where "0" specified the SATA channel number requested. The
channel numbers range from 0 to n where n is the one minus the number of channels available on the
detected SATA controller.

This driver supports the ATA SMART commands that issued by the smartmontools tool, version 5.36 
or later of that tool needed, also, the user should add "-d marvell" to the commands line parameters.

9.  USB in HOST mode
---------------------

The mode of the USB controller (device or host) is configured using the UBoot environment variables. 
To work in USB HOST mode, set the UBoot variable "usb0Mode"/"usb1Mode" to "host".
The USB driver uses the standart Linux ehci driver.

10. USB in Device mode
----------------------

To work in device mode, the UBoot environment variable "usb0Mode" should be set to "device". 
In order to operate as "Mass Storage Device" the following steps should be followed:
	- Prepare a file to be used as the storage back end (for example use the command 
          "dd bs=1M count=64 if=/dev/zero of=/root/diskFile" to create a file of size 64M.)
	- Insert the Marvell USB gadget driver: "insmod mv_udc.ko"
	- Insert the file storage driver: "insmod g_file_storage.ko file=/root/diskFile". If 
	  the backing file was created on an NFS drive then the following command should be used
          instead: "insmod g_file_storage.ko file=/root/diskFile use_directio=0"
Note: only one USB interface can be set as a device.

11.  Real Time Clock
---------------------

The driver is found under ~/arch/arm/mach-feroceon-kw/rtc.c
To read the date and time from the integrated RTC unit, use the command "hwclock".
To set the time in the RTC from the current Linux clock, use the command "hwclock --systohc"

12.  CESA
----------
OpenSSL
-------
  see cesa/openssl/

IPsec
-----
  see cesa/openswan/

Disk encryption 
---------------
o   To create the crypto partition, you are needed to perform the following steps:
    - Create physical partition on the disk - fdisk /dev/sda (example sda1 will created)
    - Create the crypto device example:
     `cryptsetup -c des3_ede -d /share/public/keyfile -s192 create mycryptsda1 /dev/sda1`
    The new device will created /dev/mapper/mycryptsda1 
    - Create the file system on crypto device:
      `mkfs.ext2 /dev/mapper/mycryptsda1 `
    - mount the formatted partition to directory
      `mount /dev/mapper/mycryptsda1 /mnt/mydevice`
    Use the /mnt/mydevice as usual to store your files. All files on the disk will be encrypted.

o   To remove the crypto devices do the following steps:
    - Exit the /mnt/mydevice directory
    - umount /dev/mapper/mycryptsda1
    - cryptsetup remove mycryptsda1


13. SD\MMC\SDIO
----------------
This driver is enabled in KW SoCs that include an SD\MMC\SDIO host. the 
driver is based on latest mmc driver from kernel 2.6.24.

o  creating mmc block devices:

# mknod /dev/mmcblk b 179 0
# mknod /dev/mmcblk1 b 179 1
# mknod /dev/mmcblk2 b 179 2
# mknod /dev/mmcblk3 b 179 3
..
..
..

o  modules:
# insmod mvsdmmc.ko
o debug parameters under /proc/mvsdmmc
o mvsdmmc.ko parameters:


highspeed -     1 - support highspeed cards (default)
                0 - don't support highspeed cards

maxfreq         value - maximum frequency supported (default 50000000)

dump_on_error	if 1 then on error dumps registers values

detect		1 - support GPIO detection interrupt
		0 - no support for GPIO detection interrupt

14. Audio
---------
o requires ALSA lib and ALSA utils version 1.0.14 
o snddevices script should be run if Alsa device doesn't exist.


15. Kernel configuartion
-------------------------

 15.1 General Configuration:
 ---------------------------
- This release has support for sending requests with length up to 1MB for the
  SATA drives, in some cases, this feature can reduce the system performance,
  for example, running Samba and a client that performs sequential reads.
  Note that the user can modify the limit of the max request using the sysfs,
  this parameter is per block device, and it's defined by special file called
  'max_sectors_kb' under the queue directory of the block device under the sysfs.
  for example, the /sys/block/sda/queue/max_sectors_kb is for the /dev/sda
  device.

- In order to use block devices that are larget then 2TB, CONFIG_LBD should be enabled.
  fdisk doesn't support block devices that are larger then 2TB, instead 'parted' should be used.
  The msdos partition table doesn't support >2TB , you need GPT support by the kernel:
  File Systems
    Partition Types
      [*] Advanced partition selection
      [*] EFI GUID Partition support

 15.2 Run-Time Configuration:
 ----------------------------
  The following features can be configured during run-time:
    o  NFP mechanism:
  	 echo D > /proc/net/mv_eth_tool (disable NFP)
  	 echo E > /proc/net/mv_eth_tool (enable NFP)
    o TX enable race:
         egigatool -txen 0/1 (0 - disable, 1 - enable)         
    o SKB reuse mechanism:
         egigatool -skb 0/1 (0 - disable, 1 - enable)
         
  * for more ethernet run-time configurations, see egigatool help.
  
 15.3 Compile-Time Configuration:
 --------------------------------
 The following features can be configured during compile-time only:   
    o L2 cache support
    o XOR offload for CPU tasks:
       - memcpy
       - memset
       - copy from/to user
       - RAID5 XOR calculation
    o UFO, LRO and TSO
    o Multi Q support - for mv_gateway driver only.
    o CESA test tool support.   

16.  Debugging  Tools
----------------------

    o  Runtime debugging is supported through the /proc virtual FS.
       See ~/arch/arm/mach-feroceon-kw/proc.c

    o  mv_Shell: Access memory, SoC registers, and SMI registers from user space.
       Egigatool: Probe mv_ethernet driver for statistic counters.
       CESA_tool: Probe CESA driver for statistic counters.
       These tools are found under ~/tools

    o The LSP supports kernel debugging using KGDB. Refer to AN232 "Using GDB to Debug the 
      Linux Kernel and Applications" for detailed information.

    o Early boot debugging is supported by the LSP. To enable this option configure the following
      settings in the kernel.
	-> Kernel hacking
	  -> Kernel low-level debugging functions
      You have this option you need first to enable the "Kernel debugging" tab first.
      
17.  Upgrading the U-Boot
-------------------------
    o See the UBoot release notes for instruction on how to build U-Boot images.
    o Updating UBoot image on boards with an old U-Boot version (1.x.x):
	- Use flinfo command to see current Flash properties.
	- Find where the current U-Boot is located (bank number and address) according to 
	  the output from flinfo. U-Boot sections are usually marked as Read Only (RO).
	- Unprotect bank, e.g. protect off bank 1.
	- Erase bank, e.g. erase bank 1.
	- Load image: tftp 100000 <u-boot image>
	- Copy image from RAM address 100000 to U-Boot Flash location, e.g. 
	  cp.b 100000 fff60000 <tftp size, rounded up>
	- Reset board.
    o Upgrading from newer UBoot releases (check if you have the command "bubt"):
	- Put the new UBoot file (u-boot-xxxxxxx.bin) under the TFTP server
	  root folder.
	- Set the board ipaddress: setenv ipaddress a.b.c.d
	- Set the IP address of the TFTP server.
	- Start the upgrade process: "bubt u-boot-xxxxxxxx.bin"
	- You will be prompted to confirm the deletion of the environment variables. 
	  This is a must if the board endianess was changed, and recommended in
	  all cases.
	- Wait for the upgrade process to finish and get back the Uboot prompt (reseting or 
          powering off the board during the upgrade process might leave the board useless).
	- Reset the board using the command "reset" or powering off then on.


