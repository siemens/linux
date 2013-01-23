The LSP consist reference switch driver, the driver do:
-- Switch init configuration, default VLAN, MAC to CPU configuration.
-- Network device per port config, each switch port represented by Linux network device.
-- SDMA configuration.
-- Connection to Switch interrupt GPP.


How to use (example):
-- Enable CONFIG_MV_PRESTERA_SWITCH in .config.
-- in run time run next script :

# Enable IP stuff in Linux
echo 1 > /proc/sys/net/ipv4/ip_forward
echo 3 > /proc/sys/net/ipv4/neigh/default/app_solicit
echo 1 > /proc/sys/net/ipv4/conf/all/forwarding

#enable Broadcast ARP trap to CPU
echo '2040000 69220511' > /proc/ppdrv/write

#Enable SDMA Interrupts
echo '2814 FFFFFFFF' > /proc/ppdrv/write
echo '2818 FFFFFFFF' > /proc/ppdrv/write
echo '34 FFFFFFFF' > /proc/ppdrv/write

#Port up and IP
ifconfig port0 1.1.1.1

-- Now you can run ping 1.1.1.10, f.e. can check cat /proc/net/arp and so on.
-- MAC address of the network device can be changed any time, it will be updated in switch.
-- See function mv_prestera_initialize() called from egiga0 init to learn the implemetation.


Known problems, restrictions and TODO list :
-- Port0 connected to switch interrupt GPP, so see next ...

-- Only port0 fully functional, next things should be impelemted :
---- On RX - get src port drom DSA Tag and switch network device of skb.
---- On TX - get port number from name of device and send to rigth switch port.

-- Checksum issues - some packets types dropped by Linux on checksum check ( or caused to some exception).
---- To fix the issue, see mvSwitchRx(), before call to netif_receive_skb() csum should be threated in rigth way.

-- In both RX and TX packets copied to/from prealoccated uncached memory from/to skb buffer.
---- To fix this performance issue SDMA rings buffers should use pointers to skb->data, 
	this is cached memory, so physical<->virtual traslation should be impelemted, 
	cached sync should be used and skb reuse/replace should be added too.

-- LE only, BE support will be added in next release.
