/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/* Portions Copyright (c) 2018-2022 Siemens Mobility GmbH */
#ifndef _LINUX_GSMMUX_H
#define _LINUX_GSMMUX_H

#include <linux/if.h>
#include <linux/ioctl.h>
#include <linux/types.h>

struct gsm_config {
	unsigned int adaption;   /* Convergence layer type */
	unsigned int encapsulation; /* Framing (0 = basic option, 1 = advanced option) */
	unsigned int initiator;  /* Initiator or responder */
	unsigned int t1;         /* Acknowledgment timer */
	unsigned int t2;         /* Response timer for multiplexer control channel */
	unsigned int t3;         /* Response timer for wake-up procedure */
	unsigned int n2;         /* Maximum number of retransmissions */
	unsigned int mru;        /* Maximum incoming frame payload size */
	unsigned int mtu;        /* Maximum outgoing frame payload size */
	unsigned int k;          /* Window size */
	unsigned int i;          /* Frame type (1 = UIH, 2 = UI) */
	unsigned int port;       /* Instance */
	unsigned int keep_alive; /* Control channel keep-alive 1/100th of a sec (0 to disable) */
	unsigned int wait_config; /* Wait for DLCI config before opening virtual link? */
	unsigned int restart;    /* Force link reset? */
	unsigned int sof_intvl;  /* Start-of-frame send interval in 1/100th of a sec (skipped in-between) */
	unsigned int unused[3];  /* Padding for expansion without breaking stuff */
};

#define GSMIOC_GETCONF           _IOR('G', 0, struct gsm_config)
#define GSMIOC_SETCONF           _IOW('G', 1, struct gsm_config)

struct gsm_netconfig {
	unsigned int adaption;   /* Adaption to use in network mode */
	unsigned short protocol; /* Protocol to use - only ETH_P_IP supported */
	unsigned short unused2;  /* Padding */
	char if_name[IFNAMSIZ];  /* Interface name format string */
	__u8 unused[28];         /* For future use */
};

#define GSMIOC_ENABLE_NET        _IOW('G', 2, struct gsm_netconfig)
#define GSMIOC_DISABLE_NET       _IO('G', 3)

/* Get the base tty number for a configured gsmmux tty. */
#define GSMIOC_GETFIRST		_IOR('G', 4, __u32)

/* Set channel accordingly before calling GSMIOC_GETCONF_DLCI. */
struct gsm_dlci_config {
	unsigned int channel;    /* DLCI (0 for the associated DLCI) */
	unsigned int adaption;   /* Convergence layer type */
	unsigned int mtu;        /* Maximum transfer unit */
	unsigned int priority;   /* Priority (0 for default value) */
	unsigned int i;          /* Frame type (1 = UIH, 2 = UI) */
	unsigned int k;          /* Window size (0 for default value) */
	unsigned int restart;    /* Force DLCI channel reset? */
	unsigned int unused[7];  /* Padding for expansion without breaking stuff */
};

#define GSMIOC_GETCONF_DLCI      _IOWR('G', 5, struct gsm_dlci_config)
#define GSMIOC_SETCONF_DLCI      _IOW('G', 6, struct gsm_dlci_config)


#endif
