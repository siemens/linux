#ifndef _VME_USER_H_
#define _VME_USER_H_

#define VME_USER_BUS_MAX	1

/*
 * IOCTL Commands and structures
 */

/* Magic number for use in ioctls */
#define VME_IOC_MAGIC 0xAE

struct vme_irq_id {
	__u8 level;
	__u8 statid;
	unsigned int timeout_usec;
};

struct vme_rmw {
	unsigned int mask;
	unsigned int compare;
	unsigned int swap;
};

#define VME_GET_SLAVE _IOR(VME_IOC_MAGIC, 1, struct vme_slave)
#define VME_SET_SLAVE _IOW(VME_IOC_MAGIC, 2, struct vme_slave)
#define VME_GET_MASTER _IOR(VME_IOC_MAGIC, 3, struct vme_master)
#define VME_SET_MASTER _IOW(VME_IOC_MAGIC, 4, struct vme_master)
#define VME_IRQ_GEN _IOW(VME_IOC_MAGIC, 5, struct vme_irq_id)
#define VME_RMW _IOW(VME_IOC_MAGIC, 6, struct vme_rmw)
#define VME_GET_SLOT_ID _IO(VME_IOC_MAGIC, 7)
#define VME_GET_STATUS _IOR(VME_IOC_MAGIC, 8, struct vme_status)

#endif /* _VME_USER_H_ */
