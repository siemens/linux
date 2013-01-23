/*
 * drivers/net/ibm_newemac/phy.c
 *
 * Driver for PowerPC 4xx on-chip ethernet controller, PHY support.
 * Borrowed from sungem_phy.c, though I only kept the generic MII
 * driver for now.
 *
 * This file should be shared with other drivers or eventually
 * merged as the "low level" part of miilib
 *
 * Copyright 2007 Benjamin Herrenschmidt, IBM Corp.
 *                <benh@kernel.crashing.org>
 *
 * Based on the arch/ppc version of the driver:
 *
 * (c) 2003, Benjamin Herrenscmidt (benh@kernel.crashing.org)
 * (c) 2004-2005, Eugene Surovegin <ebs@ebshome.net>
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/netdevice.h>
#include <linux/mii.h>
#include <linux/ethtool.h>
#include <linux/delay.h>

#include "emac.h"
#include "phy.h"

#ifdef CONFIG_SIMATIC_NET_SCALANCE_W
extern int scw_get_sfp_speed(int);
#endif

static inline int phy_read(struct mii_phy *phy, int reg)
{
	return phy->mdio_read(phy->dev, phy->address, reg);
}

static inline void phy_write(struct mii_phy *phy, int reg, int val)
{
	phy->mdio_write(phy->dev, phy->address, reg, val);
}

static inline int gpcs_phy_read(struct mii_phy *phy, int reg)
{
	return phy->mdio_read(phy->dev, phy->gpcs_address, reg);
}

static inline void gpcs_phy_write(struct mii_phy *phy, int reg, int val)
{
	phy->mdio_write(phy->dev, phy->gpcs_address, reg, val);
}

int emac_mii_reset_phy(struct mii_phy *phy)
{
	int val;
	int limit = 10000;

	val = phy_read(phy, MII_BMCR);
	val &= ~(BMCR_ISOLATE | BMCR_ANENABLE);
	val |= BMCR_RESET;
	phy_write(phy, MII_BMCR, val);

	udelay(300);

	while (--limit) {
		val = phy_read(phy, MII_BMCR);
		if (val >= 0 && (val & BMCR_RESET) == 0)
			break;
		udelay(10);
	}
	if ((val & BMCR_ISOLATE) && limit > 0)
		phy_write(phy, MII_BMCR, val & ~BMCR_ISOLATE);

	return limit <= 0;
}

int emac_mii_reset_gpcs(struct mii_phy *phy)
{
	int val;
	int limit = 10000;

	val = gpcs_phy_read(phy, MII_BMCR);
	val &= ~(BMCR_ISOLATE | BMCR_ANENABLE);
	val |= BMCR_RESET;
	gpcs_phy_write(phy, MII_BMCR, val);

	udelay(300);

	while (--limit) {
		val = gpcs_phy_read(phy, MII_BMCR);
		if (val >= 0 && (val & BMCR_RESET) == 0)
			break;
		udelay(10);
	}
	if ((val & BMCR_ISOLATE) && limit > 0)
		gpcs_phy_write(phy, MII_BMCR, val & ~BMCR_ISOLATE);

	if (limit > 0 && phy->mode == PHY_MODE_SGMII) {
		/* Configure GPCS interface to recommended setting for SGMII */
		gpcs_phy_write(phy, 0x04, 0x8120); /* AsymPause, FDX */
		gpcs_phy_write(phy, 0x07, 0x2801); /* msg_pg, toggle */
		gpcs_phy_write(phy, 0x00, 0x0140); /* 1Gbps, FDX     */
	}

	return limit <= 0;
}

static int genmii_setup_aneg(struct mii_phy *phy, u32 advertise)
{
	int ctl, adv;

	phy->autoneg = AUTONEG_ENABLE;
	phy->speed = SPEED_10;
	phy->duplex = DUPLEX_HALF;
	phy->pause = phy->asym_pause = 0;
	phy->advertising = advertise;

	ctl = phy_read(phy, MII_BMCR);
	if (ctl < 0)
		return ctl;
	ctl &= ~(BMCR_FULLDPLX | BMCR_SPEED100 | BMCR_SPEED1000 | BMCR_ANENABLE);

	/* First clear the PHY */
	phy_write(phy, MII_BMCR, ctl);

	/* Setup standard advertise */
	adv = phy_read(phy, MII_ADVERTISE);
	if (adv < 0)
		return adv;
	adv &= ~(ADVERTISE_ALL | ADVERTISE_100BASE4 | ADVERTISE_PAUSE_CAP |
		 ADVERTISE_PAUSE_ASYM);
	if (advertise & ADVERTISED_10baseT_Half)
		adv |= ADVERTISE_10HALF;
	if (advertise & ADVERTISED_10baseT_Full)
		adv |= ADVERTISE_10FULL;
	if (advertise & ADVERTISED_100baseT_Half)
		adv |= ADVERTISE_100HALF;
	if (advertise & ADVERTISED_100baseT_Full)
		adv |= ADVERTISE_100FULL;
	if (advertise & ADVERTISED_Pause)
		adv |= ADVERTISE_PAUSE_CAP;
	if (advertise & ADVERTISED_Asym_Pause)
		adv |= ADVERTISE_PAUSE_ASYM;
	phy_write(phy, MII_ADVERTISE, adv);

	if (phy->features &
	    (SUPPORTED_1000baseT_Full | SUPPORTED_1000baseT_Half)) {
		adv = phy_read(phy, MII_CTRL1000);
		if (adv < 0)
			return adv;
		adv &= ~(ADVERTISE_1000FULL | ADVERTISE_1000HALF);
		if (advertise & ADVERTISED_1000baseT_Full)
			adv |= ADVERTISE_1000FULL;
		if (advertise & ADVERTISED_1000baseT_Half)
			adv |= ADVERTISE_1000HALF;
		phy_write(phy, MII_CTRL1000, adv);
	}

	/* Start/Restart aneg */
	ctl = phy_read(phy, MII_BMCR);
	ctl |= (BMCR_ANENABLE | BMCR_ANRESTART);
	phy_write(phy, MII_BMCR, ctl);

	return 0;
}

static int genmii_setup_forced(struct mii_phy *phy, int speed, int fd)
{
	int ctl;

	phy->autoneg = AUTONEG_DISABLE;
	phy->speed = speed;
	phy->duplex = fd;
	phy->pause = phy->asym_pause = 0;

	ctl = phy_read(phy, MII_BMCR);
	if (ctl < 0)
		return ctl;
	ctl &= ~(BMCR_FULLDPLX | BMCR_SPEED100 | BMCR_SPEED1000 | BMCR_ANENABLE);

	/* First clear the PHY */
	phy_write(phy, MII_BMCR, ctl | BMCR_RESET);

	/* Select speed & duplex */
	switch (speed) {
	case SPEED_10:
		break;
	case SPEED_100:
		ctl |= BMCR_SPEED100;
		break;
	case SPEED_1000:
		ctl |= BMCR_SPEED1000;
		break;
	default:
		return -EINVAL;
	}
	if (fd == DUPLEX_FULL)
		ctl |= BMCR_FULLDPLX;
	phy_write(phy, MII_BMCR, ctl);

	return 0;
}

static int genmii_poll_link(struct mii_phy *phy)
{
	int status;

	/* Clear latched value with dummy read */
	phy_read(phy, MII_BMSR);
	status = phy_read(phy, MII_BMSR);
	if (status < 0 || (status & BMSR_LSTATUS) == 0)
		return 0;
	if (phy->autoneg == AUTONEG_ENABLE && !(status & BMSR_ANEGCOMPLETE))
		return 0;
	return 1;
}

static int genmii_read_link(struct mii_phy *phy)
{
	if (phy->autoneg == AUTONEG_ENABLE) {
		int glpa = 0;
		int lpa = phy_read(phy, MII_LPA) & phy_read(phy, MII_ADVERTISE);
		if (lpa < 0)
			return lpa;

		if (phy->features &
		    (SUPPORTED_1000baseT_Full | SUPPORTED_1000baseT_Half)) {
			int adv = phy_read(phy, MII_CTRL1000);
			glpa = phy_read(phy, MII_STAT1000);

			if (glpa < 0 || adv < 0)
				return adv;

			glpa &= adv << 2;
		}

		phy->speed = SPEED_10;
		phy->duplex = DUPLEX_HALF;
		phy->pause = phy->asym_pause = 0;

		if (glpa & (LPA_1000FULL | LPA_1000HALF)) {
			phy->speed = SPEED_1000;
			if (glpa & LPA_1000FULL)
				phy->duplex = DUPLEX_FULL;
		} else if (lpa & (LPA_100FULL | LPA_100HALF)) {
			phy->speed = SPEED_100;
			if (lpa & LPA_100FULL)
				phy->duplex = DUPLEX_FULL;
		} else if (lpa & LPA_10FULL)
			phy->duplex = DUPLEX_FULL;

		if (phy->duplex == DUPLEX_FULL) {
			phy->pause = lpa & LPA_PAUSE_CAP ? 1 : 0;
			phy->asym_pause = lpa & LPA_PAUSE_ASYM ? 1 : 0;
		}
	} else {
		int bmcr = phy_read(phy, MII_BMCR);
		if (bmcr < 0)
			return bmcr;

		if (bmcr & BMCR_FULLDPLX)
			phy->duplex = DUPLEX_FULL;
		else
			phy->duplex = DUPLEX_HALF;
		if (bmcr & BMCR_SPEED1000)
			phy->speed = SPEED_1000;
		else if (bmcr & BMCR_SPEED100)
			phy->speed = SPEED_100;
		else
			phy->speed = SPEED_10;

		phy->pause = phy->asym_pause = 0;
	}
	return 0;
}

/* Generic implementation for most 10/100/1000 PHYs */
static struct mii_phy_ops generic_phy_ops = {
	.setup_aneg	= genmii_setup_aneg,
	.setup_forced	= genmii_setup_forced,
	.poll_link	= genmii_poll_link,
	.read_link	= genmii_read_link
};

static struct mii_phy_def genmii_phy_def = {
	.phy_id		= 0x00000000,
	.phy_id_mask	= 0x00000000,
	.name		= "Generic MII",
	.ops		= &generic_phy_ops
};

/* CIS8201 */
#define MII_CIS8201_10BTCSR	0x16
#define  TENBTCSR_ECHO_DISABLE	0x2000
#define MII_CIS8201_EPCR	0x17
#define  EPCR_MODE_MASK		0x3000
#define  EPCR_GMII_MODE		0x0000
#define  EPCR_RGMII_MODE	0x1000
#define  EPCR_TBI_MODE		0x2000
#define  EPCR_RTBI_MODE		0x3000
#define MII_CIS8201_ACSR	0x1c
#define  ACSR_PIN_PRIO_SELECT	0x0004

static int cis8201_init(struct mii_phy *phy)
{
	int epcr;

	epcr = phy_read(phy, MII_CIS8201_EPCR);
	if (epcr < 0)
		return epcr;

	epcr &= ~EPCR_MODE_MASK;

	switch (phy->mode) {
	case PHY_MODE_TBI:
		epcr |= EPCR_TBI_MODE;
		break;
	case PHY_MODE_RTBI:
		epcr |= EPCR_RTBI_MODE;
		break;
	case PHY_MODE_GMII:
		epcr |= EPCR_GMII_MODE;
		break;
	case PHY_MODE_RGMII:
	default:
		epcr |= EPCR_RGMII_MODE;
	}

	phy_write(phy, MII_CIS8201_EPCR, epcr);

	/* MII regs override strap pins */
	phy_write(phy, MII_CIS8201_ACSR,
		  phy_read(phy, MII_CIS8201_ACSR) | ACSR_PIN_PRIO_SELECT);

	/* Disable TX_EN -> CRS echo mode, otherwise 10/HDX doesn't work */
	phy_write(phy, MII_CIS8201_10BTCSR,
		  phy_read(phy, MII_CIS8201_10BTCSR) | TENBTCSR_ECHO_DISABLE);

	return 0;
}

static struct mii_phy_ops cis8201_phy_ops = {
	.init		= cis8201_init,
	.setup_aneg	= genmii_setup_aneg,
	.setup_forced	= genmii_setup_forced,
	.poll_link	= genmii_poll_link,
	.read_link	= genmii_read_link
};

static struct mii_phy_def cis8201_phy_def = {
	.phy_id		= 0x000fc410,
	.phy_id_mask	= 0x000ffff0,
	.name		= "CIS8201 Gigabit Ethernet",
	.ops		= &cis8201_phy_ops
};

static struct mii_phy_def bcm5248_phy_def = {

	.phy_id		= 0x0143bc00,
	.phy_id_mask	= 0x0ffffff0,
	.name		= "BCM5248 10/100 SMII Ethernet",
	.ops		= &generic_phy_ops
};

static int m88e1111_init(struct mii_phy *phy)
{
	pr_debug("%s: Marvell 88E1111 Ethernet\n", __func__);
	phy_write(phy, 0x14, 0x0ce3);
	phy_write(phy, 0x18, 0x4101);
	phy_write(phy, 0x09, 0x0e00);
	phy_write(phy, 0x04, 0x01e1);
	phy_write(phy, 0x00, 0x9140);
	phy_write(phy, 0x00, 0x1140);

	return  0;
}

static int m88e1112_init(struct mii_phy *phy)
{
	/*
	 * Marvell 88E1112 PHY needs to have the SGMII MAC
	 * interace (page 2) properly configured to
	 * communicate with the 460EX/GT GPCS interface.
	 */

	u16 reg_short;

	pr_debug("%s: Marvell 88E1112 Ethernet\n", __func__);

	/* Set access to Page 2 */
	phy_write(phy, 0x16, 0x0002);

	phy_write(phy, 0x00, 0x0040); /* 1Gbps */
	reg_short = (u16)(phy_read(phy, 0x1a));
	reg_short |= 0x8000; /* bypass Auto-Negotiation */
	phy_write(phy, 0x1a, reg_short);
	emac_mii_reset_phy(phy); /* reset MAC interface */

	/* Reset access to Page 0 */
	phy_write(phy, 0x16, 0x0000);

	return  0;
}

static int m88e1141_init(struct mii_phy *phy)
{
        unsigned short data;

	printk(KERN_CRIT "we go to init for %d\n", phy->mode);
        switch (phy->mode) {
        case PHY_MODE_GMII:
#if defined(CONFIG_M88E1141_DEBUG)
                data = phy_read(phy, 0x00);
                        data |= 0x2000; /* Speed Select 1000Mbps */
                        phy_write(phy, 0x00, data);
                data = phy_read(phy, 0x14);
                        data |= 0x0010;  /* GMII Deafult MAC interface speed */
                        phy_write(phy, 0x14, data);
                data = phy_read(phy, 0x1B);
                        data |= 0x8000; /* Auto Selection = Disable */
                        data |= 0x0400; /* Interrupt Polarity = Active Low */
                        data |= 0x0080; /* DTE Detect Status wait time */
                        data |= 0x000F; /* HWCFG_MODE = GMII */
                        phy_write(phy, 0x1B, data);
                data = phy_read(phy, 0x04);
                        data |= 0x0C00; /* Async Pause + Pause */
                        data |= 0x01E0; /* 100FDX + 100HDX + 10FDX + 10HDX */
                        phy_write(phy, 0x04, data);
                data = phy_read(phy, 0x09);
                        //data |= 0x1C00; /* Master/Slave Config */
                        data |= 0x0300; /* 1000FDX + 1000HDX */
                        phy_write(phy, 0x09, data);
#else
                data = phy_read(phy, 0x14);
                        data |= 0x0010;  /* GMII Deafult MAC interface speed */
                        phy_write(phy, 0x14, data);
                data = phy_read(phy, 0x1B);
                        data |= 0x000F; /* HWCFG_MODE = GMII */
                        phy_write(phy, 0x1B, data);
#endif
                break;
        case PHY_MODE_RGMII:
#if defined(CONFIG_M88E1141_DEBUG)
                data = phy_read(phy, 0x00);
                        data |= 0x2000; /* Speed Select 1000Mbps */
                        phy_write(phy, 0x00, data);
                data = phy_read(phy, 0x14);
                        data |= 0x0080; /* RGMII RX Timing Control */
                        data |= 0x0002; /* RGMII TX Timing Control */
                        data |= 0x0050; /* RGMII Deafult MAC interface speed */
                        phy_write(phy, 0x14, data);
                data = phy_read(phy, 0x1B);
                        data |= 0x8000; /* Auto Selection = Disable */
                        data |= 0x0400; /* Interrupt Polarity = Active Low */
                        data |= 0x0080; /* DTE Detect Status wait time */
                        data |= 0x000B; /* HWCFG_MODE = RGMII */
                        phy_write(phy, 0x1B, data);
                data = phy_read(phy, 0x04);
                        data |= 0x0C00; /* Async Pause + Pause */
                        data |= 0x01E0; /* 100FDX + 100HDX + 10FDX + 10HDX */
                        phy_write(phy, 0x04, data);
                data = phy_read(phy, 0x09);
                        //data |= 0x1C00; /* Master/Slave Config */
                        data |= 0x0300; /* 1000FDX + 1000HDX */
                        phy_write(phy, 0x09, data);
#else
                data = phy_read(phy, 0x14);
                        data |= 0x0080; /* RGMII RX Timing Control */
                        data |= 0x0002; /* RGMII TX Timing Control */
                        data |= 0x0050; /* RGMII Deafult MAC interface speed */
                        phy_write(phy, 0x14, data);
                data = phy_read(phy, 0x1B);
                        data |= 0x000B; /* HWCFG_MODE = RGMII */
                        phy_write(phy, 0x1B, data);
#endif
                break;
        case PHY_MODE_SGMII:
                data = phy_read(phy, 0x14);
                        data &= ~0x0080; /* CLEAR - RGMII setting */
                        data &= ~0x0002; /* CLEAR - RGMII setting */
                        data &= ~0x0070; /* CLEAR - Default MAC speed */
                        data |= 0x0070;  /* GMII Deafult MAC interface speed */
                        phy_write(phy, 0x14, data);

                data = phy_read(phy, 0x1B);
                        data |= 0x8000; /* Auto Selection = Disable */
                        data &= ~0x0400; /* Interrupt Polarity = Active Low */
                        data |= 0x0120; /* DTE Detect Status wait time */
                        data &= ~0x000F;/* CLEAR - HWCFG_MODE setting */
                        data |= 0x0000; /* HWCFG_MODE = SGMII */
                        phy_write(phy, 0x1B, data);

                phy_write(phy, 0x10, 0x0068);
                phy_write(phy, 0x16, 0x0001);
                phy_write(phy, 0x00, 0x8100);
                phy_write(phy, 0x16, 0x0000);
                break;
        }

#if 0
        data = phy_read(phy, 0x00);
        data |= 0x8000; /* Reset PHY */
        phy_write(phy, 0x00, data);
        udelay(1000);
#endif

        return  0;
}

static int et1011c_init(struct mii_phy *phy)
{
	u16 reg_short;

	reg_short = (u16)(phy_read(phy, 0x16));
	reg_short &= ~(0x7);
	reg_short |= 0x6;	/* RGMII Trace Delay*/
	phy_write(phy, 0x16, reg_short);

	reg_short = (u16)(phy_read(phy, 0x17));
	reg_short &= ~(0x40);
	phy_write(phy, 0x17, reg_short);

	phy_write(phy, 0x1c, 0x74f0);
	return 0;
}

static struct mii_phy_ops et1011c_phy_ops = {
	.init		= et1011c_init,
	.setup_aneg	= genmii_setup_aneg,
	.setup_forced	= genmii_setup_forced,
	.poll_link	= genmii_poll_link,
	.read_link	= genmii_read_link
};

static struct mii_phy_def et1011c_phy_def = {
	.phy_id		= 0x0282f000,
	.phy_id_mask	= 0x0fffff00,
	.name		= "ET1011C Gigabit Ethernet",
	.ops		= &et1011c_phy_ops
};





static struct mii_phy_ops m88e1111_phy_ops = {
	.init		= m88e1111_init,
	.setup_aneg	= genmii_setup_aneg,
	.setup_forced	= genmii_setup_forced,
	.poll_link	= genmii_poll_link,
	.read_link	= genmii_read_link
};

static struct mii_phy_def m88e1111_phy_def = {

	.phy_id		= 0x01410CC0,
	.phy_id_mask	= 0x0ffffff0,
	.name		= "Marvell 88E1111 Ethernet",
	.ops		= &m88e1111_phy_ops,
};

static struct mii_phy_ops m88e1112_phy_ops = {
	.init		= m88e1112_init,
	.setup_aneg	= genmii_setup_aneg,
	.setup_forced	= genmii_setup_forced,
	.poll_link	= genmii_poll_link,
	.read_link	= genmii_read_link
};

static struct mii_phy_def m88e1112_phy_def = {
	.phy_id		= 0x01410C90,
	.phy_id_mask	= 0x0ffffff0,
	.name		= "Marvell 88E1112 Ethernet",
	.ops		= &m88e1112_phy_ops,
};

static struct mii_phy_ops m88e1141_phy_ops = {
        .init           = m88e1141_init,
        .setup_aneg     = genmii_setup_aneg,
        .setup_forced   = genmii_setup_forced,
        .poll_link      = genmii_poll_link,
        .read_link      = genmii_read_link
};

static struct mii_phy_def m88e1141_phy_def = {
        .phy_id         = 0x01410CD0,
        .phy_id_mask    = 0x0ffffff0,
        .name           = "Marvell 88E1141 Ethernet",
        .ops            = &m88e1141_phy_ops,
};

#ifdef CONFIG_SIMATIC_NET_SCALANCE_W
static int m88e6122_init(struct mii_phy *phy)
{
	int reg;
	int sfp_speed[2];
	int orig_phy = phy->address;
	static int configured = 0;

	/* SFP Variant -> else we don't  this PHY ID */
	printk("ScalanceW SFP Variant -> init Marvell 88E6122 (%d)\n", phy->address);

	sfp_speed[0] = scw_get_sfp_speed(0);
	sfp_speed[1] = scw_get_sfp_speed(1);
	printk("SFP1 (%d), SFP2 (%d)\n", sfp_speed[0], sfp_speed[1]);

	if (sfp_speed[0] != -1)
		orig_phy = 0xc;
	else if (sfp_speed[1] != -1)
		orig_phy = 0xd;

	/* Configuring the SERDES 4 & 5 - phy addr 0xc & 0xd */
	for (phy->address=0xc; phy->address<=0xd; phy->address++) {
		reg = phy_read(phy, MII_ADVERTISE);
		if (sfp_speed[phy->address - 0xc] == 1000)
			reg |= ADVERTISE_1000XFULL;
		phy_write(phy, MII_ADVERTISE, reg);
		udelay(10000);

		/* Control Register 1 */
		reg = phy_read(phy, 0x10) & ~(0x0073); /* clear i/f speed and mode bits */
		switch (sfp_speed[phy->address - 0xc]) {
		case 1000:
			reg |= 0x0061;
			break;
		case 100:
			reg |= 0x0050;
			break;
		}
		reg |= 0x0008;
		phy_write(phy, 0x10, reg);
		udelay(10000);

		reg = BMCR_FULLDPLX | BMCR_RESET;
		switch (sfp_speed[phy->address - 0xc]) {
		case 1000:
			reg |= BMCR_ANENABLE;
			reg |= BMCR_SPEED1000;
			break;
		case 100:
			reg |= BMCR_SPEED100;
			break;
		}
		phy_write(phy, MII_BMCR, reg);
		udelay(10000);
	}

	/* Port 3 config - phy addr 13 */
	if (!configured)
	{
		phy->address = 0x13;
		/* mac forcing reg */
		reg = phy_read(phy, 0x1);
		reg &= ~(0x000f);
		reg |= 0x000e;
		phy_write(phy, 0x1, reg);
		udelay(10000);
		/* port control reg - set forwarding */
		reg = phy_read(phy, 0x4);
		reg |= 0x0007;
		phy_write(phy, 0x4, reg);
		udelay(10000);
	}

	/* Port config for serdes -> phy addr 0x14 & 0x15 */
	for (phy->address=0x14; phy->address<=0x15; phy->address++) {
		/* port status reg */
		reg = phy_read(phy, 0);
		reg |= 0x0040;
		phy_write(phy, 0, reg);
		udelay(10000);
		/* mac forcing reg */
		reg = phy_read(phy, 0x1);
		reg &= ~(0x000f);
		switch (sfp_speed[phy->address - 0x14]) {
		case 1000:
			reg |= 0x000e;
			break;
		case 100:
			reg |= 0x000d;
			break;
		}
		phy_write(phy, 0x1, reg);
		udelay(10000);
		/* port control reg -> set forwarding */
		reg = phy_read(phy, 0x4);
		reg |= 0x0007;
		phy_write(phy, 0x4, reg);
		udelay(10000);
	}

	if (!configured)
	{
		/* Switch Registers bank -> phy addr 0x1b */
		phy->address =0x1b;
		/* control register */
		reg = phy_read(phy, 0x4);
		reg &= ~(0x007f);
		reg |= 0x4000;
		phy_write(phy, 0x4, reg);
		udelay(10000);
		configured = 1;
	}

	/* back to our configured port */
	phy->address = orig_phy;
	return 0;
}

/* DD: What do we need to do in these setups?? */
static int m88e6122_setup_aneg(struct mii_phy *phy, u32 advertise)
{
	printk("%s\n", __func__);
	return 0;
}

static int m88e6122_setup_forced(struct mii_phy *phy, int speed, int fd)
{
	printk("%s\n", __func__);
	return 0;
}

int global_sfp1_link = 0;
int global_sfp2_link = 0;

static int m88e6122_poll_link(struct mii_phy *phy)
{
	int status, sfp1_new_link = 0, sfp2_new_link = 0;
	int orig_phy = phy->address;

	/* Check SERDES ports (0xC & 0xD) if either link is present */
	for (phy->address=0xc; phy->address<=0xd; phy->address++) {
		/* clear old value of BMSR */
		phy_read(phy, MII_BMSR);
		status = phy_read(phy, MII_BMSR);
		if (status < 0 || (status & BMSR_LSTATUS) == 0)
			continue;
		if (phy->autoneg == AUTONEG_ENABLE && !(status & BMSR_ANEGCOMPLETE))
			continue;
		/* found link */
		if(phy->address==0xc)
			sfp1_new_link = 1;
		else if(phy->address==0xd)
			sfp2_new_link = 1;
		break;
	}
	phy->address = orig_phy;
/*
	if(!global_sfp1_link && sfp1_new_link)
	{
		printk("SFP1 LINK UP\n");
	}
	if(!global_sfp2_link && sfp2_new_link)
	{
		printk("SFP2 LINK UP\n");
	}
*/
	global_sfp1_link = sfp1_new_link;
	global_sfp2_link = sfp2_new_link;
	return (global_sfp1_link || global_sfp2_link ? 1 : 0);
}

static struct mii_phy_ops m88e6122_phy_ops = {
        .init           = m88e6122_init,
        .setup_aneg     = m88e6122_setup_aneg,
        .setup_forced   = m88e6122_setup_forced,
        .poll_link      = m88e6122_poll_link,
        .read_link      = genmii_read_link
};

/* Checking for PHY ID of SERDES ports for M88E6122 */
static struct mii_phy_def m88e6122_phy_def = {
        .phy_id         = 0x01410C00,
        .phy_id_mask    = 0x0ffffff0,
        .name           = "Marvell 88E6122 Ethernet",
        .ops            = &m88e6122_phy_ops,
};
#endif

static struct mii_phy_def *mii_phy_table[] = {
	&et1011c_phy_def,
	&cis8201_phy_def,
	&bcm5248_phy_def,
	&m88e1111_phy_def,
	&m88e1112_phy_def,
	&m88e1141_phy_def,
#ifdef CONFIG_SIMATIC_NET_SCALANCE_W
	&m88e6122_phy_def,
#endif
	&genmii_phy_def,
	NULL
};

int emac_mii_phy_probe(struct mii_phy *phy, int address)
{
	struct mii_phy_def *def;
	int i;
	u32 id;

	phy->autoneg = AUTONEG_DISABLE;
	phy->advertising = 0;
	phy->address = address;
	phy->speed = SPEED_10;
	phy->duplex = DUPLEX_HALF;
	phy->pause = phy->asym_pause = 0;

	/* Take PHY out of isolate mode and reset it. */
	if (emac_mii_reset_phy(phy))
		return -ENODEV;

	/* Read ID and find matching entry */
	id = (phy_read(phy, MII_PHYSID1) << 16) | phy_read(phy, MII_PHYSID2);
	for (i = 0; (def = mii_phy_table[i]) != NULL; i++)
		if ((id & def->phy_id_mask) == def->phy_id)
			break;
	/* Should never be NULL (we have a generic entry), but... */
	if (!def)
		return -ENODEV;

	phy->def = def;

	/* Determine PHY features if needed */
	phy->features = def->features;
	if (!phy->features) {
		u16 bmsr = phy_read(phy, MII_BMSR);
		if (bmsr & BMSR_ANEGCAPABLE)
			phy->features |= SUPPORTED_Autoneg;
		if (bmsr & BMSR_10HALF)
			phy->features |= SUPPORTED_10baseT_Half;
		if (bmsr & BMSR_10FULL)
			phy->features |= SUPPORTED_10baseT_Full;
		if (bmsr & BMSR_100HALF)
			phy->features |= SUPPORTED_100baseT_Half;
		if (bmsr & BMSR_100FULL)
			phy->features |= SUPPORTED_100baseT_Full;
		if (bmsr & BMSR_ESTATEN) {
			u16 esr = phy_read(phy, MII_ESTATUS);
			if (esr & ESTATUS_1000_TFULL)
				phy->features |= SUPPORTED_1000baseT_Full;
			if (esr & ESTATUS_1000_THALF)
				phy->features |= SUPPORTED_1000baseT_Half;
		}
		phy->features |= SUPPORTED_MII;
	}

	/* Setup default advertising */
	phy->advertising = phy->features;

	return 0;
}

MODULE_LICENSE("GPL");
