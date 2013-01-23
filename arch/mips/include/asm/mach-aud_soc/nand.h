#ifndef __SOC1_NAND_H
#define __SOC1_NAND_H

#define SOC1_NAND_BASE_ADDR   0xB8000000

#define CLE_ADDRESS    0x40
#define ALE_ADDRESS    0x80

#define CLE_PTR (unsigned char *)(SOC1_NAND_BASE_ADDR + CLE_ADDRESS)
#define ALE_PTR (unsigned char *)(SOC1_NAND_BASE_ADDR + ALE_ADDRESS)


struct soc_nand_info;

struct soc_nand_mtd {
	struct mtd_info			mtd;
	struct nand_chip		chip;
	struct soc_nand_set		*set;
	struct soc_nand_info	*info;
	int				scan_res;
};

/* overview of the soc nand state */

struct soc_nand_info {
	/* mtd info */
	struct nand_hw_control		controller;
	struct soc_nand_mtd		*mtds;
	struct soc_platform_nand	*platform;

	/* device info */
	struct device			*device;
	struct resource			*area;
	struct clk			*clk;
	void __iomem			*regs;
	int				mtd_count;
};


struct soc_nand_set {
	int			nr_chips;
	int			nr_partitions;
	char			*name;
	int			*nr_map;
	struct mtd_partition	*partitions;
};

struct soc_platform_nand {
	/* timing information for controller, all times in nanoseconds */

	int	tacls;	/* time for active CLE/ALE to nWE/nOE */
	int	twrph0;	/* active time for nWE/nOE */
	int	twrph1;	/* time for release CLE/ALE from nWE/nOE inactive */

	int			nr_sets;
	struct soc_nand_set *sets;

	void			(*select_chip)(struct soc_nand_set *,
					       int chip);
};

#endif
