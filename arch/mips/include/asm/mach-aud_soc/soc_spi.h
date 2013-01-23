/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2012 Manfred.Neugebauer@siemens.com
 */

struct soc_spi_info {
    unsigned soc_spi_sspcr0;
    unsigned soc_spi_sspcr1;
    unsigned soc_spi_sspdr;
    unsigned soc_spi_sspsr;
    unsigned soc_spi_sspcpsr;
    unsigned soc_spi_sspiir_sspicr;
};


int aud_soc_init_spi(void);
int add_soc_spi_devices(void);
