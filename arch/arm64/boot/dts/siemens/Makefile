# SPDX-License-Identifier: GPL-2.0
#
# Make file to build device tree binaries for IOT2050 boards
#
# Copyright (C) 2018-2019 Siemens AG
#

DTC_FLAGS += -@

dtb-$(CONFIG_ARCH_K3_AM6_SOC) += iot2050-basic.dtb \
	iot2050-advanced.dtb

$(obj)/%.dtbo: $(src)/%.dtso FORCE
	$(call if_changed_dep,dtc)

always		:= $(dtb-y)
clean-files	:= *.dtb *.dtbo
