// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) Siemens AG, 2023-2024
 *
 * Driver for Everlight PM-16d17 proximity sensor
 *
 * Author: Chao Zeng <chao.zeng@siemens.com>
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/events.h>

#define PM16D17_DRV_NAME		"pm16d17"
#define PM16D17_REGMAP_NAME		"pm16d17_regmap"

/* Registers Address */
#define PM16D17_OP_MODE			0x00
#define PM16D17_INTERRUPT_FLAG		0x01
#define PM16D17_PS_SETTING		0x0A
#define PM16D17_VCSEL_DRIVE_CURRENT	0x0B
#define PM16D17_VCSEL_DRIVE_PULSE	0x0C
#define PM16D17_PS_INTUPT_LTHD_L	0x0D
#define PM16D17_PS_INTUPT_LTHD_H	0x0E
#define PM16D17_PS_INTUPT_HTHD_L	0x0F
#define PM16D17_PS_INTUPT_HTHD_H	0x10
#define PM16D17_PS_DATA_L		0x11
#define PM16D17_PS_DATA_H		0x12
#define PM16D17_PS_SETTING2		0x13
#define PM16D17_PS_OFFSET_CANCEL_L	0x14
#define PM16D17_PS_OFFSET_CANCEL_H	0x15
#define PM16D17_DEV_ID			0x18

#define DEVICE_ID			0x11

#define ENABLE_PS_FUNCTION		BIT(3)
#define PS_GAIN_MASK			GENMASK(7, 6)
#define PS_ITIME_MASK			GENMASK(5, 3)
#define PS_WTIME_MASK			GENMASK(2, 0)
#define OFFSET_CANCEL_ENABLE		BIT(7)
#define PS_OFFSET_CANCEL_LSB_MASK	GENMASK(7, 0)
#define PS_OFFSET_CANCEL_MSB_MASK	GENMASK(15, 8)

enum {
	PITIME_0_POINT_4_MS = (0 << 3),
	PITIME_0_POINT_8_MS = (1 << 3),
	PITIME_1_POINT_6_MS = (2 << 3),
	PITIME_3_POINT_2_MS = (3 << 3),
	PITIME_6_POINT_3_MS = (4 << 3),
	PITIME_12_POINT_6_MS = (5 << 3),
	PITIME_25_POINT_2_MS = (6 << 3),
};

enum {
	PWTIME_12_POINT_5_MS = 0,
	PWTIME_25_MS,
	PWTIME_50_MS,
	PWTIME_100_MS,
	PWTIME_200_MS,
	PWTIME_400_MS,
	PWTIME_800_MS,
	PWTIME_1600_MS,
};

struct pm16d17_data {
	struct i2c_client *client;
	struct regmap *regmap;
};

static const struct regmap_config pm16d17_regmap_config = {
	.name = PM16D17_REGMAP_NAME,
	.reg_bits = 8,
	.val_bits = 8,
	.cache_type = REGCACHE_NONE,
};

static const struct iio_chan_spec pm16d17_channels[] = {
	{
		.type = IIO_PROXIMITY,
		.indexed = 1,
		.channel = 0,
		.scan_index = -1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	}
};

static inline int pm16d17_write_reg(struct pm16d17_data *data,
				    unsigned int reg,
				    unsigned int value)
{
	return regmap_write(data->regmap, reg, value);
}

static inline unsigned int pm16d17_read_reg(struct pm16d17_data *data,
					    unsigned int reg,
					    unsigned int *reg_val)
{
	return regmap_read(data->regmap, reg, reg_val);
}

static int pm16d17_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	struct pm16d17_data *data = iio_priv(indio_dev);
	unsigned int ps_data_l;
	unsigned int ps_data_h;
	uint16_t ps_data;
	int ret = -EINVAL;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		switch (chan->type) {
		case IIO_PROXIMITY:
			ret = pm16d17_read_reg(data, PM16D17_PS_DATA_L, &ps_data_l);
			if (ret < 0)
				return ret;

			ret = pm16d17_read_reg(data, PM16D17_PS_DATA_H, &ps_data_h);
			if (ret < 0)
				return ret;

			ps_data = (ps_data_h << 8) | ps_data_l;

			dev_dbg(&data->client->dev, "PS data: %x\n", ps_data);

			*val = ps_data;
			ret = IIO_VAL_INT;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}

	return ret;
}

static const struct iio_info pm16d17_info = {
	.read_raw = pm16d17_read_raw,
};

static int pm16d17_chip_init(struct pm16d17_data *data)
{
	struct i2c_client *client = data->client;
	struct device_node *np = client->dev.of_node;
	uint8_t op_mode_setting_val = 0;
	const char *conv_time = NULL;
	const char *wait_time = NULL;
	uint32_t ps_offset_cancel;
	uint8_t offset_lsb;
	uint8_t offset_msb;
	uint32_t pulse_count;
	uint32_t pgain;
	unsigned int val;
	int ret;

	ret = pm16d17_read_reg(data, PM16D17_DEV_ID, &val);

	if (ret < 0 || (val != DEVICE_ID)) {
		dev_err(&client->dev, "Invalid chip id 0x%04x\n", val);
		return -ENODEV;
	}

	dev_dbg(&client->dev, "Detected PM16D17 with chip id: 0x%04x\n", val);

	ret = pm16d17_write_reg(data, PM16D17_OP_MODE, ENABLE_PS_FUNCTION);
	if (ret < 0)
		return ret;

	of_property_read_u32(np, "ps-gain", &pgain);
	switch (pgain) {
	case 1:
	case 2:
	case 4:
	case 8:
		op_mode_setting_val |= (ilog2(pgain) << 6) & PS_GAIN_MASK;
		break;
	default:
		break;
	}

	if (of_property_read_string(np, "ps-itime", &conv_time) < 0) {
		dev_err(&client->dev, "Missing ps-itime property\n");
		return -EINVAL;
	}
	if (strcmp(conv_time, "0.4") == 0)
		op_mode_setting_val |= PITIME_0_POINT_4_MS & PS_ITIME_MASK;
	else if (strcmp(conv_time, "0.8") == 0)
		op_mode_setting_val |= PITIME_0_POINT_8_MS & PS_ITIME_MASK;
	else if (strcmp(conv_time, "1.6") == 0)
		op_mode_setting_val |= PITIME_1_POINT_6_MS & PS_ITIME_MASK;
	else if (strcmp(conv_time, "3.2") == 0)
		op_mode_setting_val |= PITIME_3_POINT_2_MS & PS_ITIME_MASK;
	else if (strcmp(conv_time, "6.3") == 0)
		op_mode_setting_val |= PITIME_6_POINT_3_MS & PS_ITIME_MASK;
	else if (strcmp(conv_time, "12.6") == 0)
		op_mode_setting_val |= PITIME_12_POINT_6_MS & PS_ITIME_MASK;
	else if (strcmp(conv_time, "25.2") == 0)
		op_mode_setting_val |= PITIME_25_POINT_2_MS & PS_ITIME_MASK;
	else {
		dev_info(&client->dev, "Using default ps itime value\n");
		op_mode_setting_val |= PITIME_0_POINT_4_MS & PS_ITIME_MASK;
	}

	if (of_property_read_string(np, "ps-wtime", &wait_time) < 0) {
		dev_err(&client->dev, "Missing ps-wtime property\n");
		return -EINVAL;
	}
	if (strcmp(wait_time, "12.5") == 0)
		op_mode_setting_val |= PWTIME_12_POINT_5_MS & PS_WTIME_MASK;
	else if (strcmp(wait_time, "25") == 0)
		op_mode_setting_val |= PWTIME_25_MS & PS_WTIME_MASK;
	else if (strcmp(wait_time, "50") == 0)
		op_mode_setting_val |= PWTIME_50_MS & PS_WTIME_MASK;
	else if (strcmp(wait_time, "100") == 0)
		op_mode_setting_val |= PWTIME_100_MS & PS_WTIME_MASK;
	else if (strcmp(wait_time, "200") == 0)
		op_mode_setting_val |= PWTIME_200_MS & PS_WTIME_MASK;
	else if (strcmp(wait_time, "400") == 0)
		op_mode_setting_val |= PWTIME_400_MS & PS_WTIME_MASK;
	else if (strcmp(wait_time, "800") == 0)
		op_mode_setting_val |= PWTIME_800_MS & PS_WTIME_MASK;
	else if (strcmp(wait_time, "1600") == 0)
		op_mode_setting_val |= PWTIME_1600_MS & PS_WTIME_MASK;
	else {
		dev_info(&client->dev, "Using default ps wtime value\n");
		op_mode_setting_val |= PWTIME_12_POINT_5_MS & PS_WTIME_MASK;
	}

	ret = pm16d17_write_reg(data, PM16D17_PS_SETTING, op_mode_setting_val);
	if (ret < 0)
		return ret;

	if (of_property_read_u32(np, "ps-ir-led-pulse-count", &pulse_count) < 0) {
		dev_err(&client->dev, "Missing ps-ir-led-pulse-count property\n");
		return -EINVAL;
	}
	if (pulse_count > 256)
		pulse_count = 256;
	ret = pm16d17_write_reg(data, PM16D17_VCSEL_DRIVE_PULSE, pulse_count - 1);
	if (ret < 0)
		return ret;

	if (of_property_read_u32(np, "ps-offset-cancel", &ps_offset_cancel) > 0 &&
	    ps_offset_cancel != 0) {
		ret = pm16d17_write_reg(data, PM16D17_PS_SETTING2, OFFSET_CANCEL_ENABLE);
		if (ret < 0)
			return ret;

		offset_lsb = ps_offset_cancel & PS_OFFSET_CANCEL_LSB_MASK;
		offset_msb = (ps_offset_cancel & PS_OFFSET_CANCEL_MSB_MASK) >> 8;

		ret = pm16d17_write_reg(data, PM16D17_PS_OFFSET_CANCEL_L, offset_lsb);
		if (ret < 0)
			return ret;

		ret = pm16d17_write_reg(data, PM16D17_PS_OFFSET_CANCEL_H, offset_msb);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int pm16d17_probe(struct i2c_client *client)
{
	struct pm16d17_data *data;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	indio_dev->dev.parent = &client->dev;
	indio_dev->info = &pm16d17_info;
	indio_dev->name = PM16D17_DRV_NAME;
	indio_dev->channels = pm16d17_channels;
	indio_dev->num_channels = ARRAY_SIZE(pm16d17_channels);
	indio_dev->modes = INDIO_DIRECT_MODE;

	data = iio_priv(indio_dev);
	data->client = client;

	data->regmap = devm_regmap_init_i2c(client, &pm16d17_regmap_config);
	if (IS_ERR(data->regmap)) {
		dev_err(&client->dev, "regmap initialization failed.\n");
		return PTR_ERR(data->regmap);
	}

	ret = pm16d17_chip_init(data);
	if (ret)
		return ret;

	return devm_iio_device_register(&client->dev, indio_dev);
}

static const struct i2c_device_id pm16d17_id[] = {
	{"pm16d17", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, pm16d17_id);

static const struct of_device_id pm16d17_of_match[] = {
	{ .compatible = "everlight,pm16d17" },
	{}
};
MODULE_DEVICE_TABLE(of, pm16d17_of_match);

static struct i2c_driver pm16d17_driver = {
	.driver = {
		.name = PM16D17_DRV_NAME,
		.of_match_table = pm16d17_of_match,
	},
	.probe = pm16d17_probe,
	.id_table = pm16d17_id,
};
module_i2c_driver(pm16d17_driver);

MODULE_AUTHOR("Chao Zeng <chao.zeng@siemens.com>");
MODULE_DESCRIPTION("PM16D17 proximity sensor");
MODULE_LICENSE("GPL");
