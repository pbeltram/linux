// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for AD9083 and similar mixed signal front end (MxFE®)
 *
 * Copyright 2021 Analog Devices Inc.
 */
//#define DEBUG
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <asm/unaligned.h>

static int ad9083_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct ad9083_state *st;
	int ret;

	printk("==============================================ad9083 probed===============================================");

	return 0;
}


static const struct spi_device_id ad9083_id[] = {
	{ "ad9083", 0 },
	{}
};
MODULE_DEVICE_TABLE(spi, ad9083_id);

static const struct of_device_id ad9083_of_match[] = {
	{ .compatible = "adi,ad9083" },
	{},
};
MODULE_DEVICE_TABLE(of, ad9083_of_match);

static struct spi_driver ad9083_driver = {
	.driver = {
			.name = "ad9083",
			.of_match_table = of_match_ptr(ad9083_of_match),
		},
	.probe = ad9083_probe,
	.id_table = ad9083_id,
};
module_spi_driver(ad9083_driver);

MODULE_AUTHOR("Cristian Pop <cristian.pop@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD9083 ADC");
MODULE_LICENSE("GPL v2");
