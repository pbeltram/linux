// SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause)
/*
 * Linear Technology LTC2308 SPI ADC driver
 *
 * Copyright 2020 Analog Devices Inc.
 */
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/spi/spi.h>

struct ad3552r_dev {
	struct spi_device	*spi;
	int			mask;
};

//There are spi transfers: see spi.h

enum {
	AD3552R_VAL1,
	AD3552R_VAL2,
	TEST_STREAM,
};

u16 ramp[20010];

void test_stream(struct iio_dev *indio_dev, int len)
{
	struct ad3552r_dev *dac;
	dac = iio_priv(indio_dev);

	int i;
	for (i = 1; i < 10000; i++) {
		ramp[i * 2] = i;
		ramp[i * 2 + 1] = 10000 - i;
	}

	u8 buf[1000] = {};
	u8 *buf2;
	u8 val;
	//Set stream value 4 */
	buf[0] = 0xE;
	buf[1] = 0x4;
	spi_write(dac->spi, buf, 2);
	//Set keep value */
	val = spi_w8r8(dac->spi, 0xF | 0x80);
	val |= 1 << 2;
	buf[0] = 0xF;
	buf[1] = val;
	spi_write(dac->spi, buf, 2);
	//Write data
	buf2 = (u8 *)ramp;
	buf2 += 3;
	buf2[0] = 0x2C;
	spi_write(dac->spi, buf2, 20001);
}

static ssize_t ad3552r_write_ext(struct iio_dev *indio_dev,
				 uintptr_t private,
				 const struct iio_chan_spec *chan,
				 const char *buf, size_t len)
{
	unsigned long long readin;
	int ret;

	ret = kstrtoull(buf, 10, &readin);
	if (ret)
		return ret;

	switch ((u32)private) {
	case AD3552R_VAL1:
		//dev_err(&indio_dev->dev, "Write AD3552R_VAL1");
		pr_err("Write AD3552R_VAL1");
		break;
	case AD3552R_VAL2:
		pr_err("Write AD3552R_VAL2");
		//dev_err(&indio_dev->dev, "Write AD3552R_VAL2");
		break;
	case TEST_STREAM:
		test_stream(indio_dev, readin);
		break;
	default:
		return -EINVAL;
	}

	return len;
}

static ssize_t ad3552r_read_ext(struct iio_dev *indio_dev,
			       uintptr_t private,
			       const struct iio_chan_spec *chan,
			       char *buf)
{
	ssize_t ret;

	switch ((u32)private) {
	case AD3552R_VAL1:
		pr_err("Read AD3552R_VAL1");
		//dev_err(&indio_dev->dev, "Read AD3552R_VAL1");
		break;
	case AD3552R_VAL2:
		pr_err("Read AD3552R_VAL2");
		//dev_err(&indio_dev->dev, "Read AD3552R_VAL2");
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

#define _AD3552R_CHAN_EXT_INFO(_name, _what, _shared) { \
	.name = _name, \
	.read = ad3552r_read_ext, \
	.write = ad3552r_write_ext, \
	.private = _what, \
	.shared = _shared, \
}

static const struct iio_chan_spec_ext_info ad3552r_ext_info[] = {
	_AD3552R_CHAN_EXT_INFO("val1", AD3552R_VAL1, IIO_SEPARATE),
	_AD3552R_CHAN_EXT_INFO("val2", AD3552R_VAL2, IIO_SHARED_BY_ALL),
	_AD3552R_CHAN_EXT_INFO("test_stream", TEST_STREAM, IIO_SHARED_BY_ALL),
	{},
};
 //CH_IFNO_ENBLE ?
#define _AD3552R_VOLTAGE_CHAN(index) { \
	.type = IIO_VOLTAGE,	\
	.indexed = 1, \
	.channel = index, \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | \
				BIT(IIO_CHAN_INFO_PROCESSED), \
				BIT(IIO_CHAN_INFO_ENABLE), \
	.ext_info = ad3552r_ext_info, \
	.output = true,\
	.scan_index = index,		\
	.scan_type = {			\
		.sign = 's',		\
		.realbits = 16,		\
		.storagebits = 16,	\
		.endianness = IIO_LE,	\
	}, 				\ 
}

static const struct iio_chan_spec ad3552r_channels[] = {
	_AD3552R_VOLTAGE_CHAN(0),
	_AD3552R_VOLTAGE_CHAN(1),
};

static int iio_dummy_read_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int *val,
			      int *val2,
			      long mask)
{
	dev_err(&indio_dev->dev, "Read");
	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		*val = 1;
		break;
	case IIO_CHAN_INFO_PROCESSED:
		*val = 2;
		break;
	case IIO_CHAN_INFO_ENABLE:
		break;
	default:
		return -EINVAL;
	}
	return IIO_VAL_INT;
}

static int iio_dummy_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val,
			       int val2,
			       long mask)
{
	struct ad3552r_dev *dac;
	u8 buf[3];

	dac = iio_priv(indio_dev);

	dev_err(&indio_dev->dev, "Write");
	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (chan->channel == 0)
			buf[0] = 0x29 + 1;
		else
			buf[0] = 0x2B + 1;
		buf[1] = val & 0xFF;
		buf[2] = (val >> 8) & 0xFF;
		spi_write(dac->spi, buf, 3);
	case IIO_CHAN_INFO_PROCESSED:
	case IIO_CHAN_INFO_ENABLE:
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int ad3552r_reg_access(struct iio_dev *indio_dev, unsigned int reg,
			     unsigned int writeval, unsigned int *readval)
{
	uint8_t buf[] = {reg, writeval};
	struct ad3552r_dev *dev; 

	dev = iio_priv(indio_dev);
	if (readval) {
		dev_err(&indio_dev->dev, "Read reg %d", reg);
		*readval = (unsigned int)(spi_w8r8(dev->spi, reg | 0x80) & 0xFF);
	}
	else {
		dev_err(&indio_dev->dev, "Write reg %d", reg);
		spi_write(dev->spi, buf, 2);
	}

	return 0;
}

/*
static const unsigned long adxrs290_avail_scan_masks[] = {
	BIT(ADXRS290_IDX_X) | BIT(ADXRS290_IDX_Y) | BIT(ADXRS290_IDX_TEMP),
	0
};
*/



/*
 * Device type specific information.
 */
static const struct iio_info iio_dummy_info = {
	.read_raw = &iio_dummy_read_raw,
	.write_raw = &iio_dummy_write_raw,
	.debugfs_reg_access = ad3552r_reg_access
};

irqreturn_t ad3552r_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf;
	struct iio_dev *indio_dev;
	struct iio_buffer *buffer;
	struct ad3552r_dev *dac;

	pf = p;
	indio_dev = pf->indio_dev;
	buffer = indio_dev->buffer;
	dac = iio_priv(indio_dev);
}

static int ad3552r_probe(struct spi_device *spi)
{
	struct ad3552r_dev	*dac;
	struct iio_dev		*indio_dev;
	int			err;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*dac));
	if (!indio_dev)
		return -ENOMEM;

	dac = iio_priv(indio_dev);
	dac->spi = spi;

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = "ad3552r";
	indio_dev->channels = ad3552r_channels;
	indio_dev->num_channels = ARRAY_SIZE(ad3552r_channels);
	indio_dev->available_scan_masks = 0b11;
	indio_dev->info = &iio_dummy_info;
	indio_dev->modes = INDIO_BUFFER_HARDWARE; //??
	pr_err("This is my driver");
	dev_err(&spi->dev, "This is my driver");

	//memset(&dac->ops, 0, sizoef(dac->ops))
	
	err = devm_iio_dmaengine_buffer_alloc(dac, indio_dev, NULL,
					      ad3552r_trigger_handler,
					      NULL);
	if (err)
		err_flag;
	
	return devm_iio_device_register(&spi->dev, indio_dev);

err_flag:
	return err;
}

static const struct of_device_id ad3552r_of_match[] = {
	{ .compatible = "adi,ad3552r" },
	{ }
};
MODULE_DEVICE_TABLE(of, ad3552r_of_match);

static struct spi_driver ad3552r_driver = {
	.driver = {
		.name = "ad3552r",
		.of_match_table = ad3552r_of_match,
	},
	.probe = ad3552r_probe,
};
module_spi_driver(ad3552r_driver);

MODULE_AUTHOR("Mihail Chindris <mihail.chindris@analog.com>");
MODULE_DESCRIPTION("Analog Device AD3552r ADC");
MODULE_LICENSE("Dual BSD/GPL");