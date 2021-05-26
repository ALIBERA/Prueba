#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>

struct nunchuk_accel {
	struct i2c_client *client;
};

#define NUNCHUK_IIO_CHAN(axis) {			\
	.type = IIO_ACCEL,				\
	.modified = 1,					\
	.channel2 = IIO_MOD_##axis,			\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),	\
}

static const struct iio_chan_spec nunchuk_channels[] = {
	NUNCHUK_IIO_CHAN(X),
	NUNCHUK_IIO_CHAN(Y),
	NUNCHUK_IIO_CHAN(Z),
};

static int nunchuk_read_registers(struct i2c_client *client, char *buf, int buf_size) {
	int ret;

	mdelay(10);
	
	buf[0] = 0x00;
	ret = i2c_master_send(client, buf, 1);
	if (ret >= 0 && ret != 1)
		return -EIO;
	if (ret < 0)
		return ret;

	mdelay(10);

	ret = i2c_master_recv(client, buf, buf_size);
	if (ret >= 0 && ret != buf_size)
		return -EIO;
	if (ret < 0)
		return ret;
	
	return 0;
}

static int nunchuk_accel_read_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				int *val, int *val2, long mask)
{
	char buf[6];
	struct nunchuk_accel *nunchuk_accel = iio_priv(indio_dev);
	struct i2c_client *client = nunchuk_accel->client;

	
	/* read data from nunchuk */
	if (nunchuk_read_registers(client, buf, ARRAY_SIZE(buf)) < 0)
	{	
		dev_info(&client->dev, "Error reading the nunchuk registers.\n");
		return -EIO;
	}

	/* data needs to be written to 'val', 'val2' is ignored */
	switch (chan->channel2) {
	case IIO_MOD_X:
		*val = (buf[2] << 2) | ((buf[5] >> 2) & 0x3);
		break;
	case IIO_MOD_Y:
		*val = (buf[3] << 2) | ((buf[5] >> 4) & 0x3);
		break;
	case IIO_MOD_Z:
		*val = (buf[4] << 2) | ((buf[5] >> 6) & 0x3);
		break;
	default:
		return -EINVAL;
	}

	return IIO_VAL_INT;
}

static const struct iio_info nunchuk_info = {
	.read_raw	= nunchuk_accel_read_raw,
};

static int nunchuk_accel_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	u8 buf[2];
	struct iio_dev *indio_dev;

	/* declare an instance of the private structure */
	struct nunchuk_accel *nunchuk_accel;
	
	dev_info(&client->dev, "nunchuk_accel_probe() function is called.\n");

	/* allocate the iio_dev structure */
	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*nunchuk_accel));
	if (indio_dev == NULL) 
		return -ENOMEM;

	nunchuk_accel = iio_priv(indio_dev);	
	/* Associate client->dev with nunchuk private structure */
	i2c_set_clientdata(client, nunchuk_accel);
	nunchuk_accel->client = client;

	indio_dev->name = "Nunchuk Accel";
	indio_dev->dev.parent = &client->dev;
	indio_dev->info = &nunchuk_info;
	indio_dev->channels = nunchuk_channels;
	indio_dev->num_channels = 3;
	indio_dev->modes = INDIO_DIRECT_MODE;
	
	/* Nunchuk handshake */
	buf[0] = 0xf0;
	buf[1] = 0x55;
	ret = i2c_master_send(client, buf, 2);
	if (ret >= 0 && ret != 2)
		return -EIO;
	if (ret < 0)
		return ret;

	udelay(1);

	buf[0] = 0xfb;
	buf[1] = 0x00;
	ret = i2c_master_send(client, buf, 1);
	if (ret >= 0 && ret != 1)
		return -EIO;
	if (ret < 0)
		return ret;
	
	ret = devm_iio_device_register(&client->dev, indio_dev);
	if (ret)
		return ret;

	dev_info(&client->dev, "nunchuk registered\n");

	return 0;
}

static int nunchuk_accel_remove(struct i2c_client *client)
{
	dev_info(&client->dev, "nunchuk_remove()\n");

	return 0;
}

/* Add entries to device tree */
static const struct of_device_id nunchuk_accel_of_match[] = {
	{ .compatible = "nunchuk_accel"},
	{}
};
MODULE_DEVICE_TABLE(of, nunchuk_accel_of_match);


static const struct i2c_device_id nunchuk_accel_id[] = {
	{ "nunchuk_accel", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, nunchuk_accel_id);

/* create struct i2c_driver */
static struct i2c_driver nunchuk_accel_driver = {
	.driver = {
		.name = "nunchuk_accel",
		.owner = THIS_MODULE,
		.of_match_table = nunchuk_accel_of_match,
	},
	.probe = nunchuk_accel_probe,
	.remove = nunchuk_accel_remove,
	.id_table = nunchuk_accel_id,
};

/* Register to i2c bus as a driver */
module_i2c_driver(nunchuk_accel_driver);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Alberto Liberal <aliberal@arroweurope.com>");
MODULE_DESCRIPTION("This is a nunchuk Accelerometer IIO framework I2C driver");


