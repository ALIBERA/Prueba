

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/iio/consumer.h>
#include <linux/iio/types.h>
#include <linux/input-polldev.h>
#include <linux/platform_device.h>

/* create private structure */
struct nunchuk_dev {
	struct input_polled_dev *polled_input;
	/* declare pointers to the IIO channels of the provider device */
	struct iio_channel *accel_x, *accel_y, *accel_z;
};

/* 
 * poll handler function read the harware, 
 * queue events to be reported (input_report_*) 
 * and flush the queued events (input_sync) 
 */
static void nunchuk_poll(struct input_polled_dev *polled_input)
{
	int accel_x, accel_y, accel_z;
	struct nunchuk_dev *nunchuk;
	int ret;
	
	nunchuk = polled_input->private;

	/* Read IIO "accel_x" channel raw value from the provider device */
	ret = iio_read_channel_raw(nunchuk->accel_x, &accel_x);
	if (unlikely(ret < 0))
		return;

	/* Report ABS_RX event to the input system */
	input_report_abs(polled_input->input, ABS_RX, accel_x);

	/* Read IIO "accel_y" channel raw value from the provider device */
	ret = iio_read_channel_raw(nunchuk->accel_y, &accel_y);
	if (unlikely(ret < 0))
		return;
	
	/* Report ABS_RY event to the input system */
	input_report_abs(polled_input ->input, ABS_RY, accel_y);

	/* Read IIO "accel_z" channel raw value from the provider device */
	ret = iio_read_channel_raw(nunchuk->accel_x, &accel_z);
	if (unlikely(ret < 0))
		return;

	/* Report ABS_RZ event to the input system */
	input_report_abs(polled_input->input, ABS_RZ, accel_z);
	
	/* 
	 * Tell those who receive the events 
	 * that a complete report has been sent
	 */
	input_sync(polled_input->input);
}

static int nunchuk_probe(struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;
	enum iio_chan_type type;
	
	/* declare a pointer to the private structure */
	struct nunchuk_dev *nunchuk;

	/* declare pointers to input_dev and input_polled_dev structures */
	struct input_polled_dev *polled_device;
	struct input_dev *input;
	
	dev_info(dev, "nunchuck_probe() function is called.\n");

	/* allocate private structure for nunchuk device */
	nunchuk = devm_kzalloc(dev, sizeof(*nunchuk), GFP_KERNEL);
	if (nunchuk == NULL)
		return -ENOMEM;

	/* Get pointer to channel "accel_x" of the provider device */
	nunchuk->accel_x = devm_iio_channel_get(dev, "accel_x");
	if (IS_ERR(nunchuk->accel_x))
		return PTR_ERR(nunchuk->accel_x);

	if (!nunchuk->accel_x->indio_dev)
		return -ENXIO;

	/* Get type of "accel_x" channel */
	ret = iio_get_channel_type(nunchuk->accel_x, &type);
	if (ret < 0)
		return ret;

	if (type != IIO_ACCEL) {
		dev_err(dev, "not accelerometer channel %d\n", type);
		return -EINVAL;
	}

	/* Get pointer to channel "accel_y" of the provider device */
	nunchuk->accel_y = devm_iio_channel_get(dev, "accel_y");
	if (IS_ERR(nunchuk->accel_y))
		return PTR_ERR(nunchuk->accel_y);

	if (!nunchuk->accel_y->indio_dev)
		return -ENXIO;

	/* Get type of "accel_y" channel */
	ret = iio_get_channel_type(nunchuk->accel_y, &type);
	if (ret < 0)
		return ret;

	if (type != IIO_ACCEL) {
		dev_err(dev, "not accel channel %d\n", type);
		return -EINVAL;
	}

	/* Get pointer to channel "accel_z" of the provider device */
	nunchuk->accel_z = devm_iio_channel_get(dev, "accel_z");
	if (IS_ERR(nunchuk->accel_z))
		return PTR_ERR(nunchuk->accel_z);

	if (!nunchuk->accel_z->indio_dev)
		return -ENXIO;

	/* Get type of "accel_z" channel */
	ret = iio_get_channel_type(nunchuk->accel_z, &type);
	if (ret < 0)
		return ret;

	if (type != IIO_ACCEL) {
		dev_err(dev, "not accel channel %d\n", type);
		return -EINVAL;
	}

	/* Allocate the struct input_polled_dev */
	polled_device = devm_input_allocate_polled_device(dev);
	if (!polled_device) {
		dev_err(dev, "unable to allocate input device\n");		
		return -ENOMEM;
	}

	/* Initialize the polled input device */
	/* To recover nunchuk in the poll() function */
	polled_device->private = nunchuk;

	/* Fill in the poll interval */
	polled_device->poll_interval = 50;

	/* Fill in the poll handler */
	polled_device->poll = nunchuk_poll;

	polled_device->input->name = "WII accel consumer";
	polled_device->input->id.bustype = BUS_HOST;

	/* 
	 * Store the polled device in the global structure 
         * to recover it in the remove() function 
         */
	nunchuk->polled_input = polled_device;

	input = polled_device->input;

	/* To recover nunchuck structure from remove() function */
	platform_set_drvdata(pdev, nunchuk);

	/* 
         * Set EV_ABS type events and from those 
	 * ABS_X, ABS_Y, ABS_RX, ABS_RY and ABS_RZ event codes 
         */
	set_bit(EV_ABS, input->evbit);
	set_bit(ABS_RX, input->absbit); /* accelerometer */
	set_bit(ABS_RY, input->absbit);
	set_bit(ABS_RZ, input->absbit);

	/* 
	 * fill additional fields in the input_dev struct for 
         * each absolute axis nunchuk has
         */
	input_set_abs_params(input, ABS_RX, 0x00, 0x3ff, 0, 0);
	input_set_abs_params(input, ABS_RY, 0x00, 0x3ff, 0, 0);
	input_set_abs_params(input, ABS_RZ, 0x00, 0x3ff, 0, 0);

	/* Finally, register the input device */
	ret = input_register_polled_device(nunchuk->polled_input);
	if (ret < 0) 
		return ret;

	return 0;
}

static int nunchuk_remove(struct platform_device *pdev)
{
	struct nunchuk_dev *nunchuk = platform_get_drvdata(pdev);
	input_unregister_polled_device(nunchuk->polled_input);
	dev_info(&pdev->dev, "nunchuk_remove()\n");

	return 0;
}

/* Add entries to device tree */
/* Declare a list of devices supported by the driver */
static const struct of_device_id nunchuk_of_ids[] = {
	{ .compatible = "nunchuk_consumer"},
	{},
};
MODULE_DEVICE_TABLE(of, nunchuk_of_ids);

/* Define platform driver structure */
static struct platform_driver nunchuk_platform_driver = {
	.probe = nunchuk_probe,
	.remove = nunchuk_remove,
	.driver = {
		.name = "nunchuk_consumer",
		.of_match_table = nunchuk_of_ids,
		.owner = THIS_MODULE,
	}
};

/* Register our platform driver */
module_platform_driver(nunchuk_platform_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Alberto Liberal <aliberal@arroweurope.com>");
MODULE_DESCRIPTION("This is a Nunchuk consumer platform driver");


