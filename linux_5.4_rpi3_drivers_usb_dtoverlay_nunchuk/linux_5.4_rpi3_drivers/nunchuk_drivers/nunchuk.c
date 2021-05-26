
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/input-polldev.h>

/* create private structure */
struct nunchuk_dev {
	struct input_polled_dev *polled_input;
	struct i2c_client *client;
};

static int nunchuk_read_registers(struct i2c_client *client, u8 *buf, int buf_size) {
	int status;

	mdelay(10);

	buf[0] = 0x00;
	status = i2c_master_send(client, buf, 1);
	if (status >= 0 && status != 1)
		return -EIO;
	if (status < 0)
		return status;

	mdelay(10);

	status = i2c_master_recv(client, buf, buf_size);
	if (status >= 0 && status != buf_size)
		return -EIO;
	if (status < 0)
		return status;

	return 0;
}

/* 
 * poll handler function read the harware, 
 * queue events to be reported (input_report_*) 
 * and flush the queued events (input_sync) 
 */
static void nunchuk_poll(struct input_polled_dev *polled_input)
{
	u8 buf[6];
	int joy_x, joy_y, z_button, c_button, accel_x, accel_y, accel_z;
	struct i2c_client *client;
	struct nunchuk_dev *nunchuk;
	
	/* 
	 * Recover the global nunchuk structure and from it the client address 
         * to stablish an I2C transaction with the nunchuck device
	 */
	nunchuk = polled_input->private;
	client = nunchuk->client;

	/* Read the registers of the nunchuk device */
	if (nunchuk_read_registers(client, buf, ARRAY_SIZE(buf)) < 0)
	{	
		dev_info(&client->dev, "Error reading the nunchuk registers.\n");
		return;
	}

	joy_x = buf[0];
	joy_y = buf[1];
	
      	/* Bit 0 indicates if Z button is pressed */
      	z_button = (buf[5] & BIT(0)? 0 : 1);                
      	/* Bit 1 indicates if C button is pressed */
      	c_button = (buf[5] & BIT(1)? 0 : 1);


	accel_x = (buf[2] << 2) | ((buf[5] >> 2) & 0x3);
	accel_y = (buf[3] << 2) | ((buf[5] >> 4) & 0x3);
	accel_z = (buf[4] << 2) | ((buf[5] >> 6) & 0x3);


	/* Report events to the input system */
	input_report_abs(polled_input->input, ABS_X, joy_x);
	input_report_abs(polled_input->input, ABS_Y, joy_y);

	input_event(polled_input->input, EV_KEY, BTN_Z, z_button);
	input_event(polled_input->input, EV_KEY, BTN_C, c_button);


	input_report_abs(polled_input->input, ABS_RX, accel_x);

	input_report_abs(polled_input ->input, ABS_RY, accel_y);

	input_report_abs(polled_input->input, ABS_RZ, accel_z);

	/* 
	 * Tell those who receive the events 
	 * that a complete report has been sent
	 */
	input_sync(polled_input->input);
}

static int nunchuk_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	u8 buf[2];
	struct device *dev = &client->dev;

	/* declare a pointer to the private structure */
	struct nunchuk_dev *nunchuk;

	/* declare pointers to input_dev and input_polled_dev structures */
	struct input_dev *input;
	/* For devices that can to be polled on a timer basis */
	struct input_polled_dev *polled_device;
	
	dev_info(&client->dev, "nunchuck_probe() function is called.\n");

	/* allocate private structure for new device */
	nunchuk = devm_kzalloc(&client->dev, sizeof(*nunchuk), GFP_KERNEL);
	if (nunchuk == NULL)
		return -ENOMEM;

	/* Associate client->dev with nunchuk private structure */
	i2c_set_clientdata(client, nunchuk);


	/* Allocate the struct input_polled_dev */
	polled_device = devm_input_allocate_polled_device(&client->dev);
	if (!polled_device) {
		dev_err(dev, "unable to allocate input device\n");		
		return -ENOMEM;
	}

	/* Store the client device in the global structure */
	nunchuk->client = client;

	/* Initialize the polled input device */
	/* To recover nunchuk in the poll() function */
	polled_device->private = nunchuk; 

	/* Fill in the poll interval */
	polled_device->poll_interval = 50;
	
	/* Fill in the poll handler */
	polled_device->poll = nunchuk_poll;

	polled_device->input->dev.parent = &client->dev;

	polled_device->input->name = "WII Nunchuk";
	polled_device->input->id.bustype = BUS_I2C;

	/* 
	 * Store the polled device in the global structure 
         * to recover it in the remove() function 
         */
	nunchuk->polled_input = polled_device;

	input = polled_device->input;

	/* Set EV_KEY type events and from those BTN_C and BTN_Z event codes */
	set_bit(EV_KEY, input->evbit);
	set_bit(BTN_C, input->keybit); /* buttons */
	set_bit(BTN_Z, input->keybit);

	/* 
         * Set EV_ABS type events and from those 
	 * ABS_X, ABS_Y, ABS_RX, ABS_RY and ABS_RZ event codes 
         */
	set_bit(EV_ABS, input->evbit);
	set_bit(ABS_X, input->absbit); /* joystick */
	set_bit(ABS_Y, input->absbit);
	set_bit(ABS_RX, input->absbit); /* accelerometer */
	set_bit(ABS_RY, input->absbit);
	set_bit(ABS_RZ, input->absbit);

	/* 
	 * fill additional fields in the input_dev struct for 
         * each absolute axis nunchuk has
         */
	input_set_abs_params(input, ABS_X, 0x00, 0xff, 0, 0);
	input_set_abs_params(input, ABS_Y, 0x00, 0xff, 0, 0);

	input_set_abs_params(input, ABS_RX, 0x00, 0x3ff, 0, 0);
	input_set_abs_params(input, ABS_RY, 0x00, 0x3ff, 0, 0);
	input_set_abs_params(input, ABS_RZ, 0x00, 0x3ff, 0, 0);
	
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

	/* Finally, register the input device */
	ret = input_register_polled_device(nunchuk->polled_input);
	if (ret < 0) 
		return ret;

	return 0;
}

static int nunchuk_remove(struct i2c_client *client)
{
	struct nunchuk_dev *nunchuk;
	nunchuk = i2c_get_clientdata(client);
	input_unregister_polled_device(nunchuk->polled_input);
	dev_info(&client->dev, "nunchuk_remove()\n");

	return 0;
}

/* Add entries to device tree */
static const struct of_device_id nunchuk_of_match[] = {
	{ .compatible = "nunchuk"},
	{}
};
MODULE_DEVICE_TABLE(of, nunchuk_of_match);


static const struct i2c_device_id nunchuk_id[] = {
	{ "nunchuk", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, nunchuk_id);

/* create struct i2c_driver */
static struct i2c_driver nunchuk_driver = {
	.driver = {
		.name = "nunchuk",
		.owner = THIS_MODULE,
		.of_match_table = nunchuk_of_match,
	},
	.probe = nunchuk_probe,
	.remove = nunchuk_remove,
	.id_table = nunchuk_id,
};

/* Register to i2c bus as a driver */
module_i2c_driver(nunchuk_driver);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Alberto Liberal <aliberal@arroweurope.com>");
MODULE_DESCRIPTION("This is a Nunchuk Wii I2C driver");


