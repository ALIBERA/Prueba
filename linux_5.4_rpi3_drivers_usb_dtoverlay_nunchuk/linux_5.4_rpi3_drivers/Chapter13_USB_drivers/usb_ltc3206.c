

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/i2c.h>

/* i2cset -y 4 0x1b 0x00 0xf0 0x00 i -> this is a full I2C block write blue and toggle the leds
i2cset -y 4 0x1b 0xf0 0x00 0x00 i -> red full 
i2cset -y 4 0x1b 0x10 0x00 0x00 i -> red low
i2cset -y 4 0x1b 0x00 0x0f 0x00 i -> green full
i2cset -y 4 0x1b 0x00 0x0f 0x0f i -> sub and green full
i2cset -y 4 0x1b 0x00 0x00 0xf0 i -> main full */

#define DRIVER_NAME	"usb-ltc3206"

#define USB_VENDOR_ID_LTC3206		0x04d8
#define USB_DEVICE_ID_LTC3206		0x003f

#define LTC3206_OUTBUF_LEN		3	/* USB write packet length */
#define LTC3206_I2C_DATA_LEN		3

/* Structure to hold all of our device specific stuff */
struct i2c_ltc3206 {
	u8 obuffer[LTC3206_OUTBUF_LEN];	/* USB write buffer */
	/* I2C/SMBus data buffer */
	u8 user_data_buffer[LTC3206_I2C_DATA_LEN];
	int ep_out;              	/* out endpoint */
	struct usb_device *usb_dev;	/* the usb device for this device */
	struct usb_interface *interface;/* the interface for this device */
	struct i2c_adapter adapter;	/* i2c related things */
	/* wq to wait for an ongoing write */
	wait_queue_head_t usb_urb_completion_wait;
	bool ongoing_usb_ll_op;		/* all is in progress */
	struct urb *interrupt_out_urb;
};

/*
 * Return list of I2C supported functionality
 */
static u32 ltc3206_usb_func(struct i2c_adapter *a)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL |
	       I2C_FUNC_SMBUS_READ_BLOCK_DATA | I2C_FUNC_SMBUS_BLOCK_PROC_CALL;
}

/* usb out urb callback function */
static void ltc3206_usb_cmpl_cbk(struct urb *urb)
{
	struct i2c_ltc3206 *dev = urb->context;
	int status = urb->status;
	int retval;

	switch (status) {
	case 0:			/* success */
		break;
	case -ECONNRESET:	/* unlink */
	case -ENOENT:
	case -ESHUTDOWN:
		return;
	/* -EPIPE:  should clear the halt */
	default:		/* error */
		goto resubmit;
	}

	/* 
	 * wake up the waiting function
	 * modify the flag indicating the ll status 
	 */
	dev->ongoing_usb_ll_op = 0; /* communication is OK */
	wake_up_interruptible(&dev->usb_urb_completion_wait);
	return;

resubmit:
	retval = usb_submit_urb(urb, GFP_ATOMIC);
	if (retval) {
		dev_err(&dev->interface->dev,
			"ltc3206(irq): can't resubmit intrerrupt urb, retval %d\n",
			retval);
	}
}

static int ltc3206_ll_cmd(struct i2c_ltc3206 *dev)
{
	int rv;

	/* 
	 * tell everybody to leave the URB alone
	 * we are going to write to the LTC3206
	 */
	dev->ongoing_usb_ll_op = 1; /* doing USB communication */

	/* submit the interrupt out ep packet */
	if (usb_submit_urb(dev->interrupt_out_urb, GFP_KERNEL)) {
		dev_err(&dev->interface->dev,
				"ltc3206(ll): usb_submit_urb intr out failed\n");
		dev->ongoing_usb_ll_op = 0;
		return -EIO;
	}

	/* wait for its completion, the USB URB callback will signal it */
	rv = wait_event_interruptible(dev->usb_urb_completion_wait,
			(!dev->ongoing_usb_ll_op));
	if (rv < 0) {
		dev_err(&dev->interface->dev, "ltc3206(ll): wait interrupted\n");
		goto ll_exit_clear_flag;
	}

	return 0;

ll_exit_clear_flag:
	dev->ongoing_usb_ll_op = 0;
	return rv;
}

static int ltc3206_init(struct i2c_ltc3206 *dev)
{
	int ret;

	/* initialize the LTC3206 */
	dev_info(&dev->interface->dev,
		 "LTC3206 at USB bus %03d address %03d -- ltc3206_init()\n",
		 dev->usb_dev->bus->busnum, dev->usb_dev->devnum);

	dev->interrupt_out_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!dev->interrupt_out_urb){
		ret = -ENODEV;
		goto init_error;
	}

	usb_fill_int_urb(dev->interrupt_out_urb, dev->usb_dev,
				usb_sndintpipe(dev->usb_dev,
						  dev->ep_out),
				(void *)&dev->obuffer, LTC3206_OUTBUF_LEN, 
				ltc3206_usb_cmpl_cbk, dev,
				1);

	ret = 0;
	goto init_no_error;

init_error:
	dev_err(&dev->interface->dev, "ltc3206_init: Error = %d\n", ret);
	return ret;

init_no_error:
	dev_info(&dev->interface->dev, "ltc3206_init: Success\n");
	return ret;
}

static int ltc3206_i2c_write(struct i2c_ltc3206 *dev,
					struct i2c_msg *pmsg)
{
	u8 ucXferLen;
	int rv;
	u8 *pSrc, *pDst;
	
	if (pmsg->len > LTC3206_I2C_DATA_LEN)
	{
		pr_info ("problem with the lenght\n");
		return -EINVAL;
	}

	/* I2C write lenght */
	ucXferLen = (u8)pmsg->len;

	pSrc = &pmsg->buf[0];
	pDst = &dev->obuffer[0];
	memcpy(pDst, pSrc, ucXferLen);

	pr_info("oubuffer[0] = %d\n", dev->obuffer[0]);
	pr_info("oubuffer[1] = %d\n", dev->obuffer[1]);
	pr_info("oubuffer[2] = %d\n", dev->obuffer[2]);
		
	rv = ltc3206_ll_cmd(dev);
	if (rv < 0)
		return -EFAULT;

	return 0;
}

/* device layer */
static int ltc3206_usb_i2c_xfer(struct i2c_adapter *adap,
		struct i2c_msg *msgs, int num)
{
	struct i2c_ltc3206 *dev = i2c_get_adapdata(adap);
	struct i2c_msg *pmsg;
	int ret, count;

	pr_info("number of i2c msgs is = %d\n", num);

	for (count = 0; count < num; count++) {
		pmsg = &msgs[count];
		ret = ltc3206_i2c_write(dev, pmsg);
		if (ret < 0)
			goto abort;
	}

	/* if all the messages were transferred ok, return "num" */
	ret = num;
abort:
	return ret;
}

static const struct i2c_algorithm ltc3206_usb_algorithm = {
	.master_xfer = ltc3206_usb_i2c_xfer,
	.functionality = ltc3206_usb_func,
};

static const struct usb_device_id ltc3206_table[] = {
	{ USB_DEVICE(USB_VENDOR_ID_LTC3206, USB_DEVICE_ID_LTC3206) },
	{ }
};
MODULE_DEVICE_TABLE(usb, ltc3206_table);

static void ltc3206_free(struct i2c_ltc3206 *dev)
{
	usb_put_dev(dev->usb_dev);
	usb_set_intfdata(dev->interface, NULL);
	kfree(dev);
}

static int ltc3206_probe(struct usb_interface *interface,
			    const struct usb_device_id *id)
{
	struct usb_host_interface *hostif = interface->cur_altsetting;
	struct i2c_ltc3206 *dev;
	int ret;

	dev_info(&interface->dev, "ltc3206_probe() function is called.\n");

	/* allocate memory for our device state and initialize it */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (dev == NULL) {
		pr_info("i2c-ltc3206(probe): no memory for device state\n");
		ret = -ENOMEM;
		goto error;
	}

	/* get ep_out */
	dev->ep_out = hostif->endpoint[1].desc.bEndpointAddress;

	dev->usb_dev = usb_get_dev(interface_to_usbdev(interface));
	dev->interface = interface;

	init_waitqueue_head(&dev->usb_urb_completion_wait);

	/* save our data pointer in this interface device */
	usb_set_intfdata(interface, dev);

	/* setup i2c adapter description */
	dev->adapter.owner = THIS_MODULE;
	dev->adapter.class = I2C_CLASS_HWMON;
	dev->adapter.algo = &ltc3206_usb_algorithm;
	i2c_set_adapdata(&dev->adapter, dev);

	snprintf(dev->adapter.name, sizeof(dev->adapter.name),
		 DRIVER_NAME " at bus %03d device %03d",
		 dev->usb_dev->bus->busnum, dev->usb_dev->devnum);

	dev->adapter.dev.parent = &dev->interface->dev;

	/* initialize ltc3206 i2c device */
	ret = ltc3206_init(dev);
	if (ret < 0) {  
		dev_err(&interface->dev, "failed to initialize adapter\n");
		goto error_init;
	}

	/* and finally attach to i2c layer */
	ret = i2c_add_adapter(&dev->adapter);
	if (ret < 0) {
		dev_info(&interface->dev, "failed to add I2C adapter\n");
		goto error_i2c;
	}

	dev_info(&dev->interface->dev,
			"ltc3206_probe() -> chip connected -> Success\n");
	return 0;

error_init:
	usb_free_urb(dev->interrupt_out_urb);

error_i2c:
	usb_set_intfdata(interface, NULL);
	ltc3206_free(dev);
error:
	return ret;
}

static void ltc3206_disconnect(struct usb_interface *interface)
{
	struct i2c_ltc3206 *dev = usb_get_intfdata(interface);

	i2c_del_adapter(&dev->adapter);

	usb_kill_urb(dev->interrupt_out_urb);
	usb_free_urb(dev->interrupt_out_urb);

	usb_set_intfdata(interface, NULL);
	ltc3206_free(dev);

	pr_info("i2c-ltc3206(disconnect) -> chip disconnected");
}

static struct usb_driver ltc3206_driver = {
	.name = DRIVER_NAME,
	.probe = ltc3206_probe,
	.disconnect = ltc3206_disconnect,
	.id_table = ltc3206_table,
};

module_usb_driver(ltc3206_driver);

MODULE_AUTHOR("Alberto Liberal <aliberal@arroweurope.com>");
MODULE_DESCRIPTION("This is a usb controlled i2c ltc3206 device");
MODULE_LICENSE("GPL");

