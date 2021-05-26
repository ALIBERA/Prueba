
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/usb.h>

#define USBLED_VENDOR_ID	0x04D8	
#define USBLED_PRODUCT_ID	0x003F	

static void led_urb_out_callback(struct urb *urb);
static void led_urb_in_callback(struct urb *urb);

/* table of devices that work with this driver */
static const struct usb_device_id id_table[] = {
	{ USB_DEVICE(USBLED_VENDOR_ID, USBLED_PRODUCT_ID) },
	{ }
};
MODULE_DEVICE_TABLE(usb, id_table);

struct usb_led {
	struct usb_device *udev;
	struct usb_interface *intf;
	struct urb 	  *interrupt_out_urb;
	struct urb 	  *interrupt_in_urb;
	struct usb_endpoint_descriptor *interrupt_out_endpoint;
	struct usb_endpoint_descriptor *interrupt_in_endpoint;
	u8		  irq_data;
	u8		  led_number;
	u8 		  ibuffer;
	int		  interrupt_out_interval;
	int ep_in;
	int ep_out;
};

static ssize_t led_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct usb_led *led = usb_get_intfdata(intf);			\
									\
	return sprintf(buf, "%d\n", led->led_number);
}

static ssize_t led_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	/* interface: related set of endpoints which present a single feature or function to the host */
	struct usb_interface *intf = to_usb_interface(dev);
	struct usb_led *led = usb_get_intfdata(intf);
    	u8 val;
    	int error, retval;
    
        dev_info(&intf->dev, "led_store() function is called.\n");
    
    	/* transform char array to u8 value */
	error = kstrtou8(buf, 10, &val);
	if (error)
		return error;
    
    	led->led_number = val;
	led->irq_data = val;

	if (val == 0)
		dev_info(&led->udev->dev, "read status\n");
	else if (val == 1 || val == 2 || val == 3)
        	dev_info(&led->udev->dev, "led = %d\n", led->led_number);
    	else {
        	dev_info(&led->udev->dev, "unknown value %d\n", val);
        	retval = -EINVAL;
        	return retval;
    	}
	
	/* send the data out */
	retval = usb_submit_urb(led->interrupt_out_urb, GFP_KERNEL);
	if (retval) {
        	dev_err(&led->udev->dev,
			"Couldn't submit interrupt_out_urb %d\n", retval);
		return retval;
	}

	return count;
}
static DEVICE_ATTR_RW(led);

static void led_urb_out_callback(struct urb *urb)
{
	struct usb_led *dev;

	dev = urb->context;

	dev_info(&dev->udev->dev, "led_urb_out_callback() function is called.\n");

	/* sync/async unlink faults aren't errors */
	if (urb->status) {
		if (!(urb->status == -ENOENT ||
		    urb->status == -ECONNRESET ||
		    urb->status == -ESHUTDOWN))
			dev_err(&dev->udev->dev,
				"%s - nonzero write status received: %d\n",
				__func__, urb->status);
	}
}

static void led_urb_in_callback(struct urb *urb)
{
	int retval;
	struct usb_led *dev;

	dev = urb->context;

	dev_info(&dev->udev->dev, "led_urb_in_callback() function is called.\n");

	if (urb->status) {
		if (!(urb->status == -ENOENT ||
		    urb->status == -ECONNRESET ||
		    urb->status == -ESHUTDOWN))
			dev_err(&dev->udev->dev,
				"%s - nonzero write status received: %d\n",
				__func__, urb->status);
	}

	if (dev->ibuffer == 0x00)
		pr_info ("switch is ON.\n");
	else if (dev->ibuffer == 0x01)
		pr_info ("switch is OFF.\n");
	else
		pr_info ("bad value received\n");

	retval = usb_submit_urb(dev->interrupt_in_urb, GFP_KERNEL);
	if (retval) 
        	dev_err(&dev->udev->dev,
			"Couldn't submit interrupt_in_urb %d\n", retval);
}

static int led_probe(struct usb_interface *intf,
		     const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(intf);
	struct usb_host_interface *altsetting = intf->cur_altsetting;
	struct usb_endpoint_descriptor *endpoint;
	struct usb_led *dev = NULL;
	int ep;
	int ep_in, ep_out;
	int retval, size, res;
	retval = 0;

	dev_info(&intf->dev, "led_probe() function is called.\n");

	res = usb_find_last_int_out_endpoint(altsetting, &endpoint);
	if (res) {
		dev_info(&intf->dev, "no endpoint found");
		return res;
	}

	ep = usb_endpoint_num(endpoint); /* value from 0 to 15, it is 1 */
	size = usb_endpoint_maxp(endpoint);

	/* Validate endpoint and size */
	if (size <= 0) {
		dev_info(&intf->dev, "invalid size (%d)", size);
		return -ENODEV;
	}

	dev_info(&intf->dev, "endpoint size is (%d)", size);
	dev_info(&intf->dev, "endpoint number is (%d)", ep);

	ep_in = altsetting->endpoint[0].desc.bEndpointAddress;
	ep_out = altsetting->endpoint[1].desc.bEndpointAddress;

	dev_info(&intf->dev, "endpoint in address is (%d)", ep_in);
	dev_info(&intf->dev, "endpoint out address is (%d)", ep_out);

	dev = kzalloc(sizeof(struct usb_led), GFP_KERNEL);

	if (!dev) 
		return -ENOMEM;

	dev->ep_in = ep_in;
	dev->ep_out = ep_out;

	dev->udev = usb_get_dev(udev);

	dev->intf = intf;

	/* allocate int_out_urb structure */
	dev->interrupt_out_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!dev->interrupt_out_urb)
		goto error_out;

	/* initialize int_out_urb */
	usb_fill_int_urb(dev->interrupt_out_urb, 
			dev->udev, 
			usb_sndintpipe(dev->udev, ep_out), 
			(void *)&dev->irq_data,
			1,
			led_urb_out_callback, dev, 1);

	/* allocate int_in_urb structure */
	dev->interrupt_in_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!dev->interrupt_in_urb)
		goto error_out;

	/* initialize int_in_urb */
	usb_fill_int_urb(dev->interrupt_in_urb, 
			dev->udev, 
			usb_rcvintpipe(dev->udev, ep_in), 
			(void *)&dev->ibuffer,
			1,
			led_urb_in_callback, dev, 1);

	usb_set_intfdata(intf, dev);
    
    	retval = device_create_file(&intf->dev, &dev_attr_led);
	if (retval)
		goto error_create_file;

	retval = usb_submit_urb(dev->interrupt_in_urb, GFP_KERNEL);
	if (retval) {
        	dev_err(&dev->udev->dev,
			"Couldn't submit interrupt_in_urb %d\n", retval);
		device_remove_file(&intf->dev, &dev_attr_led);
		goto error_create_file;
	}
	
	dev_info(&dev->udev->dev,"int_in_urb submitted\n");

	return 0;

error_create_file:
	usb_free_urb(dev->interrupt_out_urb);
	usb_free_urb(dev->interrupt_in_urb);
	usb_put_dev(udev);
	usb_set_intfdata(intf, NULL);

error_out:
	kfree(dev);
	return retval;
}

static void led_disconnect(struct usb_interface *interface)
{
	struct usb_led *dev;

	dev = usb_get_intfdata(interface);

	device_remove_file(&interface->dev, &dev_attr_led);
	usb_free_urb(dev->interrupt_out_urb);
	usb_free_urb(dev->interrupt_in_urb);
	usb_set_intfdata(interface, NULL);
	usb_put_dev(dev->udev);
	kfree(dev);

	dev_info(&interface->dev, "USB LED now disconnected\n");
}

static struct usb_driver led_driver = {
	.name =		"usbled",
	.probe =	led_probe,
	.disconnect =	led_disconnect,
	.id_table =	id_table,
};

module_usb_driver(led_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Alberto Liberal <aliberal@arroweurope.com>");
MODULE_DESCRIPTION("This is a led/switch usb controlled module with irq in/out endpoints");
