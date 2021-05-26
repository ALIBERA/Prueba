

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/property.h>

#include <dt-bindings/iio/maxim,max11300.h>

#include "max11300-base.h"

/* 
 * struct gpio_chip get callback function. 
 * It gets the input value of the GPIO line (0=low, 1=high)
 * accessing to the GPIO_DATA registers 
 */
static int max11300_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct max11300_state *st = gpiochip_get_data(chip);
	int ret = 0;
	u16 read_val;
	u8 reg;
	int val;

	mutex_lock(&st->gpio_lock);

	dev_info(st->dev, "The GPIO input is get\n");

	if (st->gpio_offset_mode[offset] == PORT_MODE_3)
	dev_info(st->dev, "the gpio %d cannot be configured in input mode\n", 
		offset);

	/* for GPIOs from 16 to 19 ports */
	if (st->gpio_offset[offset] > 0x0F) {
		reg = GPI_DATA_19_TO_16_ADDRESS;
		ret = st->ops->reg_read(st, reg, &read_val);
		if (ret)
			goto err_unlock;

		val = (int) (read_val);
		val = val << 16;
		
		if (val & BIT(st->gpio_offset[offset])) 
		
			val = 1;
		else 
			val = 0;

		mutex_unlock(&st->gpio_lock);
		return val;
	}
	else {
		reg = GPI_DATA_15_TO_0_ADDRESS;
		ret = st->ops->reg_read(st, reg, &read_val);
		if (ret)
			goto err_unlock;

		val = (int) read_val;
		
		if(val & BIT(st->gpio_offset[offset])) 
			val = 1;
		else 
			val = 0;

		mutex_unlock(&st->gpio_lock);
		return val;
	}

err_unlock:
	mutex_unlock(&st->gpio_lock);
	return ret;
}

/* 
 * struct gpio_chip set callback function. 
 * It sets the output value of the GPIO line in 
 * GPIO ACTIVE_HIGH mode (0=low, 1=high)
 * writing to the GPO_DATA registers
 */
static void max11300_gpio_set(struct gpio_chip *chip, unsigned int offset, 
			     int value)
{
	struct max11300_state *st = gpiochip_get_data(chip);
	u8 reg;
	unsigned int val = 0;

	mutex_lock(&st->gpio_lock);

	dev_info(st->dev, "The GPIO ouput is set\n");

	if (st->gpio_offset_mode[offset] == PORT_MODE_1)
	dev_info(st->dev, "the gpio %d cannot accept this output\n", offset);
		
	if (value == 1 && (st->gpio_offset[offset] > 0x0F)) {
		dev_info(st->dev, 
			"The GPIO ouput is set high and port_number is %d. Pin is > 0x0F\n", 
			st->gpio_offset[offset]);
		val |= BIT(st->gpio_offset[offset]);
		val = val >> 16;
		reg = GPO_DATA_19_TO_16_ADDRESS;
		st->ops->reg_write(st, reg, val);
	}
	else if (value == 0 && (st->gpio_offset[offset] > 0x0F)) {
		dev_info(st->dev, 
			"The GPIO ouput is set low and port_number is %d. Pin is > 0x0F\n", 
			st->gpio_offset[offset]);
		val &= ~BIT(st->gpio_offset[offset]);
		val = val >> 16;
		reg = GPO_DATA_19_TO_16_ADDRESS;
		st->ops->reg_write(st, reg, val);
	}
	else if (value == 1 && (st->gpio_offset[offset] < 0x0F)) {
		dev_info(st->dev, 
			"The GPIO ouput is set high and port_number is %d. Pin is < 0x0F\n", 
			st->gpio_offset[offset]);
		val |= BIT(st->gpio_offset[offset]);
		reg = GPO_DATA_15_TO_0_ADDRESS;
		st->ops->reg_write(st, reg, val);
	}
	else if (value == 0 && (st->gpio_offset[offset] < 0x0F)) {
		dev_info(st->dev, 
			"The GPIO ouput is set low and port_number is %d. Pin is < 0x0F\n", 
			st->gpio_offset[offset]);
		val &= ~BIT(st->gpio_offset[offset]);
		reg = GPO_DATA_15_TO_0_ADDRESS;
		st->ops->reg_write(st, reg, val);
	}
	else
		dev_info(st->dev, "the gpio %d cannot accept this value\n", offset);

	mutex_unlock(&st->gpio_lock);
}

/* 
 * struct gpio_chip direction_input callback function. 
 * It configures the GPIO port as an input (GPI) 
 * writing to the PORT_CFG register
 */
static int max11300_gpio_direction_input(struct gpio_chip *chip, 
					unsigned int offset)
{
	struct max11300_state *st = gpiochip_get_data(chip);
	int ret;
	u8 reg;
	u16 port_mode, val;

	mutex_lock(&st->gpio_lock);

	dev_info(st->dev, "The GPIO is set as an input\n");

	/* get the port number stored in the GPIO offset */
	if (st->gpio_offset_mode[offset] == PORT_MODE_3)
		dev_info(st->dev, 
			"Error.The gpio %d only can be set in output mode\n", 
			offset);

	/* Set the logic 1 input above 2.5V level*/
	val = 0x0fff;

	/* store the GPIO threshold value in the port DAC register */
	reg = PORT_DAC_DATA_BASE_ADDRESS + st->gpio_offset[offset];
	ret = st->ops->reg_write(st, reg, val);
	if (ret) 
		goto err_unlock;

	/* Configure the port as GPI */
	reg = PORT_CFG_BASE_ADDRESS + st->gpio_offset[offset];
	port_mode = (1 << 12);
	ret = st->ops->reg_write(st, reg, port_mode);
	if (ret)
		goto err_unlock;

	mdelay(1);

err_unlock:
	mutex_unlock(&st->gpio_lock);

	return ret;
}

/* 
 * struct gpio_chip direction_output callback function. 
 * It configures the GPIO port as an output (GPO) writing to
 * the PORT_CFG register and sets output value of the
 * GPIO line in GPIO ACTIVE_HIGH mode (0=low, 1=high)
 * writing to the GPO data registers
 */
static int max11300_gpio_direction_output(struct gpio_chip *chip,
					 unsigned int offset, int value)
{
	struct max11300_state *st = gpiochip_get_data(chip);
	int ret;
	u8 reg;
	u16 port_mode, val;

	mutex_lock(&st->gpio_lock);

	dev_info(st->dev, "The GPIO is set as an output\n");

	if (st->gpio_offset_mode[offset] == PORT_MODE_1)
		dev_info(st->dev, 
			"the gpio %d only can be set in input mode\n", 
			offset);

	/* GPIO output high is 3.3V */
	val = 0x0547;

	reg = PORT_DAC_DATA_BASE_ADDRESS + st->gpio_offset[offset];
	ret = st->ops->reg_write(st, reg, val);
	if (ret) {
		mutex_unlock(&st->gpio_lock);
		return ret;
	}
	mdelay(1);
	reg = PORT_CFG_BASE_ADDRESS + st->gpio_offset[offset];
	port_mode = (3 << 12);
	ret = st->ops->reg_write(st, reg, port_mode);
	if (ret) {
		mutex_unlock(&st->gpio_lock);
		return ret;
	}
	mdelay(1);

	mutex_unlock(&st->gpio_lock);

	max11300_gpio_set(chip, offset, value);
	
	return ret;
}

/* 
 * Initialize the MAX11300 gpio controller (struct gpio_chip) 
 * and register it to the kernel
 */
static int max11300_gpio_init(struct max11300_state *st)
{
	if (!st->num_gpios)
		return 0;

	st->gpiochip.label = "gpio-max11300";
	st->gpiochip.base = -1;
	st->gpiochip.ngpio = st->num_gpios;
	st->gpiochip.parent = st->dev;
	st->gpiochip.can_sleep = true;
	st->gpiochip.direction_input = max11300_gpio_direction_input;
	st->gpiochip.direction_output = max11300_gpio_direction_output;
	st->gpiochip.get = max11300_gpio_get;
	st->gpiochip.set = max11300_gpio_set;
	st->gpiochip.owner = THIS_MODULE;

	mutex_init(&st->gpio_lock);

	/* register a gpio_chip */
	return devm_gpiochip_add_data(st->dev, &st->gpiochip, st);
}

/* 
 * Configure the port configuration registers of each port with the values 
 * retrieved from the DT properties.These DT values were read and stored in 
 * the device global structure using the max11300_alloc_ports() function.
 * The ports in GPIO mode will be configured in the gpiochip.direction_input 
 * and gpiochip.direction_output callback functions.
 */
static int max11300_set_port_modes(struct max11300_state *st)
{
	const struct max11300_rw_ops *ops = st->ops;
	int ret;
	unsigned int i; 
	u8 reg; 
	u16 adc_range, dac_range, adc_reference, adc_samples, adc_negative_port; 
	u16 val, port_mode;
	struct iio_dev *iio_dev = iio_priv_to_dev(st);

	mutex_lock(&iio_dev->mlock);

	for (i = 0; i < st->num_ports; i++) {
		switch (st->port_modes[i]) {
		case PORT_MODE_5: case PORT_MODE_6:
			reg = PORT_CFG_BASE_ADDRESS + i;
			adc_reference = st->adc_reference[i];
			port_mode = (st->port_modes[i] << 12);
			dac_range = (st->dac_range[i] << 8);
	
			dev_info(st->dev, 
				"the value of adc cfg addr for channel %d in port mode %d is %x\n", 
				i, st->port_modes[i], reg);

			if ((st->port_modes[i]) == PORT_MODE_5)
				val = (port_mode | dac_range);
			else
				val = (port_mode | dac_range | adc_reference);

			dev_info(st->dev, "the channel %d is set in port mode %d\n", 
				i, st->port_modes[i]);
			dev_info(st->dev, 
				"the value of adc cfg val for channel %d in port mode %d is %x\n", 
				i, st->port_modes[i], val);

			ret = ops->reg_write(st, reg, val);
			if (ret)
				goto err_unlock;

			mdelay(1);
			break;
		case PORT_MODE_7:
			reg = PORT_CFG_BASE_ADDRESS + i;
			port_mode = (st->port_modes[i] << 12);
			adc_range = (st->adc_range[i] << 8);
			adc_reference = st->adc_reference[i];
			adc_samples = (st->adc_samples[i] << 5);

			dev_info(st->dev, 
				"the value of adc cfg addr for channel %d in port mode %d is %x\n", 
				i, st->port_modes[i], reg);

			val = (port_mode | adc_range | adc_reference | adc_samples);

			dev_info(st->dev, 
				"the channel %d is set in port mode %d\n", 
				i, st->port_modes[i]);
			dev_info(st->dev, 
				"the value of adc cfg val for channel %d in port mode %d is %x\n", 
				i, st->port_modes[i], val);

			ret = ops->reg_write(st, reg, val);
			if (ret)
				goto err_unlock;

			mdelay(1);

			break;
		case PORT_MODE_8:
			reg = PORT_CFG_BASE_ADDRESS + i;
			port_mode = (st->port_modes[i] << 12);
			adc_range = (st->adc_range[i] << 8);
			adc_reference = st->adc_reference[i];
			adc_samples = (st->adc_samples[i] << 5);
			adc_negative_port = st->adc_negative_port[i];

			dev_info(st->dev, 
				"the value of adc cfg addr for channel %d in port mode %d is %x\n", 
				i, st->port_modes[i], reg);

			val = (port_mode | adc_range | adc_reference | adc_samples | adc_negative_port);

			dev_info(st->dev, 
				"the channel %d is set in port mode %d\n", 
				i, st->port_modes[i]);
			dev_info(st->dev, 
				"the value of adc cfg val for channel %d in port mode %d is %x\n", 
				i, st->port_modes[i], val);

			ret = ops->reg_write(st, reg, val);
			if (ret)
				goto err_unlock;

			mdelay(1);
			break;
		case PORT_MODE_9: case PORT_MODE_10:
			reg = PORT_CFG_BASE_ADDRESS + i;
			port_mode = (st->port_modes[i] << 12);
			adc_range = (st->adc_range[i] << 8);
			adc_reference = st->adc_reference[i];

			dev_info(st->dev, 
				"the value of adc cfg addr for channel %d in port mode %d is %x\n", 
				i, st->port_modes[i], reg);

			val = (port_mode | adc_range | adc_reference);

			dev_info(st->dev, 
				"the channel %d is set in port mode %d\n", 
				i, st->port_modes[i]);
			dev_info(st->dev, 
				"the value of adc cfg val for channel %d in port mode %d is %x\n", 
				i, st->port_modes[i], val);

			ret = ops->reg_write(st, reg, val);
			if (ret)
				goto err_unlock;

			mdelay(1);
			break;
		case PORT_MODE_0:
			dev_info(st->dev, "the port %d is set in default port mode_0\n", i);
			break;
		case PORT_MODE_1:
			dev_info(st->dev, "the port %d is set in port mode_1\n", i);
			break;
		case PORT_MODE_3:
			dev_info(st->dev, "the port %d is set in port mode_3\n", i); 
			break;
		default:
			dev_info(st->dev, "bad port mode is selected\n");
			return -EINVAL;
		}
	}

err_unlock:
	mutex_unlock(&iio_dev->mlock);
	return ret;
}

/* IIO writing callback function */
static int max11300_write_dac(struct iio_dev *iio_dev,
	struct iio_chan_spec const *chan, int val, int val2, long mask)
{
	struct max11300_state *st = iio_priv(iio_dev);
	u8 reg;
	int ret;

	reg = (PORT_DAC_DATA_BASE_ADDRESS + chan->channel);

	dev_info(st->dev, "the DAC data register is %x\n", reg);
	dev_info(st->dev, "the value in the DAC data register is %x\n", val);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (!chan->output)
			return -EINVAL;

		mutex_lock(&iio_dev->mlock);
		ret = st->ops->reg_write(st, reg, val);
		mutex_unlock(&iio_dev->mlock);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

/* IIO reading callback function */
static int max11300_read_adc(struct iio_dev *iio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long m)
{
	struct max11300_state *st = iio_priv(iio_dev);
	u16 read_val_se;
	int read_val_dif;
	u8 reg;
	int ret;

	reg = PORT_ADC_DATA_BASE_ADDRESS + chan->channel;

	switch (m) {
	case IIO_CHAN_INFO_RAW: 
		mutex_lock(&iio_dev->mlock);

		if (!chan->output && ((chan->address == PORT_MODE_7) || (chan->address == PORT_MODE_6))) {
			ret = st->ops->reg_read(st, reg, &read_val_se);
			if (ret)
				goto unlock;
			*val = (int) read_val_se;
		} 
		else if (!chan->output && (chan->address == PORT_MODE_8)) {
			ret = st->ops->reg_read_differential(st, reg, &read_val_dif);
			if (ret)
				goto unlock;
			*val = read_val_dif;
		}
		else {
			ret = -EINVAL;
			goto unlock;
		}

		ret = IIO_VAL_INT;
		break;
	default:
		ret = -EINVAL;
	}

unlock:
	mutex_unlock(&iio_dev->mlock);
	return ret;
}

/* Create kernel hooks to read/write IIO sysfs attributes from user space */
static const struct iio_info max11300_info = {
	.read_raw = max11300_read_adc,
	.write_raw = max11300_write_dac,
};

/* DAC with positive voltage range */
static void max11300_setup_port_5_mode(struct iio_dev *iio_dev,
					struct iio_chan_spec *chan, bool output, 
					unsigned int id, unsigned long port_mode)
{
	chan->type = IIO_VOLTAGE;
	chan->indexed = 1;
	chan->address = port_mode;
	chan->output = output;
	chan->channel = id;
	chan->info_mask_separate = BIT(IIO_CHAN_INFO_RAW);
	chan->scan_type.sign = 'u';
	chan->scan_type.realbits = 12;
	chan->scan_type.storagebits = 16;
	chan->scan_type.endianness = IIO_BE;
	chan->extend_name = "mode_5_DAC";
}

/* DAC with positive voltage range */
static void max11300_setup_port_6_mode(struct iio_dev *iio_dev,	
				struct iio_chan_spec *chan, bool output, 
				unsigned int id, unsigned long port_mode)
{
	chan->type = IIO_VOLTAGE;
	chan->indexed = 1;
	chan->address = port_mode;
	chan->output = output;
	chan->channel = id;
	chan->info_mask_separate = BIT(IIO_CHAN_INFO_RAW);
	chan->scan_type.sign = 'u';
	chan->scan_type.realbits = 12;
	chan->scan_type.storagebits = 16;
	chan->scan_type.endianness = IIO_BE;
	chan->extend_name = "mode_6_DAC_ADC";
}

/* ADC in SE mode with positive voltage range and straight binary */
static void max11300_setup_port_7_mode(struct iio_dev *iio_dev,
				struct iio_chan_spec *chan, bool output, 
				unsigned int id, unsigned long port_mode)
{
	chan->type = IIO_VOLTAGE;
	chan->indexed = 1;
	chan->address = port_mode;
	chan->output = output;
	chan->channel = id;
	chan->info_mask_separate = BIT(IIO_CHAN_INFO_RAW);
	chan->scan_type.sign = 'u';
	chan->scan_type.realbits = 12;
	chan->scan_type.storagebits = 16;
	chan->scan_type.endianness = IIO_BE;
	chan->extend_name = "mode_7_ADC";
}

/* ADC in differential mode with 2's complement value */
static void max11300_setup_port_8_mode(struct iio_dev *iio_dev,
				struct iio_chan_spec *chan, bool output, 
				unsigned id, unsigned id2, 
				unsigned int port_mode)
{
	chan->type = IIO_VOLTAGE;
	chan->differential = 1,
	chan->address = port_mode;
	chan->indexed = 1;
	chan->output = output;
	chan->channel = id;
	chan->channel2 = id2;
	chan->info_mask_separate = BIT(IIO_CHAN_INFO_RAW);
	chan->scan_type.sign = 's';
	chan->scan_type.realbits = 12;
	chan->scan_type.storagebits = 16;
	chan->scan_type.endianness = IIO_BE;
	chan->extend_name = "mode_8_ADC";
}

/* 
 * this function will allocate and configure the iio channels of the iio device. 
 * It will also read the DT properties of each port (channel) and will store them
 * in the device global structure
 */
static int max11300_alloc_ports(struct max11300_state *st)
{
	unsigned int i, curr_port = 0, num_ports = st->num_ports, port_mode_6_count = 0, offset = 0;
	st->num_gpios = 0;

	/* recover the iio device from the global structure */
	struct iio_dev *iio_dev = iio_priv_to_dev(st);

	/* pointer to the storage of the specs of all the iio channels */
	struct iio_chan_spec *ports;

	/* pointer to struct fwnode_handle that allows a device description object */
	struct fwnode_handle *child;

	u32 reg, tmp;
	int ret;

	/* 
	 * walks for each MAX11300 child node from the DT, if an error is found in the node 
	 * then walks to the following one (continue) 
	 */
	device_for_each_child_node(st->dev, child) { 
		ret = fwnode_property_read_u32(child, "reg", &reg);
		if (ret || reg >= ARRAY_SIZE(st->port_modes)) 
			continue;

		/* 
		 * store the value of the DT "port,mode" property in the global structure 
                 * to know the mode of each port in other functions of the driver 
		 */
		ret = fwnode_property_read_u32(child, "port-mode", &tmp); 
		if (!ret)
			st->port_modes[reg] = tmp;

		/* all the DT nodes should include the port-mode property */
		else {
			dev_info(st->dev, "port mode is not found\n");
			continue;
		}

		/* 
                 * you will store other DT properties depending 
		 * of the used "port,mode" property 
		 */
		switch (st->port_modes[reg]) {
		case PORT_MODE_7:
			ret = fwnode_property_read_u32(child, "adc-range", &tmp); 
			if (!ret)
				st->adc_range[reg] = tmp;
			else
				dev_info(st->dev, "Get default ADC range\n");

			ret = fwnode_property_read_u32(child, "AVR", &tmp); 
			if (!ret)
				st->adc_reference[reg] = tmp;
			else
				dev_info(st->dev, "Get default internal ADC reference\n");

			ret = fwnode_property_read_u32(child, "adc-samples", &tmp); 
			if (!ret)
				st->adc_samples[reg] = tmp;
			else
				dev_info(st->dev, "Get default internal ADC sampling\n");

			dev_info(st->dev, "the channel %d is set in port mode %d\n", 
				reg, st->port_modes[reg]);
			break;
		case PORT_MODE_8:
			ret = fwnode_property_read_u32(child, "adc-range", &tmp); 
			if (!ret)
				st->adc_range[reg] = tmp;
			else
				dev_info(st->dev, "Get default ADC range\n");

			ret = fwnode_property_read_u32(child, "AVR", &tmp); 
			if (!ret)
				st->adc_reference[reg] = tmp;
			else
				dev_info(st->dev, "Get default internal ADC reference\n");

			ret = fwnode_property_read_u32(child, "adc-samples", &tmp); 
			if (!ret)
				st->adc_samples[reg] = tmp;
			else
				dev_info(st->dev, "Get default internal ADC sampling\n");

			ret = fwnode_property_read_u32(child, "negative-input", &tmp); 
			if (!ret)
				st->adc_negative_port[reg] = tmp;
			else {
				dev_info(st->dev, "Bad value for negative ADC channel\n");
				return -EINVAL;
			}

			dev_info(st->dev, "the channel %d is set in port mode %d\n", 
				reg, st->port_modes[reg]);
			break;
		case PORT_MODE_9: case PORT_MODE_10:	
			ret = fwnode_property_read_u32(child, "adc-range", &tmp); 
			if (!ret)
				st->adc_range[reg] = tmp;
			else
				dev_info(st->dev, "Get default ADC range\n");

			ret = fwnode_property_read_u32(child, "AVR", &tmp); 
			if (!ret)
				st->adc_reference[reg] = tmp;
			else
				dev_info(st->dev, "Get default internal ADC reference\n");
			dev_info(st->dev, "the channel %d is set in port mode %d\n", 
				reg, st->port_modes[reg]);
			break;
		case PORT_MODE_5: case PORT_MODE_6:	
			ret = fwnode_property_read_u32(child, "dac-range", &tmp); 
			if (!ret)
			st->dac_range[reg] = tmp;
			else
				dev_info(st->dev, "Get default DAC range\n");

			/* 
			 * A port in mode 6 will generate two IIO sysfs entries, 
			 * one for writing the DAC port, and another for reading 
			 * the ADC port 
			 */
			if ((st->port_modes[reg]) == PORT_MODE_6) {
				ret = fwnode_property_read_u32(child, "AVR", &tmp); 
				if (!ret)
					st->adc_reference[reg] = tmp;
				else
					dev_info(st->dev, "Get default internal ADC reference\n");
			
				/* 
				 * get the number of ports set in mode_6 to allocate space 
				 * for the related iio channels 
				 */
				port_mode_6_count++;
				dev_info(st->dev, "there are %d channels in mode_6\n", 
					port_mode_6_count);
			}

			dev_info(st->dev, "the channel %d is set in port mode %d\n", 
				reg, st->port_modes[reg]);
			break;
		/* The port is configured as a GPI in the DT */
		case PORT_MODE_1: 
			dev_info(st->dev, "the channel %d is set in port mode %d\n", 
				reg, st->port_modes[reg]);

			/* link the gpio offset with the port number, starting with offset = 0 */
			st->gpio_offset[offset] = reg;

			/* store the port_mode for each gpio offset, starting with offset = 0 */
			st->gpio_offset_mode[offset] = PORT_MODE_1;

			dev_info(st->dev, "the gpio number %d is using the gpio offset number %d\n", 
				st->gpio_offset[offset], offset);

			/* increment the gpio offset and number of configured ports as GPIOs */
			offset++;
			st->num_gpios++;
			break;
		/* The port is configured as a GPO in the DT */
		case PORT_MODE_3: 
			dev_info(st->dev, "the channel %d is set in port mode %d\n", 
				reg, st->port_modes[reg]);
			
			/* linko the gpio offset with the port number, starting with offset = 0 */
			st->gpio_offset[offset] = reg;

			/* store the port_mode for each gpio offset, starting with offset = 0 */
			st->gpio_offset_mode[offset] = PORT_MODE_3;

			dev_info(st->dev, "the gpio number %d is using the gpio offset number %d\n", 
				st->gpio_offset[offset], offset);

			/* increment the gpio offset and number of configured ports as GPIOs */
			offset++;
			st->num_gpios++;
			break;
		case PORT_MODE_0:
			dev_info(st->dev, "the channel %d is set in default port mode_0\n", reg);
			break;
		default:
			dev_info(st->dev, "bad port mode for channel %d\n", reg);
		}
			
	}

	/* 
	 * Allocate space for the storage of all the IIO channels specs. 
         * Returns a pointer to this storage 
	 */
	ports = devm_kcalloc(st->dev, num_ports + port_mode_6_count, 
			    sizeof(*ports), GFP_KERNEL); 
	if (!ports)
		return -ENOMEM;

	/* 
	 * i is the number of the channel, &ports[curr_port] is a pointer variable that 
	 * will store the "iio_chan_spec structure" address of each port 
	 */
	for (i = 0; i < num_ports; i++) {
		switch (st->port_modes[i]) {
		case PORT_MODE_5:
			dev_info(st->dev, "the port %d is configured as MODE 5\n", i);
			max11300_setup_port_5_mode(iio_dev, &ports[curr_port],
					true, i, PORT_MODE_5); // true = out
			curr_port++;
			break;
		case PORT_MODE_6:
			dev_info(st->dev, "the port %d is configured as MODE 6\n", i);
			max11300_setup_port_6_mode(iio_dev, &ports[curr_port],
					true, i, PORT_MODE_6); // true = out
			curr_port++;
			max11300_setup_port_6_mode(iio_dev, &ports[curr_port],
					false, i, PORT_MODE_6); // false = in
			curr_port++;
			break;
		case PORT_MODE_7:
			dev_info(st->dev, "the port %d is configured as MODE 7\n", i);
			max11300_setup_port_7_mode(iio_dev, &ports[curr_port],
					false, i, PORT_MODE_7); // false = in
			curr_port++;
			break;
		case PORT_MODE_8:
			dev_info(st->dev, "the port %d is configured as MODE 8\n", i);
			max11300_setup_port_8_mode(iio_dev, &ports[curr_port],
					false, i, st->adc_negative_port[i], 
					PORT_MODE_8); // false = in
			curr_port++;
			break;
		case PORT_MODE_0:
			dev_info(st->dev, "the channel is set in default port mode_0\n");
			break;
		case PORT_MODE_1:
			dev_info(st->dev, "the channel %d is set in port mode_1\n", i);
			break;
		case PORT_MODE_3:
			dev_info(st->dev, "the channel %d is set in port mode_3\n", i);
			break;
		default:
			dev_info(st->dev, "bad port mode for channel %d\n", i);
		}
	}

	iio_dev->num_channels = curr_port;
	iio_dev->channels = ports;

	return 0;
}

int max11300_probe(struct device *dev, const char *name, 
		  const struct max11300_rw_ops *ops)
{

	/* create an iio device */
	struct iio_dev *iio_dev;

	/* create the global structure that will store the info of the device */
	struct max11300_state *st;

	u16 write_val;
	u16 read_val;
	u8 reg;
	int ret;

	write_val = 0;

	dev_info(dev, "max11300_probe() function is called\n");

	/* allocates memory fot the IIO device */
	iio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!iio_dev)
		return -ENOMEM;

	/* link the global data structure with the iio device */
	st = iio_priv(iio_dev);

	/* store in the global structure the spi device */
	st->dev = dev;

	/* store in the global structure the pointer to the MAX11300 SPI read and write functions */
	st->ops = ops;

	/* setup the number of ports of the MAX11300 device */
	st->num_ports = 20; 

	/* link the spi device with the iio device */
	dev_set_drvdata(dev, iio_dev);


	iio_dev->dev.parent = dev;
	iio_dev->name = name;

	/* 
	 * store the address of the iio_info structure, which contains pointer variables
	 * to IIO write/read callbacks 
	 */
	iio_dev->info = &max11300_info;
	iio_dev->modes = INDIO_DIRECT_MODE;

	/* reset the MAX11300 device */
	reg = DCR_ADDRESS;
	dev_info(st->dev, "the value of DCR_ADDRESS is %x\n", reg);
	write_val = RESET;
	dev_info(st->dev, "the value of reset is %x\n", write_val);
	ret = ops->reg_write(st, reg, write_val);
	if (ret != 0)
		goto error;

	/* return MAX11300 Device ID */
	reg = 0x00;
	ret = ops->reg_read(st, reg, &read_val);
	if (ret != 0)
		goto error;
	dev_info(st->dev, "the value of device ID is %x\n", read_val);

	/* Configure DACREF and ADCCTL */
	reg = DCR_ADDRESS;
	write_val = (DCR_ADCCTL_CONTINUOUS_SWEEP | DCR_DACREF); 
	dev_info(st->dev, "the value of DACREF_CONT_SWEEP is %x\n", write_val);
	ret = ops->reg_write(st, reg, write_val);
	udelay(200); 
	if (ret)
		goto error;
	dev_info(dev, "the setup of the device is done\n");

	/* Configure the IIO channels of the device */
	ret = max11300_alloc_ports(st); 
	if (ret)
		goto error;

	ret = max11300_set_port_modes(st);
	if (ret)
		goto error_reset_device;

	ret = iio_device_register(iio_dev);
	if (ret)
		goto error;

	ret = max11300_gpio_init(st);
	if (ret)
		goto error_dev_unregister;

	return 0;

error_dev_unregister:
	iio_device_unregister(iio_dev);

error_reset_device:
	/* reset the device */
	reg = DCR_ADDRESS;
	write_val = RESET;
	ret = ops->reg_write(st, reg, write_val);
	if (ret != 0)
		return ret;

error:
	return ret;
}
EXPORT_SYMBOL_GPL(max11300_probe);

int max11300_remove(struct device *dev)
{
	struct iio_dev *iio_dev = dev_get_drvdata(dev);

	iio_device_unregister(iio_dev);

	return 0;
}
EXPORT_SYMBOL_GPL(max11300_remove);

MODULE_AUTHOR("Alberto Liberal <aliberal@arroweurope.com>");
MODULE_DESCRIPTION("Maxim max11300 multi-port converters");
MODULE_LICENSE("GPL v2");


