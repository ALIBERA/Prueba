

#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/iio/iio.h>
#include <linux/iio/events.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/delay.h>


/* ADXL345 Register Map */
#define DEVID		0x00	/* R   Device ID */
#define THRESH_TAP	0x1D	/* R/W Tap threshold */
#define DUR		0x21	/* R/W Tap duration */
#define TAP_AXES	0x2A	/* R/W Axis control for tap/double tap */
#define ACT_TAP_STATUS	0x2B	/* R   Source of tap/double tap */
#define BW_RATE		0x2C	/* R/W Data rate and power mode control */
#define POWER_CTL	0x2D	/* R/W Power saving features control */
#define INT_ENABLE	0x2E	/* R/W Interrupt enable control */
#define INT_MAP		0x2F	/* R/W Interrupt mapping control */
#define INT_SOURCE	0x30	/* R   Source of interrupts */
#define DATA_FORMAT	0x31	/* R/W Data format control */
#define DATAX0		0x32	/* R   X-Axis Data 0 */
#define DATAX1		0x33	/* R   X-Axis Data 1 */
#define DATAY0		0x34	/* R   Y-Axis Data 0 */
#define DATAY1		0x35	/* R   Y-Axis Data 1 */
#define DATAZ0		0x36	/* R   Z-Axis Data 0 */
#define DATAZ1		0x37	/* R   Z-Axis Data 1 */
#define FIFO_CTL	0x38	/* R/W FIFO control */
#define FIFO_STATUS	0x39	/* R   FIFO status */

/* DEVIDs */
#define ID_ADXL345	0xE5

#define ADXL345_ACCEL_FIFO_LENGTH	32+1 // 33 entries * 3 = 99

/* INT_ENABLE/INT_MAP/INT_SOURCE Bits */
#define SINGLE_TAP	(1 << 6)
#define WATERMARK	(1 << 1)

/* TAP_AXES Bits */
#define TAP_X_EN	(1 << 2)
#define TAP_Y_EN	(1 << 1)
#define TAP_Z_EN	(1 << 0)

/* BW_RATE Bits */
#define LOW_POWER	(1 << 4)
#define RATE(x)		((x) & 0xF)

/* POWER_CTL Bits */
#define PCTL_MEASURE	(1 << 3)
#define PCTL_STANDBY	0X00

/* DATA_FORMAT Bits */
#define ADXL_FULL_RES	(1 << 3)

/* FIFO_CTL Bits */
#define FIFO_MODE(x)	(((x) & 0x3) << 6)
#define FIFO_BYPASS	0
#define FIFO_FIFO	1
#define FIFO_STREAM	2
#define SAMPLES(x)	((x) & 0x1F)

/* FIFO_STATUS Bits */
#define ADXL_X_AXIS			0
#define ADXL_Y_AXIS			1
#define ADXL_Z_AXIS			2

/* Interrupt AXIS Enable */
#define ADXL_TAP_X_EN (1 << 2)
#define ADXL_TAP_Y_EN (1 << 1)
#define ADXL_TAP_Z_EN (1 << 0)

static const int adxl345_uscale = 38300;

struct axis_triple {
	int x;
	int y;
	int z;
};

struct adxl345_data {
	struct regmap *regmap;
	struct iio_trigger *trig;
	struct axis_triple saved;
	u8 data_range;
	u8 tap_threshold;
	u8 tap_duration;
	u8 tap_axis_control;
	u8 data_rate;
	u8 fifo_mode;
	u8 watermark;
	u8 low_power_mode;
	int irq;
	int ev_enable;
	u32 int_mask;
	s64 timestamp;
};

static const struct iio_event_spec adxl345_event = {
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_EITHER,
		.mask_separate = BIT(IIO_EV_INFO_VALUE) |
				 BIT(IIO_EV_INFO_ENABLE) |
				 BIT(IIO_EV_INFO_PERIOD)
};


#define ADXL345_CHANNEL(reg, axis, idx) {					\
	.type = IIO_ACCEL,										\
	.modified = 1,											\
	.channel2 = IIO_MOD_##axis,								\
	.address = reg,											\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |	\
	BIT(IIO_CHAN_INFO_SAMP_FREQ),							\
	.scan_index = idx, \
	.scan_type = {											\
			.sign = 's',									\
			.realbits = 13,									\
			.storagebits = 16,								\
			.endianness = IIO_LE,							\
		},													\
	.event_spec = &adxl345_event,							\
	.num_event_specs = 1									\
}

static const struct iio_chan_spec adxl345_channels[] = {
	ADXL345_CHANNEL(DATAX0, X, 0),
	ADXL345_CHANNEL(DATAY0, Y, 1),
	ADXL345_CHANNEL(DATAZ0, Z, 2),
	IIO_CHAN_SOFT_TIMESTAMP(3),
};

static int adxl345_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	struct adxl345_data *data = iio_priv(indio_dev);
	__le16 regval;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		
		ret = regmap_bulk_read(data->regmap, chan->address, &regval,
				       sizeof(regval));
		if (ret)
			break;

		*val = sign_extend32(le16_to_cpu(regval), 12);

		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		*val2 = adxl345_uscale;
		return IIO_VAL_INT_PLUS_MICRO;

	default:
		return -EINVAL;
	}
	return ret;
}

static int adxl345_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long mask)
{
	struct adxl345_data *data = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		
		data->data_rate = RATE(val);
		return regmap_write(data->regmap, BW_RATE,
				data->data_rate | (data->low_power_mode ? LOW_POWER : 0));
	default :
		return -EINVAL;
	}
}

static int adxl345_read_event(struct iio_dev *indio_dev,
				   const struct iio_chan_spec *chan,
				   enum iio_event_type type,
				   enum iio_event_direction dir,
				   enum iio_event_info info,
				   int *val, int *val2)
{
	struct adxl345_data *data = iio_priv(indio_dev);

	switch (info) {
	case IIO_EV_INFO_VALUE:
		*val = data->tap_threshold;
		break;
	case IIO_EV_INFO_PERIOD:
		*val = data->tap_duration;
		break;
	default:
		return -EINVAL;
	}

	return IIO_VAL_INT;
}

static int adxl345_write_event(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    enum iio_event_type type,
				    enum iio_event_direction dir,
				    enum iio_event_info info,
				    int val, int val2)
{
	struct adxl345_data *data = iio_priv(indio_dev);

	switch (info) {
	case IIO_EV_INFO_VALUE:
		data->tap_threshold = val;
		return regmap_write(data->regmap, THRESH_TAP, data->tap_threshold);

	case IIO_EV_INFO_PERIOD:
		data->tap_duration = val;
		return regmap_write(data->regmap, DUR, data->tap_duration);
	default:
		return -EINVAL;
	}
}

static const struct regmap_config adxl345_spi_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	 /* Setting bits 7 and 6 enables multiple-byte read */
	.read_flag_mask = BIT(7) | BIT(6),
};

static const struct iio_info adxl345_info = {
	.driver_module	= THIS_MODULE,
	.read_raw	= adxl345_read_raw,
	.write_raw	= adxl345_write_raw,
	.read_event_value	= adxl345_read_event,
	.write_event_value	= adxl345_write_event,
};

/* Interrupt service routine */
static irqreturn_t adxl345_event_handler(int irq, void *handle)
{
	int ret;
	u32 tap_stat, int_stat, samples_stat;
	struct iio_dev *indio_dev = handle;
	struct adxl345_data *data = iio_priv(indio_dev);

	data->timestamp = iio_get_time_ns(indio_dev);
	/*
	 * ACT_TAP_STATUS should be read before clearing the interrupt
	 * Avoid reading ACT_TAP_STATUS in case TAP detection is disabled
	 * Read the ACT_TAP_STATUS if any of the axis has been enabled
	 */
	if (data->tap_axis_control & (TAP_X_EN | TAP_Y_EN | TAP_Z_EN))
		regmap_read(data->regmap, ACT_TAP_STATUS, &tap_stat);
	else
		tap_stat = 0;

	/* Read the INT_SOURCE (0x30) register
	 * The tap interrupt is cleared
	 */
	regmap_read(data->regmap, INT_SOURCE, &int_stat);
	/*
	 * if the SINGLE_TAP event has occurred the axl345_do_tap function
	 * is called with the ACT_TAP_STATUS register as an argument
	 */
	if (int_stat & (SINGLE_TAP)) {
		pr_info("single tap interrupt has occurred\n");

		if (tap_stat & TAP_X_EN){
			iio_push_event(indio_dev,
					 	 	   IIO_MOD_EVENT_CODE(IIO_ACCEL,
						       0,
						       IIO_MOD_X,
						       IIO_EV_TYPE_THRESH,
						       0),
						       data->timestamp);
		}
		if (tap_stat & TAP_Y_EN) {
			iio_push_event(indio_dev,
						 	 	  IIO_MOD_EVENT_CODE(IIO_ACCEL,
							      0,
							      IIO_MOD_Y,
							      IIO_EV_TYPE_THRESH,
							      0),
							      data->timestamp);
		}
		if (tap_stat & TAP_Z_EN) {
			iio_push_event(indio_dev,
							 	  IIO_MOD_EVENT_CODE(IIO_ACCEL,
								  0,
								  IIO_MOD_Z,
								  IIO_EV_TYPE_THRESH,
								  0),
								  data->timestamp);
		}
	}

	return IRQ_HANDLED;
}

static irqreturn_t adxl345_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct adxl345_data *data = iio_priv(indio_dev);
	
	/* we have to align timestamp to 8 bytes */
	s16 buf[8]; // 3 axis x s16 + 1 x s16 pad + 4 x s16 tstamp
	int i, ret, j = 0, base = DATAX0;
	__le16 sample;

	for_each_set_bit(i, indio_dev->active_scan_mask,
				 indio_dev->masklength) {
		ret = regmap_bulk_read(data->regmap, base + i * sizeof(__le16), &sample, sizeof(__le16));
		
		if (ret < 0)
			goto done;
		buf[j++] = sample;
	}

	iio_push_to_buffers_with_timestamp(indio_dev, buf,
						   iio_get_time_ns(indio_dev));

done:
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

static const struct iio_trigger_ops adxl345_trigger_ops = {
	.owner = THIS_MODULE,
};

int adxl345_core_probe(struct device *dev, struct regmap *regmap,
		       const char *name)
{
	struct adxl345_data *data;
	struct iio_dev *indio_dev;
	u32 regval;
	int ret, gpio_nb, irq;

	ret = regmap_read(regmap, DEVID, &regval);
	if (ret < 0) {
		dev_err(dev, "Error reading device ID: %d\n", ret);
		return ret;
	}

	if (regval != ID_ADXL345) {
		dev_err(dev, "Invalid device ID: %x, expected %x\n",
			regval, ID_ADXL345);
		return -ENODEV;
	}

	gpio_nb = of_get_gpio(dev->of_node, 0);
	pr_info("GPIO: %d\n", gpio_nb);
	irq = irq_of_parse_and_map(dev->of_node, 0);
	pr_info("IRQ_parsing_devicetree: %d\n", irq);
	irq = gpio_to_irq(gpio_nb);
	pr_info("IRQ_using_gpio_to_irq: %d\n", irq);

	indio_dev = devm_iio_device_alloc(dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	dev_set_drvdata(dev, indio_dev);

	/* Initialize our private device structure */
	data->regmap = regmap;
	data->irq = irq;
	data->data_range = ADXL_FULL_RES;
	data->tap_threshold = 50;
	data->tap_duration = 3;
	data->tap_axis_control = ADXL_TAP_Z_EN;
	data->data_rate = 8;
	data->fifo_mode = FIFO_BYPASS;
	data->watermark = 32;
	data->low_power_mode = 0;

	indio_dev->dev.parent = dev;
	indio_dev->name = name;
	indio_dev->info = &adxl345_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = adxl345_channels;
	indio_dev->num_channels = ARRAY_SIZE(adxl345_channels);

	/* Initialize the ADXL345 registers */

	/* 13-bit full resolution right justified */
	ret = regmap_write(data->regmap, DATA_FORMAT,
				   data->data_range);
	if (ret < 0)
		goto error_reg;

	/* Set the tap threshold and duration */
	ret = regmap_write(data->regmap, THRESH_TAP, data->tap_threshold);
	if (ret < 0)
		goto error_reg;
	ret = regmap_write(data->regmap, DUR, data->tap_duration);
	if (ret < 0)
		goto error_reg;

	/* set the axis where the tap will be detected */
	ret = regmap_write(data->regmap, TAP_AXES, data->tap_axis_control);
	if (ret < 0)
		goto error_reg;

	/* set the data rate and the axis reading power
	* mode, less or higher noise reducing power
	*/
	ret = regmap_write(data->regmap, BW_RATE, RATE(data->data_rate) |
			 (data->low_power_mode ? LOW_POWER : 0));
	if (ret < 0)
		goto error_reg;

	/* Set the FIFO mode, no FIFO by default */
	ret = regmap_write(data->regmap, FIFO_CTL, FIFO_MODE(data->fifo_mode) |
				SAMPLES(data->watermark));
	if (ret < 0)
		goto error_reg;

	/* Map all INTs to INT1 pin */
	ret = regmap_write(data->regmap, INT_MAP, 0);
	if (ret < 0)
		goto error_reg;

	/* Enables interrupts */
	if (data->tap_axis_control & (TAP_X_EN | TAP_Y_EN | TAP_Z_EN))
		data->int_mask |= SINGLE_TAP;

	/* watermark is the number of samples until maximum of 32 */
	//if (data->watermark)
		//data->int_mask |= WATERMARK;

	ret = regmap_write(data->regmap, INT_ENABLE, data->int_mask);
	if (ret < 0)
		goto error_reg;

	/* Enable measurement mode */
	ret = regmap_write(data->regmap, POWER_CTL,
			   PCTL_MEASURE);
	if (ret < 0)
		goto error_reg;

	/* Request threaded interrupt */
	ret = devm_request_threaded_irq(dev, irq, NULL,
				adxl345_event_handler, IRQF_TRIGGER_HIGH | IRQF_ONESHOT, dev_name(dev), indio_dev);
	if (ret) {
		pr_err("Failed to request interrupt %d, error %d\n", irq, ret);
		return ret;
	}

	data->trig = devm_iio_trigger_alloc(dev, "%s-dev%d",
						      indio_dev->name,
						      indio_dev->id);
	if (!data->trig)
		return -ENOMEM;

	data->trig->dev.parent = dev;
	data->trig->ops = &adxl345_trigger_ops; // needed to register the trigger
	iio_trigger_set_drvdata(data->trig, data);

	ret = devm_iio_trigger_register(dev, data->trig);
	if (ret) {
		pr_info ("trigger register failed\n");
		return ret;
	}
	ret = devm_iio_triggered_buffer_setup(dev, indio_dev, NULL,
			adxl345_trigger_handler, NULL);
	if (ret) {
		dev_err(dev, "unable to setup triggered buffer\n");
		return ret;
	}

	ret = devm_iio_device_register(dev, indio_dev);
	if (ret) {
		dev_err(dev, "iio_device_register failed: %d\n", ret);
		regmap_write(data->regmap, POWER_CTL, PCTL_STANDBY);
	}

error_reg:
	return ret;
}

int adxl345_core_remove(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct adxl345_data *data = iio_priv(indio_dev);
	pr_info("my_remove() function is called.\n");
	return regmap_write(data->regmap, POWER_CTL, PCTL_STANDBY);
}

static int adxl345_spi_probe(struct spi_device *spi)
{
	const struct spi_device_id *id = spi_get_device_id(spi);
	struct regmap *regmap;

	regmap = devm_regmap_init_spi(spi, &adxl345_spi_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&spi->dev, "Error initializing spi regmap: %ld\n",
			PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	return adxl345_core_probe(&spi->dev, regmap, id->name);
}

static int adxl345_spi_remove(struct spi_device *spi)
{
	return adxl345_core_remove(&spi->dev);
}

static const struct spi_device_id adxl345_id[] = {
	{ .name = "adxl345", },
	{ }
};
MODULE_DEVICE_TABLE(spi, adxl345_id);

static const struct of_device_id adxl345_dt_ids[] = {
	{ .compatible = "arrow,adxl345" },
	{ },
};
MODULE_DEVICE_TABLE(of, adxl345_dt_ids);

static struct spi_driver adxl345_driver = {
	.driver = {
		.name	= "adxl345",
		.owner = THIS_MODULE,
		.of_match_table = adxl345_dt_ids,
	},
	.probe		= adxl345_spi_probe,
	.remove		= adxl345_spi_remove,
	.id_table	= adxl345_id,
};

module_spi_driver(adxl345_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Alberto Liberal <aliberal@arroweurope.com>");
MODULE_DESCRIPTION("ADXL345 Three-Axis Accelerometer SPI Bus Driver");


