
/*
 * Driver for Cypress cy8c9520a I/O Expander 
 * Based on Intel Corporation cy8c9520a driver.
 */

#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio/driver.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pwm.h>
#include <linux/slab.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>

#define DRV_NAME			"cy8c9520a"

/* cy8c9520a settings */
#define NGPIO				20
#define DEVID_CY8C9520A			0x20
#define NPORTS				3
#define NPWM				4
#define PWM_MAX_PERIOD			0xff
#define PWM_BASE_ID			0
#define PWM_CLK				0x00	/* see resulting PWM_TCLK_NS */
#define PWM_TCLK_NS			31250	/* 32kHz */
#define PWM_UNUSED			20

/* Register offset  */
#define REG_INPUT_PORT0			0x00
#define REG_OUTPUT_PORT0		0x08
#define REG_INTR_STAT_PORT0		0x10
#define REG_PORT_SELECT			0x18
#define REG_INTR_MASK			0x19
#define REG_PIN_DIR			0x1c
#define REG_DRIVE_PULLUP		0x1d
#define REG_DRIVE_PULLDOWN		0x1e
#define REG_DEVID_STAT			0x2e

/* Register PWM */
#define REG_SELECT_PWM			0x1a
#define REG_PWM_SELECT			0x28
#define REG_PWM_CLK			0x29
#define REG_PWM_PERIOD			0x2a
#define REG_PWM_PULSE_W			0x2b

/* definition of the global structure for the driver */
struct cy8c9520a {
	struct i2c_client 	*client;
	struct gpio_chip 	gpio_chip;
	struct pwm_chip 	pwm_chip;
	struct gpio_desc 	*gpio;
	int 			irq;
	struct mutex 		lock;
	/* protect serialized access to the interrupt controller bus */
	struct mutex 		irq_lock;
	/* cached output registers */
	u8 			outreg_cache[NPORTS];
	/* cached IRQ mask */
	u8 			irq_mask_cache[NPORTS];
	/* IRQ mask to be applied */
	u8 			irq_mask[NPORTS];
	int 			pwm_number[NPWM];

	struct pinctrl_dev	*pctldev;
	struct pinctrl_desc	pinctrl_desc;
};

/* Per-port GPIO offset */
static const u8 cy8c9520a_port_offs[] = {
	0,
	8,
	16,
};

static const struct pinctrl_pin_desc cy8c9520a_pins[] = {
	PINCTRL_PIN(0, "gpio0"),
	PINCTRL_PIN(1, "gpio1"),
	PINCTRL_PIN(2, "gpio2"),
	PINCTRL_PIN(3, "gpio3"),
	PINCTRL_PIN(4, "gpio4"),
	PINCTRL_PIN(5, "gpio5"),
	PINCTRL_PIN(6, "gpio6"),
	PINCTRL_PIN(7, "gpio7"),
	PINCTRL_PIN(8, "gpio8"),
	PINCTRL_PIN(9, "gpio9"),
	PINCTRL_PIN(10, "gpio10"),
	PINCTRL_PIN(11, "gpio11"),
	PINCTRL_PIN(12, "gpio12"),
	PINCTRL_PIN(13, "gpio13"),
	PINCTRL_PIN(14, "gpio14"),
	PINCTRL_PIN(15, "gpio15"),
	PINCTRL_PIN(16, "gpio16"),
	PINCTRL_PIN(17, "gpio17"),
	PINCTRL_PIN(18, "gpio18"),
	PINCTRL_PIN(19, "gpio19"),
};

/* return the port of the gpio */
static inline u8 cypress_get_port(unsigned int gpio)
{
	u8 i = 0;
	for (i = 0; i < sizeof(cy8c9520a_port_offs) - 1; i ++) {
		if (! (gpio / cy8c9520a_port_offs[i + 1])) 
			break;
	}
	return i;
}

/* get the gpio offset inside its respective port */
static inline u8 cypress_get_offs(unsigned gpio, u8 port)
{
	return gpio - cy8c9520a_port_offs[port];
}

static int cygpio_pinctrl_get_groups_count(struct pinctrl_dev *pctldev)
{
	return 0;
}

static const char *cygpio_pinctrl_get_group_name(struct pinctrl_dev *pctldev,
						unsigned int group)
{
	return NULL;
}

static int cygpio_pinctrl_get_group_pins(struct pinctrl_dev *pctldev,
					unsigned int group,
					const unsigned int **pins,
					unsigned int *num_pins)
{
	return -ENOTSUPP;
}

/* 
 * global pin control operations, to be implemented by
 * pin controller drivers
 * pinconf_generic_dt_node_to_map_pin function
 * will parse a device tree "pin configuration node", and create
 * mapping table entries for it
 */
static const struct pinctrl_ops cygpio_pinctrl_ops = {
	.get_groups_count = cygpio_pinctrl_get_groups_count,
	.get_group_name = cygpio_pinctrl_get_group_name,
	.get_group_pins = cygpio_pinctrl_get_group_pins,
#ifdef CONFIG_OF
	.dt_node_to_map = pinconf_generic_dt_node_to_map_pin,
	.dt_free_map = pinconf_generic_dt_free_map,
#endif
};


/* Configure the Drive Mode Register Settings */
static int cygpio_pinconf_set(struct pinctrl_dev *pctldev, unsigned int pin,
			      unsigned long *configs, unsigned int num_configs)
{
	struct cy8c9520a *cygpio = pinctrl_dev_get_drvdata(pctldev);
	struct i2c_client *client = cygpio->client;
	enum pin_config_param param;
	u32 arg;
	int ret = 0; 
	int i;
	u8 offs = 0;
	u8 val = 0;
	u8 port = cypress_get_port(pin);
	u8 pin_offset = cypress_get_offs(pin, port);

	dev_err(&client->dev, "cygpio_pinconf_set function is called\n");

	mutex_lock(&cygpio->lock);

	for (i = 0; i < num_configs; i++) {
		param = pinconf_to_config_param(configs[i]);
		arg = pinconf_to_config_argument(configs[i]);

		switch (param) {
		case PIN_CONFIG_BIAS_PULL_UP:
			offs = 0x0;
			dev_info(&client->dev, 
				"The pin %d drive mode is PIN_CONFIG_BIAS_PULL_UP\n", pin);
			break;
		case PIN_CONFIG_BIAS_PULL_DOWN:
			offs = 0x01;
			dev_info(&client->dev, 
				"The pin %d drive mode is PIN_CONFIG_BIAS_PULL_DOWN\n", pin);
			break;
		case PIN_CONFIG_DRIVE_STRENGTH:
			offs = 0x04;
			dev_info(&client->dev, 
				"The pin %d drive mode is PIN_CONFIG_DRIVE_STRENGTH\n", pin);
			break;
		case PIN_CONFIG_BIAS_HIGH_IMPEDANCE:
			offs = 0x06;
			dev_info(&client->dev, 
				"The pin %d drive mode is PIN_CONFIG_BIAS_HIGH_IMPEDANCE\n", pin);
			break;
		default:
			dev_err(&client->dev, "Invalid config param %04x\n", param);
			return -ENOTSUPP;
		}

		ret = i2c_smbus_write_byte_data(client, REG_PORT_SELECT, port);
		if (ret < 0) {
			dev_err(&client->dev, "can't select port %u\n", port);
			goto end;
		}

		ret = i2c_smbus_read_byte_data(client, REG_DRIVE_PULLUP + offs);
		if (ret < 0) {
			dev_err(&client->dev, "can't read pin direction\n");
			goto end;
		}

		val = (u8)(ret | BIT(pin_offset));

		ret = i2c_smbus_write_byte_data(client, REG_DRIVE_PULLUP + offs, val);
		if (ret < 0) {
			dev_err(&client->dev, "can't set drive mode port %u\n", port);
			goto end;
		}

	}

end:
	mutex_unlock(&cygpio->lock);
	return ret;
}

/* 
 * pin config operations, to be implemented by
 * pin configuration capable drivers
 * pin_config_set: configure an individual pin
 */
static const struct pinconf_ops cygpio_pinconf_ops = {
	.pin_config_set = cygpio_pinconf_set,
	.is_generic = true,
};


/* 
 * struct gpio_chip get callback function. 
 * It gets the input value of the GPIO line (0=low, 1=high)
 * accessing to the REG_INPUT_PORT register 
 */
static int cy8c9520a_gpio_get(struct gpio_chip *chip, 
			     unsigned int gpio)
{
	int ret;
	u8 port, in_reg;

	struct cy8c9520a *cygpio = gpiochip_get_data(chip);

	dev_info(chip->parent, "cy8c9520a_gpio_get function is called\n");

	/* get the input port address address (in_reg) for the GPIO */
	port = cypress_get_port(gpio);
	in_reg = REG_INPUT_PORT0 + port;

	dev_info(chip->parent, "the in_reg address is %u\n", in_reg);

	mutex_lock(&cygpio->lock);

	ret = i2c_smbus_read_byte_data(cygpio->client, in_reg);
	if (ret < 0) {
		dev_err(chip->parent, "can't read input port %u\n", in_reg);
	}

	dev_info(chip->parent, 
		"cy8c9520a_gpio_get function with %d value is returned\n", 
		ret);

	mutex_unlock(&cygpio->lock);

	/* 
	 * check the status of the GPIO in its input port register
	 * and return it. If expression is not 0 returns 1 
	 */
	return !!(ret & BIT(cypress_get_offs(gpio, port)));
}

/* 
 * struct gpio_chip set callback function. 
 * It sets the output value of the GPIO line in 
 * GPIO ACTIVE_HIGH mode (0=low, 1=high)
 * writing to the REG_OUTPUT_PORT register
 */
static void cy8c9520a_gpio_set(struct gpio_chip *chip,
			      unsigned int gpio, int val)
{
	int ret;
	u8 port, out_reg;
	struct cy8c9520a *cygpio = gpiochip_get_data(chip);

	dev_info(chip->parent, 
		"cy8c9520a_gpio_set_value func with %d value is called\n", 
		val);

	/* get the output port address address (out_reg) for the GPIO */
	port = cypress_get_port(gpio);
	out_reg = REG_OUTPUT_PORT0 + port;

	mutex_lock(&cygpio->lock);

	/* 
	 * if val is 1, gpio output level is high
	 * if val is 0, gpio output level is low
	 * the output registers were previously cached in cy8c9520a_setup()
	 */
	if (val) {
		cygpio->outreg_cache[port] |= BIT(cypress_get_offs(gpio, port));
	} else {
		cygpio->outreg_cache[port] &= ~BIT(cypress_get_offs(gpio, port));
	}

	ret = i2c_smbus_write_byte_data(cygpio->client, out_reg,
					cygpio->outreg_cache[port]);
	if (ret < 0) {
		dev_err(chip->parent, "can't write output port %u\n", port);
	}

	mutex_unlock(&cygpio->lock);
}

/* 
 * struct gpio_chip direction_output callback function. 
 * It configures the GPIO as an output writing to
 * the REG_PIN_DIR register of the selected port
 */
static int cy8c9520a_gpio_direction_output(struct gpio_chip *chip, 
					  unsigned int gpio, int val)
{
	int ret;
	u8 pins, port;

	struct cy8c9520a *cygpio = gpiochip_get_data(chip);

	/* gets the port number of the gpio */
	port = cypress_get_port(gpio);

	dev_info(chip->parent, "cy8c9520a_gpio_direction output is called\n");

	mutex_lock(&cygpio->lock);

	/* select the port where we want to config the GPIO as output */
	ret = i2c_smbus_write_byte_data(cygpio->client, REG_PORT_SELECT, port);
	if (ret < 0) {
		dev_err(chip->parent, "can't select port %u\n", port);
		goto err;
	}

	ret = i2c_smbus_read_byte_data(cygpio->client, REG_PIN_DIR);
	if (ret < 0) {
		dev_err(chip->parent, "can't read pin direction\n");
		goto err;
	}

	/* simply transform int to u8 */
	pins = (u8)ret & 0xff;

	/* add the direction of the new pin. Set 1 if input and set 0 is output */
	pins &= ~BIT(cypress_get_offs(gpio, port));

	ret = i2c_smbus_write_byte_data(cygpio->client, REG_PIN_DIR, pins);
	if (ret < 0) {
		dev_err(chip->parent, "can't write pin direction\n");
	}

err:
	mutex_unlock(&cygpio->lock);
	cy8c9520a_gpio_set(chip, gpio, val);
	return ret;
}

/* 
 * struct gpio_chip direction_input callback function. 
 * It configures the GPIO as an input writing to
 * the REG_PIN_DIR register of the selected port
 */
static int cy8c9520a_gpio_direction_input(struct gpio_chip *chip, 
					 unsigned int gpio)
{
	int ret;
	u8 pins, port;

	struct cy8c9520a *cygpio = gpiochip_get_data(chip);

	/* gets the port number of the gpio */
	port = cypress_get_port(gpio);

	dev_info(chip->parent, "cy8c9520a_gpio_direction input is called\n");

	mutex_lock(&cygpio->lock);

	/* select the port where we want to config the GPIO as input */
	ret = i2c_smbus_write_byte_data(cygpio->client, REG_PORT_SELECT, port);
	if (ret < 0) {
		dev_err(chip->parent, "can't select port %u\n", port);
		goto err;
	}

	ret = i2c_smbus_read_byte_data(cygpio->client, REG_PIN_DIR);
	if (ret < 0) {
		dev_err(chip->parent, "can't read pin direction\n");
		goto err;
	}

	/* simply transform int to u8 */
	pins = (u8)ret & 0xff;

	/* 
         * add the direction of the new pin. 
         * Set 1 if input (out == 0) and set 0 is ouput (out == 1) 
         */
	pins |= BIT(cypress_get_offs(gpio, port));

	ret = i2c_smbus_write_byte_data(cygpio->client, REG_PIN_DIR, pins);
	if (ret < 0) {
		dev_err(chip->parent, "can't write pin direction\n");
		goto err;
	}

err:
	mutex_unlock(&cygpio->lock);
	return ret;
}

/* function to lock access to slow bus (i2c) chips */
static void cy8c9520a_irq_bus_lock(struct irq_data *d)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(d);
	struct cy8c9520a *cygpio = gpiochip_get_data(chip);
	dev_info(chip->parent, "cy8c9520a_irq_bus_lock is called\n");
	mutex_lock(&cygpio->irq_lock);
}

/* 
 * function to sync and unlock slow bus (i2c) chips 
 * REG_INTR_MASK register is accessed via I2C
 * write 0 to the interrupt mask register line to 
 * activate the interrupt on the GPIO 
 */
static void cy8c9520a_irq_bus_sync_unlock(struct irq_data *d)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(d);
	struct cy8c9520a *cygpio = gpiochip_get_data(chip);
	int ret, i;
	unsigned int gpio;
	u8 port;
	dev_info(chip->parent, "cy8c9520a_irq_bus_sync_unlock is called\n");
	gpio = d->hwirq;
	port = cypress_get_port(gpio);

	/* irq_mask_cache stores the last value of irq_mask for each port */ 
	for (i = 0; i < NPORTS; i++) {
		/* 
		 * check if some of the bits have changed from the last cached value 
   		 * irq_mask registers were initialized in cy8c9520a_irq_setup()		 
		 */
		if (cygpio->irq_mask_cache[i] ^ cygpio->irq_mask[i]) { 
			dev_info(chip->parent, "gpio %u is unmasked\n", gpio);
			cygpio->irq_mask_cache[i] = cygpio->irq_mask[i];
			ret = i2c_smbus_write_byte_data(cygpio->client, 
							REG_PORT_SELECT, i);
			if (ret < 0) {
				dev_err(chip->parent, "can't select port %u\n", port);
				goto err;
			}

			/* enable the interrupt for the GPIO unmasked */
			ret = i2c_smbus_write_byte_data(cygpio->client, REG_INTR_MASK, 
							cygpio->irq_mask[i]);
			if (ret < 0) {
				dev_err(chip->parent, 
					"can't write int mask on port %u\n", port);
				goto err;
			}

			ret = i2c_smbus_read_byte_data(cygpio->client, REG_INTR_MASK);
			dev_info(chip->parent, "the REG_INTR_MASK value is %d\n", ret);

		}
	}

err:
	mutex_unlock(&cygpio->irq_lock);
}

/* 
 * mask (disable) the GPIO interrupt. 
 * In the initial setup all the int lines are masked
 */
static void cy8c9520a_irq_mask(struct irq_data *d)
{
	u8 port;
	struct gpio_chip *chip = irq_data_get_irq_chip_data(d);
	struct cy8c9520a *cygpio = gpiochip_get_data(chip);
	unsigned gpio = d->hwirq;
	port = cypress_get_port(gpio);
	dev_info(chip->parent, "cy8c9520a_irq_mask is called\n");

	cygpio->irq_mask[port] |= BIT(cypress_get_offs(gpio, port));
}

/* 
 * unmask (enable) the GPIO interrupt. 
 * In the initial setup all the int lines are masked
 */
static void cy8c9520a_irq_unmask(struct irq_data *d)
{
	u8 port;
	struct gpio_chip *chip = irq_data_get_irq_chip_data(d);
	struct cy8c9520a *cygpio = gpiochip_get_data(chip);
	unsigned gpio = d->hwirq;
	port = cypress_get_port(gpio);
	dev_info(chip->parent, "cy8c9520a_irq_unmask is called\n");

	cygpio->irq_mask[port] &= ~BIT(cypress_get_offs(gpio, port));
}

/* set the flow type (IRQ_TYPE_LEVEL/etc.) of the IRQ */ 
static int cy8c9520a_irq_set_type(struct irq_data *d, unsigned int type)
{
	int ret = 0;
	struct gpio_chip *chip = irq_data_get_irq_chip_data(d);
	struct cy8c9520a *cygpio = gpiochip_get_data(chip);

	dev_info(chip->parent, "cy8c9520a_irq_set_type is called\n");

	if ((type != IRQ_TYPE_EDGE_BOTH) && (type != IRQ_TYPE_EDGE_FALLING)) {
		dev_err(&cygpio->client->dev, "irq %d: unsupported type %d\n",
			d->irq, type);
		ret = -EINVAL;
		goto err;
	}

err:
	return ret; 
}

/* Iinitialization of the irq_chip structure with callback functions */
static struct irq_chip cy8c9520a_irq_chip = {
	.name			= "cy8c9520a-irq",
	.irq_mask		= cy8c9520a_irq_mask,
	.irq_unmask		= cy8c9520a_irq_unmask,
	.irq_bus_lock		= cy8c9520a_irq_bus_lock,
	.irq_bus_sync_unlock	= cy8c9520a_irq_bus_sync_unlock,
	.irq_set_type		= cy8c9520a_irq_set_type,
};

/* 
 * interrupt handler for the cy8c9520a. It is called when
 * there is a rising or falling edge in the unmasked GPIO
 */
static irqreturn_t cy8c9520a_irq_handler(int irq, void *devid)
{
	struct cy8c9520a *cygpio = devid;
	u8 stat[NPORTS], pending;
	unsigned port, gpio, gpio_irq;
	int ret;

	pr_info ("the interrupt ISR has been entered\n");	

	/* 
         * store in stat and clear (to enable ints)
         * the three interrupt status registers by reading them
         */
	ret = i2c_smbus_read_i2c_block_data(cygpio->client, 
					   REG_INTR_STAT_PORT0,
					   NPORTS, stat);
	if (ret < 0) {
		memset(stat, 0, sizeof(stat));
	}

	ret = IRQ_NONE;

	for (port = 0; port < NPORTS; port ++) {
		mutex_lock(&cygpio->irq_lock);

		/* 
		 * In every port check the GPIOs that have their int unmasked 
		 * and whose bits have been enabled in their REG_INTR_STAT_PORT
		 * register due to an interrupt in the GPIO, and store the new
		 * value in the pending register
		 */
		pending = stat[port] & (~cygpio->irq_mask[port]);
		mutex_unlock(&cygpio->irq_lock);

		/* Launch the ISRs of all the gpios that requested an interrupt */
		while (pending) {
			ret = IRQ_HANDLED;
			/* get the first gpio that has got an int */
			gpio = __ffs(pending);

			/* clears the gpio in the pending register */
			pending &= ~BIT(gpio);

			/* gets the int number associated to this gpio */
			gpio_irq = cy8c9520a_port_offs[port] + gpio;

			/* launch the ISR of the GPIO child driver */
			handle_nested_irq(irq_find_mapping(cygpio->gpio_chip.irq.domain,
						   gpio_irq));

		}
	}

	return ret;
}

/* 
 * select the period and the duty cycle of the PWM signal (in nanoseconds) 
 * echo 100000 > pwm1/period 
 * echo 50000 > pwm1/duty_cycle 
 */
static int cy8c9520a_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
			     int duty_ns, int period_ns)
{
	int ret;
	int period = 0, duty = 0;

	struct cy8c9520a *cygpio =
	    container_of(chip, struct cy8c9520a, pwm_chip);
	struct i2c_client *client = cygpio->client;

	dev_info(&client->dev, "cy8c9520a_pwm_config is called\n");

	if (pwm->pwm > NPWM) {
		return -EINVAL;
	}

	period = period_ns / PWM_TCLK_NS;
	duty = duty_ns / PWM_TCLK_NS;

	/*
	 * Check period's upper bound.  Note the duty cycle is already sanity
	 * checked by the PWM framework.
	 */
	if (period > PWM_MAX_PERIOD) {
		dev_err(&client->dev, "period must be within [0-%d]ns\n",
			PWM_MAX_PERIOD * PWM_TCLK_NS);
		return -EINVAL;
	}

	mutex_lock(&cygpio->lock);

	/* 
	 * select the pwm number (from 0 to 3) 
	 * to set the period and the duty for the enabled pwm pins 
         */
	ret = i2c_smbus_write_byte_data(client, REG_PWM_SELECT, (u8)pwm->pwm);
	if (ret < 0) {
		dev_err(&client->dev, "can't write to REG_PWM_SELECT\n");
		goto end;
	}

	ret = i2c_smbus_write_byte_data(client, REG_PWM_PERIOD, (u8)period);
	if (ret < 0) {
		dev_err(&client->dev, "can't write to REG_PWM_PERIOD\n");
		goto end;
	}

	ret = i2c_smbus_write_byte_data(client, REG_PWM_PULSE_W, (u8)duty);
	if (ret < 0) {
		dev_err(&client->dev, "can't write to REG_PWM_PULSE_W\n");
		goto end;
	}

end:
	mutex_unlock(&cygpio->lock);

	return ret;
}

/* 
 * Enable the PWM signal
 * echo 1 > pwm1/enable 
 */
static int cy8c9520a_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	int ret, gpio, port, pin;
	u8 out_reg, val;

	struct cy8c9520a *cygpio =
	    container_of(chip, struct cy8c9520a, pwm_chip);
	struct i2c_client *client = cygpio->client;

	dev_info(&client->dev, "cy8c9520a_pwm_enable is called\n");

	if (pwm->pwm > NPWM) {
		return -EINVAL;
	}

	/* 
	 * get the pin configured as pwm in the device tree 
	 * for this pwm port (pwm_device) 
	 */
	gpio = cygpio->pwm_number[pwm->pwm];
	port = cypress_get_port(gpio);
	pin = cypress_get_offs(gpio, port);
	out_reg = REG_OUTPUT_PORT0 + port;

	/* 
         * Set pin as output driving high and select the port 
         * where the pwm will be set 
	 */
	ret = cy8c9520a_gpio_direction_output(&cygpio->gpio_chip, gpio, 1);
	if (val < 0) {
		dev_err(&client->dev, "can't set pwm%u as output\n", pwm->pwm);
		return ret;
	}

	mutex_lock(&cygpio->lock);

	/* Enable PWM pin in the selected port */
	val = i2c_smbus_read_byte_data(client, REG_SELECT_PWM);
	if (val < 0) {
		dev_err(&client->dev, "can't read REG_SELECT_PWM\n");
		ret = val;
		goto end;
	}
	val |= BIT((u8)pin);
	ret = i2c_smbus_write_byte_data(client, REG_SELECT_PWM, val);
	if (ret < 0) {
		dev_err(&client->dev, "can't write to SELECT_PWM\n");
		goto end;
	}

end:
	mutex_unlock(&cygpio->lock);

	return ret;
}

/* 
 * Disable the PWM signal
 * echo 0 > pwm1/enable 
 */
static void cy8c9520a_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	int ret, gpio, port, pin;
	u8 val;

	struct cy8c9520a *cygpio =
	    container_of(chip, struct cy8c9520a, pwm_chip);
	struct i2c_client *client = cygpio->client;

	dev_info(&client->dev, "cy8c9520a_pwm_disable is called\n");

	if (pwm->pwm > NPWM) {
		return;
	}

	gpio = cygpio->pwm_number[pwm->pwm];
	if (PWM_UNUSED == gpio) {
		dev_err(&client->dev, "pwm%d is unused\n", pwm->pwm);
		return;
	}

	port = cypress_get_port(gpio);
	pin = cypress_get_offs(gpio, port);

	mutex_lock(&cygpio->lock);

	/* Disable PWM */
	val = i2c_smbus_read_byte_data(client, REG_SELECT_PWM);
	if (val < 0) {
		dev_err(&client->dev, "can't read REG_SELECT_PWM\n");
		goto end;
	}
	val &= ~BIT((u8)pin);
	ret = i2c_smbus_write_byte_data(client, REG_SELECT_PWM, val);
	if (ret < 0) {
		dev_err(&client->dev, "can't write to SELECT_PWM\n");
	}

end:
	mutex_unlock(&cygpio->lock);

	return;
}

/* 
 * Request the PWM device 
 * echo 0 > export 
 */
static int cy8c9520a_pwm_request(struct pwm_chip *chip, struct pwm_device *pwm)
{
	int gpio = 0;
	struct cy8c9520a *cygpio =
	    container_of(chip, struct cy8c9520a, pwm_chip);
	struct i2c_client *client = cygpio->client;

	dev_info(&client->dev, "cy8c9520a_pwm_request is called\n");

	if (pwm->pwm > NPWM) {
		return -EINVAL;
	}

	gpio = cygpio->pwm_number[pwm->pwm];
	if (PWM_UNUSED == gpio) {
		dev_err(&client->dev, "pwm%d unavailable\n", pwm->pwm);
		return -EINVAL;
	}

	return 0;
}

/* Declare the PWM callback functions */
static const struct pwm_ops cy8c9520a_pwm_ops = {
	.request = cy8c9520a_pwm_request,
	.config = cy8c9520a_pwm_config,
	.enable = cy8c9520a_pwm_enable,
	.disable = cy8c9520a_pwm_disable,
};

/* Initial setup for the cy8c9520a */
static int cy8c9520a_setup(struct cy8c9520a *cygpio)
{
	int ret, i;
	struct i2c_client *client = cygpio->client;
	
	/* Disable PWM, set all GPIOs as input.  */
	for (i = 0; i < NPORTS; i ++) {
		ret = i2c_smbus_write_byte_data(client, REG_PORT_SELECT, i);
		if (ret < 0) {
			dev_err(&client->dev, "can't select port %u\n", i);
			goto end;
		}

		ret = i2c_smbus_write_byte_data(client, REG_SELECT_PWM, 0x00);
		if (ret < 0) {
			dev_err(&client->dev, "can't write to SELECT_PWM\n");
			goto end;
		}

		ret = i2c_smbus_write_byte_data(client, REG_PIN_DIR, 0xff);
		if (ret < 0) {
			dev_err(&client->dev, "can't write to PIN_DIR\n");
			goto end;
		}
	}

	/* Cache the output registers (Output Port 0, Output Port 1, Output Port 2) */
	ret = i2c_smbus_read_i2c_block_data(client, REG_OUTPUT_PORT0,
					    sizeof(cygpio->outreg_cache),
					    cygpio->outreg_cache);
	if (ret < 0) {
		dev_err(&client->dev, "can't cache output registers\n");
		goto end;
	}

	/* Set default PWM clock source.  */
	for (i = 0; i < NPWM; i ++) {
		ret = i2c_smbus_write_byte_data(client, REG_PWM_SELECT, i);
		if (ret < 0) {
			dev_err(&client->dev, "can't select pwm %u\n", i);
			goto end;
		}

		ret = i2c_smbus_write_byte_data(client, REG_PWM_CLK, PWM_CLK);
		if (ret < 0) {
			dev_err(&client->dev, "can't write to REG_PWM_CLK\n");
			goto end;
		}
	}

	dev_info(&client->dev, "the cy8c9520a_setup is done\n");

end:
	return ret;
}

/* Interrupt setup for the cy8c9520a */
static int cy8c9520a_irq_setup(struct cy8c9520a *cygpio)
{
	struct i2c_client *client = cygpio->client;
	struct gpio_chip *chip = &cygpio->gpio_chip;
	u8 dummy[NPORTS];
	int ret, i;

	mutex_init(&cygpio->irq_lock);

	dev_info(&client->dev, "the cy8c9520a_irq_setup function is entered\n");

	/* 
	 * Clear interrupt state registers by reading the three registers
	 * Interrupt Status Port0, Interrupt Status Port1, Interrupt Status Port2,
	 * and store the values in a dummy array 
	 */
	ret = i2c_smbus_read_i2c_block_data(client, REG_INTR_STAT_PORT0,
		NPORTS, dummy);	
	if (ret < 0) {
		dev_err(&client->dev, "couldn't clear int status\n");
		goto err;
	}

	dev_info(&client->dev, "the interrupt state registers are cleared\n");

	/* 
	 * Initialise Interrupt Mask Port Register (19h) for each port
	 * Disable the activation of the INT lines. Each 1 in this 
	 * register masks (disables) the int from the corresponding GPIO   
         */
	memset(cygpio->irq_mask_cache, 0xff, sizeof(cygpio->irq_mask_cache));
	memset(cygpio->irq_mask, 0xff, sizeof(cygpio->irq_mask));

	/* Disable interrupts in all the gpio lines */
	for (i = 0; i < NPORTS; i++) {
		ret = i2c_smbus_write_byte_data(client, REG_PORT_SELECT, i);
		if (ret < 0) {
			dev_err(&client->dev, "can't select port %u\n", i);
			goto err;
		}

		ret = i2c_smbus_write_byte_data(client, REG_INTR_MASK, 
						cygpio->irq_mask[i]);
		if (ret < 0) {
			dev_err(&client->dev, 
				"can't write int mask on port %u\n", i);
			goto err;
		}
	}

	dev_info(&client->dev, "the interrupt mask port registers are set\n");

	/* add a nested irqchip to the gpiochip */
	ret =  gpiochip_irqchip_add_nested(chip,
					  &cy8c9520a_irq_chip,
					  0,
					  handle_simple_irq,
					  IRQ_TYPE_NONE);
	if (ret) {
		dev_err(&client->dev,
			"could not connect irqchip to gpiochip\n");
		return ret;
	}

	/* 
	 * Request interrupt on a GPIO pin of the external processor 
	 * this processor pin is connected to the INT pin of the cy8c9520a
	 */
	ret = devm_request_threaded_irq(&client->dev, client->irq, NULL, 
					cy8c9520a_irq_handler, 
					IRQF_ONESHOT | IRQF_TRIGGER_HIGH, 
					dev_name(&client->dev), cygpio);
	if (ret) {
		dev_err(&client->dev, "failed to request irq %d\n", cygpio->irq);
			return ret;
	}

	/*
         * set up a nested irq handler for a gpio_chip from a parent IRQ 
	 * you can now request interrupts from GPIO child drivers nested  
	 * to the cy8c9520a driver
	 */
	gpiochip_set_nested_irqchip(chip,
				   &cy8c9520a_irq_chip,
				   cygpio->irq);

	dev_info(&client->dev, "the interrupt setup is done\n");

	return 0;
err:
	mutex_destroy(&cygpio->irq_lock);
	return ret;
}

/* 
 * Initialize the cy8c9520a gpio controller (struct gpio_chip) 
 * and register it to the kernel
 */
static int cy8c9520a_gpio_init(struct cy8c9520a *cygpio)
{
	struct gpio_chip *gpiochip = &cygpio->gpio_chip;
	int err;

	gpiochip->label = cygpio->client->name;
	gpiochip->base = -1;
	gpiochip->ngpio = NGPIO;
	gpiochip->parent = &cygpio->client->dev;
	gpiochip->of_node = gpiochip->parent->of_node;
	gpiochip->can_sleep = true;
	gpiochip->direction_input = cy8c9520a_gpio_direction_input;
	gpiochip->direction_output = cy8c9520a_gpio_direction_output;
	gpiochip->get = cy8c9520a_gpio_get;
	gpiochip->set = cy8c9520a_gpio_set;
	gpiochip->owner = THIS_MODULE;

	/* register a gpio_chip */
	err = devm_gpiochip_add_data(gpiochip->parent, gpiochip, cygpio);
	if (err)
		return err;
	return 0;
}

static int cy8c9520a_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct cy8c9520a *cygpio;
	int ret = 0;
	int i; 
	unsigned int dev_id, tmp;
	static const char * const name[] = { "pwm0", "pwm1", "pwm2", "pwm3" };

	dev_info(&client->dev, "cy8c9520a_probe() function is called\n");

	if (!i2c_check_functionality(client->adapter,
					I2C_FUNC_SMBUS_I2C_BLOCK |
					I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "SMBUS Byte/Block unsupported\n");
		return -EIO;
	}

	/* allocate global private structure for a new device */
	cygpio = devm_kzalloc(&client->dev, sizeof(*cygpio), GFP_KERNEL);
	if (!cygpio) {
		dev_err(&client->dev, "failed to alloc memory\n");
		return -ENOMEM;
	}

	cygpio->client = client;

	mutex_init(&cygpio->lock);

	/* Whoami */
	dev_id = i2c_smbus_read_byte_data(client, REG_DEVID_STAT);
	if (dev_id < 0) {
		dev_err(&client->dev, "can't read device ID\n");
		ret = dev_id;
		goto err;
	}
	dev_info(&client->dev, "dev_id=0x%x\n", dev_id & 0xff);

	/* parse the DT to get the pwm-pin mapping */
	for (i = 0; i < NPWM; i++) {
		ret = device_property_read_u32(&client->dev, name[i], &tmp); 
		if (!ret)
			cygpio->pwm_number[i] = tmp;
		else
			goto err;
	};
	
	/* Initial setup for the cy8c9520a */
	ret = cy8c9520a_setup(cygpio);
	if (ret < 0) {
		goto err;
	}

	dev_info(&client->dev, "the initial setup for the cy8c9520a is done\n");

 	/* Initialize the cy8c9520a gpio controller */
	ret = cy8c9520a_gpio_init(cygpio);
	if (ret) {
		goto err;
	}

	dev_info(&client->dev, "the setup for the cy8c9520a gpio controller done\n");

	/* Interrupt setup for the cy8c9520a */
	ret = cy8c9520a_irq_setup(cygpio);
	if (ret) {
		goto err;
	}

	dev_info(&client->dev, "the interrupt setup for the cy8c9520a is done\n");

	/* Setup of the pwm_chip controller */
	cygpio->pwm_chip.dev = &client->dev;
	cygpio->pwm_chip.ops = &cy8c9520a_pwm_ops;
	cygpio->pwm_chip.base = PWM_BASE_ID;
	cygpio->pwm_chip.npwm = NPWM;

	ret = pwmchip_add(&cygpio->pwm_chip);
	if (ret) {
		dev_err(&client->dev, "pwmchip_add failed %d\n", ret);
		goto err;
	}

	dev_info(&client->dev, "the setup for the cy8c9520a pwm_chip controller is done\n");

	/* Setup of the pinctrl descriptor */
	cygpio->pinctrl_desc.name = "cy8c9520a-pinctrl";
	cygpio->pinctrl_desc.pctlops = &cygpio_pinctrl_ops;
	cygpio->pinctrl_desc.confops = &cygpio_pinconf_ops;
	cygpio->pinctrl_desc.npins = cygpio->gpio_chip.ngpio;
	
	cygpio->pinctrl_desc.pins = cy8c9520a_pins;
	cygpio->pinctrl_desc.owner = THIS_MODULE;

	cygpio->pctldev = devm_pinctrl_register(&client->dev, &cygpio->pinctrl_desc, cygpio);
	if (IS_ERR(cygpio->pctldev)) {
		ret = PTR_ERR(cygpio->pctldev);
		goto err;
	}

	dev_info(&client->dev, "the setup for the cy8c9520a pinctl descriptor is done\n");


	/* link the I2C device with the cygpio device */
	i2c_set_clientdata(client, cygpio);

err:
	mutex_destroy(&cygpio->lock);

	return ret;
}

static int cy8c9520a_remove(struct i2c_client *client)
{	
	struct cy8c9520a *cygpio = i2c_get_clientdata(client);
	dev_info(&client->dev, "cy8c9520a_remove() function is called\n");
	return pwmchip_remove(&cygpio->pwm_chip);
}

static const struct of_device_id my_of_ids[] = {
	{ .compatible = "cy8c9520a"},
	{},
};
MODULE_DEVICE_TABLE(of, my_of_ids);

static const struct i2c_device_id cy8c9520a_id[] = {
	{DRV_NAME, 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, cy8c9520a_id);

static struct i2c_driver cy8c9520a_driver = {
	.driver = {
		   .name = DRV_NAME,
		   .of_match_table = my_of_ids,
		   .owner = THIS_MODULE,
		   },
	.probe = cy8c9520a_probe,
	.remove = cy8c9520a_remove,
	.id_table = cy8c9520a_id,
};
module_i2c_driver(cy8c9520a_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Alberto Liberal <aliberal@arroweurope.com>");
MODULE_DESCRIPTION("This is a driver that controls the \
                   cy8c9520a I2C GPIO expander");

