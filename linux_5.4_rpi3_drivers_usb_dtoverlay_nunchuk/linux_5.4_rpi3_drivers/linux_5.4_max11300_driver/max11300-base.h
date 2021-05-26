
#ifndef __DRIVERS_IIO_DAC_max11300_BASE_H__
#define __DRIVERS_IIO_DAC_max11300_BASE_H__

#include <linux/types.h>
#include <linux/cache.h>
#include <linux/mutex.h>
#include <linux/gpio/driver.h>

struct max11300_state;

/* masks for the Device Control (DCR) Register */
#define DCR_ADCCTL_CONTINUOUS_SWEEP (BIT(0) | BIT(1))
#define DCR_DACREF BIT(6)
#define BRST BIT(14) 
#define RESET BIT(15)

/* define register addresses */
#define DCR_ADDRESS 0x10
#define PORT_CFG_BASE_ADDRESS 0x20
#define PORT_ADC_DATA_BASE_ADDRESS 0x40
#define PORT_DAC_DATA_BASE_ADDRESS 0x60
#define DACPRSTDAT1_ADDRESS 0x16
#define GPO_DATA_15_TO_0_ADDRESS 0x0D
#define GPO_DATA_19_TO_16_ADDRESS 0x0E
#define GPI_DATA_15_TO_0_ADDRESS 0x0B
#define GPI_DATA_19_TO_16_ADDRESS 0x0C

/* 
 * declare the struct with pointers to the functions that will read and write 
 * via SPI the registers of the MAX11300 device 
 */
struct max11300_rw_ops {
	int (*reg_write)(struct max11300_state *st, u8 reg, u16 value);
	int (*reg_read)(struct max11300_state *st, u8 reg, u16 *value);
	int (*reg_read_differential)(struct max11300_state *st, u8 reg, int *value);
};

/* declare the global structure that will store the info of the device */
struct max11300_state {
	struct device *dev;
	const struct max11300_rw_ops *ops;
	struct gpio_chip gpiochip;
	struct mutex gpio_lock;	
	u8 num_ports;
	u8 num_gpios;
	u8 gpio_offset[20];
	u8 gpio_offset_mode[20];
	u8 port_modes[20];
	u8 adc_range[20];
	u8 dac_range[20];
	u8 adc_reference[20];
	u8 adc_samples[20];
	u8 adc_negative_port[20];
	u8 tx_cmd; 
	__be16 tx_msg;
	__be16 rx_msg; 
};

int max11300_probe(struct device *dev, const char *name, 
		  const struct max11300_rw_ops *ops);
int max11300_remove(struct device *dev);

#endif /* __DRIVERS_IIO_DAC_max11300_BASE_H__ */


