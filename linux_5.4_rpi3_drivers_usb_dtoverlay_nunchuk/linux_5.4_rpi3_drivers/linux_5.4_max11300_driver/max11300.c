

#include "max11300-base.h"

#include <linux/bitops.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/spi/spi.h>


/* function to write MAX11300 registers */
static int max11300_reg_write(struct max11300_state *st, u8 reg, u16 val)
{
	struct spi_device *spi = container_of(st->dev, struct spi_device, dev);
	
	struct spi_transfer t[] = {
		{
			.tx_buf = &st->tx_cmd,
			.len = 1,
		}, {
			.tx_buf = &st->tx_msg,
			.len = 2,
		},
	};

	/* to transmit via SPI the LSB bit of the command byte must be 0 */
	st->tx_cmd = (reg << 1);

	/* 
	 * In little endian CPUs the byte stored in the higher address of 
         * the "val" variable (MSB of the DAC) is stored in the lower address 
	 * of the "st->tx_msg" variable using cpu_to_be16()
	 */
    
	st->tx_msg = cpu_to_be16(val);

	return spi_sync_transfer(spi, t, ARRAY_SIZE(t));
}

/* function to read MAX11300 registers in SE mode */
static int max11300_reg_read(struct max11300_state *st, u8 reg, u16 *value)
{
	struct spi_device *spi = container_of(st->dev, struct spi_device, dev);
	int ret;
	
	struct spi_transfer t[] = {
		{
			.tx_buf = &st->tx_cmd,
			.len = 1,
		}, {
			.rx_buf = &st->rx_msg,
			.len = 2,
		},
	};

	dev_info(st->dev, "read SE channel\n");

	/* to receive via SPI the LSB bit of the command byte must be 1 */
	st->tx_cmd = ((reg << 1) | 1);

	ret = spi_sync_transfer(spi, t, ARRAY_SIZE(t));
	if (ret < 0)
		return ret;

	/* 
	 * In little endian CPUs the first byte (MSB of the ADC) received via 
         * SPI (in BE format) is stored in the lower address of "st->rx_msg" 
         * variable. This byte is copied to the higher address of the "value" 
	 * variable using be16_to_cpu(). The second byte received via SPI is
         * copied from the higher address of "st->rx_msg" to the lower address
         * of the "value" variable in little endian CPUs.
         * In big endian CPUs the addresses are not swapped.
	 */
	*value = be16_to_cpu(st->rx_msg); 

	return 0;
}

/* function to read MAX11300 registers in differential mode (2's complement) */
static int max11300_reg_read_differential(struct max11300_state *st, u8 reg, 
					 int *value)
{
	struct spi_device *spi = container_of(st->dev, struct spi_device, dev);
	int ret;
	
	struct spi_transfer t[] = {
		{
			.tx_buf = &st->tx_cmd,
			.len = 1,
		}, {
			.rx_buf = &st->rx_msg,
			.len = 2,
		},
	};

	dev_info(st->dev, "read differential channel\n");

	/* to receive LSB of command byte has to be 1 */
	st->tx_cmd = ((reg << 1) | 1);

	ret = spi_sync_transfer(spi, t, ARRAY_SIZE(t));
	if (ret < 0)
		return ret;
	
	/* 
	 * extend to an int 2's complement value the received SPI value in 2's 
	 * complement value, which is stored in the "st->rx_msg" variable 
	 */  
	*value = sign_extend32(be16_to_cpu(st->rx_msg), 11);  

	return 0;
}

/* 
 * Initialize the struct max11300_rw_ops with read and write 
 * callback functions to write/read via SPI from MAX11300 registers 
 */
static const struct max11300_rw_ops max11300_rw_ops = {
	.reg_write = max11300_reg_write,
	.reg_read = max11300_reg_read,
	.reg_read_differential = max11300_reg_read_differential,
};

static int max11300_spi_probe(struct spi_device *spi)
{
	const struct spi_device_id *id = spi_get_device_id(spi);

	return max11300_probe(&spi->dev, id->name, &max11300_rw_ops);
}

static int max11300_spi_remove(struct spi_device *spi)
{
	return max11300_remove(&spi->dev);
}

static const struct spi_device_id max11300_spi_ids[] = {
	{ .name = "max11300", },
	{}
};
MODULE_DEVICE_TABLE(spi, max11300_spi_ids);

static const struct of_device_id max11300_of_match[] = {
	{ .compatible = "maxim,max11300", },
	{},
};
MODULE_DEVICE_TABLE(of, max11300_of_match);

static struct spi_driver max11300_spi_driver = {
	.driver = {
		.name = "max11300",
		.of_match_table = of_match_ptr(max11300_of_match),
	},
	.probe = max11300_spi_probe,
	.remove = max11300_spi_remove,
	.id_table = max11300_spi_ids,
};
module_spi_driver(max11300_spi_driver);


MODULE_AUTHOR("Alberto Liberal <aliberal@arroweurope.com>");
MODULE_DESCRIPTION("Maxim max11300 multi-port converters");
MODULE_LICENSE("GPL v2");


