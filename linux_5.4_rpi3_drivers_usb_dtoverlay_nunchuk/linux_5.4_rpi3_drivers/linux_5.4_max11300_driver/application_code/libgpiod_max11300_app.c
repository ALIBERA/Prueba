#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <gpiod.h>

int main(int argc, char *argv[])
{
	struct gpiod_chip *output_chip;
	struct gpiod_line *output_line;
	int line_value = 1;
	int flash = 10;
	int ret;

	/* open /dev/gpiochip3 */
	output_chip = gpiod_chip_open_by_number(3);
	if (!output_chip)
		return -1;

	/* get line 3 (port19) of the gpiochip3 device */
	output_line = gpiod_chip_get_line(output_chip, 3);
	if(!output_line) {
		gpiod_chip_close(output_chip);
		return -1;
	}

	/* config port19 (GPO) as output and set ouput to high level */
	if (gpiod_line_request_output(output_line, "Port19_GPO",
				  GPIOD_LINE_ACTIVE_STATE_HIGH) == -1) {
		gpiod_line_release(output_line);
		gpiod_chip_close(output_chip);
		return -1;
	}

	/* toggle 10 times the port19 (GPO) of the max11300 device */
	for (int i=0; i < flash; i++) {
		line_value = !line_value;
		ret = gpiod_line_set_value(output_line, line_value);
		if (ret == -1) {
			ret = -errno;
			gpiod_line_release(output_line);
			gpiod_chip_close(output_chip);
			return ret;
		}
		sleep(1);
	}

	gpiod_line_release(output_line);
	gpiod_chip_close(output_chip);

	return 0;
}
