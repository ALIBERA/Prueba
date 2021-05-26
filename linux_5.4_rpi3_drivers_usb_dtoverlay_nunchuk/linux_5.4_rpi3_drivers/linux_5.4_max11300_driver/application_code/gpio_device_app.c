#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <linux/gpio.h>
#include <sys/ioctl.h>

/* configure port19 as an output and flash an LED */

#define DEVICE_GPIO "/dev/gpiochip3"

int main(int argc, char *argv[])
{
    int fd;
    int ret;
    int flash = 10;

    struct gpiohandle_data data;
    struct gpiohandle_request req;

    /* open gpio device */
    fd = open(DEVICE_GPIO, 3);
    if (fd < 0) {
        fprintf(stderr, "Failed to open %s\n", DEVICE_GPIO);
        return -1;
    }

    /* request GPIO line 3 as an output (red LED) */
    req.lineoffsets[0] = 3;
    req.lines = 1;
    req.flags = GPIOHANDLE_REQUEST_OUTPUT;
    strcpy(req.consumer_label, "led_gpio_port19");

    ret = ioctl(fd, GPIO_GET_LINEHANDLE_IOCTL, &req);
    if (ret < 0) {
        printf("ERROR get line handle IOCTL (%d)\n", ret);
        if (close(fd) == -1)
        	perror("Failed to close GPIO char device");
        return ret;
    }

    /* start the led_red with off state */
    data.values[0] = 1;

    for (int i=0; i < flash; i++) {
        /* toggle LED */
        data.values[0] = !data.values[0];
        ret = ioctl(req.fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data);
        if (ret < 0) {
        	fprintf(stderr, "Failed to issue %s (%d)\n", "GPIOHANDLE_SET_LINE_VALUES_IOCTL", ret);
        	if (close(req.fd) == -1)
        		perror("Failed to close GPIO line");
        	if (close(fd) == -1)
        		perror("Failed to close GPIO char device");
        	return ret;

        }
        sleep(1);
    }

    /* close gpio line */
    ret = close(req.fd);
    if (ret == -1)
    	perror("Failed to close GPIO line");

    /* close gpio device */
    ret = close(fd);
        if (ret == -1)
        	perror("Failed to close GPIO char device");

    return ret;
}
