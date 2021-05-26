#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <string.h>
#include <linux/gpio.h>
#include <sys/ioctl.h>

#define DEV_GPIO  "/dev/gpiochip3"

#define POLL_TIMEOUT -1 /* No timeout */

int main(int argc, char *argv[])
{
    int fd, fd_in;
    int ret;
    int flags;

    struct gpioevent_request req;
    struct gpioevent_data evdata;
    struct pollfd fdset;

    /* open gpio */
    fd = open(DEV_GPIO, O_RDWR);
    if (fd < 0) {
        printf("ERROR: open %s ret=%d\n", DEV_GPIO, fd);
        return -1;
    }

    /* Request GPIO P0 first line interrupt */
    req.lineoffset = 0;
    req.handleflags = GPIOHANDLE_REQUEST_INPUT;
    req.eventflags  = GPIOEVENT_REQUEST_BOTH_EDGES;
    strncpy(req.consumer_label, "gpio_irq", sizeof(req.consumer_label) - 1);

    /* requrest line event handle */
    ret = ioctl(fd, GPIO_GET_LINEEVENT_IOCTL, &req);
    if (ret) {
        printf("ERROR: ioctl get line event ret=%d\n", ret);
        return -1;
    }

    /* set event fd nonbloack read */
    fd_in = req.fd;
    flags = fcntl(fd_in, F_GETFL);
    flags |= O_NONBLOCK;
    ret = fcntl(fd_in, F_SETFL, flags);
    if (ret) {
        printf("ERROR: fcntl set nonblock read\n");
    }

    for (;;) {
        fdset.fd      = fd_in;
        fdset.events  = POLLIN;
        fdset.revents = 0;

        /* poll gpio line event */
        ret = poll(&fdset, 1, POLL_TIMEOUT);
        if (ret <= 0)
            continue;

        if (fdset.revents & POLLIN) {
            printf("irq received.\n");
            /* read event data */
            ret = read(fd_in, &evdata, sizeof(evdata));
            if (ret == sizeof(evdata))
                printf("id: %d, timestamp: %lld\n", evdata.id, evdata.timestamp);
        }
    }

    /* close gpio */
    close(fd);

    return 0;
}
