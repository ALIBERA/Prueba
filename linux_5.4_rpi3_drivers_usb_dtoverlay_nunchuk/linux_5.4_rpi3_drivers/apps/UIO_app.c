

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <sys/mman.h>

#define BUFFER_LENGHT 128

#define GPIO_27			27
#define GPIO_22			22
#define GPIO_26			26

#define GPFSEL2_offset 	 0x08
#define GPSET0_offset    0x1C
#define GPCLR0_offset 	 0x28

/* to set and clear each individual LED */
#define GPIO_27_INDEX 	1 << (GPIO_27 % 32)
#define GPIO_22_INDEX 	1 << (GPIO_22 % 32)
#define GPIO_26_INDEX 	1 << (GPIO_26 % 32)

/* select the output function */
#define GPIO_27_FUNC	1 << ((GPIO_27 % 10) * 3)
#define GPIO_22_FUNC 	1 << ((GPIO_22 % 10) * 3)
#define GPIO_26_FUNC 	1 << ((GPIO_26 % 10) * 3)

#define FSEL_27_MASK 	0b111 << ((GPIO_27 % 10) * 3) /* red since bit 21 (FSEL27) */
#define FSEL_22_MASK 	0b111 << ((GPIO_22 % 10) * 3) /* green since bit 6 (FSEL22) */
#define FSEL_26_MASK 	0b111 << ((GPIO_26 % 10) * 3) /* blue since bit 18 (FSEL26) */

#define GPIO_SET_FUNCTION_LEDS (GPIO_27_FUNC | GPIO_22_FUNC | GPIO_26_FUNC)
#define GPIO_MASK_ALL_LEDS 	(FSEL_27_MASK | FSEL_22_MASK | FSEL_26_MASK)
#define GPIO_SET_ALL_LEDS (GPIO_27_INDEX  | GPIO_22_INDEX  | GPIO_26_INDEX)

#define UIO_SIZE "/sys/class/uio/uio0/maps/map0/size"

int main()
{
	int ret, devuio_fd;
	int mem_fd;
	unsigned int uio_size;
	void *temp;
	int GPFSEL_read, GPFSEL_write;
	void *demo_driver_map;
	//char *demo_driver_map;
	char sendstring[BUFFER_LENGHT];
	char *led_on = "on";
	char *led_off = "off";
	char *Exit = "exit";

	printf("Starting led example\n");

	if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
      		printf("can't open /dev/mem \n");
      		exit(-1);
        }
	printf("opened /dev/mem \n");
	
	devuio_fd = open("/dev/uio0", O_RDWR | O_SYNC);
	if (devuio_fd < 0){
		perror("Failed to open the device");
		exit(EXIT_FAILURE);
	}
	
	printf("opened /dev/ui0 \n");

	/* read the size that has to be mapped */
	FILE *size_fp = fopen(UIO_SIZE, "r");
	fscanf(size_fp, "0x%x", &uio_size);
	fclose(size_fp);
	printf("the value is %d\n", uio_size);

	/* do the mapping */
	demo_driver_map = mmap(0, uio_size, PROT_READ | PROT_WRITE, MAP_SHARED, devuio_fd, 0);
	if(demo_driver_map == MAP_FAILED) {
		perror("devuio mmap error");
		close(devuio_fd);
		exit(EXIT_FAILURE);
	}

	GPFSEL_read = *(int *)(demo_driver_map + GPFSEL2_offset);  
	
	GPFSEL_write = (GPFSEL_read & ~GPIO_MASK_ALL_LEDS) |
					  (GPIO_SET_FUNCTION_LEDS & GPIO_MASK_ALL_LEDS);
	
	*(int *)(demo_driver_map + GPFSEL2_offset) = GPFSEL_write; /* set dir leds to output */
	*(int *)(demo_driver_map + GPCLR0_offset)  = GPIO_SET_ALL_LEDS;	/* Clear all the leds, output is low */
	
	/* control the LED */
	do {
		printf("Enter led value: on, off, or exit :\n");
		scanf("%[^\n]%*c", sendstring);
		if(strncmp(led_on, sendstring, 3) == 0)
		{
			temp = demo_driver_map + GPSET0_offset;
			*(int *)temp = GPIO_27_INDEX;
		}
		else if(strncmp(led_off, sendstring, 2) == 0)
		{
			temp = demo_driver_map + GPCLR0_offset;
			*(int *)temp = GPIO_27_INDEX;
		}
		else if(strncmp(Exit, sendstring, 4) == 0)
		printf("Exit application\n");

		else {
			printf("Bad value\n");
			return -EINVAL;
		}

	} while(strncmp(sendstring, "exit", strlen(sendstring)));

	ret = munmap(demo_driver_map, uio_size);
	if(ret < 0) {
		perror("devuio munmap");
		close(devuio_fd);
		exit(EXIT_FAILURE);
	}

	close(devuio_fd);
	printf("Application termined\n");
	exit(EXIT_SUCCESS);
}

