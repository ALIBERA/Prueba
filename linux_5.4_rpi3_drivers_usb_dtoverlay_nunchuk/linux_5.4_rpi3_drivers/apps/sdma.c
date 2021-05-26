

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <string.h>

#define SDMA_BUF_SIZE  4096

int main(void)
{
	char *virtaddr;
	char phrase[128];
	int my_dev = open("/dev/sdma_test", O_RDWR);
	if (my_dev < 0) {
		perror("Fail to open device file: /dev/sdma_test.");
	} else {

		printf("Enter phrase :\n");
		scanf("%[^\n]%*c", phrase);
		virtaddr = (char *)mmap(0, SDMA_BUF_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, my_dev, 0);
		strcpy(virtaddr, phrase);
		ioctl(my_dev, NULL);
		close(my_dev);
	}
	
	return 0;
}

