
obj-m := nunchuk_accel.o nunchuk_consumer.o nunchuk.o


KERNEL_DIR ?= $(HOME)/linux_rpi3/linux

all:
	make -C $(KERNEL_DIR) \
		ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- \
		M=$(PWD) modules

clean:
	make -C $(KERNEL_DIR) \
		ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- \
		M=$(PWD) clean

deploy:
	scp *.ko root@10.0.0.10:/home/pi


