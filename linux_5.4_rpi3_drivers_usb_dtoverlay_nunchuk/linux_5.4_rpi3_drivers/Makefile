
obj-m := helloworld_rpi3.o helloworld_rpi3_with_parameters.o helloworld_rpi3_char_driver.o helloworld_rpi3_class_driver.o misc_rpi3_driver.o hellokeys_rpi3.o ledRGB_rpi3_platform.o ledRGB_rpi3_class_platform.o led_rpi3_UIO_platform.o io_rpi3_expander.o ltc3206_rpi3_led_class.o int_rpi3_key.o int_rpi3_key_wait.o keyled_rpi3_class.o linkedlist_rpi3_platform.o sdma_rpi3_m2m.o i2c_rpi3_accel.o adxl345_rpi3.o ltc2607_rpi3_dual_device.o ltc2422_rpi3_dual.o ltc2422_rpi3_trigger.o adxl345_rpi3_iio.o helloworld_rpi3_with_timing.o


KERNEL_DIR ?= $(HOME)/linux

all:
	make -C $(KERNEL_DIR) \
		ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- \
		M=$(PWD) modules

clean:
	make -C $(KERNEL_DIR) \
		ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- \
		M=$(PWD) clean

deploy:
	scp *.ko root@10.0.0.10:/home/


