CC = arm-linux-gnueabihf-gcc 

all: UIO_app

app : UIO_app.c
	$(CC) -o $@ $^
clean :
	rm UIO_app
deploy : UIO_app
	scp $^ root@10.0.0.10:/home/
