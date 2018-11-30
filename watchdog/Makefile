EXTRA_CFLAGS += $(shell rtai-config --module-cflags)

obj-m += watchdogs_rt.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean