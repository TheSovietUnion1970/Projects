spi0-y := spi0_tx.o
obj-m += spi0.o

spi1-y := spi1_rx.o
obj-m += spi1.o

KDIR := /home/vinh/build_BBB_custom/linux-stable-rcn-ee
PWD := $(shell pwd)
CROSS_COMPILE := /home/vinh/build_BBB_custom/gcc-11.3.0-nolibc/arm-linux-gnueabi/bin/arm-linux-gnueabi-

# Add debug flags
ccflags-y += -O0 -g

all:
	$(MAKE) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean


