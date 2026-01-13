can0-y := can.o main.o
# can0-y := BBB_request.o
obj-m += can0.o

KDIR := /home/vinh/build_BBB_custom/linux-stable-rcn-ee
PWD := $(shell pwd)
CROSS_COMPILE := /home/vinh/build_BBB_custom/gcc-11.3.0-nolibc/arm-linux-gnueabi/bin/arm-linux-gnueabi-

# Add debug flags
ccflags-y += -O0 -g

all:
	$(MAKE) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

# sudo cp *.c /media/vinh/DATA/BBB/Linux_embedded/03_Learning/04_LINUX_Common/04_Modules/BBB_KGDB/10_USB/04_USB/05_BULK_HMSC/V3/
# sudo cp *.h /media/vinh/DATA/BBB/Linux_embedded/03_Learning/04_LINUX_Common/04_Modules/BBB_KGDB/10_USB/04_USB/05_BULK_HMSC/V3/

# sudo cp /media/vinh/DATA/BBB/Projects/BeagleBoneBlack/09_USB/03_BULK_HMSC/*.c ./
# sudo cp /media/vinh/DATA/BBB/Projects/BeagleBoneBlack/09_USB/03_BULK_HMSC/*.h ./
