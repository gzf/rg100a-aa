include ../Build.defs

export BCMROOT = $(PWD)/../$(BCMFW)
export BRCM_BOARD = bcm963xx

KERNEL = $(BCMROOT)/kernel/linux
CROSSTOOL = $(BCMROOT)/toolchains/uclibc-crosstools-gcc-4.2.3-3
CROSS = mips-linux

all:
	make -C $(KERNEL) CC=$(CROSSTOOL)/usr/bin/$(CROSS)-gcc \
		LD=$(CROSSTOOL)/usr/bin/$(CROSS)-ld M=$(PWD) modules
clean:
	make -C $(KERNEL) CC=$(CROSSTOOL)/usr/bin/$(CROSS)-gcc \
		LD=$(CROSSTOOL)/usr/bin/$(CROSS)-ld M=$(PWD) clean
