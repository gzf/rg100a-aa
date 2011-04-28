export BCMROOT = $(PWD)/DSL-2760U
export BRCM_BOARD = bcm963xx

KERNEL = $(BCMROOT)/kernel/linux
CROSS = mips-linux

all:
	make -C $(KERNEL) CC=$(CROSS)-gcc LD=$(CROSS)-ld M=$(PWD) modules
clean:
	make -C $(KERNEL) CC=$(CROSS)-gcc LD=$(CROSS)-ld M=$(PWD) clean

