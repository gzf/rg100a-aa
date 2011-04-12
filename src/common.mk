BCMPKG = /opt/DSL-2760U
KERNEL = $(BCMPKG)/kernel/linux
BRCM_BOARD = bcm963xx

CROSS = mips-linux

MDIR=$(shell pwd)

all:
	make -I$(MDIR) -C $(KERNEL) CC=$(CROSS)-gcc LD=$(CROSS)-ld M=$(MDIR) modules

clean:
	make -I$(MDIR) -C $(KERNEL) CC=$(CROSS)-gcc LD=$(CROSS)-ld M=$(MDIR) clean
