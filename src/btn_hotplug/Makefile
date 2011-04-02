BCMPKG = /opt/DSL-2760U
KERNEL = $(BCMPKG)/kernel/linux
BRCM_BOARD = bcm963xx

CROSS = mips-linux

EXTRA_CFLAGS += -I$(BCMPKG)/bcmdrivers/broadcom/include/$(BRCM_BOARD) \
		-I$(BCMPKG)/bcmdrivers/opensource/include/$(BRCM_BOARD) \
		-I$(BCMPKG)/shared/opensource/include/$(BRCM_BOARD)

obj-m := btn_hotplug.o

all:
	make -C $(KERNEL) CC=$(CROSS)-gcc LD=$(CROSS)-ld M=`pwd` modules

clean:
	make -C $(KERNEL) CC=$(CROSS)-gcc LD=$(CROSS)-ld M=`pwd` clean
