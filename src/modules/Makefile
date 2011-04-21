BCMPKG = DSL-2760U

KERNEL = $(BCMPKG)/kernel/linux
CROSS = mips-linux

all:
	make -C $(KERNEL) CC=$(CROSS)-gcc LD=$(CROSS)-ld M=$$PWD modules
clean:
	make -C $(KERNEL) CC=$(CROSS)-gcc LD=$(CROSS)-ld M=$$PWD clean

