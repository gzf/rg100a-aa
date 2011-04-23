export BCMROOT = $(PWD)/DSL-2760U
export BRCM_BOARD = bcm963xx
export EXTRA_CFLAGS += -I$(PWD)/linux/include -I$(PWD)/linux/arch/mips/include/asm/mach-bcm63xx \
					   -DCONFIG_BCM63XX_CPU_6358

KERNEL = $(BCMROOT)/kernel/linux
CROSS = mips-linux

all:
	make -C $(KERNEL) CC=$(CROSS)-gcc LD=$(CROSS)-ld M=$(PWD) modules
clean:
	make -C $(KERNEL) CC=$(CROSS)-gcc LD=$(CROSS)-ld M=$(PWD) clean

