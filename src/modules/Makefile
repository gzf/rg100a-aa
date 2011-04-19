BCMPKG = DSL-2760U
KERNEL = $(BCMPKG)/kernel/linux
BRCM_BOARD = bcm963xx
CROSS = mips-linux

EXTRA_CFLAGS += -I$(M)/linux/include -I$(M)/linux/arch/mips/include/asm/mach-bcm63xx \
				-DCONFIG_BCM63XX_CPU_6358
EXTRA_CFLAGS += -I$(BCMPKG)/bcmdrivers/broadcom/include/$(BRCM_BOARD) \
        -I$(BCMPKG)/bcmdrivers/opensource/include/$(BRCM_BOARD) \
        -I$(BCMPKG)/shared/opensource/include/$(BRCM_BOARD)

obj-m = xt_connlimit.o \
		switch-core.o \
		switch-robo.o \
		ipt_IPID.o
#		btn_hotplug.o
#		bcm63xx_enet.o


xt_connlimit-objs := xt_connlimit_orig.o backport.o
btn_hotplug-objs := button.o hotplug.o

MDIR=$(shell pwd)
all:
	make -I$(MDIR) -C $(KERNEL) CC=$(CROSS)-gcc LD=$(CROSS)-ld M=$(MDIR) modules
clean:
	make -I$(MDIR) -C $(KERNEL) CC=$(CROSS)-gcc LD=$(CROSS)-ld M=$(MDIR) clean

