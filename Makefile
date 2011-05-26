BRCM_CHIP=6358
BRCM_BOARD_ID="96358VW2"

# TEY add for Firmware Lock
GETFLASHREVISION="658"

ifneq ($(findstring 6332,BRCM_BOARD_ID),)
FWLOCKTAG_HEAD="414c50484140$(PROFILE)6332"
else
FWLOCKTAG_HEAD="414c50484140$(PROFILE)$(BRCM_CHIP)"
endif
FIRMWARELOCKTAG="$(FWLOCKTAG_HEAD)$(GETFLASHREVISION)"

HOST_ARCH := $(shell uname -m)
ifneq ($(HOST_ARCH),x86_64)
	HOST_ARCH=i386
endif

all : rg100a.img

vmlinux.lz.deadcode : vmlinux.lz
	cp $< $@
	echo -n -e '\0336\0255\0300\0336' >> $@ # DEADCODE

%.img : vmlinux.lz cfe6358.bin rootfs.squashfs
	./tools/bcmImageBuilder --chip 6358 --board 96358VW2 --blocksize 128 \
			--output $@ --cfefile cfe6358.bin --rootfsfile rootfs.squashfs \
			--kernelfile vmlinux.lz
	./tools/bcmImageBuilder --chip 6358 --board 96358VW2 --blocksize 128 \
			--output $*.cfe --cfefile cfe6358.bin --rootfsfile rootfs.squashfs \
			--kernelfile vmlinux.lz --include-cfe
	./tools/createimg --boardid=96358VW2 --numbermac=11 --macaddr=02:10:18:01:00:01 \
	--tp=0 --psisize=24 --gponpw= --gponsn= --inputfile=$*.cfe --outputfile=$*.cfe.tmp
	./tools/addvtoken $*.cfe.tmp $*.cfe.img
	rm -f $*.cfe.tmp $*.cfe
#	./tools/createimg --boardid=96358VW2 --numbermac=11 --macaddr=02:10:18:01:00:01 \
#	--tp=0 --psisize=24 --gponpw= --gponsn= --inputfile=$@.cfe --outputfile=$@.cfe.img.tmp
#	./tools/addvtoken $@.cfe.img.tmp $@.cfe.img
#	echo $(FIRMWARELOCKTAG) >> $@

#LOADADDR = 0x80010000
#ENTRY = 0x80010000
#ENTRY = 0x8028e000
#START = 0xbfc00000

#rg100a : vmlinux.lz cfe6358.bin rootfs.squashfs
#	./bin/imagetag -i vmlinux.lz -f rootfs.squashfs \
#			-o $@ \
#			-b 96358VW2  -c 6358 \
#			-s $(START) -n 0x20000 \
#			-e $(ENTRY) -l $(LOADADDR) \
#			-k 0x20000

KERNEL_VERSION = 2.6.21.5
PWD = $(shell pwd)

rootfs.squashfs :
	rm -fr rootfs
	mkdir rootfs
	gzip -d < openwrt-brcm63xx-rootfs.cpio.gz | (cd rootfs && cpio -i)
	rm -f  rootfs/init
	rm -fr rootfs/etc/modules.d
	rm -fr rootfs/lib/modules
	rm -f  rootfs/usr/lib/opkg/lists/*
	chmod 1777 rootfs/tmp
	cd modules/lib/modules/$(KERNEL_VERSION) && \
	cat $(PWD)/config/kmod-broadcom.list $(PWD)/config/kmod-base.list $(PWD)/config/kmod-sched.list | \
	cpio -p -d $(PWD)/rootfs/lib/modules/$(KERNEL_VERSION)
	cd src/modules && cat $(PWD)/config/kmod-local.list | \
	cpio -p -d $(PWD)/rootfs/lib/modules/$(KERNEL_VERSION)
	cd fs.custom  && find | cpio -u -p -d ../rootfs
	mknod rootfs/dev/console c 5 1
	./tools/$(HOST_ARCH)/mksquashfs rootfs $@ -b 65536 -be -all-root
#	rm -fr rootfs/lib/firmware
#	rm -fr rootfs/etc/hotplug.d/atm

clean:
	rm -f rg100a.* vmlinux.lz.deadcode

distclean: clean
	rm -fr rootfs rootfs.squashfs
