#BRCM_CHIP=6358
#BRCM_BOARD_ID="96358VW2"

include Build.defs

VPATH = $(BCMFW)/targets/DSL-2760U
CMPLZMA = $(BCMFW)/hostTools/cmplzma
CFE = $(BCMFW)/targets/cfe/cfe6358.bin

all : rg100a.img 

cfe : rg100a.cfe.img

oldcfe : rg100a-oldcfe.img

%.vmlinux.lz : vmlinux vmlinux.bin
	$(CMPLZMA) -k -2 $^ $@

%-oldcfe.img : CMPLZMA = ./misc/cmplzma-oldcfe

%.cfe.img : %.vmlinux.lz rootfs.squashfs
	$(BCMFW)/hostTools/bcmImageBuilder --chip 6358 --board 96358VW2 --blocksize 128 \
			--output $*.cfe.1 --cfefile $(CFE) --rootfsfile rootfs.squashfs \
			--kernelfile $*.vmlinux.lz --include-cfe
	$(BCMFW)/hostTools/createimg --boardid=96358VW2 --numbermac=11 --macaddr=02:10:18:01:00:01 \
	--tp=0 --psisize=24 --gponpw= --gponsn= --inputfile=$*.cfe.1 --outputfile=$*.cfe.2
	$(BCMFW)/hostTools/addvtoken $*.cfe.2 $@
	rm -f $*.cfe.1 $*.cfe.2

%.img : %.vmlinux.lz rootfs.squashfs
	$(BCMFW)/hostTools/bcmImageBuilder --chip 6358 --board 96358VW2 --blocksize 128 \
			--output $@ --cfefile $(CFE) --rootfsfile rootfs.squashfs \
			--kernelfile $*.vmlinux.lz

.INTERMEDIATE: %.vmlinux.lz

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

#vmlinux.lz.deadcode : vmlinux.lz
#	cp $< $@
#	echo -n -e '\0336\0255\0300\0336' >> $@ # DEADCODE

rootfs.squashfs :
	$(BCMFW)/hostTools/fakeroot/fakeroot ./buildfs $@

clean:
	rm -f *.img

distclean: clean
	rm -fr rootfs rootfs.squashfs
