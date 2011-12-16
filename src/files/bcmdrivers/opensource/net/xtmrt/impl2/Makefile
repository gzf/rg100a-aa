# File: bcmdrivers/opensource/net/xtmrt/Makefile
#
# Makefile for the BCM63xx bcmxtmrt driver.
#

obj-m := bcmxtmrt.o

EXTRA_CFLAGS += -I$(INC_BRCMDRIVER_PUB_PATH)/$(BRCM_BOARD) -I$(INC_BRCMSHARED_PUB_PATH)/$(BRCM_BOARD) -I$(INC_BRCMBOARDPARMS_PATH)/$(BRCM_BOARD)

clean:
	rm -f core *.o *.a *.s

