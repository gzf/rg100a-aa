#****************************************************************************
#
#  Copyright (c) 2001, 2002, 2003, 2004  Broadcom Corporation
#  All Rights Reserved
#  No portions of this material may be reproduced in any form without the
#  written permission of:
#          Broadcom Corporation
#          16251 Laguna Canyon Road
#          Irvine, California 92618
#  All information contained in this document is Broadcom Corporation
#  company private, proprietary, and trade secret.
#
#****************************************************************************

# Top-level Makefile for all commengine xDSL platforms


###########################################
#
# This is the most important target: make all
# This is the first target in the Makefile, so it is also the default target.
# All the other targets are later in this file.
#
############################################

all: make_version_check kernel_version_check all_postcheck1

#all_postcheck1: sanity_check \
#     create_install kernelbuild modbuild kernelbuildlite \
#     userspace hosttools buildimage

############################################################################
#
# A lot of the stuff in the original Makefile has been moved over
# to make.common.
#
############################################################################
BUILD_DIR = $(shell pwd)
include $(BUILD_DIR)/make.common

############################################################################
#
# Make info for voice has been moved over to make.voice
#
############################################################################
ifneq ($(strip $(BUILD_VODSL)),)
include $(BUILD_DIR)/make.voice
endif


all_postcheck1: sanity_check \
     kernelbuild modbuild $(PROFILE_DIR)/vmlinux.bin

$(PROFILE_DIR)/vmlinux.bin : $(KERNEL_DIR)/vmlinux 
	cp $< $(PROFILE_DIR)
	$(STRIP) --remove-section=.note --remove-section=.comment $(PROFILE_DIR)/vmlinux
	$(OBJCOPY) -O binary $(PROFILE_DIR)/vmlinux $@

############################################################################
#
# dsl, kernel defines
#
############################################################################

ifeq ($(strip $(BRCM_KERNEL_DEBUG)),y) 
KERNEL_DEBUG=1
endif

ifeq ($(strip $(BRCM_KERNEL_OPROFILE)),y) 
	export CONFIG_DEBUG_INFO=1
	export CONFIG_FRAME_POINTER=1
endif

#Set up ADSL standard
export ADSL=$(BRCM_ADSL_STANDARD)

#Set up ADSL_PHY_MODE  {file | obj}
export ADSL_PHY_MODE=file

#Set up ADSL_SELF_TEST
export ADSL_SELF_TEST=$(BRCM_ADSL_SELF_TEST)

#Set up ADSL_PLN_TEST
export ADSL_PLN_TEST=$(BUILD_TR69_XBRCM)

#WL command
ifneq ($(strip $(WL)),)
build_nop:=$(shell cd $(BUILD_DIR)/userspace/private/apps/wlan; make clean; cd $(BUILD_DIR))
export WL
endif

ifneq ($(strip $(BRCM_DRIVER_WIRELESS_USBAP)),)
    WLBUS ?= "usbpci"
endif
#default WLBUS for wlan pci driver
WLBUS ?="pci"
export WLBUS


############################################################################
#
# When there is a directory name with the same name as a Make target,
# make gets confused.  PHONY tells Make to ignore the directory when
# trying to make these targets.
#
############################################################################
.PHONY: userspace unittests data-model hostTools



############################################################################
#
# Other Targets. The default target is "all", defined at the top of the file.
#
############################################################################

userspace: sanity_check create_install data-model replace_protocol
	$(MAKE) -C userspace


#
# libcreduction analyzes the user applications and libraries and eliminates
# unused functions from the libraries, thus reducing image size.
#
ifneq ($(strip $(BUILD_LIBCREDUCTION)),)
libcreduction:
	$(MAKE) -C hostTools/libcreduction install
else
libcreduction:
	@echo "skipping libcreduction (not configured)"
endif


menuconfig:
	@cd $(INC_KERNEL_BASE); \
	if [ ! -e linux/CREDITS ]; then \
	  echo Untarring original Linux kernel source...; \
	  (tar xkfj $(ORIGINAL_KERNEL) 2> /dev/null || true); \
	fi
	$(MAKE) -C $(HOSTTOOLS_DIR)/scripts/lxdialog HOSTCC=gcc
	$(CONFIG_SHELL) $(HOSTTOOLS_DIR)/scripts/Menuconfig $(TARGETS_DIR)/config.in


#
# the userspace apps and libs make their own directories before
# they install, so they don't depend on this target to make the
# directory for them anymore.
#
create_install:
		mkdir -p $(PROFILE_DIR)/fs.install/etc
		mkdir -p $(INSTALL_DIR)/bin
		mkdir -p $(INSTALL_DIR)/lib

replace_protocol:
ifneq ($(strip $(BUILD_2760UBZ)),)
#	sed s/TCP or UDP/TCP/UDP/g $(BUILD_DIR)/userspace/private/libs/mdm/validstrings.c > $(BUILD_DIR)/userspace/private/libs/mdm/validstrings1.c
#	mv -f $(BUILD_DIR)/userspace/private/libs/mdm/validstrings1.c $(BUILD_DIR)/userspace/private/libs/mdm/validstrings.c
#	sed s/TCP or UDP/TCP/UDP/g $(BUILD_DIR)/userspace/public/include/mdm_validstrings.h > $(BUILD_DIR)/userspace/public/include/mdm_validstrings1.h
#	mv -f $(BUILD_DIR)/userspace/public/include/mdm_validstrings1.h $(BUILD_DIR)/userspace/public/include/mdm_validstrings.h
endif

$(KERNEL_DIR)/vmlinux:
	$(GENDEFCONFIG_CMD) $(PROFILE_PATH) ${MAKEFLAGS}
	cd $(KERNEL_DIR); \
	cp -f $(KERNEL_DIR)/arch/mips/defconfig $(KERNEL_DIR)/.config; \
	$(MAKE) oldconfig; $(MAKE); $(MAKE) modules_install

kernelbuild:
ifeq ($(wildcard $(KERNEL_DIR)/vmlinux),)
	echo starting kernel build at $(INC_KERNEL_BASE)
	@cd $(INC_KERNEL_BASE); \
	if [ ! -e linux/CREDITS ]; then \
	  echo Untarring original Linux kernel source...; \
	  (tar xkfj $(ORIGINAL_KERNEL) 2> /dev/null || true); \
	fi
	$(GENDEFCONFIG_CMD) $(PROFILE_PATH) ${MAKEFLAGS}
	cd $(KERNEL_DIR); \
	cp -f $(KERNEL_DIR)/arch/mips/defconfig $(KERNEL_DIR)/.config; \
	$(MAKE) oldconfig; $(MAKE)
else
	cd $(KERNEL_DIR); $(MAKE)
endif

kernel: sanity_check create_install kernelbuild hosttools buildimage

ifneq ($(strip $(BRCM_CHIP)),6348)
ifneq ($(strip $(BRCM_CHIP)),6338)
ifeq ($(strip $(BRCM_DRIVER_BCMDSP)),m)
kernelbuildlite:
	@echo "******************** Kernel Build Lite ********************"; \
	$(BRCMDRIVERS_DIR)/broadcom/char/dspapp/impl1/getDspModSizes.sh  $(BRCMDRIVERS_DIR)/broadcom/char/dspapp/impl1/dspdd.ko $(SHARED_DIR) $(CROSS_COMPILE) $(KERNEL_DEBUG);
	cd $(KERNEL_DIR); $(MAKE)
else
kernelbuildlite:
endif
else
kernelbuildlite:
endif
else
kernelbuildlite:
endif

ifeq ($(strip $(VOXXXLOAD)),1)
modbuild: touch_voice_files
	cd $(KERNEL_DIR); $(MAKE) modules && $(MAKE) modules_install
else
modbuild:
	cd $(KERNEL_DIR); $(MAKE) modules && $(MAKE) modules_install
endif

adslmodbuild:
	cd $(KERNEL_DIR); $(MAKE) M=$(INC_ADSLDRV_PATH) modules 
adslmodbuildclean:
	cd $(KERNEL_DIR); $(MAKE) M=$(INC_ADSLDRV_PATH) clean

spumodbuild:
	cd $(KERNEL_DIR); $(MAKE) M=$(INC_SPUDRV_PATH) modules
spumodbuildclean:
	cd $(KERNEL_DIR); $(MAKE) M=$(INC_SPUDRV_PATH) clean

pwrmngtmodbuild:
	cd $(KERNEL_DIR); $(MAKE) M=$(INC_PWRMNGTDRV_PATH) modules
pwrmngtmodclean:
	cd $(KERNEL_DIR); $(MAKE) M=$(INC_PWRMNGTDRV_PATH) clean

modules: sanity_check create_install modbuild kernelbuildlite hosttools buildimage

adslmodule: adslmodbuild
adslmoduleclean: adslmodbuildclean

spumodule: spumodbuild
spumoduleclean: spumodbuildclean

pwrmngtmodule: pwrmngtmodbuild
pwrmngtmoduleclean: pwrmngtmodclean

data-model:
ifneq ($(strip $(BUILD_2760UBZ)),)
ifneq ($(strip $(BUILD_2760U)),)
	sed '/AdminUserName/ s/"admin"/"admin"/' $(BUILD_DIR)/data-model/cms-data-model.xml > $(BUILD_DIR)/data-model/cms-data-model_1.xml
	mv -f $(BUILD_DIR)/data-model/cms-data-model_1.xml $(BUILD_DIR)/data-model/cms-data-model.xml	
else
	sed '/AdminUserName/ s/"admin"/"Admin"/' $(BUILD_DIR)/data-model/cms-data-model.xml > $(BUILD_DIR)/data-model/cms-data-model_1.xml
	mv -f $(BUILD_DIR)/data-model/cms-data-model_1.xml $(BUILD_DIR)/data-model/cms-data-model.xml	
endif	
	sed '/Manufacturer/ s/"Broadcom"/"DLink"/' $(BUILD_DIR)/data-model/cms-data-model.xml > $(BUILD_DIR)/data-model/cms-data-model_1.xml
	mv -f $(BUILD_DIR)/data-model/cms-data-model_1.xml $(BUILD_DIR)/data-model/cms-data-model.xml
	sed '/"Description"/ s/"ReadOnly" maxLength/"ReadOnly" defaultValue="DLINK2760" maxLength/' $(BUILD_DIR)/data-model/cms-data-model.xml > $(BUILD_DIR)/data-model/cms-data-model_1.xml
	mv -f $(BUILD_DIR)/data-model/cms-data-model_1.xml $(BUILD_DIR)/data-model/cms-data-model.xml
	sed '/"HardwareVersion"/ s/"ReadOnly" maxLength/"ReadOnly" defaultValue="96358VW2" maxLength/' $(BUILD_DIR)/data-model/cms-data-model.xml > $(BUILD_DIR)/data-model/cms-data-model_1.xml
	mv -f $(BUILD_DIR)/data-model/cms-data-model_1.xml $(BUILD_DIR)/data-model/cms-data-model.xml
	sed '/DeviceOperationMode/ s/"true"/"false"/' $(BUILD_DIR)/data-model/cms-data-model.xml > $(BUILD_DIR)/data-model/cms-data-model_1.xml
	mv -f $(BUILD_DIR)/data-model/cms-data-model_1.xml $(BUILD_DIR)/data-model/cms-data-model.xml
	sed '/"KickURL"/ s/"ReadOnly" maxLength/"ReadOnly" defaultValue="http:\/\/www.testacs.com\/acs" maxLength/' $(BUILD_DIR)/data-model/cms-data-model.xml > $(BUILD_DIR)/data-model/cms-data-model_1.xml
	mv -f $(BUILD_DIR)/data-model/cms-data-model_1.xml $(BUILD_DIR)/data-model/cms-data-model.xml
	sed '/"ConnectionTrigger"/ s/"ReadWrite" validValuesArray/"ReadWrite" defaultValue="AlwaysOn" validValuesArray/' $(BUILD_DIR)/data-model/cms-data-model.xml > $(BUILD_DIR)/data-model/cms-data-model_1.xml
	mv -f $(BUILD_DIR)/data-model/cms-data-model_1.xml $(BUILD_DIR)/data-model/cms-data-model.xml	
	sed '/"TotalAssociations"/ s/supportLevel="ReadOnly" hideParameterFromAcs="true" \//supportLevel="ReadOnly" \//' $(BUILD_DIR)/data-model/cms-data-model.xml > $(BUILD_DIR)/data-model/cms-data-model_1.xml
	mv -f $(BUILD_DIR)/data-model/cms-data-model_1.xml $(BUILD_DIR)/data-model/cms-data-model.xml
	sed '/"WlWpa"/ s/defaultValue="tkip" maxLength="16" \//defaultValue="aes" maxLength="16" \//' $(BUILD_DIR)/data-model/cms-data-model.xml > $(BUILD_DIR)/data-model/cms-data-model_1.xml
	mv -f $(BUILD_DIR)/data-model/cms-data-model_1.xml $(BUILD_DIR)/data-model/cms-data-model.xml
else
	sed '/AdminUserName/ s/"Admin"/"admin"/' $(BUILD_DIR)/data-model/cms-data-model.xml > $(BUILD_DIR)/data-model/cms-data-model_1.xml
	mv -f $(BUILD_DIR)/data-model/cms-data-model_1.xml $(BUILD_DIR)/data-model/cms-data-model.xml	
	sed '/Manufacturer/ s/"DLink"/"Broadcom"/' $(BUILD_DIR)/data-model/cms-data-model.xml > $(BUILD_DIR)/data-model/cms-data-model_1.xml	
	mv -f $(BUILD_DIR)/data-model/cms-data-model_1.xml $(BUILD_DIR)/data-model/cms-data-model.xml
	sed '/"Description"/ s/"ReadOnly" defaultValue="DLINK2760" maxLength/"ReadOnly" maxLength/' $(BUILD_DIR)/data-model/cms-data-model.xml > $(BUILD_DIR)/data-model/cms-data-model_1.xml
	mv -f $(BUILD_DIR)/data-model/cms-data-model_1.xml $(BUILD_DIR)/data-model/cms-data-model.xml
	sed '/"HardwareVersion"/ s/"ReadOnly" defaultValue="96358VW2" maxLength/"ReadOnly" maxLength/' $(BUILD_DIR)/data-model/cms-data-model.xml > $(BUILD_DIR)/data-model/cms-data-model_1.xml
	mv -f $(BUILD_DIR)/data-model/cms-data-model_1.xml $(BUILD_DIR)/data-model/cms-data-model.xml
	sed '/DeviceOperationMode/ s/"false"/"true"/' $(BUILD_DIR)/data-model/cms-data-model.xml > $(BUILD_DIR)/data-model/cms-data-model_1.xml	
	mv -f $(BUILD_DIR)/data-model/cms-data-model_1.xml $(BUILD_DIR)/data-model/cms-data-model.xml
	sed '/"KickURL"/ s/"ReadOnly" defaultValue="http:\/\/www.testacs.com\/acs" maxLength/"ReadOnly" maxLength/' $(BUILD_DIR)/data-model/cms-data-model.xml > $(BUILD_DIR)/data-model/cms-data-model_1.xml
	mv -f $(BUILD_DIR)/data-model/cms-data-model_1.xml $(BUILD_DIR)/data-model/cms-data-model.xml
	sed '/"ConnectionTrigger"/ s/"ReadWrite" defaultValue="AlwaysOn" validValuesArray/"ReadWrite" validValuesArray/' $(BUILD_DIR)/data-model/cms-data-model.xml > $(BUILD_DIR)/data-model/cms-data-model_1.xml
	mv -f $(BUILD_DIR)/data-model/cms-data-model_1.xml $(BUILD_DIR)/data-model/cms-data-model.xml	
	sed '/"TotalAssociations"/ s/supportLevel="ReadOnly" \//supportLevel="ReadOnly" hideParameterFromAcs="true" \//' $(BUILD_DIR)/data-model/cms-data-model.xml > $(BUILD_DIR)/data-model/cms-data-model_1.xml
	mv -f $(BUILD_DIR)/data-model/cms-data-model_1.xml $(BUILD_DIR)/data-model/cms-data-model.xml
	sed '/"WlWpa"/ s/defaultValue="aes" maxLength="16" \//defaultValue="tkip" maxLength="16" \//' $(BUILD_DIR)/data-model/cms-data-model.xml > $(BUILD_DIR)/data-model/cms-data-model_1.xml
	mv -f $(BUILD_DIR)/data-model/cms-data-model_1.xml $(BUILD_DIR)/data-model/cms-data-model.xml
endif
	$(MAKE) -C data-model

unittests:
	$(MAKE) -C unittests

unittests_run:
	$(MAKE) -C unittests unittests_run

doxygen_build:
	$(MAKE) -C hostTools build_doxygen

doxygen_docs: doxygen_build
	rm -rf $(BUILD_DIR)/docs/doxygen;
	mkdir $(BUILD_DIR)/docs/doxygen;
	cd hostTools/doxygen/bin; ./doxygen

doxygen_clean:
	$(MAKE) -C hostTools clean_doxygen


# touch_voice_files doesn't clean up voice, just enables incremental build of voice code
touch_voice_files:
	find bcmdrivers/broadcom/char/endpoint/ \( -name '*.o' -o -name '*.a' -o -name '*.lib' -o -name '*.ko' -o -name '*.cmd' -o -name '.*.cmd' -o -name '*.c' -o -name '*.mod' \) -print -exec rm -f "{}" ";"
	rm -rf kernel/linux/.tmp_versions/endpointdd.mod
	rm -rf kernel/linux/arch/mips/defconfig
	rm -rf kernel/linux/include/config/bcm/endpoint/
	find kernel/linux/lib/ -name '*.o' -print -exec rm -f "{}" ";"
	find kernel/linux/lib/ -name '*.lib' -print -exec rm -f "{}" ";"



############################################################################
#
# Build user applications depending on if they are
# specified in the profile.  Most of these BUILD_ checks should eventually get
# moved down to the userspace directory.
#
############################################################################

ifneq ($(strip $(BUILD_VCONFIG)),)
export BUILD_VCONFIG=y
endif


ifneq ($(strip $(BUILD_GDBSERVER)),)
gdbserver:
	install -m 755 $(TOOLCHAIN)/mips-linux-uclibc/target-apps/usr/bin/gdbserver $(INSTALL_DIR)/bin
	$(STRIP) $(INSTALL_DIR)/bin/gdbserver
else
gdbserver:
endif

ifneq ($(strip $(BUILD_ETHWAN)),)
export BUILD_ETHWAN=y
endif



ifneq ($(strip $(BUILD_IPSEC_TOOLS)),)
ipsec-tools:
	cd $(OPENSOURCE_DIR);   (tar xkfj ipsec-tools.tar.bz2 2> /dev/null || true)
	$(MAKE) -C $(OPENSOURCE_DIR)/ipsec-tools $(BUILD_IPSEC_TOOLS)
else
ipsec-tools:
endif


ifneq ($(strip $(BUILD_SIPROXD)),)
siproxd:
	cd $(OPENSOURCE_DIR);   (tar xkfj siproxd.tar.bz2 2> /dev/null || true)
	$(MAKE) -C $(OPENSOURCE_DIR)/siproxd $(BUILD_SIPROXD)
libosip2:
	cd $(OPENSOURCE_DIR);   (tar xkfj libosip2.tar.bz2 2> /dev/null || true)
	$(MAKE) -C $(OPENSOURCE_DIR)/libosip2
else
siproxd:

libosip2:

endif

ifneq ($(strip $(BUILD_SNMP)),)

ifneq ($(strip $(BUILD_SNMP_SET)),)
export SNMP_SET=1
else
export SNMP_SET=0
endif

ifneq ($(strip $(BUILD_SNMP_ADSL_MIB)),)
export SNMP_ADSL_MIB=1
else
export SNMP_ADSL_MIB=0
endif

ifneq ($(strip $(BUILD_SNMP_ATM_MIB)),)
export SNMP_ATM_MIB=1
else
export SNMP_ATM_MIB=0
endif

ifneq ($(strip $(BUILD_SNMP_BRIDGE_MIB)),)
export SNMP_BRIDGE_MIB=1
else
export SNMP_BRIDGE_MIB=0
endif

ifneq ($(strip $(BUILD_SNMP_AT_MIB)),)
export SNMP_AT_MIB=1
else
export SNMP_AT_MIB=0
endif

ifneq ($(strip $(BUILD_SNMP_SYSOR_MIB)),)
export SNMP_SYSOR_MIB=1
else
export SNMP_SYSOR_MIB=0
endif

ifneq ($(strip $(BUILD_SNMP_TCP_MIB)),)
export SNMP_TCP_MIB=1
else
export SNMP_TCP_MIB=0
endif

ifneq ($(strip $(BUILD_SNMP_UDP_MIB)),)
export SNMP_UDP_MIB=1
else
export SNMP_UDP_MIB=0
endif

ifneq ($(strip $(BUILD_SNMP_IP_MIB)),)
export SNMP_IP_MIB=1
else
export SNMP_IP_MIB=0
endif

ifneq ($(strip $(BUILD_SNMP_ICMP_MIB)),)
export SNMP_ICMP_MIB=1
else
export SNMP_ICMP_MIB=0
endif

ifneq ($(strip $(BUILD_SNMP_SNMP_MIB)),)
export SNMP_SNMP_MIB=1
else
export SNMP_SNMP_MIB=0
endif

ifneq ($(strip $(BUILD_SNMP_ATMFORUM_MIB)),)
export SNMP_ATMFORUM_MIB=1
else
export SNMP_ATMFORUM_MIB=0
endif


ifneq ($(strip $(BUILD_SNMP_CHINA_TELECOM_CPE_MIB)),)
export BUILD_SNMP_CHINA_TELECOM_CPE_MIB=y
endif

ifneq ($(strip $(BUILD_CT_1_39_OPEN)),)
export BUILD_CT_1_39_OPEN=y
endif

ifneq ($(strip $(BUILD_SNMP_CHINA_TELECOM_CPE_MIB_V2)),)
export BUILD_SNMP_CHINA_TELECOM_CPE_MIB_V2=y
endif

ifneq ($(strip $(BUILD_SNMP_BRCM_CPE_MIB)),)
export BUILD_SNMP_BRCM_CPE_MIB=y
endif

ifneq ($(strip $(BUILD_SNMP_UDP)),)
export BUILD_SNMP_UDP=y
endif

ifneq ($(strip $(BUILD_SNMP_EOC)),)
export BUILD_SNMP_EOC=y
endif

ifneq ($(strip $(BUILD_SNMP_AAL5)),)
export BUILD_SNMP_AAL5=y
endif

ifneq ($(strip $(BUILD_SNMP_AUTO)),)
export BUILD_SNMP_AUTO=y
endif

ifneq ($(strip $(BUILD_SNMP_DEBUG)),)
export BUILD_SNMP_DEBUG=y
endif

ifneq ($(strip $(BUILD_SNMP_TRANSPORT_DEBUG)),)
export BUILD_SNMP_TRANSPORT_DEBUG=y
endif

ifneq ($(strip $(BUILD_SNMP_LAYER_DEBUG)),)
export BUILD_SNMP_LAYER_DEBUG=y
endif

endif

ifneq ($(strip $(BUILD_4_LEVEL_QOS)),)
export BUILD_4_LEVEL_QOS=y
endif

ifneq ($(strip $(BUILD_ILMI)),)
ilmi:
	cd $(OPENSOURCE_DIR);   (tar xkfj net-snmp.tar.bz2 2> /dev/null || true)
	$(MAKE) -C $(BROADCOM_DIR)/ilmi $(BUILD_ILMI)
else
ilmi:
endif


# vodsl (voice daemon)
# mwang: for now, I still need to leave vodsl at this top level because
# it depends on so many of the voice defines in this top level Makefile.
ifneq ($(strip $(BUILD_VODSL)),)
vodsl: sanity_check
	$(MAKE) -C $(BUILD_DIR)/userspace/private/apps/vodsl $(BUILD_VODSL)
else
vodsl: sanity_check
	@echo "skipping vodsl (not configured)"
endif


# Leave it for the future when soap server is decoupled from cfm
ifneq ($(strip $(BUILD_SOAP)),)
ifeq ($(strip $(BUILD_SOAP_VER)),2)
soapserver:
	$(MAKE) -C $(BROADCOM_DIR)/SoapToolkit/SoapServer $(BUILD_SOAP)
else
soap:
	$(MAKE) -C $(BROADCOM_DIR)/soap $(BUILD_SOAP)
endif
else
soap:
endif



ifeq ($(strip $(BUILD_LIBUSB)),y)
libusb:
	cd $(OPENSOURCE_DIR);   (tar xkfj libusb.tar.bz2 2> /dev/null || true)
	$(MAKE) -C $(OPENSOURCE_DIR)/libusb
	install -m 755 $(OPENSOURCE_DIR)/libusb/.libs/libusb-0.1.so.4 $(INSTALL_DIR)/lib
else
libusb:
endif



ifneq ($(strip $(BUILD_DIAGAPP)),)
diagapp:
	$(MAKE) -C $(BROADCOM_DIR)/diagapp $(BUILD_DIAGAPP)
else
diagapp:
endif



ifneq ($(strip $(BUILD_IPPD)),)
ippd:
	$(MAKE) -C $(BROADCOM_DIR)/ippd $(BUILD_IPPD)
else
ippd:
endif


ifneq ($(strip $(BUILD_PORT_MIRRORING)),)
export BUILD_PORT_MIRRORING=1
else
export BUILD_PORT_MIRRORING=0
endif

ifeq ($(BRCM_USE_SUDO_IFNOT_ROOT),y)
BRCM_BUILD_USR=$(shell whoami)
BRCM_BUILD_USR1=$(shell sudo touch foo;ls -l foo | awk '{print $$3}';sudo rm -rf foo)
else
BRCM_BUILD_USR=root
endif

ifneq ($(strip $(BUILD_OPROFILE)),)
oprofile:
	cd $(OPENSOURCE_DIR);   (tar xkfj oprofile.tar.bz2 2> /dev/null || true)
	$(MAKE) -C $(OPENSOURCE_DIR)/oprofile
else
oprofile:
endif

hosttools:
	$(MAKE) -C $(HOSTTOOLS_DIR)

ifneq ($(strip $(BUILD_PPTP_CLIENT)),)
pptp:
	$(MAKE) -C $(BUILD_DIR)/userspace/gpl/apps/pptp-client dynamic
pptp_clean:
	$(MAKE) -C $(BUILD_DIR)/userspace/gpl/apps/pptp-client clean
else
pptp:
pptp_clean:
endif

ifneq ($(strip $(BUILD_SAMBA)),)
samba:
	$(MAKE) -C $(BUILD_DIR)/userspace/gpl/apps/samba-3.0.28/source bin/smbd
	$(MAKE) -C $(BUILD_DIR)/userspace/gpl/apps/samba-3.0.28/source bin/nmbd
	install -m 755 $(BUILD_DIR)/userspace/gpl/apps/samba-3.0.28/source/bin/smbd $(INSTALL_DIR)/bin
	$(STRIP) $(INSTALL_DIR)/bin/smbd
	install -m 755 $(BUILD_DIR)/userspace/gpl/apps/samba-3.0.28/source/bin/nmbd $(INSTALL_DIR)/bin
	$(STRIP) $(INSTALL_DIR)/bin/nmbd
samba_clean:
	$(MAKE) -C $(BUILD_DIR)/userspace/gpl/apps/samba-3.0.28/source clean
else
samba:
samba_clean:
endif

############################################################################
#
# IKOS defines
#
############################################################################

export BRCM_IKOS
CMS_VERSION_FILE=$(BUILD_DIR)/userspace/public/include/version.h

ifeq ($(strip $(BRCM_IKOS)),y)
FS_COMPRESSION=-noD
else
FS_COMPRESSION=-lzma
endif


# IKOS Emulator build that does not include the CFE boot loader.
# Edit targets/ikos/ikos and change the chip and board id to desired values.
# Then build: make PROFILE=ikos ikos
ikos:
	@echo -e '#define SOFTWARE_VERSION ""\n#define RELEASE_VERSION ""\n#define PSI_VERSION ""\n' > $(CMS_VERSION_FILE)
	@-mv -f $(FSSRC_DIR)/etc/profile $(FSSRC_DIR)/etc/profile.dontuse >& /dev/null
	@if [ ! -a $(CFE_FILE) ] ; then echo "no cfe" > $(CFE_FILE); echo "no cfe" > $(CFE_FILE).del; fi
	@-rm $(HOSTTOOLS_DIR)/bcmImageBuilder >& /dev/null
	$(MAKE) PROFILE=$(PROFILE)
	@-rm $(HOSTTOOLS_DIR)/bcmImageBuilder >& /dev/null
	@mv -f $(FSSRC_DIR)/etc/profile.dontuse $(FSSRC_DIR)/etc/profile
	@cd $(PROFILE_DIR); \
	$(OBJCOPY) --output-target=srec vmlinux vmlinux.srec; \
	xxd $(FS_KERNEL_IMAGE_NAME) | grep "^00000..:" | xxd -r > bcmtag.bin; \
	$(OBJCOPY) --output-target=srec --input-target=binary --change-addresses=0xb8010000 bcmtag.bin bcmtag.srec; \
	$(OBJCOPY) --output-target=srec --input-target=binary --change-addresses=0xb8010100 rootfs.img rootfs.srec; \
	@rm bcmtag.bin; \
	grep -v "^S7" vmlinux.srec > bcm9$(BRCM_CHIP)_$(PROFILE).srec; \
	grep "^S3" bcmtag.srec >> bcm9$(BRCM_CHIP)_$(PROFILE).srec; \
	grep -v "^S0" rootfs.srec >> bcm9$(BRCM_CHIP)_$(PROFILE).srec
	@if [ ! -a $(CFE_FILE).del ] ; then rm -f $(CFE_FILE) $(CFE_FILE).del; fi
	@echo -e "\nAn image without CFE for the IKOS emulator has been built.  It is named"
	@echo -e "targets/$(PROFILE)/bcm9$(BRCM_CHIP)_$(PROFILE).srec\n"

# IKOS Emulator build that includes the CFE boot loader.
# Both Linux and CFE boot loader toolchains need to be installed.
# Edit targets/ikos/ikos and change the chip and board id to desired values.
# Then build: make PROFILE=ikos ikoscfe
ikoscfe:
	@echo -e '#define SOFTWARE_VERSION ""\n#define RELEASE_VERSION ""\n#define PSI_VERSION ""\n' > $(CMS_VERSION_FILE)
	@-mv -f $(FSSRC_DIR)/etc/profile $(FSSRC_DIR)/etc/profile.dontuse >& /dev/null
	$(MAKE) PROFILE=$(PROFILE)
	@mv -f $(FSSRC_DIR)/etc/profile.dontuse $(FSSRC_DIR)/etc/profile
	$(MAKE) -C $(BL_BUILD_DIR) clean
	$(MAKE) -C $(BL_BUILD_DIR)
	$(MAKE) -C $(BL_BUILD_DIR) ikos_finish
	cd $(PROFILE_DIR); \
	echo -n "** no kernel  **" > kernelfile; \
	$(HOSTTOOLS_DIR)/bcmImageBuilder --output $(CFE_FS_KERNEL_IMAGE_NAME) --chip $(BRCM_CHIP) --board $(BRCM_BOARD_ID) --blocksize $(BRCM_FLASHBLK_SIZE) --cfefile $(BL_BUILD_DIR)/cfe$(BRCM_CHIP).bin --rootfsfile rootfs.img --kernelfile kernelfile --include-cfe; \
	$(HOSTTOOLS_DIR)/createimg --boardid=$(BRCM_BOARD_ID) --numbermac=$(BRCM_NUM_MAC_ADDRESSES) --macaddr=$(BRCM_BASE_MAC_ADDRESS) --tp=$(BRCM_MAIN_TP_NUM) --psisize=$(BRCM_PSI_SIZE) --inputfile=$(CFE_FS_KERNEL_IMAGE_NAME) --outputfile=$(FLASH_IMAGE_NAME); \
	$(HOSTTOOLS_DIR)/addvtoken $(FLASH_IMAGE_NAME) $(FLASH_IMAGE_NAME).w; \
	$(OBJCOPY) --output-target=srec --input-target=binary --change-addresses=0xb8000000 $(FLASH_IMAGE_NAME).w $(FLASH_IMAGE_NAME).srec; \
	$(OBJCOPY) --output-target=srec vmlinux vmlinux.srec; \
	@rm kernelfile; \
	grep -v "^S7" vmlinux.srec > bcm9$(BRCM_CHIP)_$(PROFILE).srec; \
	grep "^S3" $(BL_BUILD_DIR)/cferam$(BRCM_CHIP).srec >> bcm9$(BRCM_CHIP)_$(PROFILE).srec; \
	grep -v "^S0" $(FLASH_IMAGE_NAME).srec >> bcm9$(BRCM_CHIP)_$(PROFILE).srec; \
	grep -v "^S7" vmlinux.srec > bcm9$(BRCM_CHIP)_$(PROFILE).utram.srec; \
	grep -v "^S0" $(BL_BUILD_DIR)/cferam$(BRCM_CHIP).srec >> bcm9$(BRCM_CHIP)_$(PROFILE).utram.srec;
	@echo -e "\nAn image with CFE for the IKOS emulator has been built.  It is named"
	@echo -e "targets/$(PROFILE)/bcm9$(BRCM_CHIP)_$(PROFILE).srec"
	@echo -e "\nBefore testing with the IKOS emulator, this build can be unit tested"
	@echo -e "with an existing chip and board as follows."
	@echo -e "1. Flash targets/$(PROFILE)/$(FLASH_IMAGE_NAME).w onto an existing board."
	@echo -e "2. Start the EPI EDB debugger.  At the edbice prompt, enter:"
	@echo -e "   edbice> fr m targets/$(PROFILE)/bcm9$(BRCM_CHIP)_$(PROFILE).utram.srec"
	@echo -e "   edbice> r"
	@echo -e "3. Program execution will start at 0xb8000000 (or 0xbfc00000) and,"
	@echo -e "   if successful, will enter the Linux shell.\n"



############################################################################
#
# This is where we build the image
#
############################################################################

buildimage: $(KERNEL_DIR)/vmlinux libcreduction
	@echo "MODEL=$(PROFILE)" > $(FSSRC_DIR)/etc/versions
ifneq ($(strip $(CUSTOM_FIRMWARE_NAME)),)
	@echo "VERSION=$(CUSTOM_FIRMWARE_NAME)" >> $(FSSRC_DIR)/etc/versions
endif
	@echo "BCM_VERSION=$(BRCM_RELEASETAG)" >> $(FSSRC_DIR)/etc/versions
	@echo "Revision=1489" >> $(FSSRC_DIR)/etc/versions
	@echo "FSSTAMP=$(shell date +'%Y%m%d%H%M%S')" >> $(FSSRC_DIR)/etc/versions
	@echo "BUILDDATE=$(shell export LANG=C;date +'%b. %d, %Y')" >> $(FSSRC_DIR)/etc/versions

ifeq ($(strip $(BRCM_KERNEL_ROOTFS)),squashfs)
	cd $(TARGETS_DIR); \
	./buildFS
else
ifeq ($(BRCM_BUILD_USR),root)
	su --command="cd $(TARGETS_DIR); ./buildFS"
else
ifeq ($(BRCM_BUILD_USR1),root)
	cd $(TARGETS_DIR); sudo ./buildFS
else
	cd $(TARGETS_DIR); rm -rf $(PROFILE)/fs; ln -s /work/$(BRCM_BUILD_USR)/fs $(PROFILE)/fs; sudo ./buildFS
endif
endif
endif

# (mksquashfs call moved to buildFS, for fakeroot to work)

ifneq ($(strip $(BRCM_KERNEL_ROOTFS)),nfs)
	cd $(PROFILE_DIR); \
	cp $(KERNEL_DIR)/vmlinux . ; \
	$(STRIP) --remove-section=.note --remove-section=.comment vmlinux; \
	$(OBJCOPY) -O binary vmlinux vmlinux.bin; \
	$(HOSTTOOLS_DIR)/cmplzma -k -2 vmlinux vmlinux.bin vmlinux.lz;\
	$(HOSTTOOLS_DIR)/bcmImageBuilder --output $(FS_KERNEL_IMAGE_NAME) --chip $(BRCM_CHIP) --board $(BRCM_BOARD_ID) --blocksize $(BRCM_FLASHBLK_SIZE) --cfefile $(CFE_FILE) --rootfsfile rootfs.img --kernelfile vmlinux.lz; \
	$(HOSTTOOLS_DIR)/bcmImageBuilder --output $(CFE_FS_KERNEL_IMAGE_NAME) --chip $(BRCM_CHIP) --board $(BRCM_BOARD_ID) --blocksize $(BRCM_FLASHBLK_SIZE) --cfefile $(CFE_FILE) --rootfsfile rootfs.img --kernelfile vmlinux.lz --include-cfe; \
	$(HOSTTOOLS_DIR)/createimg --boardid=$(BRCM_BOARD_ID) --numbermac=$(BRCM_NUM_MAC_ADDRESSES) --macaddr=$(BRCM_BASE_MAC_ADDRESS) --tp=$(BRCM_MAIN_TP_NUM) --psisize=$(BRCM_PSI_SIZE) --gponsn=$(BRCM_GPON_SERIAL_NUMBER) --gponpw=$(BRCM_GPON_PASSWORD) --inputfile=$(CFE_FS_KERNEL_IMAGE_NAME) --outputfile=$(FLASH_IMAGE_NAME); \
	$(HOSTTOOLS_DIR)/addvtoken $(FLASH_IMAGE_NAME) $(FLASH_IMAGE_NAME).w
	@mkdir -p $(IMAGES_DIR)
#### TEY add Image tag for firmware lock
	@echo $(FIRMWARELOCKTAG)>> $(PROFILE_DIR)/$(FS_KERNEL_IMAGE_NAME)
	@echo $(FIRMWARELOCKTAG) >> $(PROFILE_DIR)/$(CFE_FS_KERNEL_IMAGE_NAME)
	@cp $(PROFILE_DIR)/$(FS_KERNEL_IMAGE_NAME) $(IMAGES_DIR)/$(FS_KERNEL_IMAGE_NAME)_$(BRCM_RELEASETAG)-$(shell date '+%y%m%d_%H%M')
	@echo
	@echo -e "Done! Image $(PROFILE) has been built in $(IMAGES_DIR)."
else
	cd $(PROFILE_DIR); \
	cp $(KERNEL_DIR)/vmlinux . 
	@echo
	@echo -e "\t=== Following the below steps to start your NFS root file system on host ==="
	@echo -e "Step 1: Copy $(PROFILE_DIR)/vmlinux to your TFTP server boot directory, such as /tftpboot"
	@echo -e "Step 2: Add \"$(PROFILE_DIR)/fs *(rw,no_root_squash)\" to /etc/exports"
	@echo -e "Step 3: Become root and restart your nfs server, such as \"service nfs restart\""
	@echo -e "Step 4: Reboot your board and break into CFE bootloader, choose h on \"Run from flash/host\" and vmlinux on \"Default host run file name\""
endif



###########################################
#
# System code clean-up
#
###########################################

#
# mwang: since SUBDIRS are no longer defined, the next two targets are not useful anymore.
# how were they used anyways?
#
#subdirs: $(patsubst %, _dir_%, $(SUBDIRS))

#$(patsubst %, _dir_%, $(SUBDIRS)) :
#	$(MAKE) -C $(patsubst _dir_%, %, $@) $(TGT)

#clean: target_clean userspace_clean kernel_clean hosttools_clean voice_clean xchange_clean wlan_clean pptp_clean samba_clean
clean: target_clean kernel_clean
	rm -f $(HOSTTOOLS_DIR)/scripts/lxdialog/*.o
	rm -f .tmpconfig*

fssrc_clean:
	rm -fr $(FSSRC_DIR)/bin
	rm -fr $(FSSRC_DIR)/sbin
	rm -fr $(FSSRC_DIR)/lib
	rm -fr $(FSSRC_DIR)/upnp
	rm -fr $(FSSRC_DIR)/docs
	rm -fr $(FSSRC_DIR)/webs
	rm -fr $(FSSRC_DIR)/usr
	rm -fr $(FSSRC_DIR)/linuxrc
	rm -fr $(FSSRC_DIR)/images
	rm -fr $(FSSRC_DIR)/etc/wlan
	rm -fr $(FSSRC_DIR)/etc/certs

kernel_clean: sanity_check
	$(MAKE) -C $(KERNEL_DIR) mrproper
	rm -f $(KERNEL_DIR)/arch/mips/defconfig
	rm -f $(HOSTTOOLS_DIR)/lzma/decompress/*.o
	rm -rf $(XCHANGE_DIR)/dslx/lib/LinuxKernel
	rm -rf $(XCHANGE_DIR)/dslx/obj/LinuxKernel

userspace_clean: sanity_check app_expand_before_clean fssrc_clean 
	$(MAKE) -C userspace clean


app_expand_before_clean:
	# these archives need to be expended before cleaning - put makefile structures there there for cleaning
	# OProfile
	(tar xkfj $(BUILD_DIR)/userspace/gpl/apps/oprofile.tar.bz2 -C $(BUILD_DIR)/userspace/gpl/apps 2> /dev/null || true)

data-model_clean:
	$(MAKE) -C data-model clean

unittests_clean:
	$(MAKE) -C unittests clean

target_clean: sanity_check
	rm -f $(PROFILE_DIR)/rootfs.img
	rm -f $(PROFILE_DIR)/vmlinux
	rm -f $(PROFILE_DIR)/vmlinux.bin
	rm -f $(PROFILE_DIR)/vmlinux.lz
	rm -f $(PROFILE_DIR)/$(FS_KERNEL_IMAGE_NAME)
	rm -f $(PROFILE_DIR)/$(CFE_FS_KERNEL_IMAGE_NAME)
	rm -f $(PROFILE_DIR)/$(FLASH_IMAGE_NAME)
	rm -f $(PROFILE_DIR)/$(FLASH_IMAGE_NAME).w
	rm -f $(PROFILE_DIR)/*.srec
	rm -fr $(PROFILE_DIR)/modules
	rm -fr $(PROFILE_DIR)/op
	rm -fr $(INSTALL_DIR)
	find targets -name vmlinux -print -exec rm -f "{}" ";"
ifeq ($(strip $(BRCM_KERNEL_ROOTFS)),squashfs)
	rm -fr $(TARGET_FS)
else
ifeq ($(BRCM_BUILD_USR),root)
	su --command="rm -fr $(TARGET_FS)"
else 
	sudo rm -fr $(TARGET_FS)/*; rm -rf $(TARGET_FS)
endif
endif

hosttools_clean:
	$(MAKE) -C $(HOSTTOOLS_DIR) clean

xchange_clean:
	rm -rf $(XCHANGE_DIR)/dslx/lib/LinuxUser
	rm -rf $(XCHANGE_DIR)/dslx/obj/LinuxUser	

# In the data release tarball, vodsl is not present,
# so don't go in there unconditionally.
vodsl_clean: sanity_check
ifneq ($(strip $(BUILD_VODSL)),)
	find userspace/private/apps/vodsl -name '*.o' -exec rm -f "{}" ";"
	@if [ -e xChange/dslx/obj/LinuxUser ]; then \
	  (find xChange/dslx/obj/LinuxUser  -name '*' -exec rm -f "{}" ";") \
	fi
endif

voice_clean: sanity_check vodsl_clean
ifneq ($(strip $(BUILD_VODSL)),)
	find bcmdrivers/broadcom/char/endpoint -name '*.ko' -exec rm -f "{}" ";"
	find bcmdrivers/broadcom/char/endpoint -name '*.o' -exec rm -f "{}" ";"
	find bcmdrivers/broadcom/char/endpoint -name '*.a' -exec rm -f "{}" ";"
	find bcmdrivers/broadcom/char/endpoint -name '*.lib' -exec rm -f "{}" ";"
	find bcmdrivers/broadcom/char/dspapp -name '*.o' -exec rm -f "{}" ";"
	find bcmdrivers/broadcom/char/dspapp -name '*.ko' -exec rm -f "{}" ";"
	find kernel/linux/eptlib -name '*.lib' -exec rm -f "{}" ";"
endif
	rm -rf $(XCHANGE_DIR)/dslx/lib/LinuxKernel
	rm -rf $(XCHANGE_DIR)/dslx/obj/LinuxKernel
	rm -rf $(XCHANGE_DIR)/dslx/lib/LinuxUser
	rm -rf $(XCHANGE_DIR)/dslx/obj/LinuxUser
	rm -f $(KERNEL_DIR)/eptlib/*

wlan_clean:
	cd bcmdrivers/broadcom/net/wl;\
	files="\
		`find . -name 'build'`\
		`find . -type l`\
		`find . -name '*.o'` \
		`find . -name '*.ko'`\
		`find . -name '*.oo'`\
		`find . -name '*.bin'`\
		`find . -name 'dummy.c'`\
	";\
	for x in $$files; do\
		rm -rf $$x;\
	done

###########################################
#
# Temporary kernel patching mechanism
#
###########################################

.PHONY: genpatch patch

genpatch:
	@hostTools/kup_tmp/genpatch

patch:
#	@hostTools/kup_tmp/patch
	

###########################################
#
# System-wide exported variables
# (in alphabetical order)
#
###########################################

export \
BRCMAPPS                   \
BRCM_BOARD                 \
BRCM_DRIVER_PCI            \
BRCM_EXTRAVERSION          \
BRCM_KERNEL_NETQOS         \
BRCM_KERNEL_ROOTFS         \
BRCM_SQUASHFS_BLOCK_SIZE         \
BRCM_SQUASHFS_FRAGMENT_CACHE_SIZE         \
BRCM_KERNEL_OPROFILE       \
BRCM_LDX_APP               \
BRCM_MIPS_ONLY_BUILD       \
BRCM_MIPS_ONLY_BUILD       \
BRCM_PSI_VERSION           \
BRCM_PTHREADS              \
BRCM_RELEASE               \
BRCM_RELEASETAG            \
BRCM_SNMP                  \
BRCM_VERSION               \
BUILD_CMFCTL               \
BUILD_CMFVIZ               \
BUILD_XDSLCTL              \
BUILD_XTMCTL               \
BUILD_BR2684CTL            \
BUILD_BRCM_VLAN            \
BUILD_BRCTL                \
BUILD_BUSYBOX              \
BUILD_CERT                 \
BUILD_DDNSD                \
BUILD_DIAGAPP              \
BUILD_DIR                  \
BUILD_DNSPROBE             \
BUILD_DPROXY               \
BUILD_DYNAHELPER           \
BUILD_DNSSPOOF             \
BUILD_EBTABLES             \
BUILD_EPITTCP              \
BUILD_ETHWAN               \
BUILD_FTPD                 \
BUILD_FTPD_STORAGE         \
BUILD_GDBSERVER            \
BUILD_HTTPD                \
BUILD_HTTPD_SSL            \
BUILD_IGMP                 \
BUILD_IPPD                 \
BUILD_IPROUTE2             \
BUILD_IPSEC_TOOLS          \
BUILD_IPTABLES             \
BUILD_WPS_BTN              \
BUILD_LLTD                 \
BUILD_WSC                   \
BUILD_BCMCRYPTO \
BUILD_BCMSHARED \
BUILD_NAS                  \
BUILD_NVRAM                \
BUILD_OPROFILE             \
BUILD_PORT_MIRRORING			 \
BUILD_PPPD                 \
BUILD_PPTP_CLIENT          \
BUILD_RT2684D              \
BUILD_SES                  \
BUILD_SIPROXD              \
BUILD_SLACTEST             \
BUILD_SNMP                 \
BUILD_SNTP                 \
BUILD_SOAP                 \
BUILD_SOAP_VER             \
BUILD_SSHD                 \
BUILD_SSHD_MIPS_GENKEY     \
BUILD_TOD                  \
BUILD_TR64                 \
BUILD_TR64_DEVICECONFIG    \
BUILD_TR64_DEVICEINFO      \
BUILD_TR64_LANCONFIGSECURITY \
BUILD_TR64_LANETHINTERFACECONFIG \
BUILD_TR64_LANHOSTS        \
BUILD_TR64_LANHOSTCONFIGMGMT \
BUILD_TR64_LANUSBINTERFACECONFIG \
BUILD_TR64_LAYER3          \
BUILD_TR64_MANAGEMENTSERVER  \
BUILD_TR64_TIME            \
BUILD_TR64_USERINTERFACE   \
BUILD_TR64_QUEUEMANAGEMENT \
BUILD_TR64_LAYER2BRIDGE   \
BUILD_TR64_WANCABLELINKCONFIG \
BUILD_TR64_WANCOMMONINTERFACE \
BUILD_TR64_WANDSLINTERFACE \
BUILD_TR64_WANDSLLINKCONFIG \
BUILD_TR64_WANDSLCONNECTIONMGMT \
BUILD_TR64_WANDSLDIAGNOSTICS \
BUILD_TR64_WANETHERNETCONFIG \
BUILD_TR64_WANETHERNETLINKCONFIG \
BUILD_TR64_WANIPCONNECTION \
BUILD_TR64_WANPOTSLINKCONFIG \
BUILD_TR64_WANPPPCONNECTION \
BUILD_TR64_WLANCONFIG      \
BUILD_TR69C                \
BUILD_TR69_QUEUED_TRANSFERS \
BUILD_TR69C_SSL            \
BUILD_TR69_XBRCM           \
BUILD_TR69_UPLOAD          \
BUILD_GPON_OMCI            \
BUILD_UDHCP                \
BUILD_UDHCP_RELAY          \
BUILD_UPNP                 \
BUILD_VCONFIG              \
BUILD_SUPERDMZ             \
BUILD_SIP_TLS              \
BUILD_WLCTL                \
BUILD_ZEBRA                \
BUILD_LIBUSB               \
BUILD_WANVLANMUX           \
HOSTTOOLS_DIR              \
INC_KERNEL_BASE            \
INSTALL_DIR                \
JTAG_KERNEL_DEBUG          \
PROFILE_DIR                \
WEB_POPUP                  \
HTTPS_SUPPORT              \
BUILD_VIRT_SRVR            \
BUILD_PORT_TRIG            \
BUILD_TR69C_BCM_SSL        \
BUILD_IPV6                 \
LINUX_KERNEL_USBMASS       \
BUILD_MLD                  \
BUILD_IPSEC                \
BUILD_SPUCTL               \
BUILD_PWRCTL               \
RELEASE_BUILD        	   \
BUILD_SAMBA							 \
CUSTOM_FIRMWARE_NAME     \
BUILD_SIPALG		\
BUILD_2760UBZ		\
BUILD_2760U		\
ENABLE_SERIAL_LED       \
ENABLE_USB_LED1          \
ENABLE_USB_LED2          \
SUPPORT_WL_BUTTON        \
NO_PRINTK_AND_BUG		\
BUILD_DLINK_WEB_LOGO	\
FACTORY_RESET_LED		\
CUSTOM_UPNP_MANUFACTURER	\
PPP_AUTODISCONN	\
WL_SSID_FACTORY_SETTING	\
CORRECT_WAN_MAC_FROM_TR69		\
BUILD_3G_USB_DEVICE			\
PPPD_VERSION        \
DSL_2760BN		\
PROFILE
