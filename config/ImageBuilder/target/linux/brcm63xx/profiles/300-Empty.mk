#
# Copyright (C) 2009 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

define Profile/Empty
  NAME:=Empty (Fully Customized)
  PACKAGES:=
endef

define Profile/Empty/description
  Empty package set 
endef

$(eval $(call Profile,Empty))
