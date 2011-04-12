include ../common.mk

EXTRA_CFLAGS += -I$(M)/include

obj-m = xt_connlimit.o

xt_connlimit-y := xt_connlimit_orig.o backport.o

