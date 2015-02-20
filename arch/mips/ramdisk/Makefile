#
# Makefile for a ramdisk image
#
# Note! Dependencies are done automagically by 'make dep', which also
# removes any old dependencies. DON'T put your own dependencies here
# unless it's something special (ie not a .c file).
#

ifneq ($(CONFIG_EMBEDDED_RAMDISK),)
ifneq ($(CONFIG_EMBEDDED_RAMDISK_IMAGE),)
IMAGENAME := $(CONFIG_EMBEDDED_RAMDISK_IMAGE)
else
IMAGENAME := $(SMP86XX_ROOTFS_PATH)/root_fs_mipsel.cramfs
endif
RDPATH := $(TOPDIR)/arch/mips/ramdisk
O_FORMAT := $(shell $(OBJDUMP) -i | head -2 | grep elf32)
RD_MADE := $(shell $(LD) $(LDFLAGS) -T $(RDPATH)/ld.script -b binary --oformat $(O_FORMAT) -o $(RDPATH)/ramdisk.o $(IMAGENAME))
endif

obj-$(CONFIG_EMBEDDED_RAMDISK) += ramdisk.o

clean-files += ramdisk.o

