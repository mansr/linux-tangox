#
# Makefile for SigmaDesigns Tango2/Tango3 board
#
# Note! Dependencies are done automagically by 'make dep', which also
# removes any old dependencies. DON'T put your own dependencies here
# unless it's something special (ie not a .c file).
#

obj-y += irq.o setup.o prom.o gbus.o xenv_config.o delay.o 

obj-$(CONFIG_TANGO2) += cpucache.o

obj-$(CONFIG_TANGOX_PROM_CONSOLE) += console.o

obj-$(CONFIG_TANGOX_XENV_READ) += sha.o xenv.o

obj-y += gpio.o

obj-y += mbus.o

obj-$(CONFIG_TANGOX_FREQUENCY_CONTROL) += fctrl/

clean-files += $(shell $(MAKE) -C $(TOPDIR)/arch/mips/tangox/fctrl/lib/fc clean)

