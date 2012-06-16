#
# Makefile for SigmaDesigns Tango2/Tango3/Tango4 board
#
# Note! Dependencies are done automagically by 'make dep', which also
# removes any old dependencies. DON'T put your own dependencies here
# unless it's something special (ie not a .c file).
#

obj-y += irq.o setup.o prom.o gbus.o xenv_config.o delay.o platform_dev.o console.o

obj-$(CONFIG_TANGO2) += cpucache.o

obj-$(CONFIG_TANGOX_XENV_READ) += sha.o xenv.o xenv_config.o

obj-$(CONFIG_SMP) += amon.o

ifdef CONFIG_TANGO3
obj-$(CONFIG_TANGOX_XENV_READ) += zxenv.o
endif
ifdef CONFIG_TANGO4
obj-$(CONFIG_TANGOX_XENV_READ) += zxenv.o
endif

obj-$(CONFIG_TANGOX) += gpio.o

obj-$(CONFIG_TANGOX) += mbus.o

