
TOP_DIR := $(TOPDIR)

# More parameters to be configured
WITH_IR		:= $(CONFIG_TANGOX_STANDBY_WAKEUP_IR)
WITH_FIP	:= $(CONFIG_TANGOX_STANDBY_WAKEUP_FIP)
WITH_ETH	:= $(CONFIG_TANGOX_STANDBY_WAKEUP_ETH)
INPUT_TABLE_SIZE:= $(CONFIG_TANGOX_STANDBY_INPUT_TABLE_SIZE)

# For debugging purpose
FCTRL_DEBUG     := n

# 
#LOCALMEM_BASE	:= 0xa0060010
# DMEM_BASE_audio_engine_0
LOCALMEM_BASE := 0xa0190010

ifeq ($(FCTRL_DEBUG),y)
CFLAGS += -DFC_DEBUG
endif

CFLAGS += -DEM86XX_CHIP=EM86XX_CHIPID_TANGO2 -DEM86XX_REVISION=6 -DCONFIG_TANGO2 -DLOCALMEM_BASE=$(LOCALMEM_BASE)

obj-$(CONFIG_TANGOX_FREQUENCY_CONTROL) := fctrl.o

FC_MADE_CLEAN := $(shell $(MAKE) -C $(TOP_DIR)/arch/mips/tangox/fctrl/lib/fc clean )
FC_MADE := $(shell $(MAKE) -C $(TOP_DIR)/arch/mips/tangox/fctrl/lib/fc WITH_IR=$(WITH_IR) WITH_FIP=$(WITH_FIP) WITH_ETH=$(WITH_ETH) INPUT_TABLE_SIZE=$(INPUT_TABLE_SIZE) FCTRL_DEBUG=$(FCTRL_DEBUG) TOP_DIR=$(TOP_DIR) LOCALMEM_BASE=$(LOCALMEM_BASE) )

fctrl-objs := lib/freqctrl.o lib/ir.o lib/fip.o lib/eth.o fckernel.o

.PHONY: all
all: fctrl.ko

fctrl.ko: FORCE lib/freqctrl.o lib/ir.o lib/fip.o lib/eth.o fckernel.o
	$(MAKE) -C $(TOP_DIR) V=$(V) M="$$PWD" modules

.PHONY: clean
clean:
	$(MAKE) -C $(TOP_DIR)/arch/mips/tangox/fctrl/lib/fc clean
	$(MAKE) -C $(TOP_DIR) V=$(V) M="$$PWD" clean

.PHONY: FORCE
FORCE:

