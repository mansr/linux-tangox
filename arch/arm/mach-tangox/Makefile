obj-$(CONFIG_ARCH_TANGO) += setup.o smc.o
obj-$(CONFIG_SMP) += smp.o

plus_sec 		:= $(call as-instr,.arch_extension sec,+sec)
AFLAGS_smc.o		:= -Wa,-march=armv7-a$(plus_sec)
