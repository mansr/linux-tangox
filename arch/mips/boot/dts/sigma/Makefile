dtb-$(CONFIG_DTB_PCH_C200)	+= pch_c200.dtb
dtb-$(CONFIG_DTB_TANGOX_IPU)	+= smp8640_ipu.dtb

obj-y				+= $(patsubst %.dtb, %.dtb.o, $(dtb-y))

# Force kbuild to make empty built-in.o if necessary
obj-				+= dummy.o

always				:= $(dtb-y)
clean-files			:= *.dtb *.dtb.S
