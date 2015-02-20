#
# This file is subject to the terms and conditions of the GNU General Public
# License.  See the file "COPYING" in the main directory of this archive
# for more details.
#
# Copyright (C) 1995, 1998, 2001, 2002 by Ralf Baechle
# Copyright (C) 2004  Maciej W. Rozycki
#

#
# Some DECstations need all possible sections of an ECOFF executable
#
ifdef CONFIG_MACH_DECSTATION
  E2EFLAGS = -a
else
  E2EFLAGS =
endif

#
# Drop some uninteresting sections in the kernel.
# This is only relevant for ELF kernels but doesn't hurt a.out
#
drop-sections	= .reginfo .mdebug .comment .note .pdr .options .MIPS.options
strip-flags	= $(addprefix --remove-section=,$(drop-sections))

VMLINUX = vmlinux

all: vmlinux.ecoff vmlinux.srec addinitrd

vmlinux.ecoff: $(obj)/elf2ecoff $(VMLINUX)
	$(obj)/elf2ecoff $(VMLINUX) vmlinux.ecoff $(E2EFLAGS)

$(obj)/elf2ecoff: $(obj)/elf2ecoff.c
	$(HOSTCC) -o $@ $^

vmlinux.bin: $(VMLINUX)
	$(OBJCOPY) -O binary $(strip-flags) $(VMLINUX) $(obj)/vmlinux.bin

vmlinux.srec: $(VMLINUX)
	$(OBJCOPY) -S -O srec $(strip-flags) $(VMLINUX) $(obj)/vmlinux.srec

$(obj)/addinitrd: $(obj)/addinitrd.c
	$(HOSTCC) -o $@ $^

$(obj)/vmlinux.gz: $(obj)/vmlinux.bin
	gzip -c9nf $< > $@

ifdef CONFIG_TANGO2
$(obj)/vmlinux.zbf: $(obj)/vmlinux.gz 
	genzbf -l $(loadaddr) -s 0x`$(NM) $(VMLINUX) | grep kernel_entry | awk '{ print $$1 }'` -a lzf -o $@ $<

$(obj)/xrpc_xload_vmlinux_$(certtype).bin: $(obj)/vmlinux.zbf
	bash $(TOPDIR)/scripts/build_cpu_xrpc_xload.bash $(obj)/vmlinux $(certid) $(certtype)

zbimage-linux-xrpc: $(obj)/zbimage-linux-xrpc
endif

ifdef CONFIG_TANGO3
$(obj)/vmlinux_gz.zbf: $(obj)/vmlinux.gz
	cp -f $< $@

$(obj)/vmlinux_gz_$(certtype).xload: $(obj)/vmlinux_gz.zbf
	bash $(TOPDIR)/scripts/build_cpu_xload.bash $(obj)/vmlinux_gz $(certid) $(certtype)

$(obj)/vmlinux_xload.zbf: $(obj)/vmlinux_gz_$(certtype).xload
	genzbf -l $(loadaddr) -s 0x`$(NM) $(VMLINUX) | grep kernel_entry | awk '{ print $$1 }'` -a lzef -o $@ $<

zbimage-linux-xload: $(obj)/zbimage-linux-xload
endif

ifdef CONFIG_TANGO2
$(obj)/zbimage-linux-xrpc: $(obj)/xrpc_xload_vmlinux_$(certtype).bin
	mkdir romfs
	cp -f $< romfs
	genromfs -V MIPS_LINUX_XRPC -d romfs -f $@
	rm -rf romfs
endif
ifdef CONFIG_TANGO3
$(obj)/zbimage-linux-xload: $(obj)/vmlinux_xload.zbf
	mkdir romfs
	cp -f $< romfs
	genromfs -V MIPSLINUX_XLOAD -d romfs -f $@
	rm -rf romfs
endif

ifdef CONFIG_TANGO2
# The choice of .zbf extension is not really the case, simply make it
# work with the build_cpu_x* script which assumes the .zbf extension.
$(obj)/vmlinux_shax.zbf: $(obj)/vmlinux.gz
	@echo "Using sha1 digest .."
	@tobin.bash `sha1sum -b $< | awk '{ print $$1 }'` > $@

$(obj)/xrpc_xload_vmlinux_shax_$(certtype).bin: $(obj)/vmlinux_shax.zbf
	bash $(TOPDIR)/scripts/build_cpu_xrpc_xload.bash $(obj)/vmlinux_shax $(certid) $(certtype)

$(obj)/vmlinux_enc.bin: $(obj)/xrpc_xload_vmlinux_shax_$(certtype).bin $(obj)/vmlinux.gz
	@cat $^ > $@

$(obj)/vmlinux_enc.zbf: $(obj)/vmlinux_enc.bin 
	genzbf -l $(loadaddr) -s 0x`$(NM) $(VMLINUX) | grep kernel_entry | awk '{ print $$1 }'` -e `stat --format=%s $(obj)/xrpc_xload_vmlinux_shax_$(certtype).bin` -a lzef -o $@ $<

zbimage-linux-enc: $(obj)/zbimage-linux-enc

$(obj)/zbimage-linux-enc: $(obj)/vmlinux_enc.zbf
	mkdir romfs
	cp -f $< romfs
	genromfs -V MIPS_LINUX_ENC -d romfs -f $@
	rm -rf romfs

$(obj)/vmlinux_encs.zbf: $(obj)/vmlinux_enc.bin 
	genzbf -l $(loadaddr) -s 0x`$(NM) $(VMLINUX) | grep kernel_entry | awk '{ print $$1 }'` -e `stat --format=%s $(obj)/xrpc_xload_vmlinux_shax_$(certtype).bin` -a lzes -o $@ $<

zbimage-linux-encs: $(obj)/zbimage-linux-encs

$(obj)/zbimage-linux-encs: $(obj)/vmlinux_encs.zbf
	mkdir romfs
	cp -f $< romfs
	genromfs -V MIPS_LINUX_ENC -d romfs -f $@
	rm -rf romfs
endif

archhelp:
	@echo	'* vmlinux.ecoff	- ECOFF boot image'
	@echo	'* vmlinux.srec		- SREC boot image'

clean-files += addinitrd \
	       elf2ecoff \
	       vmlinux.bin \
	       vmlinux.ecoff \
	       vmlinux.srec
