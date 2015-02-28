/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2003 Ralf Baechle
 */
#ifndef __ASM_MACH_TANGO3_CPU_FEATURE_OVERRIDES_H
#define __ASM_MACH_TANGO3_CPU_FEATURE_OVERRIDES_H

#define cpu_has_tlb		1
#define cpu_has_tlbinv		0
#define cpu_has_segments	0
#define cpu_has_eva		0
#define cpu_has_htw		0
#define cpu_has_rixiex		0
#define cpu_has_maar		0

#define cpu_has_4kex		1
#define cpu_has_3k_cache	0
#define cpu_has_4k_cache	1
#define cpu_has_tx39_cache	0
#define cpu_has_counter		1
#define cpu_has_watch		1
#define cpu_has_divec		1
#define cpu_has_vce		0
#define cpu_has_cache_cdex_p	0
#define cpu_has_cache_cdex_s	0
#define cpu_has_prefetch	1
#define cpu_has_mcheck		1
#define cpu_has_ejtag		1
#define cpu_has_llsc		1

#define cpu_has_mips16		1
#define cpu_has_mdmx		0
#define cpu_has_mips3d		0
#define cpu_has_smartmips	0
#define cpu_has_mipsmt		0

#define cpu_has_mips_2		1
#define cpu_has_mips_3		0
#define cpu_has_mips_4		0
#define cpu_has_mips_5		0
#define cpu_has_mips32r1	1
#define cpu_has_mips32r2	1
#define cpu_has_mips64r1	0
#define cpu_has_mips64r2	0

#define cpu_has_nofpuex		1
#define cpu_has_64bits		0
#define cpu_has_64bit_zero_reg	0

#ifdef CONFIG_TANGOX_CPU

#define cpu_has_fpu		1
#define cpu_has_32fpr		1
#define cpu_has_dsp		1
#define cpu_has_dsp2		1

#define cpu_dcache_line_size()	32
#define cpu_icache_line_size()	32

#else

#define cpu_has_fpu		0
#define cpu_has_32fpr		0
#define cpu_has_dsp		0
#define cpu_has_dsp2		0

#define cpu_dcache_line_size()	16
#define cpu_icache_line_size()	16

#endif

#endif /* __ASM_MACH_TANGO3_CPU_FEATURE_OVERRIDES_H */
