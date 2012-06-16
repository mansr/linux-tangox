/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2011 Sigma Designs, Inc.
 * Copyright (C) 1994 - 1999, 2000, 03, 04 Ralf Baechle
 * Copyright (C) 2000, 2002  Maciej W. Rozycki
 * Copyright (C) 1990, 1999, 2000 Silicon Graphics, Inc.
 */
#ifndef _ASM_MACH_TANGO2_SPACES_H
#define _ASM_MACH_TANGO2_SPACES_H

#define PHYS_OFFSET             0x00000000UL
#define CAC_BASE		0x80000000UL
#define IO_BASE			0xa0000000UL
#define UNCAC_BASE		0xa0000000UL
#define MAP_BASE		0xc0000000UL

/*
 * This handles the memory map.
 * We handle pages at KSEG0 for kernels with 32 bit address space.
 */
#define PAGE_OFFSET		0x80000000UL

/*
 * Memory above this physical address will be considered highmem.
 */
#ifndef HIGHMEM_START
#define HIGHMEM_START		0x20000000UL
#endif

#ifndef FIXADDR_TOP
#define FIXADDR_TOP		((unsigned long)(long)(int)0xfffe0000)
#endif

#endif /* __ASM_MACH_TANGO2_SPACES_H */
