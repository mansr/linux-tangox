
/*********************************************************************
 Copyright (C) 2001-2007
 Sigma Designs, Inc. 

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/

/*
  Refer to bug #3644.

  TLB-based implementation works on the ranges:
  [0x00000000..0x20000000[ access thru kseg1
  [0x20000000..0x2xxxxxxx (size  of dram1, a  parameter of ioremap())[
  access thru tlb. Outside: unpredictable/oops.

  Remap-based implementation does:
  00xy b27b26..b0 to 101y(b27|x)b26..b0.
  and works  everywhere excepted ranges:  [0x18000000..0x20000000[ and
  [0x28000000..0xffffffff]

  Test with
  {
	volatile int q=gbus_read_uint32(pGBus,0x1020212c); // correct
	q=gbus_read_uint32(pGBus,0x1f20212c);              // incorrect
	q=gbus_read_uint32(pGBus,0x2020212c);              // correct
	q=gbus_read_uint32(pGBus,0x2720212c);              // correct
	q=gbus_read_uint32(pGBus,0x2820212c);              // incorrect
	q=gbus_read_uint32(pGBus,0x2f20212c);              // incorrect
  }
*/

#ifndef __TANGO2_GBUS_H
#define __TANGO2_GBUS_H

#include <linux/config.h>

#ifndef __ASSEMBLY__

#include "rmdefs.h"

#include <asm/addrspace.h>

struct gbus;
#define pGBus ((struct gbus *)1)

#ifdef CONFIG_TANGOX_USE_TLB_REMAP_DRAM1

__asm__ (
	"	.macro gbus_swizzle_addr res tmp addr			\n"
	"	ext	\\res, \\addr, 29, 1				\n"
	"	bnez	\\res, 1f					\n"
	"	lui	\\tmp, 0xa000					\n"
	"	or	\\res, \\tmp, \\addr				\n"
	"	j	2f						\n"
	"	nop							\n"
	"1:								\n"
	"	lui	\\tmp, 0x2000					\n"
	"	sub	\\res, \\addr, \\tmp				\n"
	"	lw	\\tmp, em86xx_tlb_dram1_map_base		\n"
	"	add	\\res, \\tmp					\n"
	"2:								\n"
	"	.endm");

#else

__asm__ (
	"	.macro gbus_swizzle_addr res tmp addr			\n"
	"	rotr	\\res, \\addr, 29				\n"
	"	ins	\\res, \\res, 30, 1				\n"
	"	or	\\res, 5					\n"
	"	rotr	\\res, 3					\n"
	"	.endm");
#endif


/*
 * we just want to set kseg1 bit, most of the time address is known at
 * compile time, so this will usually be reduced to 2 instructions
 */

#define BUILD_GBUS_READ(size)						\
static inline RMuint32 gbus_read_dram_uint##size(struct gbus *pgbus,	\
					    RMuint32 byte_address)	\
{									\
	if (__builtin_constant_p(byte_address)) {			\
		if ((byte_address & 0x70000000) == 0x20000000) {	\
			byte_address &= ~0x20000000;			\
			byte_address |= 0x08000000;			\
		}							\
		return *((volatile RMuint##size *)KSEG1ADDR(byte_address)); \
	} else {							\
		RMuint32 res, tmp;					\
									\
		__asm__ __volatile(					\
			"gbus_swizzle_addr\t%0 %1 %2\n"			\
			: "=&r" (res), "=&r" (tmp) : "r" (byte_address)); \
		return *((volatile RMuint##size *)res);			\
	}								\
}

BUILD_GBUS_READ(8);
BUILD_GBUS_READ(16);
BUILD_GBUS_READ(32);

#define BUILD_GBUS_WRITE(size)						\
static inline void gbus_write_dram_uint##size(struct gbus *pgbus,	\
				     RMuint32 byte_address,		\
				     RMuint##size data)			\
{									\
	if (__builtin_constant_p(byte_address)) {			\
		if ((byte_address & 0x70000000) == 0x20000000) {	\
			byte_address &= ~0x20000000;			\
			byte_address |= 0x08000000;			\
		}							\
		*((volatile RMuint##size *)KSEG1ADDR(byte_address)) = data; \
	} else {							\
		RMuint32 res, tmp;					\
									\
		__asm__ __volatile(					\
			"gbus_swizzle_addr\t%0 %1 %2\n"			\
			: "=&r" (res), "=&r" (tmp) : "r" (byte_address)); \
		*((volatile RMuint##size *)res) = data;			\
	}								\
}

BUILD_GBUS_WRITE(8);
BUILD_GBUS_WRITE(16);
BUILD_GBUS_WRITE(32);

RMuint32 gbus_read_uint32(struct gbus *pgbus, RMuint32 byte_address);
RMuint16 gbus_read_uint16(struct gbus *pgbus, RMuint32 byte_address);
RMuint8 gbus_read_uint8(struct gbus *pgbus, RMuint32 byte_address);
void gbus_write_uint32(struct gbus *pgbus, RMuint32 byte_address, RMuint32 data);
void gbus_write_uint16(struct gbus *pgbus, RMuint32 byte_address, RMuint16 data);
void gbus_write_uint8(struct gbus *pgbus, RMuint32 byte_address, RMuint8 data);

#define gbus_read_reg32(r)      __raw_readl((volatile void __iomem *)KSEG1ADDR(r))
#define gbus_read_reg16(r)      __raw_readw((volatile void __iomem *)KSEG1ADDR(r))
#define gbus_read_reg8(r)       __raw_readb((volatile void __iomem *)KSEG1ADDR(r))
#define gbus_write_reg32(r, v)  __raw_writel(v, (volatile void __iomem *)KSEG1ADDR(r))
#define gbus_write_reg16(r, v)  __raw_writew(v, (volatile void __iomem *)KSEG1ADDR(r))
#define gbus_write_reg8(r, v)   __raw_writeb(v, (volatile void __iomem *)KSEG1ADDR(r))

#endif /* !__ASSEMBLY__ */
#endif /* __TANGO2_GBUS_H */

