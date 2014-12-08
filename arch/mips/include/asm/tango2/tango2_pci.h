
/*********************************************************************
 Copyright (C) 2001-2007
 Sigma Designs, Inc. 

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/

#ifndef __TANGO2_PCI_H
#define __TANGO2_PCI_H

#include <asm/tango2/hardware.h>
#include <asm/tango2/tango2_gbus.h>

#define PCIEM86XX_PCIFAULT_INTR		((IRQ_CONTROLLER_IRQ_BASE) + \
						(LOG2_CPU_PCI_FAULT_INT))

#define PCIEM86XX_ARBITER_LEVEL		0x00
#define PCIEM86XX_ARBITER_GRANTTIMEOUT	0x10101010

#define PCIEM86XX_IDSEL_MAX		0x05
#define PCIEM86XX_IDSEL_BITS		0x03

/*
 * helpers to access host interface registers
 */
#define RD_HOST_REG32(r)	\
		gbus_read_reg32(REG_BASE_host_interface + (r))

#define WR_HOST_REG32(r, v)	\
		gbus_write_reg32(REG_BASE_host_interface + (r), (v))

#define RD_HOST_REG8(r)	\
		gbus_read_reg8(REG_BASE_host_interface + (r))

#define WR_HOST_REG8(r, v)	\
		gbus_write_reg8(REG_BASE_host_interface + (r), (v))


/*
 * read/write data  from/to the configuration space.  The  only way to
 * know if there is a device is by checking that we did not generate a
 * bus fault interrupt, so we need to mask them during the access.
 */
#define BUILD_TANGOX_CFG_ACCESS(pfx, __x)				\
static inline int tangox_cfg_##pfx(void __iomem *addr, u32 *data)	\
{									\
	unsigned long flags, status;					\
									\
	local_irq_save(flags);						\
									\
	__x;								\
	status = (RD_HOST_REG8(PCI_host_reg2 + 3) >> 1) & 0x3;		\
	if (status) {							\
		WR_HOST_REG8(PCI_host_reg2 + 3, 1);			\
		WR_HOST_REG8(PCI_host_reg2 + 3, 0);			\
		local_irq_restore(flags);				\
		return 1;						\
	}								\
									\
	local_irq_restore(flags);					\
	return 0;							\
}

BUILD_TANGOX_CFG_ACCESS(read8, *data = readb(addr) & 0xff)
BUILD_TANGOX_CFG_ACCESS(read16, *data = readw(addr) & 0xffff)
BUILD_TANGOX_CFG_ACCESS(read32, *data = readl(addr))
BUILD_TANGOX_CFG_ACCESS(write8, writeb(*data & 0xff, addr))
BUILD_TANGOX_CFG_ACCESS(write16, writew(*data & 0xffff, addr))
BUILD_TANGOX_CFG_ACCESS(write32, writel(*data, addr))

#endif

