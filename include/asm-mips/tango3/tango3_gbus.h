
/*********************************************************************
 Copyright (C) 2001-2007
 Sigma Designs, Inc. 

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/

#ifndef __TANGO3_GBUS_H
#define __TANGO3_GBUS_H

#include <linux/config.h>

#ifndef __ASSEMBLY__

#include "rmdefs.h"

#include <asm/addrspace.h>

struct gbus;
#define pGBus ((struct gbus *)1)

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
#endif /* __TANGO3_GBUS_H */

