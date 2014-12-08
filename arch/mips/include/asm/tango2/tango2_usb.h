
/*********************************************************************
 Copyright (C) 2001-2007
 Sigma Designs, Inc. 

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/

#ifndef __TANGO2_USB_H
#define __TANGO2_USB_H

#include <asm/tango2/hardware.h>
#include <asm/tango2/tango2_gbus.h>

#define TANGOX_EHCI_IRQ	(IRQ_CONTROLLER_IRQ_BASE + LOG2_CPU_USB_EHCI_INT)
#define TANGOX_OHCI_IRQ	(IRQ_CONTROLLER_IRQ_BASE + LOG2_CPU_USB_OHCI_INT)

#define TANGOX_EHCI_BASE_ADDR		(REG_BASE_host_interface + 0x1400)
#define TANGOX_OHCI_BASE_ADDR		(REG_BASE_host_interface + 0x1500)
#define TANGOX_USB_CTL_STATUS_REG_BASE	(REG_BASE_host_interface + 0x1700)

/*
 * helpers to access USB registers
 */
#define RD_OHCI_REG32(r)	\
		gbus_read_reg32(TANGOX_OHCI_BASE_ADDR + (r))

#define WR_OHCI_REG32(r, v)	\
		gbus_write_reg32(TANGOX_OHCI_BASE_ADDR + (r), (v))

#define RD_USB_REG32(r)	\
		gbus_read_reg32(TANGOX_USB_CTL_STATUS_REG_BASE + (r))

#define WR_USB_REG32(r, v)	\
		gbus_write_reg32(TANGOX_USB_CTL_STATUS_REG_BASE + (r), (v))

#endif
