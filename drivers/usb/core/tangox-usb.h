
/*********************************************************************
 Copyright (C) 2001-2008
 Sigma Designs, Inc. 

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/

#ifndef __TANGOX_USB_H
#define __TANGOX_USB_H
 
#include <asm/addrspace.h>

#ifdef CONFIG_TANGO2
#include <asm/tango2/hardware.h>
#include <asm/tango2/tango2_gbus.h>
#elif defined(CONFIG_TANGO3)
#include <asm/tango3/hardware.h>
#include <asm/tango3/tango3_gbus.h>
#endif

#define NON_CACHED(x)                   KSEG1ADDR((u32)(x))
#define CACHED(x)                       KSEG0ADDR((u32)(x))

#define TANGOX_EHCI_BASE_ADDR           /*NON_CACHED*/(REG_BASE_host_interface + 0x1400)
#define TANGOX_OHCI_BASE_ADDR           /*NON_CACHED*/(REG_BASE_host_interface + 0x1500)
#define TANGOX_USB_CTL_STATUS_REG_BASE  /*NON_CACHED*/(REG_BASE_host_interface + 0x1700)
#define TANGOX_EHCI_IRQ                 IRQ_CONTROLLER_IRQ_BASE + LOG2_CPU_USB_EHCI_INT
#define TANGOX_OHCI_IRQ                 IRQ_CONTROLLER_IRQ_BASE + LOG2_CPU_USB_OHCI_INT

/* For 8652 OTG host */
#define TANGOX_EHCI_REG_OFFSET		0x100
#define USBMODE				0x1A8

/* tangox ehci */
#define TANGOX_EHCI_BUS_NAME 		"tangox-ehci-bus"
#define TANGOX_EHCI_PRODUCT_DESC 	"TangoX Integrated USB 2.0"
#define EHCI_HCD_NAME		 		"tangox-ehci-hcd"
/* tangox ohci */
#define OHCI_HCD_NAME		 		"tangox-ohci-hcd"
#define TANGOX_OHCI_BUS_NAME 		"tangox-ohci-bus"


static u32 __inline__ tangox_read_reg( u32 Reg )
{
#ifdef CONFIG_TANGOX
        u32 data = gbus_read_reg32(Reg);
#else
        u32 data = __raw_readl(Reg);
#endif

//        printk("read data=0x%08x from addr=0x%08x\n", data, Reg);
        return data;
}

static void __inline__ tangox_write_reg( u32 Reg, u32 Data )
{
//       printk("write data=0x%08x to addr=0x%08x\n", Data, Reg);
#ifdef CONFIG_TANGOX
        gbus_write_reg32(Reg, Data);
#else
        __raw_writel(Data,Reg);
#endif
}

static __inline__ void wait_ms(unsigned int ms)
{
        if(!in_interrupt()) {
                current->state = TASK_UNINTERRUPTIBLE;
                schedule_timeout(1 + ms * HZ / 1000);
        }
        else
                mdelay(ms);
}
#ifdef CONFIG_TANGOX_XENV_READ
extern int tangox_usb_enabled(void);
#endif
extern int is_tango2_es89(void);
extern int is_tango3_chip(void);
extern void tangox_usb_init (void);
#endif
