
/*********************************************************************
 Copyright (C) 2001-2011 
 Sigma Designs, Inc. 

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/

#ifndef __PLATFORM_DEV_H
#define __PLATFORM_DEV_H

#include <linux/sched.h>
#include <asm/addrspace.h>

#include <asm/tango3/hardware.h>
#include <asm/tango3/tango3_gbus.h>

#define NON_CACHED(x)                   KSEG1ADDR((u32)(x))
#define CACHED(x)                       KSEG0ADDR((u32)(x))

/* SATA */

#define DRV_NAME0    "Tangox SATA 0"
#define DRV_NAME1    "Tangox SATA 1"

#define TANGOX_SATA0_BASE		/*NON_CACHED*/(REG_BASE_host_interface + 0x3000) 
#define TANGOX_SATA1_BASE		/*NON_CACHED*/(REG_BASE_host_interface + 0x3800) 
#define TANGOX_SATA0_CTL_BASE	NON_CACHED(REG_BASE_host_interface + 0x4000) 
#define TANGOX_SATA1_CTL_BASE	NON_CACHED(REG_BASE_host_interface + 0x4040) 
#define TANGOX_SATA_IRQ0		IRQ_CONTROLLER_IRQ_BASE  + LOG2_CPU_SATA_INT
#define TANGOX_SATA_DMA_IRQ0	IRQ_CONTROLLER_IRQ_BASE  + LOG2_CPU_DMASATA_INT
#define TANGOX_SATA_IRQ1		IRQ_CONTROLLER_IRQ_BASE  + LOG2_CPU_SATA1_INT
#define TANGOX_SATA_DMA_IRQ1	IRQ_CONTROLLER_IRQ_BASE  + LOG2_CPU_DMASATA1_INT

/* registers for HOST Cipher */
#define HC_KEY_TABLE_ENTRY0		KSEG1ADDR(REG_BASE_host_interface + 0x5200)
#define HC_KEY_TABLE_ENTRY1		KSEG1ADDR(REG_BASE_host_interface + 0x5204)
#define HC_KEY_TABLE_ENTRY2		KSEG1ADDR(REG_BASE_host_interface + 0x5208)
#define HC_KEY_TABLE_ENTRY3		KSEG1ADDR(REG_BASE_host_interface + 0x520C)
#define HC_KEY_TABLE_ENTRY_CTL_STAT	KSEG1ADDR(REG_BASE_host_interface + 0x5210)
#define HC_OTP_SETTINGS			KSEG1ADDR(REG_BASE_host_interface + 0x5214)
#define HC_CIPHER_CONFIGURATION		KSEG1ADDR(REG_BASE_host_interface + 0x5240)
#define HC_TDES				KSEG1ADDR(REG_BASE_host_interface + 0x5250)
#define HC_AES				KSEG1ADDR(REG_BASE_host_interface + 0x5270)
#define HC_RC4				KSEG1ADDR(REG_BASE_host_interface + 0x52C0)
#define HC_SHA1				KSEG1ADDR(REG_BASE_host_interface + 0x5300)
#define HC_SATA0_AES_CONFIG		KSEG1ADDR(REG_BASE_host_interface + 0x5290)
#define HC_SATA1_AES_CONFIG		KSEG1ADDR(REG_BASE_host_interface + 0x52B0)

/* USB */
#define TANGOX_EHCI0_BASE	(REG_BASE_host_interface + 0x1400)
#define TANGOX_OHCI0_BASE	(REG_BASE_host_interface + 0x1500)
#define TANGOX_CTRL0_BASE	(REG_BASE_host_interface + 0x1700)
#define TANGOX_EHCI0_IRQ	(IRQ_CONTROLLER_IRQ_BASE + LOG2_CPU_USB_EHCI_INT)
#define TANGOX_OHCI0_IRQ	(IRQ_CONTROLLER_IRQ_BASE + LOG2_CPU_USB_OHCI_INT)
#define TANGOX_EHCI1_BASE	(REG_BASE_host_interface + 0x5400)
#define TANGOX_OHCI1_BASE	(REG_BASE_host_interface + 0x5500)
#define TANGOX_CTRL1_BASE	(REG_BASE_host_interface + 0x5700)
#define TANGOX_EHCI1_IRQ	(IRQ_CONTROLLER_IRQ_BASE + LOG2_CPU_USB_EHCI_INT - 1)
#define TANGOX_OHCI1_IRQ	(IRQ_CONTROLLER_IRQ_BASE + LOG2_CPU_USB_OHCI_INT)

/* For 8652/867X OTG host or 8646 host */
#define TANGOX_EHCI_REG_OFFSET		0x100
#define TANGOX_USB_MODE			0x1A8

/* tangox gadget */
#define TANGOX_UDC_NAME0		"tangox_udc_0"
#define TANGOX_UDC_NAME1		"tangox_udc_1"
/* tangox ehci */
#define TANGOX_EHCI_BUS_NAME 		"tangox-ehci-bus"
#define TANGOX_EHCI_PRODUCT_DESC 	"TangoX Integrated USB 2.0"
#define EHCI_HCD_NAME		 	"tangox-ehci-hcd"
/* tangox ohci */
#define OHCI_HCD_NAME		 	"tangox-ohci-hcd"
#define TANGOX_OHCI_BUS_NAME 		"tangox-ohci-bus"

/* for 8670 has two controllers */
static const int tangox_ehci_base[2] = {TANGOX_EHCI0_BASE, TANGOX_EHCI1_BASE};
static const int tangox_ohci_base[2] = {TANGOX_OHCI0_BASE, TANGOX_OHCI1_BASE};
static const int tangox_ctrl_base[2] = {TANGOX_CTRL0_BASE, TANGOX_CTRL1_BASE};
static const int tangox_ehci_irq[2]  = {TANGOX_EHCI0_IRQ, TANGOX_EHCI1_IRQ};
static const int tangox_ohci_irq[2]  = {TANGOX_OHCI0_IRQ, TANGOX_OHCI1_IRQ};
#define TANGOX_EHCI_NAME0	"tangox-ehci-hcd-0"
#define TANGOX_EHCI_NAME1	"tangox-ehci-hcd-1"
//const char* tangox_ehci_name[2] = {TANGOX_EHCI_NAME0, TANGOX_EHCI_NAME1};

static u32 __inline__ tangox_read_reg( u32 Reg )
{
#ifdef CONFIG_TANGOX
        u32 data = gbus_read_reg32(Reg);
#else
        u32 data = __raw_readl(Reg);
#endif

        return data;
}

static void __inline__ tangox_write_reg( u32 Reg, u32 Data )
{
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
extern unsigned long tangox_chip_id(void);
extern int is_tango2_es89(void);
extern int is_tango3_chip(void);
extern void tangox_phy_power_up(int ctrl);
extern void tangox_phy_power_down(int ctrl);
extern void tangox_usb_init(int);
extern void tangox_usb_deinit(int);
#endif
