
/*********************************************************************
 Copyright (C) 2001-2011
 Sigma Designs, Inc. 

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/usb.h>
#include <linux/export.h>
#if defined(CONFIG_TANGO2)
#include <asm/tango2/platform_dev.h>
#elif defined(CONFIG_TANGO3)
#include <asm/tango3/platform_dev.h>
#elif defined(CONFIG_TANGO4)
#include <asm/tango4/platform_dev.h>
#endif

#undef DEBUG

#ifdef DEBUG
#define DBG(x...) printk(x)
#else
#define DBG(x...)
#endif

unsigned long tangox_otg_bits = 0;
EXPORT_SYMBOL(tangox_otg_bits);

static atomic_t usb_ref_cnt[2] = {ATOMIC_INIT(0), ATOMIC_INIT(0)};

void tangox_phy_power_down(int ctrl)
{
	unsigned long chip_id = (tangox_chip_id() >> 16) & 0xfffe;
	unsigned long temp;

	if ((chip_id == 0x8672) || (chip_id == 0x8674) || 
		((chip_id & 0xfff0) == 0x8680)) {
		/* put phy into suspend mode */
		temp = gbus_read_reg32(tangox_ctrl_base[ctrl] + 0x0);				
		gbus_write_reg32(tangox_ctrl_base[ctrl] + 0x0, temp | (1<<7));
	
		udelay(10);

		/* put phy into sleep mode */
		temp = gbus_read_reg32(tangox_ctrl_base[ctrl] + 0x0);				
		gbus_write_reg32(tangox_ctrl_base[ctrl] + 0x0, temp | (1<<29));
		temp = gbus_read_reg32(tangox_ctrl_base[ctrl] + 0xc);				
		gbus_write_reg32(tangox_ctrl_base[ctrl] + 0xc, temp & ~(1<<29));
	
		/* powers down the analog blocks of the usb PHY */
		temp = gbus_read_reg32(tangox_ctrl_base[ctrl] + 0x10);				
		gbus_write_reg32(tangox_ctrl_base[ctrl] + 0x10, temp | (1<<7));
	} else	if (chip_id == 0x8910) {
		/* control both phy 0 & 1*/
		ctrl = 0; 

		/* put phy into suspend mode */
		temp = gbus_read_reg32(tangox_ctrl_base[ctrl] + 0x0);				
		gbus_write_reg32(tangox_ctrl_base[ctrl] + 0x0, temp | (1<<7));
	
		udelay(10);

		/* put phy into sleep mode */
		temp = gbus_read_reg32(tangox_ctrl_base[ctrl] + 0x0);				
		gbus_write_reg32(tangox_ctrl_base[ctrl] + 0x0, temp | (1<<30));
		
		temp = gbus_read_reg32(tangox_ctrl_base[ctrl] + 0xc);				
		gbus_write_reg32(tangox_ctrl_base[ctrl] + 0xc, temp & ~(1<<29));
		temp = gbus_read_reg32(tangox_ctrl_base[ctrl] + 0x34);				
		gbus_write_reg32(tangox_ctrl_base[ctrl] + 0x34,temp & ~(1<<29));
	
		/* powers down the analog blocks of the usb PHY */
		temp = gbus_read_reg32(tangox_ctrl_base[ctrl] + 0x10);				
		gbus_write_reg32(tangox_ctrl_base[ctrl] + 0x10, temp | (1<<7));
	}
	return;
}
EXPORT_SYMBOL(tangox_phy_power_down);

void tangox_phy_power_up(int ctrl)
{
	unsigned long chip_id = (tangox_chip_id() >> 16) & 0xfffe;
	unsigned long temp;

	if ((chip_id == 0x8672) || (chip_id == 0x8674) ||
		((chip_id & 0xfff0) == 0x8680)) {
		/* Powers up the analog blocks of the usb PHY */
		temp = gbus_read_reg32(tangox_ctrl_base[ctrl] + 0x10);				
		gbus_write_reg32(tangox_ctrl_base[ctrl] + 0x10, temp & ~(1<<7));
	
		/* put phy into active mode */
		temp = gbus_read_reg32(tangox_ctrl_base[ctrl] + 0xc);				
		gbus_write_reg32(tangox_ctrl_base[ctrl] + 0xc, temp | (1<<29));

		udelay(10);

		/*transitions the phy out of suspend mode */
		temp = gbus_read_reg32(tangox_ctrl_base[ctrl] + 0x0);				
		gbus_write_reg32(tangox_ctrl_base[ctrl] + 0x0, temp & ~(1<<7));
	} else	if (chip_id == 0x8910) {
		/* control both phy 0 & 1*/
		ctrl = 0; 

		/* Powers up the analog blocks of the usb PHY */
		temp = gbus_read_reg32(tangox_ctrl_base[ctrl] + 0x10);				
		gbus_write_reg32(tangox_ctrl_base[ctrl] + 0x10, temp & ~(1<<7));
	
		/* put phy into active mode */
		temp = gbus_read_reg32(tangox_ctrl_base[ctrl] + 0xc);				
		gbus_write_reg32(tangox_ctrl_base[ctrl] + 0xc, temp | (1<<29));
		temp = gbus_read_reg32(tangox_ctrl_base[ctrl] + 0x34);				
		gbus_write_reg32(tangox_ctrl_base[ctrl] + 0x34,temp | (1<<29));

		udelay(10);

		/*transitions the phy out of suspend mode */
		temp = gbus_read_reg32(tangox_ctrl_base[ctrl] + 0x0);				
		gbus_write_reg32(tangox_ctrl_base[ctrl] + 0x0, temp & ~(1<<7));
	}
	return;	
}
EXPORT_SYMBOL(tangox_phy_power_up);

void tangox_usb_init(int ctrl)
{
	unsigned long chip_id = (tangox_chip_id() >> 16) & 0xfffe;
	int  i;
	unsigned long temp;
#ifdef CONFIG_TANGOX_XENV_READ
	if (!tangox_usb_enabled())
		return;
#endif
	if (atomic_add_return(1, &usb_ref_cnt[ctrl]) != 1) {
		/* someone is either in the middle of init or have done init */
		while ((gbus_read_reg32(tangox_ctrl_base[ctrl] + 0x0) & (1<<19)) == 0)
			schedule_timeout(HZ / 10);
		printk("TangoX USB was initialized.\n");
		return;
	}
		
	/* Unreset USB block if needed */
	if ((chip_id != 0x8652) && (chip_id != 0x8646) && 
			((chip_id & 0xfff0) != 0x8670) && (chip_id != 0x8910)) {
		/* reset phy software reset */
		temp = gbus_read_reg32(tangox_ctrl_base[ctrl] + 0x0);				
		gbus_write_reg32(tangox_ctrl_base[ctrl] + 0x0, temp & ~(1<<0));

		/* delay at least 1 ms*/
		mdelay(2);

		/* reset host software reset bit, seems didnot documented*/
		temp = gbus_read_reg32(tangox_ctrl_base[ctrl] + 0x0);				
		gbus_write_reg32(tangox_ctrl_base[ctrl] + 0x0, temp & ~(1<<1));

		/* put phy in power normal mode */
		temp = gbus_read_reg32(tangox_ctrl_base[ctrl] + 0xc);				
		gbus_write_reg32(tangox_ctrl_base[ctrl] + 0xc, temp & ~(1<<6));
	}

	/*check see if it's inited*/
	temp = gbus_read_reg32(tangox_ctrl_base[ctrl] + 0x0);
	if (temp & (1<<19)) {
		printk("TangoX USB was initialized.\n");
		return;
	}
	else
		printk("TangoX USB initializing...\n");

	tangox_phy_power_up(ctrl);

	/*
	1. Program the clean divider and clock multiplexer to provide 
	   a 48 MHz reference to the USB block.
	   This is done in bootloader.
	*/

	if ((chip_id == 0x8652) || (chip_id == 0x8646)) { 
		/*0. set bit 1 of USB control register 0x21700*/ 
		temp = gbus_read_reg32(tangox_ctrl_base[ctrl] + 0x0);
		gbus_write_reg32(tangox_ctrl_base[ctrl] + 0x0, temp | 0x2);
       		wait_ms(5);
		 
		/* 1. Program the clean divider and clock multiplexors to provide 48MHz clock reference*/
		/* this is to be done in zboot */

		/* 2. Enable the USB PHY */
       		temp = gbus_read_reg32(tangox_ctrl_base[ctrl] + 0x0);
		gbus_write_reg32(tangox_ctrl_base[ctrl] + 0x0, temp & 0xffffff7e);
		wait_ms(20);

		/* 3. Enable the USB Host EHCI/OHCI */
		temp = gbus_read_reg32(tangox_ctrl_base[ctrl] + 0x0);
		gbus_write_reg32(tangox_ctrl_base[ctrl] + 0x0, temp & 0xfffffffd);
		wait_ms(20);

		/* 4. set it to host mode*/
		temp = gbus_read_reg32(tangox_ehci_base[ctrl] + TANGOX_EHCI_REG_OFFSET +0xA8);
		temp |= 3 ;
		gbus_write_reg32(tangox_ehci_base[ctrl] + TANGOX_EHCI_REG_OFFSET +0xA8, temp);
		wait_ms(20);
	} else if ((chip_id & 0xfff0) == 0x8670) {
		/* 0. Program the clean divider and clock multiplexors to provide 48MHz clock reference*/
		/* this is to be done in zboot */

		/*1. program the phy analog ctrl reg*/ 
		gbus_write_reg32(tangox_ctrl_base[ctrl] + 0x2c, 0x0020DB91);
		//gbus_write_reg32(tangox_ctrl_base[ctrl] + 0x2c, 0x003CDB91);
		wait_ms(5);

		/*2. program the phy clock reference frequency*/ 
		gbus_write_reg32(tangox_ctrl_base[ctrl] + 0xc, 0x000f9930);
		wait_ms(5);
		 
		/* 3. Releasing USB PHY */
		gbus_write_reg32(tangox_ctrl_base[ctrl] + 0x0, 0x00000002);
		wait_ms(5);

		/* 4. Releasing the Host Controller */
		//PHY force unreset
		gbus_write_reg32(tangox_ctrl_base[ctrl] + 0x0, 0x10000002);
		wait_ms(5);
		gbus_write_reg32(tangox_ctrl_base[ctrl] + 0x0, 0x10000000);
		wait_ms(5);
		gbus_write_reg32(tangox_ctrl_base[ctrl] + 0x0, 0x00000000);
		wait_ms(5);

		/*5. Applying a 2nd HOST reset, after the NanoPHY 60MHz clock has become action */
		wait_ms(5);
		gbus_write_reg32(tangox_ctrl_base[ctrl] + 0x0, 0x10000002);
		wait_ms(5);
		gbus_write_reg32(tangox_ctrl_base[ctrl] + 0x0, 0x10000000);
		wait_ms(5);
		//Remove the PHY force unreset 
		gbus_write_reg32(tangox_ctrl_base[ctrl] + 0x0, 0x00000000);
		wait_ms(5);
#if 0
		for(i = 0; i < 5; i++) {
			temp = gbus_read_reg32(tangox_ctrl_base[ctrl] + i*4);
			printk("TangoX USB register base= 0x%x %d = 0x%x\n",tangox_ctrl_base[ctrl], i, (u32)temp);
		}
#endif
		/* 6. set it to host mode*/
		temp = gbus_read_reg32(tangox_ehci_base[ctrl] + TANGOX_EHCI_REG_OFFSET +0xA8);
		temp |= 3 ;
		gbus_write_reg32(tangox_ehci_base[ctrl] + TANGOX_EHCI_REG_OFFSET +0xA8, temp);
		wait_ms(20);

	} else if (chip_id == 0x8910) {
		printk("initialize 8910 USB Host...\n");	
		/* 0. Program the clean divider and clock multiplexors to provide 48MHz clock reference*/
		/* this is to be done in zboot */

		/*1. program the phy analog ctrl reg*/ 
		gbus_write_reg32(tangox_ctrl_base[ctrl] + 0x2C, 0x0020DB91);
		gbus_write_reg32(tangox_ctrl_base[ctrl] + 0x30, 0x0020DB91);
		wait_ms(5);

		/*2. program the phy clock reference frequency*/ 
		gbus_write_reg32(tangox_ctrl_base[ctrl] + 0xC, 0x000f9930);
		wait_ms(5);
#if 0 
		/* 3. Releasing USB PHY */
		gbus_write_reg32(tangox_ctrl_base[ctrl] + 0xC, 0x000f9930);
		gbus_write_reg32(tangox_ctrl_base[ctrl] + 0x34, 0x20030003);
		wait_ms(5);
#endif
		/* 4. Releasing the Host Controller */
		//PHY force unreset
		gbus_write_reg32(tangox_ctrl_base[ctrl] + 0x0, 0x00000002);
		wait_ms(5);
		gbus_write_reg32(tangox_ctrl_base[ctrl] + 0x0, 0x00000000);
		wait_ms(5);
#if 0
		for(i = 0; i < 4; i++) {
			temp = gbus_read_reg32(tangox_ctrl_base[ctrl] + i*4);
			printk("TangoX USB register base= 0x%x %d = 0x%x\n",tangox_ctrl_base[ctrl], i, (u32)temp);
		}
#endif
		/* 6. set it to host mode*/
		temp = gbus_read_reg32(tangox_ehci_base[ctrl] + TANGOX_EHCI_REG_OFFSET +0xA8);
		temp |= 3 ;
		gbus_write_reg32(tangox_ehci_base[ctrl] + TANGOX_EHCI_REG_OFFSET +0xA8, temp);
		wait_ms(20);

	}
#ifndef CONFIG_TANGO4
	else {
#if 0		/* If you want to use external crystal at 24MHZ */
		printk("TangoX USB using 24MHz external crystal.\n");
		gbus_write_reg32(REG_BASE_system_block + SYS_hostclk_mux, 0x300);
		gbus_write_reg32(tangox_ctrl_base[ctrl] + 0x0, 0x70);
		wait_ms(5);
		gbus_write_reg32(tangox_ctrl_base[ctrl] + 0xc, 0xf9931);
		wait_ms(30);
#endif
		/*2. PHY software reset*/
		DBG("Performing PHY Reseting...\n");
		temp = gbus_read_reg32(tangox_ctrl_base[ctrl] + 0x0);
		gbus_write_reg32(tangox_ctrl_base[ctrl] + 0x0, temp | 0x01);
		udelay (30);
		gbus_write_reg32(tangox_ctrl_base[ctrl] + 0x0, temp);
		wait_ms(5);

		/*3. Reset Bit 1 of USB register 0x21700 to enable the USB Host controller.
			This is done in bootloader */

		/*4. OHCI Software reset*/
		DBG("Performing USB OHCI Reseting...\n");
		temp = gbus_read_reg32(tangox_ohci_base[ctrl] + 0x08);
		gbus_write_reg32(tangox_ohci_base[ctrl] + 0x08,  temp | 0x01);
		wait_ms(5);

		/*5. OHCI DPLL Software reset, it says the bit is for simulation*/
		DBG("Performing USB OHCI DPLL Reseting...\n");
		temp = gbus_read_reg32(tangox_ctrl_base[ctrl] + 0x0);
		gbus_write_reg32(tangox_ctrl_base[ctrl] + 0x0, temp | (1<<19));
		wait_ms(5);
	
		/*6. EHCI Host Software Reset*/
		DBG("Performing USB EHCI Reseting...\n");
		temp = gbus_read_reg32(tangox_ehci_base[ctrl] + 0x10);
		gbus_write_reg32(tangox_ehci_base[ctrl] + 0x10,  temp | 0x02);
		wait_ms(5);

		for(i = 0; i < 4; i++){
			temp = gbus_read_reg32(tangox_ctrl_base[ctrl] + i*4);
			DBG("TangoX USB register %d = 0x%x\n", i, temp);
		}
	}
#endif
	/* increase gbus bandwidth for USB hosts */
	gbus_write_reg32(REG_BASE_system_block + 0x138,
		(gbus_read_reg32(REG_BASE_system_block + 0x138) & 0xffffff00) | 0x3f);

	return;
}

EXPORT_SYMBOL(tangox_usb_init);

void tangox_usb_deinit(int ctrl)
{
	unsigned long chip_id = (tangox_chip_id() >> 16) & 0xfffe;
	if (atomic_sub_return(1, &usb_ref_cnt[ctrl]) == 0) {
		/* Reset USB block if needed */
		if ((chip_id != 0x8652) && (chip_id != 0x8646) && 
				((chip_id & 0xfff0) != 0x8670) && (chip_id != 0x8910)) { 
			unsigned long temp;
			/* put phy in power saving mode */
			temp = gbus_read_reg32(tangox_ctrl_base[ctrl] + 0xc);				
			gbus_write_reg32(tangox_ctrl_base[ctrl] + 0xc, temp | (1<<6));

			/* set host software reset bit 1, seems didnot documented*/
			temp = gbus_read_reg32(tangox_ctrl_base[ctrl] + 0x0);				
			gbus_write_reg32(tangox_ctrl_base[ctrl] + 0x0, temp | (1<<1));

			/* delay at least 1 ms*/
			mdelay(2);

			/* set phy software reset bit */
			temp = gbus_read_reg32(tangox_ctrl_base[ctrl] + 0x0);				
			gbus_write_reg32(tangox_ctrl_base[ctrl] + 0x0, temp | (1<<0));
		}

		tangox_phy_power_down(ctrl);
	} 
}
EXPORT_SYMBOL(tangox_usb_deinit);


