
/*********************************************************************
 Copyright (C) 2001-2008
 Sigma Designs, Inc.

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/

#ifndef __TANGOX_SETUP_H
#define __TANGOX_SETUP_H

#include <linux/platform_data/sata-tangox-phy.h>

#ifdef CONFIG_EARLY_PRINTK
void prom_console_init(void);
#else
static inline void prom_console_init(void) { }
#endif

extern unsigned int tangox_chip_type;
extern unsigned int tangox_chip_rev;

static inline int is_tangox_chip(int type, int mask)
{
	return (tangox_chip_type & mask) == type;
}

static inline int is_tangox_chip_rev(int type, int mask, int rev)
{
	return is_tangox_chip(type, mask) && tangox_chip_rev == rev;
}

int tangox_clk_init(void);
unsigned long tangox_get_cpuclock(void);
unsigned long tangox_get_sysclock(void);
unsigned long tangox_get_dspclock(void);

unsigned long tangox_remap_set(int reg, unsigned long addr);
void tangox_remap_init(void);
void tangox_mem_setup(void);

int xenv_config(void);
const char *tangox_xenv_cmdline(void);
int tangox_sata_enabled(void);
int tangox_sata_cfg(struct tangox_sata_phy_pdata *pd);
int tangox_usb_enabled(void);
int tangox_ethernet_enabled(unsigned int i);
int tangox_ethernet_getmac(unsigned int i, unsigned char *addr);
u32 tangox_dram_size(unsigned int idx);
int tangox_uart_console_port(void);
int tangox_uart_baudrate(int uart);

#endif
