
/*********************************************************************
 Copyright (C) 2001-2008
 Sigma Designs, Inc. 

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/

/*
 * misc vars/func shared by platform setup code
 */

#ifndef __SETUP_H
#define __SETUP_H

#ifdef CONFIG_TANGO2
#define EM86XX_CHIP EM86XX_CHIPID_TANGO2
#include <linux/interrupt.h>
#include <asm/tango2/rmem86xxid.h>
#include <asm/tango2/rmdefs.h>
#include <asm/tango2/emhwlib_dram.h>
#include <asm/tango2/tango2_gbus.h>
#include <asm/tango2/tango2.h>
#include <asm/tango2/tango2api.h>
#include <asm/tango2/memcfg.h>
#elif defined(CONFIG_TANGO3)
#define EM86XX_CHIP EM86XX_CHIPID_TANGO3
#include <linux/interrupt.h>
#include <asm/tango3/rmem86xxid.h>
#include <asm/tango3/rmdefs.h>
#include <asm/tango3/emhwlib_dram.h>
#include <asm/tango3/tango3_gbus.h>
#include <asm/tango3/tango3.h>
#include <asm/tango3/tango3api.h>
#include <asm/tango3/hardware.h>
#else
#error "Unknown architecture"
#endif

/*
 * in console.c
 */
void prom_console_register(void);

/*
 * in prom.c
 */
extern unsigned long em8xxx_cpu_frequency;
extern unsigned long em8xxx_sys_frequency;
#ifdef CONFIG_TANGO2
extern unsigned long em8xxx_remap_registers[5];
#elif defined(CONFIG_TANGO3)
extern unsigned long em8xxx_remap_registers[8];
#endif

unsigned long tangox_get_cpuclock(void);
unsigned long tangox_get_sysclock(void);

#endif
