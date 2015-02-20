/*********************************************************************
 Copyright (C) 2001-2007
 Sigma Designs, Inc. 

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/

#ifndef __LINUX_PLATFORM_H__
#define __LINUX_PLATFORM_H__

#include <linux/config.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <asm/tlbflush.h>
#include <asm/system.h>

#include <asm/tango2/rmem86xxid.h>
#include <asm/tango2/rmdefs.h>
#include <asm/tango2/emhwlib_dram.h>
#include <asm/tango2/tango2_gbus.h>
#include <asm/tango2/tango2.h>
#include <asm/tango2/tango2api.h>
#include <asm/tango2/memcfg.h>
#include <asm/tango2/emhwlib_registers_tango2.h>

/* Flush all cache */
extern void tangox_flush_cache_all(void);

/* Reset OS timer */
extern void reset_timer(unsigned long cpu_freq, unsigned long sys_freq);

/* Get CPU frequency */
extern unsigned long tangox_get_cpuclock(void);

/* Get SYS frequency */
extern unsigned long tangox_get_sysclock(void);

/* Get DSP frequency */
extern unsigned long tangox_get_dspclock(void);

/* Devices */
extern int tangox_ir_enabled(void);
extern int tangox_fip_enabled(void);
extern int tangox_ethernet_enabled(void);

static inline void tangox_gbus_write_reg32(unsigned long addr, unsigned long val)
{
	gbus_write_reg32(addr, val);
}

static inline unsigned long tangox_gbus_read_reg32(unsigned long addr)
{
	return gbus_read_reg32(addr);
}

/* Sleep for given micro-seconds */
static inline void tangox_usdelay(unsigned long usec)
{
	udelay(usec);
}

/* Re-calibrate OS level timer */
static inline void tangox_reset_timer(unsigned long cpufreq, unsigned long sysfreq)
{
	reset_timer(cpufreq, sysfreq);
}

/* Disable interrupts and save the flags */
#define tangox_save_flags(x)      local_irq_save(x)

/* Restore the flags (re-enable interrupt) */
#define tangox_restore_flags(x)   local_irq_restore(x)

/* IO barrier */
#define tangox_iob()		  iob()

/* sync */
#define tangox_sync()		  __sync()

/* For debugging purpose */
#ifdef FC_DEBUG
#define DBG_PRINT           printk
#else
#define DBG_PRINT(x, ...)
#endif

#endif

