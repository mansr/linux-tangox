
/*********************************************************************
 Copyright (C) 2001-2007
 Sigma Designs, Inc. 

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/

/**
  @file  cpucache.c
  @brief  

  Implementing /proc/cpucache_info interface

  @author YH Lin
  @date   2006-08-16
*/

#include <linux/jiffies.h>
#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/proc_fs.h>

#ifdef CONFIG_TANGO2
#include <asm/tango2/hardware.h>
#include <asm/tango2/tango2api.h>
#include <asm/tango2/tango2_gbus.h>
#elif defined(CONFIG_TANGO3)
#include <asm/tango3/hardware.h>
#include <asm/tango3/tango3api.h>
#include <asm/tango3/tango3_gbus.h>
#endif

static int cpucache_info_read_proc(char *buf, char **start, off_t off, int count, int *eof, void *data);
static unsigned long cpucache_saved_jiffies = 0;
static unsigned icache_missed = 0, dcache_missed = 0;

extern unsigned long em8xxx_cpu_frequency;

//
// init
//
int __init cpucache_info_init(void)
{
	create_proc_read_entry("cpucache_info", S_IRUGO, NULL, cpucache_info_read_proc, NULL);

        printk("Created /proc/cpucache_info entry.\n");

	/* Enable performance monitor and start counting */
	/* 0: DCache Hit 
	 * 1: DCache Miss
	 * 2: TLB Hit
	 * 3: TLB Miss
	 * 4: ICache Hit
	 * 5: ICache Miss
	 * 6: Instn Complete
	 * 7: ITLB Hit
	 * 8: ITLB Miss
	 * 9: JTLB Hit
	 * 10: JTLB Miss
	 * 11: WTB Merge */
	gbus_write_reg32(REG_BASE_cpu_block + CPU_pm_select_1, (1<<5));
	gbus_write_reg32(REG_BASE_cpu_block + CPU_pm_select_0, (1<<1));
	cpucache_saved_jiffies = jiffies;
	icache_missed = gbus_read_reg32(REG_BASE_cpu_block + CPU_pm_counter_1);
	dcache_missed = gbus_read_reg32(REG_BASE_cpu_block + CPU_pm_counter_0);
	return 0;
}

__initcall(cpucache_info_init);

static int cpucache_info_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
        int len = 0;
	unsigned long diffj, diffi, diffd, icache_miss, dcache_miss, tmpj;
	unsigned long pd, pi;
        
	icache_miss = gbus_read_reg32(REG_BASE_cpu_block + CPU_pm_counter_1);
	dcache_miss = gbus_read_reg32(REG_BASE_cpu_block + CPU_pm_counter_0);
	tmpj = jiffies;
	diffj = tmpj - cpucache_saved_jiffies; 
	cpucache_saved_jiffies = tmpj;
	if (diffj == 0)
		diffj = 1;

	if (icache_miss < icache_missed)
		diffi = (0xffffffff - icache_missed) + icache_miss;
	else
		diffi = icache_miss - icache_missed;
	icache_missed = icache_miss;

	if (dcache_miss < dcache_missed)
		diffd = (0xffffffff - dcache_missed) + dcache_miss;
	else
		diffd = dcache_miss - dcache_missed;
	dcache_missed = dcache_miss;

	len += sprintf(page+off+len, "jiffies");
	/* The average miss per jiffies is (diffx / diffj), per second is then ((diffx * HZ) / diffj).
	 * Assuming one instruction per clock, we have (((diffx * HZ) / diffj) / em8xxx_cpu_frequency)
	 * as average ratio of cache miss per instruction at given period of time (from last 
	 * measured, as expressed as (diffj / HZ)). */
	pi = ((diffi * HZ) / diffj) / (em8xxx_cpu_frequency / 10000); /* Times 10000 to get percentage */
	pd = ((diffd * HZ) / diffj) / (em8xxx_cpu_frequency / 10000);

	len += sprintf(page+off+len, "(%ld): I-Cache miss ratio(%ld): %ld.%02ld%c, D-Cache miss ratio(%ld): %ld.%02ld%c\n",
			diffj, diffi, pi / 100, pi % 100, '%', diffd, pd / 100, pd % 100, '%');
	*eof = 1;

        return(len);
}

