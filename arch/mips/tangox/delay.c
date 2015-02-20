
/*********************************************************************
 Copyright (C) 2001-2007
 Sigma Designs, Inc. 

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/

/*
 * arch/mips/tangox/delay.c
 *
 * Copyright (C) 2003-2007 Sigma Designs, Inc
 *
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <asm/io.h>
#include <linux/irq.h>
#include <linux/sched.h>

#include "setup.h"

//#define TANGOX_USE_XTAL_UDELAY

#ifdef TANGOX_USE_XTAL_UDELAY
static inline unsigned long tangox_getxtal(void)
{
	return(gbus_read_reg32(REG_BASE_system_block + SYS_xtal_in_cnt));
}

/* This is the replacement of Linux's udelay. */
void tangox_udelay(unsigned usec)
{
	/* SYS_xtal_in_cnt is a counter running off TANGOX_BASE_FREQUENCY */
	unsigned long start = tangox_getxtal();
	unsigned long end = start + (usec * (TANGOX_BASE_FREQUENCY / 1000000));
	unsigned long xtal;

	if (end > start)
		for (xtal = tangox_getxtal(); (xtal < end) && (xtal >= start); xtal = tangox_getxtal());
	else
		for (xtal = tangox_getxtal(); (xtal < end) || (xtal >= start); xtal = tangox_getxtal());
}

void tangox_syncwith_xtal(unsigned long *mark, unsigned usec)
{
	unsigned long end = *mark + (usec * (TANGOX_BASE_FREQUENCY / 1000000));
	unsigned long xtal;

	if (end > *mark)
		for (xtal = tangox_getxtal(); (xtal < end) && (xtal >= *mark); xtal = tangox_getxtal());
	else
		for (xtal = tangox_getxtal(); (xtal < end) || (xtal >= *mark); xtal = tangox_getxtal());
	*mark = end;
}
#else
/* This is the replacement of Linux's udelay. */
void tangox_udelay(unsigned usec)
{
	extern unsigned long em8xxx_cpu_frequency;
	unsigned long start = read_c0_count();
	unsigned long end, cnt, c0_cnt;

			/* CP0_COUNT increase every two CPU clocks */
	cnt = em8xxx_cpu_frequency / (1000000 * 2);
	if ((cnt * (1000000 * 2)) < em8xxx_cpu_frequency)
		cnt++; /* take the ceiling */
	end = start + (usec * cnt);

	if (end > start)
		for (c0_cnt = read_c0_count(); (c0_cnt < end) && (c0_cnt >= start); c0_cnt = read_c0_count());
	else
		for (c0_cnt = read_c0_count(); (c0_cnt < end) || (c0_cnt >= start); c0_cnt = read_c0_count());
}
#endif

EXPORT_SYMBOL(tangox_udelay);

