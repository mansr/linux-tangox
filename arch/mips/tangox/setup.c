/*
 * Copyright 2001 MontaVista Software Inc.
 * Author: Jun Sun, jsun@mvista.com or jsun@junsun.net
 *
 * Copyright (C) 2009 Sigma Designs, Inc.
 * arch/mips/tangox/setup.c
 *     The setup file for tango2/tango3
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/of_platform.h>
#include <linux/of_fdt.h>
#include <asm/bootinfo.h>
#include <asm/reboot.h>
#include <asm/io.h>
#include <asm/time.h>
#include <asm/traps.h>
#include <asm/cpu-info.h>
#include <asm/mipsregs.h>
#include <asm/prom.h>

#include "memmap.h"
#include "setup.h"
#include "uart.h"

void tangox_machine_restart(char *command)
{
	void __iomem *wdog;

        local_irq_disable();

	wdog = ioremap(WATCHDOG_BASE, 8);

	writeb(0x80, wdog + 7);
	writeb(1, wdog + 4);
	writel(2700, wdog);
	writeb(0, wdog + 7);

	while (1)
		cpu_relax();
}

void tangox_machine_halt(void)
{
	while (1)
		cpu_relax();
}

void tangox_machine_power_off(void)
{
	while (1)
		cpu_relax();
}

static void __iomem *xtal_in_cnt;

static cycle_t tangox_read_cycles(struct clocksource *cs)
{
	return readl(xtal_in_cnt);
}

struct clocksource clocksource_tangox = {
	.name		= "TANGOX",
	.rating		= 300,
	.mask		= CLOCKSOURCE_MASK(32),
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
	.read		= tangox_read_cycles,
};

void __init plat_time_init(void)
{
	int ccres = read_c0_hwrena() >> 3 & 1;

	tangox_clk_init();

	mips_hpt_frequency = tangox_get_cpuclock() >> ccres;

	xtal_in_cnt = ioremap(CLOCK_BASE + 0x48, 4);
	clocksource_register_hz(&clocksource_tangox, CONFIG_TANGOX_EXT_CLOCK);
}

static void __init tangox_ebase_setup(void)
{
	ebase = KSEG0ADDR(PHYS_OFFSET);
}

void __init plat_mem_setup(void)
{
	tangox_mem_setup();

	board_ebase_setup = tangox_ebase_setup;

	_machine_restart = tangox_machine_restart;
	_machine_halt = tangox_machine_halt;
	pm_power_off = tangox_machine_power_off;

	ioport_resource.start = 0;
	ioport_resource.end = 0x7fffffff;

	iomem_resource.start = 0;
	iomem_resource.end = 0x7fffffff;

	__dt_setup_arch(__dtb_start);
}

void __init device_tree_init(void)
{
	unflatten_and_copy_device_tree();
}

static struct of_device_id tangox_of_ids[] = {
	{ .compatible = "sigma,smp8640"	},
	{ .compatible = "simple-bus"	},
	{ },
};

static int __init plat_of_setup(void)
{
	if (!of_have_populated_dt())
		panic("device tree not present");

	return of_platform_populate(NULL, tangox_of_ids, NULL, NULL);
}
arch_initcall(plat_of_setup);

#ifdef CONFIG_PROC_FS
static int tangox_proc_cpuinfo(struct notifier_block *np, unsigned long action,
			       void *data)
{
	struct proc_cpuinfo_notifier_args *args = data;
	struct seq_file *m = args->m;

	seq_printf(m, "System clock\t\t: %ld Hz\n", tangox_get_sysclock());
	seq_printf(m, "CPU clock\t\t: %ld Hz\n", tangox_get_cpuclock());
	seq_printf(m, "DSP clock\t\t: %ld Hz\n", tangox_get_dspclock());

	return 0;
}

static int __init tangox_proc_cpuinfo_init(void)
{
	return proc_cpuinfo_notifier(tangox_proc_cpuinfo, 0);
}
subsys_initcall(tangox_proc_cpuinfo_init);
#endif
