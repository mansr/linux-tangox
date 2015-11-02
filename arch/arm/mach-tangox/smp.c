/*
 *  Based on arch/arm/{mach-vexpress,plat-versatile}/platsmp.c
 *
 *  Copyright (C) 2002 ARM Ltd.
 *  All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/of_address.h>
#include <linux/smp.h>

#include <asm/cacheflush.h>
#include <asm/smp_plat.h>
#include <asm/smp_scu.h>

#include "smc.h"

extern void tangox_secondary_startup(void);

/*
 * Write pen_release in a way that is guaranteed to be visible to all
 * observers, irrespective of whether they're taking part in coherency
 * or not.  This is necessary for the hotplug code to work reliably.
 */
static void write_pen_release(int val)
{
	pen_release = val;
	smp_wmb();
	sync_cache_w(&pen_release);
}

static DEFINE_SPINLOCK(boot_lock);

static void tangox_secondary_init(unsigned int cpu)
{
	/*
	 * let the primary processor know we're out of the
	 * pen, then head off into the C entry point
	 */
	write_pen_release(-1);

	/*
	 * Synchronise with the boot thread.
	 */
	spin_lock(&boot_lock);
	spin_unlock(&boot_lock);
}

static int tangox_boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	unsigned long timeout;

	/*
	 * Set synchronisation state between this boot processor
	 * and the secondary one
	 */
	spin_lock(&boot_lock);

	/*
	 * This is really belt and braces; we hold unintended secondary
	 * CPUs in the holding pen until we're ready for them.  However,
	 * since we haven't sent them a soft interrupt, they shouldn't
	 * be there.
	 */
	write_pen_release(cpu_logical_map(cpu));

	/*
	 * Ask the secure monitor to start the secondary CPU.
	 */
	tangox_smc1(0x104, 0);

	timeout = jiffies + (1 * HZ);
	while (time_before(jiffies, timeout)) {
		smp_rmb();
		if (pen_release == -1)
			break;

		udelay(10);
	}

	/*
	 * Now the secondary core is starting up let it run its
	 * calibrations, then wait for it to finish
	 */
	spin_unlock(&boot_lock);

	return pen_release != -1 ? -ENOSYS : 0;
}

static const struct of_device_id tangox_smp_dt_scu_match[] __initconst = {
	{ .compatible = "arm,cortex-a9-scu" },
	{}
};

static void __init tangox_smp_prepare_cpus(unsigned int max_cpus)
{
	struct device_node *scu =
		of_find_matching_node(NULL, tangox_smp_dt_scu_match);

	if (scu)
		scu_enable(of_iomap(scu, 0));

	/*
	 * Register the address of secondary startup routine with the
	 * secure monitor.
	 */
	tangox_smc1(0x105, virt_to_phys(secondary_startup));
}

struct smp_operations tangox_smp_ops __initdata = {
	.smp_prepare_cpus	= tangox_smp_prepare_cpus,
	.smp_secondary_init	= tangox_secondary_init,
	.smp_boot_secondary	= tangox_boot_secondary,
};
CPU_METHOD_OF_DECLARE(tangox_smp, "sigma,smp8734", &tangox_smp_ops);
