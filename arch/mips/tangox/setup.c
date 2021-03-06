/*
 * Copyright (C) 2015 Mans Rullgard <mans@mansr.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/irqchip.h>
#include <linux/of_platform.h>
#include <linux/of_fdt.h>
#include <linux/slab.h>
#include <linux/seq_file.h>
#include <asm/byteorder.h>
#include <asm/bootinfo.h>
#include <asm/reboot.h>
#include <asm/time.h>
#include <asm/traps.h>
#include <asm/cpu-info.h>
#include <asm/prom.h>

#include "setup.h"

void __init arch_init_irq(void)
{
	irqchip_init();
}

static void tangox_machine_halt(void)
{
	while (1)
		cpu_relax();
}

static void tangox_machine_power_off(void)
{
	while (1)
		cpu_relax();
}

void __init plat_mem_setup(void)
{
	_machine_halt = tangox_machine_halt;
	pm_power_off = tangox_machine_power_off;

	ioport_resource.start = 0;
	ioport_resource.end = 0x7fffffff;

	iomem_resource.start = 0;
	iomem_resource.end = 0x7fffffff;

	__dt_setup_arch(__dtb_start);
}

void __init plat_time_init(void)
{
	struct device_node *cpu;
	struct clk *clk;
	unsigned rate;
	int ccres;

	of_clk_init(NULL);
	clocksource_probe();

	cpu = of_find_node_by_path("cpu0");
	if (!cpu)
		return;

	clk = of_clk_get(cpu, 0);
	if (IS_ERR(clk))
		return;

	rate = clk_get_rate(clk);

	__asm__ ("rdhwr %0, $3" : "=r" (ccres));
	mips_hpt_frequency = rate / ccres;

	pr_info("CPU clock %d Hz\n", rate);
}

void __init device_tree_init(void)
{
	unflatten_and_copy_device_tree();
}

static int __init tangox_of_set_prop(struct device_node *node, char *name,
				     void *val, int len)
{
	struct property *prop;

	prop = kzalloc(sizeof(*prop) + len, GFP_KERNEL);
	if (!prop)
		return -ENOMEM;

	prop->name = kstrdup(name, GFP_KERNEL);
	if (!prop->name) {
		kfree(prop);
		return -ENOMEM;
	}

	prop->length = len;

	if (len) {
		prop->value = prop + 1;
		memcpy(prop->value, val, len);
	}

	of_update_property(node, prop);

	return 0;
}

static int __init tangox_of_eth_setup(const char *name, int num)
{
	struct device_node *node;
	unsigned char *mac;

	node = of_find_node_by_path(name);
	if (!node)
		return -ENODEV;

	mac = tangox_ethernet_getmac(num);
	if (!mac)
		return -ENODEV;

	tangox_of_set_prop(node, "local-mac-address", mac, 6);

	return 0;
}

static int __init tangox_of_sata_phy_setup(void)
{
	static const int clk_tab[16] __initconst = {
		120, 100, 60, 50, 30, 25,
	};
	struct device_node *node;
	unsigned int cfg = tangox_sata_cfg();
	int clk_sel;
	__be32 clk_ref;
	__be32 rx_ssc[2];
	__be32 tx_ssc;
	__be32 tx_erc;

	node = of_find_node_by_path("sata_phy");
	if (!node)
		return -ENODEV;

	rx_ssc[0] = cpu_to_be32(cfg & 1);
	rx_ssc[1] = cpu_to_be32(cfg >> 1 & 1);
	tx_ssc = cpu_to_be32(cfg >> 2 & 1);
	clk_ref = cpu_to_be32(clk_tab[cfg >> 4 & 15]);
	tx_erc = cpu_to_be32(cfg >> 8 & 15);
	clk_sel = cfg >> 15 & 1;

	tangox_of_set_prop(node, "sigma,rx-ssc", rx_ssc, 8);
	tangox_of_set_prop(node, "sigma,tx-ssc", &tx_ssc, 4);
	tangox_of_set_prop(node, "clock-frequency", &clk_ref, 4);
	tangox_of_set_prop(node, "sigma,tx-erc", &tx_erc, 4);
	if (clk_sel)
		tangox_of_set_prop(node, "sigma,internal-clock", NULL, 0);

	return 0;
}

static int __init plat_of_setup(void)
{
	if (!of_have_populated_dt())
		panic("device tree not present");

	xenv_config();

	tangox_of_eth_setup("eth0", 0);
	tangox_of_eth_setup("eth1", 1);

	tangox_of_sata_phy_setup();

	return of_platform_populate(NULL, of_default_bus_match_table,
				    NULL, NULL);
}
arch_initcall(plat_of_setup);
