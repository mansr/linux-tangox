
/*********************************************************************
 Copyright (C) 2001-2009
 Sigma Designs, Inc.

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/

#include <linux/init.h>
#include <linux/kernel.h>
#include <asm/bootinfo.h>
#include <asm/io.h>

#include "memmap.h"
#include "setup.h"

unsigned int tangox_chip_type;
unsigned int tangox_chip_rev;

static char tangox_system_type[32];

const char *get_system_type(void)
{
	return tangox_system_type;
}

static void __init tangox_systype_init(void)
{
	void __iomem *cid = ioremap(HOST_BASE + 0xfee8, 8);

	tangox_chip_type = readl(cid);
	tangox_chip_rev = readl(cid + 4);

	snprintf(tangox_system_type, sizeof(tangox_system_type),
		 "Sigma Designs SMP%04x ES%d",
		 tangox_chip_type, tangox_chip_rev);

	pr_info("%s\n", tangox_system_type);
}

#define REMAP_SIZE 0x04000000

static void __init tangox_remap_init(void)
{
	void __iomem *base;
	int i;

	base = ioremap(REMAP_CTL_BASE, 32);

	writel(0x1fc00000, base);
	writel(0, base + 4);

	for (i = 0; i < 6; i++)
		writel(REMAP2_BASE + i * REMAP_SIZE, base + 8 + 4 * i);

	mb();
}

static void __init tangox_cmdline_setup(void)
{
	int argc = fw_arg0, i;
	char **argv = (char **)fw_arg1;

	for (i = 1; i < argc; i++) {
		if (arcs_cmdline[0])
			strlcat(arcs_cmdline, " ", COMMAND_LINE_SIZE);
		strlcat(arcs_cmdline, argv[i], COMMAND_LINE_SIZE);
	}
}

void __init prom_init(void)
{
	tangox_systype_init();
	tangox_remap_init();
	tangox_cmdline_setup();
	prom_console_init();

	mips_machtype = MACH_TANGOX;
}

void __init prom_free_prom_memory(void)
{
}
