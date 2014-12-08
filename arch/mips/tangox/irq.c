/*
 * Copyright 2001 MontaVista Software Inc.
 * Author: Jun Sun, jsun@mvista.com or jsun@junsun.net
 *
 * Copyright (C) 2009 Sigma Designs, Inc.
 *
 * arch_init_irq for tango2/tango3
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/init.h>
#include <linux/interrupt.h>
#include <asm/irq.h>
#include <asm/irq_cpu.h>
#include <asm/io.h>

#include "irq.h"
#include "memmap.h"

#define IRQ_CTL_BASE		0x0000
#define FIQ_CTL_BASE		0x0100
#define EDGE_CTL_BASE		0x0200
#define IIQ_CTL_BASE		0x0300

#define IRQ_CTL_HI		(IRQ_CTL_BASE + 0x18)
#define FIQ_CTL_HI		(FIQ_CTL_BASE + 0x18)
#define EDGE_CTL_HI		(EDGE_CTL_BASE + 0x20)
#define IIQ_CTL_HI		(IIQ_CTL_BASE + 0x18)

#define IRQ_STATUS		0x00
#define IRQ_RAWSTAT		0x04
#define IRQ_EN_SET		0x08
#define IRQ_EN_CLR		0x0c
#define IRQ_SOFT_SET		0x10
#define IRQ_SOFT_CLR		0x14

#define EDGE_STATUS		0x00
#define EDGE_RAWSTAT		0x04
#define EDGE_CFG_RISE		0x08
#define EDGE_CFG_FALL		0x0c
#define EDGE_CFG_RISE_SET	0x10
#define EDGE_CFG_RISE_CLR	0x14
#define EDGE_CFG_FALL_SET	0x18
#define EDGE_CFG_FALL_CLR	0x1c

static void __iomem *intc_base;

static inline u32 intc_readl(int reg)
{
	return readl(intc_base + reg);
}

static inline void intc_writel(int reg, u32 val)
{
	writel(val, intc_base + reg);
}

asmlinkage void plat_irq_dispatch(void)
{
	u32 sr = read_c0_status();
	u32 pending = read_c0_cause() & sr & ST0_IM;
	u32 status, status_hi;
	u32 mask = 0;
	int base;
	int irq;

	if (!pending) {
		pr_warn("Spurious hwirq: nothing pending\n");
		goto spurious;
	}

	irq = __ffs(pending) - CAUSEB_IP;

	switch (irq) {
	case 2:
		status = intc_readl(IRQ_CTL_BASE + IRQ_STATUS);
		status_hi = intc_readl(IRQ_CTL_HI + IRQ_STATUS);
		base = IRQ_BASE;
		break;

	case 3:
		status = intc_readl(FIQ_CTL_BASE + IRQ_STATUS);
		status_hi = intc_readl(FIQ_CTL_HI + IRQ_STATUS);
		base = FIQ_BASE;
		mask = ~STATUSF_IP2;
		break;

	case 4:
		status = intc_readl(IIQ_CTL_BASE + IRQ_STATUS);
		status_hi = intc_readl(IIQ_CTL_HI + IRQ_STATUS);
		base = IIQ_BASE;
		mask = ~(STATUSF_IP2 | STATUSF_IP3);
		break;

	default:
		do_IRQ(irq);
		return;
	}

	if (!(status | status_hi)) {
		pr_warn("Spurious hwirq %d\n", irq);
		goto spurious;
	}

	irq = status ? __ffs(status) : __ffs(status_hi) + 32;

	if (mask) {
		write_c0_status(sr & mask);
		do_IRQ(base + irq);
		write_c0_status(sr);
		return;
	}

	do_IRQ(base + irq);

	return;

spurious:
	spurious_interrupt();
}

static struct irqaction irq_cascade = {
	.handler = no_action,
	.flags = IRQF_SHARED,
	.name = "cascade",
};

static void __init tangox_irq_init(char *name, int irq, unsigned long ctl_base,
				   unsigned long edge_base)
{
	struct irq_chip_generic *gc;
	struct irq_chip_type *ct;

	gc = irq_alloc_generic_chip(name, 1, irq, intc_base, handle_level_irq);
	if (!gc) {
		pr_err("irq_alloc_generic_chip failed for IRQ %d\n", irq);
		return;
	}

	ct = gc->chip_types;

	ct->chip.irq_ack = irq_gc_ack_set_bit;
	ct->chip.irq_mask = irq_gc_mask_disable_reg;
	ct->chip.irq_mask_ack = irq_gc_mask_disable_reg_and_ack;
	ct->chip.irq_unmask = irq_gc_unmask_enable_reg;

	ct->regs.enable = ctl_base + IRQ_EN_SET;
	ct->regs.disable = ctl_base + IRQ_EN_CLR;
	ct->regs.ack = edge_base + EDGE_RAWSTAT;
	ct->regs.eoi = ctl_base + IRQ_EN_SET;

	intc_writel(ct->regs.disable, 0xffffffff);
	intc_writel(ct->regs.ack, 0xffffffff);

	irq_setup_generic_chip(gc, IRQ_MSK(32), 0, 0, 0);
}

void __init arch_init_irq(void)
{
	intc_base = ioremap(INTC_BASE, 0x1000);

	mips_cpu_irq_init();

	tangox_irq_init("IRQ", IRQ_BASE,      IRQ_CTL_BASE, EDGE_CTL_BASE);
	tangox_irq_init("IRQ", IRQ_BASE + 32, IRQ_CTL_HI,   EDGE_CTL_HI);
	tangox_irq_init("FIQ", FIQ_BASE,      FIQ_CTL_BASE, EDGE_CTL_BASE);
	tangox_irq_init("FIQ", FIQ_BASE + 32, FIQ_CTL_HI,   EDGE_CTL_HI);
	tangox_irq_init("IIQ", IIQ_BASE,      IIQ_CTL_BASE, EDGE_CTL_BASE);
	tangox_irq_init("IIQ", IIQ_BASE + 32, IIQ_CTL_HI,   EDGE_CTL_HI);

	setup_irq(MIPS_CPU_IRQ_BASE + 2, &irq_cascade);
	setup_irq(MIPS_CPU_IRQ_BASE + 3, &irq_cascade);
	setup_irq(MIPS_CPU_IRQ_BASE + 4, &irq_cascade);
}
