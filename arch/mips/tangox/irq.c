/*
 * Copyright (C) 2014 Mans Rullgard <mans@mansr.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/init.h>
#include <linux/irqchip.h>
#include <linux/of.h>
#include <asm/irq.h>
#include <asm/irq_cpu.h>

asmlinkage void plat_irq_dispatch(void)
{
	unsigned int pending;

	pending = (read_c0_cause() & read_c0_status() & ST0_IM) >> CAUSEB_IP;

	if (pending)
		do_IRQ(__ffs(pending));
	else
		spurious_interrupt();
}

void __init arch_init_irq(void)
{
	irqchip_init();
}

OF_DECLARE_2(irqchip, mips_cpu, "mti,cpu-interrupt-controller",
	     mips_cpu_intc_init);
