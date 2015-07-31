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
#include <asm/irq.h>
#include <asm/irq_cpu.h>

void __init arch_init_irq(void)
{
	irqchip_init();
}
