/*
 * Copyright (C) 2015 Mans Rullgard <mans@mansr.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/clockchips.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/io.h>

#define TIMER_LOAD		0x00
#define TIMER_VALUE		0x04
#define TIMER_CTRL		0x08
#define TIMER_INT_ACK		0x0c

#define TIMER_CTRL_ENABLE	0x80
#define TIMER_CTRL_INT_EN	0x40
#define TIMER_CTRL_DIV_512	0x08
#define TIMER_CTRL_DIV_32	0x04

struct tangox_timer {
	void __iomem *base;
	int periodic;
	int ticks_per_jiffy;
	struct clock_event_device cevt;
};

#define to_tangox_timer(p) container_of(p, struct tangox_timer, cevt)

static irqreturn_t tangox_timer_irq(int irq, void *dev_id)
{
	struct tangox_timer *tm = dev_id;

	writel(1, tm->base + TIMER_INT_ACK);

	if (!tm->periodic)
		writel(0, tm->base + TIMER_CTRL);

	tm->cevt.event_handler(&tm->cevt);

	return IRQ_HANDLED;
}

static int tangox_timer_start(struct tangox_timer *tm, unsigned long ticks)
{
	int ctrl = TIMER_CTRL_ENABLE | TIMER_CTRL_INT_EN;

	if (ticks > 0xffff << 5) {
		ticks = DIV_ROUND_UP(ticks, 512);
		ctrl |= TIMER_CTRL_DIV_512;
	} else if (ticks > 0xffff) {
		ticks = DIV_ROUND_UP(ticks, 32);
		ctrl |= TIMER_CTRL_DIV_32;
	}

	if (ticks > 0xffff)
		return -EINVAL;

	writel(ticks, tm->base + TIMER_LOAD);
	writel(ctrl, tm->base + TIMER_CTRL);

	return 0;
}

static void tangox_timer_stop(struct tangox_timer *tm)
{
	writel(0, tm->base + TIMER_CTRL);
}

static int tangox_timer_next_event(unsigned long evt,
				   struct clock_event_device *dev)
{
	struct tangox_timer *tm = to_tangox_timer(dev);

	tangox_timer_stop(tm);

	return tangox_timer_start(tm, evt);
}

static int tangox_timer_set_periodic(struct clock_event_device *dev)
{
	struct tangox_timer *tm = to_tangox_timer(dev);

	tangox_timer_stop(tm);
	tm->periodic = 1;
	tangox_timer_start(tm, tm->ticks_per_jiffy);

	return 0;
}

static int tangox_timer_set_oneshot(struct clock_event_device *dev)
{
	struct tangox_timer *tm = to_tangox_timer(dev);

	tangox_timer_stop(tm);
	tm->periodic = 0;

	return 0;
}

static void __init tangox_timer_setup(struct device_node *node)
{
	struct tangox_timer *tm;
	void __iomem *base;
	const char *name;
	struct clk *clk;
	int rate;
	int irq;
	int err;

	clk = of_clk_get(node, 0);
	if (IS_ERR(clk))
		return;

	base = of_iomap(node, 0);
	if (!base)
		return;

	irq = irq_of_parse_and_map(node, 0);
	if (irq <= 0)
		return;

	if (of_property_read_string(node, "label", &name))
		name = node->name;

	tm = kzalloc(sizeof(*tm), GFP_KERNEL);
	if (!tm)
		return;

	err = clk_prepare_enable(clk);
	if (err)
		return;

	rate = clk_get_rate(clk);

	writel(0, base + TIMER_CTRL);
	writel(1, base + TIMER_INT_ACK);

	tm->base = base;
	tm->ticks_per_jiffy = DIV_ROUND_UP(rate, HZ);

	tm->cevt.set_next_event = tangox_timer_next_event;
	tm->cevt.set_state_periodic = tangox_timer_set_periodic;
	tm->cevt.set_state_oneshot = tangox_timer_set_oneshot;
	tm->cevt.features = CLOCK_EVT_FEAT_ONESHOT;
	tm->cevt.name = name;
	tm->cevt.rating = 350;
	tm->cevt.irq = irq;
	tm->cevt.cpumask = cpu_possible_mask;

	if (tm->ticks_per_jiffy <= 0xffff << 9)
		tm->cevt.features |= CLOCK_EVT_FEAT_PERIODIC;

	pr_info("%s: SMP86xx timer rate %d Hz\n", name, rate);

	clockevents_config_and_register(&tm->cevt, rate, 1, 0xffff << 9);

	err = request_irq(irq, tangox_timer_irq, 0, name, tm);
	if (err)
		pr_warn("%s: failed to request irq %d\n", name, irq);
}
CLOCKSOURCE_OF_DECLARE(tangox_timer, "sigma,smp8640-timer", tangox_timer_setup);
