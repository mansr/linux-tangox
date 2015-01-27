/*
 * Copyright (C) 2015 Mans Rullgard <mans@mansr.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/input.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <media/rc-core.h>

#define RC6_CTRL	0x00
#define RC6_CLKDIV	0x04
#define RC6_DATA0	0x08
#define RC6_DATA1	0x0c
#define RC6_DATA2	0x10
#define RC6_DATA3	0x14
#define RC6_DATA4	0x18

#define RC6_CARRIER	36000
#define RC6_TIME_BASE	16

struct tangox_rc6 {
	void __iomem *base;
	struct rc_dev *rc;
	struct clk *clk;
};

static void tangox_rc6_input(struct tangox_rc6 *rc6)
{
	unsigned int data0, data1;
	unsigned int toggle;
	unsigned int mode;
	unsigned int addr;
	unsigned int cmd;

	data0 = readl(rc6->base + RC6_DATA0);
	data1 = readl(rc6->base + RC6_DATA1);

	mode = data0 >> 1 & 7;

	if (mode != 0)
		return;

	toggle = data0 & 1;
	addr = data0 >> 16;
	cmd = data1;

	rc_keydown(rc6->rc, RC_TYPE_RC6_0, RC_SCANCODE_RC6_0(addr, cmd),
		   toggle);
}

static irqreturn_t tangox_rc6_irq(int irq, void *dev_id)
{
	struct tangox_rc6 *rc6 = dev_id;
	unsigned int stat;

	stat = readl(rc6->base + RC6_CTRL);

	if (!(stat & 0xc0000000))
		return IRQ_NONE;

	writel(stat, rc6->base + RC6_CTRL);

	if (stat & BIT(30))
		return IRQ_HANDLED;

	tangox_rc6_input(rc6);

	return IRQ_HANDLED;
}

static int tangox_rc6_open(struct rc_dev *dev)
{
	struct tangox_rc6 *rc6 = dev->priv;

	writel(0xc1, rc6->base + RC6_CTRL);

	return 0;
}

static void tangox_rc6_close(struct rc_dev *dev)
{
	struct tangox_rc6 *rc6 = dev->priv;

	writel(0xc0000000, rc6->base + RC6_CTRL);
}

static int tangox_rc6_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tangox_rc6 *rc6;
	struct rc_dev *rc;
	struct resource *res;
	unsigned long clkrate;
	u64 clkdiv;
	int irq;
	int err;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;

	irq = platform_get_irq(pdev, 0);
	if (irq <= 0)
		return -EINVAL;

	rc6 = devm_kzalloc(dev, sizeof(*rc6), GFP_KERNEL);
	if (!rc6)
		return -ENOMEM;

	rc6->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(rc6->base))
		return PTR_ERR(rc6->base);

	rc6->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(rc6->clk))
		return PTR_ERR(rc6->clk);

	err = clk_prepare_enable(rc6->clk);
	if (err)
		return err;

	clkrate = clk_get_rate(rc6->clk);

	rc = rc_allocate_device();
	if (!rc) {
		err = -ENOMEM;
		goto err_clk;
	}

	rc->dev.parent = dev;
	rc->input_name = "tangox-ir-rc6";
	rc->input_phys = "tagnox-ir-rc6/input0";
	rc->driver_type = RC_DRIVER_SCANCODE;
	rc->map_name = RC_MAP_EMPTY;
	rc->allowed_protocols = RC_BIT_RC6_0;
	rc->open = tangox_rc6_open;
	rc->close = tangox_rc6_close;

	of_property_read_string(dev->of_node, "linux,rc-map-name",
				&rc->map_name);

	rc->priv = rc6;
	rc6->rc = rc;

	clkdiv = (u64)clkrate * RC6_TIME_BASE;
	do_div(clkdiv, RC6_CARRIER);

	writel(0xc0000000, rc6->base + RC6_CTRL);
	writel((clkdiv >> 2) << 18 | clkdiv, rc6->base + RC6_CLKDIV);

	err = devm_request_irq(dev, irq, tangox_rc6_irq, IRQF_SHARED,
			       dev_name(dev), rc6);
	if (err)
		goto err_rc;

	dev_info(dev, "SMP86xx RC-6 IR decoder at %x IRQ %d\n",
		 res->start, irq);

	err = rc_register_device(rc6->rc);
	if (err)
		goto err_rc;

	platform_set_drvdata(pdev, rc6);

	return 0;

err_rc:
	rc_free_device(rc);
err_clk:
	clk_disable_unprepare(rc6->clk);

	return err;
}

static int tangox_rc6_remove(struct platform_device *pdev)
{
	struct tangox_rc6 *rc6 = platform_get_drvdata(pdev);

	rc_unregister_device(rc6->rc);
	rc_free_device(rc6->rc);
	clk_disable_unprepare(rc6->clk);

	return 0;
}

static const struct of_device_id tangox_rc6_dt_ids[] = {
	{ .compatible = "sigma,smp8640-ir-rc6" },
	{ }
};
MODULE_DEVICE_TABLE(of, tangox_rc6_dt_ids);

static struct platform_driver tangox_rc6_driver = {
	.probe	= tangox_rc6_probe,
	.remove	= tangox_rc6_remove,
	.driver = {
		.name		= "tangox-rc6",
		.of_match_table	= tangox_rc6_dt_ids,
	},
};
module_platform_driver(tangox_rc6_driver);

MODULE_DESCRIPTION("SMP86xx RC-6 IR decoder driver");
MODULE_AUTHOR("Mans Rullgard <mans@mansr.com>");
MODULE_LICENSE("GPL");
