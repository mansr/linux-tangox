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

#define IR_NEC_CTRL	0x00
#define IR_NEC_DATA	0x04
#define IR_CTRL		0x08
#define IR_RC5_CLK_DIV	0x0c
#define IR_RC5_DATA	0x10
#define IR_INT		0x14

#define NEC_TIME_BASE	560
#define RC5_TIME_BASE	1778

struct tangox_ir {
	void __iomem *base;
	struct rc_dev *rc;
	struct clk *clk;
};

static void tangox_ir_handle_nec(struct tangox_ir *ir, unsigned int data)
{
	unsigned int addr;
	unsigned int naddr;
	unsigned int key;
	unsigned int nkey;

	if (!data) {
		rc_repeat(ir->rc);
		return;
	}

	addr = data & 0xff;
	naddr = data >> 8 & 0xff;
	key = data >> 16 & 0xff;
	nkey = data >> 24 & 0xff;

	if ((addr ^ naddr) != 255)
		addr = addr << 8 | naddr;

	if ((key ^ nkey) != 255)
		return;

	rc_keydown(ir->rc, RC_TYPE_NEC, RC_SCANCODE_NEC(addr, key), 0);
}

static void tangox_ir_handle_rc5(struct tangox_ir *ir, unsigned int data)
{
	unsigned int field;
	unsigned int toggle;
	unsigned int addr;
	unsigned int cmd;

	if (data & BIT(31))
		return;

	field = data >> 12 & 1;
	toggle = data >> 11 & 1;
	addr = data >> 6 & 0x1f;
	cmd = (data & 0x3f) | (field ^ 1) << 6;

	rc_keydown(ir->rc, RC_TYPE_RC5, RC_SCANCODE_RC5(addr, cmd), toggle);
}

static irqreturn_t tangox_ir_irq(int irq, void *dev_id)
{
	struct tangox_ir *ir = dev_id;
	unsigned int data;
	unsigned int stat;

	stat = readl(ir->base + IR_INT);

	if (!(stat & 3))
		return IRQ_NONE;

	writel(stat, ir->base + IR_INT);

	if (stat & 1) {
		data = readl(ir->base + IR_RC5_DATA);
		tangox_ir_handle_rc5(ir, data);
	}

	if (stat & 2) {
		data = readl(ir->base + IR_NEC_DATA);
		tangox_ir_handle_nec(ir, data);
	}

	return IRQ_HANDLED;
}

static int tangox_ir_open(struct rc_dev *dev)
{
	struct tangox_ir *ir = dev->priv;

	writel(0x201, ir->base + IR_CTRL);

	return 0;
}

static void tangox_ir_close(struct rc_dev *dev)
{
	struct tangox_ir *ir = dev->priv;

	writel(0x110, ir->base + IR_CTRL);
}

static int tangox_ir_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rc_dev *rc;
	struct tangox_ir *ir;
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

	ir = devm_kzalloc(dev, sizeof(*ir), GFP_KERNEL);
	if (!ir)
		return -ENOMEM;

	ir->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(ir->base))
		return PTR_ERR(ir->base);

	ir->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(ir->clk))
		return PTR_ERR(ir->clk);

	err = clk_prepare_enable(ir->clk);
	if (err)
		return err;

	clkrate = clk_get_rate(ir->clk);

	rc = rc_allocate_device();
	if (!rc) {
		err = -ENOMEM;
		goto err_clk;
	}

	rc->dev.parent = dev;
	rc->input_name = "tangox-ir-rc5";
	rc->input_phys = "tagnox-ir-rc5/input0";
	rc->driver_type = RC_DRIVER_SCANCODE;
	rc->map_name = RC_MAP_EMPTY;
	rc->allowed_protocols = RC_BIT_RC5 | RC_BIT_NEC;
	rc->open = tangox_ir_open;
	rc->close = tangox_ir_close;

	of_property_read_string(dev->of_node, "linux,rc-map-name",
				&rc->map_name);

	rc->priv = ir;
	ir->rc = rc;

	clkdiv = (u64)clkrate * NEC_TIME_BASE;
	do_div(clkdiv, 1000000);

	writel(31 << 24 | 12 << 16 | clkdiv, ir->base + IR_NEC_CTRL);

	clkdiv = (u64)clkrate * RC5_TIME_BASE;
	do_div(clkdiv, 1000000);

	writel(0x110, ir->base + IR_CTRL);
	writel(clkdiv, ir->base + IR_RC5_CLK_DIV);
	writel(0x3, ir->base + IR_INT);

	err = devm_request_irq(dev, irq, tangox_ir_irq, IRQF_SHARED,
			       dev_name(dev), ir);
	if (err)
		goto err_rc;

	dev_info(dev, "SMP86xx NEC/RC-5 IR decoder at 0x%x IRQ %d\n",
		 res->start, irq);

	err = rc_register_device(rc);
	if (err)
		goto err_rc;

	platform_set_drvdata(pdev, ir);

	return 0;

err_rc:
	rc_free_device(rc);
err_clk:
	clk_disable_unprepare(ir->clk);

	return err;
}

static int tangox_ir_remove(struct platform_device *pdev)
{
	struct tangox_ir *ir = platform_get_drvdata(pdev);

	rc_unregister_device(ir->rc);
	rc_free_device(ir->rc);
	clk_disable_unprepare(ir->clk);

	return 0;
}

static const struct of_device_id tangox_ir_dt_ids[] = {
	{ .compatible = "sigma,smp8640-ir-rc5" },
	{ }
};
MODULE_DEVICE_TABLE(of, tangox_ir_dt_ids);

static struct platform_driver tangox_ir_driver = {
	.probe	= tangox_ir_probe,
	.remove	= tangox_ir_remove,
	.driver	= {
		.name		= "tangox-rc5",
		.of_match_table	= tangox_ir_dt_ids,
	},
};
module_platform_driver(tangox_ir_driver);

MODULE_DESCRIPTION("SMP86xx NEC/RC-5 IR decoder driver");
MODULE_AUTHOR("Mans Rullgard <mans@mansr.com>");
MODULE_LICENSE("GPL");
