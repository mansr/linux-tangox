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

#define RC6_CTRL	0x00
#define RC6_CLKDIV	0x04
#define RC6_DATA0	0x08
#define RC6_DATA1	0x0c
#define RC6_DATA2	0x10
#define RC6_DATA3	0x14
#define RC6_DATA4	0x18

#define RC6_CARRIER	36000
#define RC6_TIME_BASE	16

struct tangox_ir {
	void __iomem *rc5_base;
	void __iomem *rc6_base;
	struct rc_dev *rc;
	struct clk *clk;
};

static void tangox_ir_handle_nec(struct tangox_ir *ir)
{
	unsigned int data;
	unsigned int addr;
	unsigned int naddr;
	unsigned int key;
	unsigned int nkey;

	data = readl(ir->rc5_base + IR_NEC_DATA);

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

static void tangox_ir_handle_rc5(struct tangox_ir *ir)
{
	unsigned int data;
	unsigned int field;
	unsigned int toggle;
	unsigned int addr;
	unsigned int cmd;

	data = readl(ir->rc5_base + IR_RC5_DATA);

	if (data & BIT(31))
		return;

	field = data >> 12 & 1;
	toggle = data >> 11 & 1;
	addr = data >> 6 & 0x1f;
	cmd = (data & 0x3f) | (field ^ 1) << 6;

	rc_keydown(ir->rc, RC_TYPE_RC5, RC_SCANCODE_RC5(addr, cmd), toggle);
}

static void tangox_ir_handle_rc6(struct tangox_ir *ir)
{
	unsigned int data0, data1;
	unsigned int toggle;
	unsigned int mode;
	unsigned int addr;
	unsigned int cmd;

	data0 = readl(ir->rc6_base + RC6_DATA0);
	data1 = readl(ir->rc6_base + RC6_DATA1);

	mode = data0 >> 1 & 7;

	if (mode != 0)
		return;

	toggle = data0 & 1;
	addr = data0 >> 16;
	cmd = data1;

	rc_keydown(ir->rc, RC_TYPE_RC6_0, RC_SCANCODE_RC6_0(addr, cmd),
		   toggle);
}

static irqreturn_t tangox_ir_irq(int irq, void *dev_id)
{
	struct tangox_ir *ir = dev_id;
	unsigned int rc5_stat;
	unsigned int rc6_stat;

	rc5_stat = readl(ir->rc5_base + IR_INT);
	writel(rc5_stat, ir->rc5_base + IR_INT);

	rc6_stat = readl(ir->rc6_base + RC6_CTRL);
	writel(rc6_stat, ir->rc6_base + RC6_CTRL);

	if (!(rc5_stat & 3) && !(rc6_stat & BIT(31)))
		return IRQ_NONE;

	if (rc5_stat & 1)
		tangox_ir_handle_rc5(ir);

	if (rc5_stat & 2)
		tangox_ir_handle_nec(ir);

	if (rc6_stat & BIT(31))
		tangox_ir_handle_rc6(ir);

	return IRQ_HANDLED;
}

static int tangox_ir_open(struct rc_dev *dev)
{
	struct tangox_ir *ir = dev->priv;

	writel(0x201, ir->rc5_base + IR_CTRL);
	writel(0x81, ir->rc6_base + RC6_CTRL);

	return 0;
}

static void tangox_ir_close(struct rc_dev *dev)
{
	struct tangox_ir *ir = dev->priv;

	writel(0x110, ir->rc5_base + IR_CTRL);
	writel(0, ir->rc6_base + RC6_CTRL);
}

static int tangox_ir_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rc_dev *rc;
	struct tangox_ir *ir;
	struct resource *rc5_res;
	struct resource *rc6_res;
	unsigned long clkrate;
	u64 clkdiv;
	int irq;
	int err;

	rc5_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!rc5_res)
		return -EINVAL;

	rc6_res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!rc6_res)
		return -EINVAL;

	irq = platform_get_irq(pdev, 0);
	if (irq <= 0)
		return -EINVAL;

	ir = devm_kzalloc(dev, sizeof(*ir), GFP_KERNEL);
	if (!ir)
		return -ENOMEM;

	ir->rc5_base = devm_ioremap_resource(dev, rc5_res);
	if (IS_ERR(ir->rc5_base))
		return PTR_ERR(ir->rc5_base);

	ir->rc6_base = devm_ioremap_resource(dev, rc6_res);
	if (IS_ERR(ir->rc6_base))
		return PTR_ERR(ir->rc6_base);

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
	rc->input_name = "tangox-ir";
	rc->input_phys = "tagnox-ir/input0";
	rc->driver_type = RC_DRIVER_SCANCODE;
	rc->map_name = RC_MAP_EMPTY;
	rc->allowed_protocols = RC_BIT_RC5 | RC_BIT_NEC | RC_BIT_RC6_0;
	rc->open = tangox_ir_open;
	rc->close = tangox_ir_close;

	of_property_read_string(dev->of_node, "linux,rc-map-name",
				&rc->map_name);

	rc->priv = ir;
	ir->rc = rc;

	clkdiv = (u64)clkrate * NEC_TIME_BASE;
	do_div(clkdiv, 1000000);

	writel(31 << 24 | 12 << 16 | clkdiv, ir->rc5_base + IR_NEC_CTRL);

	clkdiv = (u64)clkrate * RC5_TIME_BASE;
	do_div(clkdiv, 1000000);

	writel(0x110, ir->rc5_base + IR_CTRL);
	writel(clkdiv, ir->rc5_base + IR_RC5_CLK_DIV);
	writel(0x3, ir->rc5_base + IR_INT);

	clkdiv = (u64)clkrate * RC6_TIME_BASE;
	do_div(clkdiv, RC6_CARRIER);

	writel(0xc0000000, ir->rc6_base + RC6_CTRL);
	writel((clkdiv >> 2) << 18 | clkdiv, ir->rc6_base + RC6_CLKDIV);

	err = devm_request_irq(dev, irq, tangox_ir_irq, IRQF_SHARED,
			       dev_name(dev), ir);
	if (err)
		goto err_rc;

	dev_info(dev, "SMP86xx IR decoder at 0x%x/0x%x IRQ %d\n",
		 rc5_res->start, rc6_res->start, irq);

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
	{ .compatible = "sigma,smp8640-ir" },
	{ }
};
MODULE_DEVICE_TABLE(of, tangox_ir_dt_ids);

static struct platform_driver tangox_ir_driver = {
	.probe	= tangox_ir_probe,
	.remove	= tangox_ir_remove,
	.driver	= {
		.name		= "tangox-ir",
		.of_match_table	= tangox_ir_dt_ids,
	},
};
module_platform_driver(tangox_ir_driver);

MODULE_DESCRIPTION("SMP86xx IR decoder driver");
MODULE_AUTHOR("Mans Rullgard <mans@mansr.com>");
MODULE_LICENSE("GPL");
