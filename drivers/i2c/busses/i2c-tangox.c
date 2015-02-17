/*
 * Copyright (C) 2015 Mans Rullgard <mans@mansr.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/ioport.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/wait.h>

#define TANGOX_I2C_CONFIG		0x00
#define TANGOX_I2C_CLKDIV		0x04
#define TANGOX_I2C_DEVADDR		0x08
#define TANGOX_I2C_ADDR			0x0c
#define TANGOX_I2C_DATAOUT		0x10
#define TANGOX_I2C_DATAIN		0x14
#define TANGOX_I2C_STATUS		0x18
#define TANGOX_I2C_STARTXFER		0x1c
#define TANGOX_I2C_BYTECNT		0x20
#define TANGOX_I2C_INT_EN		0x24
#define TANGOX_I2C_INT_STAT		0x28

#define TANGOX_I2C_CFG_EN		(1 << 8)
#define TANGOX_I2C_CFG_ADDRLEN(x)	((x) << 5)
#define TANGOX_I2C_CFG_DEVADLEN(x)	((x) << 2)
#define TANGOX_I2C_CFG_ADDRDIS		(1 << 1)
#define TANGOX_I2C_CFG_DEVADDIS		(1 << 0)

#define TANGOX_I2C_STATUS_IDLE		(1 << 0)
#define TANGOX_I2C_STATUS_SDOEMPTY	(1 << 1)
#define TANGOX_I2C_STATUS_DATARDY	(1 << 2)
#define TANGOX_I2C_STATUS_ACKERR	(1 << 3)
#define TANGOX_I2C_STATUS_STARTERR	(1 << 4)

#define TANGOX_I2C_XFER_WR		0
#define TANGOX_I2C_XFER_RD		1
#define TANGOX_I2C_XFER_NODATA		2

#define TANGOX_I2C_CFG_WR		0x1f8
#define TANGOX_I2C_CFG_RD		0x1fa

#define TANGOX_I2C_TIMEOUT		(HZ / 10)

struct tangox_i2c {
	struct i2c_adapter adap;
	void __iomem *base;
	struct i2c_msg *msg;
	int pos;
	wait_queue_head_t wait;
	struct clk *clk;
};

static int tangox_i2c_idle(struct tangox_i2c *ti2c)
{
	return readl(ti2c->base + TANGOX_I2C_STATUS) & TANGOX_I2C_STATUS_IDLE;
}

static int tangox_i2c_wait(struct tangox_i2c *ti2c, unsigned long timeout)
{
	int status;
	int t;

	t = wait_event_timeout(ti2c->wait, tangox_i2c_idle(ti2c), timeout);
	if (!t)
		return -ETIMEDOUT;

	status = readl(ti2c->base + TANGOX_I2C_STATUS);

	return status & TANGOX_I2C_STATUS_ACKERR ? -EIO : 0;
}

static void tangox_i2c_tx_irq(struct tangox_i2c *ti2c, u32 status)
{
	struct i2c_msg *msg = ti2c->msg;

	if (status & TANGOX_I2C_STATUS_SDOEMPTY)
		writel(msg->buf[ti2c->pos++], ti2c->base + TANGOX_I2C_DATAOUT);
}

static void tangox_i2c_rx_irq(struct tangox_i2c *ti2c, u32 status)
{
	struct i2c_msg *msg = ti2c->msg;

	if (status & TANGOX_I2C_STATUS_DATARDY)
		msg->buf[ti2c->pos++] = readl(ti2c->base + TANGOX_I2C_DATAIN);
}

static irqreturn_t tangox_i2c_irq(int irq, void *dev_id)
{
	struct tangox_i2c *ti2c = dev_id;
	struct i2c_msg *msg = ti2c->msg;
	u32 int_stat, status;

	int_stat = readl(ti2c->base + TANGOX_I2C_INT_STAT);
	writel(int_stat, ti2c->base + TANGOX_I2C_INT_STAT);

	if (!int_stat)
		return IRQ_NONE;

	status = readl(ti2c->base + TANGOX_I2C_STATUS);

	if (!msg)
		return IRQ_HANDLED;

	if (ti2c->pos < msg->len) {
		if (msg->flags & I2C_M_RD)
			tangox_i2c_rx_irq(ti2c, status);
		else
			tangox_i2c_tx_irq(ti2c, status);
	}

	if (status & TANGOX_I2C_STATUS_IDLE)
		wake_up(&ti2c->wait);

	return IRQ_HANDLED;
}

static int tangox_i2c_tx(struct tangox_i2c *ti2c, struct i2c_msg *msg)
{
	int devaddr = msg->addr;
	int pos = 0;
	int addr;
	int xfer;
	int err;

	if (msg->len < 1)
		return -EINVAL;

	addr = msg->buf[pos++];

	writel(TANGOX_I2C_CFG_WR, ti2c->base + TANGOX_I2C_CONFIG);
	writel(devaddr, ti2c->base + TANGOX_I2C_DEVADDR);
	writel(addr, ti2c->base + TANGOX_I2C_ADDR);

	if (msg->len == 1) {
		writel(0, ti2c->base + TANGOX_I2C_BYTECNT);
		xfer = TANGOX_I2C_XFER_WR | TANGOX_I2C_XFER_NODATA;
	} else {
		writel(msg->len - 2, ti2c->base + TANGOX_I2C_BYTECNT);
		writel(msg->buf[pos++], ti2c->base + TANGOX_I2C_DATAOUT);
		xfer = TANGOX_I2C_XFER_WR;
	}

	ti2c->msg = msg;
	ti2c->pos = pos;

	writel(xfer, ti2c->base + TANGOX_I2C_STARTXFER);

	err = tangox_i2c_wait(ti2c, TANGOX_I2C_TIMEOUT * msg->len);

	ti2c->msg = NULL;

	return err;
}

static int tangox_i2c_rx(struct tangox_i2c *ti2c, struct i2c_msg *msg)
{
	int devaddr = msg->addr;
	int err;

	if (msg->len < 1)
		return -EINVAL;

	ti2c->msg = msg;
	ti2c->pos = 0;

	writel(TANGOX_I2C_CFG_RD, ti2c->base + TANGOX_I2C_CONFIG);
	writel(devaddr, ti2c->base + TANGOX_I2C_DEVADDR);
	writel(msg->len - 1, ti2c->base + TANGOX_I2C_BYTECNT);
	writel(TANGOX_I2C_XFER_RD, ti2c->base + TANGOX_I2C_STARTXFER);

	err = tangox_i2c_wait(ti2c, TANGOX_I2C_TIMEOUT * msg->len);

	ti2c->msg = NULL;

	return err;
}

static int tangox_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg *msg,
			   int num)
{
	struct tangox_i2c *ti2c = adap->algo_data;
	int completed = 0;
	int err;

	while (num--) {
		if (msg->flags & I2C_M_RD)
			err = tangox_i2c_rx(ti2c, msg);
		else
			err = tangox_i2c_tx(ti2c, msg);

		if (err)
			return err;

		completed++;
		msg++;
	}

	return completed;
}

static u32 tangox_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm tangox_i2c_algo = {
	.master_xfer	= tangox_i2c_xfer,
	.functionality	= tangox_i2c_func,
};

static int tangox_i2c_probe(struct platform_device *pdev)
{
	struct tangox_i2c *ti2c;
	struct resource *res;
	struct clk *clk;
	u32 busfreq;
	int clkdiv;
	int rate;
	int irq;
	int err;

	ti2c = devm_kzalloc(&pdev->dev, sizeof(*ti2c), GFP_KERNEL);
	if (!ti2c)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;

	ti2c->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(ti2c->base))
		return PTR_ERR(ti2c->base);

	irq = platform_get_irq(pdev, 0);
	if (irq <= 0)
		return -EINVAL;

	if (of_property_read_u32(pdev->dev.of_node, "clock-frequency",
				 &busfreq))
		busfreq = 100000;

	clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	err = clk_prepare_enable(clk);
	if (err)
		return err;

	rate = clk_get_rate(clk);

	ti2c->adap.dev.parent = &pdev->dev;
	ti2c->adap.dev.of_node = pdev->dev.of_node;
	ti2c->adap.algo = &tangox_i2c_algo;
	ti2c->adap.algo_data = ti2c;
	snprintf(ti2c->adap.name, sizeof(ti2c->adap.name), "tangox-i2c-%x",
		 res->start);

	init_waitqueue_head(&ti2c->wait);
	ti2c->clk = clk;

	platform_set_drvdata(pdev, ti2c);
	i2c_set_adapdata(&ti2c->adap, ti2c);

	clkdiv = DIV_ROUND_UP(rate, 2 * busfreq);

	writel(0, ti2c->base + TANGOX_I2C_CONFIG);
	writel(clkdiv, ti2c->base + TANGOX_I2C_CLKDIV);
	writel(0xf, ti2c->base + TANGOX_I2C_INT_STAT);

	err = devm_request_irq(&pdev->dev, irq, tangox_i2c_irq, IRQF_SHARED,
			       dev_name(&pdev->dev), ti2c);
	if (err)
		goto err_out;

	writel(0xf, ti2c->base + TANGOX_I2C_INT_EN);

	err = i2c_add_adapter(&ti2c->adap);
	if (err)
		goto err_out;

	dev_info(&ti2c->adap.dev, "SMP86xx I2C master at %x\n", res->start);

	return 0;

err_out:
	clk_disable_unprepare(clk);

	return err;
}

static int tangox_i2c_remove(struct platform_device *pdev)
{
	struct tangox_i2c *ti2c = platform_get_drvdata(pdev);

	i2c_del_adapter(&ti2c->adap);
	clk_disable_unprepare(ti2c->clk);

	return 0;
}

static const struct of_device_id tangox_i2c_dt_ids[] = {
	{ .compatible = "sigma,smp8640-i2c" },
	{ }
};

static struct platform_driver tangox_i2c_driver = {
	.probe	= tangox_i2c_probe,
	.remove	= tangox_i2c_remove,
	.driver	= {
		.name		= "tangox-i2c",
		.of_match_table	= tangox_i2c_dt_ids,
	},
};
module_platform_driver(tangox_i2c_driver);

MODULE_DESCRIPTION("SMP86xx I2C bus driver");
MODULE_AUTHOR("Mans Rullgard <mans@mansr.com>");
MODULE_LICENSE("GPL");
