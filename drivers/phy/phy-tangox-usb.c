/*
 * Copyright (C) 2014 Mans Rullgard <mans@mansr.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/phy/phy.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <asm/io.h>

#define PHY_RESET 0x0
#define PHY_POWER 0xc

struct tangox_usb_phy {
	void __iomem *base;
	struct clk *clk;
};

static int tangox_usb_phy_init(struct phy *genphy)
{
	struct tangox_usb_phy *phy = phy_get_drvdata(genphy);
	unsigned int val;
	int err;

	err = clk_prepare_enable(phy->clk);
	if (err)
		return err;

	val = readl(phy->base + PHY_RESET);
	writel(val | 1, phy->base + PHY_RESET);

	udelay(30);

	val = readl(phy->base + PHY_RESET);
	writel(val & ~1, phy->base + PHY_RESET);

	mdelay(2);

	val = readl(phy->base + PHY_RESET);
	writel(val & ~2, phy->base + PHY_RESET);

	val = readl(phy->base + PHY_RESET);
	writel(val | (1 << 19), phy->base + PHY_RESET);

	mdelay(5);

	return 0;
}

static int tangox_usb_phy_exit(struct phy *genphy)
{
	struct tangox_usb_phy *phy = phy_get_drvdata(genphy);
	unsigned int val;

	val = readl(phy->base + PHY_RESET);
	writel(val | 2, phy->base + PHY_RESET);

	mdelay(2);

	val = readl(phy->base + PHY_RESET);
	writel(val | 1, phy->base + PHY_RESET);

	clk_disable_unprepare(phy->clk);

	return 0;
}

static int tangox_usb_phy_power_on(struct phy *genphy)
{
	struct tangox_usb_phy *phy = phy_get_drvdata(genphy);
	unsigned int val;

	val = readl(phy->base + PHY_POWER);
	writel(val & ~(1 << 6), phy->base + PHY_POWER);

	return 0;
}

static int tangox_usb_phy_power_off(struct phy *genphy)
{
	struct tangox_usb_phy *phy = phy_get_drvdata(genphy);
	unsigned int val;

	val = readl(phy->base + PHY_POWER);
	writel(val & (1 << 6), phy->base + PHY_POWER);

	return 0;
}

static struct phy_ops tangox_usb_phy_ops = {
	.init		= tangox_usb_phy_init,
	.exit		= tangox_usb_phy_exit,
	.power_on	= tangox_usb_phy_power_on,
	.power_off	= tangox_usb_phy_power_off,
};

static int tangox_usb_phy_probe(struct platform_device *pdev)
{
	struct tangox_usb_phy *phy;
	struct resource *res;
	struct phy *genphy;
	struct phy_provider *phy_prov;

	phy = devm_kzalloc(&pdev->dev, sizeof(*phy), GFP_KERNEL);
	if (!phy)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;

	phy->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(phy->base))
		return PTR_ERR(phy->base);

	phy->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(phy->clk))
		return PTR_ERR(phy->clk);

	genphy = devm_phy_create(&pdev->dev, NULL, &tangox_usb_phy_ops);
	if (IS_ERR(genphy))
		return PTR_ERR(genphy);

	phy_set_drvdata(genphy, phy);

	phy_prov = devm_of_phy_provider_register(&pdev->dev,
						 of_phy_simple_xlate);

	return PTR_ERR_OR_ZERO(phy_prov);
}

static const struct of_device_id tangox_usb_phy_dt_ids[] = {
	{ .compatible = "sigma,smp8640-usb-phy" },
	{ }
};

static struct platform_driver tangox_usb_phy_driver = {
	.probe		= tangox_usb_phy_probe,
	.driver		= {
		.name		= "tangox-usb-phy",
		.of_match_table	= tangox_usb_phy_dt_ids,
	},
};
module_platform_driver(tangox_usb_phy_driver);

MODULE_AUTHOR("Mans Rullgard <mans@mansr.com>");
MODULE_DESCRIPTION("SMP86xx USB PHY driver");
MODULE_LICENSE("GPL");
