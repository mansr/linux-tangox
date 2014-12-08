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
#include <asm/io.h>

#define PHY_RESET 0x0
#define PHY_POWER 0xc

struct tangox_usb_phy {
	void __iomem *base;
	char ehci_name[32];
	char ohci_name[32];
	struct phy_consumer consumers[2];
	struct phy_init_data idata;
};

static int tangox_usb_phy_init(struct phy *genphy)
{
	struct tangox_usb_phy *phy = phy_get_drvdata(genphy);
	unsigned int val;

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

	phy = devm_kzalloc(&pdev->dev, sizeof(*phy), GFP_KERNEL);
	if (!phy)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;

	phy->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(phy->base))
		return PTR_ERR(phy->base);

	snprintf(phy->ehci_name, sizeof(phy->ehci_name),
		 "ehci-platform.%d", pdev->id);
	snprintf(phy->ohci_name, sizeof(phy->ohci_name),
		 "ohci-platform.%d", pdev->id);

	phy->consumers[0].dev_name = phy->ehci_name;
	phy->consumers[0].port = "usb";

	phy->consumers[1].dev_name = phy->ohci_name;
	phy->consumers[1].port = "usb";

	phy->idata.num_consumers = ARRAY_SIZE(phy->consumers);
	phy->idata.consumers = phy->consumers;

	genphy = devm_phy_create(&pdev->dev, NULL, &tangox_usb_phy_ops,
				 &phy->idata);
	if (IS_ERR(genphy))
		return PTR_ERR(genphy);

	phy_set_drvdata(genphy, phy);

	return 0;
}

static struct platform_driver tangox_usb_phy_driver = {
	.probe		= tangox_usb_phy_probe,
	.driver		= {
		.name	= "tangox-usb-phy",
	},
};
module_platform_driver(tangox_usb_phy_driver);

MODULE_AUTHOR("Mans Rullgard <mans@mansr.com>");
MODULE_DESCRIPTION("SMP86xx USB PHY driver");
MODULE_LICENSE("GPL");
