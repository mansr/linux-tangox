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
#include <linux/platform_data/sata-tangox-phy.h>
#include <linux/phy/phy.h>
#include <asm/io.h>

#define PHY_CTL0	0x0
#define PHY_CTL1	0x4
#define PHY_CTL2	0x8
#define PHY_CTL3	0xc

struct tangox_sata_phy {
	void __iomem *base;
	struct tangox_sata_phy_pdata *pdata;
};

static int tangox_sata_phy_init(struct phy *genphy)
{
	struct tangox_sata_phy *phy = phy_get_drvdata(genphy);
	struct tangox_sata_phy_pdata *pd = phy->pdata;
	unsigned int clkref;
	unsigned int val;

	val = readl(phy->base + PHY_CTL0);
	val &= ~(0x1f << 20);
	val |= pd->clk_sel << 24;
	val |= pd->tx_erc << 20;
	writel(val, phy->base + PHY_CTL0);

	val = 0x28903;
	val |= pd->rx_ssc0 << 9;
	val |= pd->rx_ssc1 << 12;
	writel(val, phy->base + PHY_CTL1);

	switch (pd->clk_ref) {
	default:
		dev_warn(&genphy->dev, "Invalid ref clock %d\n", pd->clk_ref);
	case 120:
		clkref = 0x12c;
		break;
	case 60:
		clkref = 0x128;
		break;
	case 30:
		clkref = 0x12a;
		break;
	case 100:
		clkref = 0x234;
		break;
	case 50:
		clkref = 0x230;
		break;
	case 25:
		clkref = 0x232;
		break;
	}

	val = readl(phy->base + PHY_CTL2);
	val &= ~0x7fe;
	val |= pd->tx_ssc << 10;
	val |= clkref;
	writel(val, phy->base + PHY_CTL2);

	val = readl(phy->base + PHY_CTL3);
	val |= 1 << 16;		/* fast tech */
	val |= 1 << 18;		/* 3.3 V */
	writel(val, phy->base + PHY_CTL3);

	return 0;
}

static struct phy_ops tangox_sata_phy_ops = {
	.init = tangox_sata_phy_init,
};

static struct phy_consumer tangox_sata_phy_consumers[] = {
	{ "tangox-sata.0", "sata" },
	{ "tangox-sata.1", "sata" },
};

struct phy_init_data tangox_sata_phy_idata = {
	.num_consumers	= ARRAY_SIZE(tangox_sata_phy_consumers),
	.consumers	= tangox_sata_phy_consumers,
};

static int tangox_sata_phy_probe(struct platform_device *pdev)
{
	struct tangox_sata_phy *phy;
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

	phy->pdata = dev_get_platdata(&pdev->dev);

	genphy = devm_phy_create(&pdev->dev, NULL, &tangox_sata_phy_ops,
				 &tangox_sata_phy_idata);
	if (IS_ERR(genphy))
		return PTR_ERR(genphy);

	phy_set_drvdata(genphy, phy);

	return 0;
}

static struct platform_driver tangox_sata_phy_driver = {
	.probe		= tangox_sata_phy_probe,
	.driver		= {
		.name	= "tangox-sata-phy",
	},
};
module_platform_driver(tangox_sata_phy_driver);

MODULE_AUTHOR("Mans Rullgard <mans@mansr.com>");
MODULE_DESCRIPTION("SMP86xx SATA PHY driver");
MODULE_LICENSE("GPL");
