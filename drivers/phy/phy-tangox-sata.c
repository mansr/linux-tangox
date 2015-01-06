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
#include <asm/io.h>

#define PHY_CTL0	0x0
#define PHY_CTL1	0x4
#define PHY_CTL2	0x8
#define PHY_CTL3	0xc

struct tangox_sata_phy {
	void __iomem *base;
	int clk_sel;
	int clk_ref;
	int tx_erc;
	int tx_ssc;
	int rx_ssc[2];
};

static int tangox_sata_phy_init(struct phy *genphy)
{
	struct tangox_sata_phy *phy = phy_get_drvdata(genphy);
	unsigned int clkref;
	unsigned int val;

	val = readl(phy->base + PHY_CTL0);
	val &= ~(0x1f << 20);
	val |= phy->clk_sel << 24;
	val |= phy->tx_erc << 20;
	writel(val, phy->base + PHY_CTL0);

	val = 0x28903;
	val |= phy->rx_ssc[0] << 9;
	val |= phy->rx_ssc[1] << 12;
	writel(val, phy->base + PHY_CTL1);

	switch (phy->clk_ref) {
	default:
		dev_warn(&genphy->dev, "Invalid ref clock %d\n", phy->clk_ref);
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
	val |= phy->tx_ssc << 10;
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

static int tangox_sata_phy_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct tangox_sata_phy *phy;
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

	phy->clk_sel = of_property_read_bool(node, "sigma,internal-clock");
	of_property_read_u32(node, "clock-frequency", &phy->clk_ref);
	of_property_read_u32(node, "sigma,tx-erc", &phy->tx_erc);
	of_property_read_u32(node, "sigma,tx-ssc", &phy->tx_ssc);
	of_property_read_u32_array(node, "sigma,rx-ssc", phy->rx_ssc, 2);

	genphy = devm_phy_create(&pdev->dev, NULL, &tangox_sata_phy_ops);
	if (IS_ERR(genphy))
		return PTR_ERR(genphy);

	phy_set_drvdata(genphy, phy);

	phy_prov = devm_of_phy_provider_register(&pdev->dev,
						 of_phy_simple_xlate);

	return PTR_ERR_OR_ZERO(phy_prov);
}

static struct of_device_id tangox_sata_phy_dt_ids[] = {
	{ .compatible = "sigma,smp8640-sata-phy" },
	{ }
};

static struct platform_driver tangox_sata_phy_driver = {
	.probe		= tangox_sata_phy_probe,
	.driver		= {
		.name		= "tangox-sata-phy",
		.of_match_table	= tangox_sata_phy_dt_ids,
	},
};
module_platform_driver(tangox_sata_phy_driver);

MODULE_AUTHOR("Mans Rullgard <mans@mansr.com>");
MODULE_DESCRIPTION("SMP86xx SATA PHY driver");
MODULE_LICENSE("GPL");
