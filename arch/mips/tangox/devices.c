#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/platform_data/sata-tangox-phy.h>
#include <linux/phy.h>
#include <asm/io.h>

#include "irq.h"
#include "memmap.h"
#include "setup.h"

static struct resource tangox_sata0_resources[] = {
	DEFINE_RES_MEM(SATA0_BASE, 0x800),
	DEFINE_RES_MEM(SATA0_CTL_BASE, 0xc),
	DEFINE_RES_IRQ(SATA0_IRQ),
	DEFINE_RES_IRQ(SATA0_DMA_IRQ),
	DEFINE_RES_DMA(0),
};

static struct platform_device tangox_sata0_device = {
	.name		= "tangox-sata",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(tangox_sata0_resources),
	.resource	= tangox_sata0_resources,
};

static struct resource tangox_sata1_resources[] = {
	DEFINE_RES_MEM(SATA1_BASE, 0x800),
	DEFINE_RES_MEM(SATA1_CTL_BASE, 0xc),
	DEFINE_RES_IRQ(SATA1_IRQ),
	DEFINE_RES_IRQ(SATA1_DMA_IRQ),
	DEFINE_RES_DMA(1),
};

static struct platform_device tangox_sata1_device = {
	.name		= "tangox-sata",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(tangox_sata1_resources),
	.resource	= tangox_sata1_resources,
};

static struct resource tangox_sata_phy_resources[] = {
	DEFINE_RES_MEM(SATA0_CTL_BASE + 0xc, 0x10),
};

static struct tangox_sata_phy_pdata tangox_sata_phy_pdata;

static struct platform_device tangox_sata_phy_device = {
	.name		= "tangox-sata-phy",
	.id		= -1,
	.dev		= {
		.platform_data		= &tangox_sata_phy_pdata,
	},
	.num_resources	= ARRAY_SIZE(tangox_sata_phy_resources),
	.resource	= tangox_sata_phy_resources,
};

static struct platform_device *tangox_sata_devices[] = {
	&tangox_sata_phy_device,
	&tangox_sata0_device,
	&tangox_sata1_device,
};

static int __init tangox_sata_register(void)
{
	if (!tangox_sata_enabled())
		return 0;

	tangox_sata_cfg(&tangox_sata_phy_pdata);

	return platform_add_devices(tangox_sata_devices,
				    ARRAY_SIZE(tangox_sata_devices));
}
device_initcall(tangox_sata_register);

static int tangox_phy_init(struct phy_device *phydev)
{
	int err;

	err = phy_write(phydev, 31, 1);
	if (!err)
		err = phy_write(phydev, 28, 0x5000);
	phy_write(phydev, 31, 0);

	return err;
}

static void __init tangox_enet_set_bw(void __iomem *reg)
{
	u32 val = readl(reg);
	val = (val & ~0xff) | 0x3f;
	writel(val, reg);
}

static int __init tangox_enet_register(void)
{
	void __iomem *ctl = ioremap(SYS_BASE + 0x130, 8);
	int i;

	phy_register_fixup_for_uid(0x00070420, 0xfffffff0, tangox_phy_init);

	for (i = 0; i < 2; i++)
		if (tangox_ethernet_enabled(i))
			tangox_enet_set_bw(ctl + i * 4);

	return 0;
}
device_initcall(tangox_enet_register);
