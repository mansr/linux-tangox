#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/platform_data/dma-tangox.h>
#include <linux/platform_data/sata-tangox-phy.h>
#include <linux/platform_data/eth-tangox.h>
#include <linux/phy.h>
#include <asm/io.h>

#include "irq.h"
#include "memmap.h"
#include "setup.h"

#define SBOX_DMA0		1
#define SBOX_DMA1		2
#define SBOX_PCI_MASTER		3
#define SBOX_PCI_SLAVE		4
#define SBOX_SATA0		5
#define SBOX_IDE_ISA		6
#define SBOX_IDE_DVD		7
#define SBOX_SATA1		8
#define SBOX_DMA2		9

static u64 dma_dmamask = DMA_BIT_MASK(32);

static struct tangox_dma_chan_data tangox_dma_chans[] = {
	DEFINE_DMA_CHAN(DMA_MEM_TO_DEV, DMA_R0_BASE, DMA_R0_IRQ, SBOX_DMA0),
	DEFINE_DMA_CHAN(DMA_DEV_TO_MEM, DMA_W0_BASE, DMA_W0_IRQ, SBOX_DMA0),
	DEFINE_DMA_CHAN(DMA_MEM_TO_DEV, DMA_R2_BASE, DMA_R2_IRQ, SBOX_DMA2),
	DEFINE_DMA_CHAN(DMA_DEV_TO_MEM, DMA_W2_BASE, DMA_W2_IRQ, SBOX_DMA2),
};

static int tangox_dma_slaves[] = {
	SBOX_SATA0,
	SBOX_SATA1,
};

static struct tangox_dma_pdata tangox_dma_pdata = {
	.num_chans	= ARRAY_SIZE(tangox_dma_chans),
	.chan		= tangox_dma_chans,
	.num_slaves	= ARRAY_SIZE(tangox_dma_slaves),
	.slave_id	= tangox_dma_slaves,
};

static struct resource tangox_dma_resources[] = {
	DEFINE_RES_MEM(SBOX_BASE, 0x10),
};

static struct platform_device tangox_dma_device = {
	.name		= "tangox-dma",
	.id		= -1,
	.dev		= {
		.dma_mask		= &dma_dmamask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
		.platform_data		= &tangox_dma_pdata,
	},
	.num_resources	= ARRAY_SIZE(tangox_dma_resources),
	.resource	= tangox_dma_resources,
};

static int __init tangox_dma_register(void)
{
	return platform_device_register(&tangox_dma_device);
}
device_initcall(tangox_dma_register);

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

static u64 enet0_dmamask = DMA_BIT_MASK(32);

static struct resource tangox_enet0_resourses[] = {
	DEFINE_RES_MEM(ETH0_BASE, 0x800),
	DEFINE_RES_IRQ(ETH0_MAC_IRQ),
};

static struct tangox_enet_pdata tangox_enet0_pdata;

static struct platform_device tangox_enet0_device = {
	.name		= "tangox-enet",
	.id		= 0,
	.dev		= {
		.platform_data		= &tangox_enet0_pdata,
		.dma_mask		= &enet0_dmamask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
	},
	.num_resources	= ARRAY_SIZE(tangox_enet0_resourses),
	.resource	= tangox_enet0_resourses,
};

static u64 enet1_dmamask = DMA_BIT_MASK(32);

static struct resource tangox_enet1_resourses[] = {
	DEFINE_RES_MEM(ETH1_BASE, 0x800),
	DEFINE_RES_IRQ(ETH1_MAC_IRQ),
};

static struct tangox_enet_pdata tangox_enet1_pdata;

static struct platform_device tangox_enet1_device = {
	.name		= "tangox-enet",
	.id		= 1,
	.dev		= {
		.platform_data		= &tangox_enet1_pdata,
		.dma_mask		= &enet1_dmamask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
	},
	.num_resources	= ARRAY_SIZE(tangox_enet1_resourses),
	.resource	= tangox_enet1_resourses,
};

static struct platform_device *tangox_enet_devices[] = {
	&tangox_enet0_device,
	&tangox_enet1_device,
};

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
	int gigabit;
	int i;

	phy_register_fixup_for_uid(0x00070420, 0xfffffff0, tangox_phy_init);

	if (is_tangox_chip(0x8656, 0xfffe))
		gigabit = 1;
	else if (is_tangox_chip_rev(0x8646, 0xfff3, 2) ||
		 is_tangox_chip(0x8670, 0xfff0) ||
		 is_tangox_chip(0x8680, 0xfff0))
		gigabit = 3;
	else
		gigabit = 0;

	for (i = 0; i < 2; i++) {
		struct platform_device *pdev = tangox_enet_devices[i];
		struct tangox_enet_pdata *pdata = pdev->dev.platform_data;

		if (tangox_ethernet_enabled(i)) {
			tangox_enet_set_bw(ctl + i * 4);
			tangox_ethernet_getmac(i, pdata->mac_addr);
			pdata->gigabit = (gigabit >> i) & 1;
			platform_device_register(pdev);
		}
	}

	return 0;
}
device_initcall(tangox_enet_register);
