#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>

#include "irq.h"
#include "memmap.h"
#include "setup.h"

static u64 ehci_dmamask = DMA_BIT_MASK(32);

static struct resource tangox_ehci_resources[] = {
	DEFINE_RES_MEM(USB0_EHCI_BASE, 0x100),
	DEFINE_RES_IRQ(USB0_EHCI_IRQ),
};

static struct platform_device tangox_ehci_device = {
	.name		= "ehci-platform",
	.id		= 0,
	.dev = {
		.dma_mask		= &ehci_dmamask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
	},
	.num_resources	= ARRAY_SIZE(tangox_ehci_resources),
	.resource	= tangox_ehci_resources,
};

static u64 ohci_dmamask = DMA_BIT_MASK(32);

static struct resource tangox_ohci_resources[] = {
	DEFINE_RES_MEM(USB0_OHCI_BASE, 0x100),
	DEFINE_RES_IRQ(USB0_OHCI_IRQ),
};

static struct platform_device tangox_ohci_device = {
	.name		= "ohci-platform",
	.id		= 0,
	.dev = {
		.dma_mask		= &ohci_dmamask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
	},
	.num_resources	= ARRAY_SIZE(tangox_ohci_resources),
	.resource	= tangox_ohci_resources,
};

static struct resource tangox_usb_phy_resources[] = {
	DEFINE_RES_MEM(USB0_PHY_BASE, 0x10),
};

static struct platform_device tangox_usb_phy_device = {
	.name		= "tangox-usb-phy",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(tangox_usb_phy_resources),
	.resource	= tangox_usb_phy_resources,
};

static struct platform_device *tangox_usb_devices[] __initdata = {
	&tangox_usb_phy_device,
	&tangox_ehci_device,
	&tangox_ohci_device,
};

static int __init tangox_usb_register(void)
{
	if (!tangox_usb_enabled())
		return 0;

	return platform_add_devices(tangox_usb_devices,
				    ARRAY_SIZE(tangox_usb_devices));
}
device_initcall(tangox_usb_register);
