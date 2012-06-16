/*
 *
 * Copyright (C) 2007-2011 Sigma Designs, Inc.
 * arch/mips/tangox/platform.c
 *     The platform device file for tango2/tango3/tango4.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
#include <linux/init.h>
#include <linux/delay.h>
#include <asm/time.h>
#include <linux/platform_device.h>
#if defined(CONFIG_TANGO2)
#include <asm/tango2/platform_dev.h>
#elif defined(CONFIG_TANGO3)
#include <asm/tango3/platform_dev.h>
#elif defined(CONFIG_TANGO4)
#include <asm/tango4/platform_dev.h>
#endif

static u64 dmamask = ~(u32)0;
static void tangox_release_dev(struct device * dev)
{
        dev->parent = NULL;
}

#if defined(CONFIG_SATA_TANGOX) || defined(CONFIG_SATA_TANGOX_MODULE)
#if defined(CONFIG_TANGO3) || defined(CONFIG_TANGO4)
/* SATA platform device */
static struct resource sata_resources0[] = {
	{
		.start	= TANGOX_SATA0_BASE,
		.end	= TANGOX_SATA0_BASE	+ 0xff,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= TANGOX_SATA_IRQ0,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource sata_resources1[] = {
	{
		.start	= TANGOX_SATA1_BASE,
		.end	= TANGOX_SATA1_BASE	+ 0xff,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= TANGOX_SATA_IRQ1,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device tangox_sata_device0 = {
	.name			= (char *)DRV_NAME0,
	.id				= -1,
	.dev = {
		.dma_mask               = &dmamask,
		.coherent_dma_mask      = 0xffffffff,
		.release	= tangox_release_dev,
	},
	.num_resources  = ARRAY_SIZE(sata_resources0),
	.resource       = sata_resources0,
};

static struct platform_device tangox_sata_device1 = {
	.name			= (char *)DRV_NAME1,
	.id				= -1,
	.dev = {
		.dma_mask               = &dmamask,
		.coherent_dma_mask      = 0xffffffff,
		.release	= tangox_release_dev,
	},
	.num_resources  = ARRAY_SIZE(sata_resources1),
	.resource       = sata_resources1,

};

static void tangox_init_sata(void)
{
	platform_device_register(&tangox_sata_device0);
	platform_device_register(&tangox_sata_device1);
}
#endif
#endif /* CONFIG_SATA_TANGOX || CONFIG_SATA_TANGOX_MODULE */

#if defined(CONFIG_USB_EHCI_HCD) || defined(CONFIG_USB_EHCI_HCD_MODULE)
#if defined(CONFIG_TANGOX_EHCI_HCD) || defined(CONFIG_TANGOX_EHCI_HCD_MODULE)
/* EHCI platform devices */
static struct resource ehci_resources0[] = {
	{
		.start	=  TANGOX_EHCI0_BASE ,
		.end	= TANGOX_EHCI0_BASE  + 0xff,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= TANGOX_EHCI0_IRQ ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device tangox_ehci_device0 = {
        .name           = (char *)TANGOX_EHCI_NAME0,
        .id             = -1,
        .dev = {
                .dma_mask               = &dmamask,
                .coherent_dma_mask      = 0xffffffff,
                .release                = tangox_release_dev,
        },
       .num_resources  = ARRAY_SIZE(ehci_resources0),
       .resource       = ehci_resources0,

};

#if defined(CONFIG_TANGO3)
static struct resource ehci_resources1[] = {
	{
		.start	=  TANGOX_EHCI1_BASE ,
		.end	= TANGOX_EHCI1_BASE  + 0xff,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= TANGOX_EHCI1_IRQ ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device tangox_ehci_device1 = {
        .name           = (char *)TANGOX_EHCI_NAME1,
        .id             = -1,
        .dev = {
                .dma_mask               = &dmamask,
                .coherent_dma_mask      = 0xffffffff,
                .release                = tangox_release_dev,
        },
       .num_resources  = ARRAY_SIZE(ehci_resources1),
       .resource       = ehci_resources1,

};
#endif

static void tangox_init_ehci(void)
{
#if defined(CONFIG_TANGO3)
	unsigned long tangox_chip_id(void);
	unsigned long chip_id = (tangox_chip_id() >> 16) & 0xfffe;
	platform_device_register(&tangox_ehci_device0);
	if ((chip_id & 0xfff0) == 0x8670){ 
		platform_device_register(&tangox_ehci_device1);
	}
#else
	platform_device_register(&tangox_ehci_device0);
#endif
}
#endif
#endif

#ifdef CONFIG_USB_GADGET_TANGOX
#if defined(CONFIG_TANGO3) || defined(CONFIG_TANGO4)
/* GADGET device, the same resouce as ehci*/
static struct platform_device tangox_udc_device0 = {
	.name           = (char *)TANGOX_UDC_NAME0,
	.id             = -1,
	.dev = {
		.dma_mask               = &dmamask,
		.coherent_dma_mask      = 0xffffffff,
		.release                = tangox_release_dev,
	 },
	.num_resources  = ARRAY_SIZE(ehci_resources0),
	.resource       = ehci_resources0,
};

#ifdef CONFIG_TANGO3
static struct platform_device tangox_udc_device1 = {
	.name           = (char *)TANGOX_UDC_NAME1,
	.id             = -1,
	.dev = {
		.dma_mask               = &dmamask,
		.coherent_dma_mask      = 0xffffffff,
		.release                = tangox_release_dev,
	 },
	.num_resources  = ARRAY_SIZE(ehci_resources1),
	.resource       = ehci_resources1,
};
#endif

static void tangox_init_udc(void)
{
	unsigned long tangox_chip_id(void);
#ifdef CONFIG_TANGO3
	unsigned long chip_id = (tangox_chip_id() >> 16) & 0xfffe;
#endif

	platform_device_register(&tangox_udc_device0);
#ifdef CONFIG_TANGO3
	if ((chip_id & 0xfff0) == 0x8670)
		platform_device_register(&tangox_udc_device1);
#endif
}
#endif
#endif /* CONFIG_USB_GADGET_TANGOX */

#if defined(CONFIG_USB_OHCI_HCD) || defined(CONFIG_USB_OHCI_HCD_MODULE)
#if defined(CONFIG_TANGOX_OHCI_HCD) || defined(CONFIG_TANGOX_OHCI_HCD_MODULE)
/* OHCI platform device */
static struct resource ohci_resources[] = {
	{
		.start	=  TANGOX_OHCI0_BASE ,
		.end	= TANGOX_OHCI0_BASE  + 0xff,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= TANGOX_OHCI0_IRQ ,
		.flags	= IORESOURCE_IRQ,
	},
};

/* The dmamask must be set for OHCI to work */
static struct platform_device tangox_ohci_device = {
	.name           = OHCI_HCD_NAME,
	.id             = -1,
	.dev = {
		.dma_mask               = &dmamask,
		.coherent_dma_mask      = 0xffffffff,
		.release		= tangox_release_dev,
	},
	.num_resources  = ARRAY_SIZE(ohci_resources),
	.resource       = ohci_resources,
};

static void tangox_init_ohci(void)
{
	platform_device_register(&tangox_ohci_device);
}
#endif /* CONFIG_TANGOX_OHCI_HCD */
#endif

static int __init tangox_init_devices(void)
{
#if defined(CONFIG_SATA_TANGOX) || defined(CONFIG_SATA_TANGOX_MODULE)
#if defined(CONFIG_TANGO3) || defined(CONFIG_TANGO4)
	tangox_init_sata();
#endif
#endif
#if defined(CONFIG_USB_EHCI_HCD) || defined(CONFIG_USB_EHCI_HCD_MODULE)
#if defined(CONFIG_TANGOX_EHCI_HCD) || defined(CONFIG_TANGOX_EHCI_HCD_MODULE)
	tangox_init_ehci();
#endif
#endif
#if defined(CONFIG_USB_OHCI_HCD) || defined(CONFIG_USB_OHCI_HCD_MODULE)
#if defined(CONFIG_TANGOX_OHCI_HCD) || defined(CONFIG_TANGOX_OHCI_HCD_MODULE)
	tangox_init_ohci();
#endif
#endif
#ifdef CONFIG_USB_GADGET_TANGOX
#if defined(CONFIG_TANGO3) || defined(CONFIG_TANGO4)
	tangox_init_udc();
#endif
#endif
	return 0;
}
arch_initcall(tangox_init_devices);

