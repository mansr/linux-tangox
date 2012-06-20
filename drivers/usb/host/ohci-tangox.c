/*
 * OHCI HCD (Host Controller Driver) for TangoX USB 1.1.
 *
 * (C) Copyright 1999 Roman Weissgaerber <weissg@vienna.at>
 * (C) Copyright 2000-2002 David Brownell <dbrownell@users.sourceforge.net>
 * (C) Copyright 2002 Hewlett-Packard Company
 * (C) Copyright 2003-2005 MontaVista Software Inc.
 * (C) Copyright 2004-2011 Sigma Designs, Inc.
 * 
 * Bus Glue for TANGOX OHCI driver. Sigma Designs, Inc.
 * This file is licenced under the GPL.
 */

#include <linux/platform_device.h>
#if defined(CONFIG_TANGO2)
#include <asm/tango2/platform_dev.h>
#elif defined(CONFIG_TANGO3)
#include <asm/tango3/platform_dev.h>
#elif defined(CONFIG_TANGO4)
#include <asm/tango4/platform_dev.h>
#endif

/**
 * tangox_ohci_probe - initialize On-Chip HCDs
 * Context: !in_interrupt()
 *
 * Allocates basic resources for this USB host controller.
 *
 * Store this function in the HCD's struct pci_driver as probe().
 */
static int tangox_ohci_probe(const struct hc_driver *driver,
			  struct platform_device *pdev)
{
	int retval;
	struct usb_hcd *hcd;
	struct ohci_hcd	*ohci;
	int irq;
	struct resource *res;

	irq = platform_get_irq(pdev, 0);
	if (irq <= 0) {
		dev_err(&pdev->dev,
			"Found HC with no IRQ. Check %s setup!\n",
			dev_name(&pdev->dev));
		return -ENODEV;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev,
			"Found HC with no register addr. Check %s setup!\n",
			dev_name(&pdev->dev));
		return -ENODEV;
	}

	hcd = usb_create_hcd (driver, &pdev->dev, dev_name(&pdev->dev));
	if (!hcd)
		return -ENOMEM;
	hcd->rsrc_start = NON_CACHED(res->start);
	hcd->regs = (void *) NON_CACHED(res->start);
	pr_debug("hcd->register=0x%x\n", (unsigned int)hcd->regs);

	ohci = hcd_to_ohci(hcd);
	ohci_hcd_init(ohci);

	/* reset before request irq */
	ohci_writel (ohci, 0, hcd->regs + 4);
	ohci_readl (ohci, hcd->regs + 4);
	msleep(50);

	retval = usb_add_hcd(hcd, irq, 0);
	if (retval == 0)
		return retval;

	pr_debug("Removing TANGOX USB OHCI Controller\n");
 	usb_put_hcd(hcd);

	return retval;
}


/* may be called without controller electrically present */
/* may be called with controller, bus, and devices active */

/**
 * tangox_ohci_remove - shutdown processing for On-Chip HCDs
 * @pdev: USB Host Controller being removed
 * Context: !in_interrupt()
 *
 * Reverses the effect of tangox_ohci_probe().
 * It is always called from a thread
 * context, normally "rmmod", "apmd", or something similar.
 *
 */
static void tangox_ohci_remove(struct usb_hcd *hcd,
		struct platform_device *pdev)
{
	usb_remove_hcd(hcd);

	pr_debug("stopping TANGOX USB OHCI Controller\n");
	usb_put_hcd(hcd);
}

static int __devinit
tangox_ohci_start(struct usb_hcd *hcd)
{
	struct ohci_hcd	*ohci = hcd_to_ohci(hcd);
	int		ret;

	if ((ret = ohci_init(ohci)) < 0)
		return ret;

	if ((ret = ohci_run(ohci)) < 0) {
		err("can't start %s", ohci_to_hcd(ohci)->self.bus_name);
		ohci_stop(hcd);
		return ret;
	}

	return 0;
}


static const struct hc_driver tangox_ohci_hc_driver = {
	.description =		OHCI_HCD_NAME,
	.hcd_priv_size =	sizeof(struct ohci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq =			ohci_irq,
	.flags =		HCD_USB11 | HCD_MEMORY,

	/*
	 * basic lifecycle operations
	 */
	.start =		tangox_ohci_start,
	.stop =			ohci_stop,
	.shutdown =		ohci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue =		ohci_urb_enqueue,
	.urb_dequeue =		ohci_urb_dequeue,
	.endpoint_disable =	ohci_endpoint_disable,

	/*
	 * scheduling support
	 */
	.get_frame_number =	ohci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data =	ohci_hub_status_data,
	.hub_control =		ohci_hub_control,
#if defined(CONFIG_PM)
	.bus_suspend 		= ohci_bus_suspend,
	.bus_resume 		= ohci_bus_resume,
#endif
	.start_port_reset =	ohci_start_port_reset,
};

static int tangox_ohci_drv_probe(struct platform_device *pdev)
{
	int ret;

	if (usb_disabled())
		return -ENODEV;

	tangox_usb_init(0);

	printk("Initializing TangoX USB OHCI Controller Membase=0x%x, irq=%d\n", 
			NON_CACHED(TANGOX_OHCI0_BASE),TANGOX_OHCI0_IRQ);

	ret = tangox_ohci_probe(&tangox_ohci_hc_driver, pdev);

	return ret;
}

static int tangox_ohci_drv_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = dev_get_drvdata(&pdev->dev);

	tangox_ohci_remove(hcd, pdev);
	tangox_usb_deinit(0);
	return 0;
}

static struct platform_driver tangox_ohci_driver = {
	.probe		= tangox_ohci_drv_probe,
	.remove		= tangox_ohci_drv_remove,
	.driver = {
		.name = (char *)OHCI_HCD_NAME,
		.bus = &platform_bus_type
	},

#if	defined(CONFIG_USB_SUSPEND) || defined(CONFIG_PM)
	.suspend	= NULL,
	.resume		= NULL,
#endif
};
