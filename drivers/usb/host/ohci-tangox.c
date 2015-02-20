/*
 * OHCI HCD (Host Controller Driver) for TangoX USB 1.1.
 *
 * (C) Copyright 1999 Roman Weissgaerber <weissg@vienna.at>
 * (C) Copyright 2000-2002 David Brownell <dbrownell@users.sourceforge.net>
 * (C) Copyright 2002 Hewlett-Packard Company
 * (C) Copyright 2003-2005 MontaVista Software Inc.
 * (C) Copyright 2004-2008 Sigma Designs, Inc.
 * 
 * Bus Glue for TANGOX OHCI driver. Sigma Designs, Inc.
 * This file is licenced under the GPL.
 */

#include <linux/platform_device.h>

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

	hcd = usb_create_hcd(driver, &pdev->dev, "TANGOX OHCI");
	if (!hcd)
		return -ENOMEM;
	hcd->rsrc_start = NON_CACHED(TANGOX_OHCI_BASE_ADDR);
	hcd->regs =(void *)NON_CACHED(TANGOX_OHCI_BASE_ADDR);
	pr_debug("hcd->register=0x%x\n", (unsigned int)hcd->regs);

	ohci = hcd_to_ohci(hcd);
	ohci_hcd_init(ohci);

	retval = usb_add_hcd(hcd, TANGOX_OHCI_IRQ, IRQF_DISABLED);
	if (retval == 0){
		if(polling_mode){
	        	init_timer(&ohci_timer);
		     	ohci_timer.function = ohci_polling;
		    	ohci_timer.data = (unsigned long)hcd;
			mod_timer(&ohci_timer, jiffies + polling_interval);
		}
		return retval;
	}

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
	if(polling_mode)
	        del_timer_sync(&ohci_timer);
	
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

/*since we cannot set read only irq member so 
  that copy this struct for polling mode here*/
static const struct hc_driver tangox_ohci_polling_hc_driver = {
	.description =		OHCI_HCD_NAME,
	.hcd_priv_size =	sizeof(struct ohci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq =			NULL,
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
#ifdef	CONFIG_USB_SUSPEND
	.hub_suspend =		ohci_hub_suspend,
	.hub_resume =		ohci_hub_resume,
#endif
	.start_port_reset =	ohci_start_port_reset,
};


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
#ifdef	CONFIG_USB_SUSPEND
	.hub_suspend =		ohci_hub_suspend,
	.hub_resume =		ohci_hub_resume,
#endif
	.start_port_reset =	ohci_start_port_reset,
};

static int tangox_ohci_drv_probe(struct platform_device *pdev)
{
	unsigned int temp= 0 ;
	int ret;

	if (usb_disabled())
		return -ENODEV;

	tangox_usb_init();

	/*check if need switch to polling mode 
	  The OHCI register reset bug should be resolved in ES8 or above,
	  as the result, using interrupt mode instead.
	*/
	if(is_tango2_es89() || is_tango3_chip())
		polling_mode = 0;
	else {
		temp = gbus_read_reg32(TANGOX_USB_CTL_STATUS_REG_BASE + 0x8);
		if(polling_mode == 0){
			if (temp & 0x7)
				polling_mode = 1;
			else
				polling_mode = 0;
		}
	}

	if(polling_mode == 0)
		printk("Initializing TangoX USB OHCI Controller Membase=0x%x, irq=%d\n", 
				NON_CACHED(TANGOX_OHCI_BASE_ADDR),TANGOX_OHCI_IRQ);
	else 
		printk("Initializing TangoX USB OHCI Controller Polling mode, Membase=0x%x Status=0x%x\n",
				NON_CACHED(TANGOX_OHCI_BASE_ADDR), temp);

	if(polling_mode)
		ret = tangox_ohci_probe(&tangox_ohci_polling_hc_driver, pdev);
	else
		ret = tangox_ohci_probe(&tangox_ohci_hc_driver, pdev);

	return ret;
}

static int tangox_ohci_drv_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = dev_get_drvdata(&pdev->dev);

	tangox_ohci_remove(hcd, pdev);
	return 0;
}

static void tangox_ohci_release_dev(struct device * dev)
{
        dev->parent = NULL;
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


/* The dmamask must be set for OHCI to work */
static u64 ohci_dmamask = ~(u32)0;
static struct platform_device tangox_ohci_device = {
	.name           = OHCI_HCD_NAME,
	.id             = -1,
	.dev = {
		.dma_mask               = &ohci_dmamask,
		.coherent_dma_mask      = 0xffffffff,
		.release				= tangox_ohci_release_dev,
	},
	.num_resources  = 0,
	.resource       = 0,
};

static struct platform_device *tangox_platform_devices[] __initdata = {
	&tangox_ohci_device,
};

