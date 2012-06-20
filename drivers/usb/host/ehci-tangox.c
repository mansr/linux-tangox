/*
 * Modified for SMP8XXX.
 *
 * Copyright (c) 2004-2011 Sigma Designs, Inc.
 *
 * EHCI HCD (Host Controller Driver) PCI Bus Glue.
 *
 * Copyright (c) 2000-2004 by David Brownell
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/platform_device.h>
#ifndef CONFIG_TANGOX
#error "This file is TANGOX EHCI bus glue.  CONFIG_TANGOX must be defined."
#endif

extern unsigned long tangox_otg_bits;

/* called during probe() after chip reset completes */
static int ehci_tangox_setup(struct usb_hcd *hcd)
{
	struct ehci_hcd         *ehci = hcd_to_ehci(hcd);
	u32                     temp;
	int                     retval;

	ehci->caps = hcd->regs;
	ehci->regs = hcd->regs + HC_LENGTH(ehci, readl(&ehci->caps->hc_capbase));
	dbg_hcs_params(ehci, "reset");
	dbg_hcc_params(ehci, "reset");

	/* cache this readonly data; minimize chip reads */
	ehci->hcs_params = readl(&ehci->caps->hcs_params);

	retval = ehci_halt(ehci);
	if (retval)
		return retval;

	/* data structure init */
	retval = ehci_init(hcd);
	if (retval)
		return retval;

	ehci_reset(ehci);

	/* at least the Genesys GL880S needs fixup here */
	temp = HCS_N_CC(ehci->hcs_params) * HCS_N_PCC(ehci->hcs_params);
	temp &= 0x0f;
	if (temp && (HCS_N_PORTS(ehci->hcs_params) > temp)) {
		ehci_dbg(ehci, "bogus port configuration: "
			"cc=%d x pcc=%d < ports=%d\n",
			HCS_N_CC(ehci->hcs_params),
			HCS_N_PCC(ehci->hcs_params),
			HCS_N_PORTS(ehci->hcs_params));
	}

	ehci_port_power(ehci, 1);

	return retval;
}


static const struct hc_driver ehci_tangox_hc_driver = {
	.description =		hcd_name,
	.product_desc =		"SMP86xx EHCI Host Controller",
	.hcd_priv_size =	sizeof(struct ehci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq =			ehci_irq,
	.flags =		HCD_MEMORY | HCD_USB2,

	/*
	 * basic lifecycle operations
	 */
	.reset =		ehci_tangox_setup,
	.start =		ehci_run,
#ifdef	CONFIG_PM
	.bus_suspend =	 ehci_bus_suspend,
	.bus_resume =	 ehci_bus_resume,
#endif
	.stop =			ehci_stop,
	.shutdown =		ehci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue =		ehci_urb_enqueue,
	.urb_dequeue =		ehci_urb_dequeue,
	.endpoint_disable =	ehci_endpoint_disable,
	.endpoint_reset = ehci_endpoint_reset,

	/*
	 * scheduling support
	 */
	.get_frame_number =	ehci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data =	ehci_hub_status_data,
	.hub_control =		ehci_hub_control,
	.bus_suspend =		ehci_bus_suspend,
	.bus_resume =		ehci_bus_resume,
	.relinquish_port = ehci_relinquish_port,
	.port_handed_over = ehci_port_handed_over,

	.clear_tt_buffer_complete = ehci_clear_tt_buffer_complete,
};

/*-------------------------------------------------------------------------*/

int tangox_hcd_probe (struct platform_device *pdev, int ctrl)
{
	unsigned long tangox_chip_id(void);
	unsigned long chip_id = (tangox_chip_id() >> 16) & 0xfffe;
	struct usb_hcd          *hcd;
	struct ehci_hcd         *ehci;
	int retval = 0;
	int irq,err;
	struct resource *res;

	printk("Initializing Tangox EHCI USB Host Controller\n");
	if (usb_disabled())
		return -ENODEV;

	irq = platform_get_irq(pdev, 0);
	if (irq <= 0) {
		dev_err(&pdev->dev,
			"Found HC with no IRQ. Check %s setup!\n",
			dev_name(&pdev->dev));
		err = -ENODEV;
		goto err1;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev,
			"Found HC with no register addr. Check %s setup!\n",
			dev_name(&pdev->dev));
		err = -ENODEV;
		goto err1;
	}

	hcd = usb_create_hcd (&ehci_tangox_hc_driver, &pdev->dev, dev_name(&pdev->dev));
	if (!hcd) {
		printk("cannot create hcd\n");
		retval = -ENOMEM;
		goto err1;
	}

	ehci = hcd_to_ehci(hcd);
	hcd->rsrc_start = NON_CACHED(res->start);
	hcd->regs = (void *) NON_CACHED(res->start);

	if ((chip_id == 0x8652) || ((chip_id & 0xfff0) == 0x8670)) {
		if (ctrl == 0) {
			if (test_and_set_bit(0, &tangox_otg_bits) != 0) {
				printk("Controller %d is used in different mode.\n", ctrl);
				return -EIO;
			}
		} else if ((chip_id & 0xfff0) == 0x8670) {
			if (test_and_set_bit(1, &tangox_otg_bits) != 0) {
				printk("Controller %d is used in different mode.\n", ctrl);
				return -EIO;
			}
		}
		
		hcd->rsrc_start += TANGOX_EHCI_REG_OFFSET;
		hcd->regs += TANGOX_EHCI_REG_OFFSET;
		/* TT is available with this controller */
		hcd->has_tt = 1;
	} else if ((chip_id == 0x8646) || (chip_id == 0x8910)) {
		hcd->rsrc_start += TANGOX_EHCI_REG_OFFSET;
		hcd->regs += TANGOX_EHCI_REG_OFFSET;
		/* TT is available with this controller */
		hcd->has_tt = 1;
	}

#ifdef CONFIG_TANGO3
	if (((chip_id == 0x8672) || (chip_id == 0x8674)) && (ctrl == 1))
		hcd->irq = IRQ_CONTROLLER_IRQ_BASE + LOG2_CPU_DVD_INT;
	else
#endif
		hcd->irq = irq;
	hcd->self.controller = &pdev->dev;
	hcd->self.bus_name = hcd_name;
	hcd->product_desc ="TangoX USB 2.0";

	tangox_usb_init(ctrl);

	retval = usb_add_hcd(hcd, hcd->irq, 0);
	if (retval != 0)
		goto err2;


	return retval;
err2:
	usb_put_hcd (hcd);
err1:
	dev_err (&pdev->dev, "init %s fail, %d\n", EHCI_HCD_NAME, retval);
	return retval;
}

int tangox_hcd_remove (struct platform_device *pdev, int ctrl)
{
	unsigned long tangox_chip_id(void);
	unsigned long chip_id = (tangox_chip_id() >> 16) & 0xfffe;
	struct usb_hcd *hcd = dev_get_drvdata(&pdev->dev);

	if (!hcd)
		return -1;

	usb_remove_hcd (hcd);
	usb_put_hcd (hcd);
	if ((chip_id == 0x8652) || ((chip_id & 0xfff0) == 0x8670)) {
		if (ctrl == 0)
			clear_bit(0, &tangox_otg_bits);
		else if ((chip_id & 0xfff0) == 0x8670)
			clear_bit(1, &tangox_otg_bits);
	}

	tangox_usb_deinit(ctrl);

	return 0;
}

MODULE_ALIAS("platform:tangox-ehci");

int tangox_hcd_probe0 (struct platform_device *pdev)
{
	return tangox_hcd_probe (pdev, 0);
}
#if defined(CONFIG_TANGO3) || defined(CONFIG_TANGO4)
int tangox_hcd_probe1 (struct platform_device *pdev)
{
	return tangox_hcd_probe (pdev, 1);
}
#endif
int tangox_hcd_remove0(struct platform_device *pdev)
{
	return tangox_hcd_remove (pdev, 0);
}
#if defined(CONFIG_TANGO3) || defined(CONFIG_TANGO4)
int tangox_hcd_remove1(struct platform_device *pdev)
{
	return tangox_hcd_remove (pdev, 1);

}
#endif
static struct platform_driver ehci_tangox_driver0 = {
        .probe   =      tangox_hcd_probe0,
        .remove  =      tangox_hcd_remove0,
		.shutdown = usb_hcd_platform_shutdown,
		.driver.name	= (char *)TANGOX_EHCI_NAME0,
		.driver.bus = &platform_bus_type
};

#if defined(CONFIG_TANGO3) 
static struct platform_driver ehci_tangox_driver1 = {
        .probe   =      tangox_hcd_probe1,
        .remove  =      tangox_hcd_remove1,
		.shutdown = usb_hcd_platform_shutdown,
		.driver.name	= (char *)TANGOX_EHCI_NAME1,
		.driver.bus = &platform_bus_type
};
#endif
