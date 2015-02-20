/*
 * Modified for SMP86XX.
 *
 * Copyright (c) 2004-2008 Sigma Designs, Inc.
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

#ifdef	CONFIG_PM
static int tangox_suspend(struct device *dev, pm_message_t state, u32 phase);
static int tangox_resume(struct device *dev, u32 phase);
#endif

extern unsigned long tangox_otg_bits;

/* called during probe() after chip reset completes */
static int ehci_tangox_setup(struct usb_hcd *hcd)
{
        struct ehci_hcd         *ehci = hcd_to_ehci(hcd);
        u32                     temp;
        int                     retval;

        ehci->caps = hcd->regs;
        ehci->regs = hcd->regs + HC_LENGTH(readl(&ehci->caps->hc_capbase));
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

        if (ehci_is_TDI(ehci))
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

        return retval;
}

static const struct hc_driver ehci_tangox_hc_driver = {
	.description =		EHCI_HCD_NAME,
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
	.suspend =		tangox_suspend,
	.resume =		tangox_resume,
#endif
	.stop =			ehci_stop,
	.shutdown =		ehci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue =		ehci_urb_enqueue,
	.urb_dequeue =		ehci_urb_dequeue,
	.endpoint_disable =	ehci_endpoint_disable,

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
};

/*-------------------------------------------------------------------------*/

int tangox_hcd_probe (struct platform_device *pdev)
{
	unsigned long tangox_chip_id(void);
	unsigned long chip_id = (tangox_chip_id() >> 16) & 0xfffe;
        struct usb_hcd          *hcd;
        struct ehci_hcd         *ehci;
        int                     retval;

        tangox_usb_init();

        if (usb_disabled())
                return -ENODEV;

        hcd = usb_create_hcd (&ehci_tangox_hc_driver, &pdev->dev, pdev->dev.bus_id);
        if (!hcd) {
			printk("cannot create hcd\n");
                retval = -ENOMEM;
                goto err1;
        }

        ehci = hcd_to_ehci(hcd);

	hcd->rsrc_start = NON_CACHED(TANGOX_EHCI_BASE_ADDR);
	hcd->regs = (void *)NON_CACHED(TANGOX_EHCI_BASE_ADDR);

	if (chip_id == 0x8652) {
		if (test_and_set_bit(0, &tangox_otg_bits) != 0) {
			printk("OTG is used in different mode.\n");
			return -EIO;
		}

		hcd->rsrc_start += TANGOX_EHCI_REG_OFFSET;
		hcd->regs += TANGOX_EHCI_REG_OFFSET;
		/* TT is available with this controller */
		ehci->is_tdi_rh_tt = 1;
	}

        hcd->irq = TANGOX_EHCI_IRQ;
        hcd->self.controller = &pdev->dev;
        hcd->self.bus_name = pdev->dev.bus_id;
        hcd->product_desc ="TangoX USB 2.0";

        retval = usb_add_hcd (hcd, TANGOX_EHCI_IRQ, IRQF_DISABLED);
        if (retval != 0)
                goto err2;
        return retval;
err2:
        usb_put_hcd (hcd);
err1:
        dev_err (&pdev->dev, "init %s fail, %d\n", EHCI_HCD_NAME, retval);
        return retval;
}

int tangox_hcd_remove (struct platform_device *pdev)
{
	unsigned long tangox_chip_id(void);
	unsigned long chip_id = (tangox_chip_id() >> 16) & 0xfffe;
        struct usb_hcd *hcd = dev_get_drvdata(&pdev->dev);
        if (!hcd)
                return -1;

        usb_remove_hcd (hcd);
        usb_put_hcd (hcd);
	if (chip_id == 0x8652) {
		clear_bit(0, &tangox_otg_bits);
	}
        return 0;
}

#ifdef	CONFIG_PM
static int tangox_suspend(struct device *dev, pm_message_t state, u32 phase)
{
        printk("TangoX EHCI suspend.\n");
        return 0;
}

static int tangox_resume(struct device *dev, u32 phase)
{
        printk("TangoX EHCI resume.\n");
        return 0;
}
#endif

static struct platform_driver ehci_tangox_driver = {
        .probe   =      tangox_hcd_probe,
        .remove  =      tangox_hcd_remove,
		.shutdown = usb_hcd_platform_shutdown,
		.driver = {
			.name = (char *)EHCI_HCD_NAME,
			.bus = &platform_bus_type
		},
#ifdef CONFIG_PM 
        .suspend =      tangox_suspend,
        .resume  =      tangox_resume,
#endif
};

static u64 ehci_dmamask = ~(u32)0;
static void tangox_ehci_release_dev(struct device * dev)
{
        dev->parent = NULL;
}


static struct platform_device tangox_ehci_device = {
        .name           = (char *)EHCI_HCD_NAME,
        .id             = -1,
        .dev = {
                .dma_mask               = &ehci_dmamask,
                .coherent_dma_mask      = 0xffffffff,
                .release                = tangox_ehci_release_dev,
        },
       .num_resources  = 0,
       .resource       = 0,

};

static struct platform_device *tangox_platform_devices[] __initdata = {
        &tangox_ehci_device,
};
