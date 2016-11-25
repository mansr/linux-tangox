/*********************************************************************
 Copyright (C) 2001-2010
 Sigma Designs, Inc.

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/
/*
 * Driver for SMP864x/SMP865x builtin SATA Based on Synopsys DW SATA Host Core
 * device driver and Linux libata driver support layer.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/platform_device.h>
#include <linux/blkdev.h>
#include <linux/libata.h>
#include <linux/hdreg.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/dmaengine.h>
#include <linux/phy/phy.h>
#include <linux/scatterlist.h>
#include <scsi/scsi_host.h>
#include <scsi/scsi.h>
#include <scsi/scsi_device.h>
#include <scsi/scsi_cmnd.h>
#include <scsi/scsi_tcq.h>

#include "libata.h"

MODULE_AUTHOR ("Sigma Designs Inc.");
MODULE_DESCRIPTION ("SMP86xx Built-in SATA Host Controller driver");
MODULE_LICENSE ("GPL");

/*HSATA Registers*/
#define HSATA_SCR0_REG		0x0024
#define HSATA_SCR1_REG		0x0028
#define HSATA_SCR2_REG		0x002C
#define HSATA_SCR3_REG		0x0030
#define HSATA_SCR4_REG		0x0034
#define HSATA_SERROR_REG	HSATA_SCR1_REG
#define HSATA_SCONTROL_REG	HSATA_SCR2_REG
#define HSATA_SACTIVE_REG	HSATA_SCR3_REG
#define HSATA_DMACR_TX_EN	(0x01 /*| HSATA_DMACR_TXMODE_BIT*/)
#define HSATA_DMACR_RX_EN	(0x02 /*| HSATA_DMACR_TXMODE_BIT*/)
#define HSATA_DMACR_TXRX_EN	(0x03 | HSATA_DMACR_TXMODE_BIT)
#define HSATA_DMACR_TXMODE_BIT	0x04
#define HSATA_FEAT_REG		0x0004
#define HSATA_CMD_REG		0x001c
#define HSATA_STATUS_REG	0x001c
#define HSATA_CONTROL_REG	0x0020
#define HSATA_DMACR_REG		0x0070
#define HSATA_DBTSR_REG		0x0074
#define HSATA_INTPR_REG		0x0078
#define HSATA_INTPR_ERR_BIT	0x00000008
#define HSATA_INTPR_FP_BIT	0x00000002
#define HSATA_INTMR_REG		0x007C
#define HSATA_INTMR_ERRM_BIT	0x00000008
#define HSATA_INTMR_NEWFP_BIT	0x00000002
#define HSATA_ERRMR_REG		0x0080
#define HSATA_ERRMR_BITS	0xFFFEF7FF
#define HSATA_VER_REG		0x00F8
#define HSATA_IDR_REG		0x00FC

struct hsata_device
{
	unsigned char *membase;
	unsigned char *ctl_base;
	struct dma_chan *dma_chan;
	int dma_status;
	struct phy *phy;
};

#define TANGOX_BURST_LENGTH_TX	16
#define BURST_LENGTH_TX		24
#define BURST_LENGTH_RX		4 /* was 64 */
#define HSATA_DMA_DBTSR		(BURST_LENGTH_RX << 16 | BURST_LENGTH_TX)

/* Interrupt stuff*/
static void hsata_enable_interrupts(struct hsata_device *hsdev)
{
	u32 val32;

	writel(HSATA_ERRMR_BITS, hsdev->membase + HSATA_ERRMR_REG);

	val32 = readl(hsdev->membase + HSATA_INTMR_REG);
	writel(val32 | HSATA_INTMR_ERRM_BIT | HSATA_INTMR_NEWFP_BIT,
			hsdev->membase + HSATA_INTMR_REG);
}

static void hsata_disable_interrupts(struct hsata_device *hsdev)
{
	writel(0, hsdev->membase + HSATA_INTMR_REG);
}

static int hsata_scr_read(struct ata_link *link, unsigned int sc_reg, u32 *val)
{
	*val = readl(link->ap->ioaddr.scr_addr + (sc_reg * 4));

	return 0;
}

static int hsata_scr_write(struct ata_link *link, unsigned int sc_reg, u32 val)
{
	writel(val, link->ap->ioaddr.scr_addr + (sc_reg * 4));

	return 0;
}

static unsigned int hsata_update_dma_status(struct hsata_device *hsdev)
{
	unsigned int status;

	status = readl(hsdev->ctl_base + 8);

	if (status && hsdev->dma_status == ATA_DMA_ACTIVE)
		hsdev->dma_status = ATA_DMA_INTR;

	return status;
}

static irqreturn_t hsata_dma_isr(int irq, void *dev_id)
{
	struct ata_host *host = dev_id;
	struct hsata_device *hsdev = host->private_data;
	unsigned int status;
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);

	status = hsata_update_dma_status(hsdev);

	if (status) {
		/* clear dma interrupt */
		writel(0, hsdev->ctl_base + 0x08);
		writel(0, hsdev->membase + HSATA_DMACR_REG);
	}

	spin_unlock_irqrestore(&host->lock, flags);

	return IRQ_HANDLED;
}

static irqreturn_t hsata_isr(int irq, void *dev_id)
{
	struct ata_host *host = dev_id;
	struct hsata_device *hsdev = host->private_data;
	struct ata_port *ap = host->ports[0];
	struct ata_queued_cmd *qc;
	unsigned int handled = 0;
	unsigned long flags;
	u32 serror;
	u32 intpr;

	spin_lock_irqsave(&host->lock, flags);

	intpr = readl(hsdev->membase + HSATA_INTPR_REG);
	writel(intpr, hsdev->membase + HSATA_INTPR_REG);

	hsata_update_dma_status(hsdev);

	if (intpr & HSATA_INTPR_ERR_BIT) {
		serror = readl(hsdev->membase + HSATA_SERROR_REG);
		writel(serror, hsdev->membase + HSATA_SERROR_REG);

		/* hotplug */
		if (serror & (SERR_PHYRDY_CHG | SERR_DEV_XCHG)) {
			struct ata_link *link = &ap->link;
			struct ata_eh_info *ehi = &link->eh_info;

			ata_ehi_clear_desc(ehi);
			ata_ehi_hotplugged(ehi);
			ata_ehi_push_desc(ehi, serror & SERR_PHYRDY_CHG ?
				"PHY RDY changed" : "device exchanged");
			ata_port_freeze(ap);
		}

		handled = 1;
	} else {
		qc = ata_qc_from_tag(ap, ap->link.active_tag);
		if (qc)
			handled = ata_bmdma_port_intr(ap, qc);
	}

	spin_unlock_irqrestore(&host->lock, flags);

	return IRQ_RETVAL(handled);
}

static void hsata_irq_clear(struct ata_port *ap)
{
	struct hsata_device *hsdev = ap->host->private_data;

	writel(0, hsdev->ctl_base + 8);
	hsdev->dma_status = 0;

	hsata_enable_interrupts(hsdev);
}

static void hsata_setup_port(struct ata_ioports *port, unsigned char *base)
{
	port->cmd_addr		= base + 0x00;
	port->data_addr		= base + 0x00;

	port->error_addr	= base + 0x04;
	port->feature_addr	= base + 0x04;

	port->nsect_addr	= base + 0x08;

	port->lbal_addr		= base + 0x0c;
	port->lbam_addr		= base + 0x10;
	port->lbah_addr		= base + 0x14;

	port->device_addr	= base + 0x18;

	port->command_addr	= base + 0x1c;
	port->status_addr	= base + 0x1c;

	port->altstatus_addr	= base + 0x20;
	port->ctl_addr		= base + 0x20;
	port->scr_addr		= base + HSATA_SCR0_REG;
	port->bmdma_addr	= NULL;
}

static int hsata_port_start(struct ata_port *ap)
{
	struct hsata_device *hsdev = ap->host->private_data;

	writel(0, hsdev->membase + HSATA_DMACR_REG);

	return 0;
}

static unsigned long get_dma_len(struct ata_queued_cmd *qc)
{
	struct scatterlist *sg;
	unsigned long len = 0;
	unsigned int si;

	for_each_sg(qc->sg, sg, qc->n_elem, si)
		len += sg_dma_len(sg);

	return len;
}

static void hsata_bmdma_setup(struct ata_queued_cmd *qc)
{
	struct ata_port *ap = qc->ap;
	struct scatterlist *sg = qc->sg;
	struct hsata_device *hsdev = ap->host->private_data;
	struct dma_chan *chan = hsdev->dma_chan;
	struct dma_async_tx_descriptor *desc;
	u32 dev_dma_len;

	dev_dma_len = get_dma_len(qc);

	desc = dmaengine_prep_slave_sg(chan, sg, qc->n_elem, qc->dma_dir, 0);
	dmaengine_submit(desc);
	dma_async_issue_pending(chan);

	writel(dev_dma_len / 4, hsdev->ctl_base + 0x04);

	ata_sff_exec_command(qc->ap, &qc->tf);
}

static void hsata_bmdma_start(struct ata_queued_cmd *qc)
{
	struct hsata_device *hsdev = qc->ap->host->private_data;

	hsdev->dma_status = ATA_DMA_ACTIVE;

	/* set DBTSR */
	writel(HSATA_DMA_DBTSR, hsdev->membase + HSATA_DBTSR_REG);
	if (qc->dma_dir == DMA_TO_DEVICE) {
		/* Tx Burst length */
		writel(TANGOX_BURST_LENGTH_TX, hsdev->ctl_base + 0x00);
		/* Enable Tx*/
		writel(HSATA_DMACR_TX_EN, hsdev->membase + HSATA_DMACR_REG);
	} else {
		/* Rx Burst length */
		writel(BURST_LENGTH_RX, hsdev->ctl_base + 0x00);
		/* Enable Rx*/
		writel(HSATA_DMACR_RX_EN, hsdev->membase + HSATA_DMACR_REG);
	}
}

static void hsata_bmdma_stop(struct ata_queued_cmd *qc)
{
	struct ata_port *ap = qc->ap;
	struct hsata_device *hsdev = ap->host->private_data;

	writel(0, hsdev->membase + HSATA_DMACR_REG);
}

static u8 hsata_bmdma_status(struct ata_port *ap)
{
	struct hsata_device *hsdev = ap->host->private_data;

	return hsdev->dma_status;
}

static int hsata_check_atapi_dma(struct ata_queued_cmd *qc)
{
	u8 cmnd = qc->scsicmd->cmnd[0];

	if (cmnd == GPCMD_READ_TOC_PMA_ATIP || cmnd == GPCMD_MODE_SENSE_10)
		return 1;

	return 0;
}

static int hsata_scsi_ioctl(struct scsi_device *scsidev, int cmd,
				void __user *arg)
{
	int ret;
	u8 args[4];

	if (arg == NULL)
		return -EINVAL;

	if (copy_from_user(args, arg, sizeof(args)))
		return -EFAULT;

	ret = ata_scsi_ioctl(scsidev, cmd, arg);

	/* HIPM mode*/
	if (cmd == HDIO_DRIVE_CMD) {
		u32 serr;
		struct ata_port *ap;
		struct hsata_device *hsdev;

		ap = ata_shost_to_port(scsidev->host);
		hsdev = ap->host->private_data;

		if (args[0] == ATA_CMD_STANDBYNOW1 ||
		    args[0] == ATA_CMD_STANDBY) {
			/* put the host in slumber bit 13 or partial mode bit 12*/
			writel(1 << 13, hsdev->membase + HSATA_SCR2_REG);

			/* clear the SError Register */
			serr = readl(hsdev->membase + HSATA_SERROR_REG);
			writel(serr, hsdev->membase + HSATA_SERROR_REG);

			writel(0, hsdev->membase + HSATA_ERRMR_REG);
		} else if (args[0] == ATA_CMD_IDLEIMMEDIATE ||
			   args[0] == ATA_CMD_IDLE) {
			/* put the host in active mode, bit 14*/
			writel(1 << 14, hsdev->membase + HSATA_SCR2_REG);

			/* clear the SError Register */
			serr = readl(hsdev->membase + HSATA_SERROR_REG);
			writel(serr, hsdev->membase + HSATA_SERROR_REG);

			writel(HSATA_ERRMR_BITS, hsdev->membase + HSATA_ERRMR_REG);
		}
	}

	return ret;
}

static struct scsi_host_template hsata_sht = {
	ATA_BMDMA_SHT("sata_tangox"),
	.ioctl			= hsata_scsi_ioctl,
	.max_sectors		= 2 * ATA_MAX_SECTORS,
};

static struct ata_port_operations hsata_ops = {
	.inherits		= &ata_bmdma_port_ops,
	.check_atapi_dma	= hsata_check_atapi_dma,
	.bmdma_setup		= hsata_bmdma_setup,
	.bmdma_start		= hsata_bmdma_start,
	.bmdma_stop		= hsata_bmdma_stop,
	.bmdma_status		= hsata_bmdma_status,
	.qc_defer		= ata_std_qc_defer,
	.qc_prep		= ata_noop_qc_prep,
	.scr_read		= hsata_scr_read,
	.scr_write		= hsata_scr_write,
	.port_start		= hsata_port_start,
	.sff_irq_clear		= hsata_irq_clear,
};

static struct ata_port_info hsata_port_info =
{
	.flags		= ATA_FLAG_SATA | ATA_DFLAG_LBA48,
	.pio_mask	= 0x1f,	/* pio0-4	- IDENTIFY DEVICE word 63 */
	.mwdma_mask	= 0x07, /* mwdma0-2	- IDENTIFY DEVICE word 64 */
	.udma_mask	= 0x7f, /* udma0-6	- IDENTIFY DEVICE word 88 */
	.port_ops	= &hsata_ops,
};

static int hsata_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ata_host *host;
	struct hsata_device *hsdev;
	const struct ata_port_info *pi = &hsata_port_info;
	struct resource *memres, *ctlres;
	int sata_irq, dma_irq;
	int err;

	hsdev = devm_kzalloc(&pdev->dev, sizeof(*hsdev), GFP_KERNEL);
	if (!hsdev)
		return -ENOMEM;

	host = ata_host_alloc_pinfo(dev, &pi, 1);
	if (!host)
		return -ENOMEM;

	memres = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ctlres = platform_get_resource(pdev, IORESOURCE_MEM, 1);

	if (!memres || !ctlres)
		return -EINVAL;

	sata_irq = platform_get_irq(pdev, 0);
	dma_irq = platform_get_irq(pdev, 1);

	if (sata_irq < 0 || dma_irq < 0)
		return -EINVAL;

	hsdev->membase = devm_ioremap_resource(dev, memres);
	hsdev->ctl_base = devm_ioremap_resource(dev, ctlres);

	hsdev->dma_chan = dma_request_slave_channel(dev, "sata-dma");
	if (!hsdev->dma_chan) {
		dev_err(dev, "unable to allocate dma channel\n");
		return -ENXIO;
	}

	hsdev->phy = devm_phy_get(dev, "sata-phy");
	if (IS_ERR(hsdev->phy))
		hsdev->phy = NULL;

	phy_init(hsdev->phy);
	phy_power_on(hsdev->phy);

	host->private_data = hsdev;

	hsata_setup_port(&host->ports[0]->ioaddr, hsdev->membase);

	err = devm_request_irq(dev, dma_irq, hsata_dma_isr, 0, dev_name(dev),
			       host);
	if (err)
		return -ENXIO;

	err = ata_host_activate(host, sata_irq, hsata_isr, 0, &hsata_sht);
	if (err)
		return err;

	hsata_enable_interrupts(hsdev);

	return 0;
}

static int hsata_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ata_host *host = dev_get_drvdata(dev);
	struct hsata_device *hsdev = host->private_data;

	hsata_disable_interrupts(hsdev);
	dma_release_channel(hsdev->dma_chan);
	ata_host_detach(host);
	phy_power_off(hsdev->phy);
	phy_exit(hsdev->phy);

	return 0;
}

static struct of_device_id hsata_dt_ids[] = {
	{ .compatible = "sigma,smp8640-sata" },
	{ }
};

static struct platform_driver hsata_driver = {
	.probe	= hsata_probe,
	.remove	= hsata_remove,
	.driver = {
		.name		= "tangox-sata",
		.of_match_table	= hsata_dt_ids,
	},
};
module_platform_driver(hsata_driver);
