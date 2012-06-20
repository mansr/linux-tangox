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
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/blkdev.h>
#include <linux/libata.h>
#include <linux/hdreg.h>
#include <scsi/scsi_host.h>
#include <scsi/scsi.h>
#include <scsi/scsi_device.h>
#include <scsi/scsi_cmnd.h>
#include <scsi/scsi_tcq.h>
#include <asm/scatterlist.h>
#if defined(CONFIG_TANGO3)
#include <asm/tango3/hardware.h>
#include <asm/tango3/tango3api.h>
#include <asm/tango3/platform_dev.h>
#elif defined(CONFIG_TANGO4)
#include <asm/tango4/hardware.h>
#include <asm/tango4/tango4api.h>
#include <asm/tango4/platform_dev.h>
#else
#error "not supported platform."
#endif
#include "libata.h"

#define HSATA_TANGOX_DMA
#undef HSATA_VERBOSE

MODULE_AUTHOR("Sigma Designs Inc.");
MODULE_DESCRIPTION ("TANGOX Bulid-in SATA Host Controller device driver");
MODULE_LICENSE ("GPL");

/*HSATA Registers*/
#define HSATA_SCR0_REG			0x0024
#define HSATA_SCR1_REG			0x0028
#define HSATA_SCR2_REG			0x002C
#define HSATA_SCR3_REG			0x0030
#define HSATA_SCR4_REG			0x0034
#define HSATA_SERROR_REG		HSATA_SCR1_REG
#define HSATA_SCONTROL_REG		HSATA_SCR2_REG
#define HSATA_SACTIVE_REG		HSATA_SCR3_REG

#define HSATA_DMACR_TX_EN		(0x01 /*| HSATA_DMACR_TXMODE_BIT*/)
#define HSATA_DMACR_RX_EN		(0x02 /*| HSATA_DMACR_TXMODE_BIT*/)
#define HSATA_DMACR_TXRX_EN		(0x03 | HSATA_DMACR_TXMODE_BIT)
#define HSATA_DMACR_TXMODE_BIT		0x04
#define HSATA_FEAT_REG			0x0004
#define HSATA_CMD_REG			0x001c
#define HSATA_CONTROL_REG		0x0020
#define HSATA_DMACR_REG			0x0070
#define HSATA_DBTSR_REG			0x0074
#define HSATA_INTPR_REG			0x0078
#define HSATA_INTPR_ERR_BIT		0x00000008
#define HSATA_INTPR_FP_BIT		0x00000002	/* new DMA setup FIS arrived */
#define HSATA_INTMR_REG			0x007C
#define HSATA_INTMR_ERRM_BIT		0x00000008
#define HSATA_INTMR_NEWFP_BIT		0x00000002
#define HSATA_ERRMR_REG			0x0080
#define HSATA_ERRMR_BITS		0xFFFEF7FF
#define HSATA_VER_REG			0x00F8
#define HSATA_IDR_REG			0x00FC

static const int tangox_ctl_base[2] = {TANGOX_SATA0_CTL_BASE, TANGOX_SATA1_CTL_BASE};
static const int tangox_sata_irq[2] = {TANGOX_SATA_IRQ0, TANGOX_SATA_IRQ1};
static const int tangox_sata_dma_irq[2] = {TANGOX_SATA_DMA_IRQ0, TANGOX_SATA_DMA_IRQ1};
static const int tangox_sbox[2] = {SBOX_SATA0, SBOX_SATA1};
static const int tangox_aes_config[2] = {HC_SATA0_AES_CONFIG, HC_SATA1_AES_CONFIG};

/*
 * Per device data struct
 */

#define MAX_TRANS_UNITS		64

struct trans_unit
{
	u32 dma_addr;
	u32 dma_len;
};

struct hsata_device
{
	struct platform_device *pdev;
	struct ata_host *host;
	unsigned long membase;
	unsigned long ctl_base;
	int sata_irq;
	int controller;
	/* dma */
	int sbox;
	int dma_irq;
	int mbus_pending;	/* mbus dma pending */
	int dev_pending;	/* device dma pending */
	unsigned long mbus_reg;
	struct trans_unit t_unit[MAX_TRANS_UNITS];
	u32 next_unit;
	u32 trans_unit_cnt;
	spinlock_t lock;
};

#define HSDEV_FROM_HOST_SET(hs) (struct hsata_device*)hs->private_data
#define HSDEV_FROM_AP(ap) (struct hsata_device*)ap->host->private_data
#define HSDEV_FROM_QC(qc) (struct hsata_device*)qc->ap->host->private_data

#ifdef HSATA_TANGOX_DMA
#define TANGOX_BURST_LENGTH_TX	16 
#define BURST_LENGTH_TX		24 
#define BURST_LENGTH_RX		4 /* was 64 */
#define HSATA_DMA_DBTSR		((BURST_LENGTH_RX << 16) | (BURST_LENGTH_TX << 0)) 
#endif 

/* Throttle to gen1 speed */
static int gen1only = 0;
static int disable_ports = 0;
module_param(disable_ports, int, 0444);
MODULE_PARM_DESC(disable_ports, "which port to disable (1: disable port 0, 2: disable port 1, 3: both ports)");

/* Interrupt stuff*/
static void hsata_enable_interrupts(struct hsata_device *hsdev) 
{ 
	volatile u32 val32; 
	unsigned long flags;

	spin_lock_irqsave(&hsdev->lock, flags);
	/* enable all err interrupts */ 
	writel(HSATA_ERRMR_BITS, (void *)(hsdev->membase + HSATA_ERRMR_REG)); 
	val32 = readl((void *)(hsdev->membase + HSATA_INTMR_REG)); 
	writel(val32 | HSATA_INTMR_ERRM_BIT | HSATA_INTMR_NEWFP_BIT,
			(void *)(hsdev->membase + HSATA_INTMR_REG)); 
	val32 = readl((void *)(hsdev->membase + HSATA_INTMR_REG)); 
	DPRINTK("%s INTMR=0x%x\n", __FUNCTION__, val32);
	spin_unlock_irqrestore(&hsdev->lock, flags);
}

static void hsata_disable_interrupts(struct hsata_device *hsdev) 
{ 
	volatile u32 val32;
	unsigned long flags;

	spin_lock_irqsave(&hsdev->lock, flags);
	/* disable all err interrupts */ 
	writel(0, (void *)(hsdev->membase + HSATA_INTMR_REG)); 
	val32 = readl((void *)(hsdev->membase + HSATA_INTMR_REG)); 
	DPRINTK("%s INTMR=0x%x\n", __FUNCTION__, val32);
	spin_unlock_irqrestore(&hsdev->lock, flags);
}

static int hsata_scr_read(struct ata_link *link, unsigned int sc_reg, u32 *val)
{
	void __iomem *mmio = (void __iomem *)(link->ap->ioaddr.scr_addr + (sc_reg * 4));

	if (mmio) {
		*val = readl(mmio);
		return 0;
	}
	return -EINVAL;
}

static int hsata_scr_write(struct ata_link *link, unsigned int sc_reg, u32 val)
{
	void __iomem *mmio = (void __iomem *)(link->ap->ioaddr.scr_addr + (sc_reg * 4));

	if (mmio) {
		writel(val, mmio);
		return 0;
	}
	return -EINVAL;
}

static void tangox_sata_init(void)
{
	unsigned int val;
	unsigned int cfg = 0;
	int tangox_get_sata_channel_cfg(unsigned int *);
	static int sata_init = 0;

	if (sata_init != 0)
		return;
	sata_init = 1;

	tangox_get_sata_channel_cfg(&cfg);

	/* bit14: force gen1? */
	gen1only = (cfg & (1 << 14)) ? 1 : 0; /* force gen1 speed? */

	val = readl((void *)(TANGOX_SATA0_CTL_BASE + 0x0c));

	/* bit15: internal clock? */
	if (cfg & (1 << 15))
		val |= (1 << 24);	/* internal clock routing */
	else
		val &= ~(1 << 24);	/* external clock is used */
	val = (val & 0xff0fffff) | ((cfg & 0x0f00) << 12);	/* TX edge rate control */

	writel(val, (void *)(TANGOX_SATA0_CTL_BASE + 0x0c));
	DPRINTK("PHY stat1(0x%x)=0x%x\n", TANGOX_SATA0_CTL_BASE + 0x0c, val);

	/* 
	 bit0: RX SSC port0
	 bit1: RX SSC port1
	 bit2: TX SSC port0/1
	 */
	writel(0x28903 | ((cfg & 1) ? 0x200 : 0) | ((cfg & 2) ? 0x1000 : 0), 
		(void *)(TANGOX_SATA0_CTL_BASE + 0x10));

	val = readl((void *)(TANGOX_SATA0_CTL_BASE + 0x14));
	val &= ~0x7ff;
	val |= ((cfg & 4) ? 0x400 : 0); /* TX SSC enable or not */

	/* bit7..4: reference clock frequency */
	switch ((cfg >> 4) & 0xf) {
		case 0: /* 120MHz ref clock */
			val |= 0x12c;
			break;
		case 2: /* 60MHz ref clock */
			val |= 0x128;
			break;
		case 4: /* 30MHz ref clock */
			val |= 0x12a;
			break;
		case 1: /* 100MHz ref clock */
			val |= 0x234;
			break;
		case 3: /* 50MHz ref clock */
			val |= 0x230;
			break;
		case 5: /* 25MHz ref clock */
			val |= 0x232;
			break;
		default:
			DPRINTK("Invalid frequency selection specified: %d\n", (cfg >> 4) & 0xf);
			val |= 0x12c;
			break;
	}
	writel(val, (void *)(TANGOX_SATA0_CTL_BASE + 0x14));

	val = readl((void *)(TANGOX_SATA0_CTL_BASE + 0x10));
	DPRINTK("PHY stat2(0x%x)=0x%x\n", TANGOX_SATA0_CTL_BASE + 0x10, val);
	val = readl((void *)(TANGOX_SATA0_CTL_BASE + 0x14));
	DPRINTK("PHY stat3(0x%x)=0x%x\n", TANGOX_SATA0_CTL_BASE + 0x14, val);
	val = readl((void *)(TANGOX_SATA0_CTL_BASE + 0x18));
	DPRINTK("PHY stat4(0x%x)=0x%x\n", TANGOX_SATA0_CTL_BASE + 0x18, val);
	val |= 1<<16; /* fast tech */
	val |= 1<<18; /* 3.3 v */
	DPRINTK("Setting PHY stat4(0x%x) to 0x%x\n", (TANGOX_SATA0_CTL_BASE + 0x18), val);
	writel(val, (void *)(TANGOX_SATA0_CTL_BASE + 0x18));
	val = readl((void *)(TANGOX_SATA0_CTL_BASE + 0x18));
	DPRINTK("PHY stat4(0x%x)=0x%x\n", TANGOX_SATA0_CTL_BASE + 0x18, val);
}

static int hsata_qc_complete(struct ata_port *ap, struct ata_queued_cmd *qc,
					u32 check_status)
{
	u8 status = 0;
	int i;

	if (check_status) {
		/* check altstatus */
		i = 0;
		do {
			/* check main status, clearing INTRQ */
			status = ata_sff_check_status(ap);
			DPRINTK("STATUS (0x%x) [%d]\n", status, i);
			if (status & ATA_BUSY)	{
				DPRINTK("STATUS BUSY (0x%x) [%d]\n", status, i);
			}
			if (++i > 10)
				break;
		} while (status & ATA_BUSY);

		if (status & (ATA_BUSY | ATA_ERR | ATA_DF)) {
			DPRINTK("STATUS BUSY/ERROR (0x%x) [%d]\n", status, i);
		}
	}
	DPRINTK("QC COMPLETE status=0x%x ata%u: protocol %d\n", 
			status, ap->print_id, qc->tf.protocol);

	/* complete taskfile transaction */
	qc->err_mask |= ac_err_mask(status);
	ata_qc_complete(qc);

	return 0;
}

#ifdef HSATA_TANGOX_DMA
static void hsata_mbus_done(int irq, void *arg)
{
	unsigned long flags;
	struct hsata_device *hsdev = (struct hsata_device *)arg;

	spin_lock_irqsave(&hsdev->lock, flags);
	hsdev->mbus_pending = 0; /* mbus is done */
	if (hsdev->dev_pending == 0) { /* free up channel if device is done as well */
		if (hsdev->mbus_reg != 0) {
			em86xx_mbus_free_dma(hsdev->mbus_reg, hsdev->sbox);
			hsdev->mbus_reg = 0;
		}
	}
	spin_unlock_irqrestore(&hsdev->lock, flags);
}

static void hsata_mbus_intr(int irq, void *arg)
{
	struct ata_queued_cmd *qc = (struct ata_queued_cmd *)arg;
	struct trans_unit *uptr = NULL;
	struct hsata_device *hsdev = NULL;
	unsigned long flags = 0;

	if (unlikely((qc->sg == NULL) || (qc->ap == NULL) || (qc->ap->host == NULL)))
		goto err_out;
	else if (unlikely((hsdev = (struct hsata_device *)HSDEV_FROM_QC(qc)) == NULL))
		goto err_out;

	spin_lock_irqsave(&hsdev->lock, flags);
	if (unlikely(hsdev->mbus_reg == 0)) /* likely due to ATA error */
		goto done;
	BUG_ON(hsdev->next_unit == hsdev->trans_unit_cnt);

	/*
	 * setup a new mbus transfer
	 */
	uptr = &hsdev->t_unit[hsdev->next_unit];
	hsdev->next_unit++;

#ifdef HSATA_VERBOSE
	printk("(%d) %s setup_dma mbus_reg =0x%lx address=0x%x len=0x%x n_unit=0x%x next_unit=0x%x\n", smp_processor_id(),
			__FUNCTION__, hsdev->mbus_reg, uptr->dma_addr, uptr->dma_len,
			hsdev->trans_unit_cnt, hsdev->next_unit);
#endif

	if (hsdev->next_unit == hsdev->trans_unit_cnt) { /* no more unit */
		if (em86xx_mbus_setup_dma(hsdev->mbus_reg, uptr->dma_addr,
						uptr->dma_len, hsata_mbus_done, hsdev, 1)) {
			printk("(%d) fail to resetup dma, wait for timeout...\n", smp_processor_id());
		}
	} else {
		if (em86xx_mbus_setup_dma(hsdev->mbus_reg, uptr->dma_addr,
						uptr->dma_len, hsata_mbus_intr, qc, 0)) {
			printk("(%d) fail to resetup dma, wait for timeout...\n", smp_processor_id());
		}
	}
done:
	spin_unlock_irqrestore(&hsdev->lock, flags);

err_out:
	return;
}

static irqreturn_t hsata_dma_isr(int irq, void *dev_id)
{
	struct ata_host *host = (struct ata_host *)dev_id;
	struct hsata_device *hsdev = HSDEV_FROM_HOST_SET(host);
	unsigned long flags;

 	spin_lock_irqsave(&host->lock, flags);
	if (readl((void *)(hsdev->ctl_base+0x08))) {
#ifdef HSATA_VERBOSE
		printk("(%d) %s got dma interrupt irq=0x%x, 08=0x%x\n", smp_processor_id(), 
			__FUNCTION__, irq, readl((void *)(hsdev->ctl_base+0x08)));
#endif
		/*clear dma interrupt*/
		writel(0, (void *)(hsdev->ctl_base+0x08));
		writel(0, (void *)(hsdev->membase + HSATA_DMACR_REG));
	}

	spin_unlock_irqrestore(&host->lock, flags);
	return IRQ_HANDLED;
}

static unsigned long get_dma_len(struct ata_queued_cmd *qc)
{
	unsigned long len = 0;
	struct scatterlist *sg;
	unsigned int si;

	for_each_sg(qc->sg, sg, qc->n_elem, si) {
		len += sg_dma_len(sg);
	}

	return len;
}

static void hsata_dma_xfer_complete(struct ata_host *host, u32 check_status)
{
	struct ata_port *ap;
	struct ata_queued_cmd *qc;
	u8 tag = 0;
	unsigned long flags;
	struct hsata_device *hsdev;

	ap = host->ports[0];
	tag = ap->link.active_tag;
	qc = ata_qc_from_tag(ap, tag);
	hsdev = HSDEV_FROM_QC(qc);

	DPRINTK("active_tag=%d protocol=%d qc=0x%x \n", tag, qc->tf.protocol, qc);

	switch (qc->tf.protocol) {
		case ATA_PROT_DMA:
		case ATAPI_PROT_DMA:
			spin_lock_irqsave(&hsdev->lock, flags);
			hsdev->dev_pending = 0; /* device is done */
			if (hsdev->mbus_pending == 0) { /* free up channel if mbus is done as well */
				if (hsdev->mbus_reg != 0) {
					em86xx_mbus_free_dma(hsdev->mbus_reg, hsdev->sbox);
					hsdev->mbus_reg = 0;
				}
			}
			spin_unlock_irqrestore(&hsdev->lock, flags);
			dma_unmap_sg((struct device *)qc->dev, qc->sg, qc->n_elem, qc->dma_dir);

			/* !!! FALL THRU TO NEXT CASE !!! */
		case ATAPI_PROT_PIO:	
			if (unlikely(hsata_qc_complete(ap, qc, check_status)))
					;
	 		break;
	
		case ATAPI_PROT_NODATA: 
		case ATA_PROT_NODATA:
			DPRINTK(KERN_ERR "WE SHOULDN'T GET HERE");
			break;
	}
}
#endif

static irqreturn_t hsata_isr(int irq, void *dev_instance)
{
	struct ata_host *host = (struct ata_host*)dev_instance;
	struct hsata_device *hsdev = HSDEV_FROM_HOST_SET(host);
	unsigned int handled = 0;
	struct ata_port *ap;
	struct ata_queued_cmd *qc;
	unsigned long flags, flgs;
	u8 status = 0;
	volatile u32 val32;
	volatile u32 intpr;
	u32 err_interrupt;
	u32 tag_mask;
	volatile u32 sactive, sactive2;
	u8 tag;
	int abort = 0, freeze = 0;

	spin_lock_irqsave(&host->lock, flags);
	ap = host->ports[0];

	/* clear all*/
	intpr = readl((void *)(hsdev->membase + HSATA_INTPR_REG));
	writel(intpr, ((void *)hsdev->membase + HSATA_INTPR_REG));

	if (intpr & HSATA_INTPR_ERR_BIT) {
		val32 = readl((void *)(hsdev->membase + HSATA_SERROR_REG)); 
		DPRINTK("SERROR=0x%08x INTPR=0x%x\n", val32, intpr);
		/* hotplug */
		if (val32 & (SERR_PHYRDY_CHG | SERR_DEV_XCHG)) {
			struct ata_link *link;
			struct ata_eh_info *ehi;

			link = &ap->link;
			ehi = &link->eh_info;
			ata_ehi_clear_desc(ehi);

			ata_ehi_hotplugged(ehi);
			ata_ehi_push_desc(ehi, "%s",
				val32 & SERR_PHYRDY_CHG ?
				"PHY RDY changed" : "device exchanged");
			freeze = 1;	
		}

		writel(val32, (void *)(hsdev->membase + HSATA_SERROR_REG));	/* to clear */
		err_interrupt = 1;
		handled = 1;
		goto DONE;
	}
	else
		err_interrupt = 0;

	/* 
	 * ACTIVE TAG
	 * At this point we need to figure out for which tags we have gotten a
	 * completion interrupt. One interrupt may serve as completion for 
	 * more than one operation when commands are queued (NCQ).
	 * We need to process each completed command.
	 */
PROCESS:	/* process completed commands */
	sactive = readl((void *)(hsdev->membase + HSATA_SACTIVE_REG));	/* remaining pending */
	if (sactive)
		DPRINTK("UNEXPECTED SACTIVE??? sactive=0x%x\n", sactive);
	tag = ap->link.active_tag;
	if (!(ata_tag_valid(tag))) {
		printk("(%d) invalid tag 0x%x\n", smp_processor_id(), tag);
		handled = 1;
		goto DONE;
	}
	tag_mask = 0x01 << tag;
	/* 
	 * Check main status, clearing INTRQ 
	 */
	status = ata_sff_check_status(ap); 
	DPRINTK("status=0x%x err_interrupt=0x%x\n", status, err_interrupt);

	if (!err_interrupt && (status & ATA_BUSY)) {
		DPRINTK("NOT OUR INTERRUPT - STATUS BUSY (0x%x) INTPR=0x%x\n", status, intpr);
		goto NOTOURINT;
	}

	tag = 0;
	while (tag_mask) {
		while (!(tag_mask & 0x01)) {
			tag++;
			tag_mask >>= 1;
		}
		tag_mask &= (~0x01);

		qc = ata_qc_from_tag(ap, tag);
		if (unlikely(!qc)) {
			printk("(%d) qc is null\n", smp_processor_id());
			handled = 1;
			goto DONE;
		}
		 /* to be picked up by downstream completion functions */
		qc->ap->link.active_tag = tag; 

		if (status & ATA_ERR) {
			DPRINTK("INTERRUPT ATA_ERR (0x%x)\n", status);
#ifdef HSATA_TANGOX_DMA
			spin_lock_irqsave(&hsdev->lock, flgs);
			if ((hsdev->dev_pending != 0) || (hsdev->mbus_pending != 0)) {
				hsdev->dev_pending = hsdev->mbus_pending = 0;
				if (hsdev->mbus_reg != 0) {
					em86xx_mbus_wait(hsdev->mbus_reg, hsdev->sbox);
					em86xx_mbus_free_dma(hsdev->mbus_reg, hsdev->sbox);
					hsdev->mbus_reg = 0;
				}
			}
			spin_unlock_irqrestore(&hsdev->lock, flgs);
#endif
			hsata_qc_complete(ap, qc, 1);
			handled = 1;
			goto DONE;
		}
#ifdef HSATA_TANGOX_DMA
		/* Process completed command*/
		if ((ap->hsm_task_state ==HSM_ST_LAST) && 
				(qc->tf.protocol == ATA_PROT_DMA || 
				 qc->tf.protocol == ATAPI_PROT_DMA))
			hsata_dma_xfer_complete(host, 1);
#endif
		handled = ata_bmdma_port_intr(ap, qc);
		if (handled == 1)
			goto DONE;
	} /* while tag_mask */

	/*
	 * Check to see if any commands completed while we were processing our initial
	 * set of completed commands (reading of status clears interrupts, so we might
	 * miss a completed command interrupt if one came in while we were processing --
	 * we read status as part of processing a completed command).
	 */
	sactive2 = readl((void *)(hsdev->membase + HSATA_SACTIVE_REG));
	if (sactive2 != sactive) {
		DPRINTK("MORE COMPLETED - sactive=0x%x sactive2=0x%x\n",
			sactive, sactive2);
		goto PROCESS;
	}

	handled = 1;
	goto DONE;

NOTOURINT:
	status = ata_sff_check_status(ap);
	DPRINTK("NOT OUR INTERRUPT status=0x%x", status);
	handled = 0;
DONE:
	/* freeze or abort */
	if (freeze)
		ata_port_freeze(ap);
	else if (abort) {
		if (qc)
			ata_link_abort(qc->dev->link);
		else
			ata_port_abort(ap);
	}

	spin_unlock_irqrestore(&host->lock, flags);
	return IRQ_RETVAL(handled);
}

static void hsata_irq_clear(struct ata_port *ap)
{
	struct hsata_device *hsdev ;
	hsdev = HSDEV_FROM_AP(ap);

	/* read status reg to clear interrupt in controller */
	ata_sff_check_status(ap);

	/* reenable interrupt, for hotplug*/
	hsata_enable_interrupts(hsdev);
}

static void hsata_host_init(struct hsata_device *hsdev)
{
	unsigned int pid, ver;
	u32 val32;

	/* Read IDR and Version registers*/
	pid = readl((void *)(hsdev->membase + HSATA_IDR_REG));
	ver = readl((void *)(hsdev->membase + HSATA_VER_REG));

	printk("SATA version 0x%x ID 0x%x is detected\n", ver, pid);

	/* some other initializations here*/
	tangox_sata_init();

	/* Enable IPM */
	val32 = readl((void *) (hsdev->membase + HSATA_SCR2_REG));
	writel((val32 & ~(0x3 << 8)), (void *)(hsdev->membase + HSATA_SCR2_REG));

	/*
	 * We clear this bit here so that we can later on check to see if other bus
	 * errors occured (during debug of this driver).
	 */
	val32 = readl((void *) (hsdev->membase + HSATA_SERROR_REG));
	writel(val32, (void *)(hsdev->membase + HSATA_SERROR_REG));
}

static void hsata_setup_port(struct ata_ioports *port, unsigned long base)
{
	port->cmd_addr = (void __iomem *)(base + 0x00); 
	port->data_addr	= (void __iomem *)(base + 0x00);

	port->error_addr = (void __iomem *)(base + 0x04);
	port->feature_addr = (void __iomem *)(base + 0x04);

	port->nsect_addr = (void __iomem *)(base + 0x08);

	port->lbal_addr	= (void __iomem *)(base + 0x0c);
	port->lbam_addr	= (void __iomem *)(base + 0x10);
	port->lbah_addr	= (void __iomem *)(base + 0x14);

	port->device_addr = (void __iomem *)(base + 0x18);

	port->command_addr = (void __iomem *)(base + 0x1c);
	port->status_addr = (void __iomem *)(base + 0x1c);

	port->altstatus_addr = (void __iomem *)(base + 0x20);
	port->ctl_addr = (void __iomem *)(base + 0x20);
	port->scr_addr = (void __iomem *)(base + HSATA_SCR0_REG);
	port->bmdma_addr = NULL; 
}

static void hsata_reset_port(struct ata_port *ap)
{
	u32 sstatus = 0 ;
	unsigned long timeout = jiffies + (HZ * 5);
		
	struct hsata_device *hsdev ;
	hsdev = HSDEV_FROM_AP(ap);

	/* hard interface reset */
	hsata_scr_write(&ap->link, 2, 0x001 | (gen1only << 4));
	mdelay(1);
	hsata_scr_write(&ap->link, 2, 0x000 | (gen1only << 4));

	/* wait for phy to become ready, if necessary */
	do {
		udelay(10);
		hsata_scr_read(&ap->link, 0, &sstatus);
		if ((sstatus & 0xf) != 1)
			break;
	} while (time_before(jiffies, timeout));

	/* soft interface reset to reset device */
	writel(1 << 2, (void *)(hsdev->membase + HSATA_CONTROL_REG)); 
	mdelay(10);
}

static int hsata_port_start(struct ata_port *ap)
{
	int status = 0, port;
	struct hsata_device *hsdev ;
	hsdev = HSDEV_FROM_AP(ap);
	port = ap->port_no;
	DPRINTK("id=%d port_num=%d port=0x%x\n", ap->print_id, ap->port_no, port);

	/* Grab ptr to this top-level device data. */
	hsdev->host = ap->host;

	hsata_reset_port(ap);

#ifdef HSATA_TANGOX_DMA
	status = request_irq(hsdev->dma_irq, hsata_dma_isr, IRQF_DISABLED,
				((hsdev->controller == 0) ? "HSATA0-DMA": "HSATA1-DMA"),
				ap->host); 
	if (status) {
		DPRINTK("DMA request_irq FAILED (status = %d) irq=0x%x\n", 
					status, hsdev->dma_irq);
		status = -ENOMEM;
		goto CLEANUP;
	}

	/* Don't enable yet -- we do that right before a xfer */
	writel(0/*HSATA_DMACR_TXMODE_BIT*/, (void *)(hsdev->membase + HSATA_DMACR_REG)); 

CLEANUP:
	if (status) {
		free_irq(hsdev->dma_irq, ap->host);
	}
#endif
	return status;
}

static void hsata_port_stop(struct ata_port *ap)
{
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
		hsdev = HSDEV_FROM_AP(ap);
		
		if (args[0] == ATA_CMD_STANDBYNOW1 || 
				args[0] == ATA_CMD_STANDBY) {

			/* put the host in slumber bit 13 or partial mode bit 12*/
			writel(1 << 13, (void *)(hsdev->membase + HSATA_SCR2_REG));

			/* clear the SError Register */
			serr = readl((void *)(hsdev->membase + HSATA_SERROR_REG));
			writel(serr, (void *)(hsdev->membase + HSATA_SERROR_REG));

			writel(0, (void *)(hsdev->membase + HSATA_ERRMR_REG)); 
		} else if (args[0] == ATA_CMD_IDLEIMMEDIATE || 
					args[0] == ATA_CMD_IDLE) {
			/* put the host in active mode, bit 14*/
			writel(1 << 14, (void *)(hsdev->membase + HSATA_SCR2_REG));

			/* clear the SError Register */
			serr = readl((void *)(hsdev->membase + HSATA_SERROR_REG));
			writel(serr, (void *)(hsdev->membase + HSATA_SERROR_REG)); 

			writel(HSATA_ERRMR_BITS, (void *)(hsdev->membase + HSATA_ERRMR_REG)); 
		}
	}
	return ret;
}

static struct ata_device *hsata_ata_find_dev(struct ata_port *ap, int id)
{
	if (likely(id < ATA_MAX_DEVICES))
		return &ap->link.device[id];
	return NULL;
}

static struct ata_device *hsata_ata_scsi_find_dev(struct ata_port *ap,
					const struct scsi_device *scsidev)
{
	/* skip commands not addressed to targets we simulate */
	if (unlikely(scsidev->channel || scsidev->lun))
		return NULL;

	return hsata_ata_find_dev(ap, scsidev->id);
}

static int hsata_scsi_queuecmd(struct Scsi_Host *shost, struct scsi_cmnd *cmd)
{
	struct ata_port *ap;
	struct ata_device *dev;
	struct scsi_device *scsidev = cmd->device;
	int dir, ret;
	struct hsata_device *hsdev;
#ifdef HSATA_TANGOX_DMA
	unsigned long flgs;
#endif

	ap = ata_shost_to_port(shost);
	dev = hsata_ata_scsi_find_dev(ap, scsidev);

#ifdef HSATA_TANGOX_DMA
	dir = cmd->sc_data_direction;
	hsdev = HSDEV_FROM_AP(ap);

	spin_lock_irqsave(&hsdev->lock, flgs);
	if ((hsdev->dev_pending != 0) || (hsdev->mbus_pending != 0)) { /* busy */
		spin_unlock_irqrestore(&hsdev->lock, flgs);
		return SCSI_MLQUEUE_HOST_BUSY;
	}

	if (hsdev->mbus_reg != 0) {
		em86xx_mbus_free_dma(hsdev->mbus_reg, hsdev->sbox);
		hsdev->mbus_reg = 0;
	}

	if ((scsi_bufflen(cmd) <= 1024) && (dev->class == ATA_DEV_ATAPI)) {
		dev->flags |= ATA_DFLAG_PIO;
		DPRINTK("set to PIO mode len=0x%x\n", scsi_bufflen(cmd));
	} else { 
		if (em86xx_mbus_alloc_dma(hsdev->sbox, (dir == DMA_FROM_DEVICE) ? 1 : 0, &hsdev->mbus_reg, NULL, 0)) {
			dev->flags |= ATA_DFLAG_PIO;
			DPRINTK("set to PIO mode len=0x%x\n", scsi_bufflen(cmd));
		} else {
			dev->flags &= ~ATA_DFLAG_PIO;
			DPRINTK("set to DMA mode len=0x%x\n", scsi_bufflen(cmd));
		}
	}
	spin_unlock_irqrestore(&hsdev->lock, flgs);
#else
	dev->flags |= ATA_DFLAG_PIO;
#endif

	DPRINTK("sn=%d\n", cmd->serial_number);
	ret = ata_scsi_queuecmd(shost, cmd);

	return ret;
}

/* the same as ata_scsi_slave_config, remove later one */
static int tangox_ata_scsi_slave_config(struct scsi_device *sdev)
{
	sdev->use_10_for_rw = 1;
	sdev->use_10_for_ms = 1;

	blk_queue_max_segments(sdev->request_queue, LIBATA_MAX_PRD);
	sdev->manage_start_stop = 1;

	if (sdev->id < ATA_MAX_DEVICES) {
		struct ata_port *ap;
		struct ata_device *dev;

		ap = ata_shost_to_port(sdev->host);
		dev = hsata_ata_scsi_find_dev(ap, sdev);

		/* set max_hw_sectors_kb=256KB */ 
		blk_queue_max_hw_sectors(sdev->request_queue, ATA_MAX_SECTORS);
		if ((dev->flags & ATA_DFLAG_LBA48) &&
			((dev->flags & ATA_DFLAG_CDB_INTR) == 0)) {
			/*
			 * do not overwrite sdev->host->max_sectors, since
			 * other drives on this host may not support LBA48
			 */
			printk(KERN_INFO "ata%u: dev %u max request %d sectors (lba48)\n",
				 ap->print_id, sdev->id, sdev->host->max_sectors);
		} else {
			printk(KERN_INFO "ata%u: dev %u max request %d sectors (non lba48)\n",
				 ap->print_id, sdev->id, sdev->host->max_sectors);
		}

		/*
		 * SATA DMA transfers must be multiples of 4 byte, so
		 * we need to pad ATAPI transfers using an extra sg.
		 * Decrement max hw segments accordingly.
		 */
		if (dev->class == ATA_DEV_ATAPI) {
			struct request_queue *q = sdev->request_queue;
			blk_queue_max_segments(q, q->limits.max_segments - 1);
		}

		if (dev->flags & ATA_DFLAG_NCQ) {
			int depth;
			depth = min(sdev->host->can_queue, ata_id_queue_depth(dev->id));
			depth = min(ATA_MAX_QUEUE - 1, depth);
			scsi_adjust_queue_depth(sdev, MSG_SIMPLE_TAG, depth);
		}	
	}
	return 0;	 /* scsi layer doesn't check return value, sigh */
}

static int hsata_scsi_slave_cfg(struct scsi_device *sdev)
{
	DPRINTK("id=%d lun=%d ch=%d mfgr=%d\n", 
		sdev->id, sdev->lun, sdev->channel, sdev->manufacturer);
#if 1 
	return tangox_ata_scsi_slave_config(sdev);
#else
	return ata_scsi_slave_config(sdev);
#endif
}

static int hsata_std_bios_param(struct scsi_device *sdev, 
					 struct block_device *bdev,
					 sector_t capacity, int geom[])
{
	DPRINTK("id=%d lun=%d ch=%d mfgr=%d\n", 
		 sdev->id, sdev->lun, sdev->channel, sdev->manufacturer);
	return ata_std_bios_param(sdev, bdev, capacity, geom);
}

static int hsata_set_mode(struct ata_link *link, struct ata_device **r_failed)
{
	return ata_do_set_mode(link, r_failed);
}

static unsigned long hsata_mode_filter(struct ata_device *adev,
				    unsigned long xfer_mask)
{
	/* filter nothing */
	return xfer_mask;
}

static void hsata_exec_command_by_tag(struct ata_port *ap, const struct ata_taskfile *tf, u8 tag)
{
#ifdef HSATA_VERBOSE
	{
		volatile u32 val32;
		struct hsata_device *hsdev = HSDEV_FROM_AP(ap);
		switch (tf->command) {
			case ATA_CMD_CHK_POWER		: DPRINTK("ATA_CMD_CHK_POWER - tag=%d\n", tag); break;
			case ATA_CMD_EDD	 	: DPRINTK("ATA_CMD_EDD - tag=%d\n", tag); break;
			case ATA_CMD_FLUSH	 	: DPRINTK("ATA_CMD_FLUSH - tag=%d\n", tag); break;
			case ATA_CMD_FLUSH_EXT		: DPRINTK("ATA_CMD_FLUSH_EXT - tag=%d\n", tag); break;
			case ATA_CMD_ID_ATA		: DPRINTK("ATA_CMD_ID_ATA - tag=%d\n", tag); break;
			case ATA_CMD_ID_ATAPI		: DPRINTK("ATA_CMD_ID_ATAPI - tag=%d\n", tag); break;
			case ATA_CMD_READ		: DPRINTK("ATA_CMD_READ - tag=%d\n", tag); break;
			case ATA_CMD_READ_EXT		: DPRINTK("ATA_CMD_READ_EXT - tag=%d\n", tag); break;
			case ATA_CMD_WRITE	 	: DPRINTK("ATA_CMD_WRITE - tag=%d\n", tag); break;
			case ATA_CMD_WRITE_EXT		: DPRINTK("ATA_CMD_WRITE_EXT - tag=%d\n", tag); break;
			case ATA_CMD_PIO_READ		: DPRINTK("ATA_CMD_PIO_READ - tag=%d\n", tag); break;
			case ATA_CMD_PIO_READ_EXT	: DPRINTK("ATA_CMD_PIO_READ_EXT - tag=%d\n", tag); break;
			case ATA_CMD_PIO_WRITE 		: DPRINTK("ATA_CMD_PIO_WRITE - tag=%d\n", tag); break;
			case ATA_CMD_PIO_WRITE_EXT 	: DPRINTK("ATA_CMD_PIO_WRITE_EXT - tag=%d\n", tag); break;
			case ATA_CMD_SET_FEATURES	: DPRINTK("ATA_CMD_SET_FEATURES - tag=%d\n", tag); break;
			case ATA_CMD_PACKET		: DPRINTK("ATA_CMD_PACKET - tag=%d\n", tag); break;
			//case HSATA_CMD_QWRITE		: DPRINTK("HSATA_CMD_QWRITE - tag=%d\n", tag); break;
			//case HSATA_CMD_QREAD		: DPRINTK("HSATA_CMD_QREAD - tag=%d\n", tag); break;
			default				: DPRINTK("ATA_CMD_??? (0x%X)\n", tf->command); break;
		}
		val32 = readl((void *)(hsdev->membase + HSATA_SERROR_REG));
		DPRINTK("SERROR=0x%X\n", val32);
		writel(val32, (void *)(hsdev->membase + HSATA_SERROR_REG));
		val32 = readl((void *)(hsdev->membase + HSATA_INTPR_REG));
		DPRINTK("INTPR=0x%x\n", val32);
	}
#endif 

	ata_sff_exec_command(ap, tf);
}

static void hsata_exec_command(struct ata_port *ap, const struct ata_taskfile *tf)
{
	hsata_exec_command_by_tag(ap, tf, 0);
}

#ifdef HSATA_TANGOX_DMA
static u8 hsata_bmdma_status( struct ata_port *ap)
{
	return 0;
}

static void hsata_bmdma_stop(struct ata_queued_cmd *qc)
{
}

/* converting scatterlist into transfer units */
static int process_sg(struct hsata_device *hsdev, struct scatterlist *sg_list, int nelem)
{
#define KB_MASK		((1<<12)-1)
#define FLAT_SIZE	((8192-1)*2)	/* limit of double transfer */
	struct scatterlist *sg = NULL; 
	struct trans_unit *uptr = &hsdev->t_unit[0];
	unsigned int si, kb, lkb;
	u32 dma_addr, dma_len;

	hsdev->trans_unit_cnt = hsdev->next_unit = 0;

	for_each_sg(sg_list, sg, nelem, si) {
		dma_addr = sg_dma_address(sg); 
		dma_len = sg_dma_len(sg);
		kb = dma_len & ~KB_MASK;
		lkb = dma_len & KB_MASK;
		if ((dma_len > FLAT_SIZE) && (lkb != 0)) {
			uptr->dma_len = kb;	/* break this sg into two units */
			uptr->dma_addr = dma_addr;
			hsdev->trans_unit_cnt++;
			uptr++;
			uptr->dma_len = lkb;
			uptr->dma_addr = dma_addr + kb;
		} else {
			uptr->dma_len = dma_len;
			uptr->dma_addr = dma_addr;
		}
		hsdev->trans_unit_cnt++;
		uptr++;
		BUG_ON(hsdev->trans_unit_cnt == MAX_TRANS_UNITS);
	}
	return 0;
}

static int hsata_bmdma_setup_noexec(struct ata_queued_cmd *qc)
{
	struct ata_port *ap = qc->ap;
	struct scatterlist *sg = qc->sg; 
	int dir;
	int nents;
	u32 dev_dma_len = 0;
	struct hsata_device *hsdev = HSDEV_FROM_AP(ap);
	struct trans_unit *uptr = NULL;
	unsigned long flags;

	dir = qc->dma_dir;

	BUG_ON(hsdev->mbus_reg == 0);

	if ((nents = dma_map_sg((struct device *)qc->dev, sg, qc->n_elem, dir)) <= 0) {
		printk("(%d) dma map sg failed, please check\n", smp_processor_id());
		return -1;
	}

	if (dir == DMA_TO_DEVICE) {
		unsigned int si;
		for_each_sg(qc->sg, sg, qc->n_elem, si) {
			dev_dma_len += sg_dma_len(sg);
		}
	} else
		dev_dma_len = get_dma_len(qc);

	/* get the first one*/
	sg = qc->sg; 

#ifdef HSATA_VERBOSE
 	printk("(%d) setup_dma address=0x%x len=0x%x n_elem=0x%x nents=0x%x dir=0x%x block_len=0x%x dma_len/8k=0x%x\n", smp_processor_id(),
		 sg_dma_address(sg), sg_dma_len(sg), qc->n_elem, nents, dir, sg_dma_len(sg) / 4, sg_dma_len(sg) / (8 * 1024));
	printk("(%d) total len=0x%x n_elem=0x%x nents=0x%x dir=0x%x block_len=0x%x dma_len/8k=0x%x\n", smp_processor_id(),
		 dev_dma_len, qc->n_elem, nents, dir, dev_dma_len / 4, dev_dma_len / (8 * 1024));
#endif

	spin_lock_irqsave(&hsdev->lock, flags);

	if (process_sg(hsdev, sg, nents)) {
		BUG();
		goto done;
	}

	hsdev->mbus_pending = 1; /* starting mbus operation */
	hsdev->next_unit = 1;
	uptr = &hsdev->t_unit[0];
	if (hsdev->trans_unit_cnt == 1) { /* only one entry */
		if (em86xx_mbus_setup_dma(hsdev->mbus_reg, uptr->dma_addr, uptr->dma_len,
			hsata_mbus_done, hsdev, 1)) {
			em86xx_mbus_free_dma(hsdev->mbus_reg, hsdev->sbox);
			hsdev->mbus_reg = hsdev->mbus_pending = 0;
			spin_unlock_irqrestore(&hsdev->lock, flags);
			return -1;
		}
	} else { /* trans_unit_cnt > 1 */
		if (em86xx_mbus_setup_dma(hsdev->mbus_reg, uptr->dma_addr, uptr->dma_len,
			hsata_mbus_intr, qc, 0)) {
			em86xx_mbus_free_dma(hsdev->mbus_reg, hsdev->sbox);
			hsdev->mbus_reg = hsdev->mbus_pending = 0;
			spin_unlock_irqrestore(&hsdev->lock, flags);
			return -1;
		}
	}

	hsdev->dev_pending = 1;	/* starting dma operation */
	writel((dev_dma_len / 4), (void *)(hsdev->ctl_base + 0x04)); /* write length in dword */

done:
	spin_unlock_irqrestore(&hsdev->lock, flags);
	return 0;
}

static void hsata_bmdma_setup(struct ata_queued_cmd *qc)
{
	if (hsata_bmdma_setup_noexec(qc)== -1)
		 hsata_reset_port(qc->ap);

	hsata_exec_command(qc->ap, &qc->tf);
}

static void hsata_bmdma_start(struct ata_queued_cmd *qc)
{
	struct hsata_device *hsdev = HSDEV_FROM_QC(qc);
	int dir = qc->dma_dir, val;
	extern int is_tango4_chip(void);
	extern unsigned long tangox_chip_id(void);
	unsigned long chip_id = (tangox_chip_id() >> 16) & 0xfffe;

	while (readl((void *)(hsdev->ctl_base + 0x08)))
		; /* wait for dma cleared */ 

	/* set DBTSR */
	writel(HSATA_DMA_DBTSR, (void *)(hsdev->membase + HSATA_DBTSR_REG));
	if (dir == DMA_TO_DEVICE) {
		if (is_tango4_chip() || (chip_id == 0x8672) || (chip_id == 0x8674)) {
			/* bypass cipher for writing */
			val =readl((void *)(tangox_aes_config[hsdev->controller]));
			val &= ~0xf0;
			val |= 0x10;
			writel(val, (void *)(tangox_aes_config[hsdev->controller]));
		}
	 	/* Tx Burst length */
		writel(TANGOX_BURST_LENGTH_TX, (void *)(hsdev->ctl_base + 0x00));
		/* Enable Tx*/
		writel(HSATA_DMACR_TX_EN, (void *)(hsdev->membase + HSATA_DMACR_REG)); 
	} else {
		if (is_tango4_chip() || (chip_id == 0x8672) || (chip_id == 0x8674)) {
			/* bypass cipher for reading */
			val = readl((void *)(tangox_aes_config[hsdev->controller]));
			val &= ~0xf0;
			writel(val, (void *)(tangox_aes_config[hsdev->controller]));
		}
	 	/* Rx Burst length */
	 	writel(BURST_LENGTH_RX, (void *)(hsdev->ctl_base + 0x00));
		/* Enable Rx*/
		writel(HSATA_DMACR_RX_EN, (void *)(hsdev->membase + HSATA_DMACR_REG)); 
	}
}
#endif /* HSATA_TANGOX_DMA */

static void hsata_qc_prep_by_tag(struct ata_queued_cmd *qc, u8 tag)
{
#ifdef HSATA_TANGOX_DMA
	int dir;
	dir = qc->dma_dir;

	DPRINTK("QC PREP id=%d dma dir=%s n_elem=%d\n", 
			qc->ap->print_id, 
			(dir == DMA_FROM_DEVICE) ? "FROM_DEVICE" : "TO_DEVICE",
			qc->n_elem);
#else 
	DPRINTK("QC PREP id=%d n_elem=%d\n", 
		qc->ap->print_id, 
		qc->n_elem);
#endif
}

static void hsata_qc_prep(struct ata_queued_cmd *qc)
{
	hsata_qc_prep_by_tag(qc, 0);
}

static unsigned int hsata_qc_issue(struct ata_queued_cmd *qc)
{
#ifdef HSATA_VERBOSE
	switch (qc->tf.protocol) {
		case ATA_PROT_DMA: DPRINTK("ATA_PROT_DMA\n"); break;
		case ATA_PROT_PIO: DPRINTK("ATA_PROT_PIO\n"); break;
	}
#endif 
	return ata_bmdma_qc_issue(qc);
}

void hsata_error_handler(struct ata_port *ap)
{
#ifdef HSATA_TANGOX_DMA
	struct hsata_device *hsdev = HSDEV_FROM_AP(ap);
	unsigned long flags;
	spin_lock_irqsave(&hsdev->lock, flags);
	if ((hsdev->dev_pending != 0) || (hsdev->mbus_pending != 0)) {
		hsdev->dev_pending = hsdev->mbus_pending = 0;
		if (hsdev->mbus_reg != 0) {
			em86xx_mbus_wait(hsdev->mbus_reg, hsdev->sbox);
			em86xx_mbus_free_dma(hsdev->mbus_reg, hsdev->sbox);
			hsdev->mbus_reg = 0;
		}
	}
	spin_unlock_irqrestore(&hsdev->lock, flags);
#endif
	ata_std_error_handler(ap);
}

static int hsata_check_atapi_dma(struct ata_queued_cmd *qc)
{
	u8 cmnd = qc->scsicmd->cmnd[0];

	if ((cmnd == GPCMD_READ_TOC_PMA_ATIP) || 
		(cmnd == GPCMD_MODE_SENSE_10)) 
		return 1;
	else
		return 0;
}

static struct scsi_host_template hsata_sht = {
	.module			= THIS_MODULE,
	.name			= DRV_NAME0,
	.ioctl			= hsata_scsi_ioctl,
	.queuecommand		= hsata_scsi_queuecmd,
	.can_queue		= ATA_DEF_QUEUE,
	.this_id		= ATA_SHT_THIS_ID,
	.sg_tablesize		= LIBATA_MAX_PRD,
	.max_sectors		= 2 * ATA_MAX_SECTORS,
	.cmd_per_lun		= ATA_SHT_CMD_PER_LUN,
	.emulated		= ATA_SHT_EMULATED,
	.use_clustering		= ATA_SHT_USE_CLUSTERING,
	.proc_name		= DRV_NAME0,
	.dma_boundary		= ATA_DMA_BOUNDARY,
	.slave_configure	= hsata_scsi_slave_cfg,
	.slave_destroy		= ata_scsi_slave_destroy,
	.bios_param		= hsata_std_bios_param,
};

static struct ata_port_operations hsata_ops = {
	.inherits		= &ata_bmdma_port_ops,
	.set_mode		= hsata_set_mode,
	.mode_filter	= hsata_mode_filter,
	.check_atapi_dma	= hsata_check_atapi_dma,
#ifdef HSATA_TANGOX_DMA
	.bmdma_setup		= hsata_bmdma_setup,
	.bmdma_start		= hsata_bmdma_start,
	.bmdma_stop		= hsata_bmdma_stop,
	.bmdma_status		= hsata_bmdma_status,
#endif
	.qc_prep		= hsata_qc_prep,
	.qc_issue		= hsata_qc_issue,
	.freeze			= ata_sff_freeze,
	.thaw			= ata_sff_thaw,
	.scr_read		= hsata_scr_read,
	.scr_write		= hsata_scr_write,
	.error_handler		= hsata_error_handler,
	.port_start		= hsata_port_start,
	.port_stop		= hsata_port_stop,
	.sff_irq_clear		= hsata_irq_clear,
	.sff_exec_command	= hsata_exec_command,
};

static struct ata_port_info hsata_port_info = 
{
	.flags	= ATA_FLAG_SATA |	 
#ifndef HSATA_TANGOX_DMA
			ATA_DFLAG_PIO |	 /* set to NOT use DMA */
#endif
			ATA_DFLAG_LBA48 |	/* READ/WRITE EXT support */
			0x0,			
	.pio_mask	= 0x1f, /* pio0-4	- IDENTIFY DEVICE word 63 */
#ifdef HSATA_TANGOX_DMA
	.mwdma_mask	= 0x07, /* mwdma0-2	- IDENTIFY DEVICE word 64 */
	.udma_mask	= 0x7f, /* udma0-6	- IDENTIFY DEVICE word 88 */
#else
	.mwdma_mask	= 0x00, 
	.udma_mask	= 0x00,
#endif
	.port_ops	= &hsata_ops,
};

int hsata_probe(struct platform_device *pdev, int controller)
{
	int status = 0;
	int num_ports = 1;
	struct ata_host *host;
	struct ata_port *ap;
	struct hsata_device *hsdev;
	const struct ata_port_info *ppi[1];
	int irq;
	struct resource *res;
	unsigned long flags;

	ppi[0] = &hsata_port_info;

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

	/*
	 * Device data struct
	 */
	if ((hsdev = kzalloc(sizeof(*hsdev), GFP_KERNEL)) == NULL) {
		status = -ENOMEM;
		goto CLEANUP;
	} else if ((host = ata_host_alloc_pinfo(&pdev->dev, ppi, num_ports)) == NULL) {
		kfree(hsdev);
		printk("cannot alloc host port info.\n");
		return -ENOMEM;
	}

	spin_lock_init(&hsdev->lock);
	spin_lock_irqsave(&hsdev->lock, flags);
	hsdev->pdev = pdev;
	hsdev->host = host;
	hsdev->membase = NON_CACHED(res->start);
	hsdev->ctl_base = tangox_ctl_base[controller];
	hsdev->sata_irq = irq;
	hsdev->controller = controller;
	hsdev->mbus_reg = 0;
	hsdev->next_unit = hsdev->trans_unit_cnt = 0;
	memset(hsdev->t_unit, 0, sizeof(struct trans_unit) * MAX_TRANS_UNITS);
	hsdev->sbox = tangox_sbox[controller];
	hsdev->dma_irq = tangox_sata_dma_irq[controller];
	hsdev->mbus_pending = hsdev->dev_pending = 0;
	spin_unlock_irqrestore(&hsdev->lock, flags);

	DPRINTK("SATA membase (0x%x)\n", hsdev->membase);
	DPRINTK("SATA ctl_base (0x%x)\n", hsdev->ctl_base);
	DPRINTK("SATA sata_irq (0x%x)\n", hsdev->sata_irq);
	DPRINTK("SATA mbus_reg (0x%x)\n", hsdev->g_mbus_reg);
	DPRINTK("SATA sbox (0x%x)\n", hsdev->sbox);
	DPRINTK("SATA dma_irq (0x%x)\n", hsdev->dma_irq);	

	host->dev = &pdev->dev;
	host->private_data = hsdev;

	ap = host->ports[0];
	hsata_setup_port(&ap->ioaddr, hsdev->membase);

	/* Init the host controller*/
	hsata_host_init(hsdev);
	DPRINTK("hsata_host_init done\n");
	/*
	 * Interrupt management for SATA interrupts is done by the libata layer 
	 * (see ata_device_add). See hsata_port_start() for init of DMAC
	 * interrupt.
	 */
	DPRINTK("start ata_host_activate , status=0x%x irq=0x%x, membase=0x%x\n", 
				status, irq, hsdev->membase);

	status = ata_host_activate(hsdev->host, irq, 
					hsata_isr, IRQF_DISABLED, &hsata_sht);

	DPRINTK("ata_host_activate done, status=0x%x\n", status);

	hsata_enable_interrupts(hsdev);

CLEANUP:
	if (status) {
		dev_set_drvdata(&pdev->dev, NULL);	/* clear private data ptr */

		if (hsdev) 
			kfree(hsdev);
	}
	DPRINTK("DONE - %s - nports=%d\n", status?"ERROR":"OK", num_ports);
	return status;
}


int hsata_probe0(struct platform_device *pdev)
{
	return hsata_probe(pdev, 0);
}

int hsata_probe1(struct platform_device *pdev)
{
	return hsata_probe(pdev, 1);
}

static void tangox_sata_shutdown(struct hsata_device *hsdev)
{
	int val = 0;

	/* put into slumber mode */
	writel(1 << 13, (void *)(hsdev->membase + HSATA_SCR2_REG));

	/* clear the SError Register */
	val = readl((void *)(hsdev->membase  + HSATA_SERROR_REG));
	writel(val, (void *)(hsdev->membase  + HSATA_SERROR_REG));
	writel(0, (void *)(hsdev->membase + HSATA_ERRMR_REG));

	udelay(1000);
}

static void tangox_sata_pll_off(void)
{
	int val = 0;

	/* and then turn off PHY PLL */
	val = readl((void *)(TANGOX_SATA0_CTL_BASE + 0x14));
	if (!(val & 0x1))
		writel(val | 0x1, (void *)(TANGOX_SATA0_CTL_BASE + 0x14));
}

static int hsata_remove(struct platform_device *pdev, int controller)
{
	struct hsata_device *hsdev;
	struct ata_host *host;
	struct ata_port *ap;

	host = dev_get_drvdata(&pdev->dev);
	hsdev = HSDEV_FROM_HOST_SET(host);
	ap = host->ports[0];

	hsata_disable_interrupts(hsdev);

#ifdef HSATA_TANGOX_DMA
	free_irq(hsdev->dma_irq, host);
#endif
	ata_host_detach(host);

	tangox_sata_shutdown(hsdev);

	if (hsdev) 
		kfree(hsdev);

	return 0;
}

static int hsata_remove0(struct platform_device *pdev)
{
	hsata_remove(pdev, 0);
	return 0;
}

static int hsata_remove1(struct platform_device *pdev)
{
	hsata_remove(pdev, 1);
	return 0;
}

static struct platform_driver hsata_driver0 = {
	.driver.name = (char *)DRV_NAME0,
	.driver.bus = &platform_bus_type,
	.probe = hsata_probe0,
	.remove = hsata_remove0,
};

static struct platform_driver hsata_driver1 = {
	.driver.name = (char *)DRV_NAME1,
	.driver.bus = &platform_bus_type,
	.probe = hsata_probe1,
	.remove = hsata_remove1,
};

static int __init tangox_hsata_module_init(void)
{
	int status = 0;
	int tangox_sata_enabled(void);

	if (tangox_sata_enabled() == 0) {
		printk("TangoX SATA support is disabled from XENV.\n");
		return -EINVAL;
	}

	if ((disable_ports & 1) == 0) {
		status = platform_driver_register(&hsata_driver0);
		if (status) {
			printk("Failed to register driver 0.\n");
			return status;
		}
	}

	if ((disable_ports & 2) == 0) {
		status = platform_driver_register(&hsata_driver1);
		if (status) {
			printk("Failed to register driver 1.\n");
			return status;
		}
	}

	return status;
}

static void __exit tangox_hsata_module_cleanup(void)
{
	if ((disable_ports & 1) == 0) {
		platform_driver_unregister(&hsata_driver0); 
	}

	if ((disable_ports & 2) == 0) {
		platform_driver_unregister(&hsata_driver1); 
	}

	tangox_sata_pll_off();
}

module_init(tangox_hsata_module_init);
module_exit(tangox_hsata_module_cleanup);

