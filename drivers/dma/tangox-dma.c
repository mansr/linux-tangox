/*
 * Copyright (C) 2014 Mans Rullgard <mans@mansr.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/dmaengine.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_dma.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/bitops.h>

#include "virt-dma.h"

#define TANGOX_DMA_MAX_CHANS	6
#define TANGOX_DMA_MAX_PCHANS	6

#define SBOX_RESET		0
#define SBOX_RESET2		4
#define SBOX_ROUTE		8
#define SBOX_ROUTE2		12

#define DMA_ADDR		0
#define DMA_COUNT		4
#define DMA_ADDR2		8
#define DMA_STRIDE		DMA_ADDR2
#define DMA_CMD			12

#define DMA_CMD_SINGLE		1
#define DMA_CMD_DOUBLE		2
#define DMA_CMD_RECT		3

#define DMA_LAST_XFER		1

#define DMA_COMPLETE_HOST	0
#define DMA_COMPLETE_DEV	1
#define DMA_COMPLETE_DONE	2

struct tangox_dma_pchan;

struct tangox_dma_sg {
	dma_addr_t addr;
	unsigned int len;
};

struct tangox_dma_desc {
	struct virt_dma_desc vd;
	enum dma_transfer_direction direction;
	struct tangox_dma_pchan *pchan;
	bool last;
	unsigned long complete;
	unsigned int num_sgs;
	unsigned int next_sg;
	struct tangox_dma_sg sg[];
};

struct tangox_dma_chan {
	struct virt_dma_chan vc;
	u32 id;
};

struct tangox_dma_pchan {
	struct tangox_dma_device *dev;
	enum dma_transfer_direction direction;
	u32 sbox_id;
	int slave_id;
	void __iomem *base;
	spinlock_t lock;
	struct tangox_dma_desc *desc;
	unsigned long issued_len;
	struct tasklet_struct complete;
};

struct tangox_dma_regs {
	unsigned int xcnt_bits;
	unsigned int xcnt_max;
	unsigned int ycnt_bits;
	unsigned int ycnt_max;
	unsigned int cmd_shift;
	unsigned int flags_shift;
};

struct tangox_dma_device {
	struct dma_device ddev;
	void __iomem *sbox_base;
	const struct tangox_dma_regs *regs;
	spinlock_t lock;
	struct list_head desc_memtodev;
	struct list_head desc_devtomem;
	int nr_pchans;
	struct tangox_dma_pchan pchan[TANGOX_DMA_MAX_PCHANS];
	struct tangox_dma_chan chan[TANGOX_DMA_MAX_CHANS];
};

static inline struct tangox_dma_device *to_tangox_dma_device(
	struct dma_device *ddev)
{
	return container_of(ddev, struct tangox_dma_device, ddev);
}

static inline struct tangox_dma_chan *to_tangox_dma_chan(struct dma_chan *c)
{
	return container_of(c, struct tangox_dma_chan, vc.chan);
}

static inline struct tangox_dma_desc *to_tangox_dma_desc(
	struct dma_async_tx_descriptor *d)
{
	return container_of(d, struct tangox_dma_desc, vd.tx);
}

static inline struct tangox_dma_desc *vdesc_to_tangox_dma_desc(
	struct virt_dma_desc *vdesc)
{
	return container_of(vdesc, struct tangox_dma_desc, vd);
}

static struct tangox_dma_desc *tangox_dma_alloc_desc(unsigned int num_sgs)
{
	return kzalloc(sizeof(struct tangox_dma_desc) +
		sizeof(struct tangox_dma_sg) * num_sgs, GFP_ATOMIC);
}

static void tangox_dma_sbox_map(struct tangox_dma_device *dev, int src, int dst)
{
	void __iomem *addr = dev->sbox_base + SBOX_ROUTE;
	int shift = (dst - 1) * 4;

	if (shift > 31) {
		addr += 4;
		shift -= 32;
	}

	writel_relaxed(src << shift, addr);
}

static void tangox_dma_pchan_setup(struct tangox_dma_pchan *pchan,
				   struct tangox_dma_desc *desc)
{
	struct tangox_dma_chan *chan = to_tangox_dma_chan(desc->vd.tx.chan);
	struct tangox_dma_device *dev = pchan->dev;

	BUG_ON(desc->direction != pchan->direction);

	if (pchan->direction == DMA_DEV_TO_MEM)
		tangox_dma_sbox_map(dev, chan->id, pchan->sbox_id);
	else
		tangox_dma_sbox_map(dev, pchan->sbox_id, chan->id);

	pchan->slave_id = chan->id;
}

static void tangox_dma_pchan_detach(struct tangox_dma_pchan *pchan)
{
	struct tangox_dma_device *dev = pchan->dev;

	BUG_ON(pchan->slave_id < 0);

	if (pchan->direction == DMA_DEV_TO_MEM)
		tangox_dma_sbox_map(dev, 0xf, pchan->sbox_id);
	else
		tangox_dma_sbox_map(dev, 0xf, pchan->slave_id);

	pchan->slave_id = -1;
}

static void tangox_dma_issue_cmd(struct tangox_dma_pchan *pchan,
				 unsigned int cmd, unsigned int flags)
{
	const struct tangox_dma_regs *regs = pchan->dev->regs;
	u32 val;

	val = cmd << regs->cmd_shift | flags << regs->flags_shift;

	writel_relaxed(val, pchan->base + DMA_CMD);
}

static int tangox_dma_issue_single(struct tangox_dma_pchan *pchan,
				   struct tangox_dma_sg *sg, int flags)
{
	const struct tangox_dma_regs *regs = pchan->dev->regs;
	unsigned int len = min(sg->len, regs->xcnt_max);

	if (len < sg->len)
		flags &= ~DMA_LAST_XFER;

	writel_relaxed(sg->addr, pchan->base + DMA_ADDR);
	writel_relaxed(len, pchan->base + DMA_COUNT);

	tangox_dma_issue_cmd(pchan, DMA_CMD_SINGLE, flags);

	return len;
}

static int tangox_dma_issue_double(struct tangox_dma_pchan *pchan,
				   struct tangox_dma_sg *sg, int flags)
{
	const struct tangox_dma_regs *regs = pchan->dev->regs;
	unsigned int len = min(sg->len, regs->xcnt_max);
	unsigned int len1 = min(sg->len - len, regs->ycnt_max);

	if (len + len1 < sg->len)
		flags &= ~DMA_LAST_XFER;

	writel_relaxed(sg->addr, pchan->base + DMA_ADDR);
	writel_relaxed(sg->addr + len, pchan->base + DMA_ADDR2);
	writel_relaxed(len | len1 << 16, pchan->base + DMA_COUNT);

	tangox_dma_issue_cmd(pchan, DMA_CMD_DOUBLE, flags);

	return len + len1;
}

static int tangox_dma_issue_rect(struct tangox_dma_pchan *pchan,
				 struct tangox_dma_sg *sg, int flags)
{
	const struct tangox_dma_regs *regs = pchan->dev->regs;
	unsigned int shift = __fls(sg->len) - regs->xcnt_bits - 1;
	unsigned int width = sg->len >> shift;
	unsigned int count = min(1u << shift, regs->ycnt_max);

	if (count * width < sg->len)
		flags &= ~DMA_LAST_XFER;

	writel_relaxed(sg->addr, pchan->base + DMA_ADDR);
	writel_relaxed(width, pchan->base + DMA_STRIDE);
	writel_relaxed(width | count << 16, pchan->base + DMA_COUNT);

	tangox_dma_issue_cmd(pchan, DMA_CMD_RECT, flags);

	return count << shift;
}

static void tangox_dma_pchan_issue(struct tangox_dma_pchan *pchan)
{
	const struct tangox_dma_regs *regs = pchan->dev->regs;
	struct tangox_dma_desc *desc = pchan->desc;
	struct tangox_dma_sg *sg = &desc->sg[desc->next_sg];
	int flags = 0;
	int len;

	if (desc->next_sg == desc->num_sgs - 1)
		flags = DMA_LAST_XFER;

	if (sg->len <= regs->xcnt_max)
		len = tangox_dma_issue_single(pchan, sg, flags);
	else if (sg->len <= regs->xcnt_max + regs->ycnt_max)
		len = tangox_dma_issue_double(pchan, sg, flags);
	else
		len = tangox_dma_issue_rect(pchan, sg, flags);

	sg->addr += len;
	sg->len  -= len;

	if (!sg->len)
		desc->next_sg++;

	pchan->issued_len = len;
}

static struct tangox_dma_desc *tangox_dma_next_desc(
	struct tangox_dma_device *dev, enum dma_transfer_direction dir)
{
	struct tangox_dma_desc *desc;
	struct list_head *list;
	unsigned long flags;

	if (dir == DMA_MEM_TO_DEV)
		list = &dev->desc_memtodev;
	else
		list = &dev->desc_devtomem;

	spin_lock_irqsave(&dev->lock, flags);

	desc = list_first_entry_or_null(list, struct tangox_dma_desc, vd.node);
	if (desc)
		list_del(&desc->vd.node);

	spin_unlock_irqrestore(&dev->lock, flags);

	return desc;
}

static void tangox_dma_desc_complete(struct tangox_dma_desc *desc, int comp)
{
	if (desc->last) {
		set_bit(comp, &desc->complete);
		if (desc->complete != 3)
			return;

		if (test_and_set_bit(DMA_COMPLETE_DONE, &desc->complete))
			return;
	}

	tasklet_schedule(&desc->pchan->complete);
}

static void tangox_dma_pchan_start(struct tangox_dma_pchan *pchan)
{
	struct tangox_dma_device *dev = pchan->dev;
	struct tangox_dma_desc *desc = pchan->desc;

	if (!desc) {
		desc = tangox_dma_next_desc(dev, pchan->direction);
		if (!desc) {
			tangox_dma_pchan_detach(pchan);
			return;
		}

		pchan->desc = desc;
		tangox_dma_pchan_setup(pchan, desc);
	}

	desc->pchan = pchan;

	if (desc->num_sgs)
		tangox_dma_pchan_issue(pchan);
	else
		tangox_dma_desc_complete(desc, DMA_COMPLETE_HOST);
}

static void tangox_dma_queue_desc(struct tangox_dma_device *dev,
				  struct tangox_dma_desc *desc)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);
	if (desc->direction == DMA_MEM_TO_DEV)
		list_add_tail(&desc->vd.node, &dev->desc_memtodev);
	else
		list_add_tail(&desc->vd.node, &dev->desc_devtomem);
	spin_unlock_irqrestore(&dev->lock, flags);
}

static void tangox_dma_pchan_complete(unsigned long data)
{
	struct tangox_dma_pchan *pchan = (struct tangox_dma_pchan *)data;
	struct tangox_dma_chan *chan;
	struct tangox_dma_desc *desc;
	struct virt_dma_desc *vdesc;
	bool last;

	desc = pchan->desc;
	chan = to_tangox_dma_chan(desc->vd.tx.chan);
	last = desc->last;

	spin_lock_irq(&chan->vc.lock);
	vchan_cookie_complete(&desc->vd);
	vdesc = vchan_next_desc(&chan->vc);
	if (vdesc)
		list_del(&vdesc->node);
	spin_unlock_irq(&chan->vc.lock);

	desc = vdesc ? vdesc_to_tangox_dma_desc(vdesc) : NULL;

	if (desc && (last || desc->direction != pchan->direction)) {
		tangox_dma_queue_desc(pchan->dev, desc);
		desc = NULL;
	}

	spin_lock_irq(&pchan->lock);
	pchan->desc = desc;
	tangox_dma_pchan_start(pchan);
	spin_unlock_irq(&pchan->lock);
}

static irqreturn_t tangox_dma_irq(int irq, void *irq_data)
{
	struct tangox_dma_pchan *pchan = irq_data;
	struct tangox_dma_desc *desc = pchan->desc;
	struct dma_chan *dc;

	if (!desc)
		return IRQ_NONE;

	dc = desc->vd.tx.chan;
	this_cpu_ptr(dc->local)->bytes_transferred += pchan->issued_len;

	if (desc->next_sg == desc->num_sgs) {
		tangox_dma_desc_complete(desc, DMA_COMPLETE_HOST);
	} else {
		tangox_dma_pchan_start(pchan);
	}

	return IRQ_HANDLED;
}

static void tangox_dma_start(struct tangox_dma_device *dev,
			     enum dma_transfer_direction dir)
{
	struct tangox_dma_pchan *pchan = NULL;
	unsigned long flags;
	int i;

	for (i = 0; i < dev->nr_pchans; i++) {
		pchan = &dev->pchan[i];
		if (pchan->direction == dir && !pchan->desc)
			break;
	}

	if (i == dev->nr_pchans)
		return;

	spin_lock_irqsave(&pchan->lock, flags);
	if (!pchan->desc)
		tangox_dma_pchan_start(pchan);
	spin_unlock_irqrestore(&pchan->lock, flags);
}

static void tangox_dma_issue_pending(struct dma_chan *c)
{
	struct tangox_dma_device *dev = to_tangox_dma_device(c->device);
	struct tangox_dma_chan *chan = to_tangox_dma_chan(c);
	struct tangox_dma_desc *desc = NULL;
	struct virt_dma_desc *vdesc;
	unsigned long flags;

	spin_lock_irqsave(&chan->vc.lock, flags);
	if (vchan_issue_pending(&chan->vc)) {
		desc = list_last_entry(&chan->vc.desc_issued,
				       struct tangox_dma_desc, vd.node);
		desc->last = true;

		vdesc = vchan_next_desc(&chan->vc);
		list_del(&vdesc->node);
		desc = vdesc_to_tangox_dma_desc(vdesc);
	}
	spin_unlock_irqrestore(&chan->vc.lock, flags);

	if (desc) {
		tangox_dma_queue_desc(dev, desc);
		tangox_dma_start(dev, desc->direction);
	}
}

static void tangox_dma_async_complete(struct dma_chan *c,
				      struct dma_async_tx_descriptor *d)
{
	struct tangox_dma_desc *desc = to_tangox_dma_desc(d);

	if (WARN_ON(!desc->last))
		return;

	tangox_dma_desc_complete(desc, DMA_COMPLETE_DEV);
}

static struct dma_async_tx_descriptor *tangox_dma_prep_slave_sg(
	struct dma_chan *c, struct scatterlist *sgl, unsigned int sg_len,
	enum dma_transfer_direction direction,
	unsigned long flags, void *context)
{
	struct tangox_dma_chan *chan = to_tangox_dma_chan(c);
	struct tangox_dma_desc *desc;
	struct scatterlist *sg;
	unsigned int i;

	if (!is_slave_direction(direction))
		return NULL;

	desc = tangox_dma_alloc_desc(sg_len);
	if (!desc)
		return NULL;

	for_each_sg(sgl, sg, sg_len, i) {
		desc->sg[i].addr = sg_dma_address(sg);
		desc->sg[i].len = sg_dma_len(sg);
	}

	desc->num_sgs = sg_len;
	desc->direction = direction;

	return vchan_tx_prep(&chan->vc, &desc->vd, flags);
}

static enum dma_status tangox_dma_tx_status(struct dma_chan *c,
	dma_cookie_t cookie, struct dma_tx_state *state)
{
	return dma_cookie_status(c, cookie, state);
}

static int tangox_dma_alloc_chan_resources(struct dma_chan *c)
{
	return 0;
}

static void tangox_dma_free_chan_resources(struct dma_chan *c)
{
	vchan_free_chan_resources(to_virt_chan(c));
}

static void tangox_dma_desc_free(struct virt_dma_desc *vd)
{
	kfree(container_of(vd, struct tangox_dma_desc, vd));
}

static void tangox_dma_reset(struct tangox_dma_device *dev)
{
	writel_relaxed(0xffffffff, dev->sbox_base + SBOX_RESET);
	writel_relaxed(0xffffffff, dev->sbox_base + SBOX_RESET2);

	udelay(2);

	writel_relaxed(0xff00ff00, dev->sbox_base + SBOX_RESET);
	writel_relaxed(0xff00ff00, dev->sbox_base + SBOX_RESET2);

	writel_relaxed(0xffffffff, dev->sbox_base + SBOX_ROUTE);
	writel_relaxed(0xffffffff, dev->sbox_base + SBOX_ROUTE2);
}

static struct dma_chan *tangox_dma_xlate(struct of_phandle_args *dma_spec,
					 struct of_dma *ofdma)
{
	struct dma_device *dev = ofdma->of_dma_data;
	struct tangox_dma_chan *chan;
	struct dma_chan *c;

	if (!dev || dma_spec->args_count != 1)
		return NULL;

	list_for_each_entry(c, &dev->channels, device_node) {
		chan = to_tangox_dma_chan(c);
		if (chan->id == dma_spec->args[0])
			return dma_get_slave_channel(c);
	}

	return NULL;
}

static const struct tangox_dma_regs tango3_regs = {
	.xcnt_bits	= 13,
	.xcnt_max	= GENMASK(12, 0),
	.ycnt_bits	= 14,
	.ycnt_max	= GENMASK(13, 0),
	.cmd_shift	= 0,
	.flags_shift	= 2,
};

static const struct tangox_dma_regs tango4_regs = {
	.xcnt_bits	= 13,
	.xcnt_max	= GENMASK(12, 0),
	.ycnt_bits	= 14,
	.ycnt_max	= GENMASK(13, 0),
	.cmd_shift	= 1,
	.flags_shift	= 0,
};

static const struct of_device_id tangox_dma_dt_ids[] = {
	{ .compatible = "sigma,smp8642-dma", .data = &tango3_regs },
	{ .compatible = "sigma,smp8758-dma", .data = &tango4_regs },
	{ }
};
MODULE_DEVICE_TABLE(of, tangox_dma_dt_ids);

static int tangox_dma_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct device_node *cnode;
	struct tangox_dma_device *dmadev;
	struct tangox_dma_pchan *pchan;
	struct tangox_dma_chan *chan;
	struct dma_device *dd;
	struct resource *res;
	struct resource cres;
	const struct of_device_id *match;
	int irq;
	int err;
	int i;

	match = of_match_device(tangox_dma_dt_ids, &pdev->dev);
	if (!match)
		return -EINVAL;

	dmadev = devm_kzalloc(&pdev->dev, sizeof(*dmadev), GFP_KERNEL);
	if (!dmadev)
		return -ENOMEM;

	dmadev->regs = match->data;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dmadev->sbox_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(dmadev->sbox_base))
		return PTR_ERR(dmadev->sbox_base);

	tangox_dma_reset(dmadev);

	dd = &dmadev->ddev;

	dma_cap_set(DMA_SLAVE, dd->cap_mask);

	dd->dev = &pdev->dev;

	dd->directions = 1 << DMA_MEM_TO_DEV | 1 << DMA_DEV_TO_MEM;
	dd->device_alloc_chan_resources = tangox_dma_alloc_chan_resources;
	dd->device_free_chan_resources = tangox_dma_free_chan_resources;
	dd->device_prep_slave_sg = tangox_dma_prep_slave_sg;
	dd->device_tx_status = tangox_dma_tx_status;
	dd->device_issue_pending = tangox_dma_issue_pending;
	dd->device_async_complete = tangox_dma_async_complete;

	INIT_LIST_HEAD(&dd->channels);

	for (i = 0; i < TANGOX_DMA_MAX_CHANS; i++) {
		chan = &dmadev->chan[i];

		if (of_property_read_u32_index(node, "sigma,slave-ids", i,
					       &chan->id))
			break;

		chan->vc.desc_free = tangox_dma_desc_free;
		vchan_init(&chan->vc, dd);
	}

	dd->chancnt = i;

	spin_lock_init(&dmadev->lock);
	INIT_LIST_HEAD(&dmadev->desc_memtodev);
	INIT_LIST_HEAD(&dmadev->desc_devtomem);

	for_each_available_child_of_node(node, cnode) {
		pchan = &dmadev->pchan[dmadev->nr_pchans];
		pchan->dev = dmadev;
		spin_lock_init(&pchan->lock);
		tasklet_init(&pchan->complete, tangox_dma_pchan_complete,
			     (unsigned long)pchan);

		if (of_property_read_bool(cnode, "sigma,mem-to-dev"))
			pchan->direction = DMA_MEM_TO_DEV;
		else
			pchan->direction = DMA_DEV_TO_MEM;

		of_property_read_u32(cnode, "sigma,sbox-id", &pchan->sbox_id);

		err = of_address_to_resource(cnode, 0, &cres);
		if (err)
			return err;

		pchan->base = devm_ioremap_resource(&pdev->dev, &cres);
		if (IS_ERR(pchan->base))
			return PTR_ERR(pchan->base);

		irq = irq_of_parse_and_map(cnode, 0);
		if (!irq)
			return -EINVAL;

		err = devm_request_irq(&pdev->dev, irq, tangox_dma_irq, 0,
				       dev_name(&pdev->dev), pchan);
		if (err)
			return err;

		if (++dmadev->nr_pchans == TANGOX_DMA_MAX_PCHANS)
			break;
	}

	err = dma_async_device_register(dd);
	if (err)
		return err;

	err = of_dma_controller_register(node, tangox_dma_xlate, dd);
	if (err) {
		dma_async_device_unregister(dd);
		return err;
	}

	platform_set_drvdata(pdev, dmadev);

	dev_info(&pdev->dev, "SMP86xx DMA with %d channels, %d slaves\n",
		 dmadev->nr_pchans, dd->chancnt);

	return 0;
}

static int tangox_dma_remove(struct platform_device *pdev)
{
	struct tangox_dma_device *dmadev = platform_get_drvdata(pdev);

	of_dma_controller_free(pdev->dev.of_node);
	dma_async_device_unregister(&dmadev->ddev);

	return 0;
}

static struct platform_driver tangox_dma_driver = {
	.probe	= tangox_dma_probe,
	.remove	= tangox_dma_remove,
	.driver	= {
		.name		= "tangox-dma",
		.of_match_table	= tangox_dma_dt_ids,
	},
};
module_platform_driver(tangox_dma_driver);

MODULE_AUTHOR("Mans Rullgard <mans@mansr.com>");
MODULE_DESCRIPTION("SMP86xx DMA driver");
MODULE_LICENSE("GPL");
