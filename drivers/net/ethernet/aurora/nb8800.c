/*
 * Copyright (C) 2015 Mans Rullgard <mans@mansr.com>
 *
 * Mostly rewritten, based on driver from Sigma Designs.  Original
 * copyright notice below.
 *
 *
 * Driver for tangox SMP864x/SMP865x/SMP867x/SMP868x builtin Ethernet Mac.
 *
 * Copyright (C) 2005 Maxime Bizon <mbizon@freebox.fr>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/etherdevice.h>
#include <linux/delay.h>
#include <linux/ethtool.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/dma-mapping.h>
#include <linux/phy.h>
#include <linux/cache.h>
#include <linux/jiffies.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <asm/barrier.h>

#include "nb8800.h"

static inline u8 nb8800_readb(struct nb8800_priv *priv, int reg)
{
	return readb(priv->base + reg);
}

static inline u32 nb8800_readl(struct nb8800_priv *priv, int reg)
{
	return readl(priv->base + reg);
}

static inline void nb8800_writeb(struct nb8800_priv *priv, int reg, u8 val)
{
	writeb(val, priv->base + reg);
}

static inline void nb8800_writew(struct nb8800_priv *priv, int reg, u16 val)
{
	writew(val, priv->base + reg);
}

static inline void nb8800_writel(struct nb8800_priv *priv, int reg, u32 val)
{
	writel(val, priv->base + reg);
}

static inline void nb8800_maskb(struct nb8800_priv *priv, int reg,
				u32 mask, u32 val)
{
	u32 old = nb8800_readb(priv, reg);
	u32 new = (old & ~mask) | val;
	if (new != old)
		nb8800_writeb(priv, reg, new);
}

static inline void nb8800_maskl(struct nb8800_priv *priv, int reg,
				u32 mask, u32 val)
{
	u32 old = nb8800_readl(priv, reg);
	u32 new = (old & ~mask) | val;
	if (new != old)
		nb8800_writel(priv, reg, new);
}

static inline void nb8800_modb(struct nb8800_priv *priv, int reg, u8 bits,
			       bool set)
{
	nb8800_maskb(priv, reg, bits, set ? bits : 0);
}

static inline void nb8800_setb(struct nb8800_priv *priv, int reg, u8 bits)
{
	nb8800_maskb(priv, reg, bits, bits);
}

static inline void nb8800_clearb(struct nb8800_priv *priv, int reg, u8 bits)
{
	nb8800_maskb(priv, reg, bits, 0);
}

static inline void nb8800_modl(struct nb8800_priv *priv, int reg, u32 bits,
			       bool set)
{
	nb8800_maskl(priv, reg, bits, set ? bits : 0);
}

static inline void nb8800_setl(struct nb8800_priv *priv, int reg, u32 bits)
{
	nb8800_maskl(priv, reg, bits, bits);
}

static inline void nb8800_clearl(struct nb8800_priv *priv, int reg, u32 bits)
{
	nb8800_maskl(priv, reg, bits, 0);
}

static int nb8800_mdio_wait(struct mii_bus *bus)
{
	struct nb8800_priv *priv = bus->priv;
	u32 val;

	return readl_poll_timeout_atomic(priv->base + NB8800_MDIO_CMD,
					 val, !(val & MDIO_CMD_GO), 1, 1000);
}

static int nb8800_mdio_cmd(struct mii_bus *bus, u32 cmd)
{
	struct nb8800_priv *priv = bus->priv;
	int err;

	err = nb8800_mdio_wait(bus);
	if (err)
		return err;

	nb8800_writel(priv, NB8800_MDIO_CMD, cmd);
	udelay(10);
	nb8800_writel(priv, NB8800_MDIO_CMD, cmd | MDIO_CMD_GO);

	return nb8800_mdio_wait(bus);
}

static int nb8800_mdio_read(struct mii_bus *bus, int phy_id, int reg)
{
	struct nb8800_priv *priv = bus->priv;
	u32 val;
	int err;

	err = nb8800_mdio_cmd(bus, MIIAR_ADDR(phy_id) | MIIAR_REG(reg));
	if (err)
		return err;

	val = nb8800_readl(priv, NB8800_MDIO_STS);
	if (val & MDIO_STS_ERR)
		return 0xffff;

	return val & 0xffff;
}

static int nb8800_mdio_write(struct mii_bus *bus, int phy_id, int reg, u16 val)
{
	u32 cmd = MIIAR_DATA(val) | MIIAR_ADDR(phy_id) | MIIAR_REG(reg) |
		MDIO_CMD_WR;

	return nb8800_mdio_cmd(bus, cmd);
}

static void nb8800_mac_tx(struct net_device *dev, bool enable)
{
	struct nb8800_priv *priv = netdev_priv(dev);

	while (nb8800_readl(priv, NB8800_TXC_CR) & TCR_EN)
		cpu_relax();

	nb8800_modb(priv, NB8800_TX_CTL1, TX_EN, enable);
}

static void nb8800_mac_rx(struct net_device *dev, bool enable)
{
	nb8800_modb(netdev_priv(dev), NB8800_RX_CTL, RX_EN, enable);
}

static void nb8800_mac_af(struct net_device *dev, bool enable)
{
	nb8800_modb(netdev_priv(dev), NB8800_RX_CTL, RX_AF_EN, enable);
}

static void nb8800_stop_rx(struct net_device *dev)
{
	struct nb8800_priv *priv = netdev_priv(dev);
	u32 val;
	int i;

	for (i = 0; i < RX_DESC_COUNT; i++)
		priv->rx_descs[i].desc.config |= DESC_EOC;

	readl_poll_timeout(priv->base + NB8800_RXC_CR, val, !(val & RCR_EN),
			   1000, 1000000);
}

static void nb8800_start_rx(struct net_device *dev)
{
	nb8800_setl(netdev_priv(dev), NB8800_RXC_CR, RCR_EN);
}

static int nb8800_alloc_rx(struct net_device *dev, int i, bool napi)
{
	struct nb8800_priv *priv = netdev_priv(dev);
	struct nb8800_rx_desc *rxd = &priv->rx_descs[i];
	struct nb8800_rx_buf *rxb = &priv->rx_bufs[i];
	int size = L1_CACHE_ALIGN(RX_BUF_SIZE);
	dma_addr_t dma_addr;
	struct page *page;
	unsigned long offset;
	void *data;

	data = napi ? napi_alloc_frag(size) : netdev_alloc_frag(size);
	if (!data)
		return -ENOMEM;

	page = virt_to_head_page(data);
	offset = data - page_address(page);

	dma_addr = dma_map_page(&dev->dev, page, offset, RX_BUF_SIZE,
				DMA_FROM_DEVICE);

	if (dma_mapping_error(&dev->dev, dma_addr)) {
		skb_free_frag(data);
		return -ENOMEM;
	}

	rxb->page = page;
	rxb->offset = offset;

	rxd->desc.config = priv->rx_dma_config;
	rxd->desc.s_addr = dma_addr;

	return 0;
}

static void nb8800_receive(struct net_device *dev, int i, int len)
{
	struct nb8800_priv *priv = netdev_priv(dev);
	struct nb8800_rx_desc *rxd = &priv->rx_descs[i];
	struct page *page = priv->rx_bufs[i].page;
	int offset = priv->rx_bufs[i].offset;
	void *data = page_address(page) + offset;
	dma_addr_t dma = rxd->desc.s_addr;
	struct sk_buff *skb;
	int size;
	int err;

	size = len <= RX_COPYBREAK ? len : RX_COPYHDR;

	skb = napi_alloc_skb(&priv->napi, size);
	if (!skb) {
		netdev_err(dev, "rx skb allocation failed\n");
		dev->stats.rx_dropped++;
		return;
	}

	if (len <= RX_COPYBREAK) {
		dma_sync_single_for_cpu(&dev->dev, dma, len, DMA_FROM_DEVICE);
		memcpy(skb_put(skb, len), data, len);
		dma_sync_single_for_device(&dev->dev, dma, len,
					   DMA_FROM_DEVICE);
	} else {
		err = nb8800_alloc_rx(dev, i, true);
		if (err) {
			dev->stats.rx_dropped++;
			return;
		}

		dma_unmap_page(&dev->dev, dma, RX_BUF_SIZE, DMA_FROM_DEVICE);
		memcpy(skb_put(skb, RX_COPYHDR), data, RX_COPYHDR);
		skb_add_rx_frag(skb, skb_shinfo(skb)->nr_frags, page,
				offset + RX_COPYHDR, len - RX_COPYHDR,
				RX_BUF_SIZE);
	}

	skb->protocol = eth_type_trans(skb, dev);
	netif_receive_skb(skb);
}

static void nb8800_rx_error(struct net_device *dev, u32 report)
{
	if (report & RX_LENGTH_ERR)
		dev->stats.rx_length_errors++;

	if (report & RX_FCS_ERR)
		dev->stats.rx_crc_errors++;

	if (report & RX_FIFO_OVERRUN)
		dev->stats.rx_fifo_errors++;

	if (report & RX_ALIGNMENT_ERROR)
		dev->stats.rx_frame_errors++;

	dev->stats.rx_errors++;
}

static int nb8800_poll(struct napi_struct *napi, int budget)
{
	struct net_device *dev = napi->dev;
	struct nb8800_priv *priv = netdev_priv(dev);
	struct nb8800_rx_desc *rxd;
	int work = 0;
	int last = priv->rx_eoc;
	int next;

again:
	while (work < budget) {
		struct nb8800_rx_buf *rxb;
		int len;

		next = (last + 1) % RX_DESC_COUNT;

		rxb = &priv->rx_bufs[next];
		rxd = &priv->rx_descs[next];

		if (!rxd->report)
			break;

		len = RX_BYTES_TRANSFERRED(rxd->report);

		if (IS_RX_ERROR(rxd->report))
			nb8800_rx_error(dev, rxd->report);
		else
			nb8800_receive(dev, next, len);

		dev->stats.rx_packets++;
		dev->stats.rx_bytes += len;

		if (rxd->report & RX_MULTICAST_PKT)
			dev->stats.multicast++;

		rxd->report = 0;
		last = next;
		work++;
	}

	if (work) {
		priv->rx_descs[last].desc.config |= DESC_EOC;
		wmb();	/* ensure new EOC is written before clearing old */
		priv->rx_descs[priv->rx_eoc].desc.config &= ~DESC_EOC;
		priv->rx_eoc = last;
		nb8800_start_rx(dev);
	}

	if (work < budget) {
		nb8800_writel(priv, NB8800_RX_ITR, priv->rx_itr_irq);

		/* If a packet arrived after we last checked but
		 * before writing RX_ITR, the interrupt will be
		 * delayed, so we retrieve it now. */
		if (priv->rx_descs[next].report)
			goto again;

		napi_complete_done(napi, work);
	}

	return work;
}

static void nb8800_tx_dma_start(struct net_device *dev)
{
	struct nb8800_priv *priv = netdev_priv(dev);
	struct nb8800_tx_buf *txb;
	unsigned long flags;
	u32 txc_cr;

	spin_lock_irqsave(&priv->tx_lock, flags);

	txb = &priv->tx_bufs[priv->tx_queue];
	if (!txb->skb)
		goto end;

	txc_cr = nb8800_readl(priv, NB8800_TXC_CR);
	if (txc_cr & TCR_EN)
		goto end;

	nb8800_writel(priv, NB8800_TX_DESC_ADDR, txb->dma_desc);
	wmb();		/* ensure desc addr is written before starting DMA */
	nb8800_writel(priv, NB8800_TXC_CR, txc_cr | TCR_EN);

	priv->tx_queue = (priv->tx_queue + 1) % TX_DESC_COUNT;

end:
	spin_unlock_irqrestore(&priv->tx_lock, flags);
}

static int nb8800_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct nb8800_priv *priv = netdev_priv(dev);
	struct nb8800_tx_desc *txd;
	struct nb8800_tx_buf *txb;
	struct nb8800_dma_desc *dma;
	dma_addr_t dma_addr;
	unsigned int dma_len;
	int align;
	int next;

	if (atomic_read(&priv->tx_free) <= NB8800_DESC_LOW) {
		netif_stop_queue(dev);
		return NETDEV_TX_BUSY;
	}

	align = (8 - (uintptr_t)skb->data) & 7;

	dma_len = skb->len - align;
	dma_addr = dma_map_single(&dev->dev, skb->data + align,
				  dma_len, DMA_TO_DEVICE);

	if (dma_mapping_error(&dev->dev, dma_addr)) {
		netdev_err(dev, "tx dma mapping error\n");
		kfree_skb(skb);
		dev->stats.tx_dropped++;
		return NETDEV_TX_OK;
	}

	if (atomic_dec_return(&priv->tx_free) <= NB8800_DESC_LOW)
		netif_stop_queue(dev);

	next = priv->tx_next;
	txb = &priv->tx_bufs[next];
	txd = &priv->tx_descs[next];
	dma = &txd->desc[0];

	if (align) {
		memcpy(txd->buf, skb->data, align);

		dma->s_addr =
			txb->dma_desc + offsetof(struct nb8800_tx_desc, buf);
		dma->n_addr = txb->dma_desc + sizeof(txd->desc[0]);
		dma->config = DESC_BTS(2) | DESC_DS | align;

		dma++;
	}

	dma->s_addr = dma_addr;
	dma->config = DESC_BTS(2) | DESC_DS | DESC_EOF | DESC_EOC | dma_len |
		priv->tx_dma_config;

	txb->dma_addr = dma_addr;
	txb->dma_len = dma_len;

	netdev_sent_queue(dev, skb->len);

	smp_wmb();
	txb->skb = skb;

	if (!skb->xmit_more)
		nb8800_tx_dma_start(dev);

	priv->tx_next = (next + 1) % TX_DESC_COUNT;

	return NETDEV_TX_OK;
}

static void nb8800_tx_error(struct net_device *dev, u32 report)
{
	if (report & TX_LATE_COLLISION)
		dev->stats.collisions++;

	if (report & TX_PACKET_DROPPED)
		dev->stats.tx_dropped++;

	if (report & TX_FIFO_UNDERRUN)
		dev->stats.tx_fifo_errors++;

	dev->stats.tx_errors++;
}

static void nb8800_tx_done(struct net_device *dev)
{
	struct nb8800_priv *priv = netdev_priv(dev);
	int done = priv->tx_done;
	unsigned int packets = 0;
	unsigned int len = 0;

	nb8800_tx_dma_start(dev);

	for (;;) {
		struct nb8800_tx_desc *txd = &priv->tx_descs[done];
		struct nb8800_tx_buf *txb = &priv->tx_bufs[done];
		struct sk_buff *skb;

		if (!txd->report)
			break;

		skb = txb->skb;
		len += skb->len;

		dma_unmap_single(&dev->dev, txb->dma_addr, txb->dma_len,
				 DMA_TO_DEVICE);

		if (IS_TX_ERROR(txd->report)) {
			nb8800_tx_error(dev, txd->report);
			dev_kfree_skb_irq(skb);
		} else {
			dev_consume_skb_irq(skb);
		}

		dev->stats.tx_packets++;
		dev->stats.tx_bytes += TX_BYTES_TRANSFERRED(txd->report);
		dev->stats.collisions += TX_EARLY_COLLISIONS(txd->report);

		txb->skb = NULL;
		txd->report = 0;

		done = (done + 1) % TX_DESC_COUNT;
		packets++;
	}

	if (packets) {
		smp_mb__before_atomic();
		atomic_add(packets, &priv->tx_free);
		netdev_completed_queue(dev, packets, len);
		netif_wake_queue(dev);
		priv->tx_done = done;
	}
}

static irqreturn_t nb8800_irq(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct nb8800_priv *priv = netdev_priv(dev);
	u32 val;

	/* tx interrupt */
	val = nb8800_readl(priv, NB8800_TXC_SR);
	if (val) {
		nb8800_writel(priv, NB8800_TXC_SR, val);

		if (likely(val & (TSR_TI | TSR_DI)))
			nb8800_tx_done(dev);

		if (unlikely(val & TSR_DE))
			netdev_err(dev, "TX DMA error\n");

		if (unlikely(val & TSR_TO))
			netdev_err(dev, "TX Status FIFO overflow\n");
	}

	/* rx interrupt */
	val = nb8800_readl(priv, NB8800_RXC_SR);
	if (val) {
		nb8800_writel(priv, NB8800_RXC_SR, val);

		if (likely(val & (RSR_RI | RSR_DI))) {
			nb8800_writel(priv, NB8800_RX_ITR, priv->rx_itr_poll);
			napi_schedule_irqoff(&priv->napi);
		}

		if (unlikely(val & RSR_DE))
			netdev_err(dev, "RX DMA error\n");

		if (unlikely(val & RSR_RO)) {
			int i;

			netdev_err(dev, "RX Status FIFO overflow\n");

			for (i = 0; i < 4; i++)
				nb8800_readl(priv, NB8800_RX_FIFO_SR);
		}
	}

	return IRQ_HANDLED;
}

static void nb8800_mac_config(struct net_device *dev)
{
	struct nb8800_priv *priv = netdev_priv(dev);
	bool gigabit = priv->speed == SPEED_1000;
	u32 mac_mode_mask = RGMII_MODE | HALF_DUPLEX | GMAC_MODE;
	u32 mac_mode = 0;
	u32 slot_time;
	u32 phy_clk;
	u32 ict;

	if (!priv->duplex)
		mac_mode |= HALF_DUPLEX;

	if (gigabit) {
		if (priv->phy_mode == PHY_INTERFACE_MODE_RGMII)
			mac_mode |= RGMII_MODE;

		mac_mode |= GMAC_MODE;
		phy_clk = 125000000;

		/* Should be 512 but register is only 8 bits */
		slot_time = 255;
	} else {
		phy_clk = 25000000;
		slot_time = 128;
	}

	ict = DIV_ROUND_UP(phy_clk, clk_get_rate(priv->clk));

	nb8800_writeb(priv, NB8800_IC_THRESHOLD, ict);
	nb8800_writeb(priv, NB8800_SLOT_TIME, slot_time);
	nb8800_maskb(priv, NB8800_MAC_MODE, mac_mode_mask, mac_mode);
}

static void nb8800_link_reconfigure(struct net_device *dev)
{
	struct nb8800_priv *priv = netdev_priv(dev);
	struct phy_device *phydev = priv->phydev;
	int change = 0;

	if (phydev->link) {
		if (phydev->speed != priv->speed) {
			priv->speed = phydev->speed;
			change = 1;
		}

		if (phydev->duplex != priv->duplex) {
			priv->duplex = phydev->duplex;
			change = 1;
		}

		if (change)
			nb8800_mac_config(dev);
	}

	if (phydev->link != priv->link) {
		priv->link = phydev->link;
		change = 1;
	}

	if (change)
		phy_print_status(priv->phydev);
}

static void nb8800_update_mac_addr(struct net_device *dev)
{
	struct nb8800_priv *priv = netdev_priv(dev);
	int i;

	for (i = 0; i < ETH_ALEN; i++)
		nb8800_writeb(priv, NB8800_SRC_ADDR(i), dev->dev_addr[i]);

	for (i = 0; i < ETH_ALEN; i++)
		nb8800_writeb(priv, NB8800_UC_ADDR(i), dev->dev_addr[i]);
}

static int nb8800_set_mac_address(struct net_device *dev, void *addr)
{
	struct sockaddr *sock = addr;

	if (netif_running(dev))
		return -EBUSY;

	ether_addr_copy(dev->dev_addr, sock->sa_data);
	nb8800_update_mac_addr(dev);

	return 0;
}

static void nb8800_mc_init(struct net_device *dev, int val)
{
	struct nb8800_priv *priv = netdev_priv(dev);

	nb8800_writeb(priv, NB8800_MC_INIT, val);
	readb_poll_timeout_atomic(priv->base + NB8800_MC_INIT, val, !val,
				  1, 1000);
}

static void nb8800_set_rx_mode(struct net_device *dev)
{
	struct nb8800_priv *priv = netdev_priv(dev);
	struct netdev_hw_addr *ha;
	bool af_en;
	int i;

	if (dev->flags & (IFF_PROMISC | IFF_ALLMULTI))
		af_en = false;
	else
		af_en = true;

	nb8800_mac_af(dev, af_en);

	if (!af_en)
		return;

	nb8800_mc_init(dev, 0);

	netdev_for_each_mc_addr(ha, dev) {
		char *addr = ha->addr;

		for (i = 0; i < ETH_ALEN; i++)
			nb8800_writeb(priv, NB8800_MC_ADDR(i), addr[i]);

		nb8800_mc_init(dev, 0xff);
	}
}

#define RX_DESC_SIZE (RX_DESC_COUNT * sizeof(struct nb8800_rx_desc))
#define TX_DESC_SIZE (TX_DESC_COUNT * sizeof(struct nb8800_tx_desc))

static void nb8800_dma_free(struct net_device *dev)
{
	struct nb8800_priv *priv = netdev_priv(dev);
	int i;

	if (priv->rx_bufs) {
		for (i = 0; i < RX_DESC_COUNT; i++)
			if (priv->rx_bufs[i].page)
				put_page(priv->rx_bufs[i].page);

		kfree(priv->rx_bufs);
		priv->rx_bufs = NULL;
	}

	if (priv->tx_bufs) {
		for (i = 0; i < TX_DESC_COUNT; i++)
			kfree_skb(priv->tx_bufs[i].skb);

		kfree(priv->tx_bufs);
		priv->tx_bufs = NULL;
	}

	if (priv->rx_descs) {
		dma_free_coherent(dev->dev.parent, RX_DESC_SIZE, priv->rx_descs,
				  priv->rx_desc_dma);
		priv->rx_descs = NULL;
	}

	if (priv->tx_descs) {
		dma_free_coherent(dev->dev.parent, TX_DESC_SIZE, priv->tx_descs,
				  priv->tx_desc_dma);
		priv->tx_descs = NULL;
	}
}

static int nb8800_dma_init(struct net_device *dev)
{
	struct nb8800_priv *priv = netdev_priv(dev);
	struct nb8800_rx_desc *rx;
	struct nb8800_tx_desc *txd;
	int n_rx = RX_DESC_COUNT;
	int n_tx = TX_DESC_COUNT;
	int i;

	priv->rx_descs = dma_alloc_coherent(dev->dev.parent, RX_DESC_SIZE,
					    &priv->rx_desc_dma, GFP_KERNEL);
	if (!priv->rx_descs)
		goto err_out;

	priv->rx_bufs = kcalloc(n_rx, sizeof(*priv->rx_bufs), GFP_KERNEL);
	if (!priv->rx_bufs)
		goto err_out;

	for (i = 0; i < n_rx; i++) {
		dma_addr_t rx_dma;
		int err;

		rx = &priv->rx_descs[i];
		rx_dma = priv->rx_desc_dma + i * sizeof(*rx);
		rx->desc.n_addr = rx_dma + sizeof(*rx);
		rx->desc.r_addr =
			rx_dma + offsetof(struct nb8800_rx_desc, report);
		rx->report = 0;

		err = nb8800_alloc_rx(dev, i, false);
		if (err)
			goto err_out;
	}

	rx->desc.n_addr = priv->rx_desc_dma;
	rx->desc.config |= DESC_EOC;

	priv->rx_eoc = RX_DESC_COUNT - 1;

	priv->tx_descs = dma_alloc_coherent(dev->dev.parent, TX_DESC_SIZE,
					    &priv->tx_desc_dma, GFP_KERNEL);
	if (!priv->tx_descs)
		goto err_out;

	priv->tx_bufs = kcalloc(n_tx, sizeof(*priv->tx_bufs), GFP_KERNEL);
	if (!priv->tx_bufs)
		goto err_out;

	for (i = 0; i < n_tx; i++) {
		dma_addr_t tx_dma;
		dma_addr_t r_dma;

		txd = &priv->tx_descs[i];
		tx_dma = priv->tx_desc_dma + i * sizeof(*txd);
		r_dma = tx_dma + offsetof(struct nb8800_tx_desc, report);

		txd->desc[0].s_addr =
			tx_dma + offsetof(struct nb8800_tx_desc, buf);
		txd->desc[0].r_addr = r_dma;

		txd->desc[1].n_addr = tx_dma + sizeof(*txd);
		txd->desc[1].r_addr = r_dma;

		txd->report = 0;

		priv->tx_bufs[i].dma_desc = tx_dma;
	}

	txd->desc[1].n_addr = priv->tx_desc_dma;

	priv->tx_next = 0;
	priv->tx_queue = 0;
	priv->tx_done = 0;
	atomic_set(&priv->tx_free, TX_DESC_COUNT);

	nb8800_writel(priv, NB8800_TX_DESC_ADDR, priv->tx_desc_dma);
	nb8800_writel(priv, NB8800_RX_DESC_ADDR, priv->rx_desc_dma);

	wmb();		/* ensure all setup is written before starting */

	return 0;

err_out:
	nb8800_dma_free(dev);

	return -ENOMEM;
}

static int nb8800_open(struct net_device *dev)
{
	struct nb8800_priv *priv = netdev_priv(dev);
	int err;

	nb8800_writel(priv, NB8800_RXC_SR, 0xf);
	nb8800_writel(priv, NB8800_TXC_SR, 0xf);

	err = nb8800_dma_init(dev);
	if (err)
		return err;

	err = request_irq(dev->irq, nb8800_irq, 0, dev_name(&dev->dev), dev);
	if (err)
		goto err_free_dma;

	nb8800_mac_rx(dev, true);
	nb8800_mac_tx(dev, true);

	priv->phydev = of_phy_connect(dev, priv->phy_node,
				      nb8800_link_reconfigure, 0,
				      priv->phy_mode);
	if (!priv->phydev)
		goto err_free_irq;

	netdev_reset_queue(dev);
	napi_enable(&priv->napi);
	netif_start_queue(dev);

	nb8800_start_rx(dev);
	phy_start(priv->phydev);

	return 0;

err_free_irq:
	free_irq(dev->irq, dev);
err_free_dma:
	nb8800_dma_free(dev);

	return err;
}

static int nb8800_stop(struct net_device *dev)
{
	struct nb8800_priv *priv = netdev_priv(dev);

	netif_stop_queue(dev);
	napi_disable(&priv->napi);

	nb8800_stop_rx(dev);

	nb8800_mac_rx(dev, false);
	nb8800_mac_tx(dev, false);

	free_irq(dev->irq, dev);

	phy_stop(priv->phydev);
	phy_disconnect(priv->phydev);

	nb8800_dma_free(dev);

	return 0;
}

static int nb8800_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
	struct nb8800_priv *priv = netdev_priv(dev);

	return phy_mii_ioctl(priv->phydev, rq, cmd);
}

static const struct net_device_ops nb8800_netdev_ops = {
	.ndo_open		= nb8800_open,
	.ndo_stop		= nb8800_stop,
	.ndo_start_xmit		= nb8800_xmit,
	.ndo_set_mac_address	= nb8800_set_mac_address,
	.ndo_set_rx_mode	= nb8800_set_rx_mode,
	.ndo_do_ioctl		= nb8800_ioctl,
	.ndo_change_mtu		= eth_change_mtu,
	.ndo_validate_addr	= eth_validate_addr,
};

static int nb8800_get_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct nb8800_priv *priv = netdev_priv(dev);

	return phy_ethtool_gset(priv->phydev, cmd);
}

static int nb8800_set_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct nb8800_priv *priv = netdev_priv(dev);

	return phy_ethtool_sset(priv->phydev, cmd);
}

static int nb8800_nway_reset(struct net_device *dev)
{
	struct nb8800_priv *priv = netdev_priv(dev);

	return genphy_restart_aneg(priv->phydev);
}

static const struct ethtool_ops nb8800_ethtool_ops = {
	.get_settings		= nb8800_get_settings,
	.set_settings		= nb8800_set_settings,
	.nway_reset		= nb8800_nway_reset,
	.get_link		= ethtool_op_get_link,
};

static int nb8800_hw_init(struct net_device *dev)
{
	struct nb8800_priv *priv = netdev_priv(dev);
	u32 val;

	val = TX_RETRY_EN | TX_PAD_EN | TX_APPEND_FCS;
	nb8800_writeb(priv, NB8800_TX_CTL1, val);

	/* collision retry count */
	nb8800_writeb(priv, NB8800_TX_CTL2, 5);

	val = RX_PAD_STRIP | RX_AF_EN | RX_RUNT;
	nb8800_writeb(priv, NB8800_RX_CTL, val);

	nb8800_writeb(priv, NB8800_RANDOM_SEED, 0x08);

	/* TX single deferral params */
	nb8800_writeb(priv, NB8800_TX_SDP, 0xc);

	/* The following three threshold values have been
	 * experimentally determined for good results.
	 */

	/* RX/TX FIFO threshold for partial empty (64-bit entries) */
	nb8800_writeb(priv, NB8800_PE_THRESHOLD, 0);

	/* RX/TX FIFO threshold for partial full (64-bit entries) */
	nb8800_writeb(priv, NB8800_PF_THRESHOLD, 255);

	/* Buffer size for transmit (64-bit entries) */
	nb8800_writeb(priv, NB8800_TX_BUFSIZE, 64);

	/* configure TX DMA Channels */
	val = nb8800_readl(priv, NB8800_TXC_CR);
	val &= TCR_LE;
	val |= TCR_DM | TCR_RS | TCR_TFI(1) | TCR_BTS(2);
	nb8800_writel(priv, NB8800_TXC_CR, val);

	/* TX Interrupt Time Register */
	nb8800_writel(priv, NB8800_TX_ITR, 1);

	/* configure RX DMA Channels */
	val = nb8800_readl(priv, NB8800_RXC_CR);
	val &= RCR_LE;
	val |= RCR_DM | RCR_RS | RCR_RFI(7) | RCR_BTS(2);
	nb8800_writel(priv, NB8800_RXC_CR, val);

	/* RX Interrupt Time Register */
	nb8800_writel(priv, NB8800_RX_ITR, priv->rx_itr_irq);

	nb8800_mc_init(dev, 0);

	return 0;
}

static int nb8800_tangox_init(struct net_device *dev)
{
	struct nb8800_priv *priv = netdev_priv(dev);
	u32 pad_mode = PAD_MODE_MII;

	switch (priv->phy_mode) {
	case PHY_INTERFACE_MODE_MII:
	case PHY_INTERFACE_MODE_GMII:
		pad_mode = PAD_MODE_MII;
		break;

	case PHY_INTERFACE_MODE_RGMII:
		pad_mode = PAD_MODE_RGMII;
		break;

	case PHY_INTERFACE_MODE_RGMII_TXID:
		pad_mode = PAD_MODE_RGMII | PAD_MODE_GTX_CLK_DELAY;
		break;

	default:
		dev_err(dev->dev.parent, "unsupported phy mode %s\n",
			phy_modes(priv->phy_mode));
		return -EINVAL;
	}

	nb8800_writeb(priv, NB8800_TANGOX_PAD_MODE, pad_mode);

	return 0;
}

static int nb8800_tangox_reset(struct net_device *dev)
{
	struct nb8800_priv *priv = netdev_priv(dev);
	int clk_div;

	nb8800_writeb(priv, NB8800_TANGOX_RESET, 0);
	usleep_range(1000, 10000);
	nb8800_writeb(priv, NB8800_TANGOX_RESET, 1);

	wmb();		/* ensure reset is cleared before proceeding */

	clk_div = DIV_ROUND_UP(clk_get_rate(priv->clk), 2 * MAX_MDC_CLOCK);
	nb8800_writew(priv, NB8800_TANGOX_MDIO_CLKDIV, clk_div);

	return 0;
}

static const struct nb8800_ops nb8800_tangox_ops = {
	.init	= nb8800_tangox_init,
	.reset	= nb8800_tangox_reset,
};

/* On tango4 interrupt on DMA completion works and gives better
 * performance despite generating more RX interrupts. */
static int nb8800_tango4_init(struct net_device *dev)
{
	struct nb8800_priv *priv = netdev_priv(dev);
	u32 val;
	int err;

	err = nb8800_tangox_init(dev);
	if (err)
		return err;

	val = nb8800_readl(priv, NB8800_RXC_CR);
	val &= ~RCR_RFI(7);
	val |= RCR_DIE;
	nb8800_writel(priv, NB8800_RXC_CR, val);

	priv->rx_dma_config |= DESC_ID;

	val = nb8800_readl(priv, NB8800_TXC_CR);
	val &= ~TCR_TFI(7);
	val |= TCR_DIE;
	nb8800_writel(priv, NB8800_TXC_CR, val);

	priv->tx_dma_config |= DESC_ID;

	return 0;
}

static const struct nb8800_ops nb8800_tango4_ops = {
	.init	= nb8800_tango4_init,
	.reset	= nb8800_tangox_reset,
};

static const struct of_device_id nb8800_dt_ids[] = {
	{
		.compatible = "aurora,nb8800",
	},
	{
		.compatible = "sigma,smp8642-ethernet",
		.data = &nb8800_tangox_ops,
	},
	{
		.compatible = "sigma,smp8734-ethernet",
		.data = &nb8800_tango4_ops,
	},
	{ }
};

static int nb8800_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	const struct nb8800_ops *ops = NULL;
	struct nb8800_priv *priv;
	struct resource *res;
	struct net_device *dev;
	struct mii_bus *bus;
	const unsigned char *mac;
	void __iomem *base;
	int irq;
	int ret;

	match = of_match_device(nb8800_dt_ids, &pdev->dev);
	if (match)
		ops = match->data;

	irq = platform_get_irq(pdev, 0);
	if (irq <= 0) {
		dev_err(&pdev->dev, "No IRQ\n");
		return -EINVAL;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	dev_dbg(&pdev->dev, "AU-NB8800 Ethernet at %pa\n", &res->start);

	dev = alloc_etherdev(sizeof(*priv));
	if (!dev)
		return -ENOMEM;

	platform_set_drvdata(pdev, dev);
	SET_NETDEV_DEV(dev, &pdev->dev);

	priv = netdev_priv(dev);
	priv->base = base;

	priv->phy_mode = of_get_phy_mode(pdev->dev.of_node);
	if (priv->phy_mode < 0)
		priv->phy_mode = PHY_INTERFACE_MODE_RGMII;

	priv->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(priv->clk)) {
		dev_err(&pdev->dev, "failed to get clock\n");
		ret = PTR_ERR(priv->clk);
		goto err_free_dev;
	}

	ret = clk_prepare_enable(priv->clk);
	if (ret)
		goto err_free_dev;

	/* The rx interrupt can fire before the DMA has completed
	 * unless a small delay is added.  50 us is hopefully enough. */
	priv->rx_itr_irq = clk_get_rate(priv->clk) / 20000;

	/* In NAPI poll mode we want to disable interrupts, but the
	 * hardware does not permit this.  Delay 10 ms instead. */
	priv->rx_itr_poll = clk_get_rate(priv->clk) / 100;

	priv->rx_dma_config = RX_BUF_SIZE | DESC_BTS(2) | DESC_DS | DESC_EOF;
	spin_lock_init(&priv->tx_lock);

	if (ops && ops->reset) {
		ret = ops->reset(dev);
		if (ret)
			goto err_free_dev;
	}

	bus = devm_mdiobus_alloc(&pdev->dev);
	if (!bus) {
		ret = -ENOMEM;
		goto err_disable_clk;
	}

	bus->name = "nb8800-mii";
	bus->read = nb8800_mdio_read;
	bus->write = nb8800_mdio_write;
	bus->parent = &pdev->dev;
	snprintf(bus->id, MII_BUS_ID_SIZE, "%lx.nb8800-mii",
		 (unsigned long)res->start);
	bus->priv = priv;

	ret = of_mdiobus_register(bus, pdev->dev.of_node);
	if (ret) {
		dev_err(&pdev->dev, "failed to register MII bus\n");
		goto err_disable_clk;
	}

	priv->phy_node = of_parse_phandle(pdev->dev.of_node, "phy-handle", 0);
	if (!priv->phy_node) {
		dev_err(&pdev->dev, "no PHY specified\n");
		ret = -ENODEV;
		goto err_free_bus;
	}

	priv->mii_bus = bus;

	ret = nb8800_hw_init(dev);
	if (ret)
		goto err_free_bus;

	if (ops && ops->init) {
		ret = ops->init(dev);
		if (ret)
			goto err_free_bus;
	}

	dev->netdev_ops = &nb8800_netdev_ops;
	dev->ethtool_ops = &nb8800_ethtool_ops;
	dev->flags |= IFF_MULTICAST;
	dev->irq = irq;

	mac = of_get_mac_address(pdev->dev.of_node);
	if (mac)
		ether_addr_copy(dev->dev_addr, mac);

	if (!is_valid_ether_addr(dev->dev_addr))
		eth_hw_addr_random(dev);

	nb8800_update_mac_addr(dev);

	netif_carrier_off(dev);

	ret = register_netdev(dev);
	if (ret) {
		netdev_err(dev, "failed to register netdev\n");
		goto err_free_dma;
	}

	netif_napi_add(dev, &priv->napi, nb8800_poll, NAPI_POLL_WEIGHT);

	netdev_info(dev, "MAC address %pM\n", dev->dev_addr);

	return 0;

err_free_dma:
	nb8800_dma_free(dev);
err_free_bus:
	mdiobus_unregister(bus);
err_disable_clk:
	clk_disable_unprepare(priv->clk);
err_free_dev:
	free_netdev(dev);

	return ret;
}

static int nb8800_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct nb8800_priv *priv = netdev_priv(ndev);

	unregister_netdev(ndev);

	mdiobus_unregister(priv->mii_bus);

	clk_disable_unprepare(priv->clk);

	nb8800_dma_free(ndev);
	free_netdev(ndev);

	return 0;
}

static struct platform_driver nb8800_driver = {
	.driver = {
		.name		= "nb8800",
		.of_match_table	= nb8800_dt_ids,
	},
	.probe	= nb8800_probe,
	.remove	= nb8800_remove,
};

module_platform_driver(nb8800_driver);

MODULE_DESCRIPTION("Aurora AU-NB8800 Ethernet driver");
MODULE_AUTHOR("Mans Rullgard <mans@mansr.com>");
MODULE_LICENSE("GPL");
