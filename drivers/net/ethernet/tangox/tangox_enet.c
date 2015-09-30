/*********************************************************************
 Copyright (C) 2001-2011
 Sigma Designs, Inc.

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/
/*
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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/etherdevice.h>
#include <linux/delay.h>
#include <linux/ethtool.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/of_net.h>
#include <linux/dma-mapping.h>
#include <linux/phy.h>
#include <linux/cache.h>
#include <linux/jiffies.h>
#include <asm/barrier.h>
#include <asm/io.h>

#include "tangox_enet.h"

static void enet_tx_reclaim(unsigned long data);

static inline u8 enet_readb(struct tangox_enet_priv *priv, int reg)
{
	return readb(priv->base + reg);
}

static inline u32 enet_readl(struct tangox_enet_priv *priv, int reg)
{
	return readl(priv->base + reg);
}

static inline void enet_writeb(struct tangox_enet_priv *priv, int reg, u8 val)
{
	writeb(val, priv->base + reg);
}

static inline void enet_writew(struct tangox_enet_priv *priv, int reg, u16 val)
{
	writew(val, priv->base + reg);
}

static inline void enet_writel(struct tangox_enet_priv *priv, int reg, u32 val)
{
	writel(val, priv->base + reg);
}

#define enet_set_bits(sz, priv, reg, bits) do {				\
		u32 __o = enet_read##sz(priv, reg);			\
		u32 __n = __o | (bits);					\
		if (__n != __o)						\
			enet_write##sz(priv, reg, __n);			\
	} while (0)

#define enet_clear_bits(sz, priv, reg, bits) do {			\
		u32 __o = enet_read##sz(priv, reg);			\
		u32 __n = __o & ~(bits);				\
		if (__n != __o)						\
			enet_write##sz(priv, reg, __n);			\
	} while (0)

#define MDIO_TIMEOUT	1000

static int enet_mdio_wait(struct mii_bus *bus)
{
	struct tangox_enet_priv *priv = bus->priv;
	int tmo = MDIO_TIMEOUT;

	while (--tmo) {
		if (!(enet_readl(priv, ENET_MDIO_CMD) & MDIO_CMD_GO))
			break;
		udelay(1);
	}

	return tmo;
}

static int enet_mdio_read(struct mii_bus *bus, int phy_id, int reg)
{
	struct tangox_enet_priv *priv = bus->priv;
	int val;

	if (!enet_mdio_wait(bus))
		return -ETIMEDOUT;

	val = MIIAR_ADDR(phy_id) | MIIAR_REG(reg);
	enet_writel(priv, ENET_MDIO_CMD, val);

	udelay(10);

	enet_writel(priv, ENET_MDIO_CMD, val | MDIO_CMD_GO);

	if (!enet_mdio_wait(bus))
		return -ETIMEDOUT;

	val = enet_readl(priv, ENET_MDIO_STS);
	if (val & MDIO_STS_ERR)
		return 0xffff;

	return val & 0xffff;
}

static int enet_mdio_write(struct mii_bus *bus, int phy_id, int reg, u16 val)
{
	struct tangox_enet_priv *priv = bus->priv;
	int tmp;

	if (!enet_mdio_wait(bus))
		return -ETIMEDOUT;

	tmp = MIIAR_DATA(val) | MIIAR_ADDR(phy_id) | MIIAR_REG(reg);
	enet_writel(priv, ENET_MDIO_CMD, tmp);

	udelay(10);

	enet_writel(priv, ENET_MDIO_CMD, tmp | MDIO_CMD_WR);

	udelay(10);

	enet_writel(priv, ENET_MDIO_CMD, tmp | MDIO_CMD_WR | MDIO_CMD_GO);

	if (!enet_mdio_wait(bus))
		return -ETIMEDOUT;

	return 0;
}

static void enet_mac_tx(struct net_device *dev, int enable)
{
	struct tangox_enet_priv *priv = netdev_priv(dev);

	while (enet_readl(priv, ENET_TXC_CR) & TCR_EN)
		cpu_relax();

	if (enable)
		enet_set_bits(b, priv, ENET_TX_CTL1, TX_EN);
	else
		enet_clear_bits(b, priv, ENET_TX_CTL1, TX_EN);
}

static void enet_mac_rx(struct net_device *dev, int enable)
{
	struct tangox_enet_priv *priv = netdev_priv(dev);

	if (enable)
		enet_set_bits(b, priv, ENET_RX_CTL, RX_EN);
	else
		enet_clear_bits(b, priv, ENET_RX_CTL, RX_EN);
}

static void enet_mac_af(struct net_device *dev, int enable)
{
	struct tangox_enet_priv *priv = netdev_priv(dev);

	if (enable)
		enet_set_bits(b, priv, ENET_RX_CTL, RX_AF_EN);
	else
		enet_clear_bits(b, priv, ENET_RX_CTL, RX_AF_EN);
}

static void enet_stop_rx(struct net_device *dev)
{
	struct tangox_enet_priv *priv = netdev_priv(dev);
	int i;

	for (i = 0; i < priv->rx_desc_count; i++)
		priv->rx_descs[i].config |= DESC_EOC;

	mb();

	while (enet_readl(priv, ENET_RXC_CR) & RCR_EN)
		udelay(1000);
}

static void enet_start_rx(struct net_device *dev)
{
	struct tangox_enet_priv *priv = netdev_priv(dev);

	enet_set_bits(l, priv, ENET_RXC_CR, RCR_EN);
}

static int enet_alloc_rx(struct net_device *dev, int i, int napi)
{
	struct tangox_enet_priv *priv = netdev_priv(dev);
	struct enet_desc *rx = &priv->rx_descs[i];
	struct rx_buf *buf = &priv->rx_bufs[i];
	int size = L1_CACHE_ALIGN(RX_BUF_SIZE);
	void *data;

	data = napi ? napi_alloc_frag(size) : netdev_alloc_frag(size);
	if (!data) {
		buf->page = NULL;
		rx->config = DESC_BTS(2) | DESC_EOF;
		return -ENOMEM;
	}

	buf->page = virt_to_head_page(data);
	buf->offset = data - page_address(buf->page);

	rx->s_addr = dma_map_page(&dev->dev, buf->page, buf->offset,
				  RX_BUF_SIZE, DMA_FROM_DEVICE);
	rx->config = RX_BUF_SIZE | DESC_BTS(2) | DESC_EOF;

	return 0;
}

static void enet_receive(struct net_device *dev, int i, int len)
{
	struct tangox_enet_priv *priv = netdev_priv(dev);
	struct enet_desc *rx = &priv->rx_descs[i];
	struct page *page = priv->rx_bufs[i].page;
	int offset = priv->rx_bufs[i].offset;
	void *data = page_address(page) + offset;
	dma_addr_t dma = rx->s_addr;
	struct sk_buff *skb;

	skb = napi_alloc_skb(&priv->napi, RX_COPYBREAK);
	if (!skb) {
		netdev_err(dev, "rx skb allocation failed\n");
		return;
	}

	if (len <= RX_COPYBREAK) {
		dma_sync_single_for_cpu(&dev->dev, dma, len, DMA_FROM_DEVICE);
		memcpy(skb_put(skb, len), data, len);
		dma_sync_single_for_device(&dev->dev, dma, len,
					   DMA_FROM_DEVICE);
	} else {
		dma_unmap_page(&dev->dev, dma, RX_BUF_SIZE, DMA_FROM_DEVICE);
		memcpy(skb_put(skb, 128), data, 128);
		skb_add_rx_frag(skb, skb_shinfo(skb)->nr_frags, page,
				offset + 128, len - 128, RX_BUF_SIZE);
		priv->rx_bufs[i].page = NULL;
	}

	skb->protocol = eth_type_trans(skb, dev);
	netif_receive_skb(skb);

	priv->stats.rx_packets++;
	priv->stats.rx_bytes += len;
}

static void enet_rx_error(struct net_device *dev, u32 report)
{
	struct tangox_enet_priv *priv = netdev_priv(dev);
	int len = RX_BYTES_TRANSFERRED(report);

	if (report & RX_FCS_ERR)
		priv->stats.rx_crc_errors++;

	if ((report & (RX_FRAME_LEN_ERROR | RX_LENGTH_ERR)) ||
	    (len > RX_BUF_SIZE))
		priv->stats.rx_length_errors++;

	priv->stats.rx_errors++;
}

static int enet_poll(struct napi_struct *napi, int budget)
{
	struct net_device *dev = napi->dev;
	struct tangox_enet_priv *priv = netdev_priv(dev);
	struct enet_desc *rx;
	int work = 0;
	int last = priv->rx_eoc;
	int next;

	while (work < budget) {
		struct rx_buf *rx_buf;
		u32 report;
		int len;

		next = (last + 1) & (priv->rx_desc_count - 1);

		rx_buf = &priv->rx_bufs[next];
		rx = &priv->rx_descs[next];
		report = rx->report;

		if (!report)
			break;

		if (IS_RX_ERROR(report)) {
			enet_rx_error(dev, report);
		} else if (likely(rx_buf->page)) {
			len = RX_BYTES_TRANSFERRED(report);
			enet_receive(dev, next, len);
		}

		rx->report = 0;
		if (!rx_buf->page)
			enet_alloc_rx(dev, next, 1);

		last = next;
		work++;
	}

	if (work) {
		priv->rx_descs[last].config |= DESC_EOC;
		wmb();
		priv->rx_descs[priv->rx_eoc].config &= ~DESC_EOC;
		priv->rx_eoc = last;
	}

	enet_start_rx(dev);

	if (work < budget)
		napi_complete(napi);

	enet_tx_reclaim((unsigned long)dev);

	return work;
}

static void enet_tx_dma_queue(struct net_device *dev, dma_addr_t data, int len,
			      int flags)
{
	struct tangox_enet_priv *priv = netdev_priv(dev);
	int next = priv->tx_next;
	struct enet_desc *tx = &priv->tx_descs[next];

	tx->s_addr = data;
	tx->config = DESC_BTS(2) | flags | len;
	tx->report = 0;

	priv->tx_next = (next + 1) & (priv->tx_desc_count - 1);
}

static void enet_tx_dma_start(struct net_device *dev, int new)
{
	struct tangox_enet_priv *priv = netdev_priv(dev);
	struct enet_desc *tx;
	struct tx_buf *tx_buf;
	u32 txc_cr;
	int next;

	next = xchg(&priv->tx_pending, -1);
	if (next < 0)
		next = new;
	if (next < 0)
		goto end;

	txc_cr = enet_readl(priv, ENET_TXC_CR) & 0xffff;
	if (txc_cr & TCR_EN)
		goto end;

	tx = &priv->tx_descs[next];
	tx_buf = &priv->tx_bufs[next];

	next = (next + tx_buf->frags) & (priv->tx_desc_count - 1);
	priv->tx_reclaim_next = next;

	enet_writel(priv, ENET_TX_DESC_ADDR, tx_buf->desc_dma);
	wmb();
	enet_writel(priv, ENET_TXC_CR, txc_cr | TCR_EN);

	if (!priv->tx_bufs[next].frags)
		next = -1;

end:
	priv->tx_pending = next;
}

static int enet_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct tangox_enet_priv *priv = netdev_priv(dev);
	struct tx_skb_data *skb_data;
	struct tx_buf *tx_buf;
	dma_addr_t dma_addr;
	unsigned int dma_len;
	int cpsz, next;
	int frags;

	if (atomic_read(&priv->tx_free) <= ENET_DESC_LOW) {
		netif_stop_queue(dev);
		return NETDEV_TX_BUSY;
	}

	cpsz = (8 - (u32)skb->data) & 7;

	frags = cpsz ? 2 : 1;
	atomic_sub(frags, &priv->tx_free);

	dma_len = skb->len - cpsz;
	dma_addr = dma_map_single(&dev->dev, skb->data + cpsz,
				  dma_len, DMA_TO_DEVICE);

	next = priv->tx_next;
	tx_buf = &priv->tx_bufs[next];

	if (cpsz) {
		dma_addr_t dma =
			tx_buf->desc_dma + offsetof(struct enet_desc, buf);
		memcpy(priv->tx_descs[next].buf, skb->data, cpsz);
		enet_tx_dma_queue(dev, dma, cpsz, 0);
	}

	enet_tx_dma_queue(dev, dma_addr, dma_len, DESC_EOF | DESC_EOC);
	netdev_sent_queue(dev, skb->len);

	tx_buf->skb = skb;
	tx_buf->frags = frags;

	skb_data = (struct tx_skb_data *)skb->cb;
	skb_data->dma_addr = dma_addr;
	skb_data->dma_len = dma_len;

	enet_tx_dma_start(dev, next);

	if (!skb->xmit_more && !timer_pending(&priv->tx_reclaim_timer))
		mod_timer(&priv->tx_reclaim_timer, jiffies + HZ / 20);

	return NETDEV_TX_OK;
}

static void enet_tx_reclaim(unsigned long data)
{
	struct net_device *dev = (struct net_device *)data;
	struct tangox_enet_priv *priv = netdev_priv(dev);
	int packets = 0, bytes = 0;
	int reclaimed = 0;
	int dirty, limit;

	dirty = xchg(&priv->tx_dirty, -1);
	if (dirty < 0)
		return;

	limit = priv->tx_reclaim_limit;
	if (dirty == limit)
		goto end;

	while (dirty != limit) {
		struct enet_desc *tx = &priv->tx_descs[dirty];
		struct tx_buf *tx_buf = &priv->tx_bufs[dirty];
		struct sk_buff *skb = tx_buf->skb;
		struct tx_skb_data *skb_data = (struct tx_skb_data *)skb->cb;
		int frags = tx_buf->frags;

		packets++;
		bytes += skb->len;

		dma_unmap_single(&dev->dev, skb_data->dma_addr,
				 skb_data->dma_len, DMA_TO_DEVICE);
		dev_kfree_skb(skb);

		tx->report = 0;
		tx_buf->skb = NULL;
		tx_buf->frags = 0;

		dirty = (dirty + frags) & (priv->tx_desc_count - 1);
		reclaimed += frags;
	}

	priv->stats.tx_packets += packets;
	priv->stats.tx_bytes += bytes;

	smp_mb__before_atomic();
	atomic_add(reclaimed, &priv->tx_free);

	netif_wake_queue(dev);

end:
	priv->tx_dirty = dirty;
}

static void enet_tx_done(struct net_device *dev)
{
	struct tangox_enet_priv *priv = netdev_priv(dev);
	struct tx_buf *tx_buf;
	int tx_mask = (priv->tx_desc_count - 1);
	int nr_dirty;

	tx_buf = &priv->tx_bufs[priv->tx_done];
	priv->tx_done = (priv->tx_done + tx_buf->frags) & tx_mask;

	netdev_completed_queue(dev, 1, tx_buf->skb->len);

	priv->tx_reclaim_limit = priv->tx_reclaim_next;

	enet_tx_dma_start(dev, -1);

	nr_dirty = (priv->tx_reclaim_limit - priv->tx_dirty) & tx_mask;
	if (nr_dirty >= ENET_DESC_RECLAIM)
		tasklet_schedule(&priv->tx_reclaim_tasklet);
}

static irqreturn_t enet_isr(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct tangox_enet_priv *priv = netdev_priv(dev);
	unsigned long val;

	/* tx interrupt */
	val = enet_readl(priv, ENET_TXC_SR);
	if (val) {
		enet_writel(priv, ENET_TXC_SR, 0xf);

		if (likely(val & TSR_TI))
			enet_tx_done(dev);

		if (unlikely(val & TSR_DE))
			netdev_err(dev, "TX DMA error\n");
		if (unlikely(val & TSR_TO))
			netdev_err(dev, "TX FIFO overflow\n");
	}

	/* rx interrupt */
	val = enet_readl(priv, ENET_RXC_SR);
	if (val) {
		enet_writel(priv, ENET_RXC_SR, 0xf);

		if (likely(val & (RSR_RI | RSR_DI | RSR_DE | RSR_RO)))
			napi_schedule(&priv->napi);

		if (unlikely(val & RSR_DE))
			netdev_err(dev, "RX DMA error\n");
		if (unlikely(val & RSR_RO)) {
			int i;
			netdev_err(dev, "RX Status FIFO overflow\n");
			for (i = 0; i < 4; i++)
				enet_readl(priv, ENET_RX_FIFO_SR);
		}
	}

	/* wake on lan */
	val = enet_readb(priv, ENET_WAKEUP);
	if (val == 1) {
		enet_writeb(priv, ENET_SLEEP_MODE, 0);
		enet_writeb(priv, ENET_WAKEUP, 0);
	}

	return IRQ_HANDLED;
}

static void enet_mac_config(struct net_device *dev)
{
	struct tangox_enet_priv *priv = netdev_priv(dev);

	if (priv->duplex)
		enet_clear_bits(b, priv, ENET_MAC_MODE, HALF_DUPLEX);
	else
		enet_set_bits(b, priv, ENET_MAC_MODE, HALF_DUPLEX);

	if (priv->speed == SPEED_1000) {
		enet_set_bits(b, priv, ENET_MAC_MODE, RGMII_MODE | GMAC_MODE);
		enet_writeb(priv, ENET_IC_THRESHOLD, 3);
		enet_writeb(priv, ENET_SLOT_TIME, 255);
	} else {
		enet_clear_bits(b, priv, ENET_MAC_MODE, RGMII_MODE | GMAC_MODE);
		enet_writeb(priv, ENET_IC_THRESHOLD, 1);
		enet_writeb(priv, ENET_SLOT_TIME, 127);
	}
}

static void enet_link_reconfigure(struct net_device *dev)
{
	struct tangox_enet_priv *priv = netdev_priv(dev);
	struct phy_device *phydev = priv->phydev;

	if (phydev->speed == priv->speed && phydev->duplex == priv->duplex &&
	    phydev->link == priv->link)
		return;

	if (phydev->link != priv->link || phydev->link)
		phy_print_status(priv->phydev);

	priv->speed = phydev->speed;
	priv->duplex = phydev->duplex;
	priv->link = phydev->link;

	if (priv->link)
		enet_mac_config(dev);
}

static void enet_update_mac_addr(struct net_device *dev)
{
	struct tangox_enet_priv *priv = netdev_priv(dev);

	enet_writeb(priv, ENET_MAC_ADDR1, dev->dev_addr[0]);
	enet_writeb(priv, ENET_MAC_ADDR2, dev->dev_addr[1]);
	enet_writeb(priv, ENET_MAC_ADDR3, dev->dev_addr[2]);
	enet_writeb(priv, ENET_MAC_ADDR4, dev->dev_addr[3]);
	enet_writeb(priv, ENET_MAC_ADDR5, dev->dev_addr[4]);
	enet_writeb(priv, ENET_MAC_ADDR6, dev->dev_addr[5]);

	enet_writeb(priv, ENET_UC_ADDR1, dev->dev_addr[0]);
	enet_writeb(priv, ENET_UC_ADDR2, dev->dev_addr[1]);
	enet_writeb(priv, ENET_UC_ADDR3, dev->dev_addr[2]);
	enet_writeb(priv, ENET_UC_ADDR4, dev->dev_addr[3]);
	enet_writeb(priv, ENET_UC_ADDR5, dev->dev_addr[4]);
	enet_writeb(priv, ENET_UC_ADDR6, dev->dev_addr[5]);
}

static int enet_set_mac_address(struct net_device *dev, void *addr)
{
	struct sockaddr *sock = addr;

	if (netif_running(dev))
		return -EBUSY;

	memcpy(dev->dev_addr, sock->sa_data, ETH_ALEN);
	enet_update_mac_addr(dev);

	return 0;
}

static void enet_mc_init(struct net_device *dev, int val)
{
	struct tangox_enet_priv *priv = netdev_priv(dev);

	enet_writeb(priv, ENET_MC_INIT, val);
	while (enet_readb(priv, ENET_MC_INIT))
		cpu_relax();
}

static void enet_set_rx_mode(struct net_device *dev)
{
	struct tangox_enet_priv *priv = netdev_priv(dev);
	struct netdev_hw_addr *ha;
	int af_en;

	if ((dev->flags & (IFF_PROMISC | IFF_ALLMULTI)) ||
	    netdev_mc_count(dev) > 64)
		af_en = 0;
	else
		af_en = 1;

	enet_mac_af(dev, af_en);

	if (!af_en)
		return;

	enet_mc_init(dev, 0);

	netdev_for_each_mc_addr(ha, dev) {
		char *addr = ha->addr;

		enet_writeb(priv, ENET_MC_ADDR1, addr[0]);
		enet_writeb(priv, ENET_MC_ADDR2, addr[1]);
		enet_writeb(priv, ENET_MC_ADDR3, addr[2]);
		enet_writeb(priv, ENET_MC_ADDR4, addr[3]);
		enet_writeb(priv, ENET_MC_ADDR5, addr[4]);
		enet_writeb(priv, ENET_MC_ADDR6, addr[5]);

		enet_mc_init(dev, 0xff);
	}
}

static void enet_dma_reinit(struct net_device *dev)
{
	struct tangox_enet_priv *priv = netdev_priv(dev);
	struct enet_desc *rx;
	int i;

	priv->tx_pending = -1;
	priv->tx_reclaim_limit = 0;
	priv->tx_dirty = 0;
	priv->tx_next = 0;
	priv->tx_done = 0;
	atomic_set(&priv->tx_free, priv->tx_desc_count);

	priv->rx_eoc = priv->rx_desc_count - 1;

	for (i = 0; i < priv->rx_desc_count - 1; i++) {
		rx = &priv->rx_descs[i];
		rx->report = 0;
		rx->config &= ~DESC_EOC;
	}

	rx = &priv->rx_descs[i];
	rx->report = 0;
	rx->config |= DESC_EOC;

	wmb();

	enet_writel(priv, ENET_TX_DESC_ADDR, priv->tx_desc_dma);
	enet_writel(priv, ENET_RX_DESC_ADDR, priv->rx_desc_dma);
}

static int enet_open(struct net_device *dev)
{
	struct tangox_enet_priv *priv = netdev_priv(dev);

	phy_resume(priv->phydev);
	phy_start(priv->phydev);

	enet_writel(priv, ENET_RXC_SR, 0xf);
	enet_writel(priv, ENET_TXC_SR, 0xf);

	enet_dma_reinit(dev);
	enet_start_rx(dev);

	enet_mac_rx(dev, 1);
	enet_mac_tx(dev, 1);

	mdelay(500);

	napi_enable(&priv->napi);
	netif_start_queue(dev);

	return 0;
}

static int enet_stop(struct net_device *dev)
{
	struct tangox_enet_priv *priv = netdev_priv(dev);

	netif_stop_queue(dev);
	napi_disable(&priv->napi);

	enet_stop_rx(dev);

	enet_mac_rx(dev, 0);
	enet_mac_tx(dev, 0);

	phy_stop(priv->phydev);
	phy_suspend(priv->phydev);

	return 0;
}

static struct net_device_stats *enet_get_stats(struct net_device *dev)
{
	struct tangox_enet_priv *priv = netdev_priv(dev);

	return &priv->stats;
}

static int enet_get_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct tangox_enet_priv *priv = netdev_priv(dev);

	return phy_ethtool_gset(priv->phydev, cmd);
}

static int enet_set_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct tangox_enet_priv *priv = netdev_priv(dev);

	return phy_ethtool_sset(priv->phydev, cmd);
}

static int enet_nway_reset(struct net_device *dev)
{
	struct tangox_enet_priv *priv = netdev_priv(dev);

	return genphy_restart_aneg(priv->phydev);
}

static u32 enet_get_link(struct net_device *dev)
{
	struct tangox_enet_priv *priv = netdev_priv(dev);

	phy_read_status(priv->phydev);

	return priv->phydev->link;
}

static struct ethtool_ops enet_ethtool_ops = {
	.get_settings		= enet_get_settings,
	.set_settings		= enet_set_settings,
	.nway_reset		= enet_nway_reset,
	.get_link		= enet_get_link,
};

static int enet_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
	struct tangox_enet_priv *priv = netdev_priv(dev);

	return phy_mii_ioctl(priv->phydev, rq, cmd);
}

static int enet_dma_init(struct net_device *dev)
{
	struct tangox_enet_priv *priv = netdev_priv(dev);
	int n_rx = priv->rx_desc_count;
	int n_tx = priv->tx_desc_count;
	int i;

	priv->rx_descs = dmam_alloc_coherent(dev->dev.parent,
					     n_rx * sizeof(*priv->rx_descs),
					     &priv->rx_desc_dma, GFP_KERNEL);
	if (!priv->rx_descs)
		return -ENOMEM;

	priv->rx_bufs = devm_kcalloc(dev->dev.parent, n_rx,
				     sizeof(*priv->rx_bufs), GFP_KERNEL);
	if (!priv->rx_bufs)
		return -ENOMEM;

	for (i = 0; i < n_rx; i++) {
		struct enet_desc *rx = &priv->rx_descs[i];
		dma_addr_t rx_dma;
		int err;

		rx_dma = priv->rx_desc_dma + i * sizeof(struct enet_desc);
		rx->n_addr = rx_dma + sizeof(struct enet_desc);
		rx->r_addr = rx_dma + offsetof(struct enet_desc, report);

		err = enet_alloc_rx(dev, i, 0);
		if (err)
			return err;
	}

	priv->rx_descs[n_rx - 1].n_addr = priv->rx_desc_dma;

	priv->tx_descs = dmam_alloc_coherent(dev->dev.parent,
					     n_tx * sizeof(*priv->tx_descs),
					     &priv->tx_desc_dma, GFP_KERNEL);
	if (!priv->tx_descs)
		return -ENOMEM;

	priv->tx_bufs = devm_kcalloc(dev->dev.parent, n_tx,
				     sizeof(*priv->tx_bufs), GFP_KERNEL);
	if (!priv->tx_bufs)
		return -ENOMEM;

	for (i = 0; i < n_tx; i++) {
		struct enet_desc *tx = &priv->tx_descs[i];
		dma_addr_t tx_dma;

		tx_dma = priv->tx_desc_dma + i * sizeof(struct enet_desc);
		tx->n_addr = tx_dma + sizeof(struct enet_desc);
		tx->r_addr = tx_dma + offsetof(struct enet_desc, report);

		priv->tx_bufs[i].desc_dma = tx_dma;
	}

	priv->tx_descs[n_tx - 1].n_addr = priv->tx_desc_dma;

	enet_dma_reinit(dev);

	return 0;
}

static void enet_dma_free(struct net_device *dev)
{
	struct tangox_enet_priv *priv = netdev_priv(dev);
	int i;

	if (priv->rx_bufs)
		for (i = 0; i < priv->rx_desc_count; i++)
			if (!priv->rx_bufs[i].page)
				put_page(priv->rx_bufs[i].page);

	if (priv->tx_bufs)
		for (i = 0; i < priv->tx_desc_count; i++)
			kfree_skb(priv->tx_bufs[i].skb);
}

static void enet_hw_reset(struct net_device *dev)
{
	struct tangox_enet_priv *priv = netdev_priv(dev);

	/* software reset IP */
	enet_writeb(priv, ENET_SW_RESET, 0);
	wmb();
	udelay(10);
	enet_writeb(priv, ENET_SW_RESET, 1);
	wmb();
}

static int enet_hw_init(struct net_device *dev)
{
	struct tangox_enet_priv *priv = netdev_priv(dev);
	unsigned int val = 0;
	int clkdiv, itrmul;

	clkdiv = priv->gigabit ? 125000000 : 25000000;
	itrmul = clk_get_rate(priv->sys_clk) / clkdiv + 2;

	val = enet_readb(priv, ENET_PAD_MODE) & 0x78;
	if (priv->phydev->supported & PHY_1000BT_FEATURES)
		val |= 1;
	enet_writeb(priv, ENET_PAD_MODE, val);

	enet_writeb(priv, ENET_RANDOM_SEED, 0x08);

	/* TX single deferral params */
	enet_writeb(priv, ENET_TX_SDP, 0xc);

	/* Threshold for partial full */
	enet_writeb(priv, ENET_PF_THRESHOLD, 0xff);

	/* Pause Quanta */
	enet_writeb(priv, ENET_PQ1, 0xff);
	enet_writeb(priv, ENET_PQ2, 0xff);

	/* configure TX DMA Channels */
	val = TCR_DM | TCR_RS | TCR_LE | TCR_TFI(TFI) | TCR_BTS(2);
	enet_writel(priv, ENET_TXC_CR, val);

	/* TX Interrupt Time Register */
	val = TFI * TX_BUF_SIZE * itrmul;
	enet_writel(priv, ENET_TX_ITR, val);

	/* configure RX DMA Channels */
	val = RCR_DM | RCR_RS | RCR_LE | RCR_RFI(RFI) | RCR_DIE | RCR_BTS(2) |
		RCR_FI;
	enet_writel(priv, ENET_RXC_CR, val);

	/* RX Interrupt Time Register */
	val = RFI * RX_BUF_SIZE * itrmul;
	enet_writel(priv, ENET_RX_ITR, val);

	val = TX_RETRY_EN | TX_PAD_EN | TX_APPEND_FCS;
	enet_writeb(priv, ENET_TX_CTL1, val);

	/* collision retry count */
	enet_writeb(priv, ENET_TX_CTL2, 5);

	val = RX_PAD_STRIP | RX_PAUSE_EN | RX_AF_EN | RX_RUNT;
	enet_writeb(priv, ENET_RX_CTL, val);

	enet_mc_init(dev, 0);

	enet_writeb(priv, ENET_TX_BUFSIZE, 0xff);

	return 0;
}

static const struct net_device_ops tangox_netdev_ops = {
	.ndo_open		= enet_open,
	.ndo_stop		= enet_stop,
	.ndo_start_xmit		= enet_xmit,
	.ndo_get_stats		= enet_get_stats,
	.ndo_set_mac_address	= enet_set_mac_address,
	.ndo_set_rx_mode	= enet_set_rx_mode,
	.ndo_do_ioctl		= enet_ioctl,
	.ndo_change_mtu		= eth_change_mtu,
	.ndo_validate_addr	= eth_validate_addr,
};

static int tangox_enet_probe(struct platform_device *pdev)
{
	struct tangox_enet_priv *priv;
	struct resource *res;
	struct net_device *dev;
	struct mii_bus *bus;
	struct phy_device *phydev;
	const unsigned char *mac;
	void __iomem *base;
	int phy_mode;
	int clk_div;
	u32 speed;
	int irq;
	int ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "No MMIO base\n");
		return -EINVAL;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq <= 0) {
		dev_err(&pdev->dev, "No IRQ\n");
		return -EINVAL;
	}

	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	dev_info(&pdev->dev, "SMP86xx internal Ethernet at 0x%x\n", res->start);

	dev = alloc_etherdev(sizeof(*priv));
	if (!dev)
		return -ENOMEM;

	platform_set_drvdata(pdev, dev);
	SET_NETDEV_DEV(dev, &pdev->dev);

	priv = netdev_priv(dev);
	priv->base = base;
	priv->rx_desc_count = DEF_RX_DESC_COUNT;
	priv->tx_desc_count = DEF_TX_DESC_COUNT;

	if (!of_property_read_u32(pdev->dev.of_node, "max-speed", &speed))
		priv->gigabit = speed >= 1000;

	phy_mode = of_get_phy_mode(pdev->dev.of_node);
	if (phy_mode < 0)
		phy_mode = PHY_INTERFACE_MODE_RGMII;

	if (priv->gigabit) {
		priv->rx_desc_count *= 2;
		priv->tx_desc_count *= 2;
	}

	priv->sys_clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(priv->sys_clk)) {
		dev_err(&pdev->dev, "failed to get sys_clk\n");
		ret = PTR_ERR(priv->sys_clk);
		goto err_free_dev;
	}

	enet_hw_reset(dev);

	clk_div = DIV_ROUND_UP(clk_get_rate(priv->sys_clk), 2 * MAX_MDC_CLOCK);
	enet_writew(priv, ENET_MDIO_CLKDIV, clk_div);

	bus = devm_mdiobus_alloc(&pdev->dev);
	if (!bus) {
		ret = -ENOMEM;
		goto err_free_dev;
	}

	bus->name = "tangox-mii";
	bus->read = enet_mdio_read;
	bus->write = enet_mdio_write;
	bus->parent = &pdev->dev;
	snprintf(bus->id, MII_BUS_ID_SIZE, "%s-mii", pdev->name);
	bus->priv = priv;

	ret = mdiobus_register(bus);
	if (ret) {
		dev_err(&pdev->dev, "failed to register MII bus\n");
		goto err_free_dev;
	}

	phydev = phy_find_first(bus);
	if (!phydev || phy_read(phydev, MII_BMSR) <= 0) {
		dev_err(&pdev->dev, "no PHY detected\n");
		ret = -ENODEV;
		goto err_free_bus;
	}

	priv->mii_bus = bus;
	priv->phydev = phydev;

	phydev->irq = PHY_POLL;

	ret = phy_connect_direct(dev, phydev, enet_link_reconfigure, phy_mode);
	if (ret)
		goto err_free_bus;

	dev_info(&pdev->dev, "PHY: found %s at 0x%x\n",
		 phydev->drv->name, phydev->addr);

	ret = enet_hw_init(dev);
	if (ret)
		goto err_detach_phy;

	ret = enet_dma_init(dev);
	if (ret)
		goto err_detach_phy;

	ret = request_irq(irq, enet_isr, 0, dev_name(&pdev->dev), dev);
	if (ret)
		goto err_free_dma;

	dev->netdev_ops = &tangox_netdev_ops;
	dev->ethtool_ops = &enet_ethtool_ops;
	dev->tx_queue_len = priv->tx_desc_count;
	dev->flags |= IFF_MULTICAST;
	dev->irq = irq;

	mac = of_get_mac_address(pdev->dev.of_node);
	if (mac)
		memcpy(dev->dev_addr, mac, ETH_ALEN);

	if (!is_valid_ether_addr(dev->dev_addr))
		eth_hw_addr_random(dev);

	enet_update_mac_addr(dev);

	tasklet_init(&priv->tx_reclaim_tasklet, enet_tx_reclaim,
		     (unsigned long)dev);
	setup_timer(&priv->tx_reclaim_timer, enet_tx_reclaim,
		    (unsigned long)dev);

	ret = register_netdev(dev);
	if (ret) {
		netdev_err(dev, "failed to register netdev\n");
		goto err_free_dma;
	}

	netif_napi_add(dev, &priv->napi, enet_poll, NAPI_POLL_WEIGHT);

	netdev_info(dev, "MAC address %pM\n", dev->dev_addr);

	return 0;

err_free_dma:
	enet_dma_free(dev);
err_detach_phy:
	phy_detach(priv->phydev);
err_free_bus:
	mdiobus_unregister(bus);
err_free_dev:
	free_netdev(dev);

	return ret;
}

static int tangox_enet_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct tangox_enet_priv *priv = netdev_priv(ndev);

	unregister_netdev(ndev);
	free_irq(ndev->irq, ndev);

	phy_detach(priv->phydev);
	mdiobus_unregister(priv->mii_bus);

	enet_dma_free(ndev);
	free_netdev(ndev);

	return 0;
}

static struct of_device_id tangox_enet_dt_ids[] = {
	{ .compatible = "sigma,smp8640-emac" },
	{ }
};

static struct platform_driver tangox_enet_driver = {
	.driver = {
		.name		= "tangox-enet",
		.of_match_table	= tangox_enet_dt_ids,
	},
	.probe	= tangox_enet_probe,
	.remove	= tangox_enet_remove,
};

module_platform_driver(tangox_enet_driver);

MODULE_DESCRIPTION("SMP86xx internal Ethernet driver");
MODULE_LICENSE("GPL");
