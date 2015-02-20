/*
 * New driver for SMP863x builtin Ethernet mac
 *
 * This driver uses NAPI and generic linux MII support.
 *
 * Tx path limits the number of interrupt by reclaiming sent buffer in
 * a timer.  In case  the tx starts  to go  faster, it will  switch to
 * interrupt mode.
 *
 * Note that OOM condition is not handled correctly, and can leave the
 * rx path  in bad  shape. down/up the  interface should make  it work
 * again though. But anyway, it's not likely to happen.
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

#include <linux/config.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/etherdevice.h>
#include <linux/delay.h>
#include <linux/ethtool.h>
#include <linux/crc32.h>

#include "tango2_enet.h"

#define PFX	"tango2_enet: "

static struct net_device *gdev;
static int gphy_id = -1;

/*
 * mdio read/write callback, can run from userspace or timer
 */
static __inline int enet_mdio_read(struct net_device *dev, int phy_id,
				   int location)
{
	int val;

	while (enet_readl(ENET_MAC_MIIAR) & MIIAR_BUSY);
	enet_writel(ENET_MAC_MIIAR, MIIAR_ADDR(phy_id) | MIIAR_REG(location));
	udelay(1);
	while (enet_readl(ENET_MAC_MIIAR) & MIIAR_BUSY);
	val = enet_readl(ENET_MAC_MIIDR);

	return val;
}

static void enet_mdio_write(struct net_device *dev, int phy_id,
				     int location, int val)
{
	enet_writel(ENET_MAC_MIIDR, val);
	enet_writel(ENET_MAC_MIIAR,
		    MIIAR_ADDR(phy_id) | MIIAR_REG(location) | MIIAR_WRITE);
	udelay(1);
	while (enet_readl(ENET_MAC_MIIAR) & MIIAR_BUSY);
}

/*
 * enable/disable interrupt helpers
 * need proper locks since we will call them from any context
 */
static __inline void enet_disable_interrupts(struct tango2_enet_priv *priv,
					     int rx_only)
{
	unsigned long flags, val;

	spin_lock_irqsave(&priv->ier_lock, flags);
	if (rx_only) {
		val = enet_readl(ENET_DMA_IER);
		val &= ~IER_R;
		enet_writel(ENET_DMA_IER, val);
	} else
		enet_writel(ENET_DMA_IER, 0);
	spin_unlock_irqrestore(&priv->ier_lock, flags);
}

static __inline void enet_enable_interrupts(struct tango2_enet_priv *priv,
					    int rx_only)
{
	unsigned long flags, val;

	spin_lock_irqsave(&priv->ier_lock, flags);
	if (rx_only) {
		val = enet_readl(ENET_DMA_IER);
		val |= IER_R;
		enet_writel(ENET_DMA_IER, val);
	} else
		enet_writel(ENET_DMA_IER, IER_NIS | IER_R | IER_T);
	spin_unlock_irqrestore(&priv->ier_lock, flags);
}


/*
 * rx poll func, called by network core
 */
static int enet_poll(struct net_device *dev, int *budget)
{
	struct tango2_enet_priv *priv;
	int limit, received;

	priv = netdev_priv(dev);

	/* calculate how many rx packet we are allowed to fetch */
	limit = *budget;
	if (*budget > dev->quota)
		limit = dev->quota;
	received = 0;

	/* process no more than "limit" done rx */
	do {
		volatile struct enet_rx_desc *rx;
		struct sk_buff *skb;
		uint32_t rdes0_cache;
		unsigned int len;

		rx = &priv->rx_descs[priv->last_rx_desc];

		/* we  need  multiple  read  on this  volatile,  avoid
		 * memory access at each time */
		rdes0_cache = rx->rdes0;
		if (rdes0_cache & RDES0_OWN) {
			break;
		}

		if (limit <= 0)
			break;
		--limit;

		if (likely(skb = priv->rx_skbs[priv->last_rx_desc])) {

			/* we don't handle multipacket frame */
			if (!(rdes0_cache & RDES0_FIRST) ||
			    !(rdes0_cache & RDES0_LAST)) {
				/* we don't handle multipacket frame */
				priv->stats.rx_errors++;
				priv->stats.rx_length_errors++;
				goto rearm;
			}

			/* check for CRC */
			if (rdes0_cache & RDES0_CRC) {
				priv->stats.rx_errors++;
				priv->stats.rx_crc_errors++;
				goto rearm;
			}

			/* sanity check on len field */
			len = RDES0_FRAME_LEN(rdes0_cache);
			if (rdes0_cache & (RDES0_TOO_LONG | RDES0_TRUNC) ||
			    len > RX_BUF_SIZE) {
				priv->stats.rx_errors++;
				priv->stats.rx_length_errors++;
				goto rearm;
			}

			/* check remaining error */
			if (rdes0_cache & (RDES0_ERR_SUM | RDES0_COLLISION |
					   RDES0_WATCHDOG_TMOUT |
					   RDES0_MII_ERROR)) {
				priv->stats.rx_errors++;
				goto rearm;
			}

			/* ok, seems  valid, adjust skb  proto and len
			 * and give it to kernel */
			skb->dev = dev;
			skb_put(skb, len);
			skb->protocol = eth_type_trans(skb, dev);
			netif_receive_skb(skb);

			priv->stats.rx_packets++;
			priv->stats.rx_bytes += len;
			dev->last_rx = jiffies;
			priv->rx_skbs[priv->last_rx_desc] = NULL;
			/* we will realloc an skb for this slot */
		}

		skb = dev_alloc_skb(RX_BUF_SIZE + SKB_RESERVE_SIZE);
		if (unlikely(!skb))
			break;

		skb_reserve(skb, SKB_RESERVE_SIZE);
		rx->rdes2 = PHYSADDR(skb->data);
		dma_cache_inv((unsigned long)skb->data, RX_BUF_SIZE);
		priv->rx_skbs[priv->last_rx_desc] = skb;

rearm:
		/* rearm descriptor */
		wmb();
		rx->rdes0 = RDES0_OWN;
		priv->last_rx_desc++;
		priv->last_rx_desc &= (ENET_RX_DESC_COUNT - 1);
		received++;

	} while (1);

	dev->quota -= received;
	*budget -= received;

	if (limit <= 0) {
		/* breaked, but there is still work to do */
		return 1;
	}

        netif_rx_complete(dev);
	enet_enable_interrupts(priv, 1);

	return 0;
}

/*
 * tx reclaim func. Called by timer or tx done tasklet to reclaim sent
 * buffers.
 */
static void enet_tx_reclaim(unsigned long data)
{
	struct net_device *dev;
	struct tango2_enet_priv *priv;
	volatile struct enet_tx_desc *tx;

	dev = (struct net_device *)data;
	priv = netdev_priv(dev);

	spin_lock(&priv->tx_lock);

	while (priv->free_tx_desc_count < ENET_TX_DESC_COUNT) {
		uint32_t tdes0_cache;
		struct sk_buff *skb;

		tx = &priv->tx_descs[priv->dirty_tx_desc];

		tdes0_cache = tx->tdes0;
		if (tdes0_cache & TDES0_OWN)
			break;

		skb = priv->tx_skbs[priv->dirty_tx_desc];
		priv->stats.tx_packets++;

		/* check  for  transmission  errors and  update  stats
		 * accordingly */
		if (tdes0_cache & (TDES0_ERR_SUM | TDES0_CARRIER_LOST |
				   TDES0_NO_CARRIER | TDES0_LATE_COLLISION |
				   TDES0_EXC_COLLISION | TDES0_HEARTBEAT |
				   TDES0_EXC_DEFERAL | TDES0_UNDERFLOW)) {
			priv->stats.tx_errors++;
		} else {
			priv->stats.tx_bytes += skb->len;
		}

		dev_kfree_skb(skb);
		priv->tx_skbs[priv->dirty_tx_desc] = NULL;
		priv->dirty_tx_desc++;
		priv->dirty_tx_desc %= ENET_TX_DESC_COUNT;
		priv->free_tx_desc_count++;
	}

	if (priv->free_tx_desc_count != 0 && netif_queue_stopped(dev))
		netif_wake_queue(dev);

	spin_unlock(&priv->tx_lock);
}

/*
 * tx done timer callback, just call tx_done and reschedule timer
 */
static void enet_tx_reclaim_timer(unsigned long data)
{
	struct net_device *dev;
	struct tango2_enet_priv *priv;

	dev = (struct net_device *)data;
	priv = netdev_priv(dev);
	enet_tx_reclaim(data);

	priv->tx_reclaim_timer.expires = jiffies + TX_RECLAIM_TIMER_FREQ;
	add_timer(&priv->tx_reclaim_timer);
}


/*
 * tx request callback
 */
static int enet_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct tango2_enet_priv *priv;
	volatile struct enet_tx_desc *tx;
	unsigned long tdes1_cache;

	spin_lock(&priv->tx_lock);

	priv = netdev_priv(dev);
	tx = &priv->tx_descs[priv->next_tx_desc];

	/* make sure the next free tx desc is available */
	if (unlikely(priv->free_tx_desc_count == 0)) {
		/* no, this  should not happen since  queue is stopped
		 * before we run out of tx desc */
		printk(KERN_WARNING PFX "no free tx desc to handle pkt\n");
		netif_stop_queue(dev);
		spin_unlock(&priv->tx_lock);
		return NETDEV_TX_BUSY;
	}

	/* fill the tx desc with this skb address */
	tdes1_cache = (TDES1_FIRST | TDES1_LAST);
	if (priv->next_tx_desc == ENET_TX_DESC_COUNT - 1)
		tdes1_cache |= TDES1_TER;

	/* if we  start to  run low  on free tx  desc, then  enable tx
	 * interrupt to reclaim them faster */
	if (priv->free_tx_desc_count == ENET_TX_DESC_LOW) {
		tdes1_cache |= (TDES1_ENABLE_ISR);
	}
	tdes1_cache |= TDES1_TBS1(skb->len);

	tx->tdes1 = tdes1_cache;
	tx->tdes2 = PHYSADDR(skb->data);
	dma_cache_wback((unsigned long)skb->data, skb->len);

	/* keep a pointer to it for later and give it to dma  */
	priv->tx_skbs[priv->next_tx_desc] = skb;
	wmb();
	tx->tdes0 = TDES0_OWN;

	/* kick tx dma in case it was suspended */
	wmb();
	enet_writel(ENET_DMA_TPDR, 0x1);

	priv->next_tx_desc++;
	priv->next_tx_desc %= ENET_TX_DESC_COUNT;

	/* if next  tx descriptor is not  clean, then we  have to stop
	 * queue */
	if (unlikely(--priv->free_tx_desc_count == 0))
		netif_stop_queue(dev);

	spin_unlock(&priv->tx_lock);

	return NETDEV_TX_OK;
}

/*
 * our  irq handler, just  ack it  and schedule  the right  tasklet to
 * handle this
 */
static irqreturn_t enet_isr(int irq, void *dev_id)
{
	struct net_device *dev;
	struct tango2_enet_priv *priv;
	unsigned long val;

	dev = (struct net_device *)dev_id;
	priv = netdev_priv(dev);

	/* fetch status & ack them */
	val = enet_readl(ENET_DMA_SR);
	enet_writel(ENET_DMA_SR, val);

	/* handle interrupt */
	if (val & SR_NIS) {
		if (val & SR_T) {
			tasklet_schedule(&priv->tx_reclaim_tasklet);
		}

		if (val & SR_R) {
			if (netif_rx_schedule_prep(dev)) {
				/* disable rx interrupt */
				enet_disable_interrupts(priv, 1);

				/* ack  any  interrupt  that may  have
				 * arrived  between last ack  to avoid
				 * reentering */
				enet_writel(ENET_DMA_SR, SR_NIS | SR_R);
				__netif_rx_schedule(dev);
			}
		}
	}

        return IRQ_HANDLED;
}

/*
 * start/stop dma engine
 */
static __inline void enet_start_dma(struct tango2_enet_priv *priv)
{
	/* send start command to rx & tx dma */
	enet_writel(ENET_DMA_CR, CR_SF | CR_SR | CR_ST);
}

static __inline void enet_stop_dma(struct tango2_enet_priv *priv)
{
	unsigned long val;

	/* send stop command to rx & tx dma */
	enet_writel(ENET_DMA_CR, 0);

	/* wait for them to reach stopped state, should not be long */
	do {
		udelay(1);
		val = enet_readl(ENET_DMA_SR);
		if ((val & SR_TPS) && (val & SR_RPS))
			break;
	} while (1);
}

/*
 * reconfigure mac for new link state
 */
static void enet_link_reconfigure(struct net_device *dev)
{
	struct tango2_enet_priv *priv;
	unsigned long val;

	priv = netdev_priv(dev);

	if (dev->flags & IFF_UP)
		enet_stop_dma(priv);

	/* reflect duplex status in dma register */
	spin_lock(&priv->maccr_lock);
	val = enet_readl(ENET_MAC_MACCR);
	if (priv->mii.full_duplex)
		val |= MACCR_F;
	else
		val &= ~MACCR_F;
	enet_writel(ENET_MAC_MACCR, val);
	spin_unlock(&priv->maccr_lock);

	if (dev->flags & IFF_UP)
		enet_start_dma(priv);
}

/*
 * link check timer callback
 */
static void enet_link_check(unsigned long data)
{
	struct net_device *dev;
	struct tango2_enet_priv *priv;
	int ret;

	dev = (struct net_device *)data;
	priv = netdev_priv(dev);

	/* check for duplex change */
	spin_lock(&priv->mii_lock);
	ret = mii_check_media(&priv->mii, 1, 0);
	spin_unlock(&priv->mii_lock);

	if (ret)
		enet_link_reconfigure(dev);

	/* reschedule timer */
	priv->link_check_timer.expires = jiffies + LINK_CHECK_TIMER_FREQ;
	add_timer(&priv->link_check_timer);
}

/*
 * program given mac address in hw registers
 */
static int enet_set_mac_address(struct net_device *dev, void *addr)
{
	unsigned long hi_mac, low_mac;
	struct sockaddr *sock = addr;

	/* to make it safe, we won't do this while running */
	if (netif_running(dev))
		return -EBUSY;

	memcpy(dev->dev_addr, sock->sa_data, ETH_ALEN);

	hi_mac = (dev->dev_addr[5] << 8) | dev->dev_addr[4];
	low_mac = (dev->dev_addr[3] << 24)| (dev->dev_addr[2] << 16) |
		(dev->dev_addr[1] << 8) | dev->dev_addr[0];

	enet_writel(ENET_MAC_MACAHR, hi_mac);
	enet_writel(ENET_MAC_MACALR, low_mac);

	return 0;
}

/*
 * update hash table to reflect new device multicast address list
 */
static void enet_set_multicast_list(struct net_device *dev)
{
	struct tango2_enet_priv *priv;
	struct dev_mc_list *mclist;
	unsigned long val;
	uint32_t mc_filter[2];
	int i;

	priv = netdev_priv(dev);

	/* the link check timer might change MACCR, we need to protect
	 * against it */
	spin_lock_bh(&priv->maccr_lock);
	val = enet_readl(ENET_MAC_MACCR);

        if (dev->flags & IFF_PROMISC) {
                val |= MACCR_PR | MACCR_PM;
	} else {
		val &= ~MACCR_PR;
		/* if we want all multicast or if address count is too
		 * high, don't try to compute hash value */
		if (dev->mc_count > 64 || dev->flags & IFF_ALLMULTI) {
			val |= MACCR_PM;
		}
	}
	enet_writel(ENET_MAC_MACCR, val);
	spin_unlock_bh(&priv->maccr_lock);

	/* we  don't  need  to  update  hash  table  if  we  pass  all
	 * multicast */
	if (val & MACCR_PM)
		return;

	mc_filter[0] = mc_filter[1] = 0;
	mclist = dev->mc_list;

	for (i = 0; i < dev->mc_count; i++) {
		unsigned int n;
		char *addr;

		addr = mclist->dmi_addr;
		mclist = mclist->next;
		if (!(*addr & 1))
			continue;

		n = ether_crc(ETH_ALEN, addr) >> 26;
		mc_filter[n >> 5] |= 1 << (n & 31);
	}

	enet_writel(ENET_MAC_MALR, mc_filter[0]);
	enet_writel(ENET_MAC_MAHR, mc_filter[1]);
}

/*
 * open callback
 */
static int enet_open(struct net_device *dev)
{
	struct tango2_enet_priv *priv;
	unsigned long val;

	priv = netdev_priv(dev);

	/* check link */
	if (mii_check_media(&priv->mii, 1, 1))
		enet_link_reconfigure(dev);

	/* start rx & tx dma engine */
	enet_start_dma(priv);

	/* enable mac rx & tx */
	val = enet_readl(ENET_MAC_MACCR);
	val |= MACCR_TE | MACCR_RE;
	enet_writel(ENET_MAC_MACCR, val);

	/*
	 * clear & enable interrupts, we want:
	 * - receive complete
	 * - transmit complete
	 */
	enet_writel(ENET_DMA_SR, SR_NIS | SR_R | SR_T);
	enet_enable_interrupts(priv, 0);

	/* start link check & tx reclaim timer */
	priv->link_check_timer.expires = jiffies + LINK_CHECK_TIMER_FREQ;
	add_timer(&priv->link_check_timer);

	priv->tx_reclaim_timer.expires = jiffies + TX_RECLAIM_TIMER_FREQ;
	add_timer(&priv->tx_reclaim_timer);

	/* and finally start tx queue */
	netif_start_queue(dev);

	return 0;
}

/*
 * stop callback
 */
static int enet_stop(struct net_device *dev)
{
	struct tango2_enet_priv *priv;
	unsigned long val;
	int i;

	priv = netdev_priv(dev);

	/* stop link timer */
	del_timer_sync(&priv->link_check_timer);

	/* stop tx queue */
	netif_stop_queue(dev);

	/* wait for all tx buffers to be reclaimed */
	while (priv->free_tx_desc_count != ENET_TX_DESC_COUNT)
		yield();

	/* stop tx reclaim timer */
	del_timer_sync(&priv->tx_reclaim_timer);

	/* disable all interrupts */
	enet_disable_interrupts(priv, 0);

	/* stop dma */
	enet_stop_dma(priv);

	/* stop mac rx & tx */
	val = enet_readl(ENET_MAC_MACCR);
	val &= ~(MACCR_TE | MACCR_RE);
	enet_writel(ENET_MAC_MACCR, val);

	/* while we were stopping it,  the rx dma may have filled some
	 * buffer, consider it junk and rearm all descriptor */
	for (i = 0; i < ENET_RX_DESC_COUNT; i++) {
		volatile struct enet_rx_desc *rx;

		rx = &priv->rx_descs[i];
		rx->rdes0 = RDES0_OWN;
	}

	/* make  the dma engine  restarts at  first descriptor  in the
	 * list */
	enet_writel(ENET_DMA_RBAR, PHYSADDR(priv->rx_descs));
	enet_writel(ENET_DMA_TBAR, PHYSADDR(priv->tx_descs));
	priv->dirty_tx_desc = priv->next_tx_desc = 0;
	priv->last_rx_desc = 0;

	return 0;
}

/*
 * get_stats callback
 */
static struct net_device_stats *enet_get_stats(struct net_device *dev)
{
	struct tango2_enet_priv *priv;

	priv = netdev_priv(dev);

	return &priv->stats;
}

/*
 * ethtool callbacks
 */
static int enet_get_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct tango2_enet_priv *priv;
	int ret;

	priv = netdev_priv(dev);

	spin_lock_bh(&priv->mii_lock);
	ret = mii_ethtool_gset(&priv->mii, cmd);
	spin_unlock_bh(&priv->mii_lock);

	return ret;
}

static int enet_set_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct tango2_enet_priv *priv;
	int ret;

	priv = netdev_priv(dev);

	spin_lock_bh(&priv->mii_lock);
	ret = mii_ethtool_sset(&priv->mii, cmd);
	spin_unlock_bh(&priv->mii_lock);

	return ret;
}

static int enet_nway_reset(struct net_device *dev)
{
	struct tango2_enet_priv *priv;
	int ret;

	priv = netdev_priv(dev);

	spin_lock_bh(&priv->mii_lock);
	ret = mii_nway_restart(&priv->mii);
	spin_unlock_bh(&priv->mii_lock);

	return ret;
}

static u32 enet_get_link(struct net_device *dev)
{
	struct tango2_enet_priv *priv;
	int ret;

	priv = netdev_priv(dev);

	spin_lock_bh(&priv->mii_lock);
	ret = mii_link_ok(&priv->mii);
	spin_unlock_bh(&priv->mii_lock);

	return ret;
}

static struct ethtool_ops enet_ethtool_ops = {
	.get_settings		= enet_get_settings,
	.set_settings		= enet_set_settings,
	.nway_reset		= enet_nway_reset,
	.get_link		= enet_get_link,
};

static int enet_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
	struct tango2_enet_priv *priv;
	int ret;

	priv = netdev_priv(dev);

	spin_lock_bh(&priv->mii);
	ret = generic_mii_ioctl(&priv->mii, if_mii(rq), cmd, NULL);
	spin_unlock_bh(&priv->mii);

	return ret;
}

/*
 * dma ring allocation is done here
 */
static int enet_dma_init(struct tango2_enet_priv *priv)
{
	unsigned int size;
	int i;

	/*
	 * allocate rx descriptor list & rx buffers
	 *
	 * We allocate  skb now and fill buffer  with their addresses,
	 * note that we reserve 4 bytes at beginning of data buffer to
	 * store skb address.
	 *
	 */
	size = ENET_RX_DESC_COUNT * sizeof (struct enet_rx_desc);
	if (!(priv->rx_descs_cached = kmalloc(size, GFP_KERNEL)))
		return -ENOMEM;
	priv->rx_descs = (volatile struct enet_rx_desc *)
		CACHE_TO_NONCACHE((unsigned long)priv->rx_descs_cached);
	dma_cache_wback_inv((unsigned long)priv->rx_descs_cached, size);

	/*
	 * initialize all rx descs
	 */
	for (i = 0; i < ENET_RX_DESC_COUNT; i++) {
		volatile struct enet_rx_desc *rx;
		struct sk_buff *skb;

		rx = &priv->rx_descs[i];
		rx->rdes0 = RDES0_OWN;

		rx->rdes1 = RDES1_RBS2(0) | RDES1_RBS1(RX_BUF_SIZE);
		if (i == ENET_RX_DESC_COUNT - 1)
			rx->rdes1 |= RDES1_RER;

		skb = dev_alloc_skb(RX_BUF_SIZE + SKB_RESERVE_SIZE);
		if (!skb)
			return -ENOMEM;

		skb_reserve(skb, SKB_RESERVE_SIZE);
		rx->rdes2 = PHYSADDR(skb->data);
		rx->rdes3 = 0;

		dma_cache_inv((unsigned long)skb->data, RX_BUF_SIZE);
		priv->rx_skbs[i] = skb;
	}
	priv->last_rx_desc = 0;

	/*
	 * allocate tx descriptor list
	 *
	 * We allocate  only the descriptor list and  prepare them for
	 * further use. When tx is needed, we will set the right flags
	 * and kick the dma.
	 */
	size = ENET_TX_DESC_COUNT * sizeof (struct enet_tx_desc);
	if (!(priv->tx_descs_cached = kmalloc(size, GFP_KERNEL)))
		return -ENOMEM;
	priv->tx_descs = (volatile struct enet_tx_desc *)
		CACHE_TO_NONCACHE((unsigned long)priv->tx_descs_cached);
	dma_cache_wback_inv((unsigned long)priv->tx_descs_cached, size);

	/*
	 * initialize tx descs
	 */
	for (i = 0; i < ENET_TX_DESC_COUNT; i++) {
		volatile struct enet_tx_desc *tx;

		tx = &priv->tx_descs[i];
		tx->tdes0 = 0;
		tx->tdes1 = 0;
		if (i == ENET_TX_DESC_COUNT - 1)
			tx->tdes1 |= TDES1_TER;
		tx->tdes2 = 0;
		tx->tdes3 = 0;
	}
	priv->dirty_tx_desc = priv->next_tx_desc = 0;
	priv->free_tx_desc_count = ENET_TX_DESC_COUNT;

	/*
	 * write rx desc list & tx desc list addresses in registers
	 */
	enet_writel(ENET_DMA_RBAR, PHYSADDR(priv->rx_descs));
	enet_writel(ENET_DMA_TBAR, PHYSADDR(priv->tx_descs));

	return 0;
}

/*
 * free  all dma rings  memory, called  at uninit  time or  when error
 * occurs at init time
 */
static void enet_dma_free(struct tango2_enet_priv *priv)
{
	int i;

	/* note: kfree(NULL) is ok */
	kfree(priv->rx_descs_cached);
	kfree(priv->tx_descs_cached);

	/* note: kfree_skb(NULL) is _not_ ok */
	for (i = 0; i < ENET_RX_DESC_COUNT; i++) {
		if (priv->rx_skbs[i])
			kfree_skb(priv->rx_skbs[i]);
	}

	for (i = 0; i < ENET_TX_DESC_COUNT; i++) {
		if (priv->tx_skbs[i])
			kfree_skb(priv->tx_skbs[i]);
	}
}

/*
 * mac hw init is done here
 */
static int enet_hw_init(struct net_device *dev)
{
	struct tango2_enet_priv *priv;
	int loop;

	priv = netdev_priv(dev);

	/* reset phy */
	enet_mdio_write(dev, priv->mii.phy_id, MII_BMCR, BMCR_RESET);

	/* wait for the reset bit to clear */
	udelay(100);
	loop = 100;
	while (loop) {
		if (!(enet_mdio_read(dev, priv->mii.phy_id,
				     MII_BMCR) & BMCR_RESET))
			break;
		mdelay(1);
		loop--;
	}

	if (!loop) {
		printk(KERN_ERR PFX "PHY reset does not complete...\n");
		return -EBUSY;
	}

	/* reset dma engine */
	enet_writel(ENET_DMA_BMR, BMR_SWR);

	/* wait for the reset bit to clear */
	udelay(100);
	loop = 100;
	while (loop) {
		if (!(enet_readl(ENET_DMA_BMR) & BMR_SWR))
			break;
		mdelay(1);
		loop--;
	}

	if (!loop) {
		printk(KERN_ERR PFX "dma engine does not exit reset...\n");
		return -EBUSY;
	}

	/* set bus mode */
	enet_writel(ENET_DMA_BMR, BMR_PBL(32));

	/* enable MAC flow ctrl */
	enet_writel(ENET_MAC_FCR, FCR_ENABLE);

	/* configure MAC ctrller to do hash perfect filtering  */
	enet_writel(ENET_MAC_MACCR, MACCR_ASTP | MACCR_HP);

	/* clear hash table */
	enet_writel(ENET_MAC_MAHR, 0xffffffff);
	enet_writel(ENET_MAC_MALR, 0xffffffff);

	return 0;
}



/*
 * allocate  netdevice structure,  do  all dma  rings allocations  and
 * register the netdevice
 */
extern int tangox_ethernet_getmac(int, unsigned char *);

static int enet_probe(void)
{
	struct tango2_enet_priv *priv;
	struct net_device *dev;
	int ret;
	struct sockaddr sock;

	printk(KERN_INFO PFX "ethernet driver for SMP863x internal mac\n");

	/* allocate  netdevice structure  with enough  length  for our
	 * context data */
	dev = alloc_etherdev(sizeof (*priv));
	if (!dev)
		return -ENOMEM;

	/* initialize private data */
	priv = netdev_priv(dev);
	memset(priv, 0, sizeof (*priv));
	spin_lock_init(&priv->tx_lock);
	spin_lock_init(&priv->ier_lock);
	spin_lock_init(&priv->maccr_lock);

	/* init tx done tasklet */
	tasklet_init(&priv->tx_reclaim_tasklet, enet_tx_reclaim,
		     (unsigned long)dev);

	/* init tx reclaim timer */
	init_timer(&priv->tx_reclaim_timer);
	priv->tx_reclaim_timer.data = (unsigned long )dev;
	priv->tx_reclaim_timer.function = enet_tx_reclaim_timer;

	/* init link check timer and mii lock */
	init_timer(&priv->link_check_timer);
	priv->link_check_timer.data = (unsigned long)dev;
	priv->link_check_timer.function = enet_link_check;
	spin_lock_init(&priv->mii_lock);

	/* fill mii info */
	priv->mii.dev = dev;
	priv->mii.phy_id_mask = 0x1f;
	priv->mii.reg_num_mask = 0x1f;
	priv->mii.mdio_read = enet_mdio_read;
	priv->mii.mdio_write = enet_mdio_write;

	if (gphy_id != -1) {
		/* phy id forced, just check for sanity */
		if (gphy_id < 0 || gphy_id > 31) {
			ret = -EINVAL;
			goto err_free;
		}
		priv->mii.phy_id = gphy_id;

	} else {
		int i;

		/* try to probe phy if not given */
		for (i = 1; i < 32; i++) {
			uint32_t id;
			int val;

			val = enet_mdio_read(dev, i, MII_PHYSID1);
			id = (val << 16);
			val = enet_mdio_read(dev, i, MII_PHYSID2);
			id |= val;

			if (id != 0xffffffff && id != 0x00000000)
				break;
		}

		if (i == 32) {
			printk(KERN_ERR PFX "unable to autodetect phy\n");
			ret = -EIO;
			goto err_free;
		}

		printk(KERN_ERR PFX "detected phy at address 0x%02x\n", i);
		priv->mii.phy_id = i;
	}

	/* initialize hardware */
	if ((ret = enet_hw_init(dev)))
		goto err_free;

	/* initialize dma rings */
	if ((ret = enet_dma_init(priv)))
		goto err_free;

	/* register interrupt handler */
	ret = request_irq(ENET_IRQ, enet_isr, IRQF_SHARED, "tango2_enet", dev);
	if (ret)
		goto err_free;
	dev->irq = ENET_IRQ;

	/* install driver callbacks and register netdevice */
	dev->open = enet_open;
	dev->stop = enet_stop;
	dev->hard_start_xmit = enet_xmit;
	dev->get_stats = enet_get_stats;
	dev->set_mac_address = enet_set_mac_address;
	dev->set_multicast_list = enet_set_multicast_list;
	dev->poll = enet_poll;
	dev->ethtool_ops = &enet_ethtool_ops;
	dev->do_ioctl = enet_ioctl;
	dev->weight = 16;

	/* set default mac address */
	tangox_ethernet_getmac(0, dev->dev_addr);
	memcpy(&(sock.sa_data), dev->dev_addr, ETH_ALEN);
	enet_set_mac_address(dev, &sock);

	if ((ret = register_netdev(dev))) {
		printk(KERN_ERR PFX "unable to register netdevice\n");
		goto err_free;
	}

	printk(KERN_INFO PFX "mac address %02x:%02x:%02x:%02x:%02x:%02x\n",
	       dev->dev_addr[0], dev->dev_addr[1], dev->dev_addr[2],
	       dev->dev_addr[3], dev->dev_addr[4], dev->dev_addr[5]);

	gdev = dev;
	return 0;

err_free:
	if (dev->irq)
		free_irq(dev->irq, dev);
	enet_dma_free(priv);
	free_netdev(dev);
	return ret;
}


/*
 * entry point, checks if ethernet is  enabled on the board and if so,
 * probes it
 */
extern int tangox_ethernet_enabled(int);

int __init tango2_enet_init(void)
{
	if (!tangox_ethernet_enabled(0)) {
		printk(KERN_NOTICE PFX "ethernet support is disabled\n");
		return -ENODEV;
	}

	return enet_probe();
}

/*
 * exit func, stops hardware and unregisters netdevice
 */
void __exit tango2_enet_exit(void)
{
	struct tango2_enet_priv *priv;
	struct net_device *dev;

	dev = gdev;

	free_irq(dev->irq, dev);
	unregister_netdev(dev);

	priv = netdev_priv(dev);
	enet_dma_free(priv);

	free_netdev(dev);
}


module_init(tango2_enet_init);
module_exit(tango2_enet_exit);

MODULE_DESCRIPTION("SMP863x internal ethernet mac driver");
MODULE_AUTHOR("Maxime Bizon <mbizon@freebox.fr>");
MODULE_LICENSE("GPL");
MODULE_PARM_DESC(gphy_id, "PHY id, else autodetect");
module_param(gphy_id, int, 0);

