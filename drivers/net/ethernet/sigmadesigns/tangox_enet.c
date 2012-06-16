/*********************************************************************
 Copyright (C) 2001-2011
 Sigma Designs, Inc. 

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/
/*
 * Driver for tangox SMP864x/SMP865x/SMP867x/SMP89xx builtin Ethernet Mac.
 * The Driver makes use of the tango2_enet framework
 * written by Maxime Bizon.
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/etherdevice.h>
#include <linux/delay.h>
#include <linux/ethtool.h>
#include <linux/crc32.h>
#include <linux/sched.h>

#include <asm/barrier.h>

#include "tangox_enet.h"

#define PFX	"tangox_enet: "

//#define ETH_DEBUG 1 
#ifdef ETH_DEBUG
#define DBG	printk
#else
static void inline DBG(const char *x, ...) { ; }
#endif /* ETH_DEBUG */

/* for multicast support */
#define ENABLE_MULTICAST

struct eth_mac_core {
	unsigned long enet_mac_base;
	int phy_id;
	const int irq;
	struct net_device *gdev;
	const char *name;
};

static struct eth_mac_core eth_mac_cores[NUM_MAC_CORES] = {
	{ REG_BASE_host_interface + 0x6000, -1, ENET_IRQ0, NULL, "tangox_enet0" },
	{ REG_BASE_host_interface + 0x6800, -1, ENET_IRQ1, NULL, "tangox_enet1" },
};

static int mac_cores = 3; /* enabled both cores */
static int phyid_0 = -1;
static int phyid_1 = -1;
static int data_aligned = DEF_DATA_ALIGNMENT; /* required alignment, 2^order */

#ifdef CONFIG_TANGOX_ENET0_NOMDIO
#if (CONFIG_TANGOX_ENET0_NOMDIO_SPEED != 10) && (CONFIG_TANGOX_ENET0_NOMDIO_SPEED != 100) 
#error Invalid speed setting
#endif
#if (CONFIG_TANGOX_ENET0_NOMDIO_PHY_ADDR < 0) || (CONFIG_TANGOX_ENET0_NOMDIO_PHY_ADDR > 31)
#error Invalid PHY address
#endif
#endif
#ifdef CONFIG_TANGOX_ENET1_NOMDIO
#if (CONFIG_TANGOX_ENET1_NOMDIO_SPEED != 10) && (CONFIG_TANGOX_ENET1_NOMDIO_SPEED != 100) 
#error Invalid speed setting
#endif
#if (CONFIG_TANGOX_ENET1_NOMDIO_PHY_ADDR < 0) || (CONFIG_TANGOX_ENET1_NOMDIO_PHY_ADDR > 31)
#error Invalid PHY address
#endif
#endif

/*
 * mdio read/write callback, can run from userspace or timer
 */

#define MDIO_TIMEOUT	1000

static __inline int enet_mdio_read(struct net_device *dev, int phy_id, int location)
{
	int val, i;
	struct tangox_enet_priv *priv = netdev_priv(dev);

	for (i = 0; (i < MDIO_TIMEOUT) && (gbus_read_reg32(ENET_MDIO_CMD1(priv->enet_mac_base)) & MDIO_CMD_GO); i++)
		udelay(1);
	if (i >= MDIO_TIMEOUT)
		goto err_out;

	val = MIIAR_ADDR(phy_id) | MIIAR_REG(location);
	gbus_write_reg32(ENET_MDIO_CMD1(priv->enet_mac_base), val);

	udelay(10);

	gbus_write_reg32(ENET_MDIO_CMD1(priv->enet_mac_base), val | MDIO_CMD_GO);

	for (i = 0; (i < MDIO_TIMEOUT) && (gbus_read_reg32(ENET_MDIO_CMD1(priv->enet_mac_base)) & MDIO_CMD_GO); i++)
		udelay(1);
	if (i >= MDIO_TIMEOUT)
		goto err_out;

	val = gbus_read_reg32(ENET_MDIO_STS1(priv->enet_mac_base));
	if (val & MDIO_STS_ERR)
		return -1;

	return val & 0xffff;

err_out:
	return -1;
}

static void enet_mdio_write(struct net_device *dev, int phy_id, int location, int val)
{
	int i, tmp;
	struct tangox_enet_priv *priv = netdev_priv(dev);

	for (i = 0; (i < MDIO_TIMEOUT) && (gbus_read_reg32(ENET_MDIO_CMD1(priv->enet_mac_base)) & MDIO_CMD_GO); i++)
		udelay(1);
	if (i >= MDIO_TIMEOUT)
		goto err_out;

	tmp = MIIAR_DATA(val) | MIIAR_ADDR(phy_id) | MIIAR_REG(location);
	gbus_write_reg32(ENET_MDIO_CMD1(priv->enet_mac_base), tmp);

	udelay(10);

	gbus_write_reg32(ENET_MDIO_CMD1(priv->enet_mac_base), tmp | MDIO_CMD_WR);

	udelay(10);

	gbus_write_reg32(ENET_MDIO_CMD1(priv->enet_mac_base), tmp | MDIO_CMD_WR | MDIO_CMD_GO);

	for (i = 0; (i < MDIO_TIMEOUT) && (gbus_read_reg32(ENET_MDIO_CMD1(priv->enet_mac_base)) & MDIO_CMD_GO); i++)
		udelay(1);
	if (i >= MDIO_TIMEOUT)
		goto err_out;

	return;

err_out:
	return;
}

#if 0 /* enable if needed */
/* statistic counter read and write functions
 * 44 counters are included for tracking 
 * occurences of frame status evernts.
 */
static unsigned long enet_stat_read(struct net_device *dev, unsigned char index)
{
	struct tangox_enet_priv *priv = netdev_priv(dev);
	gbus_write_reg8(ENET_STAT_INDEX(priv->enet_mac_base), index);
	return gbus_read_reg32(ENET_STAT_DATA1(priv->enet_mac_base));
}

static void enet_stat_write(struct net_device *dev, unsigned long val, unsigned char index)
{
	struct tangox_enet_priv *priv = netdev_priv(dev);
	gbus_write_reg8(ENET_STAT_INDEX(priv->enet_mac_base), index);
	gbus_write_reg32(ENET_STAT_DATA1(priv->enet_mac_base), val);
}
#endif

static __inline int enet_rx_error(struct net_device *dev, unsigned long report)
{
	if (report & (RX_FCS_ERR | RX_FRAME_LEN_ERROR | RX_FIFO_OVERRUN |
#ifndef ENABLE_MULTICAST
		RX_MULTICAST_PKT |
#endif
		RX_LENGTH_ERR | RX_LATE_COLLISION | RX_RUNT_PKT))		
		return 1;
	else
		return 0;
}

static void enet_mac_rx(struct net_device * dev, int enable)
{
	int val;
	struct tangox_enet_priv *priv = netdev_priv(dev);

	while (gbus_read_reg32(ENET_TXC_CR(priv->enet_mac_base)) & TCR_EN)
		;

	val = gbus_read_reg8(ENET_TX_CTL1(priv->enet_mac_base));
	if (enable)
		val |= TX_EN;
	else
		val &= ~TX_EN;
	gbus_write_reg8(ENET_TX_CTL1(priv->enet_mac_base), val);
}

static void enet_mac_tx(struct net_device * dev, int enable)
{
	int val;
	struct tangox_enet_priv *priv = netdev_priv(dev);

	val = gbus_read_reg8(ENET_RX_CTL(priv->enet_mac_base));
	if (enable)
		val |= RX_EN;
	else
		val &= ~RX_EN;
	gbus_write_reg8(ENET_RX_CTL(priv->enet_mac_base), val);
}

static int enet_get_speed(struct net_device *dev);
static void enet_ar8035_loopback(struct net_device * dev)
{
	int speed = SPEED_1000;
	struct tangox_enet_priv *priv = netdev_priv(dev);

	if(priv->rgmii != PHY_AR8035)
		return;

	speed = enet_get_speed(dev);
	if (speed == SPEED_1000) 
		enet_mdio_write(dev, priv->mii.phy_id, MII_BMCR, 0x4140);
	else if (speed == SPEED_100)	
		enet_mdio_write(dev, priv->mii.phy_id, MII_BMCR, 0x6100);
	else if (speed == SPEED_10)
		enet_mdio_write(dev, priv->mii.phy_id, MII_BMCR, 0x4100);
}

static void enet_mac_loopback(struct net_device * dev, int enable)
{
	int val;
	struct tangox_enet_priv *priv = netdev_priv(dev);

	val = gbus_read_reg8(ENET_MAC_MODE(priv->enet_mac_base));
	if (enable)
		val |= LB_EN; 
	else
		val &= ~LB_EN; 
	gbus_write_reg8(ENET_MAC_MODE(priv->enet_mac_base), val);
}

static void enet_mac_af(struct net_device * dev, int enable)
{
	int val;
	struct tangox_enet_priv *priv = netdev_priv(dev);
	val = gbus_read_reg8(ENET_RX_CTL(priv->enet_mac_base));
	if (enable)
		val |= RX_AF_EN;
	else
		val &= ~RX_AF_EN;
	gbus_write_reg8(ENET_RX_CTL(priv->enet_mac_base), val);
}

/*
 * start/stop dma engine
 */
static int __enet_xmit(struct sk_buff *skb, struct net_device *dev, int force);
static void enet_stop_rx(struct net_device *dev)
{
	volatile struct enet_desc *rx;
	int i, cnt;
	struct sk_buff *dummy_skb = NULL;
	struct ethhdr *ethr;
	struct tangox_enet_priv *priv = netdev_priv(dev);

	for (i = 0; i < priv->rx_desc_count; i++) {
		cnt = (priv->last_rx_desc + priv->rx_desc_count - i) % priv->rx_desc_count;
		rx = &priv->rx_descs[cnt];
		rx->config |= DESC_EOC;
		wmb();
	}

	/*transmit/receive a dummy packet*/
	if (unlikely((dummy_skb = dev_alloc_skb(TX_BUF_SIZE + SKB_RESERVE_SIZE)) == NULL)) {
		printk("%s: failed to allocation sk_buff for stopping rx.\n", priv->name);
		return;
	}

	ethr = (struct ethhdr *)dummy_skb->data;
	memset(dummy_skb->data, 0, ETH_ZLEN);
	for (i = 0; i < 6; i++) {
		ethr->h_dest[i] = 0xff;
		ethr->h_source[i] = dev->dev_addr[i];
	}
	dummy_skb->len= ETH_ZLEN;

	/*stop tx and rx in mac*/
	enet_mac_tx(dev, 0);
	enet_mac_rx(dev, 0);

	/* set ar8035 to digital loopback mode */
	enet_ar8035_loopback(dev);

	/* address filtering needs to be disabled */
	enet_mac_af(dev, 0);

	/*program loopback mode in mac */
	enet_mac_loopback(dev, 1);

	/*start rx and tx */
	enet_mac_tx(dev, 1);
	enet_mac_rx(dev, 1);

	/*send the packet and wait for dma stop */
	__enet_xmit(dummy_skb, dev, 0);
	while (gbus_read_reg32(ENET_RXC_CR(priv->enet_mac_base)) & RCR_EN) {
		udelay(1000);
		__enet_xmit(NULL, dev, 1);
	}

	/*stop tx and rx in mac*/
	enet_mac_tx(dev, 0);
	enet_mac_rx(dev, 0);

	/*enable Address filter */
	enet_mac_af(dev, 1);

	/* disable loopback mode */
	enet_mac_loopback(dev, 0);

	/*start rx and tx */
	enet_mac_tx(dev, 1);
	enet_mac_rx(dev, 1);

	kfree_skb(dummy_skb);
}

static __inline void rearm_rx_descs(struct net_device *dev)
{
	int i, cnt;	
	volatile struct enet_desc *rx;
	struct tangox_enet_priv *priv = netdev_priv(dev);

	for (i = 0; i < priv->rx_desc_count; i++) {
		cnt = (priv->last_rx_desc + i) % priv->rx_desc_count;
		rx = &priv->rx_descs[cnt];
		priv->rx_report[cnt] = 0;
		if (cnt != priv->rx_eoc)
			rx->config &= ~DESC_EOC;
		else
			rx->config |= DESC_EOC;
	}
	wmb();
}

static __inline void enet_start_rx(struct net_device *dev)
{
	unsigned long val, flags;
	struct tangox_enet_priv *priv = netdev_priv(dev);

	spin_lock_irqsave(&priv->ier_lock, flags);

	val = gbus_read_reg32(ENET_RXC_CR(priv->enet_mac_base));
	if ((val & RCR_EN) == 0) {
		val |= RCR_EN;
		gbus_write_reg32(ENET_RXC_CR(priv->enet_mac_base), val);
	}
	spin_unlock_irqrestore(&priv->ier_lock, flags);
}


#ifdef ETH_DEBUG
static void dump_desc(struct tangox_enet_priv *priv)
{
	volatile struct enet_desc *rx;
	int i;	
	printk("\n");
	for (i = 0; i < priv->rx_desc_count; i++) {
		int cnt;
		u32 report;
		cnt = (priv->last_rx_desc + i) % priv->rx_desc_count;
		rx = &priv->rx_descs[cnt];
		report = priv->rx_report[cnt];
		printk("%d rx[%d]=0x%08x s_addr=0x%08x n_addr=0x%08x r_addr=0x%08x config=0x%08x report=0x%x\n", 
			i, cnt, (unsigned int)rx, (unsigned int)rx->s_addr, (unsigned int)rx->n_addr,
			(unsigned int)rx->r_addr, (unsigned int)rx->config, report);
	}
	printk("\n");
}

static void dump_regs(struct tangox_enet_priv *priv)
{
	int i;
	for (i = 0; i < 7; i++)
		printk("TX reg[%d]=0x%x\n", i, gbus_read_reg32(priv->enet_mac_base +0x100 +i*4));

	for (i = 0; i < 7; i++)
		printk("RX reg[%d]=0x%x\n", i, gbus_read_reg32(priv->enet_mac_base +0x200 +i*4));
}
#endif

/*
 * rx poll func, called by network core
 */
static int enet_poll(struct napi_struct *napi, int budget)
{
	struct tangox_enet_priv *priv;
	struct net_device *dev;
	volatile struct enet_desc *rx, *rx1;
	int limit, received, eoc = 0;
	unsigned int rx_eoc, r_sz;

	priv = container_of(napi, struct tangox_enet_priv, napi);
	dev = napi->dev;
	rx_eoc = priv->rx_eoc;

	/* calculate how many rx packet we are allowed to fetch */
	limit = budget;
	received = 0;

	/* process no more than "limit" done rx */
	while (limit > 0) {
		struct sk_buff *skb;
		u32 report;
		unsigned int len = 0;
		int pkt_dropped = 0;

		rx = &priv->rx_descs[priv->last_rx_desc];

		/* we need multiple read on this volatile, avoid
		 * memory access at each time */
		report = priv->rx_report[priv->last_rx_desc];

		if (rx->config & DESC_EOC) {
			/* should not happen, only when stop rx dma */
			//DBG("%s i=0x%x rx=0x%p report=0x%x config=0x%lx limit=0x%x\n", 
			//	__FUNCTION__, priv->last_rx_desc, rx, report, rx->config, limit);
			eoc = 1;
			break;
		}

		if (report == 0) { 
			uint32_t next_report;
			next_report = priv->rx_report[(priv->last_rx_desc + 1) % priv->rx_desc_count];

			/*check see if next one on error*/
			if (enet_rx_error(dev, next_report) == 0)
				break;
		}

		--limit;

		if (likely((skb = priv->rx_skbs[priv->last_rx_desc]) != NULL)) {

			len = RX_BYTES_TRANSFERRED(report);
			if ((report == 0) || enet_rx_error(dev, report)) {

#ifndef ENABLE_MULTICAST
				if (report & RX_MULTICAST_PKT) { 
					DBG("%s RX_MULTICAST_PKT report=0x%x\n", __FUNCTION__, report);				
					priv->stats.rx_length_errors++;
				}
#endif
				if (report & RX_FCS_ERR) {
					DBG("%s RX_FCS_ERR report=0x%x\n", __FUNCTION__, report);				
					priv->stats.rx_crc_errors++;
				}

				if (report & RX_LATE_COLLISION) { 
					DBG("%s RX_LATE_COLLSION report=0x%x\n", __FUNCTION__, report);				
				}

				if (report & RX_FIFO_OVERRUN) { 
					DBG("%s RX_FIFO_OVERRUN report=0x%x fifo_ctl=0x%x\n", __FUNCTION__, report, 
							gbus_read_reg8(ENET_FIFO_CTL(priv->enet_mac_base)));
				}

				if (report & RX_RUNT_PKT) {
					DBG("%s RX_RUNT_PKT report=0x%x\n", __FUNCTION__, report);				
				}

				if ((report & (RX_FRAME_LEN_ERROR | RX_LENGTH_ERR)) || (len > RX_BUF_SIZE)) {
					DBG("%s RX LENTH ERROR report=0x%x\n", __FUNCTION__, report);		
					priv->stats.rx_length_errors++;
				}

				priv->stats.rx_errors++;
				pkt_dropped = 1;
				goto done_checking;

			} else {

				/* ok, seems valid, adjust skb proto and len
				 * and give it to kernel */
				skb->dev = dev;
				skb_put(skb, len);
				skb->protocol = eth_type_trans(skb, dev);
				netif_receive_skb(skb);
#ifdef ETH_DEBUG
				if (len > 0) {
					int i;
					DBG("-----received data len=0x%x-----\n", len);
					for (i = 0; i < len; i++) {
						if (i%16==0 && i>0)
							DBG("\n");
						DBG("%02x ", skb->data[i]);					
					}
					DBG("\n--------------------------\n");
				}
#endif
			}
done_checking:
			rx_eoc = priv->last_rx_desc;

			if (pkt_dropped)
				goto rearm;

			priv->stats.rx_packets++;
			priv->stats.rx_bytes += len;
			dev->last_rx = jiffies;
			priv->rx_skbs[priv->last_rx_desc] = NULL;
			/* we will re-alloc an skb for this slot */
		}

		if (unlikely((skb = dev_alloc_skb(RX_BUF_SIZE + SKB_RESERVE_SIZE)) == NULL)) {
			printk("%s: failed to allocation sk_buff.\n", priv->name);
			rx->config = DESC_BTS(2) | DESC_EOF/* | DESC_ID*/;
			wmb();
			break;
		}
		dma_cache_inv((unsigned long)skb->data, RX_BUF_SIZE + SKB_RESERVE_SIZE);

		rx->config = RX_BUF_SIZE | DESC_BTS(2) | DESC_EOF/* | DESC_ID*/;

		r_sz = 8 - (((u32)skb->data) & 0x7);
		skb_reserve(skb, (r_sz == 8) ? 0 : r_sz); /* make sure it's aligned to 8 bytes boundary */
		rx->s_addr = DMA_ADDR((void *)skb->data);
		priv->rx_skbs[priv->last_rx_desc] = skb;

rearm:
		/* rearm descriptor */
		priv->rx_report[priv->last_rx_desc] = 0;
		priv->last_rx_desc++;
		priv->last_rx_desc %= priv->rx_desc_count;
		received++;
		wmb();
	}

	if (received) {
		rx = &priv->rx_descs[rx_eoc];
		rx->config |= DESC_EOC;
		rx1 = &priv->rx_descs[priv->rx_eoc];
		rx1->config &= ~DESC_EOC; 
		priv->rx_eoc = rx_eoc;
		wmb();

		budget -= received;
	}

	if (eoc == 0)
		enet_start_rx(dev);

	if (limit <= 0) {
		/* breaked, but there is still work to do */
		DBG("breaked, more work to do, rx dma will be off\n");
		return 1;
	}

	__napi_complete(napi);
	atomic_set(&priv->napi_poll, 0);
	return 0;
}

/*
 * tx request callback
 */
static int __enet_xmit(struct sk_buff *skb, struct net_device *dev, int force)
{
	struct tangox_enet_priv *priv;
	volatile struct enet_desc *tx, *stx = NULL;
	unsigned long tconfig_cache;
	unsigned long val;
	int tx_busy = 0, cpsz = 0;
	unsigned char *txbuf;

	priv = netdev_priv(dev);
	spin_lock_bh(&priv->tx_lock);

	if ((skb == NULL) && (force == 0) && ((priv->pending_tx < 0) || (priv->pending_tx_cnt <= 0)))
		goto done; /* nothing needs to be done */
	else if ((skb != NULL) && (priv->free_tx_desc_count <= ENET_DESC_LOW)) {
		netif_stop_queue(dev);
		spin_unlock_bh(&priv->tx_lock);
		return NETDEV_TX_BUSY;
	}

	val = gbus_read_reg32(ENET_TXC_CR(priv->enet_mac_base)) & 0xf8ff;
	if (val & TCR_EN) { 
		if (skb == NULL) {
			spin_unlock_bh(&priv->tx_lock);
			return NETDEV_TX_BUSY;
		} if (priv->is_mdesc) {
			netif_stop_queue(dev);
			spin_unlock_bh(&priv->tx_lock);
			return NETDEV_TX_BUSY;
		}
		
		tx_busy = 1;
		if (priv->pending_tx < 0)
			priv->pending_tx = priv->next_tx_desc;
	} 

	/* to transmit the last chain */
	if (skb == NULL) {
		unsigned int last_tx = (priv->next_tx_desc + priv->tx_desc_count - 1) % priv->tx_desc_count;
		stx = &priv->tx_descs[last_tx];
		stx->config |= DESC_EOC;
		wmb();
		goto tx_pending;
	}

	dma_cache_wback((unsigned long)skb->data, skb->len);
	cpsz = (data_aligned - ((u32)skb->data & (data_aligned - 1))) % data_aligned;

	/* fill the tx desc with this skb address */
	if (cpsz) { /* not align by 8 bytes, need to use one extra descriptor */
		/* use mempcy for this extra descriptor */
		tconfig_cache = DESC_BTS(2) | cpsz; 
		stx = &priv->tx_descs[priv->next_tx_desc];
		txbuf = priv->tx_bufs[priv->next_tx_desc];
		memcpy(txbuf, skb->data, cpsz); 
		stx->s_addr = DMA_ADDR((void *)txbuf);
		stx->n_addr = DMA_ADDR((void *)&(priv->tx_descs[(priv->next_tx_desc + 1) % priv->tx_desc_count]));
		stx->config = tconfig_cache;

		/* keep a pointer to it for later and give it to dma */
		priv->tx_skbs[priv->next_tx_desc] = skb;
		priv->tx_report[priv->next_tx_desc] = 0;
		priv->next_tx_desc++;
		priv->next_tx_desc %= priv->tx_desc_count;
		wmb();
	}

	tconfig_cache = DESC_BTS(2) | DESC_EOF | (skb->len - cpsz); 
	tx = &priv->tx_descs[priv->next_tx_desc];
	tx->s_addr = DMA_ADDR(skb->data + cpsz);

	if (tx_busy) {
		tx->n_addr = DMA_ADDR((void *)&(priv->tx_descs[(priv->next_tx_desc + 1) % priv->tx_desc_count]));
	} else {
		tx->n_addr = 0;
		tconfig_cache |= DESC_EOC;
	}
	tx->config = tconfig_cache;

	/* keep a pointer to it for later and give it to dma if needed */
	if (cpsz) 
		priv->tx_skbs[priv->next_tx_desc] = NULL;
	else {
		priv->tx_skbs[priv->next_tx_desc] = skb;
		stx = tx;
	}
	priv->tx_report[priv->next_tx_desc] = 0;
	priv->next_tx_desc++;
	priv->next_tx_desc %= priv->tx_desc_count;
	wmb();

#ifdef ETH_DEBUG
	{
		int i;			
 		for (i = 0; i < skb->len; i++) {
			if (((i %16) == 0) && (i > 0))
				DBG("\n");
			DBG("%02x ", skb->data[i] & 0xff);
		}
		DBG("\n");
		DBG("DESC Mode:  TXC_CR=0x%x  desc_addr=0x%x s_addr=0x%x n_addr=0x%x r_addr=0x%x config=0x%x\n",
				gbus_read_reg32(ENET_TXC_CR(priv->enet_mac_base)), stx,
				stx->s_addr, stx->n_addr,
				stx->r_addr, stx->config); 
	}
#endif

tx_pending:
	if (tx_busy == 0) {
		if (priv->pending_tx >= 0) {
			stx = &priv->tx_descs[priv->pending_tx];
			priv->reclaim_limit = priv->pending_tx;
			priv->is_mdesc = 1;
		} else {
			priv->reclaim_limit = (priv->next_tx_desc - 1 + priv->tx_desc_count) % priv->tx_desc_count;
			priv->is_mdesc = 0;
		}

		gbus_write_reg32(ENET_TX_DESC_ADDR(priv->enet_mac_base), DMA_ADDR((void *)stx));
		gbus_write_reg32(ENET_TX_SAR(priv->enet_mac_base), 0);
		gbus_write_reg32(ENET_TX_REPORT_ADDR(priv->enet_mac_base), 0);

		/* kick tx dma in case it was suspended */
		val |= (TCR_EN | (((stx->config >> 16) & 0x7) << 8) | ((stx->config & 0xffff) << 16)); 
		gbus_write_reg32(ENET_TXC_CR(priv->enet_mac_base), val);

		/* no pending at this stage*/
		priv->pending_tx = -1;
		priv->pending_tx_cnt = 0;
	} else {
		if (skb)
			priv->pending_tx_cnt += (cpsz ? 2 : 1);
	}

	if (skb)
		priv->free_tx_desc_count -= (cpsz ? 2 : 1); 

done:
	spin_unlock_bh(&priv->tx_lock);

	return NETDEV_TX_OK;
}

static int enet_xmit(struct sk_buff *skb, struct net_device *dev)
{
	return __enet_xmit(skb, dev, 0);
}

/*
 * tx reclaim func. Called by timer or tx done tasklet to reclaim sent
 * buffers.
 */
static void enet_tx_reclaim(unsigned long data)
{
	struct net_device *dev;
	struct tangox_enet_priv *priv;

	dev = (struct net_device *)data;
	priv = netdev_priv(dev);

	__enet_xmit(NULL, dev, 0); /* kick tx dma in case the chain was suspended */

	spin_lock_bh(&priv->tx_lock);

	while (priv->free_tx_desc_count < priv->tx_desc_count) {
		u32 report;
		struct sk_buff *skb;

		report = priv->tx_report[priv->dirty_tx_desc];
		if (priv->dirty_tx_desc == priv->reclaim_limit)
			break;

		skb = priv->tx_skbs[priv->dirty_tx_desc];
		priv->stats.tx_packets++;
		if (skb) {
			/* check for transmission errors and update stats
			 * accordingly */
			if (report & (TX_FIRST_DEFERRAL | TX_LATE_COLLISION |
					TX_PACKET_DROPPED | TX_FIFO_UNDERRUN)) {
				priv->stats.tx_errors++;
			} else {
				priv->stats.tx_bytes += skb->len;
			}
			dev_kfree_skb(skb);
		}
		priv->tx_skbs[priv->dirty_tx_desc] = NULL;
		priv->dirty_tx_desc++;
		priv->dirty_tx_desc %= priv->tx_desc_count;
		priv->free_tx_desc_count++;
		priv->tx_report[priv->dirty_tx_desc] = 0;
	}

	if (priv->free_tx_desc_count && netif_queue_stopped(dev))
		netif_wake_queue(dev);

	spin_unlock_bh(&priv->tx_lock);
}

/*
 * our irq handler, just ack it and schedule the right tasklet to
 * handle this
 */
static irqreturn_t enet_isr(int irq, void *dev_id)
{
	struct net_device *dev;
	struct tangox_enet_priv *priv;
	unsigned long val = 0;

	dev = (struct net_device *)dev_id;
	priv = netdev_priv(dev);

	/* tx interrupt */
	if ((val = gbus_read_reg32(ENET_TXC_SR(priv->enet_mac_base))) != 0) {
		gbus_write_reg32(ENET_TXC_SR(priv->enet_mac_base), 0xf);
		//if (likely(val & TSR_DI)) {
		if (likely(val & TSR_TI)) {
			tasklet_schedule(&priv->tx_reclaim_tasklet);
		}
		if (unlikely(val & TSR_DE))
			printk("TX DMA error\n");
		if (unlikely(val & TSR_TO))
			printk("TX FIFO overflow\n");
	}
	/* rx interrupt */
	if ((val = gbus_read_reg32(ENET_RXC_SR(priv->enet_mac_base))) != 0) {
		gbus_write_reg32(ENET_RXC_SR(priv->enet_mac_base), 0xf);
		if (likely(val & (RSR_RI | RSR_DI | RSR_DE | RSR_RO))) {
			if (atomic_read(&priv->napi_poll) == 0) {
				if (napi_schedule_prep(&priv->napi)) {
					/*TODO: disable rx interrupt */
					/*avoid reentering */
					atomic_set(&priv->napi_poll, 1);
					__napi_schedule(&priv->napi);
				} 
			}
		}

		if (unlikely(val & RSR_DI)) 
			DBG("RX EOC\n");			
		if (unlikely(val & RSR_DE))
			DBG("RX DMA error\n");
		if (unlikely(val & RSR_RO)) {
			int i;
			DBG("RX Status FIFO overflow\n");
			for (i = 0; i < 4; i++)
				gbus_read_reg32(ENET_RX_FIFO_SR(priv->enet_mac_base));
		}
	}

 	/* wake on lan */
 	if ((val = gbus_read_reg8(ENET_WAKEUP(priv->enet_mac_base))) == 1) {
 		/* clear sleeping mode */
 		gbus_write_reg8(ENET_SLEEP_MODE(priv->enet_mac_base), 0);
 		/* clear wakeup mode */
 		gbus_write_reg8(ENET_WAKEUP(priv->enet_mac_base), 0);
 	}

	return IRQ_HANDLED;
}

static int enet_get_speed(struct net_device *dev)
{
	struct tangox_enet_priv *priv;
	int speed = SPEED_100;

	priv = netdev_priv(dev);
	if ((priv->rgmii == PHY_100) && (priv->no_mdio)) {
#ifdef CONFIG_TANGOX_ENET0_NOMDIO
			if (priv->enet_mac_base == REG_BASE_host_interface + 0x6000) 
				speed = CONFIG_TANGOX_ENET0_NOMDIO_SPEED;
#endif
#ifdef CONFIG_TANGOX_ENET1_NOMDIO
			if (priv->enet_mac_base == REG_BASE_host_interface + 0x6800) 
				speed = CONFIG_TANGOX_ENET1_NOMDIO_SPEED;
#endif
	} else {
		struct ethtool_cmd ecmd;
		mii_ethtool_gset(&priv->mii, &ecmd);
		speed = ecmd.speed;
	}
	return speed;
}

static int phy_autoneg(struct net_device *dev)
{
	struct tangox_enet_priv *priv;
	unsigned int val = 0, ctrl_1000 = 0;
	int mdio_ok = 0;
	unsigned long start_jiffies;
	unsigned short advertise =	0; 

#define MDIO_MAX_WAIT	5	/* in seconds */

	priv = netdev_priv(dev);

	/* see if complete already */
	if (enet_mdio_read(dev, priv->mii.phy_id, MII_BMSR) & BMSR_ANEGCOMPLETE)
		return 0;

	/*check BMSR to check 10/100 baseT supported */
	val = enet_mdio_read(dev, priv->mii.phy_id, MII_BMSR);
	if (val & BMSR_10HALF)
		advertise |=  ADVERTISE_10HALF; 
	if (val & BMSR_10FULL)
		advertise |=  ADVERTISE_10FULL;  
	if (val & BMSR_100HALF)
		advertise |=  ADVERTISE_100HALF;  
	if (val & BMSR_100FULL)
		advertise |=  ADVERTISE_100FULL; 

	/* check extension see if 1000 baseT supported */
	if ((priv->gigabit == 1) && (priv->rgmii != PHY_100)){
		if (val & BMSR_ESTATEN) {
			val = enet_mdio_read(dev, priv->mii.phy_id, MII_ESTATUS);
			ctrl_1000 = enet_mdio_read(dev, priv->mii.phy_id, MII_CTRL1000);
			if (val & ESTATUS_1000_TFULL)
				ctrl_1000 |= ADVERTISE_1000FULL;
			if (val & ESTATUS_1000_THALF)
				ctrl_1000 |= ADVERTISE_1000HALF;
			enet_mdio_write(dev, priv->mii.phy_id, MII_CTRL1000, ctrl_1000);
		}
	} else {
		val = enet_mdio_read(dev, priv->mii.phy_id, MII_ESTATUS);
		ctrl_1000 = enet_mdio_read(dev, priv->mii.phy_id, MII_CTRL1000);
		if (val & ESTATUS_1000_TFULL)
			ctrl_1000 &= ~ADVERTISE_1000FULL;
		if (val & ESTATUS_1000_THALF)
			ctrl_1000 &= ~ADVERTISE_1000HALF;
		enet_mdio_write(dev, priv->mii.phy_id, MII_CTRL1000, ctrl_1000);
	}

	/* program advertisement reg for autoneg */
	val = enet_mdio_read(dev, priv->mii.phy_id, MII_ADVERTISE);
	advertise |= val;
	enet_mdio_write(dev, priv->mii.phy_id, MII_ADVERTISE, advertise);

	/* power up, autoneg phy */
	priv->mii.force_media = 0;
	val = enet_mdio_read(dev, priv->mii.phy_id, MII_BMCR);
	val &= (~BMCR_PDOWN); 
	val |= BMCR_ANENABLE | BMCR_ANRESTART;
	enet_mdio_write(dev, priv->mii.phy_id, MII_BMCR, val);
	udelay(100);

	for (start_jiffies = jiffies; time_after(start_jiffies + MDIO_MAX_WAIT * HZ, jiffies);) {
		if (enet_mdio_read(dev, priv->mii.phy_id, MII_BMSR) & BMSR_ANEGCOMPLETE) {
			mdio_ok = 1;
			break;
		}
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(HZ / 10);
	}

	if (mdio_ok == 0) {
		printk(KERN_ERR "%s: PHY autonegotiation does not complete...\n", priv->name);
		return -EBUSY;
	}

	/* stabilize, ~0.5 second */
	set_current_state(TASK_INTERRUPTIBLE);
	schedule_timeout(HZ / 2);
	return 0;
}

static void enet_mac_config(struct net_device *dev)
{
	struct tangox_enet_priv *priv;
	int speed, val;

	priv = netdev_priv(dev);

	if (priv->rgmii == PHY_100) {
		/* 100 baseT, realtek*/
		val = gbus_read_reg8(ENET_MAC_MODE(priv->enet_mac_base));
		if (val & (GMAC_MODE | RGMII_MODE)) {
			val &= ~(GMAC_MODE | RGMII_MODE);	/*disable Gigabit mode for now*/
			//val |= /*LB_EN |*/ BST_EN;		/*loopback off, burst on*/
			gbus_write_reg8(ENET_MAC_MODE(priv->enet_mac_base), val);
		}
		return;
	}
	
	if (priv->rgmii == PHY_VITESSE) {
		/* Enable LED blink */
#if 0 
		/* simple LED Method */
		enet_mdio_write(dev, priv->mii.phy_id, VITESSE_LED_CTL, 0xf);
#else
		/* Enhanced LED Method */
		enet_mdio_write(dev, priv->mii.phy_id, VITESSE_EXTENDED_PAGE_CTL, 0x0001);
		enet_mdio_write(dev, priv->mii.phy_id, VITESSE_ENHANCED_LED_METHOD, 0x004a);
		enet_mdio_write(dev, priv->mii.phy_id, VITESSE_ENHANCED_LED_BEHAVIOR, 0x0c10);
		enet_mdio_write(dev, priv->mii.phy_id, VITESSE_EXTENDED_PAGE_CTL, 0x0000);
#endif
		/* Set RGMII skew timing compensation and ActiPHY mode */
		/* This added 2ns delay to RX_CLK and TX_CLK */
		val = enet_mdio_read(dev, priv->mii.phy_id, VITESSE_EXTENDED_PHY_CTL);
		enet_mdio_write(dev, priv->mii.phy_id, VITESSE_EXTENDED_PHY_CTL, val | (1 << 8) | (1 << 5));
	} 
	else if (priv->rgmii == PHY_AR8035) {
#if 0 
		/* enable regmii rx clock delay */
		enet_mdio_write(dev, priv->mii.phy_id, 0x1d, 0);
		val = enet_mdio_read(dev, priv->mii.phy_id, 0x1e) | (1<<15);
		enet_mdio_write(dev, priv->mii.phy_id, 0x1e, val);
#endif
		/* enable regmii tx clock delay */
		enet_mdio_write(dev, priv->mii.phy_id, 0x1d, 5);
		val = enet_mdio_read(dev, priv->mii.phy_id, 0x1e) | (1<<8);
		enet_mdio_write(dev, priv->mii.phy_id, 0x1e, val);

		/* LED control*/
		enet_mdio_write(dev, priv->mii.phy_id, 0x18, 0x2100);
	}

	val = gbus_read_reg8(ENET_MAC_MODE(priv->enet_mac_base));
	speed = enet_get_speed(dev);
	if (speed == 1000) {
		if (((val & RGMII_MODE) == 0) || ((val & GMAC_MODE) == 0)) {
			val |= (GMAC_MODE | RGMII_MODE);
			gbus_write_reg8(ENET_MAC_MODE(priv->enet_mac_base), val);
		}
	} else {
		if ((val & RGMII_MODE) || (val & GMAC_MODE)) {
			val &= ~(GMAC_MODE | RGMII_MODE);
			gbus_write_reg8(ENET_MAC_MODE(priv->enet_mac_base), val);
		}
	}

	/*set threshold for internal clock 0x1*/
	gbus_write_reg8(ENET_IC_THRESHOLD(priv->enet_mac_base), ((speed == 1000) ? 3 : 1));

	/*set slot time 0x7f for 10/100Mbps*/
	gbus_write_reg8(ENET_SLOT_TIME(priv->enet_mac_base), ((speed == 1000) ? 0xff : 0x7f));
}

/*
 * reconfigure mac for new link state
 */
static void enet_link_reconfigure(struct net_device *dev)
{
	struct tangox_enet_priv *priv;
	unsigned char val;

	priv = netdev_priv(dev);

	/* reflect duplex status in dma register */
	spin_lock_bh(&priv->maccr_lock);

	val = gbus_read_reg8(ENET_MAC_MODE(priv->enet_mac_base));
	if (priv->mii.full_duplex)
		val &= ~HALF_DUPLEX;
	else
		val |= HALF_DUPLEX;
	gbus_write_reg8(ENET_MAC_MODE(priv->enet_mac_base), val);

	enet_mac_config(dev);

	spin_unlock_bh(&priv->maccr_lock);
}

/*
 * link check timer callback
 */
static void enet_link_check(unsigned long data)
{
	struct net_device *dev;
	struct tangox_enet_priv *priv;
	int ret = 0, speed;
	static int old_speed, old_full_duplex;

	dev = (struct net_device *)data;
	priv = netdev_priv(dev);

	/*check speed change in gigabit*/
	speed = enet_get_speed(dev);
	
	if (priv->no_mdio == 0) {
		/* check for duplex change */
		spin_lock_bh(&priv->mii_lock);
		ret = mii_check_media(&priv->mii, 1, 0);
		spin_unlock_bh(&priv->mii_lock);
	}

	if (ret || (old_full_duplex != priv->mii.full_duplex) || (speed && (speed !=old_speed)))
		enet_link_reconfigure(dev);

	if (speed)
		old_speed = speed;

	old_full_duplex = priv->mii.full_duplex;

	/* reschedule timer */
	priv->link_check_timer.expires = jiffies + LINK_CHECK_TIMER_FREQ;
	add_timer(&priv->link_check_timer);
}

/*
 * program given mac address in hw registers
 */
static int enet_set_mac_address(struct net_device *dev, void *addr)
{
	struct sockaddr *sock = addr;
	struct tangox_enet_priv *priv = netdev_priv(dev);

	/* to make it safe, we won't do this while running */
	if (netif_running(dev))
		return -EBUSY;

	memcpy(dev->dev_addr, sock->sa_data, ETH_ALEN);

	/*set mac addr*/
	gbus_write_reg8(ENET_MAC_ADDR1(priv->enet_mac_base), dev->dev_addr[0]);
	gbus_write_reg8(ENET_MAC_ADDR2(priv->enet_mac_base), dev->dev_addr[1]);
	gbus_write_reg8(ENET_MAC_ADDR3(priv->enet_mac_base), dev->dev_addr[2]);
	gbus_write_reg8(ENET_MAC_ADDR4(priv->enet_mac_base), dev->dev_addr[3]);
	gbus_write_reg8(ENET_MAC_ADDR5(priv->enet_mac_base), dev->dev_addr[4]);
	gbus_write_reg8(ENET_MAC_ADDR6(priv->enet_mac_base), dev->dev_addr[5]);
	
	/* set unicast addr */
	gbus_write_reg8(ENET_UC_ADDR1(priv->enet_mac_base), dev->dev_addr[0]);
	gbus_write_reg8(ENET_UC_ADDR2(priv->enet_mac_base), dev->dev_addr[1]);
	gbus_write_reg8(ENET_UC_ADDR3(priv->enet_mac_base), dev->dev_addr[2]);
	gbus_write_reg8(ENET_UC_ADDR4(priv->enet_mac_base), dev->dev_addr[3]);
	gbus_write_reg8(ENET_UC_ADDR5(priv->enet_mac_base), dev->dev_addr[4]);
	gbus_write_reg8(ENET_UC_ADDR6(priv->enet_mac_base), dev->dev_addr[5]);

	return 0;
}

/*
 * update hash table to reflect new device multicast address list
 */
static void enet_set_rx_mode(struct net_device *dev)
{
#ifdef ENABLE_MULTICAST
	struct tangox_enet_priv *priv;
	unsigned char val;
	struct netdev_hw_addr *ha;

	priv = netdev_priv(dev);

	/* check if the link is ok, otherwise just return */
	if (priv->no_mdio == 0) {
		spin_lock_bh(&priv->mii_lock);
		val = mii_link_ok(&priv->mii);
		spin_unlock_bh(&priv->mii_lock);

		if (val == 0)
			return;
	}

	/* the link check timer might change RX control, we need to protect
	 * against it */
	spin_lock_bh(&priv->maccr_lock);
	val = gbus_read_reg32(ENET_RX_CTL(priv->enet_mac_base));

	if (dev->flags & IFF_PROMISC) {
		val &= ~(RX_BC_DISABLE | RX_AF_EN);
	} else {
		val |= RX_AF_EN;
		/* if we want all multicast or if address count is too
		 * high, don't try to compute hash value */
		if ((netdev_mc_count(dev) > 64) || (dev->flags & IFF_ALLMULTI))
			val &= ~(RX_BC_DISABLE | RX_AF_EN);
	}

	gbus_write_reg32(ENET_RX_CTL(priv->enet_mac_base), val);
	spin_unlock_bh(&priv->maccr_lock);

	/* we don't need to update hash table if we pass all multicast */
	if (((val & RX_BC_DISABLE) == 0) && ((val & RX_AF_EN) == 0))
		return;

	/* clear internal multicast address table */
	gbus_write_reg8(ENET_MC_INIT(priv->enet_mac_base), 0x0);
	while (gbus_read_reg8(ENET_MC_INIT(priv->enet_mac_base)))
		;

	netdev_for_each_mc_addr(ha, dev) {
		char *addr = ha->addr;

		if ((*addr & 1) == 0)
			continue;

		gbus_write_reg8(ENET_MC_ADDR1(priv->enet_mac_base), addr[0]);
		gbus_write_reg8(ENET_MC_ADDR2(priv->enet_mac_base), addr[1]);
		gbus_write_reg8(ENET_MC_ADDR3(priv->enet_mac_base), addr[2]);
		gbus_write_reg8(ENET_MC_ADDR4(priv->enet_mac_base), addr[3]);
		gbus_write_reg8(ENET_MC_ADDR5(priv->enet_mac_base), addr[4]);
		gbus_write_reg8(ENET_MC_ADDR6(priv->enet_mac_base), addr[5]);
		gbus_write_reg8(ENET_MC_INIT(priv->enet_mac_base), 0xff);
		while (gbus_read_reg8(ENET_MC_INIT(priv->enet_mac_base)))
			;
	}
#endif
}

static void enet_dma_reinit(struct net_device *dev)
{
	struct tangox_enet_priv *priv = netdev_priv(dev);

	priv->pending_tx = -1;
	priv->pending_tx_cnt = 0;
	priv->reclaim_limit = -1;
	priv->dirty_tx_desc = priv->next_tx_desc = 0;
	priv->free_tx_desc_count = priv->tx_desc_count;

	priv->last_rx_desc = 0;
	priv->rx_eoc = priv->rx_desc_count - 1;
	rearm_rx_descs(dev);
	wmb();

	/*
	 * write rx desc list & tx desc list addresses in registers
	 */
	gbus_write_reg32(ENET_TX_DESC_ADDR(priv->enet_mac_base), DMA_ADDR((void *)&priv->tx_descs[0]));
	gbus_write_reg32(ENET_RX_DESC_ADDR(priv->enet_mac_base), DMA_ADDR((void *)&priv->rx_descs[0]));
}

/*
 * open callback
 */
static int enet_open(struct net_device *dev)
{
	struct tangox_enet_priv *priv;

	priv = netdev_priv(dev);

	/* power up the PHY*/
	if ((priv->no_mdio == 0)) 
		phy_autoneg(dev);

	/* check link */
	if (priv->no_mdio || mii_check_media(&priv->mii, 0, 1))
		enet_link_reconfigure(dev);

	/*clear rx interrupts*/
	gbus_write_reg32(ENET_RXC_SR(priv->enet_mac_base), 0xf);

	/* start rx dma*/
	atomic_set(&priv->napi_poll, 0);
	enet_dma_reinit(dev);
	enet_start_rx(dev);

	/* enable mac rx */
	enet_mac_rx(dev, 1);

	/* stabilize the rx*/
	mdelay(500);

	/* start link check & tx reclaim timer */
	priv->link_check_timer.expires = jiffies + LINK_CHECK_TIMER_FREQ;
	add_timer(&priv->link_check_timer);

	//priv->tx_reclaim_timer.expires = jiffies + TX_RECLAIM_TIMER_FREQ;
	//add_timer(&priv->tx_reclaim_timer);

	/*clear tx interrupts*/
	gbus_write_reg32(ENET_TXC_SR(priv->enet_mac_base), 0xf);

	/* enable mac tx */
	enet_mac_tx(dev, 1);

	/*finally start tx */
	netif_start_queue(dev);

	/* enable napi */
	napi_enable(&priv->napi);

	return 0;
}

/*
 * stop callback
 */
static int enet_stop(struct net_device *dev)
{
	struct tangox_enet_priv *priv;
	unsigned char val;

	priv = netdev_priv(dev);

	/* stop link timer */
	del_timer_sync(&priv->link_check_timer);

	/* stop dma */
	enet_stop_rx(dev);

	/* stop mac rx */
	enet_mac_rx(dev, 0);

	/* stop tx queue */
	netif_stop_queue(dev);

	/* disable napi */
	napi_disable(&priv->napi);

	/* stop mac tx */
	enet_mac_tx(dev, 0);

	/* power down the PHY*/
	val = enet_mdio_read(dev, priv->mii.phy_id, MII_BMCR);
	enet_mdio_write(dev, priv->mii.phy_id, MII_BMCR, val | BMCR_PDOWN);

	return 0;
}

/*
 * get_stats callback
 */
static struct net_device_stats *enet_get_stats(struct net_device *dev)
{
	struct tangox_enet_priv *priv;
	priv = netdev_priv(dev);
	return &priv->stats;
}

/*
 * ethtool callbacks
 */
static int enet_get_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct tangox_enet_priv *priv;
	int ret;

	priv = netdev_priv(dev);

	spin_lock_bh(&priv->mii_lock);
	ret = mii_ethtool_gset(&priv->mii, cmd);
	spin_unlock_bh(&priv->mii_lock);

	return ret;
}

static int enet_set_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct tangox_enet_priv *priv;
	int ret;

	priv = netdev_priv(dev);

	spin_lock_bh(&priv->mii_lock);
	ret = mii_ethtool_sset(&priv->mii, cmd);
	spin_unlock_bh(&priv->mii_lock);

	return ret;
}

static int enet_nway_reset(struct net_device *dev)
{
	struct tangox_enet_priv *priv;
	int ret;

	priv = netdev_priv(dev);

	spin_lock_bh(&priv->mii_lock);
	ret = mii_nway_restart(&priv->mii);
	spin_unlock_bh(&priv->mii_lock);

	return ret;
}

static u32 enet_get_link(struct net_device *dev)
{
	struct tangox_enet_priv *priv;
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
	struct tangox_enet_priv *priv;
	int ret;

	priv = netdev_priv(dev);

	spin_lock_bh(&priv->mii_lock);
	ret = generic_mii_ioctl(&priv->mii, if_mii(rq), cmd, NULL);
	spin_unlock_bh(&priv->mii_lock);

	return ret;
}

/*
 * dma ring allocation is done here
 */
static int enet_dma_init(struct net_device *dev)
{
#define CEILING(x, c) ((((x) / (1 << (c))) + (((x) % (1 << (c))) ? 1 : 0)) * (1 << (c)))
	unsigned int r_sz, alloc_size, alloc_order;
	int i;
	void *tx_ptr;
	struct tangox_enet_priv *priv = netdev_priv(dev);
	
	/* calculate the size needed */
	alloc_size = CEILING(priv->rx_desc_count * sizeof(struct enet_desc), L1_CACHE_SHIFT) + 
			CEILING(priv->rx_desc_count * sizeof(unsigned long), L1_CACHE_SHIFT) +
			CEILING(priv->rx_desc_count * sizeof(struct skb_buff *), L1_CACHE_SHIFT) + 
			CEILING(priv->tx_desc_count * sizeof(struct enet_desc), L1_CACHE_SHIFT) + 
			CEILING(priv->tx_desc_count * sizeof(unsigned long), L1_CACHE_SHIFT) +
			CEILING(priv->tx_desc_count * sizeof(struct skb_buff *), L1_CACHE_SHIFT) +
			CEILING(priv->tx_desc_count * sizeof(char *), L1_CACHE_SHIFT) +
			(priv->tx_desc_count * CEILING(data_aligned, L1_CACHE_SHIFT));
			
	for (alloc_order = 0; (PAGE_SIZE << alloc_order) < alloc_size; alloc_order++)
		;
	if ((priv->alloc_pages_cached = (void *)__get_free_pages(GFP_KERNEL | GFP_DMA, alloc_order)) == NULL) {
		printk("%s: cannot allocate memory.\n", priv->name);
		return -ENOMEM;
	}
	memset(priv->alloc_pages_cached, 0, alloc_size);
	dma_cache_wback_inv((unsigned long)priv->alloc_pages_cached, alloc_size);
	priv->alloc_pages = (void *)KSEG1ADDR(priv->alloc_pages_cached);
	priv->alloc_order = alloc_order;

	/* arrange rx */
	priv->rx_descs = (void *)priv->alloc_pages;
	priv->rx_report = ((void *)priv->rx_descs) + CEILING(priv->rx_desc_count * sizeof(struct enet_desc), L1_CACHE_SHIFT);
	priv->rx_skbs = ((void *)priv->rx_report) + CEILING(priv->rx_desc_count * sizeof(unsigned long), L1_CACHE_SHIFT);

	/*
	 * initialize all rx descs
	 */
	for (i = 0; i < priv->rx_desc_count; i++) {
		volatile struct enet_desc *rx;
		struct sk_buff *skb;

		rx = &priv->rx_descs[i];
		rx->config = RX_BUF_SIZE | DESC_BTS(2) | DESC_EOF/* | DESC_ID*/;

		if ((skb = dev_alloc_skb(RX_BUF_SIZE + SKB_RESERVE_SIZE)) == NULL)
			return -ENOMEM;
		dma_cache_inv((unsigned long)skb->data, RX_BUF_SIZE + SKB_RESERVE_SIZE);
		
		r_sz = (data_aligned - ((u32)skb->data & (data_aligned - 1))) % data_aligned;
		if (r_sz)
			skb_reserve(skb, r_sz); /* make sure it's aligned to pre-defined boundary */
		priv->rx_report[i] = 0; 
		rx->s_addr = DMA_ADDR((void *)skb->data);
		rx->r_addr = DMA_ADDR((void *)&priv->rx_report[i]);
		rx->n_addr = DMA_ADDR((void *)&priv->rx_descs[i + 1]);
		if (i == (priv->rx_desc_count - 1)) {
			rx->n_addr = DMA_ADDR((void *)&priv->rx_descs[0]);
			rx->config |= DESC_EOC;
			priv->rx_eoc = i;
		}
#ifdef ETH_DEBUG
		DBG("rx[%d]=0x%08x\n", i, (unsigned int)rx);
		DBG("  s_addr=0x%08x\n", (unsigned int)rx->s_addr);
		DBG("  n_addr=0x%08x\n", (unsigned int)rx->n_addr);
		DBG("  r_addr=0x%08x\n", (unsigned int)rx->r_addr);
		DBG("  config=0x%08x\n", (unsigned int)rx->config);
#endif
		priv->rx_skbs[i] = skb;
	}
	priv->last_rx_desc = 0;
	wmb();

	/*
	 * allocate tx descriptor list
	 *
	 * We allocate only the descriptor list and prepare them for
	 * further use. When tx is needed, we will set the right flags
	 * and kick the dma.
	 */
	priv->tx_descs = ((void *)priv->rx_skbs) + CEILING(priv->rx_desc_count * sizeof(struct skb_buff *), L1_CACHE_SHIFT);
	priv->tx_report = ((void *)priv->tx_descs) + CEILING(priv->tx_desc_count * sizeof(struct enet_desc), L1_CACHE_SHIFT);
	priv->tx_skbs = ((void *)priv->tx_report) + CEILING(priv->tx_desc_count * sizeof(unsigned long), L1_CACHE_SHIFT);
	priv->tx_bufs = ((void *)priv->tx_skbs) + CEILING(priv->tx_desc_count * sizeof(struct skb_buff *), L1_CACHE_SHIFT);
	tx_ptr = ((void *)priv->tx_bufs) + CEILING(priv->tx_desc_count * sizeof(char *), L1_CACHE_SHIFT); 

	/*
	 * initialize tx descs
	 */
	for (i = 0; i < priv->tx_desc_count; i++) {
		volatile struct enet_desc *tx;

		priv->tx_bufs[i] = tx_ptr + (i * CEILING(data_aligned, L1_CACHE_SHIFT));
		tx = &priv->tx_descs[i];
		priv->tx_report[i] = 0; 
		tx->r_addr = DMA_ADDR((void *)&priv->tx_report[i]);
		tx->s_addr = 0;
		tx->config = DESC_EOF;
		if (i == (priv->tx_desc_count - 1)) {
			tx->config |= DESC_EOC;
			tx->n_addr = DMA_ADDR((void *)&priv->tx_descs[0]);
		}
		//DBG("tx[%d]=0x%08x\n", i, (unsigned int)tx);
	}
	priv->dirty_tx_desc = priv->next_tx_desc = 0;
	priv->pending_tx = -1;
	priv->pending_tx_cnt = 0;
	priv->reclaim_limit = -1;
	priv->free_tx_desc_count = priv->tx_desc_count;
	wmb();

	/*
	 * write rx desc list & tx desc list addresses in registers
	 */
	gbus_write_reg32(ENET_TX_DESC_ADDR(priv->enet_mac_base), DMA_ADDR((void *)&priv->tx_descs[0]));
	gbus_write_reg32(ENET_RX_DESC_ADDR(priv->enet_mac_base), DMA_ADDR((void *)&priv->rx_descs[0]));
	return 0;
}

/*
 * free all dma rings memory, called at uninit time or when error
 * occurs at init time
 */
static void enet_dma_free(struct tangox_enet_priv *priv)
{
	int i;

	if (priv->alloc_pages_cached == NULL)
		return;

	/* note: kfree_skb(NULL) is _not_ ok */
	for (i = 0; i < priv->rx_desc_count; i++) {
		if (priv->rx_skbs[i]) 
			kfree_skb(priv->rx_skbs[i]);
	}

	for (i = 0; i < priv->tx_desc_count; i++) {
		if (priv->tx_skbs[i]) 
			kfree_skb(priv->tx_skbs[i]);
	}

	free_pages((u32)priv->alloc_pages_cached, priv->alloc_order);
}

static int phy_reset(struct net_device *dev)
{
	struct tangox_enet_priv *priv;
	int loop;
	unsigned int val = 0;

	priv = netdev_priv(dev);

	/* reset phy */
	val = enet_mdio_read(dev, priv->mii.phy_id, MII_BMCR);
	enet_mdio_write(dev, priv->mii.phy_id, MII_BMCR, val | BMCR_RESET);

	/* wait for the reset bit to clear */
	udelay(100);
	loop = 100;
	while (loop) {
		if ((enet_mdio_read(dev, priv->mii.phy_id, MII_BMCR) & BMCR_RESET) == 0)
			break;
		mdelay(1);
		loop--;
	}

	if (loop == 0) {
		printk(KERN_ERR "%s: PHY reset does not complete...\n", priv->name);
		return -EBUSY;
	}

	return 0;
}

/*
 * mac hw init is done here
 */
static int enet_hw_init(struct net_device *dev)
{
	struct tangox_enet_priv *priv;
	unsigned int val = 0;

	priv = netdev_priv(dev);
	
	if (priv->no_mdio == 0) {
		if (phy_reset(dev))
			return -EBUSY;
	}

	/* set pad_mode according to rgmii or not*/
	val = gbus_read_reg8(priv->enet_mac_base + 0x400) & 0xf0;
	if (priv->rgmii != PHY_100)
		gbus_write_reg8(priv->enet_mac_base + 0x400, val | 0x01);

	/* software reset IP */
	gbus_write_reg8(priv->enet_mac_base + 0x424, 0);
	udelay(10);
	gbus_write_reg8(priv->enet_mac_base + 0x424, 1);

	/*set threshold for internal clock 0x1*/
	gbus_write_reg8(ENET_IC_THRESHOLD(priv->enet_mac_base), 1);

	/*set Random seed 0x8*/
	gbus_write_reg8(ENET_RANDOM_SEED(priv->enet_mac_base), 0x08);

	/*set TX single deferral params 0xc*/
	gbus_write_reg8(ENET_TX_SDP(priv->enet_mac_base), 0xc);

	/*set slot time 0x7f for 10/100Mbps*/
	gbus_write_reg8(ENET_SLOT_TIME(priv->enet_mac_base), 0x7f);

	/*set Threshold for partial full 0xff */
	gbus_write_reg8(ENET_PF_THRESHOLD(priv->enet_mac_base), 0xff);

	/* set Pause Quanta 65535 */
	gbus_write_reg8(ENET_PQ1(priv->enet_mac_base), 0xff);
	gbus_write_reg8(ENET_PQ2(priv->enet_mac_base), 0xff);

	/* configure TX DMA Channels */
	val = (TCR_RS | TCR_LE | TCR_TFI(1) | /*TCR_DIE |*/ TCR_BTS(2) | TCR_DM);
	gbus_write_reg32(ENET_TXC_CR(priv->enet_mac_base), val);

	/* TX Interrupt Time Register */
	val = (gbus_read_reg32(ENET_TXC_CR(priv->enet_mac_base)) >> 4) & 0x7;
	val *= (TX_BUF_SIZE * (tangox_get_sysclock()/(priv->gigabit ? 125000000 : 25000000) + 2));
	gbus_write_reg32(ENET_TX_ITR(priv->enet_mac_base), val);

 	/* configure RX DMA Channels */
	val = (RCR_RS | RCR_LE | RCR_RFI(7) | RCR_BTS(2) | RCR_FI | RCR_DIE | /*RCR_EN |*/ RCR_DM);
	val |= (RX_BUF_SIZE << 16);
	gbus_write_reg32(ENET_RXC_CR(priv->enet_mac_base), val); 

	/* RX Interrupt Time Register */
	val = (gbus_read_reg32(ENET_RXC_CR(priv->enet_mac_base)) >> 4) & 0x7;
	val *= (RX_BUF_SIZE * (tangox_get_sysclock()/(priv->gigabit ? 125000000 : 25000000) + 2));
	gbus_write_reg32(ENET_RX_ITR(priv->enet_mac_base), val);

	/* configure MAC controller */
	val = (TX_RETRY_EN | TX_PAD_EN | TX_APPEND_FCS);
	gbus_write_reg8(ENET_TX_CTL1(priv->enet_mac_base), (unsigned char)val);

	/* set retry 5 time when collision occurs */
	gbus_write_reg8(ENET_TX_CTL2(priv->enet_mac_base), 5);

	val = (RX_RUNT | RX_PAD_STRIP /*| RX_SEND_CRC */ | RX_PAUSE_EN | RX_AF_EN);
	gbus_write_reg8(ENET_RX_CTL(priv->enet_mac_base), (unsigned char)val);

#ifdef ENABLE_MULTICAST
	/* clear internal multicast address table */
	gbus_write_reg8(ENET_MC_INIT(priv->enet_mac_base), 0x00);
	while (gbus_read_reg8(ENET_MC_INIT(priv->enet_mac_base)))
		;
	DBG("Internal multicast address table is cleared\n");
#endif

	/* unicast */
	/* Threshold for internal clock*/
	/* threshold for partial empty*/
	/* threshold for partial full */

	/* buffer size for transmit must be 1 from the doc
	   however, it's said that using 0xff ??*/
	gbus_write_reg8(ENET_TX_BUFSIZE(priv->enet_mac_base), 0xff);

	/* fifo control */

	/*MAC mode*/
	enet_mac_config(dev);

	/* check gmii mode support */
	priv->mii.supports_gmii = mii_check_gmii_support(&priv->mii);
	DBG("gmii support=0x%x id=0x%x\n", priv->mii.supports_gmii, priv->mii.phy_id);

	return 0;
}

static const struct net_device_ops tangox_netdev_ops = {
	.ndo_open = enet_open,
	.ndo_stop = enet_stop,
	.ndo_start_xmit = enet_xmit,
	.ndo_get_stats = enet_get_stats,
	.ndo_set_mac_address = enet_set_mac_address,
	.ndo_set_rx_mode = enet_set_rx_mode,
	.ndo_do_ioctl = enet_ioctl,
	.ndo_change_mtu = eth_change_mtu,
	.ndo_validate_addr = eth_validate_addr,
};

/*
 * allocate netdevice structure, do all dma rings allocations and
 * register the netdevice
 */
extern int tangox_ethernet_getmac(int, unsigned char *);

static int enet_probe(int idx)
{
#define MAX_MDC_CLOCK	2500000
	struct tangox_enet_priv *priv;
	struct net_device *dev;
	int ret;
	struct sockaddr sock;
	char pad_mode;
	unsigned short clk_div;
	unsigned long enet_mac_base;
	const char *name;
	unsigned long tangox_chip_id(void);
	unsigned long chip_id = tangox_chip_id();
	unsigned long sysfreq = tangox_get_sysclock();
	unsigned int arb = 0;

	enet_mac_base = eth_mac_cores[idx].enet_mac_base;
	name = eth_mac_cores[idx].name;

	/* set pad_mode*/
	pad_mode = gbus_read_reg8(enet_mac_base + 0x400);
	gbus_write_reg8(enet_mac_base + 0x400, pad_mode & 0xf0);
	pad_mode = gbus_read_reg8(enet_mac_base + 0x400);

	/* set MDIO clock divider */
	//clk_div = gbus_read_reg16(enet_mac_base + 0x420);
	//DBG("default clk_div =%d\n", clk_div);
	//gbus_write_reg16(enet_mac_base + 0x420, 50);
	clk_div = sysfreq / (MAX_MDC_CLOCK * 2);
	if ((clk_div * MAX_MDC_CLOCK * 2) < sysfreq)
		clk_div++;
	gbus_write_reg16(enet_mac_base + 0x420, clk_div);
	//clk_div = gbus_read_reg16(enet_mac_base + 0x420);
	//DBG("clk_div =%d: set MDIO clock=200/%d=%dMHz\n", clk_div, clk_div, 200/(clk_div*2));

	/* allocate netdevice structure with enough length for our
	 * context data */
	if ((dev = alloc_etherdev(sizeof(*priv))) == NULL) {
		ret = -ENOMEM;
		goto err_out;
	}

	dev->netdev_ops = &tangox_netdev_ops;

	/* initialize private data */
	priv = netdev_priv(dev);
	memset(priv, 0, sizeof(*priv));
	priv->enet_mac_base = enet_mac_base;
	priv->name = name;
	priv->pending_tx = -1;
	priv->pending_tx_cnt = 0;
	priv->reclaim_limit = -1;
	priv->rx_desc_count = DEF_RX_DESC_COUNT;
	priv->tx_desc_count = DEF_TX_DESC_COUNT;
	spin_lock_init(&priv->tx_lock);
	spin_lock_init(&priv->ier_lock);
	spin_lock_init(&priv->maccr_lock);

	/* check gigabit mode */
	if (((chip_id >> 16) & 0xfffe) == 0x8656) {
		if (idx == 0) { /* only port 0 */
			priv->gigabit = 1;
			priv->rx_desc_count *= 2;
			priv->tx_desc_count *= 2;
		}
	} else if ((chip_id & 0xfffe00ff) == 0x86460002) {
		priv->gigabit = 1;
		priv->rx_desc_count *= 2;
		priv->tx_desc_count *= 2;
	} else if (((chip_id >> 16) & 0xfff0) == 0x8670) {
		priv->gigabit = 1;
		priv->rx_desc_count *= 2;
		priv->tx_desc_count *= 2;
		data_aligned = 4;
		if (((chip_id >> 16) & 0x000e) != 0) /* 8672/8674 */
			arb = ((125 * 32) / (sysfreq / 1000000)) + 1;
	} else if (((chip_id >> 16) & 0xff00) == 0x8900) {
		priv->gigabit = 1;
		priv->rx_desc_count *= 2;
		priv->tx_desc_count *= 2;
		data_aligned = 4;
		arb = ((125 * 32) / (sysfreq / 1000000)) + 1;
	}

	/* init tx done tasklet */
	tasklet_init(&priv->tx_reclaim_tasklet, enet_tx_reclaim, (unsigned long)dev);
#if 0
	/* init tx reclaim timer */
	init_timer(&priv->tx_reclaim_timer);
	priv->tx_reclaim_timer.data = (unsigned long)dev;
	priv->tx_reclaim_timer.function = enet_tx_reclaim_timer;
#endif
	/* init link check timer and mii lock */
	init_timer(&priv->link_check_timer);
	priv->link_check_timer.data = (unsigned long)dev;
	priv->link_check_timer.function = enet_link_check;
	spin_lock_init(&priv->mii_lock);

	atomic_set(&priv->napi_poll, 0);

	/* fill mii info */
	priv->mii.dev = dev;
	priv->mii.phy_id_mask = 0x1f;
	priv->mii.reg_num_mask = 0x1f;
	priv->mii.mdio_read = enet_mdio_read;
	priv->mii.mdio_write = enet_mdio_write;
	
	if (eth_mac_cores[idx].phy_id != -1) {
		/* phy id forced, just check for sanity */
		if (eth_mac_cores[idx].phy_id < 0 || eth_mac_cores[idx].phy_id > 31) {
			ret = -EINVAL;
			goto err_free_netdev;
		}
		priv->mii.phy_id = eth_mac_cores[idx].phy_id;

	} else {
		int i;

#ifdef CONFIG_TANGOX_ENET0_NOMDIO
		if (idx == 0) {
			priv->no_mdio = 1;
#ifdef CONFIG_TANGOX_ENET0_NOMDIO_FULLDUPLEX
			priv->mii.full_duplex = 1;
#else
			priv->mii.full_duplex = 0;
#endif
			priv->mii.phy_id = CONFIG_TANGOX_ENET0_NOMDIO_PHY_ADDR;
			goto no_mdio;
		}
#endif

#ifdef CONFIG_TANGOX_ENET1_NOMDIO
		if (idx == 1) {
			priv->no_mdio = 1;
#ifdef CONFIG_TANGOX_ENET1_NOMDIO_FULLDUPLEX
			priv->mii.full_duplex = 1;
#else
			priv->mii.full_duplex = 0;
#endif
			priv->mii.phy_id = CONFIG_TANGOX_ENET1_NOMDIO_PHY_ADDR;
			goto no_mdio;
		}
#endif

		/* try to probe phy if not given */
		for (i = 0; i < 32; i++) {
			uint32_t id;
			int val;

			val = enet_mdio_read(dev, i, MII_PHYSID1);
			id = (val << 16);
			val = enet_mdio_read(dev, i, MII_PHYSID2);
			id |= val;
			if ((id != 0xffffffff) && (id != 0x00000000)) {
				/* check PHY*/
				switch (id) {
				case 0x00070421:
					priv->rgmii = PHY_VITESSE;
					break;
				case 0x4dd072:
					priv->rgmii = PHY_AR8035;
					break;
				default:
					priv->rgmii = PHY_100;
					priv->gigabit = 0;
					priv->rx_desc_count = DEF_RX_DESC_COUNT;
					priv->tx_desc_count = DEF_TX_DESC_COUNT;
					break;
				}
				break;
			}
		}

		if (i == 32) {
			printk(KERN_ERR "%s: unable to autodetect phy\n", priv->name);
			ret = -EIO;
			goto err_free_netdev;
		}
		priv->mii.phy_id = i;

#if defined(CONFIG_TANGOX_ENET0_NOMDIO) || defined(CONFIG_TANGOX_ENET1_NOMDIO)
no_mdio:
#endif
		if (priv->no_mdio)
			printk(KERN_ERR "%s: MII interface is enabled on core %d\n", 
					name,idx);
		else
			printk(KERN_ERR "%s: detected phy %s at address 0x%02x\n", 
					name, (priv->rgmii == PHY_VITESSE ? "vsc8601 " : (priv->rgmii == PHY_AR8035 ? "AR8035 " : "")), priv->mii.phy_id);
	}

	printk(KERN_INFO "%s: Ethernet driver for SMP8xxx internal MAC core %d: %s Base at 0x%lx\n",
				 name, idx, priv->rgmii?"1000Mbps":"100Mbps", enet_mac_base);

	/* initialize hardware */
	if ((ret = enet_hw_init(dev)))
		goto err_free_netdev;

	/* initialize dma descriptors */
	if ((ret = enet_dma_init(dev)))
		goto err_free_dma;

//	printk("descs ga: 0x%lx, reports ga: 0x%lx, priv ga/sz: 0x%lx/%d, alloc_pages va: 0x%p, order: %d\n", DMA_ADDR((void *)priv->rx_descs),
//			DMA_ADDR((void *)priv->rx_report), DMA_ADDR((void *)priv), sizeof(*priv), priv->alloc_pages, priv->alloc_order);

	SET_ETHTOOL_OPS(dev, &enet_ethtool_ops);
	netif_napi_add(dev, &priv->napi, enet_poll, priv->rx_desc_count/2 + 1);

	dev->tx_queue_len = priv->tx_desc_count;
#ifdef ENABLE_MULTICAST
	dev->flags |= IFF_MULTICAST;
#else	
	dev->flags &= ~IFF_MULTICAST;
#endif

	/* set default mac address */
	tangox_ethernet_getmac(idx, dev->dev_addr);
	memcpy(&(sock.sa_data), dev->dev_addr, ETH_ALEN);

	enet_set_mac_address(dev, &sock);

	if ((ret = register_netdev(dev))) {
		printk(KERN_ERR "%s: unable to register netdevice\n", priv->name);
		goto err_free_dma;
	}

	ret = request_irq(eth_mac_cores[idx].irq, enet_isr, IRQF_DISABLED,
				eth_mac_cores[idx].name, dev);
	if (ret)
		goto err_free_dma;

	dev->irq = eth_mac_cores[idx].irq;

	printk(KERN_INFO "%s: mac address %02x:%02x:%02x:%02x:%02x:%02x\n", priv->name,
			dev->dev_addr[0], dev->dev_addr[1], dev->dev_addr[2],
			dev->dev_addr[3], dev->dev_addr[4], dev->dev_addr[5]);

	eth_mac_cores[idx].gdev = dev;

	/* increase gbus bandwidth */
	gbus_write_reg32(REG_BASE_system_block + 0x130 + (idx * 4),
		(gbus_read_reg32(REG_BASE_system_block + 0x130 + (idx * 4)) & 0xffffff00) | 0x3f);

	/* increase mbus bandwidth */
	if (arb != 0) {
		if (arb > 0x7f)
			arb = 0x7f;
		gbus_write_reg32(REG_BASE_system_block + 0x218 + (idx * 8), 
			(gbus_read_reg32(REG_BASE_system_block + 0x218 + (idx * 8)) & 0xffff0000) | (arb << 8) | arb);
		gbus_write_reg32(REG_BASE_system_block + 0x21c + (idx * 8), 
			(gbus_read_reg32(REG_BASE_system_block + 0x21c + (idx * 8)) & 0xffff0000) | (arb << 8) | arb);
	}

	return 0;

	free_irq(eth_mac_cores[idx].irq, dev);
err_free_dma:
	enet_dma_free(priv);
err_free_netdev:
	free_netdev(dev);
err_out:
	return ret;
}

/*
 * entry point, checks if ethernet is enabled on the board and if so,
 * probes it
 */
extern int tangox_ethernet_enabled(int);

int __init tangox_enet_init(void)
{
	int i;

	/* for command line overwrite */
	eth_mac_cores[0].phy_id = phyid_0;
	eth_mac_cores[1].phy_id = phyid_1;

	for (i = 0; i < NUM_MAC_CORES; i++) {
		if ((1 << i) & mac_cores) {
			if (tangox_ethernet_enabled(i) == 0) {
				printk(KERN_NOTICE "%s: ethernet mac_core %d support is disabled from XENV\n", eth_mac_cores[i].name, i);
				continue;
			}
			enet_probe(i);
		} else {
			printk(KERN_NOTICE "%s: ethernet mac_core %d support is disabled\n", eth_mac_cores[i].name, i);
		}
	}
	return 0;
}

/*
 * exit func, stops hardware and unregisters netdevice
 */
void __exit tangox_enet_exit(void)
{
	struct tangox_enet_priv *priv;
	struct net_device *dev;
	int i;

	for (i = 0; i < NUM_MAC_CORES; i++) {
		if ((dev = eth_mac_cores[i].gdev) == NULL)
			continue;

		free_irq(dev->irq, dev);
		unregister_netdev(dev);

		priv = netdev_priv(dev);
		enet_dma_free(priv);

		free_netdev(dev);
	}
}

module_init(tangox_enet_init);
module_exit(tangox_enet_exit);

MODULE_DESCRIPTION("SMP8xxx internal ethernet mac driver");
MODULE_AUTHOR("TANGOX standalone team");
MODULE_LICENSE("GPL");

MODULE_PARM_DESC(phyid_0, "PHY id for core 0, else autodetect");
module_param(phyid_0, int, 0);

MODULE_PARM_DESC(phyid_1, "PHY id for core 1, else autodetect");
module_param(phyid_1, int, 0);

MODULE_PARM_DESC(mac_cores, "MAC core id, 1 for core 0, 2 for core 1, 3 for both");
module_param(mac_cores, int, 0);

