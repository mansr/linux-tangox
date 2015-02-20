/*********************************************************************
 Copyright (C) 2001-2008
 Sigma Designs, Inc. 

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/
/*
 * Driver for tangox SMP864x/SMP865x builtin Ethernet Mac.
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

#include <linux/config.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/etherdevice.h>
#include <linux/delay.h>
#include <linux/ethtool.h>
#include <linux/crc32.h>

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

/* for tx desc chaining*/
#undef ENABLE_TX_CHAINING

/* how many MAC cores we have */
#define NUM_MAC_CORES	2

struct eth_mac_core {
	unsigned long enet_mac_base;
	int phy_id;
	const int irq;
	struct net_device *gdev;
	const char *name;
};

static struct eth_mac_core eth_mac_cores[NUM_MAC_CORES] = {
	{ REG_BASE_host_interface + 0x6000, -1, ENET_IRQ0, NULL, "tangox_enet0" },
	{ REG_BASE_host_interface + 0x6800, -1, ENET_IRQ1, NULL, "tangox_enet1" }
};

static int mac_cores = 3; /* enabled both cores */
static int phyid_0 = -1;
static int phyid_1 = -1;

static __inline void enet_start_rx(struct tangox_enet_priv *priv);
static __inline void enet_stop_rx(struct tangox_enet_priv *priv);

/*
 * mdio read/write callback, can run from userspace or timer
 */

#define MDIO_TIMEOUT	1000

static __inline int enet_mdio_read(struct net_device *dev, int phy_id,
				   int location)
{
	int val, i;
	struct tangox_enet_priv *priv = netdev_priv(dev);

	for (i = 0; (i < MDIO_TIMEOUT) && (enet_readl(ENET_MDIO_CMD1(priv->enet_mac_base)) & MDIO_CMD_GO); i++)
		udelay(1);
	if (i >= MDIO_TIMEOUT)
		goto err_out;

	val = MIIAR_ADDR(phy_id) | MIIAR_REG(location);
	enet_writel(ENET_MDIO_CMD1(priv->enet_mac_base), val);

	udelay(10);

	enet_writel(ENET_MDIO_CMD1(priv->enet_mac_base), val | MDIO_CMD_GO);

	for (i = 0; (i < MDIO_TIMEOUT) && (enet_readl(ENET_MDIO_CMD1(priv->enet_mac_base)) & MDIO_CMD_GO); i++)
		udelay(1);
	if (i >= MDIO_TIMEOUT)
		goto err_out;

	val = enet_readl(ENET_MDIO_STS1(priv->enet_mac_base));
	if (val & MDIO_STS_ERR)
		return -1;

	return val & 0xffff;

err_out:
	return -1;
}

static void enet_mdio_write(struct net_device *dev, int phy_id,
				     int location, int val)
{
	int i, tmp;
	struct tangox_enet_priv *priv = netdev_priv(dev);

	for (i = 0; (i < MDIO_TIMEOUT) && (enet_readl(ENET_MDIO_CMD1(priv->enet_mac_base)) & MDIO_CMD_GO); i++)
		udelay(1);
	if (i >= MDIO_TIMEOUT)
		goto err_out;

	tmp = MIIAR_DATA(val) | MIIAR_ADDR(phy_id) | MIIAR_REG(location);
	enet_writel(ENET_MDIO_CMD1(priv->enet_mac_base), tmp);

	udelay(10);

	enet_writel(ENET_MDIO_CMD1(priv->enet_mac_base), tmp | MDIO_CMD_WR);

	udelay(10);

	enet_writel(ENET_MDIO_CMD1(priv->enet_mac_base), tmp | MDIO_CMD_WR | MDIO_CMD_GO);

	for (i = 0; (i < MDIO_TIMEOUT) && (enet_readl(ENET_MDIO_CMD1(priv->enet_mac_base)) & MDIO_CMD_GO); i++)
		udelay(1);
	if (i >= MDIO_TIMEOUT)
		goto err_out;

	return;

err_out:
	return;
}

/* statistic counter read and write functions
 * 44 counters are included for tracking 
 * occurences of frame status evernts.
 */
static unsigned long enet_stat_read(struct net_device *dev, unsigned char index)
{
	struct tangox_enet_priv *priv = netdev_priv(dev);
	enet_writeb(ENET_STAT_INDEX(priv->enet_mac_base), index);
	return enet_readl(ENET_STAT_DATA1(priv->enet_mac_base));
}

static void enet_stat_write(struct net_device *dev, unsigned long val, unsigned char index)
{
	struct tangox_enet_priv *priv = netdev_priv(dev);
	enet_writeb(ENET_STAT_INDEX(priv->enet_mac_base), index);
	enet_writel(ENET_STAT_DATA1(priv->enet_mac_base), val);
}

static __inline int enet_rx_error(unsigned long report)
{
	if (report & (RX_FCS_ERR | RX_FRAME_LEN_ERROR | 
#ifndef ENABLE_MULTICAST
		RX_MULTICAST_PKT |
#endif
		RX_LENGTH_ERR | RX_LATE_COLLISION | 
		RX_FIFO_OVERRUN | RX_RUNT_PKT)) 
		return 1;
	else
		return 0;
}

/*
 * rx poll func, called by network core
 */
static int enet_poll(struct net_device *dev, int *budget)
{
	struct tangox_enet_priv *priv;
	volatile struct enet_desc *rx, *rx1;
	int limit, received;
	unsigned int rx_eoc;

	priv = netdev_priv(dev);
	rx_eoc = priv->rx_eoc;

	/* calculate how many rx packet we are allowed to fetch */
	limit = *budget;
	if (*budget > dev->quota)
		limit = dev->quota;
	received = 0;

	/* process no more than "limit" done rx */
	while (limit > 0) {
		struct sk_buff *skb;
		volatile u32 *r_addr;
		u32 report_cache;
		unsigned int len = 0;
		int pkt_dropped = 0;

		rx = &priv->rx_descs[priv->last_rx_desc];

		/* we need multiple read on this volatile, avoid
		 * memory access at each time */
		r_addr = (volatile u32 *)KSEG1ADDR((u32)&(priv->rx_report[priv->last_rx_desc]));
		report_cache = __raw_readl(r_addr);

#ifdef ETH_DEBUG
		if (rx->config & DESC_EOC) {
			/* should not happen */
			printk("%s i=0x%x rx=0x%x report=0x%x config=0x%x limit=0x%x\n", 
				__FUNCTION__, priv->last_rx_desc, rx, report_cache, rx->config, limit);
		}
#endif
		if (report_cache == 0){ 
			uint32_t *next_r_addr;
			uint32_t next_report_cache;
			next_r_addr = (uint32_t *)KSEG1ADDR((u32)&(priv->rx_report[(priv->last_rx_desc+1)%RX_DESC_COUNT]));
			next_report_cache = __raw_readl(next_r_addr);

			/*check see if next one on error*/
			if(!enet_rx_error(next_report_cache))
				break;
		}

		--limit;

		if (likely((skb = priv->rx_skbs[priv->last_rx_desc]) != NULL)) {

			len = RX_BYTES_TRANSFERRED(report_cache);
			if((report_cache ==0) ||enet_rx_error(report_cache)){

#ifndef ENABLE_MULTICAST
				if (report_cache & RX_MULTICAST_PKT){ 
					DBG("%s RX_MULTICAST_PKT report=0x%x\n", __FUNCTION__, report_cache);				
					priv->stats.rx_length_errors++;
				}
#endif
				if (report_cache & RX_FCS_ERR) {
					DBG("%s RX_FCS_ERR report=0x%x\n", __FUNCTION__, report_cache);				
					priv->stats.rx_crc_errors++;
				}

				if (report_cache & RX_LATE_COLLISION){ 
					DBG("%s RX_LATE_COLLSION report=0x%x\n", __FUNCTION__, report_cache);				
				}

				if (report_cache &  RX_FIFO_OVERRUN ){ 
					DBG("%s RX_FIFO_OVERRUN report=0x%x\n", __FUNCTION__, report_cache);				
				}

				if (report_cache & RX_RUNT_PKT) {
					DBG("%s RX_RUNT_PKT report=0x%x\n", __FUNCTION__, report_cache);				
				}

				if (report_cache & (RX_FRAME_LEN_ERROR | RX_LENGTH_ERR) ||
				     len > RX_BUF_SIZE) {
					priv->stats.rx_length_errors++;
				}

				priv->stats.rx_errors++;
				pkt_dropped = 1;
				goto done_checking;

			} else {

				/* ok, seems  valid, adjust skb  proto and len
				 * and give it to kernel */
				skb->dev = dev;
				skb_put(skb, len);
				skb->protocol = eth_type_trans(skb, dev);
				netif_receive_skb(skb);
#ifdef ETH_DEBUG
				if(len > 0){
					int i;
					DBG("-----received data------\n");
					for (i=0; i< len; i++){
						if(i%16==0 && i>0)
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
			mb();
			break;
		}

		rx->config = RX_BUF_SIZE | DESC_BTS(2) | DESC_EOF/* | DESC_ID*/;

		skb_reserve(skb, SKB_RESERVE_SIZE);
		rx->s_addr = PHYSADDR((void*)(skb->data));
		dma_cache_inv((unsigned long)skb->data, RX_BUF_SIZE);
		priv->rx_skbs[priv->last_rx_desc] = skb;

rearm:
		/* rearm descriptor */
		__raw_writel(0, r_addr);
		priv->last_rx_desc++;
		priv->last_rx_desc %= RX_DESC_COUNT;
		received++;

	}

	if (received != 0) {
		rx = &priv->rx_descs[rx_eoc];
		rx->config |= DESC_EOC;
		mb();
		rx1 = &priv->rx_descs[priv->rx_eoc];
		rx1->config &= ~DESC_EOC; 
		mb();
		priv->rx_eoc = rx_eoc;
	
		dev->quota -= received;
		*budget -= received;
	}

	enet_start_rx(priv);

	if (limit <= 0) {
		/* breaked, but there is still work to do */
		return 1;
	}

	netif_rx_complete(dev);
	return 0;
}

/*
 * tx request callback
 */
static int enet_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct tangox_enet_priv *priv;
	volatile struct enet_desc *tx=NULL, *ptx=NULL;
	unsigned long tconfig_cache;
	unsigned long val = 0;
	volatile u32 *r_addr;
	int len = 0;
	int tx_busy = 0;
	unsigned char *txbuf;

	priv = netdev_priv(dev);
	spin_lock(&priv->tx_lock);

	val = enet_readl(ENET_TXC_CR(priv->enet_mac_base)) & 0xffff;
#ifndef ENABLE_TX_CHAINING
#define MAX_TX_TIMEOUT	100	/* usec */
	for (len = 0; len < MAX_TX_TIMEOUT; len++) {
		val = enet_readl(ENET_TXC_CR(priv->enet_mac_base)) & 0xffff;
		if (val & TCR_EN)
			udelay(1);
		else
			break;
	}
	if (len >= MAX_TX_TIMEOUT) {
		priv->stats.tx_dropped++;
		spin_unlock(&priv->tx_lock);
		return NETDEV_TX_BUSY;
	}
#else
	if (val & TCR_EN){ 
		//BUG_ON(skb == NULL);
		tx_busy = 1;
		if (priv->pending_tx < 0)
			priv->pending_tx = priv->next_tx_desc;
	} 

	if (tx_busy && (priv->pending_tx >= 0) && (priv->pending_tx_cnt >= (TX_DESC_COUNT -1))) {
		DBG(KERN_WARNING PFX "no more tx desc can be scheduled in pending queue.\n");
		netif_stop_queue(dev);
		spin_unlock(&priv->tx_lock);
		return NETDEV_TX_BUSY;
	}
		
	if (skb == NULL) {
		unsigned int last_tx;		
		last_tx = (priv->next_tx_desc - 1 + TX_DESC_COUNT) % TX_DESC_COUNT;
		tx = &priv->tx_descs[last_tx];
		tx->config |= DESC_EOC;
		priv->tx_eoc = last_tx;
		mb();
		goto tx_pending;
	}
#endif
	len = skb->len;
	tx = &priv->tx_descs[priv->next_tx_desc];

	/* fill the tx desc with this skb address */
	tconfig_cache = 0;
	tconfig_cache |= DESC_BTS(2);
	tconfig_cache |= DESC_EOF;
	tconfig_cache |= len; 

	if (((unsigned long)(skb->data) & 0x7) != 0) { /* not align by 8 bytes */
		txbuf = priv->tx_bufs[priv->next_tx_desc];
		memcpy(txbuf, skb->data, len); 
		dma_cache_wback((unsigned long)txbuf, len);
		tx->s_addr = PHYSADDR((void *)txbuf);
	} else {
		dma_cache_wback((unsigned long)skb->data, len);
		tx->s_addr = PHYSADDR(skb->data);
	}

	if (tx_busy != 0) {
		tx->n_addr = PHYSADDR((void *)&(priv->tx_descs[(priv->next_tx_desc + 1) % TX_DESC_COUNT]));
	} else {
		tx->n_addr = 0;
		tconfig_cache |= DESC_EOC;
		priv->tx_eoc = priv->next_tx_desc;
	}
	tx->config = tconfig_cache;

	/* keep a pointer to it for later and give it to dma  */
	priv->tx_skbs[priv->next_tx_desc] = skb;

	r_addr = (volatile u32 *)KSEG1ADDR((u32)(&(priv->tx_report[priv->next_tx_desc])));
	__raw_writel(0, r_addr);
	priv->next_tx_desc++;
	priv->next_tx_desc %= TX_DESC_COUNT;

#ifdef ETH_DEBUG
	{
	int i;			
 	for(i=0; i<len; i++){
		if(i%16==0 && i>0)
			DBG("\n");
		DBG("%02x ", txbuf[i] & 0xff);
	}
	DBG("\n");

	DBG("DESC Mode:  TXC_CR=0x%x  desc_addr=0x%x s_addr=0x%x n_addr=0x%x r_addr=0x%x config=0x%x\n",
			enet_readl(ENET_TXC_CR(priv->enet_mac_base)), tx,
			tx->s_addr, tx->n_addr,
			tx->r_addr, tx->config); 
	}
#endif

tx_pending:
	if (tx_busy == 0) {
		if (priv->pending_tx >= 0) {
			ptx = &priv->tx_descs[priv->pending_tx];
			len = ptx->config & 0xffff;

			enet_writel(ENET_TX_DESC_ADDR(priv->enet_mac_base), PHYSADDR((void *)ptx));
			priv->reclaim_limit = priv->pending_tx;
			priv->pending_tx = -1;
		} else {
			priv->reclaim_limit = (priv->next_tx_desc - 1 + TX_DESC_COUNT) % TX_DESC_COUNT;
			enet_writel(ENET_TX_DESC_ADDR(priv->enet_mac_base), PHYSADDR((void *)tx));
		}

		enet_writel(ENET_TX_SAR(priv->enet_mac_base), 0);
		enet_writel(ENET_TX_REPORT_ADDR(priv->enet_mac_base), 0);

		/* kick tx dma in case it was suspended */
		val |= TCR_EN; 
		val |= TCR_BTS(2); 
		val |= (len << 16); 
		enet_writel(ENET_TXC_CR(priv->enet_mac_base), val);

		/* no pending at this stage*/
		priv->pending_tx_cnt = 0;
	} else 
		priv->pending_tx_cnt++;

	/* if next tx descriptor is not  clean, then we have to stop
	 * queue */
	if (unlikely(--priv->free_tx_desc_count == 0))
		netif_stop_queue(dev);

	spin_unlock(&priv->tx_lock);

	return NETDEV_TX_OK;
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

	spin_lock(&priv->tx_lock);

	while (priv->free_tx_desc_count < TX_DESC_COUNT) {
		volatile u32 *r_addr;
		u32 report_cache;
		struct sk_buff *skb;

		r_addr = (volatile u32 *)KSEG1ADDR((u32)(&(priv->tx_report[priv->dirty_tx_desc])));
		report_cache = __raw_readl(r_addr);
		if (priv->dirty_tx_desc == priv->reclaim_limit)
			break;

		skb = priv->tx_skbs[priv->dirty_tx_desc];
		priv->stats.tx_packets++;
		if (skb) {
			/* check  for  transmission  errors and  update  stats
			 * accordingly */
			if (report_cache & (TX_FIRST_DEFERRAL | TX_LATE_COLLISION |
					    TX_PACKET_DROPPED | TX_FIFO_UNDERRUN)) {
				priv->stats.tx_errors++;
			} else {
				priv->stats.tx_bytes += skb->len;
			}
			dev_kfree_skb(skb);
		}
		priv->tx_skbs[priv->dirty_tx_desc] = NULL;
		priv->dirty_tx_desc++;
		priv->dirty_tx_desc %= TX_DESC_COUNT;
		priv->free_tx_desc_count++;
		__raw_writel(0, r_addr);
	}

	if (priv->free_tx_desc_count != 0 && netif_queue_stopped(dev))
		netif_wake_queue(dev);

	spin_unlock(&priv->tx_lock);

#ifdef ENABLE_TX_CHAINING
	/* to rearm the xmit for some cases */
	if (priv->pending_tx >= 0) {
		unsigned int idx = (priv->dirty_tx_desc + priv->pending_tx_cnt) % TX_DESC_COUNT;
		if (priv->pending_tx == idx)
			enet_xmit(NULL, dev);
	}
#endif
}

/*
 * our  irq handler, just  ack it  and schedule  the right  tasklet to
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
	if ((val = enet_readl(ENET_TXC_SR(priv->enet_mac_base))) != 0) {
		enet_writel(ENET_TXC_SR(priv->enet_mac_base), 0xff);
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
	if ((val = enet_readl(ENET_RXC_SR(priv->enet_mac_base))) != 0) {
		enet_writel(ENET_RXC_SR(priv->enet_mac_base), 0xff);
		if (likely(val & RSR_RI)) {
			if (netif_rx_schedule_prep(dev)) {
				/*todo: disable rx interrupt */
				/*avoid reentering */
				enet_writel(ENET_RXC_SR(priv->enet_mac_base), 0xff);
				__netif_rx_schedule(dev);
			}
		}

		if (unlikely(val & RSR_DI)) 
			DBG("RX EOC\n");			
		if (unlikely(val & RSR_DE))
			DBG("RX DMA error\n");
		if (unlikely(val & RSR_RO))
			DBG("RX FIFO overflow\n");
	}

 	/* wake on lan */
 	if ((val = enet_readb(ENET_WAKEUP(priv->enet_mac_base))) == 1) {
 		/* clear sleeping mode */
 		enet_writeb(ENET_SLEEP_MODE(priv->enet_mac_base), 0);
 		/* clear wakeup mode */
 		enet_writeb(ENET_WAKEUP(priv->enet_mac_base), 0);
 	}

	return IRQ_HANDLED;
}

/*
 * start/stop dma engine
 */
static __inline void enet_start_rx(struct tangox_enet_priv *priv)
{
	unsigned long val, flags;
	spin_lock_irqsave(&priv->ier_lock, flags);
	val = enet_readl(ENET_RXC_CR(priv->enet_mac_base));
	if(!(val & RCR_EN)){
		val  |= RCR_EN;
		enet_writel(ENET_RXC_CR(priv->enet_mac_base), val);
	}
	spin_unlock_irqrestore(&priv->ier_lock, flags);
}

static __inline void enet_stop_rx(struct tangox_enet_priv *priv)
{
	unsigned long val, flags;

	spin_lock_irqsave(&priv->ier_lock, flags);
	/* stop  rx  dma */
	val = enet_readl(ENET_RXC_CR(priv->enet_mac_base));
	if((val & RCR_EN)){
		val &= ~RCR_EN;
		enet_writel(ENET_RXC_CR(priv->enet_mac_base), val);
	}
	spin_unlock_irqrestore(&priv->ier_lock, flags);
}

static int enet_get_speed(struct net_device *dev)
{
	struct tangox_enet_priv *priv;
	unsigned int physr;
	int speed = 100;

	priv = netdev_priv(dev);
	if(priv->rgmii == 0){
		/*Realtek*/
		physr = enet_mdio_read(dev, priv->mii.phy_id, MII_PHYADDR);
		switch(physr & 0x3){
		case 1:
			speed = 100;
			break;
		case 2:
			speed = 10;
			break;
		default:			
			speed = 0;
			break;
		}
		return speed;
	}
	/* Vitesse */
	physr = enet_mdio_read(dev, priv->mii.phy_id, MII_NCONFIG);
	physr >>= 3;

	switch(physr & 0x3){
		case 0:	
			speed = 10;
			break;
		case 1:
			speed = 100;
			break;
		case 2:
			speed = 1000;
			break;
		case 3:
			speed = 0;
			break;
	}
	return speed;
}



static int phy_autoneg(struct net_device *dev)
{
	struct tangox_enet_priv *priv;
	int loop;
	unsigned int val = 0;

	priv = netdev_priv(dev);

	/* reset phy */
	val = enet_mdio_read(dev, priv->mii.phy_id, MII_BMCR);
	enet_mdio_write(dev, priv->mii.phy_id, MII_BMCR, val | BMCR_ANENABLE|BMCR_ANRESTART);
	udelay(100);
	loop = 10000;
	while (loop) {
	if (enet_mdio_read(dev, priv->mii.phy_id, MII_BMSR) &  BMSR_ANEGCOMPLETE)
			break;

		mdelay(1);
		loop--;
	}

	if (!loop) {
		printk(KERN_ERR "%s: PHY autonegotiation does not complete...\n", priv->name);
		return -EBUSY;
	}
	return 0;
}



static void enet_mac_config(struct net_device *dev)
{
	struct tangox_enet_priv *priv;
	unsigned char val;
	int speed;

	priv = netdev_priv(dev);

	if(priv->rgmii == 0) {
		/* 100 baseT, realtek*/
		val = enet_readb(ENET_MAC_MODE(priv->enet_mac_base));
		if(val & (GMAC_MODE | RGMII_MODE)){
			val &= ~(GMAC_MODE | RGMII_MODE);    /*disable Gigabit mode for now*/
			//val |= /*LB_EN |*/ BST_EN;	     /*loopback off, burst on*/
			enet_writeb(ENET_MAC_MODE(priv->enet_mac_base), val);
		}
		return;
	}

	/*set RGMII skew timing compensation enable */
	/* This added 2ns delay to RX_CLK and TX_CLK*/
	val = enet_mdio_read(dev, priv->mii.phy_id, MII_RESV1);
	enet_mdio_write(dev, priv->mii.phy_id, MII_RESV1 , val | (1<<8));
	
	val = enet_readl(ENET_MAC_MODE(priv->enet_mac_base));
	speed = enet_get_speed(dev);
	if(speed == 1000 ){
		if((val & RGMII_MODE) && (val & GMAC_MODE))
			return; /* configured for gigabit */

		val |= RGMII_MODE;	
		val |= GMAC_MODE;
		enet_writeb(ENET_MAC_MODE(priv->enet_mac_base), val);
		//printk("set 1000mbps  val=0x%x\n",  val);
	}else if(speed == 100){
		if((val & RGMII_MODE) && !(val & GMAC_MODE))
			return; /* configured for 100mbps*/
		val |= RGMII_MODE;	
		val &= ~GMAC_MODE;
		enet_writeb(ENET_MAC_MODE(priv->enet_mac_base), val);
    	//printk("set 100mbps  val=0x%x\n",  val);
	}else /*TODO*/
	;	//printk("set 10 mbps or other val=0x%x\n",  val);

	/*set threshold for internal clock 0x1*/
	enet_writeb(ENET_IC_THRESHOLD(priv->enet_mac_base), (speed==1000 ? 3:1));

	/*set slot time  0x7f for 10/100Mbps*/
	enet_writeb(ENET_SLOT_TIME(priv->enet_mac_base), (speed==1000 ? 0xff : 0x7f));

	//phy_autoneg(dev);
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
	spin_lock(&priv->maccr_lock);
	val = enet_readl(ENET_MAC_MODE(priv->enet_mac_base));
	if (priv->mii.full_duplex)
		val &= ~HALF_DUPLEX;
	else
		val |= HALF_DUPLEX;
	enet_writeb(ENET_MAC_MODE(priv->enet_mac_base), val);

	enet_mac_config(dev);

	spin_unlock(&priv->maccr_lock);
}

/*
 * link check timer callback
 */
static void enet_link_check(unsigned long data)
{
	struct net_device *dev;
	struct tangox_enet_priv *priv;
	int ret, speed;
	static int old_speed;

	dev = (struct net_device *)data;
	priv = netdev_priv(dev);

	/*check speed change in gigabit*/
	speed = enet_get_speed(dev);

	/* check for duplex change */
	spin_lock(&priv->mii_lock);
	ret = mii_check_media(&priv->mii, 1, 0);
	spin_unlock(&priv->mii_lock);

	if (ret || (speed && (speed !=old_speed)))
		enet_link_reconfigure(dev);

	if(speed)
		old_speed = speed;

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
	enet_writeb(ENET_MAC_ADDR1(priv->enet_mac_base), dev->dev_addr[0]);
	enet_writeb(ENET_MAC_ADDR2(priv->enet_mac_base), dev->dev_addr[1]);
	enet_writeb(ENET_MAC_ADDR3(priv->enet_mac_base), dev->dev_addr[2]);
	enet_writeb(ENET_MAC_ADDR4(priv->enet_mac_base), dev->dev_addr[3]);
	enet_writeb(ENET_MAC_ADDR5(priv->enet_mac_base), dev->dev_addr[4]);
	enet_writeb(ENET_MAC_ADDR6(priv->enet_mac_base), dev->dev_addr[5]);
	
	/* set unicast addr */
	enet_writeb(ENET_UC_ADDR1(priv->enet_mac_base), dev->dev_addr[0]);
	enet_writeb(ENET_UC_ADDR2(priv->enet_mac_base), dev->dev_addr[1]);
	enet_writeb(ENET_UC_ADDR3(priv->enet_mac_base), dev->dev_addr[2]);
	enet_writeb(ENET_UC_ADDR4(priv->enet_mac_base), dev->dev_addr[3]);
	enet_writeb(ENET_UC_ADDR5(priv->enet_mac_base), dev->dev_addr[4]);
	enet_writeb(ENET_UC_ADDR6(priv->enet_mac_base), dev->dev_addr[5]);

	return 0;
}

/*
 * update hash table to reflect new device multicast address list
 */
static void enet_set_multicast_list(struct net_device *dev)
{
#ifdef ENABLE_MULTICAST
	struct tangox_enet_priv *priv;
	struct dev_mc_list *mclist;
	unsigned char val;
	uint32_t mc_filter[2];
	int i;

	priv = netdev_priv(dev);

	/* the link check timer might change RX control, we need to protect
	 * against it */
	spin_lock_bh(&priv->maccr_lock);
	val = enet_readl(ENET_RX_CTL(priv->enet_mac_base));

	if (dev->flags & IFF_PROMISC) {
		val &= ~(RX_BC_DISABLE | RX_AF_EN);
	} else {
		val |= RX_AF_EN	;
		/* if we want all multicast or if address count is too
		 * high, don't try to compute hash value */
		if (dev->mc_count > 64 || dev->flags & IFF_ALLMULTI) {
			val &= ~(RX_BC_DISABLE | RX_AF_EN);
		}
	}

	enet_writel(ENET_RX_CTL(priv->enet_mac_base), val);
	spin_unlock_bh(&priv->maccr_lock);

	/* we  don't  need  to  update  hash  table  if  we  pass  all
	 * multicast */
	if (!(val & RX_BC_DISABLE) && !(val & RX_AF_EN))
		return;

	/* clear internal multicast address table */
	enet_writeb(ENET_MC_INIT(priv->enet_mac_base), 0x0);
	while(enet_readb(ENET_MC_INIT(priv->enet_mac_base)));

	mc_filter[0] = mc_filter[1] = 0;
	mclist = dev->mc_list;

	for (i = 0; i < dev->mc_count; i++) {
		char *addr;

		addr = mclist->dmi_addr;
		mclist = mclist->next;
		if (!(*addr & 1))
			continue;

		enet_writeb(ENET_MC_ADDR1(priv->enet_mac_base), addr[0]);
		enet_writeb(ENET_MC_ADDR2(priv->enet_mac_base), addr[1]);
		enet_writeb(ENET_MC_ADDR3(priv->enet_mac_base), addr[2]);
		enet_writeb(ENET_MC_ADDR4(priv->enet_mac_base), addr[3]);
		enet_writeb(ENET_MC_ADDR5(priv->enet_mac_base), addr[4]);
		enet_writeb(ENET_MC_ADDR6(priv->enet_mac_base), addr[5]);
		enet_writeb(ENET_MC_INIT(priv->enet_mac_base),	0xff);
		while(enet_readb(ENET_MC_INIT(priv->enet_mac_base)));
	}
#endif
}

/*
 * open callback
 */
static int enet_open(struct net_device *dev)
{
	struct tangox_enet_priv *priv;
	unsigned char val;

	priv = netdev_priv(dev);

	/* check link */
	if (mii_check_media(&priv->mii, 1, 1))
		enet_link_reconfigure(dev);

	/* enable mac rx & tx */
	val = enet_readb(ENET_RX_CTL(priv->enet_mac_base));
	val |= RX_EN;
	enet_writeb(ENET_RX_CTL(priv->enet_mac_base), val);

	val = enet_readb(ENET_TX_CTL1(priv->enet_mac_base));
	val |= TX_EN;
	enet_writeb(ENET_TX_CTL1(priv->enet_mac_base), val);

	/*
	 * clear & enable interrupts, we want:
	 * - receive complete
	 * - transmit complete
	 */
	enet_writel(ENET_TXC_SR(priv->enet_mac_base), 0xff);
	enet_writel(ENET_RXC_SR(priv->enet_mac_base), 0xff);

	/* start link check & tx reclaim timer */
	priv->link_check_timer.expires = jiffies + LINK_CHECK_TIMER_FREQ;
	add_timer(&priv->link_check_timer);

	//priv->tx_reclaim_timer.expires = jiffies + TX_RECLAIM_TIMER_FREQ;
	//add_timer(&priv->tx_reclaim_timer);

	/* and finally start tx queue */
	netif_start_queue(dev);

	/* start rx dma engine */
	enet_start_rx(priv);

	return 0;
}

/*
 * stop callback
 */
static int enet_stop(struct net_device *dev)
{
	struct tangox_enet_priv *priv;
	unsigned char val;
	volatile struct enet_desc *rx;
	int i;

	priv = netdev_priv(dev);

	/* stop link timer */
	del_timer_sync(&priv->link_check_timer);

	/* stop tx queue */
	netif_stop_queue(dev);

	/* stop dma */
	//enet_stop_rx(priv);

	/* stop mac rx & tx */
	val = enet_readb(ENET_RX_CTL(priv->enet_mac_base));
	val &= ~RX_EN;
	enet_writeb(ENET_RX_CTL(priv->enet_mac_base), val);

	val = enet_readb(ENET_TX_CTL1(priv->enet_mac_base));
	val &= ~TX_EN;
	enet_writeb(ENET_TX_CTL1(priv->enet_mac_base), val);

	/* while we were stopping it,  the rx dma may have filled some
	 * buffer, consider it junk and rearm all descriptor */
	priv->dirty_tx_desc = priv->next_tx_desc = 0;
	priv->pending_tx  = -1;
	priv->pending_tx_cnt  = 0;
	priv->reclaim_limit  = -1;
	priv->last_rx_desc = 0;
	priv->free_tx_desc_count = TX_DESC_COUNT;

	enet_writel(ENET_RX_DESC_ADDR(priv->enet_mac_base), PHYSADDR((void *)&priv->rx_descs[0]));
	enet_writel(ENET_TX_DESC_ADDR(priv->enet_mac_base), PHYSADDR((void *)&priv->tx_descs[0]));
	/* clear eoc and set it to the last one*/
        for(i=0; i< RX_DESC_COUNT; i++){
		int cnt;
		cnt = (priv->last_rx_desc + i) % RX_DESC_COUNT;
		rx = &priv->rx_descs[cnt];
		rx->config &= ~DESC_EOC;
		*((volatile unsigned long *)KSEG1ADDR(&(priv->rx_report[i]))) = 0; 
	}
	rx = &priv->rx_descs[RX_DESC_COUNT-1];
	rx->config |= DESC_EOC;
	priv->rx_eoc = RX_DESC_COUNT - 1;
	mb();
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

	spin_lock_bh(&priv->mii);
	ret = generic_mii_ioctl(&priv->mii, if_mii(rq), cmd, NULL);
	spin_unlock_bh(&priv->mii);

	return ret;
}

/*
 * dma ring allocation is done here
 */
static int enet_dma_init(struct tangox_enet_priv *priv)
{
	unsigned int size;
	int i, rx_order, tx_order;
	
	/*
	 * allocate rx descriptor list & rx buffers
	 */
	size = RX_DESC_COUNT * sizeof (struct enet_desc);
	for (rx_order = 0; (PAGE_SIZE << rx_order) < size; rx_order++);

	if (!(priv->rx_descs_cached = (void *)__get_free_pages(GFP_KERNEL | GFP_DMA, rx_order)))
		return -ENOMEM;
	dma_cache_wback_inv((unsigned long)priv->rx_descs_cached, size);
	priv->rx_descs = (volatile struct enet_desc *)
		CACHE_TO_NONCACHE((unsigned long)priv->rx_descs_cached);

	/*
	 * initialize all rx descs
	 */
	for (i = 0; i < RX_DESC_COUNT; i++) {
		volatile struct enet_desc *rx;
		struct sk_buff *skb;

		rx = &priv->rx_descs[i];
		rx->config = RX_BUF_SIZE | DESC_BTS(2) | DESC_EOF/* | DESC_ID*/;

		skb = dev_alloc_skb(RX_BUF_SIZE + SKB_RESERVE_SIZE);
		if (!skb)
			return -ENOMEM;
		
		skb_reserve(skb, SKB_RESERVE_SIZE);
		*((volatile unsigned long *)KSEG1ADDR(&(priv->rx_report[i]))) = 0; 
		rx->s_addr = PHYSADDR((void *)skb->data);
		rx->r_addr = PHYSADDR((void *)&priv->rx_report[i]);
		rx->n_addr = PHYSADDR((void *)&priv->rx_descs[i+1]);
		if (i == (RX_DESC_COUNT - 1)) {
			rx->n_addr = PHYSADDR((void *)&priv->rx_descs[0]);
			rx->config |= DESC_EOC ;
			priv->rx_eoc = i;
		}
#ifdef ETH_DEBUG
		DBG("rx[%d]=0x%08x\n", i, (unsigned int)rx);
		DBG("  s_addr=0x%08x\n", (unsigned int)rx->s_addr);
		DBG("  n_addr=0x%08x\n", (unsigned int)rx->n_addr);
		DBG("  r_addr=0x%08x\n", (unsigned int)rx->r_addr);
		DBG("  config=0x%08x\n", (unsigned int)rx->config);
#endif
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
	size = TX_DESC_COUNT * sizeof (struct enet_desc);
	for (tx_order = 0; (PAGE_SIZE << tx_order) < size; tx_order++);

	if (!(priv->tx_descs_cached = (void *)__get_free_pages(GFP_KERNEL | GFP_DMA, tx_order))) {
		free_pages((u32)priv->rx_descs_cached, rx_order);
		return -ENOMEM;
	}
	dma_cache_wback_inv((unsigned long)priv->tx_descs_cached, size);
	priv->tx_descs = (volatile struct enet_desc *)
		CACHE_TO_NONCACHE((unsigned long)priv->tx_descs_cached);

	/*
	 * initialize tx descs
	 */
	for (i = 0; i < TX_DESC_COUNT; i++) {
		volatile struct enet_desc *tx;

		priv->tx_bufs[i] = (unsigned char *)__get_free_page(GFP_KERNEL | GFP_DMA);
		dma_cache_wback_inv((unsigned long)priv->tx_bufs[i], PAGE_SIZE);

		tx = &priv->tx_descs[i];
		*((volatile unsigned long *)KSEG1ADDR(&(priv->tx_report[i]))) = 0; 
		tx->r_addr = PHYSADDR((void *)&priv->tx_report[i]);
		tx->s_addr = 0;
		tx->config = DESC_EOF;
		if (i == (TX_DESC_COUNT - 1)) {
			tx->config |= DESC_EOC;
			tx->n_addr = PHYSADDR((void *)&priv->tx_descs[0]);
			priv->tx_eoc = i;
		}
		//DBG("tx[%d]=0x%08x\n", i, (unsigned int)tx);
	}
	priv->dirty_tx_desc = priv->next_tx_desc = 0;
	priv->pending_tx = -1;
	priv->pending_tx_cnt  = 0;
	priv->reclaim_limit  = -1;
	priv->free_tx_desc_count = TX_DESC_COUNT;

	/*
	 * write rx desc list & tx desc list addresses in registers
	 */
	enet_writel(ENET_TX_DESC_ADDR(priv->enet_mac_base), PHYSADDR((void *)&priv->tx_descs[0]));
	enet_writel(ENET_RX_DESC_ADDR(priv->enet_mac_base), PHYSADDR((void *)&priv->rx_descs[0]));
	return 0;
}

/*
 * free  all dma rings  memory, called  at uninit  time or  when error
 * occurs at init time
 */
static void enet_dma_free(struct tangox_enet_priv *priv)
{
	int i, rx_order, tx_order;

	for (rx_order = 0; (PAGE_SIZE << rx_order) < (RX_DESC_COUNT * sizeof(struct enet_desc)) ; rx_order++);
	for (tx_order = 0; (PAGE_SIZE << tx_order) < (TX_DESC_COUNT * sizeof(struct enet_desc)) ; tx_order++);

	/* note: kfree(NULL) is ok */
	if (priv->rx_descs_cached) {
		free_pages((u32)priv->rx_descs_cached, rx_order);
		priv->rx_descs_cached = NULL;
	}
	if (priv->tx_descs_cached) {
		free_pages((u32)priv->tx_descs_cached, tx_order);
		priv->tx_descs_cached = NULL;
	}

	/* note: kfree_skb(NULL) is _not_ ok */
	for (i = 0; i < RX_DESC_COUNT; i++) {
		if (priv->rx_skbs[i]) {
			kfree_skb(priv->rx_skbs[i]);
			priv->rx_skbs[i] = NULL;
		}
	}

	for (i = 0; i < TX_DESC_COUNT; i++) {
		if (priv->tx_skbs[i]) {
			kfree_skb(priv->tx_skbs[i]);
			priv->tx_skbs[i] = NULL;
		}
		if (priv->tx_bufs[i]) {
			free_page((u32)priv->tx_bufs[i]);
			priv->tx_bufs[i] = NULL;
		}
	}
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
		if (!(enet_mdio_read(dev, priv->mii.phy_id,
				     MII_BMCR) & BMCR_RESET))
			break;
		mdelay(1);
		loop--;
	}

	if (!loop) {
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

	if(phy_reset(dev))
		return -EBUSY;

	priv = netdev_priv(dev);

	/* set pad_mode according to rgmii or not*/
	val = enet_readb(priv->enet_mac_base + 0x400) & 0xf0;
	if(priv->rgmii)
		enet_writeb(priv->enet_mac_base + 0x400, val | 0x01);

	/* software reset IP */
	enet_writeb(priv->enet_mac_base + 0x424, 0);
	udelay(10);
	enet_writeb(priv->enet_mac_base + 0x424, 1);

	/*set threshold for internal clock 0x1*/
	enet_writeb(ENET_IC_THRESHOLD(priv->enet_mac_base), 1);

	/*set Random seed 0x8*/
	enet_writeb(ENET_RANDOM_SEED(priv->enet_mac_base), 0x08);

	/*set TX single deferral params 0xc*/
	enet_writeb(ENET_TX_SDP(priv->enet_mac_base), 0xc);

	/*set slot time  0x7f for 10/100Mbps*/
	enet_writeb(ENET_SLOT_TIME(priv->enet_mac_base), 0x7f);

	/*set Threshold for partial full  0x7f */
	enet_writeb(ENET_PF_THRESHOLD(priv->enet_mac_base), 0x7f);

	/* configure TX DMA Channels */
	val = enet_readl(ENET_TXC_CR(priv->enet_mac_base));
	val |=	TCR_RS  | TCR_LE  | TCR_TFI(1) | 
		/*TCR_DIE |*/ TCR_BTS(2);
	val |=	TCR_DM;
 
	enet_writel(ENET_TXC_CR(priv->enet_mac_base), val);
	val = enet_readl(ENET_TXC_CR(priv->enet_mac_base));

 	/* configure RX DMA Channels */
	val = enet_readl(ENET_RXC_CR(priv->enet_mac_base));
	val |= (RCR_RS    | RCR_LE | RCR_RFI(1) | 
		RCR_BTS(2) | RCR_FI | RCR_DIE /* | RCR_EN*/); 
	val |=	RCR_DM;

	val |=  RX_BUF_SIZE << 16;
	enet_writel(ENET_RXC_CR(priv->enet_mac_base), val); 

	/* configure MAC ctrller */
	val = enet_readb(ENET_TX_CTL1(priv->enet_mac_base));
	val |= (TX_RETRY_EN | TX_PAD_EN | TX_APPEND_FCS);
	enet_writeb(ENET_TX_CTL1(priv->enet_mac_base), (unsigned char)val);

	/* set retry 5 time when collision occurs*/
	enet_writeb(ENET_TX_CTL2(priv->enet_mac_base), 5);

	val = enet_readb(ENET_RX_CTL(priv->enet_mac_base));
	val |= (RX_RUNT | RX_PAD_STRIP | RX_SEND_CRC 
	                | RX_PAUSE_EN| RX_AF_EN);
	enet_writeb(ENET_RX_CTL(priv->enet_mac_base), (unsigned char)val);

#ifdef ENABLE_MULTICAST
	/* clear internal multicast address table */
	enet_writeb(ENET_MC_INIT(priv->enet_mac_base),  0x00);
	while(enet_readb(ENET_MC_INIT(priv->enet_mac_base)));
	DBG("Internal multicast address table is cleared\n");
#endif

	/* unicast */
	/* Threshold for internal clock*/
	/* threshold for partial empty*/
	/* threshold for partial full */

	/* buffer size for transmit must be 1 from the doc
	   however, it's said that using 0xff ??*/
	enet_writeb(ENET_TX_BUFSIZE(priv->enet_mac_base), 0xff);

	/* fifo control */

	/*MAC mode*/
	enet_mac_config(dev);

	/* check gmii mode support */
	priv->mii.supports_gmii = mii_check_gmii_support(&priv->mii);
	DBG("gmii support=0x%x id=0x%x\n", priv->mii.supports_gmii, priv->mii.phy_id);

	return 0;
}

/*
 * allocate  netdevice structure,  do  all dma  rings allocations  and
 * register the netdevice
 */
extern int tangox_ethernet_getmac(int, unsigned char *);

static int enet_probe(int idx)
{
	struct tangox_enet_priv *priv;
	struct net_device *dev;
	int ret;
	struct sockaddr sock;
	char pad_mode;
	short clk_div;
	unsigned long enet_mac_base;
	const char *name;

	enet_mac_base = eth_mac_cores[idx].enet_mac_base;
	name = eth_mac_cores[idx].name;

#if 1 /* this part may be moved to boot loader */
	/* set pad_mode*/
	pad_mode = enet_readb(enet_mac_base + 0x400);
	enet_writeb(enet_mac_base + 0x400, pad_mode & 0xf0);
	pad_mode = enet_readb(enet_mac_base + 0x400);

	/* set MDIO clock divider */
	clk_div = enet_readw(enet_mac_base + 0x420);
	//DBG("default clk_div =%d\n", clk_div);
	//enet_writew(enet_mac_base + 0x420, 50);
	enet_writew(enet_mac_base + 0x420, tangox_get_sysclock() / (2500000 * 2));
	clk_div = enet_readw(enet_mac_base + 0x420);
	//DBG("clk_div =%d: set MDIO clock=200/%d=%dMHz\n", clk_div, clk_div, 200/(clk_div*2));
#endif

	/* allocate  netdevice structure  with enough  length  for our
	 * context data */
	dev = alloc_etherdev(sizeof (*priv));
	if (!dev)
		return -ENOMEM;

	/* initialize private data */
	priv = netdev_priv(dev);
	memset(priv, 0, sizeof (*priv));
	priv->enet_mac_base = enet_mac_base;
	priv->name = name;
	priv->pending_tx = -1;
	priv->pending_tx_cnt = 0;
	priv->reclaim_limit = -1;
	spin_lock_init(&priv->tx_lock);
	spin_lock_init(&priv->ier_lock);
	spin_lock_init(&priv->maccr_lock);

	/* init tx done tasklet */
	tasklet_init(&priv->tx_reclaim_tasklet, enet_tx_reclaim,
		     (unsigned long)dev);
#if 0
	/* init tx reclaim timer */
	init_timer(&priv->tx_reclaim_timer);
	priv->tx_reclaim_timer.data = (unsigned long )dev;
	priv->tx_reclaim_timer.function = enet_tx_reclaim_timer;
#endif
	/* init link check timer and mii lock */
	init_timer(&priv->link_check_timer);
	priv->link_check_timer.data = (unsigned long)dev;
	priv->link_check_timer.function = enet_link_check;
	spin_lock_init(&priv->mii_lock);

	/* fill mii info */
	priv->mii.dev = dev;
	priv->mii.phy_id_mask  = 0x1f;
	priv->mii.reg_num_mask = 0x1f;
	priv->mii.mdio_read = enet_mdio_read;
	priv->mii.mdio_write = enet_mdio_write;
	
	if (eth_mac_cores[idx].phy_id != -1) {
		/* phy id forced, just check for sanity */
		if (eth_mac_cores[idx].phy_id < 0 || eth_mac_cores[idx].phy_id > 31) {
			ret = -EINVAL;
			goto err_free;
		}
		priv->mii.phy_id = eth_mac_cores[idx].phy_id;

	} else {
		int i;

		/* try to probe phy if not given */
		for (i = 0; i <32; i++) {
			uint32_t id;
			int val;

			val = enet_mdio_read(dev, i, MII_PHYSID1);
			id = (val << 16);
			val = enet_mdio_read(dev, i, MII_PHYSID2);
			id |= val;
			if (id != 0xffffffff && id != 0x00000000) {
				/* check vsc8061 */
				if(id == 0x00070421)
					priv->rgmii = 1;
				break;
			}
		}
		if (i == 32) {
			printk(KERN_ERR "%s: unable to autodetect phy\n", priv->name);
			ret = -EIO;
			goto err_free;
		}
		printk(KERN_ERR "%s: detected phy %s at address 0x%02x\n", name,(priv->rgmii? "vsc8601":""), i);
		priv->mii.phy_id = i;
	}

	printk(KERN_INFO "%s: Ethernet driver for SMP864x/SMP865x internal MAC core %d: %s Base at 0x%lx\n",
				 name, idx, priv->rgmii?"1000Mbps":"100Mbps",  enet_mac_base);

	/* initialize hardware */
	if ((ret = enet_hw_init(dev)))
		goto err_free;

	/* initialize dma descriptors */
	if ((ret = enet_dma_init(priv)))
		goto err_free;

	ret = request_irq(eth_mac_cores[idx].irq, enet_isr, IRQF_DISABLED,
			  eth_mac_cores[idx].name, dev);
	dev->irq = eth_mac_cores[idx].irq;

	if (ret)
		goto err_free;

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
	dev->weight = RX_DESC_COUNT;
	dev->tx_queue_len = TX_DESC_COUNT;
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
		goto err_free;
	}

	printk(KERN_INFO "%s: mac address %02x:%02x:%02x:%02x:%02x:%02x\n", priv->name,
	       dev->dev_addr[0], dev->dev_addr[1], dev->dev_addr[2],
	       dev->dev_addr[3], dev->dev_addr[4], dev->dev_addr[5]);

	eth_mac_cores[idx].gdev = dev;
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

int __init tangox_enet_init(void)
{
	int i;

	/* for command line overwrite */
	eth_mac_cores[0].phy_id = phyid_0;
	eth_mac_cores[1].phy_id = phyid_1;

	for (i = 0; i < NUM_MAC_CORES; i++) {
		if ((1 << i) & mac_cores) {
			if (!tangox_ethernet_enabled(i)) {
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

MODULE_DESCRIPTION("SMP86xx internal ethernet mac driver");
MODULE_AUTHOR("TANGO3 standalone team");
MODULE_LICENSE("GPL");

MODULE_PARM_DESC(phyid_0, "PHY id for core 0, else autodetect");
module_param(phyid_0, int, 0);

MODULE_PARM_DESC(phyid_1, "PHY id for core 1, else autodetect");
module_param(phyid_1, int, 0);

MODULE_PARM_DESC(mac_cores, "MAC core id, 1 for core 0, 2 for core 1, 3 for both");
module_param(mac_cores, int, 0);

