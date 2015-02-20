/*********************************************************************
 Copyright (C) 2001-2008
 Sigma Designs, Inc. 

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/

#ifndef __TANGOX_ENET_H
#define __TANGOX_ENET_H

#include <linux/types.h>
#include <linux/skbuff.h>
#include <linux/mii.h>
#include <linux/timer.h>

#include <asm/addrspace.h>

#if defined(CONFIG_TANGO3)
#include <asm/tango3/hardware.h>
#include <asm/tango3/tango3_gbus.h>
#else
#error "Unknown architecture"
#endif

#define ENET_IRQ0	((IRQ_CONTROLLER_IRQ_BASE) + (LOG2_CPU_ETH_MAC_INT))
#define ENET_IRQ1	((IRQ_CONTROLLER_IRQ_BASE) + (LOG2_CPU_ETH_MAC1_INT))

/*
 * number of rx/tx buffers available
 * !!! MUST BE A POWER OF 2 !!!
 */
#define RX_DESC_COUNT   256 
#define TX_DESC_COUNT	256	

/*
 * we enable tx  interrupt when there is equal to  this number of free
 * tx desc. Keep it lower than TX_DESC_COUNT if you change it.
 */
#define ENET_DESC_LOW 32	

/*
 * sizeof rx buffers we give to the dma controller
 */
#define RX_BUF_SIZE		1534	// 0x600

/* since the Ethernet header is 14 bytes, so that we need 2 more bytes to align. 
 * but s_addr need to be 4 bytes aligned, so that we have trade off here
 */
#define SKB_RESERVE_SIZE	0	

/*
 * we reclaim  transmited using a  timer, we switch to  interrupt mode
 * under high load. this is the timer frequency
 */
#define TX_RECLAIM_TIMER_FREQ	(HZ / 100)

/*
 * link status  is polled on a regular  basis by a timer,  this is its
 * frequency
 */
#define LINK_CHECK_TIMER_FREQ	(HZ)

/*
 * getting system frequency
 */
extern unsigned long tangox_get_sysclock(void);

/*
 * address space conversion
 */
#define CACHE_TO_NONCACHE(x)	KSEG1ADDR(x)

/*
 * dma address translation
 */
static inline unsigned long PHYSADDR(void *addr) 
{
	return tangox_dma_address(CPHYSADDR((unsigned long)addr));
}

/*
 * Mac/DMA registers offset, refer to documentation
 */
#define ENET_TX_CTL1(mac_base)		((mac_base) + 0x00)
#define TX_TPD		        	(1 << 5)
#define TX_APPEND_FCS       		(1 << 4)
#define TX_PAD_EN          		(1 << 3)
#define TX_RETRY_EN        		(1 << 2)
#define TX_EN		        	(1 << 0)
#define ENET_TX_CTL2(mac_base)		((mac_base) + 0x01)
#define ENET_RX_CTL(mac_base)	    	((mac_base) + 0x04)
#define RX_BC_DISABLE			(1 << 7)
#define RX_RUNT		        	(1 << 6)
#define RX_AF_EN			(1 << 5)
#define RX_PAUSE_EN	     		(1 << 3)
#define RX_SEND_CRC	    		(1 << 2)
#define RX_PAD_STRIP	   		(1 << 1)
#define RX_EN		        	(1 << 0)
#define ENET_RANDOM_SEED(mac_base)	((mac_base) + 0x8)
#define ENET_TX_SDP(mac_base)	    	((mac_base) + 0x14)
#define ENET_TX_TPDP1(mac_base)		((mac_base) + 0x18)
#define ENET_TX_TPDP2(mac_base)		((mac_base) + 0x19)
#define ENET_SLOT_TIME(mac_base)	((mac_base) + 0x1c)
#define ENET_MDIO_CMD1(mac_base)	((mac_base) + 0x20)
#define ENET_MDIO_CMD2(mac_base)	((mac_base) + 0x21)
#define ENET_MDIO_CMD3(mac_base)	((mac_base) + 0x22)
#define ENET_MDIO_CMD4(mac_base)	((mac_base) + 0x23)
#define MIIAR_ADDR(x)      		((x) << 21)
#define MIIAR_REG(x)       		((x) << 16)
#define MIIAR_DATA(x)      		((x) <<  0)
#define MDIO_CMD_GO	    		(1 << 31)
#define MDIO_CMD_WR   			(1 << 26)
#define ENET_MDIO_STS1(mac_base)	((mac_base) + 0x24)
#define ENET_MDIO_STS2(mac_base)	((mac_base) + 0x25)
#define ENET_MDIO_STS3(mac_base)	((mac_base) + 0x26)
#define ENET_MDIO_STS4(mac_base)	((mac_base) + 0x27)
#define MDIO_STS_ERR			(1 << 31)
#define ENET_MC_ADDR1(mac_base)		((mac_base) + 0x28)
#define ENET_MC_ADDR2(mac_base)		((mac_base) + 0x29)
#define ENET_MC_ADDR3(mac_base)		((mac_base) + 0x2a)
#define ENET_MC_ADDR4(mac_base)		((mac_base) + 0x2b)
#define ENET_MC_ADDR5(mac_base)		((mac_base) + 0x2c)
#define ENET_MC_ADDR6(mac_base)		((mac_base) + 0x2d)
#define ENET_MC_INIT(mac_base)		((mac_base) + 0x2e)
#define ENET_UC_ADDR1(mac_base)		((mac_base) + 0x3c)
#define ENET_UC_ADDR2(mac_base)		((mac_base) + 0x3d)
#define ENET_UC_ADDR3(mac_base)		((mac_base) + 0x3e)
#define ENET_UC_ADDR4(mac_base)		((mac_base) + 0x3f)
#define ENET_UC_ADDR5(mac_base)		((mac_base) + 0x40)
#define ENET_UC_ADDR6(mac_base)		((mac_base) + 0x41)
#define ENET_MAC_MODE(mac_base)		((mac_base) + 0x44)
#define RGMII_MODE	    		(1 << 7)
#define HALF_DUPLEX  			(1 << 4)
#define BST_EN	    	 		(1 << 3)
#define LB_EN	    			(1 << 2)
#define GMAC_MODE	    		(1 << 0)
#define ENET_IC_THRESHOLD(mac_base)	((mac_base) + 0x50)
#define ENET_PE_THRESHOLD(mac_base)	((mac_base) + 0x51)
#define ENET_PF_THRESHOLD(mac_base)	((mac_base) + 0x52)
/* TX buffer size must be set to 0x01 ??*/
#define ENET_TX_BUFSIZE(mac_base)	((mac_base) + 0x54)
#define ENET_FIFO_CTL(mac_base)		((mac_base) + 0x56)
#define ENET_PQ1(mac_base)	    	((mac_base) + 0x60)
#define ENET_PQ2(mac_base)	    	((mac_base) + 0x61)
#define ENET_MAC_ADDR1(mac_base)	((mac_base) + 0x6a)
#define ENET_MAC_ADDR2(mac_base)	((mac_base) + 0x6b)
#define ENET_MAC_ADDR3(mac_base)	((mac_base) + 0x6c)
#define ENET_MAC_ADDR4(mac_base)	((mac_base) + 0x6d)
#define ENET_MAC_ADDR5(mac_base)	((mac_base) + 0x6e)
#define ENET_MAC_ADDR6(mac_base)	((mac_base) + 0x6f)
#define ENET_STAT_DATA1(mac_base)	((mac_base) + 0x78)
#define ENET_STAT_DATA2(mac_base)	((mac_base) + 0x79)
#define ENET_STAT_DATA3(mac_base)	((mac_base) + 0x7a)
#define ENET_STAT_DATA4(mac_base)	((mac_base) + 0x7b)
#define ENET_STAT_INDEX(mac_base)	((mac_base) + 0x7c)
#define ENET_STAT_CLEAR(mac_base)	((mac_base) + 0x7d)
#define ENET_SLEEP_MODE(mac_base)	((mac_base) + 0x7e)
#define SLEEP_MODE	    		(1 << 0)
#define ENET_WAKEUP(mac_base)	    	((mac_base) + 0x7f)
#define WAKEUP		    		(1 << 0)

#define ENET_TXC_CR(mac_base)		((mac_base) + 0x100)
#define TCR_LK		    		(1 << 12)
#define TCR_DS		    		(1 << 11)
#define TCR_BTS(x)          		(((x) & 0x7) << 8)
#define TCR_DIE		    		(1 << 7)
#define TCR_TFI(x)          		(((x) & 0x7) << 4)
#define TCR_LE		    		(1 << 3)
#define TCR_RS		    		(1 << 2)
#define TCR_DM		    		(1 << 1)
#define TCR_EN		    		(1 << 0)

#define ENET_TXC_SR(mac_base)		((mac_base) + 0x104)
#define TSR_DE				(1 << 3)
#define TSR_DI				(1 << 2)
#define TSR_TO				(1 << 1)
#define TSR_TI				(1 << 0)

#define ENET_TX_SAR(mac_base)		((mac_base) + 0x108)
#define ENET_TX_DESC_ADDR(mac_base)	((mac_base) + 0x10c)

#define ENET_TX_REPORT_ADDR(mac_base)	((mac_base) + 0x110)
#define TX_BYTES_TRASFERRED(x)		(((x) >> 16) & 0xffff)
#define TX_FIRST_DEFERRAL		(1 << 7)
#define TX_EARLY_COLLISIONS(x)		(((x) >> 3) & 0xf)
#define TX_LATE_COLLISION		(1 << 2)
#define TX_PACKET_DROPPED		(1 << 1)
#define TX_FIFO_UNDERRUN		(1 << 0)

#define ENET_TX_FIFO_SR(mac_base)	((mac_base) + 0x114)
#define ENET_TX_ITR(mac_base)		((mac_base) + 0x118)

#define ENET_RXC_CR(mac_base)		((mac_base) + 0x200)
#define RCR_FI				(1 << 13)
#define RCR_LK				(1 << 12)
#define RCR_DS				(1 << 11)
#define RCR_BTS(x)         		(((x) & 7) << 8)
#define RCR_DIE				(1 << 7)
#define RCR_RFI(x)         		(((x) & 7) << 4)
#define RCR_LE				(1 << 3)
#define RCR_RS				(1 << 2)
#define RCR_DM				(1 << 1)
#define RCR_EN				(1 << 0)

#define ENET_RXC_SR(mac_base)		((mac_base) + 0x204)
#define RSR_DE				(1 << 3)
#define RSR_DI				(1 << 2)
#define RSR_RO				(1 << 1)
#define RSR_RI				(1 << 0)

#define ENET_RX_SAR(mac_base)		((mac_base) + 0x208)
#define ENET_RX_DESC_ADDR(mac_base)	((mac_base) + 0x20c)

#define ENET_RX_REPORT_ADDR(mac_base)	((mac_base) + 0x210)
#define RX_BYTES_TRANSFERRED(x)		(((x) >> 16) & 0xFFFF)
#define RX_MULTICAST_PKT		(1 << 9)
#define RX_BROADCAST_PKT		(1 << 8)
#define RX_LENGTH_ERR			(1 << 7)
#define RX_FCS_ERR			(1 << 6)
#define RX_RUNT_PKT			(1 << 5)
#define RX_FIFO_OVERRUN			(1 << 4)
#define RX_LATE_COLLISION		(1 << 3)
#define RX_FRAME_LEN_ERROR		(1 << 2)

#define ENET_RX_FIFO_SR(mac_base)	((mac_base) + 0x214)
#define ENET_RX_ITR(mac_base)		((mac_base) + 0x218)

/*
 * rx dma descriptor definition
 */
struct enet_desc {
	unsigned long s_addr;
	unsigned long n_addr;
	unsigned long r_addr;
	unsigned long config;
}; 

#define DESC_ID				(1 << 23)
#define DESC_EOC			(1 << 22)
#define DESC_EOF			(1 << 21)
#define DESC_LK				(1 << 20)
#define DESC_DS				(1 << 19)
#define DESC_BTS(x)			(((x) & 0x7) <<16)
#define DESC_TC(x)			(x)

/*
 * our private context
 */
struct tangox_enet_priv {

	/*
	 * rx related
	 */

	/* pointer to rx descriptor array */
	volatile struct enet_desc	*rx_descs __attribute__((aligned(32)));
	struct enet_desc		*rx_descs_cached __attribute__((aligned(32)));

	/* last rx descriptor processed */
	unsigned int			last_rx_desc;

	/* we keep a list of skb given */
	struct sk_buff			*rx_skbs[RX_DESC_COUNT] __attribute__((aligned(32)));
	
	/* desc status report */
	unsigned long 			rx_report[RX_DESC_COUNT] __attribute__((aligned(32)));

	/* ethernet device stats */
	struct net_device_stats		stats;

	/* rx stopped? */
	int				rx_stopped;

	/* tracking the EOC */
	unsigned int			rx_eoc;

	/*
	 * tx related
	 */

	/* access  to  tx related  dma  stuffs  is  protected by  this
	 * spinlock, this is because we  access them via a tasklet and
	 * a timer */
	spinlock_t			tx_lock;

	/* pointer to tx descriptor array */
	volatile struct enet_desc	*tx_descs __attribute__((aligned(32)));
	struct enet_desc		*tx_descs_cached __attribute__((aligned(32)));

	/* index of current dirty tx descriptor */
	unsigned int			dirty_tx_desc;

	/* index of next clean tx descriptor to use */
	unsigned int			next_tx_desc;

	/* count of free tx desc to avoid its computation */
	unsigned int			free_tx_desc_count;

	/* list of sent skb */
	struct sk_buff			*tx_skbs[TX_DESC_COUNT] __attribute__((aligned(32)));

	/* list of tx_bufs */
	unsigned char 			*tx_bufs[TX_DESC_COUNT];
	
	/* desc status report */
	unsigned long 			tx_report[RX_DESC_COUNT] __attribute__((aligned(32)));

	/* tx  done operation is  done under  these tasklet  and timer
	 * context */
	struct tasklet_struct		tx_reclaim_tasklet;
	struct timer_list		tx_reclaim_timer;

	/* the 1st pending tx  */
	int 				pending_tx;
	
	/* how many pending desc in this pending tx*/
	int 				pending_tx_cnt;

	/* reclaim to this limit */
	int 				reclaim_limit;

	/* tracking the EOC */
	unsigned int			tx_eoc;

	/*
	 * misc
	 */

	/* base address of enet registers */
	unsigned long			enet_mac_base;

	/* device name */
	const char 			*name;

	/* spinlock used to protect interrupt enable register */
	spinlock_t			ier_lock;

	/* spinlock used to protect maccr register */
	spinlock_t			maccr_lock;

	/* our mii state */
	struct mii_if_info	mii;

	/* mii access is protected by following spinlock */
	spinlock_t			mii_lock;

	/* link status is checked periodically by this timer */
	struct timer_list	link_check_timer;

	/* rgmii */
	int rgmii;
};

/*
 * helpers to access registers
 */
#define enet_readl(x)	gbus_read_uint32(0, x)
#define enet_readw(x)	gbus_read_uint16(0, x)
#define enet_readb(x)	gbus_read_uint8(0, x)

static inline void enet_writel(unsigned long addr, unsigned long data)
{
        gbus_write_uint32(0, addr, data);

        /* some write  read sequence seems  to freeze completly  if no
         * barrier is done between each access. To be safe, we do this
         * after all write access */
        iob();
}

static inline void enet_writew(unsigned long addr, unsigned short data)
{
        gbus_write_uint16(0, addr, data);
        iob();
}

static inline void enet_writeb(unsigned long addr, unsigned char data)
{
        gbus_write_uint8(0, addr, data);
        iob();
}

#endif /* __TANGOX_ENET_H */
