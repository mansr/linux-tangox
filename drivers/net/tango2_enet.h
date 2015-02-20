
/*********************************************************************
 Copyright (C) 2001-2007
 Sigma Designs, Inc. 
 
 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/
  
#ifndef __TANGO2_ENET_H
#define __TANGO2_ENET_H

#include <linux/types.h>
#include <linux/skbuff.h>
#include <linux/mii.h>
#include <linux/timer.h>

#include <asm/addrspace.h>

#ifdef CONFIG_TANGO2
#include <asm/tango2/hardware.h>
#include <asm/tango2/tango2_gbus.h>
#include <asm/tango2/emhwlib_dram_tango2.h>
#else
#error "Unknown architecture"
#endif

#define ENET_IRQ	((IRQ_CONTROLLER_IRQ_BASE) + (LOG2_CPU_ETH_MAC_INT))

/*
 * number of rx/tx buffers available
 * !!! MUST BE A POWER OF 2 !!!
 */
#define ENET_RX_DESC_COUNT	64
#define ENET_TX_DESC_COUNT	64

/*
 * we enable tx  interrupt when there is equal to  this number of free
 * tx desc. Keep it lower than ENET_TX_DESC_COUNT if you change it.
 */
#define ENET_TX_DESC_LOW	20

/*
 * sizeof rx buffers we give to the dma controller
 */
#define RX_BUF_SIZE		0x600

/*
 * ipv4 stack  assumes received  ip header is  32 bits aligned,  so we
 * keep 2 bytes to align ip header
 */
#define SKB_RESERVE_SIZE	2

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
 * address space conversion
 */
#define CACHE_TO_NONCACHE(x)	KSEG1ADDR(x)
#define PHYSADDR(x)		tangox_dma_address(CPHYSADDR(x))


/*
 * Mac/DMA registers offset, refer to documentation
 */
#define ENET_HOST_BASE		REG_BASE_host_interface

/* mac registers */
#define ENET_MAC_BASE		(ENET_HOST_BASE + 0x6000)
#define ENET_MAC_MACCR		(ENET_MAC_BASE + 0x0)
#define MACCR_F			(1 << 20)
#define MACCR_PM		(1 << 19)
#define MACCR_PR		(1 << 18)
#define MACCR_HP		(1 << 13)
#define MACCR_ASTP		(1 << 8)
#define MACCR_TE		(1 << 3)
#define MACCR_RE		(1 << 2)
#define ENET_MAC_MACAHR		(ENET_MAC_BASE + 0x4)
#define ENET_MAC_MACALR		(ENET_MAC_BASE + 0x8)
#define ENET_MAC_MAHR		(ENET_MAC_BASE + 0xc)
#define ENET_MAC_MALR		(ENET_MAC_BASE + 0x10)
#define ENET_MAC_MIIAR		(ENET_MAC_BASE + 0x14)
#define MIIAR_ADDR(x)		((x) << 11)
#define MIIAR_REG(x)		((x) << 6)
#define MIIAR_WRITE		(1 << 1)
#define MIIAR_BUSY		(1 << 0)
#define ENET_MAC_MIIDR		(ENET_MAC_BASE + 0x18)
#define ENET_MAC_FCR		(ENET_MAC_BASE + 0x1c)
#define FCR_ENABLE		(1 << 1)

/* dma registers */
#define ENET_DMA_BASE		(ENET_HOST_BASE + 0x7000)
#define ENET_DMA_BMR		(ENET_DMA_BASE + 0x00)
#define BMR_PBL(x)		((x & 0x3f) << 8)
#define BMR_SWR			(1 << 0)
#define ENET_DMA_TPDR		(ENET_DMA_BASE + 0x04)
#define ENET_DMA_RPDR		(ENET_DMA_BASE + 0x08)
#define ENET_DMA_RBAR		(ENET_DMA_BASE + 0x0c)
#define ENET_DMA_TBAR		(ENET_DMA_BASE + 0x10)
#define ENET_DMA_SR		(ENET_DMA_BASE + 0x14)
#define SR_EB(x)		(((x) >> 23) & 0x3)
#define SR_TS(x)		(((x) >> 20) & 0x7)
#define SR_RS(x)		(((x) >> 17) & 0x7)
#define SR_NIS			(1 << 16)
#define SR_AIS			(1 << 15)
#define SR_ERI			(1 << 14)
#define SR_FBE			(1 << 13)
#define SR_ETI			(1 << 12)
#define SR_RWT			(1 << 11)
#define SR_RPS			(1 << 8)
#define SR_RU			(1 << 7)
#define SR_R			(1 << 6)
#define SR_UNF			(1 << 5)
#define SR_TU			(1 << 4)
#define SR_TPS			(1 << 1)
#define SR_T			(1 << 0)
#define ENET_DMA_CR		(ENET_DMA_BASE + 0x18)
#define CR_SF			(1 << 21)
#define CR_ST			(1 << 13)
#define CR_SR			(1 << 1)
#define ENET_DMA_IER		(ENET_DMA_BASE + 0x1c)
#define IER_NIS			(1 << 16)
#define IER_AIS			(1 << 15)
#define IER_ERE			(1 << 14)
#define IER_FBE			(1 << 13)
#define IER_ETE			(1 << 10)
#define IER_RWT			(1 << 9)
#define IER_RS			(1 << 8)
#define IER_RU			(1 << 7)
#define IER_R			(1 << 6)
#define IER_UE			(1 << 5)
#define IER_TBU			(1 << 2)
#define IER_TS			(1 << 1)
#define IER_T			(1 << 0)
#define ENET_DMA_CHRBA		(ENET_DMA_BASE + 0x54)


/*
 * rx dma descriptor definition
 */
struct enet_rx_desc {
	unsigned long rdes0;
	unsigned long rdes1;
	unsigned long rdes2;
	unsigned long rdes3;
};

#define RDES0_OWN		(1 << 31)
#define RDES0_FILTER_FAIL	(1 << 30)
#define RDES0_FRAME_LEN(x)	(((x) & 0x1fff0000) >> 16)
#define RDES0_ERR_SUM		(1 << 15)
#define RDES0_TRUNC		(1 << 14)
#define RDES0_FIRST		(1 << 9)
#define RDES0_LAST		(1 << 8)
#define RDES0_TOO_LONG		(1 << 7)
#define RDES0_COLLISION		(1 << 6)
#define RDES0_WATCHDOG_TMOUT	(1 << 4)
#define RDES0_MII_ERROR		(1 << 3)
#define RDES0_CRC		(1 << 1)

#define RDES1_DISABLE_ISR	(1 << 31)
#define RDES1_RER		(1 << 25)
#define RDES1_RCH		(1 << 24)
#define RDES1_RBS2(x)		((x) << 11)
#define RDES1_RBS1(x)		(x)



/*
 * tx dma descriptor definition
 */
struct enet_tx_desc {
	uint32_t tdes0;
	uint32_t tdes1;
	uint32_t tdes2;
	uint32_t tdes3;
};

#define TDES0_OWN		(1 << 31)
#define TDES0_ERR_SUM		(1 << 15)
#define TDES0_CARRIER_LOST	(1 << 11)
#define TDES0_NO_CARRIER	(1 << 10)
#define TDES0_LATE_COLLISION	(1 << 9)
#define TDES0_EXC_COLLISION	(1 << 8)
#define TDES0_HEARTBEAT		(1 << 7)
#define TDES0_EXC_DEFERAL	(1 << 2)
#define TDES0_UNDERFLOW		(1 << 1)

#define TDES1_ENABLE_ISR	(1 << 31)
#define TDES1_LAST	 	(1 << 30)
#define TDES1_FIRST		(1 << 29)
#define TDES1_DISABLE_FCS	(1 << 26)
#define TDES1_TER	 	(1 << 25)
#define TDES1_TCH	 	(1 << 24)
#define TDES1_DISABLE_PADDING	(1 << 23)
#define TDES1_TBS2(x)		((x) << 11)
#define TDES1_TBS1(x)		(x)


/*
 * our private context
 */
struct tango2_enet_priv {

	/*
	 * rx related
	 */

	/* pointer to rx descriptor array */
	volatile struct enet_rx_desc	*rx_descs;
	struct enet_rx_desc		*rx_descs_cached;

	/* last rx descriptor processed */
	unsigned int			last_rx_desc;

	/* we keep a list of skb given */
	struct sk_buff			*rx_skbs[ENET_RX_DESC_COUNT];

	/* ethernet device stats */
	struct net_device_stats		stats;




	/*
	 * tx related
	 */

	/* access  to  tx related  dma  stuffs  is  protected by  this
	 * spinlock, this is because we  access them via a tasklet and
	 * a timer */
	spinlock_t			tx_lock;

	/* pointer to tx descriptor array */
	volatile struct enet_tx_desc	*tx_descs;
	struct enet_tx_desc		*tx_descs_cached;

	/* index of current dirty tx descriptor */
	unsigned int			dirty_tx_desc;

	/* index of next clean tx descriptor to use */
	unsigned int			next_tx_desc;

	/* count of free tx desc to avoid its computation */
	unsigned int			free_tx_desc_count;

	/* list of sent skb */
	struct sk_buff			*tx_skbs[ENET_TX_DESC_COUNT];

	/* tx  done operation is  done under  these tasklet  and timer
	 * context */
	struct tasklet_struct		tx_reclaim_tasklet;
	struct timer_list		tx_reclaim_timer;


	/*
	 * misc
	 */

	/* spinlock used to protect interrupt enable register */
	spinlock_t			ier_lock;

	/* spinlock used to protect maccr register */
	spinlock_t			maccr_lock;

	/* our mii state */
	struct mii_if_info		mii;

	/* mii access is protected by following spinlock */
	spinlock_t			mii_lock;

	/* link status is checked periodically by this timer */
	struct timer_list		link_check_timer;
};

/*
 * helpers to access registers
 */

static inline unsigned long enet_readl(unsigned long addr)
{
	unsigned long tmp = 0;
	unsigned long res = gbus_read_reg32(addr);
	extern int is_tango2_es6(void);
	if (is_tango2_es6()) 
		tmp = gbus_read_reg32(REG_BASE_system_block + SYS_xtal_in_cnt);
	return(res);
}

static inline void enet_writel(unsigned long addr, unsigned long data)
{
	gbus_write_reg32(addr, data);

	/* some write  read sequence seems  to freeze completly  if no
	 * barrier is done between each access. To be safe, we do this
	 * after all write access */
	iob();
}

#endif /* __TANGO2_ENET_H */

