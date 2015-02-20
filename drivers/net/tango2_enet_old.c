
/*********************************************************************
 Copyright (C) 2001-2007
 Sigma Designs, Inc. 
 
 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/
  
/*
 *  Embedded Ethernet driver for TANGO2 ES4 or above
 */
#ifdef BOOTLOADER
#include "config.h"
#include "version.h"
#include "util.h"
#include "net.h"
#include "net_ipv4.h"
#include "em86xxapi.h"
#include "irqs.h"
#include "errno.h"
#include "uart.h"
#include "memcfg.h"
#else
#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/module.h>
//#include <linux/compatmac.h>
#include <linux/mm.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/mii.h>
#include <linux/ethtool.h>
#include <linux/crc32.h>

#ifdef CONFIG_TANGO2
#include <asm/tango2/memcfg.h>
#else
#include <asm/virtconvert.h>
#include <asm/arch/irqs.h>
#include <asm/arch/memcfg.h>
#endif

// Use kernel timer implementation for polling the link status
// #define USE_KERNEL_TIMER

#endif //BOOTLOADER

#include "tango2_enet_old.h"

#define DRV_NAME	"em86xx-eth"
#define DRV_VERSION     "-0.0.0.0.1"

/* Uncomment this for different messages */
//#define ETH_DEBUG 
#define ETH_ERRMSG 
#define ETH_MSG
//#define ETH_DBG_INOUT
//#define DEBUG_RX_TX_STATE
//#define DEBUG_MDIO

#ifdef BOOTLOADER
#define DRIVER "em86xx_eth"

/* For static buffer allocation */
// #define STATIC_BUF_ALLOC
#define printk          uart_printf

#define PAGE_SHIFT	12
#define PAGE_SIZE	(1UL << PAGE_SHIFT)
#define PAGE_MASK	(~(PAGE_SIZE-1))
#define MAX_ORDER	10

#endif /* BOOTLOADER */

/* For software filtering*/
//#define USE_SW_FILTERING
#define USE_HW_FILTERING

#ifdef ETH_DEBUG
#define DBG_PRINT	printk
#else
static void inline DBG_PRINT(const char *x, ...) { ; }
#endif /* ETH_DEBUG */

#ifdef ETH_ERRMSG
#define ERR_PRINT	printk
#else
static void inline ERR_PRINT(const char *x, ...) { ; }
#endif /* ETH_ERRMSG */

#ifdef ETH_MSG
#define MSG_PRINT	printk
#else
static void inline MSG_PRINT(const char *x, ...) { ; }
#endif /* ETH_ERRMSG */

#ifdef ETH_DBG_INOUT
#define DBG_PRINT_INOUT_FUNC(x) printk("%s: %s\n", __func__, x);
#define MSG_PRINT	printk
#else
static void inline DBG_PRINT_INOUT_FUNC(const char *x, ...) { ; }
#endif /* ETH_ERRMSG */

/* Hack: this is the mac address by default */
#define DEF_MAC_HI	0x00000011
#ifdef BOOTLOADER
#define DEF_MAC_LO	0xdeadbeef
#else
#define DEF_MAC_LO	0xaabbccdd
#endif

/* Default maximum link wait */
#ifdef USE_KERNEL_TIMER
#define DEF_LINK_LOOP		0x10000
#else
#define DEF_LINK_LOOP		0x100
#endif
#define DEF_MAX_LINK_LOOP	(DEF_LINK_LOOP * 16)

#define ETH_IRQ_FLAGS   (DmaIntNormal     | DmaIntAbnormal    | DmaIntRxStopped         | \
		         DmaIntRxNoBuffer | DmaIntRxCompleted | DmaIeTxUnderflow        | \
                         DmaIeTxStopped   | DmaIeTxCompleted) 
/*	(DmaIntNormal	  | DmaIntAbnormal    | DmaIntRxStopped		| \
			 DmaIntRxNoBuffer | DmaIntRxCompleted | DmaIntTxCompleted)
*/
#define ETH_RXTX_FLAGS	(DmaTxStart       | DmaRxStart	      | DmaStoreAndForward)

#ifndef USE_HW_FILTERING 
#define ETH_MAC_FLAGS   (MacTxEnable      |  MacRxEnable       | MacPromiscuousModeOn	| \
                        MacPadStripEnable |  MacHashFilterOn   | MacPerfectFilterOff 	| \
                        MacRetryEnable    |  MacHeartBeatOff)
#else 
#define ETH_MAC_FLAGS	(MacTxEnable   	  | MacRxEnable        | MacPromiscuousModeOff  | \
			MacPadStripEnable | MacHashFilterOn    | MacPerfectFilterOn     | \
                        MacRetryEnable    | MacMulticastFilterOn | MacHeartBeatOff)
#define DEFAULT_BROADCAST_ADDRESS { 0xFF,0xFF,0xFF,0xFF,0xFF,0xFF }
static void em86xx_eth_set_multicast_list(struct net_device *dev);
#endif /*USE_HW_FILTERING*/

#define MD_FRAME_FLAGS  (DescRxLast        | DescRxFirst)
#define TX_DESC1_FLAGS 	(DescTxLast        | DescTxFirst        | DescChain             | \
			 DescTxIntEnable)
#define RX_ERROR_FLAGS  (DescFilteringFail | DescError		| DescRxTruncated	| \
			 DescLengthError   | DescRxDamaged 	| DescRxLongFrame	| \
			 DescRxCollision   | DescRxMiiError	| DescRxCrc)
#define TX_ERROR_FLAGS  (DescTxLostCarrier | DescTxNoCarrier	| DescTxLateCollision 	| \
			 DescTxExcCollisions| DescTxHeartbeatFail| DescTxUnderflow)

#if defined(CONFIG_TANGOX) 
extern int tangox_ethernet_getmac(int idx, unsigned char *mac);
extern int tangox_ethernet_enabled(int);
#endif

#define PHY_10BASET_HD_ANAR  	 (CSMACD | BASET10 | FLOWCONTROL | RF)
#define PHY_10BASET_FD_ANAR 	 (CSMACD | BASET10 | BASET10FD | FLOWCONTROL | RF)
#define PHY_100BASET_HD_ANAR	 (CSMACD | BASET10 | BASET10FD | BASET100 | FLOWCONTROL | RF)
#define PHY_100BASET_FD_ANAR	 (CSMACD | BASET10 | BASET10FD | BASET100 | BASET100FD | FLOWCONTROL | RF)

/* Number of descriptors and buffer size */
#define MIN_NUM_RDESC	8
#define MIN_NUM_TDESC	8
#define NUM_RDESC 	64
#define NUM_TDESC	64
#define R_BUF_SIZE	0x700	/* needs to be < 2KB */
#define T_BUF_SIZE	0x700	/* needs to be < 2KB */

#if (R_BUF_SIZE < (1 << 14))
/* For multi-descriptor frame support: only when receive buffer size < 16384 */
#define MULTI_DESCS_FRAME_SUPPORT
#endif

/* Private data for this ethernet device */
typedef struct {
	struct net_device_stats stats; /* stats */
	unsigned long last_rxidx;      /* last rx idx to descriptor */
	unsigned long next_txidx;      /* next tx idx to descriptor */


	spinlock_t lock;
	/* MII transceiver section. */
	struct mii_if_info mii_if;
        /* Mode selection */
        int autoneg_active;
        int phy_loopback;	

	/* Descriptors and buffers: notice the alignments */
#ifdef STATIC_BUF_ALLOC 
	struct em86xx_desc eth_rxdsc[NUM_RDESC] __attribute__ ((__aligned__(16)));
	struct em86xx_desc eth_txdsc[NUM_TDESC] __attribute__ ((__aligned__(16)));
	unsigned char eth_rxbuf[NUM_RDESC * R_BUF_SIZE] __attribute__ ((__aligned__(16)));
	unsigned char eth_txbuf[NUM_TDESC * T_BUF_SIZE] __attribute__ ((__aligned__(16)));
#else
	struct em86xx_desc *eth_rxdsc;
	struct em86xx_desc *eth_txdsc;
	unsigned char *eth_rxbuf;
	unsigned char *eth_txbuf;
	unsigned long desc_page;
	unsigned long rxbuf_pages;
	unsigned long txbuf_pages;
#ifndef BOOTLOADER
	unsigned long rxbuf_order;
	unsigned long txbuf_order;
#endif
#endif

	/* Number of descriptors */
	unsigned long num_rxdesc;
	unsigned long num_txdesc;

/* The real data pointer to be accessed in non-cache region */
	volatile struct em86xx_desc *rxdsc;
	volatile struct em86xx_desc *txdsc;
	volatile unsigned char *rxbuf;
	volatile unsigned char *txbuf;

	/* Reference count for device */
	int dev_count;

	int reset_flag;
	int phy_id;
	int stop_thread;

#ifndef BOOTLOADER
#ifdef USE_KERNEL_TIMER 
	struct timer_list eth_timer;
#endif
	struct sk_buff **rx_skb_list;
	int need_restart_tx_queue; 
#endif

} EM86XX_ETH_PRIV;

/* For storing MAC address */
static unsigned long def_mac_hi = DEF_MAC_HI;
static unsigned long def_mac_lo = DEF_MAC_LO;
unsigned long num_rxdesc_param = NUM_RDESC;
unsigned long num_txdesc_param = NUM_TDESC;

/* Default PHY address: -1 for search */
static int phy_address = -1;
/* Default to 100BaseT/FD*/

static int em86xx_eth_reset_desc(struct net_device *dev, int *reset);

#ifdef BOOTLOADER
static void em86xx_eth_intr_handler(int irq, void *dev_id);
#else
static struct net_device em86xx_eth_dev;
static irqreturn_t em86xx_eth_intr_handler(int irq, void *dev_id);
static int netdev_get_settings(struct net_device *dev, struct ethtool_cmd *cmd);

MODULE_AUTHOR("Craig Qu");
MODULE_DESCRIPTION("Sigma Designs ethernet driver");
MODULE_LICENSE("GPL");
MODULE_PARM_DESC(phy_address, "PHY address connecting to the MAC.");
module_param(def_mac_hi, int, 0);
module_param(def_mac_lo, int, 0);
module_param(phy_address, int, 0);
#ifndef STATIC_BUF_ALLOC 
module_param(num_rxdesc_param, int, 0);
module_param(num_txdesc_param, int, 0);
#endif
#endif /* BOOTLOADER */

#define TX_ERRORS       12
#define TX_ERRORS_START 1

#define RX_ERRORS       31
#define RX_ERRORS_START 1

static const char *rx_error_msg[RX_ERRORS]= {
	NULL,
	"A CRC error is detected.",
	NULL,
	"A receive error was detected during frame reception.",
	NULL,NULL,
	"The frame has seen a collision after the collision window.",
	"The frame is longer than 1518 bytes.",
	NULL,NULL,NULL,
	"The frame is prematurely terminated before the collision window.",	
	"The actual length does not match with the Length Type field of the incoming frame.",
	NULL,
	"No more descriptors for receive frame.",
	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	"The frame failed the address recognizing filtering.",
};

static const int rx_stat_offset[RX_ERRORS] = {
	-1,
	(int)&(((struct net_device_stats *)0)->rx_crc_errors),
	-1,-1,-1,-1,
	(int)&(((struct net_device_stats *)0)->collisions),
	-1,-1,-1,-1,-1,
	(int)&(((struct net_device_stats *)0)->rx_length_errors),
	-1,
	(int)&(((struct net_device_stats *)0)->rx_over_errors),
	-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
}; 

static const char *tx_error_msg[TX_ERRORS]= {
	NULL,
	"Late data arrival from the memory.",
	"Excessive deferral.",
	NULL,NULL,NULL,NULL,
	"Heartbeat collision check failure.",
	"Transmission aborted after 16 collisions.",
	"Transmission aborted due to collision.",
	"No carrier signal from the tranceiver.",
	"Loss of carrier during transmission.",	
};

static const int tx_stat_offset[TX_ERRORS] = {
	-1,
	(int)&(((struct net_device_stats *)0)->tx_aborted_errors),
	(int)&(((struct net_device_stats *)0)->tx_aborted_errors),
	-1,-1,-1,-1,
	(int)&(((struct net_device_stats *)0)->tx_heartbeat_errors),
	(int)&(((struct net_device_stats *)0)->collisions),
	(int)&(((struct net_device_stats *)0)->collisions),
	(int)&(((struct net_device_stats *)0)->tx_carrier_errors),
	(int)&(((struct net_device_stats *)0)->tx_carrier_errors),
}; 

/* Helper routine */
static void wait_ms (int ms)
{
	DBG_PRINT_INOUT_FUNC("START");
	if (!in_interrupt()) {
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(1 + ms * HZ / 1000);
	} else {
		mdelay(ms);
	}
	DBG_PRINT_INOUT_FUNC("END");
}


/* MDIO access routines - used by MII standard Linux API */
static int mdio_read (struct net_device *dev, int phy_addr, int reg_num)
{
	u32 addr;
	u16 ret;
	int count;
	
#ifdef DEBUG_MDIO
	DBG_PRINT("%s: start addr %d reg %d\n", __func__, phy_addr, reg_num);
#endif
    /* Wait while the PHY is busy */
	for (count = 1000; count != 0; count--) {
		if (!(em86xx_read_reg(EM86XX_MIIAR_REG) & MiiBusy))
			break;
	}
	/* Set PHY & register to read */
	addr = ((phy_addr & 0x1F) << 11) | ((reg_num & 0x1F) << 6);
	em86xx_write_reg(EM86XX_MIIAR_REG, addr);
	/* Work-around Sigma-Designs MAC bug */
	udelay(40);
	/* Wait while PHY is busy */
	for (count = 1000; count != 0; count--) {
		if (!(em86xx_read_reg(EM86XX_MIIAR_REG) & MiiBusy))
			break;
	}
	ret = em86xx_read_reg(EM86XX_MIIDR_REG) & 0xFFFF;
#ifdef DEBUG_MDIO
	DBG_PRINT(" => %04x\n", ret);
#endif
	
	return ret;
}

static void mdio_write (struct net_device *dev,
                        int phy_addr, int reg_num, int value)
{
	u32 addr;
	int count;
	
#ifdef DEBUG_MDIO
	DBG_PRINT("%s: start addr %d reg %d val %04x\n",
	       __func__, phy_addr, reg_num, value);
#endif
	/* Wait while the PHY is busy */
	for (count = 1000; count != 0; count--) {
		if (!(em86xx_read_reg(EM86XX_MIIAR_REG) & MiiBusy))
			break;
	}
	/* Set PHY & register to write */
	addr = ((phy_addr & 0x1F) << 11) | ((reg_num & 0x1F) << 6) | (1 << 1);
	em86xx_write_reg(EM86XX_MIIDR_REG, value);
	em86xx_write_reg(EM86XX_MIIAR_REG, addr);

	while (em86xx_read_reg(EM86XX_MIIAR_REG) & MiiBusy);

}

static int dump_ethtool_cmd(struct ethtool_cmd *ep)
{

        printk("       Speed: ");
        switch (ep->speed) {
        case SPEED_10:
                printk("10Mb/s\n");
                break;
        case SPEED_100:
                printk("100Mb/s\n");
                break;
        case SPEED_1000:
                printk("1000Mb/s\n");
                break;
        default:
                printk("Unknown! (%i)\n", ep->speed);
                break;
        };

        printk("       Duplex: ");
        switch (ep->duplex) {
        case DUPLEX_HALF:
                printk("Half\n");
                break;
        case DUPLEX_FULL:
                printk("Full\n");
                break;
        default:
                printk("Unknown! (%i)\n", ep->duplex);
                break;
        };

        printk("       Auto-negotiation: %s\n",
                (ep->autoneg == AUTONEG_DISABLE) ?
                "off" : "on");
        return 0;
}



/* Routine to configure the PHY */
void setup_phy (struct mii_if_info *mii_if)
{
	struct net_device *dev = mii_if->dev;
	EM86XX_ETH_PRIV *private = (EM86XX_ETH_PRIV *)dev->priv;
	u16 phy_cntl, phy_caps, ad_caps, status;
	int timeout;
	struct ethtool_cmd cmd = { ETHTOOL_GSET };
	
	DBG_PRINT_INOUT_FUNC("START");
	phy_cntl = mdio_read(dev, mii_if->phy_id, MII_BMCR);

	/* Avoid multiple instances */
	if (private->autoneg_active || !(phy_cntl & BMCR_ANENABLE) )
		return;

	private->autoneg_active = 1;
	/* Reset the PHY */
	phy_cntl = BMCR_RESET;
	if (private->phy_loopback)
		phy_cntl |= BMCR_LOOPBACK;
	DBG_PRINT("%s: Reset PHY\n", __func__);
	mdio_write(dev, mii_if->phy_id, MII_BMCR, phy_cntl);
	/* Wait up to 5 seconds for the reset to complete */
	for (timeout = 10; timeout != 0; timeout--) {
		phy_cntl = mdio_read(dev, mii_if->phy_id, MII_BMCR);
		if (!(phy_cntl & BMCR_RESET))
			break;
		/* Wait 500 milliseconds */
		wait_ms(500);
	}
	if (timeout == 0) {
		DBG_PRINT("PHY reset timeout\n");
		goto out;
	}
	/* Read phy & advertising capabilities */
	DBG_PRINT("%s: set up caps\n", __func__);
	phy_caps = mdio_read(dev, mii_if->phy_id, MII_BMSR);
	ad_caps = ADVERTISE_CSMA;
	if (phy_caps & BMSR_100BASE4)
		ad_caps |= ADVERTISE_100BASE4;
	if (phy_caps & BMSR_100FULL)
		ad_caps |= ADVERTISE_100FULL;
	if (phy_caps & BMSR_100HALF)
		ad_caps |= ADVERTISE_100HALF;
	if (phy_caps & BMSR_10FULL)
		ad_caps |= ADVERTISE_10FULL;
	if (phy_caps & BMSR_10HALF)
		ad_caps |= ADVERTISE_10HALF;

	/* Update auto-negociation advertisement register */
	mdio_write(dev, mii_if->phy_id, MII_ADVERTISE, ad_caps);
	/* Read the register back. Some (buggy) PHY won't negociate properly
	 * if this is not done.
	 */
	status = mdio_read(dev, mii_if->phy_id, MII_ADVERTISE);
	timeout = 30;
	/* Restart auto-negociation with our new advertisement capabilities */
	DBG_PRINT("%s: start autonegociation\n", __func__);
 restart_autoneg:
	phy_cntl = BMCR_ANRESTART | BMCR_ANENABLE;
	if (private->phy_loopback)
		phy_cntl |= BMCR_LOOPBACK;
	mdio_write(dev, mii_if->phy_id, MII_BMCR, phy_cntl);
	/* Wait for the auto-negociation to complete.
	 * This can take up to 3 seconds.
	 */
	for (; timeout != 0; timeout--) {
		status = mdio_read(dev, mii_if->phy_id, MII_BMSR);
		if (status & BMSR_ANEGCOMPLETE)
			break;
		if (status & BMSR_RFAULT) {
            /* In case of remote fault, restart negociation */
			DBG_PRINT("%s: restart autonegociation\n", __func__);
			goto restart_autoneg;
		}
		wait_ms(100);
	}
	status = mdio_read(dev, mii_if->phy_id, MII_BMSR);
	if (!(status & BMSR_ANEGCOMPLETE))
		DBG_PRINT("Auto-negociation timeout\n");
	else if (status & BMSR_RFAULT)
		DBG_PRINT("Auto-negociation remote fault\n");
	else
		DBG_PRINT("Auto-negociation done\n");

	netdev_get_settings(dev, &cmd);
	dump_ethtool_cmd(&cmd);
 out:
	private->autoneg_active = 0;
	DBG_PRINT_INOUT_FUNC("END");
}

/* PHY detection */
/* XXX: to be moved into mii.c */
int phy_detect (struct net_device *dev, struct mii_if_info *mii_if)
{
	int phy_n;
	int mii_status, phy_id1, phy_id2;
	
	DBG_PRINT_INOUT_FUNC("START");
	mii_if->dev = dev;
	mii_if->mdio_read = mdio_read;
	mii_if->mdio_write = mdio_write;
	mii_if->phy_id_mask = 0x1f;
	mii_if->reg_num_mask = 0x1f;
	mii_if->phy_id = -1; /* No PHY found */
	for (phy_n = 0; phy_n < 32; phy_n++) {
		DBG_PRINT("%s: check PHY %d\n", __func__, phy_n);
		mii_status = mdio_read(dev, phy_n, MII_BMSR);
		phy_id1 = mdio_read(dev, phy_n, MII_PHYSID1);
		phy_id2 = mdio_read(dev, phy_n, MII_PHYSID2);
		/* PHY_ID1 is zero in some realtek PHYs */
		if (/*phy_id1 > 0 &&*/ phy_id1 < 0xFFFF && phy_id1 != 0x8000 &&
		    phy_id2 > 0 && phy_id2 < 0xFFFF && phy_id2 != 0x8000 &&
		    mii_status != 0xffff && mii_status != 0x0000) {
			mii_if->advertising = mdio_read(dev, phy_n, 4);
			mii_if->phy_id = phy_n;
			DBG_PRINT(KERN_INFO "%s: MII PHY found at address %d, status "
			       "0x%4.4x advertising %4.4x Link %4.4x.\n",
			       dev->name, phy_n, mii_status, mii_if->advertising,
			       mdio_read(dev, phy_n, 5));
			/* set IFF_RUNNING */
			if (mii_status & BMSR_LSTATUS) {
				DBG_PRINT("%s: netif_carrier_on\n", __func__);
				netif_carrier_on(dev);
			}
			else {
				DBG_PRINT("%s: netif_carrier_off\n", __func__);
				netif_carrier_off(dev);
			}
			break;
		}
	}
	if (phy_n == 32) {
		DBG_PRINT("failed to detect PHY\n");
		return -1;
	}
	
	DBG_PRINT_INOUT_FUNC("END");
	return phy_n;
}

/* Getting MAC address from local RAM */
static int get_mac_address(unsigned long *mac_lo, unsigned long *mac_hi)
{
	unsigned long cksum, lo, hi;
	const unsigned char marker = 'M';
	
	DBG_PRINT_INOUT_FUNC("START");
	lo = em86xx_read_reg(REG_BASE_cpu_block + LR_ETH_MAC_LO);
	hi = em86xx_read_reg(REG_BASE_cpu_block + LR_ETH_MAC_HI);

	if (((unsigned char)((hi >> 16) & 0xff)) != marker)
		return(1); /* Not valid MAC */
	cksum = ((lo & 0xff) + ((lo >> 8) & 0xff) + ((lo >> 16) & 0xff) + ((lo >> 24) & 0xff) + 
		 (hi & 0xff) + ((hi >> 8) & 0xff)) & 0xff;
	if (((hi >> 24) & 0xff) != cksum)
		return(1); /* Not valid MAC */
	*mac_lo = lo;
	*mac_hi = hi & 0xffff;

	DBG_PRINT_INOUT_FUNC("END");
	return(0); /* Got valid MAC */
}

/* Get the device mac address */
static int em86xx_get_macaddr(unsigned char *addr)
{
	DBG_PRINT_INOUT_FUNC("START");
#if defined(CONFIG_TANGOX)
	unsigned long hi, lo;
	if (get_mac_address(&lo, &hi) == 0) {
		*addr++ = (hi & 0x0000ff00) >> 8;
		*addr++ = (hi & 0x000000ff);
		*addr++ = (lo & 0xff000000) >> 24;
		*addr++ = (lo & 0x00ff0000) >> 16;
		*addr++ = (lo & 0x0000ff00) >> 8;
		*addr   = (lo & 0x000000ff);
	} else if (tangox_ethernet_getmac(0, addr) == 0) {
		*addr++ = (def_mac_hi & 0x0000ff00) >> 8;
		*addr++ = (def_mac_hi & 0x000000ff);
		*addr++ = (def_mac_lo & 0xff000000) >> 24;
		*addr++ = (def_mac_lo & 0x00ff0000) >> 16;
		*addr++ = (def_mac_lo & 0x0000ff00) >> 8;
		*addr   = (def_mac_lo & 0x000000ff);
	}
#else
#ifdef BOOTLOADER
	if(	mac_read(addr) == -1) {
		*addr++ = (def_mac_hi & 0x0000ff00) >> 8;
		*addr++ = (def_mac_hi & 0x000000ff);
		*addr++ = (def_mac_lo & 0xff000000) >> 24;
		*addr++ = (def_mac_lo & 0x00ff0000) >> 16;
		*addr++ = (def_mac_lo & 0x0000ff00) >> 8;
		*addr   = (def_mac_lo & 0x000000ff);
	}
#else
#ifdef CONFIG_SD_BOOTLOADER_INTEGRATION
	{
		unsigned long lo, hi;

		if (get_mac_address(&lo, &hi) == 0) {
			def_mac_hi = hi;
			def_mac_lo = lo;
		}
	}
#endif
	*addr++ = (def_mac_hi & 0x0000ff00) >> 8;
	*addr++ = (def_mac_hi & 0x000000ff);
	*addr++ = (def_mac_lo & 0xff000000) >> 24;
	*addr++ = (def_mac_lo & 0x00ff0000) >> 16;
	*addr++ = (def_mac_lo & 0x0000ff00) >> 8;
	*addr   = (def_mac_lo & 0x000000ff);
#endif /* BOOTLOADER */	
#endif /* CONFIG_TANGOX */
	DBG_PRINT_INOUT_FUNC("END");
	return 0;
}

/* Setting up MAC address of ethernet PHY and device data */
static int em86xx_set_mac(struct net_device *dev)
{
	unsigned long hi_mac, low_mac;

	DBG_PRINT_INOUT_FUNC("START");
	/* Set up device mac address */
	if (em86xx_get_macaddr(dev->dev_addr))
		return(-EIO);

	if (!is_valid_ether_addr(dev->dev_addr))
		MSG_PRINT("%s: bogus mac address detected.\n", dev->name);

	hi_mac =  (dev->dev_addr[5] << 8)  | dev->dev_addr[4];
	low_mac = (dev->dev_addr[3] << 24) | (dev->dev_addr[2] << 16) | 
		  (dev->dev_addr[1] << 8)  | dev->dev_addr[0];

	/* Set up MAC address */
	em86xx_write_reg(EM86XX_MACAHR_REG, hi_mac);
	em86xx_write_reg(EM86XX_MACALR_REG, low_mac);

	DBG_PRINT_INOUT_FUNC("END");
	return 0;
}

#ifndef BOOTLOADER
/* Setting customized mac address */
static int em86xx_eth_set_macaddr(struct net_device *dev, void *addr)
{
	unsigned long hi_mac, low_mac;
	struct sockaddr *sock = addr;

	DBG_PRINT_INOUT_FUNC("START");
	/* Check if given address is valid ethernet MAC address */
	if (!is_valid_ether_addr(sock->sa_data))
		return(-EIO);

	/* Turn off IRQ and stop receive/transmit */
	em86xx_write_reg(EM86XX_CR_REG, 0);
	em86xx_write_reg(EM86XX_IER_REG, 0);

	netif_carrier_off(dev); /* Shutdown interface */

	/* Save the customize mac address */
	memcpy(dev->dev_addr, sock->sa_data, ETH_ALEN);
	hi_mac  = (dev->dev_addr[5] << 8) | dev->dev_addr[4];
	low_mac = (dev->dev_addr[3] << 24)| (dev->dev_addr[2] << 16) | 
		  (dev->dev_addr[1] << 8) | dev->dev_addr[0];

	def_mac_hi = hi_mac;
	def_mac_lo = low_mac;

	/* Set up MAC address */
	em86xx_write_reg(EM86XX_MACAHR_REG, hi_mac );
	em86xx_write_reg(EM86XX_MACALR_REG, low_mac );

	netif_carrier_on(dev);

	DBG_PRINT_INOUT_FUNC("END");
	return 0;
}
#endif /* BOOTLOADER */

/* RTL8201BL is a single-port PHY with an MII/SNI. The MAC layer device can 
   control a maximum of 31 devices, configured with different PHY addresses
   (00001b to 11111b). We need to find out which PHY address is using in 
   order to obtain the mode status after the auto-neotiation is completed.

   There is a module parameter phy_address to be used to override this probe,
   to set PHY address if one already knew it.
*/
static int em86xx_phy_probe(void)
{

        int phy_addr 	= 0;
	u16 phy_id 	= 0;
        u16 phy_status 	= 0;

	DBG_PRINT_INOUT_FUNC("START");
	if (phy_address < 0) {
		/* search for total of 31 possible mii phy addresses */
		for (phy_addr = 1; phy_addr < 32; phy_addr++) {
			phy_status = em86xx_mii_read(phy_addr,GEN_sts);

			if (phy_status != 0xffff && phy_status != 0x0000)
				break;
		}
	} else {
		phy_addr = phy_address;
		phy_status = em86xx_mii_read(phy_addr,GEN_sts);
	}
	phy_id =  em86xx_mii_read(phy_addr,GEN_id_lo);
	MSG_PRINT("Found PHY %04x at Addr = %d Status=0x%x\n", phy_id, phy_addr, phy_status);
 
	DBG_PRINT_INOUT_FUNC("END");
	return phy_addr;
}

/* Initialize the ethernet link status, mac, and flow control */
static int em86xx_link_config(struct net_device *dev)
{
	EM86XX_ETH_PRIV *private = (EM86XX_ETH_PRIV *)dev->priv;
	struct mii_if_info *mii = &(private->mii_if);
	unsigned long word = 0;

	DBG_PRINT_INOUT_FUNC("START");

	// Turn off IRQ and stop receive/transmit 
	em86xx_write_reg(EM86XX_CR_REG, 0);
	em86xx_write_reg(EM86XX_IER_REG, 0);

	setup_phy(mii);

	/* set the descriptor base address */
	em86xx_write_reg(EM86XX_RLBAR_REG, PHYSADDR((u32)private->rxdsc) );
	em86xx_write_reg(EM86XX_TLBAR_REG, PHYSADDR((u32)private->txdsc));

	/* set bus mode: burst length = 32 | BLE | ~DBO */
	word = (32 << 8) /*| (1 << 7) | (1 << 20)*/ ;
	em86xx_write_reg(EM86XX_BMR_REG, word); 

	/* enable MAC flow ctrl */
	word = (1 << 1);
	em86xx_write_reg(EM86XX_FCR_REG, word); 

	/* configure MAC ctrller */
	word = ETH_MAC_FLAGS;
	word |= MacLoopbackOff; 
        if (mii->full_duplex)
                word |= MacFullDuplex;
        else
                word &= ~MacFullDuplex;
	em86xx_write_reg(EM86XX_MACCR_REG, word);

	/* Turn on the IRQ and start receive/transmit */
	em86xx_write_reg(EM86XX_IER_REG, ETH_IRQ_FLAGS);
	em86xx_write_reg(EM86XX_CR_REG, ETH_RXTX_FLAGS); 

	netif_carrier_on(dev);

	DBG_PRINT_INOUT_FUNC("END");
	return 0;
}

/* Ethernet hardware initialization */
static int em86xx_eth_hw_init(struct net_device *dev)
{
	EM86XX_ETH_PRIV *private = (EM86XX_ETH_PRIV *)dev->priv;
	int rc;

	DBG_PRINT_INOUT_FUNC("START");

	/* Turn off IRQ and stop receive/transmit */
	em86xx_write_reg(EM86XX_CR_REG, 0);
	em86xx_write_reg(EM86XX_IER_REG, 0);

	/* reset dma engine*/
	em86xx_write_reg(EM86XX_BMR_REG, DmaResetOn);
	em86xx_write_reg(EM86XX_BMR_REG, DmaResetOff);

	if (em86xx_set_mac(dev))
		return(-EIO);

	/* stop MAC engine */
	em86xx_write_reg(EM86XX_CR_REG, 0x0); 
#ifndef USE_HW_FILTERING
	/* set up multicast hash table to MCHTHR MCHTLR */
	/* set multicast hash table to accept all */
	em86xx_write_reg(EM86XX_MCHTHR_REG, 0xffffffff); 
	em86xx_write_reg(EM86XX_MCHTLR_REG, 0xffffffff); 
#else
	/* clear hash table */
        em86xx_write_reg( EM86XX_MCHTHR_REG, 0 );
        em86xx_write_reg( EM86XX_MCHTLR_REG, 0 );

	em86xx_eth_set_multicast_list(dev);
#endif

	/* resetting descriptors */
#ifdef BOOTLOADER
        if (em86xx_eth_reset_desc(dev, &private->reset_flag))
#else
        if (em86xx_eth_reset_desc(&em86xx_eth_dev, &private->reset_flag))
#endif
		return(-EIO);

	/* configure PHY and speed */
	rc = em86xx_link_config(dev);
	DBG_PRINT_INOUT_FUNC("END");
	return rc;
}

#ifndef BOOTLOADER
/* Monitor the status of link, re-do link initialization if necessary. */
#ifdef USE_KERNEL_TIMER
static void em86xx_eth_link_monitor(unsigned long dummy)
#else
static int em86xx_eth_link_monitor(void *dummy)
#endif
{
	struct net_device *dev = (struct net_device *)dummy;
	EM86XX_ETH_PRIV *private = (EM86XX_ETH_PRIV *)dev->priv;
        unsigned long flags;
        int link;
	int res;

	DBG_PRINT_INOUT_FUNC("START");

#ifndef USE_KERNEL_TIMER
	daemonize("em86xx_eth");

	while (private->stop_thread == 0) {
#endif
		spin_lock_irqsave(&private->lock, flags);
		link = mii_link_ok(&private->mii_if);
		spin_unlock_irqrestore(&private->lock, flags);
		
		if (netif_carrier_ok(dev) && !link) {			
			MSG_PRINT("%s: detected link DOWN.\n", dev->name);
			netif_carrier_off(dev);
		}
		else if (!netif_carrier_ok(dev) && link) {
			MSG_PRINT("%s: detected link UP.\n", dev->name);
			res = em86xx_eth_hw_init(dev);
			netif_carrier_on(dev);
		}

#ifndef USE_KERNEL_TIMER
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(HZ);
	}
#endif

#ifdef USE_KERNEL_TIMER
	/* Schedule for the next time */
	mod_timer(&private->eth_timer, jiffies + HZ); 
	DBG_PRINT_INOUT_FUNC("END (with timer)");
#else
	DBG_PRINT_INOUT_FUNC("END (IRQ)");
	return(0);
#endif
}
#endif

/* Setting rx/tx descriptors */
static void em86xx_eth_setup_desc(struct net_device *dev)
{
	register int i;
	struct em86xx_desc *desc_ptr = NULL;
	unsigned long base_addr = 0;
	EM86XX_ETH_PRIV *private = (EM86XX_ETH_PRIV *)dev->priv;

	DBG_PRINT_INOUT_FUNC("START");
	/* Setup rx descriptor */
	desc_ptr = (struct em86xx_desc *)(private->rxdsc);
	for (i = 0; i < (private->num_rxdesc - 1); i++, desc_ptr++) {
		desc_ptr->desc3 = PHYSADDR((unsigned long)(desc_ptr + 1));
		desc_ptr->desc1 = (DescChain | R_BUF_SIZE);
  		desc_ptr->desc0 = DescOwnByDma; 
	}
	desc_ptr->desc3 = PHYSADDR((unsigned long)(private->rxdsc));
	desc_ptr->desc1 = (DescChain | DescEndOfRing | R_BUF_SIZE);
	desc_ptr->desc0 = DescOwnByDma; 

	/* Setup tx descriptor */
	desc_ptr = (struct em86xx_desc *)(private->txdsc); 
	for (i =0;  i < (private->num_txdesc - 1); i++, desc_ptr++) {
		desc_ptr->desc3 = PHYSADDR((unsigned long)(desc_ptr + 1));
		desc_ptr->desc1 = (TX_DESC1_FLAGS | T_BUF_SIZE);
  		desc_ptr->desc0 = DescOwnByCPU;
	}
	desc_ptr->desc3 = PHYSADDR((unsigned long)(private->txdsc));
	desc_ptr->desc1 = (TX_DESC1_FLAGS | DescEndOfRing | T_BUF_SIZE);
	desc_ptr->desc0 = DescOwnByCPU;

	/* Point rx descriptor to buffer */
	desc_ptr = (struct em86xx_desc *)(private->rxdsc);
#ifdef BOOTLOADER 
      	base_addr = PHYSADDR((unsigned long)(private->rxbuf)); 
	for (i = 0; i < private->num_rxdesc; i++, desc_ptr++, base_addr += R_BUF_SIZE) 
		desc_ptr->desc2 = base_addr;
#else
        for (i = 0; i < private->num_rxdesc; i++, desc_ptr++){
                private->rx_skb_list[i] = dev_alloc_skb(R_BUF_SIZE);
                skb_reserve(private->rx_skb_list[i], 2);
                desc_ptr->desc2 = PHYSADDR((u32)private->rx_skb_list[i]->data);
#ifdef CONFIG_NONCOHERENT_IO
		dma_cache_inv((u32)private->rx_skb_list[i]->data, R_BUF_SIZE);
#endif
        }
#endif

	/* Point tx descriptor to buffer */
	desc_ptr = (struct em86xx_desc *)(private->txdsc);
        base_addr = PHYSADDR((unsigned long)(private->txbuf));
	for (i = 0; i < private->num_txdesc; i++, desc_ptr++, base_addr += T_BUF_SIZE) 
		desc_ptr->desc2 = base_addr;

	DBG_PRINT_INOUT_FUNC("END");
}

/* Starting up the ethernet device */
static int em86xx_eth_open(struct net_device *dev)
{
	EM86XX_ETH_PRIV *private = (EM86XX_ETH_PRIV *)dev->priv;

	DBG_PRINT_INOUT_FUNC("END");
	if ((dev == NULL) || (private == NULL))
		return(-EIO);
	DBG_PRINT("%s: starting interface.\n", dev->name);

	/* Turn off IRQ and stop receive/transmit */
	em86xx_write_reg(EM86XX_CR_REG, 0);
	em86xx_write_reg(EM86XX_IER_REG, 0);
	/*Get phy address*/
	private->phy_id = em86xx_phy_probe();
	em86xx_eth_hw_init(dev);

#ifdef BOOTLOADER
        dev->state = NETDEV_UP;
#else
#ifdef USE_KERNEL_TIMER 
	/* Schedule timer for monitoring link status */
	init_timer(&private->eth_timer);
	private->eth_timer.function = em86xx_eth_link_monitor;
	private->eth_timer.data = (unsigned long)dev;
	mod_timer(&private->eth_timer, jiffies + HZ/10);
#else
	private->stop_thread = 0;
	if (kernel_thread(em86xx_eth_link_monitor, dev, CLONE_FS|CLONE_FILES) < 0)
		return(-ENODEV);
#endif
	netif_start_queue(dev);
#endif /*BOOTLOADER*/

	DBG_PRINT_INOUT_FUNC("END");
	return 0;
}

/* Stopping the ethernet device */
static int em86xx_eth_close(struct net_device *dev)
{
	EM86XX_ETH_PRIV *private;

	DBG_PRINT_INOUT_FUNC("START");
	if (dev == NULL) {
		DBG_PRINT_INOUT_FUNC("END");
  		return(-EIO);
	}

	private = (EM86XX_ETH_PRIV *)dev->priv;

	/* Turn off IRQ and stop receive/transmit */
	em86xx_write_reg(EM86XX_CR_REG, 0);
	em86xx_write_reg(EM86XX_IER_REG, 0);

#ifdef BOOTLOADER
	dev->state = NETDEV_DOWN;
#else

#ifdef USE_KERNEL_TIMER
	/* Kill timer */
	del_timer_sync(&private->eth_timer);
#else
	private->stop_thread = 1;
	schedule_timeout(5*HZ); /* Wait for kernel thread to terminate */
#endif

	/* Stop the transmit queue */
	netif_stop_queue(dev);
	netif_carrier_off(dev);

#endif /*BOOTLOADER*/
	DBG_PRINT_INOUT_FUNC("END");
	return 0;
}

#ifndef BOOTLOADER
#ifdef USE_HW_FILTERING
/* Setup multicast list */
static void em86xx_eth_set_multicast_list(struct net_device *dev)
{
	/* Multicast hash filter */
        u32 mc_filter[2] = { 0, 0 };
        int i, rx_mode;
 	u8  broadcast_addr[6]  = DEFAULT_BROADCAST_ADDRESS;

	DBG_PRINT_INOUT_FUNC("START");
	rx_mode =  em86xx_read_reg(EM86XX_MACCR_REG);  
        DBG_PRINT(KERN_DEBUG"%s:   set_rx_mode(0x%x) done -- rx_mode=0x%x.\n",
                           dev->name, dev->flags, rx_mode);

        if (dev->flags & IFF_PROMISC) {
                MSG_PRINT("%s: Promiscuous mode enabled.\n", dev->name);
                rx_mode |= MacPromiscuousModeOn;
                mc_filter[1] = mc_filter[0] = 0xffffffff;
        } else {
                struct dev_mc_list *mclist;
		int n;
                rx_mode &=  ~MacPromiscuousModeOn;
                for (i = 0, mclist = dev->mc_list; mclist && i < dev->mc_count;
                         i++, mclist = mclist->next){
			n = ether_crc(ETH_ALEN, mclist->dmi_addr) >> 26;
			mc_filter[n >> 5] |= 1 << (n & 31);
		}
		n = ether_crc(ETH_ALEN, broadcast_addr) >> 26;
		mc_filter[n >> 5] |= 1 << (n & 31);
        }
	em86xx_write_reg(EM86XX_MACCR_REG, rx_mode );
        em86xx_write_reg(EM86XX_MCHTLR_REG, mc_filter[0]);
        em86xx_write_reg(EM86XX_MCHTHR_REG, mc_filter[1]);

	DBG_PRINT_INOUT_FUNC("END");
        return;
}
#endif
#endif

/* Transmit a packet */
#ifdef BOOTLOADER
static int em86xx_eth_tx(struct sk_buff *skb, struct net_device *dev, int async)
#else
static int em86xx_eth_tx(struct sk_buff *skb, struct net_device *dev)
#endif
{
	EM86XX_ETH_PRIV *private = (EM86XX_ETH_PRIV *)dev->priv;
	volatile struct em86xx_desc *desc_ptr;
	struct net_device_stats *stats;
	unsigned long length, err;
	int ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&private->lock, flags);

	if ((length = ((ETH_ZLEN < skb->len) ? skb->len : ETH_ZLEN)) > T_BUF_SIZE) {
      		DBG_PRINT("em86xx_eth_tx: too big\n");
		ret = -ENOMEM; 
		goto out_unlock;
	}

	stats = &(private->stats);

	/* Send packet to device */
	desc_ptr = (volatile struct em86xx_desc *)(&private->txdsc[private->next_txidx]);
	if ((desc_ptr->desc0 & DescOwnByDma) != 0) {
		stats->tx_dropped++;
		
		/* cannot queue anymore */
		netif_stop_queue(dev);
		private->need_restart_tx_queue = 1;

        	ERR_PRINT("%s desc_ptr=0x%x: tx error (descriptor not owned by CPU).\n",
				 dev->name, (u32)desc_ptr);
		ret = -EIO;
		goto out_unlock;
	} 

	/* Check if previous transmission has error */
	if ((err = (desc_ptr->desc0 & TX_ERROR_FLAGS)) != 0) {
		int i;
		ERR_PRINT("%s: got a tx error:0x%08lx\n", dev->name, desc_ptr->desc0);
		for (i = TX_ERRORS_START; (i < TX_ERRORS) && (err != 0); i++) {
			int *ptr;
			if (err & (1 << i)) {
				err &= ~(1<<i);
				if (tx_error_msg[i] != NULL)
					ERR_PRINT("%s\n",tx_error_msg[i]);
				if (tx_stat_offset[i] >= 0) {
					ptr = (int *)(((char *)stats) + tx_stat_offset[i]);
					*ptr++;
				}
			}
		}

		/* Update stats */
 		stats->collisions += (desc_ptr->desc0 >> 3) & 15;
		stats->tx_errors++;
		stats->tx_packets--;
		stats->tx_bytes -= (desc_ptr->desc1 & DescSize1Mask);
	}

	/* Copy packet data to tx buffer */
//	DBG_PRINT("tx copy data from skb->data=0x%x to desc2=0x%x length=0x%x\n",
//		(u32)skb->data, (u32)desc_ptr->desc2, (u32)length );

	memcpy((void *)NON_CACHED(desc_ptr->desc2), skb->data, length);

	/* Setup tx descriptor */
	desc_ptr->desc1 = ( TX_DESC1_FLAGS | length );
	if (private->next_txidx == (private->num_txdesc - 1))
		desc_ptr->desc1 |=  DescEndOfRing ;
	desc_ptr->desc0 |= DescOwnByDma;

	/* Start transmission (should be in suspend mode already) */
	em86xx_write_reg(EM86XX_TPDR_REG, 0x1);

#ifdef BOOTLOADER
        private->next_txidx = imodulus((private->next_txidx + 1), private->num_txdesc);
#else
	private->next_txidx = ((private->next_txidx + 1) % private->num_txdesc);
	dev->trans_start = jiffies;
#endif
	/* Update stats */
	stats->tx_packets++;
	stats->tx_bytes += length;

	/* Check to see if TX queue is full */
	desc_ptr = (volatile struct em86xx_desc *)(&private->txdsc[private->next_txidx]);
	if ((desc_ptr->desc0 & DescOwnByDma) != 0) {
		/* cannot queue anymore */
		netif_stop_queue(dev);
		private->need_restart_tx_queue = 1;
	} 

	/* Free up socket buffer */
#ifdef BOOTLOADER
	async = 0;
	skb_free(skb);
#else
	dev_kfree_skb(skb);
#endif
 out_unlock:
	spin_unlock_irqrestore(&private->lock, flags);
	return ret;
}


static inline int is_sw_filtered_packet(struct net_device *dev, unsigned char *data)
{
#ifdef USE_SW_FILTERING
        if (((data[0] & 0x80) == 0) && (dev->dev_addr[5] != data[5])) {
                return 1;
	}
        else {
                return 0;
	}
#else
        return 0;
#endif
}


/* Receiving packet(s) */
static int em86xx_eth_rx(struct net_device *dev)
{
	EM86XX_ETH_PRIV *private;
#ifdef BOOTLOADER
	struct sk_buff *skb;
#endif
	struct net_device_stats *stats;
	unsigned char *data;
	volatile struct em86xx_desc *desc_ptr;
	unsigned long length;
	unsigned long dsc;
	unsigned long count=0, cnt= 0, err;
#ifdef MULTI_DESCS_FRAME_SUPPORT
	volatile struct em86xx_desc *mdf_desc_ptr;
	long mdf_start = -1;
#endif
//	unsigned long flags;

  	private = (EM86XX_ETH_PRIV *)dev->priv;
  	stats = &(private->stats);

	/* Go thru descriptors list until we ran out or encounterred
	   the one being processed by DMA */
#ifdef BOOTLOADER 
	for (cnt = 0, count = private->last_rxidx; cnt < private->num_rxdesc; cnt++, count = imodulus(count + 1, private->num_rxdesc)) {
#else
	for (cnt = 0, count = private->last_rxidx; cnt < private->num_rxdesc; cnt++, count = ((count + 1) % private->num_rxdesc)) {
#endif

		desc_ptr = (volatile struct em86xx_desc *)(&private->rxdsc[count]);
	 	dsc = desc_ptr->desc0;

	  	if ((dsc & DescOwnByDma) != 0) {
    			/* DMA is processing this one, break out the loop */
			break;
    		}

		/*filter out those are not for me */
      		data = (unsigned char *)NON_CACHED(desc_ptr->desc2);
		if (is_sw_filtered_packet(dev, data)
#ifdef MULTI_DESCS_FRAME_SUPPORT
		    && (mdf_start < 0)
#endif
				) {
			ERR_PRINT("%s: mac mismatched, dropped.\n", dev->name);
		} else	if ((err = (dsc & RX_ERROR_FLAGS)) != 0)  {
			int i;

#ifdef MULTI_DESCS_FRAME_SUPPORT
			 /* First check are we in MDF mode */
			 if (mdf_start >= 0) { /* Yes, we are */
				 MSG_PRINT("%s: rx error in MDF mode (s:0x%x,c:0x%x)\n", dev->name, mdf_start, count);
				 /* Something is wrong, drop all MDF related descriptors */
				 for (i = mdf_start; i != count; i = (i + 1) % private->num_rxdesc) {
					 mdf_desc_ptr = (volatile struct em86xx_desc *)(&private->rxdsc[i]);
					 mdf_desc_ptr->desc0 = DescOwnByDma;
					 wmb();
				 }
				 mdf_start = -1; /* Reset so we're out of this mode */
			 }
#endif
			/*Filtering fail is valid only when Last desc bit is set, 
			  length is 64 bytes or longer and Receive All bit is set. */
			if (dsc & DescFilteringFail) {		
				unsigned long len, rx_mode;
				len = (dsc & DescFrameLengthMask) >> DescFrameLengthShift;
				rx_mode = em86xx_read_reg(EM86XX_MACCR_REG);
				if(!((dsc & DescRxLast) && (len >= 64) && (rx_mode & MacFilterOff)))
					goto sync_up;
			}

      			ERR_PRINT("%s: rx error 0x%08lx:%ld desc2=0x%08x\n", 
				dev->name, dsc, count, (u32)desc_ptr->desc2);

			for (i = RX_ERRORS_START; (i < RX_ERRORS) && (err != 0); i++) {
				int *ptr;
				if (err & (1 << i)) {
					err &= ~(1<<i);
					if (rx_error_msg[i] != NULL)
						ERR_PRINT("%s\n", rx_error_msg[i]);
					if (rx_stat_offset[i] >= 0) {
						ptr = (int *)(((char *)stats) + rx_stat_offset[i]);
						*ptr++;
					}
				}
			}

			/* We dropped any error descriptor */
		        stats->rx_errors++;
		 } else if ((dsc & MD_FRAME_FLAGS) != MD_FRAME_FLAGS) {
#ifdef MULTI_DESCS_FRAME_SUPPORT
			 /* We encounterred multi-descriptor frame */
			 if (((dsc & DescRxFirst) != 0) && ((dsc & DescRxLast) == 0)) {
				 /* First descriptor of multi-descriptor frame */
				 if (mdf_start >= 0) { 
					 int i;
					 MSG_PRINT("%s: MDF mode error at beginning (s:0x%x,c:0x%x)\n", dev->name, mdf_start, count);
					 /* Something is wrong, drop all MDF related descriptors */
					 for (i = mdf_start; i != count; i = (i + 1) % private->num_rxdesc) {
						 mdf_desc_ptr = (volatile struct em86xx_desc *)(&private->rxdsc[i]);
						 mdf_desc_ptr->desc0 = DescOwnByDma;
						 wmb();
					 }
				 }
				 mdf_start = (long)count;
				 continue; /* Loop until we can get the whole frame */
			 } else if ((dsc & MD_FRAME_FLAGS) == 0) {
				 if (mdf_start >= 0) {
					 /* Interim descriptor within multi-descriptor frame */
					 continue; /* Loop until we can get the whole frame */
				 } else {
					 MSG_PRINT("%s: MDF mode error (c:0x%x)\n", dev->name, count);
					 /* Bogus: drop it */
					 goto sync_up;
				 }
			 } else {
				 /* Last descriptor of multi-descriptor frame */
				 unsigned long mdf_size, length;
				 long i, mdf_end;
				 char *ptr, *mdf_data;
				 struct sk_buff *mdf_skb;

				 if (mdf_start < 0) {
					 MSG_PRINT("%s: MDF mode error at the end (c:0x%x)\n", dev->name, count);
					 /* Bogus: drop it */
					 goto sync_up;
				 }

				 /* First we get the size of the frame, and allocate a new skb */
				 length = mdf_size = (dsc & DescFrameLengthMask) >> DescFrameLengthShift;
				 mdf_end = (count + 1) % private->num_rxdesc;
#ifdef BOOTLOADER
				 mdf_skb = skb_alloc(mdf_size);
#else
				 mdf_skb = dev_alloc_skb(mdf_size);
#endif
				 if (mdf_skb == NULL) {
					 /* Error: no memory available, dropping all descriptors in this frame */
					 stats->rx_dropped += (((mdf_start + private->num_rxdesc) - mdf_end) % private->num_rxdesc);
					 for (i = mdf_start; i != mdf_end; i = (i + 1) % private->num_rxdesc) {
						 mdf_desc_ptr = (volatile struct em86xx_desc *)(&private->rxdsc[i]);
						 mdf_desc_ptr->desc0 = DescOwnByDma;
						 wmb();
					 }
				 } else {
#ifndef BOOTLOADER
					 skb_reserve(mdf_skb, 2);
#endif
					 stats->rx_bytes += mdf_size;
					 stats->rx_packets += (((mdf_start + private->num_rxdesc) - mdf_end) % private->num_rxdesc);

					 /* Collecting all descriptors into skb  */
					 for (ptr = mdf_skb->data, i = mdf_start; i != mdf_end; i = (i + 1) % private->num_rxdesc) {
						 mdf_desc_ptr = (volatile struct em86xx_desc *)(&private->rxdsc[i]);
						 mdf_data = (unsigned char *)NON_CACHED(mdf_desc_ptr->desc2);
						 memcpy(ptr, mdf_data, (length > R_BUF_SIZE) ? R_BUF_SIZE : length);
						 ptr += R_BUF_SIZE;
						 length -= R_BUF_SIZE;
						 mdf_desc_ptr->desc0 = DescOwnByDma;
						 wmb();
					 }
					 mdf_skb->dev = dev;
					 mdf_skb->len = mdf_size - 4;

					 /* Send the frame back up */
#ifdef BOOTLOADER
					 skb_put(mdf_skb);
#else
					 mdf_skb->protocol = eth_type_trans(mdf_skb, dev);
					 mdf_skb->ip_summed = CHECKSUM_NONE;
					 dev->last_rx = jiffies;
					 netif_rx(mdf_skb);
#endif
				 }
				 mdf_start = -1;
				 continue; /* No need go to the end of the loop */
			}
#else
			/* We encounterred multi-descriptor frame */
	        	DBG_PRINT("%s: multi-descriptor frame detected 0x%08lx\n", dev->name, dsc);
		        /* Don't handle multi-descriptor frame */
		        stats->rx_dropped++;
#endif /* MULTI_DESCS_FRAME_SUPPORT */
		 } else if ((length = ((dsc & DescFrameLengthMask) >> DescFrameLengthShift)) > R_BUF_SIZE) {
			/* Should not happen for single-descriptor frame */
      			ERR_PRINT("%s: rx dropped (size too large: %ld)\n", dev->name, length);
			stats->rx_length_errors++;	
			stats->rx_dropped++;
#ifdef BOOTLOADER
                 } else if ((skb = skb_alloc(length)) == NULL) {
			ERR_PRINT("%s: rx dropped (memory unavailable)\n", dev->name);
	        	stats->rx_dropped++;
#endif
		 } else {
			 /* We got single descriptor frame */
#ifdef MULTI_DESCS_FRAME_SUPPORT
			 /* First check are we in MDF mode */
			 if (mdf_start >= 0) { /* Yes, we are */
				 int i;
				 /* Something is wrong, drop all MDF related descriptors */
				 for (i = mdf_start; i != count; i = (i + 1) % private->num_rxdesc) {
					 mdf_desc_ptr = (volatile struct em86xx_desc *)(&private->rxdsc[i]);
					 mdf_desc_ptr->desc0 = DescOwnByDma;
				 }
				 mdf_start = -1; /* Reset so we're out of this mode */
			 }
#endif
#ifdef BOOTLOADER
		  	 skb->dev = dev;
                         skb->len = length - 4;
                         memcpy(skb->data, data, length-4);
                         skb_put(skb);

                         /* Update stats */
                         stats->rx_packets++;
                         stats->rx_bytes += length;
#else
                         private->rx_skb_list[count]->dev = dev;
                         private->rx_skb_list[count]->len = length-4;
                         private->rx_skb_list[count]->protocol = eth_type_trans(private->rx_skb_list[count], dev);
                         private->rx_skb_list[count]->ip_summed = CHECKSUM_NONE;

                         /* Send the packet to kernel */
                         netif_rx(private->rx_skb_list[count]);

	        	 /* Update stats */
		       	 stats->rx_packets++;
		         stats->rx_bytes += length;
		       	 dev->last_rx = jiffies;
#endif /*BOOTLOADER*/
  	 	}
#ifndef BOOTLOADER
                private->rx_skb_list[count] = dev_alloc_skb(R_BUF_SIZE);
                skb_reserve(private->rx_skb_list[count], 2);
                desc_ptr->desc2 = PHYSADDR((u32)private->rx_skb_list[count]->data);
#ifdef CONFIG_NONCOHERENT_IO
		dma_cache_inv((u32)private->rx_skb_list[count]->data, R_BUF_SIZE);
#endif
#endif

sync_up:
  		desc_ptr->desc0 = DescOwnByDma;
		wmb();
  	}

#ifdef MULTI_DESCS_FRAME_SUPPORT
	/* This is the last desc we read */
	if (mdf_start >= 0)
		/* We haven't reached the end of multi-descriptor frame, mark it here
		 * so we can come back and pick it up again */
		private->last_rxidx = mdf_start; 
	else
		private->last_rxidx = count; 
#else
	/* This is the last desc we read */
	private->last_rxidx = count; 
#endif
	/* make sure rx is not suspended */
	em86xx_write_reg(EM86XX_RPDR_REG, 0x1);
  	return cnt;
}

/* Resetting rx descriptor: error recovery for "rx buf unavailable" */
static void em86xx_eth_reset_rx_buf(struct net_device *dev)
{
	register int i;
	volatile struct em86xx_desc *desc_ptr = NULL;
	EM86XX_ETH_PRIV *private = (EM86XX_ETH_PRIV *)dev->priv;

	DBG_PRINT_INOUT_FUNC("START");
	em86xx_clear_reg(EM86XX_CR_REG, DmaRxStart); 

	/* Reset rx desc */
	desc_ptr = (struct em86xx_desc *)(private->rxdsc); 
	for (i = 0; i < private->num_rxdesc; i++, desc_ptr++) 
		desc_ptr->desc0 = DescOwnByDma; 
  	
  	em86xx_write_reg(EM86XX_RPDR_REG,  0x1);
	em86xx_set_reg(EM86XX_CR_REG, DmaRxStart); 
	DBG_PRINT_INOUT_FUNC("END");
}

#ifndef BOOTLOADER
/* Get the stats information */
static struct net_device_stats *em86xx_eth_stats(struct net_device *dev)
{
	DBG_PRINT_INOUT_FUNC("START");
	if (dev != NULL) {
		DBG_PRINT_INOUT_FUNC("END");
    		return(&(((EM86XX_ETH_PRIV *)dev->priv)->stats));
	}
  	else {
		DBG_PRINT_INOUT_FUNC("END");
    		return(NULL);
	}

}
#endif

/* Ethernet interrupt handler */
#ifdef BOOTLOADER
static void em86xx_eth_intr_handler(int irq, void *dev_id)
#else
static irqreturn_t em86xx_eth_intr_handler(int irq, void *dev_id)
#endif
{
	int num_rx_desc_freed = 0;
	unsigned long status;
	struct net_device *dev = (struct net_device *)dev_id;
	EM86XX_ETH_PRIV *private = (EM86XX_ETH_PRIV *)dev->priv;

#ifdef BOOTLOADER
        if( memcmp(dev->name,DRIVER, sizeof(DRIVER)) != 0) 
                return;
#else
        if (dev_id != (void *)&em86xx_eth_dev)
                return IRQ_NONE;
#endif
        /* Check status */
        status = em86xx_read_reg(EM86XX_SR_REG);

        // clear all interrupt requests
        em86xx_write_reg(EM86XX_SR_REG, status) ;

        if (status & DmaIntNormal) {
                if (status & DmaIntRxCompleted) {
	                num_rx_desc_freed = em86xx_eth_rx(dev);
		}
                else if(status & DmaIntTxCompleted){
			if(private->need_restart_tx_queue){
				private->need_restart_tx_queue = 0;
				netif_start_queue(dev);
			}
/*			else if (status & DmaIntTxCompleted ) {
				ERR_PRINT("\t%s: DmaIntTxCompleted : Transmit completed (Normal)\n", dev->name);
			}
			else if (status & DmaIntTxNoBuffer ) {
				ERR_PRINT("\t%s: DmaIntTxNoBuffer : Transmit buffer unavailable (Normal)\n", dev->name);
			}
			else if (status & DmaIntRxCompleted ) {
				ERR_PRINT("\t%s: DmaIntRxCompleted : Completion of frame reception (Normal)\n", dev->name);
			}
			else if (status & DmaIntEarlyRx) {
				ERR_PRINT("\t%s: DmaIntEarlyRx : Early receive interrupt (Normal)\n", dev->name);
			}
*/
		}
		else {
                        ERR_PRINT("%s: unhandled NIS 0x%08lx\n", dev->name, status);
		}
	}
 
	if (status & DmaIntAbnormal) { 
                if (status & DmaIntRxNoBuffer) {
			if (num_rx_desc_freed == 0)
	                	num_rx_desc_freed = em86xx_eth_rx(dev);

			if(num_rx_desc_freed == 0){
                        	ERR_PRINT("%s: receive buffer unavailable 0x%08lx\n", dev->name, status);
                        	em86xx_eth_reset_rx_buf(dev);			
			}		
                } else if (status & DmaIntRxStopped) {
                        ERR_PRINT("%s: receive process stopped\n", dev->name);
                        em86xx_set_reg(EM86XX_CR_REG, DmaRxStart); 
                } else if (status & DmaIntTxStopped ) {
                        ERR_PRINT("%s: transmit process stopped\n", dev->name);
                        em86xx_write_reg(EM86XX_CR_REG, DmaTxStart); 
                } else if (status & DmaIntTxUnderflow) {
                        ERR_PRINT("%s: transmit underflow\n", dev->name);
                } else if (status & DmaIntEarlyTx ) {
                        ERR_PRINT("%s: Early transmit interrupt\n", dev->name);
                } else if (status & DmaIntBusError ) {
                        ERR_PRINT("%s: Fatal bus error\n", dev->name);
			if (status & DmaRxAbort) {
				ERR_PRINT("\t%s : DmaRxAbort : receiver bus abort\n", dev->name);
			}
			else if (status & DmaTxAbort) {
				ERR_PRINT("\t%s : DmaTxAbort : transmitter bus abort\n", dev->name);
			}
                } else {
                        ERR_PRINT("%s: unhandled AIS 0x%08lx\n", dev->name, status);
                }
        }
 
	if ((status & (DmaIntAbnormal | DmaIntNormal)) == 0) {
#ifdef DEBUG_RX_TX_STATE
		unsigned long st;
#endif
                DBG_PRINT("%s: Unhandled SR 0x%08lx --> %s %s\n", dev->name, status, (status & DmaTxState)?"DmaTxState":"", (status & DmaRxState)?"DmaRxState":"");
#ifdef DEBUG_RX_TX_STATE
		if (status & DmaTxState) {
			st = (status & DmaTxState);
			ERR_PRINT("\t%s: DmaTxState : Transmit process state (status == 0x%X)\n", dev->name, st);
			switch (st){
			case DmaTxStopped:
				ERR_PRINT("\t%s: DmaTxStopped : Stopped\n", dev->name);
				break;
			case DmaTxFetching:
				ERR_PRINT("\t%s: DmaTxFetching : Running - fetching the descriptor\n", dev->name);
				break;
			case DmaTxWaiting:
				ERR_PRINT("\t%s: DmaTxWaiting : Running - waiting for end of transmission\n", dev->name);
				break;
			case DmaTxReading:
				ERR_PRINT("\t%s: DmaTxReading : Running - reading the data from memory\n", dev->name);
				break;
			case DmaTxSuspended:
				ERR_PRINT("\t%s: DmaTxSuspended : Suspended\n", dev->name);
				break;
			case DmaTxClosing:
				ERR_PRINT("\t%s: DmaTxClosing : Running - closing descriptor\n", dev->name);
				break;
			default:
				ERR_PRINT("\t%s: UNKNOWN Tx state\n", dev->name);
				break;
			}
		}
		
		if (status & DmaRxState) {
			st = (status & DmaRxState);
			ERR_PRINT("\t%s: DmaRxState : Receive process state (status == 0x%X)\n", dev->name, st);
			switch (st){
			case DmaRxStopped: 
				ERR_PRINT("\t%s: DmaRxStopped : Stopped\n", dev->name); 
				break;
			case DmaRxFetching:
				ERR_PRINT("\t%s: DmaRxFetching : Running - fetching the descriptor\n", dev->name); 
				break;
			case DmaRxChecking:
				ERR_PRINT("\t%s: DmaRxChecking : Running - checking for end of packet \n", dev->name);
				break;
			case DmaRxWaiting:
				ERR_PRINT("\t%s: DmaRxWaiting : Running - waiting for packet\n", dev->name);
				break;
			case DmaRxSuspended:
				ERR_PRINT("\t%s: DmaRxSuspended : Suspended\n", dev->name);
				break;
			case DmaRxClosing:
				ERR_PRINT("\t%s: DmaRxClosing : Running - closing descriptor\n", dev->name);
				break;
			case DmaRxFlushing:
				ERR_PRINT("\t%s: DmaRxFlushing : Running - flushing the current frame\n", dev->name);
				break;
			case DmaRxQueuing:
				ERR_PRINT("\t%s: DmaRxQueuing : Running - queuing the recieve frame into host memory\n", dev->name);
				break;
			default:
				ERR_PRINT("\t%s: UNKNOWN Rx state\n", dev->name);
				break;
			}
		}
#endif
        }
	
#ifdef BOOTLOADER
	 return;
#else
        return IRQ_HANDLED;
#endif
 }

static void netdev_get_drvinfo (struct net_device *dev, struct ethtool_drvinfo *info)
{
	DBG_PRINT_INOUT_FUNC("START");
	strcpy(info->driver, DRV_NAME);
	strcpy(info->version, DRV_VERSION);
	strcpy(info->bus_info, "GBUS");
	DBG_PRINT_INOUT_FUNC("END");
}

static int netdev_get_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	EM86XX_ETH_PRIV *private = (EM86XX_ETH_PRIV *)dev->priv;
        unsigned long flags;
	int rc;

	DBG_PRINT_INOUT_FUNC("START");
        spin_lock_irqsave(&private->lock, flags);
	rc = mii_ethtool_gset(&private->mii_if, cmd);
        spin_unlock_irqrestore(&private->lock, flags);

	DBG_PRINT_INOUT_FUNC("END");
	return rc;
}

static int netdev_set_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	EM86XX_ETH_PRIV *private = (EM86XX_ETH_PRIV *)dev->priv;
        unsigned long flags;
	int rc;

	DBG_PRINT_INOUT_FUNC("START");
        spin_lock_irqsave(&private->lock, flags);
	rc = mii_ethtool_sset(&private->mii_if, cmd);
        spin_unlock_irqrestore(&private->lock, flags);

	DBG_PRINT_INOUT_FUNC("END");
	return rc;
}

static int netdev_nway_reset(struct net_device *dev)
{
	EM86XX_ETH_PRIV *private = (EM86XX_ETH_PRIV *)dev->priv;
	int rc;
	DBG_PRINT_INOUT_FUNC("START");
	rc = mii_nway_restart(&private->mii_if);
	DBG_PRINT_INOUT_FUNC("END");
	return rc;
}

static u32 netdev_get_link(struct net_device *dev)
{
	EM86XX_ETH_PRIV *private = (EM86XX_ETH_PRIV *)dev->priv;
	int rc;
	DBG_PRINT_INOUT_FUNC("START");
	rc = mii_link_ok(&private->mii_if);
	DBG_PRINT_INOUT_FUNC("END");
	return rc;
}

static u32 netdev_get_msglevel(struct net_device *dev)
{
	DBG_PRINT_INOUT_FUNC("START");
	DBG_PRINT_INOUT_FUNC("END");
	return 0;
}

static void netdev_set_msglevel(struct net_device *dev, u32 value)
{
}

static struct ethtool_ops netdev_ethtool_ops = {
	.get_drvinfo		= netdev_get_drvinfo,
	.get_settings		= netdev_get_settings,
	.set_settings		= netdev_set_settings,
	.nway_reset		= netdev_nway_reset,
	.get_link		= netdev_get_link,
	.get_msglevel		= netdev_get_msglevel,
	.set_msglevel		= netdev_set_msglevel,
	.get_sg			= ethtool_op_get_sg,
	.get_tx_csum		= ethtool_op_get_tx_csum,
};

#ifndef BOOTLOADER
/* Handling ioctl call */
//static int em86xx_eth_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
static int em86xx_eth_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
#if 1
	EM86XX_ETH_PRIV *private = (EM86XX_ETH_PRIV *)dev->priv;
        int ret;
        unsigned long flags;

        spin_lock_irqsave(&private->lock, flags);
        ret = generic_mii_ioctl(&private->mii_if, if_mii(ifr), cmd, NULL);
        spin_unlock_irqrestore(&private->lock, flags);

        return ret;

#else
	EM86XX_ETH_PRIV *private = (EM86XX_ETH_PRIV *)dev->priv;
        unsigned long flags;
	int ret = 0;
	struct mii_ioctl_data *data = (struct mii_ioctl_data *)&ifr->ifr_data;
	struct ifr_data_struct
 	{
		u32 unit;
    		u32 addr;
    		u32 data;
  	} *req = (struct ifr_data_struct *)ifr->ifr_data;

	DBG_PRINT_INOUT_FUNC("START");

	if (!netif_running(dev)) {
		DBG_PRINT( "%s : ioctl, but device not running !!\n", dev->name);
		return -EINVAL;
	}

        spin_lock_irqsave(&private->lock, flags);
	
	if( req->unit == 3 ) {
		DBG_PRINT( "em86xx::read/write MII registers(%s, cmd=%08x) unit=0x%x  reg=0x%x\n", 
			   dev->name, cmd, req->unit, req->addr );
		ret = generic_mii_ioctl(&private->mii_if, data, cmd, NULL);
	}
	else {
		switch( cmd ) {
		case SIOCDEVPRIVATE:  
			DBG_PRINT( "em86xx::read registers(%s, cmd=%08x) unit=0x%x  reg=0x%x\n", 
				   dev->name, cmd, req->unit, req->addr );
			
			if( req->unit == 0 ){        /* Read register */
      				req->data = em86xx_read_reg( req->addr );
    			}
			else if( req->unit == 1 ){   /* Read MAC register */
      				req->data = em86xx_read_mac_reg( req->addr );
    			}
    			else if( req->unit == 2 ) {   /* Read DMA register */
      				req->data = em86xx_read_dma_reg( req->addr );
    			}
			else {
				DBG_PRINT( "Unknow read request : %d\n", req->unit);
				ret = -EOPNOTSUPP;
			}
    			break;
  		case SIOCDEVPRIVATE+1:                /* Write registers */
			DBG_PRINT( "em86xx::write registers(%s, cmd=%08x) unit=0x%x reg=0x%x data=0x%x\n", 
				    dev->name, cmd ,req->unit, req->addr, req->data);

    			if( req->unit == 0 ) {        /* Write register */
      				em86xx_write_reg( req->addr, req->data );
    			}
    			else if( req->unit == 1 ) {        /* Write MAC register */
      				em86xx_write_mac_reg( req->addr, req->data );
    			}
    			else if( req->unit == 2 ) {   /* Write DMA register */
      				em86xx_write_dma_reg( req->addr, req->data );
    			}
			else {
				DBG_PRINT( "Unknow write request : %d\n", req->unit);
				ret = -EOPNOTSUPP;
			}
    			break;			
		default:
			DBG_PRINT( "Unknow cmd : %d\n", cmd);
    			ret = -EOPNOTSUPP;
		}
	}
	
        spin_unlock_irqrestore(&private->lock, flags);

	DBG_PRINT_INOUT_FUNC("END");
	return ret;
#endif
}
#endif

/* Kernel level ethernet initialization */
static int em86xx_eth_init(struct net_device *dev)
{
	EM86XX_ETH_PRIV *private;

	DBG_PRINT_INOUT_FUNC("START");
	/* Turn off IRQ and stop receive/transmit */
	em86xx_write_reg(EM86XX_CR_REG, 0);
	em86xx_write_reg(EM86XX_IER_REG, 0);
	em86xx_write_reg(EM86XX_SR_REG, em86xx_read_reg(EM86XX_SR_REG));

	if (dev == NULL)
  		goto failed;

	private = (EM86XX_ETH_PRIV *)dev->priv;

	/* Initialize private data */
	spin_lock_init(&private->lock);
	
	private->rxdsc = (volatile struct em86xx_desc *)NON_CACHED((u32)private->eth_rxdsc);
	private->txdsc = (volatile struct em86xx_desc *)NON_CACHED((u32)private->eth_txdsc);
	private->rxbuf = (volatile unsigned char *)NON_CACHED((u32)private->eth_rxbuf);
	private->txbuf = (volatile unsigned char *)NON_CACHED((u32)private->eth_txbuf);

	/* Ethernet device initialization */
#ifdef BOOTLOADER
        /* Fill in the fields of the device structure with ethernet-generic values.*/
//      dev->type               = ARPHRD_ETHER;
        dev->mtu                = ETH_DATA_LEN; /* eth_mtu */
        dev->addr_len           = ETH_ALEN;
        memset(dev->broadcast, 0xff, ETH_ALEN);
        dev->hard_header_len = ETH_HLEN;
#else
	ether_setup(dev);
#endif

	/* reset dma engine*/
	em86xx_write_reg(EM86XX_BMR_REG, DmaResetOn);
	em86xx_write_reg(EM86XX_BMR_REG, DmaResetOff);

	if (em86xx_set_mac(dev))
  		goto failed;

	/* Hook up with handlers */
#ifdef BOOTLOADER
        dev->send_packet         = em86xx_eth_tx;
        dev->open                = em86xx_eth_open;
        dev->close               = em86xx_eth_close;
#else
	dev->get_stats		 = em86xx_eth_stats;
	dev->hard_start_xmit	 = em86xx_eth_tx;
	dev->open		 = em86xx_eth_open;
	dev->stop		 = em86xx_eth_close;
#ifdef USE_HW_FILTERING
	dev->set_multicast_list  = em86xx_eth_set_multicast_list;
#endif
	dev->do_ioctl		 = em86xx_eth_ioctl;
	dev->set_mac_address	 = em86xx_eth_set_macaddr;
        dev->ethtool_ops         = &netdev_ethtool_ops;

	dev->tx_queue_len = private->num_txdesc; 
//	dev->flags &= ~IFF_MULTICAST;
//	dev->flags |= IFF_DEBUG;
#endif
	DBG_PRINT("Detect PHY\n");
	if (phy_detect(dev, &private->mii_if) < 0) {
		DBG_PRINT_INOUT_FUNC("END");
		return -ENODEV;
	}

	DBG_PRINT_INOUT_FUNC("END");
	return 0;

failed:
	DBG_PRINT_INOUT_FUNC("END");
	return(-EIO);
}

static int em86xx_eth_reset_desc(struct net_device *dev, int *reset)
{
	EM86XX_ETH_PRIV *private;
	int i;

	DBG_PRINT_INOUT_FUNC("START");
	if (dev == NULL)
  		return(-EIO);
	else
		private = (EM86XX_ETH_PRIV *)dev->priv;

	if (*reset) {
		memset((void*)&private->stats, 0, sizeof(struct net_device_stats));
		memset((void*)&private->mii_if, 0, sizeof(struct mii_if_info));
		private->autoneg_active = 0;
		private->phy_loopback = 0;	

#ifndef BOOTLOADER

#ifdef USE_KERNEL_TIMER 
		memset((void*)&private->eth_timer, 0, sizeof(struct timer_list));
#endif
		private->need_restart_tx_queue = 0; 
#endif
	}

	*reset = 0;

	/* Clear all tx/rx buffer memory */
	memset((void *)(private->rxbuf), 0, private->num_rxdesc * R_BUF_SIZE);
	memset((void *)(private->txbuf), 0, private->num_txdesc * T_BUF_SIZE);

	/* Initialize the indices */
	private->last_rxidx = 0;
	private->next_txidx = 0;

#ifndef BOOTLOADER
	if (private->rx_skb_list != NULL) {
		for (i = 0; i < private->num_rxdesc; i++) {
			if (private->rx_skb_list[i] != NULL)
				dev_kfree_skb(private->rx_skb_list[i]);
		}
	}
	memset((void *)private->rx_skb_list, 0, sizeof(struct skb_buff *) * private->num_rxdesc);
#endif

	/* Intialize the descriptors */
	em86xx_eth_setup_desc(dev);

	phy_detect(dev, &private->mii_if);


	DBG_PRINT_INOUT_FUNC("END");
	return 0;
}

/* Driver installation: this is where the thing starts */
#ifdef BOOTLOADER
int em86xx_eth_probe(struct net_device *dev)
#else
static int __init em86xx_eth_startup(void)
#endif
{
#ifndef BOOTLOADER
	struct net_device *dev 	= &em86xx_eth_dev;
#endif
	int err = 0, i;
	EM86XX_ETH_PRIV *private= NULL;
#ifndef STATIC_BUF_ALLOC
	unsigned long max_num_rxdsc, max_num_txdsc;
	unsigned long rxbuf_size, txbuf_size;
#endif

	DBG_PRINT_INOUT_FUNC("START");
#if defined(CONFIG_TANGOX)
        if (tangox_ethernet_enabled(0) == 0) {
                MSG_PRINT(KERN_WARNING "TangoX builtin ethernet is disabled.\n");
                return(0);
        }
#endif
	/* Turn off IRQ and stop receive/transmit */
	em86xx_write_reg(EM86XX_CR_REG, 0);
	em86xx_write_reg(EM86XX_IER_REG, 0);
	em86xx_write_reg(EM86XX_SR_REG, em86xx_read_reg(EM86XX_SR_REG));

	if(dev != NULL) {
		if(dev->priv != NULL) {
			private = (EM86XX_ETH_PRIV *)dev->priv;
			if (private->dev_count != 0) {
				err = -EIO;
				goto failed;
			} 
		}
	}

	/* Allocate memory for private data */
#ifdef BOOTLOADER
        dev->priv = private = (EM86XX_ETH_PRIV *)malloc(sizeof(EM86XX_ETH_PRIV));//, GFP_KERNEL);
#else
	
	DBG_PRINT("KMALLOC\n");
	dev->priv = private = (EM86XX_ETH_PRIV *)kmalloc(sizeof(EM86XX_ETH_PRIV), GFP_KERNEL);
#endif
	if (dev->priv == NULL) {
		err = -ENOMEM;
		goto failed;
	}

	memset(dev->priv, 0, sizeof(EM86XX_ETH_PRIV));

	private->num_rxdesc = num_rxdesc_param;;
	private->num_txdesc = num_txdesc_param;

	DBG_PRINT("private->num_rxdesc = %lu, private->num_txdesc = %lu \n", private->num_rxdesc, private->num_txdesc);
#ifndef STATIC_BUF_ALLOC	
	DBG_PRINT("NON STATIC_BUF_ALLOC\n");
	/* Validating module parameters */
	max_num_rxdsc = max_num_txdsc = PAGE_SIZE / (2 * sizeof(struct em86xx_desc));
 #ifdef BOOTLOADER
	DBG_PRINT("BOOTLOADER\n");
	if (((private->num_rxdesc < MIN_NUM_RDESC) || (private->num_rxdesc > max_num_rxdsc)) ||
	    ((private->num_txdesc < MIN_NUM_TDESC) || (private->num_txdesc > max_num_txdsc))) {
		err = -EIO;
		goto failed;
	} else if ((private->desc_page = NON_CACHED(malloc(PAGE_SIZE))) == 0) {
		err = -ENOMEM;
		goto failed;
	}
 #else
	DBG_PRINT("NOT BOOTLOADER\n");
	if (((private->num_rxdesc < MIN_NUM_RDESC) || (private->num_rxdesc > max_num_rxdsc)) ||
	    ((private->num_txdesc < MIN_NUM_TDESC) || (private->num_txdesc > max_num_txdsc))) {
		err = -EIO;
		goto failed;
	} else if ((private->desc_page = __get_free_page(GFP_KERNEL | GFP_DMA)) == 0) {
		err = -ENOMEM;
		goto failed;
	} else if ((private->rx_skb_list = (struct skb_buff **)kmalloc(sizeof(struct skb_buff *) 
				                 * private->num_rxdesc, GFP_KERNEL)) == NULL) {
		err = -ENOMEM;
		goto failed;
	}

	memset((void *)private->rx_skb_list, 0, sizeof(struct skb_buff *) * private->num_rxdesc);
 #endif

	/* Split a page for both rx/tx descriptor */
	private->eth_rxdsc = (struct em86xx_desc *)private->desc_page;
	private->eth_txdsc = (struct em86xx_desc *)(private->desc_page + ((1 << PAGE_SHIFT) / 2));

	/* Calculate the size needed for tx/rx -- aligned by pages */
	rxbuf_size = private->num_rxdesc * R_BUF_SIZE;
	txbuf_size = private->num_txdesc * T_BUF_SIZE;
	if ((rxbuf_size & (PAGE_SIZE - 1)) != 0)
		rxbuf_size = (rxbuf_size & PAGE_MASK) + PAGE_SIZE;
	if ((txbuf_size & (PAGE_SIZE - 1)) != 0)
		txbuf_size = (txbuf_size & PAGE_MASK) + PAGE_SIZE;
	DBG_PRINT("txbuf_size = 0x%08lx, rxbuf_size = 0x%08lx\n", txbuf_size, rxbuf_size);

 #ifndef BOOTLOADER
	/* Calculate the order needed for tx/rx */
	for (private->rxbuf_order = 0; private->rxbuf_order < MAX_ORDER; private->rxbuf_order++) {
		if ((PAGE_SIZE * (1 << private->rxbuf_order)) >= rxbuf_size)
			break;
	}

	for (private->txbuf_order = 0; private->txbuf_order < MAX_ORDER; private->txbuf_order++) {
		if ((PAGE_SIZE * (1 << private->txbuf_order)) >= txbuf_size)
			break;
	}

	if ((private->rxbuf_order >= MAX_ORDER) || (private->txbuf_order >= MAX_ORDER)) {
		err = -ENOMEM;
		goto failed;
	}

	DBG_PRINT("private->txbuf_order = 0x%08lx, private->rxbuf_order = 0x%08lx\n", private->txbuf_order, private->rxbuf_order);
 #endif
		
 #ifdef BOOTLOADER
	if ((private->rxbuf_pages = NON_CACHED(malloc(rxbuf_size))) == 0) {
		err = -ENOMEM;
		goto failed;
	} else if ((private->txbuf_pages = NON_CACHED(malloc(txbuf_size))) == 0) {
		err = -ENOMEM;
		goto failed;
	}
 #else
	if ((private->rxbuf_pages = __get_free_pages(GFP_KERNEL | GFP_DMA, private->rxbuf_order)) == 0) {
		err = -ENOMEM;
		goto failed;
	} else if ((private->txbuf_pages = __get_free_pages(GFP_KERNEL | GFP_DMA, private->txbuf_order)) == 0) {
		err = -ENOMEM;
		goto failed;
	}
 #endif

	private->eth_rxbuf  = (unsigned char *)private->rxbuf_pages;
	private->eth_txbuf  = (unsigned char *)private->txbuf_pages;
#else
	DBG_PRINT("STATIC_BUF_ALLOC\n");
#endif

#ifndef BOOTLOADER
	SET_MODULE_OWNER(&em86xx_eth_dev);
#endif

	MSG_PRINT("Ethernet driver for EM86XX (v1.0)");
        dev->irq = IRQ_ETHERNET;

	private->reset_flag 	= 1;

	/* Get a device name: normally it'll be eth0 */
#ifdef BOOTLOADER
        memcpy( dev->name, DRIVER, sizeof(DRIVER));

        /* Register ISR */
        em86xx_request_irq(IRQ_ETHERNET, em86xx_eth_intr_handler, dev);
#else
	if ((err = dev_alloc_name(&em86xx_eth_dev, "eth%d")) < 0){
		printk("dev_alloc_name failed\n");
  		goto failed;
	}

	/* Register ISR */
	if ((err = request_irq(IRQ_ETHERNET, em86xx_eth_intr_handler, IRQF_SHARED, "ethernet", 
      	 	 &em86xx_eth_dev)) != 0) {
  		err = -EIO;
		printk("request_irq failed\n");
  		goto failed;
	} else
#endif 
  		MSG_PRINT(" on %s (IRQ: %d)", dev->name, IRQ_ETHERNET);


	if (em86xx_get_macaddr(dev->dev_addr)) {
  		err = -EIO;
		printk("get macaddr failed\n");
  		goto failed;
	}

	MSG_PRINT("\n(MAC %02x", dev->dev_addr[0]);
	for (i = 1; i < 6; i++)
	  	MSG_PRINT(":%02x", dev->dev_addr[i]);
	MSG_PRINT(", tx_desc/rx_desc = %ld/%ld), ", private->num_txdesc, private->num_rxdesc);
	if (phy_address < 0)
		MSG_PRINT("PHY probing enabled\n");
	else
		MSG_PRINT("PHY address: %d\n", phy_address);

	/* Point to driver initialization routine and register the device with kernel */
	dev->init = em86xx_eth_init;
#ifdef BOOTLOADER
        em86xx_eth_init(dev);
	DBG_PRINT("AFTER em86xx_eth_init\n");
        dev->state = NETDEV_DOWN;
	DBG_PRINT("AFTER STATE\n");
#else
	if (register_netdev(dev) != 0) {
  		err = -EIO;
  		goto failed;
	}
#endif

	MSG_PRINT("%s: driver installation completed.\n", dev->name);
	private->dev_count++;
	netif_carrier_off(dev); /* By default the carrier is off */
	DBG_PRINT_INOUT_FUNC("END");
	return 0;

failed:
  	if (dev->priv != NULL) {
#ifdef BOOTLOADER
                free(dev->priv);
#else
    		kfree(dev->priv);
#endif
    		dev->priv = NULL;
  	}
#ifndef STATIC_BUF_ALLOC
#ifdef BOOTLOADER
	if (private->desc_page   != 0)
		free((void *)CACHED(private->desc_page));
	if (private->rxbuf_pages != 0)
		free((void *)CACHED(private->rxbuf_pages));
	if (private->txbuf_pages != 0)
		free((void *)CACHED(private->txbuf_pages));
#else
	if (private->desc_page   != 0)
		free_page(private->desc_page);
	if (private->rxbuf_pages != 0)
		free_pages(private->rxbuf_pages, private->rxbuf_order);
	if (private->txbuf_pages != 0)
		free_pages(private->txbuf_pages, private->txbuf_order);
	if (private->rx_skb_list != NULL);
		kfree(private->rx_skb_list);
#endif
#endif
  	MSG_PRINT("%s: driver installation failed.\n", dev->name);
	DBG_PRINT_INOUT_FUNC("END");
  	return(err);
}

#ifndef BOOTLOADER
/* Uninstallation of drive */
static void __exit em86xx_eth_shutdown(void)
{
	struct net_device *dev 	= &em86xx_eth_dev;
	EM86XX_ETH_PRIV *private= NULL;
#if defined(CONFIG_TANGOX)
        if (tangox_ethernet_enabled(0) == 0) 
                return;
#endif

	DBG_PRINT_INOUT_FUNC("START");

	if (dev == NULL)
  		return;
	else
		private = (EM86XX_ETH_PRIV *)dev->priv;

	if (private == NULL) {
		return;
	}

	if (private->dev_count != 0) {
		/* Turn off IRQ and stop receive/transmit */
  		em86xx_write_reg(EM86XX_CR_REG, 0);
  		em86xx_write_reg(EM86XX_IER_REG, 0);

  		/* Unregister the device and ISR */
  		free_irq(IRQ_ETHERNET, &em86xx_eth_dev);
		unregister_netdev(&em86xx_eth_dev);

		/* Set desc base address registers to 0 */
		em86xx_write_reg(EM86XX_RLBAR_REG, 0);
		em86xx_write_reg(EM86XX_TLBAR_REG, 0);
#ifndef STATIC_BUF_ALLOC
#ifdef BOOTLOADER
		if (private->desc_page   != 0)
			free((void *)CACHED(private->desc_page));
		if (private->rxbuf_pages != 0)
			free((void *)CACHED(private->rxbuf_pages));
		if (private->txbuf_pages != 0)
			free((void *)CACHED(private->txbuf_pages));
#else
		if (private->desc_page   != 0)
			free_page(private->desc_page);
		if (private->rxbuf_pages != 0)
			free_pages(private->rxbuf_pages, private->rxbuf_order);
		if (private->txbuf_pages != 0)
			free_pages(private->txbuf_pages, private->txbuf_order);
		if (private->rx_skb_list != NULL) {
			int i;
			for (i = 0; i < private->num_rxdesc; i++) {
				if (private->rx_skb_list[i] != NULL)
					dev_kfree_skb(private->rx_skb_list[i]);
			}
			kfree(private->rx_skb_list);
		}
#endif
#endif
		/* Free up memory */
		if (em86xx_eth_dev.priv != NULL) {
			kfree(em86xx_eth_dev.priv);
			em86xx_eth_dev.priv = NULL;
  		}
	} 

	DBG_PRINT_INOUT_FUNC("END");
}

/* Register startup/shutdown routines */
module_init(em86xx_eth_startup);
module_exit(em86xx_eth_shutdown);
#endif

