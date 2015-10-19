#ifndef _NB8800_H_
#define _NB8800_H_

#include <linux/types.h>
#include <linux/skbuff.h>
#include <linux/phy.h>
#include <linux/clk.h>
#include <linux/timer.h>

#define DEF_RX_DESC_COUNT		512
#define DEF_TX_DESC_COUNT		128

#define NB8800_DESC_LOW			16
#define NB8800_DESC_RECLAIM		64

#define RX_BUF_SIZE			1552
#define TX_BUF_SIZE			1552

#define RX_COPYBREAK			256

#define RFI				7
#define TFI				1

#define MAX_MDC_CLOCK			2500000

/* register offsets */
#define NB8800_TX_CTL1			0x00
#define TX_TPD				(1 << 5)
#define TX_APPEND_FCS			(1 << 4)
#define TX_PAD_EN			(1 << 3)
#define TX_RETRY_EN			(1 << 2)
#define TX_EN				(1 << 0)

#define NB8800_TX_CTL2			0x01

#define NB8800_RX_CTL			0x04
#define RX_BC_DISABLE			(1 << 7)
#define RX_RUNT				(1 << 6)
#define RX_AF_EN			(1 << 5)
#define RX_PAUSE_EN			(1 << 3)
#define RX_SEND_CRC			(1 << 2)
#define RX_PAD_STRIP			(1 << 1)
#define RX_EN				(1 << 0)

#define NB8800_RANDOM_SEED		0x8
#define NB8800_TX_SDP			0x14
#define NB8800_TX_TPDP1			0x18
#define NB8800_TX_TPDP2			0x19
#define NB8800_SLOT_TIME		0x1c

#define NB8800_MDIO_CMD			0x20
#define MIIAR_ADDR(x)			((x) << 21)
#define MIIAR_REG(x)			((x) << 16)
#define MIIAR_DATA(x)			((x) <<	 0)
#define MDIO_CMD_GO			(1 << 31)
#define MDIO_CMD_WR			(1 << 26)

#define NB8800_MDIO_STS			0x24
#define MDIO_STS_ERR			(1 << 31)

#define NB8800_MC_ADDR1			0x28
#define NB8800_MC_ADDR2			0x29
#define NB8800_MC_ADDR3			0x2a
#define NB8800_MC_ADDR4			0x2b
#define NB8800_MC_ADDR5			0x2c
#define NB8800_MC_ADDR6			0x2d
#define NB8800_MC_INIT			0x2e
#define NB8800_UC_ADDR1			0x3c
#define NB8800_UC_ADDR2			0x3d
#define NB8800_UC_ADDR3			0x3e
#define NB8800_UC_ADDR4			0x3f
#define NB8800_UC_ADDR5			0x40
#define NB8800_UC_ADDR6			0x41

#define NB8800_MAC_MODE			0x44
#define RGMII_MODE			(1 << 7)
#define HALF_DUPLEX			(1 << 4)
#define BURST_EN			(1 << 3)
#define LOOPBACK_EN			(1 << 2)
#define GMAC_MODE			(1 << 0)

#define NB8800_IC_THRESHOLD		0x50
#define NB8800_PE_THRESHOLD		0x51
#define NB8800_PF_THRESHOLD		0x52
#define NB8800_TX_BUFSIZE		0x54
#define NB8800_FIFO_CTL			0x56
#define NB8800_PQ1			0x60
#define NB8800_PQ2			0x61
#define NB8800_MAC_ADDR1		0x6a
#define NB8800_MAC_ADDR2		0x6b
#define NB8800_MAC_ADDR3		0x6c
#define NB8800_MAC_ADDR4		0x6d
#define NB8800_MAC_ADDR5		0x6e
#define NB8800_MAC_ADDR6		0x6f
#define NB8800_STAT_DATA1		0x78
#define NB8800_STAT_DATA2		0x79
#define NB8800_STAT_DATA3		0x7a
#define NB8800_STAT_DATA4		0x7b
#define NB8800_STAT_INDEX		0x7c
#define NB8800_STAT_CLEAR		0x7d

#define NB8800_SLEEP_MODE		0x7e
#define SLEEP_MODE			(1 << 0)

#define NB8800_WAKEUP			0x7f
#define WAKEUP				(1 << 0)

#define NB8800_TXC_CR			0x100
#define TCR_LK				(1 << 12)
#define TCR_DS				(1 << 11)
#define TCR_BTS(x)			(((x) & 0x7) << 8)
#define TCR_DIE				(1 << 7)
#define TCR_TFI(x)			(((x) & 0x7) << 4)
#define TCR_LE				(1 << 3)
#define TCR_RS				(1 << 2)
#define TCR_DM				(1 << 1)
#define TCR_EN				(1 << 0)

#define NB8800_TXC_SR			0x104
#define TSR_DE				(1 << 3)
#define TSR_DI				(1 << 2)
#define TSR_TO				(1 << 1)
#define TSR_TI				(1 << 0)

#define NB8800_TX_SAR			0x108
#define NB8800_TX_DESC_ADDR		0x10c

#define NB8800_TX_REPORT_ADDR		0x110
#define TX_BYTES_TRASFERRED(x)		(((x) >> 16) & 0xffff)
#define TX_FIRST_DEFERRAL		(1 << 7)
#define TX_EARLY_COLLISIONS(x)		(((x) >> 3) & 0xf)
#define TX_LATE_COLLISION		(1 << 2)
#define TX_PACKET_DROPPED		(1 << 1)
#define TX_FIFO_UNDERRUN		(1 << 0)
#define IS_TX_ERROR(r)			((r) & 0x87)

#define NB8800_TX_FIFO_SR		0x114
#define NB8800_TX_ITR			0x118

#define NB8800_RXC_CR			0x200
#define RCR_FI				(1 << 13)
#define RCR_LK				(1 << 12)
#define RCR_DS				(1 << 11)
#define RCR_BTS(x)			(((x) & 7) << 8)
#define RCR_DIE				(1 << 7)
#define RCR_RFI(x)			(((x) & 7) << 4)
#define RCR_LE				(1 << 3)
#define RCR_RS				(1 << 2)
#define RCR_DM				(1 << 1)
#define RCR_EN				(1 << 0)

#define NB8800_RXC_SR			0x204
#define RSR_DE				(1 << 3)
#define RSR_DI				(1 << 2)
#define RSR_RO				(1 << 1)
#define RSR_RI				(1 << 0)

#define NB8800_RX_SAR			0x208
#define NB8800_RX_DESC_ADDR		0x20c

#define NB8800_RX_REPORT_ADDR		0x210
#define RX_BYTES_TRANSFERRED(x)		(((x) >> 16) & 0xFFFF)
#define RX_MULTICAST_PKT		(1 << 9)
#define RX_BROADCAST_PKT		(1 << 8)
#define RX_LENGTH_ERR			(1 << 7)
#define RX_FCS_ERR			(1 << 6)
#define RX_RUNT_PKT			(1 << 5)
#define RX_FIFO_OVERRUN			(1 << 4)
#define RX_LATE_COLLISION		(1 << 3)
#define RX_FRAME_LEN_ERROR		(1 << 2)
#define RX_ERROR_MASK			0xfc
#define IS_RX_ERROR(r)			((r) & RX_ERROR_MASK)

#define NB8800_RX_FIFO_SR		0x214
#define NB8800_RX_ITR			0x218

#define NB8800_PAD_MODE			0x400
#define NB8800_MDIO_CLKDIV		0x420
#define NB8800_SW_RESET			0x424

struct nb8800_dma_desc {
	u32 s_addr;
	u32 n_addr;
	u32 r_addr;
	u32 config;
	u8  buf[12];
	u32 report;
};

#define DESC_ID				(1 << 23)
#define DESC_EOC			(1 << 22)
#define DESC_EOF			(1 << 21)
#define DESC_LK				(1 << 20)
#define DESC_DS				(1 << 19)
#define DESC_BTS(x)			(((x) & 0x7) <<16)

struct rx_buf {
	struct page *page;
	int offset;
};

struct tx_buf {
	struct sk_buff *skb;
	dma_addr_t desc_dma;
	int frags;
};

struct tx_skb_data {
	dma_addr_t dma_addr;
	unsigned int dma_len;
};

struct nb8800_priv {
	struct napi_struct		napi;

	void __iomem			*base;

	struct nb8800_dma_desc		*rx_descs;
	struct rx_buf			*rx_bufs;
	u16				rx_desc_count;
	u16				rx_eoc;

	struct nb8800_dma_desc		*tx_descs;
	struct tx_buf			*tx_bufs;
	atomic_t			tx_free;
	u16				tx_desc_count;
	u16				tx_next;
	u32 				tx_pending;
	u16				tx_reclaim_next;
	u16 				tx_reclaim_limit;
	u32				tx_dirty;
	u16				tx_done;

	struct tasklet_struct		tx_reclaim_tasklet;
	struct timer_list		tx_reclaim_timer;

	struct net_device_stats		stats;

	struct mii_bus			*mii_bus;
	struct phy_device		*phydev;
	int				speed;
	int				duplex;
	int				link;

	dma_addr_t			rx_desc_dma;
	dma_addr_t			tx_desc_dma;

	int				gigabit;
	struct clk			*clk;
};

#endif /* _NB8800_H_ */
