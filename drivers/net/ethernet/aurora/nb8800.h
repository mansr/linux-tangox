#ifndef _NB8800_H_
#define _NB8800_H_

#include <linux/types.h>
#include <linux/skbuff.h>
#include <linux/phy.h>
#include <linux/clk.h>
#include <linux/timer.h>
#include <linux/bitops.h>

#define RX_DESC_COUNT			256
#define TX_DESC_COUNT			256

#define NB8800_DESC_LOW			4

#define RX_BUF_SIZE			1552
#define TX_BUF_SIZE			1552

#define RX_COPYBREAK			256

#define RFI				7
#define TFI				1

#define MAX_MDC_CLOCK			2500000

/* register offsets */
#define NB8800_TX_CTL1			0x00
#define TX_TPD				BIT(5)
#define TX_APPEND_FCS			BIT(4)
#define TX_PAD_EN			BIT(3)
#define TX_RETRY_EN			BIT(2)
#define TX_EN				BIT(0)

#define NB8800_TX_CTL2			0x01

#define NB8800_RX_CTL			0x04
#define RX_BC_DISABLE			BIT(7)
#define RX_RUNT				BIT(6)
#define RX_AF_EN			BIT(5)
#define RX_PAUSE_EN			BIT(3)
#define RX_SEND_CRC			BIT(2)
#define RX_PAD_STRIP			BIT(1)
#define RX_EN				BIT(0)

#define NB8800_RANDOM_SEED		0x8
#define NB8800_TX_SDP			0x14
#define NB8800_TX_TPDP1			0x18
#define NB8800_TX_TPDP2			0x19
#define NB8800_SLOT_TIME		0x1c

#define NB8800_MDIO_CMD			0x20
#define MIIAR_ADDR(x)			((x) << 21)
#define MIIAR_REG(x)			((x) << 16)
#define MIIAR_DATA(x)			((x) <<	 0)
#define MDIO_CMD_GO			BIT(31)
#define MDIO_CMD_WR			BIT(26)

#define NB8800_MDIO_STS			0x24
#define MDIO_STS_ERR			BIT(31)

#define NB8800_MC_ADDR(i)		(0x28 + (i))
#define NB8800_MC_INIT			0x2e
#define NB8800_UC_ADDR(i)		(0x3c + (i))

#define NB8800_MAC_MODE			0x44
#define RGMII_MODE			BIT(7)
#define HALF_DUPLEX			BIT(4)
#define BURST_EN			BIT(3)
#define LOOPBACK_EN			BIT(2)
#define GMAC_MODE			BIT(0)

#define NB8800_IC_THRESHOLD		0x50
#define NB8800_PE_THRESHOLD		0x51
#define NB8800_PF_THRESHOLD		0x52
#define NB8800_TX_BUFSIZE		0x54
#define NB8800_FIFO_CTL			0x56
#define NB8800_PQ1			0x60
#define NB8800_PQ2			0x61
#define NB8800_SRC_ADDR(i)		(0x6a + (i))
#define NB8800_STAT_DATA		0x78
#define NB8800_STAT_INDEX		0x7c
#define NB8800_STAT_CLEAR		0x7d

#define NB8800_SLEEP_MODE		0x7e
#define SLEEP_MODE			BIT(0)

#define NB8800_WAKEUP			0x7f
#define WAKEUP				BIT(0)

#define NB8800_TXC_CR			0x100
#define TCR_LK				BIT(12)
#define TCR_DS				BIT(11)
#define TCR_BTS(x)			(((x) & 0x7) << 8)
#define TCR_DIE				BIT(7)
#define TCR_TFI(x)			(((x) & 0x7) << 4)
#define TCR_LE				BIT(3)
#define TCR_RS				BIT(2)
#define TCR_DM				BIT(1)
#define TCR_EN				BIT(0)

#define NB8800_TXC_SR			0x104
#define TSR_DE				BIT(3)
#define TSR_DI				BIT(2)
#define TSR_TO				BIT(1)
#define TSR_TI				BIT(0)

#define NB8800_TX_SAR			0x108
#define NB8800_TX_DESC_ADDR		0x10c

#define NB8800_TX_REPORT_ADDR		0x110
#define TX_BYTES_TRASFERRED(x)		(((x) >> 16) & 0xffff)
#define TX_FIRST_DEFERRAL		BIT(7)
#define TX_EARLY_COLLISIONS(x)		(((x) >> 3) & 0xf)
#define TX_LATE_COLLISION		BIT(2)
#define TX_PACKET_DROPPED		BIT(1)
#define TX_FIFO_UNDERRUN		BIT(0)
#define IS_TX_ERROR(r)			((r) & 0x87)

#define NB8800_TX_FIFO_SR		0x114
#define NB8800_TX_ITR			0x118

#define NB8800_RXC_CR			0x200
#define RCR_FL				BIT(13)
#define RCR_LK				BIT(12)
#define RCR_DS				BIT(11)
#define RCR_BTS(x)			(((x) & 7) << 8)
#define RCR_DIE				BIT(7)
#define RCR_RFI(x)			(((x) & 7) << 4)
#define RCR_LE				BIT(3)
#define RCR_RS				BIT(2)
#define RCR_DM				BIT(1)
#define RCR_EN				BIT(0)

#define NB8800_RXC_SR			0x204
#define RSR_DE				BIT(3)
#define RSR_DI				BIT(2)
#define RSR_RO				BIT(1)
#define RSR_RI				BIT(0)

#define NB8800_RX_SAR			0x208
#define NB8800_RX_DESC_ADDR		0x20c

#define NB8800_RX_REPORT_ADDR		0x210
#define RX_BYTES_TRANSFERRED(x)		(((x) >> 16) & 0xFFFF)
#define RX_MULTICAST_PKT		BIT(9)
#define RX_BROADCAST_PKT		BIT(8)
#define RX_LENGTH_ERR			BIT(7)
#define RX_FCS_ERR			BIT(6)
#define RX_RUNT_PKT			BIT(5)
#define RX_FIFO_OVERRUN			BIT(4)
#define RX_LATE_COLLISION		BIT(3)
#define RX_FRAME_LEN_ERROR		BIT(2)
#define RX_ERROR_MASK			0xfc
#define IS_RX_ERROR(r)			((r) & RX_ERROR_MASK)

#define NB8800_RX_FIFO_SR		0x214
#define NB8800_RX_ITR			0x218

/* Sigma Designs SMP86xx additional registers */
#define NB8800_TANGOX_PAD_MODE		0x400
#define NB8800_TANGOX_MDIO_CLKDIV	0x420
#define NB8800_TANGOX_RESET		0x424

struct nb8800_dma_desc {
	u32 s_addr;
	u32 n_addr;
	u32 r_addr;
	u32 config;
	u8  buf[12];
	u32 report;
};

#define DESC_ID				BIT(23)
#define DESC_EOC			BIT(22)
#define DESC_EOF			BIT(21)
#define DESC_LK				BIT(20)
#define DESC_DS				BIT(19)
#define DESC_BTS(x)			(((x) & 0x7) << 16)

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
	u16				rx_eoc;

	struct nb8800_dma_desc		*tx_descs;
	struct tx_buf			*tx_bufs;
	atomic_t			tx_free;
	u32				tx_pending;
	u16				tx_next;
	u16				tx_done;

	struct mii_bus			*mii_bus;
	struct phy_device		*phydev;
	int				speed;
	int				duplex;
	int				link;

	dma_addr_t			rx_desc_dma;
	dma_addr_t			tx_desc_dma;

	struct clk			*clk;
};

struct nb8800_ops {
	void (*init)(struct net_device *dev);
	void (*reset)(struct net_device *dev);
};

#endif /* _NB8800_H_ */
