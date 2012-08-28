/* linux/drivers/mtd/nand/smp8xxx.c
 *
 * Copyright (c) 2009 Sigma Designs
 *
 * SMP8xxx NAND driver
 *
 * Changelog:
 *	27-Oct-2010  DanielR  Add support for new flash controller
 *	23-Mar-2010  YH	      Work with NAND 1.2.1 or above
 *	09-Jun-2009  YH       Add Xenv config
 *	12-May-2009  BJD      Initial version
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/ctype.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>
#include <linux/dma-mapping.h>

#include <asm/io.h>

#if defined(CONFIG_TANGO3)
#include <asm/tango3/rmdefs.h>
#include <asm/tango3/tango3.h>
#include <asm/tango3/tango3api.h>
#elif defined(CONFIG_TANGO4)
#include <asm/tango4/rmdefs.h>
#include <asm/tango4/tango4.h>
#include <asm/tango4/tango4api.h>
#endif

/**
	NAND access register
*/
#define SMP8XXX_REG_CMD				0
#define SMP8XXX_REG_ADDR			4
#define SMP8XXX_REG_DATA			8

#define tReset					5 //us

#define PB_IORDY	0x80000000

/* Enabling AUTOPIO operations */
#define USE_AUTOPIO
/* Enabling CTRLER specific IRQ */
#define USE_CTRLER_IRQ

static int new_ctrler = 1; /* use new controller as default */
static int max_page_shift = 13;	/* up to 8KB page */

/* Definitions used by new controller */
#define MAXPAGES 16
/**
	New NAND interface 
 */
#define MLC_CHA_REG 			0x4400
#define MLC_CHB_REG 			0x4440
#define MLC_CHA_MEM 			0x4800
#define MLC_CHB_MEM 			0x4a00
#define MLC_BADBLOCK_OFFSET		0x100
#define MLC_ECCREPORT_OFFSET 		0x1c0

/* channnel A/B are tied to CS0/1 */
static const unsigned int chx_reg[2] = { MLC_CHA_REG, MLC_CHB_REG };
static const unsigned int chx_mem[2] = { MLC_CHA_MEM, MLC_CHB_MEM };
static const unsigned int sbox_tgt[2] = { SBOX_PCIMASTER, SBOX_PCISLAVE };

#define STATUS_REG(b)	((b) + 0x0)
#define FLASH_CMD(b)	((b) + 0x4)
#define DEVICE_CFG(b)	((b) + 0x8)
#define TIMING1(b)	((b) + 0xc)
#define TIMING2(b)	((b) + 0x10)
#define XFER_CFG(b)	((b) + 0x14)
#define PACKET_0_CFG(b)	((b) + 0x18)
#define PACKET_N_CFG(b)	((b) + 0x1c)
#define BAD_BLOCK_CFG(b)	((b) + 0x20)
#define ADD_PAGE(b)		((b) + 0x24)
#define ADD_OFFSET(b)		((b) + 0x28)

/**
	PB Chip Select 
*/
#define	MAX_NO_CS			8

/**
	gbus access micro
*/
#define RD_HOST_REG32(r)	\
		gbus_read_reg32(REG_BASE_host_interface + (r))

#define WR_HOST_REG32(r, v)	\
		gbus_write_reg32(REG_BASE_host_interface + (r), (v))

#define RD_HOST_REG16(r)	\
		gbus_read_reg16(REG_BASE_host_interface + (r))

#define WR_HOST_REG16(r, v) \
		gbus_write_reg16(REG_BASE_host_interface + (r), (v))

#define RD_HOST_REG8(r)	\
		gbus_read_reg8(REG_BASE_host_interface + (r))

#define WR_HOST_REG8(r, v)	\
		gbus_write_reg8(REG_BASE_host_interface + (r), (v))

#define SMP_NAND_DEV_NAME	"[SMP_NAND]"
#define SMP_NAND_DRV_VERSION	"0.3"

#define MAX_CS		8	/* Maximum number of CS */
#define MAX_PARTITIONS	16	/* Maximum partitions per CS */
#define MAX_NAND_DEVS	8	/* Maximum number of NAND devices, needs to be in sync with the gap in nand_ids.c */

/* XENV keys to be used */
#define CS_RESERVED	"a.cs%d_rsvd_pblk"
#define CS_PARTS	"a.cs%d_pblk_parts"
#define CS_PART_SIZE	"a.cs%d_pblk_part%d_size"
#define CS_PART_OFFSET	"a.cs%d_pblk_part%d_offset" 
#define CS_PART_NAME	"a.cs%d_pblk_part%d_name" 
#define CS_TIMING1	"a.cs%d_nand_timing1"
#define CS_TIMING2	"a.cs%d_nand_timing2"
#define CS_DEVCFG	"a.cs%d_nand_devcfg"
#define NAND_PARAM	"a.nandpart%d_params"
#define HIGH_32		"_hi"

#define BUFSIZE		256

/* Prototype of routine that gets XENV and others .. */
extern int zxenv_get(char *recordname, void *dst, u32 *datasize);
extern unsigned long tangox_virt_to_phys(void *pvaddr);

/* Internal data structure */
static struct mtd_info smp8xxx_mtds[MAX_CS];
static struct nand_chip smp8xxx_chips[MAX_CS];
static struct mtd_partition *smp8xxx_partitions[MAX_CS];
static struct nand_hw_control smp8xxx_hw_control;
static int cs_avail[MAX_CS], cs_parts[MAX_CS];
static int cs_offset;
static unsigned long chip_szs[MAX_CS] = { 0, };
static int max_chips = MAX_CS;
module_param_array(chip_szs, ulong, &max_chips, 0000);
MODULE_PARM_DESC(chip_szs, "Overridden value of chip sizes");

struct chip_private
{
	unsigned int cs;	/* chip select */
	uint8_t *bbuf;		/* bounce buffer */
};

static struct chip_private chip_privs[MAX_CS];

/* OOB layout for devices on new controller */
/* for 512B page, typically with 16B OOB, and we use 4bit ECC */
static struct nand_ecclayout smp8xxx_nand_ecclayout512_16_4 = { // may not be supported
	.eccbytes = 7,
	.eccpos = {6, 7, 8, 9, 10, 11, 12},
	.oobfree = {
		{.offset = 13, .length = 3}
	},
};

/* for 2KB page, typically with 64B OOB, and we use 8bit ECC */
static struct nand_ecclayout smp8xxx_nand_ecclayout2048_64_8 = {
	.eccbytes = 13,
	.eccpos = {49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61},
	.oobfree = {
		{.offset = 62, .length = 2}
	},
};

/* for 4KB page, typically with 128B OOB, and we use 9bit ECC */
static struct nand_ecclayout smp8xxx_nand_ecclayout4096_128_9 = {
	.eccbytes = 15,
	.eccpos = {110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124},
	.oobfree = {
		{.offset = 125, .length = 3}
	},
};

/* for 4KB page, typically with 218B OOB or above, and we use 16bit ECC */
static struct nand_ecclayout smp8xxx_nand_ecclayout4096_218_16 = {
	.eccbytes = 26,
	.eccpos = {187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212},
	.oobfree = {
		{.offset = 213, .length = 5}
	},
};

/* for 8KB page, typically with 448 OOB, and we use 16bit ECC */
static struct nand_ecclayout smp8xxx_nand_ecclayout8192_448_16 = { 
	.eccbytes = 26,
	.eccpos = {395, 396, 397, 398, 399, 400, 401, 402, 403, 404, 405 ,406, 407, 408, 409, 410, 411, 412, 413, 414, 415, 416, 417, 418, 419, 420},
	.oobfree = {
		{.offset = 421, .length = 27}
	}
};

/* OOB layout for devices on old controller */
/* for 512B page, typically with 16B OOB */
static struct nand_ecclayout smp8xxx_oobinfo_16 = {
	.eccbytes = 3,
	.eccpos = {10, 11, 12},
	.oobfree = {
		{.offset = 6, .length = 4},
	},
};

/* for 2KB page, typically with 64B OOB */
static struct nand_ecclayout smp8xxx_oobinfo_64 = {
	.eccbytes = 12,
	.eccpos = {10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21},
	.oobfree = {
		{.offset = 6, .length = 4},
		{.offset = 22, .length = 38},
	},
};

/*
 * MTD structure for NAND controller
 */
static const char *smp_nand_devname = SMP_NAND_DEV_NAME;

/**
 * smp8xxx_read_byte -  read one byte from the chip
 * @mtd:	MTD device structure
 *
 *  read function for 8bit buswidth
 */
static u_char smp8xxx_read_byte(struct mtd_info *mtd)
{
	struct nand_chip *this = mtd->priv;
	return RD_HOST_REG8((RMuint32)this->IO_ADDR_R + SMP8XXX_REG_DATA);
}

#ifdef USE_AUTOPIO
static int mbus_done = 0;
static DECLARE_WAIT_QUEUE_HEAD(mbus_wq);
static void pbi_mbus_intr(int irq, void *arg)
{
	mbus_done = 1;
	wake_up_interruptible(&mbus_wq);
}
#endif

/**
 * smp8xxx_write_buf -  write buffer to chip
 * @mtd:	MTD device structure
 * @buf:	data buffer
 * @len:	number of bytes to write
 *
 *  write function for 8bit buswidth
 */
static void smp8xxx_write_buf(struct mtd_info *mtd, const u_char *buf, int len)
{
	int i;
	struct nand_chip *this = (struct nand_chip *)mtd->priv;

	//printk("%s smp8xxx_write_buf buf = %p, len = 0x%08x(%d)\n", smp_nand_devname, buf, len, len);

#ifdef USE_AUTOPIO
	unsigned int cs = ((struct chip_private *)this->priv)->cs;
	unsigned long g_mbus_reg = 0;
	dma_addr_t dma_addr;

	if ((in_atomic()) || (len <= mtd->oobsize))
		goto pio;
	else if ((((u32)buf) < KSEG0) || (((u32)buf) >= KSEG2))
		goto pio;
	else if (em86xx_mbus_alloc_dma(SBOX_IDEFLASH, 0, &g_mbus_reg, NULL, 0))
		goto pio;

	dma_addr = dma_map_single(NULL, (void *)buf, len, DMA_TO_DEVICE);

	gbus_write_reg32(REG_BASE_host_interface + PB_automode_control + 4, 0);
	gbus_write_reg32(REG_BASE_host_interface + PB_automode_start_address, SMP8XXX_REG_DATA);
	/* 22:nand 17:8bit width 16:DRAM to PB len:number of PB accesses */
	gbus_write_reg32(REG_BASE_host_interface + PB_automode_control, (cs << 24) | (2 << 22) | (1 << 17) | (0 << 16) | len);
	
	em86xx_mbus_setup_dma(g_mbus_reg, dma_addr, len, pbi_mbus_intr, NULL, 1);

	wait_event_interruptible(mbus_wq, mbus_done != 0);
	while (gbus_read_reg32(REG_BASE_host_interface + PB_automode_control) & 0xffff)
		; /* wait for AUTOPIO completion */
	mbus_done = 0;
	
	em86xx_mbus_free_dma(g_mbus_reg, SBOX_IDEFLASH);
	dma_unmap_single(NULL, dma_addr, len, DMA_TO_DEVICE);

	goto done;

pio:
#endif
	for (i = 0; i < len; i++) 
		WR_HOST_REG8((RMuint32)this->IO_ADDR_W + SMP8XXX_REG_DATA, buf[i]);

#ifdef USE_AUTOPIO
done:
#endif
	return;
}

/**
 * smp8xxx_read_buf -  read chip data into buffer
 * @mtd:	MTD device structure
 * @buf:	buffer to store data
 * @len:	number of bytes to read
 *
 *  read function for 8bit buswith
 */
static void smp8xxx_read_buf(struct mtd_info *mtd, u_char *buf, int len)
{
	int i;
	struct nand_chip *this = mtd->priv;

#ifdef USE_AUTOPIO
	unsigned int cs = ((struct chip_private *)this->priv)->cs;
	unsigned long g_mbus_reg = 0;
	dma_addr_t dma_addr;

	if ((in_atomic()) || (len <= mtd->oobsize))
		goto pio;
	else if ((((u32)buf) < KSEG0) || (((u32)buf) >= KSEG2))
		goto pio;
	else if (em86xx_mbus_alloc_dma(SBOX_IDEFLASH, 1, &g_mbus_reg, NULL, 0))
		goto pio;

	dma_addr = dma_map_single(NULL, (void *)buf, len, DMA_FROM_DEVICE);

	gbus_write_reg32(REG_BASE_host_interface + PB_automode_control + 4, 0);
	gbus_write_reg32(REG_BASE_host_interface + PB_automode_start_address, SMP8XXX_REG_DATA);
	/* 22:nand 17:8bit width 16:DRAM to PB len:number of PB accesses */
	gbus_write_reg32(REG_BASE_host_interface + PB_automode_control, (cs << 24) | (2 << 22) | (1 << 17) | (1 << 16) | len);

	em86xx_mbus_setup_dma(g_mbus_reg, dma_addr, len, pbi_mbus_intr, NULL, 1);

	wait_event_interruptible(mbus_wq, mbus_done != 0);
	while (gbus_read_reg32(REG_BASE_host_interface + PB_automode_control) & 0xffff)
		; /* wait for AUTOPIO completion */
	mbus_done = 0;

	em86xx_mbus_free_dma(g_mbus_reg, SBOX_IDEFLASH);
	dma_unmap_single(NULL, dma_addr, len, DMA_FROM_DEVICE);

	goto done;
pio:
#endif
	for (i = 0; i < len; i++) 
		buf[i] = RD_HOST_REG8((RMuint32)this->IO_ADDR_R + SMP8XXX_REG_DATA);

#ifdef USE_AUTOPIO
done:
#endif
	return;
}

static void smp8xxx_nand_bug(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	struct mtd_ecc_stats ecc_stats = mtd->ecc_stats;
	unsigned int cs = ((struct chip_private *)chip->priv)->cs;
	printk("%s: cs = %d, options = 0x%x\n", smp_nand_devname, cs, chip->options);
	printk("%s: ecc stats: corrected = %d, failed = %d, badblocks = %d, bbtblocks = %d\n",
		smp_nand_devname, ecc_stats.corrected, ecc_stats.failed, ecc_stats.badblocks, ecc_stats.bbtblocks);
	printk("%s: to be resolved, here's the call-stack: \n", smp_nand_devname);
	dump_stack();
}

static int smp8xxx_nand_bug_calculate(struct mtd_info *mtd, const uint8_t *dat, uint8_t *ecc_code)
{
	smp8xxx_nand_bug(mtd);
	return 0;	/* should have no need to calculate */
}

static int smp8xxx_nand_bug_correct(struct mtd_info *mtd, uint8_t *dat, uint8_t *read_ecc, uint8_t *calc_ecc)
{
	smp8xxx_nand_bug(mtd);
	return 0;	/* should have no need to correct */
}

static void smp8xxx_nand_hwctl(struct mtd_info *mtd, int mode)
{
	register struct nand_chip *chip = mtd->priv;
	unsigned int cs = ((struct chip_private *)chip->priv)->cs;
	while ((RD_HOST_REG32(STATUS_REG(chx_reg[cs])) & 0x80000000) == 0)
		; /* unlikely it's not ready */
}

static void nand_command(struct mtd_info *mtd, unsigned int command, int column, int page_addr)
{
	register struct nand_chip *chip = mtd->priv;
	int ctrl = NAND_CTRL_CLE | NAND_CTRL_CHANGE;

	/*
	 * Write out the command to the device.
	 */
	if (command == NAND_CMD_SEQIN) {
		int readcmd;
		if (column >= mtd->writesize) {
			/* OOB area */
			column -= mtd->writesize;
			readcmd = NAND_CMD_READOOB;
		} else if (column < 256) {
			/* First 256 bytes --> READ0 */
			readcmd = NAND_CMD_READ0;
		} else {
			column -= 256;
			readcmd = NAND_CMD_READ1;
		}
		chip->cmd_ctrl(mtd, readcmd, ctrl);
		ctrl &= ~NAND_CTRL_CHANGE;
	}
	chip->cmd_ctrl(mtd, command, ctrl);

	/*
	 * Address cycle, when necessary
	 */
	ctrl = NAND_CTRL_ALE | NAND_CTRL_CHANGE;
	/* Serially input address */
	if (column != -1) {
		/* Adjust columns for 16 bit buswidth */
		if (chip->options & NAND_BUSWIDTH_16)
			column >>= 1;
		chip->cmd_ctrl(mtd, column, ctrl);
		ctrl &= ~NAND_CTRL_CHANGE;
	}
	if (page_addr != -1) {
		chip->cmd_ctrl(mtd, page_addr, ctrl);
		ctrl &= ~NAND_CTRL_CHANGE;
		chip->cmd_ctrl(mtd, page_addr >> 8, ctrl);
		/* One more address cycle for devices > 32MiB */
		if (chip->chipsize > (32 << 20))
			chip->cmd_ctrl(mtd, page_addr >> 16, ctrl);
	}
	chip->cmd_ctrl(mtd, NAND_CMD_NONE, NAND_NCE | NAND_CTRL_CHANGE);

	/*
	 * program and erase have their own busy handlers
	 * status and sequential in needs no delay
	 */
	switch (command) {

	case NAND_CMD_PAGEPROG:
	case NAND_CMD_ERASE1:
	case NAND_CMD_ERASE2:
	case NAND_CMD_SEQIN:
	case NAND_CMD_STATUS:
		return;

	case NAND_CMD_RESET:
		if (chip->dev_ready)
			break;
		udelay(chip->chip_delay);
		chip->cmd_ctrl(mtd, NAND_CMD_STATUS, NAND_CTRL_CLE | NAND_CTRL_CHANGE);
		chip->cmd_ctrl(mtd, NAND_CMD_NONE, NAND_NCE | NAND_CTRL_CHANGE);
		while (!(chip->read_byte(mtd) & NAND_STATUS_READY)) ;
		return;

		/* This applies to read commands */
	default:
		/*
		 * If we don't have access to the busy pin, we apply the given
		 * command delay
		 */
		if (!chip->dev_ready) {
			udelay(chip->chip_delay);
			return;
		}
	}
	/* Apply this short delay always to ensure that we do wait tWB in
	 * any case on any machine. */
	ndelay(100);

	nand_wait_ready(mtd);
}

static void nand_command_lp(struct mtd_info *mtd, unsigned int command, int column, int page_addr)
{
	register struct nand_chip *chip = mtd->priv;

	/* Emulate NAND_CMD_READOOB */
	if (command == NAND_CMD_READOOB) {
		column += mtd->writesize;
		command = NAND_CMD_READ0;
	}

	/* Command latch cycle */
	chip->cmd_ctrl(mtd, command & 0xff, NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);

	if ((column != -1) || (page_addr != -1)) {
		int ctrl = NAND_CTRL_CHANGE | NAND_NCE | NAND_ALE;

		/* Serially input address */
		if (column != -1) {
			/* Adjust columns for 16 bit buswidth */
			if (chip->options & NAND_BUSWIDTH_16)
				column >>= 1;
			chip->cmd_ctrl(mtd, column, ctrl);
			ctrl &= ~NAND_CTRL_CHANGE;
			chip->cmd_ctrl(mtd, column >> 8, ctrl);
		}
		if (page_addr != -1) {
			chip->cmd_ctrl(mtd, page_addr, ctrl);
			chip->cmd_ctrl(mtd, page_addr >> 8, NAND_NCE | NAND_ALE);
			/* One more address cycle for devices > 128MiB */
			if (chip->chipsize > (128 << 20))
				chip->cmd_ctrl(mtd, page_addr >> 16, NAND_NCE | NAND_ALE);
		}
	}
	chip->cmd_ctrl(mtd, NAND_CMD_NONE, NAND_NCE | NAND_CTRL_CHANGE);

	/*
	 * program and erase have their own busy handlers
	 * status, sequential in, and deplete1 need no delay
	 */
	switch (command) {

	case NAND_CMD_CACHEDPROG:
	case NAND_CMD_PAGEPROG:
	case NAND_CMD_ERASE1:
	case NAND_CMD_ERASE2:
	case NAND_CMD_SEQIN:
	case NAND_CMD_RNDIN:
	case NAND_CMD_STATUS:
	case NAND_CMD_DEPLETE1:
		return;

	/*
	 * read error status commands require only a short delay
	 */
	case NAND_CMD_STATUS_ERROR:
	case NAND_CMD_STATUS_ERROR0:
	case NAND_CMD_STATUS_ERROR1:
	case NAND_CMD_STATUS_ERROR2:
	case NAND_CMD_STATUS_ERROR3:
		udelay(chip->chip_delay);
		return;

	case NAND_CMD_RESET:
		if (chip->dev_ready)
			break;
		udelay(chip->chip_delay);
		chip->cmd_ctrl(mtd, NAND_CMD_STATUS, NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);
		chip->cmd_ctrl(mtd, NAND_CMD_NONE, NAND_NCE | NAND_CTRL_CHANGE);
		while (!(chip->read_byte(mtd) & NAND_STATUS_READY))
			;
		return;

	case NAND_CMD_RNDOUT:
		/* No ready / busy check necessary */
		chip->cmd_ctrl(mtd, NAND_CMD_RNDOUTSTART, NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);
		chip->cmd_ctrl(mtd, NAND_CMD_NONE, NAND_NCE | NAND_CTRL_CHANGE);
		return;

	case NAND_CMD_READ0:
		chip->cmd_ctrl(mtd, NAND_CMD_READSTART, NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);
		chip->cmd_ctrl(mtd, NAND_CMD_NONE, NAND_NCE | NAND_CTRL_CHANGE);

		/* This applies to read commands */
	default:
		/*
		 * If we don't have access to the busy pin, we apply the given
		 * command delay
		 */
		if (!chip->dev_ready) {
			udelay(chip->chip_delay);
			return;
		}
	}

	/* Apply this short delay always to ensure that we do wait tWB in
	 * any case on any machine. */
	ndelay(100);

	nand_wait_ready(mtd);
}

/**
 * smp8xxx_command - Send command to NAND device
 * @mtd:	MTD device structure
 * @command:	the command to be sent
 * @column:	the column address for this command, -1 if none
 * @page_addr:	the page address for this command, -1 if none
 *
 * Send command to NAND device. This function is used for small and large page
 * devices (256/512/2K/4K/8K Bytes per page)
 */
static void smp8xxx_command(struct mtd_info *mtd, unsigned int command, int column, int page_addr)
{
	register struct nand_chip *chip = mtd->priv;
	unsigned int cs = ((struct chip_private *)chip->priv)->cs;

	switch(mtd->writesize) {
	case 512:
		/* 512B writesize may not be supported by current nand filesystems */
		nand_command(mtd, command, column, page_addr);
		break;
	case 2048:
	case 4096:
	case 8192:
		nand_command_lp(mtd, command, column, page_addr);
		break;
	default: /* very unlikely */
		smp8xxx_nand_bug(mtd);
		break;
	}
	
	/* set New Controller address */
	WR_HOST_REG32(ADD_PAGE(chx_reg[cs]), page_addr); // page address
	WR_HOST_REG32(ADD_OFFSET(chx_reg[cs]), 0);	 // offset always 0
}

/*
 * Function for register configurations
 */
static int smp8xxx_packet_config(struct mtd_info *mtd, struct nand_chip *chip, int len)
{
	unsigned int cs = ((struct chip_private *)chip->priv)->cs;
	unsigned int csel = cs << 24;
	int ret = -1;

	switch (len) 
	{
	case 512:	/* 4 bit ECC per 512B packet */
		// May not be supported by current nand filesystems 
		WR_HOST_REG32(XFER_CFG(chx_reg[cs]), 0x10010104 | csel);//xfer
		WR_HOST_REG32(PACKET_0_CFG(chx_reg[cs]), 0x02040004);	//packet_0
		WR_HOST_REG32(BAD_BLOCK_CFG(chx_reg[cs]), 0x02040002);	//bad_block
		ret = 0;
		break;

	case 2048:	/* 8 bit ECC per 512 packet */
		WR_HOST_REG32(XFER_CFG(chx_reg[cs]), 0x00010404 | csel);//xfer
		WR_HOST_REG32(PACKET_0_CFG(chx_reg[cs]), 0x02040008);	//packet_0
		WR_HOST_REG32(PACKET_N_CFG(chx_reg[cs]), 0x02000008);	//packet_N
		WR_HOST_REG32(BAD_BLOCK_CFG(chx_reg[cs]), 0x08000006);	//bad_block
		ret = 0;
		break;

	case 4096:	/* 9 or 16 bit ECC per 512 packet depends on OOB size */
		if ((mtd->oobsize >= 128) && (mtd->oobsize < 218)) {
			WR_HOST_REG32(XFER_CFG(chx_reg[cs]), 0x00010804 | csel);//xfer
			WR_HOST_REG32(PACKET_0_CFG(chx_reg[cs]), 0x02040009);	//packet_0
			WR_HOST_REG32(PACKET_N_CFG(chx_reg[cs]), 0x02000009);	//packet_N
			WR_HOST_REG32(BAD_BLOCK_CFG(chx_reg[cs]), 0x10000001);	//bad_block
			ret = 0;
		} else if (mtd->oobsize >= 218) {
			WR_HOST_REG32(XFER_CFG(chx_reg[cs]), 0x00010804 | csel);//xfer
			WR_HOST_REG32(PACKET_0_CFG(chx_reg[cs]), 0x02040010);	//packet_0
			WR_HOST_REG32(PACKET_N_CFG(chx_reg[cs]), 0x02000010);	//packet_N
			WR_HOST_REG32(BAD_BLOCK_CFG(chx_reg[cs]), 0x10000001);	//bad_block
			ret = 0;
		}
		break;

	case 8192:	/* 16 bit ECC per 512 packet */
		//packet number 16 exceeds amount of reserved bits -
		//spec is wrong and is being changed ...
		WR_HOST_REG32(XFER_CFG(chx_reg[cs]), 0x00011004 | csel);//xfer
		WR_HOST_REG32(PACKET_0_CFG(chx_reg[cs]), 0x02040010);	//packet_0
		WR_HOST_REG32(PACKET_N_CFG(chx_reg[cs]), 0x02000010);	//packet_N
		WR_HOST_REG32(BAD_BLOCK_CFG(chx_reg[cs]), 0x20000001);	//bad_block
		ret = 0;
		break;
	}

	if (ret != 0)
		printk("%s: unsupported Packet Config on CS%d.\n", SMP_NAND_DEV_NAME, cs);

	return ret;
}

/* Check ECC correction validity
 * Return 0 if not valid */
static int smp8xxx_validecc(struct mtd_info *mtd)
{
	struct nand_chip *chip = (struct nand_chip *)mtd->priv;
	unsigned int cs = ((struct chip_private *)chip->priv)->cs, len = mtd->writesize;
	unsigned int code = RD_HOST_REG32(chx_mem[cs] + MLC_ECCREPORT_OFFSET) & ((len == 512) ? 0x00ff : 0xffff), mask = ((len == 512) ? 0x80 : 0x8080);

	if (((code & mask) != mask) && (code != 0)) { /* (code == 0) is most likey blank page */
		if (printk_ratelimit())
			printk(KERN_WARNING "%s: ecc error detected (code=0x%x)\n", smp_nand_devname, code);
		return 0;
	} 
	return 1;
}

#ifdef USE_CTRLER_IRQ
static int chx_mbus_done[2] = { 0, 0 }; 
static DECLARE_WAIT_QUEUE_HEAD(cha_mbus_wq);
static DECLARE_WAIT_QUEUE_HEAD(chb_mbus_wq);
static wait_queue_head_t *wqueues[2] = { &cha_mbus_wq, &chb_mbus_wq };
static void cha_mbus_intr(int irq, void *arg)
{
	chx_mbus_done[0] = 1;
	wake_up_interruptible(wqueues[0]);
}
static void chb_mbus_intr(int irq, void *arg)
{
	chx_mbus_done[1] = 1;
	wake_up_interruptible(wqueues[1]);
}
typedef void (*CALLBACK_PTR)(int, void *);
static CALLBACK_PTR callbacks[2] = { cha_mbus_intr, chb_mbus_intr };
#endif

/**
 * smp8xxx_read_page_hwecc - hardware ecc based page write function
 * @mtd:	MTD device info structure
 * @chip:	nand chip info structure
 * @buf:	buffer to store read data
 * @page:	page number
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
static int smp8xxx_read_page_hwecc(struct mtd_info *mtd, struct nand_chip *chip, uint8_t *buffer)
#else
static int smp8xxx_read_page_hwecc(struct mtd_info *mtd, struct nand_chip *chip, uint8_t *buffer, int page)
#endif
{
	unsigned int cs = ((struct chip_private *)chip->priv)->cs;
	int len = mtd->writesize;
	unsigned long g_mbus_reg = 0;
	uint8_t *buf = buffer;
	uint8_t *bbuf = ((struct chip_private *)chip->priv)->bbuf;
	dma_addr_t dma_addr;

	if ((in_atomic()) || (len <= mtd->oobsize))
		return -EIO;
	else if (smp8xxx_packet_config(mtd, chip, len) != 0) 
		return -EIO;

	if ((((u32)buf) < KSEG0) || (((u32)buf) >= KSEG2))
		buf = bbuf;	/* use bounce buffer */

	// Channel A used in soft for CS0 channel B used in soft for CS1
	if (em86xx_mbus_alloc_dma(sbox_tgt[cs], 1, &g_mbus_reg, NULL, 1) < 0)
		return -EIO;

	dma_addr = dma_map_single(NULL, (void *)buf, len, DMA_FROM_DEVICE);

	// poll ready status
	while ((RD_HOST_REG32(STATUS_REG(chx_reg[cs])) & 0x80000000) == 0)
		; /* unlikely it's not ready */

	// launch New Controller read command
	WR_HOST_REG32(FLASH_CMD(chx_reg[cs]), 0x1);

#ifdef USE_CTRLER_IRQ
	em86xx_mbus_setup_dma(g_mbus_reg, dma_addr, len, callbacks[cs], NULL, 1);
	wait_event_interruptible(*wqueues[cs], chx_mbus_done[cs] != 0);
#else
	em86xx_mbus_setup_dma(g_mbus_reg, dma_addr, len, NULL, NULL, 1);
#endif

	// poll ready status
	while ((RD_HOST_REG32(STATUS_REG(chx_reg[cs])) & 0x80000000) == 0)
		; /* wait for completion */
#ifdef USE_CTRLER_IRQ
	chx_mbus_done[cs] = 0;
#endif

	em86xx_mbus_free_dma(g_mbus_reg, sbox_tgt[cs]);
	dma_unmap_single(NULL, dma_addr, len, DMA_FROM_DEVICE);

	if (buf == bbuf) {
		/* copy back */
		memcpy(buffer, buf, len);
	}

	if (!smp8xxx_validecc(mtd)) {
		mtd->ecc_stats.failed++;
		return 1;
	}
	
	return 0;
}

/**
 * smp8xxx_write_page_hwecc - hardware ecc based page write function
 * @mtd:	MTD device info structure
 * @chip:	nand chip info structure
 * @buf:	data buffer
 */
static void smp8xxx_write_page_hwecc(struct mtd_info *mtd, struct nand_chip *chip, const uint8_t *buffer)
{
	unsigned int cs = ((struct chip_private *)chip->priv)->cs;
	int len = mtd->writesize;
	unsigned long g_mbus_reg = 0;
	uint8_t *buf = (uint8_t *)buffer;
	uint8_t *bbuf = ((struct chip_private *)chip->priv)->bbuf;
	dma_addr_t dma_addr;

	if ((in_atomic()) || (len <= mtd->oobsize)) {
		smp8xxx_nand_bug(mtd);
		return /* TODO: -EIO? */;
	} else if (smp8xxx_packet_config(mtd, chip, len) != 0) {
		smp8xxx_nand_bug(mtd);
		return /* TODO: -EIO? */;
	}

	if ((((u32)buf) < KSEG0) || (((u32)buf) >= KSEG2)) {
		buf = bbuf;	/* use bounce buffer */
		memcpy(buf, buffer, len);
	}

	// Channel A used in soft for CS0 channel B used in soft for CS1
	if (em86xx_mbus_alloc_dma(sbox_tgt[cs], 0, &g_mbus_reg, NULL, 1) < 0) {
		smp8xxx_nand_bug(mtd);
		return /* TODO: -EIO? */;
	}

	dma_addr = dma_map_single(NULL, (void *)buf, len, DMA_TO_DEVICE);

	// poll ready status
	while ((RD_HOST_REG32(STATUS_REG(chx_reg[cs])) & 0x80000000) == 0)
		; /* unlikely it's not ready */

	// launch New Controller write command
	WR_HOST_REG32(FLASH_CMD(chx_reg[cs]), 0x2);

#ifdef USE_CTRLER_IRQ
	em86xx_mbus_setup_dma(g_mbus_reg, dma_addr, len, callbacks[cs], NULL, 1);
	wait_event_interruptible(*wqueues[cs], chx_mbus_done[cs] != 0);
#else
	em86xx_mbus_setup_dma(g_mbus_reg, dma_addr, len, NULL, NULL, 1);
#endif
	// poll ready status
	while ((RD_HOST_REG32(STATUS_REG(chx_reg[cs])) & 0x80000000) == 0)
		; /* wait for completion */
#ifdef USE_CTRLER_IRQ
	chx_mbus_done[cs] = 0;
#endif

	em86xx_mbus_free_dma(g_mbus_reg, sbox_tgt[cs]);
	dma_unmap_single(NULL, dma_addr, len, DMA_TO_DEVICE);

	return;
}

/**
 * smp8xxx_verify_buf -  Verify chip data against buffer
 * @mtd:	MTD device structure
 * @buf:	buffer containing the data to compare
 * @len:	number of bytes to compare
 *
 *  verify function for 8bit buswith
 */
static int smp8xxx_verify_buf(struct mtd_info *mtd, const u_char *buf, int len)
{
#ifdef USE_AUTOPIO
	u_char *tmpbuf = kmalloc(2048, GFP_KERNEL | GFP_DMA);	/* up to 2KB */
	int ret;
	if (tmpbuf == NULL)
		return -ENOMEM;
	smp8xxx_read_buf(mtd, tmpbuf, len);
	ret = (memcmp(buf, tmpbuf, len) == 0) ? 0 : -EIO;
	kfree(tmpbuf);
	return ret;
#else
	int i;
	struct nand_chip *this = mtd->priv;
	for (i = 0; i < len; i++) {
		if (buf[i] != RD_HOST_REG8((RMuint32)this->IO_ADDR_R + SMP8XXX_REG_DATA))
			return -EIO;
	}
	return 0;
#endif
}

/* smp8xxx_nand_hwcontrol
 *
 * Issue command and address cycles to the chip
 */
static void smp8xxx_nand_hwcontrol(struct mtd_info *mtd, int cmd, unsigned int ctrl)
{
	register struct nand_chip *this = mtd->priv;

	if (cmd == NAND_CMD_NONE)
		return;

	if (ctrl & NAND_CLE)
		WR_HOST_REG8((RMuint32)this->IO_ADDR_W + SMP8XXX_REG_CMD, cmd);
	else
		WR_HOST_REG8((RMuint32)this->IO_ADDR_W + SMP8XXX_REG_ADDR, cmd);
}

/* smp8xxx_nand_devready()
 *
 * returns 0 if the nand is busy, 1 if it is ready
 */
static int smp8xxx_nand_devready(struct mtd_info *mtd)
{
	if (new_ctrler) {
		register struct nand_chip *chip = mtd->priv;
		unsigned int cs = ((struct chip_private *)chip->priv)->cs;
		return ((RD_HOST_REG32(PB_CS_ctrl) & PB_IORDY) != 0) && 
			((RD_HOST_REG32(STATUS_REG(chx_reg[cs])) & 0x80000000) != 0);
	} else 
		return (RD_HOST_REG32(PB_CS_ctrl) & PB_IORDY) != 0;
}

/* ECC handling functions */
static inline int mu_count_bits(u32 v)
{
	int i, count;
	for (count = i = 0; (i < 32) && (v != 0); i++, v >>= 1)
		count += (v & 1);
	return count;
}

/* correct 512B packet */
static int ecc_correct_512(struct mtd_info *mtd, u_char *dat, u_char *read_ecc, u_char *calc_ecc)
{
	u32 mem, reg;
	mem = read_ecc[0] | ((read_ecc[1] & 0x0f) << 8) | ((read_ecc[1] & 0xf0) << 12) | (read_ecc[2] << 20);
	reg = calc_ecc[0] | ((calc_ecc[1] & 0x0f) << 8) | ((calc_ecc[1] & 0xf0) << 12) | (calc_ecc[2] << 20);
	if (likely(mem == reg))
		return 0;
	else {
		u16 pe, po, is_ecc_ff;
		is_ecc_ff = ((mem & 0x0fff0fff) == 0x0fff0fff);
		mem ^= reg;

		switch(mu_count_bits(mem)) {
			case 0:
				return 0;
			case 1:
				return -1;
			case 12:
				po = (u16)(mem & 0xffff);
				pe = (u16)((mem >> 16) & 0xffff);
				po = pe ^ po;
				if (po == 0x0fff) {
					dat[pe >> 3] ^= (1 << (pe & 7));
					return 1;	/* corrected data */
				} else 
					return -1;	/* failed to correct */
			default:
				return (is_ecc_ff && (reg == 0)) ? 0 : -1;
		}
	}
	return -1;	/* should not be here */
}

/* correct 512B * 4 packets */
static int ecc_correct_2048(struct mtd_info *mtd, u_char *dat, u_char *read_ecc, u_char *calc_ecc)
{
	int ret0, ret1, ret2, ret3;
	
	ret0 = ecc_correct_512(mtd, dat, read_ecc, calc_ecc);
	ret1 = ecc_correct_512(mtd, dat + 512, read_ecc + 3, calc_ecc + 3);
	ret2 = ecc_correct_512(mtd, dat + 1024, read_ecc + 6, calc_ecc + 6);
	ret3 = ecc_correct_512(mtd, dat + 1536, read_ecc + 9, calc_ecc + 9);

	if ((ret0 < 0) || (ret1 < 0) || (ret2 << 0) || (ret3 << 0))
		return -1;
	else
		return ret0 + ret1 + ret2 + ret3;
}

/* ECC functions
 *
 * These allow the smp8xxx to use the controller's ECC
 * generator block to ECC the data as it passes through]
*/
static int smp8xxx_nand_calculate_ecc_512(struct mtd_info *mtd, const u_char *dat, u_char *ecc_code)
{
	ecc_code[0] = RD_HOST_REG8(PB_ECC_code0 + 0);
	ecc_code[1] = RD_HOST_REG8(PB_ECC_code0 + 1);
	ecc_code[2] = RD_HOST_REG8(PB_ECC_code0 + 2);

	pr_debug("%s: returning ecc %02x%02x%02x\n", __func__,
		 ecc_code[0], ecc_code[1], ecc_code[2]);

	return 0;
}

static int smp8xxx_nand_calculate_ecc_2048(struct mtd_info *mtd, const u_char *dat, u_char *ecc_code)
{
	ecc_code[0] = RD_HOST_REG8(PB_ECC_code0 + 0);
	ecc_code[1] = RD_HOST_REG8(PB_ECC_code0 + 1);
	ecc_code[2] = RD_HOST_REG8(PB_ECC_code0 + 2);
	pr_debug("%s: returning ecc %02x%02x%02x", __func__, ecc_code[0], ecc_code[1], ecc_code[2]);

	ecc_code[3] = RD_HOST_REG8(PB_ECC_code1 + 0);
	ecc_code[4] = RD_HOST_REG8(PB_ECC_code1 + 1);
	ecc_code[5] = RD_HOST_REG8(PB_ECC_code1 + 2);
	pr_debug("%02x%02x%02x", ecc_code[3], ecc_code[4], ecc_code[5]);

	ecc_code[6] = RD_HOST_REG8(PB_ECC_code2 + 0);
	ecc_code[7] = RD_HOST_REG8(PB_ECC_code2 + 1);
	ecc_code[8] = RD_HOST_REG8(PB_ECC_code2 + 2);
	pr_debug("%02x%02x%02x", ecc_code[6], ecc_code[7], ecc_code[8]);

	ecc_code[9] = RD_HOST_REG8(PB_ECC_code3 + 0);
	ecc_code[10] = RD_HOST_REG8(PB_ECC_code3 + 1);
	ecc_code[11] = RD_HOST_REG8(PB_ECC_code3 + 2);
	pr_debug("%02x%02x%02x\n", ecc_code[9], ecc_code[10], ecc_code[11]);

	return 0;
}

/**
 * function to control hardware ecc generator.
 * Must only be provided if an hardware ECC is available
 */
static void smp8xxx_nand_enable_hwecc(struct mtd_info *mtd, int mode)
{
	struct nand_chip *chip = mtd->priv;
	unsigned int cs = ((struct chip_private *)chip->priv)->cs;
	WR_HOST_REG32(PB_ECC_clear, 0x80000008 | cs);
}

static void smp8xxx_select_chip(struct mtd_info *mtd, int chipnr)
{
//	static DEFINE_SPINLOCK(__mutex_lock);
	static int cs_cnt = 0; /* workaround asymetric select/de-select */
//	spin_lock_bh(&__mutex_lock);
	if (chipnr >= 0) {
		if (cs_cnt == 0) {
			if (tangox_mutex_lock(MUTEX_PBI)) {
				printk("%s:%d mutex acquisition failure.\n", __FILE__, __LINE__);	
			} else
				cs_cnt++;
		}
	} else {
		if (cs_cnt > 0) {
			cs_cnt--;
			tangox_mutex_unlock(MUTEX_PBI);
		}
	}
//	spin_unlock_bh(&__mutex_lock);
}

/* Loading partiton information from XENV */
static void smp8xxx_nand_load_part_info(void)
{
	char buf[BUFSIZE], pname[BUFSIZE];
	u32 dsize, rsvd_blks;
	u32 cs, part, parts, cnt;
	u64 rsvd_sz, psz, poff, blkmask;
	static const char *h32str = HIGH_32;

	for (cs = 0; cs < MAX_CS; cs++) {

		if (cs_avail[cs] == 0)	/* not available */
			continue;

		sprintf(buf, CS_RESERVED, cs);
		dsize = sizeof(u32);
		if ((zxenv_get(buf, &rsvd_blks, &dsize) < 0) || (dsize != sizeof(u32))) {
			cs_avail[cs] = 0;
			continue;
		} 

		/* find out the size of reservation zone */
		smp8xxx_mtds[cs].size = rsvd_sz = min((u64)smp8xxx_mtds[cs].erasesize * (u64)rsvd_blks, smp8xxx_mtds[cs].size);
		blkmask = ~(u64)(smp8xxx_mtds[cs].erasesize - 1);

		sprintf(buf, CS_PARTS, cs);
		dsize = sizeof(u32);
		if ((zxenv_get(buf, &parts, &dsize) < 0) || (dsize != sizeof(u32)))
			continue;
		else if (parts > MAX_PARTITIONS)
			parts = MAX_PARTITIONS;

		if ((smp8xxx_partitions[cs] = kmalloc(sizeof(struct mtd_partition) * parts, GFP_KERNEL)) == NULL) {
			cs_avail[cs] = 0;
			continue;
		}
		memset(smp8xxx_partitions[cs], 0, sizeof(struct mtd_partition) * parts);

		for (part = cnt = 0; part < parts; part++) {
			u32 pl, ph;
			
			pl = ph = 0;
			sprintf(buf, CS_PART_SIZE, cs, part + 1);
			dsize = sizeof(u32);
			if ((zxenv_get(buf, &pl, &dsize) < 0) || (dsize != sizeof(u32)))
				goto next;
			strcat(buf, h32str);
			if ((zxenv_get(buf, &ph, &dsize) < 0) || (dsize != sizeof(u32)))
				ph = 0;

			psz = ((((u64)ph) << 32) | (u64)pl) & blkmask; /* make it align to block boundary */
			if (psz == 0)
				goto next;
			smp8xxx_partitions[cs][cnt].size = psz;

			pl = ph = 0;
			sprintf(buf, CS_PART_OFFSET, cs, part + 1);
			dsize = sizeof(u32);
			if ((zxenv_get(buf, &pl, &dsize) < 0) || (dsize != sizeof(u32)))
				goto next;
			strcat(buf, h32str);
			if ((zxenv_get(buf, &ph, &dsize) < 0) || (dsize != sizeof(u32)))
				ph = 0;
			poff = (((u64)ph) << 32) | (u64)pl;

			if ((poff & (u64)(smp8xxx_mtds[cs].erasesize - 1)) != 0)	/* not aligned to block boundary */
				goto next;
			smp8xxx_partitions[cs][cnt].offset = poff;

			/* check if partition is out of reservation zone */
			if ((poff >= rsvd_sz) || ((poff + psz) > rsvd_sz)) {
				printk("%s: CS%d partition%d (0x%llx-0x%llx) out of reserved zone.\n", SMP_NAND_DEV_NAME, cs, part + 1, poff, poff + psz);
				goto next;
			}

			sprintf(buf, CS_PART_NAME, cs, part + 1);
			dsize = BUFSIZE;
			memset(pname, 0, BUFSIZE);
			if ((zxenv_get(buf, pname, &dsize) == 0) && (dsize > 0)) {	/* partition name is given */
				char *p;
				u32 i;
				for (i = 0, p = pname; (*p != '\0') && (i < dsize); p++, i++) {
					if (!isspace(*p))
						break;
				}
				if (*p == '\"') {	/* found leading '\"', try strip out trailing '\"' */
					char *e;
					p++;
					for (i = strnlen(p, BUFSIZE - i), e = p + (i - 1); i > 0; i--, e--) {
						if (isspace(*e))
							*e = '\0';
						else {
							if (*e == '\"') 
								*e = '\0';
							break;
						}
					}
				}
				smp8xxx_partitions[cs][cnt].name = kmalloc(strnlen(p, BUFSIZE) + 1, GFP_KERNEL);
				if (smp8xxx_partitions[cs][cnt].name)
					strncpy(smp8xxx_partitions[cs][cnt].name, p, strnlen(p, BUFSIZE) + 1);
				else
					goto next;
			} else {	/* cooked-up partition name here */
				sprintf(buf, "CS%d/PBPart%d", cs, part + 1);
				smp8xxx_partitions[cs][cnt].name = kmalloc(16, GFP_KERNEL);
				if (smp8xxx_partitions[cs][cnt].name)
					strncpy(smp8xxx_partitions[cs][cnt].name, buf, 16);
				else
					goto next;
			}

			cnt++;
			continue;	/* next partition */
next:
			smp8xxx_partitions[cs][cnt].size = smp8xxx_partitions[cs][cnt].offset = 0;
		}

		cs_parts[cs] = cnt;
	}
}

static void __init smp8xxx_set_nand_ctrler(void)
{
	unsigned long tangox_chip_id(void);
	unsigned long chip_id = (tangox_chip_id() >> 16) & 0xfffe;
	switch(chip_id) {
#ifndef CONFIG_TANGO3_SMP8656OTP
		case 0x8656:	/* OTP part uses old controller */
#endif
		case 0x8646:
			max_page_shift = 12; /* up to 4KB page */
		case 0x8670:
		case 0x8672:
		case 0x8674:
		case 0x8680:	/* 868A uses old controller */
		case 0x8682:
		case 0x8910:
			new_ctrler = 1;
			break;
		default:
			new_ctrler = 0;	/* use old controller */
			break;
	}
}

/* appending more entries from XENV to the table */
static void __init append_nand_flash_ids(void)
{
	int i, id, maf;
	struct nand_flash_dev *type = nand_flash_ids, *ptr;
	char buf[BUFSIZE];
	u32 dsize, params[6];

#define LP_OPTIONS (NAND_SAMSUNG_LP_OPTIONS | NAND_NO_READRDY | NAND_NO_AUTOINCR)
	for (i = 0; i < MAX_NAND_DEVS; i++) {
		sprintf(buf, NAND_PARAM, i);
		dsize = sizeof(params);

		/* get entry from XENV */
		memset(params, 0, dsize);
		if ((zxenv_get(buf, params, &dsize) < 0) && (dsize == 0)) 
			continue;

		if ((id = (params[0] >> 16) & 0xff) == 0)
			continue;
		maf = (params[0] >> 24) & 0xff;

		/* going to the end of table or matching entry found */
		for (ptr = type; ptr->id; ptr++) {
			if (ptr->id == id)
				break;
		}

		if ((!ptr->name) || (ptr->id == id))
			continue;	/* end of table, or found the entry in the table */

		/* append new entry to the table */
		ptr->id = id;
		if (((params[2] >> 16) & 0xf) >= 10) { /* >=2KB page size */
			ptr->options = LP_OPTIONS;
			ptr->erasesize = ptr->pagesize = 0; /* let MTD autodetect */
		} else {
			ptr->options = 0;
			ptr->pagesize = 1 << (((params[2] >> 16) & 0xf) + 1);
			ptr->erasesize = 1 << ((((params[2] >> 16) & 0xf) + 1) + (((params[2] >> 20) & 0xf) + 1));
		}
		ptr->chipsize = 1 << (((((params[2] >> 16) & 0xf) + 1) + (((params[2] >> 20) & 0xf) + 1) + (((params[2] >> 24) & 0xf) + 1)) - 20);
		sprintf(ptr->name, "Unknown device %ldMiB (%02x:%02x)", ptr->chipsize, maf, ptr->id);
	}
}

static int __init smp8xxx_nand_init(void)
{
	struct nand_chip *this;
	u32 mem_staddr;
	RMuint8 local_pb_cs_ctrl;
	RMuint32 local_pb_cs_config, local_pb_cs_config1;
	int cs, chip_cnt = 0;

	/* append the id table from XENV first, if any */
	append_nand_flash_ids();

	memset(smp8xxx_mtds, 0, sizeof(struct mtd_info) * MAX_CS);
	memset(smp8xxx_chips, 0, sizeof(struct nand_chip) * MAX_CS);
	memset(smp8xxx_partitions, 0, sizeof(struct mtd_partition *) * MAX_CS);
	memset(cs_avail, 0, sizeof(int) * MAX_CS);
	memset(cs_parts, 0, sizeof(int) * MAX_CS);

	smp8xxx_set_nand_ctrler();

	printk("%s SMP8xxx NAND Driver %s (multi-bits ECC: %s)\n", smp_nand_devname, SMP_NAND_DRV_VERSION, new_ctrler ? "enabled" : "disabled");

	local_pb_cs_ctrl = RD_HOST_REG8(PB_CS_ctrl);
	local_pb_cs_config = RD_HOST_REG32(PB_CS_config);
	local_pb_cs_config1 = RD_HOST_REG32(PB_CS_config1);
	switch((local_pb_cs_ctrl >> 4) & 7) {
		case 0: cs_offset = 0x200;
			break;
		case 1: 
		case 2: cs_offset = 0x100;
			break;
		default:
			printk("No NAND flash is available (0x%x).\n", local_pb_cs_ctrl);
			return -EIO;
	}

	for (cs = 0; cs < 4; cs++) {
		if ((local_pb_cs_config >> (20 + cs)) & 1)
			cs_avail[cs] = 1;
	}
#if (MAX_CS >= 4)
	for (cs = 0; cs < 4; cs++) {
		if ((local_pb_cs_config1 >> (20 + cs)) & 1)
			cs_avail[cs + 4] = 1;
	}
#endif

	spin_lock_init(&smp8xxx_hw_control.lock);
	init_waitqueue_head(&smp8xxx_hw_control.wq);

	for (cs = 0; cs < MAX_CS; cs++) {
		int i;
		unsigned long pg_size, oob_size, blk_size;
		uint64_t chip_size;
		u32 dsize, params[6];
		char buf[BUFSIZE];

		if (cs_avail[cs] == 0)
			goto next;
		if (new_ctrler) { /* bounce buffer may be needed with new controller */
			if ((chip_privs[cs].bbuf = kmalloc(NAND_MAX_PAGESIZE, GFP_KERNEL | GFP_DMA)) == NULL) {	/* up to 8KB */
				cs_avail[cs] = 0;
				goto next;
			}
		} else 
			chip_privs[cs].bbuf = NULL;
		chip_privs[cs].cs = cs;
		smp8xxx_mtds[cs].priv = &smp8xxx_chips[cs];
		smp8xxx_mtds[cs].owner = THIS_MODULE;
		this = &smp8xxx_chips[cs];
		this->priv = &chip_privs[cs];

		mem_staddr = cs * cs_offset;

		/* 30 us command delay time */
		this->chip_delay   = 30;

		this->ecc.mode     = NAND_ECC_SOFT;
		this->options      = NAND_NO_AUTOINCR | BBT_AUTO_REFRESH;
		this->controller   = &smp8xxx_hw_control;

		this->read_byte    = smp8xxx_read_byte;
		this->read_buf     = smp8xxx_read_buf;
		this->write_buf    = smp8xxx_write_buf;
		this->verify_buf   = smp8xxx_verify_buf;

		this->cmd_ctrl     = smp8xxx_nand_hwcontrol;
		this->dev_ready    = smp8xxx_nand_devready;

		this->IO_ADDR_W	   = this->IO_ADDR_R = (void __iomem *)mem_staddr;

		/* nand reset */
		WR_HOST_REG8((RMuint32)this->IO_ADDR_W + SMP8XXX_REG_CMD, NAND_CMD_RESET);
		udelay(tReset);

		printk("%s: checking NAND device on CS%d ..\n", SMP_NAND_DEV_NAME, cs);

		/* Scan to find existence of the device */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
		if (nand_scan_ident(&smp8xxx_mtds[cs], 1)) 
#else
		if (nand_scan_ident(&smp8xxx_mtds[cs], 1, NULL)) 
#endif
			goto next;
		
		/* checking XENV to see if parameters are given ... */
		for (i = 0; i < MAX_NAND_DEVS; i++) {
			sprintf(buf, NAND_PARAM, i);
			dsize = sizeof(params);

			memset(params, 0, dsize);
			if ((zxenv_get(buf, params, &dsize) < 0) && (dsize == 0))
				continue;

			/* matching maf_id and dev_id?? */
			if ((smp8xxx_chips[cs].maf_id == ((params[0] >> 24) & 0xff)) && (smp8xxx_chips[cs].dev_id == ((params[0] >> 16) & 0xff)))
				break;
		}

		/* saving information from XENV */
		chip_size = pg_size = oob_size = blk_size = 0;
		if (i < MAX_NAND_DEVS) {
			oob_size = params[2] & 0xffff;
			pg_size = 1 << (((params[2] >> 16) & 0xf) + 1);
			blk_size = pg_size * (1 << (((params[2] >> 20) & 0xf) + 1));
			chip_size = ((uint64_t)1) << ((((params[2] >> 16) & 0xf) + 1) + (((params[2] >> 20) & 0xf) + 1) + (((params[2] >> 24) & 0xf) + 1));
		}

		if (new_ctrler) {
#define DEF_TIMING1	0x1b162c16	/* conservative timing1 */
#define DEF_TIMING2	0x120e1f58	/* conservative timing2 */
#define DEF_DEVCFG	0x35		/* default devcfg, may not be correct */
			u32 timing1, timing2, devcfg;

			sprintf(buf, CS_TIMING1, cs);
			dsize = sizeof(u32);
			if ((zxenv_get(buf, &timing1, &dsize) < 0) || (dsize != sizeof(u32))) 
				timing1 = (params[3] ? params[3] : DEF_TIMING1);
			sprintf(buf, CS_TIMING2, cs);
			dsize = sizeof(u32);
			if ((zxenv_get(buf, &timing2, &dsize) < 0) || (dsize != sizeof(u32))) 
				timing2 = (params[4] ? params[4] : DEF_TIMING2);
			sprintf(buf, CS_DEVCFG, cs);
			dsize = sizeof(u32);
			if ((zxenv_get(buf, &devcfg, &dsize) < 0) || (dsize != sizeof(u32))) 
				devcfg = (params[5] ? params[5] : DEF_DEVCFG);

			WR_HOST_REG32(DEVICE_CFG(chx_reg[cs]), devcfg);
			WR_HOST_REG32(TIMING1(chx_reg[cs]), timing1);
			WR_HOST_REG32(TIMING2(chx_reg[cs]), timing2);
		}

		this->ecc.mode     = NAND_ECC_HW;
		this->ecc.steps    = 1;
		this->ecc.hwctl    = smp8xxx_nand_enable_hwecc;
		this->select_chip  = smp8xxx_select_chip;

		if (new_ctrler) {
			this->ecc.write_page = smp8xxx_write_page_hwecc;
			this->ecc.read_page  = smp8xxx_read_page_hwecc;
			this->cmdfunc        = smp8xxx_command;
			this->ecc.calculate  = smp8xxx_nand_bug_calculate;
			this->ecc.correct    = smp8xxx_nand_bug_correct;
			this->ecc.hwctl      = smp8xxx_nand_hwctl;
		}

		switch (smp8xxx_mtds[cs].writesize) {
			case 512:
				if (new_ctrler) {
					this->ecc.layout = &smp8xxx_nand_ecclayout512_16_4;
					this->ecc.layout->oobfree[0].length = smp8xxx_mtds[cs].oobsize 
										- this->ecc.layout->oobfree[0].offset;
				} else {
					this->ecc.calculate = smp8xxx_nand_calculate_ecc_512;
					this->ecc.correct = ecc_correct_512;
					this->ecc.bytes = this->ecc.total = 3;
					this->ecc.layout = &smp8xxx_oobinfo_16;
				}
				this->ecc.size = 512;
				break;

			case 2048:
				if (new_ctrler) {
					this->ecc.layout = &smp8xxx_nand_ecclayout2048_64_8;
					this->ecc.layout->oobfree[0].length = smp8xxx_mtds[cs].oobsize 
										- this->ecc.layout->oobfree[0].offset;
				} else {
					this->ecc.calculate = smp8xxx_nand_calculate_ecc_2048;
					this->ecc.correct = ecc_correct_2048;
					this->ecc.bytes = this->ecc.total = 12;
					this->ecc.layout = &smp8xxx_oobinfo_64;
				}
				this->ecc.size = 2048;
				break;	

			case 4096:	
				if (new_ctrler) {
					if ((smp8xxx_mtds[cs].oobsize >= 128) && (smp8xxx_mtds[cs].oobsize < 218)) {
						this->ecc.layout = &smp8xxx_nand_ecclayout4096_128_9;
						this->ecc.layout->oobfree[0].length = smp8xxx_mtds[cs].oobsize 
											- this->ecc.layout->oobfree[0].offset;
					} else if (smp8xxx_mtds[cs].oobsize >= 218) {
						this->ecc.layout = &smp8xxx_nand_ecclayout4096_218_16;
						this->ecc.layout->oobfree[0].length = smp8xxx_mtds[cs].oobsize 
											- this->ecc.layout->oobfree[0].offset;
					} else {
						printk("%s: unsupported NAND on CS%d.\n", SMP_NAND_DEV_NAME, cs);
						printk("%s: oobsize (%d) unsupported on CS%d (pagesize: %d, erasesize: %d).\n", SMP_NAND_DEV_NAME, smp8xxx_mtds[cs].oobsize, cs, smp8xxx_mtds[cs].writesize, smp8xxx_mtds[cs].erasesize);
						goto next;
					}
					this->ecc.size = 4096;
				} else
					goto next;
				break;

			case 8192: 
				if ((new_ctrler) && (max_page_shift >= 13)) {
					this->ecc.layout = &smp8xxx_nand_ecclayout8192_448_16;
					this->ecc.layout->oobfree[0].length = smp8xxx_mtds[cs].oobsize 
										- this->ecc.layout->oobfree[0].offset;
					this->ecc.size = 8192;
				} else
					goto next;
				break;

			default:
				printk("%s: unsupported NAND on CS%d.\n", SMP_NAND_DEV_NAME, cs);
				goto next;
		}

		if (nand_scan_tail(&smp8xxx_mtds[cs]))
			goto next;

		printk("%s: detected NAND on CS%d, %lldMiB, erasesize %dKiB, pagesize %dB, oobsize %dB, oobavail %dB\n",
			SMP_NAND_DEV_NAME, cs, smp8xxx_mtds[cs].size >> 20, smp8xxx_mtds[cs].erasesize >> 10,
			smp8xxx_mtds[cs].writesize, smp8xxx_mtds[cs].oobsize, smp8xxx_mtds[cs].oobavail);

		if (new_ctrler) {
			u32 p_cyc, e_cyc, devcfg;
			if (smp8xxx_mtds[cs].writesize < 2048) {	/* small page */
				e_cyc = ((smp8xxx_mtds[cs].size >> 20) > 32) ? 3 : 2;
				p_cyc = e_cyc + 1;
			} else {
				e_cyc = ((smp8xxx_mtds[cs].size >> 20) > 128) ? 3 : 2;
				p_cyc = e_cyc + 2;
			}
			devcfg = (e_cyc << 4) | p_cyc;
			if ((RD_HOST_REG32(DEVICE_CFG(chx_reg[cs])) & 0xff) != devcfg) {
				printk(KERN_WARNING "%s: CS%d devcfg mismatch detected (0x%x specified, 0x%x detected)\n",
					SMP_NAND_DEV_NAME, cs, RD_HOST_REG32(DEVICE_CFG(chx_reg[cs])) & 0xff, devcfg);
				WR_HOST_REG32(DEVICE_CFG(chx_reg[cs]), devcfg);
			}
		}

		/* Check against the saved information from XENV */
		if (chip_size && chip_size != smp8xxx_mtds[cs].size)
			printk(KERN_WARNING "%s: CS%d size mismatch detected (%lldMiB specified, %lldMiB detected)\n",
				SMP_NAND_DEV_NAME, cs, chip_size >> 20, smp8xxx_mtds[cs].size >> 20);
		if (blk_size && blk_size != smp8xxx_mtds[cs].erasesize)
			printk(KERN_WARNING "%s: CS%d erasesize mismatch detected (%ldKiB specified, %dKiB detected)\n",
				SMP_NAND_DEV_NAME, cs, blk_size >> 10, smp8xxx_mtds[cs].erasesize >> 10);
		if (pg_size && pg_size != smp8xxx_mtds[cs].writesize)
			printk(KERN_WARNING "%s: CS%d pagesize mismatch detected (%ldB specified, %dB detected)\n",
				SMP_NAND_DEV_NAME, cs, pg_size, smp8xxx_mtds[cs].writesize);
		if (oob_size && oob_size != smp8xxx_mtds[cs].oobsize)
			printk(KERN_WARNING "%s: CS%d oobsize mismatch detected (%ldB specified, %dB detected)\n",
				SMP_NAND_DEV_NAME, cs, oob_size, smp8xxx_mtds[cs].oobsize);

		cs_avail[cs] = 1;
		chip_cnt++;
		continue;
next:
		cs_avail[cs] = 0;
		continue;
	}

	if (chip_cnt) {
		printk("%s: detection completed, load partition information from XENV ..\n", SMP_NAND_DEV_NAME);
		smp8xxx_nand_load_part_info();
		for (cs = 0; cs < MAX_CS; cs++) {
			/* Register the partitions */
			if (cs_avail[cs]) {
				/* check if chip sizes are passed in as parameters */
				if ((chip_szs[cs] != 0) && (chip_szs[cs] <= smp8xxx_chips[cs].chipsize))
					smp8xxx_mtds[cs].size = chip_szs[cs];	/* use what's been specified */
				if (cs_parts[cs]) {
					if (smp8xxx_mtds[cs].size) 
						mtd_device_register(&smp8xxx_mtds[cs], NULL, 0);
					printk("%s: load partition information for CS%d ..\n", SMP_NAND_DEV_NAME, cs);
					mtd_device_register(&smp8xxx_mtds[cs], smp8xxx_partitions[cs], cs_parts[cs]);
				} else {
					if (smp8xxx_mtds[cs].size)
						mtd_device_register(&smp8xxx_mtds[cs], NULL, 0);
				}
			}
		}
		return 0;
	} else {
		printk("No NAND flash is detected.\n");
		return -EIO;
	}
}

static void __exit smp8xxx_nand_exit(void)
{
	int cs, i;

	for (cs = 0; cs < MAX_CS; cs++) {
		/* Release resources, unregister device */
		if (cs_avail[cs]) {
			nand_release(&smp8xxx_mtds[cs]);
			if (smp8xxx_partitions[cs]) {
				for (i = 0; i < cs_parts[cs]; i++) {
					if (smp8xxx_partitions[cs][i].name)
						kfree(smp8xxx_partitions[cs][i].name);
				}
				kfree(smp8xxx_partitions[cs]);
			}
			if (chip_privs[cs].bbuf)
				kfree(chip_privs[cs].bbuf);
		}
	}
}

module_init(smp8xxx_nand_init);
module_exit(smp8xxx_nand_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ilhong Yoon <iyoon@sdesigns.com>, Daniel Ross <Daniel_Ross@sdesigns.com>");
MODULE_DESCRIPTION("SMP8xxx MTD NAND driver");

