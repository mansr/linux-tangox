
/*********************************************************************
 Copyright (C) 2001-2009
 Sigma Designs, Inc. 

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.

 01/2009: Merged/adapted the codes from Jean-Francois Thibert (SageTV)
 *********************************************************************/

#include <linux/config.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>

#include <asm/uaccess.h>
#include <asm/r4kcache.h>
#include <linux/pagemap.h>
#include <linux/dma-mapping.h>

#include "setup.h"

#if !defined(CONFIG_TANGO2) && !defined(CONFIG_TANGO3)
#error Undefined Sigma chip!!!
#endif

#ifdef CONFIG_TANGO3

/* Uncomment this only if W1/R1 can be used (typically not) */
// #define WITH_MBUS_W1R1

#endif /* CONFIG_TANGO3 */

/*
 * computed in prom.c
 */
extern unsigned long em8xxx_kmem_start;
extern unsigned long em8xxx_kmem_size;

/*
 * switchbox stuffs
 *
 * We keep  track of  current mapping using  this globals  rather than
 * reading hardware registers each time.
 */
static unsigned int g_sbox_map[SBOX_MAX];
static unsigned int sbox_inited = 0;

static inline void sbox_update_route(void)
{
	int i;
#ifdef CONFIG_TANGO3
	u64 data = 0; /* to cover two 32 bits registers */
#else
	unsigned int data = 0;
#endif

	for (i = SBOX_MAX - 1; i >= 0; --i)
		data = (data << 4) | g_sbox_map[i];

#ifdef CONFIG_TANGO3
	gbus_write_reg32(REG_BASE_host_interface + SBOX_ROUTE, data & 0xffffffff);
	gbus_write_reg32(REG_BASE_host_interface + SBOX_ROUTE2, (data >> 32) & 0xffffffff);
#else
	gbus_write_reg32(REG_BASE_host_interface + SBOX_ROUTE, data);
#endif
}

static void sbox_reset(void)
{
#ifdef CONFIG_TANGO3
#ifdef WITH_MBUS_W1R1
	gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET, 0xffffffff);
	gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET, 0xff00ff00);
#else
	/* Leave W1/R1 alone. */
	gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET, 0xfdfdfdfd);
	gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET, 0xfd00fd00);
#endif
	gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET2, 0x01010101);
	gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET2, 0x01000100);
#else
	/* Leave W1/R1 alone. */
	gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET, 0x7d7d7d7d);
	gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET, 0x7d007d00);
#endif
}

static void sbox_setup(void)
{
	unsigned long flags;

	local_irq_save(flags);

	/* W0 initially disconnected */
	g_sbox_map[SBOX_MBUS_W0] = 0xf;

#if defined(CONFIG_TANGO3) && defined(WITH_MBUS_W1R1)
	g_sbox_map[SBOX_MBUS_W1] = 0xf;
#else
	/* Leave W1 alone */
	g_sbox_map[SBOX_MBUS_W1] = 0;
#endif

	g_sbox_map[SBOX_PCIMASTER] = 0xf;
	g_sbox_map[SBOX_PCISLAVE] = SBOX_PCISLAVE + 1; /* Loopback */
	g_sbox_map[SBOX_SATA1] = 0xf;
	g_sbox_map[SBOX_IDEDVD] = 0xf;
	g_sbox_map[SBOX_IDEFLASH] = 0xf;
#ifdef CONFIG_TANGO3
	g_sbox_map[SBOX_SATA2] = 0xf;
	g_sbox_map[SBOX_MBUS_W2] = 0xf;
#else
	g_sbox_map[SBOX_UNUSED1] = 0xf;
#endif

	sbox_update_route();
	wmb();

	local_irq_restore(flags);
}

/*
 * Connect given interface to R?/W? channel
 */
#ifdef CONFIG_TANGO3
static int sbox_connect(int iface, int *channel)
#else
static int sbox_connect(int iface)
#endif
{
	unsigned long flags;
	int res = 0;
#ifdef CONFIG_TANGO3
	int chan = 0;
#endif

	if (sbox_inited == 0)
		return 1;

	local_irq_save(flags);

#ifdef CONFIG_TANGO3
	/* Already connected? */
	if (g_sbox_map[SBOX_MBUS_W0] == (iface + 1) && g_sbox_map[iface] == (SBOX_MBUS_W0 + 1)) {
		*channel = 0; /* connected to W0/R0 */
		goto done;
#ifdef WITH_MBUS_W1R1
	} else if (g_sbox_map[SBOX_MBUS_W1] == (iface + 1) && g_sbox_map[iface] == (SBOX_MBUS_W1 + 1)) {
		*channel = 1; /* connected to W1/R1 */
		goto done;
#endif
	} else if (g_sbox_map[SBOX_MBUS_W2] == (iface + 1) && g_sbox_map[iface] == (SBOX_MBUS_W2 + 1)) {
		*channel = 2; /* connected to W2/R2 */
		goto done;
	}

	if (g_sbox_map[iface] != 0xf) { /* connect to something else already */
		res = 1;
		goto done;
	}

	/* In use ? */
	if (g_sbox_map[SBOX_MBUS_W0] != 0xf) {
		chan++; /* try next */
#ifdef WITH_MBUS_W1R1
		if (g_sbox_map[SBOX_MBUS_W1] != 0xf) {
#endif
			chan++; /* try next */
			if (g_sbox_map[SBOX_MBUS_W2] != 0xf)  {
				res = 1; /* Both W0/W2 not available, and optionally W1 as well */
				goto done;
			}
#ifdef WITH_MBUS_W1R1
		}
#endif
	}
#else
	/* Already connected? */
	if (g_sbox_map[SBOX_MBUS_W0] == (iface + 1) && g_sbox_map[iface] == (SBOX_MBUS_W0 + 1)) 
		goto done;

	/* In use ? */
	if (g_sbox_map[SBOX_MBUS_W0] != 0xf || g_sbox_map[iface] != 0xf) {
		res = 1;
		goto done;
	}
#endif /* CONFIG_TANGO3 */

#ifdef CONFIG_TANGO3
	switch(chan) {
		case 0: g_sbox_map[SBOX_MBUS_W0] = iface + 1;
			g_sbox_map[iface] = SBOX_MBUS_W0 + 1; /* W0/R0 */
			break;
#ifdef WITH_MBUS_W1R1
		case 1: g_sbox_map[SBOX_MBUS_W1] = iface + 1;
			g_sbox_map[iface] = SBOX_MBUS_W1 + 1; /* W1/R1 */
			break;
#endif
		case 2: g_sbox_map[SBOX_MBUS_W2] = iface + 1;
			g_sbox_map[iface] = SBOX_MBUS_W2 + 1; /* W2/R2 */
			break;

		default: BUG();
			break;
	}
	*channel = chan;
#else
	g_sbox_map[SBOX_MBUS_W0] = iface + 1;
	g_sbox_map[iface] = SBOX_MBUS_W0 + 1;
#endif
	sbox_update_route();
	wmb();

done:
	local_irq_restore(flags);

	return res;
}

#ifdef CONFIG_TANGO3
static void sbox_disconnect(int iface, int channel)
#else
static void sbox_disconnect(int iface)
#endif
{
	unsigned long flags;

	if (iface >= 0) {
		local_irq_save(flags);

#ifdef CONFIG_TANGO3
		switch(channel) {
			case 0: g_sbox_map[SBOX_MBUS_W0] = 0xf;
				break;
#ifdef WITH_MBUS_W1R1
			case 1: g_sbox_map[SBOX_MBUS_W1] = 0xf;
				break;
#endif
			case 2: g_sbox_map[SBOX_MBUS_W2] = 0xf;
				break;

			default: BUG();
				break;
		}
#else
		g_sbox_map[SBOX_MBUS_W0] = 0xf;
#endif
		g_sbox_map[iface] = 0xf;
		sbox_update_route();
		wmb();

		local_irq_restore(flags);
	}
}

static void sbox_init(void)
{
	sbox_setup();
	sbox_reset();
	sbox_inited = 1;
}


/*
 * mbus stuffs
 *
 * to  avoid   requesting/freeing  irq   each  time,  we   keep  given
 * handler/args  for each  dma  request and  call  it in  our own  irq
 * handler.
 */
#define MBUS_LINEAR_MAX		(0x2000 - 1)

#ifdef CONFIG_TANGO3
static mbus_irq_handler_t g_mbus_intr_handler[6];
static void *g_mbus_intr_handler_arg[6];
#else
static mbus_irq_handler_t g_mbus_intr_handler[4];
static void *g_mbus_intr_handler_arg[4];
#endif

/*
 * alloc_dma, need to be called before setup, will try to connect
 * needed sbox.
 */
int em86xx_mbus_alloc_dma(int sbox, int fromdev, unsigned long *pregbase, int *pirq)
{
	int x;

#ifdef CONFIG_TANGO3
	int channel = 0;

	if (sbox_connect(sbox, &channel) != 0)
		return -1;

	switch(channel) {
		case 0: { 	/* Using W0/R0 pair */
				x = (fromdev ? 0 : 2);
				if (pirq)
					*pirq = LOG2_CPU_HOST_MBUS_W0_INT + IRQ_CONTROLLER_IRQ_BASE + x;
				if (pregbase)
					*pregbase = REG_BASE_host_interface + MIF_W0_ADD + (x * 0x40);
			}
			break;
#ifdef WITH_MBUS_W1R1
		case 1: { 	/* Using W1/R1 pair */
				x = (fromdev ? 0 : 2);
				if (pirq)
					*pirq = LOG2_CPU_HOST_MBUS_W1_INT + IRQ_CONTROLLER_IRQ_BASE + x;
				if (pregbase)
					*pregbase = REG_BASE_host_interface + MIF_W1_ADD + (x * 0x40);
			}
			break;
#endif
		case 2: { 	/* Using W2/R2 pair */
				x = (fromdev ? 0 : 1);
				if (pirq)
					*pirq = LOG2_CPU_HOST_MBUS_W2_INT + IRQ_CONTROLLER_IRQ_BASE + x;
				if (pregbase)
					*pregbase = REG_BASE_host_interface + MIF_W2_ADD + (x * 0x40);
			}
			break;

		default: BUG();
			break;
	}
#else
	if (sbox_connect(sbox) != 0)
		return -1;

	x = (fromdev ? 0 : 2);
	if (pirq)
		*pirq = LOG2_CPU_HOST_MBUS_W0_INT + IRQ_CONTROLLER_IRQ_BASE + x;
	if (pregbase)
		*pregbase = REG_BASE_host_interface + MIF_W0_ADD + x * 0x40;
#endif

	return 0;
}

#ifdef CONFIG_TANGO3
/* Convert MBUS register address to channel index */
static inline int mbus_idx2channel(unsigned int regbase)
{
	int idx = (regbase - (REG_BASE_host_interface + MIF_W0_ADD)) / 0x40;
	static const int channel[6] = { 0, 1, 0, 1, 2, 2 };
	return(channel[idx]);
}
#endif

/*
 * free_dma,  need to  be called  after  transfer is  done to  release
 * switchbox.
 */
void em86xx_mbus_free_dma(unsigned long regbase, int sbox)
{
	unsigned long flags;
	int idx = (regbase - (REG_BASE_host_interface + MIF_W0_ADD)) / 0x40;

	local_irq_save(flags);
	g_mbus_intr_handler[idx] = NULL;
	wmb();
	local_irq_restore(flags);

#ifdef CONFIG_TANGO3
	sbox_disconnect(sbox, mbus_idx2channel(regbase)); 
#else
	sbox_disconnect(sbox);
#endif
}

/*
 * irq handler for mbus interrupt
 */
static irqreturn_t mbus_intr(int irq, void *devinfo)
{
	int idx = irq - (LOG2_CPU_HOST_MBUS_W0_INT + IRQ_CONTROLLER_IRQ_BASE);

#ifdef CONFIG_TANGO3
	if (idx >= 4)
		idx = irq - (LOG2_CPU_HOST_MBUS_W2_INT + IRQ_CONTROLLER_IRQ_BASE) + 4;
#endif
	if (g_mbus_intr_handler[idx]) {
		mbus_irq_handler_t f;

		f = g_mbus_intr_handler[idx];
		g_mbus_intr_handler[idx] = NULL;
		wmb();
		f(irq, g_mbus_intr_handler_arg[idx]);
	}

	return IRQ_HANDLED;
}

/*
 * check if mbus is in use for given regbase
 */
static inline int mbus_inuse(unsigned int regbase)
{
	return (gbus_read_reg32(regbase + MIF_cmd_offset) & 0x7) != 0;
}

/*
 * setup mbus  register to start  a linear transfer (count  bytes from
 * addr, where count < MBUS_LINEAR_MAX)
 */
void mbus_setup_dma_linear(unsigned int regbase,
					 unsigned int addr,
					 unsigned int count,
					 unsigned int flags)
{
#if !defined(CONFIG_SD_DIRECT_DMA) 
	if ((addr < tangox_dma_address(CPHYSADDR(em8xxx_kmem_start))) || (addr >= (tangox_dma_address(CPHYSADDR(em8xxx_kmem_start)) + em8xxx_kmem_size)))
		printk("MBUS Warning (linear): bad transfer address 0x%08x\n", addr);
#endif

	gbus_write_reg32(regbase + MIF_add_offset, addr);
	gbus_write_reg32(regbase + MIF_cnt_offset, count);
	gbus_write_reg32(regbase + MIF_cmd_offset, (flags<<2)|0x1);
}

/*
 * setup mbus  register to start  a double transfer (count  bytes from
 * addr and count2 bytes from addr2, where count < MBUS_LINEAR_MAX and
 * count2 < MBUS_LINEAR_MAX)
 */
void mbus_setup_dma_double(unsigned int regbase,
					 unsigned int addr,
					 unsigned int count,
					 unsigned int addr2,
					 unsigned int count2,
					 unsigned int flags)
{
#if !defined(CONFIG_SD_DIRECT_DMA) 
	if ((addr < tangox_dma_address(CPHYSADDR(em8xxx_kmem_start))) || (addr >= (tangox_dma_address(CPHYSADDR(em8xxx_kmem_start)) + em8xxx_kmem_size)))
		printk("MBUS Warning (double): bad transfer address 0x%08x\n", addr);
	if ((addr2 < tangox_dma_address(CPHYSADDR(em8xxx_kmem_start))) || (addr2 >= (tangox_dma_address(CPHYSADDR(em8xxx_kmem_start)) + em8xxx_kmem_size)))
		printk("MBUS Warning (double): bad transfer address2 0x%08x\n", addr2);
#endif

	gbus_write_reg32(regbase + MIF_add_offset, addr);
	gbus_write_reg32(regbase + MIF_cnt_offset, (count2 << 16) | count);
	gbus_write_reg32(regbase + MIF_add2_skip_offset, addr2);
	gbus_write_reg32(regbase + MIF_cmd_offset, (flags<<2)|0x2);
}

/*
 * setup mbus  register to start  a rectangle transfer (horiz  * lines
 * bytes  from  addr,  where  horiz  <  MBUS_LINEAR_MAX  and  lines  <
 * MBUS_LINEAR_MAX)
 */
void mbus_setup_dma_rectangle(unsigned int regbase,
					    unsigned int addr,
					    unsigned int horiz,
					    unsigned int lines,
					    unsigned int flags)
{
#if !defined(CONFIG_SD_DIRECT_DMA) 
	if ((addr < tangox_dma_address(CPHYSADDR(em8xxx_kmem_start))) || (addr >= (tangox_dma_address(CPHYSADDR(em8xxx_kmem_start)) + em8xxx_kmem_size)))
		printk("MBUS Warning (rectangle): bad transfer address 0x%08x\n", addr);
#endif

	gbus_write_reg32(regbase + MIF_add_offset, addr);
	gbus_write_reg32(regbase + MIF_cnt_offset, (lines << 16) | horiz);
	gbus_write_reg32(regbase + MIF_add2_skip_offset, horiz);
	gbus_write_reg32(regbase + MIF_cmd_offset, (flags<<2)|0x3);
}

/*
 * register mbus interrupt if not done
 */
#ifdef CONFIG_TANGO3
static inline void mbus_register_intr(int channel)
#else
static inline void mbus_register_intr(void)
#endif
{
#ifdef CONFIG_TANGO3
	static int done[3] = { 0, 0, 0 };

	static const int rx_irq[3] = { LOG2_CPU_HOST_MBUS_R0_INT, LOG2_CPU_HOST_MBUS_R1_INT, LOG2_CPU_HOST_MBUS_R2_INT };
	static const int wx_irq[3] = { LOG2_CPU_HOST_MBUS_W0_INT, LOG2_CPU_HOST_MBUS_W1_INT, LOG2_CPU_HOST_MBUS_W2_INT };
	static const char *rx_str[3] = { "tangox_mbus_r0", "tangox_mbus_r1", "tangox_mbus_r2" };
	static const char *wx_str[3] = { "tangox_mbus_w0", "tangox_mbus_w1", "tangox_mbus_w2" };

	if (channel >= 3)
		BUG();
	else if (done[channel])
		return;

	done[channel] = 1;
	if (request_irq(rx_irq[channel] + IRQ_CONTROLLER_IRQ_BASE, mbus_intr, IRQF_DISABLED, rx_str[channel], NULL) != 0) 
		printk("MBUS: fail to register MBUS ISR(%d)\n", rx_irq[channel] + IRQ_CONTROLLER_IRQ_BASE);
	if (request_irq(wx_irq[channel] + IRQ_CONTROLLER_IRQ_BASE, mbus_intr, IRQF_DISABLED, wx_str[channel], NULL) != 0)
		printk("MBUS: fail to register MBUS ISR(%d)\n", wx_irq[channel] + IRQ_CONTROLLER_IRQ_BASE);
#else
	static int done = 0;

	if (done)
		return;
	done = 1;
	/*
	 * register irq handler for R0/W0 only (R1/W1 are not used for
	 * the moment)
	 */
	if (request_irq(LOG2_CPU_HOST_MBUS_R0_INT + IRQ_CONTROLLER_IRQ_BASE,
		    mbus_intr, IRQF_DISABLED, "tangox_mbus_r0", NULL) != 0) 
		printk("MBUS: fail to register MBUS ISR(%d)\n", LOG2_CPU_HOST_MBUS_R0_INT + IRQ_CONTROLLER_IRQ_BASE);

	if (request_irq(LOG2_CPU_HOST_MBUS_W0_INT + IRQ_CONTROLLER_IRQ_BASE,
		    mbus_intr, IRQF_DISABLED, "tangox_mbus_w0", NULL) != 0)
		printk("MBUS: fail to register MBUS ISR(%d)\n", LOG2_CPU_HOST_MBUS_W0_INT + IRQ_CONTROLLER_IRQ_BASE);

#endif
}

/*
 * setup void transaction 
 */
void em86xx_mbus_setup_dma_void(unsigned int regbase)
{
	while (mbus_inuse(regbase) != 0)
		;
	gbus_write_reg32(regbase + MIF_cmd_offset, 4);
}

/*
 * start  a   mbus  dma,   use  this  after   a  sucessfull   call  to
 * em86xx_mbus_alloc_dma
 */
int em86xx_mbus_setup_dma(unsigned int regbase, unsigned int addr,
			  unsigned int count, mbus_irq_handler_t handler,
			  void *arg, unsigned int tflags)
{
	unsigned long flags;
	unsigned int horiz, lines, sz;
	unsigned int idx = (regbase - (REG_BASE_host_interface + MIF_W0_ADD)) / 0x40;

	/*
	 * make sure no one uses the mbus before
	 */
	if (unlikely(mbus_inuse(regbase))) {
		printk(KERN_ERR "MBUS: error previous command is pending\n");
		return 1;
	}

	/*
	 * "register" given handler if any
	 */
	if (handler) {
#ifdef CONFIG_TANGO3
		mbus_register_intr(mbus_idx2channel(regbase));
#else
		mbus_register_intr();
#endif
		local_irq_save(flags);
		g_mbus_intr_handler[idx] = handler;
		g_mbus_intr_handler_arg[idx] = arg;
		wmb();
		local_irq_restore(flags);
	}

	/*
	 * decide which dma function to use depending on count
	 */
	if (count <= MBUS_LINEAR_MAX) {
		mbus_setup_dma_linear(regbase, addr, count, tflags);
		return 0;
	}

	if (count <= (MBUS_LINEAR_MAX * 2)) {
		mbus_setup_dma_double(regbase, addr, MBUS_LINEAR_MAX,
				      addr + MBUS_LINEAR_MAX,
				      count - MBUS_LINEAR_MAX, tflags);
		return 0;
	}

	/*
	 * we need to use rectangle, compute  horiz & lines
	 * values to use
	 */
	for (idx = 0, horiz = 1, sz = count; (idx < 10) && ((sz & 0x01) == 0); ++idx, horiz <<= 1, sz >>= 1)
		;
	lines = count >> idx;
	if ((horiz > MBUS_LINEAR_MAX) || (lines > MBUS_LINEAR_MAX)) {
		printk(KERN_ERR "MBUS: can't handle rectangle transfer "
		       "of %d bytes (h: %d, v: %d)\n", count, horiz, lines);
		BUG();
	}
	mbus_setup_dma_rectangle(regbase, addr, horiz, lines, tflags);

	return 0;
}

/*
 * Bit 0/8: MBUS_R0_SBOX
 * Bit 1/9: MBUS_R1_SBOX
 * Bit 2/10: PCI_MASTER_SBOX
 * Bit 3/11: PCI_SLAVE_SBOX
 * Bit 4/12: SATA1_SBOX
 * Bit 5/13: IDE_ISA_SBOX
 * Bit 6/14: IDE_DVD_SBOX
 * Bit 7/15: SATA2_SBOX (Tango3)
 * Bit 16/24: SBOX_MBUS_W0
 * Bit 17/25: SBOX_MBUS_W1
 * Bit 18/26: SBOX_PCI_MASTER
 * Bit 19/27: SBOX_PCI_SLAVE
 * Bit 20/28: SBOX_SATA1
 * Bit 21/29: SBOX_ISA
 * Bit 22/30: SBOX_DVD
 * Bit 23/31: SBOX_SATA2 (Tango3)
 *
 * Bit 32/40: MBUS_R2_SBOX (Tango3)
 * Bit 48/50: SBOX_MBUS_W2 (Tango3)
 */
#ifdef CONFIG_TANGO3
static const u64 sbox_reset_vals[4][6] = {
	{ 0x0000000001011010ULL, 0x0000000002021010ULL, 0x0000000010100101ULL, 0x0000000010100202ULL, 0x0101000000001010ULL, 0x0000010110100000ULL },
	{ 0x0000000001012020ULL, 0x0000000002022020ULL, 0x0000000020200101ULL, 0x0000000020200202ULL, 0x0101000000002020ULL, 0x0000010120200000ULL },
	{ 0x0000000001014040ULL, 0x0000000002024040ULL, 0x0000000040400101ULL, 0x0000000040400202ULL, 0x0101000000004040ULL, 0x0000010140400000ULL },
	{ 0x0000000001018080ULL, 0x0000000002028080ULL, 0x0000000080800101ULL, 0x0000000080800202ULL, 0x0101000000008080ULL, 0x0000010180800000ULL },
};
#else
static const unsigned int sbox_reset_vals[2][4] = {
	{ 0x01012020, 0x02022020, 0x20200101, 0x20200202 },
	{ 0x01014040, 0x02024040, 0x40400101, 0x40400202 }
};
#endif

/*
 * clear MBUS transaction for given regbase/sbox
 */
static void mbus_reset(unsigned int regbase, int sbox)
{
	int midx;
	int sidx;

#ifdef CONFIG_TANGO3
	unsigned int rl, rh;

	midx = (regbase - (REG_BASE_host_interface + MIF_W0_ADD)) / 0x40;
	sidx = sbox - SBOX_SATA1;

	if (((midx < 0) || (midx > 5)) || ((sidx < 0) || (sidx > 3))) {
		printk("MBUS reset: out of range, midx %d, sidx %d\n",
		       midx, sidx);
		return;
	}
	rl = sbox_reset_vals[sidx][midx] & 0xffffffff;
	rh = (sbox_reset_vals[sidx][midx] >> 32) & 0xffffffff;

	gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET, rl);
	if (rh) 
		gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET2, rh);

	gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET, rl & 0xff00ff00);
	if (rh) 
		gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET2, rh & 0xff00ff00);
#else
	midx = (regbase - (REG_BASE_host_interface + MIF_W0_ADD)) / 0x40;
	sidx = sbox - SBOX_IDEFLASH;

	if (((midx < 0) || (midx > 3)) || ((sidx < 0) || (sidx > 2))) {
		printk("MBUS reset: out of range, midx %d, sidx %d\n",
		       midx, sidx);
		return;
	}

	gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET,
		    sbox_reset_vals[sidx][midx]);
	gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET,
		    sbox_reset_vals[sidx][midx] & 0xff00ff00);
#endif
}

/* Fancy version of memcpy, both dst and src need to be physical address */
/* The channels have to be allocated already */
int mbus_memcpy(unsigned int regbase, unsigned int dst, unsigned int src, unsigned int size)
{
	/* Save the old SBOX route */
	unsigned int sbox_route;
	unsigned int w_base;
	unsigned int r_base;
	int i;
	unsigned int tmode_w, tmode_r;
#ifdef CONFIG_TANGO3
	int channel = 0;
#endif
/* 
 * TRANSFER defines 4 bits, bit 0: followed by void (1) or not (0),
 * bit 1: tiled buffer or not (tango3 only).
 * bit 3-2: 0 = 8 bit, 1 = 16 bit, 2 = 32 bit (tango3 only).
 */
#ifdef CONFIG_TANGO3
#define TRANSFER    0x1 /* or 0x9 for 32 bit transfer */
#else
#define TRANSFER    0x1
#endif
#define MBUS_MEMCPY_TIMEOUT	100	/* 100 usec */

	tmode_w = tmode_r = TRANSFER;

#ifdef CONFIG_TANGO3
	channel = mbus_idx2channel(regbase);

	if (channel == 0) /* W0/R0 channels are used */
#endif
	{
		w_base = REG_BASE_host_interface + MIF_W0_ADD;
		r_base = REG_BASE_host_interface + MIF_R0_ADD;
		sbox_route = gbus_read_reg32(REG_BASE_host_interface + SBOX_ROUTE) & 0xffffff0f;

		/* Hook up W0/R0 and left W1/R1 the same as before */
		gbus_write_reg32(REG_BASE_host_interface + SBOX_ROUTE, 0xffffff01);
	
		if (em86xx_mbus_setup_dma(w_base, dst, size, NULL, NULL, tmode_w) != 0) {
			gbus_write_reg32(REG_BASE_host_interface + SBOX_ROUTE, sbox_route);
			goto error;
		} else if (em86xx_mbus_setup_dma(r_base, src, size, NULL, NULL, tmode_r) != 0) {
			/* resetting W0 */
			gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET, 0x01010000);
			gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET, 0x01000000);
			gbus_write_reg32(REG_BASE_host_interface + SBOX_ROUTE, sbox_route);
			goto error;
		}

		for (i = 0; (i < MBUS_MEMCPY_TIMEOUT) && ((mbus_inuse(r_base) != 0) || (mbus_inuse(w_base) != 0)); i++)
			udelay(1);
		if (mbus_inuse(w_base) != 0) {
			gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET, 0x01010000);
			gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET, 0x01000000);
		}
		if (mbus_inuse(r_base) != 0) {
			gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET, 0x00000101);
			gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET, 0x00000100);
		}

		/* Restore SBOX route once we're done */
		gbus_write_reg32(REG_BASE_host_interface + SBOX_ROUTE, sbox_route);
	}
#ifdef CONFIG_TANGO3
#ifdef WITH_MBUS_W1R1
	else if (channel == 1) { /* W1/R1 channels are used */
		w_base = REG_BASE_host_interface + MIF_W1_ADD;
		r_base = REG_BASE_host_interface + MIF_R1_ADD;
		sbox_route = gbus_read_reg32(REG_BASE_host_interface + SBOX_ROUTE) & 0xfffffff0;

		/* Hook up W1/R1 and left W0/R0 the same as before */
		gbus_write_reg32(REG_BASE_host_interface + SBOX_ROUTE, 0xffffff20);
	
		if (em86xx_mbus_setup_dma(w_base, dst, size, NULL, NULL, tmode_w) != 0) {
			gbus_write_reg32(REG_BASE_host_interface + SBOX_ROUTE, sbox_route);
			goto error;
		} else if (em86xx_mbus_setup_dma(r_base, src, size, NULL, NULL, tmode_r) != 0) {
			/* resetting W1 */
			gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET, 0x02020000);
			gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET, 0x02000000);
			gbus_write_reg32(REG_BASE_host_interface + SBOX_ROUTE, sbox_route);
			goto error;
		}

		for (i = 0; (i < MBUS_MEMCPY_TIMEOUT) && ((mbus_inuse(r_base) != 0) || (mbus_inuse(w_base) != 0)); i++)
			udelay(1);
		if (mbus_inuse(w_base) != 0) {
			gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET, 0x02020000);
			gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET, 0x02000000);
		}
		if (mbus_inuse(r_base) != 0) {
			gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET, 0x00000202);
			gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET, 0x00000200);
		}

		/* Restore SBOX route once we're done */
		gbus_write_reg32(REG_BASE_host_interface + SBOX_ROUTE, sbox_route);
	}
#endif
	else { /* channel == 2: W2/R2 channels are used */
		w_base = REG_BASE_host_interface + MIF_W2_ADD;
		r_base = REG_BASE_host_interface + MIF_R2_ADD;
		sbox_route = gbus_read_reg32(REG_BASE_host_interface + SBOX_ROUTE2);

		/* Hook up W2/R2 */
		gbus_write_reg32(REG_BASE_host_interface + SBOX_ROUTE2, 0xfffffff9);
	
		if (em86xx_mbus_setup_dma(w_base, dst, size, NULL, NULL, tmode_w) != 0) {
			gbus_write_reg32(REG_BASE_host_interface + SBOX_ROUTE2, sbox_route);
			goto error;
		} else if (em86xx_mbus_setup_dma(r_base, src, size, NULL, NULL, tmode_r) != 0) {
			/* resetting W2 */
			gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET2, 0x01010000);
			gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET2, 0x01000000);
			gbus_write_reg32(REG_BASE_host_interface + SBOX_ROUTE2, sbox_route);
			goto error;
		}

		for (i = 0; (i < MBUS_MEMCPY_TIMEOUT) && ((mbus_inuse(r_base) != 0) || (mbus_inuse(w_base) != 0)); i++)
			udelay(1);
		if (mbus_inuse(w_base) != 0) {
			gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET2, 0x01010000);
			gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET2, 0x01000000);
		}
		if (mbus_inuse(r_base) != 0) {
			gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET2, 0x00000101);
			gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET2, 0x00000100);
		}

		/* Restore SBOX route once we're done */
		gbus_write_reg32(REG_BASE_host_interface + SBOX_ROUTE2, sbox_route);
	}
#endif

	return(size);

error:
	return(0);
}

/*
 * busy wait  for current mbus transfer  to finish, will  not wait for
 * more than 20 ms. 0 is ok, 1 timeout, 2 for timeout + reset error.
 */
#define MBUS_TIMEOUT	20000

int em86xx_mbus_wait(unsigned int regbase, int sbox)
{
	int timeout;

	/* wait for mbus to be released */
	timeout = 0;
	do {
		if (!mbus_inuse(regbase))
			break;
		udelay(1);
		timeout++;
	} while (timeout < MBUS_TIMEOUT);

	if (timeout < MBUS_TIMEOUT ) {
		/* ok */
		if (sbox == SBOX_IDEFLASH){
                        int i;
			unsigned int pb_count = 0;

			pb_count = gbus_read_reg32(REG_BASE_host_interface + 
					       PB_automode_control) & 0xffff;

                        for (i = 0; pb_count && (i < MBUS_TIMEOUT); i++){
                                udelay(1);
				pb_count = gbus_read_reg32(REG_BASE_host_interface +
					       PB_automode_control) & 0xffff;
			}

                        if (i < MBUS_TIMEOUT) 
				return 0;

		} else
			return 0;
	}

	/* timeout, let's dump some registers ! */
        if (sbox == SBOX_IDEFLASH) {
  		printk("MBUS timeout : MBUS CMD = %ld, PB Automode = %08x\n",
                	(unsigned long)gbus_read_reg32(regbase + MIF_cmd_offset) & 0x7,
                	(unsigned int)gbus_read_reg32(REG_BASE_host_interface + PB_automode_control));
        } else {
		printk("MBUS timeout : MBUS CMD = %08x\n",
			(u32)(gbus_read_reg32(regbase + MIF_cmd_offset) & 0x7));
	}

	printk("MBUS registers : %08x %08x %08x %08x\n",
	       (u32)gbus_read_reg32(regbase + MIF_add_offset),
	       (u32)gbus_read_reg32(regbase + MIF_cnt_offset),
	       (u32)gbus_read_reg32(regbase + MIF_add2_skip_offset),
	       (u32)gbus_read_reg32(regbase + MIF_cmd_offset));

	printk(KERN_ERR "MBUS fails, resetting %d ..\n", sbox);
	mbus_reset(regbase, sbox);

	/* If not able to reset, return  1, so the DMA can be disabled
	   accordingly  */
	return mbus_inuse(regbase) ? 0 : 1;
}

int em86xx_mbus_init(void)
{
	static int done = 0;

	if (done)
		return 0;
	done = 1;

	/* reset sbox to default values */
	sbox_init();

	/* Give better MBUS bandwidth for Wx/Rx channel */
#ifdef CONFIG_TANGO3
	gbus_write_reg32(REG_BASE_system_block + MARB_mid01_cfg, 0x12005);
	gbus_write_reg32(REG_BASE_system_block + MARB_mid21_cfg, 0x12005);
	gbus_write_reg32(REG_BASE_system_block + MARB_mid03_cfg, 0x12005);
	gbus_write_reg32(REG_BASE_system_block + MARB_mid23_cfg, 0x12005);
#ifdef WITH_MBUS_W1R1
	gbus_write_reg32(REG_BASE_system_block + MARB_mid02_cfg, 0x12005);
	gbus_write_reg32(REG_BASE_system_block + MARB_mid22_cfg, 0x12005);
#endif
#else
	gbus_write_reg32(REG_BASE_system_block + MARB_mid02_cfg, 0x11f1f);
	gbus_write_reg32(REG_BASE_system_block + MARB_mid22_cfg, 0x11f1f);
#endif

	return 0;
}

EXPORT_SYMBOL(em86xx_mbus_alloc_dma);
EXPORT_SYMBOL(em86xx_mbus_free_dma);
EXPORT_SYMBOL(em86xx_mbus_setup_dma);
EXPORT_SYMBOL(em86xx_mbus_setup_dma_void);
EXPORT_SYMBOL(em86xx_mbus_wait);
EXPORT_SYMBOL(em86xx_mbus_init);
EXPORT_SYMBOL(mbus_setup_dma_linear);
EXPORT_SYMBOL(mbus_setup_dma_double);
EXPORT_SYMBOL(mbus_setup_dma_rectangle);
EXPORT_SYMBOL(mbus_memcpy);

__initcall(em86xx_mbus_init);

/* Comment this to use DMA-like memcpy */
#define WITH_ORIGINAL_MEMCPY

/* Comment this to use DMA-like copy_to*, copy_from* */
#define WITH_ORIGINAL_COPYUSER

/* Minimum size of DMA copy (less than this original ones to be used) */
#define MINIMUM_DMACPY_SIZE	(PAGE_SIZE >> 2)

/* As a replacement of memcpy, original memcpy is renamed to memcpy_original.
   Use DMA to do copy as much as possible. */
void *memcpy(void *__to, __const__ void *__from, size_t __n)
{
	extern void *memcpy_original(void *dest, const void *source, __kernel_size_t size);
#if defined(WITH_ORIGINAL_MEMCPY) || defined(CONFIG_SWAP)
	return memcpy_original(__to, __from, __n);
#else
	unsigned long virt_to = (unsigned long)__to;
	unsigned long virt_from = (unsigned long)__from;
	int len = __n;
	unsigned long flag;
	unsigned long mbus_reg = 0;

	// JFT, We could probably handle more cases
	if (__n < MINIMUM_DMACPY_SIZE || 
#ifdef CONFIG_TANGO2
			virt_from < (KSEG0 + MEM_BASE_dram_controller_0) || virt_to < (KSEG0 + MEM_BASE_dram_controller_0) 
#elif defined(CONFIG_TANGO3)
			virt_from < (KSEG0 + CPU_REMAP_SPACE) || virt_to < (KSEG0 + CPU_REMAP_SPACE)
#endif
			|| virt_from >= KSEG2 || virt_to >= KSEG2)
		return memcpy_original(__to, __from, __n);
#ifdef CONFIG_TANGO2
	else if (unlikely(((virt_from >= KSEG1) && (virt_from < (KSEG1 + MEM_BASE_dram_controller_0))) ||
			((virt_to >= KSEG1) && (virt_to < (KSEG1 + MEM_BASE_dram_controller_0)))))
		return memcpy_original(__to, __from, __n);
#elif defined(CONFIG_TANGO3)
	else if (unlikely(((virt_from >= KSEG1) && (virt_from < (KSEG1 + CPU_REMAP_SPACE))) ||
			((virt_to >= KSEG1) && (virt_to < (KSEG1 + CPU_REMAP_SPACE)))))
		return memcpy_original(__to, __from, __n);
#endif
	else if (em86xx_mbus_alloc_dma(SBOX_PCIMASTER, 1, &mbus_reg, NULL)) 
		return memcpy_original(__to, __from, __n);

	// If the virt_to is not aligned we need to flush the data before it
	if (likely(virt_to < KSEG1)) {
#ifdef CONFIG_TANGO2
		if (virt_to & 0xf)
			flush_dcache_line(virt_to & ~0xf);
#elif defined(CONFIG_TANGO3)
		if (virt_to & 0x1f)
			flush_dcache_line(virt_to & ~0x1f);
#endif
	}
	// If the virt_to + cu_len is not aligned we need to flush the data after it
	if (likely((virt_to + __n) < KSEG1)) {
#ifdef CONFIG_TANGO2
		if ((virt_to + __n) & 0xf)
			flush_dcache_line((virt_to + __n) & ~0xf);
#elif defined(CONFIG_TANGO3)
		if ((virt_to + __n) & 0x1f)
			flush_dcache_line((virt_to + __n) & ~0x1f);
#endif
	}

	local_irq_save(flag);

	if (likely(virt_from < KSEG1))
		blast_dcache_range(virt_from, virt_from + len);
	if (likely(virt_to < KSEG1))
		blast_inv_dcache_range(virt_to, virt_to + len);

	if (mbus_memcpy(mbus_reg, tangox_dma_address(CPHYSADDR(virt_to)), tangox_dma_address(CPHYSADDR(virt_from)), len) == 0) {
		printk("%s:%d: MBUS memcpy failure.\n", __FUNCTION__, __LINE__);
		local_irq_restore(flag);
		em86xx_mbus_free_dma(mbus_reg, SBOX_PCIMASTER);
		return memcpy_original(__to, __from, __n);
	}

	local_irq_restore(flag);
	em86xx_mbus_free_dma(mbus_reg, SBOX_PCIMASTER);
	return __to;
#endif /* WITH_ORIGINAL_MEMCPY || CONFIG_SWAP */
}

#define offset_into_page(x) ((x) & (PAGE_SIZE - 1))

size_t __invoke_copy_to_user_dma(void __user *__cu_to, const void *__cu_from, long __cu_len)
{
#if defined(WITH_ORIGINAL_COPYUSER) || defined(CONFIG_SWAP)
        return __invoke_copy_to_user(__cu_to, __cu_from, __cu_len);
#else
	unsigned long virt_to = (unsigned long)__cu_to;
	unsigned long virt_from = (unsigned long)__cu_from;
	int byte;
	unsigned long _n;
	unsigned long flag;
	unsigned long mbus_reg = 0;

	// JFT, note: this is very common case so make it first
	if (__cu_len < MINIMUM_DMACPY_SIZE) 
        	return __invoke_copy_to_user(__cu_to, __cu_from, __cu_len);
	else if ((virt_to >= KSEG0) || (virt_from >= KSEG2))
        	return __invoke_copy_to_user(__cu_to, __cu_from, __cu_len);
	else if (!(virt_addr_valid(__cu_from)) || !(virt_addr_valid(__cu_from + __cu_len))) 
		return __invoke_copy_to_user(__cu_to, __cu_from, __cu_len);
	else if (em86xx_mbus_alloc_dma(SBOX_PCIMASTER, 1, &mbus_reg, NULL)) 
		return __invoke_copy_to_user(__cu_to, __cu_from, __cu_len);

	// Verify all pages exist
	for (byte = 0; byte < __cu_len; byte += PAGE_SIZE)
		__put_user_check(0, (unsigned char *)(virt_to + byte), 1);
	__put_user_check(0, (unsigned char *)(virt_to + __cu_len - 1), 1);

	// If the virt_to is not aligned we need to flush the data before it
#ifdef CONFIG_TANGO2
	if (virt_to & 0xf)
		flush_dcache_line(virt_to & ~0xf);
#elif defined(CONFIG_TANGO3)
	if (virt_to & 0x1f)
		flush_dcache_line(virt_to & ~0x1f);
#endif
	// If the virt_to + cu_len is not aligned we need to flush the data after it
#ifdef CONFIG_TANGO2
	if ((virt_to + __cu_len) & 0xf)
		flush_dcache_line((virt_to + __cu_len) & ~0xf);
#elif defined(CONFIG_TANGO3)
	if ((virt_to + __cu_len) & 0x1f)
		flush_dcache_line((virt_to + __cu_len) & ~0x1f);
#endif

	local_irq_save(flag);

	for (byte = 0, _n = __cu_len; byte < __cu_len; ) {
        	int len = min(PAGE_SIZE - offset_into_page(virt_to + byte), _n);
		pgd_t *pgd;
		pud_t *pud;
		pmd_t *pmd;
		pte_t *pte;
		unsigned long pg_addr;
		unsigned long dma_to;

		pg_addr = (virt_to + byte) & PAGE_MASK; /* address of start page */

		if (pg_addr > TASK_SIZE)
			pgd = pgd_offset_k(pg_addr);
		else
			pgd = pgd_offset_gate(current->mm, pg_addr);
		BUG_ON(pgd_none(*pgd));
		pud = pud_offset(pgd, pg_addr);
		BUG_ON(pud_none(*pud));
		pmd = pmd_offset(pud, pg_addr);
		if (pmd_none(*pmd)) 
			goto error;
		pte = pte_offset_map(pmd, pg_addr);
		if (pte_none(*pte)) {
			pte_unmap(pte);
			goto error;
		}
		dma_to = (pte_val(*pte) & PAGE_MASK) + offset_into_page(virt_to + byte);
		pte_unmap(pte);

#ifdef CONFIG_TANGO2
		if (unlikely((dma_to < MEM_BASE_dram_controller_0) || (dma_to >= (KSEG1 - KSEG0)))) 
			goto error;
#elif defined(CONFIG_TANGO3)
		if (unlikely((dma_to < CPU_REMAP_SPACE) || (dma_to >= (KSEG1 - KSEG0)))) 
			goto error;
#endif

		blast_dcache_range(virt_from + byte, virt_from + byte + len);
		blast_inv_dcache_range(virt_to + byte, virt_to + byte + len);

		if (mbus_memcpy(mbus_reg, tangox_dma_address(dma_to), tangox_dma_address(CPHYSADDR(virt_from + byte)), len) == 0) {
			printk("%s:%d: MBUS memcpy failure.\n", __FUNCTION__, __LINE__);
			local_irq_restore(flag);
			em86xx_mbus_free_dma(mbus_reg, SBOX_PCIMASTER);
			return __invoke_copy_to_user(__cu_to, __cu_from, __cu_len);
		}

		byte += len;
        	_n -= len;
	}

	local_irq_restore(flag);
	em86xx_mbus_free_dma(mbus_reg, SBOX_PCIMASTER);
	return _n;

error:
	local_irq_restore(flag);
	if (mbus_reg)
		em86xx_mbus_free_dma(mbus_reg, SBOX_PCIMASTER);
	return _n;
#endif /* WITH_ORIGINAL_COPYUSER || CONFIG_SWAP */
}

size_t __invoke_copy_from_user_dma(void *__cu_to, const void __user *__cu_from, long __cu_len)
{
#if defined(WITH_ORIGINAL_COPYUSER) || defined(CONFIG_SWAP)
        return __invoke_copy_from_user(__cu_to, __cu_from, __cu_len);
#else
	unsigned long virt_to = (unsigned long)__cu_to;
	unsigned long virt_from = (unsigned long)__cu_from;
	int byte;
	unsigned long _n;
	unsigned long flag;
	unsigned char val;
	unsigned long mbus_reg = 0;

	// JFT, this is very common case so make it first
	if (__cu_len < MINIMUM_DMACPY_SIZE) 
		return __invoke_copy_from_user(__cu_to, __cu_from, __cu_len);
	else if ((virt_from >= KSEG0) || (virt_to >= KSEG2))
        	return __invoke_copy_from_user(__cu_to, __cu_from, __cu_len);
	else if (!(virt_addr_valid(__cu_to)) || !(virt_addr_valid(__cu_to + __cu_len))) 
		return __invoke_copy_from_user(__cu_to, __cu_from, __cu_len);
	else if (em86xx_mbus_alloc_dma(SBOX_PCIMASTER, 1, &mbus_reg, NULL)) 
		return __invoke_copy_from_user(__cu_to, __cu_from, __cu_len);

	// Verify all pages exist
	for (byte = 0; byte < __cu_len; byte += PAGE_SIZE) 
		__get_user_check(val, (unsigned char *)(virt_from + byte), 1);
	__get_user_check(val, (unsigned char *)(virt_from + __cu_len - 1), 1);

	// If the virt_to is not aligned we need to flush the data before it
#ifdef CONFIG_TANGO2
	if (virt_to & 0xf)
		flush_dcache_line(virt_to & ~0xf);
#elif defined(CONFIG_TANGO3)
	if (virt_to & 0x1f)
		flush_dcache_line(virt_to & ~0x1f);
#endif
	// If the virt_to + cu_len is not aligned we need to flush the data after it
#ifdef CONFIG_TANGO2
	if ((virt_to + __cu_len) & 0xf) 
		flush_dcache_line((virt_to + __cu_len) & ~0xf);
#elif defined(CONFIG_TANGO3)
	if ((virt_to + __cu_len) & 0x1f) 
		flush_dcache_line((virt_to + __cu_len) & ~0x1f);
#endif

	local_irq_save(flag);

	for (byte = 0, _n = __cu_len; byte < __cu_len; ) {
		int len = min(PAGE_SIZE - offset_into_page(virt_from + byte), _n);
		pgd_t *pgd;
		pud_t *pud;
		pmd_t *pmd;
		pte_t *pte;
		unsigned long pg_addr;
		unsigned long dma_from;

		pg_addr = (virt_from + byte) & PAGE_MASK; /* address of start page */

		if (pg_addr > TASK_SIZE)
			pgd = pgd_offset_k(pg_addr);
		else
			pgd = pgd_offset_gate(current->mm, pg_addr);
		BUG_ON(pgd_none(*pgd));
		pud = pud_offset(pgd, pg_addr);
		BUG_ON(pud_none(*pud));
		pmd = pmd_offset(pud, pg_addr);
		if (pmd_none(*pmd)) 
			goto error;
		pte = pte_offset_map(pmd, pg_addr);
		if (pte_none(*pte)) {
			pte_unmap(pte);
			goto error;
		}
		dma_from = (pte_val(*pte) & PAGE_MASK) + offset_into_page(virt_from + byte);
		pte_unmap(pte);

#ifdef CONFIG_TANGO2
		if (unlikely((dma_from < MEM_BASE_dram_controller_0) || (dma_from >= (KSEG1 - KSEG0)))) 
			goto error;
#elif defined(CONFIG_TANGO3)
		if (unlikely((dma_from < CPU_REMAP_SPACE) || (dma_from >= (KSEG1 - KSEG0)))) 
			goto error;
#endif
		blast_dcache_range(virt_from + byte, virt_from + byte + len);
		blast_inv_dcache_range(virt_to + byte, virt_to + byte + len);
	
		if (mbus_memcpy(mbus_reg, tangox_dma_address(CPHYSADDR(virt_to + byte)), tangox_dma_address(dma_from), len) == 0) {
			printk("%s:%d: MBUS memcpy failure.\n", __FUNCTION__, __LINE__);
			local_irq_restore(flag);
			em86xx_mbus_free_dma(mbus_reg, SBOX_PCIMASTER);
			return __invoke_copy_from_user(__cu_to, __cu_from, __cu_len);
		}

		byte += len;
        	_n -= len;
	}
	
	local_irq_restore(flag);
	em86xx_mbus_free_dma(mbus_reg, SBOX_PCIMASTER);
	return _n;

error:
	local_irq_restore(flag);
	if (mbus_reg)
		em86xx_mbus_free_dma(mbus_reg, SBOX_PCIMASTER);
	return _n;
#endif /* WITH_ORIGINAL_COPYUSER || CONFIG_SWAP */
}

EXPORT_SYMBOL(__invoke_copy_to_user_dma);
EXPORT_SYMBOL(__invoke_copy_from_user_dma);

