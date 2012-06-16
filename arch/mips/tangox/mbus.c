
/*********************************************************************
 Copyright (C) 2001-2011
 Sigma Designs, Inc. 

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.

 01/2009: Merged/adapted the codes from Jean-Francois Thibert (SageTV)
 *********************************************************************/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/sched.h>

#include <asm/uaccess.h>
#include <asm/r4kcache.h>
#include <linux/pagemap.h>
#include <linux/dma-mapping.h>

#include "setup.h"

#if !defined(CONFIG_TANGO2) && !defined(CONFIG_TANGO3) && !defined(CONFIG_TANGO4)
#error Undefined Sigma chip!!!
#endif

//#define MBUS_VERBOSE

#ifdef MBUS_VERBOSE
#define MSG	printk
#else
#define MSG(x, ...)
#endif

/*
 * computed in prom.c
 */
extern unsigned long em8xxx_kmem_start;
extern unsigned long em8xxx_kmem_size;

/* uncomment this only if W1/R1 can be used (typically not) */
// #define WITH_MBUS_W1R1

/* put requester into wait queue if no channel pair available */
#define MBUS_WAIT_QUEUE

#ifdef MBUS_WAIT_QUEUE
static DECLARE_WAIT_QUEUE_HEAD(mbus_wq);
#endif

/* max size for linear transfer */
#define MBUS_LINEAR_MAX		(0x2000 - 1)

/* the number of available channel pairs */
#if defined(CONFIG_TANGO3) || defined(CONFIG_TANGO4)
#define NUM_MBUS_CHNPS	3
#else
#define NUM_MBUS_CHNPS	2
#endif /* CONFIG_TANGO3 || CONFIG_TANGO4 */

static DEFINE_SPINLOCK(mbus_lock); /* for controlling alloc/free of channel pair */
static int mbus_free_cnt = 0; /* counter of available channel pairs */

struct mbus_channel_pair
{
	unsigned int rx_base;	/* Rx base address */
	unsigned int wx_base;	/* Wx base address */
	const unsigned int shift;
	const unsigned int rx;
	const unsigned int idx;
	const unsigned int rx_irq;
	const unsigned int wx_irq;
	const char *rx_irq_name;
	const char *wx_irq_name;
	spinlock_t lock;		/* spin_lock */
	mbus_irq_handler_t handler;	
	void *arg;
	int iface;			/* which interface it's connected */
	int fromdev;
};

static struct mbus_channel_pair mchnp_list[NUM_MBUS_CHNPS] = {
	{ 
		.fromdev = 0,
		.rx_base = REG_BASE_host_interface + MIF_R0_ADD,
		.wx_base = REG_BASE_host_interface + MIF_W0_ADD, 
		.handler = NULL,
		.arg = NULL,
		.iface = 0xf,
		.shift = 0,
		.rx = 1,
		.idx = 0,
#ifdef CONFIG_TANGO4
		.rx_irq = LOG2_HOST_MBUS_R0_INT,
		.wx_irq = LOG2_HOST_MBUS_W0_INT,
#else
		.rx_irq = LOG2_CPU_HOST_MBUS_R0_INT,
		.wx_irq = LOG2_CPU_HOST_MBUS_W0_INT,
#endif
		.rx_irq_name = "tangox_mbus_r0",
		.wx_irq_name = "tangox_mbus_w0",
	},
	{
		.fromdev = 0,
		.rx_base = REG_BASE_host_interface + MIF_R1_ADD,
		.wx_base = REG_BASE_host_interface + MIF_W1_ADD, 
		.handler = NULL,
		.arg = NULL,
#ifdef WITH_MBUS_W1R1
		.iface = 0xf,
#else
		.iface = -1,	/* cannot be allocated */
#endif
		.shift = 4,
		.rx = 2,
		.idx = 1,
#ifdef CONFIG_TANGO4
		.rx_irq = LOG2_HOST_MBUS_R1_INT,
		.wx_irq = LOG2_HOST_MBUS_W1_INT,
#else
		.rx_irq = LOG2_CPU_HOST_MBUS_R1_INT,
		.wx_irq = LOG2_CPU_HOST_MBUS_W1_INT,
#endif
		.rx_irq_name = "tangox_mbus_r1",
		.wx_irq_name = "tangox_mbus_w1",
	},
#if defined(CONFIG_TANGO3) || defined(CONFIG_TANGO4)
	{
		.fromdev = 0,
		.rx_base = REG_BASE_host_interface + MIF_R2_ADD,
		.wx_base = REG_BASE_host_interface + MIF_W2_ADD, 
		.handler = NULL,
		.arg = NULL,
		.iface = 0xf,
		.shift = 32,
		.rx = 9,
		.idx = 2,
#ifdef CONFIG_TANGO4
		.rx_irq = LOG2_HOST_MBUS_R2_INT,
		.wx_irq = LOG2_HOST_MBUS_W2_INT,
#else
		.rx_irq = LOG2_CPU_HOST_MBUS_R2_INT,
		.wx_irq = LOG2_CPU_HOST_MBUS_W2_INT,
#endif
		.rx_irq_name = "tangox_mbus_r2",
		.wx_irq_name = "tangox_mbus_w2",
	},
#endif
};

static inline int mbus_channel_rewire(struct mbus_channel_pair *chnpptr, int iface, int fromdev)
{
	u64 route = 0;
	int ret = -1;

	if (chnpptr->iface != 0xf) 
		goto done;

	chnpptr->iface = iface;	/* connected */
	chnpptr->fromdev = fromdev;
	route = ((u64)(iface + 1)) << chnpptr->shift;	/* hook up Wx */
	route |= (((u64)chnpptr->rx) << (iface * 4));	/* hook up Rx */
	wmb();
	MSG("(%d) %s:%d connect route=0x%llx\n", smp_processor_id(), __FILE__, __LINE__, route);
	gbus_write_reg32(REG_BASE_host_interface + SBOX_ROUTE, (u32)(route & 0xffffffff));
#if defined(CONFIG_TANGO3) || defined(CONFIG_TANGO4)
	gbus_write_reg32(REG_BASE_host_interface + SBOX_ROUTE2, (u32)((route >> 32) & 0xffffffff));
#endif
	iob();
	MSG("(%d) %s:%d allocate idx=%d\n", smp_processor_id(), __FILE__, __LINE__, chnpptr->idx);
	ret = 0;

done:
	return ret;
}

/*
 * given SBOX interface, find/connect one of the available MBUS channel pairs
 * (i.e. Wx/Rx) and return the channel pair.
 */
static struct mbus_channel_pair *mbus_channel_pair_alloc(int iface, int fromdev, int canwait)
{
	int i;
	unsigned long flags;
	struct mbus_channel_pair *ret = NULL, *chnpptr = NULL;
	static int order = 0;

	spin_lock_irqsave(&mbus_lock, flags);

	/* look for existed connection first */
	for (chnpptr = &mchnp_list[0], i = 0; i < NUM_MBUS_CHNPS; i++, chnpptr++) {
		if (chnpptr->iface == iface) { /* allocated to it already */
			chnpptr->fromdev = fromdev;
			ret = chnpptr;
			MSG("(%d) %s:%d allocated idx=%d\n", smp_processor_id(), __FILE__, __LINE__, chnpptr->idx);
			goto done;
		}
	}

	if (mbus_free_cnt == 0) { /* no channel pair available */
#ifdef MBUS_WAIT_QUEUE
		if (canwait) {
			while (mbus_free_cnt == 0) {
				spin_unlock_irqrestore(&mbus_lock, flags);
				if (wait_event_interruptible(mbus_wq, mbus_free_cnt != 0)) 
					return NULL; /* being interrupted */
				spin_lock_irqsave(&mbus_lock, flags);
			}
		} else 
#endif
			goto done;
	}
		
	/* no existed connection found, try forming new connection */
	if (order == 0) {
		for (chnpptr = &mchnp_list[0], i = 0; i < NUM_MBUS_CHNPS; i++, chnpptr++) {
			if (mbus_channel_rewire(chnpptr, iface, fromdev) == 0) {
				ret = chnpptr;
				break;
			}
		}
	} else {
		for (chnpptr = &mchnp_list[NUM_MBUS_CHNPS - 1], i = NUM_MBUS_CHNPS - 1; i >= 0; i--, chnpptr--) {
			if (mbus_channel_rewire(chnpptr, iface, fromdev) == 0) {
				ret = chnpptr;
				break;
			}
		}
	}
	if (ret) { /* new allocation is done */
		order = (order ? 0 : 1); /* swap order */
		--mbus_free_cnt;
		wmb();
	}

done:
	spin_unlock_irqrestore(&mbus_lock, flags);
	return ret;
}

/*
 * free up allocated channel pair based on given SBOX interface.
 */
static int mbus_channel_pair_free(int iface)
{
	int i, ret = -1;
	unsigned long flags;
	struct mbus_channel_pair *chnpptr = NULL;

	spin_lock_irqsave(&mbus_lock, flags);

	for (chnpptr = &mchnp_list[0], i = 0; i < NUM_MBUS_CHNPS; i++, chnpptr++) {
		if (chnpptr->iface == iface) { /* matched */
			u64 route = 0;
			chnpptr->fromdev = 0;
			chnpptr->iface = 0xf;	/* disconnected */
			route = ((u64)0xf) << chnpptr->shift;	/* hook up Wx */
			route |= (((u64)0xf) << (iface * 4));	/* hook up Rx */
			MSG("(%d) %s:%d disconnect route=0x%llx\n", smp_processor_id(), __FILE__, __LINE__, route);
			wmb();
			gbus_write_reg32(REG_BASE_host_interface + SBOX_ROUTE, (u32)(route & 0xffffffff));
#if defined(CONFIG_TANGO3) || defined(CONFIG_TANGO4)
			gbus_write_reg32(REG_BASE_host_interface + SBOX_ROUTE2, (u32)((route >> 32) & 0xffffffff));
#endif
			iob();
			MSG("(%d) %s:%d free idx=%d\n", smp_processor_id(), __FILE__, __LINE__, chnpptr->idx);
#ifdef CONFIG_TANGO2
			MSG("(%d) %s:%d route=0x%x\n", smp_processor_id(), __FILE__, __LINE__, gbus_read_reg32(REG_BASE_host_interface + SBOX_ROUTE));
#else
			MSG("(%d) %s:%d route=0x%x%08x\n", smp_processor_id(), __FILE__, __LINE__, gbus_read_reg32(REG_BASE_host_interface + SBOX_ROUTE2), gbus_read_reg32(REG_BASE_host_interface + SBOX_ROUTE));
#endif
			ret = 0;
			++mbus_free_cnt;
			wmb();
#ifdef MBUS_WAIT_QUEUE
			if (mbus_free_cnt == 1) /* one channel pair is available */
				wake_up_interruptible(&mbus_wq);
#endif
			break;
		}
	}

	spin_unlock_irqrestore(&mbus_lock, flags);
	BUG_ON(ret != 0);
	return ret;
}

static __init void sbox_init(void)
{
	/* Resetting all (or most) channels first */
	int i;
	for (i = 0; i < 2; i++) {
#if defined(CONFIG_TANGO3) || defined(CONFIG_TANGO4)
#ifdef WITH_MBUS_W1R1
		gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET, 0xffffffff);
		iob();
		gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET, 0xff00ff00);
#else
		/* Leave W1/R1 alone. */
		gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET, 0xfdfdfdfd);
		iob();
		gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET, 0xfd00fd00);
#endif
		gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET2, 0x03030303);
		iob();
		gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET2, 0x03000300);
#else
#ifdef WITH_MBUS_W1R1
		gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET, 0x7f7f7f7f);
		iob();
		gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET, 0x7f007f00);
#else
		/* Leave W1/R1 alone. */
		gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET, 0x7d7d7d7d);
		iob();
		gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET, 0x7d007d00);
#endif
#endif
		iob();
		udelay(2); /* at least 256 system cycles, 128MHz or above */
	}

	/* Disconnect all (or most) channels */
#if defined(CONFIG_TANGO4)
#ifdef WITH_MBUS_W1R1
	gbus_write_reg32(REG_BASE_host_interface + SBOX_ROUTE, 0xffffffff);
#else
	gbus_write_reg32(REG_BASE_host_interface + SBOX_ROUTE, 0xffffff0f);
#endif
	gbus_write_reg32(REG_BASE_host_interface + SBOX_ROUTE2, 0x000000ff);
#elif defined(CONFIG_TANGO3)
#ifdef WITH_MBUS_W1R1
	gbus_write_reg32(REG_BASE_host_interface + SBOX_ROUTE, 0xffff4fff);
#else
	gbus_write_reg32(REG_BASE_host_interface + SBOX_ROUTE, 0xffff4f0f);
#endif
	gbus_write_reg32(REG_BASE_host_interface + SBOX_ROUTE2, 0x000000ff);
#else
#ifdef WITH_MBUS_W1R1
	gbus_write_reg32(REG_BASE_host_interface + SBOX_ROUTE, 0xffff4fff);
#else
	gbus_write_reg32(REG_BASE_host_interface + SBOX_ROUTE, 0xffff4f0f);
#endif
#endif
	iob();
#ifdef CONFIG_TANGO2
	MSG("(%d) %s:%d route=0x%x\n", smp_processor_id(), __FILE__, __LINE__, gbus_read_reg32(REG_BASE_host_interface + SBOX_ROUTE));
#else
	MSG("(%d) %s:%d route=0x%x%08x\n", smp_processor_id(), __FILE__, __LINE__, gbus_read_reg32(REG_BASE_host_interface + SBOX_ROUTE2), gbus_read_reg32(REG_BASE_host_interface + SBOX_ROUTE));
#endif
}

/*
 * convert MBUS register address to channel pair
 */
#if defined(CONFIG_TANGO2) || defined(CONFIG_TANGO3)
/* old mbus semantics */
static struct mbus_channel_pair *__old_mbus_reg2chnp(unsigned int regbase)
{
	int idx = (regbase - (REG_BASE_host_interface + MIF_W0_ADD)) / 0x40;
#if defined(CONFIG_TANGO3)
	static const int chnps[6] = { 0, 1, 0, 1, 2, 2 };
#elif defined(CONFIG_TANGO2)
	static const int chnps[4] = { 0, 1, 0, 1 };
#endif
	return &mchnp_list[chnps[idx]];
}
#endif

#if defined(CONFIG_TANGO3) || defined(CONFIG_TANGO4)
static struct mbus_channel_pair *__new_mbus_reg2chnp(unsigned int regbase)
{
	int idx = (regbase - (REG_BASE_host_interface + MIF_W0_ADD)) / 0x40;
	static const int chnps[8] = { 0, 1, 2, 3, 0, 1, 2, 3 };
	BUG_ON(chnps[idx] > 2); /* No W3/R3 yet */
	return &mchnp_list[chnps[idx]];
}
#endif

static struct mbus_channel_pair *(*__mbus_reg2chnp)(unsigned int) = NULL;

/*
 * alloc mbus dma channels, need to be called before setup ...
 */
struct mbus_channel_pair *tangox_mbus_alloc_dma(int iface, int fromdev, int canwait)
{
	struct mbus_channel_pair *chnpptr = NULL;

	chnpptr = mbus_channel_pair_alloc(iface, fromdev, 
			(in_atomic() || in_interrupt()) ? 0 : canwait);

	return chnpptr;
}

/*
 * free mbus dma channels, need to be called after transfer is done ...
 */
void tangox_mbus_free_dma(struct mbus_channel_pair *chnpptr, int iface)
{
//	int tangox_mbus_wait(struct mbus_channel_pair *chnpptr, int iface);
//	tangox_mbus_wait(chnpptr, iface);

	if (mbus_channel_pair_free(iface) != 0) 
		BUG(); /* freeing non-allocated channels? */
}

/*
 * check if mbus is in use for given regbase
 */
static inline int __tangox_mbus_inuse(struct mbus_channel_pair *chnpptr, unsigned int rwbase)
{
	if (unlikely(chnpptr == NULL)) {
		MSG("(%d) %s:%d ...\n", smp_processor_id(), __FILE__, __LINE__);
		BUG();
	}
	if (rwbase)
		return gbus_read_reg32(rwbase + MIF_cmd_offset) & 0x7; 
	else
		return gbus_read_reg32((chnpptr->fromdev ? chnpptr->wx_base : chnpptr->rx_base) + MIF_cmd_offset) & 0x7; 
}

static inline int tangox_mbus_inuse(struct mbus_channel_pair *chnpptr)
{
	return __tangox_mbus_inuse(chnpptr, 0);
}

/*
 * irq handler for mbus interrupt
 */
static irqreturn_t tangox_mbus_intr(int irq, void *devinfo)
{
	struct mbus_channel_pair *chnpptr = (struct mbus_channel_pair *)devinfo;
	unsigned long flags;
	mbus_irq_handler_t f;
	void *arg = NULL;
	unsigned int rwbase;
	
	spin_lock_irqsave(&chnpptr->lock, flags);
#ifdef CONFIG_TANGO2
	MSG("(%d) %s:%d intr(%d) idx %d, iface %d, route=0x%x, handler=0x%p\n", smp_processor_id(), __FILE__, __LINE__, irq, chnpptr->idx, chnpptr->iface, gbus_read_reg32(REG_BASE_host_interface + SBOX_ROUTE), chnpptr->handler);
#else
	MSG("(%d) %s:%d intr(%d) idx %d, iface %d, route=0x%x%08x, handler=0x%p\n", smp_processor_id(), __FILE__, __LINE__, irq, chnpptr->idx, chnpptr->iface, gbus_read_reg32(REG_BASE_host_interface + SBOX_ROUTE2), gbus_read_reg32(REG_BASE_host_interface + SBOX_ROUTE), chnpptr->handler);
#endif

	rwbase = chnpptr->fromdev ? chnpptr->wx_base : chnpptr->rx_base;
	if (gbus_read_reg32(rwbase + MIF_cmd_offset) & 0x7)
		MSG("(%d) %s:%d, mbus busy (irq=%d, idx=%d) ...\n", smp_processor_id(), __FILE__, __LINE__, irq, chnpptr->idx);

	if ((f = chnpptr->handler) != NULL) {
		arg = chnpptr->arg;
		chnpptr->handler = chnpptr->arg = NULL;	/* one shot only */
		wmb();
	} 
	spin_unlock_irqrestore(&chnpptr->lock, flags);
	if (f != NULL)
		f(irq, arg);

	return IRQ_HANDLED;
}

/*
 * register mbus interrupt if not done
 */
static __init void mbus_register_intr(void)
{
	struct mbus_channel_pair *chnpptr = NULL;
	int i;

	for (chnpptr = &mchnp_list[0], i = 0; i < NUM_MBUS_CHNPS; i++, chnpptr++) {
		if (chnpptr->iface < 0) 
			continue;
		if (request_irq(chnpptr->wx_irq + IRQ_CONTROLLER_IRQ_BASE, tangox_mbus_intr, IRQF_DISABLED, chnpptr->wx_irq_name, chnpptr) != 0) 
			printk("MBUS: fail to register MBUS ISR(%d)\n", chnpptr->wx_irq + IRQ_CONTROLLER_IRQ_BASE);
		if (request_irq(chnpptr->rx_irq + IRQ_CONTROLLER_IRQ_BASE, tangox_mbus_intr, IRQF_DISABLED, chnpptr->rx_irq_name, chnpptr) != 0) 
			printk("MBUS: fail to register MBUS ISR(%d)\n", chnpptr->rx_irq + IRQ_CONTROLLER_IRQ_BASE);
	}
}

#if defined(CONFIG_TANGO2) || defined(CONFIG_TANGO3)
/* old mbus semantics */
static void __old_mbus_linear(unsigned int rwbase, unsigned int flags)
{
	gbus_write_reg32(rwbase + MIF_cmd_offset, (flags<<2)|0x1);
}

static void __old_mbus_double(unsigned int rwbase, unsigned int flags)
{
	gbus_write_reg32(rwbase + MIF_cmd_offset, (flags<<2)|0x2);
}

static void __old_mbus_rectangle(unsigned int rwbase, unsigned int flags)
{
	gbus_write_reg32(rwbase + MIF_cmd_offset, (flags<<2)|0x3);
}

static void __old_mbus_void(unsigned int rwbase)
{
	gbus_write_reg32(rwbase + MIF_cmd_offset, 1<<2);
}
#endif

#if defined(CONFIG_TANGO3) || defined(CONFIG_TANGO4)
/* new mbus semantics */
static void __new_mbus_linear(unsigned int rwbase, unsigned int flags)
{
	gbus_write_reg32(rwbase + MIF_cmd_offset, flags|0x2);
}

static void __new_mbus_double(unsigned int rwbase, unsigned int flags)
{
	gbus_write_reg32(rwbase + MIF_cmd_offset, flags|0x4);
}

static void __new_mbus_rectangle(unsigned int rwbase, unsigned int flags)
{
	gbus_write_reg32(rwbase + MIF_cmd_offset, flags|0x6);
}

static void __new_mbus_void(unsigned int rwbase)
{
	gbus_write_reg32(rwbase + MIF_cmd_offset, 1);
}
#endif

static void (*__mbus_linear_action)(unsigned int, unsigned int) = NULL;
static void (*__mbus_double_action)(unsigned int, unsigned int) = NULL;
static void (*__mbus_rectangle_action)(unsigned int, unsigned int) = NULL;
static void (*__mbus_void_action)(unsigned int) = NULL;

/*
 * setup mbus register to start a linear transfer (count bytes from
 * addr, where count < MBUS_LINEAR_MAX)
 */
static void __mbus_setup_dma_linear(struct mbus_channel_pair *chnpptr,
					unsigned int addr,
					unsigned int count,
					unsigned int flags,
					unsigned int regbase)
{
	unsigned int rwbase;
#if !defined(CONFIG_SD_DIRECT_DMA) && !defined(CONFIG_HIGHMEM)
	if ((tangox_inv_dma_address(addr) < CPHYSADDR(em8xxx_kmem_start)) || (tangox_inv_dma_address(addr) >= (CPHYSADDR(em8xxx_kmem_start) + em8xxx_kmem_size)))
		printk("MBUS Warning (linear): bad transfer address 0x%08x\n", addr);
#endif
	if (unlikely(chnpptr == NULL)) {
		MSG("(%d) %s:%d ...\n", smp_processor_id(), __FILE__, __LINE__);
		BUG();
	}

#ifdef CONFIG_TANGO2
	MSG("(%d) %s:%d setup_dma_linear idx=%d, route=0x%x, flags=%d\n", smp_processor_id(), __FILE__, __LINE__, gbus_read_reg32(REG_BASE_host_interface + SBOX_ROUTE), flags);
#else
	MSG("(%d) %s:%d setup_dma_linear idx=%d, route=0x%x%08x, flags=%d\n", smp_processor_id(), __FILE__, __LINE__, chnpptr->idx, gbus_read_reg32(REG_BASE_host_interface + SBOX_ROUTE2), gbus_read_reg32(REG_BASE_host_interface + SBOX_ROUTE), flags);
#endif
	rwbase = regbase ? regbase : (chnpptr->fromdev ? chnpptr->wx_base : chnpptr->rx_base);
	gbus_write_reg32(rwbase + MIF_add_offset, addr);
	gbus_write_reg32(rwbase + MIF_cnt_offset, count);
	iob();
	(*__mbus_linear_action)(rwbase, flags);
	iob();
}

/*
 * setup mbus register to start a double transfer (count bytes from
 * addr and count2 bytes from addr2, where count < MBUS_LINEAR_MAX and
 * count2 < MBUS_LINEAR_MAX)
 */
static void __mbus_setup_dma_double(struct mbus_channel_pair *chnpptr,
					unsigned int addr,
					unsigned int count,
					unsigned int addr2,
					unsigned int count2,
					unsigned int flags,
					unsigned int regbase)
{
	unsigned int rwbase;
#if !defined(CONFIG_SD_DIRECT_DMA) && !defined(CONFIG_HIGHMEM)
	if ((tangox_inv_dma_address(addr) < CPHYSADDR(em8xxx_kmem_start)) || (tangox_inv_dma_address(addr) >= (CPHYSADDR(em8xxx_kmem_start) + em8xxx_kmem_size)))
		printk("MBUS Warning (double): bad transfer address 0x%08x\n", addr);
	if ((tangox_inv_dma_address(addr2) < CPHYSADDR(em8xxx_kmem_start)) || (tangox_inv_dma_address(addr2) >= (CPHYSADDR(em8xxx_kmem_start) + em8xxx_kmem_size)))
		printk("MBUS Warning (double): bad transfer address2 0x%08x\n", addr2);
#endif
	if (unlikely(chnpptr == NULL)) {
		MSG("(%d) %s:%d ...\n", smp_processor_id(), __FILE__, __LINE__);
		BUG();
	}

#ifdef CONFIG_TANGO2
	MSG("(%d) %s:%d setup_dma_dbl idx=%d, route=0x%x, flags=%d\n", smp_processor_id(), __FILE__, __LINE__, chnpptr->idx, gbus_read_reg32(REG_BASE_host_interface + SBOX_ROUTE), flags);
#else
	MSG("(%d) %s:%d setup_dma_dbl idx=%d, route=0x%x%08x, flags=%d\n", smp_processor_id(), __FILE__, __LINE__, chnpptr->idx, gbus_read_reg32(REG_BASE_host_interface + SBOX_ROUTE2), gbus_read_reg32(REG_BASE_host_interface + SBOX_ROUTE), flags);
#endif
	rwbase = regbase ? regbase : (chnpptr->fromdev ? chnpptr->wx_base : chnpptr->rx_base);
	gbus_write_reg32(rwbase + MIF_add_offset, addr);
	gbus_write_reg32(rwbase + MIF_cnt_offset, (count2 << 16) | count);
	gbus_write_reg32(rwbase + MIF_add2_skip_offset, addr2);
	iob();
	(*__mbus_double_action)(rwbase, flags);
	iob();
}

/*
 * setup mbus register to start a rectangle transfer (horiz * lines
 * bytes from addr, where horiz < MBUS_LINEAR_MAX and lines  <
 * MBUS_LINEAR_MAX)
 */
static void __mbus_setup_dma_rectangle(struct mbus_channel_pair *chnpptr,
					unsigned int addr,
					unsigned int horiz,
					unsigned int lines,
					unsigned int flags,
					unsigned int regbase)
{
	unsigned int rwbase;
#if !defined(CONFIG_SD_DIRECT_DMA) && !defined(CONFIG_HIGHMEM)
	if ((tangox_inv_dma_address(addr) < CPHYSADDR(em8xxx_kmem_start)) || (tangox_inv_dma_address(addr) >= (CPHYSADDR(em8xxx_kmem_start) + em8xxx_kmem_size)))
		printk("MBUS Warning (rectangle): bad transfer address 0x%08x\n", addr);
#endif
	if (unlikely(chnpptr == NULL)) {
		MSG("(%d) %s:%d ...\n", smp_processor_id(), __FILE__, __LINE__);
		BUG();
	}

#ifdef CONFIG_TANGO2
	MSG("(%d) %s:%d setup_dma_rect idx=%d, route=0x%x, flags=%d\n", smp_processor_id(), __FILE__, __LINE__, chnpptr->idx, gbus_read_reg32(REG_BASE_host_interface + SBOX_ROUTE), flags);
#else
	MSG("(%d) %s:%d setup_dma_rect idx=%d, route=0x%x%08x, flags=%d\n", smp_processor_id(), __FILE__, __LINE__, chnpptr->idx, gbus_read_reg32(REG_BASE_host_interface + SBOX_ROUTE2), gbus_read_reg32(REG_BASE_host_interface + SBOX_ROUTE), flags);
#endif
	rwbase = regbase ? regbase : (chnpptr->fromdev ? chnpptr->wx_base : chnpptr->rx_base);
	gbus_write_reg32(rwbase + MIF_add_offset, addr);
	gbus_write_reg32(rwbase + MIF_cnt_offset, (lines << 16) | horiz);
	gbus_write_reg32(rwbase + MIF_add2_skip_offset, horiz);
	iob();
	(*__mbus_rectangle_action)(rwbase, flags);
	iob();
}

/*
 * setup void transaction 
 */
static void __mbus_setup_dma_void(struct mbus_channel_pair *chnpptr,
					unsigned int regbase)
{
	unsigned int rwbase;

	if (unlikely(chnpptr == NULL)) {
		MSG("(%d) %s:%d ...\n", smp_processor_id(), __FILE__, __LINE__);
		BUG();
	}

#ifdef CONFIG_TANGO2
	MSG("(%d) %s:%d setup_dma_void idx=%d, route=0x%x\n", smp_processor_id(), __FILE__, __LINE__, chnpptr->idx, gbus_read_reg32(REG_BASE_host_interface + SBOX_ROUTE));
#else
	MSG("(%d) %s:%d setup_dma_void idx=%d, route=0x%x%08x\n", smp_processor_id(), __FILE__, __LINE__, chnpptr->idx, gbus_read_reg32(REG_BASE_host_interface + SBOX_ROUTE2), gbus_read_reg32(REG_BASE_host_interface + SBOX_ROUTE));
#endif
	rwbase = regbase ? regbase : (chnpptr->fromdev ? chnpptr->wx_base : chnpptr->rx_base);
	(*__mbus_void_action)(rwbase);
	iob();
}

/* 
 * register a mbus interrupt handler, the caller is responsible to start the MBUS 
 * transaction itself, it'd better to register notification before startng the transfer
 */
int tangox_mbus_notification(struct mbus_channel_pair *chnpptr, mbus_irq_handler_t handler, void *arg)
{
	unsigned long flgs;

	if (unlikely(chnpptr == NULL)) {
		MSG("(%d) %s:%d ...\n", smp_processor_id(), __FILE__, __LINE__);
		BUG();
	}
	spin_lock_irqsave(&chnpptr->lock, flgs);
	chnpptr->handler = handler;
	chnpptr->arg = arg;
	wmb();
	spin_unlock_irqrestore(&chnpptr->lock, flgs);
	return 0;
}

/*
 * start a mbus dma, use this after a sucessfull call to
 * tangox_mbus_alloc_dma
 */
static int __tangox_mbus_setup_dma(struct mbus_channel_pair *chnpptr,unsigned int addr,
					unsigned int count, mbus_irq_handler_t handler,
					void *arg, unsigned int tflags, unsigned int regbase)
{
	unsigned long flgs;
	unsigned int horiz, lines, sz;
	int idx;

	if (unlikely(chnpptr == NULL)) {
		MSG("(%d) %s:%d ...\n", smp_processor_id(), __FILE__, __LINE__);
		BUG();
	}

	/*
	 * make sure no one uses the mbus before
	 */
	if (unlikely((sz = __tangox_mbus_inuse(chnpptr, regbase)) != 0)) {
		printk(KERN_ERR "MBUS: error previous command is pending (%d:%d) ...\n", chnpptr->idx, sz);
		return 1;
	}

	/* registering given handler */
	spin_lock_irqsave(&chnpptr->lock, flgs);
	chnpptr->handler = handler;
	chnpptr->arg = arg;
	wmb();
	MSG("(%d) %s:%d setup_dma idx=%d\n", smp_processor_id(), __FILE__, __LINE__, chnpptr->idx);
	spin_unlock_irqrestore(&chnpptr->lock, flgs);

	/*
	 * decide which dma function to use depending on count
	 */
	if (count == 0) {
		__mbus_setup_dma_void(chnpptr, regbase);
	} else if (count <= MBUS_LINEAR_MAX) {
		__mbus_setup_dma_linear(chnpptr, addr, count, tflags, regbase);
	} else if (count <= (MBUS_LINEAR_MAX * 2)) {
		__mbus_setup_dma_double(chnpptr, addr, MBUS_LINEAR_MAX,
				      addr + MBUS_LINEAR_MAX,
				      count - MBUS_LINEAR_MAX, tflags, regbase);
	} else {
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
			return 1;
		}
		__mbus_setup_dma_rectangle(chnpptr, addr, horiz, lines, tflags, regbase);
	}
	return 0;
}

int tangox_mbus_setup_dma(struct mbus_channel_pair *chnpptr,unsigned int addr,
					unsigned int count, mbus_irq_handler_t handler,
					void *arg, unsigned int tflags)
{
	return __tangox_mbus_setup_dma(chnpptr, addr, count, handler, arg, tflags, 0);
}

EXPORT_SYMBOL(tangox_mbus_alloc_dma);
EXPORT_SYMBOL(tangox_mbus_free_dma);
EXPORT_SYMBOL(tangox_mbus_notification);
EXPORT_SYMBOL(tangox_mbus_setup_dma);

/*
 * Bit 0/8: MBUS_R0_SBOX
 * Bit 1/9: MBUS_R1_SBOX
 * Bit 2/10: PCI_MASTER_SBOX
 * Bit 3/11: PCI_SLAVE_SBOX
 * Bit 4/12: SATA0_SBOX
 * Bit 5/13: IDE_ISA_SBOX
 * Bit 6/14: IDE_DVD_SBOX
 * Bit 7/15: SATA1_SBOX (Tango3)
 * Bit 16/24: SBOX_MBUS_W0
 * Bit 17/25: SBOX_MBUS_W1
 * Bit 18/26: SBOX_PCI_MASTER
 * Bit 19/27: SBOX_PCI_SLAVE
 * Bit 20/28: SBOX_SATA0
 * Bit 21/29: SBOX_ISA
 * Bit 22/30: SBOX_DVD
 * Bit 23/31: SBOX_SATA1 (Tango3)
 *
 * Bit 32/40: MBUS_R2_SBOX (Tango3)
 * Bit 48/50: SBOX_MBUS_W2 (Tango3)
 */
#if defined(CONFIG_TANGO3)
static const u64 sbox_reset_vals[4][6] = {
	{ 0x0000000001011010ULL, 0x0000000002021010ULL, 0x0000000010100101ULL, 0x0000000010100202ULL, 0x0101000000001010ULL, 0x0000010110100000ULL },
	{ 0x0000000001012020ULL, 0x0000000002022020ULL, 0x0000000020200101ULL, 0x0000000020200202ULL, 0x0101000000002020ULL, 0x0000010120200000ULL },
	{ 0x0000000001014040ULL, 0x0000000002024040ULL, 0x0000000040400101ULL, 0x0000000040400202ULL, 0x0101000000004040ULL, 0x0000010140400000ULL },
	{ 0x0000000001018080ULL, 0x0000000002028080ULL, 0x0000000080800101ULL, 0x0000000080800202ULL, 0x0101000000008080ULL, 0x0000010180800000ULL },
};
#elif defined(CONFIG_TANGO4)
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

#if defined(CONFIG_TANGO2) || defined(CONFIG_TANGO3)
/* old mbus semantics */
static int __old_mbus_idx2chn(unsigned int idx)
{
	return idx; /* no conversion needed */
}
#endif

#if defined(CONFIG_TANGO3) || defined(CONFIG_TANGO4)
static int __new_mbus_idx2chn(unsigned int idx)
{
	/* converting w0,w1,w2,hole,r0,r1,r2,hole to w0,w1,r0,r1,w2,r2 index */
	static const int chn_conv[8] = { 0, 1, 4, -1, 2, 3, 5, -1 };
	return chn_conv[idx];
}
#endif

static int (*__mbus_idx2chn)(unsigned int) = NULL;

/*
 * clear MBUS transaction for given regbase/sbox
 */
void tangox_mbus_reset(struct mbus_channel_pair *chnpptr, int iface)
{
	int midx;
	int sidx;
	unsigned int rwbase;

#if defined(CONFIG_TANGO3) || defined(CONFIG_TANGO4)
	unsigned int rl, rh;

	/* MBUS is idle, no need to reset */
	if (tangox_mbus_inuse(chnpptr) == 0)
		return;
	
	rwbase = chnpptr->fromdev ? chnpptr->wx_base : chnpptr->rx_base;
	midx = (*__mbus_idx2chn)((rwbase - (REG_BASE_host_interface + MIF_W0_ADD)) / 0x40);
	sidx = iface - SBOX_SATA0;

	if (((midx < 0) || (midx > 5)) || ((sidx < 0) || (sidx > 3))) {
		printk("MBUS reset: out of range, midx %d, sidx %d\n", midx, sidx);
		return;
	}
	rl = sbox_reset_vals[sidx][midx] & 0xffffffff;
	rh = (sbox_reset_vals[sidx][midx] >> 32) & 0xffffffff;
	wmb();

	gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET, rl);
	if (rh) 
		gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET2, rh);
	iob();

	gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET, rl & 0xff00ff00);
	if (rh) 
		gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET2, rh & 0xff00ff00);
	iob();

	udelay(2); /* at least 256 system cycles, 128MHz or above */
	gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET, rl);
	if (rh) 
		gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET2, rh);
	iob();

	gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET, rl & 0xff00ff00);
	if (rh) 
		gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET2, rh & 0xff00ff00);
	iob();
#else
	/* MBUS is idle, no need to reset */
	if (tangox_mbus_inuse(chnpptr) == 0)
		return;

	rwbase = chnpptr->fromdev ? chnpptr->wx_base : chnpptr->rx_base;
	midx = (*__mbus_idx2chn)((rwbase - (REG_BASE_host_interface + MIF_W0_ADD)) / 0x40);
	sidx = iface - SBOX_IDEFLASH;

	if (((midx < 0) || (midx > 3)) || ((sidx < 0) || (sidx > 2))) {
		printk("MBUS reset: out of range, midx %d, sidx %d\n", midx, sidx);
		return;
	}

	gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET, sbox_reset_vals[sidx][midx]);
	gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET, sbox_reset_vals[sidx][midx] & 0xff00ff00);

	udelay(2); /* at least 256 system cycles, 128MHz or above */
	gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET, sbox_reset_vals[sidx][midx]);
	gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET, sbox_reset_vals[sidx][midx] & 0xff00ff00);
#endif
}
EXPORT_SYMBOL(tangox_mbus_reset);

typedef int (*COND_FUNC_PTR)(void *);
extern int tangox_udelay_with_condition(unsigned int usecond, COND_FUNC_PTR func_ptr, void *arg);

static int tangox_mbus_busy(void *arg)
{
	struct mbus_channel_pair *chnpptr = (struct mbus_channel_pair *)arg;
	return tangox_mbus_inuse(chnpptr);
}

static int tangox_pb_busy(void *dummy)
{
	return (gbus_read_reg32(REG_BASE_host_interface + PB_automode_control) & 0xffff) ? 1 : 0;
}

/*
 * busy wait for current mbus transfer to finish, will not wait for
 * more than 20 ms.
 */
#define MBUS_TIMEOUT	20000

int tangox_mbus_wait(struct mbus_channel_pair *chnpptr, int iface)
{
	int timeout;
	unsigned int rwbase;

	/* wait for mbus to be released */
	timeout = tangox_udelay_with_condition(MBUS_TIMEOUT, tangox_mbus_busy, (void *)chnpptr);
	if (timeout == 0) {
		/* ok */
		if (iface == SBOX_IDEFLASH) {
			if ((timeout = tangox_udelay_with_condition(MBUS_TIMEOUT, tangox_pb_busy, NULL)) == 0)
				return 0;
		} else
			return 0;
	}

	if (tangox_mbus_inuse(chnpptr) == 0)
		return 0; /* mbus is done */

	rwbase = chnpptr->fromdev ? chnpptr->wx_base : chnpptr->rx_base;
	/* timeout, let's dump some registers ! */
        if (iface == SBOX_IDEFLASH) {
  		printk("MBUS timeout : MBUS CMD = %ld, PB Automode = %08x\n",
                	(unsigned long)gbus_read_reg32(rwbase + MIF_cmd_offset) & 0x7,
                	(unsigned int)gbus_read_reg32(REG_BASE_host_interface + PB_automode_control));
        } else {
		printk("MBUS timeout : MBUS CMD = %08x\n",
			(u32)(gbus_read_reg32(rwbase + MIF_cmd_offset) & 0x7));
	}

	printk("MBUS registers : %08x %08x %08x %08x\n",
	       (u32)gbus_read_reg32(rwbase + MIF_add_offset),
	       (u32)gbus_read_reg32(rwbase + MIF_cnt_offset),
	       (u32)gbus_read_reg32(rwbase + MIF_add2_skip_offset),
	       (u32)gbus_read_reg32(rwbase + MIF_cmd_offset));

	printk(KERN_ERR "MBUS fails, resetting %d ..\n", iface);
	tangox_mbus_reset(chnpptr, iface);

	/* If not able to reset, return  1, so the DMA can be disabled
	   accordingly  */
	return tangox_mbus_inuse(chnpptr) ? 0 : 1;
}
EXPORT_SYMBOL(tangox_mbus_wait);

static __init int tangox_mbus_init(void)
{
	unsigned long tangox_chip_id(void);
	struct mbus_channel_pair *chnpptr = NULL;
	unsigned long flags;
	int i;

	/* give better MBUS bandwidth for Wx/Rx channels */
#if defined(CONFIG_TANGO3) || defined(CONFIG_TANGO4)
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

	spin_lock_irqsave(&mbus_lock, flags);

	/* how many channel pairs are available? */
	for (chnpptr = &mchnp_list[0], mbus_free_cnt = i = 0; i < NUM_MBUS_CHNPS; i++, chnpptr++) {
		spin_lock_init(&chnpptr->lock);
		if (chnpptr->iface == 0xf)
			mbus_free_cnt++; /* count the number of available channel pairs */
	}

	/* reset sbox to default values */
	sbox_init();

#ifdef CONFIG_TANGO4
	__mbus_void_action = __new_mbus_void;
	__mbus_linear_action = __new_mbus_linear;
	__mbus_double_action = __new_mbus_double;
	__mbus_rectangle_action = __new_mbus_rectangle;
	__mbus_idx2chn = __new_mbus_idx2chn;
	__mbus_reg2chnp = __new_mbus_reg2chnp;
#else
#ifdef CONFIG_TANGO3
	if ((((tangox_chip_id() >> 16) & 0xfffe) == 0x8672) || (((tangox_chip_id() >> 16) & 0xfffe) == 0x8674)) { /* 8672/8674 uses new semantics */
		__mbus_void_action = __new_mbus_void;
		__mbus_linear_action = __new_mbus_linear;
		__mbus_double_action = __new_mbus_double;
		__mbus_rectangle_action = __new_mbus_rectangle;
		__mbus_idx2chn = __new_mbus_idx2chn;
		__mbus_reg2chnp = __new_mbus_reg2chnp;
		/* mbus registers for 8672/8674 is the same as TANGO4 ... */
		mchnp_list[0].rx_base = REG_BASE_host_interface + 0xb100;
		mchnp_list[0].wx_base = REG_BASE_host_interface + 0xb000;
		mchnp_list[1].rx_base = REG_BASE_host_interface + 0xb140;
		mchnp_list[1].wx_base = REG_BASE_host_interface + 0xb040;
		mchnp_list[2].rx_base = REG_BASE_host_interface + 0xb180;
		mchnp_list[2].wx_base = REG_BASE_host_interface + 0xb080;
	} else {
#endif
		__mbus_void_action = __old_mbus_void;
		__mbus_linear_action = __old_mbus_linear;
		__mbus_double_action = __old_mbus_double;
		__mbus_rectangle_action = __old_mbus_rectangle;
		__mbus_idx2chn = __old_mbus_idx2chn;
		__mbus_reg2chnp = __old_mbus_reg2chnp;
#ifdef CONFIG_TANGO3
	}
#endif
#endif

	printk("registering mbus interrupt routines.\n");
	mbus_register_intr();

	spin_unlock_irqrestore(&mbus_lock, flags);

	return 0;
}

/* TODO: TO BE RELPLACED */
static inline int mbus_inuse(unsigned int regbase)
{
	struct mbus_channel_pair *chnpptr = (*__mbus_reg2chnp)(regbase);
	if (regbase == 0) {
		MSG("%s:%d regbase=0x%x\n", __FILE__, __LINE__, regbase);
		BUG();
	} else if (chnpptr == NULL) {
		MSG("%s:%d regbase=0x%x\n", __FILE__, __LINE__, regbase);
		BUG();
	} else if ((regbase != chnpptr->wx_base) && (regbase != chnpptr->rx_base)) {
		MSG("%s:%d regbase=0x%x\n", __FILE__, __LINE__, regbase);
		BUG();
	}
	return tangox_mbus_inuse(chnpptr);
}

/* TODO: TO BE RELPLACED */
int em86xx_mbus_alloc_dma(int iface, int fromdev, unsigned long *pregbase, int *pirq, int canwait)
{
	struct mbus_channel_pair *chnpptr;
	if ((chnpptr = tangox_mbus_alloc_dma(iface, fromdev, canwait)) == NULL)
		return -1;

	if (pirq)
		*pirq = (fromdev ? chnpptr->wx_irq : chnpptr->rx_irq);
	if (pregbase)
		*pregbase = chnpptr->fromdev ? chnpptr->wx_base : chnpptr->rx_base;
	
	return 0;
}

/* TODO: TO BE RELPLACED */
void em86xx_mbus_free_dma(unsigned long regbase, int iface)
{
	struct mbus_channel_pair *chnpptr = (*__mbus_reg2chnp)(regbase);
	if (regbase == 0) {
		MSG("%s:%d regbase=0x%x\n", __FILE__, __LINE__, regbase);
		BUG();
	} else if (chnpptr == NULL) {
		MSG("%s:%d regbase=0x%x\n", __FILE__, __LINE__, regbase);
		BUG();
	} else if ((regbase != chnpptr->wx_base) && (regbase != chnpptr->rx_base)) {
		MSG("%s:%d regbase=0x%x\n", __FILE__, __LINE__, regbase);
		BUG();
	}
	tangox_mbus_free_dma(chnpptr, iface);
}

/* TODO: TO BE RELPLACED */
int em86xx_mbus_notification(unsigned int regbase, mbus_irq_handler_t handler, void *arg)
{
	struct mbus_channel_pair *chnpptr = (*__mbus_reg2chnp)(regbase);
	if (regbase == 0) {
		MSG("%s:%d regbase=0x%x\n", __FILE__, __LINE__, regbase);
		BUG();
	} else if (chnpptr == NULL) {
		MSG("%s:%d regbase=0x%x\n", __FILE__, __LINE__, regbase);
		BUG();
	} else if ((regbase != chnpptr->wx_base) && (regbase != chnpptr->rx_base)) {
		MSG("%s:%d regbase=0x%x\n", __FILE__, __LINE__, regbase);
		BUG();
	}
	return tangox_mbus_notification(chnpptr, handler, arg);
}

/* TODO: TO BE RELPLACED */
int em86xx_mbus_setup_dma(unsigned int regbase, unsigned int addr,
			  unsigned int count, mbus_irq_handler_t handler,
			  void *arg, unsigned int tflags)
{
	struct mbus_channel_pair *chnpptr = (*__mbus_reg2chnp)(regbase);
	if (regbase == 0) {
		MSG("%s:%d regbase=0x%x\n", __FILE__, __LINE__, regbase);
		BUG();
	} else if (chnpptr == NULL) {
		MSG("%s:%d regbase=0x%x\n", __FILE__, __LINE__, regbase);
		BUG();
	} else if ((regbase != chnpptr->wx_base) && (regbase != chnpptr->rx_base)) {
		MSG("%s:%d regbase=0x%x\n", __FILE__, __LINE__, regbase);
		BUG();
	}
	return tangox_mbus_setup_dma(chnpptr, addr, count, handler, arg, tflags);
}

/* TODO: TO BE RELPLACED */
int em86xx_mbus_wait(unsigned int regbase, int iface)
{
	struct mbus_channel_pair *chnpptr = (*__mbus_reg2chnp)(regbase);
	if (regbase == 0) {
		MSG("%s:%d regbase=0x%x\n", __FILE__, __LINE__, regbase);
		BUG();
	} else if (chnpptr == NULL) {
		MSG("%s:%d regbase=0x%x\n", __FILE__, __LINE__, regbase);
		BUG();
	} else if ((regbase != chnpptr->wx_base) && (regbase != chnpptr->rx_base)) {
		MSG("%s:%d regbase=0x%x\n", __FILE__, __LINE__, regbase);
		BUG();
	}
	return tangox_mbus_wait(chnpptr, iface);
}

/* TODO: TO BE RELPLACED */
void em86xx_mbus_reset(unsigned int regbase, int iface)
{
	struct mbus_channel_pair *chnpptr = (*__mbus_reg2chnp)(regbase);
	if (regbase == 0) {
		MSG("%s:%d regbase=0x%x\n", __FILE__, __LINE__, regbase);
		BUG();
	} else if (chnpptr == NULL) {
		MSG("%s:%d regbase=0x%x\n", __FILE__, __LINE__, regbase);
		BUG();
	} else if ((regbase != chnpptr->wx_base) && (regbase != chnpptr->rx_base)) {
		MSG("%s:%d regbase=0x%x\n", __FILE__, __LINE__, regbase);
		BUG();
	}
	return tangox_mbus_reset(chnpptr, iface);
}

EXPORT_SYMBOL(em86xx_mbus_alloc_dma);
EXPORT_SYMBOL(em86xx_mbus_free_dma);
EXPORT_SYMBOL(em86xx_mbus_notification);
EXPORT_SYMBOL(em86xx_mbus_setup_dma);
EXPORT_SYMBOL(em86xx_mbus_wait);
EXPORT_SYMBOL(em86xx_mbus_reset);
__initcall(tangox_mbus_init);

/* Comment this to use DMA-like memcpy */
#define WITH_ORIGINAL_MEMCPY

/* Comment this to use DMA-like copy_to*, copy_from* */
#define WITH_ORIGINAL_COPYUSER

/* Minimum size of DMA copy (less than this original ones to be used) */
#define MINIMUM_DMACPY_SIZE	(PAGE_SIZE >> 2)

#ifdef CONFIG_TANGO2
#define CACHELINE_MASK	0xf
#define SBOX_TARGET	SBOX_PCIMASTER
#elif defined(CONFIG_TANGO3)
#define CACHELINE_MASK	0x1f
#define SBOX_TARGET	SBOX_IDEDVD
#elif defined(CONFIG_TANGO4)
#define CACHELINE_MASK	0x1f
#define SBOX_TARGET	SBOX_IDEDVD
#else
#error Not supported platform.
#endif

#if !defined(WITH_ORIGINAL_MEMCPY) && !defined(CONFIG_SWAP) && defined(CONFIG_TANGO4)
static DEFINE_SPINLOCK(memcpy_lock); /* for controlling DMA memcpy */
#endif

static int tangox_mbus_memcpy(struct mbus_channel_pair *chnpptr, unsigned int dst, unsigned int src, unsigned int size);

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
	struct mbus_channel_pair *chnpptr;

	// JFT, We could probably handle more cases
	if (__n < MINIMUM_DMACPY_SIZE || 
#ifdef CONFIG_TANGO2
			virt_from < (KSEG0 + MEM_BASE_dram_controller_0) || virt_to < (KSEG0 + MEM_BASE_dram_controller_0) 
#elif defined(CONFIG_TANGO3) || defined(CONFIG_TANGO4)
			virt_from < (KSEG0 + CPU_REMAP_SPACE) || virt_to < (KSEG0 + CPU_REMAP_SPACE)
#endif
			|| virt_from >= KSEG2 || virt_to >= KSEG2)
		return memcpy_original(__to, __from, __n);
#ifdef CONFIG_TANGO2
	else if (unlikely(((virt_from >= KSEG1) && (virt_from < (KSEG1 + MEM_BASE_dram_controller_0))) ||
			((virt_to >= KSEG1) && (virt_to < (KSEG1 + MEM_BASE_dram_controller_0)))))
		return memcpy_original(__to, __from, __n);
#elif defined(CONFIG_TANGO3) || defined(CONFIG_TANGO4)
	else if (unlikely(((virt_from >= KSEG1) && (virt_from < (KSEG1 + CPU_REMAP_SPACE))) ||
			((virt_to >= KSEG1) && (virt_to < (KSEG1 + CPU_REMAP_SPACE)))))
		return memcpy_original(__to, __from, __n);
#endif
#ifdef CONFIG_TANGO4
	else if (!spin_trylock(&memcpy_lock))
		return memcpy_original(__to, __from, __n);
#endif
	else if ((chnpptr = tangox_mbus_alloc_dma(SBOX_TARGET, 1, 0)) == NULL)
		return memcpy_original(__to, __from, __n);

	// If the virt_to is not aligned we need to flush the data before it
	if (likely(virt_to < KSEG1)) {
		if (virt_to & CACHELINE_MASK)
			flush_dcache_line(virt_to & ~CACHELINE_MASK);
	}
	// If the virt_to + cu_len is not aligned we need to flush the data after it
	if (likely((virt_to + __n) < KSEG1)) {
		if ((virt_to + __n) & CACHELINE_MASK)
			flush_dcache_line((virt_to + __n) & ~CACHELINE_MASK);
	}

	if (likely(virt_from < KSEG1))
		blast_dcache_range(virt_from, virt_from + len);
	if (likely(virt_to < KSEG1))
		blast_inv_dcache_range(virt_to, virt_to + len);

	if (tangox_mbus_memcpy(chnpptr, tangox_dma_address(CPHYSADDR(virt_to)), tangox_dma_address(CPHYSADDR(virt_from)), len) == 0) {
		printk("%s:%d: MBUS memcpy failure.\n", __FUNCTION__, __LINE__);
		tangox_mbus_free_dma(chnpptr, SBOX_TARGET);
#ifdef CONFIG_TANGO4
		spin_unlock(&memcpy_lock);
#endif
		return memcpy_original(__to, __from, __n);
	}

	tangox_mbus_free_dma(chnpptr, SBOX_TARGET);
#ifdef CONFIG_TANGO4
	spin_unlock(&memcpy_lock);
#endif
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
	struct mbus_channel_pair *chnpptr;

	// JFT, note: this is very common case so make it first
	if (__cu_len < MINIMUM_DMACPY_SIZE) 
        	return __invoke_copy_to_user(__cu_to, __cu_from, __cu_len);
	else if ((virt_to >= KSEG0) || (virt_from >= KSEG2))
        	return __invoke_copy_to_user(__cu_to, __cu_from, __cu_len);
	else if (!(virt_addr_valid(__cu_from)) || !(virt_addr_valid(__cu_from + __cu_len))) 
		return __invoke_copy_to_user(__cu_to, __cu_from, __cu_len);
#ifdef CONFIG_TANGO4
	else if (!spin_trylock(&memcpy_lock))
		return __invoke_copy_to_user(__cu_to, __cu_from, __cu_len);
#endif
	else if ((chnpptr = tangox_mbus_alloc_dma(SBOX_TARGET, 1, 0)) == NULL)
		return __invoke_copy_to_user(__cu_to, __cu_from, __cu_len);

	// Verify all pages exist
	for (byte = 0; byte < __cu_len; byte += PAGE_SIZE)
		__put_user_check(0, (unsigned char *)(virt_to + byte), 1);
	__put_user_check(0, (unsigned char *)(virt_to + __cu_len - 1), 1);

	// If the virt_to is not aligned we need to flush the data before it
	if (virt_to & CACHELINE_MASK)
		flush_dcache_line(virt_to & ~CACHELINE_MASK);
	// If the virt_to + cu_len is not aligned we need to flush the data after it
	if ((virt_to + __cu_len) & CACHELINE_MASK)
		flush_dcache_line((virt_to + __cu_len) & ~CACHELINE_MASK);

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
#elif defined(CONFIG_TANGO3) || defined(CONFIG_TANGO4)
		if (unlikely((dma_to < CPU_REMAP_SPACE) || (dma_to >= (KSEG1 - KSEG0)))) 
			goto error;
#endif

		blast_dcache_range(virt_from + byte, virt_from + byte + len);
		blast_inv_dcache_range(virt_to + byte, virt_to + byte + len);

		if (tangox_mbus_memcpy(chnpptr, tangox_dma_address(dma_to), tangox_dma_address(CPHYSADDR(virt_from + byte)), len) == 0) {
			printk("%s:%d: MBUS memcpy failure.\n", __FUNCTION__, __LINE__);
			tangox_mbus_free_dma(chnpptr, SBOX_TARGET);
#ifdef CONFIG_TANGO4
			spin_unlock(&memcpy_lock);
#endif
			return __invoke_copy_to_user(__cu_to, __cu_from, __cu_len);
		}

		byte += len;
        	_n -= len;
	}

error:
	tangox_mbus_free_dma(chnpptr, SBOX_TARGET);
#ifdef CONFIG_TANGO4
	spin_unlock(&memcpy_lock);
#endif
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
	unsigned char val;
	struct mbus_channel_pair *chnpptr;

	// JFT, this is very common case so make it first
	if (__cu_len < MINIMUM_DMACPY_SIZE) 
		return __invoke_copy_from_user(__cu_to, __cu_from, __cu_len);
	else if ((virt_from >= KSEG0) || (virt_to >= KSEG2))
        	return __invoke_copy_from_user(__cu_to, __cu_from, __cu_len);
	else if (!(virt_addr_valid(__cu_to)) || !(virt_addr_valid(__cu_to + __cu_len))) 
		return __invoke_copy_from_user(__cu_to, __cu_from, __cu_len);
#ifdef CONFIG_TANGO4
	else if (!spin_trylock(&memcpy_lock))
		return __invoke_copy_to_user(__cu_to, __cu_from, __cu_len);
#endif
	else if ((chnpptr = tangox_mbus_alloc_dma(SBOX_TARGET, 1, 0)) == NULL)
		return __invoke_copy_from_user(__cu_to, __cu_from, __cu_len);

	// Verify all pages exist
	for (byte = 0; byte < __cu_len; byte += PAGE_SIZE) 
		__get_user_check(val, (unsigned char *)(virt_from + byte), 1);
	__get_user_check(val, (unsigned char *)(virt_from + __cu_len - 1), 1);

	// If the virt_to is not aligned we need to flush the data before it
	if (virt_to & CACHELINE_MASK)
		flush_dcache_line(virt_to & ~CACHELINE_MASK);
	// If the virt_to + cu_len is not aligned we need to flush the data after it
	if ((virt_to + __cu_len) & CACHELINE_MASK) 
		flush_dcache_line((virt_to + __cu_len) & ~CACHELINE_MASK);

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
#elif defined(CONFIG_TANGO3) || defined(CONFIG_TANGO4)
		if (unlikely((dma_from < CPU_REMAP_SPACE) || (dma_from >= (KSEG1 - KSEG0)))) 
			goto error;
#endif
		blast_dcache_range(virt_from + byte, virt_from + byte + len);
		blast_inv_dcache_range(virt_to + byte, virt_to + byte + len);
	
		if (tangox_mbus_memcpy(chnpptr, tangox_dma_address(CPHYSADDR(virt_to + byte)), tangox_dma_address(dma_from), len) == 0) {
			printk("%s:%d: MBUS memcpy failure.\n", __FUNCTION__, __LINE__);
			tangox_mbus_free_dma(chnpptr, SBOX_TARGET);
#ifdef CONFIG_TANGO4
			spin_unlock(&memcpy_lock);
#endif
			return __invoke_copy_from_user(__cu_to, __cu_from, __cu_len);
		}

		byte += len;
        	_n -= len;
	}
	
error:
	tangox_mbus_free_dma(chnpptr, SBOX_TARGET);
#ifdef CONFIG_TANGO4
	spin_unlock(&memcpy_lock);
#endif
	return _n;
#endif /* WITH_ORIGINAL_COPYUSER || CONFIG_SWAP */
}

EXPORT_SYMBOL(__invoke_copy_to_user_dma);
EXPORT_SYMBOL(__invoke_copy_from_user_dma);

/* Fancy version of memcpy, both dst and src need to be physical address */
/* The channels have to be allocated already */
static int tangox_mbus_memcpy(struct mbus_channel_pair *chnpptr, unsigned int dst, unsigned int src, unsigned int size)
{
	/* Save the old SBOX route */
	unsigned int sbox_route;
	unsigned int w_base;
	unsigned int r_base;
	int i, chnp;
	unsigned int tmode_w, tmode_r;

/* 
 * TRANSFER defines 4 bits, 
 * Tango3:
 *    bit 0: followed by void (1) or not (0),
 *    bit 1: tiled buffer or not.
 *    bit 3-2: 0 = 8 bit, 1 = 16 bit, 2 = 32 bit.
 * Tango4: 
 *    bit 0: followed by void (1) or not (0),
 *    bit 13: tiled buffer or not.
 *    bit 15-14: 0 = 8 bit, 1 = 16 bit, 2 = 32 bit.
 */
#if defined(CONFIG_TANGO4)
#define TRANSFER    0x1 /* or 0x8001 for 32 bit transfer */
#elif defined(CONFIG_TANGO3)
#define TRANSFER    0x1 /* or 0x9 for 32 bit transfer */
#else
#define TRANSFER    0x1
#endif

#define MBUS_MEMCPY_TIMEOUT	100	/* 100 usec */

	tmode_w = tmode_r = TRANSFER;
	chnp = chnpptr->idx;
	w_base = chnpptr->wx_base;
	r_base = chnpptr->rx_base;

	switch(chnp) {
		case 0: { /* W0/R0 channel pair is used */
				sbox_route = gbus_read_reg32(REG_BASE_host_interface + SBOX_ROUTE) & 0xffffff0f;

				/* Hook up W0/R0 and left W1/R1 the same as before */
				gbus_write_reg32(REG_BASE_host_interface + SBOX_ROUTE, 0xffffff01);
		
				if (__tangox_mbus_setup_dma(chnpptr, dst, size, NULL, NULL, tmode_w, w_base) != 0) {
					gbus_write_reg32(REG_BASE_host_interface + SBOX_ROUTE, sbox_route);
					goto error;
				} else if (__tangox_mbus_setup_dma(chnpptr, src, size, NULL, NULL, tmode_r, r_base) != 0) {
					/* resetting W0 */
					gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET, 0x01010000);
					gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET, 0x01000000);
					gbus_write_reg32(REG_BASE_host_interface + SBOX_ROUTE, sbox_route);
					goto error;
				}

				for (i = 0; (i < MBUS_MEMCPY_TIMEOUT) && ((__tangox_mbus_inuse(chnpptr, r_base) != 0) || (__tangox_mbus_inuse(chnpptr, w_base) != 0)); i++)
					udelay(1);

				if (__tangox_mbus_inuse(chnpptr, w_base) != 0) {
					gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET, 0x01010000);
					gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET, 0x01000000);
				}
				if (__tangox_mbus_inuse(chnpptr, r_base) != 0) {
					gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET, 0x00000101);
					gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET, 0x00000100);
				}

				/* Restore SBOX route once we're done */
				gbus_write_reg32(REG_BASE_host_interface + SBOX_ROUTE, sbox_route);
			}
			break;
#ifdef WITH_MBUS_W1R1
		case 1: { /* W1/R1 channel pair is used */
				sbox_route = gbus_read_reg32(REG_BASE_host_interface + SBOX_ROUTE) & 0xfffffff0;

				/* Hook up W1/R1 and left W0/R0 the same as before */
				gbus_write_reg32(REG_BASE_host_interface + SBOX_ROUTE, 0xffffff20);
	
				if (__tangox_mbus_setup_dma(chnpptr, dst, size, NULL, NULL, tmode_w, w_base) != 0) {
					gbus_write_reg32(REG_BASE_host_interface + SBOX_ROUTE, sbox_route);
					goto error;
				} else if (__tangox_mbus_setup_dma(chnpptr, src, size, NULL, NULL, tmode_r, r_base) != 0) {
					/* resetting W1 */
					gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET, 0x02020000);
					gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET, 0x02000000);
					gbus_write_reg32(REG_BASE_host_interface + SBOX_ROUTE, sbox_route);
					goto error;
				}

				for (i = 0; (i < MBUS_MEMCPY_TIMEOUT) && ((__tangox_mbus_inuse(chnpptr, r_base) != 0) || (__tangox_mbus_inuse(chnpptr, w_base) != 0)); i++)
					udelay(1);

				if (__tangox_mbus_inuse(chnpptr, w_base) != 0) {
					gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET, 0x02020000);
					gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET, 0x02000000);
				}
				if (__tangox_mbus_inuse(chnpptr, r_base) != 0) {
					gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET, 0x00000202);
					gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET, 0x00000200);
				}

				/* Restore SBOX route once we're done */
				gbus_write_reg32(REG_BASE_host_interface + SBOX_ROUTE, sbox_route);
			}
			break;
#endif
#if defined(CONFIG_TANGO3) || defined(CONFIG_TANGO4)
		case 2: { /* W2/R2 channel pair is used */
				sbox_route = gbus_read_reg32(REG_BASE_host_interface + SBOX_ROUTE2);

				/* Hook up W2/R2 */
				gbus_write_reg32(REG_BASE_host_interface + SBOX_ROUTE2, 0xfffffff9);
	
				if (__tangox_mbus_setup_dma(chnpptr, dst, size, NULL, NULL, tmode_w, w_base) != 0) {
					gbus_write_reg32(REG_BASE_host_interface + SBOX_ROUTE2, sbox_route);
					goto error;
				} else if (__tangox_mbus_setup_dma(chnpptr, src, size, NULL, NULL, tmode_r, r_base) != 0) {
					/* resetting W2 */
					gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET2, 0x01010000);
					gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET2, 0x01000000);
					gbus_write_reg32(REG_BASE_host_interface + SBOX_ROUTE2, sbox_route);
					goto error;
				}

				for (i = 0; (i < MBUS_MEMCPY_TIMEOUT) && ((__tangox_mbus_inuse(chnpptr, r_base) != 0) || (__tangox_mbus_inuse(chnpptr, w_base) != 0)); i++)
					udelay(1);

				if (__tangox_mbus_inuse(chnpptr, w_base) != 0) {
					gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET2, 0x01010000);
					gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET2, 0x01000000);
				}
				if (__tangox_mbus_inuse(chnpptr, r_base) != 0) {
					gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET2, 0x00000101);
					gbus_write_reg32(REG_BASE_host_interface + SBOX_FIFO_RESET2, 0x00000100);
				}

				/* Restore SBOX route once we're done */
				gbus_write_reg32(REG_BASE_host_interface + SBOX_ROUTE2, sbox_route);
			}
			break;
#endif
		default: goto error;
			break;
	}
	return size;

error:
	return 0;
}

/* Fancy version of memcpy, both dst and src need to be physical address */
/* The channels have to be allocated already */
int mbus_memcpy(unsigned int regbase, unsigned int dst, unsigned int src, unsigned int size)
{
	struct mbus_channel_pair *chnpptr = (*__mbus_reg2chnp)(regbase);
	if (regbase == 0) {
		MSG("%s:%d regbase=0x%x\n", __FILE__, __LINE__, regbase);
		BUG();
	} else if (chnpptr == NULL) {
		MSG("%s:%d regbase=0x%x\n", __FILE__, __LINE__, regbase);
		BUG();
	} else if ((regbase != chnpptr->wx_base) && (regbase != chnpptr->rx_base)) {
		MSG("%s:%d regbase=0x%x\n", __FILE__, __LINE__, regbase);
		BUG();
	}
	return tangox_mbus_memcpy(chnpptr, dst, src, size);
}
EXPORT_SYMBOL(mbus_memcpy);

