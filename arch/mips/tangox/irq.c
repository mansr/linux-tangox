/*
 * Copyright 2001 MontaVista Software Inc.
 * Author: Jun Sun, jsun@mvista.com or jsun@junsun.net
 *
 * Copyright (C) 2007 Sigma Designs, Inc.
 *
 * arch_init_irq for tango2/tango3.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/init.h>
#include <linux/interrupt.h>
#include <asm/irq_cpu.h>

#include "setup.h"

/*
 * helpers to access cpu block registers
 */
#define RD_CPU_REG32(r)		\
		gbus_read_reg32(REG_BASE_cpu_block + (r))

#define WR_CPU_REG32(r, v)	\
		do {								\
			gbus_write_reg32(REG_BASE_cpu_block + (r), (v));	\
			iob();							\
		} while(0)	

static DEFINE_SPINLOCK(mips_irq_lock);

static inline u64 get_irq_status(void)
{
	u64 status;
	unsigned long flags;
	spin_lock_irqsave(&mips_irq_lock, flags);
	status = (((u64)RD_CPU_REG32(CPU_irq_status_hi))<<32) | ((u64)RD_CPU_REG32(CPU_irq_status));
	spin_unlock_irqrestore(&mips_irq_lock, flags);
	return status;
}
static inline u64 get_fiq_status(void)
{
	u64 status;
	unsigned long flags;
	spin_lock_irqsave(&mips_irq_lock, flags);
	status = (((u64)RD_CPU_REG32(CPU_fiq_status_hi))<<32) | ((u64)RD_CPU_REG32(CPU_fiq_status));
	spin_unlock_irqrestore(&mips_irq_lock, flags);
	return status;
}
static inline u64 get_iiq_status(void)
{
	u64 status;
	unsigned long flags;
	spin_lock_irqsave(&mips_irq_lock, flags);
	status = (((u64)RD_CPU_REG32(CPU_iiq_status_hi))<<32) | ((u64)RD_CPU_REG32(CPU_iiq_status));
	spin_unlock_irqrestore(&mips_irq_lock, flags);
	return status;
}
 
static void fastcall tangox_irq_flow_handler(unsigned int irq, struct irq_desc *desc)
{
	__do_IRQ(irq);
}

static inline int clz(unsigned long x)
{
	__asm__ (
	"	.set	push					\n"
	"	.set	mips32					\n"
	"	clz	%0, %1					\n"
	"	.set	pop					\n"
	: "=r" (x)
	: "r" (x));

	return x;
}

static inline int clz64(u64 x)
{
	u32 xl = (u32)x & 0xffffffff, xh = (u32)(x >> 32) & 0xffffffff;
	int lz;
	lz = clz(xh) & 0x1f;
	if (lz != 0)
		return lz;
	else {
		lz = clz(xl) & 0x1f;
		if (lz != 0)
			return lz + 32;
	}
	return 64;
}

static inline unsigned int irq_ffs(unsigned int pending)
{
	return -clz(pending) + 31 - CAUSEB_IP;
}

static int mips_timer_irq = 0;
#ifdef CONFIG_TANGO3
static int mips_pref_irq = 0;
#endif
static unsigned long edge_trig = 0;
static unsigned long edge_trig_hi = 0;

/*
 * dispatch routine called from genex.S
 */
extern void spurious_interrupt(void);
asmlinkage void plat_irq_dispatch(void)
{
	unsigned int pending = (read_c0_cause() & read_c0_status()) & ST0_IM;
	int irq;
	u64 status;

	if (pending == 0) {
		printk("spurious hwirq: nothing pending\n");
		spurious_interrupt();
		return;
	}

	for (; pending != 0; pending = (read_c0_cause() & read_c0_status()) & ST0_IM) {

		irq = irq_ffs(pending);

		switch(irq) {
			case 2: if ((status = get_irq_status()) == 0) {
					printk("spurious irq: 0x%llx\n", status);
					goto spurious;
				} else {
					do_IRQ(IRQ_CONTROLLER_IRQ_BASE + (63 - clz64(status)));
				}
				break;

			case 3: if ((status = get_fiq_status()) == 0) {
					printk("spurious fiq: 0x%llx\n", status);
					goto spurious;
				} else {
					/* We need to mask out irq, fiq > irq */
					u32 sr_old = read_c0_status();
					u32 sr_new = sr_old & (~STATUSF_IP2);

					write_c0_status(sr_new);
					do_IRQ(FIQ_CONTROLLER_IRQ_BASE + (63 - clz64(status)));
					write_c0_status(sr_old);
				}
				break;

			case 4: if ((status = get_iiq_status()) == 0) {
					printk("spurious iiq: 0x%llx\n", status);
					goto spurious;
				} else {
					/* We need to mask out fiq/irq, iiq > fiq > irq */
					u32 sr_old = read_c0_status();
					u32 sr_new = sr_old & (~(STATUSF_IP2|STATUSF_IP3));

					write_c0_status(sr_new);
					do_IRQ(IIQ_CONTROLLER_IRQ_BASE + (63 - clz64(status)));
					write_c0_status(sr_old);
				}
				break;

			default: if ((irq == mips_timer_irq)
#ifdef CONFIG_TANGO3
					|| (irq == mips_pref_irq)
#endif
				) {
					do_IRQ(irq);
				} else { 
					printk("spurious hwirq: %d\n", irq);
					goto spurious;
				}
				break;
		}

		continue;
spurious:
		spurious_interrupt();
	}

	return;
}

/*
 * our hw_irq_controller cb
 */
static inline void tangox_irq_enable(unsigned int x)
{
	int bit = x - IRQ_CONTROLLER_IRQ_BASE;
	unsigned long flags;

	local_irq_save(flags);
	if (bit >= 32) 
		WR_CPU_REG32(CPU_irq_enableset_hi, 1 << (bit - 32));
	else 
		WR_CPU_REG32(CPU_irq_enableset, 1 << bit);
	local_irq_restore(flags);
}

static inline void tangox_fiq_enable(unsigned int x)
{
	int bit = x - FIQ_CONTROLLER_IRQ_BASE;
	unsigned long flags;

	local_irq_save(flags);
	if (bit >= 32) 
		WR_CPU_REG32(CPU_fiq_enableset_hi, 1 << (bit - 32));
	else
		WR_CPU_REG32(CPU_fiq_enableset, 1 << bit);
	local_irq_restore(flags);
}

static inline void tangox_iiq_enable(unsigned int x)
{
	int bit = x - IIQ_CONTROLLER_IRQ_BASE;
	unsigned long flags;

	local_irq_save(flags);
	if (bit >= 32) 
		WR_CPU_REG32(CPU_iiq_enableset_hi, 1 << (bit - 32));
	else
		WR_CPU_REG32(CPU_iiq_enableset, 1 << bit);
	local_irq_restore(flags);
}

static inline void tangox_irq_disable(unsigned int x)
{
	int bit = x - IRQ_CONTROLLER_IRQ_BASE;
	unsigned long flags;

	local_irq_save(flags);
	if (bit >= 32) 
		WR_CPU_REG32(CPU_irq_enableclr_hi, 1 << (bit - 32));
	else
		WR_CPU_REG32(CPU_irq_enableclr, 1 << bit);
	local_irq_restore(flags);
}

static inline void tangox_fiq_disable(unsigned int x)
{
	int bit = x - FIQ_CONTROLLER_IRQ_BASE;
	unsigned long flags;

	local_irq_save(flags);
	if (bit >= 32) 
		WR_CPU_REG32(CPU_fiq_enableclr_hi, 1 << (bit - 32));
	else
		WR_CPU_REG32(CPU_fiq_enableclr, 1 << bit);
	local_irq_restore(flags);
}

static inline void tangox_iiq_disable(unsigned int x)
{
	int bit = x - IIQ_CONTROLLER_IRQ_BASE;
	unsigned long flags;

	local_irq_save(flags);
	if (bit >= 32) 
		WR_CPU_REG32(CPU_iiq_enableclr_hi, 1 << (bit - 32));
	else
		WR_CPU_REG32(CPU_iiq_enableclr, 1 << bit);
	local_irq_restore(flags);
}

static unsigned int tangox_irq_startup(unsigned int x)
{
	int bit = x - IRQ_CONTROLLER_IRQ_BASE;

	/* clear any pending interrupt before enabling it */
	if (bit >= 32) {
		if (edge_trig_hi & (1 << (bit - 32)))
			WR_CPU_REG32(CPU_edge_rawstat_hi, 1 << (bit - 32));
	} else {
		if (edge_trig & (1 << bit))
			WR_CPU_REG32(CPU_edge_rawstat, 1 << bit);
	}

	tangox_irq_enable(x);
	return 0;
}

static unsigned int tangox_fiq_startup(unsigned int x)
{
	int bit = x - FIQ_CONTROLLER_IRQ_BASE;

	/* clear any pending interrupt before enabling it */
	if (bit >= 32) {
		if (edge_trig_hi & (1 << (bit - 32)))
			WR_CPU_REG32(CPU_edge_rawstat_hi, 1 << (bit - 32));
	} else {
		if (edge_trig & (1 << bit))
			WR_CPU_REG32(CPU_edge_rawstat, 1 << bit);
	}

	tangox_fiq_enable(x);
	return 0;
}

static unsigned int tangox_iiq_startup(unsigned int x)
{
	int bit = x - IIQ_CONTROLLER_IRQ_BASE;

	/* clear any pending interrupt before enabling it */
	if (bit >= 32) {
		if (edge_trig_hi & (1 << (bit - 32)))
			WR_CPU_REG32(CPU_edge_rawstat_hi, 1 << (bit - 32));
	} else {
		if (edge_trig & (1 << bit))
			WR_CPU_REG32(CPU_edge_rawstat, 1 << bit);
	}

	tangox_iiq_enable(x);
	return 0;
}

#define	tangox_irq_shutdown tangox_irq_disable
#define	tangox_fiq_shutdown tangox_fiq_disable
#define	tangox_iiq_shutdown tangox_iiq_disable

static void tangox_irq_ack(unsigned int x)
{
	int bit = x - IRQ_CONTROLLER_IRQ_BASE;

	tangox_irq_disable(x);

	if (bit >= 32) {
		if (edge_trig_hi & (1 << (bit - 32)))
			WR_CPU_REG32(CPU_edge_rawstat_hi, 1 << (bit - 32));
	} else {
		if (edge_trig & (1 << bit))
			WR_CPU_REG32(CPU_edge_rawstat, 1 << bit);
	}
}

static void tangox_fiq_ack(unsigned int x)
{
	int bit = x - FIQ_CONTROLLER_IRQ_BASE;

	tangox_fiq_disable(x);

	if (bit >= 32) {
		if (edge_trig_hi & (1 << (bit - 32)))
			WR_CPU_REG32(CPU_edge_rawstat_hi, 1 << (bit - 32));
	} else {
		if (edge_trig & (1 << bit))
			WR_CPU_REG32(CPU_edge_rawstat, 1 << bit);
	}
}

static void tangox_iiq_ack(unsigned int x)
{
	int bit = x - IIQ_CONTROLLER_IRQ_BASE;

	tangox_iiq_disable(x);

	if (bit >= 32) {
		if (edge_trig_hi & (1 << (bit - 32)))
			WR_CPU_REG32(CPU_edge_rawstat_hi, 1 << (bit - 32));
	} else {
		if (edge_trig & (1 << bit))
			WR_CPU_REG32(CPU_edge_rawstat, 1 << bit);
	}
}

static void tangox_irq_end(unsigned int x)
{
	if (!(irq_desc[x].status & (IRQ_DISABLED | IRQ_INPROGRESS)))
		tangox_irq_enable(x);
}

static void tangox_fiq_end(unsigned int x)
{
	if (!(irq_desc[x].status & (IRQ_DISABLED | IRQ_INPROGRESS)))
		tangox_fiq_enable(x);
}

static void tangox_iiq_end(unsigned int x)
{
	if (!(irq_desc[x].status & (IRQ_DISABLED | IRQ_INPROGRESS)))
		tangox_iiq_enable(x);
}

/*
 * our hw_irq_controller
 */
static hw_irq_controller tangox_irq_controller = {
	.typename = "tangox_irq",
	.startup = tangox_irq_startup,
	.shutdown = tangox_irq_shutdown,
	.enable = tangox_irq_enable,
	.disable = tangox_irq_disable,
	.ack = tangox_irq_ack,
	.end = tangox_irq_end,
};

static hw_irq_controller tangox_fiq_controller = {
	.typename = "tangox_fiq",
	.startup = tangox_fiq_startup,
	.shutdown = tangox_fiq_shutdown,
	.enable = tangox_fiq_enable,
	.disable = tangox_fiq_disable,
	.ack = tangox_fiq_ack,
	.end = tangox_fiq_end,
};

static hw_irq_controller tangox_iiq_controller = {
	.typename = "tangox_iiq",
	.startup = tangox_iiq_startup,
	.shutdown = tangox_iiq_shutdown,
	.enable = tangox_iiq_enable,
	.disable = tangox_iiq_disable,
	.ack = tangox_iiq_ack,
	.end = tangox_iiq_end,
};

static struct irqaction irq_cascade = {
	no_action, IRQF_SHARED, { { 0, } }, "cascade", NULL, NULL
};

void __init arch_init_irq(void)
{
	unsigned long x;
	unsigned long rise = 0;
	unsigned long fall = 0;
	unsigned long rise_hi = 0;
	unsigned long fall_hi = 0;

	/* irq_desc entries 0..7 */
	mips_cpu_irq_init();

	WR_CPU_REG32(CPU_irq_enableclr, 0xffffffff);
	WR_CPU_REG32(CPU_fiq_enableclr, 0xffffffff);
	WR_CPU_REG32(CPU_iiq_enableclr, 0xffffffff);
	WR_CPU_REG32(CPU_irq_enableclr_hi, 0xffffffff);
	WR_CPU_REG32(CPU_fiq_enableclr_hi, 0xffffffff);
	WR_CPU_REG32(CPU_iiq_enableclr_hi, 0xffffffff);

	rise = RD_CPU_REG32(CPU_edge_config_rise);
	fall = RD_CPU_REG32(CPU_edge_config_fall);
	edge_trig = rise ^ fall;
	WR_CPU_REG32(CPU_edge_rawstat, edge_trig);
//WR_CPU_REG32(CPU_edge_rawstat, 0xffffffff);
	rise_hi = RD_CPU_REG32(CPU_edge_config_rise_hi);
	fall_hi = RD_CPU_REG32(CPU_edge_config_fall_hi);
	edge_trig_hi = rise_hi ^ fall_hi;
	WR_CPU_REG32(CPU_edge_rawstat_hi, edge_trig_hi);
//WR_CPU_REG32(CPU_edge_rawstat_hi, 0xffffffff);

	for (x = IRQ_CONTROLLER_IRQ_BASE; x < IRQ_CONTROLLER_IRQ_BASE + IRQ_COUNT; x++) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18) 
		irq_desc[x].status = IRQ_DISABLED;
		irq_desc[x].action = NULL;
		irq_desc[x].depth = 1;
		irq_desc[x].handle_irq = &tangox_irq_controller;
#else
		set_irq_chip_and_handler(x, &tangox_irq_controller, tangox_irq_flow_handler);
#endif
	}

	for (x = FIQ_CONTROLLER_IRQ_BASE; x < FIQ_CONTROLLER_IRQ_BASE + IRQ_COUNT; x++) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18)
		irq_desc[x].status = IRQ_DISABLED;
		irq_desc[x].action = NULL;
		irq_desc[x].depth = 1;
		irq_desc[x].handle_irq = &tangox_fiq_controller;
#else
		set_irq_chip_and_handler(x, &tangox_fiq_controller, tangox_irq_flow_handler);
#endif
	}

	for (x = IIQ_CONTROLLER_IRQ_BASE; x < IIQ_CONTROLLER_IRQ_BASE + IRQ_COUNT; x++) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18)
		irq_desc[x].status = IRQ_DISABLED;
		irq_desc[x].action = NULL;
		irq_desc[x].depth = 1;
		irq_desc[x].handle_irq = &tangox_iiq_controller;
#else
		set_irq_chip_and_handler(x, &tangox_iiq_controller, tangox_irq_flow_handler);
#endif
	}

	setup_irq(MIPS_CPU_IRQ_BASE + 2, &irq_cascade);
	setup_irq(MIPS_CPU_IRQ_BASE + 3, &irq_cascade);
	setup_irq(MIPS_CPU_IRQ_BASE + 4, &irq_cascade);

#ifdef CONFIG_TANGO2
	mips_timer_irq = 7;
#elif defined(CONFIG_TANGO3)
	mips_timer_irq = (read_c0_intctl() >> 29) & 7;
	mips_pref_irq = (read_c0_intctl() >> 26) & 7;
#endif
	return;
}

