/*
 * Copyright 2001 MontaVista Software Inc.
 * Author: Jun Sun, jsun@mvista.com or jsun@junsun.net
 *
 * Copyright (C) 2007-2011 Sigma Designs, Inc.
 *
 * arch_init_irq for tango2/tango3/tango4
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <asm/irq_cpu.h>

#ifdef CONFIG_TANGO4
#include <asm/gcmpregs.h>
#include <asm/gic.h>
#include <asm/setup.h>
#include <asm/tango4/launch.h>
#endif

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

#if defined(CONFIG_TANGO3) 
extern irqreturn_t (*perf_irq)(void);		// defined in kernel/time.c, used in oprofile
#endif

static DEFINE_SPINLOCK(mips_irq_lock);
static DEFINE_SPINLOCK(mips_fiq_lock);
static DEFINE_SPINLOCK(mips_iiq_lock);

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
	spin_lock_irqsave(&mips_fiq_lock, flags);
	status = (((u64)RD_CPU_REG32(CPU_fiq_status_hi))<<32) | ((u64)RD_CPU_REG32(CPU_fiq_status));
	spin_unlock_irqrestore(&mips_fiq_lock, flags);
	return status;
}

static inline u64 get_iiq_status(void)
{
	u64 status;
	unsigned long flags;
	spin_lock_irqsave(&mips_iiq_lock, flags);
	status = (((u64)RD_CPU_REG32(CPU_iiq_status_hi))<<32) | ((u64)RD_CPU_REG32(CPU_iiq_status));
	spin_unlock_irqrestore(&mips_iiq_lock, flags);
	return status;
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
	u32 xl = (u32)(x & 0xffffffff), xh = (u32)((x >> 32) & 0xffffffff);
	return xh ? clz(xh) : clz(xl) + 32;
}

static inline unsigned int irq_ffs(unsigned int pending)
{
	return -clz(pending) + 31 - CAUSEB_IP;
}

extern int cp0_compare_irq;
extern int cp0_perfcount_irq;

static unsigned long edge_trig = 0;
static unsigned long edge_trig_hi = 0;

extern void spurious_interrupt(void);

#ifdef CONFIG_TANGO4
/* Tango4 */
int gcmp_present = -1;
int gic_present = 0;
static unsigned long _gcmp_base;

#ifdef CONFIG_SMP
/* GIC interrupt numbers for IPI reschedule and calls */
/* We have to squeeze these interrupts in, so it won't be contiguous */
/* CPU0,1,2,3 uses GIC interrupts 1,5,6,7 */
static const unsigned int resched_int_map[NR_CPUS] = { 1, 5, 6, 7, };
static const unsigned int call_int_map[NR_CPUS] = { 1, 5, 6, 7, };
#define GIC_RESCHED_INT(cpu)	resched_int_map[cpu]
#define GIC_CALL_INT(cpu)	call_int_map[cpu]
#endif

/*
 * This table defines the association External
 * Interrupts and CPUs/Core Interrupts. The nature of the External
 * Interrupts is also defined here - polarity/trigger.
 */

static struct gic_intr_map gic_intr_map[GIC_NUM_INTRS] = {
	/* This table could be preconfigured at compile time */
};

static void __init fill_gic_map(void)
{
#ifdef CONFIG_SMP 
	int amon_cpu_avail(int cpu);
#endif
	int i, k;

	/*
	 * Initialise interrupt map for GIC
	 */

	for (i = k = 0; i < GIC_NUM_INTRS; i++) {
		struct gic_intr_map *gic = &gic_intr_map[i];
		/*
		 * Default all interrupts to CPU0/HWInt2
		 * positive polarity level triggered
		 */
		gic->cpunum = 0;
		gic->pin = GIC_CPU_INT2;
		gic->polarity = GIC_POL_POS;
		gic->trigtype = GIC_TRIG_LEVEL;
		gic->flags = 0;

		/*
		 * Route the platform interrupt controllers
		 * on GIC interrupts 2, 3, 4
		 * through the GIC in transparent mode
		 */
		switch (i) {
			case 2:	/* IRQ to HWInt0, SR_IP2 */
			case 3: /* FIQ to HWInt1, SR_IP3 */
			case 4: /* IIQ to HWInt2, SR_IP4 */
#ifdef CONFIG_SMP 
				{
					int j;
					for (j = k; j < num_possible_cpus(); j++) { /* find next available CPU */
						if (amon_cpu_avail(j))
							break;
					}
					gic->cpunum = (j < num_possible_cpus()) ? j : 0;
					gic->flags = GIC_FLAG_TRANSPARENT;
					gic->pin = GIC_CPU_INT0 + (i - 2);
					k = gic->cpunum + 1;
				}
#else
				gic->cpunum = 0;
				gic->flags = GIC_FLAG_TRANSPARENT;
				gic->pin = GIC_CPU_INT0 + (i - 2);
#endif
				break;
			default:
				break;
		}
	}

#ifdef CONFIG_SMP 
	/* 1,5,6,7 will be overwritten by IPI if SMP is enabled */
#endif
}

/* dispatching TANGO4 specific interrupts */
static void tangox_irq_dispatch(void)
{
	u64 status;
	if ((status = get_irq_status()) == 0) {
		printk("spurious irq: 0x%llx\n", status);
		spurious_interrupt();
	} else 
		do_IRQ(IRQ_CONTROLLER_IRQ_BASE + (63 - clz64(status)));
	return;
}

static void tangox_fiq_dispatch(void)
{
	u64 status;
	if ((status = get_fiq_status()) == 0) {
		printk("spurious fiq: 0x%llx\n", status);
		spurious_interrupt();
	} else 
		do_IRQ(FIQ_CONTROLLER_IRQ_BASE + (63 - clz64(status)));
	return;
}

static void tangox_iiq_dispatch(void)
{
	u64 status;
	if ((status = get_iiq_status()) == 0) {
		printk("spurious iiq: 0x%llx\n", status);
		spurious_interrupt();
	} else 
		do_IRQ(IIQ_CONTROLLER_IRQ_BASE + (63 - clz64(status)));
	return;
}

/* No need for now, as we temporarily setting all external interrupts in
   transparent mode */
#ifdef CONFIG_SMP
static void gic_irq_dispatch(void)
{
	int irq = gic_get_int();

	if (irq < 0)
		return;  /* interrupt has already been cleared */

	do_IRQ(MIPS_GIC_IRQ_BASE + irq);
}
#endif

/*
 * dispatch routine called from genex.S
 */
asmlinkage void plat_irq_dispatch(void)
{
	unsigned int pending = read_c0_cause() & read_c0_status() & ST0_IM;
	int irq;

	irq = irq_ffs(pending) + MIPS_CPU_IRQ_BASE;

	switch(irq) {
		case 2: tangox_irq_dispatch();
			break;
		case 3: tangox_fiq_dispatch();
			break;
		case 4: tangox_iiq_dispatch();
			break;
#ifdef CONFIG_SMP
		case 5: gic_irq_dispatch();
			break;
#endif
		default: if (irq >= 0)
				do_IRQ(irq);
			else
				spurious_interrupt();
			break;
	}
	return;
}

#ifdef CONFIG_SMP
unsigned int plat_ipi_call_int_xlate(unsigned int cpu)
{
	pr_debug("CPU%d: %s cpu%d->gicirq%d\n",
		 smp_processor_id(), __func__, cpu, GIC_CALL_INT(cpu));
	return GIC_CALL_INT(cpu);
}

unsigned int plat_ipi_resched_int_xlate(unsigned int cpu)
{
	pr_debug("CPU%d: %s cpu%d->gicirq%d\n",
		 smp_processor_id(), __func__, cpu, GIC_RESCHED_INT(cpu));
	return GIC_RESCHED_INT(cpu);
}

#if 0
static irqreturn_t ipi_resched_interrupt(int irq, void *dev_id)
{
	pr_debug("CPU%d: [%s:%d]\n", smp_processor_id(), __FILE__, __LINE__);
	scheduler_ipi();
	return IRQ_HANDLED;
}
#endif

static irqreturn_t ipi_call_interrupt(int irq, void *dev_id)
{
	pr_debug("CPU%d: [%s:%d]\n", smp_processor_id(), __FILE__, __LINE__);
#if 1
	scheduler_ipi();
#endif
	smp_call_function_interrupt();
	return IRQ_HANDLED;
}

#if 0
static struct irqaction irq_resched = {
	.handler    = ipi_resched_interrupt,
	.flags      = IRQF_DISABLED|IRQF_PERCPU|IRQF_NO_THREAD,
	.name       = "IPI_resched"
};
#endif

static struct irqaction irq_call = {
	.handler    = ipi_call_interrupt,
	.flags      = IRQF_DISABLED|IRQF_PERCPU|IRQF_NO_THREAD,
	.name       = "IPI_call"
};

/* GIC IPI initialisation */
static void __init init_ipiirq(int irq, struct irqaction *action)
{
	setup_irq(irq, action);
	irq_set_handler(irq, handle_percpu_irq);
}

static void __init fill_ipi_map1(int baseintr, int cpu, int cpupin)
{
	int intr = baseintr + cpu;
	gic_intr_map[intr].cpunum = cpu;
	gic_intr_map[intr].pin = cpupin;
	gic_intr_map[intr].polarity = GIC_POL_POS;
	gic_intr_map[intr].trigtype = GIC_TRIG_EDGE;
	gic_intr_map[intr].flags = GIC_FLAG_IPI;
}

static void __init fill_ipi_map(void)
{
	int cpu;

	/* Use gic_irqdispatch to handle IPI */
	for (cpu = 0; cpu < NR_CPUS; cpu++) {
		if (cpu != 0)
			fill_ipi_map1(4, cpu, GIC_CPU_INT3);
		else
			fill_ipi_map1(1, cpu, GIC_CPU_INT3);
	}
}
#endif /* CONFIG_SMP */
#else
/* Tango2/3 */
/*
 * dispatch routine called from genex.S
 */
asmlinkage void plat_irq_dispatch(void)
{
	unsigned int pending = (read_c0_cause() & read_c0_status()) & ST0_IM;
	int irq;
	u64 status;

	if (pending == 0) {
		printk("spurious hwirq: nothing pending\n");
		goto spurious;
	} else {
		irq = irq_ffs(pending);

		switch(irq) {
			case 2: if ((status = get_irq_status()) == 0) {
					printk("spurious irq: 0x%llx\n", status);
					goto spurious;
				} else 
					do_IRQ(IRQ_CONTROLLER_IRQ_BASE + (63 - clz64(status)));
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

			default: if ((irq == cp0_compare_irq)
#if defined(CONFIG_TANGO3) 
					|| (irq == cp0_perfcount_irq)
#endif
				) {
					do_IRQ(irq);
				} else { 
					printk("spurious hwirq: %d\n", irq);
					goto spurious;
				}
				break;
		}
	}
	return;

spurious:
	spurious_interrupt();
	return;
}
#endif

/*
 * our hw_irq_controller callbacks
 */
static inline void tangox_irq_enable(struct irq_data *data)
{
	int bit = data->irq - IRQ_CONTROLLER_IRQ_BASE;
	unsigned long flags;

	spin_lock_irqsave(&mips_irq_lock, flags);
	if (bit >= 32) 
		WR_CPU_REG32(CPU_irq_enableset_hi, 1 << (bit - 32));
	else 
		WR_CPU_REG32(CPU_irq_enableset, 1 << bit);
	spin_unlock_irqrestore(&mips_irq_lock, flags);
}

static inline void tangox_fiq_enable(struct irq_data *data)
{
	int bit = data->irq - FIQ_CONTROLLER_IRQ_BASE;
	unsigned long flags;

	spin_lock_irqsave(&mips_fiq_lock, flags);
	if (bit >= 32) 
		WR_CPU_REG32(CPU_fiq_enableset_hi, 1 << (bit - 32));
	else
		WR_CPU_REG32(CPU_fiq_enableset, 1 << bit);
	spin_unlock_irqrestore(&mips_fiq_lock, flags);
}

static inline void tangox_iiq_enable(struct irq_data *data)
{
	int bit = data->irq - IIQ_CONTROLLER_IRQ_BASE;
	unsigned long flags;

	spin_lock_irqsave(&mips_iiq_lock, flags);
	if (bit >= 32) 
		WR_CPU_REG32(CPU_iiq_enableset_hi, 1 << (bit - 32));
	else
		WR_CPU_REG32(CPU_iiq_enableset, 1 << bit);
	spin_unlock_irqrestore(&mips_iiq_lock, flags);
}

static inline void tangox_irq_disable(struct irq_data *data)
{
	int bit = data->irq - IRQ_CONTROLLER_IRQ_BASE;
	unsigned long flags;

	spin_lock_irqsave(&mips_irq_lock, flags);
	if (bit >= 32) 
		WR_CPU_REG32(CPU_irq_enableclr_hi, 1 << (bit - 32));
	else
		WR_CPU_REG32(CPU_irq_enableclr, 1 << bit);
	spin_unlock_irqrestore(&mips_irq_lock, flags);
}

static inline void tangox_fiq_disable(struct irq_data *data)
{
	int bit = data->irq - FIQ_CONTROLLER_IRQ_BASE;
	unsigned long flags;

	spin_lock_irqsave(&mips_fiq_lock, flags);
	if (bit >= 32) 
		WR_CPU_REG32(CPU_fiq_enableclr_hi, 1 << (bit - 32));
	else
		WR_CPU_REG32(CPU_fiq_enableclr, 1 << bit);
	spin_unlock_irqrestore(&mips_fiq_lock, flags);
}

static inline void tangox_iiq_disable(struct irq_data *data)
{
	int bit = data->irq - IIQ_CONTROLLER_IRQ_BASE;
	unsigned long flags;

	spin_lock_irqsave(&mips_iiq_lock, flags);
	if (bit >= 32) 
		WR_CPU_REG32(CPU_iiq_enableclr_hi, 1 << (bit - 32));
	else
		WR_CPU_REG32(CPU_iiq_enableclr, 1 << bit);
	spin_unlock_irqrestore(&mips_iiq_lock, flags);
}

static unsigned int tangox_irq_startup(struct irq_data *data)
{
	int bit = data->irq - IRQ_CONTROLLER_IRQ_BASE;

	/* clear any pending interrupt before enabling it */
	if (bit >= 32) {
		WR_CPU_REG32(CPU_edge_rawstat_hi, 1 << (bit - 32));
	} else {
		WR_CPU_REG32(CPU_edge_rawstat, 1 << bit);
	}

	tangox_irq_enable(data);
	return 0;
}

static unsigned int tangox_fiq_startup(struct irq_data *data)
{
	int bit = data->irq - FIQ_CONTROLLER_IRQ_BASE;

	/* clear any pending interrupt before enabling it */
	if (bit >= 32) {
		WR_CPU_REG32(CPU_edge_rawstat_hi, 1 << (bit - 32));
	} else {
		WR_CPU_REG32(CPU_edge_rawstat, 1 << bit);
	}

	tangox_fiq_enable(data);
	return 0;
}

static unsigned int tangox_iiq_startup(struct irq_data *data)
{
	int bit = data->irq - IIQ_CONTROLLER_IRQ_BASE;

	/* clear any pending interrupt before enabling it */
	if (bit >= 32) {
		WR_CPU_REG32(CPU_edge_rawstat_hi, 1 << (bit - 32));
	} else {
		WR_CPU_REG32(CPU_edge_rawstat, 1 << bit);
	}

	tangox_iiq_enable(data);
	return 0;
}

#define	tangox_irq_shutdown tangox_irq_disable
#define	tangox_fiq_shutdown tangox_fiq_disable
#define	tangox_iiq_shutdown tangox_iiq_disable

static void tangox_irq_ack(struct irq_data *data)
{
	int bit = data->irq - IRQ_CONTROLLER_IRQ_BASE;

	tangox_irq_disable(data);

	if (bit >= 32) {
		WR_CPU_REG32(CPU_edge_rawstat_hi, 1 << (bit - 32));
	} else {
		WR_CPU_REG32(CPU_edge_rawstat, 1 << bit);
	}
}

static void tangox_fiq_ack(struct irq_data *data)
{
	int bit = data->irq - FIQ_CONTROLLER_IRQ_BASE;

	tangox_fiq_disable(data);

	if (bit >= 32) {
		WR_CPU_REG32(CPU_edge_rawstat_hi, 1 << (bit - 32));
	} else {
		WR_CPU_REG32(CPU_edge_rawstat, 1 << bit);
	}
}

static void tangox_iiq_ack(struct irq_data *data)
{
	int bit = data->irq - IIQ_CONTROLLER_IRQ_BASE;

	tangox_iiq_disable(data);

	if (bit >= 32) {
		WR_CPU_REG32(CPU_edge_rawstat_hi, 1 << (bit - 32));
	} else {
		WR_CPU_REG32(CPU_edge_rawstat, 1 << bit);
	}
}

static void tangox_irq_end(struct irq_data *data)
{
	if (!irqd_irq_disabled(data) && !irqd_irq_inprogress(data))
		tangox_irq_enable(data);
}

static void tangox_fiq_end(struct irq_data *data)
{
	if (!irqd_irq_disabled(data) && !irqd_irq_inprogress(data))
		tangox_fiq_enable(data);
}

static void tangox_iiq_end(struct irq_data *data)
{
	if (!irqd_irq_disabled(data) && !irqd_irq_inprogress(data))
		tangox_iiq_enable(data);
}

/*
 * our hw_irq_controllers
 */
static struct irq_chip tangox_irq_controller = {
	.name = "tangox_irq",
	.irq_startup = tangox_irq_startup,
	.irq_shutdown = tangox_irq_shutdown,
	.irq_enable = tangox_irq_enable,
	.irq_disable = tangox_irq_disable,
	.irq_ack = tangox_irq_ack,
	.irq_mask = tangox_irq_disable,
	.irq_mask_ack = tangox_irq_ack,
	.irq_unmask = tangox_irq_enable,
	.irq_eoi = tangox_irq_end,
};

static struct irq_chip tangox_fiq_controller = {
	.name = "tangox_fiq",
	.irq_startup = tangox_fiq_startup,
	.irq_shutdown = tangox_fiq_shutdown,
	.irq_enable = tangox_fiq_enable,
	.irq_disable = tangox_fiq_disable,
	.irq_ack = tangox_fiq_ack,
	.irq_mask = tangox_fiq_disable,
	.irq_mask_ack = tangox_fiq_ack,
	.irq_unmask = tangox_fiq_enable,
	.irq_eoi = tangox_fiq_end,
};

static struct irq_chip tangox_iiq_controller = {
	.name = "tangox_iiq",
	.irq_startup = tangox_iiq_startup,
	.irq_shutdown = tangox_iiq_shutdown,
	.irq_enable = tangox_iiq_enable,
	.irq_disable = tangox_iiq_disable,
	.irq_ack = tangox_iiq_ack,
	.irq_mask = tangox_iiq_disable,
	.irq_mask_ack = tangox_iiq_ack,
	.irq_unmask = tangox_iiq_enable,
	.irq_eoi = tangox_iiq_end,
};

#if defined(CONFIG_TANGO4)
#ifdef CONFIG_SMP
static struct irqaction gicirq = {
	.handler	= no_action,
	.flags		= IRQF_SHARED|IRQF_NO_THREAD,
	.name		= "GIC",
	.dev_id		= NULL,
	.next		= NULL,
};
#endif
#endif

static struct irqaction irq_cascade = {
	.handler	= no_action,
	.flags		= IRQF_SHARED|IRQF_NO_THREAD,
	.name		= "cascade",
	.dev_id		= NULL,
	.next		= NULL,
};

#if defined(CONFIG_TANGO3)
static irqreturn_t handle_perf_irq(int irqn, void *dummy) 
{
	return perf_irq();
}

static struct irqaction irq_perf_counters = {
	.handler	= handle_perf_irq,
	.flags		= IRQF_DISABLED|IRQF_NO_THREAD,
	.name		= "perf_counter",
	.dev_id		= NULL,
	.next		= NULL,
};
#endif

#if defined(CONFIG_TANGO4)
void __cpuinit setup_cpuirq_dispatcher(unsigned int irq, struct irqaction *action, void (*dispatch)(void))
{
	if (cpu_has_vint)
		set_vi_handler(irq, dispatch);
	setup_irq(irq, action);
	irq_set_handler(irq, NULL);
}
#ifdef CONFIG_SMP
extern struct irqaction c0_compare_irqaction;
static void mips_timer_dispatch(void)
{
	do_IRQ(MIPS_CPU_IRQ_BASE + cp0_compare_irq);
}
#endif
#endif

void __init arch_init_irq(void)
{
	unsigned long x;
	unsigned long rise = 0;
	unsigned long fall = 0;
	unsigned long rise_hi = 0;
	unsigned long fall_hi = 0;

	/* irq_desc entries 0..7 */
	mips_cpu_irq_init();

	/* initialize IRQ/FIQ/IIQ */
	WR_CPU_REG32(CPU_irq_enableclr, 0xffffffff);
	WR_CPU_REG32(CPU_fiq_enableclr, 0xffffffff);
	WR_CPU_REG32(CPU_iiq_enableclr, 0xffffffff);
	WR_CPU_REG32(CPU_irq_enableclr_hi, 0xffffffff);
	WR_CPU_REG32(CPU_fiq_enableclr_hi, 0xffffffff);
	WR_CPU_REG32(CPU_iiq_enableclr_hi, 0xffffffff);

	rise = RD_CPU_REG32(CPU_edge_config_rise);
	fall = RD_CPU_REG32(CPU_edge_config_fall);
	edge_trig = rise ^ fall;
	WR_CPU_REG32(CPU_edge_rawstat, 0xffffffff);
	rise_hi = RD_CPU_REG32(CPU_edge_config_rise_hi);
	fall_hi = RD_CPU_REG32(CPU_edge_config_fall_hi);
	edge_trig_hi = rise_hi ^ fall_hi;
	WR_CPU_REG32(CPU_edge_rawstat_hi, 0xffffffff);

#ifdef CONFIG_TANGO4
	_gcmp_base = KSEG1ADDR(GCMP_BASE_ADDR);
	gcmp_present = (GCMPGCB(GCMPB) & GCMP_GCB_GCMPB_GCMPBASE_MSK) == GCMP_BASE_ADDR;

	if (gcmp_present)  {
		/* set up GIC base, may need be done at bootloader level? */
		GCMPGCB(GICBA) = GIC_BASE_ADDR | GCMP_GCB_GICBA_EN_MSK;
	}
	
	setup_cpuirq_dispatcher(MIPS_CPU_IRQ_BASE + 2, &irq_cascade, tangox_irq_dispatch);
	setup_cpuirq_dispatcher(MIPS_CPU_IRQ_BASE + 3, &irq_cascade, tangox_fiq_dispatch);
	setup_cpuirq_dispatcher(MIPS_CPU_IRQ_BASE + 4, &irq_cascade, tangox_iiq_dispatch);
#endif

	/* for Tango2/3, interrupt enumerations,
		0..7: MIPS HWInts, 8..71: IRQ, 72..135: FIQ, 136..199: IIQ 
	   for Tango4, 
		0..7: MIPS HWInts, 8..64: GIC, 
		64..127: IRQ, 128..191: FIQ, 192..255: IIQ
	*/
	for (x = IRQ_CONTROLLER_IRQ_BASE; x < IRQ_CONTROLLER_IRQ_BASE + IRQ_COUNT; x++) 
		irq_set_chip_and_handler(x, &tangox_irq_controller, handle_level_irq);

	for (x = FIQ_CONTROLLER_IRQ_BASE; x < FIQ_CONTROLLER_IRQ_BASE + IRQ_COUNT; x++) 
		irq_set_chip_and_handler(x, &tangox_fiq_controller, handle_level_irq);

	for (x = IIQ_CONTROLLER_IRQ_BASE; x < IIQ_CONTROLLER_IRQ_BASE + IRQ_COUNT; x++) 
		irq_set_chip_and_handler(x, &tangox_iiq_controller, handle_level_irq);

#ifdef CONFIG_TANGO4
	/* Initialise GIC */
	fill_gic_map();
#ifdef CONFIG_SMP
	fill_ipi_map();
	set_vi_handler(MIPS_CPU_IRQ_BASE + cp0_compare_irq, mips_timer_dispatch);
	irq_set_handler(MIPS_CPU_IRQ_BASE + cp0_compare_irq, handle_percpu_irq);
	setup_cpuirq_dispatcher(MIPS_CPU_IRQ_BASE + 5, &gicirq, gic_irq_dispatch);
#endif
	gic_init(GIC_BASE_ADDR, GIC_ADDRSPACE_SZ, gic_intr_map, ARRAY_SIZE(gic_intr_map), MIPS_GIC_IRQ_BASE);

#ifdef CONFIG_SMP
	for (x = 0; x < NR_CPUS; x++) {
#if 0
		/* Due to the lack of GIC interrupt, don't register resched interrupt. 
		 * Let the call interrupt take care of reched interrupt as well, but with some overheads. */
		init_ipiirq(MIPS_GIC_IRQ_BASE + GIC_RESCHED_INT(x), &irq_resched);
#endif
		init_ipiirq(MIPS_GIC_IRQ_BASE + GIC_CALL_INT(x), &irq_call);
	}
	/* Enable HwInt5, HwInt3, HwInt2, HwInt1 & HWInt0 */
	change_c0_status(ST0_IM, STATUSF_IP7 | STATUSF_IP5 | STATUSF_IP4 | STATUSF_IP3 | STATUSF_IP2);
#else
	/* Enable HwInt2, HwInt1 & HWInt0 */
	change_c0_status(ST0_IM, STATUSF_IP4 | STATUSF_IP3 | STATUSF_IP2);
#endif
#else
	/* Tango2/3 */
	setup_irq(MIPS_CPU_IRQ_BASE + 2, &irq_cascade);
	setup_irq(MIPS_CPU_IRQ_BASE + 3, &irq_cascade);
	setup_irq(MIPS_CPU_IRQ_BASE + 4, &irq_cascade);

#if defined(CONFIG_TANGO3)
	if (cp0_perfcount_irq > 0)
		setup_irq(cp0_perfcount_irq, &irq_perf_counters);
#endif
#endif

	return;
}

