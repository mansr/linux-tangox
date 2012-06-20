
/*********************************************************************
 Copyright (C) 2001-2011
 Sigma Designs, Inc. 

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/

/*
 * arch/mips/tangox/delay.c
 *
 * Copyright (C) 2003-2009 Sigma Designs, Inc
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <asm/io.h>
#include <linux/irq.h>
#include <linux/sched.h>

#include "setup.h"

/*
 * helpers to access cpu block registers
 */
#define RD_CPU_REG32(r)	\
		gbus_read_reg32(REG_BASE_cpu_block + (r))

#define WR_CPU_REG32(r, v)	\
		gbus_write_reg32(REG_BASE_cpu_block + (r), (v))

#define MAX_POLL_TIME		(25)	/* less than this we use busy polling */

typedef int (*COND_FUNC_PTR)(void *);

static unsigned long timer_status = 0;

/* given micro seconds, rerurn corresponding cp0 timer cout */
static inline unsigned long tangox_get_c0_timecount(unsigned usec)
{
	extern unsigned long em8xxx_cpu_frequency;
	unsigned long cnt = em8xxx_cpu_frequency / (1000000 * 2);
	if ((cnt * (1000000 * 2)) < em8xxx_cpu_frequency)
		cnt++; /* take the ceiling */
	return (usec * cnt);
}

/* check if given timestamp is in the time range of [start, end) */
static inline int tangox_time_in_range(unsigned long start, unsigned long end, unsigned long timestamp)
{
	return (end > start) ? ((timestamp < end) && (timestamp >= start)) : ((timestamp < end) || (timestamp >= start));
}

/* wait until we are outside the time range [start, end), or condition is false */
static inline int tangox_wait_until(unsigned long start, unsigned long end, COND_FUNC_PTR func_ptr, void *arg)
{
	unsigned long timestamp;

	/* wait until it's out of range, or condition evaluation is false */
	for (timestamp = read_c0_count(); tangox_time_in_range(start, end, timestamp); timestamp = read_c0_count()) {
		if ((func_ptr != NULL) && (((*func_ptr)(arg)) == 0))
			return 0;
	}
	return (func_ptr != NULL) ? (*func_ptr)(arg) : 1; /* time's up and condition may be still true */
}

/* cp0_timer is used for system timer, and timer0 is free to be used here */
/*
 * CPU_time0_load
 *  clock / HZ / (2 * prescale)
 * CPU_time0_ctrl
 *  PS(D2-3) : prescale. 0x00 = 1, 0x01 = 16, 0x10 = 256
 *    There is a bug, and the actual prescale is 0x01 = 32, 0x10 = 512
 *  M(D6) : periodic mode
 *  E(D7) : enable
 */
#define TIMER_ENABLE            0x80    // D7
#define TIMER_PERIODIC          0x40    // D6
#define TIMER_PRESCALE_1        0x00    // D[2-3] = 00b
#define TIMER_PRESCALE_32       0x04    // D[2-3] = 01b
#define TIMER_PRESCALE_512      0x08    // D[2-3] = 10b

static const unsigned timer_prescale_bit[3] = { 0, 5, 9 };
static const unsigned timer_prescale[3] = { TIMER_PRESCALE_1, TIMER_PRESCALE_32, TIMER_PRESCALE_512 };

static DECLARE_WAIT_QUEUE_HEAD(timer0_wq);
static int timer0_expired = 0, timer0_init = 0;
static spinlock_t timer0_lock;		/* Spin lock */
static unsigned int timer0_val = 0, timer0_idx = 0;

static irqreturn_t timer0_isr(int irq, void *dev_id)
{
	wait_queue_head_t *q = (wait_queue_head_t *)dev_id;
	unsigned long flags;

	if (q != &timer0_wq)		/* Paranoid check */
		return IRQ_NONE;

	spin_lock_irqsave(&timer0_lock, flags);
	WR_CPU_REG32(CPU_time0_ctrl, 0); /* disable timer */
	WR_CPU_REG32(CPU_time0_clr, 1);
	timer0_expired = 1;
	wake_up_interruptible(&timer0_wq);

	spin_unlock_irqrestore(&timer0_lock, flags);
	return IRQ_HANDLED;
}

/*
 * udelay with timer0 implementation if possible
 */
int tangox_udelay_with_condition(unsigned int usecond, COND_FUNC_PTR func_ptr, void *arg)
{
	unsigned long start = read_c0_count();
	unsigned long end = start + tangox_get_c0_timecount(usecond);
	unsigned long flags, tmp;

	if (unlikely(usecond == 0))
		return 1;
	else if ((func_ptr == NULL) || (usecond < MAX_POLL_TIME))
		goto poll;	/* no condition or too short */
	else if (in_atomic() || in_interrupt() || ((current != NULL) && signal_pending(current)))
		goto poll;	/* not safe for context switching */
	else if (unlikely(timer0_init == 0)) 
		goto poll;
	else if (test_and_set_bit(0, &timer_status) != 0)
		goto poll;

	while (usecond >= MAX_POLL_TIME) { 
		if ((func_ptr != NULL) && (((*func_ptr)(arg)) == 0)) { /* evaluate condition first */
			clear_bit(0, &timer_status);
			return 0;
		}
		if (!tangox_time_in_range(start, end, read_c0_count())) {
			clear_bit(0, &timer_status);
			return (func_ptr != NULL) ? (*func_ptr)(arg) : 1; /* time's up and condition may be still true */
		}

		/* CPU_time0_value register contains just 16-bits value so
		   take care not to let the value to overflow */
		for (timer0_idx = 0; timer0_idx < 3; timer0_idx++) {
			tmp = (MAX_POLL_TIME * (em8xxx_sys_frequency / 1000)) / 1000; /* MAX_POLL_TIME needs to be <= 10000 */
			timer0_val = tmp >> timer_prescale_bit[timer0_idx];
			if (tmp & ((1 << timer_prescale_bit[timer0_idx]) - 1))
				timer0_val++;
			if ((timer0_val & 0xffff0000) == 0)
				break;
		}
		if (timer0_idx >= 3)	/* no prescaling factor can be found */
			break;

		spin_lock_irqsave(&timer0_lock, flags);
		timer0_expired = 0;
		WR_CPU_REG32(CPU_time0_load, timer0_val);
		WR_CPU_REG32(CPU_time0_ctrl, TIMER_ENABLE | TIMER_PERIODIC | timer_prescale[timer0_idx]);
		spin_unlock_irqrestore(&timer0_lock, flags);

		/* sleep in the wait queue until timer expired, or timeout */
		if (wait_event_interruptible(timer0_wq, timer0_expired != 0)) {
			/* received signal -- switch to busy polling then */
			spin_lock_irqsave(&timer0_lock, flags);
			WR_CPU_REG32(CPU_time0_ctrl, 0); /* disable timer */
			WR_CPU_REG32(CPU_time0_clr, 1);
			spin_unlock_irqrestore(&timer0_lock, flags);
			break;
		}
		usecond -= MAX_POLL_TIME;
	}
	clear_bit(0, &timer_status);

poll:
	return tangox_wait_until(start, end, func_ptr, arg);
}

int __init tangox_timer0_init(void)
{
	unsigned int tmp;

	WR_CPU_REG32(CPU_time0_ctrl, 0); /* disable timer to begin with */
	WR_CPU_REG32(CPU_time0_clr, 1);
	spin_lock_init(&timer0_lock);

	if (request_irq(LOG2_CPU_TIMER0_INT + IRQ_CONTROLLER_IRQ_BASE, timer0_isr, IRQF_DISABLED|IRQF_NO_THREAD, "timer0", 
			&timer0_wq) != 0) {
		printk(KERN_ERR "timer0: cannot register IRQ (%d)\n", LOG2_CPU_TIMER0_INT + IRQ_CONTROLLER_IRQ_BASE);
		return -EIO;
	}

	/* CPU_time0_value register contains just 16-bits value So
	   take care not to let the value to overflow */
	for (timer0_idx = 0; timer0_idx < 3; timer0_idx++) {
		tmp = (MAX_POLL_TIME * (em8xxx_sys_frequency / 1000)) / 1000; /* MAX_POLL_TIME needs to be <= 10000 */
		timer0_val = tmp >> timer_prescale_bit[timer0_idx];
		if ((timer0_val & 0xffff0000) == 0) {
			if (tmp & ((1 << timer_prescale_bit[timer0_idx]) - 1))
				timer0_val++;
			break;
		}
	}
	timer0_init = 1;
	printk(KERN_INFO "timer0: interrupt registered.\n");
	return 0;
}

__initcall(tangox_timer0_init);

void tangox_udelay(unsigned int usec)
{
	tangox_udelay_with_condition(usec, NULL, NULL);
}

EXPORT_SYMBOL(tangox_udelay);
EXPORT_SYMBOL(tangox_udelay_with_condition);

