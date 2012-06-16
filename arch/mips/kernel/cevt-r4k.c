/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2007 MIPS Technologies, Inc.
 * Copyright (C) 2007 Ralf Baechle <ralf@linux-mips.org>
 */
#include <linux/clockchips.h>
#include <linux/interrupt.h>
#include <linux/percpu.h>
#include <linux/smp.h>
#include <linux/irq.h>

#include <asm/smtc_ipi.h>
#include <asm/time.h>
#include <asm/cevt-r4k.h>
#include <asm/gic.h>

#ifdef CONFIG_TANGOX
#include <linux/export.h>
#if defined(CONFIG_TANGO2)
#include <asm/tango2/tango2.h>
#include <asm/tango2/tango2_gbus.h>
#include <asm/tango2/emhwlib_registers_tango2.h>
#include <asm/tango3/emhwlib_lram_others.h>
#elif defined(CONFIG_TANGO3)
#include <asm/tango3/tango3.h>
#include <asm/tango3/tango3_gbus.h>
#include <asm/tango3/emhwlib_registers_tango3.h>
#include <asm/tango3/emhwlib_lram_tango3.h>
#elif defined(CONFIG_TANGO4)
#include <asm/tango4/tango4.h>
#include <asm/tango4/tango4_gbus.h>
#include <asm/tango4/emhwlib_registers_tango4.h>
#include <asm/tango4/emhwlib_lram_tango4.h>
#endif
void reset_timer(unsigned long cpu, unsigned long sys, unsigned long pll, unsigned long premux, unsigned long mux);
extern unsigned long tangox_get_cpuclock(void);
extern unsigned long tangox_get_sysclock(void);
extern unsigned long em8xxx_cpu_frequency;
extern unsigned long em8xxx_sys_frequency;
extern unsigned long em8xxx_sys_clkgen_pll;
extern unsigned long em8xxx_sys_premux;
extern unsigned long em8xxx_sys_mux;
extern unsigned long orig_cpu_freq;
#endif /* CONFIG_TANGOX */

/*
 * The SMTC Kernel for the 34K, 1004K, et. al. replaces several
 * of these routines with SMTC-specific variants.
 */

#ifndef CONFIG_MIPS_MT_SMTC

static int mips_next_event(unsigned long delta,
                           struct clock_event_device *evt)
{
	unsigned int cnt;
	int res;

	cnt = read_c0_count();
	cnt += delta;
	write_c0_compare(cnt);
	res = ((int)(read_c0_count() - cnt) >= 0) ? -ETIME : 0;
	return res;
}

#endif /* CONFIG_MIPS_MT_SMTC */

void mips_set_clock_mode(enum clock_event_mode mode,
				struct clock_event_device *evt)
{
	/* Nothing to do ...  */
}

DEFINE_PER_CPU(struct clock_event_device, mips_clockevent_device);
int cp0_timer_irq_installed;

#ifndef CONFIG_MIPS_MT_SMTC

irqreturn_t c0_compare_interrupt(int irq, void *dev_id)
{
	const int r2 = cpu_has_mips_r2;
	struct clock_event_device *cd;
	int cpu = smp_processor_id();

#ifdef CONFIG_TANGOX
	static unsigned long last_jiffies = ((unsigned long)(unsigned int)(-600*HZ));
#ifndef CONFIG_TANGOX_FIXED_FREQUENCIES
	unsigned long clkgen_pll = gbus_read_reg32(REG_BASE_system_block + SYS_clkgen_pll);
	unsigned long premux = gbus_read_reg32(REG_BASE_system_block + SYS_sysclk_premux) & 0x3;
	unsigned long mux = gbus_read_reg32(REG_BASE_system_block + SYS_sysclk_mux) & 0xf01;
#endif

	if (time_after(jiffies, last_jiffies + HZ)) { 
		last_jiffies = jiffies;
		/* Update heart beat counter */
		gbus_write_reg32(REG_BASE_cpu_block + LR_HB_CPU,
				gbus_read_reg32(REG_BASE_cpu_block + LR_HB_CPU) + 1);
#if defined(CONFIG_PRINTK_TIME)
//		printk("*** time marker *** (0x%08x)\n", gbus_read_reg32(REG_BASE_system_block + SYS_xtal_in_cnt));
#endif
	}

#ifndef CONFIG_TANGOX_FIXED_FREQUENCIES
	if ((em8xxx_sys_clkgen_pll != clkgen_pll) || (em8xxx_sys_premux != premux) || (em8xxx_sys_mux != mux)) {
		/* Detected potential CPU/System frequency change */
		reset_timer(tangox_get_cpuclock(), tangox_get_sysclock(), clkgen_pll, premux, mux);
        }
#endif
#endif /* CONFIG_TANGOX */

	/*
	 * Suckage alert:
	 * Before R2 of the architecture there was no way to see if a
	 * performance counter interrupt was pending, so we have to run
	 * the performance counter interrupt handler anyway.
	 */
	if (handle_perf_irq(r2))
		goto out;

	/*
	 * The same applies to performance counter interrupts.  But with the
	 * above we now know that the reason we got here must be a timer
	 * interrupt.  Being the paranoiacs we are we check anyway.
	 */
	if (!r2 || (read_c0_cause() & (1 << 30))) {
		/* Clear Count/Compare Interrupt */
		write_c0_compare(read_c0_compare());
		cd = &per_cpu(mips_clockevent_device, cpu);
		cd->event_handler(cd);
	}

out:
	return IRQ_HANDLED;
}

#endif /* Not CONFIG_MIPS_MT_SMTC */

struct irqaction c0_compare_irqaction = {
	.handler = c0_compare_interrupt,
	.flags = IRQF_PERCPU | IRQF_TIMER,
	.name = "timer",
};


void mips_event_handler(struct clock_event_device *dev)
{
}

/*
 * FIXME: This doesn't hold for the relocated E9000 compare interrupt.
 */
static int c0_compare_int_pending(void)
{
#ifdef CONFIG_MIPS_SEAD3
	if (cpu_has_veic)
		return gic_get_timer_pending();
#endif
	return (read_c0_cause() >> cp0_compare_irq_shift) & (1ul << CAUSEB_IP);
}

/*
 * Compare interrupt can be routed and latched outside the core,
 * so wait up to worst case number of cycle counter ticks for timer interrupt
 * changes to propagate to the cause register.
 */
#define COMPARE_INT_SEEN_TICKS 50

int c0_compare_int_usable(void)
{
	unsigned int delta;
	unsigned int cnt;

	/*
	 * IP7 already pending?  Try to clear it by acking the timer.
	 */
	if (c0_compare_int_pending()) {
		cnt = read_c0_count();
		write_c0_compare(cnt);
		back_to_back_c0_hazard();
		while (read_c0_count() < (cnt  + COMPARE_INT_SEEN_TICKS))
			if (!c0_compare_int_pending())
				break;
		if (c0_compare_int_pending())
			return 0;
	}

	for (delta = 0x10; delta <= 0x400000; delta <<= 1) {
		cnt = read_c0_count();
		cnt += delta;
		write_c0_compare(cnt);
		back_to_back_c0_hazard();
		if ((int)(read_c0_count() - cnt) < 0)
		    break;
		/* increase delta if the timer was already expired */
	}

	while ((int)(read_c0_count() - cnt) <= 0)
		;	/* Wait for expiry  */

	while (read_c0_count() < (cnt + COMPARE_INT_SEEN_TICKS))
		if (c0_compare_int_pending())
			break;
	if (!c0_compare_int_pending())
		return 0;
	cnt = read_c0_count();
	write_c0_compare(cnt);
	back_to_back_c0_hazard();
	while (read_c0_count() < (cnt + COMPARE_INT_SEEN_TICKS))
		if (!c0_compare_int_pending())
			break;
	if (c0_compare_int_pending())
		return 0;

	/*
	 * Feels like a real count / compare timer.
	 */
	return 1;
}

#ifndef CONFIG_MIPS_MT_SMTC

int __cpuinit r4k_clockevent_init(void)
{
	unsigned int cpu = smp_processor_id();
	struct clock_event_device *cd;
	unsigned int irq;

	if (!cpu_has_counter || !mips_hpt_frequency)
		return -ENXIO;

	if (!c0_compare_int_usable())
		return -ENXIO;

	/*
	 * With vectored interrupts things are getting platform specific.
	 * get_c0_compare_int is a hook to allow a platform to return the
	 * interrupt number of it's liking.
	 */
	irq = MIPS_CPU_IRQ_BASE + cp0_compare_irq;
	if (get_c0_compare_int)
		irq = get_c0_compare_int();

	cd = &per_cpu(mips_clockevent_device, cpu);

	cd->name		= "MIPS";
	cd->features		= CLOCK_EVT_FEAT_ONESHOT;

	clockevent_set_clock(cd, mips_hpt_frequency);

	/* Calculate the min / max delta */
	cd->max_delta_ns	= clockevent_delta2ns(0x7fffffff, cd);
	cd->min_delta_ns	= clockevent_delta2ns(0x300, cd);

	cd->rating		= 300;
	cd->irq			= irq;
	cd->cpumask		= cpumask_of(cpu);
	cd->set_next_event	= mips_next_event;
	cd->set_mode		= mips_set_clock_mode;
	cd->event_handler	= mips_event_handler;

	clockevents_register_device(cd);

	if (cp0_timer_irq_installed)
		return 0;

	cp0_timer_irq_installed = 1;

	setup_irq(irq, &c0_compare_irqaction);

	return 0;
}

#endif /* Not CONFIG_MIPS_MT_SMTC */

#ifdef CONFIG_TANGOX
#ifndef CONFIG_TANGOX_FIXED_FREQUENCIES
void reset_timer(unsigned long cpuf, unsigned long sysf, unsigned long pll, unsigned long premux, unsigned long mux)
{
	unsigned int cpu = smp_processor_id();
	struct clock_event_device *cd;
	unsigned long flags;

	printk("Detected frequency changed ...\n");

	em8xxx_sys_clkgen_pll = pll;
	em8xxx_sys_premux = premux;
	em8xxx_sys_mux = mux;
	em8xxx_cpu_frequency = cpuf;
	em8xxx_sys_frequency = sysf;

	cd = &per_cpu(mips_clockevent_device, cpu);
	clockevents_set_mode(cd, CLOCK_EVT_MODE_SHUTDOWN);
	local_irq_save(flags);
	mips_hpt_frequency = em8xxx_cpu_frequency / 2;
	clockevent_set_clock(cd, mips_hpt_frequency);
	cd->max_delta_ns	= clockevent_delta2ns(0x7fffffff, cd);
	cd->min_delta_ns	= clockevent_delta2ns(0x300, cd);
	local_irq_restore(flags);
	clockevents_set_mode(cd, CLOCK_EVT_MODE_ONESHOT);
}
EXPORT_SYMBOL(reset_timer);
#endif
#endif /* CONFIG_TANGOX */
