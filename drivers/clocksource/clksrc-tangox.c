#include <linux/init.h>
#include <linux/clk.h>
#include <linux/clocksource.h>
#include <linux/sched_clock.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <asm/delay.h>

static void __iomem *tick_count;

static u64 notrace tangox_csrc_sched_read(void)
{
	return readl(tick_count);
}

#ifdef CONFIG_ARM
static unsigned long tangox_read_delay_timer(void)
{
	return readl(tick_count);
}

static struct delay_timer tangox_delay_timer = {
	.read_current_timer = tangox_read_delay_timer,
};
#endif

static int __init tangox_csrc_setup(struct device_node *node)
{
	void __iomem *base;
	struct clk *clk;
	const char *name;
	int rate;

	clk = of_clk_get(node, 0);
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	base = of_iomap(node, 0);
	if (!base)
		return -ENXIO;

	if (of_property_read_string(node, "label", &name))
		name = node->name;

	clk_prepare_enable(clk);
	rate = clk_get_rate(clk);

	clocksource_mmio_init(base, name, rate, 300, 32,
			      clocksource_mmio_readl_up);

	if (!tick_count) {
		tick_count = base;
		sched_clock_register(tangox_csrc_sched_read, 32, rate);
#ifdef CONFIG_ARM
		tangox_delay_timer.freq = rate;
		register_current_timer_delay(&tangox_delay_timer);
#endif
	}

	return 0;
}
CLOCKSOURCE_OF_DECLARE(tangox_csrc, "sigma,smp8640-csrc", tangox_csrc_setup);
