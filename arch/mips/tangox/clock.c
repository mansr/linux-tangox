#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <asm/time.h>
#include <asm/mipsregs.h>

#include "setup.h"

void __init plat_time_init(void)
{
	struct device_node *cpu;
	struct clk *clk;
	unsigned rate;
	int ccres;

	of_clk_init(NULL);
	clocksource_of_init();

	cpu = of_find_node_by_path("cpu0");
	if (!cpu)
		return;

	clk = of_clk_get(cpu, 0);
	if (IS_ERR(clk))
		return;

	rate = clk_get_rate(clk);

	__asm__ ("rdhwr %0, $3" : "=r" (ccres));
	mips_hpt_frequency = rate / ccres;

	pr_info("CPU clock %d Hz\n", rate);
}
