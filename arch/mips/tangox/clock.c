#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <asm/time.h>
#include <asm/mipsregs.h>

#include "setup.h"

static const char *sys_names[] __initconst = {
	"sys_clk",
	"cpu_clk",
	"dsp_clk",
};

static struct clk *tangox_sysclk[3];

static void __init pr_freq(char sep, unsigned int f)
{
	pr_cont("%c%d.%02d", sep, f / 1000000, f / 10000 % 100);
}

void __init plat_time_init(void)
{
	int ccres;
	int i;

	of_clk_init(NULL);
	clocksource_of_init();

	for (i = 0; i < ARRAY_SIZE(sys_names); i++)
		tangox_sysclk[i] = clk_get(NULL, sys_names[i]);

	__asm__ ("rdhwr %0, $3" : "=r" (ccres));
	mips_hpt_frequency = tangox_get_cpuclock() / ccres;

	pr_info("CPU/System/DSP clocks:");
	pr_freq(' ', tangox_get_cpuclock());
	pr_freq('/', tangox_get_sysclock());
	pr_freq('/', tangox_get_dspclock());
	pr_cont(" MHz\n");
}

unsigned long tangox_get_sysclock(void)
{
	return clk_get_rate(tangox_sysclk[0]);
}

unsigned long tangox_get_cpuclock(void)
{
	return clk_get_rate(tangox_sysclk[1]);
}

unsigned long tangox_get_dspclock(void)
{
	return clk_get_rate(tangox_sysclk[2]);
}
