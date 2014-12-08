#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <asm/io.h>

#include "setup.h"

#define CLK_BASE	0x10000
#define CLK_PLL		0
#define CLK_SYS_ISEL	0x34
#define CLK_SYS_DIV	0x3c

#define ROOT_CLK "xtal_in_clk"

struct tangox_pll_clk {
	struct clk_hw	hw;
	void __iomem	*reg;
	int		n_mask;
	int		m_mask;
	int		k_shift;
	int		k_mask;
	int		step;
};

#define to_pll_clk(_hw) container_of(_hw, struct tangox_pll_clk, hw)

static unsigned long tangox_pll_recalc(struct clk_hw *hw, unsigned long prate)
{
	struct tangox_pll_clk *pc = to_pll_clk(hw);
	unsigned int pll;
	int m, n, k;

	pll = readl(pc->reg);

	n = pll & pc->n_mask;
	m = (pll >> 16) & pc->m_mask;
	k = (pll >> pc->k_shift) & pc->k_mask;

	n += pc->step;
	m += pc->step;

	return prate / m * n / (1 << k);
}

static struct clk_ops tangox_pll_ops = {
	.recalc_rate	= tangox_pll_recalc,
};

static const char *pll_names[] __initconst = {
	"pll0_clk",
	"pll1_clk",
	"pll2_clk",
	"pll3_clk",
};

static int __init tangox_clk_pll_setup(int n)
{
	struct tangox_pll_clk *pc;
	struct clk_init_data id;
	const char *pname = ROOT_CLK;
	struct clk *c;

	pc = kzalloc(sizeof(*pc), GFP_KERNEL);
	if (!pc)
		return -ENOMEM;

	id.name = pll_names[n];
	id.ops = &tangox_pll_ops;
	id.parent_names = &pname;
	id.num_parents = 1;
	id.flags = 0;

	pc->reg = ioremap(CLK_BASE + CLK_PLL + n * 8, 8);
	pc->hw.init = &id;

	if (!n) {
		pc->n_mask = 0x3ff;
		pc->m_mask = 0x1f;
		pc->k_shift = 14;
		pc->k_mask = 3;
		pc->step = 2;
	} else {
		pc->n_mask = 0x7f;
		pc->m_mask = 1;
		pc->k_shift = 13;
		pc->k_mask = 7;
		pc->step = 1;
	}

	c = clk_register(NULL, &pc->hw);

	if (IS_ERR(c))
		kfree(pc);

	return PTR_ERR_OR_ZERO(c);
}

static int __init tangox_clk_mux_setup(const char *name,
						unsigned long isel)
{
	void __iomem *reg = ioremap(CLK_BASE + isel, 4);
	struct clk *c;

	c = clk_register_mux(NULL, name, pll_names, ARRAY_SIZE(pll_names),
			     0, reg, 0, 2, 0, NULL);

	return PTR_ERR_OR_ZERO(c);
}

static const struct clk_div_table tangox_clk_div_tab[3][13] = {
	{
		{ 0, 2 }, { 1, 4 }, {  2, 3 }, {  3, 3 },
		{ 4, 3 }, { 5, 3 }, {  6, 3 }, {  7, 3 },
		{ 8, 4 }, { 9, 4 }, { 10, 4 }, { 11, 4 },
		{ 0, 0 }
	}, {
		{ 0, 2 }, { 1, 2 }, {  2, 2 }, {  3, 3 },
		{ 4, 3 }, { 5, 2 }, {  6, 3 }, {  7, 2 },
		{ 8, 4 }, { 9, 2 }, { 10, 4 }, { 11, 2 },
		{ 0, 0 }
	}, {
		{ 0, 2 }, { 1, 4 }, {  2, 3 }, {  3, 3 },
		{ 4, 3 }, { 5, 3 }, {  6, 3 }, {  7, 3 },
		{ 8, 4 }, { 9, 4 }, { 10, 4 }, { 11, 4 },
		{ 0, 0 }
	},
};

static struct clk * __init tangox_clk_div_setup(const char *name, int reg_off,
						int divtab, const char *pname)
{
	struct clk_divider *div = NULL;
	struct clk_mux *mux = NULL;
	const char *parents[2];
	void __iomem *reg;
	struct clk *c;
	int err;

	parents[0] = ROOT_CLK;
	parents[1] = pname;

	reg = ioremap(CLK_BASE + reg_off, 4);

	mux = kzalloc(sizeof(*mux), GFP_KERNEL);
	div = kzalloc(sizeof(*div), GFP_KERNEL);

	if (!mux || !div) {
		err = -ENOMEM;
		goto end;
	}

	mux->reg = reg;
	mux->mask = 1;
	mux->shift = 0;

	div->reg = reg;
	div->shift = 8;
	div->width = 4;
	div->flags = CLK_DIVIDER_READ_ONLY;
	div->table = tangox_clk_div_tab[divtab];

	c = clk_register_composite(NULL, name, parents, 2,
				   &mux->hw, &clk_mux_ro_ops,
				   &div->hw, &clk_divider_ops,
				   NULL, NULL, 0);

	err = clk_register_clkdev(c, name, NULL);

end:
	if (err) {
		kfree(mux);
		kfree(div);
		c = ERR_PTR(err);
	}

	return c;
}

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

int __init tangox_clk_init(void)
{
	struct clk *c;
	int err;
	int i;

	c = clk_register_fixed_rate(NULL, ROOT_CLK, NULL, CLK_IS_ROOT,
				    CONFIG_TANGOX_EXT_CLOCK);
	err = clk_register_clkdev(c, ROOT_CLK, NULL);
	if (err)
		return err;

	for (i = 0; i < ARRAY_SIZE(pll_names); i++) {
		err = tangox_clk_pll_setup(i);
		if (err)
			return err;
	}

	err = tangox_clk_mux_setup("sysroot_clk", CLK_SYS_ISEL);
	if (err)
		return err;

	for (i = 0; i < ARRAY_SIZE(sys_names); i++) {
		c = tangox_clk_div_setup(sys_names[i], CLK_SYS_DIV, i,
					 "sysroot_clk");
		if (IS_ERR(c))
			return PTR_ERR(c);

		tangox_sysclk[i] = c;
	}

	pr_info("CPU/System/DSP clocks:");
	pr_freq(' ', tangox_get_cpuclock());
	pr_freq('/', tangox_get_sysclock());
	pr_freq('/', tangox_get_dspclock());
	pr_cont(" MHz\n");

	return 0;
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
