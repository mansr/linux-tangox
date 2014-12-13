/*
 * Copyright (C) 2014 Mans Rullgard <mans@mansr.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/of_address.h>
#include <asm/io.h>

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

static void __init tangox_clk_pll_setup(struct device_node *node)
{
	struct tangox_pll_clk *pc;
	struct clk_init_data id;
	const char *pname;
	struct clk *c;
	u32 n_bits;
	u32 m_bits;
	u32 k_bits[2];
	u32 mn_bias;

	pc = kzalloc(sizeof(*pc), GFP_KERNEL);
	if (!pc)
		return;

	pname = of_clk_get_parent_name(node, 0);

	if (of_property_read_string(node, "clock-output-names", &id.name))
		id.name = node->name;
	id.ops = &tangox_pll_ops;
	id.parent_names = &pname;
	id.num_parents = 1;
	id.flags = 0;

	pc->reg = of_iomap(node, 0);
	pc->hw.init = &id;

	of_property_read_u32(node, "sigma,pll-n-bits", &n_bits);
	of_property_read_u32(node, "sigma,pll-m-bits", &m_bits);
	of_property_read_u32_array(node, "sigma,pll-k-bits", k_bits, 2);
	of_property_read_u32(node, "sigma,pll-mn-bias", &mn_bias);

	pc->n_mask = (1 << n_bits) - 1;
	pc->m_mask = (1 << m_bits) - 1;
	pc->k_shift = k_bits[1];
	pc->k_mask = (1 << k_bits[0]) - 1;
	pc->step = mn_bias;

	c = clk_register(NULL, &pc->hw);

	if (!IS_ERR(c))
		of_clk_add_provider(node, of_clk_src_simple_get, c);
	else
		kfree(pc);
}
CLK_OF_DECLARE(tangox_pll, "sigma,smp8640-pll-clk", tangox_clk_pll_setup);

static void __init tangox_clk_mux_setup(struct device_node *node)
{
	const char *name = node->name;
	int nparents;
	const char **pnames;
	void __iomem *reg;
	struct clk *c;
	int i;

	of_property_read_string(node, "clock-output-names", &name);

	nparents = of_clk_get_parent_count(node);
	if (nparents <= 0)
		return;

	pnames = kzalloc(nparents * sizeof(*pnames), GFP_KERNEL);
	if (!pnames)
		return;

	for (i = 0; i < nparents; i++)
		pnames[i] = of_clk_get_parent_name(node, i);

	reg = of_iomap(node, 0);

	c = clk_register_mux(NULL, name, pnames, nparents,
			     0, reg, 0, 2, 0, NULL);
	if (!IS_ERR(c))
		of_clk_add_provider(node, of_clk_src_simple_get, c);

	kfree(pnames);
}
CLK_OF_DECLARE(tangox_mux, "sigma,smp8640-mux-clk", tangox_clk_mux_setup);

static void __init tangox_clk_div_setup(struct device_node *node)
{
	struct clk_onecell_data *clk_data;
	struct clk_divider *div = NULL;
	struct clk_mux *mux = NULL;
	struct clk_div_table *divtab;
	const char *parents[2];
	const char *name;
	void __iomem *reg;
	int nclks;
	int ndivs;
	struct clk *c;
	int i, j;

	reg = of_iomap(node, 0);

	nclks = of_property_count_strings(node, "clock-output-names");
	ndivs = of_property_count_u32_elems(node, "sigma,divisors") / nclks;

	parents[0] = of_clk_get_parent_name(node, 0);
	parents[1] = of_clk_get_parent_name(node, 1);

	clk_data = kzalloc(sizeof(*clk_data) + nclks * sizeof(*clk_data->clks),
			   GFP_KERNEL);
	if (!clk_data)
		return;

	clk_data->clks = (struct clk **)(clk_data + 1);
	clk_data->clk_num = nclks;

	for (i = 0; i < nclks; i++) {
		divtab = kzalloc((ndivs + 1) * sizeof(*divtab), GFP_KERNEL);
		if (!divtab)
			return;

		for (j = 0; j < ndivs; j++) {
			divtab[j].val = j;
			of_property_read_u32_index(node, "sigma,divisors",
						   i * ndivs + j,
						   &divtab[j].div);
		}

		mux = kzalloc(sizeof(*mux), GFP_KERNEL);
		div = kzalloc(sizeof(*div), GFP_KERNEL);

		if (!mux || !div)
			return;

		mux->reg = reg;
		mux->mask = 1;
		mux->shift = 0;

		div->reg = reg;
		div->shift = 8;
		div->width = 4;
		div->flags = CLK_DIVIDER_READ_ONLY;
		div->table = divtab;

		of_property_read_string_index(node, "clock-output-names", i,
					      &name);

		c = clk_register_composite(NULL, name, parents, 2,
					   &mux->hw, &clk_mux_ro_ops,
					   &div->hw, &clk_divider_ops,
					   NULL, NULL, 0);

		if (IS_ERR(c)) {
			pr_err("%s: error creating clock %s\n",
			       node->name, name);
			return;
		}

		clk_register_clkdev(c, name, NULL);
		clk_data->clks[i] = c;
	}

	of_clk_add_provider(node, of_clk_src_onecell_get, clk_data);
}
CLK_OF_DECLARE(tangox_div, "sigma,smp8640-div-clk", tangox_clk_div_setup);
