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
#include <linux/spinlock.h>
#include <asm/io.h>

struct tangox_pll_clk {
	struct clk_hw	hw;
	void __iomem	*reg;
	int		n_mask;
	int		m_mask;
	int		k_shift;
	int		k_mask;
	int		step;
	spinlock_t	lock;
	struct clk	*clk[3];
	struct clk_onecell_data clk_data;
	struct clk_mux	mux;
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
	struct clk_mux *m;
	const char *names[3];
	const char **pnames = NULL;
	int noutputs;
	int nparents;
	int table_len;
	u32 n_bits;
	u32 m_bits;
	u32 k_bits[2];
	u32 mn_bias;
	int i;

	noutputs = of_property_read_string_array(node, "clock-output-names",
						 names, 3);

	if (noutputs != 1 && noutputs != 3)
		return;

	nparents = of_clk_get_parent_count(node);
	if (nparents <= 0)
		return;

	table_len = of_property_count_u32_elems(node, "sigma,mux-table");
	if (table_len > 0 && table_len != nparents)
		return;

	pc = kzalloc(sizeof(*pc), GFP_KERNEL);
	if (!pc)
		return;

	of_property_read_u32(node, "sigma,pll-n-bits", &n_bits);
	of_property_read_u32(node, "sigma,pll-m-bits", &m_bits);
	of_property_read_u32_array(node, "sigma,pll-k-bits", k_bits, 2);
	of_property_read_u32(node, "sigma,pll-mn-bias", &mn_bias);

	pc->reg = of_iomap(node, 0);
	pc->n_mask = (1 << n_bits) - 1;
	pc->m_mask = (1 << m_bits) - 1;
	pc->k_shift = k_bits[1];
	pc->k_mask = (1 << k_bits[0]) - 1;
	pc->step = mn_bias;
	spin_lock_init(&pc->lock);
	pc->clk_data.clk_num = noutputs;
	pc->clk_data.clks = pc->clk;

	m = &pc->mux;
	m->reg = pc->reg;
	m->mask = 7;
	m->shift = 24;
	m->lock = &pc->lock;

	pnames = kzalloc(nparents * sizeof(*pnames), GFP_KERNEL);
	if (!pnames)
		goto err;

	for (i = 0; i < nparents; i++)
		pnames[i] = of_clk_get_parent_name(node, i);

	if (table_len > 0) {
		m->table = kzalloc(table_len * sizeof(*m->table), GFP_KERNEL);
		if (!m->table)
			goto err;

		of_property_read_u32_array(node, "sigma,mux-table",
					   m->table, table_len);
	}

	pc->clk[0] = clk_register_composite(NULL, names[0], pnames, nparents,
					    &pc->mux.hw, &clk_mux_ops,
					    &pc->hw, &tangox_pll_ops,
					    NULL, NULL, 0);
	if (IS_ERR(pc->clk[0]))
		goto err;

	if (noutputs > 1) {
		pc->clk[1] = clk_register_divider(NULL, names[1], names[0], 0,
						  pc->reg + 4, 0, 4,
						  CLK_DIVIDER_ONE_BASED,
						  &pc->lock);

		pc->clk[2] = clk_register_divider(NULL, names[2], names[0], 0,
						  pc->reg + 4, 8, 4,
						  CLK_DIVIDER_ONE_BASED,
						  &pc->lock);
	}

	of_clk_add_provider(node, of_clk_src_onecell_get, &pc->clk_data);

	kfree(pnames);
	return;

err:
	kfree(pnames);
	kfree(pc->mux.table);
	kfree(pc);
}
CLK_OF_DECLARE(tangox_pll, "sigma,smp8640-pll-clk", tangox_clk_pll_setup);

struct tangox_premux_clk {
	struct clk *clks[2];
	struct clk_onecell_data clk_data;
	spinlock_t lock;
};

static void __init tangox_clk_premux_setup(struct device_node *node)
{
	struct tangox_premux_clk *mc;
	const char *names[2];
	const char **pnames = NULL;
	void __iomem *reg;
	u32 *sys_table = NULL;
	u32 *cd_table = NULL;
	int nparents;
	int nsys;
	int ncd;
	int i;

	nparents = of_clk_get_parent_count(node);
	if (nparents <= 0)
		return;

	nsys = of_property_count_u32_elems(node, "sigma,sys-table");
	ncd = of_property_count_u32_elems(node, "sigma,cd-table");

	if (nsys + ncd != nparents)
		return;

	mc = kzalloc(sizeof(*mc), GFP_KERNEL);
	if (!mc)
		return;

	pnames = kzalloc(nparents * sizeof(*pnames), GFP_KERNEL);
	if (!pnames)
		goto err;

	for (i = 0; i < nparents; i++)
		pnames[i] = of_clk_get_parent_name(node, i);

	if (of_property_read_string_array(node, "clock-output-names",
					  names, 2) < 2)
		goto err;

	sys_table = kzalloc(nsys * sizeof(*sys_table), GFP_KERNEL);
	if (!sys_table)
		goto err;

	of_property_read_u32_array(node, "sigma,sys-table", sys_table, nsys);

	cd_table = kzalloc(ncd * sizeof(*cd_table), GFP_KERNEL);
	if (!cd_table)
		goto err;

	of_property_read_u32_array(node, "sigma,cd-table", cd_table, ncd);

	mc->clk_data.clks = mc->clks;
	mc->clk_data.clk_num = 2;
	spin_lock_init(&mc->lock);

	reg = of_iomap(node, 0);
	if (!reg)
		goto err;

	mc->clks[0] = clk_register_mux_table(NULL, names[0], pnames, nsys,
					     0, reg, 0, 3, 0, sys_table,
					     &mc->lock);

	mc->clks[1] = clk_register_mux_table(NULL, names[1], pnames + nsys, ncd,
					     0, reg, 8, 7, 0, cd_table,
					     &mc->lock);

	of_clk_add_provider(node, of_clk_src_onecell_get, &mc->clk_data);

	kfree(pnames);
	return;

err:
	kfree(sys_table);
	kfree(cd_table);
	kfree(pnames);
	kfree(mc);
}
CLK_OF_DECLARE(tangox_premux, "sigma,smp8640-premux-clk",
	       tangox_clk_premux_setup);

static void __init tangox_clk_sysmux_setup(struct device_node *node)
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
CLK_OF_DECLARE(tangox_sysmux, "sigma,smp8640-sysmux-clk",
	       tangox_clk_sysmux_setup);

struct tangox_cleandiv_clk {
	struct clk_hw hw;
	void __iomem *reg;
	struct clk *clk[3];
	struct clk_onecell_data clk_data;
};

#define to_tangox_cleandiv_clk(_hw) \
	container_of(_hw, struct tangox_cleandiv_clk, hw)

static int tangox_clk_cleandiv_enable(struct clk_hw *hw)
{
	struct tangox_cleandiv_clk *cd = to_tangox_cleandiv_clk(hw);
	u32 ctrl;

	ctrl = readl(cd->reg + 4);
	writel(ctrl & ~1, cd->reg + 4);

	return 0;
}

static void tangox_clk_cleandiv_disable(struct clk_hw *hw)
{
	struct tangox_cleandiv_clk *cd = to_tangox_cleandiv_clk(hw);
	u32 ctrl;

	ctrl = readl(cd->reg + 4);
	writel(ctrl | 1, cd->reg + 4);
}

static int tangox_clk_cleandiv_is_enabled(struct clk_hw *hw)
{
	struct tangox_cleandiv_clk *cd = to_tangox_cleandiv_clk(hw);
	u32 ctrl;

	ctrl = readl(cd->reg + 4);

	return !(ctrl & 1);
}

static unsigned long tangox_clk_cleandiv_recalc(struct clk_hw *hw,
					  unsigned long prate)
{
	struct tangox_cleandiv_clk *cd = to_tangox_cleandiv_clk(hw);
	u64 rate;
	u32 div;

	div = readl(cd->reg);

	rate = (u64)prate << 27;
	do_div(rate, (2 << 27) + div);

	return rate;
}

static const struct clk_ops tangox_cleandiv_clk_ops = {
	.enable		= tangox_clk_cleandiv_enable,
	.disable	= tangox_clk_cleandiv_disable,
	.is_enabled	= tangox_clk_cleandiv_is_enabled,
	.recalc_rate	= tangox_clk_cleandiv_recalc,
};

static void __init tangox_clk_cleandiv_setup(struct device_node *node)
{
	struct tangox_cleandiv_clk *cd;
	struct clk_init_data id;
	const char *names[3];
	const char *pname;

	if (of_property_read_string_array(node, "clock-output-names",
					  names, 3) < 3)
		return;

	pname = of_clk_get_parent_name(node, 0);

	cd = kzalloc(sizeof(*cd), GFP_KERNEL);
	if (!cd)
		return;

	id.name = names[0];
	id.ops = &tangox_cleandiv_clk_ops;
	id.parent_names = &pname;
	id.num_parents = 1;
	id.flags = 0;

	cd->hw.init = &id;
	cd->reg = of_iomap(node, 0);
	cd->clk_data.clks = cd->clk;
	cd->clk_data.clk_num = 3;

	cd->clk[0] = clk_register(NULL, &cd->hw);
	cd->clk[1] = clk_register_fixed_factor(NULL, names[1], names[0],
					       0, 1, 2);
	cd->clk[2] = clk_register_fixed_factor(NULL, names[2], names[0],
					       0, 1, 4);

	of_clk_add_provider(node, of_clk_src_onecell_get, &cd->clk_data);
}
CLK_OF_DECLARE(tangox_cd, "sigma,smp8640-cleandiv-clk",
	       tangox_clk_cleandiv_setup);

struct tangox_mux_clk {
	struct clk_hw hw;
	void __iomem *reg;
	u32 shift;
	u32 mask;
	u32 *table;
};

#define to_tangox_mux_clk(_hw) container_of(_hw, struct tangox_mux_clk, hw)

static u8 tangox_clk_mux_get_parent(struct clk_hw *hw)
{
	struct tangox_mux_clk *m = to_tangox_mux_clk(hw);
	int nparents;
	u32 v;
	int i;

	v = readl(m->reg) >> m->shift & m->mask;

	if (m->table) {
		nparents = clk_hw_get_num_parents(hw);

		for (i = 0; i < nparents; i++)
			if (m->table[i] == v)
				return i;

		return -EINVAL;
	}

	return v - 1;
}

static int tangox_clk_mux_set_parent(struct clk_hw *hw, u8 index)
{
	struct tangox_mux_clk *m = to_tangox_mux_clk(hw);

	if (m->table)
		index = m->table[index];
	else
		index++;

	if (!index)
		return -EINVAL;

	writel(index << m->shift, m->reg);

	return 0;
}

static const struct clk_ops tangox_mux_clk_ops = {
	.get_parent	= tangox_clk_mux_get_parent,
	.set_parent	= tangox_clk_mux_set_parent,
};

static void __init tangox_clk_mux_setup(struct device_node *node)
{
	struct tangox_mux_clk *m;
	struct clk_init_data id;
	struct clk *clk;
	const char **pnames = NULL;
	int nparents;
	int table_len;
	u32 bits[2];
	int i;

	nparents = of_clk_get_parent_count(node);
	if (nparents <= 0)
		return;

	table_len = of_property_count_u32_elems(node, "sigma,mux-table");
	if (table_len > 0 && table_len != nparents)
		return;

	m = kzalloc(sizeof(*m), GFP_KERNEL);
	if (!m)
		return;

	pnames = kzalloc(nparents * sizeof(*pnames), GFP_KERNEL);
	if (!pnames)
		goto err;

	for (i = 0; i < nparents; i++)
		pnames[i] = of_clk_get_parent_name(node, i);

	if (table_len > 0) {
		m->table = kzalloc(table_len * sizeof(*m->table), GFP_KERNEL);
		if (!m->table)
			goto err;

		of_property_read_u32_array(node, "sigma,mux-table",
					   m->table, table_len);
	}

	if (of_property_read_u32_array(node, "sigma,bits", bits, 2))
		goto err;

	if (of_property_read_string(node, "clock-output-names", &id.name))
		id.name = node->name;
	id.ops = &tangox_mux_clk_ops;
	id.parent_names = pnames;
	id.num_parents = nparents;
	id.flags = 0;

	m->hw.init = &id;
	m->reg = of_iomap(node, 0);
	m->shift = bits[0];
	m->mask = (1 << bits[1]) - 1;

	clk = clk_register(NULL, &m->hw);
	if (IS_ERR(clk))
		goto err;

	of_clk_add_provider(node, of_clk_src_simple_get, clk);

	kfree(pnames);
	return;

err:
	kfree(pnames);
	kfree(m->table);
	kfree(m);
}
CLK_OF_DECLARE(tangox_mux, "sigma,smp8640-mux-clk", tangox_clk_mux_setup);

struct tangox_div_clk {
	struct clk_hw hw;
	void __iomem *reg;
};

#define to_tangox_div_clk(_hw) container_of(_hw, struct tangox_div_clk, hw)

static u8 tangox_clk_div_get_parent(struct clk_hw *hw)
{
	struct tangox_div_clk *dc = to_tangox_div_clk(hw);
	u32 ctrl;

	ctrl = readl(dc->reg);

	return ctrl >> 23 & 1;
}

static int tangox_clk_div_set_parent(struct clk_hw *hw, u8 parent)
{
	struct tangox_div_clk *dc = to_tangox_div_clk(hw);
	u32 ctrl;

	ctrl = readl(dc->reg);
	ctrl &= ~(1 << 23);
	ctrl |= parent << 23;
	writel(ctrl, dc->reg);

	return 0;
}

static unsigned long tangox_clk_div_recalc(struct clk_hw *hw,
					   unsigned long prate)
{
	struct tangox_div_clk *dc = to_tangox_div_clk(hw);
	u32 div, scale;
	u32 i, f;
	u32 ctrl;
	u64 rate;

	ctrl = readl(dc->reg);

	if (ctrl & (1 << 23))
		return prate;

	f = ctrl & 0xf;
	i = ctrl >> 8 & 0xff;

	if (i == 0)
		return 0;

	if (i == 1)
		scale = 32 - f;
	else
		scale = 16;

	div = i * scale + f;
	rate = (u64)prate * scale;
	do_div(rate, div);

	return rate;
}

static const struct clk_ops tangox_div_clk_ops = {
	.get_parent	= tangox_clk_div_get_parent,
	.set_parent	= tangox_clk_div_set_parent,
	.recalc_rate	= tangox_clk_div_recalc,
};

static void __init tangox_clk_div_setup(struct device_node *node)
{
	struct tangox_div_clk *dc;
	struct clk_init_data id;
	struct clk *clk;
	const char *pnames[2];
	int i;

	if (of_clk_get_parent_count(node) != 2)
		return;

	dc = kzalloc(sizeof(*dc), GFP_KERNEL);
	if (!dc)
		return;

	for (i = 0; i < 2; i++)
		pnames[i] = of_clk_get_parent_name(node, i);

	if (of_property_read_string(node, "clock-output-names", &id.name))
		id.name = node->name;
	id.ops = &tangox_div_clk_ops;
	id.parent_names = pnames;
	id.num_parents = 2;
	id.flags = 0;

	dc->hw.init = &id;
	dc->reg = of_iomap(node, 0);

	clk = clk_register(NULL, &dc->hw);
	if (IS_ERR(clk))
		goto err;

	of_clk_add_provider(node, of_clk_src_simple_get, clk);
	return;

err:
	kfree(dc);
}
CLK_OF_DECLARE(tangox_div, "sigma,smp8750-div-clk", tangox_clk_div_setup);
