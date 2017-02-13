/*
 * (C) 2017 Theobroma Systems Design und Consulting GmbH
 *
 * With sun4i_a10_get_mod0_factors(...) adapted from
 *    linux/drivers/clk/sunxi/clk-mod0.c
 * which is
 *    Copyright 2013 Emilio López
 *
 * SPDX-License-Identifier:	GPL-2.0+
 *
 */

#include <common.h>
#include <clk-uclass.h>
#include <dm.h>
#include <div64.h>
#include <wait_bit.h>
#include <dm/lists.h>
#include <asm/io.h>

DECLARE_GLOBAL_DATA_PTR;

struct sunxi_clk_priv {
	void         *reg;
	int           num_parents;
	struct clk    parent[4];
};

#define SRCSHIFT    (24)
#define SRCMASK     (0x3 << SRCSHIFT)
#define SRC(n)      (n << SRCSHIFT)
#define PREDIVMASK  (0x3 << 16)
#define PREDIV(n)   (n << 16)
#define DIVMASK     (0xf << 0)
#define DIV(n)      (n)

static ulong sunxi_mod_get_rate(struct clk *clk)
{
	struct sunxi_clk_priv *priv = dev_get_priv(clk->dev);
	u32 active_parent;
	ulong rate = -EINVAL;
	u32 regval = readl(priv->reg);

	/* if not enabled, return 0 */
	if (regval & BIT(31))
		return 0;

	active_parent = (readl(priv->reg) >> 24) & 0x3;
	if (active_parent < priv->num_parents)
		rate = clk_get_rate(&priv->parent[active_parent]);

	return rate;
}

/**
 * sun4i_a10_get_mod0_factors()
 *  - calculates m, n factors for MOD0-style clocks
 *
 * MOD0 rate is calculated as follows:
 *    rate = (parent_rate >> p) / (m + 1);
 */

struct factors_request {
	unsigned long rate;
	unsigned long parent_rate;
	u8 parent_index;
	u8 n;
	u8 k;
	u8 m;
	u8 p;
};

static void sun4i_a10_get_mod0_factors(struct factors_request *req)
{
	u8 div, calcm, calcp;

	/* These clocks can only divide, so we will never be able to
	 * achieve frequencies higher than the parent frequency */
	if (req->rate >= req->parent_rate) {
		req->rate = req->parent_rate;
		req->m = 0;
		req->p = 0;
	}

	div = DIV_ROUND_UP(req->parent_rate, req->rate);

	if (div < 16)
		calcp = 0;
	else if (div / 2 < 16)
		calcp = 1;
	else if (div / 4 < 16)
		calcp = 2;
	else
		calcp = 3;

	calcm = DIV_ROUND_UP(div, 1 << calcp);
	/* clamp calcm to 16, as that is the largest possible divider */
	if (calcm > 16)
		calcm = 16;

	req->rate = (req->parent_rate >> calcp) / calcm;
	req->m = calcm - 1;
	req->p = calcp;
}

static ulong sunxi_mod_set_rate(struct clk *clk, ulong rate)
{
	struct sunxi_clk_priv *priv = dev_get_priv(clk->dev);
	ulong best_rate = 0;
	int i;

	debug("%s (%s): id %ld rate %ld base %p\n",
	      clk->dev->name, __func__, clk->id, rate, priv->reg);

	/* check if the current rate is already the target rate */
	if (sunxi_mod_get_rate(clk) == rate)
		return rate;

	/* find the parent (iterate through) which allows us to have:
	 *     fastest rate <= rate
	 */
	for (i = 0; i < priv->num_parents; ++i) {
		ulong parent_rate = clk_get_rate(&priv->parent[i]);
		struct factors_request  req = {
			.rate = rate,
			.parent_rate = parent_rate,
			.parent_index = i
		};

		debug("%s (%s): parent %d rate %ld\n",
		      clk->dev->name, __func__, i, parent_rate);

		if (parent_rate == -ENOSYS) {
			debug("%s: parent %d does not support get_rate\n",
			      clk->dev->name, i);
			continue;
		}

		if (parent_rate == 0) {
			debug("%s: parent %d seems disabled (rate == 0)\n",
			      clk->dev->name, i);
			continue;
		}

		/* We recalculate the dividers, even if the parent's
		 * rate is less than the requested rate
		 */
		sun4i_a10_get_mod0_factors(&req);

		if (req.rate > rate) {
			debug("%s: rate %ld for parent %i exceeds rate\n",
			      clk->dev->name, req.rate, i);
			continue;
		}

		if (req.rate > best_rate) {
			debug("%s: new best => parent %d P %d M %d rate %ld\n",
			      clk->dev->name, i, req.p, req.m, req.rate);

			clrsetbits_le32(priv->reg,
					SRCMASK | PREDIVMASK | DIVMASK,
					SRC(i) | PREDIV(req.p) | DIV(req.m));
			best_rate = req.rate;

			/* don't continue, if this is the requested rate */
			if (best_rate == rate)
				break;
		}
	}

	return best_rate;
}

static int sunxi_mod_enable(struct clk *clk)
{
	struct sunxi_clk_priv *priv = dev_get_priv(clk->dev);

	setbits_le32(priv->reg, BIT(31));
	return 0;
}

static int sunxi_mod_disable(struct clk *clk)
{
	struct sunxi_clk_priv *priv = dev_get_priv(clk->dev);

	clrbits_le32(priv->reg, BIT(31));
	return 0;
}

static struct clk_ops sunxi_clk_mod_ops = {
	.set_rate = sunxi_mod_set_rate,
	.get_rate = sunxi_mod_get_rate,
	.enable = sunxi_mod_enable,
	.disable = sunxi_mod_disable,
};

static int sunxi_clk_mod_probe(struct udevice *dev)
{
	struct sunxi_clk_priv *priv = dev_get_priv(dev);
	fdt_addr_t addr;
	fdt_size_t size;
	int i;

	debug("%s: %s\n", dev->name, __func__);

	addr = fdtdec_get_addr_size_auto_noparent(gd->fdt_blob, dev->of_offset,
						  "reg", 0, &size, false);
	if (addr == FDT_ADDR_T_NONE) {
		debug("%s: could not get addr\n", dev->name);
		return -EINVAL;
	}

	priv->reg = (void *)addr;

	for (i = 0; i < 4; ++i) {
		int ret = clk_get_by_index(dev, i, &priv->parent[i]);
		if (ret != 0)
			break;
	};
	priv->num_parents = i;

	debug("%s: reg %p num-parents %d\n",
	      dev->name, priv->reg, priv->num_parents);
	return 0;
}

static const struct udevice_id sunxi_clk_mod_ids[] = {
	{ .compatible = "allwinner,sun4i-a10-mod0-clk" },
	{}
};

U_BOOT_DRIVER(sunxi_clk_mod) = {
	.name		= "sunxi_clk_mod",
	.id		= UCLASS_CLK,
	.of_match	= sunxi_clk_mod_ids,
	.ops		= &sunxi_clk_mod_ops,
	.probe		= sunxi_clk_mod_probe,
	.priv_auto_alloc_size = sizeof(struct sunxi_clk_priv),
};


