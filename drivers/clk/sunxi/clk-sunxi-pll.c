/*
 * (C) 2017 Theobroma Systems Design und Consulting GmbH
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

struct sunxi_clk_pll_data {
	u8 nshift;
	u8 nwidth;
	u8 kshift;
	u8 kwidth;
	u8 mshift;
	u8 mwidth;
	u8 pshift;
	u8 pwidth;
	u8 n_start;
};

struct sunxi_clk_priv {
	void *reg;
};

static inline uint32_t bitmask(uint8_t width, uint8_t shift)
{
	return ((1U << width) - 1) << shift;
}

static inline uint32_t factor_extract(uint32_t regval,
				      uint8_t width,
				      uint8_t shift)
{
	return ((regval & bitmask(width, shift)) >> shift) + 1;
}

static ulong sunxi_pll_get_rate(struct clk *clk)
{
	struct sunxi_clk_priv *priv = dev_get_priv(clk->dev);
	struct sunxi_clk_pll_data *data =
		(struct sunxi_clk_pll_data *)dev_get_driver_data(clk->dev);
	uint32_t regval = readl(priv->reg);
	int n = factor_extract(regval, data->nwidth, data->nshift);
	int k = factor_extract(regval, data->kwidth, data->kshift);
	int m = factor_extract(regval, data->mwidth, data->mshift);
	ulong rate = (24000000 * n * k) / m;

	debug("%s (%s): id %ld base %p\n",
	      clk->dev->name, __func__, clk->id, priv->reg);

	/* Check if the PLL is enabled... */
	if (!(regval & BIT(31)))
		return 0;

	debug("%s: n %d k %d m %d\n", clk->dev->name, n, k, m);
	debug("%s: rate %ld\n", clk->dev->name, rate);

	if (clk->id == 1)
		return 2 * rate;

	return rate;
}

static struct clk_ops sunxi_clk_pll_ops = {
	/* For now, we'll let the arch/board-specific code setup the
	   PLLs through the legacy implementation (some of this will
	   happen in SPL, which may not have device model capability)
	   and we only read the PLL rates. */
	.get_rate = sunxi_pll_get_rate,
};

static int sunxi_clk_pll_probe(struct udevice *dev)
{
	struct sunxi_clk_priv *priv = dev_get_priv(dev);
	fdt_addr_t addr;
	fdt_size_t size;

	addr = fdtdec_get_addr_size_auto_noparent(gd->fdt_blob, dev->of_offset,
						  "reg", 0, &size, false);
	if (addr == FDT_ADDR_T_NONE)
		return -EINVAL;

	priv->reg = (void *)addr;
	if (!priv->reg)
		return -EINVAL;

	debug("%s: reg %p\n", dev->name, priv->reg);

	return 0;
}

static struct sunxi_clk_pll_data pll6_data = {
	.nwidth = 5,
	.nshift = 8,
	.kwidth = 2,
	.kshift = 4,
	.mwidth = 2,
	.mshift = 0,
};

static const struct udevice_id sunxi_clk_pll_ids[] = {
	{ .compatible = "allwinner,sun6i-a31-pll6-clk",
	  .data = (uintptr_t)&pll6_data },
	{}
};

U_BOOT_DRIVER(sunxi_clk_pll) = {
	.name		= "sunxi_clk_pll",
	.id		= UCLASS_CLK,
	.of_match	= sunxi_clk_pll_ids,
	.ops		= &sunxi_clk_pll_ops,
	.probe		= sunxi_clk_pll_probe,
	.priv_auto_alloc_size = sizeof(struct sunxi_clk_priv),
};
