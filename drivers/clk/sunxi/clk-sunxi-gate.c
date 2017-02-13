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

struct sunxi_clk_priv {
	void   *base;
	size_t  size;
};

static int sunxi_gate_update(struct clk *clk, bool enable)
{
	struct sunxi_clk_priv *priv = dev_get_priv(clk->dev);
	unsigned long id = clk->id;
	uint32_t offset = id / 32;
	uint32_t bit = id % 32;

	debug("%s (%s): id %ld base %p offset %x bit %d enable %d\n",
	      clk->dev->name, __func__, id, priv->base, offset,
	      bit, enable);

	if (enable)
		setbits_le32(priv->base + offset, BIT(bit));
	else
		clrbits_le32(priv->base + offset, BIT(bit));

	return -EINVAL;
}

static int sunxi_gate_enable(struct clk *clk)
{
	return sunxi_gate_update(clk, true);
}

static int sunxi_gate_disable(struct clk *clk)
{
	return sunxi_gate_update(clk, false);
}

static struct clk_ops sunxi_clk_gate_ops = {
	.enable = sunxi_gate_enable,
	.disable = sunxi_gate_disable,
};

static int sunxi_clk_gate_probe(struct udevice *dev)
{
	struct sunxi_clk_priv *priv = dev_get_priv(dev);
	fdt_addr_t addr;
	fdt_size_t size;

	debug("%s: %s\n", dev->name, __func__);

	addr = fdtdec_get_addr_size_auto_noparent(gd->fdt_blob, dev->of_offset,
						  "reg", 0, &size, false);
	if (addr == FDT_ADDR_T_NONE) {
		debug("%s: could not get addr\n", dev->name);
		return -EINVAL;
	}

	priv->base = (void *)addr;
	priv->size = size;

	return 0;
}

static const struct udevice_id sunxi_clk_gate_ids[] = {
	{ .compatible = "allwinner,sunxi-multi-bus-gates-clk" },
	{}
};

U_BOOT_DRIVER(sunxi_clk_gate) = {
	.name		= "sunxi_clk_gate",
	.id		= UCLASS_CLK,
	.of_match	= sunxi_clk_gate_ids,
	.ops		= &sunxi_clk_gate_ops,
	.probe		= sunxi_clk_gate_probe,
	.priv_auto_alloc_size = sizeof(struct sunxi_clk_priv),
};


