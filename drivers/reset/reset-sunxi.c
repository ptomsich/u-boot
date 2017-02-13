/*
 * Copyright (C) 2017 Theobroma Systems Design und Consulting GmbH
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <reset-uclass.h>
#include <dm/device.h>
#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/sizes.h>

DECLARE_GLOBAL_DATA_PTR;

struct sunxi_reset_priv {
	void __iomem *base;
	size_t  size;
};

static int sunxi_reset_request(struct reset_ctl *reset_ctl)
{
	debug("%s (%s): id %ld\n",
	      reset_ctl->dev->name, __func__, reset_ctl->id);
	return 0;
}

static int sunxi_reset_free(struct reset_ctl *reset_ctl)
{
	debug("%s (%s): id %ld\n",
	      reset_ctl->dev->name, __func__, reset_ctl->id);
	return 0;
}

static int sunxi_reset_update(struct reset_ctl *reset_ctl, bool assert)
{
	struct sunxi_reset_priv *priv = dev_get_priv(reset_ctl->dev);
	unsigned long id = reset_ctl->id;
	unsigned long offset = id / 32; /* TODO: symbolic name */
	unsigned int bit = id % 32;

	debug("%s (%s): id %ld base %p offset %lx bit %d assert %d size %ld\n",
	      reset_ctl->dev->name, __func__, id, priv->base, offset,
	      bit, assert, priv->size);

	if (offset >= priv->size)
		return -EINVAL;

	if (assert)
		clrbits_le32(priv->base + offset, BIT(bit));
	else
		setbits_le32(priv->base + offset, BIT(bit));

	return 0;
}

static int sunxi_reset_assert(struct reset_ctl *reset_ctl)
{
	return sunxi_reset_update(reset_ctl, true);
}

static int sunxi_reset_deassert(struct reset_ctl *reset_ctl)
{
	return sunxi_reset_update(reset_ctl, false);
}

static const struct reset_ops sunxi_reset_ops = {
	.request = sunxi_reset_request,
	.free = sunxi_reset_free,
	.rst_assert = sunxi_reset_assert,
	.rst_deassert = sunxi_reset_deassert,
};

static int sunxi_reset_probe(struct udevice *dev)
{
	struct sunxi_reset_priv *priv = dev_get_priv(dev);
	fdt_addr_t addr;
	fdt_size_t size;

	addr = fdtdec_get_addr_size_auto_noparent(gd->fdt_blob, dev->of_offset,
						  "reg", 0, &size, false);
	if (addr == FDT_ADDR_T_NONE) {
		debug("%s: failed to find base address ('reg')\n", dev->name);
		return -ENODEV;
	}
	priv->base = (void *)addr;
	priv->size = size;

	if (!priv->base)
		return -ENOMEM;

	return 0;
}

static const struct udevice_id sunxi_reset_match[] = {
	{ .compatible = "allwinner,sun6i-a31-clock-reset" },
	{ }
};

U_BOOT_DRIVER(sunxi_reset) = {
	.name = "sunxi-reset",
	.id = UCLASS_RESET,
	.of_match = sunxi_reset_match,
	.ops = &sunxi_reset_ops,
	.priv_auto_alloc_size = sizeof(struct sunxi_reset_priv),
	.probe = sunxi_reset_probe,
};
