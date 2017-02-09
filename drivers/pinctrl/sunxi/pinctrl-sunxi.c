/*
 * (C) Copyright 2017 Theobroma Systems Design und Consulting GmbH
 *
 * In parts based on linux/drivers/pinctrl/pinctrl-sunxi.c, which is
 *   Copyright (C) 2012 Maxime Ripard
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <dm.h>
#include <errno.h>
#include <syscon.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/gpio.h>
#include <dm/lists.h>
#include <dm/pinctrl.h>
#include <dt-bindings/pinctrl/sun4i-a10.h>
#include <dt-bindings/gpio/gpio.h>
#include "pinctrl-sunxi.h"

DECLARE_GLOBAL_DATA_PTR;

struct sunxi_pinctrl_priv {
	void*  base;
};

static int sunxi_pinctrl_parse_drive_prop(const void *blob, int node)
{
       int val;

	/* Try the new style binding */
	val = fdtdec_get_int(blob, node, "drive-strength", -EINVAL);
	if (val < 0) {
		/* We can't go below 10mA ... */
		if (val < 10)
			return -EINVAL;

		/* ... and only up to 40 mA ... */
		if (val > 40)
			val = 40;

		/* by steps of 10 mA */
		return rounddown(val, 10);
	}

	/* And then fall back to the old binding */
	val = fdtdec_get_int(blob, node, "allwinner,drive", -EINVAL);
	if (val < 0)
		return -EINVAL;

	return (val + 1) * 10;
}

static int sunxi_pinctrl_parse_bias_prop(const void *blob, int node)
{
        /* Try the new style binding */
        if (fdtdec_get_bool(blob, node, "bias-pull-up"))
                return SUN4I_PINCTRL_PULL_UP;

        if (fdtdec_get_bool(blob, node, "bias-pull-down"))
                return SUN4I_PINCTRL_PULL_DOWN;

        if (fdtdec_get_bool(blob, node, "bias-disable"))
                return SUN4I_PINCTRL_NO_PULL;

        /* And fall back to the old binding */
        return fdtdec_get_int(blob, node, "allwinner,pull", -EINVAL);
}

static const struct sunxi_desc_pin* sunxi_pinctrl_pin_by_name(struct udevice *dev, const char* name)
{
	const struct sunxi_pinctrl_desc* data = (struct sunxi_pinctrl_desc*)dev_get_driver_data(dev);
	int i;

	for (i = 0; i < data->npins; ++i)
		if (!strcmp(data->pins[i].pin.name, name))
			return &data->pins[i];

	return NULL;
}

static int sunxi_pinctrl_muxval_by_name(const struct sunxi_desc_pin* pin, const char* name)
{
	const struct sunxi_desc_function *func;

	if (!pin)
		return -EINVAL;

	for (func = pin->functions; func->name; func++)
		if (!strcmp(func->name, name))
			return func->muxval;

	return -ENOENT;
}

static void sunxi_pinctrl_set_function(struct udevice *dev,
				       unsigned pin,
				       u8 function)
{
	struct sunxi_pinctrl_priv *priv = dev_get_priv(dev);
	const struct sunxi_pinctrl_desc* data =
		(struct sunxi_pinctrl_desc*)dev_get_driver_data(dev);
        u32 val, mask;

        pin -= data->pin_base;
        mask = MUX_PINS_MASK << sunxi_mux_offset(pin);
	val = function << sunxi_mux_offset(pin);
	clrsetbits_le32(priv->base + sunxi_mux_reg(pin), mask, val);
}

static void sunxi_pinctrl_set_dlevel(struct udevice *dev,
				     unsigned pin,
				     u8 dlevel)
{
	struct sunxi_pinctrl_priv *priv = dev_get_priv(dev);
	const struct sunxi_pinctrl_desc* data =
		(struct sunxi_pinctrl_desc*)dev_get_driver_data(dev);
        u32 val, mask;

        pin -= data->pin_base;
        mask = DLEVEL_PINS_MASK << sunxi_dlevel_offset(pin);
	val = dlevel << sunxi_dlevel_offset(pin);
	clrsetbits_le32(priv->base + sunxi_dlevel_reg(pin), mask, val);
}

static void sunxi_pinctrl_set_bias(struct udevice *dev,
				   unsigned pin,
				   u8 bias)
{
	struct sunxi_pinctrl_priv *priv = dev_get_priv(dev);
	const struct sunxi_pinctrl_desc* data =
		(struct sunxi_pinctrl_desc*)dev_get_driver_data(dev);
        u32 val, mask;

        pin -= data->pin_base;
        mask = PULL_PINS_MASK << sunxi_pull_offset(pin);
	val = bias << sunxi_pull_offset(pin);
	clrsetbits_le32(priv->base + sunxi_pull_reg(pin), mask, val);
}

static int sunxi_pinctrl_set_state(struct udevice *dev, struct udevice *config)
{
	const char *pins;
	const char *function;
	int flen;
	int len, curr_len;
	int drive, bias;
	int muxval;

	debug("%s: %s %s\n", __func__, dev->name, config->name);

	pins = fdt_getprop(gd->fdt_blob, config->of_offset, "allwinner,pins", &len);
	if (!pins) {
		debug("%s: missing allwinner,pins property in node %s\n",
		      dev->name, config->name);
		return -EINVAL;
	}

	function = fdt_getprop(gd->fdt_blob, config->of_offset, "allwinner,function", &flen);
	if (!function) {
		debug("%s: missing allwinner,function property in node %s\n",
		      dev->name, config->name);
		return -EINVAL;
	}

	drive = sunxi_pinctrl_parse_drive_prop(gd->fdt_blob, config->of_offset);
	bias = sunxi_pinctrl_parse_bias_prop(gd->fdt_blob, config->of_offset);

	/* Iterate through the pins and configure each */
	while (len && (curr_len = strnlen(pins, len))) {
		const struct sunxi_desc_pin* pin;

		if (curr_len == len) {
			error("%s: unterminated string?", __func__);
			break;
		}
		pins += (curr_len + 1);
		len -= (curr_len + 1);

		pin = sunxi_pinctrl_pin_by_name(dev, pins);

		if (!pin)
			continue;

		muxval = sunxi_pinctrl_muxval_by_name(pin, function);
		if (muxval >= 0)
			sunxi_pinctrl_set_function(dev, pin->pin.number, muxval);
		if (drive >= 0)
			sunxi_pinctrl_set_dlevel(dev, pin->pin.number, drive);
		if (bias >= 0)
			sunxi_pinctrl_set_bias(dev, pin->pin.number, bias);

	}

	return 0;
}

static int sunxi_pinctrl_probe(struct udevice *dev)
{
	struct sunxi_pinctrl_priv *priv = dev_get_priv(dev);
#if CONFIG_DM_GPIO
	struct udevice* gpiobridge;
#endif
	fdt_addr_t addr_base;
	fdt_size_t size;

        addr_base = fdtdec_get_addr_size_auto_noparent(gd->fdt_blob, dev->of_offset,
						       "reg", 0, &size, false);
	if (addr_base == FDT_ADDR_T_NONE)
		return -EINVAL;

	priv->base = (void *)addr_base;

#if CONFIG_DM_GPIO
	device_bind_driver_to_node(dev, "sunxi_pinctrl_gpiobridge", "gpiobridge",
				   dev->of_offset, &gpiobridge);

	/* The new device will have been created with no driver data, so we need
	   to set it here, as it contains the pin_base of the gpio-group.  */
	if (gpiobridge)
		gpiobridge->driver_data = dev_get_driver_data(dev);
#endif

	return 0;
}

static struct pinctrl_ops sunxi_pinctrl_ops = {
	.set_state	  = sunxi_pinctrl_set_state,
};

#if defined(CONFIG_MACH_SUN50I)
extern const struct sunxi_pinctrl_desc a64_pinctrl_data;
extern const struct sunxi_pinctrl_desc a64_r_pinctrl_data;
#endif

static const struct udevice_id sunxi_pinctrl_ids[] = {
#if defined(CONFIG_MACH_SUN50I)
	{ .compatible = "allwinner,sun50i-a64-pinctrl", .data = (ulong)&a64_pinctrl_data },
	{ .compatible = "allwinner,sun50i-a64-r-pinctrl", .data = (ulong)&a64_r_pinctrl_data },
#endif
	{ }
};

U_BOOT_DRIVER(pinctrl_sunxi) = {
	.name		= "sunxi_pinctrl",
	.id		= UCLASS_PINCTRL,
	.of_match	= sunxi_pinctrl_ids,
	.priv_auto_alloc_size = sizeof(struct sunxi_pinctrl_priv),
	.ops		= &sunxi_pinctrl_ops,
	.bind		= dm_scan_fdt_dev,
	.probe		= sunxi_pinctrl_probe,
};


#if defined(CONFIG_DM_GPIO)
/* The gpiobridge exists to translate <&pio 3 24 GPIO_ACTIVE_LOW> into the
   underlying GPIO bank.  It needs to access/understand the driver-data for
   the pinctrl device, so it lives here instead of in sunxi_gpio.c ... */

static int sunxi_gpiobridge_xlate(struct udevice *dev, struct gpio_desc *desc,
				  struct fdtdec_phandle_args *args)
{
	const struct sunxi_pinctrl_desc* data =
		(struct sunxi_pinctrl_desc*)dev_get_driver_data(dev);
	struct udevice* gpiobank;
	char name[3] = {'P', 'A', '\0' };

	if (!data) {
		debug("%s: no driver_data\n", dev->name);
		return -ENOENT;
	}

	/* Naming on each pinctrl may start with an offset (e.g. R_PIO
	   usually starts at 'PL'). */
	name[1] += data->pin_base / PINS_PER_BANK;
	/* Now add the bank-number within this pinctrl */
	name[1] += args->args[0];

	for (uclass_first_device(UCLASS_GPIO, &gpiobank);
	     gpiobank;
	     uclass_next_device(&gpiobank)) {
		int count;
		const char* bank_name = gpio_get_bank_info(gpiobank, &count);

		if (bank_name && !strcmp(bank_name, name)) {
			desc->dev = gpiobank;
			desc->offset = args->args[1];
			desc->flags = args->args[2] & GPIO_ACTIVE_LOW ? GPIOD_ACTIVE_LOW : 0;
			return 0;
		}
	}

	return -ENOENT;
}

static const struct dm_gpio_ops gpiobridge_sunxi_ops = {
	.xlate		= sunxi_gpiobridge_xlate,
};

U_BOOT_DRIVER(gpiobridge_sunxi) = {
	.name		= "sunxi_pinctrl_gpiobridge",
	.id		= UCLASS_GPIO,
	.ops            = &gpiobridge_sunxi_ops,
};
#endif /* DM_GPIO */
