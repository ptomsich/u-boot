/*
 * SPI driver for Allwinner sunxi SoCs
 *
 * Supported chips
 * - Allwinner A31 (sun6i)
 *
 * Copyright (C) 2015 Theobroma Systems Design und Consulting GmbH
 * Octav Zlatior <octav.zlatior@theobroma-systems.com>
 * Philipp Tomsich <philipp.tomsich@theobroma-systems.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#include <common.h>
#include <dm.h>
#include <spi.h>
#include <fdtdec.h>
#include <asm/errno.h>
#include <configs/sun6i.h>
#include <asm/io.h>
#include <asm/arch-sunxi/gpio.h>
#include <asm/arch-sunxi/clock_sun6i.h>

#include "sunxi_spi.h"

DECLARE_GLOBAL_DATA_PTR;

// some arrays to make things easier to express
static int sunxi_spi_mosi_pin[4] = {SUNXI_SPI0_MOSI_PIN,
	SUNXI_SPI1_MOSI_PIN, SUNXI_SPI2_MOSI_PIN, SUNXI_SPI3_MOSI_PIN};
static int sunxi_spi_miso_pin[4] = {SUNXI_SPI0_MISO_PIN,
	SUNXI_SPI1_MISO_PIN, SUNXI_SPI2_MISO_PIN, SUNXI_SPI3_MISO_PIN};
static int sunxi_spi_clk_pin[4] = {SUNXI_SPI0_CLK_PIN,
	SUNXI_SPI1_CLK_PIN, SUNXI_SPI2_CLK_PIN, SUNXI_SPI3_CLK_PIN};
static int sunxi_spi_cs0_pin[4] = {SUNXI_SPI0_CS0_PIN,
	SUNXI_SPI1_CS0_PIN, SUNXI_SPI2_CS0_PIN, SUNXI_SPI3_CS0_PIN};
static int sunxi_spi_cs1_pin[4] = {SUNXI_SPI0_CS1_PIN,
	SUNXI_SPI1_CS1_PIN, SUNXI_SPI2_CS1_PIN, SUNXI_SPI3_CS1_PIN};
static int sunxi_spi_mosi_val[4] = {SUNXI_SPI0_MOSI_VAL,
	SUNXI_SPI1_MOSI_VAL, SUNXI_SPI2_MOSI_VAL, SUNXI_SPI3_MOSI_VAL};
static int sunxi_spi_miso_val[4] = {SUNXI_SPI0_MISO_VAL,
	SUNXI_SPI1_MISO_VAL, SUNXI_SPI2_MISO_VAL, SUNXI_SPI3_MISO_VAL};
static int sunxi_spi_clk_val[4] = {SUNXI_SPI0_CLK_VAL,
	SUNXI_SPI1_CLK_VAL, SUNXI_SPI2_CLK_VAL, SUNXI_SPI3_CLK_VAL};
static int sunxi_spi_cs0_val[4] = {SUNXI_SPI0_CS0_VAL,
	SUNXI_SPI1_CS0_VAL, SUNXI_SPI2_CS0_VAL, SUNXI_SPI3_CS0_VAL};
static int sunxi_spi_cs1_val[4] = {SUNXI_SPI0_CS1_VAL,
	SUNXI_SPI1_CS1_VAL, SUNXI_SPI2_CS1_VAL, SUNXI_SPI3_CS1_VAL};


/* Common xfer-function used both by the DM and non-DM (e.g. SPL) interfaces */
static int _sunxi_spi_xfer(struct sunxi_spi_reg* spi, unsigned int bitlen,
			   const void *dout, void *din, unsigned long flags)
{
	int n_bytes = DIV_ROUND_UP(bitlen, 8);
	int ret;
	u32 blk_size, cnt;
	u8 *p_outbuf = (u8*)dout;
	u8 *p_inbuf = (u8*)din;
	unsigned char* p;
#ifdef CONFIG_SPL_BUILD
	int i;

	printf("_sunxi_spi_xfer %p\n", spi);
#if 0
	if (dout)
	  {
	    for (i = 0; i < n_bytes; ++i)
	      printf("%02x ", p_outbuf[i]);
	  }
	printf("\n");
#endif
#endif
	debug("%s: %p, %d, %p, %016lx\n", __func__, spi, bitlen, din, flags);

	while (!(readl(&spi->ISR) & (1 << 1) /* RXFIFO empty */ ))
		(void)readb(&spi->RXD);

	if (flags & SPI_XFER_BEGIN) {
		setbits_le32(&spi->TCR, (1 << 6));  // SS_OWNER
		clrbits_le32(&spi->TCR, (1 << 7));  // SS_LEVEL
	}

	while (n_bytes>0) {
		ret = 0;

		if (n_bytes<MAX_SPI_BYTES)
			blk_size = n_bytes;
		else
			blk_size = MAX_SPI_BYTES;

		cnt = blk_size;

		// start XCHG
		writel(blk_size, &spi->MBC);
		writel(blk_size, &spi->MTC);
		writel(blk_size, &spi->BCC);

		p = p_outbuf;
		while (cnt--) {
			writeb(*p++, &spi->TXD);
		}

		setbits_le32(&spi->TCR, (1 << 31)); // XCHG bit

		while (readl(&spi->TCR) & (1 << 31))
			/* wait for the xfer to complete */;

		setbits_le32(&spi->ISR, (1 << 12)); // clear TC
		cnt = blk_size;

		p = p_inbuf;
		while (cnt--) {
			unsigned char c;
			c = readb(&spi->RXD);
			if (din) {
				*p++ = c;
			}
		}

		if (ret)
			return ret;
		if (dout)
			p_outbuf += blk_size;
		if (din)
			p_inbuf += blk_size;
		n_bytes -= blk_size;
	}

	if (flags & SPI_XFER_END) {
		setbits_le32(&spi->TCR, (1 << 6));  // SS_OWNER
		setbits_le32(&spi->TCR, (1 << 7));  // SS_LEVEL
	}
	udelay(1);

#ifdef CONFIG_SPL_BUILD
#if 0
	{
	  n_bytes = DIV_ROUND_UP(bitlen, 8);
	  if (din)
	    {
	      p_inbuf = (u8*)din;
	      printf("<- ");
	      for (i = 0; i < n_bytes; ++i)
		printf("%02x ", p_inbuf[i]);
	    }
	  printf("\n");
	}
#endif
#endif

	return 0;
};

#if !defined(CONFIG_DM_SPI)
struct spi_slave *spi_setup_slave(unsigned int bus, unsigned int cs,
				     unsigned int max_hz, unsigned int mode)
{
        struct sunxi_spi_privdata *priv;
	printf("sunxi:spi_setup_slave\n");
	priv = spi_alloc_slave(struct sunxi_spi_privdata, bus, cs);

	priv->base = SUNXI_SPI0_BASE;

	return &priv->slave;
}

int spi_claim_bus(struct spi_slave *slave)
{
	struct sunxi_ccm_reg* const ccm = (struct sunxi_ccm_reg *)SUNXI_CCM_BASE;
	struct sunxi_spi_reg* spi = (struct sunxi_spi_reg *)SUNXI_SPI0_BASE;
	int seq = 0; /* which SPI controller ? */
	int cs = 0; /* which CS */
	int clk_div_m = 0;
	int clk_div_n = 0;

	printf("sunxi:spi_claim_bus\n");

	sunxi_gpio_set_cfgpin(sunxi_spi_mosi_pin[seq], sunxi_spi_mosi_val[seq]);
	sunxi_gpio_set_cfgpin(sunxi_spi_miso_pin[seq], sunxi_spi_miso_val[seq]);
	sunxi_gpio_set_cfgpin(sunxi_spi_clk_pin[seq], sunxi_spi_clk_val[seq]);
	if (cs == 0)
		sunxi_gpio_set_cfgpin(sunxi_spi_cs0_pin[seq], sunxi_spi_cs0_val[seq]);
	else
		sunxi_gpio_set_cfgpin(sunxi_spi_cs1_pin[seq], sunxi_spi_cs1_val[seq]);

	/* Reset FIFOs */
	setbits_le32(&spi->FCR, (1 << 31) | (1 << 15));

	switch (seq) {
		case 0:
			setbits_le32(&ccm->spi0_clk_cfg,
				     (clk_div_m << 0) | (clk_div_n << 16));
			setbits_le32(&ccm->spi0_clk_cfg, (1 << 31));
			setbits_le32(&ccm->ahb_gate0, CCM_AHB_GATE_SPI0);
			setbits_le32(&ccm->ahb_reset0_cfg, CCM_AHB_RESET_SPI0);
			break;
		case 1:
			setbits_le32(&ccm->spi1_clk_cfg,
				     (clk_div_m << 0) | (clk_div_n << 16));
			setbits_le32(&ccm->spi1_clk_cfg, (1 << 31));
			setbits_le32(&ccm->ahb_gate0, CCM_AHB_GATE_SPI1);
			setbits_le32(&ccm->ahb_reset0_cfg, CCM_AHB_RESET_SPI1);
			break;
		case 2:
			setbits_le32(&ccm->spi2_clk_cfg,
				     (clk_div_m << 0) | (clk_div_n << 16));
			setbits_le32(&ccm->spi2_clk_cfg, (1 << 31));
			setbits_le32(&ccm->ahb_gate0, CCM_AHB_GATE_SPI2);
			setbits_le32(&ccm->ahb_reset0_cfg, CCM_AHB_RESET_SPI2);
			break;
		case 3:
			setbits_le32(&ccm->spi3_clk_cfg,
				     (clk_div_m << 0) | (clk_div_n << 16));
			setbits_le32(&ccm->spi3_clk_cfg, (1 << 31));
			setbits_le32(&ccm->ahb_gate0, CCM_AHB_GATE_SPI3);
			setbits_le32(&ccm->ahb_reset0_cfg, CCM_AHB_RESET_SPI3);
			break;
	}

	setbits_le32(&spi->GCR, SUNXI_SPI_GCR_MASTER | SUNXI_SPI_GCR_EN);
	setbits_le32(&spi->TCR, 3 /* priv->clk_pol | priv->clk_pha */);  /* TODO */

	udelay(10);

	return 0;
}

void spi_release_bus(struct spi_slave *slave)
{
	struct sunxi_ccm_reg* const ccm = (struct sunxi_ccm_reg *)SUNXI_CCM_BASE;
	struct sunxi_spi_reg* spi = (struct sunxi_spi_reg *)SUNXI_SPI0_BASE;
	int seq = 0; /* which SPI controller ? */
	int cs = 0; /* which CS */
	int clk_div_m = 0;
	int clk_div_n = 0;

	printf("sunxi:spi_release_bus\n");

	clrbits_le32(&spi->GCR, SUNXI_SPI_GCR_MASTER | SUNXI_SPI_GCR_EN);
	clrbits_le32(&spi->TCR, SUNXI_SPI_CPOL_LOW | SUNXI_SPI_CPOL_LOW);

	switch (seq) {
		case 0:
			clrbits_le32(&ccm->spi0_clk_cfg, (1 << 31));
			clrbits_le32(&ccm->ahb_gate0, CCM_AHB_GATE_SPI0);
			clrbits_le32(&ccm->ahb_reset0_cfg, CCM_AHB_RESET_SPI0);
			break;
		case 1:
			clrbits_le32(&ccm->spi1_clk_cfg, (1 << 31));
			clrbits_le32(&ccm->ahb_gate0, CCM_AHB_GATE_SPI1);
			clrbits_le32(&ccm->ahb_reset0_cfg, CCM_AHB_RESET_SPI1);
			break;
		case 2:
			clrbits_le32(&ccm->spi2_clk_cfg, (1 << 31));
			clrbits_le32(&ccm->ahb_gate0, CCM_AHB_GATE_SPI2);
			clrbits_le32(&ccm->ahb_reset0_cfg, CCM_AHB_RESET_SPI2);
			break;
		case 3:
			clrbits_le32(&ccm->spi3_clk_cfg, (1 << 31));
			clrbits_le32(&ccm->ahb_gate0, CCM_AHB_GATE_SPI3);
			clrbits_le32(&ccm->ahb_reset0_cfg, CCM_AHB_RESET_SPI3);
			break;
	}

	sunxi_gpio_set_cfgpin(sunxi_spi_mosi_pin[seq], SUNXI_PIN_INPUT_DISABLE);
	sunxi_gpio_set_cfgpin(sunxi_spi_miso_pin[seq], SUNXI_PIN_INPUT_DISABLE);
	sunxi_gpio_set_cfgpin(sunxi_spi_clk_pin[seq], SUNXI_PIN_INPUT_DISABLE);
	sunxi_gpio_set_cfgpin(sunxi_spi_cs0_pin[seq], SUNXI_PIN_INPUT_DISABLE);
	sunxi_gpio_set_cfgpin(sunxi_spi_cs1_pin[seq], SUNXI_PIN_INPUT_DISABLE);
}

void spi_init(void)
{
        struct sunxi_spi_reg*  spi0 = (struct sunxi_spi_reg*)SUNXI_SPI0_BASE;
        u32  spi_ver = readl(&spi0->VER);
        printf("SUNXI SPI version %d.%d\n", spi_ver >> 16, spi_ver & 0xffff);
}

void spi_set_speed(struct spi_slave *slave, uint hz)
{
        printf("sunxi_spi: spi_set_speed(..., %d)\n", hz);
}

int spi_xfer(struct spi_slave *slave, unsigned int bitlen, const void *dout,
		void *din, unsigned long flags)
{
	struct sunxi_spi_slave* sunxi_slave = container_of(slave, struct sunxi_spi_slave, slave);
        struct sunxi_spi_reg*  spi = sunxi_slave->base;

	return _sunxi_spi_xfer(spi, bitlen, dout, din, flags);
}

void spi_free_slave(struct spi_slave *slave)
{
	struct sunxi_spi_slave* sunxi_slave = container_of(slave, struct sunxi_spi_slave, slave);

	free(sunxi_slave);
}

#else /* defined(CONFIG_DM_SPI) */
static int sunxi_spi_ofdata_to_platdata(struct udevice *dev) {
	debug("%s: %p\n", __func__, dev);
	if (!dev)
		return -ENODEV;

	struct sunxi_spi_platdata *plat = dev_get_platdata(dev);
	const void *blob = gd->fdt_blob;
	int node = dev->of_offset;
	u32 data[2];
	int ret;

	ret = fdtdec_get_int_array(blob, node, "reg", data, ARRAY_SIZE(data));
	if (ret) {
		printf("Error: Can't get base address (ret=%d)\n", ret);
		return -ENODEV;
	}

	plat->base = data[0];
	plat->max_hz = fdtdec_get_int(blob, node, "spi-max-frequency", 24000000);

	debug("%s: base=%x, max-frequency=%d\n",
		__func__, plat->base, plat->max_hz);

	return 0;
};

static int sunxi_spi_init(struct udevice *dev) {
	debug("%s: %p\n", __func__, dev);
	if (!dev)
		return -ENODEV;

	struct sunxi_spi_platdata *plat = dev_get_platdata(dev);
	struct sunxi_spi_reg *spi = (struct sunxi_spi_reg *)plat->base;
	u32 spi_ver = readl(&spi->VER);
	printf("sunxi SPI @0x%08x, version %d.%d\n",
		plat->base, spi_ver >> 16, spi_ver & 0xffff);
	return 0;
};

static int sunxi_spi_probe(struct udevice *dev) {
	debug("%s: %p\n", __func__, dev);
	if (!dev)
		return -ENODEV;

	struct sunxi_spi_privdata *priv = dev_get_priv(dev);

	if (!priv->spi_is_init) {
		sunxi_spi_init(dev);
		priv->spi_is_init = 1;
	}

	return 0;
};

static int sunxi_spi_claim_bus(struct udevice *dev) {
	debug("%s: %p %p\n", __func__, dev, dev->parent);
	if (!dev)
		return -ENODEV;

	struct sunxi_spi_platdata *plat = dev_get_platdata(dev->parent);
	struct sunxi_spi_privdata *priv = dev_get_priv(dev->parent);
#if defined(CONFIG_DM_SPI)
	struct dm_spi_slave_platdata *slave = dev_get_parent_platdata(dev);
#endif

	int seq = dev->parent->seq;
	if (seq<0 || seq>SUNXI_MAX_SPI_SEQ)
		return -ENODEV;

#if defined(CONFIG_DM_SPI)
	// cs can be either 0 or 1
	if (slave->cs<0 || slave->cs>1)
		return -ENODEV;
	// cs can be one only for bus 1 and bus 3
	if ((seq==0 || seq==2) && slave->cs==1)
		return -ENODEV;
#endif

	struct sunxi_ccm_reg* const ccm = (struct sunxi_ccm_reg *)SUNXI_CCM_BASE;
	struct sunxi_spi_reg* spi = (struct sunxi_spi_reg *)plat->base;

	sunxi_gpio_set_cfgpin(sunxi_spi_mosi_pin[seq], sunxi_spi_mosi_val[seq]);
	sunxi_gpio_set_cfgpin(sunxi_spi_miso_pin[seq], sunxi_spi_miso_val[seq]);
	sunxi_gpio_set_cfgpin(sunxi_spi_clk_pin[seq], sunxi_spi_clk_val[seq]);
#if defined(CONFIG_DM_SPI)
	if (slave->cs==0)
		sunxi_gpio_set_cfgpin(sunxi_spi_cs0_pin[seq], sunxi_spi_cs0_val[seq]);
	else
		sunxi_gpio_set_cfgpin(sunxi_spi_cs1_pin[seq], sunxi_spi_cs1_val[seq]);
#else
	/* TODO: CS1 */
	sunxi_gpio_set_cfgpin(sunxi_spi_cs0_pin[seq], sunxi_spi_cs0_val[seq]);
#endif

	/* Reset FIFOs */
	setbits_le32(&spi->FCR, (1 << 31) | (1 << 15));

	switch (seq) {
		case 0:
			setbits_le32(&ccm->spi0_clk_cfg,
				(priv->clk_div_m << 0) | (priv->clk_div_n << 16));
			setbits_le32(&ccm->spi0_clk_cfg, (1 << 31));
			setbits_le32(&ccm->ahb_gate0, CCM_AHB_GATE_SPI0);
			setbits_le32(&ccm->ahb_reset0_cfg, CCM_AHB_RESET_SPI0);
			break;
		case 1:
			setbits_le32(&ccm->spi1_clk_cfg,
				(priv->clk_div_m << 0) | (priv->clk_div_n << 16));
			setbits_le32(&ccm->spi1_clk_cfg, (1 << 31));
			setbits_le32(&ccm->ahb_gate0, CCM_AHB_GATE_SPI1);
			setbits_le32(&ccm->ahb_reset0_cfg, CCM_AHB_RESET_SPI1);
			break;
		case 2:
			setbits_le32(&ccm->spi2_clk_cfg,
				(priv->clk_div_m << 0) | (priv->clk_div_n << 16));
			setbits_le32(&ccm->spi2_clk_cfg, (1 << 31));
			setbits_le32(&ccm->ahb_gate0, CCM_AHB_GATE_SPI2);
			setbits_le32(&ccm->ahb_reset0_cfg, CCM_AHB_RESET_SPI2);
			break;
		case 3:
			setbits_le32(&ccm->spi3_clk_cfg,
				(priv->clk_div_m << 0) | (priv->clk_div_n << 16));
			setbits_le32(&ccm->spi3_clk_cfg, (1 << 31));
			setbits_le32(&ccm->ahb_gate0, CCM_AHB_GATE_SPI3);
			setbits_le32(&ccm->ahb_reset0_cfg, CCM_AHB_RESET_SPI3);
			break;
	}

	setbits_le32(&spi->GCR, SUNXI_SPI_GCR_MASTER | SUNXI_SPI_GCR_EN);
	setbits_le32(&spi->TCR, priv->clk_pol | priv->clk_pha);

	udelay(10);

	return 0;
};

static int sunxi_spi_release_bus(struct udevice *dev) {
	debug("%s: %p\n", __func__, dev);
	if (!dev)
		return -ENODEV;

	struct sunxi_spi_platdata *plat = dev_get_platdata(dev->parent);
	int seq = dev->parent->seq;
	if (seq<0 || seq>SUNXI_MAX_SPI_SEQ)
		return -ENODEV;

	struct sunxi_ccm_reg* const ccm = (struct sunxi_ccm_reg *)SUNXI_CCM_BASE;
	struct sunxi_spi_reg* spi = (struct sunxi_spi_reg *)plat->base;

	sunxi_gpio_set_cfgpin(sunxi_spi_mosi_pin[seq], SUNXI_PIN_INPUT_DISABLE);
	sunxi_gpio_set_cfgpin(sunxi_spi_miso_pin[seq], SUNXI_PIN_INPUT_DISABLE);
	sunxi_gpio_set_cfgpin(sunxi_spi_clk_pin[seq], SUNXI_PIN_INPUT_DISABLE);
	sunxi_gpio_set_cfgpin(sunxi_spi_cs0_pin[seq], SUNXI_PIN_INPUT_DISABLE);
	sunxi_gpio_set_cfgpin(sunxi_spi_cs1_pin[seq], SUNXI_PIN_INPUT_DISABLE);

	switch (seq) {
		case 0:
			clrbits_le32(&ccm->spi0_clk_cfg, (1 << 31));
			clrbits_le32(&ccm->ahb_gate0, CCM_AHB_GATE_SPI0);
			clrbits_le32(&ccm->ahb_reset0_cfg, CCM_AHB_RESET_SPI0);
			break;
		case 1:
			clrbits_le32(&ccm->spi1_clk_cfg, (1 << 31));
			clrbits_le32(&ccm->ahb_gate0, CCM_AHB_GATE_SPI1);
			clrbits_le32(&ccm->ahb_reset0_cfg, CCM_AHB_RESET_SPI1);
			break;
		case 2:
			clrbits_le32(&ccm->spi2_clk_cfg, (1 << 31));
			clrbits_le32(&ccm->ahb_gate0, CCM_AHB_GATE_SPI2);
			clrbits_le32(&ccm->ahb_reset0_cfg, CCM_AHB_RESET_SPI2);
			break;
		case 3:
			clrbits_le32(&ccm->spi3_clk_cfg, (1 << 31));
			clrbits_le32(&ccm->ahb_gate0, CCM_AHB_GATE_SPI3);
			clrbits_le32(&ccm->ahb_reset0_cfg, CCM_AHB_RESET_SPI3);
			break;
	}

	clrbits_le32(&spi->GCR, SUNXI_SPI_GCR_MASTER | SUNXI_SPI_GCR_EN);
	clrbits_le32(&spi->TCR, SUNXI_SPI_CPOL_LOW | SUNXI_SPI_CPOL_LOW);

	return 0;
};

static int sunxi_spi_set_wordlen(struct udevice *dev, unsigned int wordlen) {
	debug("%s: %p,  %d\n", __func__, dev, wordlen);
	if (!dev)
		return -ENODEV;

	return 0;
};

static int sunxi_spi_xfer(struct udevice *dev, unsigned int bitlen,
	const void *dout, void *din, unsigned long flags)
{
	debug("%s: %p, %d, %p, %016lx\n", __func__, dev, bitlen, din, flags);
	if (!dev)
		return -ENODEV;

	struct sunxi_spi_platdata *plat = dev_get_platdata(dev->parent);
	struct sunxi_spi_reg* spi = (struct sunxi_spi_reg *)plat->base;

	return _sunxi_spi_xfer(spi, bitlen, dout, din, flags);
};

static int sunxi_spi_set_speed(struct udevice *bus, unsigned int hz) {
	debug("%s: %p, %d\n", __func__, bus, hz);
	if (!bus)
		return -ENODEV;

	struct sunxi_spi_platdata *plat = dev_get_platdata(bus);
	struct sunxi_spi_privdata *priv = dev_get_priv(bus);

	if (hz > plat->max_hz) {
		debug("selected speed exceeds maximum of %d, using max instead\n",
			plat->max_hz);
		hz = plat->max_hz;
	}

	// frequency is base / (M * N)
	// M = m+1, N = 2^n
	unsigned int base_hz = 24000000;
	unsigned int M, N, m, n;
	unsigned int factor = base_hz / hz; // M * N

	if (factor > 4*16) {
		N = 8;
		n = 3;
	}
	else if (factor > 2*16) {
		N = 4;
		n = 2;
	}
	else if (factor > 16) {
		N = 2;
		n = 1;
	}
	else {
		N = 1;
		n = 0;
	}

	M = factor/N;
	if (M>16)
		M = 16;
	if (M<1)
		M = 1;
	m = M-1;

	debug("spi frequency: %d, (24 / (%d x %d) )\n",
		base_hz / (M * N), M, N);

	priv->clk_div_n = n;
	priv->clk_div_m = m;

	return 0;
};

static int sunxi_spi_set_mode(struct udevice *bus, unsigned int mode) {
	debug("%s: %p, %08x\n", __func__, bus, mode);
	if (!bus)
		return -ENODEV;

	struct sunxi_spi_privdata *priv = dev_get_priv(bus);

	priv->clk_pol = (mode & SUNXI_SPI_CPOL_LOW);
	priv->clk_pha = (mode & SUNXI_SPI_CPHA_LOW);

	return 0;
};

static int sunxi_spi_cs_info(struct udevice *bus, unsigned int cs,
	struct spi_cs_info *info)
{
	debug("%s: %p, %d\n", __func__, bus, cs);
	if (!bus)
		return -ENODEV;
	if (!info)
		return -EFAULT;

	int seq = bus->seq;

	// TODO: for now, spi0 and spi2 support only cs0, while spi1 and spi3
	// support cs0 and cs1; anything else is not supported; do we need
	// something fancier here?
	if (seq<0 || seq>SUNXI_MAX_SPI_SEQ)
		return -ENODEV;
	if (cs>1)
		return -ENODEV;
	if ((seq==0 || seq==2) && (cs!=0))
		return -ENODEV;

	return 0;
};

static const struct dm_spi_ops sunxi_spi_ops = {
	.claim_bus		= sunxi_spi_claim_bus,
	.release_bus	= sunxi_spi_release_bus,
	.set_wordlen	= sunxi_spi_set_wordlen,
	.xfer			= sunxi_spi_xfer,
	.set_speed		= sunxi_spi_set_speed,
	.set_mode		= sunxi_spi_set_mode,
	.cs_info		= sunxi_spi_cs_info,
};

static const struct udevice_id sunxi_spi_ids[] = {
	{ .compatible = "allwinner,sun6i-a31-spi" },
	{ }
};

U_BOOT_DRIVER(sunxi_spi) = {
	.name = "sunxi_spi",
	.id = UCLASS_SPI,
	.of_match = sunxi_spi_ids,
	.ofdata_to_platdata = sunxi_spi_ofdata_to_platdata,
	.platdata_auto_alloc_size = sizeof(struct sunxi_spi_platdata),
	.priv_auto_alloc_size = sizeof(struct sunxi_spi_privdata),
	.probe = sunxi_spi_probe,
	.ops = &sunxi_spi_ops,
};
#endif
