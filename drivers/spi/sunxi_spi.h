/*
 * SPI driver for Allwinner sunxi SoCs
 *
 * Supported chips
 * - Allwinner A31 (sun6i, sun8iw1p1)
 * - Allwinner A64 (sun50iw1p1)
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

#include <spi.h>

#ifndef __SUNXI_SPI_H__
#define __SUNXI_SPI_H__

#define MAX_SPI_BYTES			64

#define CCM_AHB_GATE_SPI0		(1 << 20)
#define CCM_AHB_GATE_SPI1		(1 << 21)
#define CCM_AHB_GATE_SPI2		(1 << 22)
#define CCM_AHB_GATE_SPI3		(1 << 23)

#define CCM_AHB_RESET_SPI0		(1 << 20)
#define CCM_AHB_RESET_SPI1		(1 << 21)
#define CCM_AHB_RESET_SPI2		(1 << 22)
#define CCM_AHB_RESET_SPI3		(1 << 23)

#define SUNXI_MAX_SPI_SEQ		3

#define SUNXI_SPI0_MOSI_PIN		SUNXI_GPC(0)
#define SUNXI_SPI0_MISO_PIN		SUNXI_GPC(1)
#define SUNXI_SPI0_CLK_PIN		SUNXI_GPC(2)
#if defined(CONFIG_MACH_SUN50I)
#define SUNXI_SPI0_CS0_PIN              SUNXI_GPC(3)
#else
#define SUNXI_SPI0_CS0_PIN		SUNXI_GPC(27)
#endif
#define SUNXI_SPI0_CS1_PIN		-1
#if defined(CONFIG_MACH_SUN50I)
#define SUNXI_SPI0_MOSI_VAL		4
#define SUNXI_SPI0_MISO_VAL		4
#define SUNXI_SPI0_CLK_VAL		4
#define SUNXI_SPI0_CS0_VAL		4
#else
#define SUNXI_SPI0_MOSI_VAL		3
#define SUNXI_SPI0_MISO_VAL		3
#define SUNXI_SPI0_CLK_VAL		3
#define SUNXI_SPI0_CS0_VAL		3
#endif
#define SUNXI_SPI0_CS1_VAL		-1

#define SUNXI_SPI1_MOSI_PIN		SUNXI_GPG(15)
#define SUNXI_SPI1_MISO_PIN		SUNXI_GPG(16)
#define SUNXI_SPI1_CLK_PIN		SUNXI_GPG(14)
#define SUNXI_SPI1_CS0_PIN		SUNXI_GPG(13)
#define SUNXI_SPI1_CS1_PIN		SUNXI_GPG(12)
#define SUNXI_SPI1_MOSI_VAL		2
#define SUNXI_SPI1_MISO_VAL		2
#define SUNXI_SPI1_CLK_VAL		2
#define SUNXI_SPI1_CS0_VAL		2
#define SUNXI_SPI1_CS1_VAL		2

#define SUNXI_SPI2_MOSI_PIN		SUNXI_GPH(11)
#define SUNXI_SPI2_MISO_PIN		SUNXI_GPH(12)
#define SUNXI_SPI2_CLK_PIN		SUNXI_GPH(10)
#define SUNXI_SPI2_CS0_PIN		SUNXI_GPH(9)
#define SUNXI_SPI2_CS1_PIN		-1
#define SUNXI_SPI2_MOSI_VAL		2
#define SUNXI_SPI2_MISO_VAL		2
#define SUNXI_SPI2_CLK_VAL		2
#define SUNXI_SPI2_CS0_VAL		2
#define SUNXI_SPI2_CS1_VAL		-1

#define SUNXI_SPI3_MOSI_PIN		SUNXI_GPA(23)
#define SUNXI_SPI3_MISO_PIN		SUNXI_GPA(24)
#define SUNXI_SPI3_CLK_PIN		SUNXI_GPA(22)
#define SUNXI_SPI3_CS0_PIN		SUNXI_GPA(21)
#define SUNXI_SPI3_CS1_PIN		SUNXI_GPA(25)
#define SUNXI_SPI3_MOSI_VAL		4
#define SUNXI_SPI3_MISO_VAL		4
#define SUNXI_SPI3_CLK_VAL		4
#define SUNXI_SPI3_CS0_VAL		4
#define SUNXI_SPI3_CS1_VAL		4

#define SUNXI_PIN_INPUT_DISABLE 7

#define SUNXI_SPI_GCR_MASTER	(1 << 1)
#define SUNXI_SPI_GCR_EN		(1 << 0)

#define SUNXI_SPI_CPOL_LOW		(1 << 1)
#define SUNXI_SPI_CPHA_LOW		(1 << 0)

#define SUNXI_SPI_CPOL_HIGH		(1 << 1)
#define SUNXI_SPI_CPHA_HIGH		(1 << 0)

#define SUNXI_SPI_MOSI_PIN(n)	SUNXI_SPI##n##_MOSI_PIN
#define SUNXI_SPI_MOSI_VAL(n)	SUNXI_SPI##n##_MOSI_VAL
#define SUNXI_SPI_MISO_PIN(n)	SUNXI_SPI##n##_MISO_PIN
#define SUNXI_SPI_MISO_VAL(n)	SUNXI_SPI##n##_MISO_VAL
#define SUNXI_SPI_CLK_PIN(n)	SUNXI_SPI##n##_CLK_PIN
#define SUNXI_SPI_CLK_VAL(n)	SUNXI_SPI##n##_CLK_VAL
#define SUNXI_SPI_CS0_PIN(n)	SUNXI_SPI##n##_CS0_PIN
#define SUNXI_SPI_CS0_VAL(n)	SUNXI_SPI##n##_CS0_VAL
#define SUNXI_SPI_CS1_PIN(n)	SUNXI_SPI##n##_CS1_PIN
#define SUNXI_SPI_CS1_VAL(n)	SUNXI_SPI##n##_CS1_VAL

struct sunxi_spi_platdata {
	unsigned int max_hz;
	void* base;
};

struct sunxi_spi_privdata {
#if !defined(CONFIG_DM_SPI)
	struct spi_slave slave;
        void* base;
#endif
	u8 spi_is_init;
	u8 clk_pol;
	u8 clk_pha;
	u8 clk_div_n;
	u8 clk_div_m;
};

struct sunxi_spi_reg {
	u32		_reserved0;
	u32		GCR;   /* SPI Global Control register */
	u32		TCR;   /* SPI Transfer Control register */
	u32		_reserved1;
	u32		IER;   /* SPI Interrupt Control register */
	u32		ISR;   /* SPI Interrupt Status register */
	u32		FCR;   /* SPI FIFO Control register */
	u32		FSR;   /* SPI FIFO Status register */
	u32		WCR;   /* SPI Wait Clock Counter register */
	u32		CCR;   /* SPI Clock Rate Control register */
	u32		_reserved2;
	u32		_reserved3;
	u32		MBC;   /* SPI Burst Counter register */
	u32		MTC;   /* SPI Transmit Counter register */
	u32		BCC;   /* SPI Burst Control register */
	u32		_reserved4[113];
	u32		TXD;   /* SPI TX Data register */
	u32		_reserved5[63];
	u32		RXD;   /* SPI RX Data register */
};

struct sunxi_spi_slave {
	struct spi_slave	slave;
	struct sunxi_spi_reg*	base;
	int			polarity;
	unsigned int		max_hz;
	unsigned int		mode;
};

#endif // __SUNXI_SPI_H__
