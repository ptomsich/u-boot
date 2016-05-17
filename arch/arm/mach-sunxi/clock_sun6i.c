/*
 * sun6i specific clock code
 *
 * (C) Copyright 2007-2012
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 * Tom Cubie <tangliang@allwinnertech.com>
 *
 * (C) Copyright 2013 Luke Kenneth Casson Leighton <lkcl@lkcl.net>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/prcm.h>
#include <asm/arch/sys_proto.h>

#ifdef CONFIG_SPL_BUILD
void clock_init_safe(void)
{
	struct sunxi_ccm_reg * const ccm =
		(struct sunxi_ccm_reg *)SUNXI_CCM_BASE;

#if !defined(CONFIG_MACH_SUN8I_H3) && !defined(CONFIG_MACH_SUN50I)
	struct sunxi_prcm_reg * const prcm =
		(struct sunxi_prcm_reg *)SUNXI_PRCM_BASE;

	/* Set PLL ldo voltage without this PLL6 does not work properly.
	 *
	 * As the A31 manual states, that "before enable PLL, PLLVDD
	 * LDO should be set to 1.37v", we need to configure this to 2.5v
	 * in the "PLL Input Power Select" (0 << 15) and (7 << 16).
	 */
	clrsetbits_le32(&prcm->pll_ctrl1, PRCM_PLL_CTRL_LDO_KEY_MASK,
			PRCM_PLL_CTRL_LDO_KEY);
	clrsetbits_le32(&prcm->pll_ctrl1, ~PRCM_PLL_CTRL_LDO_KEY_MASK,
			PRCM_PLL_CTRL_LDO_DIGITAL_EN | PRCM_PLL_CTRL_LDO_ANALOG_EN |
			PRCM_PLL_CTRL_EXT_OSC_EN | PRCM_PLL_CTRL_LDO_OUT_L(1370) );
	clrbits_le32(&prcm->pll_ctrl1, PRCM_PLL_CTRL_LDO_KEY_MASK);
#endif

	/* Give the PLL LDO voltage setting some time to take hold.
	 * Notes:
	 *   1) We need to use sdelay() as the timers aren't set up yet.
	 *   2) The 100k iterations come from Boot1, which spin's for 100k
	 *      iterations through a loop.
	 */
	sdelay(100000);

	/* Reinitialise PLL1 (CPUX) to 408MHz */
	clock_set_pll1(408000000);
	/* Enable PLL6 at 600MHz */
	clock_pll6_init();

	writel(AHB1_ABP1_DIV_DEFAULT, &ccm->ahb1_apb1_div);

	writel(MBUS_CLK_DEFAULT, &ccm->mbus0_clk_cfg);
	if (IS_ENABLED(CONFIG_MACH_SUN6I))
		writel(MBUS_CLK_DEFAULT, &ccm->mbus1_clk_cfg);
}
#endif

void clock_init_sec(void)
{
#ifdef CONFIG_MACH_SUN8I_H3
	struct sunxi_ccm_reg * const ccm =
		(struct sunxi_ccm_reg *)SUNXI_CCM_BASE;

	setbits_le32(&ccm->ccu_sec_switch,
		     CCM_SEC_SWITCH_MBUS_NONSEC |
		     CCM_SEC_SWITCH_BUS_NONSEC |
		     CCM_SEC_SWITCH_PLL_NONSEC);
#endif
}

void clock_init_uart(void)
{
#if CONFIG_CONS_INDEX < 5
	struct sunxi_ccm_reg *const ccm =
		(struct sunxi_ccm_reg *)SUNXI_CCM_BASE;

	/* uart clock source is apb2 */
	writel(APB2_CLK_SRC_OSC24M|
	       APB2_CLK_RATE_N_1|
	       APB2_CLK_RATE_M(1),
	       &ccm->apb2_div);

	/* open the clock for uart */
	setbits_le32(&ccm->apb2_gate,
		     CLK_GATE_OPEN << (APB2_GATE_UART_SHIFT +
				       CONFIG_CONS_INDEX - 1));

	/* deassert uart reset */
	setbits_le32(&ccm->apb2_reset_cfg,
		     1 << (APB2_RESET_UART_SHIFT +
			   CONFIG_CONS_INDEX - 1));
#else
	/* enable R_PIO and R_UART clocks, and de-assert resets */
	prcm_apb0_enable(PRCM_APB0_GATE_PIO | PRCM_APB0_GATE_UART);
#endif
}

static inline void wait_for_pll_lock(u32 *reg)
{
	do {
		/* spin */
	} while ((readl(reg) & CCM_PLLx_CTRL_LOCK) != CCM_PLLx_CTRL_LOCK);
}

#ifdef CONFIG_SPL_BUILD
void clock_set_pll1(unsigned int clk)
{
	struct sunxi_ccm_reg * const ccm =
		(struct sunxi_ccm_reg *)SUNXI_CCM_BASE;
	const int p = 0;
	int k = 2;
	int m = 2;

	if (clk > 1152000000) {
		k = 4;
		m = 2;
	} else if (clk > 768000000) {
		k = 3;
		m = 2;
	}

	/* Switch to 24MHz clock while changing PLL1 */
	writel(AXI_DIV_3 << AXI_DIV_SHIFT |
	       ATB_DIV_2 << ATB_DIV_SHIFT |
	       CPU_CLK_SRC_OSC24M << CPU_CLK_SRC_SHIFT,
	       &ccm->cpu_axi_cfg);

	/*
	 * sun6i: PLL1 rate = ((24000000 * n * k) >> 0) / m   (p is ignored)
	 * sun8i: PLL1 rate = ((24000000 * n * k) >> p) / m
	 */
	writel(CCM_PLL1_CTRL_EN | CCM_PLL1_CTRL_P(p) |
	       CCM_PLL1_CTRL_N(clk / (24000000 * k / m)) |
	       CCM_PLL1_CTRL_K(k) | CCM_PLL1_CTRL_M(m), &ccm->pll1_cfg);

	wait_for_pll_lock(&ccm->pll1_cfg);

	/* Switch CPU to PLL1 */
	writel(AXI_DIV_3 << AXI_DIV_SHIFT |
	       ATB_DIV_2 << ATB_DIV_SHIFT |
	       CPU_CLK_SRC_PLL1 << CPU_CLK_SRC_SHIFT,
	       &ccm->cpu_axi_cfg);
}
#endif

void clock_set_pll3(unsigned int clk)
{
	struct sunxi_ccm_reg * const ccm =
		(struct sunxi_ccm_reg *)SUNXI_CCM_BASE;
	const int m = 8; /* 3 MHz steps just like sun4i, sun5i and sun7i */

	if (clk == 0) {
		clrbits_le32(&ccm->pll3_cfg, CCM_PLL3_CTRL_EN);
		return;
	}

	/* PLL3 rate = 24000000 * n / m */
	writel(CCM_PLL3_CTRL_EN | CCM_PLL3_CTRL_INTEGER_MODE |
	       CCM_PLL3_CTRL_N(clk / (24000000 / m)) | CCM_PLL3_CTRL_M(m),
	       &ccm->pll3_cfg);

	wait_for_pll_lock(&ccm->pll3_cfg);
}

void clock_set_pll5(unsigned int clk, bool sigma_delta_enable)
{
	struct sunxi_ccm_reg * const ccm =
		(struct sunxi_ccm_reg *)SUNXI_CCM_BASE;
	int k = 3, m = 4;

#ifdef CONFIG_MACH_SUN8I_H3
	clrsetbits_le32(&ccm->pll5_tuning_cfg, CCM_PLL5_TUN_LOCK_TIME_MASK |
			CCM_PLL5_TUN_INIT_FREQ_MASK,
			CCM_PLL5_TUN_LOCK_TIME(2) | CCM_PLL5_TUN_INIT_FREQ(16));
#endif

	if (sigma_delta_enable)
		writel(CCM_PLL5_PATTERN, &ccm->pll5_pattern_cfg);

	/* PLL5 rate = 24000000 * n * k / m */
	if (clk > 1152000000) {
		k = 4;
		m = 2;
	} else if (clk > 768000000) {
		k = 3;
		m = 2;
	} else if (clk > 576000000) {
	        k = 2;
	        m = 2;
	}

	writel(CCM_PLL5_CTRL_EN |
	       (sigma_delta_enable ? CCM_PLL5_CTRL_SIGMA_DELTA_EN : 0) |
	       CCM_PLL5_CTRL_UPD |
	       CCM_PLL5_CTRL_N(clk / (24000000 * k / m)) |
	       CCM_PLL5_CTRL_K(k) | CCM_PLL5_CTRL_M(m), &ccm->pll5_cfg);

       /* On PLL5, the update needs first to take effect (i.e. CCM_PLL5_CTRL_UPD
	* must auto-clear... and only then we can wait for the PLL to lock.  */
	while (readl(&ccm->pll5_cfg) & CCM_PLL5_CTRL_UPD)
	  /* spin */ ;

	wait_for_pll_lock(&ccm->pll5_cfg);
}

#ifdef CONFIG_MACH_SUN6I
void clock_set_mipi_pll(unsigned int clk)
{
	struct sunxi_ccm_reg * const ccm =
		(struct sunxi_ccm_reg *)SUNXI_CCM_BASE;
	unsigned int k, m, n, value, diff;
	unsigned best_k = 0, best_m = 0, best_n = 0, best_diff = 0xffffffff;
	unsigned int src = clock_get_pll3();

	/* All calculations are in KHz to avoid overflows */
	clk /= 1000;
	src /= 1000;

	/* Pick the closest lower clock */
	for (k = 1; k <= 4; k++) {
		for (m = 1; m <= 16; m++) {
			for (n = 1; n <= 16; n++) {
				value = src * n * k / m;
				if (value > clk)
					continue;

				diff = clk - value;
				if (diff < best_diff) {
					best_diff = diff;
					best_k = k;
					best_m = m;
					best_n = n;
				}
				if (diff == 0)
					goto done;
			}
		}
	}

done:
	writel(CCM_MIPI_PLL_CTRL_EN | CCM_MIPI_PLL_CTRL_LDO_EN |
	       CCM_MIPI_PLL_CTRL_N(best_n) | CCM_MIPI_PLL_CTRL_K(best_k) |
	       CCM_MIPI_PLL_CTRL_M(best_m), &ccm->mipi_pll_cfg);
}
#endif

void clock_pll6_init()
{
	struct sunxi_ccm_reg * const ccm =
		(struct sunxi_ccm_reg *)SUNXI_CCM_BASE;

	/* The A31 reference manual states, that "the PLL6 output
	 * should be fixed to 600MHz, and is not recommended to be
	 * modified.".
	 *
	 * We always configure this to N=25 and K=2 for 600Mhz:
	 *	 600MHz = 24MHz x N(25) x K(2) / 2
	 *
	 * N.B.: The lowest 2 bits need to be writte 01b (0x1),
	 *       as the fixed divider doesn't seem to be fixed.
	 */
	writel(CCM_PLL6_CTRL_EN | CCM_PLL6_CTRL_24M_OUT_EN |
	       CCM_PLL6_CTRL_N(25) | CCM_PLL6_CTRL_K(2) |
	       CCM_PLL6_CTRL_M(2),
	       &ccm->pll6_cfg);

	wait_for_pll_lock(&ccm->pll6_cfg);
}

#if defined(CONFIG_MACH_SUN8I_A33) || defined(CONFIG_MACH_SUN50I)
void clock_set_pll11(unsigned int clk, bool sigma_delta_enable)
{
	struct sunxi_ccm_reg * const ccm =
		(struct sunxi_ccm_reg *)SUNXI_CCM_BASE;

	if (sigma_delta_enable)
		writel(CCM_PLL11_PATTERN, &ccm->pll11_pattern_cfg0);

	writel(CCM_PLL11_CTRL_EN | CCM_PLL11_CTRL_UPD |
	       (sigma_delta_enable ? CCM_PLL11_CTRL_SIGMA_DELTA_EN : 0) |
	       CCM_PLL11_CTRL_N(clk / 24000000), &ccm->pll11_cfg);

	while (readl(&ccm->pll11_cfg) & CCM_PLL11_CTRL_UPD)
		;
}
#endif

unsigned int clock_get_pll3(void)
{
	struct sunxi_ccm_reg *const ccm =
		(struct sunxi_ccm_reg *)SUNXI_CCM_BASE;
	uint32_t rval = readl(&ccm->pll3_cfg);
	int n = ((rval & CCM_PLL3_CTRL_N_MASK) >> CCM_PLL3_CTRL_N_SHIFT) + 1;
	int m = ((rval & CCM_PLL3_CTRL_M_MASK) >> CCM_PLL3_CTRL_M_SHIFT) + 1;

	/* Multiply by 1000 after dividing by m to avoid integer overflows */
	return (24000 * n / m) * 1000;
}

unsigned int clock_get_pll6(void)
{
	struct sunxi_ccm_reg *const ccm =
		(struct sunxi_ccm_reg *)SUNXI_CCM_BASE;
	uint32_t rval = readl(&ccm->pll6_cfg);
	int n = ((rval & CCM_PLL6_CTRL_N_MASK) >> CCM_PLL6_CTRL_N_SHIFT) + 1;
	int k = ((rval & CCM_PLL6_CTRL_K_MASK) >> CCM_PLL6_CTRL_K_SHIFT) + 1;
	return 24000000 * n * k / 2;
}

unsigned int clock_get_mipi_pll(void)
{
	struct sunxi_ccm_reg *const ccm =
		(struct sunxi_ccm_reg *)SUNXI_CCM_BASE;
	uint32_t rval = readl(&ccm->mipi_pll_cfg);
	unsigned int n = ((rval & CCM_MIPI_PLL_CTRL_N_MASK) >> CCM_MIPI_PLL_CTRL_N_SHIFT) + 1;
	unsigned int k = ((rval & CCM_MIPI_PLL_CTRL_K_MASK) >> CCM_MIPI_PLL_CTRL_K_SHIFT) + 1;
	unsigned int m = ((rval & CCM_MIPI_PLL_CTRL_M_MASK) >> CCM_MIPI_PLL_CTRL_M_SHIFT) + 1;
	unsigned int src = clock_get_pll3();

	/* Multiply by 1000 after dividing by m to avoid integer overflows */
	return ((src / 1000) * n * k / m) * 1000;
}

void clock_set_de_mod_clock(u32 *clk_cfg, unsigned int hz)
{
	int pll = clock_get_pll6() * 2;
	int div = 1;

	while ((pll / div) > hz)
		div++;

	writel(CCM_DE_CTRL_GATE | CCM_DE_CTRL_PLL6_2X | CCM_DE_CTRL_M(div),
	       clk_cfg);
}
