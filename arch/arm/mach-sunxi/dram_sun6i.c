/*
 * Sun6i platform dram controller init.
 *
 * (C) Copyright 2007-2012
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 * Berg Xing <bergxing@allwinnertech.com>
 * Tom Cubie <tangliang@allwinnertech.com>
 *
 * (C) Copyright 2014 Hans de Goede <hdegoede@redhat.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#include <common.h>
#include <errno.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/dram.h>
#include <asm/arch/prcm.h>
#include "dram-timings/timings.h"


#define DRAM_CLK (CONFIG_DRAM_CLK * 1000000)

#define PS2CYCLES_FLOOR(n)    ((n * CONFIG_DRAM_CLK) / 1000000)
#define PS2CYCLES_ROUNDUP(n)  ((n * CONFIG_DRAM_CLK + 999999) / 1000000)
#define NS2CYCLES_FLOOR(n)    ((n * CONFIG_DRAM_CLK) / 1000)
#define NS2CYCLES_ROUNDUP(n)  ((n * CONFIG_DRAM_CLK + 999) / 1000)
#define MAX(a, b)             ((a) > (b) ? (a) : (b))


struct dram_sun6i_para {
	u8 bus_width;
	u8 chan;
	u8 rank;
	u8 rows;
	u16 page_size;
        u8 odten;
	const struct dram_bin* speed_bin;
};


/* TODO: Share these conversion functions across all dram-timings based controllers */
static inline int ns_floor_t(unsigned nanoseconds)
{
	return (CONFIG_DRAM_CLK * nanoseconds) / 1000;
}

static inline unsigned ns_roundup_t(unsigned nanoseconds)
{
	return DIV_ROUND_UP(CONFIG_DRAM_CLK * nanoseconds, 1000);
}

static inline unsigned ps_floor_t(unsigned picoseconds)
{
	return (CONFIG_DRAM_CLK * picoseconds) / 1000000;
}

static inline unsigned ps_roundup_t(unsigned picoseconds)
{
	return DIV_ROUND_UP(CONFIG_DRAM_CLK * picoseconds, 1000000);
}

static inline unsigned timing_to_t(const struct dram_timing* const constraints)
{
	return max(ps_floor_t(constraints->ps), constraints->ck);
}

static void mctl_sys_init(void)
{
	struct sunxi_ccm_reg * const ccm =
		(struct sunxi_ccm_reg *)SUNXI_CCM_BASE;
	const int dram_clk_div = 2;

	clock_set_pll5(DRAM_CLK * dram_clk_div, true);

	/* TODO: investigate whether the increased mdelay is related to enabling
	 *       sigma-delta above.  If so, we might as well increase the value
	 *       in the PLL lock-time register.
	 */
	mdelay(20);

	clrsetbits_le32(&ccm->dram_clk_cfg, CCM_DRAMCLK_CFG_DIV0_MASK,
		CCM_DRAMCLK_CFG_DIV0(dram_clk_div) | CCM_DRAMCLK_CFG_RST |
		CCM_DRAMCLK_CFG_UPD);
	mctl_await_completion(&ccm->dram_clk_cfg, CCM_DRAMCLK_CFG_UPD, 0);

	writel(MDFS_CLK_DEFAULT, &ccm->mdfs_clk_cfg);

	/* deassert mctl reset */
	setbits_le32(&ccm->ahb_reset0_cfg, 1 << AHB_RESET_OFFSET_MCTL);

	/* enable mctl clock */
	setbits_le32(&ccm->ahb_gate0, 1 << AHB_GATE_OFFSET_MCTL);
}

static void mctl_dll_init(int ch_index, struct dram_sun6i_para *para)
{
	struct sunxi_mctl_phy_reg *mctl_phy;

	if (ch_index == 0)
		mctl_phy = (struct sunxi_mctl_phy_reg *)SUNXI_DRAM_PHY0_BASE;
	else
		mctl_phy = (struct sunxi_mctl_phy_reg *)SUNXI_DRAM_PHY1_BASE;

	/* disable + reset dlls */
	writel(MCTL_DLLCR_DISABLE, &mctl_phy->acdllcr);
	writel(MCTL_DLLCR_DISABLE, &mctl_phy->dx0dllcr);
	writel(MCTL_DLLCR_DISABLE, &mctl_phy->dx1dllcr);
	if (para->bus_width == 32) {
		writel(MCTL_DLLCR_DISABLE, &mctl_phy->dx2dllcr);
		writel(MCTL_DLLCR_DISABLE, &mctl_phy->dx3dllcr);
	}
	udelay(2);

	/* enable + reset dlls */
	writel(0, &mctl_phy->acdllcr);
	writel(0, &mctl_phy->dx0dllcr);
	writel(0, &mctl_phy->dx1dllcr);
	if (para->bus_width == 32) {
		writel(0, &mctl_phy->dx2dllcr);
		writel(0, &mctl_phy->dx3dllcr);
	}
	udelay(22);

	/* enable and release reset of dlls */
	writel(MCTL_DLLCR_NRESET, &mctl_phy->acdllcr);
	writel(MCTL_DLLCR_NRESET, &mctl_phy->dx0dllcr);
	writel(MCTL_DLLCR_NRESET, &mctl_phy->dx1dllcr);
	if (para->bus_width == 32) {
		writel(MCTL_DLLCR_NRESET, &mctl_phy->dx2dllcr);
		writel(MCTL_DLLCR_NRESET, &mctl_phy->dx3dllcr);
	}
	udelay(22);
}

static bool mctl_rank_detect(u32 *gsr0, int rank)
{
	const u32 done = MCTL_DX_GSR0_RANK0_TRAIN_DONE << rank;
	const u32 err = MCTL_DX_GSR0_RANK0_TRAIN_ERR << rank;

	mctl_await_completion(gsr0, done, done);
	mctl_await_completion(gsr0 + 0x10, done, done);

	return !(readl(gsr0) & err) && !(readl(gsr0 + 0x10) & err);
}

static int mctl_set_timing_params(struct sunxi_mctl_ctl_reg *mctl_ctl,
				  struct sunxi_mctl_phy_reg *mctl_phy,
				  const struct dram_bin * const speed_bin)
	u8 trcd		= ps_roundup_t(speed_bin->tRCD);
	u8 trc		= ps_roundup_t(speed_bin->tRC);
	u8 trp		= ps_roundup_t(speed_bin->tRP);
	u8 tras		= ps_roundup_t(speed_bin->tRAS);
	/* On the A31 DRAM controller, t_refi is measured in 100ns units. */
	u16 trefi	= speed_bin->tREFI / 100 ;
	u16 trfc	= ns_roundup_t(speed_bin->tRFC);

	/* command and address timings */
	u8 tdllk        = speed_bin->tDLLK;
	u8 tccd		= speed_bin->tCCD;
	u8 tfaw		= ps_roundup_t(speed_bin->tFAW);
	u8 tmod		= timing_to_t(&speed_bin->tMOD);
	u8 tmrd		= speed_bin->tMRD;
	u8 trrd		= timing_to_t(&speed_bin->tRRD);
	u8 trtp		= timing_to_t(&speed_bin->tRTP);
	u8 twr		= ns_floor_t(speed_bin->tWR);
	u8 twtr		= timing_to_t(&speed_bin->tWTR);

	/* calibration timing */
	u8 tzqcs        = timing_to_t(&speed_bin->tZQCS);

	/* power-down timings */
	u8 txp		= timing_to_t(&speed_bin->tXP);
	u8 tcke		= timing_to_t(&speed_bin->tCKE);
	u8 txpdll       = timing_to_t(&speed_bin->tXPDLL);

	/* self refresh timings */
	u8 tcksrx	= timing_to_t(&speed_bin->tXS);
	u8 tcksre	= timing_to_t(&speed_bin->tCKSRE);
	u8 tckesr	= tcke + 1;

	u32 tdinit0	= (500 * CONFIG_DRAM_CLK);		/* 500us */
	u32 tdinit1	= (360 * CONFIG_DRAM_CLK + 999) / 1000;	/* 360ns */
	u32 tdinit2	= (200 * CONFIG_DRAM_CLK);		/* 200us */
	u32 tdinit3	= CONFIG_DRAM_CLK;		        /* 1us */

	u8 CL = 0, CWL = 0;

	if (dram_calculate_CL_CWL(speed_bin, CONFIG_DRAM_CLK, &CL, &CWL))
	    return 1;

	debug("DRAM timings: found CL = %d, CWL = %d for %d MHz\n", CL, CWL, CONFIG_DRAM_CLK);

	writel(DDR3_MR0_PPD_FAST_EXIT | DDR3_MR0_WR(twr) | DDR3_MR0_CL(CL), &mctl_phy->mr0);
	writel(DDR3_MR1_RTT120OHM, &mctl_phy->mr1);
	writel(DDR3_MR2_TWL(CWL), &mctl_phy->mr2);
	writel(0, &mctl_phy->mr3);

	writel((MCTL_TITMSRST << 18) | (MCTL_TDLLLOCK << 6) | MCTL_TDLLSRST,
	       &mctl_phy->ptr0);

	writel((tdinit1 << 19) | tdinit0, &mctl_phy->ptr1);
	writel((tdinit3 << 17) | tdinit2, &mctl_phy->ptr2);

	writel((tccd << 31) | (trc << 25) | (trrd << 21) |
	       (tras << 16) | (trcd << 12) | (trp << 8) |
	       (twtr << 5) | (trtp << 2) | (tmrd << 0),
	       &mctl_phy->dtpr0);

	/* tDQSCK and tDQSCKmax are used LPDDR2/LPDDR3 only */
	writel((MCTL_TDQSCKMAX << 27) | (MCTL_TDQSCK << 24) |
	       (trfc << 16) | (MCTL_TRTODT << 11) |
	       ((tmod - 12) << 9) | (tfaw << 3) | (0 << 2) |
	       (MCTL_TAOND << 0), &mctl_phy->dtpr1);

	writel((tdllk << 19) | (tcke << 15) | (txpdll << 10) |
	       (MCTL_TEXSR << 0), &mctl_phy->dtpr2);

	/* Set number of clks per micro-second */
	writel(DRAM_CLK / 1000000, &mctl_ctl->togcnt1u);
	/* Set number of clks per 100 nano-seconds */
	writel(DRAM_CLK / 10000000, &mctl_ctl->togcnt100n);
	/* Set memory timing registers */
	writel(trefi, &mctl_ctl->trefi);
	writel(tmrd, &mctl_ctl->tmrd);
	writel(trfc, &mctl_ctl->trfc);
	writel((MCTL_TPREA << 16) | trp, &mctl_ctl->trp);  /* TODO */
	writel(MCTL_TRTW, &mctl_ctl->trtw);
	writel(MCTL_TAL, &mctl_ctl->tal);
	writel(CL, &mctl_ctl->tcl);
	writel(CWL, &mctl_ctl->tcwl);
	writel(tras, &mctl_ctl->tras);
	writel(trc, &mctl_ctl->trc);
	writel(trcd, &mctl_ctl->trcd);
	writel(trrd, &mctl_ctl->trrd);
	writel(trtp, &mctl_ctl->trtp);
	writel(twr, &mctl_ctl->twr);
	writel(twtr, &mctl_ctl->twtr);
	writel(MCTL_TEXSR, &mctl_ctl->texsr);
	writel(txp, &mctl_ctl->txp);
	writel(txpdll, &mctl_ctl->txpdll);
	writel(tzqcs, &mctl_ctl->tzqcs);
	writel(MCTL_TZQCSI, &mctl_ctl->tzqcsi);
	writel(MCTL_TDQS, &mctl_ctl->tdqs);
	writel(tcksre, &mctl_ctl->tcksre);
	writel(tcksrx, &mctl_ctl->tcksrx);
	writel(tcke, &mctl_ctl->tcke);
	writel(tmod, &mctl_ctl->tmod);
	writel(MCTL_TRSTL, &mctl_ctl->trstl);
	writel(MCTL_TZQCL, &mctl_ctl->tzqcl);
	writel(MCTL_TMRR, &mctl_ctl->tmrr);
	writel(tckesr, &mctl_ctl->tckesr);
	writel(MCTL_TDPD, &mctl_ctl->tdpd);

	/* Set DFI timing registers */
	writel(CWL, &mctl_ctl->dfitphywrl);
	writel(CL - 1, &mctl_ctl->dfitrdden);
	writel(MCTL_DFITPHYRDL, &mctl_ctl->dfitphyrdl);
	writel(MCTL_DFISTCFG0, &mctl_ctl->dfistcfg0);

	return 0;
}

static int mctl_channel_init(int ch_index, struct dram_sun6i_para *para)
{
	struct sunxi_mctl_com_reg * const mctl_com =
		(struct sunxi_mctl_com_reg *)SUNXI_DRAM_COM_BASE;
	struct sunxi_mctl_ctl_reg *mctl_ctl;
	struct sunxi_mctl_phy_reg *mctl_phy;
	const struct dram_bin * const speed_bin = para->speed_bin;
	unsigned int ret;

	if (ch_index == 0) {
		mctl_ctl = (struct sunxi_mctl_ctl_reg *)SUNXI_DRAM_CTL0_BASE;
		mctl_phy = (struct sunxi_mctl_phy_reg *)SUNXI_DRAM_PHY0_BASE;
	} else {
		mctl_ctl = (struct sunxi_mctl_ctl_reg *)SUNXI_DRAM_CTL1_BASE;
		mctl_phy = (struct sunxi_mctl_phy_reg *)SUNXI_DRAM_PHY1_BASE;
	}

	/* Initialisation sequence */
	writel(MCTL_MCMD_NOP, &mctl_ctl->mcmd);
	mctl_await_completion(&mctl_ctl->mcmd, MCTL_MCMD_BUSY, 0);

	/* PHY initialization */
	writel(MCTL_PGCR, &mctl_phy->pgcr);

	ret = mctl_set_timing_params(mctl_ctl, mctl_phy, para->speed_bin);
	if (ret)
		return ret;

	writel(1, &mctl_ctl->dfitphyupdtype0);
	writel(MCTL_DCR_DDR3, &mctl_phy->dcr);
	writel(MCTL_DSGCR, &mctl_phy->dsgcr);
	writel(MCTL_DXCCR, &mctl_phy->dxccr);

	if (para->odten)
	  {
	    writel(0x2e80 | MCTL_DX_GCR_EN, &mctl_phy->dx0gcr);
	    writel(0x2e80 | MCTL_DX_GCR_EN, &mctl_phy->dx1gcr);
	    writel(0x2e80 | MCTL_DX_GCR_EN, &mctl_phy->dx2gcr);
	    writel(0x2e80 | MCTL_DX_GCR_EN, &mctl_phy->dx3gcr);
	  }
	else
	  {
	    writel(MCTL_DX_GCR | MCTL_DX_GCR_EN, &mctl_phy->dx0gcr);
	    writel(MCTL_DX_GCR | MCTL_DX_GCR_EN, &mctl_phy->dx1gcr);
	    writel(MCTL_DX_GCR | MCTL_DX_GCR_EN, &mctl_phy->dx2gcr);
	    writel(MCTL_DX_GCR | MCTL_DX_GCR_EN, &mctl_phy->dx3gcr);
	  }

	mctl_await_completion(&mctl_phy->pgsr, 0x03, 0x03);

	writel(CONFIG_DRAM_ZQ, &mctl_phy->zq0cr1);

	setbits_le32(&mctl_phy->pir, MCTL_PIR_CLEAR_STATUS);
	writel(MCTL_PIR_STEP1, &mctl_phy->pir);
	udelay(10);
	mctl_await_completion(&mctl_phy->pgsr, 0x1f, 0x1f);

	/* rank detect */
	if (!mctl_rank_detect(&mctl_phy->dx0gsr0, 1)) {
		para->rank = 1;
		clrbits_le32(&mctl_phy->pgcr, MCTL_PGCR_RANK);
	}

	/*
	 * channel detect, check channel 1 dx0 and dx1 have rank 0, if not
	 * assume nothing is connected to channel 1.
	 */
	if (ch_index == 1 && !mctl_rank_detect(&mctl_phy->dx0gsr0, 0)) {
		para->chan = 1;
		clrbits_le32(&mctl_com->ccr, MCTL_CCR_CH1_CLK_EN);
		return 0;
	}

	/* bus width detect, if dx2 and dx3 don't have rank 0, assume 16 bit */
	if (!mctl_rank_detect(&mctl_phy->dx2gsr0, 0)) {
		para->bus_width = 16;
		para->page_size = 2048;
		setbits_le32(&mctl_phy->dx2dllcr, MCTL_DLLCR_DISABLE);
		setbits_le32(&mctl_phy->dx3dllcr, MCTL_DLLCR_DISABLE);
		clrbits_le32(&mctl_phy->dx2gcr, MCTL_DX_GCR_EN);
		clrbits_le32(&mctl_phy->dx3gcr, MCTL_DX_GCR_EN);
	}

	setbits_le32(&mctl_phy->pir, MCTL_PIR_CLEAR_STATUS);
	writel(MCTL_PIR_STEP2, &mctl_phy->pir);
	udelay(10);
	mctl_await_completion(&mctl_phy->pgsr, 0x11, 0x11);

	if (readl(&mctl_phy->pgsr) & MCTL_PGSR_TRAIN_ERR_MASK)
		panic("Training error initialising DRAM\n");

	/* Move to configure state */
	writel(MCTL_SCTL_CONFIG, &mctl_ctl->sctl);
	mctl_await_completion(&mctl_ctl->sstat, 0x07, 0x01);

	/* Unknown magic performed by boot0 */
	setbits_le32(&mctl_ctl->dfiodtcfg, 1 << 3 | 1 << 14); /* rank0_odt_write_sel */
	clrbits_le32(&mctl_ctl->dfiodtcfg1, 0x1f);

	/* Select 16/32-bits mode for MCTL */
	if (para->bus_width == 16)
		setbits_le32(&mctl_ctl->ppcfg, 1);

#if defined(MODE2T)
	writel(MCTL_MCFG_DDR3 | (1 << 3), &mctl_ctl->mcfg);
#else
	writel(MCTL_MCFG_DDR3, &mctl_ctl->mcfg);
#endif

	/* DFI update configuration register */
	writel(MCTL_DFIUPDCFG_UPD, &mctl_ctl->dfiupdcfg);

	/* Move to access state */
	writel(MCTL_SCTL_ACCESS, &mctl_ctl->sctl);
	mctl_await_completion(&mctl_ctl->sstat, 0x07, 0x03);

	return 0;
}

static void mctl_com_init(struct dram_sun6i_para *para)
{
	struct sunxi_mctl_com_reg * const mctl_com =
		(struct sunxi_mctl_com_reg *)SUNXI_DRAM_COM_BASE;
	struct sunxi_mctl_phy_reg * const mctl_phy1 =
		(struct sunxi_mctl_phy_reg *)SUNXI_DRAM_PHY1_BASE;
	struct sunxi_prcm_reg * const prcm =
		(struct sunxi_prcm_reg *)SUNXI_PRCM_BASE;

	writel(MCTL_CR_UNKNOWN | MCTL_CR_CHANNEL(para->chan) | MCTL_CR_DDR3 |
	       ((para->bus_width == 32) ? MCTL_CR_BUSW32 : MCTL_CR_BUSW16) |
	       MCTL_CR_PAGE_SIZE(para->page_size) | MCTL_CR_ROW(para->rows) |
	       MCTL_CR_BANK(1) | MCTL_CR_RANK(para->rank), &mctl_com->cr);

	/* Unknown magic performed by boot0 */
	setbits_le32(&mctl_com->dbgcr, (1 << 6));

	if (para->chan == 1) {
		/* Shutdown channel 1 */
		setbits_le32(&mctl_phy1->aciocr, MCTL_ACIOCR_DISABLE);
		setbits_le32(&mctl_phy1->dxccr, MCTL_DXCCR_DISABLE);
		clrbits_le32(&mctl_phy1->dsgcr, MCTL_DSGCR_ENABLE);
		/*
		 * CH0 ?? this is what boot0 does. Leave as is until we can
		 * confirm this.
		 */
		setbits_le32(&prcm->vdd_sys_pwroff,
			     PRCM_VDD_SYS_DRAM_CH0_PAD_HOLD_PWROFF);
	}
}

static void mctl_port_cfg(void)
{
	struct sunxi_mctl_com_reg * const mctl_com =
		(struct sunxi_mctl_com_reg *)SUNXI_DRAM_COM_BASE;
	struct sunxi_ccm_reg * const ccm =
		(struct sunxi_ccm_reg *)SUNXI_CCM_BASE;

	/* enable DRAM AXI clock for CPU access */
	setbits_le32(&ccm->axi_gate, 1 << AXI_GATE_OFFSET_DRAM);

	/* Bunch of magic writes performed by boot0 */
	writel(0x00400302, &mctl_com->rmcr[0]);
	writel(0x01000307, &mctl_com->rmcr[1]);
	writel(0x00400302, &mctl_com->rmcr[2]);
	writel(0x01000307, &mctl_com->rmcr[3]);
	writel(0x01000307, &mctl_com->rmcr[4]);
	writel(0x01000303, &mctl_com->rmcr[6]);
	writel(0x01000303, &mctl_com->mmcr[0]);
	writel(0x00400310, &mctl_com->mmcr[1]);
	writel(0x01000307, &mctl_com->mmcr[2]);
	writel(0x01000303, &mctl_com->mmcr[3]);
	writel(0x01800303, &mctl_com->mmcr[4]);
	writel(0x01800303, &mctl_com->mmcr[5]);
	writel(0x01800303, &mctl_com->mmcr[6]);
	writel(0x01800303, &mctl_com->mmcr[7]);
	writel(0x01000303, &mctl_com->mmcr[8]);
	writel(0x00000002, &mctl_com->mmcr[15]);
	writel(0x00000310, &mctl_com->mbagcr[0]);
	writel(0x00400310, &mctl_com->mbagcr[1]);
	writel(0x00400310, &mctl_com->mbagcr[2]);
	writel(0x00000307, &mctl_com->mbagcr[3]);
	writel(0x00000317, &mctl_com->mbagcr[4]);
	writel(0x00000307, &mctl_com->mbagcr[5]);
}

unsigned long sunxi_dram_init(void)
{
	struct sunxi_mctl_com_reg * const mctl_com =
		(struct sunxi_mctl_com_reg *)SUNXI_DRAM_COM_BASE;
	u32 offset;
	int bank, bus, columns;

	/* Set initial parameters, these get modified by the autodetect code */
	struct dram_sun6i_para para = {
		.bus_width = 32,
		.chan = 2,
		.rank = 2,
		.page_size = 4096,
		.rows = 16,
#if defined(CONFIG_DRAM_ODT_EN)
		.odten = 1,
#else
		.odten = 0,
#endif
		.speed_bin = &DDR3_1600K,
	};

	/* A31s only has one channel */
	if (sunxi_get_ss_bonding_id() == SUNXI_SS_BOND_ID_A31S)
		para.chan = 1;

	mctl_sys_init();

	mctl_dll_init(0, &para);
	setbits_le32(&mctl_com->ccr, MCTL_CCR_CH0_CLK_EN);

	if (para.chan == 2) {
		mctl_dll_init(1, &para);
		setbits_le32(&mctl_com->ccr, MCTL_CCR_CH1_CLK_EN);
	}

	setbits_le32(&mctl_com->ccr, MCTL_CCR_MASTER_CLK_EN);
	udelay(200);

	if (mctl_channel_init(0, &para))
		return 0;

	if (para.chan == 2)
		mctl_channel_init(1, &para);

	mctl_com_init(&para);
	mctl_port_cfg();

	/*
	 * Change to 1 ch / sequence / 8192 byte pages / 16 rows /
	 * 8 bit banks / 1 rank mode.
	 */
	clrsetbits_le32(&mctl_com->cr,
		MCTL_CR_CHANNEL_MASK | MCTL_CR_PAGE_SIZE_MASK |
		    MCTL_CR_ROW_MASK | MCTL_CR_BANK_MASK | MCTL_CR_RANK_MASK,
		MCTL_CR_CHANNEL(1) | MCTL_CR_SEQUENCE |
		    MCTL_CR_PAGE_SIZE(8192) | MCTL_CR_ROW(16) |
		    MCTL_CR_BANK(1) | MCTL_CR_RANK(1));

	/* Detect and set page size */
	for (columns = 7; columns < 20; columns++) {
		if (mctl_mem_matches(1 << columns))
			break;
	}
	bus = (para.bus_width == 32) ? 2 : 1;
	columns -= bus;
	para.page_size = (1 << columns) * (bus << 1);
	clrsetbits_le32(&mctl_com->cr, MCTL_CR_PAGE_SIZE_MASK,
			MCTL_CR_PAGE_SIZE(para.page_size));

	/* Detect and set rows */
	for (para.rows = 11; para.rows < 16; para.rows++) {
		offset = 1 << (para.rows + columns + bus);
		if (mctl_mem_matches(offset))
			break;
	}
	clrsetbits_le32(&mctl_com->cr, MCTL_CR_ROW_MASK,
			MCTL_CR_ROW(para.rows));

	/* Detect bank size */
	offset = 1 << (para.rows + columns + bus + 2);
	bank = mctl_mem_matches(offset) ? 0 : 1;

	/* Restore interleave, chan and rank values, set bank size */
	clrsetbits_le32(&mctl_com->cr,
			MCTL_CR_CHANNEL_MASK | MCTL_CR_SEQUENCE |
			    MCTL_CR_BANK_MASK | MCTL_CR_RANK_MASK,
			MCTL_CR_CHANNEL(para.chan) | MCTL_CR_BANK(bank) |
			    MCTL_CR_RANK(para.rank));

	return 1 << (para.rank + para.rows + bank + columns + para.chan + bus);
}
