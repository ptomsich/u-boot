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

#undef CONFIG_DRAM_CLK
#define CONFIG_DRAM_CLK 388
//#define CONFIG_DRAM_CLK 480

//#define MODE2T

//#undef CONFIG_DRAM_ZQ
//#define CONFIG_DRAM_ZQ 0x44

#define DRAM_CLK (CONFIG_DRAM_CLK * 1000000)

#define PS2CYCLES_FLOOR(n)    ((n * CONFIG_DRAM_CLK) / 1000000)
#define PS2CYCLES_ROUNDUP(n)  ((n * CONFIG_DRAM_CLK + 999999) / 1000000)
#define NS2CYCLES_FLOOR(n)    ((n * CONFIG_DRAM_CLK) / 1000)
#define NS2CYCLES_ROUNDUP(n)  ((n * CONFIG_DRAM_CLK + 999) / 1000)
#define MAX(a, b)             ((a) > (b) ? (a) : (b))



/* A number of DDR3 timings are given as "the greater of a fixed number of
   clock cycles (CK) or nanoseconds.  We express these using a structure
   that holds a cycle count and a duration in picoseconds (so we can model
   sub-ns timings, such as 7.5ns without losing precision or resorting to
   rounding up early. */
struct dram_sun6i_timing
{
	u32 ck;
	u32 ps;
};

/* */
struct dram_sun6i_cl_cwl_timing
{
	u32 CL;
	u32 CWL;
	u32 tCKmin;  /* in ps */
	u32 tCKmax;  /* in ps */
};

struct dram_sun6i_para {
	u8 bus_width;
	u8 chan;
	u8 rank;
	u8 rows;
	u16 page_size;

  u8 odten;

	/* Timing information for each speed-bin */
	struct dram_sun6i_cl_cwl_timing *cl_cwl_table;
	u32 cl_cwl_numentries;

	/* For the timings, we try to keep the order and grouping used in
	   JEDEC Standard No. 79-3F */

	/* timings */
	u32 tREFI; /* in ns */
	u32 tRFC;  /* in ns */

	u32 tRAS;  /* in ps */

	/* command and address timing */
	u32 tDLLK; /* in nCK */
	struct dram_sun6i_timing tRTP;
	struct dram_sun6i_timing tWTR;
	u32 tWR;   /* in nCK */
	u32 tMRD;  /* in nCK */
	struct dram_sun6i_timing tMOD;
	u32 tRCD;  /* in ps */
	u32 tRP;   /* in ps */
	u32 tRC;   /* in ps */
	u32 tCCD;  /* in nCK */
	struct dram_sun6i_timing tRRD;
	u32 tFAW;  /* in ps */

	/* calibration timing */
	//	struct dram_sun6i_timing tZQinit;
	struct dram_sun6i_timing tZQoper;
	struct dram_sun6i_timing tZQCS;

	/* reset timing */
	//	struct dram_sun6i_timing tXPR;

	/* self-refresh timings */
	struct dram_sun6i_timing tXS;
	u32 tXSDLL; /* in nCK */
	//	struct dram_sun6i_timing tCKESR;
	struct dram_sun6i_timing tCKSRE;
	struct dram_sun6i_timing tCKSRX;

	/* power-down timings */
	struct dram_sun6i_timing tXP;
	struct dram_sun6i_timing tXPDLL;
	struct dram_sun6i_timing tCKE;

	/* write leveling timings */
	u32 tWLMRD;    /* min, in nCK */
	//	u32 tWLDQSEN;  /* min, in nCK */
	u32 tWLO;      /* max, in ns */
	//	u32 tWLOE;     /* max, in ns */

	//	u32 tCKDPX;  /* in nCK */
	//	u32 tCKCSX;  /* in nCK */
};


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

static void mctl_channel_init(int ch_index, struct dram_sun6i_para *para)
{
	struct sunxi_mctl_com_reg * const mctl_com =
		(struct sunxi_mctl_com_reg *)SUNXI_DRAM_COM_BASE;
	struct sunxi_mctl_ctl_reg *mctl_ctl;
	struct sunxi_mctl_phy_reg *mctl_phy;

	u32 CL = 0;
	u32 CWL = 0;
	u16 mr[4] = { 0, }; /* initialise to silence compiler warnings due to non-DDR3
			       placeholder code */

	/* Convert the values to cycle counts (nCK) from what is provided by the
	   definition of each speed bin. */
	const u32 tREFI = para->tREFI / 100;
	const u32 tRFC  = NS2CYCLES_ROUNDUP(para->tRFC);
	const u32 tRCD  = PS2CYCLES_ROUNDUP(para->tRCD);
	const u32 tRP   = PS2CYCLES_ROUNDUP(para->tRP);
	const u32 tRC   = PS2CYCLES_ROUNDUP(para->tRC);
	const u32 tRAS  = PS2CYCLES_ROUNDUP(para->tRAS);

	/* command and address timing */
	const u32 tDLLK = para->tDLLK;
	const u32 tRTP  = MAX(para->tRTP.ck, PS2CYCLES_ROUNDUP(para->tRTP.ps));
	const u32 tWTR  = MAX(para->tWTR.ck, PS2CYCLES_ROUNDUP(para->tWTR.ps));
	const u32 tWR   = NS2CYCLES_FLOOR(para->tWR);
	const u32 tMRD  = para->tMRD;
	const u32 tMOD  = MAX(para->tMOD.ck, PS2CYCLES_ROUNDUP(para->tMOD.ps));
	const u32 tCCD  = para->tCCD;
	const u32 tRRD  = MAX(para->tRRD.ck, PS2CYCLES_ROUNDUP(para->tRRD.ps));
	const u32 tFAW  = PS2CYCLES_ROUNDUP(para->tFAW);

	/* calibration timings */
	//	const u32 tZQinit = MAX(para->tZQinit.ck, PS2CYCLES_ROUNDUP(para->tZQinit.ps));
	const u32 tZQoper = MAX(para->tZQoper.ck, PS2CYCLES_ROUNDUP(para->tZQoper.ps));
	const u32 tZQCS   = MAX(para->tZQCS.ck, PS2CYCLES_ROUNDUP(para->tZQCS.ps));

	/* reset timing */
	//	const u32 tXPR  = MAX(para->tXPR.ck, PS2CYCLES_ROUNDUP(para->tXPR.ps));

	/* power-down timings */
	const u32 tXP    = MAX(para->tXP.ck, PS2CYCLES_ROUNDUP(para->tXP.ps));
	const u32 tXPDLL = MAX(para->tXPDLL.ck, PS2CYCLES_ROUNDUP(para->tXPDLL.ps));
	const u32 tCKE   = MAX(para->tCKE.ck, PS2CYCLES_ROUNDUP(para->tCKE.ps));

	/* self-refresh timings (keep below power-down timings, as tCKESR needs to
	   be calculated based on the nCK value of tCKE) */
	const u32 tXS    = MAX(para->tXS.ck, PS2CYCLES_ROUNDUP(para->tXS.ps));
	const u32 tXSDLL = para->tXSDLL;
	const u32 tCKSRE = MAX(para->tCKSRE.ck, PS2CYCLES_ROUNDUP(para->tCKSRE.ps));
	const u32 tCKESR = tCKE + 1;
	const u32 tCKSRX = MAX(para->tCKSRX.ck, PS2CYCLES_ROUNDUP(para->tCKSRX.ps));

	/* write leveling timings */
	const u32 tWLMRD = para->tWLMRD;
	//	const u32 tWLDQSEN = para->tWLDQSEN;
	const u32 tWLO = PS2CYCLES_FLOOR(para->tWLO);
	//	const u32 tWLOE = PS2CYCLES_FLOOR(para->tWLOE);

	const u32 tRASmax = tREFI * 9;
	int i;

	for (i = 0; i < para->cl_cwl_numentries; ++i) {
		const u32 tCK = 1000000 / CONFIG_DRAM_CLK;
		if ((para->cl_cwl_table[i].tCKmin <= tCK)
		    && (tCK < para->cl_cwl_table[i].tCKmax)) {
			CL = para->cl_cwl_table[i].CL;
			CWL = para->cl_cwl_table[i].CWL;

			//			printf("found CL/CWL: CL = %d, CWL = %d\n", CL, CWL);
			break;
		}
	}

	if ((CL == 0) && (CWL == 0)) {
	  //		printf("failed to find valid CL/CWL for operating point %d MHz\n", CONFIG_DRAM_CLK);
		return 0;
	}

	/* DRAM_TYPE_DDR3 */
	  /* Constants for assembling MR0 */
#define DDR3_MR0_PPD_FAST_EXIT             (1 << 12)
#define DDR3_MR0_WR(n) \
	  ( (n <= 8) ? ((n - 4) << 9) : (((n >> 1) & 0x7) << 9) )
#define DDR3_MR0_CL(n) \
	  ( (((n - 4) & 0x7) << 4) | (((n - 4) & 0x8) >> 2) )
#define DDR3_MR0_BL8                       (0b00 << 0)

#define DDR3_MR1_RTT120OHM                 ((0 << 9) | (1 << 6) | (0 << 2))

#define DDR3_MR2_TWL(n) \
	  ( ((n - 5) & 0x7) << 3 )

	        mr[0] = DDR3_MR0_PPD_FAST_EXIT | DDR3_MR0_WR(tWR) | DDR3_MR0_CL(CL);
		mr[1] = DDR3_MR1_RTT120OHM;
		mr[2] = DDR3_MR2_TWL(CWL);
		mr[3] = 0;

	/* end of timings calculation */

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
	writel(mr[0], &mctl_phy->mr0);
	writel(mr[1], &mctl_phy->mr1);
	writel(mr[2], &mctl_phy->mr2);
	writel(mr[3], &mctl_phy->mr3);

	writel((MCTL_TITMSRST << 18) | (MCTL_TDLLLOCK << 6) | MCTL_TDLLSRST,
	       &mctl_phy->ptr0);

	writel((MCTL_TDINIT1 << 19) | MCTL_TDINIT0, &mctl_phy->ptr1);
	writel((MCTL_TDINIT3 << 17) | MCTL_TDINIT2, &mctl_phy->ptr2);

	writel((tCCD << 31) | (tRC << 25) | (tRRD << 21) |
	       (tRAS << 16) | (tRCD << 12) | (tRP << 8) |
	       (tWTR << 5) | (tRTP << 2) | (tMRD << 0),
	       &mctl_phy->dtpr0);

	/* tDQSCK and tDQSCKmax are used LPDDR2/LPDDR3 only */
	writel((MCTL_TDQSCKMAX << 27) | (MCTL_TDQSCK << 24) |
	       (tRFC << 16) | (MCTL_TRTODT << 11) |
	       ((tMOD - 12) << 9) | (tFAW << 3) | (0 << 2) |
	       (MCTL_TAOND << 0), &mctl_phy->dtpr1);

	writel((tDLLK << 19) | (tCKE << 15) | (tXPDLL << 10) |
	       (MCTL_TEXSR << 0), &mctl_phy->dtpr2);

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
		return;
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

	/* Set number of clks per micro-second */
	writel(DRAM_CLK / 1000000, &mctl_ctl->togcnt1u);
	/* Set number of clks per 100 nano-seconds */
	writel(DRAM_CLK / 10000000, &mctl_ctl->togcnt100n);
	/* Set memory timing registers */
	writel(tREFI, &mctl_ctl->trefi);
	writel(tMRD, &mctl_ctl->tmrd);
	writel(tRFC, &mctl_ctl->trfc);
	writel((MCTL_TPREA << 16) | tRP, &mctl_ctl->trp);  /* TODO */
	writel(MCTL_TRTW, &mctl_ctl->trtw);
	writel(MCTL_TAL, &mctl_ctl->tal);
	writel(CL, &mctl_ctl->tcl);
	writel(CWL, &mctl_ctl->tcwl);
	writel(tRAS, &mctl_ctl->tras);
	writel(tRC, &mctl_ctl->trc);
	writel(tRCD, &mctl_ctl->trcd);
	writel(tRRD, &mctl_ctl->trrd);
	writel(tRTP, &mctl_ctl->trtp);
	writel(tWR, &mctl_ctl->twr);
	writel(tWTR, &mctl_ctl->twtr);
	writel(MCTL_TEXSR, &mctl_ctl->texsr);
	writel(tXP, &mctl_ctl->txp);
	writel(tXPDLL, &mctl_ctl->txpdll);
	writel(tZQCS, &mctl_ctl->tzqcs);
	writel(MCTL_TZQCSI, &mctl_ctl->tzqcsi);
	writel(MCTL_TDQS, &mctl_ctl->tdqs);
	writel(tCKSRE, &mctl_ctl->tcksre);
	writel(tCKSRX, &mctl_ctl->tcksrx);
	writel(tCKE, &mctl_ctl->tcke);
	writel(tMOD, &mctl_ctl->tmod);
	writel(MCTL_TRSTL, &mctl_ctl->trstl);
	writel(MCTL_TZQCL, &mctl_ctl->tzqcl);
	writel(MCTL_TMRR, &mctl_ctl->tmrr);
	writel(tCKESR, &mctl_ctl->tckesr);
	writel(MCTL_TDPD, &mctl_ctl->tdpd);

	/* Unknown magic performed by boot0 */
	setbits_le32(&mctl_ctl->dfiodtcfg, 1 << 3 | 1 << 14); /* rank0_odt_write_sel */
	clrbits_le32(&mctl_ctl->dfiodtcfg1, 0x1f);

	/* Select 16/32-bits mode for MCTL */
	if (para->bus_width == 16)
		setbits_le32(&mctl_ctl->ppcfg, 1);

	/* Set DFI timing registers */
	writel(CWL, &mctl_ctl->dfitphywrl);
	writel(CL - 1, &mctl_ctl->dfitrdden);
	writel(MCTL_DFITPHYRDL, &mctl_ctl->dfitphyrdl);
	writel(MCTL_DFISTCFG0, &mctl_ctl->dfistcfg0);

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

	struct dram_sun6i_cl_cwl_timing cl_cwl[] = {
		{ .CL =  5, .CWL = 5, .tCKmin = 3000, .tCKmax = 3300 },
		{ .CL =  6, .CWL = 5, .tCKmin = 2500, .tCKmax = 3300 },
		{ .CL =  8, .CWL = 6, .tCKmin = 1875, .tCKmax = 2500 },
		{ .CL = 10, .CWL = 7, .tCKmin = 1500, .tCKmax = 1875 },
		{ .CL = 11, .CWL = 8, .tCKmin = 1250, .tCKmax = 1500 }
	};

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

		/* CL/CWL table for the speed bin */
		.cl_cwl_table = cl_cwl,
		.cl_cwl_numentries = sizeof(cl_cwl) / sizeof(struct dram_sun6i_cl_cwl_timing),

		/* timings */
		.tREFI = 7800,   /* 7.8us (up to 85 degC) */
		.tRFC  = 260,    /* 260ns for 4GBit devices */ // 350ns @ 8GBit

		.tRCD  = 13750,
		.tRP   = 13750,
		.tRC   = 48750,
		.tRAS  = 35000,

		.tDLLK = 512,
		.tRTP  = { .ck = 4, .ps = 7500 },
		.tWTR  = { .ck = 4, .ps = 7500 },
		.tWR   = 15,
		.tMRD  = 4,
		.tMOD  = { .ck = 12, .ps = 15000 },
		.tCCD  = 4,
		.tRRD  = { .ck = 4, .ps = 7500 },
		.tFAW  = 40000,

		/* calibration timing */
		//		.tZQinit = { .ck = 512, .ps = 640000 },
		.tZQoper = { .ck = 256, .ps = 320000 },
		.tZQCS   = { .ck = 64,  .ps = 80000 },

		/* reset timing */
		//		.tXPR  = { .ck = 5, .ps = 10000 },

		/* self-refresh timings */
		.tXS  = { .ck = 5, .ps = 10000 },
		.tXSDLL = 512,
		.tCKSRE = { .ck = 5, .ps = 10000 },
		.tCKSRX = { .ck = 5, .ps = 10000 },

		/* power-down timings */
		.tXP = { .ck = 3, .ps = 6000 },
		.tXPDLL = { .ck = 10, .ps = 24000 },
		.tCKE = { .ck = 3, .ps = 5000 },

		/* write leveling timings */
		.tWLMRD = 40,
		.tWLO = 7500,
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

	mctl_channel_init(0, &para);
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
