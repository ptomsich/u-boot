/*
 * sun8i H3 platform dram controller init
 *
 * (C) Copyright 2007-2015 Allwinner Technology Co.
 *                         Jerry Wang <wangflord@allwinnertech.com>
 * (C) Copyright 2015      Vishnu Patekar <vishnupatekar0510@gmail.com>
 * (C) Copyright 2015      Hans de Goede <hdegoede@redhat.com>
 * (C) Copyright 2015      Jens Kuske <jenskuske@gmail.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#include <common.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/dram.h>
#include <linux/kconfig.h>
#include "dram-timings/timings.h"


struct dram_para {
	u16 page_size;
	u8 bus_width;
	u8 dual_rank;
	u8 row_bits;
#if defined(CONFIG_MACH_SUN8I_H3)
	const u8 dx_read_delays[4][11];
	const u8 dx_write_delays[4][11];
	const u8 ac_delays[31];
#else
	struct {
		const u8 clk;
		const u8 ca;
		const u8 cs[2];
		struct {
			const u8 dq_wr;
			const u8 dqs_wr;
			const u8 dq_rd;
			const u8 dqs_rd;
		} dx[4];
	} pcb_delays;
#endif
	const struct dram_bin* speed_bin;
};


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

static void mctl_phy_init(u32 val)
{
	struct sunxi_mctl_ctl_reg * const mctl_ctl =
			(struct sunxi_mctl_ctl_reg *)SUNXI_DRAM_CTL0_BASE;

	writel(val | PIR_INIT, &mctl_ctl->pir);
	mctl_await_completion(&mctl_ctl->pgsr[0], PGSR_INIT_DONE, 0x1);
}

#if defined(CONFIG_MACH_SUN8I_H3)
static void mctl_set_bit_delays(struct dram_para *para)
{
	struct sunxi_mctl_ctl_reg * const mctl_ctl =
			(struct sunxi_mctl_ctl_reg *)SUNXI_DRAM_CTL0_BASE;
	int i, j;

	clrbits_le32(&mctl_ctl->pgcr[0], 1 << 26);

	for (i = 0; i < 4; i++)
		for (j = 0; j < 11; j++)
			writel(DXBDLR_WRITE_DELAY(para->dx_write_delays[i][j]) |
			       DXBDLR_READ_DELAY(para->dx_read_delays[i][j]),
			       &mctl_ctl->dx[i].bdlr[j]);

	for (i = 0; i < 31; i++)
		writel(ACBDLR_WRITE_DELAY(para->ac_delays[i]),
		       &mctl_ctl->acbdlr[i]);

	setbits_le32(&mctl_ctl->pgcr[0], 1 << 26);
}
#elif defined(CONFIG_MACH_SUN50I)
static void mctl_set_bit_delays(struct dram_para *para)
{
	struct sunxi_mctl_ctl_reg *mctl_ctl =
	                (struct sunxi_mctl_ctl_reg*)SUNXI_DRAM_CTL0_BASE;

	/* These arrays describe the IC package delays, as retrieved from the
	   .rodata-section of libdram. */
	const u8 ic_ca_wr_delay[31] = {
		5, 5, 5, 2, 2, 5, 3, 3,	0, 3, 3, 3, 1, 0, 0, 0,
		3, 4, 0 ,3, 4, 1, 4, 0, 1, 1, 0, 1, 5, 5, 4
	};
	const u8 ic_dxN_wr_delay[4][11] = {
		[0] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 10 },
		[1] = { 0, 0, 0, 0, 1, 1, 1, 1, 0,  8,  8 },
		[2] = { 1, 0, 1, 1, 1, 1, 1, 1, 0, 11, 11 },
		[3] = { 1, 0, 0, 1, 1, 1, 1, 1, 0, 10, 10 }
	};
	const u8 ic_dxN_rd_delay[4][11] = {
		[0] = { 16, 16, 16, 16, 17, 16, 16, 17, 16,  1,  0 },
		[1] = { 17, 17, 17, 17, 17, 17, 17, 17, 17,  1,	 0 },
		[2] = { 16, 17, 17, 16, 16, 16, 16, 16, 16,  0,  0 },
		[3] = { 17, 17, 17, 17, 17, 17, 17, 17, 17,  1,  0 }
	};

	/* Assert the PHY FIFO reset */
	clrbits_le32(mctl_ctl->pgcr[0], (1 << 26));

	for (int dx = 0; dx < 4; ++dx)
	{
		for (int i = DXBDLR_DQ(0); i <= DXBDLR_DM; ++i)
		{
			unsigned wr_delay = ic_dxN_wr_delay[dx][i] + para->pcb_delays.dx[dx].dq_wr;
			unsigned rd_delay = ic_dxN_rd_delay[dx][i] + para->pcb_delays.dx[dx].dq_rd;
			unsigned val = ((wr_delay & 0xff) << 8) | (rd_delay & 0xff);

			writel(val, &mctl_ctl->dx[dx].iocr[i]);
		}

		for (int i = DXBDLR_DQS; i <= DXBDLR_DQSN; ++i)
		{
			unsigned wr_delay = ic_dxN_wr_delay[dx][i] + para->pcb_delays.dx[dx].dqs_wr;
			unsigned rd_delay = ic_dxN_rd_delay[dx][i] + para->pcb_delays.dx[dx].dqs_rd;
			unsigned val = ((wr_delay & 0xff) << 8) | (rd_delay & 0xff);

			unsigned reg;

			/* This looks wrong and I would rather expect
			   the SDLR6 register to need to be programmed
			   to the 'highest delay of DQS and DQSN', but
			   this (i.e. a bitwise OR) what was was used
			   in libdram. */
			reg = readl(&mctl_ctl->dx[dx].sdlr6);
			reg |= (val << 24);
			writel(reg, &mctl_ctl->dx[dx].sdlr6);

			writel(val, &mctl_ctl->dx[dx].iocr[i]);
		}
	}

	for (int i = 0; i < 31; i++)
	{
		unsigned delay = ic_ca_wr_delay[i];

		if (i == ACBLDR_CLK)
			delay += para->pcb_delays.clk;
		else if (i == ACBLDR_CS0)
			delay += para->pcb_delays.cs[0];
		else if (i == ACBLDR_CS1)
			delay += para->pcb_delays.cs[1];
		else if ((i >= ACBLDR_A(0)) && (i <= ACBLDR_A(15)))
			delay += para->pcb_delays.ca;

		if (delay > 0x3f)
			delay = 0x3f;  /* saturate to 0x3f */

		writel(delay << 8, &mctl_ctl->acbdlr[i]);
	}

	/* Release the PHY FIFO reset */
	setbits_le32(mctl_ctl->pgcr[0], (1 << 26));
}
#endif

#if defined(CONFIG_MACH_SUN50I)
enum {
	MBUS_PORT_CPU           = 0,
	MBUS_PORT_GPU           = 1,
	/* unused */
	MBUS_PORT_DMA           = 3,
	MBUS_PORT_VE            = 4,
	MBUS_PORT_CSI           = 5,
	MBUS_PORT_NAND          = 6,
	MBUS_PORT_SS            = 7,
	MBUS_PORT_TS            = 8,
	MBUS_PORT_DI            = 9,
	MBUS_PORT_DE            = 10,
	MBUS_PORT_DE_CFD        = 11,
};

enum {
	MBUS_QOS_LOWEST = 0,
	MBUS_QOS_LOW,
	MBUS_QOS_HIGH,
	MBUS_QOS_HIGHEST
};

inline void mbus_configure_port(u8 port,
				bool bwlimit,
				bool priority,
				u8 qos,         /* MBUS_QOS_LOWEST .. MBUS_QOS_HIGEST */
				u8 waittime,    /* 0 .. 0xf */
				u8 acs,         /* 0 .. 0xff */
				u16 bwl0,       /* 0 .. 0xffff, bandwidth limit in MB/s */
				u16 bwl1,
				u16 bwl2)
{
	struct sunxi_mctl_com_reg * const mctl_com =
			(struct sunxi_mctl_com_reg *)SUNXI_DRAM_COM_BASE;

	const u32 cfg0 = ( (bwlimit ? (1 << 0) : 0)
			   | (priority ? (1 << 1) : 0)
			   | ((qos & 0x3) << 2)
			   | ((waittime & 0xf) << 4)
			   | ((acs & 0xff) << 8)
			   | (bwl0 << 16) );
	const u32 cfg1 = ((u32)bwl2 << 16) | (bwl1 & 0xffff);

	printf("port %d cfg0 %08x cfg1 %08x\n", port, cfg0, cfg1);
	writel(cfg0, &mctl_com->mcr[port][0]);
	writel(cfg1, &mctl_com->mcr[port][1]);
}
#endif

static void mctl_set_master_priority(void)
{
	struct sunxi_mctl_com_reg * const mctl_com =
			(struct sunxi_mctl_com_reg *)SUNXI_DRAM_COM_BASE;

#if defined(CONFIG_MACH_SUN8I_H3)
	/* enable bandwidth limit windows and set windows size 1us */
	writel((1 << 16) | (400 < 0), &mctl_com->bwcr);

	/* set cpu high priority */
	writel(0x00000001, &mctl_com->mapr);

	writel(0x0200000d, &mctl_com->mcr[0][0]);
	writel(0x00800100, &mctl_com->mcr[0][1]);
	writel(0x06000009, &mctl_com->mcr[1][0]);
	writel(0x01000400, &mctl_com->mcr[1][1]);
	writel(0x0200000d, &mctl_com->mcr[2][0]);
	writel(0x00600100, &mctl_com->mcr[2][1]);
	writel(0x0100000d, &mctl_com->mcr[3][0]);
	writel(0x00200080, &mctl_com->mcr[3][1]);
	writel(0x07000009, &mctl_com->mcr[4][0]);
	writel(0x01000640, &mctl_com->mcr[4][1]);
	writel(0x0100000d, &mctl_com->mcr[5][0]);
	writel(0x00200080, &mctl_com->mcr[5][1]);
	writel(0x01000009, &mctl_com->mcr[6][0]);
	writel(0x00400080, &mctl_com->mcr[6][1]);
	writel(0x0100000d, &mctl_com->mcr[7][0]);
	writel(0x00400080, &mctl_com->mcr[7][1]);
	writel(0x0100000d, &mctl_com->mcr[8][0]);
	writel(0x00400080, &mctl_com->mcr[8][1]);
	writel(0x04000009, &mctl_com->mcr[9][0]);
	writel(0x00400100, &mctl_com->mcr[9][1]);
	writel(0x2000030d, &mctl_com->mcr[10][0]);
	writel(0x04001800, &mctl_com->mcr[10][1]);
	writel(0x04000009, &mctl_com->mcr[11][0]);
	writel(0x00400120, &mctl_com->mcr[11][1]);
#elif defined(CONFIG_MACH_SUN50I)
	/* enable bandwidth limit windows and set windows size 1us */
	writel(399, &mctl_com->tmr);
	writel((1 << 16), &mctl_com->bwcr);

	mbus_configure_port(MBUS_PORT_CPU,    true,  false, MBUS_QOS_HIGHEST, 0, 0,  160,  100,   80);
	mbus_configure_port(MBUS_PORT_GPU,    false, false, MBUS_QOS_HIGH,    0, 0, 1536, 1400,  256);

	/* Port 2 is reserved per Allwinner's linux-3.10 source, yet they initialise it */
	mbus_configure_port(2,                true,  false, MBUS_QOS_HIGHEST, 0, 0,  512,  256,   96);

	mbus_configure_port(MBUS_PORT_DMA,    true,  false, MBUS_QOS_HIGH,    0, 0,  256,   80,  100);
	mbus_configure_port(MBUS_PORT_VE,     true,  false, MBUS_QOS_HIGH,    0, 0, 1792, 1600,  256);
	mbus_configure_port(MBUS_PORT_CSI,    true,  false, MBUS_QOS_HIGH,    0, 0,  256,  128,    0);
	mbus_configure_port(MBUS_PORT_NAND,   true,  false, MBUS_QOS_HIGH,    0, 0,  256,  128,   64);
	mbus_configure_port(MBUS_PORT_SS,     true,  false, MBUS_QOS_HIGHEST, 0, 0,  256,  128,   64);
	mbus_configure_port(MBUS_PORT_TS,     true,  false, MBUS_QOS_HIGHEST, 0, 0,  256,  128,   64);
	mbus_configure_port(MBUS_PORT_DI,     true,  false, MBUS_QOS_HIGH,    0, 0, 1024,  256,   64);
	mbus_configure_port(MBUS_PORT_DE,     true,  false, MBUS_QOS_HIGH,    0, 2, 8192, 6144, 2048);
	mbus_configure_port(MBUS_PORT_DE_CFD, true,  false, MBUS_QOS_HIGH,    0, 0, 1280,  144,   64);

	writel(0x81000004, &mctl_com->mdfs_bwlr[2]);
#endif
}

static int mctl_set_timing_params(const struct dram_bin * const speed_bin)
{
	struct sunxi_mctl_ctl_reg * const mctl_ctl =
			(struct sunxi_mctl_ctl_reg *)SUNXI_DRAM_CTL0_BASE;

	u8 trcd		= ps_roundup_t(speed_bin->tRCD);
	u8 trc		= ps_roundup_t(speed_bin->tRC);
	u8 trp		= ps_roundup_t(speed_bin->tRP);
	u8 tras		= ps_roundup_t(speed_bin->tRAS);
	u16 trefi	= ns_floor_t(speed_bin->tREFI);
	u16 trfc	= ns_roundup_t(speed_bin->tRFC);

	/* command and address timings */
	u8 tccd		= speed_bin->tCCD;
	u8 tfaw		= ps_roundup_t(speed_bin->tFAW);
	u8 tmod		= timing_to_t(&speed_bin->tMOD);
	u8 tmrd		= speed_bin->tMRD;
	u8 tmrw		= 0;  /* unused for DDR3 */
	u8 trrd		= timing_to_t(&speed_bin->tRRD);
	u8 trtp		= timing_to_t(&speed_bin->tRTP);
	u8 twr		= ns_floor_t(speed_bin->tWR);
	u8 twtr		= timing_to_t(&speed_bin->tWTR);
	
	/* power-down timings */
	u8 txp		= timing_to_t(&speed_bin->tXP);
	u8 tcke		= timing_to_t(&speed_bin->tCKE);

	/* self refresh timings */
	u8 tcksrx	= timing_to_t(&speed_bin->tXS);
	u8 tcksre	= timing_to_t(&speed_bin->tCKSRE);
	u8 tckesr	= tcke + 1;

	u8 trasmax	= trefi * 9;

	u8 t_rdata_en	= 4;
	u8 wr_latency	= 2;

	u32 tdinit0	= (500 * CONFIG_DRAM_CLK);		/* 500us */
	u32 tdinit1	= (360 * CONFIG_DRAM_CLK + 999) / 1000;	/* 360ns */
	u32 tdinit2	= (200 * CONFIG_DRAM_CLK);		/* 200us */
	u32 tdinit3	= CONFIG_DRAM_CLK;		        /* 1us */

	/* derived timings, using the same terminology as Designware */
	u8 wr2pre;
	u8 wr2rd;
	u8 rd2wr;

	u8 CL = 0, CWL = 0;

	if (dram_calculate_CL_CWL(speed_bin, CONFIG_DRAM_CLK, &CL, &CWL))
	    return 1;

	wr2pre = CWL + (MCTL_BL/2) + twr;
	wr2rd  = CWL + (MCTL_BL/2) + twtr;
	rd2wr  = CL + (MCTL_BL/2) + 2 - CWL;

	/* set mode register */
	writel(DDR3_MR0_PPD_FAST_EXIT | DDR3_MR0_WR(twr) | DDR3_MR0_CL(CL), &mctl_ctl->mr[0]);
	writel(DDR3_MR1_RTT120OHM, &mctl_ctl->mr[1]);
	writel(DDR3_MR2_TWL(CWL), &mctl_ctl->mr[2]);
	writel(2, &mctl_ctl->mr[3]);

	/* set DRAM timing */
	writel(DRAMTMG0_TWR2PRE(wr2pre) | DRAMTMG0_TFAW(tfaw) |
	       DRAMTMG0_TRAS_MAX(trasmax) | DRAMTMG0_TRAS(tras),
	       &mctl_ctl->dramtmg[0]);
	writel(DRAMTMG1_TXP(txp) | DRAMTMG1_TRTP(trtp) | DRAMTMG1_TRC(trc),
	       &mctl_ctl->dramtmg[1]);
	writel(DRAMTMG2_TCWL(CWL) | DRAMTMG2_TCL(CL) |
	       DRAMTMG2_TRD2WR(rd2wr) | DRAMTMG2_TWR2RD(wr2rd),
	       &mctl_ctl->dramtmg[2]);
	writel(DRAMTMG3_TMRW(tmrw) | DRAMTMG3_TMRD(tmrd) | DRAMTMG3_TMOD(tmod),
	       &mctl_ctl->dramtmg[3]);
	writel(DRAMTMG4_TRCD(trcd) | DRAMTMG4_TCCD(tccd) | DRAMTMG4_TRRD(trrd) |
	       DRAMTMG4_TRP(trp), &mctl_ctl->dramtmg[4]);
	writel(DRAMTMG5_TCKSRX(tcksrx) | DRAMTMG5_TCKSRE(tcksre) |
	       DRAMTMG5_TCKESR(tckesr) | DRAMTMG5_TCKE(tcke),
	       &mctl_ctl->dramtmg[5]);

	/* set two rank timing */
	clrsetbits_le32(&mctl_ctl->dramtmg[8], (0xff << 8) | (0xff << 0),
			(0x66 << 8) | (0x10 << 0));

	/* set PHY interface timing, write latency and read latency configure */
	writel((0x2 << 24) | (t_rdata_en << 16) | (0x1 << 8) |
	       (wr_latency << 0), &mctl_ctl->pitmg[0]);

	/* set PHY timing, PTR0-2 use default */
	writel(PTR3_TDINIT0(tdinit0) | PTR3_TDINIT1(tdinit1), &mctl_ctl->ptr[3]);
	writel(PTR4_TDINIT2(tdinit2) | PTR4_TDINIT3(tdinit3), &mctl_ctl->ptr[4]);

	/* set refresh timing */
	writel(RFSHTMG_TREFI(trefi) | RFSHTMG_TRFC(trfc), &mctl_ctl->rfshtmg);

	return 0;
}

#ifdef CONFIG_MACH_SUN8I_H3
static u32 bin_to_mgray(int val)
{
	static const u8 lookup_table[32] = {
		0x00, 0x01, 0x02, 0x03, 0x06, 0x07, 0x04, 0x05,
		0x0c, 0x0d, 0x0e, 0x0f, 0x0a, 0x0b, 0x08, 0x09,
		0x18, 0x19, 0x1a, 0x1b, 0x1e, 0x1f, 0x1c, 0x1d,
		0x14, 0x15, 0x16, 0x17, 0x12, 0x13, 0x10, 0x11,
	};

	return lookup_table[clamp(val, 0, 31)];
}

static int mgray_to_bin(u32 val)
{
	static const u8 lookup_table[32] = {
		0x00, 0x01, 0x02, 0x03, 0x06, 0x07, 0x04, 0x05,
		0x0e, 0x0f, 0x0c, 0x0d, 0x08, 0x09, 0x0a, 0x0b,
		0x1e, 0x1f, 0x1c, 0x1d, 0x18, 0x19, 0x1a, 0x1b,
		0x10, 0x11, 0x12, 0x13, 0x16, 0x17, 0x14, 0x15,
	};

	return lookup_table[val & 0x1f];
}

static void mctl_h3_zq_calibration_quirk(struct dram_para *para)
{
	struct sunxi_mctl_ctl_reg * const mctl_ctl =
			(struct sunxi_mctl_ctl_reg *)SUNXI_DRAM_CTL0_BASE;

	int i;
	u16 zq_val[6];
	u8 val;

	writel(0x0a0a0a0a, &mctl_ctl->zqdr[2]);

	for (i = 0; i < 6; i++) {
		u8 zq = (CONFIG_DRAM_ZQ >> (i * 4)) & 0xf;

		writel((zq << 20) | (zq << 16) | (zq << 12) |
				(zq << 8) | (zq << 4) | (zq << 0),
				&mctl_ctl->zqcr);

		writel(PIR_CLRSR, &mctl_ctl->pir);
		mctl_phy_init(PIR_ZCAL);

		zq_val[i] = readl(&mctl_ctl->zqdr[0]) & 0xff;
		writel(REPEAT_BYTE(zq_val[i]), &mctl_ctl->zqdr[2]);

		writel(PIR_CLRSR, &mctl_ctl->pir);
		mctl_phy_init(PIR_ZCAL);

		val = readl(&mctl_ctl->zqdr[0]) >> 24;
		zq_val[i] |= bin_to_mgray(mgray_to_bin(val) - 1) << 8;
	}

	writel((zq_val[1] << 16) | zq_val[0], &mctl_ctl->zqdr[0]);
	writel((zq_val[3] << 16) | zq_val[2], &mctl_ctl->zqdr[1]);
	writel((zq_val[5] << 16) | zq_val[4], &mctl_ctl->zqdr[2]);
}
#endif

static void mctl_set_cr(struct dram_para *para)
{
	struct sunxi_mctl_com_reg * const mctl_com =
			(struct sunxi_mctl_com_reg *)SUNXI_DRAM_COM_BASE;

	writel(MCTL_CR_BL8 | MCTL_CR_2T | MCTL_CR_DDR3 | MCTL_CR_INTERLEAVED |
	       MCTL_CR_EIGHT_BANKS | MCTL_CR_BUS_WIDTH(para->bus_width) |
	       (para->dual_rank ? MCTL_CR_DUAL_RANK : MCTL_CR_SINGLE_RANK) |
	       MCTL_CR_PAGE_SIZE(para->page_size) |
	       MCTL_CR_ROW_BITS(para->row_bits), &mctl_com->cr);
}

static void mctl_sys_init(struct dram_para *para)
{
	struct sunxi_ccm_reg * const ccm =
			(struct sunxi_ccm_reg *)SUNXI_CCM_BASE;
	struct sunxi_mctl_ctl_reg * const mctl_ctl =
			(struct sunxi_mctl_ctl_reg *)SUNXI_DRAM_CTL0_BASE;

	clrbits_le32(&ccm->mbus0_clk_cfg, MBUS_CLK_GATE);
	clrbits_le32(&ccm->mbus_reset, CCM_MBUS_RESET_RESET);
	clrbits_le32(&ccm->ahb_gate0, 1 << AHB_GATE_OFFSET_MCTL);
	clrbits_le32(&ccm->ahb_reset0_cfg, 1 << AHB_RESET_OFFSET_MCTL);
	clrbits_le32(&ccm->pll5_cfg, CCM_PLL5_CTRL_EN);
#ifdef CONFIG_MACH_SUN50I
	clrbits_le32(&ccm->pll11_cfg, CCM_PLL11_CTRL_EN);
#endif
	udelay(10);

	clrbits_le32(&ccm->dram_clk_cfg, CCM_DRAMCLK_CFG_RST);
	udelay(1000);

#ifdef CONFIG_MACH_SUN50I
	clock_set_pll11(CONFIG_DRAM_CLK * 2 * 1000000, false);
	clrsetbits_le32(&ccm->dram_clk_cfg,
			CCM_DRAMCLK_CFG_DIV_MASK | CCM_DRAMCLK_CFG_SRC_MASK,
			CCM_DRAMCLK_CFG_DIV(1) | CCM_DRAMCLK_CFG_SRC_PLL11 |
			CCM_DRAMCLK_CFG_UPD);
#else
	clock_set_pll5(CONFIG_DRAM_CLK * 2 * 1000000, false);
	clrsetbits_le32(&ccm->dram_clk_cfg,
			CCM_DRAMCLK_CFG_DIV_MASK | CCM_DRAMCLK_CFG_SRC_MASK,
			CCM_DRAMCLK_CFG_DIV(1) | CCM_DRAMCLK_CFG_SRC_PLL5 |
			CCM_DRAMCLK_CFG_UPD);
#endif
	mctl_await_completion(&ccm->dram_clk_cfg, CCM_DRAMCLK_CFG_UPD, 0);

	setbits_le32(&ccm->ahb_reset0_cfg, 1 << AHB_RESET_OFFSET_MCTL);
	setbits_le32(&ccm->ahb_gate0, 1 << AHB_GATE_OFFSET_MCTL);
	setbits_le32(&ccm->mbus_reset, CCM_MBUS_RESET_RESET);
	setbits_le32(&ccm->mbus0_clk_cfg, MBUS_CLK_GATE);

	setbits_le32(&ccm->dram_clk_cfg, CCM_DRAMCLK_CFG_RST);
	udelay(10);

	writel(0xc00e, &mctl_ctl->clken);
	udelay(500);
}

static int mctl_channel_init(struct dram_para *para)
{
	struct sunxi_mctl_com_reg * const mctl_com =
			(struct sunxi_mctl_com_reg *)SUNXI_DRAM_COM_BASE;
	struct sunxi_mctl_ctl_reg * const mctl_ctl =
			(struct sunxi_mctl_ctl_reg *)SUNXI_DRAM_CTL0_BASE;

	unsigned int i;
	unsigned int ret;

	mctl_set_cr(para);
	ret = mctl_set_timing_params(para->speed_bin);
	if (ret)
		return ret;
	mctl_set_master_priority();

	/* setting VTC, default disable all VT */
	clrbits_le32(&mctl_ctl->pgcr[0], (1 << 30) | 0x3f);
	clrsetbits_le32(&mctl_ctl->pgcr[1], 1 << 24, 1 << 26);

	/* increase DFI_PHY_UPD clock */
	writel(PROTECT_MAGIC, &mctl_com->protect);
	udelay(100);
	clrsetbits_le32(&mctl_ctl->upd2, 0xfff << 16, 0x50 << 16);
	writel(0x0, &mctl_com->protect);
	udelay(100);

	/* set dramc odt */
	for (i = 0; i < 4; i++)
		clrsetbits_le32(&mctl_ctl->dx[i].gcr, (0x3 << 4) |
				(0x1 << 1) | (0x3 << 2) | (0x3 << 12) |
				(0x3 << 14),
				IS_ENABLED(CONFIG_DRAM_ODT_EN) ? 0x0 : 0x2);

	/* AC PDR should always ON */
	setbits_le32(&mctl_ctl->aciocr, 0x1 << 1);

	/* set DQS auto gating PD mode */
	setbits_le32(&mctl_ctl->pgcr[2], 0x3 << 6);

#if defined(CONFIG_MACH_SUN8I_H3)
	/* dx ddr_clk & hdr_clk dynamic mode */
	clrbits_le32(&mctl_ctl->pgcr[0], (0x3 << 14) | (0x3 << 12));

	/* dphy & aphy phase select 270 degree */
	clrsetbits_le32(&mctl_ctl->pgcr[2], (0x3 << 10) | (0x3 << 8),
			(0x1 << 10) | (0x2 << 8));
#elif defined(CONFIG_MACH_SUN50I)
	/* dphy & aphy phase select ? */
	clrsetbits_le32(&mctl_ctl->pgcr[2], (0x3 << 10) | (0x3 << 8) |
	               (0x3 << 12), (0x0 << 10) | (0x3 << 8));
#endif

	/* set half DQ */
	if (para->bus_width != 32) {
		writel(0x0, &mctl_ctl->dx[2].gcr);
		writel(0x0, &mctl_ctl->dx[3].gcr);
	}

	/* data training configuration */
	clrsetbits_le32(&mctl_ctl->dtcr, 0xf << 24,
			(para->dual_rank ? 0x3 : 0x1) << 24);

	mctl_set_bit_delays(para);
	udelay(50);

#ifdef CONFIG_MACH_SUN8I_H3
	mctl_h3_zq_calibration_quirk(para);

	mctl_phy_init(PIR_PLLINIT | PIR_DCAL | PIR_PHYRST | PIR_DRAMRST |
		      PIR_DRAMINIT | PIR_QSGATE);
#else
	clrsetbits_le32(&mctl_ctl->zqcr, 0xffffff, CONFIG_DRAM_ZQ);

	mctl_phy_init(PIR_ZCAL | PIR_PLLINIT | PIR_DCAL | PIR_PHYRST |
		      PIR_DRAMRST | PIR_DRAMINIT | PIR_QSGATE);
#endif

	/* detect ranks and bus width */
	if (readl(&mctl_ctl->pgsr[0]) & (0xfe << 20)) {
		/* only one rank */
		if (((readl(&mctl_ctl->dx[0].gsr[0]) >> 24) & 0x2) ||
		    ((readl(&mctl_ctl->dx[1].gsr[0]) >> 24) & 0x2)) {
			clrsetbits_le32(&mctl_ctl->dtcr, 0xf << 24, 0x1 << 24);
			para->dual_rank = 0;
		}

		/* only half DQ width */
		if (((readl(&mctl_ctl->dx[2].gsr[0]) >> 24) & 0x1) ||
		    ((readl(&mctl_ctl->dx[3].gsr[0]) >> 24) & 0x1)) {
			writel(0x0, &mctl_ctl->dx[2].gcr);
			writel(0x0, &mctl_ctl->dx[3].gcr);
			para->bus_width = 16;
		}

		mctl_set_cr(para);
		udelay(20);

		/* re-train */
		mctl_phy_init(PIR_QSGATE);
		if (readl(&mctl_ctl->pgsr[0]) & (0xfe << 20))
			return 1;
	}

	/* check the dramc status */
	mctl_await_completion(&mctl_ctl->statr, 0x1, 0x1);

	/* liuke added for refresh debug */
	setbits_le32(&mctl_ctl->rfshctl0, 0x1 << 31);
	udelay(10);
	clrbits_le32(&mctl_ctl->rfshctl0, 0x1 << 31);
	udelay(10);

	/* set PGCR3, CKE polarity */
#ifdef CONFIG_MACH_SUN50I
	writel(0xc0aa0060, &mctl_ctl->pgcr[3]);
#else
	writel(0x00aa0060, &mctl_ctl->pgcr[3]);
#endif

	/* power down zq calibration module for power save */
	setbits_le32(&mctl_ctl->zqcr, ZQCR_PWRDOWN);

	/* enable master access */
	writel(0xffffffff, &mctl_com->maer);

	return 0;
}

static void mctl_auto_detect_dram_size(struct dram_para *para)
{
	/* detect row address bits */
	para->page_size = 512;
	para->row_bits = 16;
	mctl_set_cr(para);

	for (para->row_bits = 11; para->row_bits < 16; para->row_bits++)
		if (mctl_mem_matches((1 << (para->row_bits + 3)) * para->page_size))
			break;

	/* detect page size */
	para->page_size = 8192;
	mctl_set_cr(para);

	for (para->page_size = 512; para->page_size < 8192; para->page_size *= 2)
		if (mctl_mem_matches(para->page_size))
			break;
}

unsigned long sunxi_dram_init(void)
{
	struct sunxi_mctl_com_reg * const mctl_com =
			(struct sunxi_mctl_com_reg *)SUNXI_DRAM_COM_BASE;
	struct sunxi_mctl_ctl_reg * const mctl_ctl =
			(struct sunxi_mctl_ctl_reg *)SUNXI_DRAM_CTL0_BASE;

	struct dram_para para = {
		.dual_rank = 0,
		.bus_width = 32,
		.row_bits = 15,
		.page_size = 4096,

#if defined(CONFIG_MACH_SUN8I_H3)
		.dx_read_delays =  {{ 18, 18, 18, 18, 18, 18, 18, 18, 18,  0,  0 },
		                    { 14, 14, 14, 14, 14, 14, 14, 14, 14,  0,  0 },
		                    { 18, 18, 18, 18, 18, 18, 18, 18, 18,  0,  0 },
		                    { 14, 14, 14, 14, 14, 14, 14, 14, 14,  0,  0 }},
		.dx_write_delays = {{  0,  0,  0,  0,  0,  0,  0,  0,  0, 10, 10 },
		                    {  0,  0,  0,  0,  0,  0,  0,  0,  0, 10, 10 },
		                    {  0,  0,  0,  0,  0,  0,  0,  0,  0, 10, 10 },
		                    {  0,  0,  0,  0,  0,  0,  0,  0,  0,  6,  6 }},
		.ac_delays = {  0,  0,  0,  0,  0,  0,  0,  0,
		                0,  0,  0,  0,  0,  0,  0,  0,
		                0,  0,  0,  0,  0,  0,  0,  0,
		                0,  0,  0,  0,  0,  0,  0      },
#elif defined(CONFIG_MACH_SUN50I)
		.pcb_delays = {
			.clk    = 8,
			.ca     = 0,
			.cs     = { 8, 8 },
			.dx[0] = { 0, 5, 0, 0 },
			.dx[1] = { 0, 2, 0, 0 },
			.dx[2] = { 0, 0, 0, 0 },
			.dx[3] = { 0, 2, 0, 0 },
		},
#endif
		.speed_bin = &DDR3_1600K,
	};

	mctl_sys_init(&para);
	if (mctl_channel_init(&para))
		return 0;

	if (para.dual_rank)
		writel(0x00000303, &mctl_ctl->odtmap);
	else
		writel(0x00000201, &mctl_ctl->odtmap);
	udelay(1);

#ifdef CONFIG_MACH_SUN8I_H3
	/* odt delay */
	writel(0x0c000400, &mctl_ctl->odtcfg);
#endif

#ifdef CONFIG_MACH_SUN50I
	setbits_le32(&mctl_ctl->vtfcr, (1 << 9));
#endif

	/* clear credit value */
	setbits_le32(&mctl_com->cccr, 1 << 31);
	udelay(10);

	mctl_auto_detect_dram_size(&para);
	mctl_set_cr(&para);

	printf("row bits: %d\n", para.row_bits);
	printf("page size: %d\n", para.page_size);
	printf("bus width: %d\n", para.bus_width);
	
	return (1 << (para.row_bits + 3)) * para.page_size *
						(para.dual_rank ? 2 : 1);
}
