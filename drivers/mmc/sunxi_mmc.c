/*
 * (C) Copyright 2007-2011
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 * Aaron <leafy.myeh@allwinnertech.com>
 *
 * (C) 2017 Theobroma Systems Design und Consulting GmbH
 *
 * MMC driver for allwinner sunxi platform.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm-generic/gpio.h>
#include <asm/arch/clock.h>
#include <asm/arch/cpu.h>
#include <asm/arch/gpio.h>
#include <asm/arch/mmc.h>
#include <asm/io.h>
#include <clk.h>
#include <dm/device.h>
#include <dt-structs.h>
#include <errno.h>
#include <linux/iopoll.h>
#include <malloc.h>
#include <mmc.h>
#include <reset.h>

DECLARE_GLOBAL_DATA_PTR;

struct sunxi_mmc_plat {
	struct mmc mmc;
};

struct sunxi_mmc_host {
	unsigned mmc_no;
#if !defined(CONFIG_DM_MMC)
	uint32_t *mclkreg;
#endif
	unsigned fatal_err;
	struct sunxi_mmc *reg;
	struct mmc_config cfg;
	bool cd_inverted;
#if defined(CONFIG_DM_MMC)
	struct mmc *mmc;
	struct gpio_desc cd_gpio;	/* card-detect (optional) */
	struct gpio_desc pwr_gpio;	/* power-enabled (optional) */
	struct gpio_desc wp_gpio;	/* write-protect (optional) */
	bool wp_inverted;
	struct reset_ctl reset;
	struct clk ahb_clk_gate;
	struct clk mmc_clk;
#else
	int cd_pin;
#endif
};

#if defined(CONFIG_DM_MMC) && defined(CONFIG_DM_MMC_OPS)
static const struct dm_mmc_ops sunxi_mmc_ops;
#else
static const struct mmc_ops sunxi_mmc_ops;
#endif

#if !defined(CONFIG_DM_MMC)
/* support 4 mmc hosts */
struct sunxi_mmc_host mmc_host[4];

static int sunxi_mmc_getcd_gpio(int sdc_no)
{
	switch (sdc_no) {
	case 0: return sunxi_name_to_gpio(CONFIG_MMC0_CD_PIN);
	case 1: return sunxi_name_to_gpio(CONFIG_MMC1_CD_PIN);
	case 2: return sunxi_name_to_gpio(CONFIG_MMC2_CD_PIN);
#if !defined(CONFIG_ARCH_SUN50I)  /* only 3 MMC controllers on the A64 */
	case 3: return sunxi_name_to_gpio(CONFIG_MMC3_CD_PIN);
#endif
	}
	return -EINVAL;
}

static int mmc_resource_init(int sdc_no)
{
	struct sunxi_mmc_host *mmchost = &mmc_host[sdc_no];
	struct sunxi_ccm_reg *ccm = (struct sunxi_ccm_reg *)SUNXI_CCM_BASE;
	int cd_pin, ret = 0;

	debug("init mmc %d resource\n", sdc_no);

	switch (sdc_no) {
	case 0:
		mmchost->reg = (struct sunxi_mmc *)SUNXI_MMC0_BASE;
		mmchost->mclkreg = &ccm->sd0_clk_cfg;
		break;
	case 1:
		mmchost->reg = (struct sunxi_mmc *)SUNXI_MMC1_BASE;
		mmchost->mclkreg = &ccm->sd1_clk_cfg;
		break;
	case 2:
		mmchost->reg = (struct sunxi_mmc *)SUNXI_MMC2_BASE;
		mmchost->mclkreg = &ccm->sd2_clk_cfg;
		break;
#if !defined(CONFIG_ARCH_SUN50I)  /* only 3 MMC controllers on the A64 */
	case 3:
		mmchost->reg = (struct sunxi_mmc *)SUNXI_MMC3_BASE;
		mmchost->mclkreg = &ccm->sd3_clk_cfg;
		break;
#endif
	default:
		printf("Wrong mmc number %d\n", sdc_no);
		return -1;
	}
	mmchost->mmc_no = sdc_no;

	cd_pin = sunxi_mmc_getcd_gpio(sdc_no);
	if (cd_pin >= 0) {
		ret = gpio_request(cd_pin, "mmc_cd");
		if (!ret) {
			sunxi_gpio_set_pull(cd_pin, SUNXI_GPIO_PULL_UP);
			ret = gpio_direction_input(cd_pin);
		}
	}
	mmchost->cd_pin = cd_pin;

	return ret;
}
#endif

#if defined(CONFIG_DM_MMC)
static int mmc_resource_init_from_udev(struct udevice *dev)
{
	struct sunxi_mmc_host *mmchost = dev_get_priv(dev);
	int ret = 0;

	debug("%s: %s\n", dev->name, __func__);

	switch ((uintptr_t)mmchost->reg) {
	case SUNXI_MMC0_BASE:
		mmchost->mmc_no = 0;
		break;
	case SUNXI_MMC1_BASE:
		mmchost->mmc_no = 1;
		break;
	case SUNXI_MMC2_BASE:
		mmchost->mmc_no = 2;
		break;
#if !defined(CONFIG_ARCH_SUN50I)  /* only 3 MMC controllers on the A64 */
	case SUNXI_MMC3_BASE:
		mmchost->mmc_no = 3;
		break;
#endif
	default:
		debug("%s: unknown base address %p\n", __func__, mmchost->reg);
		return -1;
	}

	debug("%s: mmc_no %d\n", dev->name, mmchost->mmc_no);

	return ret;
}
#endif

#if !defined(CONFIG_DM_MMC)
static int mmc_set_mod_clk(struct sunxi_mmc_host *mmchost, unsigned int hz)
{
	unsigned int pll, pll_hz, div, n, oclk_dly, sclk_dly;

	if (hz <= 24000000) {
		pll = CCM_MMC_CTRL_OSCM24;
		pll_hz = 24000000;
	} else {
#ifdef CONFIG_MACH_SUN9I
		pll = CCM_MMC_CTRL_PLL_PERIPH0;
		pll_hz = clock_get_pll4_periph0();
#else
		pll = CCM_MMC_CTRL_PLL6;
		pll_hz = clock_get_pll6();
#endif
	}

	div = pll_hz / hz;
	if (pll_hz % hz)
		div++;

	n = 0;
	while (div > 16) {
		n++;
		div = (div + 1) / 2;
	}

	if (n > 3) {
		printf("mmc %u error cannot set clock to %u\n",
		       mmchost->mmc_no, hz);
		return -1;
	}

	/* determine delays */
	if (hz <= 400000) {
		oclk_dly = 0;
		sclk_dly = 0;
	} else if (hz <= 25000000) {
		oclk_dly = 0;
		sclk_dly = 5;
#ifdef CONFIG_MACH_SUN9I
	} else if (hz <= 50000000) {
		oclk_dly = 5;
		sclk_dly = 4;
	} else {
		/* hz > 50000000 */
		oclk_dly = 2;
		sclk_dly = 4;
#else
	} else if (hz <= 50000000) {
		oclk_dly = 3;
		sclk_dly = 4;
	} else {
		/* hz > 50000000 */
		oclk_dly = 1;
		sclk_dly = 4;
#endif
	}

	writel(CCM_MMC_CTRL_ENABLE | pll | CCM_MMC_CTRL_SCLK_DLY(sclk_dly) |
	       CCM_MMC_CTRL_N(n) | CCM_MMC_CTRL_OCLK_DLY(oclk_dly) |
	       CCM_MMC_CTRL_M(div), mmchost->mclkreg);

	debug("mmc %u set mod-clk req %u parent %u n %u m %u rate %u\n",
	      mmchost->mmc_no, hz, pll_hz, 1u << n, div,
	      pll_hz / (1u << n) / div);

	return 0;
}

static int mmc_clk_io_on(struct sunxi_mmc_host *mmchost)
{
	struct sunxi_ccm_reg *ccm = (struct sunxi_ccm_reg *)SUNXI_CCM_BASE;
	int sdc_no = mmchost->mmc_no;

	debug("init mmc %d clock and io\n", sdc_no);

	/* config ahb clock */
	setbits_le32(&ccm->ahb_gate0, 1 << AHB_GATE_OFFSET_MMC(sdc_no));

#ifdef CONFIG_SUNXI_GEN_SUN6I
	/* unassert reset */
	setbits_le32(&ccm->ahb_reset0_cfg, 1 << AHB_RESET_OFFSET_MMC(sdc_no));
#endif
#if defined(CONFIG_MACH_SUN9I)
	/* sun9i has a mmc-common module, also set the gate and reset there */
	writel(SUNXI_MMC_COMMON_CLK_GATE | SUNXI_MMC_COMMON_RESET,
	       SUNXI_MMC_COMMON_BASE + 4 * sdc_no);
#endif

	return mmc_set_mod_clk(mmchost, 24000000);
}
#endif

#if defined(CONFIG_DM_MMC)
static int mmc_clk_io_on(struct sunxi_mmc_host *mmchost)
{
	/* Enable the AHB clock gate */
	clk_enable(&mmchost->ahb_clk_gate);

	/* Deassert the AHB module reset */
	reset_deassert(&mmchost->reset);

#if defined(CONFIG_MACH_SUN9I)
	/* TODO --- covert this to DM */
	/* sun9i has a mmc-common module, also set the gate and reset there */
	writel(SUNXI_MMC_COMMON_CLK_GATE | SUNXI_MMC_COMMON_RESET,
	       SUNXI_MMC_COMMON_BASE + 4 * sdc_no);
#endif

	clk_set_rate(&mmchost->mmc_clk, 24000000);
	clk_enable(&mmchost->mmc_clk);

	return 0;
}
#endif

static int mmc_update_clk(struct mmc *mmc)
{
	struct sunxi_mmc_host *mmchost = mmc->priv;
	unsigned int cmd;
	unsigned timeout_msecs = 2000;

	debug("%s: base %p\n", __func__, mmchost->reg);

	cmd = SUNXI_MMC_CMD_START |
	      SUNXI_MMC_CMD_UPCLK_ONLY |
	      SUNXI_MMC_CMD_WAIT_PRE_OVER;
	writel(cmd, &mmchost->reg->cmd);
	while (readl(&mmchost->reg->cmd) & SUNXI_MMC_CMD_START) {
		if (!timeout_msecs--)
			return -1;
		udelay(1000);
	}

	/* clock update sets various irq status bits, clear these */
	writel(readl(&mmchost->reg->rint), &mmchost->reg->rint);

	return 0;
}

static int mmc_config_clock(struct mmc *mmc)
{
	struct sunxi_mmc_host *mmchost = mmc->priv;
	unsigned rval = readl(&mmchost->reg->clkcr);

	/* Disable Clock */
	rval &= ~SUNXI_MMC_CLK_ENABLE;
	writel(rval, &mmchost->reg->clkcr);
	if (mmc_update_clk(mmc))
		return -1;

#if !defined(CONFIG_DM_MMC)
	/* Set mod_clk to new rate */
	if (mmc_set_mod_clk(mmchost, mmc->clock))
		return -1;
#else
	if (clk_set_rate(&mmchost->mmc_clk, mmc->clock) == 0)
		return -1;
#endif

	/* Clear internal divider */
	rval &= ~SUNXI_MMC_CLK_DIVIDER_MASK;
	writel(rval, &mmchost->reg->clkcr);

	/* Re-enable Clock */
	rval |= SUNXI_MMC_CLK_ENABLE;
	writel(rval, &mmchost->reg->clkcr);
	if (mmc_update_clk(mmc))
		return -1;

	return 0;
}

#if defined(CONFIG_DM_MMC_OPS)
static int sunxi_mmc_set_ios(struct udevice *dev)
{
	struct mmc *mmc = mmc_get_mmc_dev(dev);
#else
static int sunxi_mmc_set_ios(struct mmc *mmc)
{
#endif
	struct sunxi_mmc_host *mmchost = mmc->priv;

	debug("set ios: bus_width: %x, clock: %d\n",
	      mmc->bus_width, mmc->clock);

	/* Change clock first */
	if (mmc->clock && mmc_config_clock(mmc) != 0) {
		mmchost->fatal_err = 1;
		return -EINVAL;
	}

	/* Change bus width */
	if (mmc->bus_width == 8)
		writel(0x2, &mmchost->reg->width);
	else if (mmc->bus_width == 4)
		writel(0x1, &mmchost->reg->width);
	else
		writel(0x0, &mmchost->reg->width);

	return 0;
}

static int sunxi_mmc_core_init(struct mmc *mmc)
{
	struct sunxi_mmc_host *mmchost = mmc->priv;
	uint32_t regval;
	int ret = 0;

	debug("%s: base %p", __func__, mmchost->reg);

	/* Reset controller */
	writel(SUNXI_MMC_GCTRL_RESET, &mmchost->reg->gctrl);

	/* Wait for the reset bit (auto-clearing) to deassert */
	ret = readl_poll_timeout(&mmchost->reg->gctrl, regval,
				 !(regval & SUNXI_MMC_GCTRL_RESET), 1000000);

	return ret;
}

static int mmc_trans_data_by_cpu(struct mmc *mmc, struct mmc_data *data)
{
	struct sunxi_mmc_host *mmchost = mmc->priv;
	const int reading = !!(data->flags & MMC_DATA_READ);
	const uint32_t status_bit = reading ? SUNXI_MMC_STATUS_FIFO_EMPTY :
					      SUNXI_MMC_STATUS_FIFO_FULL;
	unsigned i;
	unsigned *buff = (unsigned int *)(reading ? data->dest : data->src);
	unsigned byte_cnt = data->blocksize * data->blocks;
	unsigned timeout_usecs = (byte_cnt >> 8) * 1000;
	if (timeout_usecs < 2000000)
		timeout_usecs = 2000000;

	/* Always read / write data through the CPU */
	setbits_le32(&mmchost->reg->gctrl, SUNXI_MMC_GCTRL_ACCESS_BY_AHB);

	for (i = 0; i < (byte_cnt >> 2); i++) {
		while (readl(&mmchost->reg->status) & status_bit) {
			if (!timeout_usecs--)
				return -1;
			udelay(1);
		}

		if (reading)
			buff[i] = readl(&mmchost->reg->fifo);
		else
			writel(buff[i], &mmchost->reg->fifo);
	}

	return 0;
}

static int mmc_rint_wait(struct mmc *mmc, unsigned int timeout_msecs,
			 unsigned int done_bit, const char *what)
{
	struct sunxi_mmc_host *mmchost = mmc->priv;
	unsigned int status;

	do {
		status = readl(&mmchost->reg->rint);
		if (!timeout_msecs-- ||
		    (status & SUNXI_MMC_RINT_INTERRUPT_ERROR_BIT)) {
			debug("%s timeout %x\n", what,
			      status & SUNXI_MMC_RINT_INTERRUPT_ERROR_BIT);
			return -ETIMEDOUT;
		}
		udelay(1000);
	} while (!(status & done_bit));

	return 0;
}

#if defined(CONFIG_DM_MMC_OPS)
static int sunxi_mmc_send_cmd(struct udevice *dev, struct mmc_cmd *cmd,
			      struct mmc_data *data)
{
	struct mmc *mmc = mmc_get_mmc_dev(dev);
#else
static int sunxi_mmc_send_cmd(struct mmc *mmc, struct mmc_cmd *cmd,
			      struct mmc_data *data)
{
#endif
	struct sunxi_mmc_host *mmchost = mmc->priv;
	unsigned int cmdval = SUNXI_MMC_CMD_START;
	unsigned int timeout_msecs;
	int error = 0;
	unsigned int status = 0;
	unsigned int bytecnt = 0;

	if (mmchost->fatal_err)
		return -1;
	if (cmd->resp_type & MMC_RSP_BUSY)
		debug("mmc cmd %d check rsp busy\n", cmd->cmdidx);
	if (cmd->cmdidx == 12)
		return 0;

	if (!cmd->cmdidx)
		cmdval |= SUNXI_MMC_CMD_SEND_INIT_SEQ;
	if (cmd->resp_type & MMC_RSP_PRESENT)
		cmdval |= SUNXI_MMC_CMD_RESP_EXPIRE;
	if (cmd->resp_type & MMC_RSP_136)
		cmdval |= SUNXI_MMC_CMD_LONG_RESPONSE;
	if (cmd->resp_type & MMC_RSP_CRC)
		cmdval |= SUNXI_MMC_CMD_CHK_RESPONSE_CRC;

	if (data) {
		if ((u32)(long)data->dest & 0x3) {
			error = -1;
			goto out;
		}

		cmdval |= SUNXI_MMC_CMD_DATA_EXPIRE|SUNXI_MMC_CMD_WAIT_PRE_OVER;
		if (data->flags & MMC_DATA_WRITE)
			cmdval |= SUNXI_MMC_CMD_WRITE;
		if (data->blocks > 1)
			cmdval |= SUNXI_MMC_CMD_AUTO_STOP;
		writel(data->blocksize, &mmchost->reg->blksz);
		writel(data->blocks * data->blocksize, &mmchost->reg->bytecnt);
	}

	debug("mmc %p, cmd %d(0x%08x), arg 0x%08x\n", mmchost,
	      cmd->cmdidx, cmdval | cmd->cmdidx, cmd->cmdarg);
	writel(cmd->cmdarg, &mmchost->reg->arg);

	if (!data)
		writel(cmdval | cmd->cmdidx, &mmchost->reg->cmd);

	/*
	 * transfer data and check status
	 * STATREG[2] : FIFO empty
	 * STATREG[3] : FIFO full
	 */
	if (data) {
		int ret = 0;

		bytecnt = data->blocksize * data->blocks;
		debug("trans data %d bytes\n", bytecnt);
		writel(cmdval | cmd->cmdidx, &mmchost->reg->cmd);
		ret = mmc_trans_data_by_cpu(mmc, data);
		if (ret) {
			error = readl(&mmchost->reg->rint) & \
				SUNXI_MMC_RINT_INTERRUPT_ERROR_BIT;
			error = -ETIMEDOUT;
			goto out;
		}
	}

	error = mmc_rint_wait(mmc, 1000, SUNXI_MMC_RINT_COMMAND_DONE, "cmd");
	if (error)
		goto out;

	if (data) {
		timeout_msecs = 120;
		debug("cacl timeout %x msec\n", timeout_msecs);
		error = mmc_rint_wait(mmc, timeout_msecs,
				      data->blocks > 1 ?
				      SUNXI_MMC_RINT_AUTO_COMMAND_DONE :
				      SUNXI_MMC_RINT_DATA_OVER,
				      "data");
		if (error)
			goto out;
	}

	if (cmd->resp_type & MMC_RSP_BUSY) {
		timeout_msecs = 2000;
		do {
			status = readl(&mmchost->reg->status);
			if (!timeout_msecs--) {
				debug("busy timeout\n");
				error = -ETIMEDOUT;
				goto out;
			}
			udelay(1000);
		} while (status & SUNXI_MMC_STATUS_CARD_DATA_BUSY);
	}

	if (cmd->resp_type & MMC_RSP_136) {
		cmd->response[0] = readl(&mmchost->reg->resp3);
		cmd->response[1] = readl(&mmchost->reg->resp2);
		cmd->response[2] = readl(&mmchost->reg->resp1);
		cmd->response[3] = readl(&mmchost->reg->resp0);
		debug("mmc resp 0x%08x 0x%08x 0x%08x 0x%08x\n",
		      cmd->response[3], cmd->response[2],
		      cmd->response[1], cmd->response[0]);
	} else {
		cmd->response[0] = readl(&mmchost->reg->resp0);
		debug("mmc resp 0x%08x\n", cmd->response[0]);
	}
out:
	if (error < 0) {
		writel(SUNXI_MMC_GCTRL_RESET, &mmchost->reg->gctrl);
		mmc_update_clk(mmc);
	}
	writel(0xffffffff, &mmchost->reg->rint);
	writel(readl(&mmchost->reg->gctrl) | SUNXI_MMC_GCTRL_FIFO_RESET,
	       &mmchost->reg->gctrl);

	return error;
}

static inline int cdpin_is_valid(struct sunxi_mmc_host *priv)
{
#if !defined(CONFIG_DM_MMC)
	return priv->cd_pin >= 0;
#else
	return dm_gpio_is_valid(&priv->cd_gpio);
#endif
}

static inline int cdpin_get_value(struct sunxi_mmc_host *priv)
{
#if !defined(CONFIG_DM_MMC)
	return gpio_get_value(priv->cd_pin);
#else
	return dm_gpio_get_value(&priv->cd_gpio);
#endif
}

#if defined(CONFIG_DM_MMC_OPS)
static int sunxi_mmc_getcd(struct udevice *dev)
{
	struct mmc *mmc = mmc_get_mmc_dev(dev);
#else
static int sunxi_mmc_getcd(struct mmc *mmc)
{
#endif
	struct sunxi_mmc_host *priv = mmc->priv;
	int value = 1;

	if (cdpin_is_valid(priv)) {
		value = cdpin_get_value(priv);

		if (priv->cd_inverted)
			return !value;
	}

	return value;
}

#if !defined(CONFIG_DM_MMC)
struct mmc *sunxi_mmc_init(int sdc_no)
{
	struct mmc_config *cfg = &mmc_host[sdc_no].cfg;

	memset(&mmc_host[sdc_no], 0, sizeof(struct sunxi_mmc_host));
	mmc_host[sdc_no].cd_inverted = true;

	cfg->name = "SUNXI SD/MMC";
	cfg->ops  = &sunxi_mmc_ops;

	cfg->voltages = MMC_VDD_32_33 | MMC_VDD_33_34;
	cfg->host_caps = MMC_MODE_4BIT;
#if defined(CONFIG_MACH_SUN50I) || defined(CONFIG_MACH_SUN8I)
	if (sdc_no == 2)
		cfg->host_caps = MMC_MODE_8BIT;
#endif
	cfg->host_caps |= MMC_MODE_HS_52MHz | MMC_MODE_HS;
	cfg->b_max = CONFIG_SYS_MMC_MAX_BLK_COUNT;

	cfg->f_min = 400000;
	cfg->f_max = 52000000;

	if (mmc_resource_init(sdc_no) != 0)
		return NULL;

	mmc_clk_io_on(&mmc_host[sdc_no]);

	return mmc_create(cfg, &mmc_host[sdc_no]);
}
#endif

#if defined(CONFIG_DM_MMC) && defined(CONFIG_DM_MMC_OPS)
static const struct dm_mmc_ops sunxi_mmc_ops = {
	.send_cmd	= sunxi_mmc_send_cmd,
	.set_ios	= sunxi_mmc_set_ios,
	.get_cd		= sunxi_mmc_getcd,
};
#else
static const struct mmc_ops sunxi_mmc_ops = {
	.send_cmd	= sunxi_mmc_send_cmd,
	.set_ios	= sunxi_mmc_set_ios,
	.init		= sunxi_mmc_core_init,
	.getcd		= sunxi_mmc_getcd,
};
#endif

#if defined(CONFIG_DM_MMC)
static int sunxi_mmc_ofdata_to_platdata(struct udevice *dev)
{
	return 0;
}

static int sunxi_mmc_probe(struct udevice *dev)
{
	struct mmc_uclass_priv *upriv = dev_get_uclass_priv(dev);
#if defined(CONFIG_BLK)
	struct sunxi_mmc_plat *plat = dev_get_platdata(dev);
#endif
	struct sunxi_mmc_host *priv = dev_get_priv(dev);
	struct mmc_config *cfg = &priv->cfg;
	int bus_width;
	u32 f_minmax[2];

	priv->reg = (void *)dev_get_addr(dev);
	cfg->name = "SUNXI SD/MMC";
#if !(defined(CONFIG_DM_MMC) && defined(CONFIG_DM_MMC_OPS))
	cfg->ops  = &sunxi_mmc_ops;
#endif
	cfg->voltages = MMC_VDD_32_33 | MMC_VDD_33_34;
	cfg->b_max = CONFIG_SYS_MMC_MAX_BLK_COUNT;
	cfg->host_caps = MMC_MODE_HS_52MHz | MMC_MODE_HS;

	bus_width = fdtdec_get_int(gd->fdt_blob, dev->of_offset,
				   "bus-width", 4);
	if (bus_width == 8)
		cfg->host_caps |= MMC_MODE_8BIT;
	else if (bus_width == 4)
		cfg->host_caps |= MMC_MODE_4BIT;

	debug("%s: reg %p bus_width %d\n", dev->name, priv->reg, bus_width);

	if (!fdtdec_get_int_array(gd->fdt_blob, dev->of_offset,
				  "clock-freq-min-max", f_minmax, 2)) {
		cfg->f_min = f_minmax[0];
		cfg->f_max = f_minmax[1];
	} else {
		/* use the defaults */
		cfg->f_min = 400000;
		cfg->f_max = 52000000;
	}

	/* Some legacy functionality in our tree still depends on the
	 * mmchost->mmc_no... until we can get rid of this, initialise
	 * it based on the base address of the device.
	 */
	if (mmc_resource_init_from_udev(dev) != 0)
		return -EINVAL;

	/* All GPIOs are optional */
	gpio_request_by_name(dev, "cd-gpios", 0,
			     &priv->cd_gpio, GPIOD_IS_IN);
	priv->cd_inverted = fdtdec_get_bool(gd->fdt_blob, dev->of_offset,
					    "cd-inverted");
	gpio_request_by_name(dev, "wp-gpios", 0,
			     &priv->wp_gpio, GPIOD_IS_IN);
	priv->wp_inverted = fdtdec_get_bool(gd->fdt_blob, dev->of_offset,
					    "wp-inverted");
	gpio_request_by_name(dev, "power-gpios", 0,
			     &priv->pwr_gpio, GPIOD_IS_OUT);
	if (dm_gpio_is_valid(&priv->pwr_gpio))
		dm_gpio_set_value(&priv->pwr_gpio, 1);

	if (reset_get_by_name(dev, "ahb", &priv->reset)) {
		error("%s: failed to get 'ahb' reset\n", dev->name);
		return -EINVAL;
	}

	if (clk_get_by_name(dev, "ahb", &priv->ahb_clk_gate) ||
	    clk_get_by_name(dev, "mmc", &priv->mmc_clk)) {
		error("%s: failed to get all required clocks ('ahb', 'mmc')\n",
		      dev->name);
		return -EINVAL;
	}

	mmc_clk_io_on(priv);

#if defined(CONFIG_BLK)
	priv->mmc = &plat->mmc;
#else
	priv->mmc = mmc_create(cfg, priv);
	if (priv->mmc == NULL)
		return -1;
#endif
	priv->mmc->priv = priv;
	priv->mmc->dev = dev;
	priv->mmc->cfg = cfg;
	priv->mmc->has_init = 0;
	upriv->mmc = priv->mmc;

#if defined(CONFIG_DM_MMC) && defined(CONFIG_DM_MMC_OPS)
	sunxi_mmc_core_init(priv->mmc);
#endif
	return 0;
}

#if defined(CONFIG_BLK)
static int sunxi_mmc_bind(struct udevice *dev)
{
	struct sunxi_mmc_plat *plat = dev_get_platdata(dev);
	struct sunxi_mmc_host *priv = dev_get_priv(dev);

	debug("%s: %s\n", dev->name, __func__);

	/* TODO: To move cfg into plat, we need to change the legacy
	   code, which references through the arrays... */

	return mmc_bind(dev, &plat->mmc, &priv->cfg);
}
#endif

static const struct udevice_id sunxi_mmc_ids[] = {
	{ .compatible = "allwinner,sun50i-a64-mmc" },
	{ }
};

U_BOOT_DRIVER(sunxi_mmc_drv) = {
	.name		= "sunxi_mmc",
	.id		= UCLASS_MMC,
	.of_match	= sunxi_mmc_ids,
	.ofdata_to_platdata = sunxi_mmc_ofdata_to_platdata,
	.probe		= sunxi_mmc_probe,
	.priv_auto_alloc_size = sizeof(struct sunxi_mmc_host),
	.platdata_auto_alloc_size = sizeof(struct sunxi_mmc_plat),
#if defined(CONFIG_DM_MMC_OPS)
	.ops		= &sunxi_mmc_ops,
#endif
#if defined(CONFIG_BLK)
	.bind           = sunxi_mmc_bind,
#endif
};

#endif
