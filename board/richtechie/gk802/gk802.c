/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 * Copyright (C) 2013 James Laird <jhl@mafipulation.org>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6x_pins.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <linux/fb.h>
#include <ipu_pixfmt.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/mxc_hdmi.h>
DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |            \
    PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |               \
    PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |            \
    PAD_CTL_PUS_47K_UP  | PAD_CTL_SPEED_HIGH |               \
    PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |     \
    PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED   |     \
    PAD_CTL_DSE_40ohm   | PAD_CTL_HYS)

int dram_init(void)
{
    gd->ram_size = get_ram_size((void *)PHYS_SDRAM, PHYS_SDRAM_SIZE);

    return 0;
}

iomux_v3_cfg_t const uart4_pads[] = {
	MX6Q_PAD_KEY_COL0__UART4_TXD | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6Q_PAD_KEY_ROW0__UART4_RXD | MUX_PAD_CTRL(UART_PAD_CTRL),
};

iomux_v3_cfg_t const usdhc3_pads[] = {
    MX6Q_PAD_SD3_CLK__USDHC3_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6Q_PAD_SD3_CMD__USDHC3_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6Q_PAD_SD3_DAT0__USDHC3_DAT0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6Q_PAD_SD3_DAT1__USDHC3_DAT1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6Q_PAD_SD3_DAT2__USDHC3_DAT2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6Q_PAD_SD3_DAT3__USDHC3_DAT3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

iomux_v3_cfg_t const usdhc4_pads[] = {
    MX6Q_PAD_SD4_CLK__USDHC4_CLK    | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6Q_PAD_SD4_CMD__USDHC4_CMD    | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6Q_PAD_SD4_DAT0__USDHC4_DAT0  | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6Q_PAD_SD4_DAT1__USDHC4_DAT1  | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6Q_PAD_SD4_DAT2__USDHC4_DAT2  | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6Q_PAD_SD4_DAT3__USDHC4_DAT3  | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

static void setup_iomux_uart(void)
{
    imx_iomux_v3_setup_multiple_pads(uart4_pads, ARRAY_SIZE(uart4_pads));
}

#ifdef CONFIG_FSL_ESDHC
struct fsl_esdhc_cfg usdhc_cfg[4] = {
    {USDHC4_BASE_ADDR},
    {USDHC3_BASE_ADDR},
};

int board_mmc_getcd(struct mmc *mmc)
{
    struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
    if (cfg->esdhc_base == USDHC3_BASE_ADDR) {
        gpio_direction_input(IMX_GPIO_NR(6, 11));
        return !gpio_get_value(IMX_GPIO_NR(6, 11));
    } else {
        return 1;   // we booted off that one probably
    }
}

int board_mmc_init(bd_t *bis)
{
    int ret = 0;
    imx_iomux_v3_setup_multiple_pads(usdhc3_pads, ARRAY_SIZE(usdhc3_pads));
    imx_iomux_v3_setup_multiple_pads(usdhc4_pads, ARRAY_SIZE(usdhc4_pads));

    usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);
    usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);

    ret |= fsl_esdhc_initialize(bis, &usdhc_cfg[0]);
    ret |= fsl_esdhc_initialize(bis, &usdhc_cfg[1]);
    return ret;
}
#endif

#if defined(CONFIG_VIDEO_IPUV3)

struct display_info_t {
	int	bus;
	int	addr;
	int	pixfmt;
	int	(*detect)(struct display_info_t const *dev);
	void	(*enable)(struct display_info_t const *dev);
	struct	fb_videomode mode;
};

static int detect_hdmi(struct display_info_t const *dev)
{
	return __raw_readb(HDMI_ARB_BASE_ADDR+HDMI_PHY_STAT0) & HDMI_PHY_HPD;
}

static void enable_hdmi(struct display_info_t const *dev)
{
	u8 reg;
	printf("%s: setup HDMI monitor\n", __func__);
	reg = __raw_readb(
			HDMI_ARB_BASE_ADDR
			+HDMI_PHY_CONF0);
	reg |= HDMI_PHY_CONF0_PDZ_MASK;
	__raw_writeb(reg,
		     HDMI_ARB_BASE_ADDR
			+HDMI_PHY_CONF0);
	udelay(3000);
	reg |= HDMI_PHY_CONF0_ENTMDS_MASK;
	__raw_writeb(reg,
		     HDMI_ARB_BASE_ADDR
			+HDMI_PHY_CONF0);
	udelay(3000);
	reg |= HDMI_PHY_CONF0_GEN2_TXPWRON_MASK;
	__raw_writeb(reg,
		     HDMI_ARB_BASE_ADDR
			+HDMI_PHY_CONF0);
	__raw_writeb(HDMI_MC_PHYRSTZ_ASSERT,
		     HDMI_ARB_BASE_ADDR+HDMI_MC_PHYRSTZ);
}

static struct display_info_t display = {
	.bus	= -1,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= detect_hdmi,
	.enable	= enable_hdmi,
	.mode	= {
		.name           = "HDMI",
		.refresh        = 60,
		.xres           = 1024,
		.yres           = 768,
		.pixclock       = 15385,
		.left_margin    = 220,
		.right_margin   = 40,
		.upper_margin   = 21,
		.lower_margin   = 7,
		.hsync_len      = 60,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} };

int board_video_skip(void)
{
	int ret;

    int xres, yres, refresh;
    xres = getenv_ulong("hdmi_xres", 10, CONFIG_HDMI_XRES);
    yres = getenv_ulong("hdmi_yres", 10, CONFIG_HDMI_YRES);
    refresh = getenv_ulong("hdmi_refresh", 10, CONFIG_HDMI_REFRESH);
    printf("got mode %dx%d@%d\n", xres, yres, refresh);
    
    display.mode.hsync_len = xres/12;
    display.mode.left_margin = (xres + display.mode.right_margin + display.mode.hsync_len)/4 + 10;
    long active_y = display.mode.upper_margin + yres +
                    display.mode.lower_margin + display.mode.vsync_len;
    long active_x = display.mode.left_margin + xres +
                    display.mode.right_margin + display.mode.hsync_len;
    long long pixclock_calc = 1e12;

    pixclock_calc /= active_x * active_y * refresh;
    printf("%d active pixels, pixel clock %lld ps\n", active_x*active_y, pixclock_calc);

    if (pixclock_calc > 1e9 || pixclock_calc < 1e3) {
        puts("invalid video mode selected, going to default mode");
    } else {
        display.mode.xres = xres;
        display.mode.yres = yres;
        display.mode.refresh = refresh;
        display.mode.pixclock = pixclock_calc;
    }

    ret = ipuv3_fb_init(&display.mode, 0,
                display.pixfmt);
    if (!ret) {
        display.enable(&display);
        printf("Display: %s (%ux%u)\n",
               display.mode.name,
               display.mode.xres,
               display.mode.yres);
    } else {
        printf("display %s cannot be configured: %d\n",
               display.mode.name, ret);
        ret = -EINVAL;
    }
	return (0 != ret);
}

static void setup_display(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct anatop_regs *anatop = (struct anatop_regs *)ANATOP_BASE_ADDR;
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;

	int reg;

	/* Turn on LDB0,IPU,IPU DI0 clocks */
	reg = __raw_readl(&mxc_ccm->CCGR3);
	reg |=   MXC_CCM_CCGR3_IPU1_IPU_DI0_OFFSET
		|MXC_CCM_CCGR3_LDB_DI0_MASK;
	writel(reg, &mxc_ccm->CCGR3);

	/* Turn on HDMI PHY clock */
	reg = __raw_readl(&mxc_ccm->CCGR2);
	reg |=  MXC_CCM_CCGR2_HDMI_TX_IAHBCLK_MASK
	       |MXC_CCM_CCGR2_HDMI_TX_ISFRCLK_MASK;
	writel(reg, &mxc_ccm->CCGR2);

	/* clear HDMI PHY reset */
	__raw_writeb(HDMI_MC_PHYRSTZ_DEASSERT,
		     HDMI_ARB_BASE_ADDR+HDMI_MC_PHYRSTZ);

	/* set PFD1_FRAC to 0x13 == 455 MHz (480*18)/0x13 */
	writel(ANATOP_PFD_480_PFD1_FRAC_MASK, &anatop->pfd_480_clr);
	writel(0x13<<ANATOP_PFD_480_PFD1_FRAC_SHIFT, &anatop->pfd_480_set);

	/* set LDB0, LDB1 clk select to 011/011 */
	reg = readl(&mxc_ccm->cs2cdr);
	reg &= ~(MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK
		 |MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_MASK);
	reg |= (3<<MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET)
	      |(3<<MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->cs2cdr);

	reg = readl(&mxc_ccm->cscmr2);
	reg |= MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV;
	writel(reg, &mxc_ccm->cscmr2);
    
	reg = readl(&mxc_ccm->chsccdr);
	reg &= ~(MXC_CCM_CHSCCDR_IPU1_DI0_PRE_CLK_SEL_MASK
		|MXC_CCM_CHSCCDR_IPU1_DI0_PODF_MASK
		|MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_MASK);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<<MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET)
	      |(CHSCCDR_PODF_DIVIDE_BY_3
		<<MXC_CCM_CHSCCDR_IPU1_DI0_PODF_OFFSET)
	      |(CHSCCDR_IPU_PRE_CLK_540M_PFD
		<<MXC_CCM_CHSCCDR_IPU1_DI0_PRE_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->chsccdr);

}
#endif


u32 get_board_rev(void)
{
    return 0x63011;
}

int board_recovery_init_f(void)
{
    gpio_direction_input(IMX_GPIO_NR(3, 16));
    if (!gpio_get_value(IMX_GPIO_NR(3, 16))) {
        puts("RECOVERY SWITCH PRESSED\n");
        puts("trying to assume sensible defaults\n");

        setenv("bootdelay", "30");
        setenv("recovery", "1");

        /* these have to be set before setup_display() */
        setenv("hdmi_xres", "640");
        setenv("hdmi_yres", "480");
        setenv("hdmi_refresh", "60");

        /* prevent a misconfigured preboot from breaking recovery */
        setenv("preboot", "usb start");
    } else {
        setenv("recovery", "0");
    }

    return 0;
}


int board_early_init_f(void)
{
    setup_iomux_uart();

#if defined(CONFIG_VIDEO_IPUV3)
	setup_display();
#endif
    return 0;
}

int board_init(void)
{
    /* address of boot parameters */
    gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

    /* config Reset Controller (SRC) to disable warm resets (ie uboot
       reset command should go 100% back to strapped bootloader)
    */
    clrbits_le32(SRC_BASE_ADDR, 1);

    return 0;
}

int checkboard(void)
{
    puts("Board: MX6Q-gk802\n");

    return 0;
}

#ifdef CONFIG_USB_EHCI_MX6
int board_ehci_hcd_init(int port)
{
	return 0;
}
#endif
