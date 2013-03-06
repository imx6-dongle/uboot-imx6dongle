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
#include <miiphy.h>
#include <netdev.h>
DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |            \
    PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |               \
    PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |            \
    PAD_CTL_PUS_47K_UP  | PAD_CTL_SPEED_LOW |               \
    PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

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
iomux_v3_cfg_t const usdhc2_pads[] = {
    MX6Q_PAD_SD2_CLK__USDHC2_CLK    | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6Q_PAD_SD2_CMD__USDHC2_CMD    | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6Q_PAD_SD2_DAT0__USDHC2_DAT0  | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6Q_PAD_SD2_DAT1__USDHC2_DAT1  | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6Q_PAD_SD2_DAT2__USDHC2_DAT2  | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6Q_PAD_SD2_DAT3__USDHC2_DAT3  | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

static void setup_iomux_uart(void)
{
    imx_iomux_v3_setup_multiple_pads(uart4_pads, ARRAY_SIZE(uart4_pads));
}

#ifdef CONFIG_FSL_ESDHC
struct fsl_esdhc_cfg usdhc_cfg[2] = {
    {USDHC3_BASE_ADDR},
    {USDHC2_BASE_ADDR},
};

int board_mmc_getcd(struct mmc *mmc)
{
    struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
    if (cfg->esdhc_base == USDHC3_BASE_ADDR)
        return 1;   // we booted off that one probably

    gpio_direction_input(IMX_GPIO_NR(6, 11));
    return !gpio_get_value(IMX_GPIO_NR(6, 11));
}

int board_mmc_init(bd_t *bis)
{
    int ret;
    imx_iomux_v3_setup_multiple_pads(usdhc3_pads, ARRAY_SIZE(usdhc3_pads));
    imx_iomux_v3_setup_multiple_pads(usdhc2_pads, ARRAY_SIZE(usdhc2_pads));

    usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
    usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);

    ret = fsl_esdhc_initialize(bis, &usdhc_cfg[1]);
    ret |= fsl_esdhc_initialize(bis, &usdhc_cfg[0]);
    return ret;
}
#endif

u32 get_board_rev(void)
{
    return 0;
}

int board_early_init_f(void)
{
    setup_iomux_uart();

    return 0;
}

int board_init(void)
{
    /* address of boot parameters */
    gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

    return 0;
}

int checkboard(void)
{
    puts("Board: MX6Q-gk802\n");

    return 0;
}

int misc_init_r(void) {
    gpio_direction_input(IMX_GPIO_NR(3, 16));
    if (!gpio_get_value(IMX_GPIO_NR(3, 16))) {
        puts("RECOVERY SWITCH PRESSED\n");

        setenv("bootdelay", "5");
        setenv("recovery", "1");
    } else {
        setenv("recovery", "0");
    }
    return 0;
}
