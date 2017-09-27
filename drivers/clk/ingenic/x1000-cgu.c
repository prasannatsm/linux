/*
 * Ingenic X1000 SoC CGU driver
 *
 * Copyright (c) 2016 PrasannaKumar Muralidharan <prasannatsmkumar@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0
 */

#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <dt-bindings/clock/x1000-cgu.h>
#include "cgu.h"

/* CGU register offsets */
#define CGU_REG_CLOCKCONTROL	0x00
#define CGU_REG_APLL		0x10
#define CGU_REG_MPLL		0x14

/* CLKGR0, OPCR are not defined in X1000, JZ4780 PM so assuming them to be correct */
#define CGU_REG_CLKGR0		0x20
#define CGU_REG_OPCR		0x24
#define CGU_REG_CLKGR1		0x28
#define CGU_REG_DDRCDR		0x2c

/* bits within the OPCR register */
#define OPCR_SPENDN0		(1 << 7)
#define OPCR_SPENDN1		(1 << 6)

static struct ingenic_cgu *cgu;

static const s8 pll_od_encoding[16] = {
	0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7,
	0x8, 0x9, 0xa, 0xb, 0xc, 0xd, 0xe, 0xf,
};

static const struct ingenic_cgu_clk_info x1000_cgu_clocks[] = {

	/* External clocks */

	[X1000_CLK_EXCLK] = { "ext", CGU_CLK_EXT },
	[X1000_CLK_RTCLK] = { "rtc", CGU_CLK_EXT },

	/* PLLs */

#define DEF_PLL(name) { \
	.reg = CGU_REG_ ## name, \
	.m_shift = 19, \
	.m_bits = 13, \
	.m_offset = 1, \
	.n_shift = 13, \
	.n_bits = 6, \
	.n_offset = 1, \
	.od_shift = 9, \
	.od_bits = 4, \
	.od_max = 16, \
	.od_encoding = pll_od_encoding, \
	.stable_bit = 6, \
	.bypass_bit = 1, \
	.enable_bit = 0, \
}

	[X1000_CLK_APLL] = {
		"apll", CGU_CLK_PLL,
		.parents = { X1000_CLK_EXCLK, -1, -1, -1 },
		.pll = DEF_PLL(APLL),
	},

	[X1000_CLK_MPLL] = {
		"mpll", CGU_CLK_PLL,
		.parents = { X1000_CLK_EXCLK, -1, -1, -1 },
		.pll = DEF_PLL(MPLL),
	},

#undef DEF_PLL

	/* Muxes & dividers */

	[X1000_CLK_SCLKA] = {
		"sclk_a", CGU_CLK_MUX,
		.parents = { -1, X1000_CLK_APLL, X1000_CLK_EXCLK,
			     X1000_CLK_RTCLK },
		.mux = { CGU_REG_CLOCKCONTROL, 30, 2 },
	},

	[X1000_CLK_CPUMUX] = {
		"cpumux", CGU_CLK_MUX,
		.parents = { -1, X1000_CLK_SCLKA, X1000_CLK_MPLL,
			     -1 },
		.mux = { CGU_REG_CLOCKCONTROL, 28, 2 },
	},

	[X1000_CLK_CPU] = {
		"cpu", CGU_CLK_DIV,
		.parents = { X1000_CLK_CPUMUX, -1, -1, -1 },
		.div = { CGU_REG_CLOCKCONTROL, 0, 1, 4, 22, -1, -1 },
	},

	[X1000_CLK_L2CACHE] = {
		"l2cache", CGU_CLK_DIV,
		.parents = { X1000_CLK_CPUMUX, -1, -1, -1 },
		.div = { CGU_REG_CLOCKCONTROL, 4, 1, 4, -1, -1, -1 },
	},

	[X1000_CLK_AHB0] = {
		"ahb0", CGU_CLK_MUX | CGU_CLK_DIV,
		.parents = { -1, X1000_CLK_SCLKA, X1000_CLK_MPLL,
			     -1 },
		.mux = { CGU_REG_CLOCKCONTROL, 26, 2 },
		.div = { CGU_REG_CLOCKCONTROL, 8, 1, 4, 21, -1, -1 },
	},

	[X1000_CLK_AHB2PMUX] = {
		"ahb2_apb_mux", CGU_CLK_MUX,
		.parents = { -1, X1000_CLK_SCLKA, X1000_CLK_MPLL,
			     X1000_CLK_RTCLK },
		.mux = { CGU_REG_CLOCKCONTROL, 24, 2 },
	},

	[X1000_CLK_AHB2] = {
		"ahb2", CGU_CLK_DIV,
		.parents = { X1000_CLK_AHB2PMUX, -1, -1, -1 },
		.div = { CGU_REG_CLOCKCONTROL, 12, 1, 4, 20, -1, -1 },
	},

	[X1000_CLK_PCLK] = {
		"pclk", CGU_CLK_DIV,
		.parents = { X1000_CLK_AHB2PMUX, -1, -1, -1 },
		.div = { CGU_REG_CLOCKCONTROL, 16, 1, 4, 20, -1, -1 },
	},

	[X1000_CLK_DDR] = {
		"ddr", CGU_CLK_MUX | CGU_CLK_DIV,
		.parents = { -1, X1000_CLK_SCLKA, X1000_CLK_MPLL, -1 },
		.mux = { CGU_REG_DDRCDR, 30, 2 },
		.div = { CGU_REG_DDRCDR, 0, 1, 4, 29, 28, 27 },
	},

	/* Gate-only clocks */

	[X1000_CLK_UART0] = {
		"uart0", CGU_CLK_GATE,
		.parents = { X1000_CLK_EXCLK, -1, -1, -1 },
		.gate = { CGU_REG_CLKGR0, 15 },
	},

	[X1000_CLK_UART1] = {
		"uart1", CGU_CLK_GATE,
		.parents = { X1000_CLK_EXCLK, -1, -1, -1 },
		.gate = { CGU_REG_CLKGR0, 16 },
	},

	[X1000_CLK_UART2] = {
		"uart2", CGU_CLK_GATE,
		.parents = { X1000_CLK_EXCLK, -1, -1, -1 },
		.gate = { CGU_REG_CLKGR0, 17 },
	},

	[X1000_CLK_PDMA] = {
		"pdma", CGU_CLK_GATE,
		.parents = { X1000_CLK_EXCLK, -1, -1, -1 },
		.gate = { CGU_REG_CLKGR0, 21 },
	},

	[X1000_CLK_DDR0] = {
		"ddr0", CGU_CLK_GATE,
		.parents = { X1000_CLK_DDR, -1, -1, -1 },
		.gate = { CGU_REG_CLKGR0, 30 },
	},

	[X1000_CLK_DDR1] = {
		"ddr1", CGU_CLK_GATE,
		.parents = { X1000_CLK_DDR, -1, -1, -1 },
		.gate = { CGU_REG_CLKGR0, 31 },
	},

	[X1000_CLK_CORE1] = {
		"core1", CGU_CLK_GATE,
		.parents = { X1000_CLK_CPU, -1, -1, -1 },
		.gate = { CGU_REG_CLKGR1, 15 },
	},
};

static void __init x1000_cgu_init(struct device_node *np)
{
	int retval;

	cgu = ingenic_cgu_new(x1000_cgu_clocks,
			      ARRAY_SIZE(x1000_cgu_clocks), np);
	if (!cgu) {
		pr_err("%s: failed to initialise CGU\n", __func__);
		return;
	}

	retval = ingenic_cgu_register_clocks(cgu);
	if (retval) {
		pr_err("%s: failed to register CGU Clocks\n", __func__);
		return;
	}
}
CLK_OF_DECLARE(x1000_cgu, "ingenic,x1000-cgu", x1000_cgu_init);
