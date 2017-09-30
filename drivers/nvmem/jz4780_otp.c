/*
 * JZ4780 OTP memory NVMEM driver
 *
 * Copyright (c) 2017 PrasannaKumar Muralidharan <prasannatsmkumar@gmail.com>
 * Copyright (C) 2014 Imagination Technologies
 *
 * Based on work by Ingenic Semiconductor.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 */

/*
 * Currently supports JZ4780 which has 8K efuse.
 *
 * The rom itself is accessed using a 9 bit address line and an 8 word wide bus
 * which reads/writes based on strobes. The strobe is configured in the config
 * register and is based on number of cycles of the bus clock.
 *
 * Driver supports reading only as writes are done in the Factory. Reading
 * Customer ID and Chip ID are supported.
 */
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/nvmem-provider.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/timer.h>

#define JZ_EFUCTRL			(0x0)	/* Control Register */
#define JZ_EFUCFG			(0x4)	/* Configure Register*/
#define JZ_EFUSTATE			(0x8)	/* Status Register */
#define JZ_EFUDATA(n)			(0xC + (n)*4)

#define JZ4780_OTP_START_ADDR		0x200
#define JZ4780_OTP_SEG1_OFF		0x00	/* 64 bit Random Number */
#define JZ4780_OTP_SEG2_OFF		0x08	/* 128 bit Ingenic Chip ID */
#define JZ4780_OTP_SEG3_OFF		0x18	/* 128 bit Customer ID */
#define JZ4780_OTP_SEG4_OFF		0x28	/* 3520 bit Reserved */
#define JZ4780_OTP_SEG5_OFF		0x1E0	/* 8 bit Protect Segment */
#define JZ4780_OTP_SEG6_OFF		0x1E1	/* 2296 bit HDMI Key */
#define JZ4780_OTP_SEG7_OFF		0x300	/* 2048 bit Security boot key */
#define JZ4780_OTP_END_ADDR		0x5FF

#define JZ4780_OTP_EFUCTRL_CS		BIT(30)
#define JZ4780_OTP_EFUCTRL_ADDR_MASK	0x1FF
#define JZ4780_OTP_EFUCTRL_ADDR_SHIFT	21
#define JZ4780_OTP_EFUCTRL_LEN_MASK	0x1F
#define JZ4780_OTP_EFUCTRL_LEN_SHIFT	16
#define JZ4780_OTP_EFUCTRL_PG_EN		BIT(15)
#define JZ4780_OTP_EFUCTRL_WR_EN		BIT(1)
#define JZ4780_OTP_EFUCTRL_RD_EN		BIT(0)

#define JZ4780_OTP_EFUCFG_INT_EN		BIT(31)
#define JZ4780_OTP_EFUCFG_RD_ADJ_MASK	0xF
#define JZ4780_OTP_EFUCFG_RD_ADJ_SHIFT	20
#define JZ4780_OTP_EFUCFG_RD_STR_MASK	0xF
#define JZ4780_OTP_EFUCFG_RD_STR_SHIFT	16

#define JZ4780_OTP_EFUSTATE_RD_DONE	BIT(0)

#define JZ4780_OTP_WORD_SIZE		16
#define JZ4780_OTP_STRIDE		8

struct jz4780_otp {
	void __iomem *base;
	struct clk *clk;
	unsigned int read_adjust;
	unsigned int read_strobe;
};

static int jz4780_otp_read(void *context, unsigned int offset,
			    void *val, size_t bytes)
{
	struct jz4780_otp *otp = context;
	unsigned int tmp = 0;
	int timeout = 1000;
	unsigned int addr = JZ4780_OTP_START_ADDR + JZ4780_OTP_SEG2_OFF +
			    offset / 4;

	if (bytes != JZ4780_OTP_WORD_SIZE)
		return -EINVAL;

	/* 1. Set config register */
	tmp = readl(otp->base + JZ_EFUCFG);
	tmp &= ~((JZ4780_OTP_EFUCFG_RD_ADJ_MASK << JZ4780_OTP_EFUCFG_RD_ADJ_SHIFT)
	       | (JZ4780_OTP_EFUCFG_RD_STR_MASK << JZ4780_OTP_EFUCFG_RD_STR_SHIFT));
	tmp |= (otp->read_adjust << JZ4780_OTP_EFUCFG_RD_ADJ_SHIFT)
	       | (otp->read_strobe << JZ4780_OTP_EFUCFG_RD_STR_SHIFT);
	writel(tmp, otp->base + JZ_EFUCFG);

	/*
	 * 2. Set control register to indicate what to read: set data address,
	 * data size and read enable.
	 */
	tmp = readl(otp->base + JZ_EFUCTRL);
	tmp &= ~(JZ4780_OTP_EFUCFG_RD_STR_SHIFT
		| (JZ4780_OTP_EFUCTRL_ADDR_MASK << JZ4780_OTP_EFUCTRL_ADDR_SHIFT)
		| JZ4780_OTP_EFUCTRL_PG_EN | JZ4780_OTP_EFUCTRL_WR_EN
		| JZ4780_OTP_EFUCTRL_WR_EN);

	/* Need to select CS bit if address accesses upper 4Kbits memory */
	if (addr >= (JZ4780_OTP_START_ADDR + 512))
		tmp |= JZ4780_OTP_EFUCTRL_CS;

	tmp |= (addr << JZ4780_OTP_EFUCTRL_ADDR_SHIFT)
		| ((8 - 1) << JZ4780_OTP_EFUCTRL_LEN_SHIFT)
		| JZ4780_OTP_EFUCTRL_RD_EN;
	writel(tmp, otp->base + JZ_EFUCTRL);

	/*
	 * 3. Wait status register RD_DONE to be set to 1 or EFUSE interrupted,
	 * software can read EFUSE data buffer 0 – 8 registers.
	 */
	do {
		tmp = readl(otp->base + JZ_EFUSTATE);
		usleep_range(1000, 2000);
		if (timeout--)
			break;
	} while (!(tmp & JZ4780_OTP_EFUSTATE_RD_DONE));

	if (timeout <= 0) {
		return -EAGAIN;
	}

	*(unsigned int *) val = readl(otp->base + JZ_EFUDATA(0));
	*(unsigned int *) (val + 4) = readl(otp->base + JZ_EFUDATA(1));

	return 0;
}

static struct nvmem_config jz4780_otp_nvmem_config = {
	.name = "jz4780-otp",
	.read_only = true,
	.word_size = JZ4780_OTP_WORD_SIZE,
	.stride = JZ4780_OTP_STRIDE,
	.owner = THIS_MODULE,
	.reg_read = jz4780_otp_read,
};

static int jz4780_otp_probe(struct platform_device *pdev)
{
	struct nvmem_device *nvmem;
	struct jz4780_otp *otp;
	struct resource *res;
	unsigned long clk_rate;

	otp = devm_kzalloc(&pdev->dev, sizeof(*otp), GFP_KERNEL);
	if (!otp)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	otp->base = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (IS_ERR(otp->base))
		return PTR_ERR(otp->base);

	otp->clk = devm_clk_get(&pdev->dev, "bus_clk");
	if (IS_ERR(otp->clk))
		return PTR_ERR(otp->clk);

	clk_rate = clk_get_rate(otp->clk);
	otp->read_adjust = (((6500 * (clk_rate / 1000000)) / 1000000) + 1) - 1;
	otp->read_strobe = ((((35000 * (clk_rate / 1000000)) / 1000000) + 1)
			      - 5 - otp->read_adjust);

	if ((otp->read_adjust > 0x1F) || (otp->read_strobe > 0x1F)) {
		dev_err(&pdev->dev, "Cannot set clock configuration\n");
		return -EINVAL;
	}

	jz4780_otp_nvmem_config.size = JZ4780_OTP_WORD_SIZE * 2;
	jz4780_otp_nvmem_config.dev = &pdev->dev;
	jz4780_otp_nvmem_config.priv = otp;

	nvmem = nvmem_register(&jz4780_otp_nvmem_config);
	if (IS_ERR(nvmem))
		return PTR_ERR(nvmem);

	platform_set_drvdata(pdev, nvmem);

	return 0;
}

static int jz4780_otp_remove(struct platform_device *pdev)
{
	struct nvmem_device *nvmem = platform_get_drvdata(pdev);

	return nvmem_unregister(nvmem);
}

static const struct of_device_id jz4780_otp_dt_ids[] = {
	{ .compatible = "ingenic,jz4780-otp" },
	{ },
};
MODULE_DEVICE_TABLE(of, jz4780_otp_dt_ids);

static struct platform_driver jz4780_otp_driver = {
	.probe	= jz4780_otp_probe,
	.remove	= jz4780_otp_remove,
	.driver = {
		.name	= "jz4780_otp",
		.of_match_table = jz4780_otp_dt_ids,
	},
};
module_platform_driver(jz4780_otp_driver);

MODULE_AUTHOR("PrasannaKumar Muralidharan <prasannatsmkumar@gmail.com>");
MODULE_DESCRIPTION("Ingenic JZ4780 OTP NVMEM driver");
MODULE_LICENSE("GPL v2");
