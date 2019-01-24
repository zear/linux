// SPDX-License-Identifier: GPL-2.0
/*
 * JZ4740 backend code for the jz4780-bch driver
 * based on jz4740-nand.c
 *
 * Copyright (c) 2019 Paul Cercueil <paul@crapouillou.net>
 */

#include <linux/bitops.h>
#include <linux/device.h>

#include "jz4780_bch.h"
#include "jz4780_bch_internal.h"

#define JZ_REG_NAND_ECC_CTRL	0x00
#define JZ_REG_NAND_DATA	0x04
#define JZ_REG_NAND_PAR0	0x08
#define JZ_REG_NAND_PAR1	0x0C
#define JZ_REG_NAND_PAR2	0x10
#define JZ_REG_NAND_IRQ_STAT	0x14
#define JZ_REG_NAND_IRQ_CTRL	0x18
#define JZ_REG_NAND_ERR(x)	(0x1C + ((x) << 2))

#define JZ_NAND_ECC_CTRL_PAR_READY	BIT(4)
#define JZ_NAND_ECC_CTRL_ENCODING	BIT(3)
#define JZ_NAND_ECC_CTRL_RS		BIT(2)
#define JZ_NAND_ECC_CTRL_RESET		BIT(1)
#define JZ_NAND_ECC_CTRL_ENABLE		BIT(0)

#define JZ_NAND_STATUS_ERR_COUNT	(BIT(31) | BIT(30) | BIT(29))
#define JZ_NAND_STATUS_PAD_FINISH	BIT(4)
#define JZ_NAND_STATUS_DEC_FINISH	BIT(3)
#define JZ_NAND_STATUS_ENC_FINISH	BIT(2)
#define JZ_NAND_STATUS_UNCOR_ERROR	BIT(1)
#define JZ_NAND_STATUS_ERROR		BIT(0)

static const uint8_t empty_block_ecc[] = {
	0xcd, 0x9d, 0x90, 0x58, 0xf4, 0x8b, 0xff, 0xb7, 0x6f
};

static void jz4740_bch_init(struct jz4780_bch *bch, bool encode)
{
	uint32_t reg;

	/* Clear interrupt status */
	writel(0, bch->base + JZ_REG_NAND_IRQ_STAT);

	/* Initialize and enable BCH */
	reg = readl(bch->base + JZ_REG_NAND_ECC_CTRL);
	reg |= JZ_NAND_ECC_CTRL_RESET;
	reg |= JZ_NAND_ECC_CTRL_ENABLE;
	reg |= JZ_NAND_ECC_CTRL_RS;
	if (encode)
		reg |= JZ_NAND_ECC_CTRL_ENCODING;
	else
		reg &= ~JZ_NAND_ECC_CTRL_ENCODING;

	writel(reg, bch->base + JZ_REG_NAND_ECC_CTRL);
}

static int jz4740_bch_calculate(struct jz4780_bch *bch,
				struct jz4780_bch_params *params,
				const u8 *buf, u8 *ecc_code)
{
	uint32_t reg, status;
	unsigned int timeout = 1000;
	int i;

	jz4740_bch_init(bch, true);

	do {
		status = readl(bch->base + JZ_REG_NAND_IRQ_STAT);
	} while (!(status & JZ_NAND_STATUS_ENC_FINISH) && --timeout);

	if (timeout == 0)
		return -ETIMEDOUT;

	reg = readl(bch->base + JZ_REG_NAND_ECC_CTRL);
	reg &= ~JZ_NAND_ECC_CTRL_ENABLE;
	writel(reg, bch->base + JZ_REG_NAND_ECC_CTRL);

	for (i = 0; i < params->bytes; ++i)
		ecc_code[i] = readb(bch->base + JZ_REG_NAND_PAR0 + i);

	/* If the written data is completly 0xff, we also want to write 0xff as
	 * ecc, otherwise we will get in trouble when doing subpage writes. */
	if (memcmp(ecc_code, empty_block_ecc, ARRAY_SIZE(empty_block_ecc)) == 0)
		memset(ecc_code, 0xff, ARRAY_SIZE(empty_block_ecc));

	return 0;
}

static void jz_nand_correct_data(uint8_t *buf, int index, int mask)
{
	int offset = index & 0x7;
	uint16_t data;

	index += (index >> 3);

	data = buf[index];
	data |= buf[index + 1] << 8;

	mask ^= (data >> offset) & 0x1ff;
	data &= ~(0x1ff << offset);
	data |= (mask << offset);

	buf[index] = data & 0xff;
	buf[index + 1] = (data >> 8) & 0xff;
}

static int jz4740_bch_correct(struct jz4780_bch *bch,
			      struct jz4780_bch_params *params,
			      u8 *buf, u8 *ecc_code)
{
	int i, error_count, index;
	uint32_t reg, status, error;
	unsigned int timeout = 1000;

	jz4740_bch_init(bch, false);

	for (i = 0; i < params->bytes; ++i)
		writeb(ecc_code[i], bch->base + JZ_REG_NAND_PAR0 + i);

	reg = readl(bch->base + JZ_REG_NAND_ECC_CTRL);
	reg |= JZ_NAND_ECC_CTRL_PAR_READY;
	writel(reg, bch->base + JZ_REG_NAND_ECC_CTRL);

	do {
		status = readl(bch->base + JZ_REG_NAND_IRQ_STAT);
	} while (!(status & JZ_NAND_STATUS_DEC_FINISH) && --timeout);

	if (timeout == 0)
		return -ETIMEDOUT;

	reg = readl(bch->base + JZ_REG_NAND_ECC_CTRL);
	reg &= ~JZ_NAND_ECC_CTRL_ENABLE;
	writel(reg, bch->base + JZ_REG_NAND_ECC_CTRL);

	if (status & JZ_NAND_STATUS_ERROR) {
		if (status & JZ_NAND_STATUS_UNCOR_ERROR)
			return -EBADMSG;

		error_count = (status & JZ_NAND_STATUS_ERR_COUNT) >> 29;

		for (i = 0; i < error_count; ++i) {
			error = readl(bch->base + JZ_REG_NAND_ERR(i));
			index = ((error >> 16) & 0x1ff) - 1;
			if (index >= 0 && index < params->size)
				jz_nand_correct_data(buf, index, error & 0x1ff);
		}

		return error_count;
	}

	return 0;
}

static void jz4740_bch_disable(struct jz4780_bch *bch)
{
	u32 reg;

	writel(0, bch->base + JZ_REG_NAND_IRQ_STAT);
	reg = readl(bch->base + JZ_REG_NAND_ECC_CTRL);
	reg &= ~JZ_NAND_ECC_CTRL_ENABLE;
	writel(reg, bch->base + JZ_REG_NAND_ECC_CTRL);
}

const struct jz4780_bch_ops jz4780_bch_jz4740_ops = {
	.disable = jz4740_bch_disable,
	.calculate = jz4740_bch_calculate,
	.correct = jz4740_bch_correct,
};
