// SPDX-License-Identifier: GPL-2.0
/*
 * JZ4780 backend code for the jz4780-bch driver
 *
 * Copyright (C) 2018 Paul Cercueil <paul@crapouillou.net>
 *
 * Based on jz4780_bch.c
 */

#include <linux/bitops.h>
#include <linux/iopoll.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/device.h>

#include "jz4780_bch.h"
#include "jz4780_bch_internal.h"

#define BCH_BHCR			0x0
#define BCH_BHCSR			0x4
#define BCH_BHCCR			0x8
#define BCH_BHCNT			0xc
#define BCH_BHDR			0x10
#define BCH_BHPAR0			0x14
#define BCH_BHERR0			0x28
#define BCH_BHINT			0x24
#define BCH_BHINTES			0x3c
#define BCH_BHINTEC			0x40
#define BCH_BHINTE			0x38

#define BCH_BHCR_BSEL_SHIFT		2
#define BCH_BHCR_BSEL_MASK		(0x1 << BCH_BHCR_BSEL_SHIFT)
#define BCH_BHCR_ENCE			BIT(3)
#define BCH_BHCR_INIT			BIT(1)
#define BCH_BHCR_BCHE			BIT(0)

#define BCH_BHCNT_DEC_COUNT_SHIFT	16
#define BCH_BHCNT_DEC_COUNT_MASK	(0x3ff << BCH_BHCNT_DEC_COUNT_SHIFT)
#define BCH_BHCNT_ENC_COUNT_SHIFT	0
#define BCH_BHCNT_ENC_COUNT_MASK	(0x3ff << BCH_BHCNT_ENC_COUNT_SHIFT)

#define BCH_BHERR_INDEX0_SHIFT		0
#define BCH_BHERR_INDEX0_MASK		(0x1fff << BCH_BHERR_INDEX0_SHIFT)
#define BCH_BHERR_INDEX1_SHIFT		16
#define BCH_BHERR_INDEX1_MASK		(0x1fff << BCH_BHERR_INDEX1_SHIFT)

#define BCH_BHINT_ERRC_SHIFT		28
#define BCH_BHINT_ERRC_MASK		(0xf << BCH_BHINT_ERRC_SHIFT)
#define BCH_BHINT_TERRC_SHIFT		16
#define BCH_BHINT_TERRC_MASK		(0x7f << BCH_BHINT_TERRC_SHIFT)
#define BCH_BHINT_ALL_0			BIT(5)
#define BCH_BHINT_ALL_F			BIT(4)
#define BCH_BHINT_DECF			BIT(3)
#define BCH_BHINT_ENCF			BIT(2)
#define BCH_BHINT_UNCOR			BIT(1)
#define BCH_BHINT_ERR			BIT(0)

/* Timeout for BCH calculation/correction. */
#define BCH_TIMEOUT_US			100000

static void jz4725b_bch_init(struct jz4780_bch *bch,
			     struct jz4780_bch_params *params, bool encode)
{
	u32 reg;

	/* Clear interrupt status. */
	writel(readl(bch->base + BCH_BHINT), bch->base + BCH_BHINT);

	/* Initialise and enable BCH. */
	writel(0x1f, bch->base + BCH_BHCCR);
	writel(BCH_BHCR_BCHE, bch->base + BCH_BHCSR);

	if (params->strength == 8)
		writel(BCH_BHCR_BSEL_MASK, bch->base + BCH_BHCSR);
	else
		writel(BCH_BHCR_BSEL_MASK, bch->base + BCH_BHCCR);

	if (encode)
		writel(BCH_BHCR_ENCE, bch->base + BCH_BHCSR);
	else
		writel(BCH_BHCR_ENCE, bch->base + BCH_BHCCR);

	writel(BCH_BHCR_INIT, bch->base + BCH_BHCSR);

	/* Set up BCH count register. */
	reg = params->size << BCH_BHCNT_ENC_COUNT_SHIFT;
	reg |= (params->size + params->bytes) << BCH_BHCNT_DEC_COUNT_SHIFT;
	writel(reg, bch->base + BCH_BHCNT);
}

static void jz4725b_bch_disable(struct jz4780_bch *bch)
{
	writel(readl(bch->base + BCH_BHINT), bch->base + BCH_BHINT);
	writel(BCH_BHCR_BCHE, bch->base + BCH_BHCCR);
}

static void jz4725b_bch_write_data(struct jz4780_bch *bch, const u8 *buf,
				   size_t size)
{
	while (size--)
		writeb(*buf++, bch->base + BCH_BHDR);
}

static void jz4725b_bch_read_parity(struct jz4780_bch *bch, u8 *buf,
				    size_t size)
{
	size_t size32 = size / sizeof(u32);
	size_t size8 = size % sizeof(u32);
	u32 *dest32;
	u8 *dest8;
	u32 val, offset = 0;

	dest32 = (u32 *)buf;
	while (size32--) {
		*dest32++ = readl(bch->base + BCH_BHPAR0 + offset);
		offset += sizeof(u32);
	}

	dest8 = (u8 *)dest32;
	val = readl(bch->base + BCH_BHPAR0 + offset);
	switch (size8) {
	case 3:
		dest8[2] = (val >> 16) & 0xff;
	case 2:
		dest8[1] = (val >> 8) & 0xff;
	case 1:
		dest8[0] = val & 0xff;
		break;
	}
}

static bool jz4725b_bch_wait_complete(struct jz4780_bch *bch, unsigned int irq,
				     u32 *status)
{
	u32 reg;
	int ret;

	/*
	 * While we could use interrupts here and sleep until the operation
	 * completes, the controller works fairly quickly (usually a few
	 * microseconds) and so the overhead of sleeping until we get an
	 * interrupt quite noticeably decreases performance.
	 */
	ret = readl_poll_timeout(bch->base + BCH_BHINT, reg,
				 (reg & irq) == irq, 0, BCH_TIMEOUT_US);
	if (ret)
		return false;

	if (status)
		*status = reg;

	writel(reg, bch->base + BCH_BHINT);
	return true;
}

static int jz4725b_calculate(struct jz4780_bch *bch,
			     struct jz4780_bch_params *params,
			     const u8 *buf, u8 *ecc_code)
{
	int ret = 0;

	mutex_lock(&bch->lock);
	jz4725b_bch_init(bch, params, true);
	jz4725b_bch_write_data(bch, buf, params->size);

	if (jz4725b_bch_wait_complete(bch, BCH_BHINT_ENCF, NULL)) {
		jz4725b_bch_read_parity(bch, ecc_code, params->bytes);
	} else {
		dev_err(bch->dev, "timed out while calculating ECC\n");
		ret = -ETIMEDOUT;
	}

	jz4725b_bch_disable(bch);
	mutex_unlock(&bch->lock);
	return ret;
}

static int jz4725b_correct(struct jz4780_bch *bch,
			   struct jz4780_bch_params *params,
			   u8 *buf, u8 *ecc_code)
{
	u32 reg, errors, bit;
	unsigned int i;
	int ret = 0;

	mutex_lock(&bch->lock);

	jz4725b_bch_init(bch, params, false);
	jz4725b_bch_write_data(bch, buf, params->size);
	jz4725b_bch_write_data(bch, ecc_code, params->bytes);

	if (!jz4725b_bch_wait_complete(bch, BCH_BHINT_DECF, &reg)) {
		dev_err(bch->dev, "timed out while correcting data\n");
		ret = -ETIMEDOUT;
		goto out;
	}

	if (reg & (BCH_BHINT_ALL_F | BCH_BHINT_ALL_0)) {
		/* Data and ECC is all 0xff or 0x00 - nothing to correct */
		ret = 0;
		goto out;
	}

	if (reg & BCH_BHINT_UNCOR) {
		/* Uncorrectable ECC error */
		ret = -EBADMSG;
		goto out;
	}

	errors = (reg & BCH_BHINT_ERRC_MASK) >> BCH_BHINT_ERRC_SHIFT;

	/* Correct any detected errors. */
	for (i = 0; i < errors; i++) {
		if (i & 1) {
			bit = (reg & BCH_BHERR_INDEX1_MASK) >> BCH_BHERR_INDEX1_SHIFT;
		} else {
			reg = readl(bch->base + BCH_BHERR0 + (i * 4));
			bit = (reg & BCH_BHERR_INDEX0_MASK) >> BCH_BHERR_INDEX0_SHIFT;
		}

		buf[(bit >> 3)] ^= BIT(bit & 0x7);
	}

out:
	jz4725b_bch_disable(bch);
	mutex_unlock(&bch->lock);
	return ret;
}

const struct jz4780_bch_ops jz4780_bch_jz4725b_ops = {
	.disable = jz4725b_bch_disable,
	.calculate = jz4725b_calculate,
	.correct = jz4725b_correct,
};
