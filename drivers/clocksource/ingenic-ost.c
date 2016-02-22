// SPDX-License-Identifier: GPL-2.0
/*
 * JZ47xx SoCs TCU Operating System Timer driver
 *
 * Copyright (C) 2016 Maarten ter Huurne <maarten@treewalker.org>
 * Copyright (C) 2018 Paul Cercueil <paul@crapouillou.net>
 */

#include <linux/clk.h>
#include <linux/clocksource.h>
#include <linux/mfd/ingenic-tcu.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/regmap.h>
#include <linux/sched_clock.h>

#define TCU_OST_TCSR_MASK	0xffc0
#define TCU_OST_TCSR_CNT_MD	BIT(15)

#define TCU_OST_CHANNEL		15

struct ingenic_ost_soc_info {
	bool is64bit;
};

struct ingenic_ost {
	struct regmap *map;
	struct clk *clk;

	struct clocksource cs;
};

static struct ingenic_ost *ingenic_ost;

static u64 notrace ingenic_ost_read_cntl(void)
{
	u32 val;

	regmap_read(ingenic_ost->map, TCU_REG_OST_CNTL, &val);

	return val;
}

static u64 notrace ingenic_ost_read_cnth(void)
{
	u32 val;

	regmap_read(ingenic_ost->map, TCU_REG_OST_CNTH, &val);

	return val;
}

static u64 notrace ingenic_ost_clocksource_read64(struct clocksource *cs)
{
	u32 val1, val2;
	u64 count, recount;
	s64 diff;

	/*
	 * The buffering of the upper 32 bits of the timer prevents wrong
	 * results from the bottom 32 bits overflowing due to the timer ticking
	 * along. However, it does not prevent wrong results from simultaneous
	 * reads of the timer, which could reset the buffer mid-read.
	 * Since this kind of wrong read can happen only when the bottom bits
	 * overflow, there will be minutes between wrong reads, so if we read
	 * twice in succession, at least one of the reads will be correct.
	 */

	/* Bypass the regmap here as we must return as soon as possible */
	regmap_read(ingenic_ost->map, TCU_REG_OST_CNTL, &val1);
	regmap_read(ingenic_ost->map, TCU_REG_OST_CNTHBUF, &val2);
	count = (u64)val1 | (u64)val2 << 32;

	regmap_read(ingenic_ost->map, TCU_REG_OST_CNTL, &val1);
	regmap_read(ingenic_ost->map, TCU_REG_OST_CNTHBUF, &val2);
	recount = (u64)val1 | (u64)val2 << 32;

	/*
	 * A wrong read will produce a result that is 1<<32 too high: the bottom
	 * part from before overflow and the upper part from after overflow.
	 * Therefore, the lower value of the two reads is the correct value.
	 */

	diff = (s64)(recount - count);
	if (unlikely(diff < 0))
		count = recount;

	return count;
}

static u64 notrace ingenic_ost_clocksource_read32(struct clocksource *cs)
{
	return ingenic_ost_read_cnth();
}

static int __init ingenic_ost_probe(struct platform_device *pdev)
{
	const struct ingenic_ost_soc_info *soc_info;
	struct device *dev = &pdev->dev;
	struct ingenic_ost *ost;
	struct clocksource *cs;
	unsigned long rate, flags;
	int err;

	soc_info = device_get_match_data(dev);
	if (!soc_info)
		return -EINVAL;

	ost = devm_kzalloc(dev, sizeof(*ost), GFP_KERNEL);
	if (!ost)
		return -ENOMEM;

	ingenic_ost = ost;

	ost->map = device_node_to_regmap(dev->parent->of_node);
	if (!ost->map) {
		dev_err(dev, "regmap not found\n");
		return -EINVAL;
	}

	ost->clk = devm_clk_get(dev, "ost");
	if (IS_ERR(ost->clk))
		return PTR_ERR(ost->clk);

	err = clk_prepare_enable(ost->clk);
	if (err)
		return err;

	/* Clear counter high/low registers */
	if (soc_info->is64bit)
		regmap_write(ost->map, TCU_REG_OST_CNTL, 0);
	regmap_write(ost->map, TCU_REG_OST_CNTH, 0);

	/* Don't reset counter at compare value. */
	regmap_update_bits(ost->map, TCU_REG_OST_TCSR,
			   TCU_OST_TCSR_MASK, TCU_OST_TCSR_CNT_MD);

	rate = clk_get_rate(ost->clk);

	/* Enable OST TCU channel */
	regmap_write(ost->map, TCU_REG_TESR, BIT(TCU_OST_CHANNEL));

	cs = &ost->cs;
	cs->name	= "ingenic-ost";
	cs->rating	= 320;
	cs->flags	= CLOCK_SOURCE_IS_CONTINUOUS;

	if (soc_info->is64bit) {
		cs->mask = CLOCKSOURCE_MASK(64);
		cs->read = ingenic_ost_clocksource_read64;
	} else {
		cs->mask = CLOCKSOURCE_MASK(32);
		cs->read = ingenic_ost_clocksource_read32;
	}

	err = clocksource_register_hz(cs, rate);
	if (err) {
		dev_err(dev, "clocksource registration failed: %d\n", err);
		clk_disable_unprepare(ost->clk);
		return err;
	}

	/* Cannot register a sched_clock with interrupts on */
	local_irq_save(flags);
	if (soc_info->is64bit)
		sched_clock_register(ingenic_ost_read_cntl, 32, rate);
	else
		sched_clock_register(ingenic_ost_read_cnth, 32, rate);
	local_irq_restore(flags);

	return 0;
}

static int __maybe_unused ingenic_ost_suspend(struct device *dev)
{
	struct ingenic_ost *ost = dev_get_drvdata(dev);

	clk_disable(ost->clk);

	return 0;
}

static int __maybe_unused ingenic_ost_resume(struct device *dev)
{
	struct ingenic_ost *ost = dev_get_drvdata(dev);

	return clk_enable(ost->clk);
}

static const struct dev_pm_ops __maybe_unused ingenic_ost_pm_ops = {
	/* _noirq: We want the OST clock to be gated last / ungated first */
	.suspend_noirq = ingenic_ost_suspend,
	.resume_noirq  = ingenic_ost_resume,
};

static const struct ingenic_ost_soc_info jz4725b_ost_soc_info = {
	.is64bit = false,
};

static const struct ingenic_ost_soc_info jz4770_ost_soc_info = {
	.is64bit = true,
};

static const struct of_device_id ingenic_ost_of_match[] = {
	{ .compatible = "ingenic,jz4725b-ost", .data = &jz4725b_ost_soc_info, },
	{ .compatible = "ingenic,jz4770-ost", .data = &jz4770_ost_soc_info, },
	{ }
};

static struct platform_driver ingenic_ost_driver = {
	.driver = {
		.name = "ingenic-ost",
#ifdef CONFIG_PM_SUSPEND
		.pm = &ingenic_ost_pm_ops,
#endif
		.of_match_table = ingenic_ost_of_match,
	},
};
builtin_platform_driver_probe(ingenic_ost_driver, ingenic_ost_probe);
