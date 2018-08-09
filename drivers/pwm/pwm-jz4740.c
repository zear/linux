// SPDX-License-Identifier: GPL-2.0+
/*
 *  Copyright (C) 2010, Lars-Peter Clausen <lars@metafoo.de>
 *  JZ4740 platform PWM support
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/mfd/ingenic-tcu.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/regmap.h>

#define NUM_PWM 8

struct jz4740_pwm_chip {
	struct pwm_chip chip;
	struct clk *clks[NUM_PWM];
	struct regmap *map;
};

static inline struct jz4740_pwm_chip *to_jz4740(struct pwm_chip *chip)
{
	return container_of(chip, struct jz4740_pwm_chip, chip);
}

static int jz4740_pwm_request(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct jz4740_pwm_chip *jz = to_jz4740(chip);
	struct clk *clk;
	char clk_name[16];
	int ret;

	if (!ingenic_tcu_pwm_can_use_chn(chip->dev, pwm->hwpwm))
		return -EBUSY;

	snprintf(clk_name, sizeof(clk_name), "timer%u", pwm->hwpwm);

	clk = clk_get(chip->dev, clk_name);
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	ret = clk_prepare_enable(clk);
	if (ret) {
		clk_put(clk);
		return ret;
	}

	jz->clks[pwm->hwpwm] = clk;
	return 0;
}

static void jz4740_pwm_free(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct jz4740_pwm_chip *jz = to_jz4740(chip);
	struct clk *clk = jz->clks[pwm->hwpwm];

	clk_disable_unprepare(clk);
	clk_put(clk);
}

static int jz4740_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct jz4740_pwm_chip *jz = to_jz4740(chip);

	/* Enable PWM output */
	regmap_update_bits(jz->map, TCU_REG_TCSRc(pwm->hwpwm),
			   TCU_TCSR_PWM_EN, TCU_TCSR_PWM_EN);

	/* Start counter */
	regmap_write(jz->map, TCU_REG_TESR, BIT(pwm->hwpwm));
	return 0;
}

static void jz4740_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct jz4740_pwm_chip *jz = to_jz4740(chip);

	/* Disable PWM output.
	 * In TCU2 mode (channel 1/2 on JZ4750+), this must be done before the
	 * counter is stopped, while in TCU1 mode the order does not matter.
	 */
	regmap_update_bits(jz->map, TCU_REG_TCSRc(pwm->hwpwm),
			   TCU_TCSR_PWM_EN, 0);

	/* Stop counter */
	regmap_write(jz->map, TCU_REG_TECR, BIT(pwm->hwpwm));
}

static int jz4740_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			    struct pwm_state *state)
{
	struct jz4740_pwm_chip *jz4740 = to_jz4740(pwm->chip);
	struct clk *clk = jz4740->clks[pwm->hwpwm],
		   *parent_clk = clk_get_parent(clk);
	unsigned long rate, parent_rate, period, duty;
	unsigned long long tmp;
	int ret;

	parent_rate = clk_get_rate(parent_clk);

	jz4740_pwm_disable(chip, pwm);

	/* Reset the clock to the maximum rate, and we'll reduce it if needed */
	ret = clk_set_rate(clk, parent_rate);
	if (ret)
		return ret;

	/* Limit the clock to a maximum rate that still gives us a period value
	 * which fits in 16 bits.
	 */
	tmp = 0xffffull * NSEC_PER_SEC;
	do_div(tmp, state->period);

	ret = clk_set_max_rate(clk, tmp);
	if (ret) {
		dev_err(chip->dev, "Unable to set max rate: %i\n", ret);
		return ret;
	}

	/* Read back the clock rate, as it may have been modified by
	 * clk_set_max_rate()
	 */
	rate = clk_get_rate(clk);

	if (rate != parent_rate)
		dev_dbg(chip->dev, "PWM clock updated to %lu Hz\n", rate);

	/* Calculate period value */
	tmp = (unsigned long long)rate * state->period;
	do_div(tmp, NSEC_PER_SEC);
	period = (unsigned long)tmp;

	/* Calculate duty value */
	tmp = (unsigned long long)period * state->duty_cycle;
	do_div(tmp, state->period);
	duty = period - tmp;

	if (duty >= period)
		duty = period - 1;

	/* Set abrupt shutdown */
	regmap_update_bits(jz4740->map, TCU_REG_TCSRc(pwm->hwpwm),
			   TCU_TCSR_PWM_SD, TCU_TCSR_PWM_SD);

	/* Reset counter to 0 */
	regmap_write(jz4740->map, TCU_REG_TCNTc(pwm->hwpwm), 0);

	/* Set duty */
	regmap_write(jz4740->map, TCU_REG_TDHRc(pwm->hwpwm), duty);

	/* Set period */
	regmap_write(jz4740->map, TCU_REG_TDFRc(pwm->hwpwm), period);

	/* Set polarity */
	switch (state->polarity) {
	case PWM_POLARITY_NORMAL:
		regmap_update_bits(jz4740->map, TCU_REG_TCSRc(pwm->hwpwm),
				   TCU_TCSR_PWM_INITL_HIGH, 0);
		break;
	case PWM_POLARITY_INVERSED:
		regmap_update_bits(jz4740->map, TCU_REG_TCSRc(pwm->hwpwm),
				   TCU_TCSR_PWM_INITL_HIGH,
				   TCU_TCSR_PWM_INITL_HIGH);
		break;
	}

	if (state->enabled)
		jz4740_pwm_enable(chip, pwm);

	return 0;
}

static const struct pwm_ops jz4740_pwm_ops = {
	.request = jz4740_pwm_request,
	.free = jz4740_pwm_free,
	.apply = jz4740_pwm_apply,
	.owner = THIS_MODULE,
};

static int jz4740_pwm_probe(struct platform_device *pdev)
{
	struct jz4740_pwm_chip *jz4740;
	struct device *dev = &pdev->dev;

	jz4740 = devm_kzalloc(dev, sizeof(*jz4740), GFP_KERNEL);
	if (!jz4740)
		return -ENOMEM;

	jz4740->map = dev_get_regmap(dev->parent, NULL);
	if (!jz4740->map) {
		dev_err(dev, "regmap not found\n");
		return -EINVAL;
	}

	jz4740->chip.dev = dev;
	jz4740->chip.ops = &jz4740_pwm_ops;
	jz4740->chip.npwm = NUM_PWM;
	jz4740->chip.base = -1;
	jz4740->chip.of_xlate = of_pwm_xlate_with_flags;
	jz4740->chip.of_pwm_n_cells = 3;

	platform_set_drvdata(pdev, jz4740);

	return pwmchip_add(&jz4740->chip);
}

static int jz4740_pwm_remove(struct platform_device *pdev)
{
	struct jz4740_pwm_chip *jz4740 = platform_get_drvdata(pdev);

	return pwmchip_remove(&jz4740->chip);
}

#ifdef CONFIG_OF
static const struct of_device_id jz4740_pwm_dt_ids[] = {
	{ .compatible = "ingenic,jz4740-pwm", },
	{},
};
MODULE_DEVICE_TABLE(of, jz4740_pwm_dt_ids);
#endif

static struct platform_driver jz4740_pwm_driver = {
	.driver = {
		.name = "jz4740-pwm",
		.of_match_table = of_match_ptr(jz4740_pwm_dt_ids),
	},
	.probe = jz4740_pwm_probe,
	.remove = jz4740_pwm_remove,
};
module_platform_driver(jz4740_pwm_driver);

MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_DESCRIPTION("Ingenic JZ4740 PWM driver");
MODULE_ALIAS("platform:jz4740-pwm");
MODULE_LICENSE("GPL");
