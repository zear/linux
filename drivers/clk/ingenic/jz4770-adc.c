// SPDX-License-Identifier: GPL-2.0
/*
 * Ingenic JZ4770 ADC clock driver
 * (C) 2018, Paul Cercueil <paul@crapouillou.net>
 */

#include <linux/bitops.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

static const struct {
	const char *name;
	const char *parent_name;
	unsigned int width;
	unsigned int shift;
} jz4770_adc_clocks[] = {
	{ "adc", "adc-gate", 8, 0, },
	{ "adc-ms", "adc", 16, 16, },
	{ "adc-us", "adc", 8, 8, },
};

static int jz4770_adc_clock_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *mem;
	void __iomem *reg;
	unsigned int i;
	spinlock_t *lock;
	int err;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	reg = devm_ioremap_resource(dev, mem);
	if (IS_ERR(reg))
		return PTR_ERR(reg);

	lock = devm_kzalloc(dev, sizeof(*lock), GFP_KERNEL);
	if (!lock)
		return -ENOMEM;

	spin_lock_init(lock);

	for (i = 0; i < ARRAY_SIZE(jz4770_adc_clocks); i++) {
		struct clk_hw *hw = clk_hw_register_divider(dev,
				jz4770_adc_clocks[i].name,
				jz4770_adc_clocks[i].parent_name,
				CLK_OPS_PARENT_ENABLE, reg,
				jz4770_adc_clocks[i].shift,
				jz4770_adc_clocks[i].width, 0, lock);
		if (IS_ERR(hw))
		    return PTR_ERR(hw);

		devm_add_action(dev, (void (*)(void *)) clk_hw_unregister_divider, hw);

		err = clk_hw_register_clkdev(hw, "adc", NULL);
		if (err)
			return err;

		err = devm_of_clk_add_hw_provider(dev, of_clk_hw_simple_get, hw);
		if (err)
			return err;
	}

	return 0;
}

static const struct of_device_id jz4770_adc_clock_of_match[] = {
	{ .compatible = "ingenic,jz4770-adc-clock" },
	{},
};
MODULE_DEVICE_TABLE(of, jz4770_adc_clock_of_match);

static struct platform_driver jz4770_adc_clock_driver = {
	.probe	 = jz4770_adc_clock_probe,
	.driver	 = {
		.name  = "jz4770-adc-clock",
		.of_match_table = of_match_ptr(jz4770_adc_clock_of_match),
	},
};
module_platform_driver(jz4770_adc_clock_driver);
