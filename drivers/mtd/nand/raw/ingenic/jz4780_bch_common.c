// SPDX-License-Identifier: GPL-2.0
/*
 * JZ4780 BCH controller
 *
 * Copyright (c) 2015 Imagination Technologies
 * Author: Alex Smith <alex.smith@imgtec.com>
 */

#include <linux/clk.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

#include "jz4780_bch_internal.h"
#include "jz4780_bch.h"

/**
 * jz4780_bch_calculate() - calculate ECC for a data buffer
 * @bch: BCH device.
 * @params: BCH parameters.
 * @buf: input buffer with raw data.
 * @ecc_code: output buffer with ECC.
 *
 * Return: 0 on success, -ETIMEDOUT if timed out while waiting for BCH
 * controller.
 */
int jz4780_bch_calculate(struct jz4780_bch *bch, struct jz4780_bch_params *params,
			 const u8 *buf, u8 *ecc_code)
{
	return bch->ops->calculate(bch, params, buf, ecc_code);
}
EXPORT_SYMBOL(jz4780_bch_calculate);

/**
 * jz4780_bch_correct() - detect and correct bit errors
 * @bch: BCH device.
 * @params: BCH parameters.
 * @buf: raw data read from the chip.
 * @ecc_code: ECC read from the chip.
 *
 * Given the raw data and the ECC read from the NAND device, detects and
 * corrects errors in the data.
 *
 * Return: the number of bit errors corrected, -EBADMSG if there are too many
 * errors to correct or -ETIMEDOUT if we timed out waiting for the controller.
 */
int jz4780_bch_correct(struct jz4780_bch *bch, struct jz4780_bch_params *params,
		       u8 *buf, u8 *ecc_code)
{
	return bch->ops->correct(bch, params, buf, ecc_code);
}
EXPORT_SYMBOL(jz4780_bch_correct);

/**
 * jz4780_bch_get() - get the BCH controller device
 * @np: BCH device tree node.
 *
 * Gets the BCH controller device from the specified device tree node. The
 * device must be released with jz4780_bch_release() when it is no longer being
 * used.
 *
 * Return: a pointer to jz4780_bch, errors are encoded into the pointer.
 * PTR_ERR(-EPROBE_DEFER) if the device hasn't been initialised yet.
 */
static struct jz4780_bch *jz4780_bch_get(struct device_node *np)
{
	struct platform_device *pdev;
	struct jz4780_bch *bch;

	pdev = of_find_device_by_node(np);
	if (!pdev || !platform_get_drvdata(pdev))
		return ERR_PTR(-EPROBE_DEFER);

	get_device(&pdev->dev);

	bch = platform_get_drvdata(pdev);
	clk_prepare_enable(bch->clk);

	return bch;
}

/**
 * of_jz4780_bch_get() - get the BCH controller from a DT node
 * @of_node: the node that contains a bch-controller property.
 *
 * Get the bch-controller property from the given device tree
 * node and pass it to jz4780_bch_get to do the work.
 *
 * Return: a pointer to jz4780_bch, errors are encoded into the pointer.
 * PTR_ERR(-EPROBE_DEFER) if the device hasn't been initialised yet.
 */
struct jz4780_bch *of_jz4780_bch_get(struct device_node *of_node)
{
	struct jz4780_bch *bch = NULL;
	struct device_node *np;

	np = of_parse_phandle(of_node, "ingenic,bch-controller", 0);

	if (np) {
		bch = jz4780_bch_get(np);
		of_node_put(np);
	}
	return bch;
}
EXPORT_SYMBOL(of_jz4780_bch_get);

/**
 * jz4780_bch_release() - release the BCH controller device
 * @bch: BCH device.
 */
void jz4780_bch_release(struct jz4780_bch *bch)
{
	clk_disable_unprepare(bch->clk);
	put_device(bch->dev);
}
EXPORT_SYMBOL(jz4780_bch_release);

static int jz4780_bch_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct jz4780_bch *bch;
	struct resource *res;
	int ret = 0;

	bch = devm_kzalloc(dev, sizeof(*bch), GFP_KERNEL);
	if (!bch)
		return -ENOMEM;

	bch->ops = device_get_match_data(dev);
	if (!bch->ops)
		return -EINVAL;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	bch->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(bch->base))
		return PTR_ERR(bch->base);

	bch->ops->disable(bch);

	bch->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(bch->clk)) {
		dev_err(dev, "failed to get clock: %ld\n", PTR_ERR(bch->clk));
		return PTR_ERR(bch->clk);
	}

	mutex_init(&bch->lock);

	bch->dev = dev;
	platform_set_drvdata(pdev, bch);

	if (bch->ops->probe)
		ret = bch->ops->probe(bch);
	
	return ret;
}

static const struct of_device_id jz4780_bch_dt_match[] = {
	{ .compatible = "ingenic,jz4740-bch", .data = &jz4780_bch_jz4740_ops},
	{ .compatible = "ingenic,jz4725b-bch", .data = &jz4780_bch_jz4725b_ops},
	{ .compatible = "ingenic,jz4780-bch", .data = &jz4780_bch_jz4780_ops },
	{},
};
MODULE_DEVICE_TABLE(of, jz4780_bch_dt_match);

static struct platform_driver jz4780_bch_driver = {
	.probe		= jz4780_bch_probe,
	.driver	= {
		.name	= "jz4780-bch",
		.of_match_table = of_match_ptr(jz4780_bch_dt_match),
	},
};
module_platform_driver(jz4780_bch_driver);

MODULE_AUTHOR("Alex Smith <alex@alex-smith.me.uk>");
MODULE_AUTHOR("Harvey Hunt <harveyhuntnexus@gmail.com>");
MODULE_DESCRIPTION("Ingenic JZ4780 BCH error correction driver");
MODULE_LICENSE("GPL v2");
