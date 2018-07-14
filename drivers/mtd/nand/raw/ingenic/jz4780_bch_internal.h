/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __DRIVERS_MTD_NAND_JZ4780_BCH_INTERNAL_H__
#define __DRIVERS_MTD_NAND_JZ4780_BCH_INTERNAL_H__

#include <linux/compiler_types.h>
#include <linux/mutex.h>
#include <linux/types.h>

struct jz4780_bch_params;
struct jz4780_bch;
struct device;
struct clk;

struct jz4780_bch_ops {
	int (*probe)(struct jz4780_bch *bch);
	void (*disable)(struct jz4780_bch *bch);
	int (*calculate)(struct jz4780_bch *bch,
			 struct jz4780_bch_params *params,
			 const u8 *buf, u8 *ecc_code);
	int (*correct)(struct jz4780_bch *bch,
			struct jz4780_bch_params *params,
			u8 *buf, u8 *ecc_code);
};

struct jz4780_bch {
	struct device *dev;
	const struct jz4780_bch_ops *ops;
	void __iomem *base;
	struct clk *clk;
	struct mutex lock;
};

extern const struct jz4780_bch_ops jz4780_bch_jz4780_ops;

#endif /* __DRIVERS_MTD_NAND_JZ4780_BCH_INTERNAL_H__ */
