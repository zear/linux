/*
 *  Copyright (C) 2010, Maarten ter Huurne <maarten@treewalker.org>
 *  Flash Translation Layer for media players using firmware from China Chip.
 *
 *  This initial implementation provides read-only access.
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/errno.h>
#include <linux/hdreg.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mtd/blktrans.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <mtd/mtd-abi.h>


#define SECTOR_SIZE		2048

struct cc_ftl_partition {
	struct mtd_blktrans_dev mbd;

	uint32_t *map;
};

struct cc_ftl_eb_entry {
	uint32_t virt_blk;
	uint8_t  flags;
	uint8_t  reserved[3];
	int32_t  mapped;
};

static int cc_ftl_readsect(struct mtd_blktrans_dev *dev, unsigned long block,
			   char *buffer)
{
	struct cc_ftl_partition *partition = (struct cc_ftl_partition *)dev;
	struct mtd_info *mtd = dev->mtd;
	uint64_t phy_offs;
	uint32_t phy_blk;
	uint32_t sectors_per_eb;
	size_t retlen;
	int ret;

	/* Find physical location. */
	if (block >= dev->size)
		return -EIO;

	sectors_per_eb = (mtd->erasesize / SECTOR_SIZE) - 1;

	phy_blk = partition->map[block / sectors_per_eb];
	if (phy_blk == 0xFFFFFFFF)
		return -EIO;

	phy_offs = (uint64_t)phy_blk * mtd->erasesize;
	phy_offs += (block % sectors_per_eb) * SECTOR_SIZE;
	phy_offs += SECTOR_SIZE; /* First sector of each EB is reserved */

	/* Read data. */
	ret = mtd_read(mtd, phy_offs, SECTOR_SIZE, &retlen, buffer);
	if (ret == -EUCLEAN) /* sector contains correctable errors */
		ret = 0;
	if (ret)
		return ret;
	if (retlen != SECTOR_SIZE)
		return -EIO;

	return 0;
}

uint32_t *cc_ftl_build_block_map(struct mtd_info *mtd)
{
	uint32_t num_blk = mtd_div_by_eb(mtd->size, mtd);
	uint32_t blk_found;
	struct cc_ftl_eb_entry entry;
	uint32_t *map;
	uint32_t phy_blk;

	struct mtd_oob_ops oob_ops = {
		.mode		= MTD_OPS_RAW,
		.len		= sizeof(entry),
		.ooblen		= 0,
		.datbuf		= (uint8_t *)&entry,
		.oobbuf		= NULL,
	};

	map = kmalloc(sizeof(uint32_t) * num_blk, GFP_KERNEL);
	if (!map)
		return NULL;
	memset(map, 0xFF, sizeof(uint32_t) * num_blk);

	blk_found = 0;
	for (phy_blk = 0; phy_blk < num_blk; phy_blk++) {
		loff_t ofs = (loff_t)phy_blk << mtd->erasesize_shift;
		int err;

		/*
		if (mtd_block_isbad(mtd, ofs))
			continue;
		*/

		err = mtd_read_oob(mtd, ofs, &oob_ops);
		if (err) {
			dev_err(&mtd->dev, "Unable to read from NAND: %i\n", err);
			goto err_free_map;
		}

		if (oob_ops.retlen != sizeof(entry)) {
			dev_err(&mtd->dev, "Unable to read FTL entry\n");
			goto err_free_map;
		}

		if (entry.mapped != 0 && entry.mapped != 0xFFFFFFFF) {
			dev_err(&mtd->dev, "Found non-0 non-F entry\n");
			goto err_free_map;
		}

		if (!entry.mapped)
			continue;

		if (entry.virt_blk >= num_blk) {
			dev_warn(&mtd->dev, "Physical block %d claims "
					"logical block %d which is beyond "
					"partition end %d\n",
					phy_blk, entry.virt_blk, num_blk);
			continue;
		}
		if (map[entry.virt_blk] != 0xFFFFFFFF) {
			dev_warn(&mtd->dev, "Physical block %d and %d both "
					"claim logical block %d\n",
					map[entry.virt_blk], phy_blk,
					entry.virt_blk);
			continue;
		}

		map[entry.virt_blk] = phy_blk;
		blk_found++;
	}

	if (blk_found == 0) {
		kfree(map);
		return NULL;
	}

	return map;

err_free_map:
	kfree(map);
	return NULL;
}

static void cc_ftl_add_mtd(struct mtd_blktrans_ops *tr, struct mtd_info *mtd)
{
	struct cc_ftl_partition *partition;
	uint32_t *map;
	int err;

	/* Check for NAND first, so we know we can use the "chip" pointer. */
	if (mtd->type != MTD_NANDFLASH)
		return;

	/* A bad block table is expected. */
	/*
	if (!mtd->block_isbad)
		return;
		*/

	/* Erase size must be a power of two. */
	if (mtd->erasesize_shift == 0) {
		dev_err(&mtd->dev, "Erase size must be a power of two\n");
		return;
	}

	/* Erase size must be a multiple of sector size. */
	if ((mtd->erasesize & (SECTOR_SIZE - 1)) != 0) {
		dev_err(&mtd->dev, "Erase size must be a multiple of sector size\n");
		return;
	}

	map = cc_ftl_build_block_map(mtd);
	if (!map) {
		dev_err(&mtd->dev, "Failed to build block map\n");
		return;
	}

	partition = kzalloc(sizeof(struct cc_ftl_partition), GFP_KERNEL);
	if (!partition)
		goto err_map;

	partition->mbd.mtd = mtd;
	// TODO: More reasonable guess.
	partition->mbd.size = mtd->size / SECTOR_SIZE;
	partition->mbd.tr = tr;
	partition->mbd.devnum = -1;
	partition->mbd.readonly = 1;
	partition->map = map;

	err = add_mtd_blktrans_dev((struct mtd_blktrans_dev *)partition);
	if (err) {
		dev_err(&mtd->dev, "Unable to add partitions: %i\n", err);
		goto err_partition;
	}

	return;

err_partition:
	kfree(partition);
err_map:
	kfree(map);
}

static void cc_ftl_remove_dev(struct mtd_blktrans_dev *dev)
{
	struct cc_ftl_partition *partition = (struct cc_ftl_partition *)dev;

	if (del_mtd_blktrans_dev(dev))
		return;

	kfree(partition->map);
	kfree(partition);
}

static struct mtd_blktrans_ops cc_ftl_tr = {
	.name		= "rs90nand",
	.major		= 242,	/* TODO: Register an official major number. */
	.part_bits	= 1,
	.blksize 	= SECTOR_SIZE,

	.readsect	= cc_ftl_readsect,
	.add_mtd	= cc_ftl_add_mtd,
	.remove_dev	= cc_ftl_remove_dev,

	.owner		= THIS_MODULE,
};

static int __init cc_ftl_init(void)
{
	return register_mtd_blktrans(&cc_ftl_tr);
}
module_init(cc_ftl_init);

static void __exit cc_ftl_exit(void)
{
	deregister_mtd_blktrans(&cc_ftl_tr);
}
module_exit(cc_ftl_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Maarten ter Huurne <maarten@treewalker.org>");
MODULE_DESCRIPTION("Flash Translation Layer for media players "
		   "using firmware from China Chip");
