/*
 * SPI bus driver for the Ingenic JZ4xx SoCs
 *
 * Copyright (c) 2017-2018 Artur Rojek <contact@artur-rojek.eu>
 *
 * Licensed under GPLv2 or later.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>

#define REG_SSIDR	0x0
#define REG_SSICR0	0x4
#define REG_SSICR1	0x8
#define REG_SSISR	0xC
#define REG_SSITR	0x10
#define REG_SSICR	0x14
#define REG_SSIGR	0x18

#define REG_SSICR1_FLEN_OFFSET	0x3
#define REG_SSICR1_FRMHL	BIT(30)
#define REG_SSICR1_PHA		BIT(1)
#define REG_SSICR1_POL		BIT(0)

#define REG_SSISR_END		BIT(7)
#define REG_SSISR_TFF		BIT(5)
#define REG_SSISR_RFE		BIT(4)

#define REG_SSICR0_TENDIAN_LSB	(BIT(18) | BIT(19))
#define REG_SSICR0_RENDIAN_LSB	(BIT(16) | BIT(17))
#define REG_SSICR0_SSIE		BIT(15)
#define REG_SSICR0_LOOP		BIT(10)
#define REG_SSICR0_TFLUSH	BIT(2)
#define REG_SSICR0_RFLUSH	BIT(1)

struct ingenic_spi {
	struct clk *clk;
	void __iomem *base;
};

static const struct of_device_id spi_ingenic_of_match[] = {
	{ .compatible = "ingenic,ingenic-spi"},
	{}
};
MODULE_DEVICE_TABLE(of, spi_ingenic_of_match);

static void spi_ingenic_wait(struct ingenic_spi *spi, unsigned long mask)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(5);

	do {
		if (!(readl(spi->base + REG_SSISR) & mask))
			return;
	} while (!time_after(jiffies, timeout));
}

static void spi_ingenic_set_cs(struct spi_device *spi, bool enable)
{
	struct spi_master *master = spi->master;
	int gpio;

	gpio = master->cs_gpios[spi->chip_select];

	if (gpio >= 0)
		gpio_set_value(gpio, enable);
}

static int spi_ingenic_transfer_one(struct spi_master *master,
					struct spi_device *spi,
					struct spi_transfer *transfer)
{
	struct ingenic_spi *ingenic_spi = spi_master_get_devdata(master);
	unsigned int bits = spi->bits_per_word;
	unsigned int i, count;
	unsigned long clk_hz = clk_get_rate(ingenic_spi->clk);
	u32 val;
	u8 cdiv;

	if (transfer->speed_hz >= clk_hz/2)
		cdiv = 0; /* max_speed_hz/2 is the fastest we can go. */
	else if (transfer->speed_hz) {
		unsigned long cdivl = (clk_hz / (transfer->speed_hz * 2)) - 1;
		cdiv = cdivl > 255 ? 255 : cdivl; /* 255 is the slowest we can go. */
	}
	else
		cdiv = 255; /* 255 is the slowest we can go. */

	writeb(cdiv, ingenic_spi->base + REG_SSIGR);

	if (spi->bits_per_word <= 8)
		count = transfer->len;
	else
		count = transfer->len / 2;

	for (i = 0; i < count; ++i) {
		if (transfer->tx_buf) {
			spi_ingenic_wait(ingenic_spi, REG_SSISR_TFF);

			val = (bits <= 8) ?
				((u8 *)transfer->tx_buf)[i] :
				((u16 *)transfer->tx_buf)[i];
			writel(val, ingenic_spi->base + REG_SSIDR);
		}

		if (transfer->rx_buf) {
			spi_ingenic_wait(ingenic_spi, REG_SSISR_RFE);

			val = readl(ingenic_spi->base + REG_SSIDR);
			if (bits <= 8)
				((u8 *)transfer->rx_buf)[i] = val;
			else
				((u16 *)transfer->rx_buf)[i] = val;
		}
	}

	spi_ingenic_wait(ingenic_spi, REG_SSISR_END);

	return 0;
}

static int spi_ingenic_setup(struct spi_device *spi)
{
	struct spi_master *master = spi->master;
	struct ingenic_spi *ingenic_spi = spi_master_get_devdata(master);
	u32 ctrl;
	int ret;

	if (spi->bits_per_word < 2 || spi->bits_per_word > 16)
		return -EINVAL;

	ret = clk_prepare_enable(ingenic_spi->clk);
	if (ret)
		return ret;

	ctrl = (spi->bits_per_word - 2) << REG_SSICR1_FLEN_OFFSET;

	if (spi->mode & SPI_CPHA)
		ctrl |= REG_SSICR1_PHA;

	if (spi->mode & SPI_CPOL)
		ctrl |= REG_SSICR1_POL;

	if (spi->mode & SPI_CS_HIGH)
		ctrl |= (REG_SSICR1_FRMHL + spi->chip_select);

	writel(ctrl, ingenic_spi->base + REG_SSICR1);

	ctrl = REG_SSICR0_SSIE | REG_SSICR0_TFLUSH | REG_SSICR0_RFLUSH; 

	if (spi->mode & SPI_LSB_FIRST)
		ctrl |= REG_SSICR0_RENDIAN_LSB | REG_SSICR0_TENDIAN_LSB;

	if (spi->mode & SPI_LOOP)
		ctrl |= REG_SSICR0_LOOP;

	writel(ctrl, ingenic_spi->base + REG_SSICR0);

	return 0;
}

static void spi_ingenic_cleanup(struct spi_device *spi)
{
	struct spi_master *master = spi->master;
	struct ingenic_spi *ingenic_spi = spi_master_get_devdata(master);

	writel(0, ingenic_spi->base + REG_SSICR0);
	clk_disable_unprepare(ingenic_spi->clk);
}

static int spi_ingenic_probe(struct platform_device *pdev)
{
	struct ingenic_spi *ingenic_spi;
	struct spi_master *master;
	int ret;

	master = spi_alloc_master(&pdev->dev, sizeof(struct ingenic_spi));
	if (!master) {
		dev_err(&pdev->dev, "Unable to allocate SPI master.\n");
		return -ENOMEM;
	}

	ingenic_spi = spi_master_get_devdata(master);

	ingenic_spi->clk = devm_clk_get(&pdev->dev, "spi");
	if (IS_ERR(ingenic_spi->clk)) {
		dev_err(&pdev->dev, "Clock not found.\n");
		spi_master_put(master);
		return PTR_ERR(ingenic_spi->clk);
	}

	ingenic_spi->base = devm_ioremap_resource(&pdev->dev,
			platform_get_resource(pdev, IORESOURCE_MEM, 0));
	if (IS_ERR(ingenic_spi->base)) {
		spi_master_put(master);

		return PTR_ERR(ingenic_spi->base);
	}

	platform_set_drvdata(pdev, master);

	master->setup = spi_ingenic_setup;
	master->cleanup = spi_ingenic_cleanup;
	master->mode_bits = SPI_MODE_3 | SPI_CS_HIGH | SPI_LSB_FIRST | SPI_LOOP;
	master->set_cs = spi_ingenic_set_cs;
	master->transfer_one = spi_ingenic_transfer_one;
	master->dev.of_node = pdev->dev.of_node;

	ret = devm_spi_register_master(&pdev->dev, master);
	if (ret) {
		dev_err(&pdev->dev, "Unable to register SPI master.\n");
		spi_master_put(master);
	}

	return ret;
}

static int spi_ingenic_remove(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);

	spi_master_put(master);

	return 0;
}

static struct platform_driver spi_ingenic_driver = {
	.driver = {
		.name = "spi-ingenic",
		.of_match_table = spi_ingenic_of_match,
	},
	.probe = spi_ingenic_probe,
	.remove = spi_ingenic_remove,
};
module_platform_driver(spi_ingenic_driver);
