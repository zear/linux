// SPDX-License-Identifier: GPL-2.0
/*
 * Ingenic JZ4770 USB PHY driver
 * Copyright (c) Paul Cercueil <paul@crapouillou.net>
 */

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/usb/otg.h>
#include <linux/usb/phy.h>

#define REG_USBPCR_OFFSET	0x00
#define REG_USBRDT_OFFSET	0x04
#define REG_USBVBFIL_OFFSET	0x08
#define REG_USBPCR1_OFFSET	0x0c

/* USBPCR */
#define USBPCR_USB_MODE		BIT(31)
#define USBPCR_AVLD_REG		BIT(30)
#define USBPCR_INCRM		BIT(27)	/* INCR_MASK bit */
#define USBPCR_CLK12_EN		BIT(26)
#define USBPCR_COMMONONN	BIT(25)
#define USBPCR_VBUSVLDEXT	BIT(24)
#define USBPCR_VBUSVLDEXTSEL	BIT(23)
#define USBPCR_POR		BIT(22)
#define USBPCR_SIDDQ		BIT(21)
#define USBPCR_OTG_DISABLE	BIT(20)
#define USBPCR_TXPREEMPHTUNE	BIT(6)

#define USBPCR_IDPULLUP_LSB	28	/* IDPULLUP_MASK bit */
#define USBPCR_IDPULLUP_MASK	GENMASK(29, USBPCR_IDPULLUP_LSB)
#define USBPCR_IDPULLUP_ALWAYS	(3 << USBPCR_IDPULLUP_LSB)
#define USBPCR_IDPULLUP_SUSPEND	(1 << USBPCR_IDPULLUP_LSB)
#define USBPCR_IDPULLUP_OTG	(0 << USBPCR_IDPULLUP_LSB)

#define USBPCR_COMPDISTUNE_LSB	17
#define USBPCR_COMPDISTUNE_MASK	GENMASK(19, USBPCR_COMPDISTUNE_LSB)

#define USBPCR_OTGTUNE_LSB	14
#define USBPCR_OTGTUNE_MASK	GENMASK(16, USBPCR_OTGTUNE_LSB)

#define USBPCR_SQRXTUNE_LSB	11
#define USBPCR_SQRXTUNE_MASK	GENMASK(13, USBPCR_SQRXTUNE_LSB)

#define USBPCR_TXFSLSTUNE_LSB	7
#define USBPCR_TXFSLSTUNE_MASK	GENMASK(10, USBPCR_TXFSLSTUNE_LSB)

#define USBPCR_TXRISETUNE_LSB	4
#define USBPCR_TXRISETUNE_MASK	GENMASK(5, USBPCR_TXRISETUNE_LSB)

#define USBPCR_TXVREFTUNE_LSB	0
#define USBPCR_TXVREFTUNE_MASK	GENMASK(3, USBPCR_TXVREFTUNE_LSB)

/* USBRDT */
#define USBRDT_VBFIL_LD_EN	BIT(25)
#define USBRDT_IDDIG_EN		BIT(24)
#define USBRDT_IDDIG_REG	BIT(23)

#define USBRDT_USBRDT_LSB	0
#define USBRDT_USBRDT_MASK	GENMASK(22, USBRDT_USBRDT_LSB)

/* USBPCR1 */
#define USBPCR1_UHC_POWON	BIT(5)

struct jz4770_phy {
	struct usb_phy phy;
	struct usb_otg otg;
	struct device *dev;
	void __iomem *base;
	struct clk *clk;
};

static inline struct jz4770_phy *otg_to_jz4770_phy(struct usb_otg *otg)
{
	return container_of(otg, struct jz4770_phy, otg);
}

static inline struct jz4770_phy *phy_to_jz4770_phy(struct usb_phy *phy)
{
	return container_of(phy, struct jz4770_phy, phy);
}

static int jz4770_phy_set_peripheral(struct usb_otg *otg,
				     struct usb_gadget *gadget)
{
	struct jz4770_phy *priv = otg_to_jz4770_phy(otg);
	u32 reg;

	reg = readl(priv->base + REG_USBPCR_OFFSET);
	reg = (reg | USBPCR_VBUSVLDEXT) & ~USBPCR_USB_MODE;
	writel(reg, priv->base + REG_USBPCR_OFFSET);

	return 0;
}

static int jz4770_phy_set_host(struct usb_otg *otg, struct usb_bus *host)
{
	struct jz4770_phy *priv = otg_to_jz4770_phy(otg);
	u32 reg;

	reg = readl(priv->base + REG_USBPCR_OFFSET);
	reg = (reg & ~(USBPCR_VBUSVLDEXT |
		       USBPCR_VBUSVLDEXTSEL |
		       USBPCR_OTG_DISABLE |
		       USBPCR_IDPULLUP_MASK)) |
		USBPCR_USB_MODE | USBPCR_IDPULLUP_ALWAYS;
	writel(reg, priv->base + REG_USBPCR_OFFSET);

	return 0;
}

static int jz4770_phy_init(struct usb_phy *phy)
{
	struct jz4770_phy *priv = phy_to_jz4770_phy(phy);
	int err;
	u32 reg;

	err = clk_prepare_enable(priv->clk);
	if (err) {
		dev_err(priv->dev, "Unable to start clock");
		return err;
	}

	/* fil */
	writel(0x80, priv->base + REG_USBVBFIL_OFFSET);

	/* rdt */
	writel(0x02000096, priv->base + REG_USBRDT_OFFSET);

	/* TXRISETUNE & TXVREFTUNE. */
	reg = readl(priv->base + REG_USBPCR_OFFSET);
	reg = (reg & ~(USBPCR_TXRISETUNE_MASK | USBPCR_TXVREFTUNE_MASK)) |
		(3 << USBPCR_TXRISETUNE_LSB) |
		(5 << USBPCR_TXVREFTUNE_LSB);
	writel(reg, priv->base + REG_USBPCR_OFFSET);

	/* Reset PHY */
	reg = readl(priv->base + REG_USBPCR_OFFSET);
	writel(reg | USBPCR_POR, priv->base + REG_USBPCR_OFFSET);
	usleep_range(30, 300);
	writel(reg & ~USBPCR_POR, priv->base + REG_USBPCR_OFFSET);
	usleep_range(300, 1000);

	jz4770_phy_set_host(&priv->otg, NULL);

	return 0;
}

static void jz4770_phy_shutdown(struct usb_phy *phy)
{
	struct jz4770_phy *priv = phy_to_jz4770_phy(phy);

	clk_disable_unprepare(priv->clk);
}

static int jz4770_phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct jz4770_phy *priv;
	struct resource *mem;
	int err;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	platform_set_drvdata(pdev, priv);
	priv->dev = dev;
	priv->phy.dev = dev;
	priv->phy.otg = &priv->otg;
	priv->phy.label = "jz4770-phy";
	priv->phy.init = jz4770_phy_init;
	priv->phy.shutdown = jz4770_phy_shutdown;

	priv->otg.state = OTG_STATE_UNDEFINED;
	priv->otg.usb_phy = &priv->phy;
	priv->otg.set_host = jz4770_phy_set_host;
	priv->otg.set_peripheral = jz4770_phy_set_peripheral;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->base = devm_ioremap_resource(dev, mem);
	if (IS_ERR(priv->base)) {
		dev_err(dev, "Failed to map registers");
		return PTR_ERR(priv->base);
	}

	priv->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(priv->clk)) {
		dev_err(dev, "Failed to get clock");
		return PTR_ERR(priv->clk);
	}

	err = usb_add_phy(&priv->phy, USB_PHY_TYPE_USB2);
	if (err) {
		dev_err(dev, "Unable to register PHY");
		return err;
	}

	return devm_add_action_or_reset(dev, (void (*)(void *))usb_remove_phy,
					&priv->phy);
}

static const struct of_device_id jz4770_phy_of_matches[] = {
	{ .compatible = "ingenic,jz4770-phy" },
	{ }
};
MODULE_DEVICE_TABLE(of, jz4770_phy_of_matches);

static struct platform_driver jz4770_phy_driver = {
	.probe		= jz4770_phy_probe,
	.driver		= {
		.name	= "jz4770-phy",
		.of_match_table = jz4770_phy_of_matches,
	},
};

static int __init jz4770_phy_driver_init(void)
{
	return platform_driver_register(&jz4770_phy_driver);
}
subsys_initcall(jz4770_phy_driver_init);

static void __exit jz4770_phy_driver_exit(void)
{
	platform_driver_unregister(&jz4770_phy_driver);
}
module_exit(jz4770_phy_driver_exit);

MODULE_AUTHOR("Paul Cercueil <paul@crapouillou.net>");
MODULE_DESCRIPTION("Ingenic JZ4770 USB PHY driver");
MODULE_LICENSE("GPL");
