/*
 *  Copyright (C) 2009-2010, Lars-Peter Clausen <lars@metafoo.de>
 *  JZ4740 platform devices
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General	 Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/resource.h>

#include <linux/dma-mapping.h>

#include <linux/usb/musb.h>

#include <asm/mach-jz4740/platform.h>
#include <asm/mach-jz4740/base.h>
#include <asm/mach-jz4740/irq.h>

#include <linux/serial_core.h>
#include <linux/serial_8250.h>

#include "clock.h"

/* LCD controller */
static struct resource jz4740_framebuffer_resources[] = {
	{
		.start	= JZ4740_LCD_BASE_ADDR,
		.end	= JZ4740_LCD_BASE_ADDR + 0x1000 - 1,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device jz4740_framebuffer_device = {
	.name		= "jz4740-fb",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(jz4740_framebuffer_resources),
	.resource	= jz4740_framebuffer_resources,
	.dev = {
		.dma_mask = &jz4740_framebuffer_device.dev.coherent_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};

/* ADC controller */
static struct resource jz4740_adc_resources[] = {
	{
		.start	= JZ4740_SADC_BASE_ADDR,
		.end	= JZ4740_SADC_BASE_ADDR + 0x30,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= JZ4740_IRQ_SADC,
		.end	= JZ4740_IRQ_SADC,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= JZ4740_IRQ_ADC_BASE,
		.end	= JZ4740_IRQ_ADC_BASE,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device jz4740_adc_device = {
	.name		= "jz4740-adc",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(jz4740_adc_resources),
	.resource	= jz4740_adc_resources,
};
