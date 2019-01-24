/*
 * linux/arch/mips/jz4740/board-qi_lb60.c
 *
 * QI_LB60 board support
 *
 * Copyright (c) 2009 Qi Hardware inc.,
 * Author: Xiangfu Liu <xiangfu@qi-hardware.com>
 * Copyright 2010, Lars-Peter Clausen <lars@metafoo.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or later
 * as published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/gpio/machine.h>

#include <linux/input.h>
#include <linux/pinctrl/machine.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/power_supply.h>
#include <linux/power/jz4740-battery.h>
#include <linux/power/gpio-charger.h>

#include <asm/mach-jz4740/gpio.h>
#include <asm/mach-jz4740/jz4740_fb.h>

#include <linux/regulator/fixed.h>
#include <linux/regulator/machine.h>

#include <asm/mach-jz4740/platform.h>

#include "clock.h"

/* Display */
static struct fb_videomode qi_lb60_video_modes[] = {
	{
		.name = "320x240",
		.xres = 320,
		.yres = 240,
		.refresh = 30,
		.left_margin = 140,
		.right_margin = 273,
		.upper_margin = 20,
		.lower_margin = 2,
		.hsync_len = 1,
		.vsync_len = 1,
		.sync = 0,
		.vmode = FB_VMODE_NONINTERLACED,
	},
};

static struct jz4740_fb_platform_data qi_lb60_fb_pdata = {
	.width		= 60,
	.height		= 45,
	.num_modes	= ARRAY_SIZE(qi_lb60_video_modes),
	.modes		= qi_lb60_video_modes,
	.bpp		= 24,
	.lcd_type	= JZ_LCD_TYPE_8BIT_SERIAL,
	.pixclk_falling_edge = 1,
};

/* Battery */
static struct jz_battery_platform_data qi_lb60_battery_pdata = {
	.gpio_charge =	JZ_GPIO_PORTC(27),
	.gpio_charge_active_low = 1,
	.info = {
		.name = "battery",
		.technology = POWER_SUPPLY_TECHNOLOGY_LIPO,
		.voltage_max_design = 4200000,
		.voltage_min_design = 3600000,
	},
};

/* charger */
static char *qi_lb60_batteries[] = {
	"battery",
};

static struct gpio_charger_platform_data qi_lb60_charger_pdata = {
	.name = "usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.gpio = JZ_GPIO_PORTD(28),
	.gpio_active_low = 1,
	.supplied_to = qi_lb60_batteries,
	.num_supplicants = ARRAY_SIZE(qi_lb60_batteries),
};

static struct platform_device qi_lb60_charger_device = {
	.name = "gpio-charger",
	.dev = {
		.platform_data = &qi_lb60_charger_pdata,
	},
};

static struct platform_device *jz_platform_devices[] __initdata = {
	&jz4740_framebuffer_device,
	&jz4740_adc_device,
	&qi_lb60_charger_device,
};

static struct pinctrl_map pin_map[] __initdata = {
	/* fbdev pin configuration */
	PIN_MAP_MUX_GROUP("jz4740-fb", PINCTRL_STATE_DEFAULT,
			"10010000.pin-controller", "lcd-8bit", "lcd"),
	PIN_MAP_MUX_GROUP("jz4740-fb", PINCTRL_STATE_SLEEP,
			"10010000.pin-controller", "lcd-no-pins", "lcd"),
};


static int __init qi_lb60_init_platform_devices(void)
{
	jz4740_framebuffer_device.dev.platform_data = &qi_lb60_fb_pdata;
	jz4740_adc_device.dev.platform_data = &qi_lb60_battery_pdata;

	pinctrl_register_mappings(pin_map, ARRAY_SIZE(pin_map));

	return platform_add_devices(jz_platform_devices,
					ARRAY_SIZE(jz_platform_devices));

}

static int __init qi_lb60_board_setup(void)
{
	printk(KERN_INFO "Qi Hardware JZ4740 QI LB60 setup\n");

	if (qi_lb60_init_platform_devices())
		panic("Failed to initialize platform devices");

	return 0;
}
arch_initcall(qi_lb60_board_setup);
