/* Copyright (c) 2012, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "msm_fb.h"

#include "../board-8064.h"
#include <linux/hrtimer.h>
#include <linux/timer.h>
#include <asm/mach-types.h>
#include <linux/pwm.h>
#include <linux/mfd/pm8xxx/pm8921.h>
#include <linux/gpio.h>
#include <linux/syscore_ops.h>
#include <linux/i2c.h>
#include <mach/board_asustek.h>

#define PWM_FREQ_HZ 20000
#define PWM_PERIOD_p1USEC (USEC_PER_SEC * 10 / PWM_FREQ_HZ)
#define PWM_LEVEL 255
#define PWM_DUTY_LEVEL \
	(PWM_PERIOD_p1USEC / PWM_LEVEL)

#define gpio_LCD_BL_EN PM8921_GPIO_PM_TO_SYS(30)

static struct lvds_panel_platform_data *cm_pdata;
static struct platform_device *cm_fbpdev;
static struct i2c_client* bl_i2c_client;
extern struct pwm_device *bl_lpm;

static s32 rt8555_read_reg(struct i2c_client*client, u8 reg)
{	return i2c_smbus_read_byte_data(client, reg);}

static s32 rt8555_write_reg(struct i2c_client*client, u8 reg, u8 val)
{	return i2c_smbus_write_byte_data(client, reg, val);}

static const struct i2c_device_id rt8555_id[] = {
	{"rt8555", 0},
	{ }
};

/* This function get called whenever ic is waked up */
static int rt8555_init_seq(void)
{
	gpio_set_value_cansleep(gpio_LCD_BL_EN, 1);
	msleep(50);
	if (bl_i2c_client) {
		pr_debug("gpio 30 %d", gpio_get_value(gpio_LCD_BL_EN));
		rt8555_write_reg(bl_i2c_client, 0x00, 0x5c);
		pr_debug("rt8555 0x00 %d\n", rt8555_read_reg(bl_i2c_client, 0x00));
		rt8555_write_reg(bl_i2c_client, 0x01, 0x07);
		pr_debug("rt8555 0x01 %d\n", rt8555_read_reg(bl_i2c_client, 0x01));
		rt8555_write_reg(bl_i2c_client, 0x02, 0xa1);
		pr_debug("rt8555 0x02 %d\n", rt8555_read_reg(bl_i2c_client, 0x02));
	} else {
		pr_err("rt8555 no i2c client instances\n");
		return -ENODEV;
	}
	return 0;
}

static int rt8555_probe(struct i2c_client*client,
		const struct i2c_device_id*id)
{
	int rc;
	bl_i2c_client = client;
	pr_info("rt8555 probe\n");

	//LCD_BL_EN
	rc = gpio_request(gpio_LCD_BL_EN, "LCD_BL_EN");
	if (rc) {
		pr_err("request gpio 30 failed, rc=%d\n", rc);
		return -ENODEV;
	}

	return rt8555_init_seq();
}

static int rt8555_remove(struct i2c_client*client)
{
	pr_info("rt8555 remove\n");
	return 0;
}

MODULE_DEVICE_TABLE(i2c, rt8555_id);

static struct i2c_driver rt8555_driver = {
	.driver = {
		.name = "rt8555",
		.owner = THIS_MODULE,
	},
	.probe = rt8555_probe,
	.remove = rt8555_remove,
	.id_table = rt8555_id,
};

static int lvds_chimei_panel_on(struct platform_device *pdev)
{
	static int first_boot = 1;
	if (unlikely(first_boot)) {
		pr_info("%s: lvds backlight on\n", __func__);
		gpio_set_value_cansleep(gpio_LCD_BL_EN, 1);
		first_boot = 0;
	}
	return 0;
}

static int lvds_chimei_panel_off(struct platform_device *pdev)
{
	int ret;
	pr_info("%s+\n", __func__);

	msleep(10);

	if (bl_lpm) {
		ret = pwm_config_in_p1us_unit(bl_lpm, 0, PWM_PERIOD_p1USEC);
		if (ret) {
			pr_err("pwm_config on lpm failed %d\n", ret);
		}
		pwm_disable(bl_lpm);
	}

	msleep(180);

	gpio_set_value_cansleep(gpio_LCD_BL_EN, 0);
	pr_info("%s-\n", __func__);

	return 0;
}

static enum hrtimer_restart boot_event_timer_func(struct hrtimer *data) {
	pr_info("%s: lvds backlight off\n", __func__);
	gpio_set_value_cansleep(gpio_LCD_BL_EN, 0);
	return HRTIMER_NORESTART;
}

static void lvds_chimei_set_backlight(struct msm_fb_data_type *mfd)
{
	int ret;
	static int bl_enable_sleep_control = 0; // msleep(10) only when suspend or resume
	static int is_bl_first_set = 0;

	if (!is_bl_first_set) {
		is_bl_first_set = 1;
		if (mfd->bl_level == 0) {
			pr_debug("%s: skip to set backlight for first time.\n", __func__);
			return;
		}
	}

	pr_debug("%s: back light level %d\n", __func__, mfd->bl_level);

	if (bl_lpm) {
		if (mfd->bl_level) {
			printk("%s %d\n", __func__, mfd->bl_level);
			if (!bl_enable_sleep_control) {
				msleep(200);
				if (rt8555_init_seq()) {
					pr_err("rt8555 init failure.\n");
					return ;
				}
				msleep(10);
				pr_debug("%s: sleep 1 when resume\n", __func__);
			}
			ret = pwm_config_in_p1us_unit(bl_lpm, PWM_PERIOD_p1USEC *
				mfd->bl_level/ PWM_LEVEL, PWM_PERIOD_p1USEC);
			if (ret) {
				pr_err("pwm_config on lpm failed %d\n", ret);
				return;
			}
			ret = pwm_enable(bl_lpm);
			if (ret)
				pr_err("pwm enable/disable on lpm failed"
				"for bl %d\n",	mfd->bl_level);
			if (!bl_enable_sleep_control) {
				msleep(10);
				bl_enable_sleep_control = 1;
				pr_debug("%s: sleep 2 when resume\n", __func__);
			}
		} else {
			ret = pwm_config_in_p1us_unit(bl_lpm, PWM_PERIOD_p1USEC *
				mfd->bl_level/ PWM_LEVEL, PWM_PERIOD_p1USEC);
			if (ret) {
				pr_err("pwm_config on lpm failed %d\n", ret);
				return;
			}
			if (bl_enable_sleep_control) {
				msleep(50);
				pr_debug("%s: sleep 1 when suspend\n", __func__);
			}
			pwm_disable(bl_lpm);
			gpio_set_value_cansleep(gpio_LCD_BL_EN, 0);
			if (bl_enable_sleep_control) {
				bl_enable_sleep_control = 0;
				msleep(200);
				pr_debug("%s: sleep 2 when suspend\n", __func__);
			}
		}
	}
}

static int __devinit lvds_chimei_probe(struct platform_device *pdev)
{
	int rc = 0;
	pr_info("%s +++\n", __func__);

	if (pdev->id == 0) {
		cm_pdata = pdev->dev.platform_data;
		if (cm_pdata == NULL)
			pr_err("%s: no PWM gpio specified\n", __func__);
		return 0;
	}

	if (bl_lpm == NULL || IS_ERR(bl_lpm)) {
		pr_err("%s pwm_request() failed\n", __func__);
		bl_lpm = NULL;
	}
	pr_debug("bl_lpm = %p lpm = %d\n", bl_lpm,
		cm_pdata->gpio[0]);

	cm_fbpdev = msm_fb_add_device(pdev);
	if (!cm_fbpdev) {
		dev_err(&pdev->dev, "failed to add msm_fb device\n");
		rc = -ENODEV;
		goto probe_exit;
	}
	pr_info("%s ---\n", __func__);
probe_exit:
	return rc;
}

static void lvds_chimei_set_recovery_backlight(struct msm_fb_data_type *mfd)
{
	pr_info("%s +++\n", __func__);
	mfd->bl_level = 102;
	lvds_chimei_set_backlight(mfd);

}

static struct platform_driver this_driver = {
	.probe  = lvds_chimei_probe,
	.driver = {
		.name   = "lvds_chimei_wxga",
	},
};

static struct msm_fb_panel_data lvds_chimei_panel_data = {
	.on = lvds_chimei_panel_on,
	.off = lvds_chimei_panel_off,
	.set_backlight = lvds_chimei_set_backlight,
	.set_recovery_backlight = lvds_chimei_set_recovery_backlight,
};

static struct platform_device this_device = {
	.name   = "lvds_chimei_wxga",
	.id	= 1,
	.dev	= {
		.platform_data = &lvds_chimei_panel_data,
	}
};

struct hrtimer boot_timer;
static int __init lvds_chimei_wxga_init(void)
{
	int ret;
	struct msm_panel_info *pinfo;
	if (!machine_is_apq8064_leopardcat()) {
		return -ENODEV;
	}
	pr_info("%s +++ \n", __func__);


	ret = platform_driver_register(&this_driver);
	if (ret)
		return ret;

	pinfo = &lvds_chimei_panel_data.panel_info;
	pinfo->xres = 1280;
	pinfo->yres = 800;
	MSM_FB_SINGLE_MODE_PANEL(pinfo);
	pinfo->type = LVDS_PANEL;
	pinfo->pdest = DISPLAY_1;
	pinfo->wait_cycle = 0;
	pinfo->bpp = 24;
	pinfo->fb_num = 2;
	pinfo->clk_rate = 68900000;
	pinfo->bl_max = 255;
	pinfo->bl_min = 1;

	/*
	 * this panel is operated by de,
	 * vsycn and hsync are ignored
	 */
	pinfo->lcdc.h_back_porch = 40;
	pinfo->lcdc.h_front_porch = 40;
	pinfo->lcdc.h_pulse_width = 48;
	pinfo->lcdc.v_back_porch = 4;
	pinfo->lcdc.v_front_porch = 4;
	pinfo->lcdc.v_pulse_width = 8;
	pinfo->lcdc.underflow_clr = 0xff;
	pinfo->lcdc.hsync_skew = 0;
	pinfo->lvds.channel_mode = LVDS_SINGLE_CHANNEL_MODE;

	/* Set border color, padding only for reducing active display region */
	pinfo->lcdc.border_clr = 0x0;
	pinfo->lcdc.xres_pad = 0;
	pinfo->lcdc.yres_pad = 0;

	pr_info("rt8555 registration\n");
	bl_i2c_client = NULL;
	ret = i2c_add_driver(&rt8555_driver);
	if (ret) {
		pr_err("%s: failed rt5648 i2c registration\n", __func__);
		return ret;
	}

	ret = platform_device_register(&this_device);
	if (ret)
		platform_driver_unregister(&this_driver);

	hrtimer_init(&boot_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	boot_timer.function = boot_event_timer_func;
	hrtimer_start(&boot_timer,
			ktime_set(5, 500000000), HRTIMER_MODE_REL);
	pr_info("%s ---\n", __func__);
	return ret;
}

module_init(lvds_chimei_wxga_init);
