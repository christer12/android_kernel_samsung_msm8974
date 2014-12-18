/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/err.h>
#include <linux/regulator/consumer.h>
#include <linux/pwm.h>

#include <asm/system.h>
#include <asm/mach-types.h>

#include <mach/hardware.h>
#include <mach/dma.h>

#include "mdss_edp.h"

#define RGB_COMPONENTS		3
#define VDDA_MIN_UV			1800000	/* uV units */
#define VDDA_MAX_UV			1800000	/* uV units */
#define VDDA_UA_ON_LOAD		100000	/* uA units */
#define VDDA_UA_OFF_LOAD	100		/* uA units */

static int mdss_edp_get_base_address(struct mdss_edp_drv_pdata *edp_drv);
static int mdss_edp_get_mmss_cc_base_address(struct mdss_edp_drv_pdata
		*edp_drv);
static int mdss_edp_regulator_init(struct mdss_edp_drv_pdata *edp_drv);
static int mdss_edp_regulator_on(struct mdss_edp_drv_pdata *edp_drv);
static int mdss_edp_regulator_off(struct mdss_edp_drv_pdata *edp_drv);
static int mdss_edp_gpio_panel_en(struct mdss_edp_drv_pdata *edp_drv);
static int mdss_edp_pwm_config(struct mdss_edp_drv_pdata *edp_drv);

static void mdss_edp_edid2pinfo(struct mdss_edp_drv_pdata *edp_drv);
static void mdss_edp_fill_edid_data(struct mdss_edp_drv_pdata *edp_drv);
static void mdss_edp_fill_dpcd_data(struct mdss_edp_drv_pdata *edp_drv);

static int mdss_edp_device_register(struct mdss_edp_drv_pdata *edp_drv);

static void mdss_edp_link_fast_train(struct mdss_edp_drv_pdata *edp_drv);
static void mdss_edp_config_sync(unsigned char *edp_base);
static void mdss_edp_config_sw_div(unsigned char *edp_base);
static void mdss_edp_config_static_mdiv(unsigned char *edp_base);
static void mdss_edp_enable(unsigned char *edp_base, int enable);

struct mdss_edp_drv_pdata *test_edp_drv;

#define AUX_DELAY 500
#define TRAINING_DELAY 1000

#if defined(CONFIG_FB_MSM_EDP_SAMSUNG)
extern int eDP_TON_NDRA;
int edp_tcon_read(u16 reg, u8 *val, unsigned int len);
int edp_tcon_write(u16 reg,  u8 val, unsigned int len);
extern void edp_backlight_enable(void);
extern void edp_backlight_disable(void);
#endif

/*
 * Init regulator needed for edp, 8974_l12
 */
static int mdss_edp_regulator_init(struct mdss_edp_drv_pdata *edp_drv)
{
	int ret;

	edp_drv->vdda_vreg = devm_regulator_get(&(edp_drv->pdev->dev), "vdda");
	if (IS_ERR(edp_drv->vdda_vreg)) {
		pr_err("%s: Could not get 8941_l12, ret = %ld\n", __func__,
				PTR_ERR(edp_drv->vdda_vreg));
		return -ENODEV;
	}

	ret = regulator_set_voltage(edp_drv->vdda_vreg,
			VDDA_MIN_UV, VDDA_MAX_UV);
	if (ret) {
		pr_err("%s: vdda_vreg set_voltage failed, ret=%d\n", __func__,
				ret);
		return -EINVAL;
	}

	ret = mdss_edp_regulator_on(edp_drv);
	if (ret)
		return ret;

	return 0;
}

/*
 * Set uA and enable vdda
 */
static int mdss_edp_regulator_on(struct mdss_edp_drv_pdata *edp_drv)
{
	int ret;

	ret = regulator_set_optimum_mode(edp_drv->vdda_vreg, VDDA_UA_ON_LOAD);
	if (ret < 0) {
		pr_err("%s: vdda_vreg set regulator mode failed.\n", __func__);
		return ret;
	}

	ret = regulator_enable(edp_drv->vdda_vreg);
	if (ret) {
		pr_err("%s: Failed to enable vdda_vreg regulator.\n", __func__);
		return ret;
	}

	msleep(100);
	return 0;
}

/*
 * Disable vdda and set uA
 */
static int mdss_edp_regulator_off(struct mdss_edp_drv_pdata *edp_drv)
{
	int ret;

	ret = regulator_disable(edp_drv->vdda_vreg);
	if (ret) {
		pr_err("%s: Failed to disable vdda_vreg regulator.\n",
				__func__);
		return ret;
	}

	ret = regulator_set_optimum_mode(edp_drv->vdda_vreg, VDDA_UA_OFF_LOAD);
	if (ret < 0) {
		pr_err("%s: vdda_vreg set regulator mode failed.\n",
				__func__);
		return ret;
	}

	return 0;
}

/*
 * Enables the gpio that supply power to the panel and enable the backlight
 */
static int mdss_edp_gpio_panel_en(struct mdss_edp_drv_pdata *edp_drv)
{
	int ret = 0;

	edp_drv->gpio_panel_en = of_get_named_gpio(edp_drv->pdev->dev.of_node,
			"gpio-panel-en", 0);
	if (!gpio_is_valid(edp_drv->gpio_panel_en)) {
		pr_err("%s: gpio_panel_en=%d not specified\n", __func__,
				edp_drv->gpio_panel_en);
		goto gpio_err;
	}

	ret = gpio_request(edp_drv->gpio_panel_en, "disp_enable");
	if (ret) {
		pr_err("%s: Request reset gpio_panel_en failed, ret=%d\n",
				__func__, ret);
		return ret;
	}

	ret = gpio_direction_output(edp_drv->gpio_panel_en, 1);
	if (ret) {
		pr_err("%s: Set direction for gpio_panel_en failed, ret=%d\n",
				__func__, ret);
		goto gpio_free;
	}

	return 0;

gpio_free:
	gpio_free(edp_drv->gpio_panel_en);
gpio_err:
	return -ENODEV;
}

static void mdss_edp_panel_power(struct mdss_edp_drv_pdata *edp_drv, int enable)
{
	
	gpio_set_value(edp_drv->gpio_panel_en, enable);

	msleep(500);
}

static int mdss_edp_pwm_config(struct mdss_edp_drv_pdata *edp_drv)
{
	int ret = 0;

	ret = of_property_read_u32(edp_drv->pdev->dev.of_node,
			"qcom,panel-pwm-period", &edp_drv->pwm_period);
	if (ret) {
		pr_err("%s: panel pwm period is not specified, %d", __func__,
				edp_drv->pwm_period);
		return -EINVAL;
	}

	ret = of_property_read_u32(edp_drv->pdev->dev.of_node,
			"qcom,panel-lpg-channel", &edp_drv->lpg_channel);
	if (ret) {
		pr_err("%s: panel lpg channel is not specified, %d", __func__,
				edp_drv->lpg_channel);
		return -EINVAL;
	}

	edp_drv->bl_pwm = pwm_request(edp_drv->lpg_channel, "lcd-backlight");
	if (edp_drv->bl_pwm == NULL || IS_ERR(edp_drv->bl_pwm)) {
		pr_err("%s: pwm request failed", __func__);
		edp_drv->bl_pwm = NULL;
		return -EIO;
	}

	edp_drv->gpio_panel_pwm = of_get_named_gpio(edp_drv->pdev->dev.of_node,
			"gpio-panel-pwm", 0);
	if (!gpio_is_valid(edp_drv->gpio_panel_pwm)) {
		pr_err("%s: gpio_panel_pwm=%d not specified\n", __func__,
				edp_drv->gpio_panel_pwm);
		goto edp_free_pwm;
	}

	ret = gpio_request(edp_drv->gpio_panel_pwm, "disp_pwm");
	if (ret) {
		pr_err("%s: Request reset gpio_panel_pwm failed, ret=%d\n",
				__func__, ret);
		goto edp_free_pwm;
	}

	return 0;

edp_free_pwm:
	pwm_free(edp_drv->bl_pwm);
	return -ENODEV;
}

void mdss_edp_set_backlight(struct mdss_panel_data *pdata, u32 bl_level)
{
	int ret = 0;
	struct mdss_edp_drv_pdata *edp_drv = NULL;
	int bl_max;

	edp_drv = container_of(pdata, struct mdss_edp_drv_pdata, panel_data);
	if (!edp_drv) {
		pr_err("%s: Invalid input data\n", __func__);
		return;
	}

	bl_max = edp_drv->panel_data.panel_info.bl_max;
	if (bl_level > bl_max)
		bl_level = bl_max;

	if (edp_drv->bl_pwm == NULL) {
		pr_err("%s: edp_drv->bl_pwm=NULL.\n", __func__);
		return;
	}

	ret = pwm_config(edp_drv->bl_pwm,
			edp_drv->pwm_period - (bl_level * edp_drv->pwm_period / bl_max),
			edp_drv->pwm_period);
	if (ret) {
		pr_err("%s: pwm_config() failed err=%d.\n", __func__, ret);
		return;
	}

	ret = pwm_enable(edp_drv->bl_pwm);
	if (ret) {
		pr_err("%s: pwm_enable() failed err=%d\n", __func__, ret);
		return;
	}

#if defined(CONFIG_FB_MSM_EDP_SAMSUNG)
	edp_backlight_enable();
#endif
	pr_info("%s bl_level : %d duty : %d period : %d", __func__, bl_level, edp_drv->pwm_period - (bl_level * edp_drv->pwm_period / bl_max), edp_drv->pwm_period);
}

void mdss_edp_config_sync(unsigned char *edp_base)
{
	int ret = 0;

	ret = edp_read(edp_base + 0xc); /* EDP_CONFIGURATION_CTRL */
	ret &= ~0x733;
	ret |= (0x75 & 0x733);
	edp_write(edp_base + 0xc, ret);
	edp_write(edp_base + 0xc, 0x177); /* EDP_CONFIGURATION_CTRL */
}

static void mdss_edp_config_sw_div(unsigned char *edp_base)
{
	edp_write(edp_base + 0x14, 0x217); /* EDP_SOFTWARE_MVID */
	edp_write(edp_base + 0x18, 0x21a); /* EDP_SOFTWARE_NVID */
}

static void mdss_edp_config_static_mdiv(unsigned char *edp_base)
{
	int ret = 0;

	ret = edp_read(edp_base + 0xc); /* EDP_CONFIGURATION_CTRL */
	edp_write(edp_base + 0xc, ret | 0x2); /* EDP_CONFIGURATION_CTRL */
	edp_write(edp_base + 0xc, 0x77); /* EDP_CONFIGURATION_CTRL */
}

void aux_register_log(unsigned char *edp_base);

static void mdss_edp_enable(unsigned char *edp_base, int enable)
{
	//NEVER CHANGE
	edp_write(edp_base + 0x308, 0x0); /* EDP_STATE_CTRL */
	edp_write(edp_base + 0x30c, 0x0); /* EDP_STATE_CTRL */

#if 1 // TEST pattern.
	edp_write(edp_base + 0x130, 0xa6f0070);
	edp_write(edp_base + 0x134, 0x0);
	edp_write(edp_base + 0x110, 0x1c950);
	edp_write(edp_base + 0x118, 0x4430af);


	edp_write(edp_base + 0x9c, 0xaa00020);
	edp_write(edp_base + 0x100, 0x4450c0);
	edp_write(edp_base + 0x108, 0x3fc0);



	edp_write(edp_base + 0x140, 0x80);
	edp_write(edp_base + 0x144, 0x5);


	edp_write(edp_base + 0x28, 0x6400A00);
	edp_write(edp_base + 0x24, 0x68020);
	edp_write(edp_base + 0x20, 0x2b0070);
	edp_write(edp_base + 0x1c, 0x66e0aa0);


	edp_write(edp_base + 0xc, 0x175);

	edp_write(edp_base + 0x14, 0x217);
	edp_write(edp_base + 0x18, 0x21a);
	edp_write(edp_base + 0xc, 0x137);

	edp_write(edp_base + 0x140, 0x100);
	edp_write(edp_base + 0x144, 0x5);
#if 0 //need for test pattern.
	
	edp_write(edp_base + 0x8, 0x0);
	edp_write(edp_base + 0x8, 0x40);
	msleep(100);
	need for test pattern.
	edp_write(edp_base + 0x90, 0x1);
	edp_write(edp_base + 0x94, 0x1);
#endif
#endif
	edp_write(edp_base + 0x8, 0x0); /* EDP_STATE_CTRL */
	edp_write(edp_base + 0x8, 0x40); /* EDP_STATE_CTRL */
	edp_write(edp_base + 0x94, enable); /* EDP_TIMING_ENGINE_EN */
	edp_write(edp_base + 0x4, enable); /* EDP_MAINLINK_CTRL */
}

void aux_register_log(unsigned char *edp_base)
{
	printk("\n\n\n");
	pr_info("%s 0x04 0x%x", __func__,edp_read(edp_base+0x04));
	pr_info("%s 0x0C 0x%x", __func__,edp_read(edp_base+0x0C));
	pr_info("%s 0x84 0x%x", __func__,edp_read(edp_base+0x84));
	pr_info("%s 0x308 0x%x", __func__,edp_read(edp_base+0x308));
	pr_info("%s 0x30C 0x%x", __func__,edp_read(edp_base+0x30C));
	pr_info("%s 0x310 0x%x", __func__,edp_read(edp_base+0x310));
	pr_info("%s 0x318 0x%x", __func__,edp_read(edp_base+0x318));
	pr_info("%s 0x324 0x%x", __func__,edp_read(edp_base+0x324));
	pr_info("%s 0x514 0x%x", __func__,edp_read(edp_base+0x514));
	pr_info("%s 0x550 0x%x", __func__,edp_read(edp_base+0x550));
	pr_info("%s 0x554 0x%x", __func__,edp_read(edp_base+0x554));
}

void aux_write(unsigned char *edp_base, int addr, int data)
{
	edp_write(edp_base + 0x318, edp_read(edp_base + 0x318) & (~(0x1 << 9)));

	edp_write(edp_base + 0x314, 0x80008000);
	edp_write(edp_base + 0x314, 0x80010000 | (addr & 0x0000FF00));
	edp_write(edp_base + 0x314, 0x80020000 | ((addr & 0x000000FF) << 8));
	edp_write(edp_base + 0x314, 0x80030000);

	edp_write(edp_base + 0x314, 0x80040000|((data&0xff)<<8)); 

	edp_write(edp_base + 0x318, 0x200);

	usleep(AUX_DELAY);
}

int aux_read(unsigned char *edp_base, int addr)
{
	int read_data;

	edp_write(edp_base + 0x318, edp_read(edp_base + 0x318) & (~(0x1 << 9)));
	//0x00023714 EDP_AUX_DATA 
	edp_write(edp_base + 0x314, 0x80009000);
	edp_write(edp_base + 0x314, 0x80010000 | (addr & 0x0000FF00));
	edp_write(edp_base + 0x314, 0x80020000 | ((addr & 0x000000FF) << 8));
	edp_write(edp_base + 0x314, 0x80030000);
	edp_write(edp_base + 0x318, 0x200); 
	usleep(AUX_DELAY);

           edp_write(edp_base + 0x314, 0x80000001);

//	aux_register_log(edp_base);

	read_data = edp_read(edp_base+0x314);
	read_data = edp_read(edp_base+0x314);
//	pr_info("%s addr : 0x%x value : 0x%x", __func__, addr, read_data >> 8);
	return read_data >> 8;
}

#if defined(CONFIG_FB_MSM_EDP_SAMSUNG)
static void parade_tcon(struct mdss_edp_drv_pdata *edp_drv)
{
	// Training 1
	edp_write(edp_drv->edp_base + 0x8, BIT(0)); /* EDP_STATE_CTRL */
	usleep(TRAINING_DELAY);

	if (edp_read(edp_drv->edp_base + 0x84) & BIT(3))
		pr_info("%s: Training pattern 1 was sent properly\n", __func__);
	else
		pr_err("%s: Error in sending training pattern 1\n", __func__);


	// Training 2
	edp_write(edp_drv->edp_base + 0x8, 0); /* EDP_STATE_CTRL */
	edp_write(edp_drv->edp_base + 0x8, BIT(1)); /* EDP_STATE_CTRL */

	if (edp_read(edp_drv->edp_base + 0x84) & BIT(4))
		pr_info("%s: Training pattern 2 was sent properly\n", __func__);
	else
		pr_err("%s: Error in sending training pattern 2\n", __func__);

	usleep(TRAINING_DELAY);

	//pr_info("%s aux_read addr : 0x%x value : 0x%x", __func__, 0x202, aux_read(edp_drv->edp_base, 0x202));
	//pr_info("%s aux_read addr : 0x%x value : 0x%x", __func__, 0x203, aux_read(edp_drv->edp_base, 0x203));

}

static void ndra_tcon(struct mdss_edp_drv_pdata *edp_drv)
{
	u8 test1,test2;

	// Training 1
	do {
		edp_write(edp_drv->edp_base + 0x8, BIT(0)); /* EDP_STATE_CTRL */
		aux_write(edp_drv->edp_base, 0x102, 0x21);
			usleep(TRAINING_DELAY);

		if (edp_read(edp_drv->edp_base + 0x84) & BIT(3))
			pr_debug("%s: Training pattern 1 was sent properly\n", __func__);
		else
			pr_err("%s: Error in sending training pattern 1\n", __func__);

		test1 = aux_read(edp_drv->edp_base, 0x206);
		test2 = aux_read(edp_drv->edp_base, 0x207);

		aux_write(edp_drv->edp_base, 0x103, (test1 & 0x03) | ((test1 & 0x0C) << 1)); /* LANE 0*/
		aux_write(edp_drv->edp_base, 0x104, ((test1 & 0x30) >> 4) | ((test1 & 0xC0) >> 3)); /*LANE 1*/
		aux_write(edp_drv->edp_base, 0x105, (test2 & 0x03) | ((test2 & 0x0C) << 1)); /* LANE 2*/
		aux_write(edp_drv->edp_base, 0x106, ((test2 & 0x30) >> 4) | ((test2 & 0xC0) >> 3)); /*LANE 3*/

		//VOLATEGE TX

	} while (aux_read(edp_drv->edp_base, 0x202) != 0x11 || \
		aux_read(edp_drv->edp_base, 0x203) != 0x11);

	pr_info("%s: Training pattern 1 was sent properly\n", __func__);
	// Training 2
	do {
		aux_write(edp_drv->edp_base, 0x102, 0x22);
		edp_write(edp_drv->edp_base + 0x8, 0); /* EDP_STATE_CTRL */
		edp_write(edp_drv->edp_base + 0x8, BIT(1)); /* EDP_STATE_CTRL */

		if (edp_read(edp_drv->edp_base + 0x84) & BIT(4))
			pr_debug("%s: Training pattern 2 was sent properly\n", __func__);
		else
			pr_err("%s: Error in sending training pattern 2\n", __func__);

		usleep(TRAINING_DELAY);


		test1 = aux_read(edp_drv->edp_base, 0x206);
		test2 = aux_read(edp_drv->edp_base, 0x207);

		aux_write(edp_drv->edp_base, 0x103, (test1 & 0x03) | ((test1 & 0x0C) << 1)); /* LANE 0*/
		aux_write(edp_drv->edp_base, 0x104, ((test1 & 0x30) >> 4) | ((test1 & 0xC0) >> 3)); /*LANE 1*/
		aux_write(edp_drv->edp_base, 0x105, (test2 & 0x03) | ((test2 & 0x0C) << 1)); /* LANE 2*/
		aux_write(edp_drv->edp_base, 0x106, ((test2 & 0x30) >> 4) | ((test2 & 0xC0) >> 3)); /*LANE 3*/

		//PRE-EMPHA TX

	} while (aux_read(edp_drv->edp_base, 0x202) != 0x77 || \
		aux_read(edp_drv->edp_base, 0x203) != 0x77);

	pr_info("%s: Training pattern 2 was sent properly\n", __func__);
	aux_write(edp_drv->edp_base, 0x102, 0);
#if 0
	aux_read(edp_drv->edp_base, 0x0);
	aux_read(edp_drv->edp_base, 0x1);
	aux_read(edp_drv->edp_base, 0x2);
	aux_read(edp_drv->edp_base, 0x3);
	aux_read(edp_drv->edp_base, 0x100);
	aux_read(edp_drv->edp_base, 0x101);
	aux_read(edp_drv->edp_base, 0x202);
	aux_read(edp_drv->edp_base, 0x203);	

	pr_info("%s aux_read addr : 0x%x value : 0x%x", __func__, 0x202, aux_read(edp_drv->edp_base, 0x202));
	pr_info("%s aux_read addr : 0x%x value : 0x%x", __func__, 0x203, aux_read(edp_drv->edp_base, 0x203));
#endif

}
#endif

static void mdss_edp_link_fast_train(struct mdss_edp_drv_pdata *edp_drv)
{
	pr_info("%s start", __func__);

	//NEVER CHANGE
	edp_write(edp_drv->edp_base + 0x308, 0x0); /* EDP_STATE_CTRL */
	edp_write(edp_drv->edp_base + 0x30c, 0x0); /* EDP_STATE_CTRL */

	edp_write(edp_drv->edp_base + 0x8, 0); /* EDP_STATE_CTRL */
	usleep(AUX_DELAY);

	edp_write(edp_drv->edp_base + 0x28, 0x6400A00);
	edp_write(edp_drv->edp_base + 0x24, 0x68020);
	edp_write(edp_drv->edp_base + 0x20, 0x2b0070);
	edp_write(edp_drv->edp_base + 0x1c, 0x66e0aa0);
	wmb();

#if defined(CONFIG_FB_MSM_EDP_SAMSUNG)
	if (eDP_TON_NDRA < 0)
		parade_tcon(edp_drv);
	else
		ndra_tcon(edp_drv);
#endif
	pr_info("%s end", __func__);
}


extern void mdss_edp_enable_aux(unsigned char *edp_base, int enable);

int mdss_edp_on(struct mdss_panel_data *pdata)
{
	struct mdss_edp_drv_pdata *edp_drv = NULL;
	int i;

	pr_info("%s", __func__);
	edp_drv = container_of(pdata, struct mdss_edp_drv_pdata,
			panel_data);
	if (!edp_drv) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	mdss_edp_regulator_on(edp_drv);
	mdss_edp_panel_power(edp_drv, 1);

	mdss_edp_prepare_clocks(edp_drv);
	mdss_edp_phy_sw_reset(edp_drv->edp_base);
	mdss_edp_hw_powerup(edp_drv->edp_base, 1);
	mdss_edp_pll_configure(edp_drv->edp_base, edp_drv->edid.timing[0].pclk);
	mdss_edp_clk_enable(edp_drv);

	for (i = 0; i < edp_drv->dpcd.max_lane_count; ++i)
		mdss_edp_enable_lane_bist(edp_drv->edp_base, i, 1);

	mdss_edp_enable_mainlink(edp_drv->edp_base, 1);
	mdss_edp_config_clk(edp_drv->edp_base, edp_drv->mmss_cc_base);

	mdss_edp_phy_misc_cfg(edp_drv->edp_base);
	mdss_edp_config_sync(edp_drv->edp_base);
	mdss_edp_config_sw_div(edp_drv->edp_base);
	mdss_edp_config_static_mdiv(edp_drv->edp_base);

	mdss_edp_enable_aux(edp_drv->edp_base, 1);
	mdss_edp_link_fast_train(edp_drv);
	mdss_edp_enable(edp_drv->edp_base, 1);
	gpio_set_value(edp_drv->gpio_panel_en, 1);

	return 0;
}

int mdss_edp_off(struct mdss_panel_data *pdata)
{
	struct mdss_edp_drv_pdata *edp_drv = NULL;
	int ret = 0;
	int i;

	pr_info("%s", __func__);
	edp_drv = container_of(pdata, struct mdss_edp_drv_pdata,
				panel_data);
	if (!edp_drv) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}
#if defined(CONFIG_FB_MSM_EDP_SAMSUNG)
	edp_backlight_disable();
#endif
	gpio_set_value(edp_drv->gpio_panel_en, 0);
	pwm_disable(edp_drv->bl_pwm);
	mdss_edp_enable(edp_drv->edp_base, 0);
	mdss_edp_unconfig_clk(edp_drv->edp_base, edp_drv->mmss_cc_base);
	mdss_edp_enable_mainlink(edp_drv->edp_base, 0);

	for (i = 0; i < edp_drv->dpcd.max_lane_count; ++i)
		mdss_edp_enable_lane_bist(edp_drv->edp_base, i, 0);

	mdss_edp_clk_disable(edp_drv);
	mdss_edp_hw_powerup(edp_drv->edp_base, 0);
	mdss_edp_unprepare_clocks(edp_drv);

	mdss_edp_regulator_off(edp_drv);
	mdss_edp_panel_power(edp_drv, 0);
	return ret;
}

static int mdss_edp_event_handler(struct mdss_panel_data *pdata,
				  int event, void *arg)
{
	int rc = 0;

	pr_info("%s: event=%d\n", __func__, event);
	switch (event) {
	case MDSS_EVENT_UNBLANK:
		rc = mdss_edp_on(pdata);
		break;
	case MDSS_EVENT_PANEL_OFF:
		rc = mdss_edp_off(pdata);
		break;
	}
	return rc;
}

/*
 * Converts from EDID struct to mdss_panel_info
 */
static void mdss_edp_edid2pinfo(struct mdss_edp_drv_pdata *edp_drv)
{
	struct display_timing_desc *dp;
	struct mdss_panel_info *pinfo;

	dp = &edp_drv->edid.timing[0];
	pinfo = &edp_drv->panel_data.panel_info;

	pinfo->clk_rate = dp->pclk;

	pinfo->xres = dp->h_addressable + dp->h_border * 2;
	pinfo->yres = dp->v_addressable + dp->v_border * 2;

	pinfo->lcdc.h_back_porch = dp->h_blank - dp->h_fporch \
		- dp->h_sync_pulse;
	pinfo->lcdc.h_front_porch = dp->h_fporch;
	pinfo->lcdc.h_pulse_width = dp->h_sync_pulse;

	pinfo->lcdc.v_back_porch = dp->v_blank - dp->v_fporch \
		- dp->v_sync_pulse;
	pinfo->lcdc.v_front_porch = dp->v_fporch;
	pinfo->lcdc.v_pulse_width = dp->v_sync_pulse;

	pinfo->type = EDP_PANEL;
	pinfo->pdest = DISPLAY_1;
	pinfo->wait_cycle = 0;
	pinfo->bpp = edp_drv->edid.color_depth * RGB_COMPONENTS;
	pinfo->fb_num = 2;

	pinfo->lcdc.border_clr = 0;	 /* black */
	pinfo->lcdc.underflow_clr = 0xff; /* blue */
	pinfo->lcdc.hsync_skew = 0;
}

static int __devexit mdss_edp_remove(struct platform_device *pdev)
{
	struct mdss_edp_drv_pdata *edp_drv = NULL;

	edp_drv = platform_get_drvdata(pdev);

	gpio_free(edp_drv->gpio_panel_en);
	mdss_edp_regulator_off(edp_drv);
	iounmap(edp_drv->edp_base);
	iounmap(edp_drv->mmss_cc_base);
	edp_drv->edp_base = NULL;

	return 0;
}

static int mdss_edp_device_register(struct mdss_edp_drv_pdata *edp_drv)
{
	int ret;

	mdss_edp_edid2pinfo(edp_drv);
	edp_drv->panel_data.panel_info.bl_min = 1;
	edp_drv->panel_data.panel_info.bl_max = 255;

	edp_drv->panel_data.event_handler = mdss_edp_event_handler;
	edp_drv->panel_data.set_backlight = mdss_edp_set_backlight;

	ret = mdss_register_panel(edp_drv->pdev, &edp_drv->panel_data);
	if (ret) {
		dev_err(&(edp_drv->pdev->dev), "unable to register eDP\n");
		return ret;
	}

	pr_info("%s: eDP initialized\n", __func__);

	return 0;
}

/*
 * Retrieve edp base address
 */
static int mdss_edp_get_base_address(struct mdss_edp_drv_pdata *edp_drv)
{
	struct resource *res;

	res = platform_get_resource_byname(edp_drv->pdev, IORESOURCE_MEM,
			"edp_base");
	if (!res) {
		pr_err("%s: Unable to get the MDSS EDP resources", __func__);
		return -ENOMEM;
	}

	edp_drv->edp_base = ioremap(res->start, resource_size(res));
	if (!edp_drv->edp_base) {
		pr_err("%s: Unable to remap EDP resources",  __func__);
		return -ENOMEM;
	}

	return 0;
}

static int mdss_edp_get_mmss_cc_base_address(struct mdss_edp_drv_pdata
		*edp_drv)
{
	struct resource *res;

	res = platform_get_resource_byname(edp_drv->pdev, IORESOURCE_MEM,
			"mmss_cc_base");
	if (!res) {
		pr_err("%s: Unable to get the MMSS_CC resources", __func__);
		return -ENOMEM;
	}

	edp_drv->mmss_cc_base = ioremap(res->start, resource_size(res));
	if (!edp_drv->mmss_cc_base) {
		pr_err("%s: Unable to remap MMSS_CC resources",  __func__);
		return -ENOMEM;
	}

	return 0;
}

static void mdss_edp_fill_edid_data(struct mdss_edp_drv_pdata *edp_drv)
{
	struct edp_edid *edid = &edp_drv->edid;

	edid->id_name[0] = 'A';
	edid->id_name[0] = 'U';
	edid->id_name[0] = 'O';
	edid->id_name[0] = 0;
	edid->id_product = 0x305D;
	edid->version = 1;
	edid->revision = 4;
	edid->ext_block_cnt = 0;
	edid->video_digital = 0x5;
	edid->color_depth = 8;
	edid->dpm = 0;
	edid->color_format = 0;
	edid->timing[0].pclk = 268500000;
//	edid->timing[0].pclk = 810000000;
	edid->timing[0].h_addressable = 2560;
	edid->timing[0].h_blank = 160;
	edid->timing[0].v_addressable = 1600;
	edid->timing[0].v_blank = 46;
	edid->timing[0].h_fporch = 48;
	edid->timing[0].h_sync_pulse = 32;
	edid->timing[0].v_sync_pulse = 14;
	edid->timing[0].v_fporch = 8;
	edid->timing[0].width_mm =  256;
	edid->timing[0].height_mm = 144;
	edid->timing[0].h_border = 0;
	edid->timing[0].v_border = 0;
	edid->timing[0].interlaced = 0;
	edid->timing[0].stereo = 0;
	edid->timing[0].sync_type = 1;
	edid->timing[0].sync_separate = 1;
	edid->timing[0].vsync_pol = 0;
	edid->timing[0].hsync_pol = 0;

}

static void mdss_edp_fill_dpcd_data(struct mdss_edp_drv_pdata *edp_drv)
{
	struct dpcd_cap *cap = &edp_drv->dpcd;

	cap->max_lane_count = 4;
	cap->max_link_clk = 270;
}


static int __devinit mdss_edp_probe(struct platform_device *pdev)
{
	int ret;
	struct mdss_edp_drv_pdata *edp_drv;

	pr_info("%s", __func__);

	if (!pdev->dev.of_node) {
		pr_err("%s: Failed\n", __func__);
		return -EPERM;
	}

	test_edp_drv = edp_drv = devm_kzalloc(&pdev->dev, sizeof(*edp_drv), GFP_KERNEL);
	if (edp_drv == NULL) {
		pr_err("%s: Failed, could not allocate edp_drv", __func__);
		return -ENOMEM;
	}

	edp_drv->pdev = pdev;
	edp_drv->pdev->id = 1;
	edp_drv->clk_on = 0;

	ret = mdss_edp_get_base_address(edp_drv);
	if (ret)
		goto probe_err;

	ret = mdss_edp_get_mmss_cc_base_address(edp_drv);
	if (ret)
		goto edp_base_unmap;

	ret = mdss_edp_regulator_init(edp_drv);
	if (ret)
		goto mmss_cc_base_unmap;

	ret = mdss_edp_clk_init(edp_drv);
	if (ret)
		goto edp_clk_deinit;

	ret = mdss_edp_gpio_panel_en(edp_drv);
	if (ret)
		goto edp_clk_deinit;

	ret = mdss_edp_pwm_config(edp_drv);
	if (ret)
		goto edp_free_gpio_panel_en;

	mdss_edp_fill_edid_data(edp_drv);
	mdss_edp_fill_dpcd_data(edp_drv);
	mdss_edp_device_register(edp_drv);

	return 0;


edp_free_gpio_panel_en:
	gpio_free(edp_drv->gpio_panel_en);
edp_clk_deinit:
	mdss_edp_clk_deinit(edp_drv);
	mdss_edp_regulator_off(edp_drv);
mmss_cc_base_unmap:
	iounmap(edp_drv->mmss_cc_base);
edp_base_unmap:
	iounmap(edp_drv->edp_base);
probe_err:
	return ret;

}

static const struct of_device_id msm_mdss_edp_dt_match[] = {
	{.compatible = "qcom,mdss-edp"},
	{}
};
MODULE_DEVICE_TABLE(of, msm_mdss_edp_dt_match);

static struct platform_driver mdss_edp_driver = {
	.probe = mdss_edp_probe,
	.remove = __devexit_p(mdss_edp_remove),
	.shutdown = NULL,
	.driver = {
		.name = "mdss_edp",
		.of_match_table = msm_mdss_edp_dt_match,
	},
};

static int __init mdss_edp_init(void)
{
	int ret;

	ret = platform_driver_register(&mdss_edp_driver);
	if (ret) {
		pr_err("%s driver register failed", __func__);
		return ret;
	}

	return ret;
}
module_init(mdss_edp_init);

static void __exit mdss_edp_driver_cleanup(void)
{
	platform_driver_unregister(&mdss_edp_driver);
}
module_exit(mdss_edp_driver_cleanup);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("eDP controller driver");
