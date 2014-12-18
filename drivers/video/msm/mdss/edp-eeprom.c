/*
 * cypress_touchkey.c - Platform data for edp eeprom driver
 *
 * Copyright (C) 2011 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

/* #define SEC_TOUCHKEY_VERBOSE_DEBUG */

#include <linux/kernel.h>
#include <asm/unaligned.h>
#include <mach/cpufreq.h>
#include <linux/input/mt.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/earlysuspend.h>
#include <linux/regulator/consumer.h>
#include <asm/mach-types.h>
#include <linux/device.h>
/* #include <mach/msm8974-gpio.h> */
#include <linux/of_gpio.h>

struct edp_eeprom_platform_data {
	int gpio_sda;
	u32 sda_gpio_flags;
	int gpio_scl;
	u32 scl_gpio_flags;
};

struct edp_eeprom_info {
	struct i2c_client			*client;
	struct edp_eeprom_platform_data	*pdata;
};

struct i2c_client *pclient;
int eDP_TON_NDRA;

static int eeprom_i2c_read(struct i2c_client *client,
		u16 reg, u8 *val, unsigned int len)
{
	int err = 0;
	struct i2c_adapter *adapter = client->adapter;
	struct i2c_msg msg[2];
	u8 buf1[] = { reg >> 8, reg & 0xFF };

	msg[0].addr  = client->addr;
	msg[0].flags = 0x00;
	msg[0].len   = 2;
	msg[0].buf   = buf1;

	msg[1].addr  = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len   = 1;
	msg[1].buf   = val;

	err = i2c_transfer(adapter, msg, 2);

	if  (err == 2)
		pr_info("%s ok", __func__);
	else
		pr_info("%s fail err = %d", __func__, err);

	return err;

}

static int eeprom_i2c_write(struct i2c_client *client,
		u16 reg,  u8 val, unsigned int len)
{
	int err = 0;
	struct i2c_adapter *adapter = client->adapter;
	struct i2c_msg msg[2];
	u8 buf1[] = { reg >> 8, reg & 0xFF };

	msg[0].addr  = client->addr;
	msg[0].flags = 0x00;
	msg[0].len   = 2;
	msg[0].buf   = buf1;

	msg[1].addr  = client->addr;
	msg[1].flags = 0x00;
	msg[1].len   = 1;
	msg[1].buf   = &val;

	err = i2c_transfer(adapter, msg, 2);

	if  (err == 2)
		pr_info("%s ok", __func__);
	else
		pr_info("%s fail err = %d", __func__, err);

	return err;
}

int edp_tcon_read(u16 reg, u8 *val, unsigned int len)
{
	if (pclient)
		return eeprom_i2c_read(pclient, reg, val, len);
	else
		return -EFAULT;
}

int edp_tcon_write(u16 reg,  u8 val, unsigned int len)
{
	if (pclient)
		return eeprom_i2c_write(pclient, reg, val, len);
	else
		return -EFAULT;
}

static void eeprom_request_gpio(struct edp_eeprom_platform_data *pdata)
{
	gpio_tlmm_config(GPIO_CFG(pdata->gpio_scl, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 1);
	gpio_tlmm_config(GPIO_CFG(pdata->gpio_sda, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 1);
}

static int edp_eeprom_parse_dt(struct device *dev,
			struct edp_eeprom_platform_data *pdata)
{
	struct device_node *np = dev->of_node;

	/* reset, irq gpio info */
	pdata->gpio_scl = of_get_named_gpio_flags(np, "edp,scl-gpio",
				0, &pdata->scl_gpio_flags);
	pdata->gpio_sda = of_get_named_gpio_flags(np, "edp,sda-gpio",
				0, &pdata->sda_gpio_flags);

	pr_info("%s gpio_scl : %d , gpio_sda : %d", __func__, pdata->gpio_scl, pdata->gpio_sda);
	return 0;
}

static int __devinit edp_eeprom_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct edp_eeprom_platform_data *pdata;
	struct edp_eeprom_info *info;

	int error;
	int loop;
	u8 data;

	pr_info("%s", __func__);

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -EIO;

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(struct edp_eeprom_platform_data),
				GFP_KERNEL);
		if (!pdata) {
			dev_info(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		error = edp_eeprom_parse_dt(&client->dev, pdata);
		if (error)
			return error;
	} else
		pdata = client->dev.platform_data;

	eeprom_request_gpio(pdata);

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		dev_info(&client->dev, "%s: fail to memory allocation.\n", __func__);
	}

	pclient = info->client = client;
	info->pdata = pdata;

	i2c_set_clientdata(client, info);

	printk(KERN_INFO "%s ", __func__);
	for(loop = 0; loop < 11 ; loop++) {
		eDP_TON_NDRA |= eeprom_i2c_read(client, 0x300 + loop, &data, 1);
		printk(KERN_INFO "addr : 0x%x value : 0x%x", 0x300 + loop, data);
	}
	printk(KERN_INFO "\n");

	return error;
}

static int __devexit edp_eeprom_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id edp_eeprom_id[] = {
	{"edp_eeprom", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, edp_eeprom_id);

static struct of_device_id edp_eeprom_match_table[] = {
	{ .compatible = "edp,eeprom-control",},
	{ },
};
MODULE_DEVICE_TABLE(of, edp_eeprom_match_table);

struct i2c_driver edp_eeprom_driver = {
	.probe = edp_eeprom_probe,
	.remove = edp_eeprom_remove,
	.driver = {
		.name = "edp_eeprom",
		.owner = THIS_MODULE,
		.of_match_table = edp_eeprom_match_table,
		   },
	.id_table = edp_eeprom_id,
};

static int __init edp_eeprom_init(void)
{

	int ret = 0;

	pr_info("%s", __func__);

	ret = i2c_add_driver(&edp_eeprom_driver);
	if (ret) {
		printk(KERN_ERR "edp_eeprom_init registration failed. ret= %d\n",
			ret);
	}

	return ret;
}

static void __exit edp_eeprom_exit(void)
{
	pr_info("%s", __func__);
	i2c_del_driver(&edp_eeprom_driver);
}

late_initcall(edp_eeprom_init);
module_exit(edp_eeprom_exit);

MODULE_DESCRIPTION("edp eeprom driver");
MODULE_LICENSE("GPL");
