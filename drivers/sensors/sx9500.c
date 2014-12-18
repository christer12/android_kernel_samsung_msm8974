/*
 * Copyright (C) 2013 Samsung Electronics. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include "sx9500_reg.h"
#include "sensors_core.h"

#define VENDOR_NAME              "SEMTECH"
#define MODEL_NAME               "SX9500"
#define MODULE_NAME              "grip_sensor"

#define IDLE                     0
#define ACTIVE                   1

#define SX9500_MODE_SLEEP        0
#define SX9500_MODE_NORMAL       1

#define MAIN_SENSOR              0
#define REF_SENSOR               1

#define TOUCH_THRESHOLD          0x0D
#define RELEASE_THRESHOLD        0x0A

#define CSX_STATUS_REG           SX9500_TCHCMPSTAT_TCHSTAT0_FLAG

#define IRQ_PROCESS_CONDITION   (SX9500_IRQSTAT_TOUCH_FLAG \
				| SX9500_IRQSTAT_RELEASE_FLAG \
				| SX9500_IRQSTAT_COMPDONE_FLAG)

struct sx9500_p {
	struct i2c_client *client;
	struct input_dev *input;
	struct device *factory_device;
	struct delayed_work init_work;

	u8 touchTh;
	u8 releaseTh;

	int irq;
	int gpioNirq;
	int csxStatus;

	atomic_t enable;
};

static int sx9500_get_nirq_state(struct sx9500_p *data)
{
	return gpio_get_value_cansleep(data->gpioNirq);
}

static int sx9500_i2c_write(struct sx9500_p *data, u8 addr, u8 val)
{
	char buffer[2];
	int ret;

	buffer[0] = addr;
	buffer[1] = val;

	ret = i2c_master_send(data->client, buffer, 2);
	if (ret < 0)
		pr_err("[SX9500]: %s - i2c write error %d\n", __func__, ret);

	return ret;
}

static int sx9500_i2c_read(struct sx9500_p *data, u8 addr, u8 *val)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(data->client, addr);
	if (ret < 0)
		pr_err("[SX9500]: %s - i2c read error %d\n", __func__, ret);
	else
		*val = ret & 0x000000ff;

	return ret;
}

/*! \brief Perform a manual offset calibration
 * \param data Pointer to main parent struct
 * \return Value return value from the write register
 */
static int sx9500_manual_offset_calibration(struct sx9500_p *data)
{
	s32 ret = 0;

	ret = sx9500_i2c_write(data, SX9500_IRQSTAT_REG, 0xFF);

	return ret;
}

/*! \fn static int sx9500_read_irqstate(struct sx9500_p *data)
 * \brief Shortcut to read what caused interrupt.
 * \details data is to keep the drivers a unified
 * function that will read whatever register(s)
 * provide information on why the interrupt was caused.
 * \param data Pointer to main parent struct
 * \return If successful, Value of bit(s) that cause interrupt, else 0
 */
static u8 sx9500_read_irqstate(struct sx9500_p *data)
{
	u8 val = 0;

	if (sx9500_i2c_read(data, SX9500_IRQSTAT_REG, &val) >= 0)
		return (val & 0x00FF);

	return 0;
}

static void sx9500_initialize_register(struct sx9500_p *data)
{
	u8 val = 0;
	int idx;

	for (idx = 0; idx < (sizeof(setup_reg) >> 1); idx++) {
		sx9500_i2c_write(data, setup_reg[idx].reg, setup_reg[idx].val);
		pr_info("[SX9500]: %s - Write Reg: 0x%x Value: 0x%x\n",
			__func__, setup_reg[idx].reg, setup_reg[idx].val);

		sx9500_i2c_read(data, setup_reg[idx].reg, &val);
		pr_info("[SX9500]: %s - Read Reg: 0x%x Value: 0x%x\n\n",
			__func__, setup_reg[idx].reg, val);
	}

	msleep(100);
}

static void sx9500_initialize_chip(struct sx9500_p *data)
{
	int cnt = 0;
	while((sx9500_get_nirq_state(data) == 0) && (cnt++ < 10))
	{
		sx9500_read_irqstate(data);
		msleep(20);
	}

	if (cnt >= 10)
		pr_err("[SX9500]: %s - s/w reset fail(%d)\n", __func__, cnt);

	sx9500_initialize_register(data);
	sx9500_manual_offset_calibration(data);
}

static int sx9500_set_mode(struct sx9500_p *data, unsigned char mode)
{
	int ret = -EINVAL;

	if (mode == SX9500_MODE_SLEEP) {
		ret = sx9500_i2c_write(data,
			mode_reg[SX9500_MODE_SLEEP].reg,
			mode_reg[SX9500_MODE_SLEEP].val);
		disable_irq(data->irq);
	} else if (mode == SX9500_MODE_NORMAL) {
		ret = sx9500_i2c_write(data,
			mode_reg[SX9500_MODE_NORMAL].reg,
			mode_reg[SX9500_MODE_NORMAL].val);
		enable_irq(data->irq);

		/* make sure no interrupts are pending since enabling irq
		 * will only work on next falling edge */
		sx9500_read_irqstate(data);
	}

	pr_info("[SX9500]: %s - change the mode : %u\n", __func__, mode);
	return ret;
}

static void sx9500_set_enable(struct sx9500_p *data, int enable)
{
	int pre_enable = atomic_read(&data->enable);

	if (enable) {
		if (pre_enable == 0) {
			sx9500_set_mode(data, SX9500_MODE_NORMAL);
			atomic_set(&data->enable, 1);
		}
	} else {
		if (pre_enable == 1) {
			sx9500_set_mode(data, SX9500_MODE_SLEEP);
			atomic_set(&data->enable, 0);
		}
	}
}

static ssize_t sx9500_release_threshold_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sx9500_p *data = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", data->releaseTh);
}

static ssize_t sx9500_release_threshold_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;
	struct sx9500_p *data = dev_get_drvdata(dev);

	if (strict_strtoul(buf, 10, &val)) {
		pr_err("[SX9500]: %s - Invalid Argument\n", __func__);
		return -EINVAL;
	}

	pr_info("[SX9500]: %s - release threshold %lu\n", __func__, val);
	data->releaseTh = (unsigned char)val;

	return count;
}

static ssize_t sx9500_touch_threshold_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sx9500_p *data = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", data->touchTh);
}

static ssize_t sx9500_touch_threshold_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;
	struct sx9500_p *data = dev_get_drvdata(dev);

	if (strict_strtoul(buf, 10, &val)) {
		pr_err("[SX9500]: %s - Invalid Argument\n", __func__);
		return -EINVAL;
	}

	pr_info("[SX9500]: %s - touch threshold %lu\n", __func__, val);
	data->touchTh = (unsigned char)val;

	return count;
}

static ssize_t sx9500_manual_offset_calibration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u8 val = 0;
	struct sx9500_p *data = dev_get_drvdata(dev);

	sx9500_i2c_read(data, SX9500_IRQSTAT_REG, &val);

	return sprintf(buf, "%d\n", val);
}

static ssize_t sx9500_manual_offset_calibration_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;
	struct sx9500_p *data = dev_get_drvdata(dev);

	if (strict_strtoul(buf, 10, &val)) {
		pr_err("[SX9500]: %s - Invalid Argument\n", __func__);
		return -EINVAL;
	}

	if (val)
		sx9500_manual_offset_calibration(data);

	return count;
}

static ssize_t sx9500_register_write_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int regist = 0, val = 0;
	struct sx9500_p *data = dev_get_drvdata(dev);

	if (sscanf(buf, "%d,%d", &regist, &val) != 2) {
		pr_err("[SX9500]: %s - The number of data are wrong\n",
			__func__);
		return -EINVAL;
	}

	sx9500_i2c_write(data, (unsigned char)regist, (unsigned char)val);
	pr_info("[SX9500]: %s - Register(0x%2x) data(0x%2x)\n",
		__func__, regist, val);

	return count;
}

static ssize_t sx9500_register_read_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int regist = 0;
	unsigned char val = 0;
	struct sx9500_p *data = dev_get_drvdata(dev);

	if (sscanf(buf, "%d", &regist) != 1) {
		pr_err("[SX9500]: %s - The number of data are wrong\n",
			__func__);
		return -EINVAL;
	}

	sx9500_i2c_read(data, (unsigned char)regist, &val);
	pr_info("[SX9500]: %s - Register(0x%2x) data(0x%2x)\n",
		__func__, regist, val);

	return count;
}

static ssize_t sx9500_read_data_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u8 val, reg;
	struct sx9500_p *data = dev_get_drvdata(dev);

	sx9500_i2c_write(data, SX9500_REGSENSORSELECT, MAIN_SENSOR);
	for (reg = SX9500_REGUSEMSB; reg <= SX9500_REGOFFSETLSB; reg++) {
		sx9500_i2c_read(data, reg, &val);
		pr_info("[SX9500]: %s - Register(0x%2x) data(0x%2x)\n",
			__func__, reg, val);
	}

	return sprintf(buf, "%d\n", val);
}

static ssize_t sx9500_sw_reset_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct sx9500_p *data = dev_get_drvdata(dev);

	if (atomic_read(&data->enable) == 1)
		sx9500_set_mode(data, SX9500_MODE_SLEEP);

	ret = sx9500_i2c_write(data, SX9500_SOFTRESET_REG, SX9500_SOFTRESET);
	sx9500_initialize_chip(data);
	msleep(200);

	if (atomic_read(&data->enable) == 1)
		sx9500_set_mode(data, SX9500_MODE_NORMAL);

	return sprintf(buf, "%d\n", ret);
}

static ssize_t sx9500_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", VENDOR_NAME);
}

static ssize_t sx9500_name_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", MODEL_NAME);
}

static ssize_t sx9500_diff_data_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u8 msb, lsb;
	struct sx9500_p *data = dev_get_drvdata(dev);

	sx9500_i2c_write(data, SX9500_REGSENSORSELECT, MAIN_SENSOR);

	sx9500_i2c_read(data, SX9500_REGDIFFMSB, &msb);
	sx9500_i2c_read(data, SX9500_REGDIFFLSB, &lsb);
	pr_info("[SX9500]: %s - CS%d: 0x%x\n",
		__func__, MAIN_SENSOR, (msb << 8) | lsb);

	return snprintf(buf, PAGE_SIZE, "%u\n", (msb << 8) | lsb);
}

static DEVICE_ATTR(ontouch_threshold, S_IRUGO | S_IWUSR | S_IWGRP,
		sx9500_touch_threshold_show,
		sx9500_touch_threshold_store);
static DEVICE_ATTR(onrelease_threshold, S_IRUGO | S_IWUSR | S_IWGRP,
		sx9500_release_threshold_show,
		sx9500_release_threshold_store);
static DEVICE_ATTR(calibrate, S_IRUGO | S_IWUSR | S_IWGRP,
		sx9500_manual_offset_calibration_show,
		sx9500_manual_offset_calibration_store);
static DEVICE_ATTR(register_write, S_IRUGO | S_IWUSR | S_IWGRP,
		NULL, sx9500_register_write_store);
static DEVICE_ATTR(register_read, S_IRUGO | S_IWUSR | S_IWGRP,
		NULL, sx9500_register_read_store);
static DEVICE_ATTR(readback, S_IRUGO, sx9500_read_data_show, NULL);
static DEVICE_ATTR(reset, S_IRUGO, sx9500_sw_reset_show, NULL);
static DEVICE_ATTR(name, S_IRUGO, sx9500_name_show, NULL);
static DEVICE_ATTR(vendor, S_IRUGO, sx9500_vendor_show, NULL);
static DEVICE_ATTR(raw_data, S_IRUGO, sx9500_diff_data_show, NULL);

static struct device_attribute *sensor_attrs[] = {
	&dev_attr_calibrate,
	&dev_attr_ontouch_threshold,
	&dev_attr_onrelease_threshold,
	&dev_attr_register_write,
	&dev_attr_register_read,
	&dev_attr_readback,
	&dev_attr_reset,
	&dev_attr_name,
	&dev_attr_vendor,
	&dev_attr_raw_data,
	NULL,
};

/*****************************************************************************/
static ssize_t sx9500_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	u8 enable;
	int ret;
	struct sx9500_p *data = dev_get_drvdata(dev);

	ret = kstrtou8(buf, 2, &enable);
	if (ret) {
		pr_err("[SX9500]: %s - Invalid Argument\n", __func__);
		return ret;
	}

	pr_info("[SX9500]: %s - new_value = %u\n", __func__, enable);
	if ((enable == 0) || (enable == 1))
		sx9500_set_enable(data, (int)enable);

	return size;
}

static ssize_t sx9500_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sx9500_p *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&data->enable));
}

static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
		sx9500_enable_show, sx9500_enable_store);

static struct attribute *sx9500_attributes[] = {
	&dev_attr_enable.attr,
	NULL
};

static struct attribute_group sx9500_attribute_group = {
	.attrs = sx9500_attributes
};

static void sx9500_touch_process(struct sx9500_p *data)
{
	u8 status = 0;

	sx9500_i2c_read(data, SX9500_TCHCMPSTAT_REG, &status);

	if (data->csxStatus == IDLE) { /* User pressed button */
		if (status & CSX_STATUS_REG) {
			pr_info("######## [SX9500]: %s - cap button touched\n",
				__func__);
			input_report_rel(data->input, REL_MISC, ACTIVE + 1);
			input_sync(data->input);
			data->csxStatus = ACTIVE;
			sx9500_i2c_write(data, SX9500_CPS_CTRL6_REG,
				data->touchTh);
		} else {
			pr_info("[SX9500]: %s - Button already released.\n",
				__func__);
		}
	} else { /* User released button */
		if (!(status & CSX_STATUS_REG)) {
			pr_info("######## [SX9500]: %s - cap button released\n",
				__func__);
			input_report_rel(data->input, REL_MISC, IDLE + 1);
			input_sync(data->input);
			data->csxStatus = IDLE;
			sx9500_i2c_write(data, SX9500_CPS_CTRL6_REG,
				data->releaseTh);
		} else {
			pr_info("[SX9500]: %s - Button still touched\n",
				__func__);
		}
	}
}

static void sx9500_process_interrupt(struct sx9500_p *data)
{
	u8 status = 0;

	/* since we are not in an interrupt don't need to disable irq. */
	status = sx9500_read_irqstate(data);
	pr_info("[SX9500]: %s - Refresh Status 0x%x\n", __func__, status);

	if (status & IRQ_PROCESS_CONDITION)
		sx9500_touch_process(data);
}

static void sx9500_init_work_func(struct work_struct *work)
{
	struct sx9500_p *data = container_of((struct delayed_work *)work,
		struct sx9500_p, init_work);

	sx9500_initialize_chip(data);
}

static irqreturn_t sx9500_interrupt_thread(int irq, void *pdata)
{
	struct sx9500_p *data = pdata;

	if (sx9500_get_nirq_state(data) == 0)
		sx9500_process_interrupt(data);
	else
		pr_err("[SX9500]: %s - nirq read high %d\n",
			__func__, sx9500_get_nirq_state(data));

	return IRQ_HANDLED;
}

static int sx9500_input_init(struct sx9500_p *data)
{
	int ret = 0;
	struct input_dev *dev = NULL;

	/* Create the input device */
	dev = input_allocate_device();
	if (!dev)
		return -ENOMEM;

	dev->name = MODULE_NAME;
	dev->id.bustype = BUS_I2C;

	input_set_capability(dev, EV_REL, REL_MISC);
	input_set_drvdata(dev, data);

	ret = input_register_device(dev);
	if (ret < 0) {
		input_free_device(dev);
		return ret;
	}

	/* Setup sysfs */
	ret = sysfs_create_group(&dev->dev.kobj, &sx9500_attribute_group);
	if (ret < 0) {
		input_unregister_device(dev);
		return ret;
	}

	ret = sensors_create_symlink(&dev->dev.kobj, dev->name);
	if (ret < 0) {
		input_unregister_device(dev);
		return ret;
	}

	/* save the input pointer and finish initialization */
	data->input = dev;

	return 0;
}

static int sx9500_setup_pin(struct sx9500_p *data)
{
	int ret;

	ret = gpio_request(data->gpioNirq, "SX9500_nIRQ");
	if (ret < 0) {
		pr_err("[SX9500]: %s - gpio %d request failed (%d)\n",
			__func__, data->gpioNirq, ret);
		return ret;
	}

	ret = gpio_direction_input(data->gpioNirq);
	if (ret < 0) {
		pr_err("[SX9500]: %s - failed to set gpio %d as input (%d)\n",
			__func__, data->gpioNirq, ret);
		gpio_free(data->gpioNirq);
		return ret;
	}

	data->irq = gpio_to_irq(data->gpioNirq);

	/* initailize interrupt reporting */
	ret = request_threaded_irq(data->irq, NULL, sx9500_interrupt_thread,
			IRQF_TRIGGER_FALLING , "sx9500_irq", data);
	if (ret < 0) {
		pr_err("[SX9500]: %s - failed to set request_threaded_irq %d"
			" as returning (%d)\n", __func__, data->irq, ret);
		free_irq(data->irq, data);
		gpio_free(data->gpioNirq);
		return ret;
	}

	disable_irq(data->irq);
	return 0;
}

static void sx9500_initialize_variable(struct sx9500_p *data)
{
	data->csxStatus= IDLE;
	data->touchTh = TOUCH_THRESHOLD;
	data->releaseTh = RELEASE_THRESHOLD;
	atomic_set(&data->enable, 0);
}

static int sx9500_parse_dt(struct sx9500_p *data, struct device *dev)
{
	struct device_node *dNode = dev->of_node;
	enum of_gpio_flags flags;

	if (dNode == NULL)
		return -ENODEV;

	data->gpioNirq = of_get_named_gpio_flags(dNode,
		"sx9500-i2c,nirq-gpio", 0, &flags);
	if (data->gpioNirq < 0) {
		pr_err("[SENSOR]: %s - get gpioNirq error\n", __func__);
		return -ENODEV;
	}

	return 0;
}

static int sx9500_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret = -ENODEV;
	struct sx9500_p *data = NULL;

	pr_info("[SX9500]: %s - Probe Start!\n", __func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("[SX9500]: %s - i2c_check_functionality error\n",
			__func__);
		goto exit;
	}

	/* create memory for main struct */
	data = kzalloc(sizeof(struct sx9500_p), GFP_KERNEL);
	if (data == NULL) {
		pr_err("[SX9500]: %s - kzalloc error\n", __func__);
		ret = -ENOMEM;
		goto exit_kzalloc;
	}

	ret = sx9500_parse_dt(data, &client->dev);
	if (ret < 0) {
		pr_err("[SX9500]: %s - of_node error\n", __func__);
		ret = -ENODEV;
		goto exit_of_node;
	}

	ret = sx9500_setup_pin(data);
	if (ret) {
		pr_err("[SX9500]: %s - could not setup pin\n", __func__);
		goto exit_setup_pin;
	}

	i2c_set_clientdata(client, data);
	data->client = client;

	/* read chip id */
	ret = sx9500_i2c_write(data, SX9500_SOFTRESET_REG, SX9500_SOFTRESET);
	if (ret < 0) {
		pr_err("[SX9500]: %s - chip reset failed %d\n", __func__, ret);
		goto exit_chip_reset;
	}

	ret = sx9500_input_init(data);
	if (ret < 0)
		goto exit_input_init;

	sensors_register(data->factory_device, data, sensor_attrs, MODULE_NAME);
	sx9500_initialize_variable(data);

	INIT_DELAYED_WORK(&data->init_work, sx9500_init_work_func);
	schedule_delayed_work(&data->init_work, msecs_to_jiffies(1000));

	pr_info("[SX9500]: %s - Probe done!\n", __func__);

	return 0;

exit_input_init:
exit_chip_reset:
	free_irq(data->irq, data);
	gpio_free(data->gpioNirq);
exit_setup_pin:
exit_of_node:
	kfree(data);
exit_kzalloc:
exit:
	pr_err("[SX9500]: %s - Probe fail!\n", __func__);
	return ret;
}

static int __devexit sx9500_remove(struct i2c_client *client)
{
	struct sx9500_p *data = (struct sx9500_p *)i2c_get_clientdata(client);

	if (atomic_read(&data->enable) == 1)
		sx9500_set_mode(data, SX9500_MODE_SLEEP);

	cancel_delayed_work_sync(&data->init_work);
	free_irq(data->irq, data);
	gpio_free(data->gpioNirq);

	sensors_unregister(data->factory_device, sensor_attrs);
	sensors_remove_symlink(&data->input->dev.kobj, data->input->name);
	sysfs_remove_group(&data->input->dev.kobj, &sx9500_attribute_group);
	input_unregister_device(data->input);

	kfree(data);

	return 0;
}

static int sx9500_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sx9500_p *data = i2c_get_clientdata(client);

	if (atomic_read(&data->enable) == 1)
		sx9500_set_mode(data, SX9500_MODE_SLEEP);

	return 0;
}

static int sx9500_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sx9500_p *data = i2c_get_clientdata(client);

	if (atomic_read(&data->enable) == 1)
		sx9500_set_mode(data, SX9500_MODE_NORMAL);

	return 0;
}

static struct of_device_id sx9500_match_table[] = {
	{ .compatible = "sx9500-i2c",},
	{},
};

static const struct i2c_device_id sx9500_id[] = {
	{ "sx9500_match_table", 0 },
	{ }
};

static const struct dev_pm_ops sx9500_pm_ops = {
	.suspend = sx9500_suspend,
	.resume = sx9500_resume,
};

static struct i2c_driver sx9500_driver = {
	.driver = {
		.name	= MODEL_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = sx9500_match_table,
		.pm = &sx9500_pm_ops
	},
	.probe		= sx9500_probe,
	.remove		= __devexit_p(sx9500_remove),
	.id_table	= sx9500_id,
};

static int __init sx9500_init(void)
{
	return i2c_add_driver(&sx9500_driver);
}

static void __exit sx9500_exit(void)
{
	i2c_del_driver(&sx9500_driver);
}

module_init(sx9500_init);
module_exit(sx9500_exit);

MODULE_DESCRIPTION("Semtech Corp. SX9500 Capacitive Touch Controller Driver");
MODULE_AUTHOR("Samsung Electronics");
MODULE_LICENSE("GPL");
