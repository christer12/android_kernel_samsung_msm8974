/*
 *  Copyright (C) 2012, Samsung Electronics Co. Ltd. All Rights Reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */
#include "../ssp.h"
#include <linux/qpnp/qpnp-adc.h>

/*************************************************************************/
/* factory Sysfs                                                         */
/*************************************************************************/

#define VENDOR		"SENSIRION"
#define CHIP_ID		"SHTC1"
#define DONE_CAL	3

#if defined (CONFIG_MACH_HLTEATT)
#define MODEL_NAME	"SM-N900A"
#elif defined (CONFIG_MACH_HLTEEUR)
#define MODEL_NAME	"SM-N9005"
#elif defined (CONFIG_MACH_HLTETMO)
#define MODEL_NAME	"SM-N900T"
#elif defined (CONFIG_MACH_HLTESPR)
#define MODEL_NAME	"SM-N900S"
#elif defined (CONFIG_MACH_HLTEVZW)
#define MODEL_NAME	"SM-N900V"
#elif defined (CONFIG_MACH_HLTEUSC)
#define MODEL_NAME	"SM-N900R4"
#else
#define MODEL_NAME	"SM-N900A"
#endif

#define CP_THM_ADC_SAMPLING_CNT 7
#if defined(CONFIG_MACH_JF_ATT) || defined(CONFIG_MACH_JF_TMO) || \
	defined(CONFIG_MACH_JF_EUR) || defined(CONFIG_MACH_JF_SPR) || \
	defined(CONFIG_MACH_JF_USC) || defined(CONFIG_MACH_JF_VZW)
/* {adc, temp*10}, -20 to +70 */
static struct cp_thm_adc_table temp_table_cp[] = {
	{200, 700}, {207, 690}, {214, 680}, {221, 670}, {248, 660},
	{235, 650}, {243, 640}, {251, 630}, {259, 620}, {267, 610},
	{276, 600}, {286, 590}, {295, 580}, {304, 570}, {314, 560},
	{324, 550}, {336, 540}, {348, 530}, {360, 520}, {372, 510},
	{383, 500}, {402, 490}, {422, 480}, {441, 470}, {461, 460},
	{480, 450}, {495, 440}, {510, 430}, {526, 420}, {542, 410},
	{558, 400}, {574, 390}, {591, 380}, {607, 370}, {624, 360},
	{641, 350}, {659, 340}, {677, 330}, {696, 320}, {705, 310},
	{733, 300}, {752, 290}, {771, 280}, {790, 270}, {808, 260},
	{828, 250}, {848, 240}, {869, 230}, {890, 220}, {910, 210},
	{931, 200}, {951, 190}, {971, 180}, {992, 170}, {1012, 160},
	{1032, 150}, {1053, 140}, {1074, 130}, {1095, 120}, {1116, 110},
	{1137, 100}, {1155, 90}, {1173, 80}, {1191, 70}, {1210, 60},
	{1228, 50}, {1245, 40}, {1262, 30}, {1279, 20}, {1296, 10},
	{1313, 0},
	{1322, -10}, {1331, -20}, {1340, -30}, {1350, -40}, {1359, -50},
	{1374, -60}, {1389, -70}, {1404, -80}, {1419, -90}, {1434, -100},
	{1446, -110}, {1458, -120}, {1470, -130}, {1483, -140},{1495, -150},
	{1504, -160}, {1513, -170}, {1522, -180}, {1532, -190}, {1542, -200},
 };
#endif
static int get_cp_thm_value(struct ssp_data *data)
{
	int err = 0;
	struct qpnp_vadc_result results;
		mutex_lock(&data->cp_temp_adc_lock);
	err = qpnp_vadc_read(LR_MUX6_PU2_AMUX_THM3, &results);
		mutex_unlock(&data->cp_temp_adc_lock);
		if (err) {
		pr_err("%s : error reading chn %d, rc = %d\n",
			__func__, LR_MUX6_PU2_AMUX_THM3, err);
			return err;
		}

	return results.adc_code / 100;
}

static int convert_adc_to_temp(struct ssp_data *data, unsigned int adc)
{
#if defined(CONFIG_MACH_JF_ATT) || defined(CONFIG_MACH_JF_TMO) || \
	defined(CONFIG_MACH_JF_EUR) || defined(CONFIG_MACH_JF_SPR) || \
	defined(CONFIG_MACH_JF_USC) || defined(CONFIG_MACH_JF_VZW)

	int low = 0;
	int high = 0;
	int mid = 0;
	u8 array_size = ARRAY_SIZE(temp_table_cp);

	if (temp_table_cp == NULL) {
		/* No voltage vs temperature table, using fake temp */
		return -990;
	}

	high = array_size - 1;

	while (low <= high) {
		mid = (low + high) / 2;
		if (temp_table_cp[mid].adc > adc)
			high = mid - 1;
		else if (temp_table_cp[mid].adc < adc)
			low = mid + 1;
		else
			break;
	}
	return temp_table_cp[mid].temperature;
#else
{
	int err = 0;
	struct qpnp_vadc_result results;
	mutex_lock(&data->cp_temp_adc_lock);
	err = qpnp_vadc_read(LR_MUX6_PU2_AMUX_THM3, &results);
	mutex_unlock(&data->cp_temp_adc_lock);
	if (err) {
		pr_err("%s : error reading chn %d, rc = %d\n",
			__func__, LR_MUX6_PU2_AMUX_THM3, err);
		return err;
	}

	return results.physical * 10;
}
#endif
}

static ssize_t temphumidity_vendor_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", VENDOR);
}

static ssize_t temphumidity_name_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", CHIP_ID);
}

static ssize_t engine_version_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	pr_info("[SSP] %s - engine_ver = %s_%s\n",
		__func__, MODEL_NAME, data->comp_engine_ver);

	return sprintf(buf, "%s_%s\n",
		MODEL_NAME, data->comp_engine_ver);
}

static ssize_t engine_version_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	kfree(data->comp_engine_ver);
	data->comp_engine_ver =
		    kzalloc(((strlen(buf)+1) * sizeof(char)), GFP_KERNEL);
	strncpy(data->comp_engine_ver, buf, strlen(buf)+1);
	pr_info("[SSP] %s - engine_ver = %s, %s\n",
		__func__, data->comp_engine_ver, buf);

	return size;
}

static ssize_t engine_version2_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	pr_info("[SSP] %s - engine_ver2 = %s_%s\n",
		__func__, MODEL_NAME, data->comp_engine_ver2);

	return sprintf(buf, "%s_%s\n",
		MODEL_NAME, data->comp_engine_ver2);
}

static ssize_t engine_version2_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	kfree(data->comp_engine_ver2);
	data->comp_engine_ver2 =
		    kzalloc(((strlen(buf)+1) * sizeof(char)), GFP_KERNEL);
	strncpy(data->comp_engine_ver2, buf, strlen(buf)+1);
	pr_info("[SSP] %s - engine_ver2 = %s, %s\n",
		__func__, data->comp_engine_ver2, buf);

	return size;
}

static ssize_t pam_adc_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	int adc = 0;
	if (data->bSspShutdown == true) {
		adc = 0;
		goto exit;
	}
		adc = get_cp_thm_value(data);
	/* pr_info("[SSP] %s cp_thm = %dmV\n", __func__, adc); */
exit:
	return sprintf(buf, "%d\n", adc);
}

static ssize_t pam_temp_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	int adc, temp;

	adc = get_cp_thm_value(data);
	if (adc < 0) {
		pr_err("[SSP] %s - reading adc failed.(%d)\n", __func__, adc);
		temp = adc;
	} else
		temp = convert_adc_to_temp(data, adc);
	pr_info("[SSP] %s cp_temperature(Celsius * 10) = %d\n",
		__func__, temp);
	return sprintf(buf, "%d\n", temp);
}

static ssize_t temphumidity_crc_check(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	char chTempBuf[2] = {0, 10};
	int iDelayCnt = 0, iRet;
	struct ssp_data *data = dev_get_drvdata(dev);

	data->uFactorydata[0] = 0xff;
	data->uFactorydataReady = 0;
	iRet = send_instruction(data, FACTORY_MODE,
		TEMPHUMIDITY_CRC_FACTORY, chTempBuf, 2);

	while (!(data->uFactorydataReady &
		(1 << TEMPHUMIDITY_CRC_FACTORY))
		&& (iDelayCnt++ < 50)
		&& (iRet == SUCCESS))
		msleep(20);

	if ((iDelayCnt >= 50) || (iRet != SUCCESS)) {
		pr_err("[SSP]: %s - Temphumidity check crc Timeout!! %d\n",
			__func__, iRet);
			goto exit;
	}

	mdelay(5);

	pr_info("[SSP] : %s -Check_CRC : %d\n", __func__,
			data->uFactorydata[0]);

exit:
	if (data->uFactorydata[0] == 1)
		return sprintf(buf, "%s\n", "OK");
	else if (data->uFactorydata[0] == 2)
		return sprintf(buf, "%s\n","NG_NC");
	else
		return sprintf(buf, "%s\n","NG");
}

ssize_t temphumidity_send_accuracy(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	u8 accuracy;

	if (kstrtou8(buf, 10, &accuracy) < 0) {
		pr_err("[SSP] %s - read buf is fail(%s)\n", __func__, buf);
		return size;
	}

	if (accuracy == DONE_CAL)
		ssp_send_cmd(data, MSG2SSP_AP_TEMPHUMIDITY_CAL_DONE);
	pr_info("[SSP] %s - accuracy = %d\n", __func__, accuracy);

	return size;
}

static DEVICE_ATTR(name, S_IRUGO, temphumidity_name_show, NULL);
static DEVICE_ATTR(vendor, S_IRUGO, temphumidity_vendor_show, NULL);
static DEVICE_ATTR(engine_ver, S_IRUGO | S_IWUSR | S_IWGRP,
	engine_version_show, engine_version_store);
static DEVICE_ATTR(engine_ver2, S_IRUGO | S_IWUSR | S_IWGRP,
	engine_version2_show, engine_version2_store);
static DEVICE_ATTR(cp_thm, S_IRUGO, pam_adc_show, NULL);
static DEVICE_ATTR(cp_temperature, S_IRUGO, pam_temp_show, NULL);
static DEVICE_ATTR(crc_check, S_IRUGO,
	temphumidity_crc_check, NULL);
static DEVICE_ATTR(send_accuracy,  S_IWUSR | S_IWGRP,
	NULL, temphumidity_send_accuracy);

static struct device_attribute *temphumidity_attrs[] = {
	&dev_attr_name,
	&dev_attr_vendor,
	&dev_attr_engine_ver,
	&dev_attr_engine_ver2,
	&dev_attr_cp_thm,
	&dev_attr_cp_temperature,
	&dev_attr_crc_check,
	&dev_attr_send_accuracy,
	NULL,
};

void initialize_temphumidity_factorytest(struct ssp_data *data)
{
	sensors_register(data->temphumidity_device,
		data, temphumidity_attrs, "temphumidity_sensor");
}

void remove_temphumidity_factorytest(struct ssp_data *data)
{
	if (data->comp_engine_ver != NULL)
		kfree(data->comp_engine_ver);
	if (data->comp_engine_ver2 != NULL)
		kfree(data->comp_engine_ver2);
	sensors_unregister(data->temphumidity_device, temphumidity_attrs);
}
