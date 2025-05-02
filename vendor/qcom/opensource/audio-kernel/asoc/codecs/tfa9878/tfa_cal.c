/*
 * Copyright (C) 2014-2020 NXP Semiconductors, All Rights Reserved.
 * Copyright 2020 GOODIX, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include "inc/tfa_sysfs.h"

#define TFA_CAL_DEV_NAME	"tfa_cal"
#define FILESIZE_CAL	(10)

/* ---------------------------------------------------------------------- */

static ssize_t rdc_show(struct device *dev,
	struct device_attribute *attr, char *buf);
static ssize_t rdc_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size);
static DEVICE_ATTR_RW(rdc);

#if defined(CONFIG_TFA_STEREO_NODE) || defined(TFA_STEREO_NODE)
static ssize_t rdc_r_show(struct device *dev,
	struct device_attribute *attr, char *buf);
static ssize_t rdc_r_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size);
static DEVICE_ATTR_RW(rdc_r);
#endif /* (CONFIG_TFA_STEREO_NODE) || (TFA_STEREO_NODE) */

static ssize_t ref_temp_show(struct device *dev,
	struct device_attribute *attr, char *buf);
static ssize_t ref_temp_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size);
static DEVICE_ATTR_RW(ref_temp);

static ssize_t config_show(struct device *dev,
	struct device_attribute *attr, char *buf);
static ssize_t config_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size);
static DEVICE_ATTR(config, 0664, config_show, config_store);

static struct attribute *tfa_cal_attr[] = {
	&dev_attr_rdc.attr,
#if defined(CONFIG_TFA_STEREO_NODE) || defined(TFA_STEREO_NODE)
	&dev_attr_rdc_r.attr,
#endif /* (CONFIG_TFA_STEREO_NODE) || (TFA_STEREO_NODE) */
	&dev_attr_ref_temp.attr,
	&dev_attr_config.attr,
	NULL,
};

static struct attribute_group tfa_cal_attr_grp = {
	.attrs = tfa_cal_attr,
};

struct tfa_cal {
	int rdc;
	int temp;
};

/* ---------------------------------------------------------------------- */

static struct device *tfa_cal_dev;
static int cur_status;
static struct tfa_cal cal_data[MAX_HANDLES];

/* ---------------------------------------------------------------------- */

static ssize_t rdc_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int idx = tfa_get_dev_idx_from_inchannel(0);
	char cal_result[FILESIZE_CAL] = {0};
	int ret;

	if (idx > TFA_NOT_FOUND)
		snprintf(cal_result, FILESIZE_CAL,
			"%d", cal_data[idx].rdc);

	if (cal_result[0] == 0)
		ret = snprintf(buf, 7 + 1, "no_data");
	else
		ret = snprintf(buf, strlen(cal_result) + 1,
			"%s", cal_result);

	if (idx > TFA_NOT_FOUND && ret > 0)
		pr_info("%s: tfa_cal - dev %d - calibration data (rdc %d)\n",
			__func__, idx, cal_data[idx].rdc);
	else
		pr_err("%s: tfa_cal dev %d - error %d\n",
			__func__, idx, ret);

	return ret;
}

static ssize_t rdc_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int idx = tfa_get_dev_idx_from_inchannel(0);
	int ret = 0, value = 0;

	if(idx <= TFA_NOT_FOUND ) {
		pr_err("%s: TFA_NOT_FOUND \n", __func__);
		return -ENODEV;
	}

	ret = kstrtou32(buf, 10, &value);
	if (ret < 0) {
		pr_info("%s: wrong value: %s\n", __func__, buf);
		return -EINVAL;
	}

	cal_data[idx].rdc = value;
	pr_info("%s: tfa_cal - dev %d - calibration data (rdc %d)\n",
		__func__, idx, value);

	return size;
}

#if defined(CONFIG_TFA_STEREO_NODE) || defined(TFA_STEREO_NODE)
static ssize_t rdc_r_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int idx = tfa_get_dev_idx_from_inchannel(1);
	char cal_result[FILESIZE_CAL] = {0};
	int ret;

	if (idx > TFA_NOT_FOUND)
		snprintf(cal_result, FILESIZE_CAL,
			"%d", cal_data[idx].rdc);

	if (cal_result[0] == 0)
		ret = snprintf(buf, 7 + 1, "no_data");
	else
		ret = snprintf(buf, strlen(cal_result) + 1,
			"%s", cal_result);

	if (idx > TFA_NOT_FOUND && ret > 0)
		pr_info("%s: tfa_cal - dev %d - calibration data (rdc %d)\n",
			__func__, idx, cal_data[idx].rdc);
	else
		pr_err("%s: tfa_cal dev %d - error %d\n",
			__func__, idx, ret);

	return ret;
}

static ssize_t rdc_r_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int idx = tfa_get_dev_idx_from_inchannel(1);
	int ret = 0, value = 0;

	if(idx <= TFA_NOT_FOUND ) {
		pr_err("%s: TFA_NOT_FOUND \n", __func__);
		return -ENODEV;
	}

	ret = kstrtou32(buf, 10, &value);
	if (ret < 0) {
		pr_info("%s: wrong value: %s\n", __func__, buf);
		return -EINVAL;
	}

	cal_data[idx].rdc = value;
	pr_info("%s: tfa_cal - dev %d - calibration data (rdc %d)\n",
		__func__, idx, value);

	return size;
}
#endif /* (CONFIG_TFA_STEREO_NODE) || (TFA_STEREO_NODE) */

static ssize_t ref_temp_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	u16 temp_val = DEFAULT_REF_TEMP;
	char cal_result[FILESIZE_CAL] = {0};
	int size;
	enum tfa98xx_error ret;

	/* EXT_TEMP */
	ret = tfa98xx_read_reference_temp(&temp_val);
	if (ret)
		pr_err("error in reading reference temp\n");

	snprintf(cal_result, FILESIZE_CAL,
		"%d", temp_val);

	if (cal_result[0] == 0)
		size = snprintf(buf, 7 + 1, "no_data");
	else
		size = snprintf(buf, strlen(cal_result) + 1,
			"%s", cal_result);

	if (size > 0)
		pr_info("%s: tfa_cal - ref_temp %d for calibration\n",
			__func__, temp_val);
	else
		pr_err("%s: tfa_cal - ref_temp error\n",
			__func__);

	return size;
}

static ssize_t ref_temp_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	pr_info("%s: not allowed to write temperature for calibration\n",
		__func__);

	return size;
}

static ssize_t config_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int size;
	char cal_state[FILESIZE_CAL] = {0};

	snprintf(cal_state, FILESIZE_CAL,
		"%s", cur_status ?
		"configured" /* calibration is active */
		: "not configured"); /* calibration is inactive */

	size = snprintf(buf, strlen(cal_state) + 1,
		"%s", cal_state);

	return size;
}

static ssize_t config_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = 0, status;

	/* Compare string, excluding the trailing \0 and the potentials eol */
	if (!sysfs_streq(buf, "1") && !sysfs_streq(buf, "0")
		 && !sysfs_streq(buf, "-1")) {
		pr_info("%s: tfa_cal invalid value to configure calibration\n",
			__func__);
		return -EINVAL;
	}

	ret = kstrtou32(buf, 10, &status);
	if (status != 1) {
		pr_info("%s: tfa_cal to restore setting after calibration\n",
			__func__);
		cur_status = 0; /* done - changed to inactive */
		tfa_restore_after_cal(0, status);
		return -EINVAL;
	}
	if (cur_status)
		pr_info("%s: tfa_cal already configured\n", __func__);

	pr_info("%s: tfa_cal triggered\n", __func__);

	memset(cal_data, 0, sizeof(struct tfa_cal) * MAX_HANDLES);

	/* configure registers for calibration */
	ret = tfa_run_cal(0, NULL);
	if (ret != TFA98XX_ERROR_FAIL) {
		pr_info("%s: tfa_cal configured\n", __func__);
		cur_status = status; /* run - changed to active */

		return size;
	}

	pr_info("%s: tfa_cal failed to calibrate speaker\n", __func__);
	cur_status = 0; /* done - changed to inactive */

	return -EINVAL;
}

int tfa98xx_cal_init(struct class *tfa_class)
{
	int ret = 0;

	if (tfa_class) {
		tfa_cal_dev = device_create(tfa_class,
			NULL, DEV_ID_TFA_CAL, NULL, TFA_CAL_DEV_NAME);
		if (!IS_ERR(tfa_cal_dev)) {
			ret = sysfs_create_group(&tfa_cal_dev->kobj,
				&tfa_cal_attr_grp);
			if (ret)
				pr_err("%s: failed to create sysfs group. ret (%d)\n",
					__func__, ret);
		}
	}

	pr_info("%s: initialized (%d)\n", __func__,
		(tfa_class != NULL) ? 1 : 0);

	return ret;
}

void tfa98xx_cal_exit(struct class *tfa_class)
{
	device_destroy(tfa_class, DEV_ID_TFA_CAL);
	pr_info("exited\n");
}
