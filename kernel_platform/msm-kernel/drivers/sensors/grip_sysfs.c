#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/pm_wakeup.h>
#include <asm/unaligned.h>
#include <linux/regulator/consumer.h>
#include <linux/sec_class.h>
#include <linux/pinctrl/consumer.h>
#include <linux/version.h>

#include "grip_common.h"
#include "a96t396.h"
#include "grip_sysfs.h"


static ssize_t grip_sar_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct grip_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n", !data->skip_event);
}

static ssize_t grip_sar_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct grip_data *data = dev_get_drvdata(dev);
	int ret, enable, i;

	ret = sscanf(buf, "%2d", &enable);
	if (ret != 1) {
		GRIP_ERR("cmd read err\n");
		return count;
	}

	if (!(enable >= 0 && enable <= 3)) {
		GRIP_ERR("wrong cmd %d\n", enable);
		return count;
	}

	GRIP_INFO("enable %d\n", enable);

	/* enable 0:off, 1:on, 2:skip event , 3:cancel skip event */
	if (enable == 2) {
		data->skip_event = true;
		data->motion = 1;
		for (i = 0; i < NUM_OF_MAX_CHANNEL; i++) {
			if (is_unavailable_channel(data->ch[i]))
				continue;
			data->ch[i]->is_unknown_mode = UNKNOWN_OFF;
			data->ch[i]->first_working = false;
			input_report_rel(data->input_dev, data->ch[i]->grip_code, IDLE);
			input_report_rel(data->input_dev, data->ch[i]->unknown_code, UNKNOWN_OFF);
		}
		input_sync(data->input_dev);
	} else if (enable == 3) {
		data->skip_event = false;
	} else {
		grip_set_enable(data, enable);
	}

	return count;
}

static ssize_t grip_sw_reset_ready_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct grip_data *data = dev_get_drvdata(dev);
	int ret;
	int retry = 5;
	u8 r_buf[1] = {0};

	GRIP_INFO("Wait start\n");

	if (data->check_abnormal_working == true) {
		GRIP_INFO("abnormal working, skip reset ready\n");
		return snprintf(buf, PAGE_SIZE, "0\n");
	}

	/* To garuantee grip sensor sw reset delay*/
	msleep(1000);

	while (retry--) {
		ret = grip_i2c_read(data->client, REG_SW_RESET, r_buf, 1);
		if (r_buf[0] == 0x20) //have to use define nunber
			break;
		else if (r_buf[0] == 0x11)
			GRIP_INFO("reset in progress(%d)\n", retry);
		if (ret < 0) {
			GRIP_ERR("i2c err %d\n", retry);
			return snprintf(buf, PAGE_SIZE, "0\n");
		}
		msleep(100);
	}

	if (r_buf[0] == 0x20) {
		GRIP_INFO("reset done");
		grip_check_diff_and_cap(data);

		return snprintf(buf, PAGE_SIZE, "1\n");
	}

	GRIP_INFO("expect 0x20 read 0x%x\n", r_buf[0]);
	return snprintf(buf, PAGE_SIZE, "0\n");
}

static ssize_t grip_sw_reset_show(struct device *dev,
	struct device_attribute *attr, const char *buf,
	size_t count)
{
	struct grip_data *data = dev_get_drvdata(dev);
	u8 cmd;
	int i, ret;

	ret = kstrtou8(buf, 2, &cmd);
	if (ret) {
		GRIP_ERR("cmd read err\n");
		return count;
	}

	if (!(cmd == 1)) {
		GRIP_ERR("wrong cmd %d\n", cmd);
		return count;
	}

	for (i = 0; i < NUM_OF_MAX_CHANNEL; i++) {
		if (is_unavailable_channel(data->ch[i]))
			continue;
		data->ch[i]->grip_event = 0;
	}

	GRIP_INFO("cmd %d\n", cmd);

	grip_sw_reset(data);

	return count;
}

static ssize_t grip_ref_cap_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct grip_data *data = dev_get_drvdata(dev);

	u8 r_buf[2];
	int ref_cap;
	int ret;

	ret = grip_i2c_read(data->client, REG_REF_CAP, r_buf, 2);
	if (ret < 0) {
		GRIP_ERR("err %d\n", ret);
		return snprintf(buf, PAGE_SIZE, "%d\n", 0);
	}

	ref_cap = (r_buf[0] << 8) | r_buf[1];
	do_div(ref_cap, 100);

	GRIP_INFO("Ref Cap %x,%x\n", r_buf[0], r_buf[1]);
	GRIP_INFO("Ref Cap / 100 %d\n", ref_cap);

	return sprintf(buf, "%d\n", ref_cap);
}

static ssize_t grip_name_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct grip_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%s\n", device_name[data->ic_num]);
}

static ssize_t grip_vendor_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", VENDOR_NAME);
}

static ssize_t bin_fw_ver_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct grip_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "0x%02x%02x\n",
		data->firmware.md_ver_bin, data->firmware.fw_ver_bin);
}

static ssize_t read_fw_ver_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct grip_data *data = dev_get_drvdata(dev);
	int ret;
	bool check_abnormal = data->check_abnormal_working;

	data->check_abnormal_working = false;

	ret = grip_get_fw_version(data, false, true);
	if (ret < 0) {
		GRIP_ERR("read err\n");
		data->firmware.fw_ver = 0;
	}

	if (check_abnormal)
		data->check_abnormal_working = true;

	return snprintf(buf, PAGE_SIZE, "0x%02x%02x\n",
		data->firmware.md_ver, data->firmware.fw_ver);
}

static ssize_t grip_fw_update_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct grip_data *data = dev_get_drvdata(dev);
	int ret;
	u8 cmd;

	switch (*buf) {
	case 's':
	case 'S':
		cmd = BUILT_IN;
		break;
	case 'i':
	case 'I':
		cmd = SDCARD;
		break;
	default:
		data->firmware.fw_update_state = 2;
		goto fw_update_out;
	}

	if (data->is_irq_active) {
		disable_irq(data->irq);
		data->is_irq_active = false;
	}

	data->firmware.fw_update_state = 1;
	data->enabled = false;

	if (cmd == BUILT_IN) {
		ret = grip_load_fw_kernel(data);
		if (ret < 0) {
			GRIP_ERR("load fw err(%d)\n", ret);
			goto fw_update_out;
		} else {
			GRIP_INFO("fw ver read success %d\n", ret);
		}
	} else {
		data->firmware.firm_data_ums = NULL;
	}
	ret = grip_flash_fw(data, false, cmd);

	data->enabled = true;

	if (!data->is_irq_active) {
		enable_irq(data->irq);
		data->is_irq_active = true;
	}
	if (ret) {
		GRIP_ERR("flash fw err %d\n", ret);
		data->firmware.fw_update_state = 2;
	} else {
		GRIP_INFO("success\n");
		data->firmware.fw_update_state = 0;
	}

	if (data->current_state) {
		cmd = CMD_ON;
		ret = grip_i2c_write(data->client, REG_SAR_ENABLE, &cmd);
		if (ret < 0)
			GRIP_INFO("enable irq err\n");

		data->is_first_event = true;
	}

#if defined(CONFIG_SENSORS_A96T396_LDO_SHARE)
	GRIP_INFO("register recovery\n");
	input_report_rel(data->input_dev, REL_WHEEL, 1);
	input_sync(data->input_dev);
#endif

fw_update_out:
	GRIP_INFO("fw_update_state %d\n", data->firmware.fw_update_state);

	return count;
}

static ssize_t grip_fw_update_status_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct grip_data *data = dev_get_drvdata(dev);
	int count = 0;

	GRIP_INFO("%d\n", data->firmware.fw_update_state);

	if (data->firmware.fw_update_state == 0)
		count = snprintf(buf, PAGE_SIZE, "PASS\n");
	else if (data->firmware.fw_update_state == 1)
		count = snprintf(buf, PAGE_SIZE, "Downloading\n");
	else if (data->firmware.fw_update_state == 2)
		count = snprintf(buf, PAGE_SIZE, "Fail\n");

	return count;
}

static ssize_t grip_irq_state_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct grip_data *data = dev_get_drvdata(dev);
	int status = 0;

	status = gpio_get_value(data->grip_int);
	GRIP_INFO("status=%d\n", status);

	return snprintf(buf, PAGE_SIZE, "%d\n", status);
}

static ssize_t grip_irq_en_cnt_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct grip_data *data = dev_get_drvdata(dev);

	GRIP_INFO("irq_en_cnt=%d\n", data->irq_en_cnt);

	return snprintf(buf, PAGE_SIZE, "%d\n", data->irq_en_cnt);
}

static ssize_t grip_crc_check_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct grip_data *data = dev_get_drvdata(dev);

#ifndef CONFIG_SENSORS_A96T396_CRC_CHECK
	int ret;
	unsigned char cmd[3] = {0x0A, 0x00, 0x10};
	unsigned char checksum[2] = {0, };

	i2c_master_send(data->client, cmd, 3);
	usleep_range(50 * 1000, 50 * 1000);

	ret = grip_i2c_read(data->client, 0x0A, checksum, 2);

	if (ret < 0) {
		GRIP_ERR("err\n");
		return snprintf(buf, PAGE_SIZE, "NG,0000\n");
	}

	GRIP_INFO("CRC:%02x%02x, BIN:%02x%02x\n", checksum[0], checksum[1],
		data->firmware.checksum_h_bin, data->firmware.checksum_l_bin);

	if ((checksum[0] != data->firmware.checksum_h_bin) ||
		(checksum[1] != data->firmware.checksum_l_bin))
		return snprintf(buf, PAGE_SIZE, "NG,%02x%02x\n",
			checksum[0], checksum[1]);
	else
		return snprintf(buf, PAGE_SIZE, "OK,%02x%02x\n",
			checksum[0], checksum[1]);
#else
	{
		int val;

		val = grip_crc_check(data);

		if (data->crc_check == CRC_PASS)
			return snprintf(buf, PAGE_SIZE, "OK,%02x\n", val);
		else
			return snprintf(buf, PAGE_SIZE, "NG,%02x\n", val);
	}
#endif
}

static ssize_t grip_motion_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct grip_data *data = dev_get_drvdata(dev);

	if (data->motion)
		return snprintf(buf, PAGE_SIZE, "motion_detect\n");
	else
		return snprintf(buf, PAGE_SIZE, "motion_non_detect\n");
}

static ssize_t grip_motion_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int val;
	int ret;
	struct grip_data *data = dev_get_drvdata(dev);

	ret = kstrtoint(buf, 10, &val);
	if (ret) {
		GRIP_ERR("strtoint err\n");
		return ret;
	}

	if (val == 0) {
		GRIP_INFO("motion event off\n");
		data->motion = val;
	} else if (val == 1) {
		GRIP_INFO("motion event\n");
		data->motion = val;
	} else {
		GRIP_INFO("Invalid val %u\n", val);
	}

	GRIP_INFO("%u\n", val);
	return count;
}

static ssize_t grip_noti_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct grip_data *data = dev_get_drvdata(dev);
	int i;

	for (i = 0; i < NUM_OF_MAX_CHANNEL; i++) {
		if (is_unavailable_channel(data->ch[i]))
			continue;

		GRIP_INFO("%dch noti_enable %d\n", i + 1, data->ch[i]->noti_enable);
		return sprintf(buf, "%d\n", data->ch[i]->noti_enable);
	}
	return sprintf(buf, "%d\n", 0);
}

static ssize_t grip_noti_enable_store(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t size)
{
	int ret, i;
	u8 enable;
	struct grip_data *data = dev_get_drvdata(dev);

	ret = kstrtou8(buf, 2, &enable);
	if (ret) {
		GRIP_ERR("strtou8 val\n");
		return size;
	}

	GRIP_INFO("new val %d\n", (int)enable);

	for (i = 0; i < NUM_OF_MAX_CHANNEL; i++) {
		if (is_unavailable_channel(data->ch[i]))
			continue;

		if ((data->unknown_ch_selection >> i) & 0x01) {
			data->ch[i]->noti_enable = enable;
			enter_unknown_mode(data, TYPE_BOOT, i);
		}
	}

	return size;
}

#if defined(CONFIG_SENSORS_A96T396_LDO_SHARE)
static ssize_t grip_register_recover_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct grip_data *data = dev_get_drvdata(dev);
	int ret = 0;
	u8 reg_value = 0;
	u8 cmd = 0;
	u8 check = 0;

	GRIP_INFO("start\n");
	ret = kstrtou8(buf, 10, &check);

	if (check == 1) {
		//register reset
		ret = grip_i2c_read(data->client, REG_SAR_ENABLE, &reg_value, 1);
		if (ret < 0) {
			GRIP_ERR("fail(%d)\n", ret);
			return size;
		}

		GRIP_INFO("reg=0x24 val=%02X\n", reg_value);

		if (data->current_state) {
			if (reg_value == CMD_OFF) {
				GRIP_INFO("register recover after HW reset\n");
				cmd = CMD_ON;
				ret = grip_i2c_write(data->client, REG_SAR_ENABLE, &cmd);
				if (ret < 0)
					GRIP_INFO("enable err %d\n", ret);

				data->is_first_event = true;
			}
		}

		GRIP_INFO("reg=0x25 val=%02X\n", reg_value);
	} else {
		GRIP_INFO("Unsupport cmd\n");
	}

	return size;
}
#endif

static ssize_t grip_ch_count_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct grip_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", data->num_of_channels);
}

static ssize_t grip_tuning_version_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct grip_data *data = dev_get_drvdata(dev);
	int ret;
	u8 tuning_ver, project_id;

	ret = grip_i2c_read(data->client, REG_PROJECT_ID, &project_id, 1);
	if (ret < 0) {
		GRIP_ERR("err %d\n", ret);
		project_id = 0;
	}

	ret = grip_i2c_read(data->client, REG_TUNING_VER, &tuning_ver, 1);
	if (ret < 0) {
		GRIP_ERR("err %d\n", ret);
		tuning_ver = 0;
	}

	return snprintf(buf, PAGE_SIZE, "0x%02x%02x\n", project_id, tuning_ver);
}

static ssize_t grip_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct grip_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", data->current_state);
}

void get_grip_gain(struct grip_data *data, int ch_num, u8 *buf)
{
	const u8 reg_gain[2] = {REG_GAINDATA, REG_GAINDATA_2CH};
	int ret;

	ret = grip_i2c_read(data->client, reg_gain[ch_num], &buf[0], 1);
	if (ret < 0)
		GRIP_ERR("%dch rst err %d\n", ch_num + 1, ret);

	ret = grip_i2c_read(data->client, reg_gain[ch_num] + 3, &buf[1], 1);
	if (ret < 0)
		GRIP_ERR("%dch int err %d\n", ch_num + 1, ret);

	ret = grip_i2c_read(data->client, REG_REF_GAINDATA, &buf[2], 1);
	if (ret < 0)
		GRIP_ERR("%dch ref rst err %d\n", ch_num + 1, ret);

	ret = grip_i2c_read(data->client, REG_REF_GAINDATA + 3, &buf[3], 1);
	if (ret < 0)
		GRIP_ERR("%dch ref int err %d\n", ch_num + 1, ret);

	GRIP_INFO("%dch Gain %d,%d\n", ch_num + 1, (int)buf[0], (int)buf[1]);
	GRIP_INFO("Ref Gain %d,%d\n", (int)buf[2], (int)buf[3]);
}

void write_sar_press_threshold(struct grip_data *data, int threshold, int ch_num)
{
	int ret;
	u8 cmd[2];
	const u8 reg_thd[NUM_OF_MAX_CHANNEL] = {REG_SAR_THRESHOLD, REG_SAR_THRESHOLD_2CH};

	if (threshold > 0xff) {
		cmd[0] = (threshold >> 8) & 0xff;
		cmd[1] = 0xff & threshold;
	} else if (threshold < 0) {
		cmd[0] = 0x0;
		cmd[1] = 0x0;
	} else {
		cmd[0] = 0x0;
		cmd[1] = (u8)threshold;
	}

	GRIP_INFO("buf %d, threshold %d\n", threshold,
			(cmd[0] << 8) | cmd[1]);

	ret = grip_i2c_write(data->client, reg_thd[ch_num], &cmd[0]);
	if (ret != 0) {
		GRIP_INFO("fail to write 2ch data1");
		return;
	}
	ret = grip_i2c_write(data->client, reg_thd[ch_num] + 0x01, &cmd[1]);
	if (ret != 0) {
		GRIP_INFO("fail to write 2ch data2");
		return;
	}
}

void write_sar_release_threshold(struct grip_data *data, int threshold, int ch_num)
{
	int ret;
	u8 cmd[2];
	const u8 reg_thd[NUM_OF_MAX_CHANNEL] = {REG_SAR_THRESHOLD, REG_SAR_THRESHOLD_2CH};

	if (threshold > 0xff) {
		cmd[0] = (threshold >> 8) & 0xff;
		cmd[1] = 0xff & threshold;
	} else if (threshold < 0) {
		cmd[0] = 0x0;
		cmd[1] = 0x0;
	} else {
		cmd[0] = 0x0;
		cmd[1] = (u8)threshold;
	}

	GRIP_INFO("buf %d, threshold %d\n", threshold, (cmd[0] << 8) | cmd[1]);

	ret = grip_i2c_write(data->client, reg_thd[ch_num] + 0x02,
				&cmd[0]);
	GRIP_INFO("ret %d\n", ret);

	if (ret < 0) {
		GRIP_INFO("fail to write 2ch data1");
		goto release_threshold_out;
	}
	ret = grip_i2c_write(data->client, reg_thd[ch_num] + 0x03,
				&cmd[1]);
	GRIP_INFO("ret %d\n", ret);
	if (ret < 0) {
		GRIP_INFO("fail to write 2ch data2");
		goto release_threshold_out;
	}
release_threshold_out:
	return;
}

int get_max_diff_on_irq_count(struct grip_data *data, int ch_num, s16 *max_diff_val)
{
	int result = 0;

	if (data->irq_count) {
		result = -1;
		*max_diff_val = data->ch[ch_num]->max_diff;
	} else {
		*max_diff_val = data->ch[ch_num]->max_normal_diff;
	}
	return result;
}

void initialize_irq_count_for_factory_test(struct grip_data *data, int ch_num, u8 onoff)
{
	mutex_lock(&data->lock);
	if (onoff == 0) {
		data->abnormal_mode = 0;
	} else if (onoff == 1) {
		data->abnormal_mode = 1;
		data->irq_count = 0;
		data->ch[ch_num]->max_diff = 0;
		data->ch[ch_num]->max_normal_diff = 0;
	} else {
		GRIP_ERR("Invalid val %d\n", onoff);
	}
	mutex_unlock(&data->lock);

	GRIP_INFO("onoff %d\n", onoff);
}

int grip_get_raw_data(struct grip_data *data, u8 ch_num)
{
	int ret;
	u8 r_buf[4] = {0,};
	u8 raw_reg[NUM_OF_MAX_CHANNEL] = {REG_SAR_RAWDATA, REG_SAR_RAWDATA_2CH};
	struct channel *ch = data->ch[ch_num];

	ret = grip_i2c_read(data->client, raw_reg[ch->num], r_buf, 2);
	if (ret < 0) {
		GRIP_ERR("err %d\n", ret);
		data->ch[ch_num]->grip_raw = 0;
		return ret;
	}

	ch->grip_raw = (r_buf[0] << 8) | r_buf[1];

	GRIP_INFO("%dch grip_raw %d\n", ch->num + 1, ch->grip_raw);

	return ret;
}

void get_baseline(struct grip_data *data, int ch_num)
{
	int ret;
	u8 r_buf[2];
	const u8 reg_baseline[NUM_OF_MAX_CHANNEL] = {REG_SAR_BASELINE, REG_SAR_BASELINE_2CH};

	ret = grip_i2c_read(data->client, reg_baseline[ch_num], r_buf, 2);
	if (ret < 0) {
		GRIP_ERR("err %d\n", ret);
		data->ch[ch_num]->grip_baseline = 0;
		return;
	}
	data->ch[ch_num]->grip_baseline = (r_buf[0] << 8) | r_buf[1];
}

void get_grip_diff(struct grip_data *data, int ch_num)
{
	int ret;
	u8 r_buf[4] = {0,};
	const u8 reg_diff[NUM_OF_MAX_CHANNEL] = {REG_SAR_DIFFDATA, REG_SAR_DIFFDATA_D_2CH};

	ret = grip_i2c_read(data->client, reg_diff[ch_num], r_buf, 4);
	if (ret < 0)
		GRIP_ERR("err %d\n", ret);

	data->ch[ch_num]->diff = (r_buf[0] << 8) | r_buf[1];
	data->ch[ch_num]->diff_d = (r_buf[2] << 8) | r_buf[3];
}

int get_total_cap(struct grip_data *data, int ch_num)
{
	int ret;
	int value = 0;
	u8 r_buf[2];
	const u8 reg_total_cap[NUM_OF_MAX_CHANNEL] = {REG_SAR_TOTALCAP_READ, REG_SAR_TOTALCAP_READ_2CH};

	ret = grip_i2c_read(data->client, reg_total_cap[ch_num], r_buf, 2);
	if (ret < 0) {
		GRIP_ERR("err %d\n", ret);
		return value;
	}
	value = (r_buf[0] << 8) | r_buf[1];

	return value;
}

#ifdef CONFIG_SENSORS_A96T396_CH1

static ssize_t grip_1ch_threshold_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct grip_data *data = dev_get_drvdata(dev);

	get_threshold(data, data->ch[0]->num);

	return sprintf(buf, "%u,%u,%u\n",
		data->ch[0]->grip_p_thd, data->ch[0]->grip_r_thd, data->ch[0]->grip_n_thd);
}

static ssize_t grip_1ch_total_cap_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct grip_data *data = dev_get_drvdata(dev);
	int value;

	value = get_total_cap(data, data->ch[0]->num);

	return snprintf(buf, PAGE_SIZE, "%d\n", value / 100);
}

static ssize_t grip_1ch_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct grip_data *data = dev_get_drvdata(dev);

	get_grip_diff(data, data->ch[0]->num);

	return sprintf(buf, "%u,%u\n", data->ch[0]->diff, data->ch[0]->diff_d);
}

static ssize_t grip_1ch_baseline_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct grip_data *data = dev_get_drvdata(dev);

	get_baseline(data, data->ch[0]->num);

	return snprintf(buf, PAGE_SIZE, "%u\n", data->ch[0]->grip_baseline);
}

static ssize_t grip_1ch_raw_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct grip_data *data = dev_get_drvdata(dev);
	int ret;

	ret = grip_get_raw_data(data, 0);
	if (ret < 0)
		return sprintf(buf, "%d\n", 0);
	else
		return sprintf(buf, "%u,%u\n", data->ch[0]->grip_raw,
				data->ch[0]->grip_raw_d);
}

static ssize_t grip_gain_1ch_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct grip_data *data = dev_get_drvdata(dev);
	u8 ch1_rst_buf, ref_rst_buf;
	u8 ch1_int_buf, ref_int_buf;
	u8 gain_buf[4] = {0};

	get_grip_gain(data, data->ch[0]->num, gain_buf);

	ch1_rst_buf = gain_buf[0];
	ch1_int_buf = gain_buf[1];
	ref_rst_buf = gain_buf[2];
	ref_int_buf = gain_buf[3];

	return sprintf(buf, "%d,%d,%d,%d\n", (int)ch1_rst_buf, (int)ch1_int_buf, (int)ref_rst_buf, (int)ref_int_buf);
}

static ssize_t grip_1ch_check_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct grip_data *data = dev_get_drvdata(dev);

	grip_diff_getdata(data, true, 0);

	return snprintf(buf, PAGE_SIZE, "%d\n", data->ch[0]->grip_event);
}

static ssize_t grip_irq_count_1ch_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct grip_data *data = dev_get_drvdata(dev);
	int result = 0;
	s16 max_diff_val;

	result = get_max_diff_on_irq_count(data, data->ch[0]->num, &max_diff_val);

	return snprintf(buf, PAGE_SIZE, "%d,%d,%d\n", result,
			data->irq_count, (int)max_diff_val);
}

static ssize_t grip_irq_count_1ch_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct grip_data *data = dev_get_drvdata(dev);
	u8 onoff;
	int ret;

	if (data->check_abnormal_working == true) {
		GRIP_INFO("abnormal skip");
		return -EIO;
	}

	ret = kstrtou8(buf, 10, &onoff);
	if (ret < 0) {
		GRIP_ERR("strtou8 err %d\n", ret);
		return count;
	}

	initialize_irq_count_for_factory_test(data, data->ch[0]->num, onoff);

	return count;
}

static ssize_t grip_1ch_unknown_state_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct grip_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%s\n",
		(data->ch[0]->is_unknown_mode == 1) ? "UNKNOWN" : "NORMAL");
}

static ssize_t grip_1ch_unknown_state_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int val;
	int ret;
	struct grip_data *data = dev_get_drvdata(dev);

	ret = kstrtoint(buf, 10, &val);
	if (ret) {
		GRIP_ERR("Invalid val\n");
		return ret;
	}

	if (val == 1)
		enter_unknown_mode(data, TYPE_FORCE, data->ch[0]->num);
	else if (val == 0)
		data->ch[0]->is_unknown_mode = UNKNOWN_OFF;
	else
		GRIP_INFO("Invalid Argument %d\n", val);

	GRIP_INFO("%u\n", val);
	return count;
}
#endif /*CONFIG_SENSORS_A96T396_CH1*/

#ifdef CONFIG_SENSORS_A96T396_CH2

static ssize_t grip_gain_2ch_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct grip_data *data = dev_get_drvdata(dev);
	u8 ch2_rst_buf, ref_rst_buf;
	u8 ch2_int_buf, ref_int_buf;
	u8 gain_buf[4] = {0};

	get_grip_gain(data, data->ch[1]->num, gain_buf);

	ch2_rst_buf = gain_buf[0];
	ch2_int_buf = gain_buf[1];
	ref_rst_buf = gain_buf[2];
	ref_int_buf = gain_buf[3];

	return sprintf(buf, "%d,%d,%d,%d\n", (int)ch2_rst_buf, (int)ch2_int_buf, (int)ref_rst_buf, (int)ref_int_buf);
}

static ssize_t grip_2ch_threshold_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct grip_data *data = dev_get_drvdata(dev);

	get_threshold(data, data->ch[1]->num);

	return sprintf(buf, "%u,%u,%u\n", data->ch[1]->grip_p_thd,
				data->ch[1]->grip_r_thd, data->ch[1]->grip_n_thd);
}

static ssize_t grip_2ch_total_cap_show(struct device *dev,
						struct device_attribute *attr, char *buf)
{
	struct grip_data *data = dev_get_drvdata(dev);
	int value;

	value = get_total_cap(data, data->ch[1]->num);

	return snprintf(buf, PAGE_SIZE, "%d\n", value / 100);
}

static ssize_t grip_2ch_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct grip_data *data = dev_get_drvdata(dev);

	get_grip_diff(data, data->ch[1]->num);

	return sprintf(buf, "%u,%u\n", data->ch[1]->diff, data->ch[1]->diff_d);
}

static ssize_t grip_2ch_baseline_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct grip_data *data = dev_get_drvdata(dev);

	get_baseline(data, data->ch[1]->num);

	return snprintf(buf, PAGE_SIZE, "%u\n", data->ch[1]->grip_baseline);
}

static ssize_t grip_2ch_raw_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct grip_data *data = dev_get_drvdata(dev);
	int ret;

	ret = grip_get_raw_data(data, data->ch[1]->num);
	if (ret < 0)
		return sprintf(buf, "%d\n", 0);
	else
		return sprintf(buf, "%u,%u\n", data->ch[1]->grip_raw, data->ch[1]->grip_raw_d);
}

static ssize_t grip_2ch_check_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct grip_data *data = dev_get_drvdata(dev);

	grip_diff_getdata(data, true, 1);

	return snprintf(buf, PAGE_SIZE, "%d\n", data->ch[1]->grip_event);
}

static ssize_t grip_2ch_unknown_state_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct grip_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%s\n",
		(data->ch[1]->is_unknown_mode == 1) ? "UNKNOWN" : "NORMAL");
}

static ssize_t grip_irq_count_2ch_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct grip_data *data = dev_get_drvdata(dev);
	int result = 0;
	s16 max_diff_val_2ch;

	result = get_max_diff_on_irq_count(data, data->ch[1]->num, &max_diff_val_2ch);

	return snprintf(buf, PAGE_SIZE, "%d,%d,%d\n", result,
			data->irq_count, max_diff_val_2ch);
}

static ssize_t grip_irq_count_2ch_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct grip_data *data = dev_get_drvdata(dev);
	u8 onoff;
	int ret;

	ret = kstrtou8(buf, 10, &onoff);
	if (ret < 0) {
		GRIP_ERR("strtou8 err %d\n", ret);
		return count;
	}

	initialize_irq_count_for_factory_test(data, data->ch[1]->num, onoff);

	return count;
}

static ssize_t grip_2ch_unknown_state_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int val;
	int ret;
	struct grip_data *data = dev_get_drvdata(dev);

	ret = kstrtoint(buf, 10, &val);
	if (ret) {
		GRIP_ERR("Invalid val\n");
		return ret;
	}

	if (val == 1)
		enter_unknown_mode(data, TYPE_FORCE, data->ch[1]->num);
	else if (val == 0)
		data->ch[1]->is_unknown_mode = UNKNOWN_OFF;
	else
		GRIP_INFO("Invalid Argument %d\n", val);

	GRIP_INFO("%u\n", val);
	return count;
}
#endif /*CONFIG_SENSORS_A96T396_CH2*/

#ifndef CONFIG_SAMSUNG_PRODUCT_SHIP

static int string_to_hex(char *pbSrc, unsigned char *pbDest)
{
	char *p = pbSrc;
	char msb = 0, lsb = 0;
	int tmplen = 0, cnt = 0;

	tmplen = strlen(p);
	while (cnt < (tmplen / 2)) {
		msb = ((*p > '9') && ((*p <= 'F') || (*p <= 'f'))) ? *p - 48 - 7 : *p - 48;
		lsb = (*(++ p) > '9' && ((*p <= 'F') || (*p <= 'f'))) ? *(p) - 48 - 7 : *(p) - 48;
		pbDest[cnt] = ((msb & 0x0f) << 4 | (lsb & 0x0f));
		p++;
		cnt++;
	}
	if (tmplen % 2 != 0)
		pbDest[cnt] = ((*p > '9') && ((*p <= 'F') || (*p <= 'f'))) ? *p - 48 - 7 : *p - 48;

	return tmplen / 2 + tmplen % 2;
}

static ssize_t grip_reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 r_buf[6] = {0,};
	u8 i = 0;
	char *p = buf;
	struct grip_data *data = dev_get_drvdata(dev);

	if (data->read_flag) {
		if (data->read_reg_count > 6)
			data->read_reg_count = 6;
		grip_i2c_read(data->client, data->read_reg, r_buf, data->read_reg_count);
		for (i = 0; i < data->read_reg_count; i++)
			p += snprintf(p, PAGE_SIZE, "(0x%02x)=0x%02x\n", data->read_reg + i, r_buf[i]);

		return (p-buf);
	}

	if (data->param.is_tuning_mode) {
		if (data->param.read_tuning_register_flag == 1) {
			u8 cmd;

			cmd = CHANGE_TUNING_MAP_CMD;
			grip_i2c_write(data->client, REG_GRIP_TUNING_STATE, &cmd);
			msleep(20);
			for (i = 0; i < 0x80; i++) {
				grip_i2c_read(data->client, i, r_buf, 1);
				p += snprintf(p, PAGE_SIZE, "(0x%02x)=0x%02x\n", i, r_buf[0]);
			}
			cmd = CHANGE_REGISTER_MAP_CMD;
			grip_i2c_write(data->client, REG_GRIP_TUNING_STATE, &cmd);

			data->param.read_tuning_register_flag = 0;
		} else {
			for (i = 0; i < 0x80; i++) {
				grip_i2c_read(data->client, i, r_buf, 1);
				p += snprintf(p, PAGE_SIZE, "(0x%02x)=0x%02x\n", i, r_buf[0]);
			}
		}
	} else {
		for (i = 0; i < 0x91; i++) {
			grip_i2c_read(data->client, i, r_buf, 1);
			p += snprintf(p, PAGE_SIZE, "(0x%02x)=0x%02x\n", i, r_buf[0]);
		}
	}

	return (p-buf);
}

static ssize_t grip_reg_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct grip_data *data = dev_get_drvdata(dev);
	unsigned int val, reg, opt;
	u8 reg_temp_buf[512];
	u8 reg_buf[256];
	int reg_buf_len;
	u8 i;

	if (sscanf(buf, "%x,%x,%x", &reg, &val, &opt) == 3) {
		GRIP_INFO("read reg 0x%02x\n", reg);
		data->read_reg = *((u8 *)&reg);
		data->read_reg_count = *((u8 *)&val);
		if (!opt)
			data->read_flag = 1;
		else
			data->read_flag = 0;
	} else if (sscanf(buf, "%x,%x", &reg, &val) == 2) {
		GRIP_INFO("reg 0x%02x, val 0x%02x\n", reg, val);
		grip_i2c_write(data->client, *((u8 *)&reg), (u8 *)&val);
	} else if (kstrtoint(buf, 10, &data->param.read_tuning_register_flag)) {
		GRIP_INFO("check tuning register");
	} else {
		sscanf(buf, "%511s", reg_temp_buf);
		reg_buf_len = string_to_hex(reg_temp_buf, reg_buf);
		for (i = 0; i < reg_buf_len-1; i++) {
			grip_i2c_write(data->client, reg_buf[0]+i, &reg_buf[i+1]);
			GRIP_INFO("reg 0x%02x, val 0x%02x\n", reg_buf[0]+i, reg_buf[i+1]);
		}
	}
	return size;
}

#ifdef CONFIG_SENSORS_A96T396_CH1

static ssize_t grip_1ch_sar_press_threshold_store(struct device *dev,
	struct device_attribute *attr, const char *buf,
	size_t count)
{
	struct grip_data *data = dev_get_drvdata(dev);
	int ret;
	int threshold;

	ret = sscanf(buf, "%11d", &threshold);
	if (ret != 1) {
		GRIP_ERR("fail to read buf is %s\n", buf);
		return count;
	}

	write_sar_press_threshold(data, threshold, data->ch[0]->num);

	return count;
}

static ssize_t grip_1ch_sar_release_threshold_store(struct device *dev,
	struct device_attribute *attr, const char *buf,
	size_t count)
{
	struct grip_data *data = dev_get_drvdata(dev);
	int ret;
	int threshold;

	ret = sscanf(buf, "%11d", &threshold);
	if (ret != 1) {
		GRIP_ERR("fail to read buf is %s\n", buf);
		return count;
	}

	write_sar_release_threshold(data, threshold, data->ch[0]->num);

	return count;
}
#endif

#ifdef CONFIG_SENSORS_A96T396_CH2

static ssize_t grip_2ch_sar_release_threshold_store(struct device *dev,
	struct device_attribute *attr, const char *buf,
	size_t count)
{
	struct grip_data *data = dev_get_drvdata(dev);
	int ret;
	int threshold;

	ret = sscanf(buf, "%11d", &threshold);
	if (ret != 1) {
		GRIP_ERR("fail to read buf is %s\n", buf);
		return count;
	}

	write_sar_release_threshold(data, threshold, data->ch[1]->num);

	return count;
}

static ssize_t grip_2ch_sar_press_threshold_store(struct device *dev,
	struct device_attribute *attr, const char *buf,
	size_t count)
{
	struct grip_data *data = dev_get_drvdata(dev);
	int ret;
	int threshold;

	ret = sscanf(buf, "%11d", &threshold);
	if (ret != 1) {
		GRIP_ERR("fail to read, buf is %s\n", buf);
		return count;
	}

	write_sar_press_threshold(data, threshold, data->ch[1]->num);

	return count;
}
#endif
#endif /*CONFIG_SAMSUNG_PRODUCT_SHIP*/

static DEVICE_ATTR(grip_sar_enable, 0664, grip_sar_enable_show, grip_sar_enable_store);
static DEVICE_ATTR(grip_sw_reset_ready, 0444, grip_sw_reset_ready_show, NULL);
static DEVICE_ATTR(grip_sw_reset, 0220, NULL, grip_sw_reset_show);
static DEVICE_ATTR(grip_ref_cap, 0444, grip_ref_cap_show, NULL);
#ifndef CONFIG_SAMSUNG_PRODUCT_SHIP
static DEVICE_ATTR(grip_reg_rw, 0664, grip_reg_show, grip_reg_store);
#endif
static DEVICE_ATTR(name, 0444, grip_name_show, NULL);
static DEVICE_ATTR(vendor, 0444, grip_vendor_show, NULL);
static DEVICE_ATTR(grip_firm_version_phone, 0444, bin_fw_ver_show, NULL);
static DEVICE_ATTR(grip_firm_version_panel, 0444, read_fw_ver_show, NULL);
static DEVICE_ATTR(grip_firm_update, 0220, NULL, grip_fw_update_store);
static DEVICE_ATTR(grip_firm_update_status, 0444, grip_fw_update_status_show, NULL);
static DEVICE_ATTR(grip_irq_state, 0444, grip_irq_state_show, NULL);
static DEVICE_ATTR(grip_irq_en_cnt, 0444, grip_irq_en_cnt_show, NULL);
static DEVICE_ATTR(grip_crc_check, 0444, grip_crc_check_show, NULL);
static DEVICE_ATTR(motion, 0664, grip_motion_show, grip_motion_store);

static DEVICE_ATTR(noti_enable, 0664, grip_noti_enable_show, grip_noti_enable_store);
#ifdef CONFIG_SENSORS_A96T396_LDO_SHARE
static DEVICE_ATTR(grip_register_recover, 0220, NULL, grip_register_recover_store);
#endif
static DEVICE_ATTR(ch_count, 0444, grip_ch_count_show, NULL);
static DEVICE_ATTR(grip_tuning_version, 0444, grip_tuning_version_show, NULL);
static DEVICE_ATTR(enable, 0664, grip_enable_show, grip_sar_enable_store);

struct device_attribute *grip_common_sensor_attrs[] = {
	&dev_attr_grip_sar_enable,
	&dev_attr_grip_sw_reset,
	&dev_attr_grip_sw_reset_ready,
	&dev_attr_grip_ref_cap,
#ifndef CONFIG_SAMSUNG_PRODUCT_SHIP
	&dev_attr_grip_reg_rw,
#endif
	&dev_attr_name,
	&dev_attr_vendor,
	&dev_attr_grip_firm_version_phone,
	&dev_attr_grip_firm_version_panel,
	&dev_attr_grip_firm_update,
	&dev_attr_grip_firm_update_status,
	&dev_attr_grip_irq_state,
	&dev_attr_grip_irq_en_cnt,
	&dev_attr_grip_crc_check,
#ifdef CONFIG_SENSORS_A96T396_LDO_SHARE
	&dev_attr_grip_register_recover,
#endif
	&dev_attr_motion,
	&dev_attr_noti_enable,
	&dev_attr_grip_tuning_version,
	&dev_attr_ch_count,
	NULL,
};

static struct attribute *grip_attributes[] = {
	&dev_attr_enable.attr,
	NULL
};

struct attribute_group grip_attribute_group = {
	.attrs = grip_attributes
};

#ifdef CONFIG_SENSORS_A96T396_CH1

static DEVICE_ATTR(unknown_state, 0664, grip_1ch_unknown_state_show, grip_1ch_unknown_state_store);
static DEVICE_ATTR(grip_gain, 0444, grip_gain_1ch_show, NULL);
#ifndef CONFIG_SAMSUNG_PRODUCT_SHIP
static DEVICE_ATTR(grip_sar_press_threshold, 0220,
		NULL, grip_1ch_sar_press_threshold_store);
static DEVICE_ATTR(grip_sar_release_threshold, 0220,
		NULL, grip_1ch_sar_release_threshold_store);
#endif
static DEVICE_ATTR(grip_irq_count, 0664, grip_irq_count_1ch_show,
			grip_irq_count_1ch_store);
static DEVICE_ATTR(grip_threshold, 0444, grip_1ch_threshold_show, NULL);
static DEVICE_ATTR(grip_total_cap, 0444, grip_1ch_total_cap_show, NULL);
static DEVICE_ATTR(grip, 0444, grip_1ch_show, NULL);
static DEVICE_ATTR(grip_baseline, 0444, grip_1ch_baseline_show, NULL);
static DEVICE_ATTR(grip_raw, 0444, grip_1ch_raw_show, NULL);
static DEVICE_ATTR(grip_check, 0444, grip_1ch_check_show, NULL);

struct device_attribute *grip_ch1_sensor_attrs[] = {
	&dev_attr_grip_gain,
#ifndef CONFIG_SAMSUNG_PRODUCT_SHIP
	&dev_attr_grip_sar_press_threshold,
	&dev_attr_grip_sar_release_threshold,
#endif
	&dev_attr_grip_irq_count,
	&dev_attr_grip_threshold,
	&dev_attr_grip_total_cap,
	&dev_attr_grip,
	&dev_attr_grip_baseline,
	&dev_attr_grip_raw,
	&dev_attr_grip_check,
	&dev_attr_unknown_state,
	NULL,
};
#endif

#ifdef CONFIG_SENSORS_A96T396_CH2

static DEVICE_ATTR(unknown_state_b, 0664, grip_2ch_unknown_state_show, grip_2ch_unknown_state_store);
static DEVICE_ATTR(grip_gain_b, 0444, grip_gain_2ch_show, NULL);
#ifndef CONFIG_SAMSUNG_PRODUCT_SHIP
static DEVICE_ATTR(grip_sar_press_threshold_b, 0220,
		NULL, grip_2ch_sar_press_threshold_store);
static DEVICE_ATTR(grip_sar_release_threshold_b, 0220,
		NULL, grip_2ch_sar_release_threshold_store);
#endif
static DEVICE_ATTR(grip_irq_count_b, 0664, grip_irq_count_2ch_show,
			grip_irq_count_2ch_store);
static DEVICE_ATTR(grip_threshold_b, 0444, grip_2ch_threshold_show, NULL);
static DEVICE_ATTR(grip_total_cap_b, 0444, grip_2ch_total_cap_show, NULL);
static DEVICE_ATTR(grip_b, 0444, grip_2ch_show, NULL);
static DEVICE_ATTR(grip_baseline_b, 0444, grip_2ch_baseline_show, NULL);
static DEVICE_ATTR(grip_raw_b, 0444, grip_2ch_raw_show, NULL);
static DEVICE_ATTR(grip_check_b, 0444, grip_2ch_check_show, NULL);

struct device_attribute *grip_ch2_sensor_attrs[] = {
	&dev_attr_grip_gain_b,
#ifndef CONFIG_SAMSUNG_PRODUCT_SHIP
	&dev_attr_grip_sar_press_threshold_b,
	&dev_attr_grip_sar_release_threshold_b,
#endif
	&dev_attr_grip_irq_count_b,
	&dev_attr_grip_threshold_b,
	&dev_attr_grip_total_cap_b,
	&dev_attr_grip_b,
	&dev_attr_grip_baseline_b,
	&dev_attr_grip_raw_b,
	&dev_attr_grip_check_b,
	&dev_attr_unknown_state_b,
	NULL,
};
#endif

int get_channel_attr_size(int ch_idx)
{
	int channel_attr_size[] = {
#ifdef CONFIG_SENSORS_A96T396_CH1
		sizeof(grip_ch1_sensor_attrs),
#endif
#ifdef CONFIG_SENSORS_A96T396_CH2
		sizeof(grip_ch2_sensor_attrs),
#endif
	};
	return channel_attr_size[ch_idx];
}

int sizeof_grip_sensor_attr(void)
{
	return sizeof(grip_common_sensor_attrs);
}

MODULE_AUTHOR("Samsung Electronics");
MODULE_DESCRIPTION("Grip sensor driver for grip chip");
MODULE_LICENSE("GPL");
