/* a96t396.c -- Linux driver for a96t396 chip as grip sensor
 *
 * Copyright (C) 2017 Samsung Electronics Co.Ltd
 * Author: YunJae Hwang <yjz.hwang@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2, or (at your option) any
 * later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 */



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
#if defined(CONFIG_SENSORS_CORE_AP)
#include <linux/sensor/sensors_core.h>
#endif
#if IS_ENABLED(CONFIG_CCIC_NOTIFIER) || IS_ENABLED(CONFIG_PDIC_NOTIFIER)
#include <linux/usb/typec/common/pdic_notifier.h>
#endif
#include <linux/usb/typec/common/pdic_param.h>
#if IS_ENABLED(CONFIG_USB_TYPEC_MANAGER_NOTIFIER)
#include <linux/usb/typec/manager/usb_typec_manager_notifier.h>
#endif

#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#endif

#if IS_ENABLED(CONFIG_HALL_NOTIFIER)
#include <linux/hall/hall_ic_notifier.h>
#define HALL_NAME		"hall"
#define HALL_CERT_NAME		"certify_hall"
#define HALL_FLIP_NAME		"flip"
#define HALL_ATTACH		1
#define HALL_DETACH		0
#endif

#if IS_ENABLED(CONFIG_TABLET_MODEL_CONCEPT)
#if IS_ENABLED(CONFIG_KEYBOARD_STM32_POGO_V3)
#include "../input/sec_input/stm32/pogo_notifier_v3.h"
#elif IS_ENABLED(CONFIG_KEYBOARD_STM32_POGO_V2) || IS_ENABLED(CONFIG_KEYBOARD_STM32_POGO)
#include <linux/input/pogo_i2c_notifier.h>
#endif
#endif

#include "a96t396.h"
#include "grip_common.h"
#include "grip_sysfs.h"

static struct delayed_work* gp_fw_work[GRIP_MAX_CNT];
static int probe_count;
static int max_probe_count;

static void a96t396_check_first_working(struct grip_data *data, u8 ch_num);
static int a96t396_fw_check(struct grip_data *data);
static void a96t396_tuning_check(struct delayed_work *work, int ic_num);
static int a96t396_tuning_mode(struct grip_data *data);

static void pdic_attach_reset_work_func(struct work_struct *work)
{
	struct grip_data *data = container_of((struct work_struct *)work,
						struct grip_data, pdic_attach_reset_work);
	u8 cmd = CMD_OFF;

	mutex_lock(&data->tuning_lock);
	grip_i2c_write(data->client, REG_TSPTA, &cmd);
	GRIP_INFO("pdic_attach");
	mutex_unlock(&data->tuning_lock);
}

static void pdic_detach_reset_work_func(struct work_struct *work)
{
	struct grip_data *data = container_of((struct work_struct *)work,
						struct grip_data, pdic_detach_reset_work);
	u8 cmd = CMD_ON;

	mutex_lock(&data->tuning_lock);
	grip_i2c_write(data->client, REG_TSPTA, &cmd);
	GRIP_INFO("pdic_detach");
	mutex_unlock(&data->tuning_lock);
}

static void reset_work_func(struct work_struct *work)
{
	struct grip_data *data = container_of((struct work_struct *)work,
						struct grip_data, reset_work);

	mutex_lock(&data->tuning_lock);
	grip_sw_reset(data);
	mutex_unlock(&data->tuning_lock);
}

static void a96t396_firmware_work_func(struct work_struct *work)
{
	struct grip_data *data = container_of((struct delayed_work *)work,
		struct grip_data, firmware_work);
	int ret;
	int next_idx = data->ic_num;
	int i = 0;

	GRIP_INFO("start - probe_count %d\n", probe_count);

	if (probe_count <= max_probe_count) {
		if (probe_count == max_probe_count) {
			GRIP_INFO("All chip probe sequences are complete!\n");
			probe_count = max_probe_count + 1;
		}
		schedule_delayed_work(&data->firmware_work,
				msecs_to_jiffies(100));
		return;
	}

	ret = a96t396_fw_check(data);
	if (ret < 0) {
		if (data->firmware.firmware_count++ < FIRMWARE_VENDOR_CALL_CNT) {
			GRIP_ERR("fail to load fw %d\n",
				data->firmware.firmware_count);
			schedule_delayed_work(&data->firmware_work,
					msecs_to_jiffies(1000));
			return;
		}
		GRIP_ERR("final retry fail\n");
	} else {
		GRIP_INFO("fw check success\n");
	}

	while (next_idx < GRIP_MAX_CNT - 1) {
		next_idx = next_idx + 1;
		if (gp_fw_work[next_idx] == NULL) {
			GRIP_INFO("skip GRIP[%d] fw download\n", next_idx);
		} else {
			GRIP_INFO("schedule GRIP[%d] fw download\n", next_idx);
			schedule_delayed_work(gp_fw_work[next_idx],
				msecs_to_jiffies(500));
			return;
		}
	}

	for (i = 0; i < GRIP_MAX_CNT; i++) {
		if (gp_fw_work[i] == NULL) {
			GRIP_INFO("skip tuning check GRIP[%d]\n", i);
		} else {
			GRIP_INFO("tuning check GRIP[%d]\n", i);
			a96t396_tuning_check(gp_fw_work[i], i);
		}
	}
}

static void a96t396_debug_work_func(struct work_struct *work)
{
	struct grip_data *data = container_of((struct delayed_work *)work,
		struct grip_data, debug_work);
	u8 i;

	if (data->check_abnormal_working)
		return;

	if (data->resume_called == true) { //??
		data->resume_called = false;
		schedule_delayed_work(&data->debug_work, msecs_to_jiffies(1000));
		return;
	}

	if (data->current_state) {
		check_irq_status(data, BY_DEBUG_WORK);

		if (data->current_state == ON && data->abnormal_mode) {
			for (i = 0; i < NUM_OF_MAX_CHANNEL; i++) {
				if (is_unavailable_channel(data->ch[i]))
					continue;
				grip_diff_getdata(data, true, i);
				if (data->ch[i]->max_normal_diff < data->ch[i]->diff)
					data->ch[i]->max_normal_diff = data->ch[i]->diff;
			}
		} else {
			if (data->debug_count >= GRIP_LOG_TIME) {
				for (i = 0; i < NUM_OF_MAX_CHANNEL; i++) {
					if (is_unavailable_channel(data->ch[i]))
						continue;

					grip_diff_getdata(data, true, i);
					if (data->ch[i]->is_unknown_mode == UNKNOWN_ON && data->motion)
						a96t396_check_first_working(data, i);
				}

				data->debug_count = 0;
			} else {
				for (i = 0; i < NUM_OF_MAX_CHANNEL; i++) {
					if (is_unavailable_channel(data->ch[i]))
						continue;

					if (data->ch[i]->is_unknown_mode == UNKNOWN_ON && data->motion) {
						grip_diff_getdata(data, false, i);
						a96t396_check_first_working(data, i);
					}
				}
				data->debug_count++;
			}
		}

		schedule_delayed_work(&data->debug_work, msecs_to_jiffies(2000));
	}
}

static void a96t396_set_debug_work(struct grip_data *data, u8 enable,
	unsigned int time_ms)
{
	GRIP_INFO("enable %d\n", enable);

	if (enable == 1) {
		data->debug_count = 0;
		schedule_delayed_work(&data->debug_work,
			msecs_to_jiffies(time_ms));
	} else {
		cancel_delayed_work_sync(&data->debug_work);
	}
}

static void a96t396_set_firmware_work(struct grip_data *data, u8 enable,
	unsigned int time_ms)
{
	GRIP_INFO("%s\n", enable ? "enabled" : "disabled");

	if (enable == 1) {
		data->firmware.firmware_count = 0;
		schedule_delayed_work(&data->firmware_work,
			msecs_to_jiffies(time_ms * 1000));
	} else {
		cancel_delayed_work_sync(&data->firmware_work);
	}
}

static irqreturn_t a96t396_interrupt(int irq, void *ptr)
{
	struct grip_data *data = ptr;

	GRIP_INFO("called\n");

	__pm_wakeup_event(data->grip_ws, jiffies_to_msecs(3 * HZ));
	schedule_work(&data->irq_work);

	return IRQ_HANDLED;
}

static int a96t396_checksum_for_usermode(struct grip_data *data)
{
	int ret = 0;
	int length = 3;
	int retry = 2;
	unsigned char cmd[3] = {0x0A, 0x00, 0x10};
	unsigned char checksum[2] = {0, };

	while (retry--) {
		ret = i2c_master_send(data->client, cmd, length);
		if (ret != length) {
			GRIP_ERR("i2c_write fail %d\n", ret);
			ret = -1;
			continue;
		}

		usleep_range(160 * 1000, 161 * 1000);

		ret = grip_i2c_read(data->client, 0x0A, checksum, 2);
		if (ret < 0) {
			GRIP_ERR("i2c read fail : %d\n", ret);
			continue;
		}

		GRIP_INFO("CRC:%02x%02x, BIN:%02x%02x\n", checksum[0], checksum[1],
			data->firmware.checksum_h_bin, data->firmware.checksum_l_bin);

		if ((checksum[0] == data->firmware.checksum_h_bin) && (checksum[1] == data->firmware.checksum_l_bin)) {
			ret = 0;
			break;
		} else {
			GRIP_INFO("Checksum fail. retry:%d\n", retry);
			ret = -1;
		}
	}

	return ret;
}

static int a96t396_check_tuning_checksum(struct grip_data *data)
{
	int ret = 0;
	u8 cmd;
	unsigned char checksum[2] = {0, };

	cmd = 0x00;
	ret = grip_i2c_write(data->client, REG_TUNING_CHECKSUM_MSB, &cmd);
	if (ret < 0) {
		GRIP_INFO("i2c write fail(%d)\n", ret);
		return ret;
	}

	cmd = 0x10;
	ret = grip_i2c_write(data->client, REG_TUNING_CHECKSUM_LSB, &cmd);
	if (ret < 0) {
		GRIP_INFO("i2c write fail(%d)\n", ret);
		return ret;
	}

	usleep_range(10000, 10010);

	ret = grip_i2c_read(data->client, REG_TUNING_CHECKSUM_MSB, checksum, 2); // 0x10, 0x00
	if (ret < 0) {
		GRIP_ERR("i2c read fail(%d)\n", ret);
		return ret;
	}

	if (checksum[0] != data->param.checksum_msb || checksum[1] != data->param.checksum_lsb) {
		GRIP_INFO("0x%x, 0x%x, 0x%x, 0x%x\n", checksum[0], checksum[1],
			data->param.checksum_msb, data->param.checksum_lsb);
		ret = -1;
	}

	return ret;
}

static void a96t396_tuning_check(struct delayed_work *work, int ic_num)
{
	struct grip_data *data = container_of((struct delayed_work *)work,
		struct grip_data, firmware_work);

	int ret;

	mutex_lock(&data->tuning_lock);
	ret = a96t396_tuning_mode(data);
	mutex_unlock(&data->tuning_lock);

	if (ret < 0) {
		GRIP_INFO("fail to check tuning mode");
	} else {
		GRIP_INFO("success to check tuning mode");
	}
}

static int a96t396_tuning_mode(struct grip_data *data)
{
	int ret, i;
	u8 cmd;
	u8 r_buf[2] = {0};
	int index = 0;
	bool is_register_setting_fail = 0;

	grip_always_active(data, 1);

	ret = grip_get_fw_version(data, true, false);
	if (ret)
		GRIP_ERR("i2c fail(%d), addr[%d]\n", ret, data->client->addr);

	grip_always_active(data, 0);

	// check logic FW
	if ((data->firmware.fw_ver != data->firmware.fw_ver_bin) || (data->firmware.md_ver != data->firmware.md_ver_bin))
		return -1;
	if (data->firmware.md_ver != 0XAC)
		return -1;

	GRIP_INFO("tuning mode start!\n");

	// change tuning mode
	cmd = CHANGE_TUNING_MAP_CMD;
	ret = grip_i2c_write(data->client, REG_GRIP_TUNING_STATE, &cmd);
	if (ret < 0) {
		GRIP_INFO("i2c write fail(%d)\n", ret);
		is_register_setting_fail = 1;
		goto register_mode;
	}

	msleep(20);

	ret = grip_i2c_read(data->client, REG_GRIP_TUNING_STATE, r_buf, 1);
	if (ret < 0) {
		GRIP_ERR("i2c read fail(%d)\n", ret);
		is_register_setting_fail = 1;
		goto register_mode;
	}

	// change tuning mode success
	if (r_buf[0] == CHANGE_TUNING_MAP_FINISHED) {
		// set tunging parameter
		if (data->param.setup_reg_exist) {
			for (i = 0; i < TUNINGMAP_MAX - 2; i++) {
				index = i << 1;
				ret = grip_i2c_write(data->client,
					data->param.setup_reg[index],
					&data->param.setup_reg[index + 1]);
				if (ret < 0) {
					GRIP_INFO("i2c write fail(%d)\n", ret);
					is_register_setting_fail = 1;
					goto register_mode;
				}
				// verify
				ret = grip_i2c_read(data->client,
					data->param.setup_reg[index], r_buf, 1);
				if (r_buf[0] != data->param.setup_reg[index + 1]) {
					GRIP_INFO("%x, %x, %x\n", data->param.setup_reg[index],
						r_buf[0], data->param.setup_reg[index + 1]);
					is_register_setting_fail = 1;
					goto register_mode;
				}
			}
		}
	}

	// check checksum
	ret = a96t396_check_tuning_checksum(data);
	if (ret < 0) {
		GRIP_INFO("tuning checksum fail(%d)\n", ret);
		is_register_setting_fail = 1;
		goto register_mode;
	}

register_mode:
	// check register mode
	cmd = CHANGE_REGISTER_MAP_CMD;
	ret = grip_i2c_write_retry(data->client, REG_GRIP_TUNING_STATE, &cmd, 3);
	if (ret < 0) {
		GRIP_INFO("i2c write fail(%d)\n", ret);
		return ret;
	}

	msleep(20);

	ret = grip_i2c_read_retry(data->client, REG_GRIP_TUNING_STATE, r_buf, 1, 3);
	if (ret < 0) {
		GRIP_ERR("i2c read fail(%d)\n", ret);
		return ret;
	}

	if (is_register_setting_fail) {
		enter_error_mode(data, FAIL_SETUP_REGISTER);
		return -1;
	}

	if (r_buf[0] == CHANGE_REGISTER_MAP_FINISHED) {
		data->param.is_tuning_mode = 1;
		GRIP_INFO("success to finish register mode!\n");
		return SUCCESS;
	} else {
		GRIP_INFO("fail to finish register mode!\n");
		return -1;
	}
}

static int a96t396_fw_check(struct grip_data *data)
{
	int ret, i;
	bool force = false;
	u8 r_buf[4] = {0,};

	ret = grip_load_fw_kernel(data);

	if (ret < 0) {
#ifdef CONFIG_SENSORS_FW_VENDOR
		GRIP_ERR("fw was not loaded yet from ueventd\n");
		return ret;
#else
		GRIP_ERR("failed load_fw_kernel(%d)\n", ret);
#endif
	} else
		GRIP_INFO("fw version read success (%d)\n", ret);

	grip_always_active(data, 1);

	ret = grip_get_fw_version(data, true, false);
	if (ret)
		GRIP_ERR("i2c fail(%d), addr[%d]\n", ret, data->client->addr);

	if (data->firmware.md_ver != data->firmware.md_ver_bin) {
		GRIP_ERR("MD version is different.(IC %x, BN %x). Do force FW update\n",
			data->firmware.md_ver, data->firmware.md_ver_bin);
		force = true;
	} else if (data->firmware.fw_ver == data->firmware.fw_ver_bin) {
		ret = a96t396_checksum_for_usermode(data);
		if (ret < 0) {
			GRIP_ERR("checksum fail\n");
			force = true;
		}
	}

	grip_always_active(data, 0);

	if (data->firmware.fw_ver < data->firmware.fw_ver_bin || data->firmware.fw_ver > TEST_FIRMWARE_DETECT_VER
				|| force == true || data->crc_check == CRC_FAIL) {
		GRIP_ERR("excute fw update (0x%x -> 0x%x)\n",
			data->firmware.fw_ver, data->firmware.fw_ver_bin);
		ret = grip_flash_fw(data, true, BUILT_IN);

		if (ret) {
			GRIP_ERR("failed to grip_flash_fw (%d)\n", ret);
			enter_error_mode(data, FAIL_SETUP_REGISTER);
		} else
			GRIP_INFO("fw update success\n");
	}

	for (i = 0; i < NUM_OF_MAX_CHANNEL; i++) {
		if (is_unavailable_channel(data->ch[i]))
			continue;
		get_threshold(data, i);
	}

	ret = grip_i2c_read(data->client, REG_SAR_THRESHOLD, r_buf, 4);
	if (ret < 0)
		GRIP_ERR("fail(%d)\n", ret);

	if (data->check_abnormal_working == true) {
		GRIP_INFO("stop to grip_flash_fw\n");
		ret = 0;
	}

	return ret;
}

static int a96t396_power_onoff(void *pdata, bool on)
{
	struct grip_data *data = (struct grip_data *)pdata;

	int ret = 0;
	int voltage = 0;
	int reg_enabled = 0;

	if (data->ldo_en) {
		ret = gpio_request(data->ldo_en, "a96t396_ldo_en");
		if (ret < 0) {
			GRIP_ERR("gpio %d request failed %d\n", data->ldo_en, ret);
			return ret;
		}
		gpio_set_value(data->ldo_en, on);
		GRIP_INFO("ldo_en power %d\n", gpio_get_value(data->ldo_en));
		gpio_free(data->ldo_en);
	}

	if (data->dvdd_vreg_name) {
		if (data->dvdd_vreg == NULL) {
			data->dvdd_vreg = regulator_get(NULL, data->dvdd_vreg_name);
			if (IS_ERR(data->dvdd_vreg)) {
				data->dvdd_vreg = NULL;
				GRIP_ERR("failed to get dvdd_vreg %s\n", data->dvdd_vreg_name);
			}
		}
	}

	if (data->dvdd_vreg) {
		voltage = regulator_get_voltage(data->dvdd_vreg);
		reg_enabled = regulator_is_enabled(data->dvdd_vreg);
		GRIP_INFO("reg_enabled=%d, voltage=%d\n", reg_enabled, voltage);
	}

	if (on) {
		if (data->dvdd_vreg) {
			if (reg_enabled == 0) {
				ret = regulator_enable(data->dvdd_vreg);
				if (ret) {
					GRIP_ERR("dvdd reg enable fail\n");
					return ret;
				}
				GRIP_INFO("dvdd_vreg turned on\n");
			}
		}
	} else {
		if (data->dvdd_vreg) {
			if (reg_enabled == 1) {
				ret = regulator_disable(data->dvdd_vreg);
				if (ret) {
					GRIP_ERR("dvdd reg disable fail\n");
					return ret;
				}
				GRIP_INFO("dvdd_vreg turned off\n");
			}
		}
	}

	GRIP_INFO("%s\n", on ? "on" : "off");
	if (data->dvdd_vreg) {
		voltage = regulator_get_voltage(data->dvdd_vreg);
		reg_enabled = regulator_is_enabled(data->dvdd_vreg);
		GRIP_INFO("regulator status check : reg_enabled=%d, voltage=%d\n", reg_enabled, voltage);
	}

	return ret;
}

static int a96t396_irq_init(struct device *dev,
	struct grip_data *data)
{
	int ret = 0;

	ret = gpio_request(data->grip_int, "a96t396_IRQ");
	if (ret < 0) {
		GRIP_ERR("gpio %d request failed (%d)\n", data->grip_int, ret);
		return ret;
	}

	ret = gpio_direction_input(data->grip_int);
	if (ret < 0) {
		GRIP_ERR("failed to set direction input gpio %d(%d)\n",
				data->grip_int, ret);
		gpio_free(data->grip_int);
		return ret;
	}
	// assigned power function to function ptr
	data->power = a96t396_power_onoff;

	return ret;
}

static int a96t396_parse_dt(struct grip_data *data, struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct pinctrl *p;
	const char *grip_ldo_name[] = {
		"grip_main_ldo",
		"grip_sub_ldo",
		"grip_sub2_ldo",
		"grip_wifi_ldo"
	};
	int ret = 0;
	int count;
	u32 temp;

	ret = of_property_read_u32(np, "max_probe_count", &count);
	if (ret < 0) {
		GRIP_INFO("skip to update\n");
	} else {
		max_probe_count = count;
		GRIP_INFO("max probe count %d\n", max_probe_count);
	}

	ret = of_property_read_u32(np, "set_up_channels", &temp);
	if (ret < 0) {
		GRIP_INFO("skip to update\n");
	} else {
		data->set_up_channels = temp;
		GRIP_INFO("set_up_channels %d\n", data->set_up_channels);
	}

	data->grip_int = of_get_named_gpio(np, "irq_gpio", 0);
	if (data->grip_int < 0) {
		GRIP_ERR("Can't get grip_int\n");
		return data->grip_int;
	}

	data->ldo_en = of_get_named_gpio(np, "ldo_en", 0);
	if (data->ldo_en < 0) {
		GRIP_INFO("set ldo_en 0\n");
		data->ldo_en = 0;
	} else {
		ret = gpio_request(data->ldo_en, grip_ldo_name[data->ic_num]);
		if (ret < 0) {
			GRIP_ERR("gpio %d request failed %d\n", data->ldo_en, ret);
			return ret;
		}
		gpio_direction_output(data->ldo_en, 1);
		gpio_free(data->ldo_en);
	}

	if (of_property_read_string_index(np, "dvdd_vreg_name", 0,
			(const char **)&data->dvdd_vreg_name)) {
		data->dvdd_vreg_name = NULL;
	}
	GRIP_INFO("dvdd_vreg_name: %s\n", data->dvdd_vreg_name);

	ret = of_property_read_string(np, "fw_path", (const char **)&data->firmware.fw_path);
	if (ret < 0) {
		GRIP_INFO("use TK_FW_PATH_BIN %d\n", ret);
		data->firmware.fw_path = TK_FW_PATH_BIN;
	}
	GRIP_INFO("fw path %s\n", data->firmware.fw_path);

	ret = of_property_read_u32(np, "firmup_cmd", &data->firmware.firmup_cmd);
	if (ret < 0)
		data->firmware.firmup_cmd = 0;

	ret = of_property_read_u32(np, "unknown_ch_selection", &data->unknown_ch_selection);
	if (ret < 0) {
		GRIP_ERR("set unknown ch 3\n");
		data->unknown_ch_selection = 3;
	}

	ret = of_property_read_u8_array(np, "fail_safe_concept", data->fail_safe_concept, 4);
	if (ret < 0)
		GRIP_ERR("set fail_safe_concept 0\n");

	p = pinctrl_get_select_default(dev);
	if (IS_ERR(p))
		GRIP_INFO("failed pinctrl_get\n");

	GRIP_INFO("grip_int:%d, ldo_en:%d\n", data->grip_int, data->ldo_en);

	ret = of_property_read_u32(np, "retry_i2c", &data->retry_i2c);
	if (ret < 0) {
		GRIP_ERR("set retry_i2c 1\n");
		data->retry_i2c = 1;
	}

	ret = of_property_read_u32(np, "checksum_msb", &data->param.checksum_msb);
	if (ret < 0) {
		GRIP_ERR("%d checksum_msb fail\n", data->ic_num);
		data->param.checksum_msb = 0x00;
	} else {
		GRIP_INFO("checksum_msb : 0x%x\n", data->param.checksum_msb);
	}

	ret = of_property_read_u32(np, "checksum_lsb", &data->param.checksum_lsb);
	if (ret < 0) {
		GRIP_ERR("%d checksum_lsb fail\n", data->ic_num);
		data->param.checksum_lsb = 0x00;
	} else {
		GRIP_INFO("checksum_lsb : 0x%x\n", data->param.checksum_lsb);
	}

	ret = of_property_read_u8_array(np, "set_reg", data->param.setup_reg,
						TUNINGMAP_MAX * 2);
	if (ret < 0) {
		GRIP_ERR("set_reg fail\n");
		data->param.setup_reg_exist = false;
	} else {
		GRIP_INFO("set_reg success\n");
		data->param.setup_reg_exist = true;
	}
	return SUCCESS;
}

#if (IS_ENABLED(CONFIG_CCIC_NOTIFIER) || IS_ENABLED(CONFIG_PDIC_NOTIFIER)) && IS_ENABLED(CONFIG_USB_TYPEC_MANAGER_NOTIFIER)
static int a96t396_pdic_handle_notification(struct notifier_block *nb,
					unsigned long action, void *pdic_data)
{
	PD_NOTI_ATTACH_TYPEDEF usb_typec_info = *(PD_NOTI_ATTACH_TYPEDEF *)pdic_data;
	struct grip_data *data = container_of(nb, struct grip_data, pdic_nb);
	int i;

#if IS_ENABLED(CONFIG_TABLET_MODEL_CONCEPT)
	GRIP_INFO("tablet model : reset skip!\n");
	return SUCCESS;
#endif

	if (data->firmware.fw_update_flag == true) {
		GRIP_INFO("fw updating, skip TA/USB reset");
		return SUCCESS;
	} else if (data->check_abnormal_working == true) {
		GRIP_INFO("abnormal working, skip TA/USB reset\n");
		return SUCCESS;
	}

	if (usb_typec_info.id != PDIC_NOTIFY_ID_ATTACH && usb_typec_info.id != PDIC_NOTIFY_ID_OTG) {
#if defined(CONFIG_SEC_FACTORY)
		if (usb_typec_info.id == PDIC_NOTIFY_ID_RID) {
			PD_NOTI_RID_TYPEDEF usb_fac_cable_info = *(PD_NOTI_RID_TYPEDEF *)pdic_data;

			switch (usb_fac_cable_info.rid) {
			case RID_301K:
			case RID_523K:
			case RID_619K:
				schedule_work(&data->pdic_detach_reset_work);
				GRIP_INFO("fac cable info rid %d", usb_fac_cable_info.rid);
				break;
			default:
				break;
			}
		}
#endif
		return SUCCESS;
	}
	if (data->pre_attach == usb_typec_info.attach)
		return SUCCESS;

	GRIP_INFO("src %d id %d attach %d\n", usb_typec_info.src, usb_typec_info.id, usb_typec_info.attach);

	if (usb_typec_info.attach)
		schedule_work(&data->pdic_attach_reset_work);
	else
		schedule_work(&data->pdic_detach_reset_work);

	//usb host (otg)
	if (usb_typec_info.rprd == PDIC_NOTIFY_HOST) {
		data->pre_otg_attach = usb_typec_info.rprd;
		GRIP_INFO("otg attach");
	} else if (data->pre_otg_attach) {
		data->pre_otg_attach = 0;
		GRIP_INFO("otg detach");
	}

	for (i = 0; i < NUM_OF_MAX_CHANNEL; i++) {
		if (is_unavailable_channel(data->ch[i]))
			continue;
		enter_unknown_mode(data, TYPE_USB, i);
	}

	data->pre_attach = usb_typec_info.attach;

	return SUCCESS;
}
#endif

#if IS_ENABLED(CONFIG_HALL_NOTIFIER)
static int a96t396_hall_notifier(struct notifier_block *nb,
				unsigned long flip_cover, void *hall_data)
{
#if IS_ENABLED(CONFIG_TABLET_MODEL_CONCEPT)
	struct hall_notifier_context *hall_notifier = hall_data;
#endif
	struct grip_data *data = container_of(nb, struct grip_data,
					hall_nb);
	int i;

	if (data == NULL) {
		GRIP_ERR("data is null\n");
		return SUCCESS;
	}

	if (data->firmware.fw_update_flag == true) {
		GRIP_INFO("fw updating, skip hall reset");
		return SUCCESS;
	} else if (data->check_abnormal_working == true) {
		GRIP_INFO("abnormal working, skip hall reset\n");
		return SUCCESS;
	}

	for (i = 0; i < NUM_OF_MAX_CHANNEL; i++) {
		if (is_unavailable_channel(data->ch[i]))
			continue;

		GRIP_INFO("%dch set flip unknown mode(flip %s,prev %d)\n",
				i + 1, flip_cover ? "close" : "open", data->ch[i]->is_unknown_mode);
	}

#if IS_ENABLED(CONFIG_TABLET_MODEL_CONCEPT)
	if (flip_cover) {
		GRIP_INFO("%s attach\n", hall_notifier->name);
		if (strncmp(hall_notifier->name, "certify_hall", sizeof("certify_hall") - 1) == 0 ||
			strncmp(hall_notifier->name, "hall_wacom", sizeof("hall_wacom") - 1) == 0) {
			schedule_work(&data->reset_work);
			GRIP_INFO("reset only without unknown, %s\n", hall_notifier->name);
		} else if (strncmp(hall_notifier->name, "hall", sizeof("hall") - 1) == 0)
			GRIP_INFO("reset skip, %s\n", hall_notifier->name);
		else
			GRIP_INFO("%s is not defined, hall_notifier_name\n", hall_notifier->name);
	}
#else
	if (flip_cover)
		schedule_work(&data->reset_work);
	for (i = 0; i < NUM_OF_MAX_CHANNEL; i++) {
		if (is_unavailable_channel(data->ch[i]))
			continue;
		enter_unknown_mode(data, TYPE_HALL, i);
	}

#endif
	return SUCCESS;
}
#endif

#if IS_ENABLED(CONFIG_TABLET_MODEL_CONCEPT)
#if IS_ENABLED(CONFIG_KEYBOARD_STM32_POGO_V3) || IS_ENABLED(CONFIG_KEYBOARD_STM32_POGO_V2) || IS_ENABLED(CONFIG_KEYBOARD_STM32_POGO)
static int a96t396_pogo_notifier(struct notifier_block *nb,
		unsigned long action, void *pogo_data)
{
	struct grip_data *data = container_of(nb, struct grip_data,
					pogo_nb);

	switch (action) {
	case POGO_NOTIFIER_ID_ATTACHED:
		schedule_work(&data->reset_work);
		GRIP_INFO("pogo attach\n");
		break;
	case POGO_NOTIFIER_ID_DETACHED:
		GRIP_INFO("pogo dettach\n");
		break;
	};

	return SUCCESS;
}
#endif
#endif

static void a96t396_check_first_working(struct grip_data *data, u8 ch_num)
{
	struct channel *ch = data->ch[ch_num];

	if (ch->grip_p_thd < ch->diff) {
		if (!ch->first_working) {
			ch->first_working = true;
			GRIP_INFO("first working detected %d\n", ch->diff);
		}
	} else {
		if (ch->first_working) {
			ch->is_unknown_mode = UNKNOWN_OFF;
			GRIP_INFO("Release detected %d, unknown mode off\n", ch->diff);
		}
	}
}

static void a96t396_report_event(struct grip_data *data, u8 state, u8 ch_num)
{
	struct channel *ch = data->ch[ch_num];

	if (data->skip_event) {
		GRIP_INFO("int was generated, but event skipped\n");
	} else {
		if (state) {
			input_report_rel(data->input_dev, ch->grip_code, ACTIVE);
			if (ch->is_unknown_mode == UNKNOWN_ON && data->motion)
				ch->first_working = true;
		} else {
			input_report_rel(data->input_dev, ch->grip_code, IDLE);
			if (ch->is_unknown_mode == UNKNOWN_ON && data->motion) {
				if (ch->first_working) {
					GRIP_INFO("unknown mode off\n");
					ch->is_unknown_mode = UNKNOWN_OFF;
				}
			}
		}
		input_report_rel(data->input_dev, ch->unknown_code, ch->is_unknown_mode);
		input_sync(data->input_dev);
		ch->grip_event = state;
	}
}

static bool is_grip_data_changed(u8 prev_buf, u8 buf, u8 ch_idx)
{
	const u8 channel_event_mask[2] = {CH1_BIT_MASK, CH2_BIT_MASK};

	return (buf ^ prev_buf) & channel_event_mask[ch_idx];
}

static bool is_grip_state_active(u8 buf, u8 ch_idx)
{
	const u8 channel_event_mask[2] = {CH1_BIT_MASK, CH2_BIT_MASK};

	return (buf & channel_event_mask[ch_idx]) >> ch_idx;
}

static void irq_work_func(struct work_struct *work)
{
	struct grip_data *data = container_of((struct work_struct *)work,
						struct grip_data, irq_work);
	struct i2c_client *client = data->client;
	int ret;
	u8 ch_idx;
	u8 buf;
	u8 grip_state = 0;

	ret = grip_i2c_read_retry(client, REG_BTNSTATUS, &buf, 1, 3);
	if (ret < 0)
		GRIP_ERR("Fail to get status reset\n");

	GRIP_INFO("buf 0x%02x\n", buf);

	if (data->prev_buf != buf || data->is_first_event) {
		for (ch_idx = 0; ch_idx < NUM_OF_MAX_CHANNEL; ch_idx++) {
			if (is_unavailable_channel(data->ch[ch_idx]))
				continue;

			if (is_grip_data_changed(data->prev_buf, buf, ch_idx) || data->is_first_event) {
				grip_state = is_grip_state_active(buf, ch_idx);
				a96t396_report_event(data, grip_state, ch_idx);
				grip_diff_getdata(data, true, ch_idx);
				grip_check_irq_error(data, grip_state, BY_INTERRUPT_HANDLER, ch_idx);
				GRIP_INFO("%dch %s %x\n", (ch_idx + 1),
					data->ch[ch_idx]->grip_event ? "grip P" : "grip R", buf);
			}
		}
		data->is_first_event = false;
		data->prev_buf = buf;
	} else {
		GRIP_ERR("irq is called but data is same\n");
	}

	if (data->abnormal_mode) {
		for (ch_idx = 0; ch_idx < NUM_OF_MAX_CHANNEL; ch_idx++) {
			if (is_unavailable_channel(data->ch[ch_idx]))
				continue;

			if (data->ch[ch_idx]->grip_event) {
				if (data->ch[ch_idx]->max_diff < data->ch[ch_idx]->diff)
					data->ch[ch_idx]->max_diff = data->ch[ch_idx]->diff;
				data->irq_count++;
			}
		}
	}
}

static bool is_unset_channel(u8 set_up_channels, int ch_idx)
{
	int channel = ch_idx + 1;

	return !(set_up_channels & channel);
}

#if IS_ENABLED(CONFIG_TABLET_MODEL_CONCEPT)
#if IS_ENABLED(CONFIG_KEYBOARD_STM32_POGO_V3) || IS_ENABLED(CONFIG_KEYBOARD_STM32_POGO_V2) || IS_ENABLED(CONFIG_KEYBOARD_STM32_POGO)
static void a96t396_init_work_func(struct work_struct *work)
{
	struct grip_data *data = container_of((struct delayed_work *)work,
		struct grip_data, init_work);

	GRIP_INFO("register pogo_notifier\n");

	pogo_notifier_register(&data->pogo_nb, a96t396_pogo_notifier,
					POGO_NOTIFY_DEV_SENSOR);
}
#endif
#endif

static int setup_channels(struct grip_data *data)
{
	int ch_idx;

	for (ch_idx = 0; ch_idx < NUM_OF_MAX_CHANNEL; ch_idx++) {
		if (is_unset_channel(data->set_up_channels, ch_idx)) {
			GRIP_ERR("%dch skip mem alloc\n", ch_idx + 1);
			data->ch[ch_idx] = NULL;
			continue;
		}
		data->ch[ch_idx] = kzalloc(sizeof(struct channel), GFP_KERNEL);
		if (data->ch[ch_idx] == NULL) {
			GRIP_ERR("%ch Fail to channel mem alloc\n", ch_idx + 1);
			return -ENOMEM;
		}
		data->num_of_channels++;
	}

	return SUCCESS;
}

static void free_channel_mem(struct grip_data *data)
{
	int ch_idx;

	for (ch_idx = 0; ch_idx < NUM_OF_MAX_CHANNEL; ch_idx++) {
		if (is_unavailable_channel(data->ch[ch_idx]))
			continue;
		kfree(data->ch[ch_idx]);
	}
}

static void setup_work_func(struct grip_data *data)
{
	INIT_DELAYED_WORK(&data->debug_work, a96t396_debug_work_func);
	INIT_WORK(&data->irq_work, irq_work_func);
	INIT_WORK(&data->pdic_attach_reset_work, pdic_attach_reset_work_func);
	INIT_WORK(&data->pdic_detach_reset_work, pdic_detach_reset_work_func);
	INIT_WORK(&data->reset_work, reset_work_func);
	INIT_DELAYED_WORK(&data->firmware_work, a96t396_firmware_work_func);
#if IS_ENABLED(CONFIG_TABLET_MODEL_CONCEPT)
#if IS_ENABLED(CONFIG_KEYBOARD_STM32_POGO_V3) || IS_ENABLED(CONFIG_KEYBOARD_STM32_POGO_V2) || IS_ENABLED(CONFIG_KEYBOARD_STM32_POGO)
	INIT_DELAYED_WORK(&data->init_work, a96t396_init_work_func);
#endif
#endif
}

static int input_init(struct grip_data *data)
{
	struct input_dev *input_dev;
	int ch_idx;
	int ret = SUCCESS;
	unsigned int grip_code[2] = {REL_MISC, REL_DIAL};
	unsigned int unknown_code[2] = {REL_X, REL_Y};

	input_dev = input_allocate_device();
	if (!input_dev) {
		GRIP_ERR("input dev alloc err\n");
		return -ENOMEM;
	}
	input_dev->name = module_name[data->ic_num];
	input_dev->id.bustype = BUS_I2C;

	for (ch_idx = 0; ch_idx < NUM_OF_MAX_CHANNEL; ch_idx++) {
		if (is_unavailable_channel(data->ch[ch_idx]))
			continue;
		input_set_capability(input_dev, EV_REL, grip_code[ch_idx]);
		input_set_capability(input_dev, EV_REL, unknown_code[ch_idx]);
	}

	input_set_drvdata(input_dev, data);
	data->input_dev = input_dev;


	ret = input_register_device(input_dev);
	if (ret) {
		GRIP_ERR("fail to register input dev %d\n", ret);
		input_free_device(input_dev);
		return ret;
	}

	return SUCCESS;
}

static int noti_input_init(struct grip_data *data)
{
	struct input_dev *noti_input_dev;
	int ret = SUCCESS;

	if (!data->unknown_ch_selection)
		return ret;

	noti_input_dev = input_allocate_device();
	if (!noti_input_dev) {
		GRIP_ERR("noti_input_allocate fail\n");
		return -ENOMEM;
	}

	data->noti_input_dev = noti_input_dev;
	noti_input_dev->name = NOTI_MODULE_NAME;
	noti_input_dev->id.bustype = BUS_I2C;

	input_set_capability(noti_input_dev, EV_REL, REL_X);
	input_set_drvdata(noti_input_dev, data);

	ret = input_register_device(noti_input_dev);
	if (ret) {
		input_free_device(noti_input_dev);
		GRIP_ERR("fail to register noti input dev %d\n", ret);
		return ret;
	}

	return SUCCESS;
}

static int create_sysfs_attributes(struct grip_data *data)
{
	struct device_attribute *sensor_attributes[SENSOR_ATTR_SIZE];
	struct device_attribute **ch_sensor_attrs[] = {
#ifdef CONFIG_SENSORS_A96T396_CH1
		grip_ch1_sensor_attrs,
#endif
#ifdef CONFIG_SENSORS_A96T396_CH2
		grip_ch2_sensor_attrs,
#endif
	};

	int grip_sensor_attr_size = sizeof_grip_sensor_attr() / sizeof(ssize_t *);
	int ch_size = 0, offset = 0;
	int i, ret = 0;

	memcpy(sensor_attributes, grip_common_sensor_attrs, sizeof_grip_sensor_attr());
	offset = grip_sensor_attr_size - 1;

	for (i = 0; i < NUM_OF_MAX_CHANNEL; i++) {
		if (is_unavailable_channel(data->ch[i]))
			continue;

		ch_size = get_channel_attr_size(i) / sizeof(ssize_t *);

		if (ch_size + grip_sensor_attr_size > SENSOR_ATTR_SIZE) {
			GRIP_ERR("fail mem size of sensor_attr is exceeded size %d, %d\n", ch_size, grip_sensor_attr_size);
			return -ENOMEM;
		}
		memcpy(sensor_attributes + offset, ch_sensor_attrs[i], get_channel_attr_size(i));
		offset = offset + ch_size - 1;
		grip_sensor_attr_size = grip_sensor_attr_size + ch_size - 1;
	}

	ret = sensors_register(&data->dev, data, sensor_attributes,
				(char *)module_name[data->ic_num]);

	return ret;
}

static int setup_irq(struct grip_data *data)
{
	struct i2c_client *client = data->client;
	int ret;

	client->irq = gpio_to_irq(data->grip_int);

	ret = request_threaded_irq(client->irq, NULL, a96t396_interrupt,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT, device_name[data->ic_num], data);

	disable_irq(client->irq);
	data->is_irq_active = false;

	if (ret < 0) {
		GRIP_ERR("Failed to register interrupt: %d\n", ret);
		return ret;
	}

	data->irq = client->irq;
	device_init_wakeup(&client->dev, true);

	return ret;
}

static void initialize_variable(struct grip_data *data)
{
	int i = 0;
	unsigned int grip_code[2] = {REL_MISC, REL_DIAL};
	unsigned int unknown_code[2] = {REL_X, REL_Y};

	data->probe_done = false;
	data->current_state = false;
	data->skip_event = false;
	data->prev_buf = 0x00;
	data->crc_check = CRC_PASS;
	data->motion = 1;
	data->num_of_channels = 0;
	data->param.is_tuning_mode = 0;
	data->pre_attach = -1;
	data->is_power_down_on_error = !(data->fail_safe_concept[1]);

	for (i = 0; i < NUM_OF_MAX_CHANNEL; i++) {
		if (data->ch[i] == NULL)
			continue;
		data->ch[i]->num = i;
		data->ch[i]->grip_code = grip_code[i];
		data->ch[i]->unknown_code = unknown_code[i];
		data->ch[i]->is_unknown_mode = UNKNOWN_OFF;
		data->ch[i]->first_working = false;
		data->ch[i]->grip_event = IDLE;
		data->ch[i]->is_fail_safe_concept_enabled = data->fail_safe_concept[0];
		data->ch[i]->is_working_state_on_error = data->fail_safe_concept[2 + i];
	}
}

static void register_pdic_notifier(struct grip_data *data)
{
#if IS_ENABLED(CONFIG_PDIC_NOTIFIER) && IS_ENABLED(CONFIG_USB_TYPEC_MANAGER_NOTIFIER)
	data->pdic_status = OFF;
	data->pdic_pre_attach = 0;
	manager_notifier_register(&data->pdic_nb,
					a96t396_pdic_handle_notification,
					MANAGER_NOTIFY_PDIC_SENSORHUB);
#endif
}

static void register_hall_notifier(struct grip_data *data)
{
#if IS_ENABLED(CONFIG_HALL_NOTIFIER)
	GRIP_INFO("register hall notifier\n");
	data->hall_nb.priority = 1;
	data->hall_nb.notifier_call = a96t396_hall_notifier;
	hall_notifier_register(&data->hall_nb);
#endif
}

#if (KERNEL_VERSION(6, 3, 0) <= LINUX_VERSION_CODE)
static int a96t396_probe(struct i2c_client *client)
#else
static int a96t396_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
#endif
{
	struct grip_data *data;
	int ret;
	int ic_num = 0;
	static int fw_count;

	probe_count = probe_count + 1;

	ic_num = (enum ic_num) of_device_get_match_data(&client->dev);


	pr_info("[GRIP_%s] %s: start (0x%x) - probe count %d\n",
		grip_name[ic_num], __func__, client->addr, probe_count);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("[GRIP_%s] i2c_check_functionality fail\n", grip_name[ic_num]);
		return -EIO;
	}

	data = kzalloc(sizeof(struct grip_data), GFP_KERNEL);
	if (!data) {
		pr_info("[GRIP %d] Fail to alloc\n", ic_num);
		ret = -ENOMEM;
		goto err_alloc;
	}
	data->ic_num = ic_num;
	data->client = client;
	data->grip_ws = wakeup_source_register(&client->dev, "grip_wake_lock");

	ret = a96t396_parse_dt(data, &client->dev);
	if (ret) {
		GRIP_ERR("fail to a96t396_parse_dt\n");
		goto err_config;
	}

	if (setup_channels(data))
		goto exit_channel_kmalloc;

	initialize_variable(data);

	ret = a96t396_irq_init(&client->dev, data);
	if (ret) {
		GRIP_ERR("fail to irq init\n");
		goto pwr_config;
	}

	if (data->power) {
		data->power(data, true);
		usleep_range(RESET_DELAY, RESET_DELAY + 1);
	}

	mutex_init(&data->lock);
	mutex_init(&data->tuning_lock);

	i2c_set_clientdata(client, data);
	data->dev = &client->dev;

	if (input_init(data))
		goto err_input_alloc;
	if (noti_input_init(data))
		goto err_noti_input_alloc;

	setup_work_func(data);

#if defined(CONFIG_SENSORS_CORE_AP)
	ret = sensors_create_symlink(&data->input_dev->dev.kobj, data->input_dev->name);
#else // !CONFIG_SENSORS_CORE_AP
	ret = sensors_create_symlink(data->input_dev);
#endif
	if (ret < 0) {
		GRIP_ERR("fail to create symlink %d\n", ret);
		goto err_sysfs_symlink;
	}

	ret = sysfs_create_group(&data->input_dev->dev.kobj, &grip_attribute_group);
	if (ret < 0) {
		GRIP_ERR("fail to create sysfs group %d\n", ret);
		goto err_sysfs_group;
	}

	if (create_sysfs_attributes(data))
		goto err_sensor_register;

	data->enabled = true;
	data->dev = &client->dev;

	if (setup_irq(data))
		goto err_req_irq;

	a96t396_set_debug_work(data, ON, 20000);

	if (fw_count == 0)
		a96t396_set_firmware_work(data, ON, 0);
	gp_fw_work[ic_num] = &data->firmware_work;
	fw_count = fw_count + 1;

	register_pdic_notifier(data);
	register_hall_notifier(data);

#if IS_ENABLED(CONFIG_TABLET_MODEL_CONCEPT)
#if IS_ENABLED(CONFIG_KEYBOARD_STM32_POGO_V3) || IS_ENABLED(CONFIG_KEYBOARD_STM32_POGO_V2) || IS_ENABLED(CONFIG_KEYBOARD_STM32_POGO)
	schedule_delayed_work(&data->init_work, msecs_to_jiffies(5000));
#endif
#endif

	GRIP_INFO("done\n");
	data->probe_done = true;
	data->resume_called = false;
	return SUCCESS;

err_req_irq:
	sensors_unregister(data->dev, grip_common_sensor_attrs);
err_sensor_register:
	sysfs_remove_group(&data->input_dev->dev.kobj,
			&grip_attribute_group);
err_sysfs_group:
#if defined(CONFIG_SENSORS_CORE_AP)
	sensors_remove_symlink(&data->input_dev->dev.kobj, data->input_dev->name);
#else
	sensors_remove_symlink(data->input_dev);
#endif
err_sysfs_symlink:
	input_unregister_device(data->noti_input_dev);
err_noti_input_alloc:
	input_unregister_device(data->input_dev);
err_input_alloc:
	mutex_destroy(&data->lock);
	mutex_destroy(&data->tuning_lock);
	gpio_free(data->grip_int);
#ifndef CONFIG_SENSORS_A96T396_LDO_SHARE
	if (data->power)
		data->power(data, false);
#endif
pwr_config:
	free_channel_mem(data);
exit_channel_kmalloc:
err_config:
	wakeup_source_unregister(data->grip_ws);
	kfree(data);
err_alloc:
	pr_info("[GRIP %s] Failed\n", grip_name[ic_num]);
	return ret;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
void a96t396_remove(struct i2c_client *client)
#else
static int a96t396_remove(struct i2c_client *client)
#endif
{
	struct grip_data *data = i2c_get_clientdata(client);

	if (data->enabled)
		data->power(data, false);

	data->enabled = false;
	device_init_wakeup(&client->dev, false);
	wakeup_source_unregister(data->grip_ws);
	cancel_delayed_work_sync(&data->debug_work);
#ifdef CONFIG_SENSORS_FW_VENDOR
	cancel_delayed_work_sync(&data->firmware_work);
#endif
	if (data->irq >= 0)
		free_irq(data->irq, data);
	sensors_unregister(data->dev, grip_common_sensor_attrs);
	sysfs_remove_group(&data->input_dev->dev.kobj,
				&grip_attribute_group);
#if defined(CONFIG_SENSORS_CORE_AP)
	sensors_remove_symlink(&data->input_dev->dev.kobj, data->input_dev->name);
#else
	sensors_remove_symlink(data->input_dev);
#endif
	input_unregister_device(data->input_dev);
	input_free_device(data->input_dev);

	free_channel_mem(data);

	kfree(data);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
	return;
#else
	return SUCCESS;
#endif
}

static int a96t396_suspend(struct device *dev)
{
	struct grip_data *data = dev_get_drvdata(dev);

	if (data->current_state && data->is_irq_active) {
		data->prevent_sleep_irq = true;
#if !defined(CONFIG_SENSORS_CORE_AP)
		disable_irq(data->irq);
#endif
		enable_irq_wake(data->irq);
	}
	data->resume_called = false;
	GRIP_INFO("current_state %d\n", data->current_state);
	a96t396_set_debug_work(data, 0, 0);

	cancel_work_sync(&data->pdic_attach_reset_work);
	cancel_work_sync(&data->pdic_detach_reset_work);
	cancel_work_sync(&data->reset_work);

	return SUCCESS;
}

static int a96t396_resume(struct device *dev)
{
	struct grip_data *data = dev_get_drvdata(dev);

	GRIP_INFO("current_state %d\n", data->current_state);
	data->resume_called = true;
	a96t396_set_debug_work(data, 1, 0);

	if (data->prevent_sleep_irq) {
		data->prevent_sleep_irq = false;
		disable_irq_wake(data->irq);
#if !defined(CONFIG_SENSORS_CORE_AP)
		enable_irq(data->irq);
#endif
	}
	return SUCCESS;
}

static void a96t396_shutdown(struct i2c_client *client)
{
	struct grip_data *data = i2c_get_clientdata(client);

	a96t396_set_debug_work(data, 0, 0);

	if (data->enabled) {
		if (data->is_irq_active)
			disable_irq(data->irq);
		data->power(data, false);
	}
	data->enabled = false;
	data->check_abnormal_working = true;

	cancel_work_sync(&data->pdic_attach_reset_work);
	cancel_work_sync(&data->pdic_detach_reset_work);
	cancel_work_sync(&data->reset_work);
}

static const struct dev_pm_ops a96t396_pm_ops = {
	.suspend = a96t396_suspend,
	.resume = a96t396_resume,
};

static const struct i2c_device_id a96t396_device_id[] = {
	{"grip_sensor", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, a96t396_device_id);


static const struct of_device_id a96t396_match_table[] = {

#if IS_ENABLED(CONFIG_SENSORS_A96T396)
	{ .compatible = "a96t396", .data = (void *)MAIN_GRIP},
#endif
#if IS_ENABLED(CONFIG_SENSORS_A96T396_SUB)
	{ .compatible = "a96t396_sub", .data = (void *)SUB_GRIP},
#endif
#if IS_ENABLED(CONFIG_SENSORS_A96T396_SUB2)
	{ .compatible = "a96t396_sub2", .data = (void *)SUB2_GRIP},
#endif
#if IS_ENABLED(CONFIG_SENSORS_A96T396_WIFI)
	{ .compatible = "a96t396_wifi", .data = (void *)WIFI_GRIP},
#endif
	{},
};

static struct i2c_driver a96t396_driver = {
	.probe = a96t396_probe,
	.remove = a96t396_remove,
	.shutdown = a96t396_shutdown,
	.id_table = a96t396_device_id,
	.driver = {
		.name = "A96T396",
		.owner = THIS_MODULE,
		.of_match_table = a96t396_match_table,
		.pm = &a96t396_pm_ops
	},
};

static int __init a96t396_init(void)
{
	int ret = 0;

	if (is_lpcharge_pdic_param()) {
		pr_err("[GRIP] %s: lpm : Do not load driver\n", __func__);
		return SUCCESS;
	}

	max_probe_count = GRIP_MAX_CNT;

	ret = i2c_add_driver(&a96t396_driver);
	if (ret)
		pr_err("[GRIP] a96t396 probe fail\n");

	return ret;
}

static void __exit a96t396_exit(void)
{
	i2c_del_driver(&a96t396_driver);
}

module_init(a96t396_init);
module_exit(a96t396_exit);

MODULE_AUTHOR("Samsung Electronics");
MODULE_DESCRIPTION("Grip sensor driver for a96t396 chip");
MODULE_LICENSE("GPL");
