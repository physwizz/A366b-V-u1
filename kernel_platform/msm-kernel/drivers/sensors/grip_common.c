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

bool is_unavailable_channel(struct channel *ch)
{
	return (ch == NULL);
}

void enter_unknown_mode(struct grip_data *data, int type, u8 ch_num)
{
	struct channel *ch = data->ch[ch_num];

	if (ch->noti_enable && !data->skip_event) {
		data->motion = 0;
		ch->first_working = false;

		if (ch->is_unknown_mode == UNKNOWN_OFF) {
			ch->is_unknown_mode = UNKNOWN_ON;
			if (data->current_state) {
				input_report_rel(data->input_dev, ch->unknown_code, ch->is_unknown_mode);
				input_sync(data->input_dev);
			}
			GRIP_INFO("%dch UNKNOWN Re-enter\n", ch->num + 1);
		} else {
			GRIP_INFO("%dch already UNKNOWN\n", ch->num + 1);
		}
	}

	if (ch->noti_enable) {
		GRIP_INFO("%dch enable %d, type %d\n", ch->num + 1, ch->noti_enable, type);
		input_report_rel(data->noti_input_dev, ch->noti_code, type);
		input_sync(data->noti_input_dev);
	}

	if (data->check_abnormal_working == true && !data->skip_event &&
		ch->is_fail_safe_concept_enabled) {
		GRIP_INFO("%dch send abnormal event (%d)\n", ch->num + 1, ch->is_fail_safe_concept_enabled);
		input_report_rel(data->input_dev, ch->grip_code, -1);
		input_sync(data->input_dev);
	}
}

void enter_error_mode(struct grip_data *data, enum grip_error_state err_state)
{
	u8 i;

	GRIP_INFO("enter %d\n", data->err_state);
	if (data->is_irq_active) {
		disable_irq(data->irq);
		disable_irq_wake(data->irq);
		data->is_irq_active = false;
	}

#if !defined(CONFIG_SENSORS_A96T396_LDO_SHARE)
	if (data->is_power_down_on_error && !data->check_abnormal_working) {
		GRIP_INFO("forced dvdd_vreg turned off\n\n");
		data->power(data, false);
	}
#endif

	data->check_abnormal_working = true;
	data->err_state |= 0x1 << err_state;

	for (i = 0; i < NUM_OF_MAX_CHANNEL; i++) {
		if (is_unavailable_channel(data->ch[i]))
			continue;
		enter_unknown_mode(data, TYPE_FORCE + data->err_state, i);
	}

#if IS_ENABLED(CONFIG_SENSORS_GRIP_FAILURE_DEBUG)
	update_grip_error(data->ic_num, data->err_state);
#endif

	GRIP_INFO("exit %d\n", data->err_state);
}

int grip_i2c_write(struct i2c_client *client, u8 reg, u8 *val)
{
	struct grip_data *data = i2c_get_clientdata(client);
	struct i2c_msg msg[1];
	unsigned char buf[2];
	int ret = -1;
	int retry = data->retry_i2c;

	mutex_lock(&data->lock);

	if (data->i2c_fail_count > 3)
		goto exit_i2c_write;

	buf[0] = reg;
	buf[1] = *val;
	msg->addr = client->addr;
	msg->flags = I2C_M_WR;
	msg->len = 2;
	msg->buf = buf;

	while (retry--) {
		ret = i2c_transfer(client->adapter, msg, 1);
		if (ret < 0) {
			data->i2c_fail_count++;
			if (data->i2c_fail_count >= 3 && !data->check_abnormal_working) {
				enter_error_mode(data, FAIL_I2C_READ_3_TIMES);
				goto exit_i2c_write;
			}
			GRIP_ERR("i2c_fail_count %d, ret %d\n", data->i2c_fail_count, ret);
			usleep_range(10000, 11000);
		} else {
			data->i2c_fail_count = 0;
			goto exit_i2c_write;
		}
		GRIP_ERR("addr set err %d,%d\n", retry, ret);
	}
exit_i2c_write:
	mutex_unlock(&data->lock);
	return ret;
}

int grip_i2c_write_retry(struct i2c_client *client, u8 reg, u8 *val, int retry)
{
	struct grip_data *data = i2c_get_clientdata(client);
	int ret = 0;

	while (retry--) {
		ret = grip_i2c_write(data->client, reg, val);
		if (ret >= 0)
			break;
		GRIP_ERR("retry err %d,%d\n", retry, ret);
	}
	return ret;
}

int grip_i2c_read(struct i2c_client *client, u8 reg, u8 *val, unsigned int len)
{
	struct grip_data *data = i2c_get_clientdata(client);
	struct i2c_msg msg[2];
	int ret = -1;
	int retry = data->retry_i2c;

	mutex_lock(&data->lock);

	if (data->i2c_fail_count > 3)
		goto exit_i2c_read;

	msg[0].addr = data->client->addr;
	msg[0].flags = I2C_M_WR;
	msg[0].len = 1;
	msg[0].buf = &reg;

	msg[1].addr = data->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = val;

	while (retry--) {
		ret = i2c_transfer(client->adapter, msg, 2);
		if (ret < 0) {
			data->i2c_fail_count++;
			if (data->i2c_fail_count >= 3 && !data->check_abnormal_working) {
				enter_error_mode(data, FAIL_I2C_READ_3_TIMES);
				goto exit_i2c_read;
			}
			GRIP_ERR("i2c_fail_count %d, ret %d\n", data->i2c_fail_count, ret);
			usleep_range(10000, 11000);
		} else {
			data->i2c_fail_count = 0;
			break;
		}
	}
exit_i2c_read:
	mutex_unlock(&data->lock);
	return ret;
}

int grip_i2c_read_data(struct i2c_client *client, u8 *val, unsigned int len)
{
	struct grip_data *data = i2c_get_clientdata(client);
	struct i2c_msg msg;
	int ret = -1;
	int retry = data->retry_i2c;

	mutex_lock(&data->lock);

	if (data->i2c_fail_count > 3)
		goto exit_i2c_read_data;

	msg.addr = client->addr;
	msg.flags = 1;/*I2C_M_RD*/
	msg.len = len;
	msg.buf = val;

	while (retry--) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret < 0) {
			data->i2c_fail_count++;
			if (data->i2c_fail_count >= 3 && !data->check_abnormal_working) {
				enter_error_mode(data, FAIL_I2C_READ_3_TIMES);
				goto exit_i2c_read_data;
			}
			GRIP_ERR("i2c_fail_count %d, ret %d\n", data->i2c_fail_count, ret);
			usleep_range(10000, 11000);
		} else {
			data->i2c_fail_count = 0;
			goto exit_i2c_read_data;
		}
		GRIP_ERR("addr set err %d,%d\n", retry, ret);
	}
exit_i2c_read_data:
	mutex_unlock(&data->lock);
	return ret;
}

int grip_i2c_read_retry(struct i2c_client *client, u8 reg, u8 *val, unsigned int len, int retry)
{
	struct grip_data *data = i2c_get_clientdata(client);
	int ret = 0;

	while (retry--) {
		ret = grip_i2c_read(data->client, reg, val, len);
		if (ret >= 0)
			break;
		GRIP_ERR("retry err %d,%d\n", retry, ret);
	}
	return ret;
}

int grip_i2c_read_checksum(struct grip_data *data)
{
	unsigned char buf[6] = {0xAC, 0x9E, 0x04, 0x00, 0x37, 0xFF};
	unsigned char buf2[1] = {0x00};
	unsigned char checksum[6] = {0, };
	int ret;

	i2c_master_send(data->client, buf, 6);
	usleep_range(5000, 6000);

	i2c_master_send(data->client, buf2, 1);
	usleep_range(5000, 6000);

	ret = grip_i2c_read_data(data->client, checksum, 6);

	GRIP_INFO("ret:%d [%X][%X][%X][%X][%X]\n", ret,
			checksum[0], checksum[1], checksum[2], checksum[4], checksum[5]);
	data->firmware.checksum_h = checksum[4];
	data->firmware.checksum_l = checksum[5];
	return ret;
}

void grip_check_irq_error(struct grip_data *data, u8 irq_state, enum entry_path path, u8 ch_num)
{
	struct channel *ch = data->ch[ch_num];

	if (data->is_irq_active && !data->check_abnormal_working) {

		if (path == BY_INTERRUPT_HANDLER) {
			ch->state_miss_matching_count = 0;
			ch->prev_state = irq_state;
		} else if (path == BY_ENABLE_FUNC) {
			ch->prev_state = irq_state;
		} else if (ch->prev_state != irq_state) {
			data->state_miss_matching_count++;
			GRIP_ERR("ch[%d] prev %x state %x miss_cnt %d\n", ch->num,
				ch->prev_state, irq_state, ch->state_miss_matching_count);
			data->prev_state = irq_state;
		}

		if (ch->state_miss_matching_count >= 3) {
			GRIP_INFO("enter_error_mode with IRQ\n");
			enter_error_mode(data, FAIL_IRQ_MISS_MATCH);
		}
	}
}

void check_irq_status(struct grip_data *data, enum entry_path path)
{
	const u8 register_bit[NUM_OF_MAX_CHANNEL] = {0x01, 0x02};
	u8 buf, i, status;
	int ret;

	ret = grip_i2c_read_retry(data->client, REG_BTNSTATUS, &buf, 1, 3);
	if (ret < 0) {
		GRIP_ERR("Fail to get status\n");
		for (i = 0; i < NUM_OF_MAX_CHANNEL; i++) {
			if (is_unavailable_channel(data->ch[i]))
				continue;
			enter_unknown_mode(data, TYPE_FORCE, i);
		}
	} else {
		for (i = 0; i < NUM_OF_MAX_CHANNEL; i++) {
			if (is_unavailable_channel(data->ch[i]))
				continue;
			status = (buf & register_bit[i]) >> i;
			grip_check_irq_error(data, status, path, i);
		}
	}
}

void grip_set_enable(struct grip_data *data, int enable)
{
	u8 cmd;
	int ret;

	if (data->check_abnormal_working == true) {
		data->current_state = enable;

		if (enable) {
			GRIP_INFO("%d, abnormal_working\n", enable);
			enter_error_mode(data, FAIL_UPDATE_PREV_STATE);
		}
		return;
	}

	if (data->current_state == enable) {
		GRIP_INFO("%d, skip exception case\n", enable);
		return;
	}

	GRIP_INFO("enable old %d, new %d\n", data->current_state, enable);
	if (enable) {
		data->prev_buf = 0;
		data->is_first_event = true;
		cmd = CMD_ON;

		ret = grip_i2c_write(data->client, REG_SAR_ENABLE, &cmd);
		if (ret < 0)
			GRIP_ERR("fail to enable\n");

		if (!data->is_irq_active) {
			enable_irq(data->irq);
			data->is_irq_active = true;
		}
		data->irq_en_cnt++;

		check_irq_status(data, BY_ENABLE_FUNC);
	} else {
		cmd = CMD_OFF;

		if (data->is_irq_active) {
			disable_irq(data->irq);
			data->is_irq_active = false;
		}

		ret = grip_i2c_write(data->client, REG_SAR_ENABLE, &cmd);
		if (ret < 0)
			GRIP_ERR("fail to disable\n");
	}
	data->current_state = enable;
}

void grip_diff_getdata(struct grip_data *data, bool log, u8 ch_num)
{
	struct channel *ch;
	int ret;
	u8 r_buf[2] = {0,};
	u8 diff_reg[2] = {REG_SAR_DIFFDATA, REG_SAR_DIFFDATA_D_2CH};

	ch = data->ch[ch_num];
	ret = grip_i2c_read(data->client, diff_reg[ch->num], r_buf, 2);
	if (ret < 0)
		GRIP_ERR("err %d\n", ret);
	ch->diff = (r_buf[0] << 8) | r_buf[1];

	if (log)
		GRIP_INFO("%dch diff :%u\n", ch_num + 1, ch->diff);
}

void grip_check_diff_and_cap(struct grip_data *data)
{
	u8 r_buf[2] = {0, 0};
	int i, value, ret;

	ret = grip_i2c_read(data->client, REG_SAR_TOTALCAP_READ, r_buf, 2);
	if (ret < 0)
		GRIP_ERR("fail %d\n", ret);

	value = (r_buf[0] << 8) | r_buf[1];
	GRIP_INFO("Cap %d\n", value);

	for (i = 0; i < NUM_OF_MAX_CHANNEL; i++) {
		if (is_unavailable_channel(data->ch[i]))
			continue;
		grip_diff_getdata(data, true, i);
	}
}

void get_threshold(struct grip_data *data, int ch_num)
{
	int ret;
	u8 r_buf[4];
	const u8 reg_sar_threshold[NUM_OF_MAX_CHANNEL] = {REG_SAR_THRESHOLD, REG_SAR_THRESHOLD_2CH};
	const u8 reg_release_threshold[NUM_OF_MAX_CHANNEL] = {REG_SAR_RELEASE_THRESHOLD, REG_SAR_RELEASE_THRESHOLD_2CH};
	const u8 reg_noise_threshold[NUM_OF_MAX_CHANNEL] = {REG_SAR_NOISE_THRESHOLD, REG_SAR_NOISE_THRESHOLD_2CH};

	ret = grip_i2c_read(data->client, reg_sar_threshold[ch_num], r_buf, 2);
	if (ret < 0) {
		GRIP_ERR("err %d\n", ret);
		data->ch[ch_num]->grip_p_thd = 0;
	}
	data->ch[ch_num]->grip_p_thd = (r_buf[0] << 8) | r_buf[1];

	ret = grip_i2c_read(data->client, reg_release_threshold[ch_num], r_buf, 2);
	if (ret < 0) {
		GRIP_ERR("err %d\n", ret);
		data->ch[ch_num]->grip_r_thd = 0;
	}
	data->ch[ch_num]->grip_r_thd = (r_buf[0] << 8) | r_buf[1];

	ret = grip_i2c_read(data->client, reg_noise_threshold[ch_num], r_buf, 2);
	if (ret < 0) {
		GRIP_ERR("err %d\n", ret);
		data->ch[ch_num]->grip_n_thd = 0;
	}
	data->ch[ch_num]->grip_n_thd = (r_buf[0] << 8) | r_buf[1];
}

void grip_sw_reset(struct grip_data *data)
{
	int ret;
	u8 cmd = CMD_SW_RESET;

	GRIP_INFO("\n");

	if (data->check_abnormal_working == true) {
		GRIP_INFO("abnormal working, skip reset\n");
		return;
	}

	grip_check_diff_and_cap(data);
	usleep_range(10000, 10010);

	ret = grip_i2c_write(data->client, REG_SW_RESET, &cmd);
	if (ret < 0)
		GRIP_ERR("err %d\n", ret);
	else
		usleep_range(35000, 35010);
}

void grip_reset_for_bootmode(struct grip_data *data)
{
	GRIP_INFO("\n");

	if (data->check_abnormal_working == true) {
		GRIP_INFO("abnormal working, skip reset for bootmode\n");
		return;
	}

	data->power(data, false);
	usleep_range(50000, 50010);
	data->power(data, true);
}

static void grip_reset(struct grip_data *data)
{
	if (data->enabled == false)
		return;

	GRIP_INFO("start\n");
	if (data->is_irq_active) {
		disable_irq_nosync(data->irq);
		data->is_irq_active = false;
	}

	data->enabled = false;

	grip_reset_for_bootmode(data);
	usleep_range(RESET_DELAY, RESET_DELAY + 1);

	if (data->current_state)
		grip_set_enable(data, 1);

	data->enabled = true;

	GRIP_INFO("done\n");
}

int grip_load_fw_kernel(struct grip_data *data)
{
	int ret = 0;

	ret = request_firmware(&data->firmware.firm_data_bin,
		data->firmware.fw_path, &data->client->dev);
	if (ret < 0) {
		GRIP_ERR("req firmware err\n");
		return ret;
	}
	data->firmware.firm_size = data->firmware.firm_data_bin->size;
	data->firmware.fw_ver_bin = data->firmware.firm_data_bin->data[5];
	data->firmware.md_ver_bin = data->firmware.firm_data_bin->data[1];
	GRIP_INFO("fw 0x%x, md 0x%x\n", data->firmware.fw_ver_bin, data->firmware.md_ver_bin);

	data->firmware.checksum_h_bin = data->firmware.firm_data_bin->data[8];
	data->firmware.checksum_l_bin = data->firmware.firm_data_bin->data[9];

	GRIP_INFO("crc 0x%x 0x%x\n", data->firmware.checksum_h_bin, data->firmware.checksum_l_bin);

	return ret;
}

void grip_always_active(struct grip_data *data, int on)
{
	int ret;
	u8 cmd, r_buf;

	GRIP_INFO("Always active mode %d\n", on);

	if (on == 1)
		cmd = GRIP_ALWAYS_ACTIVE_ENABLE;
	else
		cmd = GRIP_ALWAYS_ACTIVE_DISABLE;

	ret = grip_i2c_write(data->client, REG_GRIP_ALWAYS_ACTIVE, &cmd);
	if (ret < 0) {
		GRIP_ERR("err %d\n", ret);
		return;
	}

	msleep(20);

	ret = grip_i2c_read(data->client, REG_GRIP_ALWAYS_ACTIVE, &r_buf, 1);
	if (ret < 0) {
		GRIP_ERR("err %d\n", ret);
		return;
	}

	if ((cmd == GRIP_ALWAYS_ACTIVE_ENABLE && r_buf == GRIP_ALWAYS_ACTIVE_ENABLE) ||
			(cmd == GRIP_ALWAYS_ACTIVE_DISABLE && r_buf == GRIP_ALWAYS_ACTIVE_DISABLE))
		GRIP_INFO("cmd 0x%x, return 0x%x\n", cmd, r_buf);
	else
		GRIP_INFO("always_active set fail 0x%x wrong val 0x%x\n", cmd, r_buf);
}

int grip_get_fw_version(struct grip_data *data, bool bootmode, bool activemode)
{
	struct i2c_client *client = data->client;
	u8 buf;
	int ret;

	if (activemode)
		grip_always_active(data, 1);

	ret = grip_i2c_read(client, REG_FW_VER, &buf, 1);
	if (ret < 0) {
		GRIP_ERR("err %d\n", ret);
		if (!bootmode)
			grip_reset(data);
		else
			goto err_grip_revert_mode;
		ret = grip_i2c_read(client, REG_FW_VER, &buf, 1);
		if (ret < 0)
			goto err_grip_revert_mode;
	}
	data->firmware.fw_ver = buf;

	ret = grip_i2c_read(client, REG_MODEL_NO, &buf, 1);
	if (ret < 0) {
		GRIP_ERR("err %d\n", ret);
		if (!bootmode)
			grip_reset(data);
		else
			goto err_grip_revert_mode;
		ret = grip_i2c_read(client, REG_MODEL_NO, &buf, 1);
		if (ret < 0)
			goto err_grip_revert_mode;
	}
	data->firmware.md_ver = buf;

	GRIP_INFO("fw 0x%x, md 0x%x\n", data->firmware.fw_ver, data->firmware.md_ver);

	if (activemode)
		grip_always_active(data, 0);

	return SUCCESS;

err_grip_revert_mode:
	if (activemode)
		grip_always_active(data, 0);

	return -1;
}

int grip_fw_mode_enter(struct grip_data *data)
{
	unsigned char buf[2] = {0xAC, 0x5B};
	u8 cmd = 0;
	int ret = 0;

	GRIP_INFO("cmd send\n");
	ret = i2c_master_send(data->client, buf, 2);
	if (ret != 2) {
		GRIP_ERR("write err\n");
		return -1;
	}

	ret = i2c_master_recv(data->client, &cmd, 1);
	GRIP_INFO("cmd receive %2x, %2x\n", data->firmware.firmup_cmd, cmd);
	if (data->firmware.firmup_cmd != cmd) {
		GRIP_ERR("cmd not matched, firm up err %d\n", ret);
		return -2;
	}

	return SUCCESS;
}

int grip_flash_erase(struct grip_data *data)
{
	unsigned char buf[2] = {0xAC, 0x2D};
	int ret = 0;

	ret = i2c_master_send(data->client, buf, 2);
	if (ret != 2) {
		GRIP_ERR("write err\n");
		return -1;
	}

	return SUCCESS;
}

int grip_check_busy(struct grip_data *data)
{
	int ret, count = 0;
	unsigned char val = 0x00;

	do {
		ret = i2c_master_recv(data->client, &val, sizeof(val));

		if (val)
			count++;
		else
			break;

		if (count > 1000)
			break;
	} while (1);

	if (count > 1000)
		GRIP_ERR("busy %d\n", count);
	return ret;
}

int grip_fw_write(struct grip_data *data, unsigned char *addrH, unsigned char *addrL, unsigned char *val)
{
	int length = 36, ret = 0;
	unsigned char buf[36];

	buf[0] = 0xAC;
	buf[1] = 0x7A;
	memcpy(&buf[2], addrH, 1);
	memcpy(&buf[3], addrL, 1);
	memcpy(&buf[4], val, 32);

	ret = i2c_master_send(data->client, buf, length);
	if (ret != length) {
		GRIP_ERR("write fail[%x%x] %d\n", *addrH, *addrL, ret);
		return ret;
	}

	usleep_range(3000, 3000);
	grip_check_busy(data);

	return SUCCESS;
}

int grip_fw_mode_exit(struct grip_data *data)
{
	unsigned char buf[2] = {0xAC, 0xE1};
	int ret = 0;

	ret = i2c_master_send(data->client, buf, 2);
	if (ret != 2) {
		GRIP_ERR("write err\n");
		return -1;
	}

	usleep_range(RESET_DELAY, RESET_DELAY);
	return SUCCESS;
}

int grip_fw_update(struct grip_data *data, u8 cmd)
{
	int ret, i = 0;
	int count;
	int retry = 3;
	unsigned short address;
	unsigned char addrH, addrL;
	unsigned char buf[32] = {0, };

	GRIP_INFO("start\n");

	count = data->firmware.firm_size / 32;
	address = USER_CODE_ADDRESS;

	while (retry > 0) {
		grip_reset_for_bootmode(data);
		usleep_range(BOOT_DELAY, BOOT_DELAY + 1);

		ret = grip_fw_mode_enter(data);
		if (ret < 0)
			GRIP_ERR("fw_mode_enter fail, retry %d\n",
				((5-retry)+1));
		else
			break;
		retry--;
	}

	if (ret < 0 && retry == 0) {
		GRIP_ERR("fw_mode_enter fail\n");
		return ret;
	}
	usleep_range(5000, 5010);
	GRIP_INFO("fw_mode_cmd sent\n");

	ret = grip_flash_erase(data);

	if (ret < 0) {
		GRIP_ERR("fw_erase err\n");
		return ret;
	}
	usleep_range(FLASH_DELAY, FLASH_DELAY);

	GRIP_INFO("fw_write start\n");
	for (i = 1; i < count; i++) {
		/* first 32byte is header */
		addrH = (unsigned char)((address >> 8) & 0xFF);
		addrL = (unsigned char)(address & 0xFF);
		if (cmd == BUILT_IN)
			memcpy(buf, &data->firmware.firm_data_bin->data[i * 32], 32);
		else if (cmd == SDCARD)
			memcpy(buf, &data->firmware.firm_data_ums[i * 32], 32);

		ret = grip_fw_write(data, &addrH, &addrL, buf);
		if (ret < 0) {
			GRIP_ERR("err, no device %d\n", ret);
			return ret;
		}

		address += 0x20;

		memset(buf, 0, 32);
	}

	ret = grip_i2c_read_checksum(data);
	GRIP_INFO("checksum read%d\n", ret);

	ret = grip_fw_mode_exit(data);
	GRIP_INFO("fw_write end\n");

	return ret;
}

void grip_release_fw(struct grip_data *data, u8 cmd)
{
	switch (cmd) {
	case BUILT_IN:
		release_firmware(data->firmware.firm_data_bin);
		break;

	case SDCARD:
		kfree(data->firmware.firm_data_ums);
		break;

	default:
		break;
	}
}

#ifdef CONFIG_SENSORS_A96T396_CRC_CHECK
int grip_crc_check(struct grip_data *data)
{
	unsigned char cmd = 0xAA;
	unsigned char val = 0xFF;
	unsigned char retry = 2;
	int ret;

/*
* abov grip fw uses active/deactive mode in each period
* To check crc check, make the mode as always active mode.
*/

	grip_always_active(data, 1);

	/* crc check */
	ret = grip_i2c_write(data->client, REG_FW_VER, &cmd);
	if (ret < 0) {
		GRIP_INFO("enter err\n");
		grip_always_active(data, 0);
		return ret;
	}
	// Note: The final decision of 'write result' is done in 'grip_flash_fw()'.
	data->crc_check = CRC_FAIL;

	while (retry--) {
		msleep(400);

		ret = grip_i2c_read(data->client, REG_FW_VER, &val, 1);
		if (ret < 0) {
			GRIP_INFO("fw read err\n");
			continue;
		}

		ret = (int)val;
		if (val == CRC_FAIL) {
			GRIP_INFO("err 0x%2x\n", val);
		} else {

			data->crc_check = CRC_PASS;
			GRIP_INFO("check normal 0x%2x\n", val);
			break;
		}
	}

	grip_always_active(data, 0);
	return ret;
}
#endif

int grip_flash_fw(struct grip_data *data, bool probe, u8 cmd)
{
	int retry = 2;
	int ret;

	ret = grip_get_fw_version(data, probe, true);
	if (ret)
		data->firmware.fw_ver = 0;

	switch (cmd) {
	case BUILT_IN:
		break;

	case SDCARD:
		if (data->firmware.firm_data_ums == NULL) {
			GRIP_ERR("data is NULL\n");
			return -1;
		}
		break;

	default:
		return -1;
	}

	data->firmware.fw_update_flag = true;

	while (retry--) {
		ret = grip_fw_update(data, cmd);
		if (ret < 0)
			break;

		if (cmd == BUILT_IN) {
			if ((data->firmware.checksum_h != data->firmware.checksum_h_bin) ||
				(data->firmware.checksum_l != data->firmware.checksum_l_bin)) {
				GRIP_ERR("checksum err 0x%x,0x%x/0x%x,0x%x retry:%d\n",
						data->firmware.checksum_h, data->firmware.checksum_l,
						data->firmware.checksum_h_bin, data->firmware.checksum_l_bin, retry);
				ret = -1;
				continue;
			}
#if defined(CONFIG_SEC_FACTORY) && defined(CONFIG_SENSORS_A96T396_CRC_CHECK)
			grip_crc_check(data);
			if (data->crc_check == CRC_FAIL) {
				GRIP_INFO("CRC fail. retry:%d\n", retry);
				ret = -1;
				continue;
			}
#endif
		}

		grip_reset_for_bootmode(data);
		usleep_range(RESET_DELAY, RESET_DELAY + 1);

		ret = grip_get_fw_version(data, true, true);
		if (ret) {
			GRIP_ERR("read ver err\n");
			ret = -1;
			continue;
		}

		if (data->firmware.fw_ver == 0) {
			GRIP_ERR("ver err 0x%x\n", data->firmware.fw_ver);
			ret = -1;
			continue;
		}

		if ((cmd == BUILT_IN) && (data->firmware.fw_ver != data->firmware.fw_ver_bin)) {
			GRIP_ERR("ver miss match 0x%x, 0x%x\n",
						data->firmware.fw_ver, data->firmware.fw_ver_bin);
			ret = -1;
			continue;
		}
		ret = 0;
		break;
	}

	grip_release_fw(data, cmd);
	data->firmware.fw_update_flag = false;

	return ret;
}

MODULE_AUTHOR("Samsung Electronics");
MODULE_DESCRIPTION("Grip sensor driver for grip chip");
MODULE_LICENSE("GPL");
