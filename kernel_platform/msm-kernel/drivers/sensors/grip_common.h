
#ifndef CONFIG_GRIP_COMMON_H
#define CONFIG_GRIP_COMMON_H


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


#define NUM_OF_MAX_CHANNEL 2
#define TUNINGMAP_MAX 126

#define SENSOR_ATTR_SIZE 55

#define HAS_ERROR			   -1
#define IDLE					2
#define ACTIVE				    1

#define TYPE_USB   1
#define TYPE_HALL  2
#define TYPE_BOOT  3
#define TYPE_FORCE 4
#define TYPE_COVER 5

#define SHCEDULE_INTERVAL       2
#define SUCCESS 0


#define CHECKSUM_MSB 0x7E
#define CHECKSUM_LSB 0x7F

#define REG_GRIP_TUNING_STATE 0xF1
#define REG_TUNING_CHECKSUM_MSB 0xF2
#define REG_TUNING_CHECKSUM_LSB 0xF3

#define CH1_BIT_MASK 0x01
#define CH2_BIT_MASK 0x02

struct logic_firmware {
	const struct firmware *firm_data_bin;
	const char *fw_path;
	const u8 *firm_data_ums;
	long firm_size;
	int firmup_cmd;
	int firmware_count;
	u8 fw_update_state;
	u8 fw_ver;
	u8 md_ver;
	u8 fw_ver_bin;
	u8 md_ver_bin;
	u8 checksum_h;
	u8 checksum_h_bin;
	u8 checksum_l;
	u8 checksum_l_bin;
	bool fw_update_flag;
};

struct parameter {
	int is_tuning_mode;
	int read_tuning_register_flag;
	bool setup_reg_exist;
	u8 setup_reg[TUNINGMAP_MAX * 2 + 1];
	u32 checksum_msb;
	u32 checksum_lsb;
};


enum grip_error_state {
	FAIL_UPDATE_PREV_STATE = 1,
	FAIL_SETUP_REGISTER,
	FAIL_I2C_ENABLE,
	FAIL_I2C_READ_3_TIMES,
	FAIL_DATA_STUCK,
	FAIL_RESET,
	FAIL_MCC_RESET,
	FAIL_IRQ_MISS_MATCH
};


struct channel {
	unsigned int grip_code;
	unsigned int unknown_code;
	unsigned int noti_code;

	int noti_enable;
	int is_unknown_mode;

	u16 grip_p_thd;
	u16 grip_r_thd;
	u16 grip_n_thd;
	u16 grip_baseline;
	u16 grip_raw;
	u16 grip_raw_d;
	u16 diff;
	u16 diff_d;
	u16 grip_event;

	s16 max_diff;
	s16 max_normal_diff;

	u8 num;
	u8 prev_state;
	u8 state_miss_matching_count;

	bool is_fail_safe_concept_enabled;
	bool is_working_state_on_error;
	bool first_working;
};



struct grip_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct input_dev *noti_input_dev;
	struct device *dev;
	struct mutex lock;
	struct mutex tuning_lock;
	struct work_struct irq_work;
	struct work_struct pdic_attach_reset_work;
	struct work_struct pdic_detach_reset_work;
	struct work_struct reset_work;
	struct delayed_work debug_work;
	struct delayed_work firmware_work;
	struct wakeup_source *grip_ws;
#if IS_ENABLED(CONFIG_HALL_NOTIFIER)
	struct notifier_block hall_nb;
#endif
#if IS_ENABLED(CONFIG_TABLET_MODEL_CONCEPT)
#if IS_ENABLED(CONFIG_KEYBOARD_STM32_POGO_V3) || IS_ENABLED(CONFIG_KEYBOARD_STM32_POGO_V2) || IS_ENABLED(CONFIG_KEYBOARD_STM32_POGO)
	struct notifier_block pogo_nb;
	struct delayed_work init_work;
#endif
#endif
	struct logic_firmware firmware;
	struct channel *ch[NUM_OF_MAX_CHANNEL];
	struct parameter param;
#if IS_ENABLED(CONFIG_CCIC_NOTIFIER) || IS_ENABLED(CONFIG_PDIC_NOTIFIER)
	struct notifier_block pdic_nb;
	int pdic_status;
	int pdic_pre_attach;
	int pre_attach;
	int pre_otg_attach;
#endif
	const char *dvdd_vreg_name;	/* regulator name */
	struct regulator *dvdd_vreg;	/* regulator */
	int (*power)(void *, bool on);	/* power onoff function ptr */
	int num_of_channels;
	int ldo_en;			/* ldo_en pin gpio */
	int grip_int;		/* irq pin gpio */
	int debug_count;
	int irq_en_cnt;
	int irq_count;
	int abnormal_mode;

	int motion;
	int retry_i2c;
	int irq;

	u32 err_state;
	u8 prev_buf;
	u8 ic_num;
	u8 i2c_fail_count;
	u8 prev_state;
	u8 state_miss_matching_count;
	u8 fail_safe_concept[4];
	u32 unknown_ch_selection;
	u32 set_up_channels;
	u8 read_reg_count;
	u8 read_reg;

	bool read_flag;
	bool is_power_down_on_error;
	bool is_first_event;
	bool crc_check;
	bool probe_done;
	bool enabled;
	bool skip_event;
	bool resume_called;//?
	bool current_state;
	bool expect_state;

	bool is_irq_active;
	bool check_abnormal_working;

	bool prevent_sleep_irq;
};

enum entry_path {
	BY_INTERRUPT_HANDLER = 0,
	BY_ENABLE_FUNC,
	BY_DEBUG_WORK
};

enum{
	CHANGE_REGISTER_MAP_FINISHED    = 0x00,
	CHANGE_TUNING_MAP_CMD           = 0x01,
	CHANGE_TUNING_MAP_FINISHED      = 0x02,
	CHANGE_REGISTER_MAP_CMD         = 0x03,
};

bool is_unavailable_channel(struct channel *ch);

void enter_error_mode(struct grip_data *data, enum grip_error_state err_state);

void enter_unknown_mode(struct grip_data *data, int type, u8 ch_num);

int grip_i2c_write(struct i2c_client *client, u8 reg, u8 *val);

int grip_i2c_write_retry(struct i2c_client *client, u8 reg, u8 *val, int retry);

int grip_i2c_read(struct i2c_client *client, u8 reg, u8 *val, unsigned int len);

int grip_i2c_read_retry(struct i2c_client *client, u8 reg, u8 *val, unsigned int len, int retry);

int grip_i2c_read_data(struct i2c_client *client, u8 *val, unsigned int len);

int grip_i2c_read_checksum(struct grip_data *data);

void grip_diff_getdata(struct grip_data *data, bool log, u8 ch_num);

void grip_check_diff_and_cap(struct grip_data *data);

void get_threshold(struct grip_data *data, int ch_num);

void grip_check_irq_error(struct grip_data *data, u8 irq_state, enum entry_path path, u8 ch_num);

void check_irq_status(struct grip_data *data, enum entry_path path);

void grip_set_enable(struct grip_data *data, int enable);

void grip_sw_reset(struct grip_data *data);

int grip_load_fw_kernel(struct grip_data *data);

void grip_always_active(struct grip_data *data, int on);

int grip_get_fw_version(struct grip_data *data, bool bootmode, bool activemode);

void grip_reset_for_bootmode(struct grip_data *data);

int grip_fw_mode_enter(struct grip_data *data);

int grip_flash_erase(struct grip_data *data);

int grip_check_busy(struct grip_data *data);

int grip_fw_write(struct grip_data *data, unsigned char *addrH, unsigned char *addrL, unsigned char *val);

int grip_fw_mode_exit(struct grip_data *data);

int grip_fw_update(struct grip_data *data, u8 cmd);

void grip_release_fw(struct grip_data *data, u8 cmd);

int grip_flash_fw(struct grip_data *data, bool probe, u8 cmd);

#ifdef CONFIG_SENSORS_A96T396_CRC_CHECK
int grip_crc_check(struct grip_data *data);
#endif

#endif
