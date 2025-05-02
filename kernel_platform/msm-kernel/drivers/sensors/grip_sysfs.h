#ifndef CONFIG_GRIP_SYSFS_H
#define CONFIG_GRIP_SYSFS_H

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


#ifdef CONFIG_SENSORS_A96T396_CH1
int sizeof_grip_ch1_attr(void);
#endif

#ifdef CONFIG_SENSORS_A96T396_CH2
int sizeof_grip_ch2_attr(void);
#endif

int sizeof_grip_sensor_attr(void);
int get_channel_attr_size(int ch_idx);

extern struct device_attribute *grip_ch1_sensor_attrs[];
extern struct device_attribute *grip_ch2_sensor_attrs[];
extern struct device_attribute *grip_common_sensor_attrs[];
extern struct attribute_group grip_attribute_group;

#endif