/*
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/sec_class.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/proc_fs.h>
#include <linux/sec_debug.h>
#include <linux/seq_file.h>
#if IS_ENABLED(CONFIG_SEC_CRASHKEY_LONG)
#include <linux/samsung/debug/sec_crashkey_long.h>
#endif
#include <linux/mfd/sec_ap_pmic.h>
#include <trace/events/power.h>
#include <linux/suspend.h>

#define WS_LOG_PERIOD	5
#define MAX_WAKE_SOURCES_LEN	256

static struct device *sec_ap_pmic_dev;

static void wake_sources_print_acquired(void)
{
	char wake_sources_acquired[MAX_WAKE_SOURCES_LEN];

	pm_get_active_wakeup_sources(wake_sources_acquired, MAX_WAKE_SOURCES_LEN);
	pr_info("PM: %s\n", wake_sources_acquired);
}

static void wake_sources_print_acquired_work(struct work_struct *work)
{
	struct sec_ap_pmic_info *info = container_of(to_delayed_work(work),
			struct sec_ap_pmic_info, ws_work);

	wake_sources_print_acquired();
	schedule_delayed_work(&info->ws_work, info->ws_log_period * HZ);
}


#if IS_ENABLED(CONFIG_SEC_GPIO_DUMP)
static ssize_t gpio_dump_show(struct device *in_dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", (gpio_dump_enabled) ? 1 : 0);
}

static ssize_t gpio_dump_store(struct device *in_dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	int onoff;

	if (kstrtoint(buf, 10, &onoff) < 0)
		return -EINVAL;

	pr_info("%s: onoff=%d\n", __func__, onoff);
	gpio_dump_enabled = (onoff) ? true : false;

	return len;
}
static DEVICE_ATTR_RW(gpio_dump);
#endif

static struct attribute *sec_ap_pmic_attributes[] = {
#if IS_ENABLED(CONFIG_SEC_GPIO_DUMP)
	&dev_attr_gpio_dump.attr,
#endif
	NULL,
};

static struct attribute_group sec_ap_pmic_attr_group = {
	.attrs = sec_ap_pmic_attributes,
};

#if IS_ENABLED(CONFIG_SEC_GPIO_DUMP)
static void gpio_state_debug_suspend_trace_probe(void *unused,
					const char *action, int val, bool start)
{
	/* SUSPEND: start(1), val(1), action(machine_suspend) */
	if (gpio_dump_enabled && start && val > 0 && !strcmp("machine_suspend", action)) {
		sec_ap_gpio_debug_print();
		sec_pmic_gpio_debug_print();
	}
}
#endif

static int suspend_resume_pm_event(struct notifier_block *notifier,
		unsigned long pm_event, void *unused)
{
	struct sec_ap_pmic_info *info = container_of(notifier,
			struct sec_ap_pmic_info, sec_pm_debug_nb);

	switch (pm_event) {
	case PM_SUSPEND_PREPARE:
		cancel_delayed_work_sync(&info->ws_work);
		break;
	case PM_POST_SUSPEND:
		schedule_delayed_work(&info->ws_work, info->ws_log_period * HZ);
		break;
	default:
		break;
	}

	return NOTIFY_DONE;
}

static int sec_ap_pmic_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct sec_ap_pmic_info *info;
	int err;

	if (!node) {
		dev_err(&pdev->dev, "device-tree data is missing\n");
		return -ENXIO;
	}

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info) {
		dev_err(&pdev->dev, "%s: Fail to alloc info\n", __func__);
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, info);
	info->dev = &pdev->dev;

#if IS_ENABLED(CONFIG_SEC_CLASS)
	sec_ap_pmic_dev = sec_device_create(NULL, "ap_pmic");

	if (unlikely(IS_ERR(sec_ap_pmic_dev))) {
		pr_err("%s: Failed to create ap_pmic device\n", __func__);
		err = PTR_ERR(sec_ap_pmic_dev);
		goto err_device_create;
	}

	err = sysfs_create_group(&sec_ap_pmic_dev->kobj,
				&sec_ap_pmic_attr_group);
	if (err < 0) {
		pr_err("%s: Failed to create sysfs group\n", __func__);
		goto err_device_create;
	}
#endif

#if IS_ENABLED(CONFIG_SEC_GPIO_DUMP)
	/* Register callback for cheking subsystem stats */
	err = register_trace_suspend_resume(
		gpio_state_debug_suspend_trace_probe, NULL);
	if (err) {
		pr_err("%s: Failed to register suspend trace callback, ret=%d\n",
			__func__, err);
	}
#endif

	/* Set to default logging period (5s) */
	info->ws_log_period = WS_LOG_PERIOD;

	/* Register PM notifier */
	info->sec_pm_debug_nb.notifier_call = suspend_resume_pm_event;
	err = register_pm_notifier(&info->sec_pm_debug_nb);
	if (err) {
		dev_err(info->dev, "%s: failed to register PM notifier(%d)\n",
				__func__, err);
		return err;
	}

	INIT_DELAYED_WORK(&info->ws_work, wake_sources_print_acquired_work);
	schedule_delayed_work(&info->ws_work, info->ws_log_period * HZ);

	pr_info("%s: ap_pmic successfully inited.\n", __func__);

	return 0;

#if IS_ENABLED(CONFIG_SEC_CLASS)
err_device_create:
	sec_device_destroy(sec_ap_pmic_dev->devt);
	return err;
#endif
}

static int sec_ap_pmic_remove(struct platform_device *pdev)
{
#if IS_ENABLED(CONFIG_SEC_GPIO_DUMP)
	int ret;

	ret = unregister_trace_suspend_resume(
		gpio_state_debug_suspend_trace_probe, NULL);
#endif

#if IS_ENABLED(CONFIG_SEC_CLASS)
	if (sec_ap_pmic_dev) {
		sec_device_destroy(sec_ap_pmic_dev->devt);
	}
#endif

	return 0;
}

static const struct of_device_id sec_ap_pmic_match_table[] = {
	{ .compatible = "samsung,sec-ap-pmic" },
	{}
};

static struct platform_driver sec_ap_pmic_driver = {
	.driver = {
		.name = "samsung,sec-ap-pmic",
		.of_match_table = sec_ap_pmic_match_table,
	},
	.probe = sec_ap_pmic_probe,
	.remove = sec_ap_pmic_remove,
};

module_platform_driver(sec_ap_pmic_driver);

MODULE_DESCRIPTION("sec_ap_pmic driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jiman Cho <jiman85.cho@samsung.com");
