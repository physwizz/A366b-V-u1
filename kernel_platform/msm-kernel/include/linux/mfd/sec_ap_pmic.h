#ifndef __SEC_AP_PMIC_H__
#define __SEC_AP_PMIC_H__

#define SEC_PON_KEY_CNT	2

struct sec_ap_pmic_info {
	struct device		*dev;
	int chg_det_gpio;

	struct notifier_block sec_pm_debug_nb;
	struct delayed_work ws_work;
	unsigned int ws_log_period;
};

extern void msm_gpio_print_enabled(void);
extern void pmic_gpio_sec_dbg_enabled(void);

#if IS_ENABLED(CONFIG_SEC_GPIO_DUMP)
extern void sec_ap_gpio_debug_print(void);
extern void sec_pmic_gpio_debug_print(void);
static bool gpio_dump_enabled;
#endif

#endif /* __SEC_AP_PMIC_H__ */
