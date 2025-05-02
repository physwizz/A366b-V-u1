obj-$(CONFIG_SENSORS)		+= sensors_core.o


ccflags-y += -Wall -Werror -Wno-missing-braces
ifneq (,$(filter m,$(CONFIG_SENSORS_A96T396) $(CONFIG_SENSORS_A96T396_SUB) $(CONFIG_SENSORS_A96T396_SUB2) $(CONFIG_SENSORS_A96T396_WIFI)))
	obj-m += gripsensor.o
	gripsensor-objs += a96t396.o grip_common.o grip_sysfs.o
else ifneq (,$(filter y,$(CONFIG_SENSORS_A96T396) $(CONFIG_SENSORS_A96T396_SUB) $(CONFIG_SENSORS_A96T396_SUB2) $(CONFIG_SENSORS_A96T396_WIFI)))
	obj-y += a96t396.o
endif
