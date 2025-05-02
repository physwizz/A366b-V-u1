LOCAL_PATH:= $(call my-dir)

# ---------------------------------------------------------------------------------
#             Populate ACDB data files to file system for audconf
# ---------------------------------------------------------------------------------

include $(CLEAR_VARS)

$(shell mkdir -p $(TARGET_OUT_VENDOR_ETC)/audconf)
$(shell mkdir -p $(TARGET_OUT_VENDOR_ETC)/audconf/OPEN)
$(shell cp -rf $(wildcard $(LOCAL_PATH)/audconf/$(PROJECT_NAME)/*.acdb) $(TARGET_OUT_VENDOR_ETC)/audconf/OPEN)
$(shell cp -rf $(wildcard $(LOCAL_PATH)/audconf/$(PROJECT_NAME)/*.qwsp) $(TARGET_OUT_VENDOR_ETC)/audconf/OPEN)

ifeq ($(shell if [ -d $(TARGET_OUT_VENDOR)/firmware ]; then echo true; fi), true)
    $(info "$(TARGET_OUT_VENDOR)/firmware exists")
else
    $(shell mkdir -p $(TARGET_OUT_VENDOR)/firmware)
endif

$(shell cp -rf $(wildcard $(LOCAL_PATH)/audconf/$(PROJECT_NAME)/*.cnt) $(TARGET_OUT_VENDOR)/firmware)

# ---------------------------------------------------------------------------------
#                     END
# ---------------------------------------------------------------------------------

