LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := fuzz-audio-hal
LOCAL_VENDOR_MODULE := true

LOCAL_SRC_FILES := \
    main.cpp

# { SEC_AUDIO_COMMON
SEC_AUDIO_VARS := vendor/samsung/variant/audio/sec_audioreach_vars.mk
include $(SEC_AUDIO_VARS)
# } SEC_AUDIO_COMMON

LOCAL_SHARED_LIBRARIES := \
    libbase \
    libbinder_ndk \
    libbinder \
    libcutils \
    libhardware \
    libutils \
    android.media.audio.common.types-V3-ndk \
    android.hardware.audio.core-V2-ndk \
    libclang_rt.ubsan_standalone

include $(BUILD_FUZZ_TEST)
