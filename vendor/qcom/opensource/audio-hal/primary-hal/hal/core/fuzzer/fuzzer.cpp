/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#define LOG_TAG "AIDL_FUZZER_AUDIO_CORE_HAL"

#include <dlfcn.h>
#include <android-base/logging.h>

#include <aidl/android/hardware/audio/core/IModule.h>
#include <aidl/android/hardware/audio/core/IConfig.h>


#include <fuzzbinder/libbinder_ndk_driver.h>
#include <fuzzer/FuzzedDataProvider.h>

static ::aidl::android::hardware::audio::core::IConfig* gConfigDefaultAosp;
static ::aidl::android::hardware::audio::core::IModule* gModuleDefaultQti;

static void* loadAndGetInstance(const std::pair<std::string, std::string>& librarySymbolPair) {
    const auto& libraryName = librarySymbolPair.first;
    const auto& functionName = librarySymbolPair.second;
    void* handle = dlopen(libraryName.c_str(), RTLD_LAZY);
    if (!handle) {
        LOG(ERROR) << "Cannot open library: " << libraryName << dlerror();
        return nullptr;
    }

    using GetInstance = void* (*)();
    GetInstance getInstance = reinterpret_cast<GetInstance>(dlsym(handle, functionName.c_str()));

    if (getInstance == nullptr) {
        LOG(ERROR) << "Cannot load symbol " << functionName << dlerror();
        dlclose(handle);  // Close the library if dlsym fails
        return nullptr;
    }

    return getInstance();
}

// init
extern "C" int LLVMFuzzerInitialize(int* argc, char*** argv) {
    gConfigDefaultAosp = static_cast<::aidl::android::hardware::audio::core::IConfig*>(
            loadAndGetInstance(std::make_pair("libaudiocorehal.qti.so", "getIModuleDefaultQti")));
    gModuleDefaultQti =
            static_cast<::aidl::android::hardware::audio::core::IModule*>(loadAndGetInstance(
                    std::make_pair("libaudiocorehal.default.so", "getIConfigDefaultAosp")));

    if (gConfigDefaultAosp == nullptr || gModuleDefaultQti == nullptr) {
        return -1;
    }

    return 0;
}

// one fuzzing test case
extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size) {
    FuzzedDataProvider provider(data, size);

    uint32_t index = provider.ConsumeIntegralInRange<uint32_t>(1, 2);

    if (index == 2 && gModuleDefaultQti != nullptr) {
        android::fuzzService(gModuleDefaultQti->asBinder().get(), std::move(provider));
    }

    return 0;
}