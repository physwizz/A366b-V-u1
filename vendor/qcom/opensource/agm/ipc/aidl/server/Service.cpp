/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define LOG_TAG "AgmIpc::Service"

#include <android/binder_manager.h>
#include <android/binder_process.h>
#include <log/log.h>

#include "AgmServerWrapper.h"

using namespace aidl::vendor::qti::hardware::agm;

extern "C" __attribute__((visibility("default"))) binder_status_t registerService() {
    ALOGI("register AGM Service");
    auto agmService = ::ndk::SharedRefBase::make<AgmServerWrapper>();
    ndk::SpAIBinder agmBinder = agmService->asBinder();
    const std::string interfaceName = std::string() + IAGM::descriptor + "/default";

    binder_status_t status = AServiceManager_addService(agmBinder.get(), interfaceName.c_str());
    ALOGI("register AGM Service interface %s registered %s ", interfaceName.c_str(),
          (status == STATUS_OK) ? "yes" : "no");
    return status;
}
