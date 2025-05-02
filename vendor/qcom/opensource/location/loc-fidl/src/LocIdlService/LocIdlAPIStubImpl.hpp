/*
Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the
disclaimer below) provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.

    * Neither the name of Qualcomm Innovation Center, Inc. nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
GRANTED BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef LOCIDLAPISTUBIMPL_HPP
#define LOCIDLAPISTUBIMPL_HPP

#include <iostream>
#include <thread>
#include <stdio.h>
#include <CommonAPI/CommonAPI.hpp>
#include <v1/com/qualcomm/qti/location/Location.hpp>
#include <v1/com/qualcomm/qti/location/LocationStub.hpp>
#include <v1/com/qualcomm/qti/location/LocationTypes.hpp>
#include "LocIdlAPIService.h"

using namespace std;
using namespace v1::com::qualcomm::qti::location;

class LocIdlAPIService;
class LocIdlAPIStubImpl: public v1::com::qualcomm::qti::location::LocationStub {

public:

    virtual const CommonAPI::Version& getInterfaceVersion(
            std::shared_ptr<CommonAPI::ClientId> client)
    {
        return CommonAPI::Version(0, 1);
    }

    /// This is the method that will be called on remote calls on the method StartPositionSessionLocationReport.
    virtual void StartPositionSessionLocationReport(
            const std::shared_ptr<CommonAPI::ClientId> client,
            uint32_t intervalInMs, uint32_t gnssReportCallbackMask,
            StartPositionSessionLocationReportReply_t reply);

    /// This is the method that will be called on remote calls on the method StartPositionSessionEngineSpecificLocation.
    virtual void StartPositionSessionEngineSpecificLocation(
            const std::shared_ptr<CommonAPI::ClientId> client,
            uint32_t intervalInMs, uint32_t locReqEngMask, uint32_t engReportCallbackMask,
            StartPositionSessionEngineSpecificLocationReply_t reply);

    /// This is the method that will be called on remote calls on the method StopPositionSession.
    virtual void StopPositionSession(const std::shared_ptr<CommonAPI::ClientId> client,
            StopPositionSessionReply_t reply);

    /// This is the method that will be called on remote calls on the method DeleteAidingData.
    virtual void DeleteAidingData(const std::shared_ptr<CommonAPI::ClientId> client,
            uint32_t deleteMask, DeleteAidingDataReply_t reply);

    /// This is the method that will be called on remote calls on the method ConfigConstellations.
    virtual void ConfigConstellations(const std::shared_ptr<CommonAPI::ClientId> client,
            std::vector<LocationTypes::GnssSvIdInfoT > svList, ConfigConstellationsReply_t reply);

    /// This is the method that will be called on remote calls on the method InjectMapMatchedFeedbackData.
    virtual void InjectMapMatchedFeedbackData(const std::shared_ptr<CommonAPI::ClientId> client,
            LocationTypes::MapMatchingFeedbackDataT mmfData);

    virtual void GetLocationCapabilities(const std::shared_ptr<CommonAPI::ClientId> client,
            GetLocationCapabilitiesReply_t reply);

    const LocIdlAPIService* mApiService;
    LocIdlAPIStubImpl(const LocIdlAPIService* apiService);
    ~LocIdlAPIStubImpl();
};

#endif // LOCIDLAPISTUBIMPL_HPP
