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

#ifndef LOCATIOCLIENTAPI_SERVICE_H
#define LOCATIOCLIENTAPI_SERVICE_H

#include <CommonAPI/CommonAPI.hpp>
#include "LocIdlAPIStubImpl.hpp"
#include <v1/com/qualcomm/qti/location/LocationStub.hpp>
#include "LocationClientApi.h"
#include "MsgTask.h"
#include "log_util.h"
#include "LocLcaIdlConverter.h"
#include "LocationIntegrationApi.h"
#include "LocIdlServiceLog.h"

#ifdef POWER_DAEMON_MGR_ENABLED
#include "LocIdlPowerEvtManager.h"
#endif

using namespace location_client;
using namespace v1::com::qualcomm::qti::location;
using namespace std;
using namespace loc_util;
using namespace location_integration;

#define IDL_MEMORY_CHECK_INTERVAL_SEC (2)

// Enum to define supported Power states in power-daemon
enum IDLPowerStateType {
    IDL_POWER_STATE_UNKNOWN = 0,
    IDL_POWER_STATE_SUSPEND = 1,
    IDL_POWER_STATE_RESUME  = 2,
    IDL_POWER_STATE_SHUTDOWN = 3,
    IDL_POWER_STATE_DEEP_SLEEP_ENTRY = 4,
    IDL_POWER_STATE_DEEP_SLEEP_EXIT = 5
};

#ifdef POWER_DAEMON_MGR_ENABLED
class LocIdlPowerEvtHandler;
#endif
class LocIdlAPIStubImpl;
class LocIdlServiceLog;

class LocIdlAPIService {
public:
    std::shared_ptr<LocIdlAPIStubImpl> mService;
    // singleton instance
    static LocIdlAPIService* getInstance();
    bool init();
    bool createLocIdlService();
    bool registerWithFIDLService();
    bool unRegisterWithFIDLService();
    bool processCapabilities(::LocationCapabilitiesMask mask);

    /* Process Fused/Detailed Position request */
    void startPositionSession
    (
        const std::shared_ptr<CommonAPI::ClientId> client,
        uint32_t intervalInMs, uint32_t gnssReportCallbackMask,
        LocationStub::StartPositionSessionLocationReportReply_t reply
    ) const;

    /* Process Engine specific Position request */
    void startPositionSession
    (
        const std::shared_ptr<CommonAPI::ClientId> client,
        uint32_t intervalInMs, uint32_t locReqEngMask, uint32_t engReportCallbackMask,
        LocationStub::StartPositionSessionEngineSpecificLocationReply_t reply
    ) const;
    void stopPositionSession
    (
        const std::shared_ptr<CommonAPI::ClientId> client,
        LocationStub::StopPositionSessionReply_t reply
    ) const;

    void deleteAidingDataRequest
    (
        const std::shared_ptr<CommonAPI::ClientId> client,
        uint32_t deleteMask, LocationStub::DeleteAidingDataReply_t reply
    ) const;

    /// This is the method that will be called on remote calls on the method configConstellations.
    void configConstellationsRequest
    (
        const std::shared_ptr<CommonAPI::ClientId> client,
        std::vector<LocationTypes::GnssSvIdInfoT > svList,
        LocationStub::ConfigConstellationsReply_t reply
    ) const;

    /** Utility functions to support Location reports */
    LocationTypes::LocationReportT parseLocationReport
    (
        const ::GnssLocation &lcaLoc
    ) const;

    LocationTypes::GnssSvDataT parseGnssSvReport
    (
        const location_client::GnssSv& gnssSvs
    ) const;

    LocationTypes::GnssMeasurementsT parseGnssMeasurements
    (
        const location_client::GnssMeasurements& gnssMeasurements
    ) const;

    LocationTypes::GnssDataT parseGnssDataReport
    (
        const location_client::GnssData& gnssData
    ) const;

    LocationTypes::LocationStatusT parseIDLResponse
    (
        const location_client::LocationResponse lcaResponse
    ) const;

    void onPowerEvent(IDLPowerStateType powerEvent);
    void injectMapMatchedFeedbackData
    (
        const std::shared_ptr<CommonAPI::ClientId> client,
        LocationTypes::MapMatchingFeedbackDataT mapData
    ) const;

    uint16_t parseDeleteAidingDataMask
    (
        uint32_t deleteMask
    ) const;

    /** To start a new thread to monito memory
     *  usahe every 2second  by LocIDlService  */
    void monitorMemoryUsage();
    /** To pass the system health status to Diag structure */
    void updateSystemStatus(uint32_t totalRss);

    inline uint32_t getLocationCapabilitiesMask() const {
        return mGnssCapabilites;
    }

    void checkMinIntervalForUpdate(uint32_t clientRequestedTbf) const;

#ifdef POWER_DAEMON_MGR_ENABLED
    LocIdlPowerEvtHandler* mPowerEventObserver;
#endif
    LocIdlServiceLog* mDiagLogIface;
    LocLcaIdlConverter* mLcaIdlConverter;
    bool mIsGptpInitialized;

    /**Keeps track of latest caps recieved */
    uint32_t mGnssCapabilites;

private:
    static LocIdlAPIService *mInstance;
    LocationClientApi* mLcaInstance;
    MsgTask* mMsgTask;
    MsgTask* mMemoryMonitorMsgTask;
    LocationIntegrationApi* mLIAInstance;
    /** To store superset mask for all IDL service clients */
    mutable uint32_t mGnssReportMask;
    /** Keep cont of number of start session requests, this variable
     *  shall be incremented on each startSession and decremented on
     *  every stop Session, Stop session request shall be sent to LCA
     *  only if this variable is 0 */
    mutable uint32_t numControlRequests;

    /** To configure minInterval for all IDL clients. IDL service will
     *  operate at minimum tbf value */
    mutable uint32_t mTimeBetweenFixes;

    /** Keeps track if service is registered or not
     *  True: Service registered Successfully
     *  False: Service not registered */
    mutable bool serviceRegisterationStatus;

    LocIdlAPIService();
    ~LocIdlAPIService();

};


#endif //LOCATIOCLIENTAPI_SERVICE_H
