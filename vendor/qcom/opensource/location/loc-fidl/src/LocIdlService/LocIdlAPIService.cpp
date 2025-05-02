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

#define LOG_TAG "LOC_IDL_SERVICE"
#include <iostream>
#include <thread>
#include <stdio.h>
#include <functional>
#include <stdint.h>
#include <unistd.h>
#include <inttypes.h>
#include <string.h>

#include <CommonAPI/CommonAPI.hpp>
#include "LocIdlAPIStubImpl.hpp"
#include "LocIdlAPIService.h"
#include "gptp_helper.h"

using namespace std;
using namespace v1::com::qualcomm::qti::location;
using namespace location_client;

static bool capabilitiesReceived = false;
static uint64_t posCount = 0;
static uint64_t latentPosCount = 0;
static bool exitFromMemUsageMsgTask = false;
/** Latency threshold for Position reports in msec */
#define MAX_POSITION_LATENCY   20
#define IDL_MAX_RETRY 5
#define MIN_POS_REPORT_INTERVAL_MSEC (100)

static void onConfigResponseCb(location_integration::LocConfigTypeEnum      requestType,
                               location_integration::LocIntegrationResponse response) {
     LOC_LOGd("<<< onConfigResponseCb, type %d, err %d\n", requestType, response);
}

class LocationTrackingSessCbHandler {
    public:
        LocationTrackingSessCbHandler(const LocIdlAPIService *pClientApiService,
                const uint32_t &reportCbMask) {
            if (NULL != pClientApiService) {
                LOC_LOGd(" ==== LocationTrackingSessCbHandler --> ");
                memset(&mCallbackOptions, 0, sizeof(GnssReportCbs));
                if (reportCbMask &
                    LocationTypes::GnssReportCbInfoMaskT::GRCIMT_LOCATION_CB_INFO_BIT) {
                    mCallbackOptions.gnssLocationCallback =
                            [pClientApiService] (const ::GnssLocation n) {
                        //Convert Location report from LCA to FIDL format
                        if (pClientApiService->mLcaIdlConverter) {
                            LocationTypes::LocationReportT idlLocRpt =
                                    pClientApiService->parseLocationReport(n);
                            posCount++;
                            struct timespec curBootTime = {};
                            clock_gettime(CLOCK_BOOTTIME, &curBootTime);
                            int64_t curBootTimeNs = ((int64_t)curBootTime.tv_sec * 1000000000) +
                                    (int64_t)curBootTime.tv_nsec;
                            int16_t latencyMs = 0;
                            if (LOCATION_HAS_ELAPSED_REAL_TIME_BIT  & n.flags) {
                                latencyMs = (int16_t)((curBootTimeNs -
                                        n.elapsedRealTimeNs)/1000000);
                            }
                            if (latencyMs > MAX_POSITION_LATENCY) {
                                  latentPosCount++;
                            }
                            if (posCount % 600 == 0) {
                                LOC_LOGe("%"PRId64" out of %"PRId64" Position samples are"
                                         " latent by 20 msec",
                                        latentPosCount, posCount);
                            }
                            idlLocRpt.setReportingLatency(latencyMs);
                            if (pClientApiService->mDiagLogIface) {
                                pClientApiService->mDiagLogIface->diagLogGnssReportInfo(
                                    OUTPUT_PVT_REPORT, latencyMs, latentPosCount);
                            }
                            if (pClientApiService->mService) {
                                pClientApiService->mService->fireGnssLocationReportEvent(idlLocRpt);
                            }
                        }
                    };
                }
                if (reportCbMask &
                        LocationTypes::GnssReportCbInfoMaskT::GRCIMT_SV_CB_INFO_BIT) {
                    mCallbackOptions.gnssSvCallback =
                            [pClientApiService](const std::vector<::GnssSv>& gnssSvs) {
                        std::vector<LocationTypes::GnssSvDataT> idlSVReportVector;
                        LOC_LOGd("Number of SV's recevived -- %d", gnssSvs.size());
                        if (pClientApiService->mLcaIdlConverter) {
                            for (auto GnssSv : gnssSvs) {
                                //Convert Location report from LCA to FIDL format
                                LocationTypes::GnssSvDataT idlSVRpt =
                                    pClientApiService->parseGnssSvReport(GnssSv);
                                idlSVReportVector.push_back(idlSVRpt);
                            }
                            if (pClientApiService->mService) {
                                pClientApiService->mService->fireGnssSvReportEvent(
                                        idlSVReportVector);
                            }
                        }
                    };
                }
                if (reportCbMask &
                        LocationTypes::GnssReportCbInfoMaskT::GRCIMT_NMEA_CB_INFO_BIT) {
                    mCallbackOptions.nmeaSentencesCallback =
                            [pClientApiService](::LocOutputEngineType engType,
                                    uint64_t timestamp, const std::string nmea) {
                        if (pClientApiService->mService) {
                            pClientApiService->mService->fireGnssNmeaEvent(timestamp, nmea);
                        }
                    };
                }
                if (reportCbMask &
                        LocationTypes::GnssReportCbInfoMaskT::GRCIMT_MEAS_CB_INFO_BIT) {
                    mCallbackOptions.gnssMeasurementsCallback =
                            [pClientApiService](const ::GnssMeasurements gnssMeasurements) {
                        if (pClientApiService->mLcaIdlConverter) {
                            LocationTypes::GnssMeasurementsT idlGnssMeasurement =
                                    pClientApiService->parseGnssMeasurements(gnssMeasurements);
                            struct timespec curBootTime = {};
                            clock_gettime(CLOCK_BOOTTIME, &curBootTime);
                            int64_t curBootTimeNs = ((int64_t)curBootTime.tv_sec * 1000000000) +
                                    (int64_t)curBootTime.tv_nsec;
                            int16_t latencyMs = 0;
                            latencyMs = (int16_t)((curBootTimeNs -
                                    gnssMeasurements.clock.elapsedRealTime)/1000000);
                            idlGnssMeasurement.setReportingLatency(latencyMs);
                            if (pClientApiService->mDiagLogIface) {
                                pClientApiService->mDiagLogIface->diagLogGnssReportInfo(
                                        OUTPUT_MEAS_REPORT, latencyMs, 0);
                            }
                            if (pClientApiService->mService) {
                                pClientApiService->mService->fireGnssMeasurementReportEvent(
                                        idlGnssMeasurement);
                            }
                        }
                    };
                }
                 // GnssDataCb
                 if (reportCbMask &
                 LocationTypes::GnssReportCbInfoMaskT::GRCIMT_DATA_CB_INFO_BIT) {
                    mCallbackOptions.gnssDataCallback =
                            [pClientApiService] (const ::GnssData n) {
                        //Convert GnssData report from LCA to FIDL format
                        if (pClientApiService->mLcaIdlConverter) {
                            LocationTypes::GnssDataT idlGnssDataRpt =
                                    pClientApiService->parseGnssDataReport(n);
                            if (pClientApiService->mService) {
                                pClientApiService->mService->fireGnssDataReportEvent(idlGnssDataRpt);
                            }
                        }
                    };
                }
            }
        }

        LocationTrackingSessCbHandler(const LocIdlAPIService *pClientApiService,
                const uint32_t &locReqEngMask, const uint32_t &engReportCallbackMask) {
            if (NULL != pClientApiService) {
                memset(&mEngineCallbackOptions, 0, sizeof(EngineReportCbs));

                if (engReportCallbackMask &
                    LocationTypes::EngineReportCbMaskT::ERCMT_LOCATION_CB_INFO_BIT) {
                    mEngineCallbackOptions.engLocationsCallback =
                            [pClientApiService] (const std::vector<::GnssLocation> &engLocations) {
                        if (pClientApiService->mLcaIdlConverter) {
                            std::vector<LocationTypes::LocationReportT> idlEngLocVector;
                            for (auto gnssLocation : engLocations) {
                                LocationTypes::LocationReportT idlLocRpt =
                                        pClientApiService->parseLocationReport(gnssLocation);
                                struct timespec curBootTime = {};
                                clock_gettime(CLOCK_BOOTTIME, &curBootTime);
                                int64_t curBootTimeNs = (
                                        (int64_t)curBootTime.tv_sec * 1000000000) +
                                        (int64_t)curBootTime.tv_nsec;
                                int16_t latencyMs = 0;
                                if (LOCATION_HAS_ELAPSED_REAL_TIME_BIT  & gnssLocation.flags) {
                                    latencyMs = (int16_t)((curBootTimeNs -
                                            gnssLocation.elapsedRealTimeNs)/1000000);
                                }
                                idlLocRpt.setReportingLatency(latencyMs);
                                if (pClientApiService->mDiagLogIface) {
                                    pClientApiService->mDiagLogIface->diagLogGnssReportInfo(
                                            OUTPUT_PVT_REPORT, latencyMs, latentPosCount);
                                }
                                idlEngLocVector.push_back(idlLocRpt);
                            }
                            if (pClientApiService->mService) {
                                pClientApiService->mService->fireGnssEngineLocationsReportEvent(
                                        idlEngLocVector);
                            }
                        }
                    };
                }
            if (engReportCallbackMask &
                        LocationTypes::EngineReportCbMaskT::ERCMT_SV_CB_INFO_BIT) {
                    mEngineCallbackOptions.gnssSvCallback =
                            [pClientApiService](const std::vector<::GnssSv>& gnssSvs) {
                        std::vector<LocationTypes::GnssSvDataT> idlSVReportVector;
                        LOC_LOGd("Number of SV's recevived -- %d", gnssSvs.size());
                        if (pClientApiService->mLcaIdlConverter) {
                            for (auto GnssSv : gnssSvs) {
                                //Convert Location report from LCA to FIDL format
                                LocationTypes::GnssSvDataT idlSVRpt =
                                    pClientApiService->parseGnssSvReport(GnssSv);
                                idlSVReportVector.push_back(idlSVRpt);
                            }
                            if (pClientApiService->mService) {
                                pClientApiService->mService->fireGnssSvReportEvent(
                                        idlSVReportVector);
                            }
                        }
                    };
                }
                if (engReportCallbackMask &
                        LocationTypes::EngineReportCbMaskT::ERCMT_NMEA_CB_INFO_BIT) {
                    mEngineCallbackOptions.nmeaSentencesCallback =
                            [pClientApiService](::LocOutputEngineType engType,
                                    uint64_t timestamp, const std::string nmea) {
                        if (pClientApiService->mService) {
                            pClientApiService->mService->fireGnssNmeaEvent(timestamp, nmea);
                        }
                    };
                }
                if (engReportCallbackMask &
                        LocationTypes::EngineReportCbMaskT::ERCMT_ENGINE_NMEA_CB_INFO_BIT) {
                    mEngineCallbackOptions.nmeaSentencesCallback =
                            [pClientApiService](::LocOutputEngineType engType,
                                    uint64_t timestamp, const std::string nmea) {
                        LocationTypes::LocOutputEngineTypeT idlEngType =
                                LocationTypes::LocOutputEngineTypeT::LOETT_COUNT;
                        switch (engType)  {
                            case LOC_OUTPUT_ENGINE_FUSED:
                                idlEngType =
                                    LocationTypes::LocOutputEngineTypeT::LOETT_FUSED;
                                break;
                            case LOC_OUTPUT_ENGINE_SPE:
                                idlEngType = LocationTypes::LocOutputEngineTypeT::LOETT_SPE;
                                break;
                            case LOC_OUTPUT_ENGINE_PPE:
                                idlEngType = LocationTypes::LocOutputEngineTypeT::LOETT_PPE;
                                break;
                            case LOC_OUTPUT_ENGINE_VPE:
                                idlEngType = LocationTypes::LocOutputEngineTypeT::LOETT_VPE;
                                break;
                            default:
                                idlEngType =
                                    LocationTypes::LocOutputEngineTypeT::LOETT_COUNT;
                            break;
                        }
                        if (pClientApiService->mService) {
                            pClientApiService->mService->fireEngineNmeaEvent(
                                    idlEngType, timestamp, nmea);
                        }
                    };
                }
                if (engReportCallbackMask &
                        LocationTypes::EngineReportCbMaskT::ERCMT_MEAS_CB_INFO_BIT) {
                    mEngineCallbackOptions.gnssMeasurementsCallback =
                            [pClientApiService](const ::GnssMeasurements gnssMeasurements) {
                        if (pClientApiService->mLcaIdlConverter) {
                            LocationTypes::GnssMeasurementsT idlGnssMeasurement =
                                    pClientApiService->parseGnssMeasurements(gnssMeasurements);
                            struct timespec curBootTime = {};
                            clock_gettime(CLOCK_BOOTTIME, &curBootTime);
                            int64_t curBootTimeNs = ((int64_t)curBootTime.tv_sec * 1000000000) +
                                    (int64_t)curBootTime.tv_nsec;
                            int16_t latencyMs = 0;
                            latencyMs = (int16_t)((curBootTimeNs -
                                    gnssMeasurements.clock.elapsedRealTime)/1000000);
                            idlGnssMeasurement.setReportingLatency(latencyMs);
                            if (pClientApiService->mDiagLogIface) {
                                pClientApiService->mDiagLogIface->diagLogGnssReportInfo(
                                        OUTPUT_MEAS_REPORT, latencyMs, 0);
                            }
                            if (pClientApiService->mService) {
                                pClientApiService->mService->fireGnssMeasurementReportEvent(
                                        idlGnssMeasurement);
                            }
                        }
                    };
                }
                 // GnssDataCb
                 if (engReportCallbackMask &
                        LocationTypes::EngineReportCbMaskT::ERCMT_DATA_CB_INFO_BIT) {
                    mEngineCallbackOptions.gnssDataCallback =
                            [pClientApiService] (const ::GnssData n) {
                        //Convert GnssData report from LCA to FIDL format
                        if (pClientApiService->mLcaIdlConverter) {
                            LocationTypes::GnssDataT idlGnssDataRpt =
                                    pClientApiService->parseGnssDataReport(n);
                            if (pClientApiService->mService) {
                                pClientApiService->mService->fireGnssDataReportEvent(
                                        idlGnssDataRpt);
                            }
                        }
                    };
                }

                //Fill LocReqEngMask
                memset(&mLcaLocReqEngMask, 0, sizeof(location_client::LocReqEngineTypeMask));
                uint32_t lcaLocReqEngMask = 0;
                if (locReqEngMask &
                        LocationTypes::LocReqEngineTypeMaskT::LRETM_FUSED) {
                    lcaLocReqEngMask |= location_client::LOC_REQ_ENGINE_FUSED_BIT;
                }
                if (locReqEngMask & LocationTypes::LocReqEngineTypeMaskT::LRETM_SPE) {
                    lcaLocReqEngMask |= location_client::LOC_REQ_ENGINE_SPE_BIT;
                }
                if (locReqEngMask & LocationTypes::LocReqEngineTypeMaskT::LRETM_PPE) {
                    lcaLocReqEngMask |= location_client::LOC_REQ_ENGINE_PPE_BIT;
                }
                if (locReqEngMask & LocationTypes::LocReqEngineTypeMaskT::LRETM_VPE) {
                    lcaLocReqEngMask |= location_client::LOC_REQ_ENGINE_VPE_BIT;
                }

                mLcaLocReqEngMask = (location_client::LocReqEngineTypeMask)lcaLocReqEngMask;
            }
        }

        GnssReportCbs& getLocationCbs() { return mCallbackOptions; }
        EngineReportCbs& getEngineLocationCbs() { return mEngineCallbackOptions; }
        location_client::LocReqEngineTypeMask& getLcaLocReqEngMask() { return mLcaLocReqEngMask; }

    private:
        GnssReportCbs mCallbackOptions;
        EngineReportCbs mEngineCallbackOptions;
        location_client::LocReqEngineTypeMask mLcaLocReqEngMask;

};

LocationTypes::LocationReportT LocIdlAPIService::parseLocationReport
(
    const location_client::GnssLocation &lcaLoc

) const {
    return (mLcaIdlConverter->parseLocReport(lcaLoc));
}

LocationTypes::GnssSvDataT LocIdlAPIService::parseGnssSvReport
(
    const location_client::GnssSv& gnssSvs
) const {
    return (mLcaIdlConverter->parseSvReport(gnssSvs));
}

LocationTypes::GnssMeasurementsT LocIdlAPIService::parseGnssMeasurements
(
    const location_client::GnssMeasurements& gnssMeasurements
) const {
    return (mLcaIdlConverter->parseMeasurements(gnssMeasurements));
}

LocationTypes::GnssDataT LocIdlAPIService::parseGnssDataReport
(
    const location_client::GnssData& gnssData
) const {
    return (mLcaIdlConverter->parseGnssData(gnssData));
}

LocIdlAPIService* LocIdlAPIService::mInstance = nullptr;
LocIdlAPIService* LocIdlAPIService::getInstance()
{
    if (nullptr == mInstance) {
        mInstance = new LocIdlAPIService();
    }
    return mInstance;
}

LocIdlAPIService::LocIdlAPIService():
        mLcaInstance(nullptr),
        mMsgTask(new MsgTask("LocIDLService")),
        mLIAInstance(nullptr),
#ifdef POWER_DAEMON_MGR_ENABLED
        mPowerEventObserver(nullptr),
#endif
        mLcaIdlConverter(new LocLcaIdlConverter()),
        mDiagLogIface(new LocIdlServiceLog()),
        mGnssReportMask(0),
        numControlRequests(0),
        mMemoryMonitorMsgTask(new MsgTask("LocIDLServiceMem")),
        serviceRegisterationStatus(false),
        mIsGptpInitialized(false),
        mGnssCapabilites(0),
        mTimeBetweenFixes(0)
{
    if (mDiagLogIface) {
        mDiagLogIface->initializeDiagIface();
    }
}

LocIdlAPIService::~LocIdlAPIService()
{
    exitFromMemUsageMsgTask = true;
    sleep(IDL_MEMORY_CHECK_INTERVAL_SEC);
}

void LocIdlAPIService::onPowerEvent(IDLPowerStateType powerEvent) {
    LOC_LOGi("Recieved Power Event %d", powerEvent);
    struct PowerEventMsg : public LocMsg {

        LocIdlAPIService* mIDLService;
        IDLPowerStateType mPowerEvent;
        inline PowerEventMsg(LocIdlAPIService* IDLService,
                IDLPowerStateType event) :
            LocMsg(),
            mIDLService(IDLService),
            mPowerEvent(event){};
        inline virtual void proc() const {
            bool retVal = false;
            uint8_t serviceStatus = SERVICE_STAUS_UNKNOWN;
            if (mIDLService) {
                switch (mPowerEvent) {
                    case POWER_STATE_SUSPEND:
                    case POWER_STATE_SHUTDOWN:
                        retVal = mIDLService->unRegisterWithFIDLService();
                        if (retVal) {
                            serviceStatus = UNREGISTER_SERVICE_SUCCESS;
                            mIDLService->serviceRegisterationStatus = false;
                        } else {
                            serviceStatus = UNREGISTER_SERVICE_FAILED;
                            mIDLService->serviceRegisterationStatus = true;
                        }
                        break;
                    case POWER_STATE_RESUME:
                        retVal = mIDLService->registerWithFIDLService();
                        if (retVal) {
                            serviceStatus = REGISTER_SERVICE_SUCCESS;
                            mIDLService->serviceRegisterationStatus = true;
                        } else {
                            serviceStatus =REGISTER_SERVICE_FAILED;
                            mIDLService->serviceRegisterationStatus = false;
                        }
                        break;
                    default:
                        LOC_LOGd(" Unknown Power Event: %d !!", mPowerEvent);
                }
                if (mIDLService->mDiagLogIface) {
                    mIDLService->mDiagLogIface->diagLogPowerEventInfo(mPowerEvent, serviceStatus);
                }
            }
        }
    };
    mMsgTask->sendMsg(new PowerEventMsg(this, powerEvent));
}

void LocIdlAPIService::updateSystemStatus(uint32_t totalRss) {
    bool gptpSyncStatus = false;
    if (!mIsGptpInitialized && gptpInit()) {
        mIsGptpInitialized = true;
    }
    if (mIsGptpInitialized) {
        gptpSyncStatus = gptpGetSyncStatus();
    }
    if (mDiagLogIface) {
        mDiagLogIface->updateSystemHealth(totalRss,  gptpSyncStatus);
    }
}

void LocIdlAPIService::monitorMemoryUsage () {
     struct MonitorMemoryUsageMsg : public LocMsg {
        LocIdlAPIService* mIdlService;
        inline MonitorMemoryUsageMsg(LocIdlAPIService* idlService):
            LocMsg(),
            mIdlService(idlService){};
        inline virtual void proc() const {
            const char *pFileName = "/proc/self/statm";
            const uint16_t pageSize = 4; //4K
            unsigned long size = 0, rssPages = 0, totalRSS = 0;
            unsigned long share = 0, text = 0, lib = 0, data = 0, dt = 0;
            if (mIdlService) {
                do {
                    FILE *fp = fopen(pFileName, "r");
                    if (NULL != fp) {
                        if (7 == fscanf(fp, "%ld %ld %ld %ld %ld %ld %ld",
                               &size, &rssPages, &share, &text, &lib, &data, &dt)) {
                            totalRSS = rssPages * pageSize;
                            mIdlService->updateSystemStatus(totalRSS);
                        } else {
                            LOC_LOGe("Failed to read data from %s!! error: %s",
                                    pFileName, strerror(errno));
                        }
                        fclose(fp);
                    } else {
                        LOC_LOGe("Failed to open the file %s!! error: %s",
                                    pFileName, strerror(errno));
                    } if (exitFromMemUsageMsgTask) {
                         LOC_LOGd("Normal exit from monitorMemoryUsage");
                         break;
                    }
                     sleep(IDL_MEMORY_CHECK_INTERVAL_SEC); //Sleep and re-attempt
                } while (true);
                LOC_LOGe("Exiting monitorMemoryUsage...!");
            }
        }
    };
    mMemoryMonitorMsgTask->sendMsg(new MonitorMemoryUsageMsg(this));
}
bool LocIdlAPIService::init()
{

    struct InitMsg : public LocMsg {
        LocIdlAPIService* mLCAService;
        inline InitMsg(LocIdlAPIService* LCAService) :
            LocMsg(),
            mLCAService(LCAService){};

        inline virtual void proc() const {
            if (mLCAService) {
                mLCAService->createLocIdlService();
                mLCAService->monitorMemoryUsage();
            }
        }
    };

    mMsgTask->sendMsg(new InitMsg(this));
    return true;
}

bool LocIdlAPIService::createLocIdlService()
{
     LOC_LOGe("Initializing IDL Service ");
    // Create LCA object
    if (!mLcaInstance) {
        //Create capability callback
        CapabilitiesCb capabilitiesCb = [pClientApiService=this] (::LocationCapabilitiesMask mask) {
            LOC_LOGe("<<< onCapabilitiesCb mask=0x%" PRIx64 "", mask);
            LOC_LOGd("<<< onCapabilitiesCb mask string=%s ",
                    LocationClientApi::capabilitiesToString(mask).c_str());
            if (pClientApiService && mask) {
                pClientApiService->processCapabilities(mask);
                if (pClientApiService->mDiagLogIface)
                    pClientApiService->mDiagLogIface->diagLogCapabilityInfo(
                            LocationClientApi::capabilitiesToString(mask));
            }
        };
        // Create LCA instance
        mLcaInstance = new LocationClientApi(capabilitiesCb);
        LOC_LOGi (" LCA instance created Successfully ");
    }

    if (nullptr == mLIAInstance) {
        LocConfigPriorityMap priorityMap;
        LocIntegrationCbs intCbs;
        intCbs.configCb = LocConfigCb(onConfigResponseCb);
        mLIAInstance = new LocationIntegrationApi(priorityMap, intCbs);
        LOC_LOGi (" LIA instance created Successfully ");
    }
#ifdef POWER_DAEMON_MGR_ENABLED
    if (nullptr == mPowerEventObserver && mInstance != NULL) {
        mPowerEventObserver = LocIdlPowerEvtHandler::getPwrEvtHandler(mInstance);
        if (nullptr == mPowerEventObserver) {
            LOC_LOGe(" mPowerEventObserver null !! ");
        }
    }
#endif
    serviceRegisterationStatus = registerWithFIDLService();

    return true;
}

bool LocIdlAPIService::processCapabilities(::LocationCapabilitiesMask mask)
{
    struct ProcessCapsMsg : public LocMsg {

        LocIdlAPIService* mLCAService;
        ::LocationCapabilitiesMask mMask;
        string mCapsMask;
        inline ProcessCapsMsg(LocIdlAPIService* LCAService,
                ::LocationCapabilitiesMask mask, string capsMask) :
            LocMsg(),
            mLCAService(LCAService),
            mMask(mask),
            mCapsMask(capsMask){};
        inline virtual void proc() const {
            //convert capabilities from LCA to FIDL format
            uint32_t idlCapsMask = 0;
            if (mMask & LOCATION_CAPS_TIME_BASED_TRACKING_BIT) {
                idlCapsMask |=
                    LocationTypes::LocationCapabilitiesMaskT::LCMT_TIME_BASED_TRACKING_BIT;
            }
            if (mMask & LOCATION_CAPS_GNSS_MEASUREMENTS_BIT) {
                idlCapsMask |= LocationTypes::LocationCapabilitiesMaskT::LCMT_GNSS_MEAS_BIT;
            }
            mLCAService->mGnssCapabilites = idlCapsMask;
            //Update Capabilities to clients
            if (mLCAService->mService) {
                mLCAService->mService->fireGnssCapabilitiesEvent(idlCapsMask);
                mLCAService->mDiagLogIface->diagLogCapabilityInfo(mCapsMask);
            } else {
                LOC_LOGe("mLCAService->mService == NULL !! \n");
            }
        }
    };

    mMsgTask->sendMsg(new ProcessCapsMsg(this, mask,
            LocationClientApi::capabilitiesToString(mask)));
    return true;
}

bool LocIdlAPIService::registerWithFIDLService()
{
    LOC_LOGe("Registering IDL Service ");

    CommonAPI::Runtime::setProperty("LogContext", "LOCIDL");
    CommonAPI::Runtime::setProperty("LogApplication", "LOCIDL");
    CommonAPI::Runtime::setProperty("LibraryBase", "LocIdlAPI");
    std::shared_ptr<CommonAPI::Runtime> runtime = CommonAPI::Runtime::get();

    std::string domain = "local";
    std::string instance = "com.qualcomm.qti.location.Location";
    std::string connection = "location-fidl-service";
    CommonAPI::Version version = v1::com::qualcomm::qti::location::Location::getInterfaceVersion();
    LOC_LOGe("Version: Major %d Minor %d", version.Major, version.Minor);
    bool successfullyRegistered = false;
    if (!serviceRegisterationStatus) {
        mService = std::make_shared<LocIdlAPIStubImpl>(this);
        if (runtime && mService) {
            successfullyRegistered = runtime->registerService(domain, instance,
                    mService, connection);
            while (!successfullyRegistered) {
                LOC_LOGd("Register IDL Service failed, trying again in 100 milliseconds !!");
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                successfullyRegistered = runtime->registerService(domain,
                        instance, mService, connection);
            }
            LOC_LOGe("Successfully Registered Service!");
        } else {
            LOC_LOGe(" Either mService or runtime is NULL !! ");
        }
    } else {
       successfullyRegistered = true;
    }

    return successfullyRegistered;
}

bool LocIdlAPIService::unRegisterWithFIDLService()
{
    LOC_LOGi("UnRegistering IDL Service ");

    std::shared_ptr<CommonAPI::Runtime> runtime = CommonAPI::Runtime::get();

    std::string domain = "local";
    std::string instance = "com.qualcomm.qti.location.Location";
    std::string connection = "location-fidl-service";
    bool successfullyUnRegistered = false;
    if (serviceRegisterationStatus) {
        successfullyUnRegistered = runtime->unregisterService(domain,
                v1::com::qualcomm::qti::location::Location::getInterface(), instance);
        uint8_t count =  1;
            while (!successfullyUnRegistered && count <= IDL_MAX_RETRY) {
                LOC_LOGi("UnRegister IDL Service failed, No. of retires: %d !!", count);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                successfullyUnRegistered = runtime->unregisterService(domain,
                        v1::com::qualcomm::qti::location::Location::getInterface(), instance);
                count++;
            }
        LOC_LOGi("Successfully UnRegistered Service!");
    }
    return successfullyUnRegistered;
}

LocationTypes::LocationStatusT LocIdlAPIService::parseIDLResponse(
        const location_client::LocationResponse lcaResponse) const {

    LocationTypes::LocationStatusT resp =
            LocationTypes::LocationStatusT::LOCATION_STATUS_T_UNKNOWN;
    switch (lcaResponse) {
        case LOCATION_RESPONSE_SUCCESS:
            resp = LocationTypes::LocationStatusT::LOCATION_STATUS_T_SUCCESS;
            break;
        case LOCATION_RESPONSE_UNKOWN_FAILURE:
            resp = LocationTypes::LocationStatusT::LOCATION_STATUS_T_UNKOWN_FAILURE;
            break;
        case LOCATION_RESPONSE_NOT_SUPPORTED:
            resp = LocationTypes::LocationStatusT::LOCATION_STATUS_T_NOT_SUPPORTED;
            break;
        case LOCATION_RESPONSE_PARAM_INVALID:
            resp = LocationTypes::LocationStatusT::LOCATION_STATUS_T_PARAM_INVALID;
            break;
        case LOCATION_RESPONSE_TIMEOUT:
            resp = LocationTypes::LocationStatusT::LOCATION_STATUS_T_TIMEOUT;
            break;
        case LOCATION_RESPONSE_REQUEST_ALREADY_IN_PROGRESS:
            resp = LocationTypes::LocationStatusT::LOCATION_STATUS_T_REQUEST_ALREADY_IN_PROGRESS;
            break;
        case LOCATION_RESPONSE_SYSTEM_NOT_READY:
            resp = LocationTypes::LocationStatusT::LOCATION_STATUS_T_SYSTEM_NOT_READY;
            break;
        case LOCATION_RESPONSE_EXCLUSIVE_SESSION_IN_PROGRESS:
            resp = LocationTypes::LocationStatusT::LOCATION_STATUS_T_EXCLUSIVE_SESSION_IN_PROGRESS;
            break;
        default:
            resp = LocationTypes::LocationStatusT::LOCATION_STATUS_T_UNKNOWN;
    }
    return resp;
}

void LocIdlAPIService::checkMinIntervalForUpdate(uint32_t clientRequestedTbf) const {
    if (clientRequestedTbf) {
        if (!mTimeBetweenFixes) {
            mTimeBetweenFixes = clientRequestedTbf;
        } else {
            if (mTimeBetweenFixes > clientRequestedTbf) {
                mTimeBetweenFixes = clientRequestedTbf;
            }
        }
    } else {
        mTimeBetweenFixes = MIN_POS_REPORT_INTERVAL_MSEC;
    }
}

/* Process Fused/Detailed Position request */
void LocIdlAPIService::startPositionSession
(
    const std::shared_ptr<CommonAPI::ClientId> client,
    uint32_t intervalInMs, uint32_t gnssReportCallbackMask,
    LocationStub::StartPositionSessionLocationReportReply_t reply
) const
{
    struct StartFusedPosMsg : public LocMsg {
        const LocIdlAPIService* mLCAService;
        const std::shared_ptr<CommonAPI::ClientId> mClient;
        uint32_t mIntervalInMs;
        uint32_t mGnssReportCbMask;
        LocationStub::StartPositionSessionLocationReportReply_t mReply;
        inline StartFusedPosMsg(const LocIdlAPIService* LCAService,
                const std::shared_ptr<CommonAPI::ClientId> client,
                uint32_t intervalInMs,
                uint32_t gnssReportCallbackMask,
                LocationStub::StartPositionSessionLocationReportReply_t reply) :
            LocMsg(),
            mLCAService(LCAService),
            mClient(client),
            mIntervalInMs(intervalInMs),
            mGnssReportCbMask(gnssReportCallbackMask),
            mReply(reply){};
        inline virtual void proc() const {
            if (mLCAService) {
                mLCAService->numControlRequests++;
                mLCAService->mGnssReportMask |= mGnssReportCbMask;
                LOC_LOGi("==== startPositionSession intervalMs %u GnssReportCbMask 0X%X"
                         " LCAReportMask 0X%X numControlRequests %u", mIntervalInMs,
                         mGnssReportCbMask, mLCAService->mGnssReportMask,
                         mLCAService->numControlRequests);
                LocationTrackingSessCbHandler cbHandler(mLCAService, mLCAService->mGnssReportMask);
                ResponseCb rspCb = [client=mClient, reply=mReply] (::LocationResponse response) {
                    LOC_LOGd("==== responseCb %d", response);
                    //convert response from LCA to FIDL format
                    LocationTypes::LocationStatusT resp =
                            mInstance->parseIDLResponse(response);
                    reply(resp);
                };
                mLCAService->checkMinIntervalForUpdate(mIntervalInMs);
                if (mLCAService->mLcaInstance) {
                    mLCAService->mLcaInstance->startPositionSession(mLCAService->mTimeBetweenFixes,
                            cbHandler.getLocationCbs(), rspCb);
                }
                diagControlCommandInfo idlSessionInfo = {};
                idlSessionInfo.sessionRequestType = SESSION_START_REQUEST;
                idlSessionInfo.intervalMs = mLCAService->mTimeBetweenFixes;
                idlSessionInfo.requestedCallbackMask = mGnssReportCbMask;
                idlSessionInfo.updatedCallbackMask = mLCAService->mGnssReportMask;
                idlSessionInfo.numControlRequests = mLCAService->numControlRequests;
                if (mLCAService->mDiagLogIface) {
                    mLCAService->mDiagLogIface->diagLogSessionInfo(idlSessionInfo,
                            mClient->hashCode());
                }
            }
        }
    };
    mMsgTask->sendMsg(new StartFusedPosMsg(this, client, intervalInMs,
            gnssReportCallbackMask, reply));
}

/* Process Engine specific Position request */
void LocIdlAPIService::startPositionSession
(
       const std::shared_ptr<CommonAPI::ClientId> client,
       uint32_t intervalInMs, uint32_t locReqEngMask, uint32_t engReportCallbackMask,
    LocationStub::StartPositionSessionEngineSpecificLocationReply_t reply
) const
{
    struct StartPosMsg : public LocMsg {

        const LocIdlAPIService* mLCAService;
        const std::shared_ptr<CommonAPI::ClientId> mClient;
        uint32_t mIntervalInMs;
        uint32_t mLocReqEngMask;
        uint32_t mEngReportCallbackMask;
        LocationStub::StartPositionSessionLocationReportReply_t mReply;
        inline StartPosMsg(const LocIdlAPIService* LCAService,
                const std::shared_ptr<CommonAPI::ClientId> client,
                uint32_t intervalInMs,
                uint32_t locReqEngMask,
                uint32_t engReportCallbackMask,
                LocationStub::StartPositionSessionEngineSpecificLocationReply_t reply) :
            LocMsg(),
            mLCAService(LCAService),
            mClient(client),
            mIntervalInMs(intervalInMs),
            mLocReqEngMask(locReqEngMask),
            mEngReportCallbackMask(engReportCallbackMask),
            mReply(reply){};
        inline virtual void proc() const {
            if (mLCAService) {
                mLCAService->numControlRequests++;
                mLCAService->mGnssReportMask |= mEngReportCallbackMask;
                LOC_LOGi("==== startPositionSession Engine Specific %u 0X%X 0X%X ",
                        mIntervalInMs, mLocReqEngMask, mEngReportCallbackMask);
                LocationTrackingSessCbHandler cbHandler(mLCAService, mLocReqEngMask,
                        mEngReportCallbackMask);
                ResponseCb rspCb = [client=mClient, reply=mReply] (::LocationResponse response) {
                    LOC_LOGd("==== responseCb %d", response);
                    //convert response from LCA to FIDL format
                    LocationTypes::LocationStatusT resp =
                            mInstance->parseIDLResponse(response);
                    reply(resp);
                };
                mLCAService->checkMinIntervalForUpdate(mIntervalInMs);
                if (mLCAService->mLcaInstance) {
                    mLCAService->mLcaInstance->startPositionSession(mLCAService->mTimeBetweenFixes,
                            cbHandler.getLcaLocReqEngMask(),
                            cbHandler.getEngineLocationCbs(),
                            rspCb);
                }
            }
        }
    };

    mMsgTask->sendMsg(new StartPosMsg(this, client, intervalInMs,
            locReqEngMask, engReportCallbackMask, reply));
}

void LocIdlAPIService::stopPositionSession
(
    const std::shared_ptr<CommonAPI::ClientId> client,
    LocationStub::StopPositionSessionReply_t reply

) const
{
    struct StopPosMsg : public LocMsg {
        const LocIdlAPIService* mLCAService;
        const std::shared_ptr<CommonAPI::ClientId> mClient;
        LocationStub::StopPositionSessionReply_t mReply;
        inline StopPosMsg(const LocIdlAPIService* LCAService,
                const std::shared_ptr<CommonAPI::ClientId> client,
                LocationStub::StopPositionSessionReply_t reply) :
            LocMsg(),
            mLCAService(LCAService),
            mClient(client),
            mReply(reply){};
        inline virtual void proc() const {
            if (mLCAService) {
                if (mLCAService->numControlRequests > 0) {
                    mLCAService->numControlRequests--;
                    diagControlCommandInfo idlSessionInfo = {};
                    idlSessionInfo.sessionRequestType = SESSION_STOP_REQUEST;
                    idlSessionInfo.numControlRequests = mLCAService->numControlRequests;
                    if (mLCAService->mDiagLogIface) {
                        mLCAService->mDiagLogIface->diagLogSessionInfo(
                                idlSessionInfo, mClient->hashCode());
                    }
                    if (!mLCAService->numControlRequests) {
                        LOC_LOGe(" Sending STOP Session request !!");
                        mLCAService->mLcaInstance->stopPositionSession();
                        posCount = 0;
                        latentPosCount = 0;
                        mLCAService->mGnssReportMask = 0;
                        mLCAService->mTimeBetweenFixes = 0;
                    }
                } else {
                    LOC_LOGe(" Faulty STOP request numOfRequests %d",
                            mLCAService->numControlRequests);
                }
            }
       }
    };
    mMsgTask->sendMsg(new StopPosMsg(this, client, reply));
}

uint16_t LocIdlAPIService::parseDeleteAidingDataMask(
       uint32_t deleteMask) const{

    uint16_t mask =0;
    if (LocationTypes::AidingDataDeletionMaskT::ADDMT_EPHEMERIS & deleteMask) {
        mask |= LocationTypes::AidingDataDeletionMaskT::ADDMT_EPHEMERIS;
    }
    if (LocationTypes::AidingDataDeletionMaskT::ADDMT_DR_SENSOR_CALIBRATION & deleteMask) {
        mask |= LocationTypes::AidingDataDeletionMaskT::ADDMT_DR_SENSOR_CALIBRATION;
    }
    if (LocationTypes::AidingDataDeletionMaskT::ADDMT_ALL & deleteMask) {
        mask |= LocationTypes::AidingDataDeletionMaskT::ADDMT_ALL;
    }
    return mask;
}

void LocIdlAPIService::deleteAidingDataRequest
(
    const std::shared_ptr<CommonAPI::ClientId> client,
    uint32_t deleteMask, LocationStub::DeleteAidingDataReply_t reply) const {

    uint16_t mask = parseDeleteAidingDataMask(deleteMask);
    LOC_LOGd(" DeleteAssistance Mask recieved: %x ", mask);
    if (mLIAInstance) {
        bool ret = mLIAInstance->deleteAidingData((location_integration::\
                    AidingDataDeletionMask)mask);
        if (ret) {
            reply(LocationTypes::LocationStatusT::LOCATION_STATUS_T_SUCCESS);
        } else {
            reply(LocationTypes::LocationStatusT::LOCATION_STATUS_T_UNKOWN_FAILURE);
        }
    }
    if (mDiagLogIface) {
        mDiagLogIface->diagLogDeleteAidingRequest(client->hashCode(), mask);
    }
}

void LocIdlAPIService::configConstellationsRequest
(
    const std::shared_ptr<CommonAPI::ClientId> client,
    std::vector<LocationTypes::GnssSvIdInfoT > svListSrc,
    LocationStub::ConfigConstellationsReply_t reply
) const
{

    LOC_LOGd(" ");
    location_integration::LocConfigBlacklistedSvIdList svList;
    for (int i = 0; i < svListSrc.size(); i++) {
        location_integration::GnssSvIdInfo svIdConstellation = {};

        switch (svListSrc[i].getConstellation()) {
            case LocationTypes::GnssConstellationTypeT::GCTT_GLONASS:
                svIdConstellation.constellation = location_integration::\
                        GNSS_CONSTELLATION_TYPE_GLONASS;
                break;
            case LocationTypes::GnssConstellationTypeT::GCTT_QZSS:
                svIdConstellation.constellation = location_integration::\
                        GNSS_CONSTELLATION_TYPE_QZSS;
                break;
            case LocationTypes::GnssConstellationTypeT::GCTT_BEIDOU:
                svIdConstellation.constellation = location_integration::\
                        GNSS_CONSTELLATION_TYPE_BEIDOU;
                break;
            case LocationTypes::GnssConstellationTypeT::GCTT_GALILEO:
                svIdConstellation.constellation = location_integration::\
                        GNSS_CONSTELLATION_TYPE_GALILEO;
                break;
            case LocationTypes::GnssConstellationTypeT::GCTT_SBAS:
                svIdConstellation.constellation = location_integration::\
                        GNSS_CONSTELLATION_TYPE_SBAS;
                break;
            case LocationTypes::GnssConstellationTypeT::GCTT_NAVIC:
                svIdConstellation.constellation = location_integration::\
                        GNSS_CONSTELLATION_TYPE_NAVIC;
                break;
            case LocationTypes::GnssConstellationTypeT::GCTT_GPS:
                svIdConstellation.constellation = location_integration::\
                        GNSS_CONSTELLATION_TYPE_GPS;
                break;
            default:
                svIdConstellation.constellation = location_integration::\
                        GNSS_CONSTELLATION_TYPE_MAX;
       }
       svIdConstellation.svId = svListSrc[i].getSvId();
       svList.push_back(svIdConstellation);
    }
    if (mLIAInstance) {
        bool retVal = mLIAInstance->configConstellations(&svList);
        if (retVal) {
            reply(LocationTypes::LocationStatusT::LOCATION_STATUS_T_SUCCESS);
        } else {
            reply(LocationTypes::LocationStatusT::LOCATION_STATUS_T_UNKOWN_FAILURE);
        }
    }
    if (mDiagLogIface) {
        mDiagLogIface->diagLogConfigConstellationRequest(client->hashCode(), svListSrc);
    }
}

void LocIdlAPIService::injectMapMatchedFeedbackData
(
    const std::shared_ptr<CommonAPI::ClientId> client,
    LocationTypes::MapMatchingFeedbackDataT mapData
) const
{
    location_integration::mapMatchedFeedbackData mmfData = {};

    mmfData.validityMask = mapData.getValidityMask();
    mmfData.utcTimestampMs = mapData.getUtcTimestampMs();
    mmfData.mapMatchedLatitudeDifference = mapData.getMapMatchedLatitudeDifference();
    mmfData.mapMatchedLongitudeDifference = mapData.getMapMatchedLongitudeDifference();
    mmfData.isTunnel = mapData.getIsTunnel();
    mmfData.bearing = mapData.getBearing();
    mmfData.altitude = mapData.getAltitude();
    mmfData.horizontalAccuracy = mapData.getHorizontalAccuracy();
    mmfData.altitudeAccuracy = mapData.getAltitudeAccuracy();
    mmfData.bearingAccuracy = mapData.getBearingAccuracy();

    if (mLIAInstance) {
        bool retVal = mLIAInstance->injectMapMatchedData(mmfData);
    }
}

int main() {
    LocIdlAPIService *pLCAService = LocIdlAPIService::getInstance();
    if (pLCAService) {
        pLCAService->init();
    }

    if (gptpInit()) {
        LOC_LOGd(" GPTP init success ");
    } else {
        LOC_LOGe(" GPTP init failed ");
    }
    // Waiting for calls
    int fd[2], n = 0;
    char buffer[10];
    if (pipe(fd) != -1) {
        n = read(fd[0], buffer, 10);
        if (n > 0) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    return 0;
}
