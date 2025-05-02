/**
* Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

package vendor.qti.hardware.agm;

import vendor.qti.hardware.agm.AgmBuff;
import vendor.qti.hardware.agm.AgmBufferConfig;
import vendor.qti.hardware.agm.AgmCalConfig;
import vendor.qti.hardware.agm.AgmDumpInfo;
import vendor.qti.hardware.agm.AgmEventRegistrationConfig;
import vendor.qti.hardware.agm.AgmGaplessSilenceType;
import vendor.qti.hardware.agm.AgmGroupMediaConfig;
import vendor.qti.hardware.agm.AgmMediaConfig;
import vendor.qti.hardware.agm.AgmSessionConfig;
import vendor.qti.hardware.agm.AgmSessionMode;
import vendor.qti.hardware.agm.AgmTagConfig;
import vendor.qti.hardware.agm.AifInfo;
import vendor.qti.hardware.agm.Direction;
import vendor.qti.hardware.agm.IAGMCallback;
import vendor.qti.hardware.agm.MmapBufInfo;

@VintfStability
interface IAGM {

    void ipc_agm_init();

    void ipc_agm_deinit();

    long ipc_agm_session_open(in int sessionId, in AgmSessionMode sessionMode);

    void ipc_agm_session_start(in long handle);

    void ipc_agm_session_stop(in long handle);

    void ipc_agm_session_suspend(in long handle);

    void ipc_agm_session_pause(in long handle);

    void ipc_agm_session_resume(in long handle);

    void ipc_agm_session_prepare(in long handle);

    void ipc_agm_session_eos(in long handle);

    void ipc_agm_session_flush(in long handle);

    void ipc_agm_session_close(in long handle);

    MmapBufInfo ipc_agm_session_get_buf_info(in int sessionId, in int flag);

    byte[] ipc_agm_session_get_params(in int sessionId , in byte[] buffer);

    byte[] ipc_agm_session_read(in long handle, in int count);

    parcelable AgmReadWithMetadataReturn {
        AgmBuff buffer;
        int capturesSize;
    }
    AgmReadWithMetadataReturn ipc_agm_session_read_with_metadata(in long handle,
                                                         in AgmBuff buffer, in int capturedSize);

    void ipc_agm_session_register_callback(in IAGMCallback callback, in int sessionId,
                                     in int eventType, in boolean register, in long clientData);

    void ipc_agm_session_register_for_events(in int sessionId,
                                         in AgmEventRegistrationConfig eventConfig);

    void ipc_agm_session_set_config(in long handle, in AgmSessionConfig sessionConfig,
        in AgmMediaConfig mediaConfig, in AgmBufferConfig bufferConfig);

    void ipc_agm_session_set_ec_ref(in int sessionId, in int aifId, in boolean state);

    void ipc_agm_session_set_loopback(in int captureSessionId, in int playbackSessionId,
        in boolean state);

   void ipc_agm_session_set_metadata(in int sessionId, in byte[] metadata);

    void ipc_agm_session_set_non_tunnel_mode_config(in long handle,
        in AgmSessionConfig sessionConfig, in AgmMediaConfig inMediaConfig,
        in AgmMediaConfig outMediaConfig, in AgmBufferConfig inBufferConfig,
        in AgmBufferConfig outBufferConfig);

    void ipc_agm_session_set_params(in int sessionId, in byte[] payload);

    int ipc_agm_session_write(in long handle, in byte[] buff);

    void ipc_agm_session_write_datapath_params(in int sessionId, in AgmBuff buff);

    int ipc_agm_session_write_with_metadata(in long handle, in AgmBuff buff);

    void ipc_agm_aif_group_set_media_config(in int groupId, in AgmGroupMediaConfig config);

    void ipc_agm_aif_set_media_config(in int aifId, in AgmMediaConfig config);

    void ipc_agm_aif_set_metadata(in int aifId, in byte[] metadata);

    void ipc_agm_aif_set_params(in int aifId, in byte[] payload);

    void ipc_agm_session_aif_connect(in int sessionId, in int aifId, in boolean state);

    byte[] ipc_agm_session_aif_get_tag_module_info(in int sessionId, in int aifId, in int size);

    void ipc_agm_session_aif_set_cal(in int sessionId, in int aifId,
        in AgmCalConfig calConfig);

    void ipc_agm_session_aif_set_metadata(in int sessionId, in int aifId, in byte[] metadata);

    void ipc_agm_session_aif_set_params(in int sessionId, in int aifId, in byte[] payload);

    AifInfo[] ipc_agm_get_aif_info_list(in int numAifInfo);

    long ipc_agm_get_buffer_timestamp(in int sessiondId);

    AifInfo[] ipc_agm_get_group_aif_info_list(in int numberOfGroups);

    void ipc_agm_get_hw_processed_buff_cnt(in long handle, in Direction direction);

    byte[] ipc_agm_get_params_from_acdb_tunnel(in byte[] payload);

    long ipc_agm_get_session_time(in long handle);

    void ipc_agm_sessionid_flush(in int sessiondId);

    void ipc_agm_set_gapless_session_metadata(in long handle, in AgmGaplessSilenceType type, in int silence);

    void ipc_agm_set_params_to_acdb_tunnel(in byte[] payload);

    void ipc_agm_set_params_with_tag(in int sessiondId, in int aifId,
        in AgmTagConfig tagConfig);

    void ipc_agm_set_params_with_tag_to_acdb(in int sessiondId, in int aifId, in byte[] payload);

    void ipc_agm_dump(in AgmDumpInfo[] dumpInfo);
}
