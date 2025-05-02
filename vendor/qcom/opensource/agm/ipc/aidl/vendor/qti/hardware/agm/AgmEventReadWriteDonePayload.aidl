/**
* Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

package vendor.qti.hardware.agm;

import vendor.qti.hardware.agm.AgmBuff;

@VintfStability
parcelable AgmEventReadWriteDonePayload {
    int tag;
    int status;
    int metadataStatus;
    AgmBuff buffer;
}
