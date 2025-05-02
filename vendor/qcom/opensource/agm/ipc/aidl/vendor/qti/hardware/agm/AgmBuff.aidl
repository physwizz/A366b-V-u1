/**
* Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

package vendor.qti.hardware.agm;

import vendor.qti.hardware.agm.AgmExternAllocBuffInfo;

@VintfStability
parcelable AgmBuff {
    long timestamp;
    int flags;
    int size;
    int offset;
    byte[] buffer;
    byte[] metadata;
    AgmExternAllocBuffInfo externalAllocInfo;
}
