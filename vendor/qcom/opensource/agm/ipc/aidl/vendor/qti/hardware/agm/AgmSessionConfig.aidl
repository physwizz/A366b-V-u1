/**
* Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

package vendor.qti.hardware.agm;

import vendor.qti.hardware.agm.AgmDataMode;
import vendor.qti.hardware.agm.AgmSessionCodec;
import vendor.qti.hardware.agm.AgmSessionMode;
import vendor.qti.hardware.agm.Direction;

@VintfStability
parcelable AgmSessionConfig {
    Direction direction;
    AgmSessionMode sessionMode;
    int startThreshold;
    int stopThreshold;
    @nullable AgmSessionCodec codec;
    AgmDataMode dataMode;
    int flags;
}
