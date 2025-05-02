/**
* Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

package vendor.qti.hardware.agm;

@VintfStability
parcelable AgmSessionFlacDec {
    int channels;
    int sampleSize;
    int minBlockSize;
    int maxBlockSize;
    int sampleRate;
    int minFrameSize;
    int maxFrameSize;
}
