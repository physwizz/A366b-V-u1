/**
* Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

package vendor.qti.hardware.agm;

@VintfStability
parcelable AgmSessionAlacDec {
    int frameLength;
    byte compatibleVersion;
    byte bitDepth;
    byte pb;
    byte mb;
    byte kb;
    byte channels;
    int maxRun;
    int maxFrameBytes;
    int averageBitRate;
    int sampleRate;
    int channelLayoutTag;
}
