/**
* Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

package vendor.qti.hardware.agm;

@VintfStability
parcelable AgmSessionWmaproDec {
    int formatTag;
    int channels;
    int sampleRate;
    int averageBytesPerSecond;
    int blockAlign;
    int bitsPerSample;
    int channelMask;
    int encoderOption;
    int advancedEncoderOption;
    int advancedEncoderOption2;
}
