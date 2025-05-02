/**
* Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

package vendor.qti.hardware.agm;

@VintfStability
parcelable AgmSessionApeDec {
    int compatibleVersion;
    int compressionLevel;
    int formatFlags;
    int blocksPerFrame;
    int finalFrameBlocks;
    int totalFrames;
    int bitWidth;
    int channels;
    int sampleRate;
    int seekTablePresent;
}
