/**
* Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

package vendor.qti.hardware.agm;

/**
 * AAC decoder parameters
 */
@VintfStability
parcelable AgmSessionAacDec {
    int formatFlag;
    int objectType;
    int channels;
    int sizeOfPCEBits;
    int sampleRate;
}
