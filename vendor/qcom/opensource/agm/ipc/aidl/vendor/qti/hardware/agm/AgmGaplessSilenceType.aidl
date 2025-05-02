/**
* Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

package vendor.qti.hardware.agm;

/**
 * Gapless playback Silence type
 */
@VintfStability
@Backing(type="int")
enum AgmGaplessSilenceType {
    INITIAL_SILENCE,
    TRAILING_SILENCE,
}
