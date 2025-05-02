/**
* Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

package vendor.qti.hardware.agm;

/**
 * AGM session modes
 */
@VintfStability
@Backing(type="int")
enum AgmSessionMode {
    AGM_SESSION_DEFAULT,
    AGM_SESSION_NO_HOST,
    AGM_SESSION_NON_TUNNEL,
    AGM_SESSION_NO_CONFIG,
    AGM_SESSION_COMPRESS,
}
