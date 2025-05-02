/**
* Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

package vendor.qti.hardware.agm;

/**
 * Data Events that will be notified to client from AGM
 */
@VintfStability
@Backing(type="int")
enum AgmEventId {
    AGMEVENTEOSRENDERED = 0x0,
    AGMEVENTREADDONE = 0x1,
    AGMEVENTWRITEDONE = 0x2,
    AGM_EVENT_EARLY_EOS = 0x08001126,
    AGMEVENTIDMAX,
}
