/**
* Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

package vendor.qti.hardware.agm;

/**
 * Event registration structure.
 */
@VintfStability
parcelable AgmEventRegistrationConfig {
    int moduleInstanceId;
    int eventId;
    byte registerEvent;
    byte[] eventConfigPayload;
}
