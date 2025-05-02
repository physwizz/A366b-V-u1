/**
* Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

package vendor.qti.hardware.agm;

import vendor.qti.hardware.agm.AgmEventCallbackParameter;
import vendor.qti.hardware.agm.AgmReadWriteEventCallbackParams;

@VintfStability
interface IAGMCallback {

    void eventCallback(in AgmEventCallbackParameter eventParam);

    void eventCallbackReadWriteDone(in AgmReadWriteEventCallbackParams rwDonePayload);
}
