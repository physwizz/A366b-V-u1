/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

package vendor.qti.hardware.paleventnotifier;

import vendor.qti.hardware.paleventnotifier.IPALEventNotifierCallback;
import vendor.qti.hardware.paleventnotifier.PalDeviceId;
import vendor.qti.hardware.paleventnotifier.PalStreamType;

@VintfStability
interface IPALEventNotifier {

    int ipc_pal_notify_register_callback(in IPALEventNotifierCallback callback);
    int ipc_pal_notify_register_callback_v2(in IPALEventNotifierCallback callback, in PalDeviceId[] devID, in PalStreamType[] streamType);

}
