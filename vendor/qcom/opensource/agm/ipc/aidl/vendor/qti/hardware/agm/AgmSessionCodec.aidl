/**
* Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

package vendor.qti.hardware.agm;

import vendor.qti.hardware.agm.AgmSessionAacDec;
import vendor.qti.hardware.agm.AgmSessionAlacDec;
import vendor.qti.hardware.agm.AgmSessionApeDec;
import vendor.qti.hardware.agm.AgmSessionFlacDec;
import vendor.qti.hardware.agm.AgmSessionWmaDec;
import vendor.qti.hardware.agm.AgmSessionWmaproDec;
import vendor.qti.hardware.agm.AgmSessionAacEnc;

@VintfStability
union AgmSessionCodec {
    AgmSessionAacDec aacDecoder;
    AgmSessionFlacDec flacDecoder;
    AgmSessionAlacDec alacDecoder;
    AgmSessionApeDec apeDecoder;
    AgmSessionWmaDec wmaDecoder;
    AgmSessionWmaproDec wmaproDecoder;
    AgmSessionAacEnc aacEncoder;
}
