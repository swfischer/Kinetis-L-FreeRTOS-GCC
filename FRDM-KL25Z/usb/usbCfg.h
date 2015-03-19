// ----------------------------------------------------------------------------
// Copyright (c) 2015, Steven W. Fischer
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer. 
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
// ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// The views and conclusions contained in the software and documentation are those
// of the authors and should not be interpreted as representing official policies,
// either expressed or implied, of the FreeBSD Project.
// ----------------------------------------------------------------------------

#ifndef _USBCFG_H_
#define _USBCFG_H_

#include <stdint.h>
#include <stdbool.h>

#include "usb.h"

typedef struct __attribute__((packed))
{
   usbConfigDesc_t      cfg;
   usbInterfaceDesc_t   inf0;
   usbCdcHdrDesc_t      cdcHdr;
   usbCdcCallMgtDesc_t  cdcCm;
   usbCdcAcmDesc_t      cdcAcm;
   usbCdcUnionDesc_t    cdcUnion;
   usbEndpointDesc_t    ep1;
   usbInterfaceDesc_t   inf1;
   usbEndpointDesc_t    ep2;
   usbEndpointDesc_t    ep3;
} usbCfg_t;

#define USBCFG_EP_COUNT          (4) // EP0 + 3 additional
#define USBCFG_BDT_ENTRY_COUNT   (4 * USBCFG_EP_COUNT)
#define USBCFG_EP_BUF_SIZE       (32) // All the same size

#define USBCFG_EP0_SIZE    (32)
#define USBCFG_EP1_SIZE    (32)
#define USBCFG_EP2_SIZE    (32)
#define USBCFG_EP3_SIZE    (32)

extern const usbDeviceDesc_t usbCfgDevDesc;
extern const usbCfg_t usbCfg;
extern const uint8_t* usbCfgStringTable[];

extern bool usbCfgIsEpIN(uint8_t ep);

#endif // _USBCFG_H_
