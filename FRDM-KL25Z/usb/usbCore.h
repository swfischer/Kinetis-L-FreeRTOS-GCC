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
// Based on the Freescale USB echo device demo
// ----------------------------------------------------------------------------

#ifndef _USBCORE_H_
#define _USBCORE_H_

#include <stdint.h>

#include "usb.h"

#define USB_BDT_STAT_STALL       (1 << 2)
#define USB_BDT_STAT_DTS         (1 << 3)
#define USB_BDT_STAT_NINC        (1 << 4)
#define USB_BDT_STAT_KEEP        (1 << 5)
#define USB_BDT_STAT_DATA1       (1 << 6)
#define USB_BDT_STAT_USBOWN      (1 << 7)
#define USB_BDT_STAT_PID_MASK    (0x3C)
#define USB_BDT_STAT_PID_SHIFT   (2)
#define usbCoreBdtGetPid(x)      ((x & USB_BDT_STAT_PID_MASK) >> USB_BDT_STAT_PID_SHIFT)

typedef struct
{
   uint8_t  stat;
   uint8_t  reserved;
   uint16_t cnt;
   uint32_t addr;
} bdt_t;

// Macros
#define usbSIE_CONTROL(EP)   (bdtTable[EP << 2].stat = kSIE)
#define usbMCU_CONTROL(EP)   (bdtTable[EP << 2].stat = kMCU)
#define usbEP_Reset(EP)      (bdtTable[EP << 2].cnt = 0x0020)

// BDT status value
#define kMCU      (0)
#define kSIE      (USB_BDT_STAT_USBOWN)

#define kUDATA0   (USB_BDT_STAT_USBOWN | USB_BDT_STAT_DTS)
#define kUDATA1   (USB_BDT_STAT_USBOWN | USB_BDT_STAT_DTS | USB_BDT_STAT_DATA1)

enum
{ uSETUP
, uDATA
};

enum
{ EP0
, EP1
, EP2
, EP3
, EP4
, EP5
, DUMMY
, LOADER
};

enum
{ uPOWER
, uENUMERATED
, uENABLED
, uADDRESS
, uREADY
};

enum
{ fIN
, fOUT
};

enum
{ bEP0OUT_ODD
, bEP0OUT_EVEN
, bEP0IN_ODD
, bEP0IN_EVEN
, bEP1OUT_ODD
, bEP1OUT_EVEN
, bEP1IN_ODD
, bEP1IN_EVEN
, bEP2OUT_ODD
, bEP2OUT_EVEN
, bEP2IN_ODD
, bEP2IN_EVEN
, bEP3OUT_ODD
, bEP3OUT_EVEN
, bEP3IN_ODD
, bEP3IN_EVEN
};

// It's used in some macros here.
extern bdt_t bdtTable[];

typedef uint8_t (*usbInterfaceReqHandler)(uint8_t ep, usbSetupPacket_t *pkt);

extern void usbCoreInit(usbInterfaceReqHandler handler);
extern void usbCoreEpInTransfer(uint8_t ep, uint8_t *data, uint8_t size);
extern uint8_t  usbCoreEpOutTransfer(uint8_t ep, uint8_t *data);
extern uint16_t usbCoreEpOutSizeCheck(uint8_t ep);

#endif // _USBCORE_H_
