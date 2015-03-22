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
#include "usbCfg.h"

#define USB_BDT_STAT_STALL       (1 << 2)
#define USB_BDT_STAT_DTS         (1 << 3)
#define USB_BDT_STAT_NINC        (1 << 4)
#define USB_BDT_STAT_KEEP        (1 << 5)
#define USB_BDT_STAT_DATA1       (1 << 6)
#define USB_BDT_STAT_HW_OWNED    (1 << 7)
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
#define kSIE      (USB_BDT_STAT_HW_OWNED)
#define kUDATA0   (USB_BDT_STAT_HW_OWNED | USB_BDT_STAT_DTS)
#define kUDATA1   (USB_BDT_STAT_HW_OWNED | USB_BDT_STAT_DTS | USB_BDT_STAT_DATA1)

enum
{ EP0
, EP1
, EP2
, EP3
};

// The order here is important as it corresponds to the USB_STAT register numbering
#define USB_EP_ENUMS(x) USB_EP##x##_RX_ODD, USB_EP##x##_RX_EVEN, \
                        USB_EP##x##_TX_ODD, USB_EP##x##_TX_EVEN
enum
{ USB_EP_ENUMS(0)
, USB_EP_ENUMS(1)
, USB_EP_ENUMS(2)
, USB_EP_ENUMS(3)
};

// It's used in some macros here.
extern bdt_t bdtTable[USBCFG_BDT_ENTRY_COUNT];
extern uint8_t epBuffers[USBCFG_BDT_ENTRY_COUNT][USBCFG_EP_BUF_SIZE];

#define USB_CTRL_EVENT_RESET        (1)  // Reset occurred
#define USB_CTRL_EVENT_ENUMERATION  (2)  // Enumeration occurred
#define USB_CTRL_EVENT_REQUEST      (3)  // Request for CDC processing

typedef void (*usbCtrlIsrHandler)(int event);
typedef void (*usbDataIsrHandler)(uint8_t ep, uint8_t *data, uint16_t len);
typedef bool (*usbInterfaceReqHandler)(uint8_t ep, usbSetupPacket_t *pkt);

extern void usbDevInit( usbCtrlIsrHandler ctrlHandler
                      , usbDataIsrHandler dataHandler
                      , usbInterfaceReqHandler reqHandler
                      );
extern void usbDevEpTxTransfer(uint8_t ep, uint8_t *data, uint8_t len);

#endif // _USBCORE_H_
