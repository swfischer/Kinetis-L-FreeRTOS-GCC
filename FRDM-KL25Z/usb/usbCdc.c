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

#include <stdint.h>
#include <stdbool.h>

#include "kinetis.h"
#include "os.h"
#include "ringBuffer.h"
#include "usbCdc.h"
#include "usbCore.h"

extern uint8_t gu8EP2_IN_ODD_Buffer[];
extern uint8_t gu8EP3_OUT_ODD_Buffer[];

extern uint8_t usbFlagsHack;
extern uint8_t usbStateHack;

// Not really used, but could be used to denote VBUS state changes
volatile uint8_t usbCdcIsrFlags = 0;

#define EVENT_SET_LINE_CODING       (1 << 0)
#define EVENT_SET_CTRL_LINE_STATE   (1 << 1)
static uint8_t sEventFlags = 0;
static bool    sDteActive = 0;

static uint8_t sAltInterface = 0;  // FIXME: should be coordinated with interface descriptor
static usbCdcLineCoding_t sLineCoding;

// Local function prototypes
static uint8_t interfaceReqHandler(uint8_t ep, usbSetupPacket_t *pkt);

// ----------------------------------------------------------------------------
// External Functions
// ----------------------------------------------------------------------------

void usbCdcInit(void)
{
   // USB core initialization
   usbCoreInit(interfaceReqHandler);

   // Line Coding Initialization
   sLineCoding.dteRate    = 9600;
   sLineCoding.charFormat = 0;
   sLineCoding.parityType = 0;
   sLineCoding.databits   = 8;
}

void usbCdcEngine(void)
{
   // Re-init CDC class if a VBUS HIGH event was detected
   if (usbCdcIsrFlags & VBUS_HIGH_EVENT)
   {
      usbCdcIsrFlags &= ~(VBUS_HIGH_EVENT);

      USB0_CTL |= USB_CTL_USBENSOFEN_MASK;
      usbCdcInit();
   }

   if (sEventFlags)
   {
      if (sEventFlags && EVENT_SET_LINE_CODING)
      {
         if (usbFlagsHack & (1 << EP0))
         {
            usbFlagsHack &= ~(1 << EP0);

            usbCoreEpOutTransfer(EP0, (uint8_t*) &sLineCoding);
            usbCoreEpInTransfer(EP0, 0, 0);
         }

         sEventFlags &= ~(EVENT_SET_LINE_CODING);
      }
      
      if (sEventFlags && EVENT_SET_CTRL_LINE_STATE)
      {
         usbCoreEpInTransfer(EP0, 0, 0);
         
         sEventFlags &= ~(EVENT_SET_CTRL_LINE_STATE);
      }
   }
   else
   {
      // Wait for USB Enumeration
      while (usbStateHack != uENUMERATED)
      {
         osDelay(500); // slow things down to allow other tasks to run
      };
   }
}

// ----------------------------------------------------------------------------
// Local Functions
// ----------------------------------------------------------------------------

static uint8_t interfaceReqHandler(uint8_t ep, usbSetupPacket_t *pkt)
{
   uint8_t state = uSETUP;

   switch (pkt->bRequest)
   {
   case USB_REQ_GET_INTERFACE:
      usbCoreEpInTransfer(ep, &sAltInterface, 1);
      break;
   case USB_CDC_REQ_GET_LINE_CODING:
      usbCoreEpInTransfer(ep, (uint8_t*)&sLineCoding, sizeof(sLineCoding)); //7);
      break;
   case USB_CDC_REQ_SET_LINE_CODING:
      sEventFlags |= EVENT_SET_LINE_CODING;
      state = uDATA;
      break;
   case USB_CDC_REQ_SET_CTRL_LINE_STATE:
      sDteActive = (pkt->wValue.word & USB_CDC_SCLS_DTE_PRESENT) ? true : false;
      sEventFlags |= EVENT_SET_CTRL_LINE_STATE;
      state = uSETUP;
      break;
   }

   return (state);
}
