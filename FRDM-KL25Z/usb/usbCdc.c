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

// Not really used, but could be used to denote VBUS state changes
volatile uint8_t usbCdcIsrFlags = 0;

#define EVENT_SET_LINE_CODING       (1 << 0)
#define EVENT_SET_CTRL_LINE_STATE   (1 << 1)
static uint8_t sEventFlags = 0;
static bool    sDteActive = 0;

static uint8_t sAltInterface = 0;  // FIXME: should be coordinated with interface descriptor
static usbCdcLineCoding_t sLineCoding;

static usbCtrlIsrHandler sAppCtrlIsrHandler = NULL;
static usbDataIsrHandler sAppDataIsrHandler = NULL;

// Local function prototypes
static void cdcDataIsrHandler(uint8_t ep, uint8_t *data, uint16_t len);
static bool interfaceReqHandler(uint8_t ep, usbSetupPacket_t *pkt);

// ----------------------------------------------------------------------------
// External Functions
// ----------------------------------------------------------------------------

void usbCdcInit(usbCtrlIsrHandler ctrlHandler, usbDataIsrHandler dataHandler)
{
   sAppCtrlIsrHandler = ctrlHandler;
   sAppDataIsrHandler = dataHandler;

   // USB core initialization
   usbCoreInit(ctrlHandler, cdcDataIsrHandler, interfaceReqHandler);

   // Line Coding Initialization
   sLineCoding.dteRate    = 9600;
   sLineCoding.charFormat = 0;
   sLineCoding.parityType = 0;
   sLineCoding.databits   = 8;
}

void usbCdcEngine(void)
{
   if (sEventFlags)
   {
      if (sEventFlags && EVENT_SET_LINE_CODING)
      {
         // Nothing to do
      }
      
      if (sEventFlags && EVENT_SET_CTRL_LINE_STATE)
      {
         usbCoreEpTxTransfer(EP0, 0, 0);
         
         sEventFlags &= ~(EVENT_SET_CTRL_LINE_STATE);
      }
   }
}

// ----------------------------------------------------------------------------
// Local Functions
// ----------------------------------------------------------------------------

static void cdcDataIsrHandler(uint8_t ep, uint8_t *data, uint16_t len)
{
   if (ep == EP0)
   {
      if (sEventFlags && EVENT_SET_LINE_CODING && len == sizeof(sLineCoding))
      {
         uint8_t *p = (uint8_t*) &sLineCoding;
         uint8_t *d = data;
         int i = sizeof(sLineCoding);

         while (i--)
         {
            *p++ = *d++;
         }

         usbCoreEpTxTransfer(EP0, 0, 0);

         sEventFlags &= ~(EVENT_SET_LINE_CODING);
      }
   }
   else if (sAppDataIsrHandler)
   {
      sAppDataIsrHandler(ep, data, len);
   }
}

static bool interfaceReqHandler(uint8_t ep, usbSetupPacket_t *pkt)
{
   bool expectDataPkt = false;

   switch (pkt->bRequest)
   {
   case USB_REQ_GET_INTERFACE:
      usbCoreEpTxTransfer(ep, &sAltInterface, 1);
      break;
   case USB_CDC_REQ_GET_LINE_CODING:
      usbCoreEpTxTransfer(ep, (uint8_t*)&sLineCoding, sizeof(sLineCoding));
      break;
   case USB_CDC_REQ_SET_LINE_CODING:
      sEventFlags |= EVENT_SET_LINE_CODING;
      expectDataPkt = true;
      break;
   case USB_CDC_REQ_SET_CTRL_LINE_STATE:
      sDteActive = (pkt->wValue.word & USB_CDC_SCLS_DTE_PRESENT) ? true : false;
      sEventFlags |= EVENT_SET_CTRL_LINE_STATE;
      break;
   }

   if (sAppCtrlIsrHandler)
   {
      sAppCtrlIsrHandler(USB_CTRL_EVENT_REQUEST);
   }

   return (expectDataPkt);
}
