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

static uint8_t sCdcState = WAITING_FOR_ENUMERATION;
static uint8_t sCdcOutData[CDC_BUFFER_SIZE];
static uint8_t sAltInterface = 0;  // should be coordinated with interface descriptor
static usbCdcLineCoding_t sLineCoding;

// Local function prototypes
static uint8_t interfaceReqHandler(uint8_t ep, usbSetupPacket_t *pkt);

// ----------------------------------------------------------------------------
// External Functions
// ----------------------------------------------------------------------------

void usbCdcInit(void)
{
   sCdcState = WAITING_FOR_ENUMERATION;

   // USB core initialization
   usbCoreInit(interfaceReqHandler);

   // Line Coding Initialization
   sLineCoding.dteRate    = 9600;
   sLineCoding.charFormat = 0;
   sLineCoding.parityType = 0;
   sLineCoding.databits   = 8;

   // Initialize Data Buffers
//   ringBufferInit(sCdcOutData, CDC_BUFFER_SIZE);
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

   switch(sCdcState)
   {
   case WAITING_FOR_ENUMERATION:
      // Wait for USB Enumeration
      while (usbStateHack != uENUMERATED)
      {
         osDelay(500); // slow things down to allow other tasks to run
      };

      sCdcState = WAITING_FOR_ENUMERATION;
      break;
   case SET_LINE_CODING:
      if (usbFlagsHack & (1 << EP0))
      {
         usbFlagsHack &= ~(1 << EP0);

         usbCoreEpOutTransfer(EP0, (uint8_t*)&sLineCoding);
         usbCoreEpInTransfer(EP0,0,0);
      }
      break;
   case SET_CONTROL_LINE_STATE:
      usbCoreEpInTransfer(EP0,0,0);
      break;
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
   case GET_INTERFACE:
      usbCoreEpInTransfer(ep, &sAltInterface, 1);
      break;
   case GET_LINE_CODING:
      usbCoreEpInTransfer(ep, (uint8_t*)&sLineCoding, 7);
      break;
   case SET_LINE_CODING:
      sCdcState = SET_LINE_CODING;
      state = uDATA;
      break;
   case SET_CONTROL_LINE_STATE:
      sCdcState = SET_CONTROL_LINE_STATE;
      state = uSETUP;
      break;
   case LOADER_MODE:
//      ringBufferInit(sCdcOutData, CDC_BUFFER_SIZE);
      usbFlagsHack |= (1 << LOADER);
      sCdcOutData[0] = 0xFF;
      break;
   }

   return (state);
}
