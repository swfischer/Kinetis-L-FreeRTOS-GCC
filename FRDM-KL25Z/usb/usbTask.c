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

#include <stdint.h>

#include "clk.h"
#include "os.h"
#include "usbCdc.h"
#include "usbDev.h"
#include "usbTask.h"

// Event Group Bits:
#define USB_STATE_CHANGE_BIT  (1 << 0)
#define USB_RX_DATA_BIT       (1 << 1)
#define USB_CDC_PROCESS_BIT   (1 << 2)
static osSignalId sEvents;

static bool sEnumerated = false;

static void taskCtrlIsrHandler(int event);
static void taskDataIsrHandler(uint8_t ep, uint8_t *data, uint16_t len);

// ----------------------------------------------------------------------------
// External Functions
// ----------------------------------------------------------------------------

void usbTaskEntry(void *pParameters)
{
   osDelay(5000); // this is a bit of a hack just to allow for console debugging

   sEvents = osSignalGroupCreate();
   usbCdcInit(taskCtrlIsrHandler, taskDataIsrHandler);

   while (1)
   {
      if (!sEnumerated)
      {
         osSignalWait(sEvents, USB_STATE_CHANGE_BIT, WAIT_FOREVER, true);
      }
      else
      {
         usbCdcEngine();

         osSignalWait(sEvents, USB_STATE_CHANGE_BIT | USB_CDC_PROCESS_BIT, WAIT_FOREVER, true);
      }
   }
}

// ----------------------------------------------------------------------------
// Local Functions
// ----------------------------------------------------------------------------

static void taskCtrlIsrHandler(int event)
{
   switch (event)
   {
   case USB_CTRL_EVENT_RESET:
      sEnumerated = false;
      osSignalSet(sEvents, USB_STATE_CHANGE_BIT);
      break;
   case USB_CTRL_EVENT_ENUMERATION:
      sEnumerated = true;
      osSignalSet(sEvents, USB_STATE_CHANGE_BIT);
      break;
   case USB_CTRL_EVENT_REQUEST:
      osSignalSet(sEvents, USB_CDC_PROCESS_BIT);
      break;
   };
}

static void taskDataIsrHandler(uint8_t ep, uint8_t *data, uint16_t len)
{
   if (ep == EP_OUT)
   {
      usbEP_Reset(EP_OUT);
      usbSIE_CONTROL(EP_OUT);

      // Send it back to the PC
      usbDevEpTxTransfer(EP_IN, data, len);
   }
}
