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
#include "utilsBuf.h"

// Event Group Bits:
#define USB_STATE_CHANGE_BIT  (1 << 0)
#define USB_CDC_PROCESS_BIT   (1 << 1)
#define USB_RX_DATA_BIT       (1 << 2)
#define USB_TX_DONE_BIT       (1 << 3)
static osSignalId sEvents;

static bool sEnumerated = false;
static bool sTxInProgress = false;

#define IN_BUFFER_SIZE  (64)
static utilsBuf_t sInBuf;
static uint8_t sInBuffer[IN_BUFFER_SIZE];
static uint8_t sStagingBuffer[IN_BUFFER_SIZE];

static void loopback(void);
static void taskCtrlIsrHandler(int event);
static void taskDataIsrHandler(uint8_t ep, uint8_t *data, uint16_t len);

// ----------------------------------------------------------------------------
// External Functions
// ----------------------------------------------------------------------------

void usbTaskEntry(void *pParameters)
{
   int32_t events = 0;

//   osDelay(5000); // this is a bit of a hack just to allow for console debugging

   sInBuf.size = IN_BUFFER_SIZE;
   sInBuf.buf = sInBuffer;
   utilsBufInit(&sInBuf);

   sEvents = osSignalGroupCreate();
   usbCdcInit(taskCtrlIsrHandler, taskDataIsrHandler);

   while (1)
   {
      if (!sEnumerated)
      {
         utilsBufReset(&sInBuf);
         osSignalWait(sEvents, USB_STATE_CHANGE_BIT, WAIT_FOREVER, true);
      }
      else
      {
         if (events & USB_CDC_PROCESS_BIT || events & USB_STATE_CHANGE_BIT)
         {
            usbCdcEngine();
         }
         if (events & USB_TX_DONE_BIT)
         {
            sTxInProgress = false;
         }

         loopback();

         events = osSignalWait(sEvents, WAIT_ANY_SIGNAL, WAIT_FOREVER, true);
      }
   }
}

// ----------------------------------------------------------------------------
// Local Functions
// ----------------------------------------------------------------------------

static void loopback(void)
{
   int cnt;
   int i;

   cnt = utilsBufCount(&sInBuf);
   if (!sTxInProgress && cnt > 0)
   {
      for (i = 0; i < cnt; i++)
      {
         sStagingBuffer[i] = utilsBufPull(&sInBuf);
      }
      sTxInProgress = true;
      usbDevEpTxTransfer(EP_DATA_TX, sStagingBuffer, cnt);
   }
}

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
   case USB_CTRL_EVENT_TX_DONE:
      osSignalSet(sEvents, USB_TX_DONE_BIT);
      break;
   };
}

static void taskDataIsrHandler(uint8_t ep, uint8_t *data, uint16_t len)
{
   int i;

   if (ep == EP_DATA_RX)
   {
      usbDevEpReset(EP_DATA_RX);
      usbDevEpControl(EP_DATA_RX, USB_CTRL_SIE);

      if (len > 0)
      {
         for (i = 0; i < len; i++)
         {
            utilsBufPush(&sInBuf, data[i]);
         }
         osSignalSet(sEvents, USB_RX_DATA_BIT);
      }
   }
}
