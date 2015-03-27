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

#define CHARACTER_CR    (0x0D)
#define CHARACTER_LF    (0x0A)
#define CHARACTER_BS    (0x08)
#define CHARACTER_DEL   (0x7F)

// Event Group Bits:
#define USB_STATE_CHANGE_BIT  (1 << 0)
#define USB_CDC_PROCESS_BIT   (1 << 1)
#define USB_RX_DATA_BIT       (1 << 2)
#define USB_TX_DONE_BIT       (1 << 3)
static osSignalId sEvents;

static bool sEnumerated = false;
static bool sTxInProgress = false;

#define STAGING_BUFFER_SIZE  (64)
static utilsBuf_t sRxStagingBuf;
static uint8_t sRxStagingBuffer[STAGING_BUFFER_SIZE];
static uint8_t sTxStagingBuffer[STAGING_BUFFER_SIZE];

#ifdef USB_CONSOLE_ENABLED
static usbConsoleCb sCallback = NULL;
static bool    sEchoEnable = true;
static bool    sWriteCbNeeded = false;
static int     sPrevIdx = 0;

#define RX_BUFFER_SIZE  (128)
static utilsBuf_t sRxBuf;
static uint8_t sRxBuffer[RX_BUFFER_SIZE];

static void consoleProcess(int32_t events);
static void handleBackspace(uint8_t byte);
static void handleEcho(uint8_t cnt);
#endif

#ifdef USB_LOOPBACK_ENABLED
static void loopbackProcess(void);
#endif

static void taskCtrlIsrHandler(int event);
static void taskDataIsrHandler(uint8_t ep, uint8_t *data, uint16_t len);

// ----------------------------------------------------------------------------
// External Functions
// ----------------------------------------------------------------------------

void usbTaskEntry(void *pParameters)
{
   int32_t events = 0;

//   osDelay(5000); // this is a bit of a hack just to allow for console debugging

   // Init the circular Rx staging buffer
   sRxStagingBuf.size = STAGING_BUFFER_SIZE;
   sRxStagingBuf.buf = sRxStagingBuffer;
   utilsBufInit(&sRxStagingBuf);

#ifdef USB_CONSOLE_ENABLED
   // Init the circular Rx staging buffer
   sRxBuf.size = RX_BUFFER_SIZE;
   sRxBuf.buf = sRxBuffer;
   utilsBufInit(&sRxBuf);
#endif

   sEvents = osSignalGroupCreate();
   usbCdcInit(taskCtrlIsrHandler, taskDataIsrHandler);

   while (1)
   {
      if (!sEnumerated)
      {
         utilsBufReset(&sRxStagingBuf);
#ifdef USB_CONSOLE_ENABLED
         utilsBufReset(&sRxBuf);
#endif
         osSignalWait(sEvents, USB_STATE_CHANGE_BIT, WAIT_FOREVER, true);
      }
      else
      {
         if (events & USB_CDC_PROCESS_BIT || events & USB_STATE_CHANGE_BIT)
         {
            usbCdcEngine();
         }

#ifdef USB_CONSOLE_ENABLED
         consoleProcess(events);
#endif
#ifdef USB_LOOPBACK_ENABLED
         loopbackProcess();
#endif

         events = osSignalWait(sEvents, WAIT_ANY_SIGNAL, WAIT_FOREVER, true);
      }
   }
}

#ifdef USB_CONSOLE_ENABLED
int usbConsoleInit(usbConsoleCb callback)
{
   int rc = -1;

   if (callback == NULL)
   {
      // Invalid input
   }
   else if (sCallback != NULL)
   {
      // Already initialized
   }
   else
   {
      sCallback = callback;
      sEchoEnable = true;
      utilsBufReset(&sRxBuf);
      rc = 0;
   }

   return rc;
}

void usbConsoleEchoEnable(bool en)
{
   if (sCallback == NULL)
   {
      // Not initialized
   }
   else if (sEchoEnable != en)
   {
      sEchoEnable = en;
      if (en == false)
      {
         while (sTxInProgress)
            ;
      }
   }
}

int usbConsoleRead(uint8_t *buf, uint16_t len)
{
   int bytes = 0;

   if (buf == NULL || len == 0)
   {
      // Invalid input
   }
   else if (sCallback == NULL)
   {
      // Not initialized
   }
   else
   {
      int cnt;
      int i;

      cnt = utilsBufCount(&sRxBuf);
      bytes = (cnt >= len) ? len : cnt;

      for (i = 0; i < bytes; i++)
      {
         buf[i] = utilsBufPull(&sRxBuf);
         if (buf[i] == CHARACTER_CR)
         {
            buf[i] = 0;
            bytes = i + 1;
         }
      }
   }

   return bytes;
}

int usbConsoleReadCnt(void)
{
   int count = 0;

   if (sCallback == NULL)
   {
      // Not initialized
   }
   else
   {
      count = utilsBufCount(&sRxBuf);
   }

   return count;
}

void usbConsoleReadFlush(void)
{
   if (sCallback == NULL)
   {
      // Not initialized
   }
   else
   {
      utilsBufReset(&sRxBuf);
   }
}

int usbConsoleWrite(uint8_t *buf, uint16_t len)
{
   int rc = -1;

   if (sCallback == NULL)
   {
      // Not initialized
   }
   else if (buf == NULL || len == 0)
   {
      // Not initialized
   }
   else if (!sEnumerated || sTxInProgress)
   {
      // Not ready or already in process of handling a write
   }
   else
   {
      uint8_t cnt;

      cnt = (len > UINT8_MAX) ? UINT8_MAX : len;
      sTxInProgress = true;
      sWriteCbNeeded = true;
      usbDevEpTxTransfer(EP_DATA_TX, buf, cnt);

      rc = 0;
   }

   return rc;
}
#endif

// ----------------------------------------------------------------------------
// Local Functions
// ----------------------------------------------------------------------------

#ifdef USB_CONSOLE_ENABLED
static void consoleProcess(int32_t events)
{
   uint8_t byte;
   int cbCnt = 0;
   int cnt;
   int i, j;

   // Handle Tx completion
   if ((events & USB_TX_DONE_BIT) && sWriteCbNeeded)
   {
      sWriteCbNeeded = false;
      sCallback(USB_CONSOLE_EVENT_WRITE_BIT);
   }

   // Check for new Rx bytes
   cnt = utilsBufCount(&sRxStagingBuf);
   if (cnt > sPrevIdx)
   {
      for (i = sPrevIdx, j = 0; i < cnt; i++, j++)
      {
         byte = utilsBufPull(&sRxStagingBuf);

         utilsBufPush(&sRxBuf, byte);
         handleBackspace(byte);
         if (byte == CHARACTER_CR)
         {
            cbCnt++;  // Accumulate for callbacks
         }

         // FIXME: This may be an issue if the last echo has not completed Tx'ing yet
         sTxStagingBuffer[j] = byte;  // Accumulate for echoing
      }

      handleEcho(cnt);
   }

   // Handle all callbacks needed
   for (i = 0; i < cbCnt; i++)
   {
      sCallback(USB_CONSOLE_EVENT_READ_BIT);
   }
}

static void handleBackspace(uint8_t byte)
{
   if (byte == CHARACTER_BS || byte == CHARACTER_DEL)
   {
      utilsBufPop(&sRxBuf); // Remove the backspace character
      if (utilsBufPeek(&sRxBuf) != CHARACTER_CR)
      {
         utilsBufPop(&sRxBuf); // Remove the previous character
      }
   }
}

static void handleEcho(uint8_t cnt)
{
   if (sEchoEnable && !sTxInProgress)
   {
      sTxInProgress = true;
      usbDevEpTxTransfer(EP_DATA_TX, sTxStagingBuffer, cnt);
   }
}
#endif

#ifdef USB_LOOPBACK_ENABLED
static void loopbackProcess(void)
{
   int cnt;
   int i;

   cnt = utilsBufCount(&sRxStagingBuf);
   if (!sTxInProgress && cnt > 0)
   {
      for (i = 0; i < cnt; i++)
      {
         sTxStagingBuffer[i] = utilsBufPull(&sRxStagingBuf);
      }
      sTxInProgress = true;
      usbDevEpTxTransfer(EP_DATA_TX, sTxStagingBuffer, cnt);
   }
}
#endif

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
      sTxInProgress = false;
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
            utilsBufPush(&sRxStagingBuf, data[i]);
         }
         osSignalSet(sEvents, USB_RX_DATA_BIT);
      }
   }
}
