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

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

#include "console.h"
#include "os.h"
#include "uart.h"
#include "utils.h"

#define WRITEQ_COUNT       (4)
#define INVALID_WRITEQ_IDX (-1)

typedef struct
{
   char    buf[CONSOLE_STRING_SIZE_MAX];
   uint8_t bytes; // Make it small, but there is a risk of issues if the max size changes

#define WRITEQ_STATE_EMPTY       (0)
#define WRITEQ_STATE_WAITING     (1)
#define WRITEQ_STATE_OUTPUTING   (2)
   uint8_t state;

} writeQ_t;

static int sInitialized = 0;
static writeQ_t sWriteQ[WRITEQ_COUNT];
static int sWriteQOutputIdx = INVALID_WRITEQ_IDX;

// Event Group Bits:
#define CONSOLE_READ_PENDING_BIT    (1 << 0)
#define CONSOLE_FLUSH_COMPLETE_BIT  (1 << 1)
#define CONSOLE_WRITE_FREED_BIT     (1 << 2)
static osSignalId sEvents;

static uint8_t sPendingStrings = 0;

static int  getNextWriteIdx(void);
static bool prepareWrite(void);
static void uartCallback(int event);

// ----------------------------------------------------------------------------
// External Functions
// ----------------------------------------------------------------------------

int consoleInit(uint32_t baud)
{
   int rc = -1;
   int i;

   if ((sEvents = osSignalGroupCreate()) == NULL)
   {
      // Failed to create the event group
   }
   else if (uartInit(baud, uartCallback) != 0)
   {
      // Failed to initialize the uart
   }
   else
   {
      for (i = 0; i < WRITEQ_COUNT; i++)
      {
         sWriteQ[i].state = WRITEQ_STATE_EMPTY;
      }
      sWriteQOutputIdx = 0;
      sPendingStrings = 0;

      sInitialized = 1;
      rc = 0;
   }

   return rc;
}

void consoleFlush(void)
{
   if (!sInitialized)
   {
      // Not initialized, ignore
   }
   else
   {
      osSignalWait(sEvents, CONSOLE_FLUSH_COMPLETE_BIT, WAIT_FOREVER, false);
   }
}

int consoleGetInput(char *buf, int len)
{
   int bytes = 0;

   if (!sInitialized)
   {
      // Not initialized, ignore
   }
   else
   {
      if (!sPendingStrings)
      {
         osSignalWait(sEvents, CONSOLE_READ_PENDING_BIT, WAIT_FOREVER, true);
      }

      bytes = uartRead((uint8_t*) buf, len);
      sPendingStrings --;
   }

   return bytes;
}

int consoleGetInputCnt(void)
{
   int cnt = 0;

   if (!sInitialized)
   {
      // Not initialized, ignore
   }
   else
   {
      cnt = uartReadCnt();
   }

   return cnt;
}

void consolePrintf(const char *format, ...)
{
   int idx;

   if (!sInitialized)
   {
      // Not initialized, ignore
   }
   else
   {
      if ((idx = getNextWriteIdx()) == INVALID_WRITEQ_IDX)
      {
         osSignalWait(sEvents, CONSOLE_WRITE_FREED_BIT, WAIT_FOREVER, true);
         idx = getNextWriteIdx();
      }

      if (idx == INVALID_WRITEQ_IDX)
      {
         // No buffer to print into, should not happen
      }
      else
      {
         va_list varg;

         va_start(varg, format);
         sWriteQ[idx].bytes = utilsVsnprintf(sWriteQ[idx].buf, CONSOLE_STRING_SIZE_MAX, format, varg);
         va_end(varg);

         if (sWriteQ[idx].bytes != 0)
         {
            sWriteQ[idx].state = WRITEQ_STATE_WAITING;
            prepareWrite();
         }
      }
   }
}

// ----------------------------------------------------------------------------
// Local Functions
// ----------------------------------------------------------------------------

static int getNextWriteIdx(void)
{
   int idx = INVALID_WRITEQ_IDX;
   int i, j;

   // Clear the freed bit in case the buffer pool is full
   osSignalClear(sEvents, CONSOLE_WRITE_FREED_BIT);

   for (i = 0, j = sWriteQOutputIdx; i < WRITEQ_COUNT; i++, j++)
   {
      if (j >= WRITEQ_COUNT)
      {
         j = 0;
      }

      if (sWriteQ[j].state == WRITEQ_STATE_EMPTY)
      {
         idx = j;
         break;
      }
   }

   return idx;
}

static bool prepareWrite(void)
{
   bool push = false;
   int idx;

   osDisableInterrupts();
   idx = sWriteQOutputIdx;
   if (sWriteQ[idx].state == WRITEQ_STATE_WAITING)
   {
      push = true;
   }
   osEnableInterrupts();

   if (push)
   {
      uartEchoEnable(false);
      sWriteQ[idx].state = WRITEQ_STATE_OUTPUTING;
      osSignalClear(sEvents, CONSOLE_FLUSH_COMPLETE_BIT);
      uartWrite((uint8_t*) sWriteQ[idx].buf, sWriteQ[idx].bytes);
   }

   return push;
}

static void uartCallback(int event)
{
   if (event & UART_EVENT_READ_BIT)
   {
      sPendingStrings++;
      osSignalSet(sEvents, CONSOLE_READ_PENDING_BIT);
   }
   if (event & UART_EVENT_WRITE_BIT)
   {
      register int idx = sWriteQOutputIdx;

      sWriteQ[idx].state = WRITEQ_STATE_EMPTY;
      idx ++;
      sWriteQOutputIdx = (idx >= WRITEQ_COUNT) ? 0 : (idx);
      osSignalSet(sEvents, CONSOLE_WRITE_FREED_BIT);

      if (!prepareWrite())
      {
         uartEchoEnable(true);
         osSignalSet(sEvents, CONSOLE_FLUSH_COMPLETE_BIT);
      }
   }
}
