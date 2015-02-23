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
#include <stdbool.h>

#include "os.h"

uint32_t osIsrDepth = 0;

// ----------------------------------------------------------------------------
// External Functions
// ----------------------------------------------------------------------------

uint32_t osKernelSysTick(void)
{
   uint32_t tick;

   if (osIsrDepth == 0)
   {
      tick = xTaskGetTickCount();
   }
   else
   {
      tick = xTaskGetTickCountFromISR();
   }

   return tick;
}

osStatus osMutexWait(osMutexId mutex, uint32_t millisec)
{
   uint32_t ticks = millisec / MS_PER_TICK;
   osStatus rc = osOK;

   if (osIsrDepth == 0)
   {
      rc = (xSemaphoreTake(mutex, ticks) == pdTRUE) ? osOK : osEventTimeout;
   }
   else
   {
      rc = (xSemaphoreTakeFromISR(mutex, NULL) == pdTRUE) ? osOK : osEventTimeout;
   }

   return rc;
}

osStatus osMutexRelease(osMutexId mutex)
{
   if (osIsrDepth == 0)
   {
      xSemaphoreGive(mutex);
   }
   else
   {
      xSemaphoreGiveFromISR(mutex, NULL);
   }

   return osOK;
}

// A bit of a hack since CMSIS associates signals with threads
int32_t osSignalClear(osSignalId signal_id, int32_t signal)
{
   int32_t prev = 0;

   if (osIsrDepth == 0)
   {
      prev = xEventGroupClearBits(signal_id, signal);
   }
   else
   {
      // Need pending function calls for this
      prev = xEventGroupClearBitsFromISR(signal_id, signal);
   }

   return prev;
}

// A bit of a hack since CMSIS associates signals with threads
int32_t osSignalSet(osSignalId signal_id, int32_t signal)
{
   int32_t prev = 0;

   if (osIsrDepth == 0)
   {
      prev = xEventGroupGetBits(signal_id);
      xEventGroupSetBits(signal_id, signal);
   }
   else
   {
      // Need pending function calls for this
      prev = xEventGroupGetBitsFromISR(signal_id);
      xEventGroupSetBitsFromISR(signal_id, signal, NULL);
   }

   return prev;
}

// A bit of a hack since CMSIS associates signals with threads
// Returns the previous signal bits
// The "clear" flag is to request clearing of the found signals or not
int32_t osSignalWait(osSignalId signal_id, int32_t signals, uint32_t millisec, bool clear)
{
   uint32_t ticks = millisec / MS_PER_TICK;
   bool waitForAll = true;

   if (signals == 0)
   {
      signals = 0xff; // 8 event bits by default
      waitForAll = false;
   }

   return xEventGroupWaitBits(signal_id, signals, clear, waitForAll, ticks);
}

osThreadId osThreadCreate(osThreadDef_t *thread_def, void /*const*/ *argument)
{
   osThreadId t;
   BaseType_t x;

   x = xTaskCreate( (TaskFunction_t) thread_def->pthread
                  , thread_def->name
                  , thread_def->stacksize
                  , argument
                  , thread_def->tpriority
                  , &t
                  );
   return (x = pdPASS) ? t : NULL;
}

osTimerId osTimerCreate(osTimerDef_t *timer_def, os_timer_type type, void *argument)
{
   return xTimerCreate( timer_def->name
                      , timer_def->millisec // Should be ticks
                      , (type == osTimerPeriodic) ? 1 : 0
                      , argument
                      , timer_def->ptimer
                      );
}

osStatus osTimerStart(osTimerId timer_id, uint32_t millisec)
{
   uint32_t ticks = millisec / MS_PER_TICK;
   osStatus rc = osOK;

   if (osIsrDepth == 0)
   {
      if (xTimerChangePeriod(timer_id, ticks, 0) != pdPASS)
      {
         rc = osErrorOS;
      }
      else
      {
         rc = (xTimerStart(timer_id, 0) == pdPASS) ? osOK : osErrorOS;
      }
   }
   else
   {
      if (xTimerChangePeriodFromISR(timer_id, ticks, 0) != pdPASS)
      {
         rc = osErrorOS;
      }
      else
      {
         rc = (xTimerStartFromISR(timer_id, 0) == pdPASS) ? osOK : osErrorOS;
      }
   }

   return rc;
}

osStatus osTimerStop(osTimerId timer_id)
{
   osStatus rc = osOK;

   if (osIsrDepth == 0)
   {
      rc = (xTimerStop(timer_id, 0) == pdPASS) ? osOK : osErrorOS;
   }
   else
   {
      rc = (xTimerStopFromISR(timer_id, 0) == pdPASS) ? osOK : osErrorOS;
   }

   return rc;
}
