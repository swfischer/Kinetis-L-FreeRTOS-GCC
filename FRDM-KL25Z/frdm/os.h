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

#ifndef _OS_H_
#define _OS_H_

#include <stdint.h>
#include <stdbool.h>

#include <FreeRTOS.h>
#include <event_groups.h>
#include <queue.h>
#include <semphr.h>
#include <task.h>
#include <timers.h>

#include "frdmCfg.h"

typedef enum
{ osOK                    =     0       ///< function completed; no event occurred.
, osEventSignal           =  0x08       ///< function completed; signal event occurred.
, osEventMessage          =  0x10       ///< function completed; message event occurred.
, osEventMail             =  0x20       ///< function completed; mail event occurred.
, osEventTimeout          =  0x40       ///< function completed; timeout occurred.
, osErrorParameter        =  0x80       ///< parameter error: a mandatory parameter was missing or specified an incorrect object.
, osErrorResource         =  0x81       ///< resource not available: a specified resource was not available.
, osErrorTimeoutResource  =  0xC1       ///< resource not available within given time: a specified resource was not available within the timeout period.
, osErrorISR              =  0x82       ///< not allowed in ISR context: the function cannot be called from interrupt service routines.
, osErrorISRRecursive     =  0x83       ///< function called multiple times from ISR with same object.
, osErrorPriority         =  0x84       ///< system cannot determine priority or thread has illegal priority.
, osErrorNoMemory         =  0x85       ///< system is out of memory: it was impossible to allocate or reserve memory for the operation.
, osErrorValue            =  0x86       ///< value of a parameter is out of range.
, osErrorOS               =  0xFF       ///< unspecified RTOS error: run-time error but no other error message fits.
, os_status_reserved      =  0x7FFFFFFF ///< prevent from enum down-size compiler optimization.
} osStatus; 

// 0 when not in an ISR, otherwise greater than 0
extern uint32_t osIsrDepth;

// ----------------------------------------------------------------------------
// Some global debug routines

static inline void dbgPutChar(int c)
{
   while (!(UART0_S1 & UART_S1_TDRE_MASK))
      ;
   UART0_D = c;
}

static inline void dbgPutData(uint32_t d, int cnt)
{
   static const char tohex[] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
   int i;
   char c;
   
   for (i = (cnt - 1); i >= 0; i--)
   {
      while (!(UART0_S1 & UART_S1_TDRE_MASK))
         ;
      c = tohex[((d >> (4 * i)) & 0xf)];
      UART0_D = c;
   }

   while (!(UART0_S1 & UART_S1_TDRE_MASK))
      ;
   UART0_D = '\r';

   while (!(UART0_S1 & UART_S1_TDRE_MASK))
      ;
   UART0_D = '\n';
}

static inline void dbgPutByte(uint8_t b)
{
   dbgPutData(b, 2);
}

static inline void dbgPutWord(uint32_t w)
{
   dbgPutData(w, 8);
}

// ----------------------------------------------------------------------------
// OS ISR Handlers

extern void xPortPendSVHandler(void);
extern void xPortSysTickHandler(void);

static inline void osPendSvIsrHandler(void)
{
   xPortPendSVHandler();
}

static inline void osTickIsrHandler(void)
{
   xPortSysTickHandler();
}

// ----------------------------------------------------------------------------
// Interrupts

static inline void osDisableInterrupts(void)
{
   taskDISABLE_INTERRUPTS();
}

static inline void osEnableInterrupts(void)
{
   taskENABLE_INTERRUPTS();
}

// ----------------------------------------------------------------------------
// Kernel

extern bool osKernelStarted;

static inline osStatus osKernelInitialize(void)
{
   return osOK;
}
 
static inline osStatus osKernelStart(void)
{
   // Does not normally return
   vTaskStartScheduler();

   return osOK;
}
 
static inline int32_t osKernelRunning(void)
{
   return (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) ? 0 : 1;
}

extern uint32_t osKernelSysTick(void);

// ----------------------------------------------------------------------------
// MUTEX

typedef struct os_mutex_def
{
  uint32_t noData;
} osMutexDef_t;

typedef SemaphoreHandle_t osMutexId;

static inline osMutexId osMutexCreate(osMutexDef_t *mutex_def)
{
   return xSemaphoreCreateMutex();
}

static inline osStatus osMutexDelete(osMutexId mutex)
{
   vSemaphoreDelete(mutex);
   return osOK;
}

#define WAIT_FOREVER  (portMAX_DELAY)
extern osStatus osMutexWait(osMutexId mutex, uint32_t millisec);
extern osStatus osMutexRelease(osMutexId mutex);

// ----------------------------------------------------------------------------
// SIGNAL - not correctly implemented, but didn't want the overhead of tack specific data

typedef EventGroupHandle_t osSignalId;

static inline osSignalId osSignalGroupCreate(void)
{
   return xEventGroupCreate();
}

#define WAIT_ANY_SIGNAL (0)

extern int32_t osSignalClear(osSignalId signal_id, int32_t signal);
extern int32_t osSignalSet(osSignalId signal_id, int32_t signal);
extern int32_t osSignalWait(osSignalId signal_id, int32_t signals, uint32_t millisec, bool clear);

// ----------------------------------------------------------------------------
// THREAD (really Task)

typedef enum
{ osPriorityIdle        = 0      ///< priority: idle (lowest)
, osPriorityLow         = 1      ///< priority: low
, osPriorityBelowNormal = 2      ///< priority: below normal
, osPriorityNormal      = 3      ///< priority: normal (default)
, osPriorityAboveNormal = 4      ///< priority: above normal
, osPriorityHigh        = 5      ///< priority: high 
, osPriorityRealtime    = 6      ///< priority: realtime (highest)
, osPriorityError       = 0x84   ///< system cannot determine priority or thread has illegal priority
} osPriority;

typedef void (*os_pthread) (void /*const*/ *argument); 

typedef struct os_thread_def
{
  os_pthread   pthread;    ///< start address of thread function
  osPriority   tpriority;  ///< initial thread priority
  uint32_t     stacksize;  ///< stack size requirements in bytes
  char*        name;
} osThreadDef_t;

typedef TaskHandle_t osThreadId;

extern osThreadId osThreadCreate(osThreadDef_t *thread_def, void /*const*/ *argument);

static inline osStatus osThreadTerminate(osThreadId thread_id)
{
   vTaskDelete(thread_id);
   return osOK;
}

static inline osStatus osThreadYield(void)
{
   taskYIELD();
   return osOK;
}

static inline osStatus osDelay(uint32_t millisec)
{
   // ticks = millisec / ms per tick
   vTaskDelay( (millisec / MS_PER_TICK) );
   return osOK;
}

// ----------------------------------------------------------------------------
// TIMER

typedef enum
{ osTimerOnce     = 0   ///< one-shot timer 
, osTimerPeriodic = 1   ///< repeating timer 
} os_timer_type; 

typedef void (*os_ptimer) (void /*const*/ *argument); 

typedef struct os_timer_def
{
  os_ptimer ptimer;  ///< start address of a timer function
  uint32_t  millisec;
  char*     name;
} osTimerDef_t;

typedef TimerHandle_t osTimerId;

extern osTimerId osTimerCreate(osTimerDef_t *timer_def, os_timer_type type, void *argument);
extern osStatus osTimerStart(osTimerId timer_id, uint32_t millisec);
extern osStatus osTimerStop(osTimerId timer_id);

static inline osStatus osTimerDelete(osTimerId timer_id)
{
   return (xTimerDelete(timer_id, 0) == pdPASS) ? osOK : osErrorOS;
}

// ----------------------------------------------------------------------------

#endif // _OS_H_
