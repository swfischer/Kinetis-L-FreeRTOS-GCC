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

#include <stdbool.h>
#include <stdint.h>

#include "irq.h"
#include "kinetis.h"
#include "os.h"

#define IRQ_MIN  (0)
#define IRQ_MAX  (IRQ(INT_VIRTUAL_LAST))

#define NVIC_BIT(x)       ((uint32_t)(1 << x))

static irqCallback sCallback[IRQ_MAX];
static uint32_t    sCookie[IRQ_MAX];
static uint32_t    sVirtualEnables[2];  // Just ports A & D
static osMutexId sLock;  // Used to protect callbacks, cookies, and enables

static void disable(int irq);
static void enable(int irq);
static int  isEnabled(int irq);

static void virtualInit(void);
static void virtualDisable(int irq);
static void virtualEnable(int irq);

static void virtualPortHandler(PORT_MemMapPtr base, int firstIrq);
static void virtualPortAHandler(uint32_t cookie);
static void virtualPortDHandler(uint32_t cookie);

// ----------------------------------------------------------------------------
// External Functions
// ----------------------------------------------------------------------------

void irqInit(void)
{
   int i;

   for (i = 0; i < IRQ_MAX; i++)
   {
      sCallback[i] = NULL;
   }

   // Must be called before virtualInit() since irqRegister() called from within
   sLock = osMutexCreate(NULL);

   virtualInit();
}

void irqHandler(void)
{
   int irq;

   irq = IRQ((SCB_ICSR & SCB_ICSR_VECTACTIVE_MASK));

   if (irq < IRQ_MIN || irq >= IRQ_MAX)
   {
      // Invalid IRQ, should never occur, ignore
   }
   else if (sCallback[irq] != NULL)
   {
      osIsrDepth++;
      sCallback[irq](sCookie[irq]);
      osIsrDepth--;
   }
   else
   {
      while (1)
         ;
   }
}

int irqRegister(intIdx_t intIdx, irqCallback callback, uint32_t cookie)
{
   int success = -1;
   int irq = IRQ(intIdx);

   if (irq < IRQ_MIN || irq >= IRQ_MAX)
   {
      // Invalid IRQ
   }
   else if (callback == NULL)
   {
      // Invalid callback
   }
   else
   {
      osMutexWait(sLock, WAIT_FOREVER);

      if (sCallback[irq] != NULL)
      {
         // Already used
      }
      else
      {
         sCallback[irq] = callback;
         sCookie[irq]   = cookie;
         success = 0;
      }

      osMutexRelease(sLock);
   }

   return success;
}

void irqUnregister(intIdx_t intIdx)
{
   int irq = IRQ(intIdx);

   if (irq < IRQ_MIN || irq >= IRQ_MAX)
   {
      // Invalid IRQ
   }
   else if (sCallback[irq] == NULL)
   {
      // Not registered
   }
   else
   {
      osMutexWait(sLock, WAIT_FOREVER);

      // Make sure the interrupt is disabled
      if (isEnabled(irq))
      {
         disable(irq);
      }

      sCallback[irq] = NULL;

      osMutexRelease(sLock);
   }
}

void irqEnable(intIdx_t intIdx)
{
   int irq = IRQ(intIdx);

   if (irq < IRQ_MIN || irq >= IRQ_MAX)
   {
      // Invalid IRQ
   }
   else if (sCallback[irq] == NULL)
   {
      // Not registered
   }
   else if (isEnabled(irq))
   {
      // Already enabled
   }
   else
   {
      enable(irq);
   }
}

void irqDisable(intIdx_t intIdx)
{
   int irq = IRQ(intIdx);

   if (irq < IRQ_MIN || irq >= IRQ_MAX)
   {
      // Invalid IRQ
   }
   else if (!isEnabled(irq))
   {
      // Not enabled
   }
   else
   {
      disable(irq);
   }
}

// ----------------------------------------------------------------------------
// Local Functions
// ----------------------------------------------------------------------------

static void disable(int irq)
{
   if (irq >= IRQ(INT_VIRTUAL_FIRST))
   {
      virtualDisable(irq);
   }
   else
   {
      NVIC_ICER = NVIC_BIT(irq);
   }
}

static void enable(int irq)
{
   if (irq >= IRQ(INT_VIRTUAL_FIRST))
   {
      virtualEnable(irq);
   }
   else
   {
      NVIC_ICPR = NVIC_BIT(irq);
      NVIC_ISER = NVIC_BIT(irq);
   }
}

static int isEnabled(int irq)
{
   int enabled = 0;

   if (irq >= IRQ(INT_VIRTUAL_FIRST))
   {
      enabled = (sVirtualEnables[PORT_IRQ_TO_PORT(irq)] & (1 << PORT_IRQ_TO_PIN(irq)));
   }
   else
   {
      enabled = (NVIC_ISER & NVIC_BIT(irq));
   }

   return (enabled) ? 1 : 0;
}

static void virtualInit(void)
{
   sVirtualEnables[portToVirtualPortIdx(PORT_A)] = 0;
   irqRegister(INT_PORTA, virtualPortAHandler, 0);
   sVirtualEnables[portToVirtualPortIdx(PORT_D)] = 0;
   irqRegister(INT_PORTD, virtualPortDHandler, 0);
}

static void virtualDisable(int irq)
{
   int port = PORT_IRQ_TO_PORT(irq);

   sVirtualEnables[port] &= ~(1 << PORT_IRQ_TO_PIN(irq));

   if (sVirtualEnables[port] == 0)
   {
      if (port == PORT_D)
      {
         disable(IRQ(INT_PORTD));
      }
      else
      {
         disable(IRQ(INT_PORTA));
      }
   }
}

static void virtualEnable(int irq)
{
   int port = PORT_IRQ_TO_PORT(irq);

   if (sVirtualEnables[port] == 0)
   {
      if (port == PORT_D)
      {
         enable(IRQ(INT_PORTD));
      }
      else
      {
         enable(IRQ(INT_PORTA));
      }
   }

   sVirtualEnables[port] |= (1 << PORT_IRQ_TO_PIN(irq));
}

static void virtualPortHandler(PORT_MemMapPtr base, int firstIrq)
{
   register uint32_t interrupts;
   int irq;
   int i;

   interrupts = PORT_ISFR_REG(base);
   for ( i = 0; i < 32; i++ )
   {
      if ( (interrupts & (1 << i)) )
      {
         irq = firstIrq + i;

         if (sCallback[irq])
         {
            sCallback[irq](sCookie[irq]);
         }
      }
   }
}

static void virtualPortAHandler(uint32_t cookie)
{
   virtualPortHandler(PORTA_BASE_PTR, IRQ(INT_VIRTUAL_PORT_A_FIRST));
}

static void virtualPortDHandler(uint32_t cookie)
{
   virtualPortHandler(PORTD_BASE_PTR, IRQ(INT_VIRTUAL_PORT_D_FIRST));
}

