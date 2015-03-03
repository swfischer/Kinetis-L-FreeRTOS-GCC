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

#include "gpio.h"
#include "kinetis.h"

static const GPIO_MemMapPtr sGpioBase[] =
{ PTA_BASE_PTR
, PTB_BASE_PTR
, PTC_BASE_PTR
, PTD_BASE_PTR
, PTE_BASE_PTR
};

static const PORT_MemMapPtr sPortBase[] =
{ PORTA_BASE_PTR
, PORTB_BASE_PTR
, PORTC_BASE_PTR
, PORTD_BASE_PTR
, PORTE_BASE_PTR
};

// ----------------------------------------------------------------------------
// External Functions
// ----------------------------------------------------------------------------

void gpioClearInterrupt(uint8_t gpioID)
{
   PORT_ISFR_REG(sPortBase[GPIO_ID_TO_PORT(gpioID)]) |= (1 << GPIO_ID_TO_PIN(gpioID));
}

GPIO_MemMapPtr gpioIdToBase(uint8_t gpioID)
{
   return sGpioBase[GPIO_ID_TO_PORT(gpioID)];
}

int gpioInputGet(uint8_t gpioID)
{
   return gpioInputIsHigh(gpioID);
}

int gpioInputIsHigh(uint8_t gpioID)
{
   uint32_t data;

   data = GPIO_PDIR_REG(sGpioBase[GPIO_ID_TO_PORT(gpioID)]) & (1 << GPIO_ID_TO_PIN(gpioID));

   return (data) ? 1 : 0;
}

int gpioInputIsLow(uint8_t gpioID)
{
   uint32_t data;

   data = GPIO_PDIR_REG(sGpioBase[GPIO_ID_TO_PORT(gpioID)]) & (1 << GPIO_ID_TO_PIN(gpioID));

   return (data) ? 0 : 1;
}

void gpioOutputHigh(uint8_t gpioID)
{
   GPIO_PSOR_REG(sGpioBase[GPIO_ID_TO_PORT(gpioID)]) |= (1 << GPIO_ID_TO_PIN(gpioID));
}

void gpioOutputLow(uint8_t gpioID)
{
   GPIO_PCOR_REG(sGpioBase[GPIO_ID_TO_PORT(gpioID)]) |= (1 << GPIO_ID_TO_PIN(gpioID));
}

void gpioOutputSet(uint8_t gpioID, bool value)
{
   if (value)
   {
      gpioOutputHigh(gpioID);
   }
   else
   {
      gpioOutputLow(gpioID);
   }
}

void gpioOutputToggle(uint8_t gpioID)
{
   GPIO_PTOR_REG(sGpioBase[GPIO_ID_TO_PORT(gpioID)]) |= (1 << GPIO_ID_TO_PIN(gpioID));
}

void gpioSetAsInput(uint8_t gpioID)
{
   GPIO_PDDR_REG(sGpioBase[GPIO_ID_TO_PORT(gpioID)]) &= ~(1 << GPIO_ID_TO_PIN(gpioID));
}

void gpioSetAsOutput(uint8_t gpioID)
{
   GPIO_PDDR_REG(sGpioBase[GPIO_ID_TO_PORT(gpioID)]) |= (1 << GPIO_ID_TO_PIN(gpioID));
}
