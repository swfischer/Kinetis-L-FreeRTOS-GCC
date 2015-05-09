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

#include "adc.h"
#include "clk.h"
#include "kinetis.h"
#include "os.h"
#include "utils.h"

static osMutexId sAdc0Lock = NULL; // Used to control ADC0 use

static void adcCal(ADC_MemMapPtr base);
static void adcInit(ADC_MemMapPtr base);
static uint16_t adcPerformRead(ADC_MemMapPtr base, int channel);

// ----------------------------------------------------------------------------
// External Functions
// ----------------------------------------------------------------------------

int adcDeviceInit(adcDevice_t *dev, ADC_MemMapPtr base)
{
   int rc = -1;

   if (dev == NULL || base == NULL || base != ADC0_BASE_PTR)
   {
      // Invalid parameter(s)
   }
   else
   {
      dev->base = base;
      dev->acquired = 0;

      if (sAdc0Lock == NULL)
      {
         sAdc0Lock = osMutexCreate(NULL);
      }

      rc = 0;
   }

   return rc;
}

int adcAcquire(adcDevice_t *dev, uint32_t waitMs)
{
   clkGate_t clk;
   osStatus status;
   int rc = -1;

   status = osMutexWait(sAdc0Lock, waitMs);

   if (status == osOK)
   {
      clk = CLK_ADC0;
      if (!clkIsEnabled(clk))
      {
         // Enable clocks
         clkEnable(clk);
         // Init device
         adcInit(dev->base);
      }

      dev->acquired = 1;
      rc = 0;
   }

   return rc;
}

void adcRelease(adcDevice_t *dev)
{
   if (dev->acquired)
   {
#if ADC_KEEP_CLKS_ENABLED
      clkDisable(CLK_ADC0);
#endif
      dev->acquired = 0;

      osMutexRelease(sAdc0Lock);
   }
}

uint16_t adcRead(adcDevice_t *dev, int channel)
{
   uint16_t data = 0;

   if (dev->acquired)
   {
      if (channel < 0 || channel >= 32)
      {
         // Ignore, invalid channel
      }
      else
      {
         data = adcPerformRead(dev->base, channel);
      }
   }

   return data;   
}

// ----------------------------------------------------------------------------
// Local Functions
// ----------------------------------------------------------------------------

static void adcCal(ADC_MemMapPtr base)
{
   uint16_t cal;

   ADC_SC1_REG(base, 0) = ADC_SC1_ADCH_MASK;
   ADC_SC3_REG(base) |= ADC_SC3_CAL_MASK;

   while (!(ADC_SC1_REG(base, 0) & ADC_SC1_COCO_MASK))
   {
      utilsHwDelay(1);
   }

   // Perform plus-side calibration
   cal = ADC_CLP0_REG(base) + ADC_CLP1_REG(base) + ADC_CLP2_REG(base)
         + ADC_CLP3_REG(base) + ADC_CLP4_REG(base) + ADC_CLPS_REG(base);
   cal = cal / 2;
   cal |= 0x8000;
   ADC_PG_REG(base) = cal;

   // Perform minus-side calibration
   cal = ADC_CLM0_REG(base) + ADC_CLM1_REG(base) + ADC_CLM2_REG(base)
         + ADC_CLM3_REG(base) + ADC_CLM4_REG(base) + ADC_CLMS_REG(base);
   cal = cal / 2;
   cal |= 0x8000;
   ADC_MG_REG(base) = cal;
}

static void adcInit(ADC_MemMapPtr base)
{
   ADC_SC1_REG(base, 0) = ADC_SC1_ADCH_MASK;
   ADC_CFG1_REG(base) = ADC_CFG1_ADICLK(1) | ADC_CFG1_ADIV(0) | ADC_CFG1_MODE(3);
   ADC_SC2_REG(base) = ADC_SC2_REFSEL(0);
   ADC_SC3_REG(base) = 0;
   utilsHwDelay(1);
   adcCal(base);
}

static uint16_t adcPerformRead(ADC_MemMapPtr base, int channel)
{
   uint16_t data;

   ADC_SC1_REG(base, 0) = channel;

   while (!(ADC_SC1_REG(base, 0) & ADC_SC1_COCO_MASK))
      ;

   data = ADC_R_REG(base, 0);

   return data;
}

