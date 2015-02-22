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
// This code is based on Andrew Payne's work.
//
//  Copyright (c) 2012-2013 Andrew Payne <andy@payne.org>
// ----------------------------------------------------------------------------

#include <stdint.h>

#include "clk.h"
#include "irq.h"
#include "MKL25Z4.h"
#include "os.h"

#define NUM_CHANNELS    (16)

#define TOUCH_SCAN_TIME (50) // ms

static volatile uint16_t sRawCounts[NUM_CHANNELS];
static volatile uint16_t sBaseCounts[NUM_CHANNELS];
static uint32_t sEnableMask; // Bitmask of enabled channels
static osTimerId sStartTimer;

inline static uint16_t scanData(void);
inline static void scanStart(int channel);
static void startTimerCb(void* argument);
static void tsiIrqHandler(uint32_t cookie);

// ----------------------------------------------------------------------------
// External Functions
// ----------------------------------------------------------------------------

// Initialize touch input
void touchInit(uint32_t channelMask)
{
   int i;

   // Turn on clock gating for TSI and configure touch input
   clkEnable(CLK_TSI);

   TSI0_GENCS |= ( TSI_GENCS_ESOR_MASK    // Enable end of scan interrupt
                 | TSI_GENCS_MODE(0)      // Capacitive sensing
                 | TSI_GENCS_REFCHRG(4)   // Reference charge 4 uA
                 | TSI_GENCS_DVOLT(0)     // Voltage rails
                 | TSI_GENCS_EXTCHRG(7)   // External osc charge
                 | TSI_GENCS_PS(4)        // Pre-scalar divide by 4
                 | TSI_GENCS_NSCN(11)     // Scans per electrode
                 | TSI_GENCS_TSIIEN_MASK  // Input interrupt enable
                 | TSI_GENCS_STPE_MASK    // Enable in STOP mode
                 );

   TSI0_GENCS |= TSI_GENCS_TSIEN_MASK;

   // Read initial (baseline) values for each enabled channel
   sEnableMask = channelMask;
   for (i = (NUM_CHANNELS - 1); i >= 0; i--)
   {
      if ((1 << i) & sEnableMask)
      {
         scanStart(i);
         while (!(TSI0_GENCS & TSI_GENCS_EOSF_MASK)) // Wait until done
         ;

         sBaseCounts[i] = scanData();
      }
   }

   // Enable TSI interrupts and start the scan timer
   irqRegister(INT_TSI0, tsiIrqHandler, 0);
   irqEnable(INT_TSI0);

   {
      osTimerDef_t tDef;
      
      tDef.ptimer = startTimerCb;
      tDef.millisec = TOUCH_SCAN_TIME;
      tDef.name = "touch";

      sStartTimer = osTimerCreate(&tDef, osTimerPeriodic, 0);
      osTimerStart(sStartTimer, TOUCH_SCAN_TIME);
   }
}

// Get current touch input value (normalized to baseline) for specified
// input channel
int touchData(int channel)
{
   return sRawCounts[channel] - sBaseCounts[channel];
}

// ----------------------------------------------------------------------------
// Local Functions
// ----------------------------------------------------------------------------

// Return scan data for most recent scan
inline static uint16_t scanData(void)
{
   TSI0_GENCS |= TSI_GENCS_EOSF_MASK;
   return TSI0_DATA & TSI_DATA_TSICNT_MASK;
}

// Initiate a touch scan on the given channel
inline static void scanStart(int channel)
{
   TSI0_DATA = TSI_DATA_TSICH(channel) | TSI_DATA_SWTS_MASK;
}

static void startTimerCb( TimerHandle_t pxTimer )
{
   uint32_t channel = (TSI0_DATA & TSI_DATA_TSICH_MASK) >> TSI_DATA_TSICH_SHIFT;

   // Start a new scan on next enabled channel
   for(;;)
   {
      channel = (channel + 1) % NUM_CHANNELS;
      if ((1 << channel) & sEnableMask)
      {
         scanStart(channel);
         break;
      }
   }
}

// Touch input interrupt handler
static void tsiIrqHandler(uint32_t cookie)
{
   // Save data for channel
   uint32_t channel = (TSI0_DATA & TSI_DATA_TSICH_MASK) >> TSI_DATA_TSICH_SHIFT;
   sRawCounts[channel] = scanData();
}
