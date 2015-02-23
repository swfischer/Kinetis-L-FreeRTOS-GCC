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

#include "accel.h"
#include "led.h"
#include "ledTask.h"
#include "os.h"
#include "touch.h"

#define TOUCH_MIN   (10)
#define TOUCH_MAX  (600)

#define ACCEL_MAX (4096)

static int sMode = LED_MODE_GLOW;

static int sGlowDir = 1;
static int sGlowCnt = 1;
static int sGlowIdx = 0;

static int sTouchPercent = 0;

static int  accelMode(void);
static int  glowMode(void);
static int  touchMode(void);

// ----------------------------------------------------------------------------
// External Functions
// ----------------------------------------------------------------------------

void ledTaskEntry( void *pvParameters )
{
   int delay = 100; // 100ms

   while (1)
   {
      switch (sMode)
      {
      case LED_MODE_ACCEL:
         delay = accelMode();
         break;
      case LED_MODE_TOUCH:
      case LED_MODE_TOUCH_RED:
      case LED_MODE_TOUCH_GREEN:
      case LED_MODE_TOUCH_BLUE:
         delay = touchMode();
         break;
      case LED_MODE_GLOW:
      case LED_MODE_GLOW_RED:
      case LED_MODE_GLOW_GREEN:
      case LED_MODE_GLOW_BLUE:
      default:
         delay = glowMode();
         break;
      }

      osDelay( delay );
   }
}

void ledSetMode(int mode)
{
   if (mode != sMode)
   {
      ledSet(0, 0, 0); // Turn off the LEDs

      switch (mode)
      {
      case LED_MODE_GLOW:
         sGlowDir = 1;
         sGlowCnt = 1;
         sGlowIdx = 0;
         sMode = mode;
         break;
      case LED_MODE_GLOW_RED:
         sGlowDir = 1;
         sGlowCnt = 0;
         sGlowIdx = 0;
         sMode = mode;
         break;
      case LED_MODE_GLOW_GREEN:
         sGlowDir = 1;
         sGlowCnt = 1;
         sGlowIdx = 0;
         sMode = mode;
         break;
      case LED_MODE_GLOW_BLUE:
         sGlowDir = 1;
         sGlowCnt = 2;
         sGlowIdx = 0;
         sMode = mode;
         break;
      case LED_MODE_TOUCH:
      case LED_MODE_TOUCH_RED:
      case LED_MODE_TOUCH_GREEN:
      case LED_MODE_TOUCH_BLUE:
         sTouchPercent = 50;
         sMode = mode;
         break;
      case LED_MODE_ACCEL:
         sMode = mode;
         break;
      default:
         // Do nothing
         break;
      }
   }
}

// ----------------------------------------------------------------------------
// Local Functions
// ----------------------------------------------------------------------------

static int accelMode(void)
{
   int16_t x, y, z;

   accelGet(&x, &y, &z);

   x = (x < 0) ? -x : x;
   x = (x > ACCEL_MAX) ? ACCEL_MAX : x;
   y = (y < 0) ? -y : y;
   y = (y > ACCEL_MAX) ? ACCEL_MAX : y;
   z = (z < 0) ? -z : z;
   z = (z > ACCEL_MAX) ? ACCEL_MAX : z;
   
   x = ((((uint32_t) x) * 100) / ACCEL_MAX);
   y = ((((uint32_t) y) * 100) / ACCEL_MAX);
   z = ((((uint32_t) z) * 100) / ACCEL_MAX);

   ledSet(x, y, z);

   return 50; // 50ms
}

static int glowMode(void)
{
   switch (sGlowCnt)
   {
   case 0: ledRed(sGlowIdx);   break;
   case 1: ledGreen(sGlowIdx); break;
   case 2: ledBlue(sGlowIdx);  break;
   }
  
   sGlowIdx += sGlowDir;
   if (sGlowIdx >= 100)
   {
      sGlowDir = -1;
   }
   if (sGlowIdx <= 0)
   {
      sGlowDir = 1;
      
      if (sMode == LED_MODE_GLOW)
      {
         sGlowCnt++;
         if (sGlowCnt > 2)
         {
            sGlowCnt = 0;
         }
      }
   }

   return 10; // 10ms
}

static int touchMode(void)
{
   int d = touchData(TOUCH_PAD_A);

   if (d > TOUCH_MIN) // < TOUCH_MIN pretty much means no touch
   {
      if (d > TOUCH_MAX) // Empirical max
      {
         d = TOUCH_MAX;
      }
      
      sTouchPercent = (d * 100) / TOUCH_MAX;
   }

   switch (sMode)
   {
   case LED_MODE_TOUCH_RED:
      ledRed(sTouchPercent);
      break;
   case LED_MODE_TOUCH_GREEN:
      ledGreen(sTouchPercent);
      break;
   case LED_MODE_TOUCH:
   case LED_MODE_TOUCH_BLUE:
   default:
      ledBlue(sTouchPercent);
      break;
   }

   return 20; // 20ms
}

