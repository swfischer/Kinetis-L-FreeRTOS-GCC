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
//
// This driver supports outputing PWM signals for up to 16 servos at a time.
//
// Assumptions:
//
// a) A 40ms cycle time is acceptable to all servos.
// b) The minimum pulse time is 1ms.
// c) The maximum pulse time is 2ms.
// d) The ISR lag time is constant hence not dealt with here.
//
// Details:
//
// A single PIT timer is used here for all timing.  The 40ms cycle time is
// broken into periods of time, one for each servo channel needed (known at
// init time).  When enabled, the timer will be configured to timeout at the
// next needed GPIO transition.  Each enabled channel has two transitions, the
// leading rising edge and the trailing falling edge.
//
// The "sNextStep" variable is used to track what channel and which edge the
// next ISR is expected for.  The channel is held in the upper 7 bits.  The
// low bit, bit 0, is used to denote which edge is the next edge at the next
// ISR (0 = rising, 1 = falling).
//
// ----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>

#include "clk.h"
#include "frdmCfg.h"
#include "gpio.h"
#include "irq.h"
#include "os.h"
#include "servo.h"

#define PIT_1_USEC     (BUS_CLOCK / 1000000)

#define PIT_WARMUP_DELAY  (PIT_1_USEC * 1000)  // 1ms

#define PWM_CYCLE_TIME    (40000) // 40ms in microseconds
#define PWM_CHANNEL_TIME  (2000) // 2ms in microseconds
#define PWM_MIN_CNT       (PIT_1_USEC * 1000)  // 1ms
#define PWM_PULSE_CNT(x)  (PWM_MIN_CNT + (PIT_1_USEC * x))

#define MAX_POSITION   (1000)
#define MAX_DEGREES    (180)

typedef struct
{
#define SERVO_FLAG_OPEN      (1 << 0)
#define SERVO_FLAG_ENABLED   (1 << 1)
#define SERVO_FLAG_CR        (1 << 2) // Continuous Rotation
   uint8_t  flags;
   uint8_t  gpioId;
   uint16_t stop; // The CR stop position
   uint32_t cnt;  // The number of PIT clocks pulses for the output to be high
} servoCh_t;

static int sInitialized = 0;
static servoCh_t sServo[SERVO_MAX_CHANNELS];
static uint8_t   sChannels;
static uint8_t   sEnabledChCnt;
static uint8_t   sNextStep;  // (Channel# * 2) w/bit 0 denoting falling edge ISR
static uint32_t  sInterChDelay; // Delay between channels

static int  findEmptyChannel(void);
static uint32_t getNextDelay(void);
static void pitIrq(uint32_t cookie);
static void pitStart(uint32_t delay1, uint32_t delay2);
static void pitStop(void);
static inline bool verifyOpen(uint8_t id);

// ----------------------------------------------------------------------------
// External Functions
// ----------------------------------------------------------------------------

int servoInit(uint8_t channels)
{
   int rc = -1;

   if (sInitialized)
   {
      // Already initialized, return success
      rc = 0;
   }
   else if (channels >= SERVO_MAX_CHANNELS)
   {
      // Just return an error
   }
   else
   {
      uint8_t *p = (uint8_t*) sServo;
      int i;

      sInitialized = 1;
      sChannels = channels;
      sEnabledChCnt = 0;
      sInterChDelay = ((PWM_CYCLE_TIME / sChannels) - PWM_CHANNEL_TIME) * PIT_1_USEC;

      clkEnable(CLK_PIT);

      i = sizeof(sServo);
      while (i--)
      {
         *p++ = 0;
      }

      rc = 0;
   }

   return rc;
}

void servoTerm(void)
{
   if (sInitialized)
   {
      int i;
      
      for (i = 0; i < sChannels; i++)
      {
         if (sServo[i].flags & SERVO_FLAG_OPEN)
         {
            servoClose(i);
         }

         clkDisable(CLK_PIT);
      }
   }
}

uint8_t servoOpen(uint8_t mode, uint8_t gpioId)
{
   uint8_t ch = SERVO_INVALID_CHANNEL;

   if (!sInitialized)
   {
      // Just return an error
   }
   else if (mode != SERVO_MODE_POSITIONAL && mode != SERVO_MODE_CONTINUOUS_ROTATION)
   {
      // Just return an error
   }
   else if (GPIO_ID_TO_PORT(gpioId) > GPIO_PORT_MAX)
   {
      // Just return an error
   }
   else if ((ch = findEmptyChannel()) < 0)
   {
      // Just return an error
   }
   else
   {
      sServo[ch].flags = SERVO_FLAG_OPEN;
      sServo[ch].gpioId = gpioId;
      // Default to mid range
      sServo[ch].cnt = PWM_PULSE_CNT(MAX_POSITION / 2);

      if (mode == SERVO_MODE_CONTINUOUS_ROTATION)
      {
         sServo[ch].flags = SERVO_FLAG_CR;
         sServo[ch].stop = MAX_POSITION / 2;
      }
   }

   return ch;
}

void servoClose(uint8_t id)
{
   if (!sInitialized)
   {
      // Just ignore
   }
   else if (id >= sChannels)
   {
      // just ignore
   }
   else
   {
      if (sServo[id].flags & SERVO_FLAG_ENABLED)
      {
         servoDisable(id);
      }
      sServo[id].flags = 0;
   }
}

int servoEnable(uint8_t id)
{
   int rc = -1;

   if (!sInitialized)
   {
      // Just return an error
   }
   else if (id >= sChannels)
   {
      // Just return an error
   }
   else if (!(sServo[id].flags & SERVO_FLAG_OPEN))
   {
      // Just return an error
   }
   else if (sServo[id].flags & SERVO_FLAG_ENABLED)
   {
      // Already enable, return success
      rc = 0;
   }
   else
   {
      sServo[id].flags |= SERVO_FLAG_ENABLED;
      gpioOutputLow(sServo[id].gpioId);
      gpioSetAsOutput(sServo[id].gpioId);

      sEnabledChCnt ++;
      if (sEnabledChCnt == 1)
      {
         uint32_t delay;

         sNextStep = id << 1;
         delay = getNextDelay();
         pitStart(PIT_WARMUP_DELAY, delay);
      }
   }

   return rc;
}

void servoDisable(uint8_t id)
{
   if (!verifyOpen(id))
   {
      // Just ignore
   }
   else if (!(sServo[id].flags & SERVO_FLAG_ENABLED))
   {
      // Just ignore
   }
   else
   {
      sServo[id].flags &= ~(SERVO_FLAG_ENABLED);
      gpioSetAsInput(sServo[id].gpioId);

      sEnabledChCnt --;
      if (sEnabledChCnt == 0)
      {
         pitStop();
      }
   }
}

int servoMoveDegree(uint8_t id, uint8_t degrees)
{
   int rc = 0;

   if (!verifyOpen(id))
   {
      rc = -1;
   }
   else
   {
      uint32_t position;

      if (degrees > MAX_DEGREES)
      {
         degrees = MAX_DEGREES;
      }

      position = (MAX_POSITION * degrees) / MAX_DEGREES;
      sServo[id].cnt = PWM_PULSE_CNT(position);
   }

   return rc;
}

int servoMovePosition(uint8_t id, uint16_t position)
{
   int rc = 0;

   if (!verifyOpen(id))
   {
      rc = -1;
   }
   else
   {
      if (position > MAX_POSITION)
      {
         position = MAX_POSITION;
      }

      sServo[id].cnt = PWM_PULSE_CNT(position);
   }

   return rc;
}

int servoCrStoppedPosition(uint8_t id, uint16_t position)
{
   int rc = 0;

   if (!verifyOpen(id))
   {
      rc = -1;
   }
   else
   {
      if (position > MAX_POSITION)
      {
         position = MAX_POSITION;
      }

      sServo[id].stop = position;
   }

   return rc;
}

int servoCrMovePercent(uint8_t id, int8_t percent)
{
   int rc = 0;

   if (!verifyOpen(id))
   {
      rc = -1;
   }
   else
   {
   }

   return rc;
}

// ----------------------------------------------------------------------------
// Local Functions
// ----------------------------------------------------------------------------

// Returns -1 if no empty channel found
static int findEmptyChannel(void)
{
   int ch = -1;
   int i;

   for (i = 0; i < sChannels; i++)
   {
      if (!(sServo[i].flags & SERVO_FLAG_OPEN))
      {
         ch = i;
         break;
      }
   }

   return ch;
}

static uint32_t getNextDelay(void)
{
   // When entering this function, the sNextStep variable denotes the current
   // ISR state.  If the current state is for the rising edge, return the
   // channel pulse time.  If the current state is for the falling edge, return
   // the time to the next enabled channels rising edge.
   //
   // The sNextStep variable will be updated before exiting this function.

   uint32_t delay = PWM_CYCLE_TIME * PIT_1_USEC;
   uint8_t ch = sNextStep >> 1;
   uint8_t falling = sNextStep & 0x1;

   if (  !(sServo[ch].flags & SERVO_FLAG_OPEN)
      || !(sServo[ch].flags & SERVO_FLAG_ENABLED)
      )
   {
      // This should not occur, but skip this channel if it does
      delay = PWM_CYCLE_TIME * PIT_1_USEC;

      ch ++;
      ch = (ch >= sChannels) ? 0 : ch;
      sNextStep = ch << 1;
   }
   else if (!falling) // Rising edge
   {
      // Return the pulse time for this channel
      delay = sServo[ch].cnt;
      sNextStep |= 0x1; // Mark for falling edge next
   }
   else // Falling edge
   {
      // Need to find the next enabled channel

      int i;
      
      // Add the last of the current channel
      delay  = (PWM_CHANNEL_TIME * PIT_1_USEC) - sServo[ch].cnt;
      // Add the inter-channel delay
      delay += sInterChDelay;
      
      for (i = 0; i < sChannels; i++)
      {
         ch ++;
         ch = (ch >= sChannels) ? 0 : ch;

         if (  (sServo[ch].flags & SERVO_FLAG_OPEN)
            && (sServo[ch].flags & SERVO_FLAG_ENABLED)
            )
         {
            break;
         }

         // Skip to the next channel
         delay += (PWM_CHANNEL_TIME * PIT_1_USEC) + sInterChDelay;
      }
      
      sNextStep = ch << 1;
   }

   return delay;
}

static void pitIrq(uint32_t cookie)
{
   uint8_t ch = sNextStep >> 1;
   uint8_t falling = sNextStep & 0x1;

   if (cookie == SERVO_PIT)
   {
      // Clear the interrupt bit
      PIT_TFLG(SERVO_PIT) = PIT_TFLG_TIF_MASK;

      if (  (sServo[ch].flags & SERVO_FLAG_OPEN)
         && (sServo[ch].flags & SERVO_FLAG_ENABLED)
         )
      {
         // Transition the PWM GPIO
         if (!falling)
         {
            gpioOutputHigh(sServo[ch].gpioId);
         }
         else
         {
            gpioOutputLow(sServo[ch].gpioId);
         }
      }

      // Load the next timeout
      PIT_LDVAL(SERVO_PIT) = getNextDelay();
   }
}

static void pitStart(uint32_t delay1, uint32_t delay2)
{
   // Enable the PIT module
   PIT_MCR = 0;

   // Register and enable the interrupt
   irqRegister(INT_PIT, pitIrq, SERVO_PIT);
   PIT_TFLG(SERVO_PIT) = PIT_TFLG_TIF_MASK; // Clr the interrupt
   irqEnable(INT_PIT);

   // Load the first timeout
   PIT_LDVAL(SERVO_PIT) = delay1;

   // Start the counter and enable the interrupt
   PIT_TCTRL(SERVO_PIT) = PIT_TCTRL_TEN_MASK | PIT_TCTRL_TIE_MASK;

   // Load the second timeout
   PIT_LDVAL(SERVO_PIT) = delay2;
}

static void pitStop(void)
{
   // Stop the counter and disable the interrupt
   PIT_TCTRL(SERVO_PIT) = 0;

   // Disable and unregister the interrupt
   irqDisable(INT_PIT);
   irqUnregister(INT_PIT);
}

static inline bool verifyOpen(uint8_t id)
{
   bool rc = false;

   if (!sInitialized)
   {
      // Just return false
   }
   else if (id >= sChannels)
   {
      // Just return false
   }
   else if (!(sServo[id].flags & SERVO_FLAG_OPEN))
   {
      // Just return false
   }
   else
   {
      rc = true;
   }

   return rc;
}


