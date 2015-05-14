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
// Functional Description:
//
// This code was created to provide a simple interface to the Freedom boards
// multi-color LED hardware.  The code is based on Andrew Payne's work (see
// led.c for his copyright notice).
//
// The LED color brightness values input as a percentage of full brightness
// with valid input values from 0 to 100, where 0 implies the LED color is off
// and 100 implies the LED color is at full brightness.
// ----------------------------------------------------------------------------

#ifndef _LED_H_
#define _LED_H_

#include <stdint.h>
#include <stdbool.h>

#include "kinetis.h"

// One-time initialization of the LED hardware.
extern void ledInit(void);

// Used for setting the LED brightness of all colors at once.
static inline void ledSet(int red, int green, int blue)
{
   TPM2_C0V = red;
   TPM2_C1V = green;
   TPM0_C1V = blue;
}

// Used for setting the LED brightness of the blue color only.
static inline void ledBlue(int percent)
{
   TPM0_C1V = percent;
}

// Used for setting the LED brightness of the green color only.
static inline void ledGreen(int percent)
{
   TPM2_C1V = percent;
}

// Used for setting the LED brightness of the red color only.
static inline void ledRed(int percent)
{
   TPM2_C0V = percent;
}

#endif // _LED_H_

