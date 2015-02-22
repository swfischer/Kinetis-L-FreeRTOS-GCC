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

#ifndef _GPIO_H_
#define _GPIO_H_

#include <stdint.h>
#include <stdbool.h>

#include "MKL25Z4.h"

#define GPIO_PORT_PIN_TO_ID(port, pin) ((port << 5) | (pin & 0x1f))
#define GPIO_ID_TO_PORT(id) (id >> 5)
#define GPIO_ID_TO_PIN(id)  (id & 0x1f)

extern void gpioClearInterrupt(uint8_t gpioID);
extern GPIO_MemMapPtr gpioIdToBase(uint8_t gpioID);
extern int  gpioInputGet(uint8_t gpioID);
extern int  gpioInputIsHigh(uint8_t gpioID);
extern int  gpioInputIsLow(uint8_t gpioID);
extern void gpioOutputHigh(uint8_t gpioID);
extern void gpioOutputLow(uint8_t gpioID);
extern void gpioOutputSet(uint8_t gpioID, bool value);
extern void gpioOutputToggle(uint8_t gpioID);
extern void gpioSetAsInput(uint8_t gpioID);
extern void gpioSetAsOutput(uint8_t gpioID);

#endif // _GPIO_H_
