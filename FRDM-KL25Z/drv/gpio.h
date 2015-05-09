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
// This code was created to provide as a wrapper around the GPIO HW interfaces.
//
// A secondary purpose is to serialize the GPIO in such a way as to allow the
// designation of a GPIO with a single value (a GPIO ID) versus a port/pin pair
// of values. The reason for this is that it makes abstracting GPIOs simplier
// since only one value is needed to designate the specific GPIO.  For
// instance, it becomes easier to have a define that denotes which GPIO is used
// for a switch signal input when different GPIOs are used for a Freedom-KL25Z
// board versus the same code on a Freedom-KL26Z board.  Yes, it could be done
// as a pair of defines, one for the port and one for the pin, but it is
// simplier in my opinion to manage as a single value with truly very little
// overhead involved.
// ----------------------------------------------------------------------------

#ifndef _GPIO_H_
#define _GPIO_H_

#include <stdint.h>
#include <stdbool.h>

#include "kinetis.h"

#define GPIO_PIN_MAX   (31)

// Macros to convert mappings from port/pin to GPIO ID or the reverse
#define GPIO_PORT_PIN_TO_ID(port, pin) ((port << 5) | (pin & 0x1f))
#define GPIO_ID_TO_PORT(id) (id >> 5)
#define GPIO_ID_TO_PIN(id)  (id & 0x1f)

enum
{ GPIO_PORT_A = 0
, GPIO_PORT_B
, GPIO_PORT_C
, GPIO_PORT_D
, GPIO_PORT_E
, GPIO_PORT_MAX = GPIO_PORT_E
};

// Used for clearing the interrupt bit for the given GPIO
extern void gpioClearInterrupt(uint8_t gpioID);
// Used for determining the register set base address for the given GPIO
extern GPIO_MemMapPtr gpioIdToBase(uint8_t gpioID);
// Used for retrieving the current state of the given GPIO
extern int  gpioInputGet(uint8_t gpioID);
// Used for determining if the given GPIO is currently in the high state
extern int  gpioInputIsHigh(uint8_t gpioID);
// Used for determining if the given GPIO is currently in the low state
extern int  gpioInputIsLow(uint8_t gpioID);
// Used for setting the given GPIO to output a high state (assuming its configured for output)
extern void gpioOutputHigh(uint8_t gpioID);
// Used for setting the given GPIO to output a low state (assuming its configured for output)
extern void gpioOutputLow(uint8_t gpioID);
// Used for setting the given GPIO output state (assuming its configured for output)
extern void gpioOutputSet(uint8_t gpioID, bool value);
// Used for toggling the given GPIO output state (assuming its configured for output)
extern void gpioOutputToggle(uint8_t gpioID);
// Used for setting the given GPIO as an input
extern void gpioSetAsInput(uint8_t gpioID);
// Used for setting the given GPIO as an output
extern void gpioSetAsOutput(uint8_t gpioID);

#endif // _GPIO_H_

