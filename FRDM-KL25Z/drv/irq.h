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
// This code was created to provide an interrupt framework to reduce hardcoding
// of interrupt handlers either by direct mapping or handler naming.
//
// This interrupt framework, along with the interrupt vector setup in
// "startup.c", funnels all peripheral related interrupts into a single generic
// handler which than dispatches the interrupt handling to an appropriate
// registered client handling function.  This allows for code using interrupts
// to be written more generically and thus allowing more flexiblity to switch
// HW blocks (for instance I2C0 to I2C1) with little to no code impact.
//
// Additionally, this framework virtualizes the GPIO interrupts meaning that
// even though all interrupts for a GPIO port occur on the same physical
// interrupt, each pin of the port has its own virtual interrupt which can be
// registered for and serviced as if it were an individual interrupt.  This
// same virtualization could be done for other peripherals with shared
// interrupts, but as yet that has not been needed.
// ----------------------------------------------------------------------------

#ifndef _IRQ_H_
#define _IRQ_H_

#include <stdint.h>

#include "kinetis.h"

// typedef'ing the Freescale interrupt type to clarify that an "int" numbering
// includes all interrupts and an "irq" numbering skips over the system
// interrupts thus only including the peripheral interrupts.
typedef IRQInterruptIndex intIdx_t;

#define INT_PHYSICAL_FIRST    (INT_SysTick + 1)
#define INT_PHYSICAL_LAST     (INT_PHYSICAL_FIRST + 31)
#define INT_VIRTUAL_FIRST     (INT_PHYSICAL_LAST + 1)

enum
{ INT_VIRTUAL_PORT_FIRST = (INT_VIRTUAL_FIRST)
, INT_VIRTUAL_PORT_A_FIRST = (INT_VIRTUAL_PORT_FIRST)
, INT_VIRTUAL_PORT_D_FIRST = (INT_VIRTUAL_PORT_A_FIRST + 32)
, INT_VIRTUAL_PORT_LAST = ((INT_VIRTUAL_PORT_D_FIRST + 32) - 1)
, INT_VIRTUAL_LAST // Must be last
};

enum
{ PORT_A = 0
, PORT_B
, PORT_C
, PORT_D
, PORT_E
};

#define PORT_PIN_TO_INT(port, pin) ((intIdx_t) (INT_VIRTUAL_PORT_A_FIRST + ((portToVirtualPortIdx(port) * 32) + (pin & 0x1f))))
#define PORT_IRQ_TO_PORT(irq)      (virtualPortIdxToPort((irq - IRQ(INT_VIRTUAL_PORT_A_FIRST)) >> 5))
#define PORT_IRQ_TO_PIN(irq)       ((irq - IRQ(INT_VIRTUAL_PORT_A_FIRST)) & 0x1f)

#define IRQ(x)            (x - INT_PHYSICAL_FIRST)

// The IRQ callback function pattern
typedef void (*irqCallback)(uint32_t cookie);

// One-time initialization of this interrupt framework
extern void irqInit(void);
// The single framework interrupt handler function
extern void irqHandler(void);
// Used for registering a callback to a given interrupt
extern int  irqRegister(intIdx_t intIdx, irqCallback callback, uint32_t cookie);
// Used for deleting the previous registeration of a given interrupt
extern void irqUnregister(intIdx_t intIdx);
// Used for enabling a given interrupt (allowing the interrupt to occur)
extern void irqEnable(intIdx_t intIdx);
// Used for disabling a given interrupt (blocking the interrupt from occurring)
extern void irqDisable(intIdx_t intIdx);

// A few inlines functions which help map to interruptible ports as consecutive
static inline int portToVirtualPortIdx(int port)
{
   return (port == PORT_D) ? 1 : 0;
}

static inline int virtualPortIdxToPort(int idx)
{
   return (idx == 1) ? PORT_D : PORT_A;
}

#endif // _IRQ_H_

