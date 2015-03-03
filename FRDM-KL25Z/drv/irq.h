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

#ifndef _IRQ_H_
#define _IRQ_H_

#include <stdint.h>

#include "kinetis.h"

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

// This should be somewhere else
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

typedef void (*irqCallback)(uint32_t cookie);

extern void irqInit(void);
extern void irqHandler(void);

extern int  irqRegister(intIdx_t intIdx, irqCallback callback, uint32_t cookie);
extern void irqUnregister(intIdx_t intIdx);
extern void irqEnable(intIdx_t intIdx);
extern void irqDisable(intIdx_t intIdx);

static inline int portToVirtualPortIdx(int port)
{
   return (port == PORT_D) ? 1 : 0;
}

static inline int virtualPortIdxToPort(int idx)
{
   return (idx == 1) ? PORT_D : PORT_A;
}

#endif // _IRQ_H_
