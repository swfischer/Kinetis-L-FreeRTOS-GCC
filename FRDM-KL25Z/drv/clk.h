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
// This code was created to provide peripheral clock resource management.
// All processor clock gates are managed such that they will be turned on upon
// the first enable request (via the clkEnable() function) and will stay
// enabled until all outstanding requests have been released (via the
// clkDisable() function).
//
// The mechanism used for this is a counting mechanism such that each
// clkEnable() request increases a clock specific count and each clkDisable()
// request decreases a clock specific count.  The clock is disabled when the
// count is zero.
//
// Additional functions exist to allow for determining is a specific clock is
// currently enabled or disabled (via the clkIsEnabled() function) and to allow
// reading of the current use count (via the clkGetUseCount() function).
// ----------------------------------------------------------------------------

#ifndef _CLK_H_
#define _CLK_H_

#include <stdint.h>

typedef enum ClkGate
{ CLK_I2C0 // SCGC4
, CLK_I2C1
, CLK_UART0
, CLK_UART1
, CLK_UART2
, CLK_USBOTG
, CLK_CMP
, CLK_SPI0
, CLK_SPI1
, CLK_LPTIMER // SCGC5
, CLK_TSI
, CLK_PORTA
, CLK_PORTB
, CLK_PORTC
, CLK_PORTD
, CLK_PORTE
, CLK_FTF // SCGC6
, CLK_DMAMUX
, CLK_PIT
, CLK_TPM0
, CLK_TPM1
, CLK_TPM2
, CLK_ADC0
, CLK_RTC
, CLK_DAC0
, CLK_DMA // SCGC7
, CLK_NUMBER_OF_CLKS // Must be last
} clkGate_t;

// External functions.

// Used for the one-time power-up initialization of the clock framework
extern void clkInit(void);
// Used to enable a given clock gate
extern void clkEnable(clkGate_t clk);
// Used to disable a given clock gate
extern void clkDisable(clkGate_t clk);
// Used to retrieve a text name of a given clock gate
extern const char* clkGetName(clkGate_t clk);
// Used to retrieve the use count of a given clock gate
extern int  clkGetUseCount(clkGate_t clk);
// Used to retrieve the current state of a given clock gate
extern int  clkIsEnabled(clkGate_t clk);

#endif // _CLK_H_

