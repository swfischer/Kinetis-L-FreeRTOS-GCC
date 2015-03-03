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
#include "kinetis.h"
#include "led.h"

// ----------------------------------------------------------------------------
// External Functions
// ----------------------------------------------------------------------------

void ledInit(void)
{
   // Turn on clock gating to PortB module (red and green LEDs) and PortD module (blue LED)  
   clkEnable(CLK_PORTB);
   clkEnable(CLK_PORTD);
   clkEnable(CLK_TPM0);
   clkEnable(CLK_TPM2);

   ledSet(0,0,0); // Off
    
   TPM0_MOD  = 99;
   TPM0_C1SC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSA_MASK;
   TPM2_MOD  = 99;
   TPM2_C0SC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSA_MASK;
   TPM2_C1SC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSA_MASK;

   TPM2_SC = TPM_SC_CMOD(1) | TPM_SC_PS(0); // Edge Aligned PWM running from BUSCLK/1
   TPM0_SC = TPM_SC_CMOD(1) | TPM_SC_PS(0); // Edge Aligned PWM running from BUSCLK/1
}
