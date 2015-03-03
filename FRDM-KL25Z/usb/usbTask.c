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

#include "clk.h"
#include "os.h"
#include "usbTask.h"

#include "usb_cdc.h"
#include "usb_reg.h"

#include "irq.h"

extern uint8 gu8USB_Flags; 
extern uint8 gu8EP3_OUT_ODD_Buffer[];
extern tBDT tBDTtable[];

volatile uint8  gu8ISR_Flags=0;

void usbTaskEntry(void *pParameters)
{
   osDelay(5000); // this is a bit of a hack just to allow for console debugging

   USB_REG_SET_ENABLE;
   USB_REG_SET_STDBY_STOP;      
   USB_REG_SET_STDBY_VLPx;

   irqRegister(INT_USB0, USB_ISR, 0);
   irqEnable(INT_USB0);

   clkEnable(CLK_PORTC);
   CDC_Init();

   while (1)
   {
      CDC_Engine();

      // If data transfer arrives
      if(FLAG_CHK(EP_OUT,gu8USB_Flags))
      {
         (void)USB_EP_OUT_SizeCheck(EP_OUT);
         usbEP_Reset(EP_OUT);
         usbSIE_CONTROL(EP_OUT);
         FLAG_CLR(EP_OUT,gu8USB_Flags);

         // Send it back to the PC
         EP_IN_Transfer(EP2,CDC_OUTPointer,1);
      }

      osDelay(100);
   }
}
