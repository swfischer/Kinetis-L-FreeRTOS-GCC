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

#include <stdint.h>

#include "clk.h"
#include "os.h"
#include "usbCdc.h"
#include "usbCore.h"
#include "usbTask.h"

extern uint8_t usbFlagsHack; 
extern uint8_t gu8EP3_OUT_ODD_Buffer[];

// ----------------------------------------------------------------------------
// External Functions
// ----------------------------------------------------------------------------

void usbTaskEntry(void *pParameters)
{
   uint16_t x;

   osDelay(5000); // this is a bit of a hack just to allow for console debugging

   usbCdcInit();

   while (1)
   {
      usbCdcEngine();

      // If data transfer arrives
      if (usbFlagsHack & (1 << EP_OUT))
      {
         x = usbCoreEpOutSizeCheck(EP_OUT);

         usbEP_Reset(EP_OUT);
         usbSIE_CONTROL(EP_OUT);

         usbFlagsHack &= ~(1 << EP_OUT);

         // Send it back to the PC
         usbCoreEpInTransfer(EP2, CDC_OUTPointer, x);
      }

      osDelay(100);
   }
}

// ----------------------------------------------------------------------------
// Local Functions
// ----------------------------------------------------------------------------
