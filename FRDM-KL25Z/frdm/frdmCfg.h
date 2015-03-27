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

#ifndef _FRDMCFG_H_
#define _FRDMCFG_H_

#include "kinetis.h"

// Uncomment to enable the USB device driver
#define USB_DEVICE_ENABLED

#ifdef USB_DEVICE_ENABLED
#define CLK_DIV_1 (1)
#define PR_DIV_0  (3)
#define V_DIV_0   (24)
#else
#define CLK_DIV_1 (0)
#define PR_DIV_0  (3)
#define V_DIV_0   (0)
#endif

#define CORE_CLOCK   (48000000) // Hz
#define BUS_CLOCK    (CORE_CLOCK / 2) // Hz

#ifdef USB_DEVICE_ENABLED
#define PERIPH_CLOCK (48000000) // Hz - really MCGPLLCLK/MCGFLLCLK
#else
#define PERIPH_CLOCK (24000000) // Hz - really MCGPLLCLK/MCGFLLCLK
#endif

#define TICK_RATE    (100) // Hz
#define MS_PER_TICK  (1000/TICK_RATE) // Milliseconds per tick
#define HZ           (1000/MS_PER_TICK) // Number of ticks per second

#define BAUD_RATE    (115200) // bps

#define TOUCH_PAD_A   (9)
#define TOUCH_PAD_B  (10)

//#define I2C_KEEP_CLKS_ENABLED
#define ACCEL_I2C    (I2C0_BASE_PTR)
#define MMA8451_SA0_ONE // SA0 pulled high

//#define ADC_KEEP_CLKS_ENABLED

#ifdef USB_DEVICE_ENABLED
// Uncomment to enable the console through the USB device driver
#define USB_CONSOLE_ENABLED

#ifndef USB_CONSOLE_ENABLED
#define USB_LOOPBACK_ENABLED
#endif
#endif

#endif // _FRDMCFG_H_
