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

#include "accel.h"
#include "clk.h"
#include "consoleTask.h"
#include "frdmCfg.h"
#include "irq.h"
#include "led.h"
#include "ledTask.h"
#include "os.h"
#include "pinmux.h"
#include "touch.h"
#include "usbTask.h"

#define CONSOLE_PRIORITY    (tskIDLE_PRIORITY + 2)
#define CONSOLE_STACK_SIZE  (2 *configMINIMAL_STACK_SIZE)

#define LED_PRIORITY	       (tskIDLE_PRIORITY + 3)
#define LED_STACK_SIZE      (configMINIMAL_STACK_SIZE)

#define USB_PRIORITY	       (tskIDLE_PRIORITY + 1)
#define USB_STACK_SIZE      (4 * configMINIMAL_STACK_SIZE)

static void simInit(void);

// ----------------------------------------------------------------------------
// OS and IRQ Interrupt Handlers
// ----------------------------------------------------------------------------

void PendSV_Handler(void)
{
   osPendSvIsrHandler();
}

void SysTick_Handler(void)
{
   osTickIsrHandler();
}

void IRQ_Handler(void)
{
   irqHandler();
}

// ----------------------------------------------------------------------------
// External Functions
// ----------------------------------------------------------------------------

int main( void )
{
   osThreadDef_t tDef;

   simInit();

   clkInit(); // Must be first
   pinmuxInit();
   irqInit();

   ledInit();
   touchInit( (1 << TOUCH_PAD_A) | (1 << TOUCH_PAD_B) );
   accelInit();

   osKernelInitialize();

   tDef.pthread = consoleTaskEntry;
   tDef.tpriority = CONSOLE_PRIORITY;
   tDef.stacksize = CONSOLE_STACK_SIZE;
   tDef.name = "cons";
   osThreadCreate(&tDef, NULL);

   tDef.pthread = ledTaskEntry;
   tDef.tpriority = LED_PRIORITY;
   tDef.stacksize = LED_STACK_SIZE;
   tDef.name = "led";
   osThreadCreate(&tDef, NULL);

#ifdef USB_DEVICE_ENABLED
   tDef.pthread = usbTaskEntry;
   tDef.tpriority = LED_PRIORITY;
   tDef.stacksize = LED_STACK_SIZE;
   tDef.name = "usb";
   osThreadCreate(&tDef, NULL);
#endif

   osKernelStart();

   // Should never get here

   return 0;
}

// ----------------------------------------------------------------------------
// Local Functions
// ----------------------------------------------------------------------------

static void simInit(void)
{
   // System oscillator drives 32 kHz clock for various peripherals (OSC32KSEL=0)
   SIM_SOPT1 &= ~SIM_SOPT1_OSC32KSEL(3);

   // Select PLL as a clock source for various peripherals (PLLFLLSEL=1)
   SIM_SOPT2 |= SIM_SOPT2_PLLFLLSEL_MASK;

   // UART0 clock setup
   SIM_SOPT2 &= ~SIM_SOPT2_UART0SRC_MASK;
   SIM_SOPT2 |= SIM_SOPT2_UART0SRC(1);

   // TPM clock setup
   SIM_SOPT2 = (SIM_SOPT2 & ~(SIM_SOPT2_TPMSRC(2))) | SIM_SOPT2_TPMSRC(1);

#ifdef USB_DEVICE_ENABLED
   // USB clock source MCGPLLCLK/2 or MCGFLLCLK
   SIM_SOPT2 |= SIM_SOPT2_USBSRC_MASK;

   // Setup USB regulators
   SIM_SOPT1 |= SIM_SOPT1_USBREGEN_MASK | SIM_SOPT1_USBSSTBY_MASK | SIM_SOPT1_USBVSTBY_MASK;
   SIM_SOPT1CFG |= SIM_SOPT1CFG_URWE_MASK | SIM_SOPT1CFG_USSWE_MASK | SIM_SOPT1CFG_UVSWE_MASK;
#endif
}
