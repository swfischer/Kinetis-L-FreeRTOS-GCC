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
#include "kinetis.h"
#include "os.h"

typedef struct
{
   uint8_t offset : 3;
   uint8_t bit    : 5;

} clkToReg_t;

// Clk gate register offset values.
#define REG_SCGC4  (0)
#define REG_SCGC5  (1)
#define REG_SCGC6  (2)
#define REG_SCGC7  (3)

static const clkToReg_t sClkToReg[] =
{ { REG_SCGC4,  6 }  // I2C0 --- SCGC4
, { REG_SCGC4,  7 }  // I2C1
, { REG_SCGC4, 10 }  // UART0
, { REG_SCGC4, 11 }  // UART1
, { REG_SCGC4, 12 }  // UART2
, { REG_SCGC4, 18 }  // USBOTG
, { REG_SCGC4, 19 }  // CMP
, { REG_SCGC4, 22 }  // SPI0
, { REG_SCGC4, 23 }  // SPI1
, { REG_SCGC5,  0 }  // LPTIMER --- SCGC5
, { REG_SCGC5,  5 }  // TSI
, { REG_SCGC5,  9 }  // PORTA
, { REG_SCGC5, 10 }  // PORTB
, { REG_SCGC5, 11 }  // PORTC
, { REG_SCGC5, 12 }  // PORTD
, { REG_SCGC5, 13 }  // PORTE
, { REG_SCGC6,  0 }  // FTF --- SCGC6
, { REG_SCGC6,  1 }  // DMAMUX
, { REG_SCGC6, 23 }  // PIT
, { REG_SCGC6, 24 }  // TPM0
, { REG_SCGC6, 25 }  // TPM1
, { REG_SCGC6, 26 }  // TPM2
, { REG_SCGC6, 27 }  // ADC0
, { REG_SCGC6, 29 }  // RTC
, { REG_SCGC6, 31 }  // DAC0
, { REG_SCGC7,  8 }  // DMA --- SCGC7
};

static uint8_t sUseCount[CLK_NUMBER_OF_CLKS];
static osMutexId sLock;   // Used to protect access to the use counts.

// ----------------------------------------------------------------------------
// External Functions
// ----------------------------------------------------------------------------

void clkInit(void)
{
   int i;

   sLock = osMutexCreate(NULL);

   // Search for clks that are defaulted on and adjust the count
   for (i = 0; i < CLK_NUMBER_OF_CLKS; i++)
   {
      if (clkIsEnabled((clkGate_t) i))
      {
         sUseCount[i] = 1;
      }
   }
}

void clkEnable(clkGate_t clk)
{
   if (clk < 0 || clk >= CLK_NUMBER_OF_CLKS)
   {
      // Invalid clk, ignore
   }
   else if (sUseCount[clk] == 0xFF)
   {
      // Clock count max'd out, ignore
   }
   else
   {
      osMutexWait(sLock, WAIT_FOREVER);

      if (sUseCount[clk] == 0)
      {
         switch (sClkToReg[clk].offset)
         {
         case REG_SCGC4:
            SIM_SCGC4 |= (1 << sClkToReg[clk].bit);
            break;
         case REG_SCGC5:
            SIM_SCGC5 |= (1 << sClkToReg[clk].bit);
            break;
         case REG_SCGC6:
            SIM_SCGC6 |= (1 << sClkToReg[clk].bit);
            break;
         case REG_SCGC7:
            SIM_SCGC7 |= (1 << sClkToReg[clk].bit);
            break;
         }
      }

      sUseCount[clk]++;

      osMutexRelease(sLock);
   }
}

void clkDisable(clkGate_t clk)
{
   if (clk < 0 || clk >= CLK_NUMBER_OF_CLKS)
   {
      // Invalid clk, ignore
   }
   else if (sUseCount[clk] == 0)
   {
      // Clock count zero, ignore
   }
   else
   {
      osMutexWait(sLock, WAIT_FOREVER);

      if (sUseCount[clk] == 1)
      {
         switch (sClkToReg[clk].offset)
         {
         case REG_SCGC4:
            SIM_SCGC4 &= ~(1 << sClkToReg[clk].bit);
            break;
         case REG_SCGC5:
            SIM_SCGC5 &= ~(1 << sClkToReg[clk].bit);
            break;
         case REG_SCGC6:
            SIM_SCGC6 &= ~(1 << sClkToReg[clk].bit);
            break;
         case REG_SCGC7:
            SIM_SCGC7 &= ~(1 << sClkToReg[clk].bit);
            break;
         }
      }

      sUseCount[clk]--;

      osMutexRelease(sLock);
   }
}

const char* clkGetName(clkGate_t clk)
{
   typedef struct
   {
      clkGate_t clk;
      const char* name;
   } clkName_t;
   
   static const clkName_t sNameList[] =
   { { CLK_I2C0, "I2C0" }
   , { CLK_I2C1, "I2C1" }
   , { CLK_UART0, "UART0" }
   , { CLK_UART1, "UART1" }
   , { CLK_UART2, "UART2" }
   , { CLK_USBOTG, "USBOTG" }
   , { CLK_CMP, "CMP" }
   , { CLK_SPI0, "SPI0" }
   , { CLK_SPI1, "SPI1" }
   , { CLK_LPTIMER, "LPTIMER" }
   , { CLK_TSI, "TSI" }
   , { CLK_PORTA, "PORTA" }
   , { CLK_PORTB, "PORTB" }
   , { CLK_PORTC, "PORTC" }
   , { CLK_PORTD, "PORTD" }
   , { CLK_PORTE, "PORTE" }
   , { CLK_FTF, "FTF" }
   , { CLK_DMAMUX, "DMAMUX" }
   , { CLK_PIT, "PIT" }
   , { CLK_TPM0, "TPM0" }
   , { CLK_TPM1, "TPM1" }
   , { CLK_TPM2, "TPM2" }
   , { CLK_ADC0, "ADC0" }
   , { CLK_RTC, "RTC" }
   , { CLK_DAC0, "DAC0" }
   , { CLK_DMA, "DMA" }
   };
   static const char* unknown = "Unknown";
   const char *name = unknown;
   int i;

   for (i = 0; i < CLK_NUMBER_OF_CLKS; i++)
   {
      if (clk == sNameList[i].clk)
      {
         name = sNameList[i].name;
         break;
      }
   }

   return name;
}

int clkGetUseCount(clkGate_t clk)
{
   int count = 0;

   if (clk < 0 || clk >= CLK_NUMBER_OF_CLKS)
   {
      // Invalid clk, ignore
   }
   else
   {
      count = sUseCount[clk];
   }

   return count;
}

int clkIsEnabled(clkGate_t clk)
{
   int enabled = 0;

   if (clk < 0 || clk >= CLK_NUMBER_OF_CLKS)
   {
      // Invalid clk, ignore
   }
   else
   {
      // Read the enable value directly from the SCGC register
      switch (sClkToReg[clk].offset)
      {
      case REG_SCGC4:
         enabled = (SIM_SCGC4 & (1 << sClkToReg[clk].bit)) ? 1 : 0;
         break;
      case REG_SCGC5:
         enabled = (SIM_SCGC5 & (1 << sClkToReg[clk].bit)) ? 1 : 0;
         break;
      case REG_SCGC6:
         enabled = (SIM_SCGC6 & (1 << sClkToReg[clk].bit)) ? 1 : 0;
         break;
      case REG_SCGC7:
         enabled = (SIM_SCGC7 & (1 << sClkToReg[clk].bit)) ? 1 : 0;
         break;
      }
   }

   return enabled;
}

