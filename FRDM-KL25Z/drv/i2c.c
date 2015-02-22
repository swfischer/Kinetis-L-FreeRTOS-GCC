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
// Portions based on Andrew Payne's accel demo
//
//  Copyright (c) 2012-2013 Andrew Payne <andy@payne.org>
//
// Based on demo example from Freescale
// ----------------------------------------------------------------------------

#include <stdint.h>

#include "clk.h"
#include "frdmCfg.h"
#include "i2c.h"
#include "MKL25Z4.h"
#include "os.h"
#include "utils.h"

#define I2C_READ_CMD    (1)
#define I2C_WRITE_CMD   (0)

#define I2C_ACK(p)      (p->C1 &= ~I2C_C1_TXAK_MASK)
#define I2C_NAK(p)      (p->C1 |=  I2C_C1_TXAK_MASK)
#define I2C_S_ACK(p)    (((p->S & I2C_S_RXAK_MASK) == 0) ? 1 : 0)

#define I2C_MASTER(p)   (p->C1 |=  I2C_C1_MST_MASK)
#define I2C_SLAVE(p)    (p->C1 &= ~I2C_C1_MST_MASK)

#define I2C_RX(p)       (p->C1 &= ~I2C_C1_TX_MASK)
#define I2C_TX(p)       (p->C1 |=  I2C_C1_TX_MASK)

#define I2C_START(p)    { I2C_MASTER(p); I2C_TX(p); }
#define I2C_START_REPEAT(p)   (p->C1 |= I2C_C1_RSTA_MASK)
#define I2C_STOP(p)     { I2C_SLAVE(p); I2C_RX(p); }

#define I2C_READ(p)     (p->D)
#define I2C_WRITE(p,x)  (p->D = x)

static osMutexId sBus0Lock = NULL; // Used to control I2C bus 0 use
static osMutexId sBus1Lock = NULL; // Used to control I2C bus 1 use

static int  determineBaudRateRegValue(uint32_t rateHz, uint8_t *fReg);
static clkGate_t determineClock(i2cDevice_t *dev);
static inline void i2cWait(I2C_MemMapPtr p);
static void testBaudRates( uint32_t rateHz, int icrLo, int icrHi
                         , uint32_t *closestRate, int *closestMult, int *closestIcr);

// ----------------------------------------------------------------------------
// External Functions
// ----------------------------------------------------------------------------

int i2cDeviceInit(i2cDevice_t *dev, I2C_MemMapPtr base, uint8_t devAddr, uint32_t rateHz)
{
   uint8_t fReg = 0;
   int rc = -1;

   if (dev == NULL || base == NULL || (base != I2C0_BASE_PTR && base != I2C1_BASE_PTR))
   {
      // Invalid parameter(s)
   }
   else if (determineBaudRateRegValue(rateHz, &fReg) != 0)
   {
      // Invalid rate request
   }
   else
   {
      dev->base = base;
      dev->devAddr = devAddr;
      dev->fReg = fReg;
      dev->acquired = 0;

      if (base == I2C0_BASE_PTR && sBus0Lock == NULL)
      {
         sBus0Lock = osMutexCreate(NULL);
      }
      else if (sBus1Lock == NULL)
      {
         sBus0Lock = osMutexCreate(NULL);
      }

      rc = 0;
   }

   return rc;
}

int i2cAcquireBus(i2cDevice_t *dev, uint32_t waitMs)
{
   osStatus status;
   int rc = -1;

   if (dev->base == I2C0_BASE_PTR)
   {
      status = osMutexWait(sBus0Lock, waitMs);
   }
   else
   {
      status = osMutexWait(sBus1Lock, waitMs);
   }
   
   if (status == osOK)
   {
      // Enable clocks
      clkEnable(determineClock(dev));

      dev->base->F  = I2C_F_ICR(dev->fReg);
      dev->base->C1 = I2C_C1_IICEN_MASK;

      dev->acquired = 1;
      rc = 0;
   }

   return rc;
}

void i2cReleaseBus(i2cDevice_t *dev)
{
   if (dev->acquired)
   {
      if (dev->base == I2C0_BASE_PTR)
      {
         osMutexRelease(sBus0Lock);
      }
      else
      {
         osMutexRelease(sBus1Lock);
      }

      clkDisable(determineClock(dev));

      dev->acquired = 0;
   }
}

int i2CReadAddr8(i2cDevice_t *dev, uint8_t addr)
{
   volatile int data = -1;

   if (dev->acquired)
   {
      utilsHwDelay(1);

      I2C_START(dev->base);
      I2C_WRITE(dev->base, dev->devAddr | I2C_WRITE_CMD);
      i2cWait(dev->base);
      I2C_WRITE(dev->base, addr);
      i2cWait(dev->base);
      I2C_START_REPEAT(dev->base);
      I2C_WRITE(dev->base, dev->devAddr | I2C_READ_CMD);
      i2cWait(dev->base);
      I2C_RX(dev->base);
      I2C_NAK(dev->base);
      data = I2C_READ(dev->base);
      i2cWait(dev->base);
      I2C_STOP(dev->base);

      data = I2C_READ(dev->base);
   }

   return data;   
}

int i2CWriteAddr8(i2cDevice_t *dev, uint8_t addr, uint8_t data)
{
   int rc = -1;

   if (dev->acquired)
   {
      utilsHwDelay(1);

      I2C_START(dev->base);
      I2C_WRITE(dev->base, dev->devAddr | I2C_WRITE_CMD);
      i2cWait(dev->base);
      I2C_WRITE(dev->base, addr);
      i2cWait(dev->base);
      I2C_WRITE(dev->base, data);
      i2cWait(dev->base);
      I2C_STOP(dev->base);

      rc = (I2C_S_ACK(dev->base)) ? 0 : -1;
   }

   return rc;
}

// ----------------------------------------------------------------------------
// Local Functions
// ----------------------------------------------------------------------------

static int determineBaudRateRegValue(uint32_t rateHz, uint8_t *fReg)
{
   uint32_t closestRate = 0;
   int closestMult;
   int closestIcr;
   int rc = -1;

   // Run through all non-varying combinations until one matches or we find the closest
   testBaudRates(rateHz, 16, 64, &closestRate, &closestMult, &closestIcr);
   
   // If the closest is more that 10% off, try the varying combinations
   if (closestRate < ((rateHz * 9) / 10))
   {
      testBaudRates(rateHz, 0, 15, &closestRate, &closestMult, &closestIcr);
   }

   if (closestRate > 0)
   {
      *fReg = (uint8_t) (((closestMult & 0x3) << 6) | (closestIcr & 0x3f));
      rc = 0;
   }

   return rc;
}

static clkGate_t determineClock(i2cDevice_t *dev)
{
   clkGate_t clk;

   if (dev->base == I2C0_BASE_PTR)
   {
      clk = CLK_I2C0;
   }
   else
   {
      clk = CLK_I2C1;
   }

   return clk;
}

static inline void i2cWait(I2C_MemMapPtr p)
{
   // Spin wait for the interrupt flag to be set
   while((p->S & I2C_S_IICIF_MASK) == 0)
      ;

   p->S |= I2C_S_IICIF_MASK;           // Clear flag
}

// The "closestRate" parameter is both an input and an output
static void testBaudRates( uint32_t rateHz, int icrLo, int icrHi
                         , uint32_t *closestRate, int *closestMult, int *closestIcr
                         )
{
   static const uint16_t sclDiv[] =
   { 20, 22, 24, 26, 28, 30, 34, 40, 28, 32
   , 36, 40, 44, 48, 56, 68, 48, 56, 64, 72
   , 80, 88, 104, 128, 80, 96, 112, 128, 144, 160
   , 192, 240, 160, 192, 224, 256, 288, 320, 384, 480
   , 320, 384, 448, 512, 576, 640, 768, 960, 640, 768
   , 896, 1024, 1152, 1280, 1536, 1920, 1280, 1536, 1792, 2048
   , 2304, 2560, 3072, 3840
   };
   uint32_t rate;
   int match = 0;
   int mult;
   int i;
   int j;

   i = 0;
   while (!match && i < 3)
   {
      mult = (1 << i); // 1, 2, or 4

      for (j = icrLo; j < icrHi; j++)
      {
         // Calculate the rate
         rate = BUS_CLOCK / (mult * sclDiv[j]);

         if (rate == rateHz)
         {
            *closestRate = rate;
            *closestMult = i;
            *closestIcr = j;
            match = 1;
            break;
         }
         // The clock must be no greater that the requested rate
         else if ((rate < rateHz) && (rate > *closestRate))
         {
            *closestRate = rate;
            *closestMult = i;
            *closestIcr = j;
         }
      }

      i ++;
   }
}
