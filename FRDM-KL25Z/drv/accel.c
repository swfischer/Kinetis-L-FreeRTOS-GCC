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

#include "accel.h"
#include "clk.h"
#include "frdmCfg.h"
#include "i2c.h"
#include "kinetis.h"
#include "os.h"
#include "utils.h"

#ifdef MMA8451_SA0_ZERO
#define MMA8451_I2C_ADDRESS (0x1c << 1)
#else // MMA8451_SA0_ONE
#define MMA8451_I2C_ADDRESS (0x1d << 1)
#endif

#define MMA8451_BAUD_RATE   (300000)

#define MMA8451_OUT_X_MSB   (0x01)
#define MMA8451_OUT_X_LSB   (0x02)
#define MMA8451_OUT_Y_MSB   (0x03)
#define MMA8451_OUT_Y_LSB   (0x04)
#define MMA8451_OUT_Z_MSB   (0x05)
#define MMA8451_OUT_Z_LSB   (0x06)
#define MMA8451_CTRL_REG1   (0x2a)

static i2cDevice_t mma8451device;

static int16_t mma8451ReadAxis(uint8_t addr);
static uint8_t mma8451ReadReg(uint8_t addr);
static void mma8451WriteReg(uint8_t addr, uint8_t data);

// ----------------------------------------------------------------------------
// External Functions
// ----------------------------------------------------------------------------

int accelInit(void)
{
   uint8_t tmp;
   int rc = 0;

   if (i2cDeviceInit(&mma8451device, ACCEL_I2C, MMA8451_I2C_ADDRESS, MMA8451_BAUD_RATE) != 0)
   {
      rc = -1;
   }
   else if (i2cAcquireBus(&mma8451device, WAIT_FOREVER) != 0)
   {
      rc = -1;
   }
   else
   {
      tmp = mma8451ReadReg(MMA8451_CTRL_REG1);
      mma8451WriteReg(MMA8451_CTRL_REG1, tmp | 0x01); // ACTIVE = 1

      i2cReleaseBus(&mma8451device);
   }

   return rc;
}

int accelGet(int16_t *x, int16_t *y, int16_t *z)
{
   int rc = 0;
   if (i2cAcquireBus(&mma8451device, WAIT_FOREVER) != 0)
   {
      rc = -1;
   }
   else
   {
      if (x)
      {
         *x = mma8451ReadAxis(MMA8451_OUT_X_MSB);
      }
      if (y)
      {
         *y = mma8451ReadAxis(MMA8451_OUT_Y_MSB);
      }
      if (z)
      {
         *z = mma8451ReadAxis(MMA8451_OUT_Z_MSB);
      }

      i2cReleaseBus(&mma8451device);
   }

   return rc;
}

// ----------------------------------------------------------------------------
// Local Functions
// ----------------------------------------------------------------------------

static int16_t mma8451ReadAxis(uint8_t addr)
{
   return (int16_t)((mma8451ReadReg(addr) << 8) | (mma8451ReadReg(addr + 1))) >> 2;
}

static uint8_t mma8451ReadReg(uint8_t addr)
{
   int v;

   utilsHwDelay(1);

   v = i2CReadAddr8(&mma8451device, addr);
   if (v == -1)
   {
      v = 0;
   }

   return (uint8_t) v;
}

static void mma8451WriteReg(uint8_t addr, uint8_t data)
{
   utilsHwDelay(1);
   i2CWriteAddr8(&mma8451device, addr, data);
}
