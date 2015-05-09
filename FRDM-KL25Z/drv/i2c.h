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
// This code was created to provide a means of reading and writing to I2C bus
// devices while still sharing the bus with other potential devices.  In order
// to perform a read or write of an I2C device, the bus must first be acquired
// which allows for exclusive use of the bus, hence user A can't clobber user
// B's I2C operations.  Once a user's current I2C read and write needs are
// completed, the bus should be released to allow other users the chance of
// using the bus.
// ----------------------------------------------------------------------------

#ifndef _I2C_H_
#define _I2C_H_

#include <stdint.h>

#include "kinetis.h"

typedef struct
{
   I2C_MemMapPtr base;
   uint8_t devAddr;
   uint8_t fReg;
   uint8_t acquired;

} i2cDevice_t;

// One-time initialization on a users "i2cDevice_t" object.
// Returns 0 on success, otherwise -1.
extern int  i2cDeviceInit(i2cDevice_t *dev, I2C_MemMapPtr base, uint8_t devAddr, uint32_t rateHz);

// Used for acquiring exclusive use of a given I2C bus.
// Returns 0 on success, otherwise -1.
extern int  i2cAcquireBus(i2cDevice_t *dev, uint32_t waitMs);
// Used for releasing a previously acquired exclusive use of an I2C bus.
extern void i2cReleaseBus(i2cDevice_t *dev);

// Used for performing an 8-bit I2C read operation from a given I2C device.
// Returns the requested data (unsigned 8 bit value) or -1 on error.
extern int i2CReadAddr8(i2cDevice_t *dev, uint8_t addr);
// Used for performing an 8-bit I2C write operation to a given I2C device.
// Returns 0 on success, otherwise -1.
extern int i2CWriteAddr8(i2cDevice_t *dev, uint8_t addr, uint8_t data);

#endif // _I2C_H_

