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
// This code was created to provide a means of reading and writing to ADC
// devices while still sharing the device with other potential users.  In order
// to perform a read or write of an ADC device, the device must first be
// acquired which allows for exclusive use of the device, hence user A can't
// clobber user B's operations.  Once a user's current ADC needs are completed,
// the bus should be released to allow other users the chance of using the
// device.
// ----------------------------------------------------------------------------

#ifndef _ADC_H_
#define _ADC_H_

#include <stdint.h>

#include "kinetis.h"

typedef struct
{
   ADC_MemMapPtr base;
   uint8_t acquired;

} adcDevice_t;

// One-time initialization on a users "adcDevice_t" object.
// Returns 0 on success, otherwise -1
extern int  adcDeviceInit(adcDevice_t *dev, ADC_MemMapPtr base);

// Used for acquiring exclusive use of a given ADC device.
// Returns 0 on success, otherwise -1
extern int  adcAcquire(adcDevice_t *dev, uint32_t waitMs);
// Used for releasing a previously acquired exclusive use of a given ADC device.
extern void adcRelease(adcDevice_t *dev);

// Used for performing read of a given channel on a given acquired ADC device.
extern uint16_t adcRead(adcDevice_t *dev, int channel);

#endif // _ADC_H_

