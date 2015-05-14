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
// This code was created to provide a simple interface to the Freedom boards
// touch pad hardware.  The code is based on Andrew Payne's work (see touch.c
// for his copyright notice).
//
// It is important to note that this driver, once initialized, produces a
// constant stream of timer callbacks and ISRs.  The method used to acquire
// the touch data is via constant scanning of the two channels involved.  This
// scanning occurs at a rate defined by TOUCH_SCAN_TIME.  This time is the
// delay between scan attempts.  Note that since two channels are scanned for
// the touch pad, each channel is scanned at a rate twice the TOUCH_SCAN_TIME.
// In other words, each channel is scan every other timer expiration.
// ----------------------------------------------------------------------------

#ifndef _TOUCH_H_
#define _TOUCH_H_

// One-time initialization of the touch framework.
extern void touchInit(uint32_t channelMask);
// Used for retrieving the most recent touch data.
extern int  touchData(int channel);

#endif // _TOUCH_H_

