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
// This task was created to provide a set of LED behaviors based on a "mode"
// settable through a console command.  The three basic modes are:
//
// GLOW  - in this mode the LED is slowly cycled in brightness from off to fully
//         on and back to off.  Only one color is cycled at a time.
// TOUCH - in this mode the LED brightness is set via the position of the most
//         recent touch pad sample.  One one color is used.
// ACCEL - in this mode the three colors of the LED are controlled by the three
//         axis of the accelerometer.  The red color is controlled by the X
//         axis.  The green color is controlled by the Y axis.  The blue color
//         is controlled by the Z axis.
// ----------------------------------------------------------------------------

#ifndef _LEDTASK_H_
#define _LEDTASK_H_

enum
{ LED_MODE_GLOW = 0 // All three colors, one after the other
, LED_MODE_GLOW_RED
, LED_MODE_GLOW_GREEN
, LED_MODE_GLOW_BLUE
, LED_MODE_TOUCH // Same as TOUCH_BLUE
, LED_MODE_TOUCH_RED
, LED_MODE_TOUCH_GREEN
, LED_MODE_TOUCH_BLUE
, LED_MODE_ACCEL //
};   

// The task main function. 
extern void ledTaskEntry( void *pParameters );
// Used for setting the LED operating mode.
extern void ledSetMode(int mode);

#endif // _LEDTASK_H_

