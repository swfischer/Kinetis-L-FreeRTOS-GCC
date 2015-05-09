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
// This code was created to provide a framework for producing the PWM signals
// used to control hobby servo motors for up to 16 separate motors.
//
// The approach here is to use a single PIT channel and interrupt.  The PIT is
// used to produce interrupts at the apropriate times for PWM edge transitions.
// The cycle time for a single PWM signal is set at 40ms.  This period is
// broken into 16 slots or channels and during each channel a single PWM pulse
// is produced.  The minimum PWM pulse time within a channel is 1ms and the
// maximum pulse time is 2ms.  This varying 1ms period, the time between the
// min and max pulse time, has a 1us resolution.
//
// All toll, if all 16 channels were active, the CPU would need to handle 32
// ISRs per 40ms, or 800 ISRs per second.  This is a lot of ISRs, but at 48MHz
// this micro-processor should be able to handle the load.
// ----------------------------------------------------------------------------

#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

#define SERVO_MAX_CHANNELS      (16)
#define SERVO_INVALID_CHANNEL   (0xff)

#define SERVO_MODE_POSITIONAL          (1)
#define SERVO_MODE_CONTINUOUS_ROTATION (2)

// Initialize the servo framework for "channels" channels.
// Returns 0 on success, otherwise -1.
extern int  servoInit(uint8_t channels);
// Terminate the servo framework.
extern void servoTerm(void);

// Open a servo channel.
// Returns a servo ID upon success, otherwise SERVO_INVALID_CHANNEL.
extern uint8_t servoOpen(uint8_t mode, uint8_t gpioId);
// Close an opened servo channel.
extern void servoClose(uint8_t id);

// Enable the output of the servo PWM signal.
extern int  servoEnable(uint8_t id);
// disable the output of the servo PWM signal.
extern void servoDisable(uint8_t id);

// Move the servo to a position given in degrees (0 to 180).
// This routine can be used regardless of the mode.
// Returns 0 on success, otherwise -1.
extern int servoMoveDegree(uint8_t id, uint8_t degrees);
// Move the servo to a position given as a value (0 to 1000).
// The given position value is a number between 0 and 1000
// which can be thought of as the number of micro-seconds that
// the pulse will be active beyond the minimum 1ms pulse width.
// This routine can be used regardless of the mode.
// Returns 0 on success, otherwise -1.
extern int servoMovePosition(uint8_t id, uint16_t position);

// Set the stopped position for a continuous rotation servo.
// The "position" parameter has the same possible values as
// the "servoMovePosition()" parameter.  The default stopped
// value is half of the position range.
// Returns 0 on success, otherwise -1.
extern int servoCrStoppedPosition(uint8_t id, uint16_t position);
// Rotate the servo at the given percentage value (-100 to 100).
// A negative percent rotates in one direction and a positive
// percent rotates in the opposite direction.  A value of zero
// will configure for the "stopped" position.
// Returns 0 on success, otherwise -1.
extern int servoCrMovePercent(uint8_t id, int8_t percent);

#endif // SERVO_H

