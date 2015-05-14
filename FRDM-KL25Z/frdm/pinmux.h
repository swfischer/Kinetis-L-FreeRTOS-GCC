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
// This code was created to provide a single location where all pin mux'ing is
// performed.  The idea here is that pin mux'ing is board specific not CPU
// specific, thus in order to write more generic drivers the board specific
// pieces should not live with the drivers themselfs, hence this code.  Also,
// by placing all pin mux'ing is a single place it becomes much easier to see
// the full picture, thus avoiding issues with conflicting uses of pins or at
// being aware of them at the earilest point in time.
//
// All pin mux'ing is configured based on entries in the "muxConfig[]" table.
// The mux'ing options are all available for use in this table, including the
// interrupt settings.  One entry equals one pin.
// ----------------------------------------------------------------------------

#ifndef _PINMUX_H_
#define _MUX_H_

// One-time initialization of the chip pin muxes.
extern void pinmuxInit(void);

#endif // _PINMUX_H_

