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
// This code was created to provide a not so generic driver for a console type
// use of a serial connection.
//
// Several console specific assumptions were made while writing this driver,
// mostly just to reduce complexity and overhead:
//
// a) It is assumed that a carriage return character ends a line of input.
// b) It is assumed that echoing of character input is desired.
// c) It is assumed that backspace and delete character handling is desired.
//
// Beyond this the driver is generic, but the above are fairly specific so the
// driver is assumed to be console specific.
//
// The input path produces an interrupt for every character received, thus
// allowing the per-character processing needed on the input.  The output path
// is DMA based and only produces one interrupt per write, which is when the
// transmission completes.
// ----------------------------------------------------------------------------

#ifndef _UART_H_
#define _UART_H_

#include <stdint.h>
#include <stdbool.h>

#include "frdmCfg.h"

#define UART_EVENT_READ_BIT   (1 << 0)
#define UART_EVENT_WRITE_BIT  (1 << 1)

// The UART callback function pattern.
typedef void (*uartCb)(int event);

// One-time initialization of the console UART driver.
extern int  uartInit(uint32_t baud, uartCb callback);

// Used for disabling the driver auto-echo feature while writes are occurring.
// This is needed to prevent interference with the write operations since the
// echo feature drops the echo'd characters right into the output FIFO, hence
// can overwrite characters and more importantly cause early completions thus
// truncating the write output.
extern void uartEchoEnable(bool en);

// Used for retrieving an input line into the given buffer.
extern int  uartRead(uint8_t *buf, uint16_t len);
// Used for determining how many input characters are waiting to be read.
extern int  uartReadCnt(void);
// Used for clearing the input character buffer.
extern void uartReadFlush(void);

// Used for outputting a line of text over the UART.
extern int  uartWrite(uint8_t *buf, uint16_t len);

#endif // _UART_H_

