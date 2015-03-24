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

#ifndef _UTILSBUF_H_
#define _UTILSBUF_H_

#include <stdint.h>

// This utility is a circular buffer of bytes object which handles only a small
// size buffer, less than or equal to 256 bytes.

typedef struct
{
   // Must be set when utilBufInit() is called
   uint8_t *buf;  // Pointer to the buffer to use/manage
   uint8_t size;  // the size in bytes of "buf"

   // Init'd/managed by the code
   uint8_t rdIdx;
   uint8_t wrIdx;
   uint8_t state;
} utilsBuf_t;

// Init the buffer object.  "buf" and "size" must be properly set when called
// Returns 0 on success, otherwise -1
extern int utilsBufInit(utilsBuf_t *obj);
// Returns the number of bytes on the buffer
extern int utilsBufCount(utilsBuf_t *obj);
// Returns the idx'th byte on the buffer
extern uint8_t utilsBufPeekIdx(utilsBuf_t *obj, uint8_t idx);
// Throw away the buffer's top byte
extern void utilsBufPop(utilsBuf_t *obj);
// Returns the buffer's top byte
extern uint8_t utilsBufPull(utilsBuf_t *obj);
// Push a byte to the buffer
extern void utilsBufPush(utilsBuf_t *obj, uint8_t byte);
// Reset the buffer to the empty state
extern void utilsBufReset(utilsBuf_t *obj);

#endif // _UTILSBUF_H_
