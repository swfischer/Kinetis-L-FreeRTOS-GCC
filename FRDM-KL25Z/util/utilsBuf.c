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

#include <stdlib.h> // for NULL
#include <stdint.h>

#include "os.h"
#include "utilsBuf.h"

#define STATE_VALID  (0xC8)

static int getCount(utilsBuf_t *obj);

// ----------------------------------------------------------------------------
// External Functions
// ----------------------------------------------------------------------------

int utilsBufInit(utilsBuf_t *obj)
{
   int error = -1;

   if (obj == NULL || obj->buf == NULL || obj->size <= 2 || obj->size > UINT8_MAX)
   {
      // Just return an error
   }
   else
   {
      obj->state = STATE_VALID;
      utilsBufReset(obj);
      error = 0;
   }
   
   return error;
}

int utilsBufCount(utilsBuf_t *obj)
{
   int cnt = 0;

   if (obj != NULL && obj->state == STATE_VALID)
   {
      cnt = getCount(obj);
   }

   return cnt;
}

uint8_t utilsBufPeek(utilsBuf_t *obj)
{
   uint8_t byte = 0;

   if (obj != NULL && obj->state == STATE_VALID)
   {
      int cnt = getCount(obj);

      if (cnt > 0)
      {
         int i = obj->wrIdx - 1;

         if (i < 0)
         {
            i += obj->size;
         }

         byte = obj->buf[i];
      }
   }

   return byte;
}

void utilsBufPop(utilsBuf_t *obj)
{
   if (obj != NULL && obj->state == STATE_VALID)
   {
      if (obj->rdIdx != obj->wrIdx)
      {
         obj->wrIdx --;
         if (obj->wrIdx < 0)
         {
            obj->wrIdx = obj->size - 1;
         }
      }
   }
}

uint8_t utilsBufPull(utilsBuf_t *obj)
{
   uint8_t byte = 0;

   if (obj != NULL && obj->state == STATE_VALID)
   {
      if (obj->rdIdx != obj->wrIdx)
      {
         byte = obj->buf[obj->rdIdx++];
         if (obj->rdIdx >= obj->size)
         {
            obj->rdIdx = 0;
         }
      }
   }

   return byte;
}

void utilsBufPush(utilsBuf_t *obj, uint8_t byte)
{
   if (obj != NULL && obj->state == STATE_VALID)
   {
      int idx = obj->wrIdx;

      obj->buf[idx++] = byte;
      if (idx >= obj->size)
      {
         idx = 0;
      }
      
      // Prevent overflowing, drop the new byte instead
      if (idx != obj->rdIdx)
      {
         obj->wrIdx = idx;
      }
   }
}

void utilsBufReset(utilsBuf_t *obj)
{
   if (obj != NULL && obj->state == STATE_VALID)
   {
      obj->rdIdx = 0;
      obj->wrIdx = 0;
   }
}

// ----------------------------------------------------------------------------
// Local Functions
// ----------------------------------------------------------------------------

static int getCount(utilsBuf_t *obj)
{
   int cnt = 0;

   cnt = obj->wrIdx - obj->rdIdx;
   if (cnt < 0)
   {
      cnt += obj->size;
   }

   return cnt;
}
