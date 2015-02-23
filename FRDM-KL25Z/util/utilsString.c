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

#include <limits.h>

#include "utils.h"

static uint32_t _strto_l(const char *str, char **endptr, int base, int uflag);

// ----------------------------------------------------------------------------
// External Functions
// ----------------------------------------------------------------------------

int utilsIsSpace(const char c)
{
   int space = 0;

   if (c == ' ' || c == '\t' || c == '\n' || c == '\v' || c == '\f' || c == '\r')
   {
      space = 1;
   }

   return space;   
}

int utilsStrcmp(const char *s1, const char *s2)
{
   int ret = 0;

   while (!(ret = *(unsigned char *) s1 - *(unsigned char *) s2) && *s2) ++s1, ++s2;

   if (ret < 0)
      ret = -1;
   else if (ret > 0)
      ret = 1 ;

   return ret;
}

uint32_t utilsStrlen(const char *str)
{
   const char *s;

   for (s = str; *s; ++s)
      ;

   return (s - str);
}

int32_t utilsStrtol(const char *str, char **endptr, int base)
{
    return _strto_l(str, endptr, base, 0);
}

uint32_t utilsStrtoul(const char *str, char **endptr, int base)
{
    return _strto_l(str, endptr, base, 1);
}

// ----------------------------------------------------------------------------
// Local Functions
// ----------------------------------------------------------------------------

static uint32_t _strto_l(const char *str, char **endptr, int base, int uflag)
{
   uint32_t number = 0;
   uint32_t cutoff;
   char *pos = (char *) str;
   char *fail_char = (char *) str;
   int digit, cutoff_digit;
   int negative;

   while (utilsIsSpace(*pos)) {	/* skip leading whitespace */
      ++pos;
   }

   /* handle optional sign */
   negative = 0;
   switch(*pos) {
   case '-': negative = 1;	/* fall through to increment pos */
   case '+': ++pos;
   }

   if ((base == 16) && (*pos == '0')) { /* handle option prefix */
      ++pos;
      fail_char = pos;
      if ((*pos == 'x') || (*pos == 'X')) {
         ++pos;
      }
   }
   
   if (base == 0) {		/* dynamic base */
      base = 10;		/* default is 10 */
      if (*pos == '0') {
         ++pos;
         base -= 2;		/* now base is 8 (or 16) */
         fail_char = pos;
         if ((*pos == 'x') || (*pos == 'X')) {
            base += 8;	/* base is 16 */
            ++pos;
         }
      }
   }

   if ((base < 2) || (base > 36)) { /* illegal base */
      goto DONE;
   }

   cutoff = ULONG_MAX / base;
   cutoff_digit = ULONG_MAX - cutoff * base;

   while (1) {
      digit = 40;
      if ((*pos >= '0') && (*pos <= '9')) {
         digit = (*pos - '0');
      } else if (*pos >= 'a') {
         digit = (*pos - 'a' + 10);
      } else if (*pos >= 'A') {
         digit = (*pos - 'A' + 10);
      } else break;

      if (digit >= base) {
         break;
      }

      ++pos;
      fail_char = pos;

      /* adjust number, with overflow check */
      if ((number > cutoff)
            || ((number == cutoff) && (digit > cutoff_digit))) {
         number = ULONG_MAX;
         if (uflag) {
            negative = 0; /* since unsigned returns ULONG_MAX */
         }
      } else {
         number = number * base + digit;
      }

   }

DONE:
   if (endptr) {
      *endptr = fail_char;
   }

   if (negative) {
      if (!uflag && (number > ((unsigned long)(-(1+LONG_MIN)))+1)) {
         return (unsigned long) LONG_MIN;
      }
      return (unsigned long)(-((long)number));
   } else {
      if (!uflag && (number > (unsigned long) LONG_MAX)) {
         return LONG_MAX;
      }
      return number;
   }
}
