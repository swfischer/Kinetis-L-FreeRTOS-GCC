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

#include <stddef.h>  // For NULL

#include "accel.h"
#include "clk.h"
#include "console.h"
#include "consoleCmds.h"
#include "consoleTask.h"
#include "frdmCfg.h"
#include "ledTask.h"
#include "os.h"
#include "touch.h"
#include "utils.h"

#define IS_1_TOKEN()      (count == 1)
#define IS_2_TOKENS()     (count == 2)
#define IS_3_TOKENS()     (count == 3)
#define IS_4_TOKENS()     (count == 4)
#define IS_5_TOKENS()     (count == 5)
#define TOKEN_MATCH(x,y)  (utilsStrcmp(token[x],y) == 0)
#define TOKEN_VALUE(x)    (utilsStrtoul(token[x], NULL, 0))

static bool commandAccel(int count, char **token);
static bool commandClock(int count, char **token);
static bool commandLed(int count, char **token);
static bool commandMem(int count, char **token);
static bool commandReboot(int count, char **token);
static bool commandTouch(int count, char **token);
static bool commandVersion(int count, char **token);

// ----------------------------------------------------------------------------
// External Functions
// ----------------------------------------------------------------------------

void consoleCmdsInit(void)
{
   consoleTaskRegisterCommand("accel", commandAccel);
   consoleTaskRegisterCommand("clock", commandClock);
   consoleTaskRegisterCommand("led", commandLed);
   consoleTaskRegisterCommand("mem", commandMem);
   consoleTaskRegisterCommand("reboot", commandReboot);
   consoleTaskRegisterCommand("touch", commandTouch);
   consoleTaskRegisterCommand("version", commandVersion);
}

// ----------------------------------------------------------------------------
// Local Functions
// ----------------------------------------------------------------------------

static bool commandAccel(int count, char **token)
{
   int16_t x, y, z;

   if ((IS_2_TOKENS() || IS_3_TOKENS()) && TOKEN_MATCH(1, "help"))
   {
      consolePrintf("%s - command to view accelerometer data\n", token[0]);

      if (IS_3_TOKENS())
      {
         consolePrintf("\nCommand options are:\n\n");
         consolePrintf("  <none> - shows the data once\n");
         consolePrintf("  timed - shows the data twice per second until stopped\n");
      }
   }
   else if (IS_2_TOKENS() && TOKEN_MATCH(1, "timed"))
   {
      consolePrintf("\n<press any key to stop>\n");
      while (consoleGetInputCnt() == 0)
      {
         accelGet(&x, &y, &z);
         consolePrintf("Accelerometer X/Y/Z = %d/%d/%d\n", x, y, z);
         osDelay( 500 ); // 500ms
      }
   }
   else
   {
      accelGet(&x, &y, &z);
      consolePrintf("Accelerometer X/Y/Z = %d/%d/%d\n", x, y, z);
   }

   return true;
}

static bool commandClock(int count, char **token)
{
   int i;

   // Check for a help request
   if ((IS_2_TOKENS() || IS_3_TOKENS()) && TOKEN_MATCH(1, "help"))
   {
      consolePrintf("%s - command to interact with clock gates\n", token[0]);
      
      if (IS_3_TOKENS())
      {
         consolePrintf("\nCommand options are:\n\n");
         consolePrintf("  <none> - lists the current clock gate state\n");
         consolePrintf("  disable <gate> - disable a given clock\n");
         consolePrintf("  enable <gate> - enable a given clock\n");
      }
   }
   else if (IS_3_TOKENS() && TOKEN_MATCH(1, "disable"))
   {
      for (i = 0; i < CLK_NUMBER_OF_CLKS; i++)
      {
         if (TOKEN_MATCH(2, clkGetName((clkGate_t)i)))
         {
            clkDisable((clkGate_t)i);
            break;
         }
      }
   }
   else if (IS_3_TOKENS() && TOKEN_MATCH(1, "enable"))
   {
      for (i = 0; i < CLK_NUMBER_OF_CLKS; i++)
      {
         if (TOKEN_MATCH(2, clkGetName((clkGate_t)i)))
         {
            clkEnable((clkGate_t)i);
            break;
         }
      }
   }
   else // List clock gates
   {
      consolePrintf("   Name: EN - Cnt\n  ----------------\n");
      for (i = 0; i < CLK_NUMBER_OF_CLKS; i++)
      {
         consolePrintf( "%07s: %s - %d\n"
                      , clkGetName((clkGate_t)i)
                      , (clkIsEnabled((clkGate_t)i)) ? "on " : "off"
                      , clkGetUseCount((clkGate_t)i)
                      );
      }
   }

   return true;
}

static bool commandLed(int count, char **token)
{
   bool handled = true;

   if ((IS_2_TOKENS() || IS_3_TOKENS()) && TOKEN_MATCH(1, "help"))
   {
      consolePrintf("%s - command to configure LED updates\n", token[0]);

      if (IS_3_TOKENS())
      {
         consolePrintf("\nCommand options are:\n\n");
         consolePrintf("  glow - set state to glow all 3 colors\n");
         consolePrintf("  glow <color> - set state to glow in one color\n");
         consolePrintf("    <color> values can be \"red\", \"green\", or \"blue\"\n");
         consolePrintf("  touch - set state to change blue brightness based on touch pad\n");
         consolePrintf("  touch <color> - set state to change brightness based on touch pad\n");
         consolePrintf("  accel - set state to change color based on the accelerometer\n");
      }
   }
   else if (IS_2_TOKENS() && TOKEN_MATCH(1, "glow"))
   {
      consolePrintf("Setting LED glow mode\n");
      ledSetMode(LED_MODE_GLOW);
   }
   else if (IS_3_TOKENS() && TOKEN_MATCH(1, "glow") && TOKEN_MATCH(2, "red"))
   {
      consolePrintf("Setting LED glow red mode\n");
      ledSetMode(LED_MODE_GLOW_RED);
   }
   else if (IS_3_TOKENS() && TOKEN_MATCH(1, "glow") && TOKEN_MATCH(2, "green"))
   {
      consolePrintf("Setting LED glow green mode\n");
      ledSetMode(LED_MODE_GLOW_GREEN);
   }
   else if (IS_3_TOKENS() && TOKEN_MATCH(1, "glow") && TOKEN_MATCH(2, "blue"))
   {
      consolePrintf("Setting LED glow blue mode\n");
      ledSetMode(LED_MODE_GLOW_BLUE);
   }
   else if (IS_2_TOKENS() && TOKEN_MATCH(1, "touch"))
   {
      consolePrintf("Setting LED touch mode\n");
      ledSetMode(LED_MODE_TOUCH);
   }
   else if (IS_3_TOKENS() && TOKEN_MATCH(1, "touch") && TOKEN_MATCH(2, "red"))
   {
      consolePrintf("Setting LED touch red mode\n");
      ledSetMode(LED_MODE_TOUCH_RED);
   }
   else if (IS_3_TOKENS() && TOKEN_MATCH(1, "touch") && TOKEN_MATCH(2, "green"))
   {
      consolePrintf("Setting LED touch green mode\n");
      ledSetMode(LED_MODE_TOUCH_GREEN);
   }
   else if (IS_3_TOKENS() && TOKEN_MATCH(1, "touch") && TOKEN_MATCH(2, "blue"))
   {
      consolePrintf("Setting LED touch blue mode\n");
      ledSetMode(LED_MODE_TOUCH_BLUE);
   }
   else if (IS_2_TOKENS() && TOKEN_MATCH(1, "accel"))
   {
      consolePrintf("Setting LED accelerometer mode\n");
      ledSetMode(LED_MODE_ACCEL);
   }
   else
   {
      handled = false;
   }

   return handled;
}

static bool commandMem(int count, char **token)
{
   bool handled = true;
   uint32_t *p;
   uint32_t cnt;
   int i;

   // Check for a help request
   if ((IS_2_TOKENS() || IS_3_TOKENS()) && TOKEN_MATCH(1, "help"))
   {
      consolePrintf("%s - command to view memory space\n", token[0]);

      if (IS_3_TOKENS())
      {
         consolePrintf("\nCommand options are:\n\n");
         consolePrintf("  <addr> - view a 32 bit word of data at <addr>\n");
         consolePrintf("      NOTE: Hex addresses need to begin with \"0x\"\n");
         consolePrintf("  <addr> <cnt> - view <cnt> 32 bit words of data starting from <addr>\n");
      }
   }
   else if (IS_2_TOKENS())
   {
      p = (uint32_t*) ((TOKEN_VALUE(1)) & 0xfffffffc);  // Align to a word boundary
      consolePrintf("%08x: %08x\n", (uint32_t)p, p[0]);
      
   }
   else if (IS_3_TOKENS())
   {
      p = (uint32_t*) ((TOKEN_VALUE(1)) & 0xfffffffc);  // Align to a word boundary
      cnt = (TOKEN_VALUE(1)) / 2;

      for (i = 0; i < cnt; i++)
      {
         consolePrintf("%08x: %08x %08x\n", (uint32_t)p, p[0], p[1]);
         p += 2;
      }
   }
   else
   {
      handled = false;
   }

   return handled;
}

#define VECTKEY_WRITE_KEY  (0x05FA)
static bool commandReboot(int count, char **token)
{
   if ((IS_2_TOKENS() || IS_3_TOKENS()) && TOKEN_MATCH(1, "help"))
   {
      consolePrintf("%s - command to reboot the device\n", token[0]);
   }
   else
   {
      uint32_t  v;

      v = SCB_AIRCR;
      v &= ~SCB_AIRCR_VECTKEY_MASK;
      v |= SCB_AIRCR_VECTKEY(VECTKEY_WRITE_KEY) | SCB_AIRCR_SYSRESETREQ_MASK;
      SCB_AIRCR = v;
   }

   return true;
}

static bool commandTouch(int count, char **token)
{
   if ((IS_2_TOKENS() || IS_3_TOKENS()) && TOKEN_MATCH(1, "help"))
   {
      consolePrintf("%s - command to view touch data\n", token[0]);

      if (IS_3_TOKENS())
      {
         consolePrintf("\nCommand options are:\n\n");
         consolePrintf("  <none> - shows the data once\n");
         consolePrintf("  timed - shows the data twice per second until stopped\n");
      }
   }
   else if (IS_2_TOKENS() && TOKEN_MATCH(1, "timed"))
   {
      consolePrintf("\n<press any key to stop>\n");
      while (consoleGetInputCnt() == 0)
      {
         consolePrintf("Touch Pad A/B = %d/%d\n", touchData(TOUCH_PAD_A), touchData(TOUCH_PAD_B));
         osDelay( 500 ); // 500ms
      }
   }
   else
   {
      consolePrintf("Touch Pad A/B = %d/%d\n", touchData(TOUCH_PAD_A), touchData(TOUCH_PAD_B));
   }

   return true;
}

static bool commandVersion(int count, char **token)
{
   if ((IS_2_TOKENS() || IS_3_TOKENS()) && TOKEN_MATCH(1, "help"))
   {
      consolePrintf("%s - command to view build version info, etc.\n", token[0]);
   }
   else
   {
      consoleTaskShowBanner();
   }

   return true;
}
