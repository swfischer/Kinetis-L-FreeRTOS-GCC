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

#include <stdbool.h>

#include "console.h"
#include "consoleCmds.h"
#include "consoleTask.h"
#include "os.h"
#include "console.h"
#include "utils.h"

#define COMMAND_MAX_SIZE      (64)
#define MAX_TOKENS            (10)
#define MAX_NUMBER_OF_TESTS   (10)

#define IS_1_TOKEN()    (count == 1)
#define IS_2_TOKENS()   (count == 2)
#define IS_3_TOKENS()   (count == 3)
#define IS_4_TOKENS()   (count == 4)
#define IS_5_TOKENS()   (count == 5)
#define IS_6_TOKENS()   (count == 6)
#define IS_7_TOKENS()   (count == 7)
#define IS_8_TOKENS()   (count == 8)
#define IS_9_TOKENS()   (count == 9)
#define TOKEN_MATCH(x,y)  (utilsStrcmp(token[x],y) == 0)

typedef struct CommandItem
{
   char *cmd;
   cmdHandler_t handler;
} cmdItem_t;

// The list of available commands
static cmdItem_t sCmdList[MAX_NUMBER_OF_TESTS];

static char sCmd[COMMAND_MAX_SIZE];
static char *sToken[MAX_TOKENS];

static bool commandHelp(int count, char **token);
static char* parseClocking(char *buf, int len);
static void parseSDID(void);
static void parseSRS(void);
static int  parseTokens(char *pIn, char **token, int count);

// ----------------------------------------------------------------------------
// External Functions
// ----------------------------------------------------------------------------

void consoleTaskEntry( void *parameters )
{
   bool handled = true;
   int tokenCnt;
   int len;
   int i;

   consoleInit(BAUD_RATE);

   consoleTaskRegisterCommand("help", commandHelp);
   consoleCmdsInit();

   consoleTaskShowBanner();

   while (1)
   {
      consoleFlush();
      consolePrintf("\nyes? ");

      len = consoleGetInput(sCmd, COMMAND_MAX_SIZE);

      osDelay( 20 ); // 20 ms

      consolePrintf("\n");
      
      // Make sure the string is terminated
      sCmd[COMMAND_MAX_SIZE - 1] = 0;

      if (len <= 0)
         continue;

      tokenCnt = parseTokens(sCmd, sToken, MAX_TOKENS);

      if (tokenCnt <= 0)
      {
         osDelay( 100 ); // 100ms
      }
      else
      {
         handled = false;
         for (i = 0; i < MAX_NUMBER_OF_TESTS; i++)
         {
            if (sCmdList[i].cmd == NULL)
            {
               continue;
            }
            else if (utilsStrcmp(sCmdList[i].cmd, sToken[0]) == 0)
            {
               handled = sCmdList[i].handler(tokenCnt, sToken);
               break;
            }
         }
         
         if (!handled)
         {
            consolePrintf("Invalid command\n");
            osDelay( 100 ); // 100ms
         }
      }
   }
}

void consoleTaskRegisterCommand(char *cmd, cmdHandler_t test)
{
   int i;

   if (cmd != NULL && test != NULL)
   {
      for (i = 0; i < MAX_NUMBER_OF_TESTS; i++)
      {
         if (sCmdList[i].cmd == NULL)
         {
            sCmdList[i].cmd = cmd;
            sCmdList[i].handler = test;
            break;
         }
      }
   }
}

void consoleTaskShowBanner(void)
{
   char buf[30];

   consolePrintf("\nBoard .......: Freescale FRDM-KL25Z\n");
   consolePrintf("Built .......: %s %s\n", __DATE__, __TIME__);
   consolePrintf("Clocking ....: %s\n", parseClocking(buf, 30));
   consolePrintf("FreeRTOS ....: %s\n", tskKERNEL_VERSION_NUMBER);
   consolePrintf("Reset code ..: %02x %02x\n", RCM_SRS0, RCM_SRS1);
   consoleFlush();
   parseSRS();
   consolePrintf("System DID ..: %08x\n", SIM_SDID );
   parseSDID();
   consolePrintf("MCU UID .....: %08x %08x %08x\n", SIM_UIDMH, SIM_UIDML, SIM_UIDL );
   consoleFlush();
}

// ----------------------------------------------------------------------------
// Local Functions
// ----------------------------------------------------------------------------

static bool commandHelp(int count, char **token)
{
   bool handled = true;
   char *pHelp[3];
   int i;

   pHelp[1] = "help";

   // Check for a detaled help request
   if (IS_3_TOKENS() && TOKEN_MATCH(1, "help") && TOKEN_MATCH(2, "long"))
   {
      consolePrintf("%s - help command\n", token[0]);
      consolePrintf("\nCommand options are:\n\n");
      consolePrintf("  <none> - shows the list of commands\n");
      consolePrintf("  <command> - shows detailed help on the given command\n");
   }
   else if (IS_1_TOKEN())  // Output full help page
   {
      consolePrintf("\nCommand List:\n\n");
      for (i = 0; i < MAX_NUMBER_OF_TESTS; i++)
      {
         if (sCmdList[i].cmd == NULL)
         {
            continue;
         }

         if (utilsStrcmp(sCmdList[i].cmd, "help") == 0)
         {
            consolePrintf("%s - help command\n", token[0]);
         }
         else
         {
            pHelp[0] = sCmdList[i].cmd;
            sCmdList[i].handler(2, pHelp);
         }
      }
      consolePrintf("\nType \"help <command>\" for detailed help on a command\n");
   }
   else // Find the command and output it's help
   {
      for (i = 0; i < MAX_NUMBER_OF_TESTS; i++)
      {
         if (sCmdList[i].cmd == NULL)
         {
            continue;
         }
         else if (utilsStrcmp(sCmdList[i].cmd, token[1]) == 0)
         {
            pHelp[0] = sCmdList[i].cmd;
            pHelp[2] = "long";
            sCmdList[i].handler(3, pHelp);
            break;
         }
      }

      if (i >= MAX_NUMBER_OF_TESTS)
      {
         handled = false;
      }
   }

   return handled;
}

static char* parseClocking(char *buf, int len)
{
   if (MCG_C6 & MCG_C6_PLLS_MASK)
   {
      int pll = (8 / ((MCG_C5 & MCG_C5_PRDIV0_MASK) + 1)) * ((MCG_C6 & MCG_C6_VDIV0_MASK) + 24);
      int core = pll / (((SIM_CLKDIV1 & SIM_CLKDIV1_OUTDIV1_MASK) >> SIM_CLKDIV1_OUTDIV1_SHIFT) + 1);
      utilsSnprintf(buf, len, "PLL @ %dMHz w/Core @ %dMHz", pll, core);
   }
   else
   {
      utilsSnprintf(buf, len, "FLL");
   }

   return buf;
}

static void parseSDID(void)
{
   uint32_t sdid;
   uint8_t fam;
   uint8_t ram = 0;
   uint8_t pins = 0;

   sdid = SIM_SDID;
   fam = ((sdid >> 24) & 0xff);
   
   switch ((sdid >> 16) & 0xf)
   {
   case 1: ram = 1; break;
   case 2: ram = 2; break;
   case 3: ram = 4; break;
   case 4: ram = 8; break;
   case 5: ram = 16; break;
   case 6: ram = 32; break;
   case 7: ram = 64; break;
   }

   switch (sdid & 0xf)
   {
   case 0: pins = 16; break;
   case 1: pins = 24; break;
   case 2: pins = 32; break;
   case 4: pins = 48; break;
   case 5: pins = 64; break;
   case 6: pins = 80; break;
   case 8: pins = 100; break;
   }

   consolePrintf("  Details ...: KL%02x, %d pins, %dKB RAM\n", fam, pins, ram );
}

static void parseSRS(void)
{
   uint8_t srs0;
   uint8_t srs1;
   char *p = "unknown";

   srs0 = RCM_SRS0;
   srs1 = RCM_SRS1;

   if (srs0 & RCM_SRS0_POR_MASK)
   {
      p = "Power-On-Reset";
   }
   else if (srs0 & RCM_SRS0_PIN_MASK)
   {
      p = "External Reset Pin";
   }
   else if (srs0 & RCM_SRS0_WDOG_MASK)
   {
      p = "Watchdog";
   }
   else if (srs0 & RCM_SRS0_LOL_MASK)
   {
      p = "Loss Of Lock";
   }
   else if (srs0 & RCM_SRS0_LOC_MASK)
   {
      p = "Loss Of Clock";
   }
   else if (srs0 & RCM_SRS0_LVD_MASK)
   {
      p = "Low Voltage Detect";
   }
   else if (srs0 & RCM_SRS0_WAKEUP_MASK)
   {
      p = "Low Leakage Wakeup";
   }
   else if (srs1 & RCM_SRS1_MDM_AP_MASK)
   {
      p = "Debugger Reset";
   }
   else if (srs1 & RCM_SRS1_SW_MASK)
   {
      p = "Software Reset";
   }
   else if (srs1 & RCM_SRS1_LOCKUP_MASK)
   {
      p = "Core Lockup";
   }

   consolePrintf("  Details ...: %s\n", p );   
}

static int parseTokens(char *pIn, char **token, int count)
{
   int tokenCnt;
   int inToken;
   int size;
   int i;
   char c;

   size = utilsStrlen(pIn);

   tokenCnt = 0;
   inToken = 0;
   i = 0;
   while ( i < size && tokenCnt < count )
   {
      c = pIn[i];

      if (!inToken)
      {
         if (!utilsIsSpace(c))
         {
            token[tokenCnt++] = &pIn[i];
            inToken = 1;
         }
      }
      else
      {
         if (utilsIsSpace(c))
         {
            pIn[i] = 0;
            inToken = 0;
         }
      }

      i++;
   }
   
   return tokenCnt;
}

