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

#include <stdint.h>
#include <stdbool.h>

#include "clk.h"
#include "irq.h"
#include "MKL25Z4.h"
#include "os.h"
#include "uart.h"

#define CHARACTER_CR    (0x0D)
#define CHARACTER_LF    (0x0A)
#define CHARACTER_BS    (0x08)
#define CHARACTER_DEL   (0x7F)

#define UART_BASE       (UART0_BASE_PTR)
#define UART_TX_DMA     (0)
#define OVER_SAMPLE     (16)
#define RX_BUFFER_SIZE  (128)

#define UART_FLAG_READ  (0x1)
#define UART_FLAG_WRITE (0x2)

typedef struct
{
   intIdx_t  interrupt;
   clkGate_t clkGate;
   int       dmaSrcTx;
} uartInfo_t;

static int     sBaud;
static uartCb  sCallback = NULL;
static osTimerId sCbTimer;

static bool    sEchoEnable;
static uint8_t sReadSeqNum;
static uint8_t sReadLast;
static uint8_t sWriteSeqNum;
static uint8_t sWriteLast;
static uint8_t sRxBuffer[RX_BUFFER_SIZE];
static uint8_t sRxBufReadIdx;
static uint8_t sRxBufWriteIdx;

// Local function prototypes
static int  bufCount(void);
static void bufPop(void); // Drop a byte from the write (input) side.
static uint8_t bufPull(void); // Return a byte from the read (output) side.
static void bufPush(uint8_t byte);
static void bufReset(void);
static void cbTimerEvent(void* argument);
static void dmaSetup(uint8_t *buf, uint16_t len);
static void handleBackspace(uint8_t byte);
static void handleEcho(uint8_t byte);
static int  hwInit(uint32_t baud);
static void startWrite(uint8_t *buf, uint16_t len);
static void uartEvent(int flag);
static void uartGetInfo(uint32_t base, uartInfo_t *info);
static void uartIrqHandler(uint32_t unused);
static void uartSetBaud(uint32_t baud);

// ----------------------------------------------------------------------------
// External Functions
// ----------------------------------------------------------------------------

int uartInit(uint32_t baud, uartCb callback)
{
   uartInfo_t info;
   int rc = -1;

   if (callback == NULL)
   {
      // Invalid input
   }
   else if (sCallback != NULL)
   {
      // Already initialized
   }
   else
   {
      uartGetInfo((uint32_t) UART_BASE, &info);

      sBaud = baud;
      sCallback = callback;
      bufReset();
      sEchoEnable = true;

      if (hwInit(baud) != 0)
      {
         sCallback = NULL;
      }
      else
      {
         osTimerDef_t tDef;
         
         tDef.ptimer = cbTimerEvent;
         tDef.millisec = 1;
         tDef.name = "uart";

         sCbTimer = osTimerCreate(&tDef, osTimerOnce, 0);
         if (sCbTimer == NULL)
         {
            sCallback = NULL;
         }
         else
         {
            rc = 0;
         }
      }
   }

   return rc;
}

void uartEchoEnable(bool en)
{
   if (sCallback == NULL)
   {
      // Not initialized
   }
   else if (sEchoEnable != en)
   {
      sEchoEnable = en;
      if (en == false)
      {
         while (!(UART0_S1 & UART_S1_TDRE_MASK))
            ;
      }
   }
}

int uartRead(uint8_t *buf, uint16_t len)
{
   int bytes = 0;

   if (buf == NULL || len == 0)
   {
      // Invalid input
   }
   else if (sCallback == NULL)
   {
      // Not initialized
   }
   else
   {
      int cnt;
      int i;

      cnt = bufCount();
      bytes = (cnt >= len) ? len : cnt;
      
      for (i = 0; i < bytes; i++)
      {
         buf[i] = bufPull();
         if (buf[i] == CHARACTER_CR)
         {
            buf[i] = 0;
            bytes = i + 1;
         }
      }
   }

   return bytes;
}

int uartReadCnt(void)
{
   int count = 0;

   if (sCallback == NULL)
   {
      // Not initialized
   }
   else
   {
      count = bufCount();
   }

   return count;
}

void uartReadFlush(void)
{
   if (sCallback == NULL)
   {
      // Not initialized
   }
   else
   {
      bufReset();
   }
}

int uartWrite(uint8_t *buf, uint16_t len)
{
   int rc = -1;

   if (sCallback == NULL)
   {
      // Not initialized
   }
   else if (buf == NULL || len == 0)
   {
      // Not initialized
   }
   else if (UART_C2_REG(UART_BASE) & UART_C2_TCIE_MASK)
   {
      // Already in process of handling a write
   }
   else
   {
      startWrite(buf, len);
   }

   return rc;
}

// ----------------------------------------------------------------------------
// Local Functions
// ----------------------------------------------------------------------------

static int bufCount(void)
{
   int cnt;

   cnt = sRxBufWriteIdx - sRxBufReadIdx;
   if (cnt < 0)
   {
      cnt += RX_BUFFER_SIZE;
   }

   return cnt;
}

static void bufPop(void)
{
   if (sRxBufReadIdx != sRxBufWriteIdx)
   {
      sRxBufWriteIdx --;
      if (sRxBufWriteIdx < 0)
      {
         sRxBufWriteIdx = RX_BUFFER_SIZE - 1;
      }
   }
}

static uint8_t bufPull(void)
{
   uint8_t byte = 0;

   if (sRxBufReadIdx != sRxBufWriteIdx)
   {
      byte = sRxBuffer[sRxBufReadIdx++];
      if (sRxBufReadIdx >= RX_BUFFER_SIZE)
      {
         sRxBufReadIdx = 0;
      }
   }

   return byte;
}

static void bufPush(uint8_t byte)
{
   int idx = sRxBufWriteIdx;

   sRxBuffer[idx++] = byte;
   if (idx >= RX_BUFFER_SIZE)
   {
      idx = 0;
   }
   
   // Prevent overflowing, drop the new byte instead
   if (idx != sRxBufReadIdx)
   {
      sRxBufWriteIdx = idx;
   }
}

static void bufReset(void)
{
   sRxBufReadIdx = 0;
   sRxBufWriteIdx = 0;
}

static void cbTimerEvent(void* argument)
{
   int callback = 1;

   while (callback)
   {
      callback = 0;
      if (sReadSeqNum != sReadLast)
      {
         callback++;
         sReadLast++;
         sCallback(UART_EVENT_READ_BIT);
      }
      if (sWriteSeqNum != sWriteLast)
      {
         callback++;
         sWriteLast++;
         sCallback(UART_EVENT_WRITE_BIT);
      }
   }
}

static void dmaSetup(uint8_t *buf, uint16_t len)
{
   uartInfo_t info;
   int channel;

   uartGetInfo((uint32_t) UART_BASE, &info);

   channel = UART_TX_DMA;

   // Setup the DMA channel
   DMAMUX0_CHCFG(channel) = DMAMUX_CHCFG_ENBL_MASK | info.dmaSrcTx;
   DMA_SAR(channel) = (uint32_t) buf;
   DMA_DAR(channel) = (uint32_t) &UART_D_REG(UART_BASE);
   DMA_DSR_BCR(channel) = DMA_DSR_BCR_DONE_MASK; // Clear the Done & BCR
   DMA_DSR_BCR(channel) = DMA_DSR_BCR_BCR(len);
   DMA_DCR(channel) = ( DMA_DCR_D_REQ_MASK // Clear the request when done
                      | DMA_DCR_SSIZE(1) // 8 bit source
                      | DMA_DCR_DSIZE(1) // 8 bit destination
                      | DMA_DCR_SINC_MASK // Increment the source but not destination
                      | DMA_DCR_CS_MASK // One transfer per HW signal
                      );
   DMA_DCR(channel) |= DMA_DCR_ERQ_MASK; // Enable the transfer
}

static void handleBackspace(uint8_t byte)
{
   if (byte == CHARACTER_BS || byte == CHARACTER_DEL)
   {
      bufPop(); // Remove the backspace character
      bufPop(); // Remove the previous character
   }
}

static void handleEcho(uint8_t byte)
{
   if (sEchoEnable)
   {
      UART_D_REG(UART_BASE) = byte;
   }
}

static int hwInit(uint32_t baud)
{
   uartInfo_t info;
   uint8_t s1;

   uartGetInfo((uint32_t) UART_BASE, &info);

   clkEnable(CLK_DMA);
   clkEnable(CLK_DMAMUX);
   clkEnable(info.clkGate);

   UART_C2_REG(UART_BASE) = 0;
   UART_C1_REG(UART_BASE) = 0;
   UART_C3_REG(UART_BASE) = 0;
   UART_S2_REG(UART_BASE) = 0;

   uartSetBaud(baud);

   UART_C2_REG(UART_BASE) |= (UART_C2_TE_MASK |  UART_C2_RE_MASK |  UART_C2_RIE_MASK);

   // Clear interrupts
   s1 = UART_S1_REG(UART_BASE);
   s1 = s1; // Keep the compile quiet

   // Setup the Tx interrupt
   irqRegister(info.interrupt, uartIrqHandler, 0);
   irqEnable(info.interrupt);

   return 0;
}

static void startWrite(uint8_t *buf, uint16_t len)
{
   dmaSetup(buf, len);

   // Enable transmit DMA
   if (UART_BASE == UART0_BASE_PTR)
   {
      // UARTLP
      UARTLP_C5_REG(UART_BASE) = UARTLP_C5_TDMAE_MASK;
   }
   else
   {
      // UART
      UART_C4_REG(UART_BASE) |= UART_C4_TDMAS_MASK;
   }

   // Enable the transmitter and interrupt
   UART_C2_REG(UART_BASE) |= (UART_C2_TCIE_MASK);
}

// Always called from an ISR context
static void uartEvent(int flag)
{
   int event = 0;

   if (flag == UART_FLAG_WRITE)
   {
      event = ((sReadSeqNum == sReadLast) && (sWriteSeqNum == sWriteLast)) ? 1 : 0;
      sWriteSeqNum++;
   }
   if (flag == UART_FLAG_READ)
   {
      event = ((sReadSeqNum == sReadLast) && (sWriteSeqNum == sWriteLast)) ? 1 : 0;
      sReadSeqNum++;
   }

   if (event)
   {
      osTimerStart(sCbTimer, 1);
   }
}

static void uartGetInfo(uint32_t base, uartInfo_t *info)
{
   if (info != NULL)
   {
      switch (base)
      {
      default:
      case (uint32_t) UART0_BASE_PTR:
         info->interrupt = INT_UART0;
         info->clkGate = CLK_UART0;
         info->dmaSrcTx = 3;
         break;
      case (uint32_t) UART1_BASE_PTR:
         info->interrupt = INT_UART1;
         info->clkGate = CLK_UART1;
         info->dmaSrcTx = 5;
         break;
      case (uint32_t) UART2_BASE_PTR:
         info->interrupt = INT_UART2;
         info->clkGate = CLK_UART2;
         info->dmaSrcTx = 7;
         break;
      }
   }
}

static void uartIrqHandler(uint32_t unused)
{
   uint32_t flags = 0;
   int status;
    
   status = UART_S1_REG(UART_BASE);

   if (  (status & UART_S1_TC_MASK) // Transmit complete interrupt
      && (UART_C2_REG(UART_BASE) & UART_C2_TCIE_MASK) // Something was being sent
      )
   {
      UART_C2_REG(UART_BASE) &= ~(UART_C2_TCIE_MASK);
      flags |= UART_FLAG_WRITE;
   }
   
   if (status & UART_S1_RDRF_MASK) // Receive data waiting
   {
      uint8_t byte;
      
      byte = UART_D_REG(UART_BASE);
      bufPush(byte);
      handleEcho(byte);
      handleBackspace(byte);

      if (byte == CHARACTER_CR)
      {
         flags |= UART_FLAG_READ;
      }
   }
   
   if (flags)
   {
      uartEvent(flags);
   }
}

static void uartSetBaud(uint32_t baud)
{
   uint16_t sbr;

   if (UART_BASE == UART0_BASE_PTR)
   {
      // UARTLP
      sbr = (CORE_CLOCK / OVER_SAMPLE) / baud;
      UARTLP_C4_REG(UART_BASE) = UARTLP_C4_OSR(OVER_SAMPLE - 1);
      UARTLP_BDH_REG(UART_BASE) &= ~UART_BDH_SBR_MASK;
      UARTLP_BDH_REG(UART_BASE) |= UARTLP_BDH_SBR((sbr >> 8));
      UARTLP_BDL_REG(UART_BASE) = UARTLP_BDL_SBR(sbr);
   }
   else
   {
      // UART
      sbr = (CORE_CLOCK / (baud * 16));
      UART_BDH_REG(UART_BASE) &= ~UART_BDH_SBR_MASK;
      UART_BDH_REG(UART_BASE) |= UART_BDH_SBR((sbr >> 8));
      UART_BDL_REG(UART_BASE) = UART_BDL_SBR(sbr);
   }
}
