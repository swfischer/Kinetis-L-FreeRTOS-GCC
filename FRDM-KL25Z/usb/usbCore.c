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
// Based on the Freescale USB echo device demo
// ----------------------------------------------------------------------------

#include <stdint.h>

#include "Settings.h"

#include "clk.h"
#include "irq.h"
#include "kinetis.h"
#include "os.h"
#include "usb.h"
#include "usbCfg.h"
#include "usbCore.h"
#include "utils.h"

bdt_t bdtTable[USBCFG_BDT_ENTRY_COUNT] __attribute__((__aligned__(512)));
uint8_t epBuffers[USBCFG_BDT_ENTRY_COUNT][USBCFG_EP_BUF_SIZE] __attribute__((__aligned__(4)));

uint8_t usbFlagsHack;
uint8_t usbStateHack;

static uint8_t  sClearFlags;
static uint8_t *sInData;
static uint8_t  sInCounter;
static uint8_t  sToogleFlags;

// FIXME: If this entry is removed, when the USB cable is inserted or removed, DMAERR errors occur.
uint8_t gu8HALT_EP;

static usbSetupPacket_t *sSetupPkt;
static usbInterfaceReqHandler sInterfaceReqHandler = NULL;

static void ep0InHandler(void);
static void ep0OutHandler(void);
static void ep0Stall(void);
static void epSetupHandler(void);
static void errorHandler(void);
static void isrHandler(uint32_t unused);
static void resetHandler(void);
static void setInterface(void);
static void setupHandler(void);
static void standardReqHandler(void);
static void tokenHandler(void);

// ----------------------------------------------------------------------------
// External Functions
// ----------------------------------------------------------------------------

void usbCoreInit(usbInterfaceReqHandler reqHandler)
{
   sInterfaceReqHandler = reqHandler;

   // Software Configuration

   sSetupPkt = (usbSetupPacket_t*) &epBuffers[USB_EP0_OUT_ODD][0];
   usbStateHack = uPOWER;

   // MPU Configuration

   clkEnable(CLK_USBOTG);
   irqRegister(INT_USB0, isrHandler, 0);
   irqEnable(INT_USB0);

   // USB Module Configuration

   // Reset USB Module
   USB0_USBTRC0 |= USB_USBTRC0_USBRESET_MASK;
   while (USB0_USBTRC0 & USB_USBTRC0_USBRESET_MASK)
      ;

   // Set BDT Base Register
   USB0_BDTPAGE1 = (uint8_t)((uint32_t) bdtTable >>  8);
   USB0_BDTPAGE2 = (uint8_t)((uint32_t) bdtTable >> 16);
   USB0_BDTPAGE3 = (uint8_t)((uint32_t) bdtTable >> 24);

   // Clear USB Reset flag
   USB0_ISTAT = USB_ISTAT_USBRST_MASK;

   // Enable USB Reset Interrupt
   USB0_INTEN |= USB_INTEN_USBRSTEN_MASK;

   // Disable weak pull downs
   USB0_USBCTRL &= ~(uint8_t)(USB_USBCTRL_PDE_MASK | USB_USBCTRL_SUSP_MASK);

   USB0_USBTRC0 |= 0x40;

   USB0_CTL |= 0x01;

   // Pull up enable
   FLAG_SET(USB_CONTROL_DPPULLUPNONOTG_SHIFT, USB0_CONTROL);
}

void usbCoreEpInTransfer(uint8_t ep, uint8_t *data, uint8_t length)
{
   uint16_t bytes = 0;
   uint8_t *buffer;
   uint8_t  epSize;
   uint8_t  epFlag;

   // Adjust the buffer location
   epFlag = ep;
   if (ep)
   {
      ep = (uint8_t)(ep << 2);
   }
   ep += 2;

   // Assign the proper EP buffer
   buffer = &epBuffers[ep][0];

   // Check if is a pending transfer
   if (FLAG_CHK(fIN, sClearFlags))
   {
      sInData = data;
      sInCounter = length;

      bytes = (sSetupPkt->wLength.bytes.hi << 8) + sSetupPkt->wLength.bytes.lo;
      if ((bytes < length) && (ep == 2))
      {
         sInCounter = sSetupPkt->wLength.bytes.lo;
      }
   }

   // Check transfer Size
   if (sInCounter > USBCFG_EP_BUF_SIZE)
   {
      epSize = USBCFG_EP_BUF_SIZE;
      sInCounter -= USBCFG_EP_BUF_SIZE;
      FLAG_CLR(fIN, sClearFlags);
   }
   else
   {
      epSize = sInCounter;
      sInCounter = 0;
      FLAG_SET(fIN, sClearFlags);
   }

   // Copy User buffer to EP buffer
   bdtTable[ep].cnt = epSize;

   while (epSize--)
   {
      *buffer++ = *sInData++;
   }

   // USB Flags Handling
   if (FLAG_CHK(epFlag, sToogleFlags))
   {
      bdtTable[ep].stat = kUDATA0;
      FLAG_CLR(epFlag, sToogleFlags);
   }
   else
   {
      bdtTable[ep].stat = kUDATA1;
      FLAG_SET(epFlag, sToogleFlags);
   }
}

uint8_t usbCoreEpOutTransfer(uint8_t ep, uint8_t *data)
{
   uint8_t *buffer;
   uint8_t bytes;
   int i;

   // Adjust the buffer location
   ep++;

   // Assign the proper EP buffer
   buffer = &epBuffers[ep][0];

   // Copy User buffer to EP buffer
   bytes = bdtTable[ep].cnt;

   for (i = 0; i < bytes; i++)
   {
      *data++ = *buffer++;
   }

   return (bytes);
}

uint16_t usbCoreEpOutSizeCheck(uint8_t ep)
{
   return (bdtTable[ep << 2].cnt & 0x03FF);
}

// ----------------------------------------------------------------------------
// Local Functions
// ----------------------------------------------------------------------------

static void ep0InHandler(void)
{
   if (usbStateHack == uADDRESS)
   {
      USB0_ADDR = sSetupPkt->wValue.bytes.lo;
      usbStateHack = uREADY;
      FLAG_SET(fIN, sClearFlags);
   }
   usbCoreEpInTransfer(0, 0, 0);
}

static void ep0OutHandler(void)
{
   FLAG_SET(EP0, usbFlagsHack);
   bdtTable[USB_EP0_OUT_ODD].stat = kUDATA0;
}

static void ep0Stall(void)
{
   USB0_ENDPT0 |= USB_ENDPT_EPSTALL_MASK;

   bdtTable[USB_EP0_OUT_ODD].stat = kUDATA0;
   bdtTable[USB_EP0_OUT_ODD].cnt = USBCFG_EP0_SIZE;
}

static void epSetupHandler(void)
{
   static uint16_t sEpHalt = 0;
   uint16_t status;

   switch (sSetupPkt->bRequest)
   {
   case USB_REQ_GET_STATUS:
      status = (sEpHalt & (1 << (sSetupPkt->wIndex.bytes.lo & 0xf))) ? 0x0100 : 0x0000;
      usbCoreEpInTransfer(EP0, (uint8_t*) &status, 2);
      break;
   case USB_REQ_CLR_FEATURE:
      sEpHalt &= ~(1 << (sSetupPkt->wIndex.bytes.lo & 0xf));
      usbCoreEpInTransfer(EP0, 0, 0);
      break;
   case USB_REQ_SET_FEATURE:
      sEpHalt |= (1 << (sSetupPkt->wIndex.bytes.lo & 0xf));
      usbCoreEpInTransfer(EP0, 0, 0);
      break;
   default:
      break;
   }
}

static void errorHandler(void)
{
   uint8_t errstat = USB0_ERRSTAT;
   char *p1 = "\r\nUSB-ERROR: ";
   char *p2 = "Unknown\r\n";
   int i;

   USB0_ERRSTAT = errstat; // Clear the error bit(s)
   
   if (errstat & USB_ERRSTAT_PIDERR_MASK)
   {
      p2 = "PID\r\n";
   }
   else if (errstat & USB_ERRSTAT_CRC5EOF_MASK)
   {
      p2 = "CRC5EOF\r\n";
   }
   else if (errstat & USB_ERRSTAT_CRC16_MASK)
   {
      p2 = "CRC16\r\n";
   }
   else if (errstat & USB_ERRSTAT_DFN8_MASK)
   {
      p2 = "DFN8\r\n";
   }
   else if (errstat & USB_ERRSTAT_BTOERR_MASK)
   {
      p2 = "BTOERR\r\n";
   }
   else if (errstat & USB_ERRSTAT_DMAERR_MASK)
   {
      p2 = "DMAERR\r\n";
   }
   else if (errstat & USB_ERRSTAT_BTSERR_MASK)
   {
      p2 = "BTSERR\r\n";
   }

   i = 0;
   while (p1[i] != 0)
   {
      while (!(UART0_S1 & UART_S1_TDRE_MASK))
         ;
      UART0_D = p1[i];
      i++;
   }
   i = 0;
   while (p2[i] != 0)
   {
      while (!(UART0_S1 & UART_S1_TDRE_MASK))
         ;
      UART0_D = p2[i];
      i++;
   }
}

static void isrHandler(uint32_t unused)
{
   if (USB0_ISTAT & USB_ISTAT_USBRST_MASK)
   {
      // Handle RESET Interrupt
      resetHandler();
   }
   else
   {
      // This occurs often while a cable is attached
      if (USB0_ISTAT & USB_ISTAT_SOFTOK_MASK)
      {
         USB0_ISTAT = USB_ISTAT_SOFTOK_MASK;
      }

      if (USB0_ISTAT & USB_ISTAT_STALL_MASK)
      {
         if (USB0_ENDPT0 & USB_ENDPT_EPSTALL_MASK)
         {
            USB0_ENDPT0 &= ~(USB_ENDPT_EPSTALL_MASK);
         }
         USB0_ISTAT = USB_ISTAT_STALL_MASK;
      }

      if (USB0_ISTAT & USB_ISTAT_TOKDNE_MASK)
      {
         USB0_CTL |= USB_CTL_ODDRST_MASK;
         tokenHandler();
         USB0_ISTAT = USB_ISTAT_TOKDNE_MASK;
      }

      if (USB0_ISTAT & USB_ISTAT_SLEEP_MASK)
      {
         USB0_ISTAT = USB_ISTAT_SLEEP_MASK;
      }

      if (USB0_ISTAT & USB_ISTAT_ERROR_MASK)
      {
         errorHandler();
         USB0_ISTAT = USB_ISTAT_ERROR_MASK;
      }

      if (USB0_ISTAT & USB_ISTAT_RESUME_MASK)
      {
         USB0_ISTAT = USB_ISTAT_RESUME_MASK;
      }
   }
}

static void resetHandler(void)
{
   // Software Flags
   sClearFlags = 0xFF;
   sToogleFlags = 0;

   // Disable all data EP registers
   USB0_ENDPT1 = 0;
   USB0_ENDPT2 = 0;
   USB0_ENDPT3 = 0;
   USB0_ENDPT4 = 0;
   USB0_ENDPT5 = 0;
   USB0_ENDPT6 = 0;

   // EP0 BDT Setup
   // EP0 OUT BDT Settings
   bdtTable[USB_EP0_OUT_ODD].cnt = USBCFG_EP0_SIZE;
   bdtTable[USB_EP0_OUT_ODD].addr = (uint32_t) &epBuffers[USB_EP0_OUT_ODD][0];
   bdtTable[USB_EP0_OUT_ODD].stat = kUDATA1;
   // EP0 OUT BDT Settings
   bdtTable[USB_EP0_OUT_EVEN].cnt = USBCFG_EP0_SIZE;
   bdtTable[USB_EP0_OUT_EVEN].addr = (uint32_t) &epBuffers[USB_EP0_OUT_EVEN][0];
   bdtTable[USB_EP0_OUT_EVEN].stat = kUDATA1;
   // EP0 IN BDT Settings
   bdtTable[USB_EP0_IN_ODD].cnt = USBCFG_EP0_SIZE;
   bdtTable[USB_EP0_IN_ODD].addr = (uint32_t) &epBuffers[USB_EP0_IN_ODD][0];
   bdtTable[USB_EP0_IN_ODD].stat = kUDATA0;
   // EP0 IN BDT Settings
   bdtTable[USB_EP0_IN_EVEN].cnt = USBCFG_EP0_SIZE;
   bdtTable[USB_EP0_IN_EVEN].addr = (uint32_t) &epBuffers[USB_EP0_IN_EVEN][0];
   bdtTable[USB_EP0_IN_EVEN].stat = kUDATA0;

   // Enable EP0
   USB0_ENDPT0 = 0x0D;

   // Clear all Error flags
   USB0_ERRSTAT = 0xFF;

   // CLear all USB ISR flags
   USB0_ISTAT = 0xFF;

   // Set default Address
   USB0_ADDR = 0x00;

   // Enable all error sources
   USB0_ERREN = 0xFF;

   // USB Interrupt Enablers
   USB0_INTEN |= ( USB_INTEN_TOKDNEEN_MASK | USB_INTEN_SOFTOKEN_MASK
                 | USB_INTEN_ERROREN_MASK | USB_INTEN_USBRSTEN_MASK
                 );
}

static void setInterface(void)
{
   uint8_t ep_mask;
   int i;

   // FIXME: Should all EPs be disabled first?

   for (i = 1; i < USBCFG_EP_COUNT; i++)
   {
      ep_mask  = USB_ENDPT_EPHSHK_MASK;
      ep_mask |= (usbCfgIsEpIN(i)) ? USB_ENDPT_EPTXEN_MASK : USB_ENDPT_EPRXEN_MASK;
      USB0_ENDPT(i) = ep_mask;
   }

   // EndPoint 1 BDT Settings
   bdtTable[USB_EP1_IN_ODD].stat = kMCU;
   bdtTable[USB_EP1_IN_ODD].cnt = 0x00;
   bdtTable[USB_EP1_IN_ODD].addr = (uint32_t) &epBuffers[USB_EP1_IN_ODD][0];
   // EndPoint 2 BDT Settings
   bdtTable[USB_EP2_IN_ODD].stat = kMCU;
   bdtTable[USB_EP2_IN_ODD].cnt = 0x00;
   bdtTable[USB_EP2_IN_ODD].addr = (uint32_t) &epBuffers[USB_EP2_IN_ODD][0];
   // EndPoint 3 BDT Settings
   bdtTable[USB_EP3_OUT_ODD].stat = kSIE;
   bdtTable[USB_EP3_OUT_ODD].cnt = 0xFF;
   bdtTable[USB_EP3_OUT_ODD].addr = (uint32_t) &epBuffers[USB_EP3_OUT_ODD][0];
}

static void setupHandler(void)
{
   uint8_t state;

   FLAG_CLR(0, sToogleFlags);

   switch (sSetupPkt->bmRequestType & USB_REQ_TYPE_TO_MASK)
   {
   case USB_REQ_TYPE_TO_DEVICE:
      if ((sSetupPkt->bmRequestType & USB_REQ_TYPE_TYPE_MASK) == USB_REQ_TYPE_TYPE_STANDARD)
      {
         standardReqHandler();
      }
      bdtTable[USB_EP0_OUT_ODD].stat = kUDATA0;
      break;
   case USB_REQ_TYPE_TO_INTERFACE:
      state = sInterfaceReqHandler(EP0, sSetupPkt);

      if (state == uSETUP)
      {
         bdtTable[USB_EP0_OUT_ODD].stat = kUDATA0;
      }
      else
      {
         bdtTable[USB_EP0_OUT_ODD].stat = kUDATA1;
      }
      break;
   case USB_REQ_TYPE_TO_ENDPOINT:
      epSetupHandler();
      bdtTable[USB_EP0_OUT_ODD].stat = kUDATA0;
      break;
   default:
      ep0Stall();
      break;
   }

   USB0_CTL &= ~(USB_CTL_TXSUSPENDTOKENBUSY_MASK);
}

static void standardReqHandler(void)
{
   static uint8_t sConfig = 0;
   uint16_t status = 0;

   switch (sSetupPkt->bRequest)
   {
   case USB_REQ_SET_ADDRESS:
      usbCoreEpInTransfer(EP0, 0, 0);
      usbStateHack = uADDRESS;
      break;
   case USB_REQ_GET_DESC:
      switch (sSetupPkt->wValue.bytes.hi)
      {
      case USB_DEVICE_DESC:
         usbCoreEpInTransfer(EP0, (uint8_t*)&usbCfgDevDesc, sizeof(usbCfgDevDesc));
         break;
      case USB_CONFIG_DESC:
         usbCoreEpInTransfer(EP0, (uint8_t*)&usbCfg, sizeof(usbCfg));
         break;
      case USB_STRING_DESC:
         usbCoreEpInTransfer( EP0
                            , (uint8_t*) usbCfgStringTable[sSetupPkt->wValue.bytes.lo]
                            , usbCfgStringTable[sSetupPkt->wValue.bytes.lo][0]
                            );
         break;
      default:
         ep0Stall();
         break;
      }
      break;
   case USB_REQ_SET_CONFIG:
      sConfig = sSetupPkt->wValue.bytes.hi + sSetupPkt->wValue.bytes.lo;
      if (sSetupPkt->wValue.bytes.hi + sSetupPkt->wValue.bytes.lo)
      {
         setInterface();
         usbCoreEpInTransfer(EP0, 0, 0);
         usbStateHack = uENUMERATED;
      }
      break;
   case USB_REQ_GET_CONFIG:
      usbCoreEpInTransfer(EP0, (uint8_t*) &sConfig, 1);
      break;
   case USB_REQ_GET_STATUS:
      status = 0;
      usbCoreEpInTransfer(EP0, (uint8_t*) &status, 2);
      break;
   default:
      ep0Stall();
      break;
   }
}

static void tokenHandler(void)
{
   uint8_t ep;
   uint8_t in;

   in = USB0_STAT & 0x08;
   ep = USB0_STAT >> 4;

   // Data EndPoints
   if (ep)
   {
      if (!in)
      {
         usbMCU_CONTROL(ep);
         FLAG_SET(ep, usbFlagsHack);
      }
   }
   // Control EndPoint
   else
   {
      if (in)
      {
         ep0InHandler();
      }
      else
      {
         if (usbCoreBdtGetPid(bdtTable[USB_EP0_OUT_ODD].stat) == USB_PID_TOKEN_SETUP)
         {
            setupHandler();
         }
         else
         {
            ep0OutHandler();
         }
      }
   }
}
