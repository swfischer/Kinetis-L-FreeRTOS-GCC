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

#include "clk.h"
#include "irq.h"
#include "kinetis.h"
#include "os.h"
#include "usb.h"
#include "usbCfg.h"
#include "usbDev.h"
#include "utils.h"

bdt_t bdtTable[USBCFG_BDT_ENTRY_COUNT] __attribute__((__aligned__(512)));
uint8_t epBuffers[USBCFG_BDT_ENTRY_COUNT][USBCFG_EP_BUF_SIZE] __attribute__((__aligned__(4)));

static uint8_t *sTxData;
static uint8_t  sTxDataCnt;
static uint8_t  sTxDataFlags;

static uint8_t sTokenBdtIdx;
static uint8_t sUsbAddr;
static usbSetupPacket_t *sSetupPkt;

static usbCtrlIsrHandler sCtrlIsrHandler = NULL;
static usbDataIsrHandler sDataIsrHandler = NULL;
static usbInterfaceReqHandler sInterfaceReqHandler = NULL;

static void ep0TxHandler(void);
static void ep0RxHandler(void);
static void ep0Stall(void);
static void epSetupHandler(usbSetupPacket_t *pkt);
static void errorHandler(void);
static void isrHandler(uint32_t unused);
static void resetHandler(void);
static void setInterface(void);
static void setupHandler(usbSetupPacket_t *pkt);
static void standardReqHandler(usbSetupPacket_t *pkt);
static void tokenHandler(void);

// ----------------------------------------------------------------------------
// External Functions
// ----------------------------------------------------------------------------

void usbDevInit( usbCtrlIsrHandler ctrlHandler
               , usbDataIsrHandler dataHandler
               , usbInterfaceReqHandler reqHandler
               )
{
   sCtrlIsrHandler = ctrlHandler;
   sDataIsrHandler = dataHandler;
   sInterfaceReqHandler = reqHandler;

   sSetupPkt = (usbSetupPacket_t*) &epBuffers[USB_EP0_RX_ODD][0];

   clkEnable(CLK_USBOTG);
   irqRegister(INT_USB0, isrHandler, 0);
   irqEnable(INT_USB0);

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
   USB0_CONTROL |= USB_CONTROL_DPPULLUPNONOTG_MASK;
}

void usbDevEpTxTransfer(uint8_t ep, uint8_t *data, uint8_t length)
{
   uint16_t epIdx;
   uint8_t *buffer;
   uint8_t  bytes;

   // Adjust the buffer location
   epIdx = ((uint16_t)(ep << 2)) + 2;

   if (length != 0)
   {
      sTxData = data;
      sTxDataCnt = length;
   }

   // Check transfer Size
   if (sTxDataCnt > USBCFG_EP_BUF_SIZE)
   {
      bytes = USBCFG_EP_BUF_SIZE;
      sTxDataCnt -= USBCFG_EP_BUF_SIZE;
   }
   else
   {
      bytes = sTxDataCnt;
      sTxDataCnt = 0;
   }

   // Copy User buffer to EP buffer
   bdtTable[epIdx].cnt = bytes;
   buffer = &epBuffers[epIdx][0];
   while (bytes--)
   {
      *buffer++ = *sTxData++;
   }

   // USB Flags Handling
   if (sTxDataFlags & (1 << ep))
   {
      bdtTable[epIdx].stat = kUDATA0;
      sTxDataFlags &= ~(1 << ep);
   }
   else
   {
      bdtTable[epIdx].stat = kUDATA1;
      sTxDataFlags |= (1 << ep);
   }
}

// ----------------------------------------------------------------------------
// Local Functions
// ----------------------------------------------------------------------------

static void ep0TxHandler(void)
{
   if (USB0_ADDR == 0 && sUsbAddr != 0)
   {
      USB0_ADDR = sUsbAddr;
   }
   usbDevEpTxTransfer(0, 0, 0);
}

static void ep0RxHandler(void)
{
   sDataIsrHandler(EP0, (uint8_t*) bdtTable[sTokenBdtIdx].addr, bdtTable[sTokenBdtIdx].cnt);

   bdtTable[USB_EP0_RX_ODD].stat = kUDATA0;
}

static void ep0Stall(void)
{
   USB0_ENDPT0 |= USB_ENDPT_EPSTALL_MASK;

   bdtTable[USB_EP0_RX_ODD].stat = kUDATA0;
   bdtTable[USB_EP0_RX_ODD].cnt = USBCFG_EP0_SIZE;
}

static void epSetupHandler(usbSetupPacket_t *pkt)
{
   static uint16_t sEpHalt = 0;
   uint16_t status;

   switch (pkt->bRequest)
   {
   case USB_REQ_GET_STATUS:
      status = (sEpHalt & (1 << (pkt->wIndex.bytes.lo & 0xf))) ? 0x0100 : 0x0000;
      usbDevEpTxTransfer(EP0, (uint8_t*) &status, 2);
      break;
   case USB_REQ_CLR_FEATURE:
      sEpHalt &= ~(1 << (pkt->wIndex.bytes.lo & 0xf));
      usbDevEpTxTransfer(EP0, 0, 0);
      break;
   case USB_REQ_SET_FEATURE:
      sEpHalt |= (1 << (pkt->wIndex.bytes.lo & 0xf));
      usbDevEpTxTransfer(EP0, 0, 0);
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
   sTxDataFlags = 0;

   // Disable all data EP registers
   USB0_ENDPT1 = 0;
   USB0_ENDPT2 = 0;
   USB0_ENDPT3 = 0;
   USB0_ENDPT4 = 0;
   USB0_ENDPT5 = 0;
   USB0_ENDPT6 = 0;

   // EP0 BDT Setup
   // EP0 OUT BDT Settings
   bdtTable[USB_EP0_RX_ODD].cnt = USBCFG_EP0_SIZE;
   bdtTable[USB_EP0_RX_ODD].addr = (uint32_t) &epBuffers[USB_EP0_RX_ODD][0];
   bdtTable[USB_EP0_RX_ODD].stat = kUDATA1;
   // EP0 OUT BDT Settings
   bdtTable[USB_EP0_RX_EVEN].cnt = USBCFG_EP0_SIZE;
   bdtTable[USB_EP0_RX_EVEN].addr = (uint32_t) &epBuffers[USB_EP0_RX_EVEN][0];
   bdtTable[USB_EP0_RX_EVEN].stat = kUDATA1;
   // EP0 IN BDT Settings
   bdtTable[USB_EP0_TX_ODD].cnt = USBCFG_EP0_SIZE;
   bdtTable[USB_EP0_TX_ODD].addr = (uint32_t) &epBuffers[USB_EP0_TX_ODD][0];
   bdtTable[USB_EP0_TX_ODD].stat = kUDATA0;
   // EP0 IN BDT Settings
   bdtTable[USB_EP0_TX_EVEN].cnt = USBCFG_EP0_SIZE;
   bdtTable[USB_EP0_TX_EVEN].addr = (uint32_t) &epBuffers[USB_EP0_TX_EVEN][0];
   bdtTable[USB_EP0_TX_EVEN].stat = kUDATA0;

   // Enable EP0
   USB0_ENDPT0 = 0x0D;

   // Clear all Error flags
   USB0_ERRSTAT = 0xFF;

   // Clear all USB ISR flags
   USB0_ISTAT = 0xFF;

   // Set default Address
   USB0_ADDR = 0x00;
   sUsbAddr = 0;

   // Enable all error sources
   USB0_ERREN = 0xFF;

   // USB Interrupt Enablers
   USB0_INTEN |= ( USB_INTEN_TOKDNEEN_MASK | USB_INTEN_SOFTOKEN_MASK
                 | USB_INTEN_ERROREN_MASK | USB_INTEN_USBRSTEN_MASK
                 );

   if (sCtrlIsrHandler)
   {
      sCtrlIsrHandler(USB_CTRL_EVENT_RESET);
   }
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
   bdtTable[USB_EP1_TX_ODD].stat = kMCU;
   bdtTable[USB_EP1_TX_ODD].cnt = 0x00;
   bdtTable[USB_EP1_TX_ODD].addr = (uint32_t) &epBuffers[USB_EP1_TX_ODD][0];
   // EndPoint 2 BDT Settings
   bdtTable[USB_EP2_TX_ODD].stat = kMCU;
   bdtTable[USB_EP2_TX_ODD].cnt = 0x00;
   bdtTable[USB_EP2_TX_ODD].addr = (uint32_t) &epBuffers[USB_EP2_TX_ODD][0];
   // EndPoint 3 BDT Settings
   bdtTable[USB_EP3_RX_ODD].stat = kSIE;
   bdtTable[USB_EP3_RX_ODD].cnt = 0xFF;
   bdtTable[USB_EP3_RX_ODD].addr = (uint32_t) &epBuffers[USB_EP3_RX_ODD][0];
}

static void setupHandler(usbSetupPacket_t *pkt)
{
   bool expectDataPkt = false;

   sTxDataFlags &= ~(1 << EP0);

   switch (pkt->bmRequestType & USB_REQ_TYPE_TO_MASK)
   {
   case USB_REQ_TYPE_TO_DEVICE:
      if ((pkt->bmRequestType & USB_REQ_TYPE_TYPE_MASK) == USB_REQ_TYPE_TYPE_STANDARD)
      {
         standardReqHandler(pkt);
      }
      break;
   case USB_REQ_TYPE_TO_INTERFACE:
      expectDataPkt = sInterfaceReqHandler(EP0, pkt);
      break;
   case USB_REQ_TYPE_TO_ENDPOINT:
      epSetupHandler(pkt);
      break;
   default:
      ep0Stall();
      break;
   }

   bdtTable[sTokenBdtIdx].stat = (expectDataPkt) ? kUDATA1 : kUDATA0;

   // Remove the HW suspend due to a setup packet
   USB0_CTL &= ~(USB_CTL_TXSUSPENDTOKENBUSY_MASK);
}

static void standardReqHandler(usbSetupPacket_t *pkt)
{
   static uint8_t sConfig = 0;
   uint16_t status = 0;

   switch (pkt->bRequest)
   {
   case USB_REQ_SET_ADDRESS:
      usbDevEpTxTransfer(EP0, 0, 0);
      sUsbAddr = pkt->wValue.bytes.lo;
      break;
   case USB_REQ_GET_DESC:
      switch (pkt->wValue.bytes.hi)
      {
      case USB_DEVICE_DESC:
         usbDevEpTxTransfer(EP0, (uint8_t*)&usbCfgDevDesc, sizeof(usbCfgDevDesc));
         break;
      case USB_CONFIG_DESC:
         usbDevEpTxTransfer(EP0, (uint8_t*)&usbCfg, sizeof(usbCfg));
         break;
      case USB_STRING_DESC:
         usbDevEpTxTransfer( EP0
                            , (uint8_t*) usbCfgStringTable[pkt->wValue.bytes.lo]
                            , usbCfgStringTable[pkt->wValue.bytes.lo][0]
                            );
         break;
      default:
         ep0Stall();
         break;
      }
      break;
   case USB_REQ_SET_CONFIG:
      sConfig = pkt->wValue.bytes.hi + pkt->wValue.bytes.lo;
      if (sConfig)
      {
         setInterface();
         usbDevEpTxTransfer(EP0, 0, 0);
         if (sCtrlIsrHandler)
         {
            sCtrlIsrHandler(USB_CTRL_EVENT_ENUMERATION);
         }
      }
      break;
   case USB_REQ_GET_CONFIG:
      usbDevEpTxTransfer(EP0, (uint8_t*) &sConfig, 1);
      break;
   case USB_REQ_GET_STATUS:
      status = 0;
      usbDevEpTxTransfer(EP0, (uint8_t*) &status, 2);
      break;
   default:
      ep0Stall();
      break;
   }
}

static void tokenHandler(void)
{
   uint8_t stat = USB0_STAT;
   uint8_t ep;
   uint8_t in;

   sTokenBdtIdx = stat >> 2;
   in = stat & USB_STAT_TX_MASK;
   ep = stat >> USB_STAT_ENDP_SHIFT;

   // Data EndPoints
   if (ep)
   {
      if (!in)
      {
         usbMCU_CONTROL(ep);

         sDataIsrHandler(ep, (uint8_t*) bdtTable[sTokenBdtIdx].addr, bdtTable[sTokenBdtIdx].cnt);
      }
   }
   // Control EndPoint
   else
   {
      if (in)
      {
         ep0TxHandler();
      }
      else
      {
         if (usbCoreBdtGetPid(bdtTable[sTokenBdtIdx].stat) == USB_PID_TOKEN_SETUP)
         {
            setupHandler((usbSetupPacket_t*) bdtTable[sTokenBdtIdx].addr);
         }
         else
         {
            ep0RxHandler();
         }
      }
   }
}
