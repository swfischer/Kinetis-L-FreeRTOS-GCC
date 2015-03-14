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

// Arrays and global buffers

bdt_t bdtTable[USBCFG_BDT_ENTRY_COUNT] __attribute__((__aligned__(512)));

uint8_t gu8EP0_OUT_ODD_Buffer[USBCFG_EP0_SIZE];
uint8_t gu8EP0_OUT_EVEN_Buffer[USBCFG_EP0_SIZE];
uint8_t gu8EP0_IN_ODD_Buffer[USBCFG_EP0_SIZE];
uint8_t gu8EP0_IN_EVEN_Buffer[USBCFG_EP0_SIZE];
uint8_t gu8EP1_OUT_ODD_Buffer[USBCFG_EP1_SIZE];
uint8_t gu8EP1_OUT_EVEN_Buffer[USBCFG_EP1_SIZE];
uint8_t gu8EP1_IN_ODD_Buffer[USBCFG_EP1_SIZE];
uint8_t gu8EP1_IN_EVEN_Buffer[USBCFG_EP1_SIZE];
uint8_t gu8EP2_OUT_ODD_Buffer[USBCFG_EP2_SIZE];
uint8_t gu8EP2_OUT_EVEN_Buffer[USBCFG_EP2_SIZE];
uint8_t gu8EP2_IN_ODD_Buffer[USBCFG_EP2_SIZE];
uint8_t gu8EP2_IN_EVEN_Buffer[USBCFG_EP2_SIZE];
uint8_t gu8EP3_OUT_ODD_Buffer[USBCFG_EP3_SIZE];
uint8_t gu8EP3_OUT_EVEN_Buffer[USBCFG_EP3_SIZE];
uint8_t gu8EP3_IN_ODD_Buffer[USBCFG_EP3_SIZE];
uint8_t gu8EP3_IN_EVEN_Buffer[USBCFG_EP3_SIZE];

uint8_t *BufferPointer[] =
{
    gu8EP0_OUT_ODD_Buffer,
    gu8EP0_OUT_EVEN_Buffer,
    gu8EP0_IN_ODD_Buffer,
    gu8EP0_IN_EVEN_Buffer,
    gu8EP1_OUT_ODD_Buffer,
    gu8EP1_OUT_EVEN_Buffer,
    gu8EP1_IN_ODD_Buffer,
    gu8EP1_IN_EVEN_Buffer,
    gu8EP2_OUT_ODD_Buffer,
    gu8EP2_OUT_EVEN_Buffer,
    gu8EP2_IN_ODD_Buffer,
    gu8EP2_IN_EVEN_Buffer,
    gu8EP3_OUT_ODD_Buffer,
    gu8EP3_OUT_EVEN_Buffer,
    gu8EP3_IN_ODD_Buffer,
    gu8EP3_IN_EVEN_Buffer
};

const uint8_t sEpSize[] = // FIXME: This should really be part of usbCfg.c/h
{ USBCFG_EP0_SIZE, USBCFG_EP0_SIZE, USBCFG_EP0_SIZE, USBCFG_EP0_SIZE
, USBCFG_EP1_SIZE, USBCFG_EP1_SIZE, USBCFG_EP1_SIZE, USBCFG_EP1_SIZE
, USBCFG_EP2_SIZE, USBCFG_EP2_SIZE, USBCFG_EP2_SIZE, USBCFG_EP2_SIZE
, USBCFG_EP3_SIZE, USBCFG_EP3_SIZE, USBCFG_EP3_SIZE, USBCFG_EP3_SIZE
};

// Global Variables for USB Handling
uint8_t usbFlagsHack;
uint8_t usbStateHack;

static uint8_t  sClearFlags;
static uint8_t *sInData;
static uint8_t  sInCounter;
static uint8_t  sToogleFlags;

uint8_t gu8USB_PingPong_flags;
//uint8_t gu8Dummy;
uint8_t gu8HALT_EP;

static usbSetupPacket_t *sSetupPkt;
static usbInterfaceReqHandler sInterfaceReqHandler = NULL;

static void ep0InHandler(void);
static void ep0OutHandler(void);
static void ep0Stall(void);
static void epSetupHandler(void);
static void isrHandler(uint32_t unused);
static void resetHandler(void);
static void setInterface(void);
static void setupHandler(void);
static void stallHandler(void);
static void standardReqHandler(void);
static void tokenHandler(void);

// ----------------------------------------------------------------------------
// External Functions
// ----------------------------------------------------------------------------

void usbCoreInit(usbInterfaceReqHandler handler)
{
   sInterfaceReqHandler = handler;

   // Software Configuration

   sSetupPkt = (usbSetupPacket_t*) BufferPointer[bEP0OUT_ODD];
   usbStateHack = uPOWER;

   // MPU Configuration

   clkEnable(CLK_USBOTG);
   irqRegister(INT_USB0, isrHandler, 0);
   irqEnable(INT_USB0);

   // USB Module Configuration

   // Reset USB Module
   USB0_USBTRC0 |= USB_USBTRC0_USBRESET_MASK;
   while (FLAG_CHK(USB_USBTRC0_USBRESET_SHIFT, USB0_USBTRC0))
      ;

   // Set BDT Base Register
   USB0_BDTPAGE1 = (uint8_t)((uint32_t) bdtTable >>  8);
   USB0_BDTPAGE2 = (uint8_t)((uint32_t) bdtTable >> 16);
   USB0_BDTPAGE3 = (uint8_t)((uint32_t) bdtTable >> 24);

   // Clear USB Reset flag
   FLAG_SET(USB_ISTAT_USBRST_MASK,USB0_ISTAT);

   // Enable USB Reset Interrupt
   FLAG_SET(USB_INTEN_USBRSTEN_SHIFT,USB0_INTEN);

   // Disable weak pull downs
   USB0_USBCTRL &= ~(uint8_t)(USB_USBCTRL_PDE_MASK | USB_USBCTRL_SUSP_MASK);

   USB0_USBTRC0 |= 0x40;

   USB0_CTL |= 0x01;

   // Pull up enable
   FLAG_SET(USB_CONTROL_DPPULLUPNONOTG_SHIFT, USB0_CONTROL);
}

void usbCoreEpInTransfer(uint8_t u8EP, uint8_t *pu8DataPointer, uint8_t u8DataSize)
{
   uint8_t *pu8EPBuffer;
   uint8_t u8EPSize;
   uint16_t length = 0;
   uint8_t u8EndPointFlag;

   // Adjust the buffer location
   u8EndPointFlag = u8EP;
   if (u8EP)
   {
      u8EP = (uint8_t)(u8EP << 2);
   }
   u8EP += 2;

   // Assign the proper EP buffer
   pu8EPBuffer = BufferPointer[u8EP];

   // Check if is a pending transfer
   if (FLAG_CHK(fIN, sClearFlags))
   {
      sInData = pu8DataPointer;
      sInCounter = u8DataSize;

      length = (sSetupPkt->wLength.bytes.hi << 8) + sSetupPkt->wLength.bytes.lo;
      if ((length < u8DataSize) && (u8EP == 2))
      {
         sInCounter = sSetupPkt->wLength.bytes.lo;
      }
   }

   // Check transfer Size
   if (sInCounter > sEpSize[u8EP])
   {
      u8EPSize = sEpSize[u8EP];
      sInCounter -= sEpSize[u8EP];
      FLAG_CLR(fIN, sClearFlags);
   }
   else
   {
      u8EPSize = sInCounter;
      sInCounter = 0;
      FLAG_SET(fIN, sClearFlags);
   }

   // Copy User buffer to EP buffer
   bdtTable[u8EP].cnt = (u8EPSize);

   while (u8EPSize--)
   {
      *pu8EPBuffer++=*sInData++;
   }

   // USB Flags Handling
   if (FLAG_CHK(u8EndPointFlag, sToogleFlags))
   {
      bdtTable[u8EP].stat = kUDATA0;
      FLAG_CLR(u8EndPointFlag, sToogleFlags);
   }
   else
   {
      bdtTable[u8EP].stat = kUDATA1;
      FLAG_SET(u8EndPointFlag, sToogleFlags);
   }
}

uint8_t usbCoreEpOutTransfer(uint8_t u8EP, uint8_t *pu8DataPointer)
{
   uint8_t *pu8EPBuffer;
   uint8_t u8EPSize;

   // Adjust the buffer location
   u8EP++;

   // Assign the proper EP buffer
   pu8EPBuffer = BufferPointer[u8EP];

   // Copy User buffer to EP buffer
   u8EPSize = bdtTable[u8EP].cnt;
   u8EP = u8EPSize;

   while (u8EPSize--)
   {
      *pu8DataPointer++ = *pu8EPBuffer++;
   }
   return(u8EP);
}

uint16_t usbCoreEpOutSizeCheck(uint8_t ep)
{
   uint8_t u8EPSize;

   // Read Buffer Size
   u8EPSize = utilsSwap16(bdtTable[ep << 2].cnt);

   return (u8EPSize & 0x03FF);
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
   usbCoreEpInTransfer(0,0,0);
}

static void ep0OutHandler(void)
{
   FLAG_SET(EP0, usbFlagsHack);
   bdtTable[bEP0OUT_ODD].stat = kUDATA0;
}

static void ep0Stall(void)
{
   FLAG_SET(USB_ENDPT_EPSTALL_SHIFT, USB0_ENDPT0);
   bdtTable[bEP0OUT_ODD].stat = kUDATA0;
   bdtTable[bEP0OUT_ODD].cnt = USBCFG_EP0_SIZE;
}

static void epSetupHandler(void)
{
   static uint8_t sEpHalt = 0;
   uint16_t status;

   switch(sSetupPkt->bRequest)
   {
   case USB_REQ_GET_STATUS:
      status = (FLAG_CHK(sSetupPkt->wIndex.bytes.hi, sEpHalt)) ? 0x0100 : 0x0000;
      usbCoreEpInTransfer(EP0, (uint8_t*) &status, 2);
      break;
   case USB_REQ_CLR_FEATURE:
      FLAG_CLR(sSetupPkt->wIndex.bytes.hi, sEpHalt);
      usbCoreEpInTransfer(EP0, 0, 0);
      break;
   case USB_REQ_SET_FEATURE:
      FLAG_SET(sSetupPkt->wIndex.bytes.hi, sEpHalt);
      usbCoreEpInTransfer(EP0, 0, 0);
      break;
   default:
      break;
   }
}

static volatile int xyz; // this is just a hack to keep the compile quiet

static void isrHandler(uint32_t unused)
{
   if(FLAG_CHK(USB_ISTAT_USBRST_SHIFT,USB0_ISTAT))
   {
      // Handle RESET Interrupt
      resetHandler();
      return;
   }

   if(FLAG_CHK(USB_ISTAT_SOFTOK_SHIFT,USB0_ISTAT))
   {
      USB0_ISTAT = USB_ISTAT_SOFTOK_MASK;
   }

   if(FLAG_CHK(USB_ISTAT_STALL_SHIFT,USB0_ISTAT))
   {
      stallHandler();
   }

   if(FLAG_CHK(USB_ISTAT_TOKDNE_SHIFT,USB0_ISTAT))
   {

      FLAG_SET(USB_CTL_ODDRST_SHIFT,USB0_CTL);
      tokenHandler();
      FLAG_SET(USB_ISTAT_TOKDNE_SHIFT,USB0_ISTAT);
   }

   if(FLAG_CHK(USB_ISTAT_SLEEP_SHIFT,USB0_ISTAT))
   //if(INT_STAT_SLEEP && INT_ENB_SLEEP_EN_MASK)
   {
      // Clear RESUME Interrupt if Pending
      //INT_STAT = INT_STAT_RESUME_MASK;
      //u8ISRCounter++;
      //FLAG_SET(USB_ISTAT_RESUME_SHIFT,USB0_ISTAT);

      // Clear SLEEP Interrupt
      FLAG_SET(USB_ISTAT_SLEEP_SHIFT,USB0_ISTAT);
      //INT_STAT = INT_STAT_SLEEP_MASK;
      //FLAG_SET(USB0_INTEN_RESUME_SHIFT,USB0_ISTAT);
      //INT_ENB_RESUME_EN = 1;

   }

   if(FLAG_CHK(USB_ISTAT_ERROR_SHIFT,USB0_ISTAT))
   //if(INT_STAT_ERROR && INT_ENB_ERROR_EN )
   {
      xyz = FLAG_CHK(USB_ISTAT_ERROR_SHIFT,USB0_ISTAT);
      //        printf("\nUSB Error\n");
      //INT_STAT_ERROR=1;

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
   bdtTable[bEP0OUT_ODD].cnt = USBCFG_EP0_SIZE;
   bdtTable[bEP0OUT_ODD].addr = (uint32_t) gu8EP0_OUT_ODD_Buffer;
   bdtTable[bEP0OUT_ODD].stat = kUDATA1;
   // EP0 OUT BDT Settings
   bdtTable[bEP0OUT_EVEN].cnt = USBCFG_EP0_SIZE;
   bdtTable[bEP0OUT_EVEN].addr = (uint32_t) gu8EP0_OUT_EVEN_Buffer;
   bdtTable[bEP0OUT_EVEN].stat = kUDATA1;
   // EP0 IN BDT Settings
   bdtTable[bEP0IN_ODD].cnt = USBCFG_EP0_SIZE;
   bdtTable[bEP0IN_ODD].addr = (uint32_t) gu8EP0_IN_ODD_Buffer;
   bdtTable[bEP0IN_ODD].stat = kUDATA0;
   // EP0 IN BDT Settings
   bdtTable[bEP0IN_EVEN].cnt = USBCFG_EP0_SIZE;
   bdtTable[bEP0IN_EVEN].addr = (uint32_t) gu8EP0_IN_EVEN_Buffer;
   bdtTable[bEP0IN_EVEN].stat = kUDATA0;

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
   FLAG_SET(USB_INTEN_TOKDNEEN_SHIFT, USB0_INTEN);
   FLAG_SET(USB_INTEN_SOFTOKEN_SHIFT, USB0_INTEN);
   FLAG_SET(USB_INTEN_ERROREN_SHIFT, USB0_INTEN);
   FLAG_SET(USB_INTEN_USBRSTEN_SHIFT, USB0_INTEN);
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
   bdtTable[bEP1IN_ODD].stat = kMCU;
   bdtTable[bEP1IN_ODD].cnt = 0x00;
   bdtTable[bEP1IN_ODD].addr = (uint32_t) gu8EP1_IN_ODD_Buffer;

   // EndPoint 2 BDT Settings
   bdtTable[bEP2IN_ODD].stat = kMCU;
   bdtTable[bEP2IN_ODD].cnt = 0x00;
   bdtTable[bEP2IN_ODD].addr = (uint32_t) gu8EP2_IN_ODD_Buffer;

   // EndPoint 3 BDT Settings
   bdtTable[bEP3OUT_ODD].stat = kSIE;
   bdtTable[bEP3OUT_ODD].cnt = 0xFF;
   bdtTable[bEP3OUT_ODD].addr = (uint32_t) gu8EP3_OUT_ODD_Buffer;
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
      bdtTable[bEP0OUT_ODD].stat = kUDATA0;
      break;
   case USB_REQ_TYPE_TO_INTERFACE:
      state = sInterfaceReqHandler(EP0, sSetupPkt);

      if (state == uSETUP)
      {
         bdtTable[bEP0OUT_ODD].stat = kUDATA0;
      }
      else
      {
         bdtTable[bEP0OUT_ODD].stat = kUDATA1;
      }
      break;
   case USB_REQ_TYPE_TO_ENDPOINT:
      epSetupHandler();
      bdtTable[bEP0OUT_ODD].stat = kUDATA0;
      break;
   default:
      ep0Stall();
      break;
   }

   FLAG_CLR(USB_CTL_TXSUSPENDTOKENBUSY_SHIFT, USB0_CTL);
}

static void stallHandler(void)
{
   if (FLAG_CHK(USB_ENDPT_EPSTALL_SHIFT, USB0_ENDPT0))
   {
      FLAG_CLR(USB_ENDPT_EPSTALL_SHIFT, USB0_ENDPT0);
   }
   FLAG_SET(USB_ISTAT_STALL_SHIFT, USB0_ISTAT);
}

static void standardReqHandler(void)
{
   static uint8_t sConfig = 0;
   uint16_t status = 0;

   switch (sSetupPkt->bRequest)
   {
   case USB_REQ_SET_ADDRESS:
      usbCoreEpInTransfer(EP0,0,0);
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
   uint8_t u8EndPoint;
   uint8_t u8IN;

   u8IN = USB0_STAT & 0x08;
   u8EndPoint = USB0_STAT >> 4;

   // Data EndPoints
   if(u8EndPoint)
   {
      if(!u8IN)
      {
         usbMCU_CONTROL(u8EndPoint);
         FLAG_SET(u8EndPoint,usbFlagsHack);
      }
   }
   // Control EndPoint
   else
   {
      if (u8IN)
      {
         ep0InHandler();
      }
      else
      {
         if (usbCoreBdtGetPid(bdtTable[bEP0OUT_ODD].stat) == USB_PID_TOKEN_SETUP)
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
