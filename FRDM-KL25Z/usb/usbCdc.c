
// ----------------------------------------------------------------------------
// Based on the Freescale USB echo device demo
// ----------------------------------------------------------------------------

#include <stdint.h>

#include "kinetis.h"
#include "os.h"
#include "usbCdc.h"

#include "Settings.h"

extern uint8 gu8USB_Flags;
extern uint8 gu8USB_State;
extern tUSB_Setup *Setup_Pkt;
extern uint8 gu8EP2_IN_ODD_Buffer[];
extern uint8 gu8EP3_OUT_ODD_Buffer[];
extern tBDT tBDTtable[];
extern uint8 gu8Interface;


volatile uint8_t usbCdcIsrFlags = 0;

static uint8_t sCdcState = WAITING_FOR_ENUMERATION;
static uint8_t sCdcOutData[CDC_BUFFER_SIZE];
static usbCdcLineCoding_t sLineCoding;

// Local function prototypes
static uint8_t interfaceReqHandler(void);

// ----------------------------------------------------------------------------
// External Functions
// ----------------------------------------------------------------------------

void usbCdcInit(void)
{
   sCdcState = WAITING_FOR_ENUMERATION;

   // USB Initialization
   USB_Init(interfaceReqHandler);

   // Line Coding Initialization
   sLineCoding.dteRate    = 9600;
   sLineCoding.charFormat = 0;
   sLineCoding.parityType = 0;
   sLineCoding.databits   = 8;

   // Initialize Data Buffers
   Buffer_Init(sCdcOutData, CDC_BUFFER_SIZE);
}

void usbCdcEngine(void)
{
   //uint16 u8RecData;
   
   // Re-init CDC class if a VBUS HIGH event was detected
   if (usbCdcIsrFlags & VBUS_HIGH_EVENT)
   {
      usbCdcIsrFlags &= ~(VBUS_HIGH_EVENT);

      USB0_CTL |= USB_CTL_USBENSOFEN_MASK;
      usbCdcInit();
   }

   switch(sCdcState)
   {
   case WAITING_FOR_ENUMERATION:
      // Wait for USB Enumeration
      while (gu8USB_State != uENUMERATED)
      {
         osDelay(500); // slow things down to allow other tasks to run
      };

      sCdcState = WAITING_FOR_ENUMERATION;
      break;
   case SET_LINE_CODING:
      if(FLAG_CHK(EP0, gu8USB_Flags))
      {
         FLAG_CLR(EP0, gu8USB_Flags);
         (void)EP_OUT_Transfer(EP0, (uint8*)&sLineCoding);
         EP_IN_Transfer(EP0,0,0);
      }
      break;
   case SET_CONTROL_LINE_STATE:
      EP_IN_Transfer(EP0,0,0);
      break;
   }

   /* Data stage */
   /*
   if(FLAG_CHK(EP_OUT,gu8USB_Flags))
      {
            u8RecData=USB_EP_OUT_SizeCheck(EP_OUT);         // Check how many bytes from the PC
            usbEP_Reset(EP_OUT);
            usbSIE_CONTROL(EP_OUT);
            FLAG_CLR(EP_OUT,gu8USB_Flags);
            EP_IN_Transfer(EP2,CDC_OUTPointer,2);
            u8RecData=0;
      }
*/
}

// ----------------------------------------------------------------------------
// Local Functions
// ----------------------------------------------------------------------------

static uint8_t interfaceReqHandler(void)
{
   uint8_t state = uSETUP;

   switch (Setup_Pkt->bRequest)
   {
   case GET_INTERFACE:
      EP_IN_Transfer(EP0, &gu8Interface, 1);
      break;
   case GET_LINE_CODING:
      EP_IN_Transfer(EP0, (uint8*)&sLineCoding, 7);
      break;
   case SET_LINE_CODING:
      sCdcState = SET_LINE_CODING;
      state = uDATA;
      break;
   case SET_CONTROL_LINE_STATE:
      sCdcState = SET_CONTROL_LINE_STATE;
      state = uSETUP;
      break;
   case LOADER_MODE:
      Buffer_Init(sCdcOutData, CDC_BUFFER_SIZE);
      FLAG_SET(LOADER, gu8USB_Flags);
      sCdcOutData[0] = 0xFF;
      break;
   }

   return (state);
}
