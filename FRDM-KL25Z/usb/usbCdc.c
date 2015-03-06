
// ----------------------------------------------------------------------------
// Based on the Freescale USB echo device demo
// ----------------------------------------------------------------------------

#include <stdint.h>

#include "kinetis.h"
#include "os.h"
#include "ringBuffer.h"
#include "usbCdc.h"
#include "usbCore.h"

extern uint8_t gu8EP2_IN_ODD_Buffer[];
extern uint8_t gu8EP3_OUT_ODD_Buffer[];

extern uint8_t usbFlagsHack;
extern uint8_t usbStateHack;

// Not really used, but could be used to denote VBUS state changes
volatile uint8_t usbCdcIsrFlags = 0;

static uint8_t sCdcState = WAITING_FOR_ENUMERATION;
static uint8_t sCdcOutData[CDC_BUFFER_SIZE];
static uint8_t sAltInterface = 0;  // should be coordinated with interface descriptor
static usbCdcLineCoding_t sLineCoding;

// Local function prototypes
static uint8_t interfaceReqHandler(uint8_t ep, tUSB_Setup *pkt);

// ----------------------------------------------------------------------------
// External Functions
// ----------------------------------------------------------------------------

void usbCdcInit(void)
{
   sCdcState = WAITING_FOR_ENUMERATION;

   // USB core initialization
   usbCoreInit(interfaceReqHandler);

   // Line Coding Initialization
   sLineCoding.dteRate    = 9600;
   sLineCoding.charFormat = 0;
   sLineCoding.parityType = 0;
   sLineCoding.databits   = 8;

   // Initialize Data Buffers
//   ringBufferInit(sCdcOutData, CDC_BUFFER_SIZE);
}

void usbCdcEngine(void)
{
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
      while (usbStateHack != uENUMERATED)
      {
         osDelay(500); // slow things down to allow other tasks to run
      };

      sCdcState = WAITING_FOR_ENUMERATION;
      break;
   case SET_LINE_CODING:
      if (usbFlagsHack & (1 << EP0))
      {
         usbFlagsHack &= ~(1 << EP0);

         usbCoreEpOutTransfer(EP0, (uint8_t*)&sLineCoding);
         usbCoreEpInTransfer(EP0,0,0);
      }
      break;
   case SET_CONTROL_LINE_STATE:
      usbCoreEpInTransfer(EP0,0,0);
      break;
   }
}

// ----------------------------------------------------------------------------
// Local Functions
// ----------------------------------------------------------------------------

static uint8_t interfaceReqHandler(uint8_t ep, tUSB_Setup *pkt)
{
   uint8_t state = uSETUP;

   switch (pkt->bRequest)
   {
   case GET_INTERFACE:
      usbCoreEpInTransfer(ep, &sAltInterface, 1);
      break;
   case GET_LINE_CODING:
      usbCoreEpInTransfer(ep, (uint8_t*)&sLineCoding, 7);
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
//      ringBufferInit(sCdcOutData, CDC_BUFFER_SIZE);
      usbFlagsHack |= (1 << LOADER);
      sCdcOutData[0] = 0xFF;
      break;
   }

   return (state);
}
