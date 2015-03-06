
// ----------------------------------------------------------------------------
// Based on the Freescale USB echo device demo
// ----------------------------------------------------------------------------

#include <stdint.h>

#include "Settings.h"

#include "clk.h"
#include "irq.h"
#include "kinetis.h"
#include "os.h"
#include "usbCfg.h"
#include "usbCore.h"

static usbInterfaceReqHandler sInterfaceReqHandler = NULL;

/* Arrays and global buffers */

__attribute__((__aligned__(512)))
 tBDT tBDTtable[16];

uint8_t gu8EP0_OUT_ODD_Buffer[EP0_SIZE];
uint8_t gu8EP0_OUT_EVEN_Buffer[EP0_SIZE];
uint8_t gu8EP0_IN_ODD_Buffer[EP0_SIZE];
uint8_t gu8EP0_IN_EVEN_Buffer[EP0_SIZE];
uint8_t gu8EP1_OUT_ODD_Buffer[EP1_SIZE];
uint8_t gu8EP1_OUT_EVEN_Buffer[EP1_SIZE];
uint8_t gu8EP1_IN_ODD_Buffer[EP1_SIZE];
uint8_t gu8EP1_IN_EVEN_Buffer[EP1_SIZE];
uint8_t gu8EP2_OUT_ODD_Buffer[EP2_SIZE];
uint8_t gu8EP2_OUT_EVEN_Buffer[EP2_SIZE];
uint8_t gu8EP2_IN_ODD_Buffer[EP2_SIZE];
uint8_t gu8EP2_IN_EVEN_Buffer[EP2_SIZE];
uint8_t gu8EP3_OUT_ODD_Buffer[EP3_SIZE];
uint8_t gu8EP3_OUT_EVEN_Buffer[EP3_SIZE];
uint8_t gu8EP3_IN_ODD_Buffer[EP3_SIZE];
uint8_t gu8EP3_IN_EVEN_Buffer[EP3_SIZE];

uint8_t *BufferPointer[]=
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

const uint8_t cEP_Size[]=
{
    EP0_SIZE,
    EP0_SIZE,
    EP0_SIZE,
    EP0_SIZE,
    EP1_SIZE,
    EP1_SIZE,
    EP1_SIZE,
    EP1_SIZE,
    EP2_SIZE,
    EP2_SIZE,
    EP2_SIZE,
    EP2_SIZE,
    EP3_SIZE,
    EP3_SIZE,
    EP3_SIZE,
    EP3_SIZE
};

static const uint8_t* sStringTable[] =
{ (uint8_t*) &usbCfgLanguages
, (uint8_t*) &usbCfgManufacturer
, (uint8_t*) &usbCfgProduct
, (uint8_t*) &usbCfgSerialNumber
};

/* Global Variables for USB Handling */
uint8_t usbFlagsHack;
uint8_t usbStateHack;

uint8_t gu8USBClearFlags;
uint8_t *pu8IN_DataPointer;
uint8_t gu8IN_Counter;
uint8_t gu8USB_Toogle_flags;
uint8_t gu8USB_PingPong_flags;
uint8_t gu8Dummy;
uint8_t gu8HALT_EP;
tUSB_Setup *Setup_Pkt;

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

   Setup_Pkt = (tUSB_Setup*)BufferPointer[bEP0OUT_ODD];
   usbStateHack = uPOWER;

   // MPU Configuration

   clkEnable(CLK_USBOTG);
//   SIM_SCGC4 |= SIM_SCGC4_USBOTG_MASK; // USB Clock Gating
   irqRegister(INT_USB0, isrHandler, 0);
   irqEnable(INT_USB0);

   // USB Module Configuration

   // Reset USB Module
   USB0_USBTRC0 |= USB_USBTRC0_USBRESET_MASK;
   while (FLAG_CHK(USB_USBTRC0_USBRESET_SHIFT, USB0_USBTRC0))
      ;

   // Set BDT Base Register
   USB0_BDTPAGE1 = (uint8_t)((uint32_t)tBDTtable>>8);
   USB0_BDTPAGE2 = (uint8_t)((uint32_t)tBDTtable>>16);
   USB0_BDTPAGE3 = (uint8_t)((uint32_t)tBDTtable>>24);

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
    uint16_t u16Lenght=0;
    uint8_t u8EndPointFlag;

    /* Adjust the buffer location */
    u8EndPointFlag=u8EP;
    if(u8EP)
        u8EP=(uint8_t)(u8EP<<2);
    u8EP+=2;



    /* Assign the proper EP buffer */
    pu8EPBuffer=BufferPointer[u8EP];

    /* Check if is a pending transfer */
    if(FLAG_CHK(fIN,gu8USBClearFlags))
    {
        pu8IN_DataPointer=pu8DataPointer;
        gu8IN_Counter=u8DataSize;

        u16Lenght=(Setup_Pkt->wLength_h<<8)+Setup_Pkt->wLength_l ;
        if((u16Lenght < u8DataSize) && (u8EP==2))
        {
            gu8IN_Counter=Setup_Pkt->wLength_l;
        }
    }

    /* Check transfer Size */
    if(gu8IN_Counter > cEP_Size[u8EP])
    {
        u8EPSize = cEP_Size[u8EP];
        gu8IN_Counter-=cEP_Size[u8EP];
        FLAG_CLR(fIN,gu8USBClearFlags);
    }
    else
    {
        u8EPSize = gu8IN_Counter;
        gu8IN_Counter=0;
        FLAG_SET(fIN,gu8USBClearFlags);
    }

    /* Copy User buffer to EP buffer */
    tBDTtable[u8EP].Cnt=(u8EPSize);

    while(u8EPSize--)
         *pu8EPBuffer++=*pu8IN_DataPointer++;


    /* USB Flags Handling */
    if(FLAG_CHK(u8EndPointFlag,gu8USB_Toogle_flags))
    {
        tBDTtable[u8EP].Stat._byte= kUDATA0;
        FLAG_CLR(u8EndPointFlag,gu8USB_Toogle_flags);
    }
    else
    {
        tBDTtable[u8EP].Stat._byte= kUDATA1;
        FLAG_SET(u8EndPointFlag,gu8USB_Toogle_flags);
    }

}

uint8_t usbCoreEpOutTransfer(uint8_t u8EP, uint8_t *pu8DataPointer)
{
    uint8_t *pu8EPBuffer;
    uint8_t u8EPSize;


    /* Adjust the buffer location */
    u8EP++;

    /* Assign the proper EP buffer */
    pu8EPBuffer=BufferPointer[u8EP];

    /* Copy User buffer to EP buffer */
    u8EPSize=tBDTtable[u8EP].Cnt;
    u8EP=u8EPSize;

    while(u8EPSize--)
         *pu8DataPointer++=*pu8EPBuffer++;
    return(u8EP);
}

uint16_t usbCoreEpOutSizeCheck(uint8_t ep)
{
   uint8_t u8EPSize;

   // Read Buffer Size
   u8EPSize = SWAP16(tBDTtable[ep << 2].Cnt);

   return (u8EPSize & 0x03FF);
}

// ----------------------------------------------------------------------------
// Local Functions
// ----------------------------------------------------------------------------

static void ep0InHandler(void)
{
   if (usbStateHack == uADDRESS)
   {
      USB0_ADDR = Setup_Pkt->wValue_l;
      usbStateHack = uREADY;
      FLAG_SET(fIN, gu8USBClearFlags);
   }
   usbCoreEpInTransfer(0,0,0);
}

static void ep0OutHandler(void)
{
   FLAG_SET(EP0, usbFlagsHack);
   tBDTtable[bEP0OUT_ODD].Stat._byte = kUDATA0;
}

static void ep0Stall(void)
{
   FLAG_SET(USB_ENDPT_EPSTALL_SHIFT, USB0_ENDPT0);
   tBDTtable[bEP0OUT_ODD].Stat._byte = kUDATA0;
   tBDTtable[bEP0OUT_ODD].Cnt = EP0_SIZE;
}

static void epSetupHandler(void)
{
   uint16_t u16Status;

   switch(Setup_Pkt->bRequest)
   {
   case GET_STATUS:
      if(FLAG_CHK(Setup_Pkt->wIndex_h, gu8HALT_EP))
         u16Status = 0x0100;
      else
         u16Status = 0x0000;

      usbCoreEpInTransfer(EP0, (uint8_t*)&u16Status, 2);
      break;
   case CLEAR_FEATURE:
      FLAG_CLR(Setup_Pkt->wIndex_h, gu8HALT_EP);
      usbCoreEpInTransfer(EP0,0,0);
      break;
   case SET_FEATURE:
      FLAG_SET(Setup_Pkt->wIndex_h, gu8HALT_EP);
      usbCoreEpInTransfer(EP0,0,0);
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
   gu8USBClearFlags = 0xFF;
   gu8USB_Toogle_flags = 0;
   gu8USB_PingPong_flags = 0;

   // Disable all data EP registers
   USB0_ENDPT1 = 0;
   USB0_ENDPT2 = 0;
   USB0_ENDPT3 = 0;
   USB0_ENDPT4 = 0;
   USB0_ENDPT5 = 0;
   USB0_ENDPT6 = 0;

   // EP0 BDT Setup
   // EP0 OUT BDT Settings
   tBDTtable[bEP0OUT_ODD].Cnt = EP0_SIZE;
   tBDTtable[bEP0OUT_ODD].Addr = (uint32_t)gu8EP0_OUT_ODD_Buffer;
   tBDTtable[bEP0OUT_ODD].Stat._byte = kUDATA1;
   // EP0 OUT BDT Settings
   tBDTtable[bEP0OUT_EVEN].Cnt = EP0_SIZE;
   tBDTtable[bEP0OUT_EVEN].Addr = (uint32_t)gu8EP0_OUT_EVEN_Buffer;
   tBDTtable[bEP0OUT_EVEN].Stat._byte = kUDATA1;
   // EP0 IN BDT Settings
   tBDTtable[bEP0IN_ODD].Cnt = EP0_SIZE;
   tBDTtable[bEP0IN_ODD].Addr = (uint32_t)gu8EP0_IN_ODD_Buffer;
   tBDTtable[bEP0IN_ODD].Stat._byte = kUDATA0;
   // EP0 IN BDT Settings
   tBDTtable[bEP0IN_EVEN].Cnt = (EP0_SIZE);
   tBDTtable[bEP0IN_EVEN].Addr =(uint32_t)gu8EP0_IN_EVEN_Buffer;
   tBDTtable[bEP0IN_EVEN].Stat._byte = kUDATA0;

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
   // EndPoint Register settings
   USB0_ENDPT1= EP1_VALUE | USB_ENDPT_EPHSHK_MASK;
   USB0_ENDPT2= EP2_VALUE | USB_ENDPT_EPHSHK_MASK;
   USB0_ENDPT3= EP3_VALUE | USB_ENDPT_EPHSHK_MASK;
   USB0_ENDPT4= EP4_VALUE | USB_ENDPT_EPHSHK_MASK;
   USB0_ENDPT5= EP5_VALUE | USB_ENDPT_EPHSHK_MASK;
   USB0_ENDPT6= EP6_VALUE | USB_ENDPT_EPHSHK_MASK;

   // EndPoint 1 BDT Settings
   tBDTtable[bEP1IN_ODD].Stat._byte= kMCU;
   tBDTtable[bEP1IN_ODD].Cnt = 0x00;
   tBDTtable[bEP1IN_ODD].Addr =(uint32_t)gu8EP1_IN_ODD_Buffer;

   // EndPoint 2 BDT Settings
   tBDTtable[bEP2IN_ODD].Stat._byte= kMCU;
   tBDTtable[bEP2IN_ODD].Cnt = 0x00;
   tBDTtable[bEP2IN_ODD].Addr =(uint32_t)gu8EP2_IN_ODD_Buffer;

   // EndPoint 3 BDT Settings
   tBDTtable[bEP3OUT_ODD].Stat._byte= kSIE;
   tBDTtable[bEP3OUT_ODD].Cnt = 0xFF;
   tBDTtable[bEP3OUT_ODD].Addr =(uint32_t)gu8EP3_OUT_ODD_Buffer;
}

static void setupHandler(void)
{
   uint8_t u8State;

   FLAG_CLR(0,gu8USB_Toogle_flags);
   switch(Setup_Pkt->bmRequestType & 0x1F)
   {
   case DEVICE_REQ:
      if((Setup_Pkt->bmRequestType & 0x1F)== STANDARD_REQ)
      {
         //tBDTtable[bEP0IN_ODD].Stat._byte= kUDATA1;

         standardReqHandler();
      }
      tBDTtable[bEP0OUT_ODD].Stat._byte= kUDATA0;
      break;
   case INTERFACE_REQ:
      u8State = sInterfaceReqHandler(EP0, Setup_Pkt);

      if(u8State==uSETUP)
         tBDTtable[bEP0OUT_ODD].Stat._byte= kUDATA0;
      else
         tBDTtable[bEP0OUT_ODD].Stat._byte= kUDATA1;
      break;
   case ENDPOINT_REQ:
      epSetupHandler();
      tBDTtable[bEP0OUT_ODD].Stat._byte= kUDATA0;
      break;
   default:
      ep0Stall();
      break;
   }

   //USB0_CTL&=!USB_CTL_TXSUSPENDTOKENBUSY_MASK;

   //CTL_TXSUSPEND_TOKENBUSY=0;
   FLAG_CLR(USB_CTL_TXSUSPENDTOKENBUSY_SHIFT,USB0_CTL);
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
   uint16_t status = 0;

   switch (Setup_Pkt->bRequest)
   {
   case mSET_ADDRESS:
      usbCoreEpInTransfer(EP0,0,0);
      usbStateHack = uADDRESS;
      break;
   case mGET_DESC:
      switch (Setup_Pkt->wValue_h)
      {
      case mDEVICE:
         usbCoreEpInTransfer(EP0, (uint8_t*)&usbCfgDevDesc, sizeof(usbCfgDevDesc));
         break;
      case mCONFIGURATION:
         usbCoreEpInTransfer(EP0, (uint8_t*)&usbCfg, sizeof(usbCfg));
         break;
      case mSTRING:
         usbCoreEpInTransfer(EP0, (uint8_t*)sStringTable[Setup_Pkt->wValue_l], sStringTable[Setup_Pkt->wValue_l][0]);
         break;
      default:
         ep0Stall();
         break;
      }
      break;
   case mSET_CONFIG:
      gu8Dummy = Setup_Pkt->wValue_h+Setup_Pkt->wValue_l;
      if (Setup_Pkt->wValue_h + Setup_Pkt->wValue_l)
      {
         setInterface();
         usbCoreEpInTransfer(EP0,0,0);
         usbStateHack = uENUMERATED;
      }
      break;
   case mGET_CONFIG:
      usbCoreEpInTransfer(EP0, (uint8_t*)&gu8Dummy, 1);
      break;
   case mGET_STATUS:
      status = 0;
      usbCoreEpInTransfer(EP0, (uint8_t*)&status, 2);
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
         //tBDTtable[u8EndPoint+1].Stat._byte= kSIE;
         //tBDTtable[u8EndPoint+1].Stat._byte= kMCU;
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
         if(tBDTtable[bEP0OUT_ODD].Stat.RecPid.PID == mSETUP_TOKEN)
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
