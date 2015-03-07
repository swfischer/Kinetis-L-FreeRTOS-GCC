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

#include "usb.h"
#include "usbCfg.h"

const usbDeviceDesc_t usbCfgDevDesc =
{ sizeof(usbDeviceDesc_t) // bLength
, USB_DEVICE_DESC // bDescType
, USB_BCD_USB_1_1 // bcdUSB
, 2  // bDevClass
, 0  // bDevSubClass
, 0  // bDevProtocol
, USB_MAX_PKT_SIZE_32_BYTES // bMaxPktSize0
, 0x15A2 // idVendor
, 0xA50F // idProduct
, 0x0000 // bcdDevice
, 1  // iManufacturer
, 2  // iProduct
, 3  // iSerialNum
, 1  // bNumConfigs
};

const usbCfg_t usbCfg =
{ // Config
  { sizeof(usbConfigDesc_t) // bLength
  , USB_CONFIG_DESC // bDescType
  , sizeof(usbCfg_t) // wTotalLength
  , 2 // bNumInterfaces
  , 1 // bConfigValue
  , 0 // iConfig
  , USB_CFG_ATTR_ALWAYS_SET // bmAttributes - none
  , 50 // bMaxPower - 200mA
  }
  // Interface0
, { sizeof(usbInterfaceDesc_t) // bLength
  , USB_INTERFACE_DESC // bDescType
  , 0 // bInterfaceNum
  , 0 // bAltSetting
  , 1 // bNumEPs
  , 2 // bInterfaceClass
  , 2 // bInterfaceSubClass
  , 1 // bInterfaceProtocol
  , 0 // iInterface
  }
  // CDC header
, { sizeof(usbCdcHdrDesc_t) // bFnLength
  , USB_CDC_INTERFACE // bDescType
  , USB_CDC_SUB_TYPE_HEADER // bDescSubType
  , USB_BCD_CDC_1_1 // bcdCdcVer
  }
  // CDC call mgt
, { sizeof(usbCdcCallMgtDesc_t) // bFnLength
  , USB_CDC_INTERFACE // bDescType
  , USB_CDC_SUB_TYPE_CALL_MGT // bDescSubType
  , 0 // bmCapabilities
  , 1 // bDataInterface
  }
  // CDC ACM
, { sizeof(usbCdcAcmDesc_t) // bFnLength
  , USB_CDC_INTERFACE // bDescType
  , USB_CDC_SUB_TYPE_ACM // bDescSubType
  , 0 // bmCapabilities
  }
  // CDC union
, { sizeof(usbCdcUnionDesc_t) // bFnLength
  , USB_CDC_INTERFACE // bDescType
  , USB_CDC_SUB_TYPE_UNION // bDescSubType
  , 0 // bControlInterface
  , 1 // bSubInterface0
  }
  // EP1
, { sizeof(usbEndpointDesc_t) // bLength
  , USB_ENDPOINT_DESC // bDescType
  , USB_EP_ADDR_EP_NUMBER(1) | USB_EP_ADDR_DIRECTION_IN // bEPAddr
  , USB_EP_ATTR_TRANS_INTERRUPT // bmAttributes;
  , 32 // wMaxPktSize;
  , 2 // bInterval;
  }
  // Interface1
, { sizeof(usbInterfaceDesc_t) // bLength
  , USB_INTERFACE_DESC // bDescType
  , 1 // bInterfaceNum
  , 0 // bAltSetting
  , 2 // bNumEPs
  , 0x0a // bInterfaceClass
  , 0 // bInterfaceSubClass
  , 0 // bInterfaceProtocol
  , 0 // iInterface
  }
  // EP2
, { sizeof(usbEndpointDesc_t) // bLength
  , USB_ENDPOINT_DESC // bDescType
  , USB_EP_ADDR_EP_NUMBER(2) | USB_EP_ADDR_DIRECTION_IN // bEPAddr
  , USB_EP_ATTR_TRANS_BULK // bmAttributes;
  , 32 // wMaxPktSize;
  , 0 // bInterval;
  }
  // EP3
, { sizeof(usbEndpointDesc_t) // bLength
  , USB_ENDPOINT_DESC // bDescType
  , USB_EP_ADDR_EP_NUMBER(3) | USB_EP_ADDR_DIRECTION_OUT // bEPAddr
  , USB_EP_ATTR_TRANS_BULK // bmAttributes;
  , 32 // wMaxPktSize;
  , 0 // bInterval;
  }
};

static const usbStringDesc_t usbCfgLanguages =
{ sizeof(usbStringDesc_t) // bLength
, USB_STRING_DESC // bDescType
, { 0x09, 0x04 } // American English
};

static const usbStringDesc_t usbCfgManufacturer = USB_STRING("F\0r\0e\0e\0s\0c\0a\0l\0e\0");
static const usbStringDesc_t usbCfgProduct      = USB_STRING("U\0S\0B\0-\0U\0A\0R\0T\0");
static const usbStringDesc_t usbCfgSerialNumber = USB_STRING("0\00\00\01\0");

const uint8_t* usbCfgStringTable[] =
{ (uint8_t*) &usbCfgLanguages
, (uint8_t*) &usbCfgManufacturer
, (uint8_t*) &usbCfgProduct
, (uint8_t*) &usbCfgSerialNumber
};

// ----------------------------------------------------------------------------
// External Functions
// ----------------------------------------------------------------------------

bool usbCfgIsEpIN(uint8_t ep)
{
   bool in = false;

   if (ep == 1 || ep == 2)
   {
      in = true;
   }

   return in;
}
