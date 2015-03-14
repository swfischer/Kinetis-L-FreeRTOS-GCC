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

#ifndef _USB_H_
#define _USB_H_

#include <stdint.h>

// Values for PID tokens
#define USB_PID_TOKEN_OUT     (0x1)
#define USB_PID_TOKEN_SOF     (0x5)
#define USB_PID_TOKEN_IN      (0x9)
#define USB_PID_TOKEN_SETUP   (0xd)

// Values for bDescType fields
#define USB_DEVICE_DESC             (1)
#define USB_CONFIG_DESC             (2)
#define USB_STRING_DESC             (3)
#define USB_INTERFACE_DESC          (4)
#define USB_ENDPOINT_DESC           (5)
#define USB_DEVICE_QUALIFIER_DESC   (6)
#define USB_OTHER_SPEED_CFG_DESC    (7)
#define USB_INTEFACE_PWR_DESC       (8)

// Values for the device descriptor bcdUSB field
#define USB_BCD_USB_1_0 (0x0100)
#define USB_BCD_USB_1_1 (0x0110)
#define USB_BCD_USB_2_0 (0x0200)

// Values for the device descriptor bDevClass field
#define USB_CLASS_INTERFACE_SPECIFIED  (0)
#define USB_CLASS_VENDOR_SPECIFIED     (0xff)

// Values for the device descriptor bMaxPktSize0 field
#define USB_MAX_PKT_SIZE_8_BYTES    (8)
#define USB_MAX_PKT_SIZE_16_BYTES   (16)
#define USB_MAX_PKT_SIZE_32_BYTES   (32)
#define USB_MAX_PKT_SIZE_64_BYTES   (64)

// Device descriptor
typedef struct __attribute__((packed))
{
   uint8_t  bLength;
   uint8_t  bDescType;
   uint16_t bcdUSB;
   uint8_t  bDevClass;
   uint8_t  bDevSubClass;
   uint8_t  bDevProtocol;
   uint8_t  bMaxPktSize0;
   uint16_t idVendor;
   uint16_t idProduct;
   uint16_t bcdDevice;
   uint8_t  iManufacturer;
   uint8_t  iProduct;
   uint8_t  iSerialNum;
   uint8_t  bNumConfigs;
} usbDeviceDesc_t;

// A bit mask of attributes for the config descriptor bmAttribute field
#define USB_CFG_ATTR_REMOTE_WAKEUP  (1 << 5)
#define USB_CFG_ATTR_SELF_POWERED   (1 << 6)
#define USB_CFG_ATTR_ALWAYS_SET     (1 << 7)

// Configuration descriptor
typedef struct __attribute__((packed))
{
   uint8_t  bLength;
   uint8_t  bDescType;
   uint16_t wTotalLength;
   uint8_t  bNumInterfaces;
   uint8_t  bConfigValue;
   uint8_t  iConfig;
   uint8_t  bmAttributes;
   uint8_t  bMaxPower;
} usbConfigDesc_t;

// Interface descriptor
typedef struct __attribute__((packed))
{
   uint8_t  bLength;
   uint8_t  bDescType;
   uint8_t  bInterfaceNum;
   uint8_t  bAltSetting;
   uint8_t  bNumEndpoints;
   uint8_t  bInterfaceClass;
   uint8_t  bInterfaceSubClass;
   uint8_t  bInterfaceProtocol;
   uint8_t  iInterface;
} usbInterfaceDesc_t;

// Macros & bitmasks for the endpoint descriptor bEndpointAddr field
#define USB_EP_ADDR_EP_NUMBER(x)    (x & 0x0F)
#define USB_EP_ADDR_DIRECTION_IN    (1 << 7)
#define USB_EP_ADDR_DIRECTION_OUT   (0)

// Bitmasks for the endpoint descriptor bmAttributes field
#define USB_EP_ATTR_TRANS_CONTROL      (0 << 0)
#define USB_EP_ATTR_TRANS_ISOCHRONOUS  (1 << 0)
#define USB_EP_ATTR_TRANS_BULK         (2 << 0)
#define USB_EP_ATTR_TRANS_INTERRUPT    (3 << 0)
#define USB_EP_ATTR_TRANS_MASK         (3 << 0)
#define USB_EP_ATTR_ISO_SYNC_NONE      (0 << 2)
#define USB_EP_ATTR_ISO_SYNC_ASYNC     (1 << 2)
#define USB_EP_ATTR_ISO_SYNC_ADAPTIVE  (2 << 2)
#define USB_EP_ATTR_ISO_SYNC_SYNC      (3 << 2)
#define USB_EP_ATTR_ISO_SYNC_MASK      (3 << 2)
#define USB_EP_ATTR_ISO_USE_DATA       (0 << 4)
#define USB_EP_ATTR_ISO_USE_FEEDBACK   (1 << 4)
#define USB_EP_ATTR_ISO_USE_EXPLICIT   (2 << 4)
#define USB_EP_ATTR_ISO_USE_MASK       (3 << 4)

// Endpoint descriptor
typedef struct __attribute__((packed))
{
   uint8_t  bLength;
   uint8_t  bDescType;
   uint8_t  bEndpointAddr;
   uint8_t  bmAttributes;
   uint16_t wMaxPktSize;
   uint8_t  bInterval;
} usbEndpointDesc_t;

// A macro to convert a specially formatted ASCII string into a USB UTF-16LE string
//   The input strings, "s", must be formatted such that a '\0' character follows 
//   every normal ASCII character in the string, as in: "T\0E\0S\0T\0"
#define USB_STRING(s)      { (sizeof(s) + 1), USB_STRING_DESC, s }
#define USB_UTF_16LE(c)    'c','\0'

// String descriptor - a bit of a hack but easily converts ASCII strings to the structure
typedef struct usb_string
{
   uint8_t bLength;
   uint8_t bDescriptorType;
   char    data[];
} usbStringDesc_t;

// Bitmasks for the setup packet bmRequestType field
#define USB_REQ_TYPE_TO_DEVICE      (0 << 0)
#define USB_REQ_TYPE_TO_INTERFACE   (1 << 0)
#define USB_REQ_TYPE_TO_ENDPOINT    (2 << 0)
#define USB_REQ_TYPE_TO_OTHER       (3 << 0)
#define USB_REQ_TYPE_TO_MASK        (0x1f << 0)
#define USB_REQ_TYPE_TYPE_STANDARD  (0 << 5)
#define USB_REQ_TYPE_TYPE_CLASS     (1 << 5)
#define USB_REQ_TYPE_TYPE_VENDOR    (2 << 5)
#define USB_REQ_TYPE_TYPE_MASK      (3 << 5)
#define USB_REQ_TYPE_DEV2HOST       (1 << 7)

// Standard request codes for the setup packet bRequest field
#define USB_REQ_GET_STATUS       (0)
#define USB_REQ_CLR_FEATURE      (1)
#define USB_REQ_SET_FEATURE      (3)
#define USB_REQ_SET_ADDRESS      (5)
#define USB_REQ_GET_DESC         (6)
#define USB_REQ_SET_DESC         (7)
#define USB_REQ_GET_CONFIG       (8)
#define USB_REQ_SET_CONFIG       (9)
#define USB_REQ_GET_INTERFACE    (10)
#define USB_REQ_SET_INTERFACE    (11)
#define USB_REQ_SYNC_FRAME       (12)

typedef struct
{
   uint8_t lo;
   uint8_t hi;
} le16_t;

typedef struct
{
   union
   {
      uint16_t word; // this only works for little-endian compilers
      le16_t   bytes;
   };
} utf16le_t;

// Setup Packet
typedef struct
{
   uint8_t  bmRequestType;
   uint8_t  bRequest;
   utf16le_t wValue;
   utf16le_t wIndex;
   utf16le_t wLength;
} usbSetupPacket_t;

// CDC descriptor type values, bDescType
#define USB_CDC_INTERFACE  (0x24)
#define USB_CDC_ENDPOINT   (0x25)

// CDC descriptor sub-class values, bDescSubType
#define USB_CDC_SUB_TYPE_HEADER     (0x00)
#define USB_CDC_SUB_TYPE_CALL_MGT   (0x01)
#define USB_CDC_SUB_TYPE_ACM        (0x02) // Abstract Control Management
#define USB_CDC_SUB_TYPE_UNION      (0x06)

// Values for the CDC header descriptor bcdCDC field
#define USB_BCD_CDC_1_1 (0x0110)

// CDC header descriptor
typedef struct __attribute__((packed))
{
   uint8_t  bFnLength;
   uint8_t  bDescType;
   uint8_t  bDescSubType;
   uint16_t bcdCDC;
} usbCdcHdrDesc_t;

// Bitmask values for the call management descriptor bmCapabilities field
#define USB_CDC_CM_CAPS_HANDLES_CM     (1 << 0) // Handles call mgt itself
#define USB_CDC_CM_CAPS_CM_VIA_DATA    (1 << 1) // Supports call mgt via data class interface

// CDC call management descriptor
typedef struct __attribute__((packed))
{
   uint8_t  bFnLength;
   uint8_t  bDescType;
   uint8_t  bDescSubType;
   uint8_t  bmCapabilities;
   uint8_t  bDataInterface;
} usbCdcCallMgtDesc_t;

// Bitmask values for the ACM descriptor bmCapabilities field
#define USB_CDC_ACM_CAPS_COMM    (1 << 0) // Supports Get/Set/Clear Comm Features
#define USB_CDC_ACM_CAPS_LINE    (1 << 1) // Supports Get/Set/Ctrl Line Features
#define USB_CDC_ACM_CAPS_BREAK   (1 << 2) // Supports Send_Break request
#define USB_CDC_ACM_CAPS_CONN    (1 << 3) // Supports Network_Connection notification

// CDC ACM (Abstract Control Management) descriptor
typedef struct __attribute__((packed))
{
   uint8_t  bFnLength;
   uint8_t  bDescType;
   uint8_t  bDescSubType;
   uint8_t  bmCapabilities;
} usbCdcAcmDesc_t;

// CDC union descriptor - only listing a single subordinate interface
typedef struct __attribute__((packed))
{
   uint8_t  bFnLength;
   uint8_t  bDescType;
   uint8_t  bDescSubType;
   uint8_t  bControlInterface;
   uint8_t  bSubInterface0;
} usbCdcUnionDesc_t;

#endif // _USB_H_
