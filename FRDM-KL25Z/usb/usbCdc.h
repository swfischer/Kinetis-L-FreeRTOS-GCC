
// ----------------------------------------------------------------------------
// Based on the Freescale USB echo device demo
// ----------------------------------------------------------------------------

#ifndef _USBCDC_H_
#define _USBCDC_H_

#include <stdint.h>

#include "usb.h"

#include "fs_types.h"
#include "ring_buffer.h"

// Re-enumeration macros and flag
extern volatile uint8_t usbCdcIsrFlags;
#define VBUS_LOW_EVENT     (1 << 0)
#define VBUS_HIGH_EVENT    (1 << 1)

#define CDC_INPointer   gu8EP2_IN_ODD_Buffer
#define CDC_OUTPointer  gu8EP3_OUT_ODD_Buffer

// Defines
#define CDC_BUFFER_SIZE 128
#define EP_OUT          3
#define EP_IN           2

// USB_CDC Definitions
#define WAITING_FOR_ENUMERATION 0x00
#define SET_LINE_CODING         0x20
#define GET_LINE_CODING         0x21
#define SET_CONTROL_LINE_STATE  0x22
#define LOADER_MODE             0xAA
#define GET_INTERFACE           0x0A

// TypeDefs
typedef struct
{
   uint32_t dteRate;
   uint8_t  charFormat;
   uint8_t  parityType;
   uint8_t  databits;
} usbCdcLineCoding_t;

extern void usbCdcInit(void);
extern void usbCdcEngine(void);

#endif // _USBCDC_H_
