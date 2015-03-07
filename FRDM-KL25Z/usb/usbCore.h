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

#ifndef _USBCORE_H_
#define _USBCORE_H_

#include <stdint.h>

#define EP0_SIZE            32
#define EP1_SIZE            32
#define EP2_SIZE            32
#define EP3_SIZE            32

// Macros
#define EP3_CTR   tBDTtable[4].Stat._byte= kSIE 

#define usbSIE_CONTROL(EP)   (tBDTtable[EP<<2].Stat._byte = kSIE)
#define usbMCU_CONTROL(EP)   (tBDTtable[EP<<2].Stat._byte = kMCU)
#define usbEP_Reset(EP)      (tBDTtable[EP<<2].Cnt = 0x0020)

#define INTERRUPT   0x03
#define BULK        0x05

// BDT status value
#define kMCU      0x00
#define kSIE      0x80

#define kUDATA0   0x88
#define kUDATA1   0xC8

#define mSETUP_TOKEN    0x0D
#define mOUT_TOKEN      0x01
#define mIN_TOKEN       0x09

// USB commands
#define mGET_STATUS           0
#define mCLR_FEATURE          1
#define mSET_FEATURE          3
#define mSET_ADDRESS          5
#define mGET_DESC             6
#define mSET_DESC             7
#define mGET_CONFIG           8
#define mSET_CONFIG           9
#define mGET_INTF             10
#define mSET_INTF             11
#define mSYNC_FRAME           12
#define	mGET_MAXLUN	          0xFE		// Mass Storage command
  
#define mDEVICE		            1
#define mCONFIGURATION	            2
#define mSTRING	        	    3
#define mINTERFACE	            4
#define mENDPOINT       	    5
#define	mDEVICE_QUALIFIE            6
#define mOTHER_SPEED_CONFIGURATION  7
#define mINTERFACE_POWER	    8

#define mREPORT                     0x22

// Request Types
#define STANDARD_REQ    0x00
#define SPECIFIC_REQ    0x20
#define VENDORSPEC_REQ  0x40
#define DEVICE_REQ      0x00
#define INTERFACE_REQ   0x01
#define ENDPOINT_REQ    0x02

enum
{    uSETUP,
    uDATA
};

enum
{    EP0,
    EP1,
    EP2,
    EP3,
    EP4,
    EP5,
    DUMMY,
    LOADER
    
};

enum
{    uPOWER,
    uENUMERATED,
    uENABLED,
    uADDRESS,
    uREADY    
};
enum
{    fIN,
    fOUT
};

enum
{    bEP0OUT_ODD,
    bEP0OUT_EVEN,
    bEP0IN_ODD,
    bEP0IN_EVEN,
    bEP1OUT_ODD,
    bEP1OUT_EVEN,
    bEP1IN_ODD,
    bEP1IN_EVEN,
    bEP2OUT_ODD,
    bEP2OUT_EVEN,
    bEP2IN_ODD,
    bEP2IN_EVEN,
    bEP3OUT_ODD,
    bEP3OUT_EVEN,
    bEP3IN_ODD,
    bEP3IN_EVEN
};

/***** Data Types *****/
/*     */

typedef union _tBDT_STAT
{    uint8_t _byte;
    struct
    {
        uint8_t :1;
        uint8_t :1;
        uint8_t BSTALL:1;              //Buffer Stall Enable
        uint8_t DTS:1;                 //Data Toggle Synch Enable
        uint8_t NINC:1;                //Address Increment Disable
        uint8_t KEEP:1;                //BD Keep Enable
        uint8_t DATA:1;                //Data Toggle Synch Value
        uint8_t UOWN:1;                //USB Ownership
    }McuCtlBit;
       
    struct
    {
        uint8_t    :2;
        uint8_t PID:4;                 //Packet Identifier
        uint8_t    :2;
    }RecPid;
} tBDT_STAT;                         //Buffer Descriptor Status Register

typedef struct _tBDT
{    tBDT_STAT Stat;
    uint8_t  dummy;
    uint16_t Cnt;
    uint32_t Addr;             
} tBDT;                             

// It's used in some macros here.
extern tBDT tBDTtable[];

#define SWAP16(val)                                                 \
    (uint16_t)((((uint16_t)(val) >> 0x8) & 0xFF) |                    \
    (((uint16_t)(val) & 0xFF) << 0x8))

#define SWAP32(val)                                                 \
    (uint32_t)((SWAP16((uint32_t)(val) & (uint32_t)0xFFFF) << 0x10) |  \
    (SWAP16((uint32_t)((val) >> 0x10))))

typedef struct _tUSB_Setup 
{       uint8_t bmRequestType;
       uint8_t bRequest;
       uint8_t wValue_l;
       uint8_t wValue_h;
       uint8_t wIndex_l;
       uint8_t wIndex_h;
       uint8_t wLength_l;
       uint8_t wLength_h;
} tUSB_Setup;

#define GET_STATUS              0x00
#define CLEAR_FEATURE           0x01
#define SET_FEATURE             0x03

typedef uint8_t (*usbInterfaceReqHandler)(uint8_t ep, tUSB_Setup *pkt);

extern void usbCoreInit(usbInterfaceReqHandler handler);
extern void usbCoreEpInTransfer(uint8_t ep, uint8_t *data, uint8_t size);
extern uint8_t  usbCoreEpOutTransfer(uint8_t ep, uint8_t *data);
extern uint16_t usbCoreEpOutSizeCheck(uint8_t ep);

#endif // _USBCORE_H_
