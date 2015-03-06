
// ----------------------------------------------------------------------------
// Based on the Freescale USB echo device demo
// ----------------------------------------------------------------------------

#ifndef _RINGBUFFER_H_
#define _RINGBUFFER_H_

#include <stdint.h>

// Error Codes
#define OK                 (0)
#define NOT_ENOUGH_SPACE   (1)

// Prototypes
extern void ringBufferInit(uint8_t* ,uint8_t);
extern uint8_t ringBufferRequest(uint8_t* ,uint16_t);

#endif // _RINGBUFFER_H_
