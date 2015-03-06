
// ----------------------------------------------------------------------------
// Based on the Freescale USB echo device demo
// ----------------------------------------------------------------------------

#include <stdint.h>

#include "ringBuffer.h"

// Pointers
static volatile uint8_t *OUT_StartAddress;
static volatile uint8_t *OUT_EndAddress;
static volatile uint8_t *OUT_UsbPointer;
static volatile uint8_t *OUT_SciPointer;

// Variables
static volatile uint8_t sBufferMaxSize;
static volatile uint8_t sBufferOverflow;

// ----------------------------------------------------------------------------
// External Functions
// ----------------------------------------------------------------------------

void ringBufferInit(uint8_t* buffer, uint8_t maxSize)
{
   // Buffer Initialization
   OUT_EndAddress = buffer + maxSize - 1;
   OUT_StartAddress = buffer;
   sBufferMaxSize = maxSize;
   OUT_UsbPointer = buffer;
   OUT_SciPointer = buffer;
   sBufferOverflow = 0;
}

uint8_t ringBufferRequest(uint8_t* data, uint16_t requestSize)
{
   uint8_t freeSpace;
   
   // Check for OverFlow
   if (sBufferOverflow)
   {
      return (NOT_ENOUGH_SPACE);
   }

   // Calculate Free Space
   if (OUT_UsbPointer < OUT_SciPointer)
   {
      freeSpace = (uint8_t)(OUT_SciPointer - OUT_UsbPointer);
   }
   else
   {
      freeSpace = sBufferMaxSize - (OUT_UsbPointer - OUT_SciPointer);
   }

   // Validate requested size
   if (freeSpace < requestSize)
   {
      return(NOT_ENOUGH_SPACE);
   }

   if (freeSpace == requestSize)
   {
      sBufferOverflow = 1;
   }

   // Buffer Copy
   while (requestSize--)
   {
      *OUT_UsbPointer = *data;
      OUT_UsbPointer++;
      data++;
      if (OUT_UsbPointer > OUT_EndAddress)
      {
         OUT_UsbPointer = OUT_StartAddress;
      }
   }

   return (OK);
}
