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
// This code is based on Andrew Payne's work.
//
//  Copyright (c) 2012-2013 Andrew Payne <andy@payne.org>
// ----------------------------------------------------------------------------

#include "led.h"
#include "MKL25Z4.h"
#include "os.h"

// Memory locations defined by the linker
extern uint32_t __heap_start[];
extern uint32_t __StackTop[];
extern uint8_t __bss_start[];
extern uint8_t __bss_end[];
extern uint8_t __data_start[];
extern uint8_t __data_end[];
extern uint8_t __data_rom_start[];

// The main function
extern int main(void);

// The register frame pushed onto the stack during exceptions
typedef struct
{ uint32_t r0;
  uint32_t r1;
  uint32_t r2;
  uint32_t r3;
  uint32_t r12;
  uint32_t lr;
  void    *pc;
  uint32_t psr;
} hw_stackframe_t; 

       void _resetHandler(void) __attribute__((naked, aligned(8)));
static void defaultHandler(void) __attribute__((interrupt("IRQ")));
static void fault(uint32_t pattern);
       void faultPutString(char *p);
       void faultPutWord(uint32_t w);
static void faultUartInit(int baud_rate);
static void hardFaultEntry(void) __attribute__((naked)); 
       void hardFaultHandler(uint32_t lr, void *psp, void *msp) __attribute__((naked));
static void initClocks(void);

// Weak definitions of handlers point to defaultHandler if not implemented
void NMI_Handler(void)      __attribute__ ((weak, alias("defaultHandler")));
void SVC_Handler(void)      __attribute__ ((weak, alias("defaultHandler")));
void PendSV_Handler(void)   __attribute__ ((weak, alias("defaultHandler")));
void SysTick_Handler(void)  __attribute__ ((weak, alias("defaultHandler")));
void IRQ_Handler(void)      __attribute__ ((weak, alias("defaultHandler")));

// ----------------------------------------------------------------------------
// Flash configuration field (loaded into flash memory at 0x400)
//
// Note: RESET_PIN_CFG is set to enable external RESET signal
//
__attribute__ ((section (".cfmconfig"))) const uint8_t _cfm[0x10] = 
{ 0xFF // NV_BACKKEY3: KEY=0xFF
, 0xFF // NV_BACKKEY2: KEY=0xFF
, 0xFF // NV_BACKKEY1: KEY=0xFF
, 0xFF // NV_BACKKEY0: KEY=0xFF
, 0xFF // NV_BACKKEY7: KEY=0xFF
, 0xFF // NV_BACKKEY6: KEY=0xFF
, 0xFF // NV_BACKKEY5: KEY=0xFF
, 0xFF // NV_BACKKEY4: KEY=0xFF
, 0xFF // NV_FPROT3: PROT=0xFF
, 0xFF // NV_FPROT2: PROT=0xFF
, 0xFF // NV_FPROT1: PROT=0xFF
, 0xFF // NV_FPROT0: PROT=0xFF
, 0x7E // NV_FSEC: KEYEN=1, MEEN=3, FSLACC=3, SEC=2
, 0xFF // NV_FOPT: ??=1,??=1,FAST_INIT=1,LPBOOT1=1,RESET_PIN_CFG=1,NMI_DIS=1,EZPORT_DIS=1,LPBOOT0=1
, 0xFF
, 0xFF
};

// ----------------------------------------------------------------------------
// Interrupt vector table (loaded into flash memory at 0x000)
//
void (* const InterruptVector[])() __attribute__ ((section(".isr_vector"))) =
{ (void(*)(void)) __StackTop // Initial stack pointer
, _resetHandler // Reset handler
, NMI_Handler
, hardFaultEntry
, 0
, 0
, 0
, 0
, 0
, 0
, 0
, SVC_Handler
, 0
, 0
, PendSV_Handler
, SysTick_Handler
// IRQs
, IRQ_Handler // DMA0
, IRQ_Handler // DMA1
, IRQ_Handler // DMA2
, IRQ_Handler // DMA3
, IRQ_Handler // Unused
, IRQ_Handler // FTFA
, IRQ_Handler // PMC
, IRQ_Handler // LLWU
, IRQ_Handler // I2C0
, IRQ_Handler // I2C1
, IRQ_Handler // SPI0
, IRQ_Handler // SPI1
, IRQ_Handler // UART0
, IRQ_Handler // UART1
, IRQ_Handler // UART2
, IRQ_Handler // ADC0
, IRQ_Handler // CMP0
, IRQ_Handler // TPM0
, IRQ_Handler // TPM1
, IRQ_Handler // TPM2
, IRQ_Handler // RTC Alarm
, IRQ_Handler // RTC Seconds
, IRQ_Handler // PIT
, IRQ_Handler // Unused
, IRQ_Handler // USBOTG
, IRQ_Handler // DAC0
, IRQ_Handler // TSI0
, IRQ_Handler // MCG
, IRQ_Handler // LPTMR0
, IRQ_Handler // Unused
, IRQ_Handler // PORTA
, IRQ_Handler // PORTD
};

// ----------------------------------------------------------------------------
// _resetHandler() -- Reset entry point.  
//
//      The CPU reset vector points here.  Initialize the CPU, and jump
//      to the C runtime start, which will eventually invoke main()
//

void _resetHandler(void)
{
   uint32_t len;
   uint8_t *fr;
   uint8_t *to;

   SIM_COPC = 0; // Disable the watchdog timer   
   SCB_VTOR = (uint32_t) InterruptVector;

   // Copy values to initialize data segment
   fr = __data_rom_start;
   to = __data_start;
   len = __data_end - __data_start;
   while(len--)
      *to++ = *fr++;

   // Clear the bss
   to = __bss_start;
   len = __bss_end - __bss_start;
   while(len--)
      *to++ = 0;

   initClocks();

   main();

   fault(0xAAAAAAAA); // ...should never get here.
}

// ----------------------------------------------------------------------------
//  Default interrupt handler
//

static void defaultHandler(void)
{
   uint32_t x;

   x = (SCB_ICSR & SCB_ICSR_VECTACTIVE_MASK);

   faultUartInit(BAUD_RATE);
   faultPutString("defaultHandler\n");
   faultPutString("vector = ");
   faultPutWord(x);

   fault(0b11111110);          // Blink LED and halt
}

// ----------------------------------------------------------------------------
// Blink an LED based on a pattern bitmask
//

static void fault(uint32_t pattern)
{
   volatile int i;

   for(;;)
   {
      ledRed(pattern & 1 ? 100 : 0); // Set RED led based on LSB
      pattern = (pattern >> 1) | (pattern << 31); // Rotate

      // Just a hacky delay
      i = 1000000;
      while (i--)
         ;
   }
}

// ----------------------------------------------------------------------------
//

void faultPutString(char *p)
{
   int i = 0;

   while (p[i])
   {
      if (p[i] == '\n')
      {
         while (!(UART0_S1 & UART_S1_TDRE_MASK)) {};
         UART0_D = '\r';
      }

      while (!(UART0_S1 & UART_S1_TDRE_MASK)) {};
      UART0_D = p[i];
      i++;
   }
}

// ----------------------------------------------------------------------------
//

void faultPutWord(uint32_t w)
{
   static const char tohex[] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F' };
   int i;
   char c;
   
   for (i = 7; i >= 0; i--)
   {
      while (!(UART0_S1 & UART_S1_TDRE_MASK)) {};
      c = tohex[((w >> (4 * i)) & 0xf)];
      UART0_D = c;
   }

   while (!(UART0_S1 & UART_S1_TDRE_MASK)) {};
   UART0_D = '\r';
   while (!(UART0_S1 & UART_S1_TDRE_MASK)) {};
   UART0_D = '\n';
}

// ----------------------------------------------------------------------------
// Init TX only - Just for fault handling

static void faultUartInit(int baud_rate)
{
   // Only init if it's not already done
   if (!(SIM_SCGC4 & SIM_SCGC4_UART0_MASK))
   {
      SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
      SIM_SCGC4 |= SIM_SCGC4_UART0_MASK;
      SIM_SOPT2 &= ~SIM_SOPT2_UART0SRC_MASK;
      SIM_SOPT2 |= SIM_SOPT2_UART0SRC(1);

      PORTA_PCR1 = PORT_PCR_MUX(2);
      PORTA_PCR2 = PORT_PCR_MUX(2);

      UART0_C2 = 0;
      UART0_C1 = 0;
      UART0_C3 = 0;
      UART0_S2 = 0;     

      uint16_t divisor = (CORE_CLOCK / 16) / baud_rate;
      UART0_C4 = UARTLP_C4_OSR(16 - 1);
      UART0_BDH = (divisor >> 8) & UARTLP_BDH_SBR_MASK;
      UART0_BDL = (divisor & UARTLP_BDL_SBR_MASK);
   }

   // Enable TX only
   UART0_C2 = UARTLP_C2_TE_MASK;

   // Disable the UART interrupt
   NVIC_ICER = (1 << (INT_UART0 - 16));
}

// ----------------------------------------------------------------------------
// Handle hard faults:  print debugging information and halt
//

static void hardFaultEntry(void) 
{
    // Set up arguments and invoke hardFaultHandler()
    asm("mov  r0, lr\n"   // Arg 0
        "mrs  r1, psp\n"  // Arg 1
        "mrs  r2, msp\n"  // Arg 2
        "b  hardFaultHandler\n");
}

// ----------------------------------------------------------------------------
//

void hardFaultHandler(uint32_t lr, void *psp, void *msp)
{
   hw_stackframe_t *frame;
  
   // Find the active stack pointer (MSP or PSP)
   if(lr & 0x4)
      frame = psp;
   else
      frame = msp;

   faultUartInit(BAUD_RATE);
   faultPutString("\nhardFaultHandler\n");
   faultPutString("PC = ");
   faultPutWord((uint32_t) frame->pc);
   faultPutString("LR = ");
   faultPutWord(frame->lr);
   faultPutString("R0 = ");
   faultPutWord(frame->r0);
   faultPutString("R1 = ");
   faultPutWord(frame->r1);
   faultPutString("R2 = ");
   faultPutWord(frame->r2);
   faultPutString("R3 = ");
   faultPutWord(frame->r3);
   faultPutString("R12 = ");
   faultPutWord(frame->r12);

   while(1) {}; // Just hang here for now

   fault(0b1111111000);
}

// ----------------------------------------------------------------------------
// Initialize the system clocks to a 48 Mhz core clock speed
// Mode progression:  FEI (reset) -> FBE -> PBE -> PEE
//
// Note:  Generated by Processor Expert, cleaned up by hand. 
//        For detailed information on clock modes, see the 
//        "KL25 Sub-Family Reference Manual" section 24.5.3.1
//

static void initClocks(void)
{   
   // Enable clock gate to Port A module to enable pin routing (PORTA=1)
   SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
    
   // Divide-by-2 for clock 1 and clock 4 (OUTDIV1=1, OUTDIV4=1)   
   SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0x01) | SIM_CLKDIV1_OUTDIV4(0x01);

   // System oscillator drives 32 kHz clock for various peripherals (OSC32KSEL=0)
   SIM_SOPT1 &= ~SIM_SOPT1_OSC32KSEL(0x03);

   // Select PLL as a clock source for various peripherals (PLLFLLSEL=1)
   // Clock source for TPM counter clock is MCGFLLCLK or MCGPLLCLK/2
   SIM_SOPT2 |= SIM_SOPT2_PLLFLLSEL_MASK;
   SIM_SOPT2 = (SIM_SOPT2 & ~(SIM_SOPT2_TPMSRC(0x02))) | SIM_SOPT2_TPMSRC(0x01);
                  
   // PORTA_PCR18: ISF=0,MUX=0
   // PORTA_PCR19: ISF=0,MUX=0
   PORTA_PCR18 &= ~((PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x07)));
   PORTA_PCR19 &= ~((PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x07)));                                                   
   // Switch to FBE Mode
    
   // OSC0_CR: ERCLKEN=0,??=0,EREFSTEN=0,??=0,SC2P=0,SC4P=0,SC8P=0,SC16P=0
   OSC0_CR = 0;                                                   
   // MCG_C2: LOCRE0=0,??=0,RANGE0=2,HGO0=0,EREFS0=1,LP=0,IRCS=0
   MCG_C2 = (MCG_C2_RANGE0(0x02) | MCG_C2_EREFS0_MASK);
   // MCG_C1: CLKS=2,FRDIV=3,IREFS=0,IRCLKEN=0,IREFSTEN=0
   MCG_C1 = (MCG_C1_CLKS(0x02) | MCG_C1_FRDIV(0x03));
   // MCG_C4: DMX32=0,DRST_DRS=0
   MCG_C4 &= ~((MCG_C4_DMX32_MASK | MCG_C4_DRST_DRS(0x03)));
   // MCG_C5: ??=0,PLLCLKEN0=0,PLLSTEN0=0,PRDIV0=1
   MCG_C5 = MCG_C5_PRDIV0(0x01);                                                   
   // MCG_C6: LOLIE0=0,PLLS=0,CME0=0,VDIV0=0
   MCG_C6 = 0;
    
   // Check that the source of the FLL reference clock is 
   // the external reference clock.
   while((MCG_S & MCG_S_IREFST_MASK) != 0)
      ;

   // Wait until external reference clock is selected as MCG output
   while((MCG_S & MCG_S_CLKST_MASK) != 8)
      ;
    
   // Switch to PBE mode
   //    Select PLL as MCG source (PLLS=1)
   MCG_C6 = MCG_C6_PLLS_MASK;

   // Wait until PLL locked
   while((MCG_S & MCG_S_LOCK0_MASK) == 0)
      ;
    
   // Switch to PEE mode
   //    Select PLL output (CLKS=0)
   //    FLL external reference divider (FRDIV=3)
   //    External reference clock for FLL (IREFS=0)
   MCG_C1 = MCG_C1_FRDIV(0x03);

   // Wait until PLL output
   while((MCG_S & MCG_S_CLKST_MASK) != 0x0CU)
      ;
}
