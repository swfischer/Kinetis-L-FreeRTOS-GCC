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

#include <stdint.h>

#include "clk.h"
#include "MKL25Z4.h"
#include "pinmux.h"

#define PIN_DEFAULTS_WITH_ALT(x) \
    INT_CFG_DEFAULT, MUX_CNTL_ALT_##x \
  , DRV_STRENGTH_DEFAULT, PASSIVE_FILTER_DEFAULT \
  , SLEW_RATE_DEFAULT, PULL_ENABLE_DEFAULT, PULL_SELECT_DEFAULT

enum
{ PORT_A = 0
, PORT_B
, PORT_C
, PORT_D
, PORT_E
, PORT_MAX
, PORT_INVALID // Must be last
};

enum
{ PORT_PIN_0 = 0
,              PORT_PIN_1,  PORT_PIN_2,  PORT_PIN_3
, PORT_PIN_4,  PORT_PIN_5,  PORT_PIN_6,  PORT_PIN_7
, PORT_PIN_8,  PORT_PIN_9,  PORT_PIN_10, PORT_PIN_11
, PORT_PIN_12, PORT_PIN_13, PORT_PIN_14, PORT_PIN_15
, PORT_PIN_16, PORT_PIN_17, PORT_PIN_18, PORT_PIN_19
, PORT_PIN_20, PORT_PIN_21, PORT_PIN_22, PORT_PIN_23
, PORT_PIN_24, PORT_PIN_25, PORT_PIN_26, PORT_PIN_27
, PORT_PIN_28, PORT_PIN_29, PORT_PIN_30, PORT_PIN_31
, PORT_PIN_INVALID // Must be last
};

enum
{ INT_CFG_DISABLE = 0
, INT_CFG_DMA_RISING
, INT_CFG_DMA_FALLING
, INT_CFG_DMA_BOTH
, INT_CFG_RESERVED_4
, INT_CFG_RESERVED_5
, INT_CFG_RESERVED_6
, INT_CFG_RESERVED_7
, INT_CFG_ZERO
, INT_CFG_RISING
, INT_CFG_FALLING
, INT_CFG_BOTH
, INT_CFG_ONE
, INT_CFG_RESERVED_13
, INT_CFG_RESERVED_14
, INT_CFG_RESERVED_15
, INT_CFG_DEFAULT = INT_CFG_DISABLE
};

enum
{ MUX_CNTL_ALT_0 = 0
, MUX_CNTL_ALT_1
, MUX_CNTL_ALT_2
, MUX_CNTL_ALT_3
, MUX_CNTL_ALT_4
, MUX_CNTL_ALT_5
, MUX_CNTL_ALT_6
, MUX_CNTL_ALT_7
, MUX_CNTL_DEFAULT = MUX_CNTL_ALT_0
};

enum
{ DRV_STRENGTH_LOW = 0
, DRV_STRENGTH_HIGH
, DRV_STRENGTH_DEFAULT = DRV_STRENGTH_LOW
};

enum
{ PASSIVE_FILTER_DISABLE = 0
, PASSIVE_FILTER_ENABLE
, PASSIVE_FILTER_DEFAULT = PASSIVE_FILTER_DISABLE
};

enum
{ SLEW_RATE_FAST = 0
, SLEW_RATE_SLOW
, SLEW_RATE_DEFAULT = SLEW_RATE_FAST
};

enum
{ PULL_DISABLE = 0
, PULL_ENABLE
, PULL_ENABLE_DEFAULT = PULL_DISABLE
};

enum
{ PULL_DOWN = 0
, PULL_UP
, PULL_SELECT_DEFAULT = PULL_DOWN
};

typedef struct
{
   uint32_t port            : 3;
   uint32_t pin             : 5;
   uint32_t intCfg          : 4;
   uint32_t pinMuxCntl      : 3;
   uint32_t drvStrengthEn   : 1;
   uint32_t passiveFilterEn : 1;
   uint32_t slewRateEn      : 1;
   uint32_t pullEn          : 1;
   uint32_t pullSelect      : 1;
   uint32_t reserved        : 12;
} muxConfig_t;

const muxConfig_t muxConfig[] =
{ { PORT_B, PORT_PIN_18, PIN_DEFAULTS_WITH_ALT(3) } // Red LED (TPM2_CH0)
, { PORT_B, PORT_PIN_19, PIN_DEFAULTS_WITH_ALT(3) } // Green LED (TPM2_CH1)
, { PORT_D, PORT_PIN_1,  PIN_DEFAULTS_WITH_ALT(4) } // Blue LED (TPM0_CH1)
, { PORT_A, PORT_PIN_1,  PIN_DEFAULTS_WITH_ALT(2) } // Console RX (UART0)
, { PORT_A, PORT_PIN_2,  PIN_DEFAULTS_WITH_ALT(2) } // Console TX (UART0)
, { PORT_B, PORT_PIN_16, PIN_DEFAULTS_WITH_ALT(0) } // Touch Channel 9
, { PORT_B, PORT_PIN_17, PIN_DEFAULTS_WITH_ALT(0) } // Touch Channel 10
, { PORT_E, PORT_PIN_24, PIN_DEFAULTS_WITH_ALT(5) } // Accelerometer (I2C CLK)
, { PORT_E, PORT_PIN_25, PIN_DEFAULTS_WITH_ALT(5) } // Accelerometer (I2C DATA)

// Already set in startup code
//, { PORT_A, PORT_PIN_18, PIN_DEFAULTS_WITH_ALT(0) } // EXTAL
//, { PORT_A, PORT_PIN_19, PIN_DEFAULTS_WITH_ALT(0) } // XTAL

// Table Termination - must be last
, { PORT_INVALID }
};

static void configurePin(const muxConfig_t *pConfig);
static void portClkEnable(int port);

// ----------------------------------------------------------------------------
// External Functions
// ----------------------------------------------------------------------------

void pinmuxInit(void)
{
   int clkEnables[PORT_MAX] = {0};
   int i = 0;

   while (muxConfig[i].port != PORT_INVALID)
   {
      if (!clkEnables[muxConfig[i].port])
      {
         portClkEnable(muxConfig[i].port);
         clkEnables[muxConfig[i].port] = 1;
      }

      configurePin(&muxConfig[i]);
      i++;
   }
}

// ----------------------------------------------------------------------------
// Local Functions
// ----------------------------------------------------------------------------

static void configurePin(const muxConfig_t *pConfig)
{
   static const PORT_MemMapPtr ports[] =
   { PORTA_BASE_PTR, PORTB_BASE_PTR, PORTC_BASE_PTR
   , PORTD_BASE_PTR, PORTE_BASE_PTR
   };

   uint32_t config;

   config  = PORT_PCR_IRQC(pConfig->intCfg);
   config |= PORT_PCR_MUX(pConfig->pinMuxCntl);
   config |= pConfig->drvStrengthEn << PORT_PCR_DSE_SHIFT;
   config |= pConfig->passiveFilterEn << PORT_PCR_PFE_SHIFT;
   config |= pConfig->slewRateEn << PORT_PCR_SRE_SHIFT;
   config |= pConfig->pullEn << PORT_PCR_PE_SHIFT;
   config |= pConfig->pullSelect << PORT_PCR_PS_SHIFT;

   PORT_PCR_REG(ports[pConfig->port], pConfig->pin) = config;
}

static void portClkEnable(int port)
{
   static const clkGate_t port_clks[] = { CLK_PORTA, CLK_PORTB, CLK_PORTC, CLK_PORTD, CLK_PORTE };

   clkEnable(port_clks[port]);
}
