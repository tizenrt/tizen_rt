/****************************************************************************
 * arch/arm/src/rda5981x/rda5981x_gpio.c
 *
 *   Copyright (C) 2010-2011, 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <arch/irq.h>

#include "up_arch.h"
#include "chip.h"
#include "rda_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Default input pin configuration */

#define DEFAULT_INPUT (GPIO_DIR_INPUT)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/
/* These tables have global scope because they are also used in
 * rda5981x_gpiodbg.c
 */

/* We have to remember the configured interrupt setting.. PINs are not
 * actually set up to interrupt until the interrupt is enabled.
 */

#ifdef CONFIG_RDA5981X_GPIOIRQ
uint64_t g_intedge0;
uint64_t g_intedge2;
#endif

const uint32_t g_pinsel[GPIO_NPORTS * 2] =
{
  RDA_PINCONN_PINSEL0,
  RDA_PINCONN_PINSEL1,
  RDA_PINCONN_PINMODE0,
  RDA_PINCONN_PINMODE1,
  RDA_PINCONN_PINSEL2,
  RDA_PINCONN_PINSEL3,
  RDA_PINCONN_PINMODE2,
  RDA_PINCONN_PINMODE3
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if 0
/****************************************************************************
 * Name: rda_pullup
 *
 * Description:
 *   Get the address of the PINMODE register corresponding to this port and
 *   pin number.
 *
 ****************************************************************************/

static int rda_pullup(rda_pinset_t cfgset, unsigned int port,
                        unsigned int pin)
{
    /* Not support, return directly */
    return OK;
}
#endif

/****************************************************************************
 * Name: rda_setintedge
 *
 * Description:
 *  Remember the configured interrupt edge.  We can't actually enable the
 *  the edge interrupts until the called calls IRQ enabled function.
 *
 ****************************************************************************/

#ifdef CONFIG_RDA5981X_GPIOIRQ
static void rda_setintedge(unsigned int port, unsigned int pin,
                             unsigned int value)
{
  uint64_t *intedge;
  unsigned int shift;

  /* Which word to we use? */

  if (port == 0)
    {
      intedge = &g_intedge0;
    }
  else if (port == 2)
    {
      intedge  = &g_intedge2;
    }
  else
    {
      return;
    }

  /* Set the requested value in the PINSEL register */

  shift     = pin << 1;
  *intedge &= ~((uint64_t)3     << shift);
  *intedge |=  ((uint64_t)value << shift);
}
#endif /* CONFIG_RDA5981X_GPIOIRQ */

#if 0
/****************************************************************************
 * Name: rda_setopendrain
 *
 * Description:
 *   Set the ODMODE register for open drain mode
 *
 ****************************************************************************/

static void rda_setopendrain(unsigned int port, unsigned int pin)
{
    /* Not support */
}

/****************************************************************************
 * Name: rda_clropendrain
 *
 * Description:
 *   Reset the ODMODE register to disable open drain mode
 *
 ****************************************************************************/

static void rda_clropendrain(unsigned int port, unsigned int pin)
{
    /* Not support */
}
#endif

/****************************************************************************
 * Name: rda_configinterrupt
 *
 * Description:
 *   Configure a GPIO interrupt pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

static inline int rda_configinterrupt(rda_pinset_t cfgset, unsigned int port,
                                        unsigned int pin)
{
  /* TBD.
   */
#ifdef CONFIG_RDA5981X_GPIOIRQ
  rda_setintedge(port, pin, (cfgset & GPIO_EDGE_MASK) >> GPIO_EDGE_SHIFT);
#endif
  return OK;
}

/****************************************************************************
 * Name: rda_isgpiofuncsel
 *
 * Description:
 *   Check if pin selects the gpio function
 *
 ****************************************************************************/

static inline int rda_isgpiofuncsel(rda_pinset_t cfgset, unsigned int port, unsigned int pin)
{
    int isgpsel = 1;
    uint16_t func = cfgset & GPIO_FUNC_MASK;
    if(((0 == port) && (8 > pin)) || ((4 == port) && (2 > pin))) {
        if(func != GPIO_ALT1) {
            isgpsel = 0;
        }
    } else {
        if(func != GPIO_ALT0) {
            isgpsel = 0;
        }
    }
    return isgpsel;
}

/****************************************************************************
 * Name: rda_gpioindex
 *
 * Description:
 *   Get the GPIO index
 *
 ****************************************************************************/

static inline unsigned int rda_gpioindex(unsigned int port, unsigned int pin)
{
    unsigned int idx = 0U;
    if(4U <= port) {
        idx = (port - 4U) * 10U + 12U + pin;
    } else if(0U == port) {
        if(2U > pin) {
            idx = 26U + pin;
        } else if(7U < pin) {
            idx = 2U + pin;
        } else {
            idx = 12U + pin;
        }
    } else {
        idx = pin;
    }
    if(idx > GPIO_NPINS) {
        idx = GPIO_NPINS;
    }
    return idx;
}

/****************************************************************************
 * Name: rda_configdirection
 *
 * Description:
 *   Configure a GPIO pin input/output direction.
 *
 ****************************************************************************/

static inline int rda_configdirection(rda_pinset_t cfgset, unsigned int port, unsigned int pin)
{
    int ret = 0;
    unsigned int gpioidx = rda_gpioindex(port, pin);
    if(GPIO_NPINS > gpioidx) {
        uint32_t regval;
        regval = getreg32(RDA_GPIO_DIR) & ~(0x01UL << gpioidx);
        if(GPIO_DIR_OUTPUT != (cfgset & GPIO_DIR)) {
            regval |= (0x01UL << gpioidx);
            ret = 1;
        }
        putreg32(regval, RDA_GPIO_DIR);
    }
    return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rda_pinsel
 *
 * Description:
 *   Get the address of the PINSEL register corresponding to this port and
 *   pin number.
 *
 ****************************************************************************/

void rda_pinsel(unsigned int port, unsigned int pin, unsigned int value)
{
  uint32_t regaddr;
  uint32_t regval;
  unsigned int shift;

  /* Set the requested value in the PINMODE/PINSEL register */

  switch(port) {
    case 4:
      regaddr = g_pinsel[6];
      if(1 < pin) {
        shift = pin << 1;
        regval = getreg32(regaddr);
        regval &= ~(PINCONN_PINMODE_MASK << shift);
        regval |= (PINCONN_PINMODE_IO0 << shift);
        putreg32(regval, regaddr);
      }
      break;
    case 5:
      if(2 > pin) {
        regaddr = g_pinsel[6];
        shift = (pin << 1) + 20;
        regval = getreg32(regaddr);
        regval &= ~(PINCONN_PINMODE_MASK << shift);
        regval |= (PINCONN_PINMODE_IO0 << shift);
        putreg32(regval, regaddr);
      } else {
        regaddr = g_pinsel[7];
        shift = (pin << 1) - 4;
        regval = getreg32(regaddr);
        regval &= ~(PINCONN_PINMODE_MASK << shift);
        regval |= (PINCONN_PINMODE_IO0 << shift);
        putreg32(regval, regaddr);
      }
      break;
    default:
      break;
  }

  regaddr = g_pinsel[port];
  shift = 3 * pin;
  regval = getreg32(regaddr);
  regval &= ~(PINCONN_PINSEL_MASK << shift);
  regval |= (value << shift);
  putreg32(regval, regaddr);
}


/****************************************************************************
 * Name: rda_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

int rda_configgpio(rda_pinset_t cfgset)
{
  unsigned int port;
  unsigned int pin;
  int ret = -EINVAL;

  /* Verify that this hardware supports the select GPIO port */

  port = (cfgset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  if (port < (GPIO_NPORTS * 2))
    {
      /* Get the pin number and select the port configuration register for that pin */

      pin = (cfgset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

      /* Config pin function */

      rda_pinsel(port, pin, ((cfgset & GPIO_FUNC_MASK) >> GPIO_FUNC_SHIFT));

      /* gpio func */
      if(rda_isgpiofuncsel(cfgset, port, pin) != 0) {
        if(rda_configdirection(cfgset, port, pin) != 0) {
          if(GPIO_INTNONE != (cfgset & GPIO_EDGE_MASK)) {
            rda_configinterrupt(cfgset, port, pin);
          }
        } else {
          rda_gpiowrite(cfgset, ((cfgset & GPIO_VALUE) != GPIO_VALUE_ZERO));
        }
      }
      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: rda_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void rda_gpiowrite(rda_pinset_t pinset, bool value)
{
  unsigned int port;
  unsigned int pin;

  port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  if (port < (GPIO_NPORTS * 2))
    {
      uint32_t regval;
      unsigned int gpioidx;

      /* Get the pin number  */
      pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
      gpioidx = rda_gpioindex(port, pin);
      regval = getreg32(RDA_GPIO_DOUT) & ~(0x01UL << gpioidx);
      if(GPIO_VALUE_ZERO != (pinset & GPIO_VALUE)) {
        regval |= (0x01UL << gpioidx);
      }
      putreg32(regval, RDA_GPIO_DOUT);
    }
}

/****************************************************************************
 * Name: rda_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool rda_gpioread(rda_pinset_t pinset)
{
  unsigned int port;
  unsigned int pin;

  port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  if (port < (GPIO_NPORTS * 2))
    {
      unsigned int gpioidx;

      /* Get the pin number and return the input state of that pin */
      pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
      gpioidx = rda_gpioindex(port, pin);
      return ((getreg32(RDA_GPIO_DIN) & (0x01UL << gpioidx)) != 0x00UL);
    }

  return false;
}

//Need Modify
uint8_t rda_gpio_irqvector(uint32_t pincfg)
{
#if 0
	uint8_t pin  = (pincfg & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
	uint8_t port = (pincfg & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;

	if (port == GPIO_PORTG1 >> GPIO_PORT_SHIFT) {
		return IRQ_WEINT_GPG10 + pin;
	} else if (port == GPIO_PORTG2 >> GPIO_PORT_SHIFT) {
		return IRQ_WEINT_GPG20 + pin;
	} else if (port == GPIO_PORTA0 >> GPIO_PORT_SHIFT) {
		return IRQ_EINT0 + pin;
	}
#endif
	return 0;
}

void rda_gpio_clear_pending(uint32_t pincfg)
{
#if 0
	uint32_t regbase;
	uint8_t pin  = (pincfg & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
	uint8_t port = (pincfg & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;

	if (port == GPIO_PORTG1 >> GPIO_PORT_SHIFT) {
		regbase = S5J_GPIOINT_PEND_GPG1;
	} else if (port == GPIO_PORTG2 >> GPIO_PORT_SHIFT) {
		regbase = S5J_GPIOINT_PEND_GPG2;
	} else if (port  == GPIO_PORTA0 >> GPIO_PORT_SHIFT) {
		regbase = S5J_GPIOINT_PEND_GPA0;
	} else {
		return;
	}

	putreg32(1 << pin, regbase);
	#endif
}

