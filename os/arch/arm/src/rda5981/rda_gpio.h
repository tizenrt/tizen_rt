/************************************************************************************
 * arch/arm/src/rda5981x/rda_gpio.h
 *
 *   Copyright (C) 2010, 2013 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_RDA5981X_RDA_GPIO_H
#define __ARCH_ARM_SRC_RDA5981X_RDA_GPIO_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <tinyara/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#endif

#include <arch/chip/chip.h>

#include "chip/rda5981x_gpio.h"
#include "chip/rda5981x_pinconn.h"
#include "chip/rda5981x_pinconfig.h"

/* Include the GPIO definitions for the selected RDA5981x family. */
//#define RDA5981x
#if defined(RDA5981x)
#  include "rda5981x_gpio.h"
#else
#  error "Unrecognized RDA5981x family"
#endif

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__
#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* These tables have global scope only because they are shared between rda5981x_gpio.c,
 * rda5981x_gpioint.c, and rda5981x_gpiodbg.c
 */

#ifdef CONFIG_RDA5981X_GPIOIRQ
EXTERN uint64_t g_intedge0;
EXTERN uint64_t g_intedge2;
#endif

#if 0
EXTERN const uint32_t g_fiobase[GPIO_NPORTS];
EXTERN const uint32_t g_intbase[GPIO_NPORTS];
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/************************************************************************************
 * Name: rda_gpioirqinitialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for GPIO pins.
 *
 ************************************************************************************/

#ifdef CONFIG_RDA5981X_GPIOIRQ
void rda_gpioirqinitialize(void);
#else
#  define rda_gpioirqinitialize()
#endif

/************************************************************************************
 * Name: rda_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 ************************************************************************************/

int rda_configgpio(rda_pinset_t cfgset);

/************************************************************************************
 * Name: rda_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ************************************************************************************/

void rda_gpiowrite(rda_pinset_t pinset, bool value);

/************************************************************************************
 * Name: rda_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ************************************************************************************/

bool rda_gpioread(rda_pinset_t pinset);

/************************************************************************************
 * Name: rda_gpioirqenable
 *
 * Description:
 *   Enable the interrupt for specified GPIO IRQ
 *
 ************************************************************************************/

#ifdef CONFIG_RDA5981X_GPIOIRQ
void rda_gpioirqenable(int irq);
#else
#  define rda_gpioirqenable(irq)
#endif

/************************************************************************************
 * Name: rda_gpioirqdisable
 *
 * Description:
 *   Disable the interrupt for specified GPIO IRQ
 *
 ************************************************************************************/

#ifdef CONFIG_RDA5981X_GPIOIRQ
void rda_gpioirqdisable(int irq);
#else
#  define rda_gpioirqdisable(irq)
#endif

/************************************************************************************
 * Function:  rda_dumpgpio
 *
 * Description:
 *   Dump all GPIO registers associated with the base address of the provided pinset.
 *
 ************************************************************************************/

#ifdef CONFIG_DEBUG_GPIO_INFO
int rda_dumpgpio(rda_pinset_t pinset, const char *msg);
#else
#  define rda_dumpgpio(p,m)
#endif

#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_RDA5981X_RDA_GPIO_H */
