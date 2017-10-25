/************************************************************************************
 * arch/arm/src/rda5981x/rda5981x_gpio.h
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

#ifndef __ARCH_ARM_SRC_RDA5981X_RDA5981X_GPIO_H
#define __ARCH_ARM_SRC_RDA5981X_RDA5981X_GPIO_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Bit-encoded input to rda_configgpio() ******************************************/

/* Encoding: FFFx EEVD PPPN NNNN
 *
 *   Pin Function:   FFF(0-7)
 *   Interrupt Edge: EE (input pins)
 *   Initial value:  V (output pins)
 *   Pin Direction:  D (input/output)
 *   Port number:    PPP (0-4)
 *   Pin number:     NNNNN (0-9)
 */

/* Pin Function:   FFF */

#define GPIO_FUNC_SHIFT         (13)        /* Bits 13-15: Pin alternate functions */
#define GPIO_FUNC_MASK          (7 << GPIO_FUNC_SHIFT)
#  define GPIO_ALT0             (0 << GPIO_FUNC_SHIFT) /* 000 Alternate function 0 */
#  define GPIO_ALT1             (1 << GPIO_FUNC_SHIFT) /* 001 Alternate function 1 */
#  define GPIO_ALT2             (2 << GPIO_FUNC_SHIFT) /* 010 Alternate function 2 */
#  define GPIO_ALT3             (3 << GPIO_FUNC_SHIFT) /* 011 Alternate function 3 */
#  define GPIO_ALT4             (4 << GPIO_FUNC_SHIFT) /* 100 Alternate function 4 */
#  define GPIO_ALT5             (5 << GPIO_FUNC_SHIFT) /* 101 Alternate function 5 */
#  define GPIO_ALT6             (6 << GPIO_FUNC_SHIFT) /* 110 Alternate function 6 */
#  define GPIO_ALT7             (7 << GPIO_FUNC_SHIFT) /* 111 Alternate function 7 */

#if 0
#define GPIO_ISGPIO(ps)         ((uint16_t(ps) & GPIO_FUNC_MASK) <= GPIO_OUTPUT)
#define GPIO_ISALT(ps)          ((uint16_t(ps) & GPIO_FUNC_MASK) > GPIO_OUTPUT)
#define GPIO_ISINPUT(ps)        (((ps) & GPIO_FUNC_MASK) == GPIO_INPUT)
#define GPIO_ISOUTPUT(ps)       (((ps) & GPIO_FUNC_MASK) == GPIO_OUTPUT)
#define GPIO_ISINORINT(ps)      (((ps) & GPIO_INOUT_MASK) == 0)
#define GPIO_ISOUTORALT(ps)     (((ps) & GPIO_INOUT_MASK) != 0)
#define GPIO_ISINTERRUPT(ps)    (GPIO_ISOUTPUT(ps) && !GPIO_ISINPUT(ps))
#define GPIO_ISFE(ps)           (((ps) & GPIO_FE_MASK) != 0)
#define GPIO_ISRE(ps)           (((ps) & GPIO_RE_MASK) != 0)
#endif

/* Interrupt Edge: EE */

#define GPIO_EDGE_SHIFT         (10)        /* Bits 10-11: Interrupt edge bits */
#define GPIO_EDGE_MASK          (3 << GPIO_EDGE_SHIFT)
#  define GPIO_INTNONE          (0 << GPIO_EDGE_SHIFT) /* 000 GPIO interrupt disable */
#  define GPIO_INTFE            (1 << GPIO_EDGE_SHIFT) /* 001 GPIO interrupt falling edge */
#  define GPIO_INTRE            (2 << GPIO_EDGE_SHIFT) /* 010 GPIO interrupt rising edge */
#  define GPIO_INTBOTH          (3 << GPIO_EDGE_SHIFT) /* 011 GPIO interrupt both edges */

/* Initial value:  V */

#define GPIO_VALUE              (1 << 9)    /* Bit 9:  Initial GPIO output value */
#define GPIO_VALUE_ONE          GPIO_VALUE
#define GPIO_VALUE_ZERO         (0)

/* Pin direction:  D */

#define GPIO_DIR                (1 << 8)    /* Bit 8:  GPIO input/output direction */
#define GPIO_DIR_INPUT          GPIO_DIR
#define GPIO_DIR_OUTPUT         (0)

/* Port number:    PPP (0-4) */

#define GPIO_PORT_SHIFT         (5)         /* Bit 5-7:  Port number */
#define GPIO_PORT_MASK          (7 << GPIO_PORT_SHIFT)
#  define GPIO_PORTA            (0 << GPIO_PORT_SHIFT)
#  define GPIO_PORTB            (1 << GPIO_PORT_SHIFT)
#  define GPIO_PORTC            (4 << GPIO_PORT_SHIFT)
#  define GPIO_PORTD            (5 << GPIO_PORT_SHIFT)

#define GPIO_NPORTS             4

/* Pin number:     NNNNN (0-9) */

#define GPIO_PIN_SHIFT          0           /* Bits 0-4: Pin number: 0-9 */
#define GPIO_PIN_MASK           (31 << GPIO_PIN_SHIFT)
#  define GPIO_PIN0             (0  << GPIO_PIN_SHIFT)
#  define GPIO_PIN1             (1  << GPIO_PIN_SHIFT)
#  define GPIO_PIN2             (2  << GPIO_PIN_SHIFT)
#  define GPIO_PIN3             (3  << GPIO_PIN_SHIFT)
#  define GPIO_PIN4             (4  << GPIO_PIN_SHIFT)
#  define GPIO_PIN5             (5  << GPIO_PIN_SHIFT)
#  define GPIO_PIN6             (6  << GPIO_PIN_SHIFT)
#  define GPIO_PIN7             (7  << GPIO_PIN_SHIFT)
#  define GPIO_PIN8             (8  << GPIO_PIN_SHIFT)
#  define GPIO_PIN9             (9  << GPIO_PIN_SHIFT)
#  define GPIO_PIN10            (10  << GPIO_PIN_SHIFT)
#  define GPIO_PIN11            (11  << GPIO_PIN_SHIFT)
#  define GPIO_PIN12            (12  << GPIO_PIN_SHIFT)
#  define GPIO_PIN13            (13  << GPIO_PIN_SHIFT)
#  define GPIO_PIN14            (14  << GPIO_PIN_SHIFT)
#  define GPIO_PIN15            (15  << GPIO_PIN_SHIFT)
#  define GPIO_PIN16            (16  << GPIO_PIN_SHIFT)
#  define GPIO_PIN17            (17  << GPIO_PIN_SHIFT)
#  define GPIO_PIN18            (18  << GPIO_PIN_SHIFT)
#  define GPIO_PIN19            (19  << GPIO_PIN_SHIFT)
#  define GPIO_PIN20            (20  << GPIO_PIN_SHIFT)
#  define GPIO_PIN21            (21  << GPIO_PIN_SHIFT)
#  define GPIO_PIN22            (22  << GPIO_PIN_SHIFT)
#  define GPIO_PIN23            (23  << GPIO_PIN_SHIFT)
#  define GPIO_PIN24            (24  << GPIO_PIN_SHIFT)
#  define GPIO_PIN25            (25  << GPIO_PIN_SHIFT)
#  define GPIO_PIN26            (26  << GPIO_PIN_SHIFT)
#  define GPIO_PIN27            (27  << GPIO_PIN_SHIFT)

#define GPIO_NPINS              28

/************************************************************************************
 * Public Types
 ************************************************************************************/

typedef uint16_t rda_pinset_t;

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

EXTERN const uint32_t g_pinsel[GPIO_NPORTS * 2];
EXTERN const uint32_t g_pinmap[GPIO_NPINS];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_RDA5981X_RDA5981X_GPIO_H */
