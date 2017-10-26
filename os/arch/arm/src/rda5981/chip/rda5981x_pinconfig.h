/************************************************************************************
 * arch/arm/src/rda5981x/chip/rda5981x_pinconfig.h
 *
 *   Copyright (C) 2009-2011, 2013 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_RDA5981X_CHIP_RDA5981X_PINCONFIG_H
#define __ARCH_ARM_SRC_RDA5981X_CHIP_RDA5981X_PINCONFIG_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <tinyara/config.h>
#include "../rda5981x_gpio.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* GPIO pin definitions *************************************************************/
/* NOTE that functions have a alternate pins that can be selected.  These alternates
 * are identified with a numerica suffix like _1, _2, or _3.  Your board.h file
 * should select the correct alternative for your board by including definitions
 * such as:
 *
 * #define GPIO_UART1_RXD GPIO_UART1_RXD_1
 *
 * (without the suffix)
 */

#define GPIO_UART0_RXD     (GPIO_ALT0 | GPIO_PORTA | GPIO_PIN0)
#define GPIO_UART0_TXD     (GPIO_ALT0 | GPIO_PORTA | GPIO_PIN1)
#define GPIO_UART1_RXD_1   (GPIO_ALT5 | GPIO_PORTB | GPIO_PIN1)
#define GPIO_UART1_TXD_1   (GPIO_ALT5 | GPIO_PORTB | GPIO_PIN2)
#define GPIO_UART1_CTS     (GPIO_ALT2 | GPIO_PORTD | GPIO_PIN0)
#define GPIO_UART1_RTS     (GPIO_ALT2 | GPIO_PORTD | GPIO_PIN1)
#define GPIO_UART1_RXD_2   (GPIO_ALT2 | GPIO_PORTD | GPIO_PIN2)
#define GPIO_UART1_TXD_2   (GPIO_ALT2 | GPIO_PORTD | GPIO_PIN3)

/************************************************************************************
 * Public Types
 ************************************************************************************/
/* SPI0 */
#define GPIO_SPI0_CLK      (GPIO_PORTB | GPIO_ALT4 | GPIO_PIN4)
#define GPIO_SPI0_CS       (GPIO_PORTB | GPIO_ALT4 | GPIO_PIN5)
#define GPIO_SPI0_MISO     (GPIO_PORTC | GPIO_ALT6 | GPIO_PIN1)
#define GPIO_SPI0_MOSI     (GPIO_PORTC | GPIO_ALT6 | GPIO_PIN0)

/*I2C*/
#define GPIO_I2C0_SCL     (GPIO_ALT1 | GPIO_PORTB | GPIO_PIN2)
#define GPIO_I2C0_SDA    (GPIO_ALT1 | GPIO_PORTB | GPIO_PIN3)

/*ADC*/
#define GPIO_RDA_ADC   (GPIO_ALT0 | GPIO_PORTB | GPIO_PIN6)

/*PWM*/
#define GPIO_PWM_TOUT0  (GPIO_ALT4 | GPIO_PORTB | GPIO_PIN8)
#define GPIO_PWM_TOUT1  (GPIO_ALT5 | GPIO_PORTC | GPIO_PIN13)
#define GPIO_PWM_TOUT2  (GPIO_ALT4 | GPIO_PORTB | GPIO_PIN0)
#define GPIO_PWM_TOUT3  (GPIO_ALT4 | GPIO_PORTA | GPIO_PIN27) 

/*I2S*/

#define I2S_TX_BCLK  (GPIO_ALT3 | GPIO_PORTB | GPIO_PIN3)
#define I2S_TX_WS (GPIO_ALT3 | GPIO_PORTB | GPIO_PIN2)
#define I2S_TX_SD (GPIO_ALT3 | GPIO_PORTB | GPIO_PIN1)
#define I2S_RX_SD (GPIO_ALT3 | GPIO_PORTB | GPIO_PIN4)
#define I2S_MCLK (GPIO_ALT1 | GPIO_PORTB | GPIO_PIN5)

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_RDA5981X_CHIP_RDA5981X_PINCONFIG_H */
