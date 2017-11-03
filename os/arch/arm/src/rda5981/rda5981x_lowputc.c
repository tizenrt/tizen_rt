/****************************************************************************
 * arch/arm/src/rda5981x/rda5981x_lowputc.c
 *
 *   Copyright (C) 2010-2013 Gregory Nutt. All rights reserved.
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

#include <stdint.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip/rda5981x_syscon.h"
#include "chip/rda5981x_uart.h"

#include "rda5981x_gpio.h"
#include "rda5981x_lowputc.h"
#include "rda5981x_serial.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Select UART parameters for the selected console */

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
#  define CONSOLE_BASE     RDA_UART0_BASE
#  define CONSOLE_BAUD     CONFIG_UART0_BAUD
#  define CONSOLE_BITS     CONFIG_UART0_BITS
#  define CONSOLE_PARITY   CONFIG_UART0_PARITY
#  define CONSOLE_2STOP    CONFIG_UART0_2STOP
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#  define CONSOLE_BASE     RDA_UART1_BASE
#  define CONSOLE_BAUD     CONFIG_UART1_BAUD
#  define CONSOLE_BITS     CONFIG_UART1_BITS
#  define CONSOLE_PARITY   CONFIG_UART1_PARITY
#  define CONSOLE_2STOP    CONFIG_UART1_2STOP
#elif defined(HAVE_CONSOLE)
#  error "No CONFIG_UARTn_SERIAL_CONSOLE Setting"
#endif

/* Get word length setting for the console */

#if CONSOLE_BITS == 5
#  define CONSOLE_LCR_WLS UART_LCR_WLS_5BIT
#elif CONSOLE_BITS == 6
#  define CONSOLE_LCR_WLS UART_LCR_WLS_6BIT
#elif CONSOLE_BITS == 7
#  define CONSOLE_LCR_WLS UART_LCR_WLS_7BIT
#elif CONSOLE_BITS == 8
#  define CONSOLE_LCR_WLS UART_LCR_WLS_8BIT
#elif defined(HAVE_CONSOLE)
#  error "Invalid CONFIG_UARTn_BITS setting for console "
#endif

/* Get parity setting for the console */

#if CONSOLE_PARITY == 0
#  define CONSOLE_LCR_PAR 0
#elif CONSOLE_PARITY == 1
#  define CONSOLE_LCR_PAR (UART_LCR_PE|UART_LCR_PS_ODD)
#elif CONSOLE_PARITY == 2
#  define CONSOLE_LCR_PAR (UART_LCR_PE|UART_LCR_PS_EVEN)
#elif CONSOLE_PARITY == 3
#  define CONSOLE_LCR_PAR (UART_LCR_PE|UART_LCR_PS_STICK1)
#elif CONSOLE_PARITY == 4
#  define CONSOLE_LCR_PAR (UART_LCR_PE|UART_LCR_PS_STICK0)
#elif defined(HAVE_CONSOLE)
#    error "Invalid CONFIG_UARTn_PARITY setting for CONSOLE"
#endif

/* Get stop-bit setting for the console and UART0-3 */

#if CONSOLE_2STOP != 0
#  define CONSOLE_LCR_STOP UART_LCR_STOP
#else
#  define CONSOLE_LCR_STOP 0
#endif

/* LCR and FCR values for the console */

#define CONSOLE_LCR_VALUE (CONSOLE_LCR_WLS | CONSOLE_LCR_PAR | CONSOLE_LCR_STOP)
#define CONSOLE_FCR_VALUE (UART_FCR_RXTRIGGER_L0 | UART_FCR_TXRST |\
                           UART_FCR_RXRST | UART_FCR_FIFOEN)

/* Select a CCLK divider to produce the UART PCLK.  The strategy is to select the
 * smallest divisor that results in an solution within range of the 16-bit
 * DLM and DLL divisor:
 *
 *   BAUD = PCLK / (16 * DL), or
 *   DL   = PCLK / BAUD / 16
 *
 */

#ifdef RDA5981x
/* Use AHB CLK frequency */
#  define CONSOLE_NUMERATOR RDA_AHB_CLK_FREQUENCY
#endif /* RDA5981x */

/* Then this is the value to use for the DLM and DLL registers */

#define CONSOLE_DIV ((CONSOLE_NUMERATOR / CONSOLE_BAUD) >> 4)
#define CONSOLE_MOD ((CONSOLE_NUMERATOR / CONSOLE_BAUD) & 0x0F)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_lowputc
 *
 * Description:
 *   Output one byte on the serial console
 *
 ****************************************************************************/

void up_lowputc(char ch)
{
#if defined HAVE_UART && defined HAVE_CONSOLE
	/* Wait for the transmitter to be available */

	while ((getreg32(CONSOLE_BASE + RDA_UART_LSR_OFFSET) & UART_LSR_THRE) == 0);

	/* Send the character */

	putreg32((uint32_t)ch, CONSOLE_BASE + RDA_UART_THR_OFFSET);
#endif
}

/****************************************************************************
 * Name: rda_lowsetup
 *
 * Description:
 *   This performs basic initialization of the UART used for the serial
 *   console.  Its purpose is to get the console output availabe as soon
 *   as possible.
 *
 *   The UART0/1/2/3 peripherals are configured using the following registers:
 *   1. Power: In the PCONP register, set bits PCUART0/1/2/3.
 *      On reset, UART0 and UART 1 are enabled (PCUART0 = 1 and PCUART1 = 1)
 *      and UART2/3 are disabled (PCUART1 = 0 and PCUART3 = 0).
 *   2. Peripheral clock: In the PCLKSEL0 register, select PCLK_UART0 and
 *      PCLK_UART1; in the PCLKSEL1 register, select PCLK_UART2 and PCLK_UART3.
 *   3. Baud rate: In the LCR register, set bit DLAB = 1. This enables access
 *      to registers DLL and DLM for setting the baud rate. Also, if needed,
 *      set the fractional baud rate in the fractional divider.
 *   4. UART FIFO: Use bit FIFO enable (bit 0) in FCR register to
 *      enable FIFO.
 *   5. Pins: Select UART pins through the PINSEL registers and pin modes
 *      through the PINMODE registers. UART receive pins should not have
 *      pull-down resistors enabled.
 *   6. Interrupts: To enable UART interrupts set bit DLAB = 0 in the LCRF
 *      register. This enables access to IER. Interrupts are enabled
 *      in the NVIC using the appropriate Interrupt Set Enable register.
 *   7. DMA: UART transmit and receive functions can operate with the
 *      GPDMA controller.
 *
 ****************************************************************************/

void rda_lowsetup(void)
{
#ifdef HAVE_UART
	/* Step 1: Enable power for all console UART and disable power for
	 * other UARTs
	 */

	/* Step 2: Enable peripheral clocking for the console UART and disable
	 * clocking for all other UARTs
	 */

	/* Configure UART pins for the selected CONSOLE */

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
	rda_configgpio(GPIO_UART0_TXD);
	rda_configgpio(GPIO_UART0_RXD);
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
	rda_configgpio(GPIO_UART1_TXD);
	rda_configgpio(GPIO_UART1_RXD);
#if defined(CONFIG_UART1_IFLOWCONTROL) || defined(CONFIG_UART1_OFLOWCONTROL)
	rda_configgpio(GPIO_UART1_CTS);
	rda_configgpio(GPIO_UART1_RTS);
#endif
#endif

	/* Configure the console (only) */

#if defined(HAVE_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)

	/* Clear fifos */

	putreg32(UART_FCR_RXRST | UART_FCR_TXRST,
			 CONSOLE_BASE + RDA_UART_FCR_OFFSET);

	/* Set trigger */

	putreg32(UART_FCR_FIFOEN | UART_FCR_RXTRIGGER_L0,
			 CONSOLE_BASE + RDA_UART_FCR_OFFSET);

	/* Set up the LCR and set DLAB=1 */

	putreg32(CONSOLE_LCR_VALUE | UART_LCR_DLAB,
			 CONSOLE_BASE + RDA_UART_LCR_OFFSET);

	/* Set the BAUD divisor */

	putreg32(CONSOLE_DIV >> 8, CONSOLE_BASE + RDA_UART_DLM_OFFSET);
	putreg32(CONSOLE_DIV & 0xFF, CONSOLE_BASE + RDA_UART_DLL_OFFSET);
	putreg32(((CONSOLE_MOD - (CONSOLE_MOD >> 1)) << 4) | (CONSOLE_MOD >> 1), CONSOLE_BASE + RDA_UART_DL2_OFFSET);

	/* Clear DLAB */

	putreg32(CONSOLE_LCR_VALUE, CONSOLE_BASE + RDA_UART_LCR_OFFSET);

	/* Configure the FIFOs */

	putreg32(UART_FCR_RXTRIGGER_L0 | UART_FCR_TXRST | UART_FCR_RXRST |
			 UART_FCR_FIFOEN,
			 CONSOLE_BASE + RDA_UART_FCR_OFFSET);
#endif
#endif /* HAVE_UART */
}

