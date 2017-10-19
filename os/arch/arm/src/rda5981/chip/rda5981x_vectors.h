/********************************************************************************
 * arch/arm/src/rda5981x/chip/rda5981x_vectors.h
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
 ********************************************************************************/

/********************************************************************************
 * Included Files
 ********************************************************************************/

/********************************************************************************
 * Pre-processor Definitions
 ********************************************************************************/
/* This file is included by rda5981x_vectors.S.  It provides the macro VECTOR that
 * supplies each RDA5981x vector in terms of a (lower-case) ISR label and an
 * (upper-case) IRQ number as defined in arch/arm/include/rda5981x/rda5981x_irq.h.
 * rda5981x_vectors.S will defined the VECTOR in different ways in order to generate
 * the interrupt vectors and handlers in their final form.
 */

/* If the common ARMv7-M vector handling is used, then all it needs is the following
 * definition that provides the number of supported vectors.
 */

#ifdef CONFIG_ARMV7M_CMNVECTOR

/* Reserve 35 interrupt table entries for I/O interrupts. */

#  define ARMV7M_PERIPHERAL_INTERRUPTS 35

#else

VECTOR(rda_spiflash, RDA_IRQ_SPIFLASH) /* Vector 16+0:  SPI Flash Interrupt    */
VECTOR(rda_pta     , RDA_IRQ_PTA     ) /* Vector 16+1:  PTA Interrupt          */
VECTOR(rda_sdio    , RDA_IRQ_SDIO    ) /* Vector 16+2:  SDIO Interrupt         */
VECTOR(rda_usbdma  , RDA_IRQ_USBDMA  ) /* Vector 16+3:  USBDMA Interrupt       */
VECTOR(rda_usb     , RDA_IRQ_USB     ) /* Vector 16+4:  USB Interrupt          */
VECTOR(rda_gpio    , RDA_IRQ_GPIO    ) /* Vector 16+5:  GPIO Interrupt         */
VECTOR(rda_timer   , RDA_IRQ_TIMER   ) /* Vector 16+6:  Timer Interrupt        */
VECTOR(rda_uart0   , RDA_IRQ_UART0   ) /* Vector 16+7:  UART0 Interrupt        */
VECTOR(rda_machw   , RDA_IRQ_MACHW   ) /* Vector 16+8:  MAC Hardware Interrupt */
VECTOR(rda_uart1   , RDA_IRQ_UART1   ) /* Vector 16+9:  UART1 Interrupt        */
VECTOR(rda_ahbdma  , RDA_IRQ_AHBDMA  ) /* Vector 16+10: AHBDMA Interrupt       */
VECTOR(rda_psram   , RDA_IRQ_PSRAM   ) /* Vector 16+11: PSRAM Interrupt        */
VECTOR(rda_sdmmc   , RDA_IRQ_SDMMC   ) /* Vector 16+12: SDMMC Interrupt        */
VECTOR(rda_exif    , RDA_IRQ_EXIF    ) /* Vector 16+13: EXIF Interrupt         */
VECTOR(rda_i2c     , RDA_IRQ_I2C     ) /* Vector 16+14: I2C Interrupt          */

#endif

/********************************************************************************
 * Public Types
 ********************************************************************************/

/********************************************************************************
 * Public Data
 ********************************************************************************/

/********************************************************************************
 * Public Function Prototypes
 ********************************************************************************/
