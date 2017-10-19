/****************************************************************************
 * arch/rda5981x/rda5981x_irq.h
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

/* This file should never be included directed but, rather, only indirectly
 * through nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_RDA5981X_RDA5981X_IRQ_H
#define __ARCH_ARM_INCLUDE_RDA5981X_RDA5981X_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* IRQ numbers.  The IRQ number corresponds vector number and hence map
 * directly to bits in the NVIC.  This does, however, waste several words of
 * memory in the IRQ to handle mapping tables.
 */

/* External interrupts (vectors >= 16) */

#define RDA_IRQ_SPIFLASH        (RDA_IRQ_EXTINT +  0) /* SPI Flash Interrupt    */
#define RDA_IRQ_PTA             (RDA_IRQ_EXTINT +  1) /* PTA Interrupt          */
#define RDA_IRQ_SDIO            (RDA_IRQ_EXTINT +  2) /* SDIO Interrupt         */
#define RDA_IRQ_USBDMA          (RDA_IRQ_EXTINT +  3) /* USBDMA Interrupt       */
#define RDA_IRQ_USB             (RDA_IRQ_EXTINT +  4) /* USB Interrupt          */
#define RDA_IRQ_GPIO            (RDA_IRQ_EXTINT +  5) /* GPIO Interrupt         */
#define RDA_IRQ_TIMER           (RDA_IRQ_EXTINT +  6) /* Timer Interrupt        */
#define RDA_IRQ_UART0           (RDA_IRQ_EXTINT +  7) /* UART0 Interrupt        */
#define RDA_IRQ_MACHW           (RDA_IRQ_EXTINT +  8) /* MAC Hardware Interrupt */
#define RDA_IRQ_UART1           (RDA_IRQ_EXTINT +  9) /* UART1 Interrupt        */
#define RDA_IRQ_AHBDMA          (RDA_IRQ_EXTINT + 10) /* AHBDMA Interrupt       */
#define RDA_IRQ_PSRAM           (RDA_IRQ_EXTINT + 11) /* PSRAM Interrupt        */
#define RDA_IRQ_SDMMC           (RDA_IRQ_EXTINT + 12) /* SDMMC Interrupt        */
#define RDA_IRQ_EXIF            (RDA_IRQ_EXTINT + 13) /* EXIF Interrupt         */
#define RDA_IRQ_I2C             (RDA_IRQ_EXTINT + 14) /* I2C Interrupt          */
#define RDA_IRQ_NEXTINT         (15)
#define RDA_IRQ_NIRQS           (RDA_IRQ_EXTINT + RDA_IRQ_NEXTINT)

/* Total number of IRQ numbers */

#define NR_VECTORS              RDA_IRQ_NIRQS
#define NR_IRQS                 (RDA_IRQ_EXTINT + RDA_IRQ_NEXTINT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_INCLUDE_RDA5981X_RDA5981X_IRQ_H */

