/************************************************************************************
 * arch/arm/src/rda5981x/chip/rda5981x_uart.h
 *
 *   Copyright (C) 2010, 2012-2013 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_RDA5981X_CHIP_RDA5981X_UART_H
#define __ARCH_ARM_SRC_RDA5981X_CHIP_RDA5981X_UART_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <tinyara/config.h>

#include "chip.h"
#include "chip/rda5981x_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/

#define RDA_UART_RBR_OFFSET         0x0000 /* (DLAB =0) Receiver Buffer Register (all) */
#define RDA_UART_THR_OFFSET         0x0000 /* (DLAB =0) Transmit Holding Register (all) */
#define RDA_UART_DLL_OFFSET         0x0000 /* (DLAB =1) Divisor Latch LSB (all) */
#define RDA_UART_DLM_OFFSET         0x0004 /* (DLAB =1) Divisor Latch MSB (all) */
#define RDA_UART_IER_OFFSET         0x0004 /* (DLAB =0) Interrupt Enable Register (all) */
#define RDA_UART_IIR_OFFSET         0x0008 /* Interrupt ID Register (all) */
#define RDA_UART_FCR_OFFSET         0x0008 /* FIFO Control Register (all) */
#define RDA_UART_LCR_OFFSET         0x000c /* Line Control Register (all) */
#define RDA_UART_MCR_OFFSET         0x0010 /* Modem Control Register (UART1 only) */
#define RDA_UART_LSR_OFFSET         0x0014 /* Line Status Register (all) */
#define RDA_UART_MSR_OFFSET         0x0018 /* Modem Status Register (UART1 only) */
#define RDA_UART_SCR_OFFSET         0x001c /* Scratch Pad Register (all) */
#define RDA_UART_FSR_OFFSET         0x0020 /* FIFO Status Register (all) */
#define RDA_UART_FRR_OFFSET         0x0024 /* FIFO TX/RX Trigger Register */
#define RDA_UART_DL2_OFFSET         0x0028 /* Baud Rate Adjust Register (all) */
#define RDA_UART_BAUD_OFFSET        0x003C /* Auto Baud Counter (all) */
#define RDA_UART_DL_SLOW_OFFSET     0x0040 /* Divisor Adjust When Slow CLK (all) */
#define RDA_UART_DL_FAST_OFFSET     0x0044 /* Divisor Adjust When Fast CLK (all) */

/* Register addresses ***************************************************************/

#define RDA_UART0_RBR               (RDA_UART0_BASE + RDA_UART_RBR_OFFSET)
#define RDA_UART0_THR               (RDA_UART0_BASE + RDA_UART_THR_OFFSET)
#define RDA_UART0_DLL               (RDA_UART0_BASE + RDA_UART_DLL_OFFSET)
#define RDA_UART0_DLM               (RDA_UART0_BASE + RDA_UART_DLM_OFFSET)
#define RDA_UART0_IER               (RDA_UART0_BASE + RDA_UART_IER_OFFSET)
#define RDA_UART0_IIR               (RDA_UART0_BASE + RDA_UART_IIR_OFFSET)
#define RDA_UART0_FCR               (RDA_UART0_BASE + RDA_UART_FCR_OFFSET)
#define RDA_UART0_LCR               (RDA_UART0_BASE + RDA_UART_LCR_OFFSET)
#define RDA_UART0_LSR               (RDA_UART0_BASE + RDA_UART_LSR_OFFSET)
#define RDA_UART0_SCR               (RDA_UART0_BASE + RDA_UART_SCR_OFFSET)
#define RDA_UART0_FSR               (RDA_UART0_BASE + RDA_UART_FSR_OFFSET)
#define RDA_UART0_FRR               (RDA_UART0_BASE + RDA_UART_FRR_OFFSET)
#define RDA_UART0_DL2               (RDA_UART0_BASE + RDA_UART_DL2_OFFSET)
#define RDA_UART0_BAUD              (RDA_UART0_BASE + RDA_UART_BAUD_OFFSET)
#define RDA_UART0_DL_SLOW           (RDA_UART0_BASE + RDA_UART_DL_SLOW_OFFSET)
#define RDA_UART0_DL_FAST           (RDA_UART0_BASE + RDA_UART_DL_FAST_OFFSET)

#define RDA_UART1_RBR               (RDA_UART1_BASE + RDA_UART_RBR_OFFSET)
#define RDA_UART1_THR               (RDA_UART1_BASE + RDA_UART_THR_OFFSET)
#define RDA_UART1_DLL               (RDA_UART1_BASE + RDA_UART_DLL_OFFSET)
#define RDA_UART1_DLM               (RDA_UART1_BASE + RDA_UART_DLM_OFFSET)
#define RDA_UART1_IER               (RDA_UART1_BASE + RDA_UART_IER_OFFSET)
#define RDA_UART1_IIR               (RDA_UART1_BASE + RDA_UART_IIR_OFFSET)
#define RDA_UART1_FCR               (RDA_UART1_BASE + RDA_UART_FCR_OFFSET)
#define RDA_UART1_LCR               (RDA_UART1_BASE + RDA_UART_LCR_OFFSET)
#define RDA_UART1_MCR               (RDA_UART1_BASE + RDA_UART_MCR_OFFSET)
#define RDA_UART1_LSR               (RDA_UART1_BASE + RDA_UART_LSR_OFFSET)
#define RDA_UART1_MSR               (RDA_UART1_BASE + RDA_UART_MSR_OFFSET)
#define RDA_UART1_SCR               (RDA_UART1_BASE + RDA_UART_SCR_OFFSET)
#define RDA_UART1_FSR               (RDA_UART1_BASE + RDA_UART_FSR_OFFSET)
#define RDA_UART1_FRR               (RDA_UART1_BASE + RDA_UART_FRR_OFFSET)
#define RDA_UART1_DL2               (RDA_UART1_BASE + RDA_UART_DL2_OFFSET)
#define RDA_UART1_BAUD              (RDA_UART1_BASE + RDA_UART_BAUD_OFFSET)
#define RDA_UART1_DL_SLOW           (RDA_UART1_BASE + RDA_UART_DL_SLOW_OFFSET)
#define RDA_UART1_DL_FAST           (RDA_UART1_BASE + RDA_UART_DL_FAST_OFFSET)

/* Register bit definitions *********************************************************/

/* RBR (DLAB =0) Receiver Buffer Register (all) */

#define UART_RBR_MASK                (0xFF)    /* Bits 0-7: Oldest received byte in RX FIFO */
                                               /* Bits 8-31: Reserved */

/* THR (DLAB =0) Transmit Holding Register (all) */

#define UART_THR_MASK                (0xFF)    /* Bits 0-7: Adds byte to TX FIFO */
                                               /* Bits 8-31: Reserved */

/* DLL (DLAB =1) Divisor Latch LSB (all) */

#define UART_DLL_MASK                (0xFF)    /* Bits 0-7: DLL */
                                               /* Bits 8-31: Reserved */

/* DLM (DLAB =1) Divisor Latch MSB (all) */

#define UART_DLM_MASK                (0xFF)    /* Bits 0-7: DLM */
                                               /* Bits 8-31: Reserved */

/* IER (DLAB =0) Interrupt Enable Register (all) */

#define UART_IER_RBRIE               (1 << 0)  /* Bit 0: RBR Interrupt Enable */
#define UART_IER_THREIE              (1 << 1)  /* Bit 1: THRE Interrupt Enable */
#define UART_IER_RLSIE               (1 << 2)  /* Bit 2: RX Line Status Interrupt Enable */
#define UART_IER_MSIE                (1 << 3)  /* Bit 3: Modem Status Interrupt Enable (UART1 only) */
                                               /* Bits 4-6: Reserved */
#define UART_IER_CTSIE               (1 << 7)  /* Bit 7: CTS transition interrupt (UART1 only) */
                                               /* Bits 8-31: Reserved */
#define UART_IER_ALLIE               (0x008F)

/* IIR Interrupt ID Register (all) */

#define UART_IIR_INTSTATUS           (1 << 0)  /* Bit 0:  Interrupt status (active low) */
#define UART_IIR_INTID_SHIFT         (1)       /* Bits 1-3: Interrupt identification */
#define UART_IIR_INTID_MASK          (7 << UART_IIR_INTID_SHIFT)
#  define UART_IIR_INTID_MSI         (0 << UART_IIR_INTID_SHIFT) /* Modem Status (UART1 only) */
#  define UART_IIR_INTID_THRE        (1 << UART_IIR_INTID_SHIFT) /* THRE Interrupt */
#  define UART_IIR_INTID_RDA         (2 << UART_IIR_INTID_SHIFT) /* 2a - Receive Data Available (RDA) */
#  define UART_IIR_INTID_RLS         (3 << UART_IIR_INTID_SHIFT) /* 1 - Receive Line Status (RLS) */
#  define UART_IIR_INTID_CTI         (6 << UART_IIR_INTID_SHIFT) /* 2b - Character Time-out Indicator (CTI) */
                                               /* Bits 4-5: Reserved */
#define UART_IIR_FIFOEN_SHIFT        (6)       /* Bits 6-7: Copies of FCR bit 0 */
#define UART_IIR_FIFOEN_MASK         (3 << UART_IIR_FIFOEN_SHIFT)
                                               /* Bits 8-31: Reserved */
/* FCR FIFO Control Register (all) */

#define UART_FCR_FIFOEN              (1 << 0)  /* Bit 0:  Enable FIFOs */
#define UART_FCR_RXRST               (1 << 1)  /* Bit 1:  RX FIFO Reset */
#define UART_FCR_TXRST               (1 << 2)  /* Bit 2:  TX FIFO Reset */
#define UART_FCR_DMAMODE             (1 << 3)  /* Bit 3:  DMA Mode Select */
#define UART_FCR_TXTRIGGER_SHIFT     (4)       /* Bits 4-5: TX Trigger Level */
#define UART_FCR_TXTRIGGER_MASK      (3 << UART_FCR_TXTRIGGER_SHIFT)
#  define UART_FCR_TXTRIGGER_L0      (0 << UART_FCR_TXTRIGGER_SHIFT) /* Trigger level 0 (FIFO is empty) */
#  define UART_FCR_TXTRIGGER_L1      (1 << UART_FCR_TXTRIGGER_SHIFT) /* Trigger level 1 (2 characters in the FIFO) */
#  define UART_FCR_TXTRIGGER_L2      (2 << UART_FCR_TXTRIGGER_SHIFT) /* Trigger level 2 (FIFO is quarter-full) */
#  define UART_FCR_TXTRIGGER_L3      (3 << UART_FCR_TXTRIGGER_SHIFT) /* Trigger level 3 (FIFO is half-full) */
#define UART_FCR_RXTRIGGER_SHIFT     (6)       /* Bits 6-7: RX Trigger Level */
#define UART_FCR_RXTRIGGER_MASK      (3 << UART_FCR_RXTRIGGER_SHIFT)
#  define UART_FCR_RXTRIGGER_L0      (0 << UART_FCR_RXTRIGGER_SHIFT) /* Trigger level 0 (1 character in the FIFO) */
#  define UART_FCR_RXTRIGGER_L1      (1 << UART_FCR_RXTRIGGER_SHIFT) /* Trigger level 1 (FIFO is quarter-full) */
#  define UART_FCR_RXTRIGGER_L2      (2 << UART_FCR_RXTRIGGER_SHIFT) /* Trigger level 2 (FIFO is half-full) */
#  define UART_FCR_RXTRIGGER_L3      (3 << UART_FCR_RXTRIGGER_SHIFT) /* Trigger level 3 (FIFO is almost-full) */
                                               /* Bits 8-31: Reserved */
/* LCR Line Control Register (all) */

#define UART_LCR_WLS_SHIFT           (0)       /* Bit 0-1: Word Length Select */
#define UART_LCR_WLS_MASK            (3 << UART_LCR_WLS_SHIFT)
#  define UART_LCR_WLS_5BIT          (0 << UART_LCR_WLS_SHIFT)
#  define UART_LCR_WLS_6BIT          (1 << UART_LCR_WLS_SHIFT)
#  define UART_LCR_WLS_7BIT          (2 << UART_LCR_WLS_SHIFT)
#  define UART_LCR_WLS_8BIT          (3 << UART_LCR_WLS_SHIFT)
#define UART_LCR_STOP                (1 << 2)  /* Bit 2:  Stop Bit Select */
#define UART_LCR_PE                  (1 << 3)  /* Bit 3:  Parity Enable */
#define UART_LCR_PS_SHIFT            (4)       /* Bits 4-5: Parity Select */
#define UART_LCR_PS_MASK             (3 << UART_LCR_PS_SHIFT)
#  define UART_LCR_PS_ODD            (0 << UART_LCR_PS_SHIFT) /* Odd parity */
#  define UART_LCR_PS_EVEN           (1 << UART_LCR_PS_SHIFT) /* Even Parity */
#  define UART_LCR_PS_STICK1         (2 << UART_LCR_PS_SHIFT) /* Forced "1" stick parity */
#  define UART_LCR_PS_STICK0         (3 << UART_LCR_PS_SHIFT) /* Forced "0" stick parity */
#define UART_LCR_BRK                 (1 << 6)  /* Bit 6: Break Control */
#define UART_LCR_DLAB                (1 << 7)  /* Bit 7: Divisor Latch Access Bit (DLAB) */
                                               /* Bits 8-31: Reserved */
/* MCR Modem Control Register (UART1 only) */

#define UART_MCR_DTR                 (1 << 0)  /* Bit 0:  DTR Control Source for DTR output */
#define UART_MCR_RTS                 (1 << 1)  /* Bit 1:  Control Source for  RTS output */
                                               /* Bits 2-3: Reserved */
#define UART_MCR_LPBK                (1 << 4)  /* Bit 4:  Loopback Mode Select */
#define UART_MCR_AFCE                (1 << 5)  /* Bit 5:  Enable auto flow control */
#define UART_MCR_SIRE                (1 << 6)  /* Bit 6:  Enable IrDA mode */
#define UART_MCR_ABDE                (1 << 7)  /* Bit 7:  Enable auto baud detection */
#define UART_MCR_CSEL                (1 << 8)  /* Bit 8:  Select UART clock, 0:24M, 1:48M */
                                               /* Bits 9-31: Reserved */
/* LSR Line Status Register (all) */

#define UART_LSR_RDR                 (1 << 0)  /* Bit 0:  Receiver Data Ready */
#define UART_LSR_OE                  (1 << 1)  /* Bit 1:  Overrun Error */
#define UART_LSR_PE                  (1 << 2)  /* Bit 2:  Parity Error */
#define UART_LSR_FE                  (1 << 3)  /* Bit 3:  Framing Error */
#define UART_LSR_BI                  (1 << 4)  /* Bit 4:  Break Interrupt */
#define UART_LSR_THRE                (1 << 5)  /* Bit 5:  Transmitter Holding Register Empty */
#define UART_LSR_TEMT                (1 << 6)  /* Bit 6:  Transmitter Empty */
#define UART_LSR_RXFE                (1 << 7)  /* Bit 7:  Error in RX FIFO (RXFE) */
                                               /* Bits 8-31: Reserved */
/* MSR Modem Status Register (UART1 only) */

#define UART_MSR_DELTACTS            (1 << 0)  /* Bit 0:  CTS state change */
#define UART_MSR_DELTADSR            (1 << 1)  /* Bit 1:  DSR state change */
#define UART_MSR_RIEDGE              (1 << 2)  /* Bit 2:  RI ow to high transition */
#define UART_MSR_DELTADCD            (1 << 3)  /* Bit 3:  DCD state change */
#define UART_MSR_CTS                 (1 << 4)  /* Bit 4:  CTS State */
#define UART_MSR_DSR                 (1 << 5)  /* Bit 5:  DSR State */
#define UART_MSR_RI                  (1 << 6)  /* Bit 6:  Ring Indicator State */
#define UART_MSR_DCD                 (1 << 7)  /* Bit 7:  Data Carrier Detect State */
                                               /* Bits 8-31: Reserved */
/* SCR Scratch Pad Register (all) */

#define UART_SCR_MASK                (0xFF)    /* Bits 0-7: SCR data */
                                               /* Bits 8-31: Reserved */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_RDA5981X_CHIP_RDA5981X_UART_H */
