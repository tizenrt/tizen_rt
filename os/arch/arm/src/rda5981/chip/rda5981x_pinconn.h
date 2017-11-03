/************************************************************************************
 * arch/arm/src/rda5981x/chip/rda5981x_pinconn.h
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

#ifndef __ARCH_ARM_SRC_RDA5981X_CHIP_RDA5981X_PINCONN_H
#define __ARCH_ARM_SRC_RDA5981X_CHIP_RDA5981X_PINCONN_H

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

#define RDA_PINCONN_PINSEL0_OFFSET      0x0000 /* Pin function select register 0 */
#define RDA_PINCONN_PINSEL1_OFFSET      0x0004 /* Pin function select register 1 */
#define RDA_PINCONN_PINSEL2_OFFSET      0x0010 /* Pin function select register 2 */
#define RDA_PINCONN_PINSEL3_OFFSET      0x0014 /* Pin function select register 3 */
#define RDA_PINCONN_PINMODE0_OFFSET     0x0008 /* Pin tie mode select register 0 */
#define RDA_PINCONN_PINMODE1_OFFSET     0x000C /* Pin tie mode select register 1 */
#define RDA_PINCONN_PINMODE2_OFFSET     0x0018 /* Pin tie mode select register 2 */
#define RDA_PINCONN_PINMODE3_OFFSET     0x001C /* Pin tie mode select register 3 */

/* Register addresses ***************************************************************/

#define RDA_PINCONN_PINSEL0             (RDA_PINCONN_BASE + RDA_PINCONN_PINSEL0_OFFSET)
#define RDA_PINCONN_PINSEL1             (RDA_PINCONN_BASE + RDA_PINCONN_PINSEL1_OFFSET)
#define RDA_PINCONN_PINSEL2             (RDA_PINCONN_BASE + RDA_PINCONN_PINSEL2_OFFSET)
#define RDA_PINCONN_PINSEL3             (RDA_PINCONN_BASE + RDA_PINCONN_PINSEL3_OFFSET)
#define RDA_PINCONN_PINMODE0            (RDA_PINCONN_BASE + RDA_PINCONN_PINMODE0_OFFSET)
#define RDA_PINCONN_PINMODE1            (RDA_PINCONN_BASE + RDA_PINCONN_PINMODE1_OFFSET)
#define RDA_PINCONN_PINMODE2            (RDA_PINCONN_BASE + RDA_PINCONN_PINMODE2_OFFSET)
#define RDA_PINCONN_PINMODE3            (RDA_PINCONN_BASE + RDA_PINCONN_PINMODE3_OFFSET)

/* Register bit definitions *********************************************************/
#define PINCONN_PINSEL_ALT0             (0)
#define PINCONN_PINSEL_ALT1             (1)
#define PINCONN_PINSEL_ALT2             (2)
#define PINCONN_PINSEL_ALT3             (3)
#define PINCONN_PINSEL_ALT4             (4)
#define PINCONN_PINSEL_ALT5             (5)
#define PINCONN_PINSEL_ALT6             (6)
#define PINCONN_PINSEL_ALT7             (7)
#define PINCONN_PINSEL_MASK             (7)

/* Pin Function Select register 0 (PINSEL0: 0x40001044) */

#define PINCONN_PINSEL0_P0p0_SHIFT      (0)       /* Bits 0-2:   P0.26  000=RXD0 001=GPIO 010=Reserved 011=SPI_CS2 100=PWM0 101/110/111=Reserved */
#define PINCONN_PINSEL0_P0p0_MASK       (7 << PINCONN_PINSEL0_P0p0_SHIFT)
#define PINCONN_PINSEL0_P0p1_SHIFT      (3)       /* Bits 3-5:   P0.27  000=TXD0 001=GPIO 010=Reserved 011=SPI_CS3 100=PWM3 101/110/111=Reserved */
#define PINCONN_PINSEL0_P0p1_MASK       (7 << PINCONN_PINSEL0_P0p1_SHIFT)
#define PINCONN_PINSEL0_P0p2_SHIFT      (6)       /* Bits 6-8:   P0.14a 000=SDIO_CLK 001=GPIO 010=Reserved 011=SPI_CLK 100=SDMMC_CLK 101/110/111=Reserved */
#define PINCONN_PINSEL0_P0p2_MASK       (7 << PINCONN_PINSEL0_P0p2_SHIFT)
#define PINCONN_PINSEL0_P0p3_SHIFT      (9)       /* Bits 9-11:  P0.15a 000=SDIO_CMD 001=GPIO 010=Reserved 011=SPI_CS0 100=SDMMC_CMD 101/110/111=Reserved */
#define PINCONN_PINSEL0_P0p3_MASK       (7 << PINCONN_PINSEL0_P0p3_SHIFT)
#define PINCONN_PINSEL0_P0p4_SHIFT      (12)      /* Bits 12-14: P0.16a 000=SDIO_D0 001=GPIO 010=Reserved 011=SPI_S0 100=SDMMC_D0 101/110/111=Reserved */
#define PINCONN_PINSEL0_P0p4_MASK       (7 << PINCONN_PINSEL0_P0p4_SHIFT)
#define PINCONN_PINSEL0_P0p5_SHIFT      (15)      /* Bits 15-17: P0.17a 000=SDIO_D1 001=GPIO 010=Reserved 011=SPI_S1 100=SDMMC_D1 101/110/111=Reserved */
#define PINCONN_PINSEL0_P0p5_MASK       (7 << PINCONN_PINSEL0_P0p5_SHIFT)
#define PINCONN_PINSEL0_P0p6_SHIFT      (18)      /* Bits 18-20: P0.18a 000=SDIO_D2 001=GPIO 010=Reserved 011=SPI_S2 100=SDMMC_D2 101/110/111=Reserved */
#define PINCONN_PINSEL0_P0p6_MASK       (7 << PINCONN_PINSEL0_P0p6_SHIFT)
#define PINCONN_PINSEL0_P0p7_SHIFT      (21)      /* Bits 21-23: P0.19a 000=SDIO_D3 001=GPIO 010=Reserved 011=SPI_S3 100=SDMMC_D3 101/110/111=Reserved */
#define PINCONN_PINSEL0_P0p7_MASK       (7 << PINCONN_PINSEL0_P0p7_SHIFT)
#define PINCONN_PINSEL0_P0p8_SHIFT      (24)      /* Bits 24-26: P0.10  000=GPIO 001=SPI_CS1 010/011/100/101/110/111=Reserved */
#define PINCONN_PINSEL0_P0p8_MASK       (7 << PINCONN_PINSEL0_P0p8_SHIFT)
#define PINCONN_PINSEL0_P0p9_SHIFT      (27)      /* Bits 27-29: P0.11  000=GPIO 001/010/011/100/101/110/111=Reserved */
#define PINCONN_PINSEL0_P0p9_MASK       (7 << PINCONN_PINSEL0_P0p9_SHIFT)
/* Bits 30-31: Reserved */

/* Pin Function Select register 1 (PINSEL1: 0x40001048) */

#define PINCONN_PINSEL1_P0p0_SHIFT      (0)       /* Bits 0-2:   P0.0 000=GPIO 001=WIFIWU 010=Reserved 011=SDMMC_CMD 100=PWM2 101/110/111=Reserved */
#define PINCONN_PINSEL1_P0p0_MASK       (7 << PINCONN_PINSEL1_P0p0_SHIFT)
#define PINCONN_PINSEL1_P0p1_SHIFT      (3)       /* Bits 3-5:   P0.1 000=GPIO 001=NTRST 010=Reserved 011=I2S_O_SD 100=PWM1 101=RXD1 110/111=Reserved */
#define PINCONN_PINSEL1_P0p1_MASK       (7 << PINCONN_PINSEL1_P0p1_SHIFT)
#define PINCONN_PINSEL1_P0p2_SHIFT      (6)       /* Bits 6-8:   P0.2 000=GPIO 001=SDA0 010=Reserved 011=I2S_O_WS 100=PWM_LPG 101=TXD1 110/111=Reserved */
#define PINCONN_PINSEL1_P0p2_MASK       (7 << PINCONN_PINSEL1_P0p2_SHIFT)
#define PINCONN_PINSEL1_P0p3_SHIFT      (9)       /* Bits 9-11:  P0.3 000=GPIO 001=SCL0 010=Reserved 011=I2S_O_BCLK 100=PWM_PWT 101=SDMMC_D0 110/111=Reserved */
#define PINCONN_PINSEL1_P0p3_MASK       (7 << PINCONN_PINSEL1_P0p3_SHIFT)
#define PINCONN_PINSEL1_P0p4_SHIFT      (12)      /* Bits 12-14: P0.4 000=GPIO 001=TMS 010=Reserved 011=I2S_I_SD 100=SPI_CLK 101/110/111=Reserved */
#define PINCONN_PINSEL1_P0p4_MASK       (7 << PINCONN_PINSEL1_P0p4_SHIFT)
#define PINCONN_PINSEL1_P0p5_SHIFT      (15)      /* Bits 15-17: P0.5 000=GPIO 001=TCK 010=Reserved 011=I2S_I_WS 100=SPI_CS1 101/110/111=Reserved */
#define PINCONN_PINSEL1_P0p5_MASK       (7 << PINCONN_PINSEL1_P0p5_SHIFT)
#define PINCONN_PINSEL1_P0p6_SHIFT      (18)      /* Bits 18-20: P0.6 000=GPIO 001/010=Reserved 011=SPI_MOSI 100=Reserved 101=SPI_DATA 110=Reserved 111=SDMMC_D0 */
#define PINCONN_PINSEL1_P0p6_MASK       (7 << PINCONN_PINSEL1_P0p6_SHIFT)
#define PINCONN_PINSEL1_P0p7_SHIFT      (21)      /* Bits 21-23: P0.7 000=GPIO 001/010=Reserved 011=SPI_MISO 100=Reserved 101=SDMMC_D1 110/111=Reserved */
#define PINCONN_PINSEL1_P0p7_MASK       (7 << PINCONN_PINSEL1_P0p7_SHIFT)
#define PINCONN_PINSEL1_P0p8_SHIFT      (24)      /* Bits 24-26: P0.8 000=GPIO 001=TDO 010=Reserved 011=I2S_I_BCLK 100=PWM0 101/110/111=Reserved */
#define PINCONN_PINSEL1_P0p8_MASK       (7 << PINCONN_PINSEL1_P0p8_SHIFT)
#define PINCONN_PINSEL1_P0p9_SHIFT      (27)      /* Bits 27-29: P0.9 000=GPIO 001=TDI 010=Reserved 011=SDMMC_CLK 100/101/110/111=Reserved */
#define PINCONN_PINSEL1_P0p9_MASK       (7 << PINCONN_PINSEL1_P0p9_SHIFT)
/* Bits 30-31: Reserved */

/* Pin Function Select register 2 (PINSEL2: 0x40001054) */

#define PINCONN_PINSEL2_P0p0_SHIFT      (0)       /* Bits 0-2:   P0.12 000=SCL1 001=GPIO 010=Reserved 011=SDMMC_D2 100/101=Reserved 110=SPI_MOSI 111=Reserved */
#define PINCONN_PINSEL2_P0p0_MASK       (7 << PINCONN_PINSEL2_P0p0_SHIFT)
#define PINCONN_PINSEL2_P0p1_SHIFT      (3)       /* Bits 3-5:   P0.13 000=SDA1 001=GPIO 010=Reserved 011=SDMMC_D3 100=Reserved 101=PWM1 110=SPI_MISO 111=Reserved */
#define PINCONN_PINSEL2_P0p1_MASK       (7 << PINCONN_PINSEL2_P0p1_SHIFT)
#define PINCONN_PINSEL2_P0p2_SHIFT      (6)       /* Bits 6-8:   P0.14 000=GPIO 001/010/011/100/101/110/111=Reserved */
#define PINCONN_PINSEL2_P0p2_MASK       (7 << PINCONN_PINSEL2_P0p2_SHIFT)
#define PINCONN_PINSEL2_P0p3_SHIFT      (9)       /* Bits 9-11:  P0.15 000=GPIO 001/010/011/100/101/110/111=Reserved */
#define PINCONN_PINSEL2_P0p3_MASK       (7 << PINCONN_PINSEL2_P0p3_SHIFT)
#define PINCONN_PINSEL2_P0p4_SHIFT      (12)      /* Bits 12-14: P0.16 000=GPIO 001/010/011/100/101/110/111=Reserved */
#define PINCONN_PINSEL2_P0p4_MASK       (7 << PINCONN_PINSEL2_P0p4_SHIFT)
#define PINCONN_PINSEL2_P0p5_SHIFT      (15)      /* Bits 15-17: P0.17 000=GPIO 001/010/011/100/101/110/111=Reserved */
#define PINCONN_PINSEL2_P0p5_MASK       (7 << PINCONN_PINSEL2_P0p5_SHIFT)
#define PINCONN_PINSEL2_P0p6_SHIFT      (18)      /* Bits 18-20: P0.18 000=GPIO 001/010/011/100/101/110/111=Reserved */
#define PINCONN_PINSEL2_P0p6_MASK       (7 << PINCONN_PINSEL2_P0p6_SHIFT)
#define PINCONN_PINSEL2_P0p7_SHIFT      (21)      /* Bits 21-23: P0.19 000=GPIO 001/010/011/100/101/110/111=Reserved */
#define PINCONN_PINSEL2_P0p7_MASK       (7 << PINCONN_PINSEL2_P0p7_SHIFT)
#define PINCONN_PINSEL2_P0p8_SHIFT      (24)      /* Bits 24-26: P0.20 000=GPIO 001/010/011/100/101/110/111=Reserved */
#define PINCONN_PINSEL2_P0p8_MASK       (7 << PINCONN_PINSEL2_P0p8_SHIFT)
#define PINCONN_PINSEL2_P0p9_SHIFT      (27)      /* Bits 27-29: P0.21 000=GPIO 001/010/011/100/101/110/111=Reserved */
#define PINCONN_PINSEL2_P0p9_MASK       (7 << PINCONN_PINSEL2_P0p9_SHIFT)
/* Bits 30-31: Reserved */

/* Pin Function Select register 3 (PINSEL3: 0x40001058) */

#define PINCONN_PINSEL3_P0p0_SHIFT      (0)       /* Bits 0-2:   P0.22 000=GPIO 001=SPI_CLK 010=CTS1n 011=SDA0 100=PWM0 101/110/111=Reserved */
#define PINCONN_PINSEL3_P0p0_MASK       (7 << PINCONN_PINSEL3_P0p0_SHIFT)
#define PINCONN_PINSEL3_P0p1_SHIFT      (3)       /* Bits 3-5:   P0.23 000=GPIO 001=SPI_CS0 010=RTS1n 011=SCL0 100=PWM1 101/110/111=Reserved */
#define PINCONN_PINSEL3_P0p1_MASK       (7 << PINCONN_PINSEL3_P0p1_SHIFT)
#define PINCONN_PINSEL3_P0p2_SHIFT      (6)       /* Bits 6-8:   P0.24 000=GPIO 001=SPI_MOSI 010=RXD1 011=SPI_DATA 100=PWM2 101/110/111=Reserved */
#define PINCONN_PINSEL3_P0p2_MASK       (7 << PINCONN_PINSEL3_P0p2_SHIFT)
#define PINCONN_PINSEL3_P0p3_SHIFT      (9)       /* Bits 9-11:  P0.25 000=GPIO 001=SPI_MISO 010=TXD1 011=Reserved 100=PWM3 101/110/111=Reserved */
#define PINCONN_PINSEL3_P0p3_MASK       (7 << PINCONN_PINSEL3_P0p3_SHIFT)
/* Bits 12-31: Reserved */


/* Pin Mode select register 0 (PINMODE0: 0x4000104C) */

#define PINCONN_PINMODE_IO0             (0)       /* 00: pin has value from IO */
#define PINCONN_PINMODE_T0              (1)       /* 01: pin has tie to 0 */
#define PINCONN_PINMODE_T1              (2)       /* 10: pin has tie to 1 */
#define PINCONN_PINMODE_IO1             (3)       /* 11: pin has value from IO */
#define PINCONN_PINMODE_MASK            (3)

#define PINCONN_PINMODE0_P0p0_SHIFT     (0)       /* Bits 0-1: P0.26 mode control */
#define PINCONN_PINMODE0_P0p0_MASK      (3 << PINCONN_PINMODE0_P0p0_SHIFT)
#define PINCONN_PINMODE0_P0p1_SHIFT     (2)       /* Bits 2-3: P0.27 mode control */
#define PINCONN_PINMODE0_P0p1_MASK      (3 << PINCONN_PINMODE0_P0p1_SHIFT)
#define PINCONN_PINMODE0_P0p2_SHIFT     (4)       /* Bits 4-5: P0.14a mode control */
#define PINCONN_PINMODE0_P0p2_MASK      (3 << PINCONN_PINMODE0_P0p2_SHIFT)
#define PINCONN_PINMODE0_P0p3_SHIFT     (6)       /* Bits 6-7: P0.15a mode control */
#define PINCONN_PINMODE0_P0p3_MASK      (3 << PINCONN_PINMODE0_P0p3_SHIFT)
#define PINCONN_PINMODE0_P0p4_SHIFT     (8)       /* Bits 8-9: P0.16a mode control */
#define PINCONN_PINMODE0_P0p4_MASK      (3 << PINCONN_PINMODE0_P0p4_SHIFT)
#define PINCONN_PINMODE0_P0p5_SHIFT     (10)      /* Bits 10-11: P0.17a mode control */
#define PINCONN_PINMODE0_P0p5_MASK      (3 << PINCONN_PINMODE0_P0p5_SHIFT)
#define PINCONN_PINMODE0_P0p6_SHIFT     (12)      /* Bits 12-13: P0.18a mode control */
#define PINCONN_PINMODE0_P0p6_MASK      (3 << PINCONN_PINMODE0_P0p6_SHIFT)
#define PINCONN_PINMODE0_P0p7_SHIFT     (14)      /* Bits 14-15: P0.19a mode control */
#define PINCONN_PINMODE0_P0p7_MASK      (3 << PINCONN_PINMODE0_P0p7_SHIFT)
/* Bits 16-31: Reserved */

/* Pin Mode select register 1 (PINMODE1: 0x40001050) */

#define PINCONN_PINMODE1_P0p0_SHIFT     (0)       /* Bits 0-1: P0.0 mode control */
#define PINCONN_PINMODE1_P0p0_MASK      (3 << PINCONN_PINMODE1_P0p0_SHIFT)
#define PINCONN_PINMODE1_P0p1_SHIFT     (2)       /* Bits 2-3: P0.1 mode control */
#define PINCONN_PINMODE1_P0p1_MASK      (3 << PINCONN_PINMODE1_P0p1_SHIFT)
#define PINCONN_PINMODE1_P0p2_SHIFT     (4)       /* Bits 4-5: P0.2 mode control */
#define PINCONN_PINMODE1_P0p2_MASK      (3 << PINCONN_PINMODE1_P0p2_SHIFT)
#define PINCONN_PINMODE1_P0p3_SHIFT     (6)       /* Bits 6-7: P0.3 mode control */
#define PINCONN_PINMODE1_P0p3_MASK      (3 << PINCONN_PINMODE1_P0p3_SHIFT)
#define PINCONN_PINMODE1_P0p4_SHIFT     (8)       /* Bits 8-9: P0.4 mode control */
#define PINCONN_PINMODE1_P0p4_MASK      (3 << PINCONN_PINMODE1_P0p4_SHIFT)
#define PINCONN_PINMODE1_P0p5_SHIFT     (10)      /* Bits 10-11: P0.5 mode control */
#define PINCONN_PINMODE1_P0p5_MASK      (3 << PINCONN_PINMODE1_P0p5_SHIFT)
#define PINCONN_PINMODE1_P0p6_SHIFT     (12)      /* Bits 12-13: P0.6 mode control */
#define PINCONN_PINMODE1_P0p6_MASK      (3 << PINCONN_PINMODE1_P0p6_SHIFT)
#define PINCONN_PINMODE1_P0p7_SHIFT     (14)      /* Bits 14-15: P0.7 mode control */
#define PINCONN_PINMODE1_P0p7_MASK      (3 << PINCONN_PINMODE1_P0p7_SHIFT)
#define PINCONN_PINMODE1_P0p8_SHIFT     (16)      /* Bits 16-17: P0.8 mode control */
#define PINCONN_PINMODE1_P0p8_MASK      (3 << PINCONN_PINMODE1_P0p8_SHIFT)
#define PINCONN_PINMODE1_P0p9_SHIFT     (18)      /* Bits 18-19: P0.9 mode control */
#define PINCONN_PINMODE1_P0p9_MASK      (3 << PINCONN_PINMODE1_P0p9_SHIFT)
#define PINCONN_PINMODE1_P0p10_SHIFT    (20)      /* Bits 20-21: P0.10 mode control */
#define PINCONN_PINMODE1_P0p10_MASK     (3 << PINCONN_PINMODE1_P0p10_SHIFT)
#define PINCONN_PINMODE1_P0p11_SHIFT    (22)      /* Bits 22-23: P0.11 mode control */
#define PINCONN_PINMODE1_P0p11_MASK     (3 << PINCONN_PINMODE1_P0p11_SHIFT)
/* Bits 24-31: Reserved */

/* Pin Mode select register 2 (PINMODE2: 0x4000105C) */

#define PINCONN_PINMODE2_P0p0_SHIFT     (0)       /* Bits 0-1: P0.12 mode control */
#define PINCONN_PINMODE2_P0p0_MASK      (3 << PINCONN_PINMODE2_P0p0_SHIFT)
#define PINCONN_PINMODE2_P0p1_SHIFT     (2)       /* Bits 2-3: P0.13 mode control */
#define PINCONN_PINMODE2_P0p1_MASK      (3 << PINCONN_PINMODE2_P0p1_SHIFT)
#define PINCONN_PINMODE2_P0p2_SHIFT     (4)       /* Bits 4-5: P0.14 mode control */
#define PINCONN_PINMODE2_P0p2_MASK      (3 << PINCONN_PINMODE2_P0p2_SHIFT)
#define PINCONN_PINMODE2_P0p3_SHIFT     (6)       /* Bits 6-7: P0.15 mode control */
#define PINCONN_PINMODE2_P0p3_MASK      (3 << PINCONN_PINMODE2_P0p3_SHIFT)
#define PINCONN_PINMODE2_P0p4_SHIFT     (8)       /* Bits 8-9: P0.16 mode control */
#define PINCONN_PINMODE2_P0p4_MASK      (3 << PINCONN_PINMODE2_P0p4_SHIFT)
#define PINCONN_PINMODE2_P0p5_SHIFT     (10)      /* Bits 10-11: P0.17 mode control */
#define PINCONN_PINMODE2_P0p5_MASK      (3 << PINCONN_PINMODE2_P0p5_SHIFT)
#define PINCONN_PINMODE2_P0p6_SHIFT     (12)      /* Bits 12-13: P0.18 mode control */
#define PINCONN_PINMODE2_P0p6_MASK      (3 << PINCONN_PINMODE2_P0p6_SHIFT)
#define PINCONN_PINMODE2_P0p7_SHIFT     (14)      /* Bits 14-15: P0.19 mode control */
#define PINCONN_PINMODE2_P0p7_MASK      (3 << PINCONN_PINMODE2_P0p7_SHIFT)
#define PINCONN_PINMODE2_P0p8_SHIFT     (16)      /* Bits 16-17: P0.20 mode control */
#define PINCONN_PINMODE2_P0p8_MASK      (3 << PINCONN_PINMODE2_P0p8_SHIFT)
#define PINCONN_PINMODE2_P0p9_SHIFT     (18)      /* Bits 18-19: P0.21 mode control */
#define PINCONN_PINMODE2_P0p9_MASK      (3 << PINCONN_PINMODE2_P0p9_SHIFT)
#define PINCONN_PINMODE2_P0p10_SHIFT    (20)      /* Bits 20-21: P0.22 mode control */
#define PINCONN_PINMODE2_P0p10_MASK     (3 << PINCONN_PINMODE2_P0p10_SHIFT)
#define PINCONN_PINMODE2_P0p11_SHIFT    (22)      /* Bits 22-23: P0.23 mode control */
#define PINCONN_PINMODE2_P0p11_MASK     (3 << PINCONN_PINMODE2_P0p11_SHIFT)
/* Bits 24-31: Reserved */

/* Pin Mode select register 3 (PINMODE3: 0x40001060) */

#define PINCONN_PINMODE3_P0p0_SHIFT     (0)       /* Bits 0-1: P0.24 mode control */
#define PINCONN_PINMODE3_P0p0_MASK      (3 << PINCONN_PINMODE3_P0p0_SHIFT)
#define PINCONN_PINMODE3_P0p1_SHIFT     (2)       /* Bits 2-3: P0.25 mode control */
#define PINCONN_PINMODE3_P0p1_MASK      (3 << PINCONN_PINMODE3_P0p1_SHIFT)
/* Bits 4-31: Reserved */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_RDA5981X_CHIP_RDA5981X_PINCONN_H */
