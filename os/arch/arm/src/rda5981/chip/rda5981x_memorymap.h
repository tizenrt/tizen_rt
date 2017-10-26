/************************************************************************************
 * arch/arm/src/rda5981x/rda5981x_memorymap.h
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

#ifndef __ARCH_ARM_SRC_RDA5981X_RDA5981X_MEMORYMAP_H
#define __ARCH_ARM_SRC_RDA5981X_RDA5981X_MEMORYMAP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <tinyara/config.h>

//#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Memory Map ***********************************************************************/

#define RDA_ROM_BASE        (0x00000000UL)
#define RDA_IRAM_BASE       (0x00100000UL)
#define RDA_DRAM_BASE       (0x00180000UL)
#define RDA_PSRAM_BASE      (0x10000000UL)
#define RDA_FLASH_BASE      (0x14000000UL)
#define RDA_ICACHE_BASE     (0x18000000UL)
#define RDA_PER_BASE        (0x40000000UL)
#define RDA_AHB0_BASE       (0x40000000UL)
#define RDA_APB_BASE        (RDA_AHB0_BASE)
#define RDA_AHB1_BASE       (0x40100000UL)
#define RDA_PERBTBND_BASE   (0x42000000UL)
#define RDA_CM4_BASE        (0xE0000000UL)

/* APB peripherals ******************************************************************/
#define RDA_SCU_BASE        (RDA_APB_BASE  + 0x00000)
#define RDA_GPIO_BASE       (RDA_APB_BASE  + 0x01000)
#define RDA_TIM0_BASE       (RDA_APB_BASE  + 0x02000)
#define RDA_TIM1_BASE       (RDA_APB_BASE  + 0x02008)
#define RDA_TIMINTST_BASE   (RDA_APB_BASE  + 0x02010)
#define RDA_I2C0_BASE       (RDA_APB_BASE  + 0x03000)

/* AHB0 peripherals *****************************************************************/
#define RDA_PWM_BASE        (RDA_AHB0_BASE + 0x04000)
#define RDA_PSRAMCFG_BASE   (RDA_AHB0_BASE + 0x05000)
#define RDA_SDMMC_BASE      (RDA_AHB0_BASE + 0x06000)
#define RDA_I2C_BASE        (RDA_AHB0_BASE + 0x10000)
#define RDA_TRAP_BASE       (RDA_AHB0_BASE + 0x11000)
#define RDA_UART0_BASE      (RDA_AHB0_BASE + 0x12000)
#define RDA_EXIF_BASE       (RDA_AHB0_BASE + 0x13000)
#define RDA_PA_BASE         (RDA_AHB0_BASE + 0x20000)
#define RDA_CE_BASE         (RDA_AHB0_BASE + 0x22000)
#define RDA_MON_BASE        (RDA_AHB0_BASE + 0x24000)
#define RDA_SDIO_BASE       (RDA_AHB0_BASE + 0x30000)
#define RDA_USB_BASE        (RDA_AHB0_BASE + 0x31000)

/* AHB1 peripherals *****************************************************************/
#define RDA_MEMC_BASE       (RDA_AHB1_BASE + 0x00000)
#define RDA_UART1_BASE      (RDA_AHB1_BASE + 0x80000)
#define RDA_DMACFG_BASE     (RDA_AHB1_BASE + 0x81000)
#define RDA_RNG_BASE        (RDA_AHB1_BASE + 0x81100)

/* EXIF peripherals *****************************************************************/
#define RDA_SPI0_BASE       (RDA_EXIF_BASE + 0x00000)
#define RDA_I2S_BASE        (RDA_EXIF_BASE + 0x0000C)

/* MISC peripherals *****************************************************************/
#define RDA_WDT_BASE        (RDA_SCU_BASE  + 0x0000C)
#define RDA_PINCONN_BASE    (RDA_GPIO_BASE + 0x00044)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_RDA5981X_RDA5981X_MEMORYMAP_H */
