/************************************************************************************
 * arch/arm/src/rda5981x/chip/rda5981x_syscon.h
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

#ifndef __ARCH_ARM_SRC_RDA5981X_CHIP_RDA5981X_SYSCON_H
#define __ARCH_ARM_SRC_RDA5981X_CHIP_RDA5981X_SYSCON_H

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

#define RDA_SCU_CLKGATE0_OFSET              0x0000 /* Clock Gating 0                */
#define RDA_SCU_PWRCTRL_OFSET               0x0004 /* Power Control                 */
#define RDA_SCU_CLKGATE1_OFSET              0x0008 /* Clock Gating 1                */
#define RDA_SCU_CLKGATE2_OFSET              0x000C /* Clock Gating 2                */
#define RDA_SCU_RESETCTRL_OFSET             0x0010 /* Reset Control                 */
#define RDA_SCU_CLKGATE3_OFSET              0x0014 /* Clock Gating 3                */
#define RDA_SCU_CORECFG_OFSET               0x0018 /* Core Config                   */
#define RDA_SCU_CPUCFG_OFSET                0x001C /* CPU Config                    */
#define RDA_SCU_FTMRINITVAL_OFSET           0x0020 /* Free Timer Initial Value      */
#define RDA_SCU_FTMRTS_OFSET                0x0024 /* Free Timer Timestamp          */
#define RDA_SCU_CLKGATEBP_OFSET             0x0028 /* Clock Gating Bypass           */
#define RDA_SCU_PWMCFG_OFSET                0x0034 /* PWM Config                    */
#define RDA_SCU_FUN0WAKEVAL_OFSET           0x0038 /* SDIO Func0 Wake Val           */
#define RDA_SCU_FUN1WAKEVAL_OFSET           0x003C /* SDIO Func1 Wake Val           */
#define RDA_SCU_BOOTJUMPADDR_OFSET          0x0040 /* Boot Jump Addr                */
#define RDA_SCU_SDIOINTVAL_OFSET            0x0044 /* SDIO Int Val                  */
#define RDA_SCU_I2SCLKDIV_OFSET             0x0048 /* I2S Clock Divider             */
#define RDA_SCU_BOOTJUMPADDRCFG_OFSET       0x004C /* Boot Jump Addr Config         */
#define RDA_SCU_FTMRPREVAL_OFSET            0x0050 /* Free Timer Prescale Init Val  */
#define RDA_SCU_PWROPENCFG_OFSET            0x0054 /* Power Open Config             */
#define RDA_SCU_PWRCLOSECFG_OFSET           0x0058 /* Power Close Config            */

/* Register addresses ***************************************************************/
/* 0x0000 : Clock Gating 0               */
#define RDA_SCU_CLKGATE0        (RDA_SCU_BASE + RDA_SCU_CLKGATE0_OFSET       )

/* 0x0004 : Power Control                */
#define RDA_SCU_PWRCTRL         (RDA_SCU_BASE + RDA_SCU_PWRCTRL_OFSET        )

/* 0x0008 : Clock Gating 1               */
#define RDA_SCU_CLKGATE1        (RDA_SCU_BASE + RDA_SCU_CLKGATE1_OFSET       )

/* 0x000C : Clock Gating 2               */
#define RDA_SCU_CLKGATE2        (RDA_SCU_BASE + RDA_SCU_CLKGATE2_OFSET       )

/* 0x0010 : Reset Control                */
#define RDA_SCU_RESETCTRL       (RDA_SCU_BASE + RDA_SCU_RESETCTRL_OFSET      )

/* 0x0014 : Clock Gating 3               */
#define RDA_SCU_CLKGATE3        (RDA_SCU_BASE + RDA_SCU_CLKGATE3_OFSET       )

/* 0x0018 : Core Config                  */
#define RDA_SCU_CORECFG         (RDA_SCU_BASE + RDA_SCU_CORECFG_OFSET        )

/* 0x001C : CPU Config                   */
#define RDA_SCU_CPUCFG          (RDA_SCU_BASE + RDA_SCU_CPUCFG_OFSET         )

/* 0x0020 : Free Timer Initial Value     */
#define RDA_SCU_FTMRINITVAL     (RDA_SCU_BASE + RDA_SCU_FTMRINITVAL_OFSET    )

/* 0x0024 : Free Timer Timestamp         */
#define RDA_SCU_FTMRTS          (RDA_SCU_BASE + RDA_SCU_FTMRTS_OFSET         )

/* 0x0028 : Clock Gating Bypass          */
#define RDA_SCU_CLKGATEBP       (RDA_SCU_BASE + RDA_SCU_CLKGATEBP_OFSET      )

/* 0x0034 : PWM Config                   */
#define RDA_SCU_PWMCFG          (RDA_SCU_BASE + RDA_SCU_PWMCFG_OFSET         )

/* 0x0038 : SDIO Func0 Wake Val          */
#define RDA_SCU_FUN0WAKEVAL     (RDA_SCU_BASE + RDA_SCU_FUN0WAKEVAL_OFSET    )

/* 0x003C : SDIO Func1 Wake Val          */
#define RDA_SCU_FUN1WAKEVAL     (RDA_SCU_BASE + RDA_SCU_FUN1WAKEVAL_OFSET    )

/* 0x0040 : Boot Jump Addr               */
#define RDA_SCU_BOOTJUMPADDR    (RDA_SCU_BASE + RDA_SCU_BOOTJUMPADDR_OFSET   )

/* 0x0044 : SDIO Int Val                 */
#define RDA_SCU_SDIOINTVAL      (RDA_SCU_BASE + RDA_SCU_SDIOINTVAL_OFSET     )

/* 0x0048 : I2S Clock Divider            */
#define RDA_SCU_I2SCLKDIV       (RDA_SCU_BASE + RDA_SCU_I2SCLKDIV_OFSET      )

/* 0x004C : Boot Jump Addr Config        */
#define RDA_SCU_BOOTJUMPADDRCFG (RDA_SCU_BASE + RDA_SCU_BOOTJUMPADDRCFG_OFSET)

/* 0x0050 : Free Timer Prescale Init Val */
#define RDA_SCU_FTMRPREVAL      (RDA_SCU_BASE + RDA_SCU_FTMRPREVAL_OFSET     )

/* 0x0054 : Power Open Config            */
#define RDA_SCU_PWROPENCFG      (RDA_SCU_BASE + RDA_SCU_PWROPENCFG_OFSET     )

/* 0x0058 : Power Close Config           */
#define RDA_SCU_PWRCLOSECFG     (RDA_SCU_BASE + RDA_SCU_PWRCLOSECFG_OFSET    )

/* Register bit definitions *********************************************************/
/* 0x0000 : Clock Gating 0               */
#define SCU_CLKGATE0_DSM        (1 << 0)        /* Bit 0: Deep sleep mode */

/* 0x0004 : Power Control                */
                                                /* Bits 0-2: Reserved */
#define SCU_PWRCTRL_GPWE        (1 << 3)        /* Bit 3:    GPIO wakeup en */
#define SCU_PWRCTRL_UAWE        (1 << 4)        /* Bit 4:    UART wakeup en */
#define SCU_PWRCTRL_SDWE        (1 << 5)        /* Bit 5:    SDIO wakeup en */
#define SCU_PWRCTRL_USWE        (1 << 6)        /* Bit 6:    USB wakeup en */
                                                /* Bit 7:    Reserved */
#define SCU_PWRCTRL_FTRWE       (1 << 8)        /* Bit 8:    Free timer reg write en */

/* 0x0008 : Clock Gating 1               */
#define SCU_CLKGATE1_PCKGPE     (1 << 0)        /* Bit 0: PCLK GPIO en */

/* 0x000C : Clock Gating 2               */
#define SCU_CLKGATE2_USC_SHIFT  (0)             /* Bits 0-2: USB sleep config */
#define SCU_CLKGATE2_USC_MASK   (0xFF << SCU_CLKGATE2_USC_SHIFT)
#define SCU_CLKGATE2_TWE        (1 << 8)        /* Bit 8:    Timer wakeup en */

/* 0x0010 : Reset Control                */
#define SCU_RESETCTRL_SRSYS     (1 << 0)        /* Bit 0: Soft RESETn system */
#define SCU_RESETCTRL_SRUSB     (1 << 1)        /* Bit 1: Soft RESETn USB */
#define SCU_RESETCTRL_SRTIM     (1 << 2)        /* Bit 2: Soft RESETn timer */
#define SCU_RESETCTRL_SRGP      (1 << 3)        /* Bit 3: Soft RESETn GPIO */
#define SCU_RESETCTRL_SRUA      (1 << 4)        /* Bit 4: Soft RESETn UART */
#define SCU_RESETCTRL_SRSD      (1 << 5)        /* Bit 5: Soft RESETn SDIO */
#define SCU_RESETCTRL_SRCN      (1 << 6)        /* Bit 6: Soft RESETn cntl */
#define SCU_RESETCTRL_SRI2C     (1 << 7)        /* Bit 7: Soft RESETn I2C */
#define SCU_RESETCTRL_SRCPU     (1 << 8)        /* Bit 8: Soft RESETn CPU */
#define SCU_RESETCTRL_SRSCU     (1 << 9)        /* Bit 9: Soft RESETn SCU */

/* 0x0014 : Clock Gating 3               */
                                                /* Bits 0-1: Reserved */
#define SCU_CLKGATE3_SE         (1 << 2)        /* Bit 2:    Sleep en */

/* 0x0018 : Core Config                  */
#define SCU_CORECFG_CKSEL_SHIFT (0)             /* Bits 0-1: CLK40_41_42_43M sel */
#define SCU_CORECFG_CKSEL_MASK  (0x03 << SCU_CORECFG_CKSEL_SHIFT)
#define SCU_CORECFG_I2CWE       (1 << 2)        /* Bit 2:    I2C wakeup ENABLEn */

/* 0x001C : CPU Config                   */
#define SCU_CPUCFG_ST1MS_SHIFT  (0)             /* Bits 0-23:  Systick 1 ms */
#define SCU_CPUCFG_ST1MS_MASK   (0xFFFFFF << SCU_CPUCFG_ST1MS_SHIFT)
#define SCU_CPUCFG_MPUDIS       (1 << 24)       /* Bit 24:     MPU disable */
                                                /* Bits 25-31: Reserved */

/* 0x0028 : Clock Gating Bypass          */
#define SCU_CLKGATEBP_PACGBP    (1 << 0)        /* Bit 0:     PA CLK gate bypass */
#define SCU_CLKGATEBP_PHYTCGBP  (1 << 1)        /* Bit 1:     PHY TX CLK gate bypass */
#define SCU_CLKGATEBP_PHYRCGBP  (1 << 2)        /* Bit 2:     PHY RX CLK gate bypass */
#define SCU_CLKGATEBP_TSFCGBP   (1 << 3)        /* Bit 3:     TSF CLK gate bypass */
                                                /* Bits 4-30: Reserved */
#define SCU_CLKGATEBP_SDMCOGBP  (1 << 31)       /* Bit 31:    SDMMC CLK out gate bypass */

/* 0x0034 : PWM Config                   */
#define SCU_PWMCFG_PWTCD_SHIFT  (0)             /* Bits 0-6: PWT clock divider */
#define SCU_PWMCFG_PWTCD_MASK   (0x7F << SCU_PWMCFG_PWTCD_SHIFT)
#define SCU_PWMCFG_PWTFC        (1 << 7)        /* Bit 7:    PWT fast clock */

/* 0x003C : SDIO Func1 Wake Val          */
#define SCU_FUN1WAKEVAL_SHIFT   (0)             /* Bits 0-15:  func1 wake val */
#define SCU_FUN1WAKEVAL_MASK    (0xFFFF << SCU_FUN1WAKEVAL_SHIFT)
                                                /* Bits 16-31: Reserved */

/* 0x0050 : Free Timer Prescale Init Val */
#define SCU_FTMRPREVAL_SHIFT    (0)             /* Bits 0-11:  Free timer prescale val */
#define SCU_FTMRPREVAL_MASK     (0x0FFF << SCU_FTMRPREVAL_SHIFT)
                                                /* Bits 12-31: Reserved */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_RDA5981X_CHIP_RDA5981X_SYSCON_H */
