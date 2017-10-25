/****************************************************************************
 *
 * Copyright 2016 Samsung Electronics All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the License.
 *
 ****************************************************************************/
/****************************************************************************
 * arch/arm/src/rda5981x/rda5981x_rtc.h
 *
 *   Copyright (C) 2009-2010, 2014-2015 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_RDA5981X_RDA5981X_RTC_H
#define __ARCH_ARM_SRC_RDA5981X_RDA5981X_RTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <tinyara/config.h>
#include <up_arch.h>

#if defined(CONFIG_RDA5981_RTC)
#include "chip/rda5981x_rtc.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/
/* The form of an alarm callback */
typedef void (*alarmcb_t)(void);

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * Name: rda5981x_rtc_lowerhalf
 *
 * Description:
 *   Instantiate the RTC lower half driver for the RDA5981X.
 *   General usage:
 *
 *     #include <tinyara/rtc.h>
 *     #include "rda5981x_rtc.h"
 *
 *     struct rtc_lowerhalf_s *lower;
 *     lower = rda5981x_rtc_lowerhalf();
 *     rtc_initialize(0, lower);
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success, a non-NULL RTC lower interface is returned. NULL is
 *   returned on any failure.
 *
 ****************************************************************************/

#include "chip/rda5981x_memorymap.h"

#define DEVICE_LOWPOWERTIMER    0
#define RDA5991H_HW_VER         4


#define RDA_BUS_CLK_FREQUENCY_80M                       ( 80000000UL)


#define RDA_CLKGATE1          (RDA_SCU_BASE+0x08)

#define RDA_TIMER0_LDCNT       (RDA_TIM0_BASE+0x00)
#define RDA_TIMER0_CVAL        (RDA_TIM0_BASE+0x04)
#define RDA_TIMER0_TCTRL       (RDA_TIM0_BASE+0x08)
#define RDA_TIMER0_INTCLR      (RDA_TIM0_BASE+0x0c)

#if 0
/*------------- Timer0 (TIM0) ------------------------------------------------*/
typedef struct
{
  __IO uint32_t LDCNT;                  /* 0x00 : Timer Load Count Register   */
  __I  uint32_t CVAL;                   /* 0x04 : Current Timer Value Register*/
  __IO uint32_t TCTRL;                  /* 0x08 : Timer Control Register      */
  __I  uint32_t INTCLR;                 /* 0x0C : Interrupt Clear Register    */
} RDA_TIM0_TypeDef;
#endif

#define RDA_TIMER1_TCTRL       (RDA_TIM1_BASE+0x00)
#define RDA_TIMER1_LDCNT       (RDA_TIM1_BASE+0x0c)
#define RDA_TIMER1_CVAL        (RDA_TIM1_BASE+0x04)
#define RDA_TIMER1_INTCLR      (RDA_TIM1_BASE+0x18)

#if 0
typedef struct
{
  __IO uint32_t TCTRL;                  /* 0x00 : Timer Control Register      */
       uint32_t RESERVED0[2];
  __IO uint32_t LDCNT;                  /* 0x0C : Timer Load Count Register   */
  __I  uint32_t CVAL;                   /* 0x10 : Current Timer Value Register*/
       uint32_t RESERVED1;
  __I  uint32_t INTCLR;                 /* 0x18 : Interrupt Clear Register    */
} RDA_TIM1_TypeDef;
#endif


#define RTC_TIMER_INITVAL_REG  (RDA_SCU_BASE+0x20) /* 0x20 : Free Timer Initial Value    */
#define RTC_TIMER_PRESCALE     (RDA_SCU_BASE+0x50) /* 0x50 : Free Timer Prescale Init Val*/
#define RTC_TIMER_TIMESTAMP    (RDA_SCU_BASE+0x24) /* 0x24 : Free Timer Timestamp        */
#define RDA_POWER_CONTROL      (RDA_SCU_BASE+0x4)/* 0x04 : Power Control               */


#if 0
typedef struct
{
  __IO uint32_t CLKGATE0;               /* 0x00 : Clock Gating 0              */
  __IO uint32_t PWRCTRL;                /* 0x04 : Power Control               */
  __IO uint32_t CLKGATE1;               /* 0x08 : Clock Gating 1              */
  __IO uint32_t CLKGATE2;               /* 0x0C : Clock Gating 2              */
  __IO uint32_t RESETCTRL;              /* 0x10 : Power Control               */
  __IO uint32_t CLKGATE3;               /* 0x14 : Clock Gating 3              */
  __IO uint32_t CORECFG;                /* 0x18 : Core Config                 */
  __IO uint32_t CPUCFG;                 /* 0x1C : CPU Config                  */
  __IO uint32_t FTMRINITVAL;            /* 0x20 : Free Timer Initial Value    */
  __IO uint32_t FTMRTS;                 /* 0x24 : Free Timer Timestamp        */
  __IO uint32_t CLKGATEBP;              /* 0x28 : Clock Gating Bypass         */
       uint32_t RESERVED0[2];
  __IO uint32_t PWMCFG;                 /* 0x34 : PWM Config                  */
  __IO uint32_t FUN0WAKEVAL;            /* 0x38 : SDIO Func0 Wake Val         */
  __IO uint32_t FUN1WAKEVAL;            /* 0x3C : SDIO Func1 Wake Val         */
  __IO uint32_t BOOTJUMPADDR;           /* 0x40 : Boot Jump Addr              */
  __IO uint32_t SDIOINTVAL;             /* 0x44 : SDIO Int Val                */
  __IO uint32_t I2SCLKDIV;              /* 0x48 : I2S Clock Divider           */
  __IO uint32_t BOOTJUMPADDRCFG;        /* 0x4C : Boot Jump Addr Config       */
  __IO uint32_t FTMRPREVAL;             /* 0x50 : Free Timer Prescale Init Val*/
  __IO uint32_t PWROPENCFG;             /* 0x54 : Power Open Config           */
  __IO uint32_t PWRCLOSECFG;            /* 0x58 : Power Close Config          */
}
#endif



#define TIMER0_CONTROL_ENABLE       (0x01)
#define TIMER0_CONTROL_MODE         (0x02)
#define TIMER0_CONTROL_INT_MSK      (0x04)
#define TIMER1_CONTROL_ENABLE       (0x20)
#define TIMER1_CONTROL_MODE         (0x40)
#define TIMER1_CONTROL_INT_MSK      (0x80)

#define US_TIMER_CLOCK_SOURCE       (RDA_BUS_CLK_FREQUENCY_80M >> 1)
#define US_TIMER_HZ                 (1000000)
#define TIMER1_CLOCK_SOURCE         (32768)
#define FREE_TIMER_CLOCK_SOURCE     (32768)

#define TIMER1_LDCNT_INIT_VAL       (0x08637BD0UL)
#define TIMER1_CURVAL_DELTA         (0xF79C842FUL)
#define FREE_TIMER_INIT_VAL         (0x08637BD0UL)

#define RDA_TIMER_INTSTATE          (RDA_TIMINTST_BASE)




#ifdef CONFIG_RTC_DRIVER
FAR struct rtc_lowerhalf_s *rda5981x_rtc_lowerhalf(void);
#endif /* CONFIG_RTC_DRIVER */


#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_RDA5981X_RDA5981X_RTC_H */
