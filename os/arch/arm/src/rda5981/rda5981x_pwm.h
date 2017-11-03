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
 * arch/arm/src/s5j/s5j_pwm.h
 *
 *   Copyright (C) 2013-2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * The Samsung sample code has a BSD compatible license that requires this
 * copyright notice:
 *
 *   Copyright (c) 2016 Samsung Electronics, Inc.
 *   All rights reserved.
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
 */

#ifndef	__ARCH_ARM_SRC_S5J_S5J_PWM_H
#define	__ARCH_ARM_SRC_S5J_S5J_PWM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <tinyara/config.h>

#include <tinyara/pwm.h>

/*rw register as mbed style*/
//#include "chip/rda5981x_pwm.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

/** Pwmout hal structure. pwmout_s is declared in the target's hal
 */
#define __IO volatile
#define __I  volatile
typedef enum {
	PWM_0 = 0,
	PWM_1,
	PWM_2,
	PWM_3,
	PWM_4,
	PWM_5,
	PWM_6,
	PWM_7
} PWMName;

typedef struct {
	__IO uint32_t CFG;
	__IO uint32_t D0CMD;
	__IO uint32_t D1CMD;
} RDA_SPI_TypeDef;

/*------------- Integrated Interchip Sound (I2S) -----------------------------*/
typedef struct {
	__IO uint32_t CFG;
	__IO uint32_t DOUTWR;
	__I  uint32_t DINRD;
} RDA_I2S_TypeDef;

/*------------- External Interface (EXIF) ------------------------------------*/
typedef struct {
	RDA_SPI_TypeDef SPI0;                 /* 0x00 - 0x08 : SPI0 registers group */
	RDA_I2S_TypeDef I2S;                  /* 0x0C - 0x14 : I2S registers group  */
	__IO uint32_t MISCSTCFG;              /* 0x18 : Misc status config register */
	__IO uint32_t SPI1CTRL;               /* 0x1C : SPI1 Control register       */
	uint32_t RESERVED0[4];
	__IO uint32_t MISCINTCFG;             /* 0x30 : Misc int config register    */
	__IO uint32_t MBB2W;                  /* 0x34 : BT to WiFi mailbox register */
	__IO uint32_t MBW2B;                  /* 0x38 : WiFi to BT mailbox register */
	__IO uint32_t MISCCFG;                /* 0x3C : Misc configure register     */
	__IO uint32_t PWM0CFG;                /* 0x40 : PWM0 configure register     */
	__IO uint32_t PWM1CFG;                /* 0x44 : PWM1 configure register     */
	__IO uint32_t PWM2CFG;                /* 0x48 : PWM2 configure register     */
	__IO uint32_t PWM3CFG;                /* 0x4C : PWM3 configure register     */
} RDA_EXIF_TypeDef;

#define RDA_EXIF ((RDA_EXIF_TypeDef*) RDA_EXIF_BASE)

struct pwmout_s {
	uint32_t channel;
	uint32_t base_clk;
	uint32_t period_ticks;
	uint32_t pulsewidth_ticks;
	__IO uint32_t *CFGR;
};

typedef struct {
	__IO uint32_t PWTCFG;                 /* 0x00 : PWT Config Register         */
	__IO uint32_t LPGCFG;                 /* 0x04 : LPG Config Register         */
	__IO uint32_t PWL0CFG;                /* 0x08 : PWL0 Config Register        */
	__IO uint32_t PWL1CFG;                /* 0x0C : PWL1 Config Register        */
	__IO uint32_t CLKR;                   /* 0x10 : Clock Config Register       */
} RDA_PWM_TypeDef;

typedef struct {
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
} RDA_SCU_TypeDef;


#define RDA_PWM  ((RDA_PWM_TypeDef *) RDA_PWM_BASE)
typedef struct pwmout_s pwmout_t;

/**
 * \defgroup hal_pwmout Pwmout hal functions
 * @{
 */

/** Initialize the pwm out peripheral and configure the pin
 *
 * @param obj The pwmout object to initialize
 * @param pin The pwmout pin to initialize
 */
void pwmout_init(pwmout_t *obj, int id);

/** Deinitialize the pwmout object
 *
 * @param obj The pwmout object
 */
void pwmout_free(pwmout_t *obj);

/** Set the output duty-cycle in range <0.0f, 1.0f>
 *
 * Value 0.0f represents 0 percentage, 1.0f represents 100 percent.
 * @param obj     The pwmout object
 * @param percent The floating-point percentage number
 */
void pwmout_write(pwmout_t *obj, float percent);

/** Read the current float-point output duty-cycle
 *
 * @param obj The pwmout object
 * @return A floating-point output duty-cycle
 */
float pwmout_read(pwmout_t *obj);

/** Set the PWM period specified in seconds, keeping the duty cycle the same
 *
 * Periods smaller than microseconds (the lowest resolution) are set to zero.
 * @param obj     The pwmout object
 * @param seconds The floating-point seconds period
 */
void pwmout_period(pwmout_t *obj, float seconds);

/** Set the PWM period specified in miliseconds, keeping the duty cycle the same
 *
 * @param obj The pwmout object
 * @param ms  The milisecond period
 */
void pwmout_period_ms(pwmout_t *obj, int ms);

/** Set the PWM period specified in microseconds, keeping the duty cycle the same
 *
 * @param obj The pwmout object
 * @param us  The microsecond period
 */
void pwmout_period_us(pwmout_t *obj, int us);

/** Set the PWM pulsewidth specified in seconds, keeping the period the same.
 *
 * @param obj     The pwmout object
 * @param seconds The floating-point pulsewidth in seconds
 */
void pwmout_pulsewidth(pwmout_t *obj, float seconds);

/** Set the PWM pulsewidth specified in miliseconds, keeping the period the same.
 *
 * @param obj The pwmout object
 * @param ms  The floating-point pulsewidth in miliseconds
 */
void pwmout_pulsewidth_ms(pwmout_t *obj, int ms);

/** Set the PWM pulsewidth specified in microseconds, keeping the period the same.
 *
 * @param obj The pwmout object
 * @param us  The floating-point pulsewidth in microseconds
 */
void pwmout_pulsewidth_us(pwmout_t *obj, int us);

/**@}*/

#ifdef __cplusplus
}
#endif

FAR struct pwm_lowerhalf_s *rda5981x_pwminitialize(int timer);

#endif /* __ARCH_ARM_SRC_S5J_S5J_PWM_H */
