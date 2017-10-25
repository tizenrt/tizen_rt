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
 * arch/arm/src/rda5981x/rda5981x_pwm.c
 *
 *   Copyright (C) 2011-2012, 2014 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <tinyara/config.h>

#include <sys/types.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <tinyara/pwm.h>
#include "up_arch.h"
#include "rda5981x_pwm.h"
#include "rda5981x_gpio.h"
#include "chip/rda5981x_pinconfig.h"
#include "chip/rda5981x_memorymap.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/
struct rda5981x_pwmtimer_s {
	FAR const struct pwm_ops_s *ops;
	uint8_t id;
	uint32_t pincfg;
	pwmout_t obj;
};

#define RDA_SCU   ((RDA_SCU_TypeDef*)RDA_SCU_BASE)

#define PWM_CLK_SRC_20MHZ       (20000000)
#define PWM_CLK_SRC_32KHZ       (32768)

#define PWM_CLKGATE_REG         (RDA_SCU->CLKGATE1)
#define PWM_CLKSRC_REG          (RDA_SCU->PWMCFG)
#define PWM_CLKDIV_REG          (RDA_PWM->CLKR)
#define EXIF_PWM_EN_REG         (RDA_EXIF->MISCCFG)

__IO uint32_t *PWM_MATCH[] = {
    &(RDA_EXIF->PWM0CFG),
    &(RDA_EXIF->PWM1CFG),
    &(RDA_EXIF->PWM2CFG),
    &(RDA_EXIF->PWM3CFG),
    &( RDA_PWM->PWTCFG ),
    &( RDA_PWM->LPGCFG ),
    &( RDA_PWM->PWL0CFG),
    &( RDA_PWM->PWL1CFG)
};

static uint8_t is_pwmout_started(pwmout_t* obj);
static void pwmout_start(pwmout_t* obj);
static void pwmout_stop(pwmout_t* obj);
static void pwmout_update_cfgreg(pwmout_t* obj);

void pwmout_init(pwmout_t* obj, int chanel_id)
{
    uint32_t reg_val = 0U;

    obj->channel = chanel_id;
    obj->CFGR = PWM_MATCH[chanel_id];

    /* Enable PWM Clock-gating */
    PWM_CLKGATE_REG |= (0x01UL << 2);

    /* Init PWM clock source and divider */
    if(PWM_4 >= chanel_id) {
        /* Src: 20MHz, Divider: (3 + 1) */
        reg_val = PWM_CLKSRC_REG & ~0x0000FFUL;
        PWM_CLKSRC_REG = reg_val | (0x01UL << 7) | 0x03UL | (0x01UL << 24);
        obj->base_clk = (PWM_CLK_SRC_20MHZ >> 2) >> 1;
    } else if(PWM_5 == chanel_id) {
        /* Src: 32KHz, Divider: (0 + 1) */
        reg_val = PWM_CLKSRC_REG & ~0x00FF00UL;
        PWM_CLKSRC_REG = reg_val | (0x00UL << 8) | (0x01UL << 25);
        obj->base_clk = PWM_CLK_SRC_32KHZ >> 1;
    } else {
        /* Src: 32KHz, Divider: (0 + 1) */
        reg_val = PWM_CLKSRC_REG & ~0xFF0000UL;
        PWM_CLKSRC_REG = reg_val | (0x00UL << 16) | (0x01UL << 26);
        obj->base_clk = PWM_CLK_SRC_32KHZ >> 1;
    }

    // default to 20ms: standard for servos, and fine for e.g. brightness control
    pwmout_period_ms(obj, 20);
    pwmout_write    (obj, 0);

}

void pwmout_free(pwmout_t* obj)
{
    /* Disable PWM Clock-gating */
    PWM_CLKGATE_REG &= ~(0x01UL << 2);
}

void pwmout_write(pwmout_t* obj, float value)
{
    uint32_t ticks;

    /* Check if already started */
    if(is_pwmout_started(obj))
        pwmout_stop(obj);

    if (value < 0.0f) {
        value = 0.0;
    } else if (value > 1.0f) {
        value = 1.0;
    }

    /* Set channel match to percentage */
    ticks = (uint32_t)((float)(obj->period_ticks) * value);

    if (0 == ticks) {
        obj->pulsewidth_ticks = 0;
    } else {
        /* Update Hw reg */
        if(ticks != obj->pulsewidth_ticks) {
            obj->pulsewidth_ticks = ticks;
            pwmout_update_cfgreg(obj);
        }

        /* Start PWM module */
        pwmout_start(obj);
    }
}

float pwmout_read(pwmout_t* obj)
{
    float v = (float)(obj->pulsewidth_ticks) / (float)(obj->period_ticks);
    return (v > 1.0f) ? (1.0f) : (v);
}

void pwmout_period(pwmout_t* obj, float seconds)
{
    pwmout_period_us(obj, seconds * 1000000.0f);
}

void pwmout_period_ms(pwmout_t* obj, int ms)
{
    pwmout_period_us(obj, ms * 1000);
}

/* Set the PWM period, keeping the duty cycle the same. */
void pwmout_period_us(pwmout_t* obj, int us)
{
    uint32_t ticks;
    /* Check if already started */
    if(is_pwmout_started(obj))
        pwmout_stop(obj);

    /* Calculate number of ticks */
    ticks = (uint64_t)obj->base_clk * us / 1000000;

    if(ticks != obj->period_ticks) {
        float duty_ratio;

        /* Preserve the duty ratio */
        duty_ratio = (float)obj->pulsewidth_ticks / (float)obj->period_ticks;
        obj->period_ticks = ticks;
        obj->pulsewidth_ticks = (uint32_t)(ticks * duty_ratio);
//        MBED_ASSERT(obj->period_ticks >= obj->pulsewidth_ticks);
	pwmout_update_cfgreg(obj);
    }

    /* Start PWM module */
    pwmout_start(obj);
}

void pwmout_pulsewidth(pwmout_t* obj, float seconds)
{
    pwmout_pulsewidth_us(obj, seconds * 1000000.0f);
}

void pwmout_pulsewidth_ms(pwmout_t* obj, int ms)
{
    pwmout_pulsewidth_us(obj, ms * 1000);
}

/* Set the PWM pulsewidth, keeping the period the same. */
void pwmout_pulsewidth_us(pwmout_t* obj, int us)
{
    uint32_t ticks;
    /* Check if already started */
    if(is_pwmout_started(obj))
        pwmout_stop(obj);

    /* Calculate number of ticks */
    ticks = (uint64_t)obj->base_clk * us / 1000000;

    if(ticks != obj->pulsewidth_ticks) {
        obj->pulsewidth_ticks = ticks;
        pwmout_update_cfgreg(obj);
    }

    /* Start PWM module */
    pwmout_start(obj);
}

static uint8_t is_pwmout_started(pwmout_t* obj)
{
    uint8_t retVal = 0;
    uint32_t reg_val;

    if(PWM_3 >= (PWMName)obj->channel) {
        reg_val = (EXIF_PWM_EN_REG >> 8) & 0x0FUL;
        if(reg_val & (0x01UL << obj->channel))
            retVal = 1;
    } else if(PWM_4 == (PWMName)obj->channel) {
        if(*(obj->CFGR) & (0x01UL << 1))
            retVal = 1;
    } else if(PWM_5 == (PWMName)obj->channel) {
        retVal = 1;
    } else {
        if(*(obj->CFGR) & (0x01UL << 16))
            retVal = 1;
    }
    return retVal;
}

static void pwmout_start(pwmout_t* obj)
{
    if(PWM_3 >= (PWMName)obj->channel) {
        EXIF_PWM_EN_REG |= (0x01UL << (8 + obj->channel));
    } else if(PWM_4 == (PWMName)obj->channel) {
        *(obj->CFGR) |= 0x01UL;
    } else if(PWM_5 == (PWMName)obj->channel) {
        /* Nothing to be done */
    } else {
        *(obj->CFGR) |= (0x01UL << 16);
    }
}

static void pwmout_stop(pwmout_t* obj)
{
    if(PWM_3 >= (PWMName)obj->channel) {
        EXIF_PWM_EN_REG &= ~(0x01UL << (8 + obj->channel));
    } else if(PWM_4 == (PWMName)(obj->channel)) {
        *(obj->CFGR) &= ~0x01UL;
    } else if(PWM_5 == (PWMName)(obj->channel)) {
        /* Nothing to be done */
    } else {
        *(obj->CFGR) &= ~(0x01UL << 16);
    }
}

static void pwmout_update_cfgreg(pwmout_t* obj)
{
    if(PWM_3 >= (obj->channel)) {
        if (obj->period_ticks == obj->pulsewidth_ticks) {
            *(obj->CFGR) = ((obj->pulsewidth_ticks - 1) << 16);
        } else {
            *(obj->CFGR) = ((obj->period_ticks - obj->pulsewidth_ticks - 1) & 0xFFFFUL) |
                ((obj->pulsewidth_ticks - 1) << 16);
        }
    } else if(PWM_4 == (obj->channel)) {
        if (obj->pulsewidth_ticks < 8)
            obj->pulsewidth_ticks = 8;
        //MBED_ASSERT(((obj->period_ticks >> 1) >= obj->pulsewidth_ticks) &&
          //  (obj->pulsewidth_ticks >= 8));
        *(obj->CFGR) = ((obj->pulsewidth_ticks & 0x3FFUL) << 4) | ((obj->period_ticks & 0x7FF) << 16);
    } else if(PWM_5 == (obj->channel)) {
        /* TBD */
        uint32_t reg_val = *(obj->CFGR) & ~(0xFUL << 4) & ~(0x7UL << 16);
        uint32_t lpg_field_ontime = (0x01UL << 4) & (0xFUL << 4); // to be confirm
        uint32_t lpg_field_period = (obj->period_ticks << 4) & (0x7UL << 16);
        *(obj->CFGR) = reg_val | lpg_field_ontime | lpg_field_period;
    } else {
    }
}

#if 0
static uint32_t pwm_getreg32(struct rda5981x_pwmtimer_s *priv, int offset)
{
	return getreg32(priv->base + offset);
}

static void pwm_putreg32(struct rda5981x_pwmtimer_s *priv, int offset, uint32_t value)
{
	putreg32(value, priv->base + offset);
}

static void pwm_modifyreg32(struct rda5981x_pwmtimer_s *priv, int offset,
				uint32_t clearbits, uint32_t setbits)
{
	modifyreg32(priv->base + offset, clearbits, setbits);
}
#endif

/****************************************************************************
 * Name: rda5981x_pwm_setup
 *
 * Description:
 *   This method is called when the driver is opened.  The lower half driver
 *   should configure and initialize the device so that it is ready for use.
 *   It should not, however, output pulses until the start method is called.
 *
 * Input parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 * Assumptions:
 *   APB1 or 2 clocking for the GPIOs has already been configured by the RCC
 *   logic at power up.
 *
 ****************************************************************************/
static int rda5981x_pwm_setup(FAR struct pwm_lowerhalf_s *dev)
{
	FAR struct rda5981x_pwmtimer_s *priv = (FAR struct rda5981x_pwmtimer_s *)dev;
	rda_configgpio(priv->pincfg);
	pwmout_init(&(priv->obj), priv->id);
	return 0;	
}

/****************************************************************************
 * Name: rda5981x_pwm_start
 *
 * Description:
 *   (Re-)initialize the timer resources and start the pulsed output
 *
 * Input parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *   info - A reference to the characteristics of the pulsed output
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/
static int rda5981x_pwm_start(FAR struct pwm_lowerhalf_s *dev,
				FAR const struct pwm_info_s *info)
{
	FAR struct rda5981x_pwmtimer_s *priv = (FAR struct rda5981x_pwmtimer_s *)dev;
        pwmout_start(&(priv->obj));		 	
	return OK;
}

/****************************************************************************
 * Name: rda5981x_pwm_stop
 *
 * Description:
 *   Stop the pulsed output and reset the timer resources
 *
 * Input parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 * Assumptions:
 *   This function is called to stop the pulsed output at anytime.  This
 *   method is also called from the timer interrupt handler when a repetition
 *   count expires... automatically stopping the timer.
 *
 ****************************************************************************/
static int rda5981x_pwm_stop(FAR struct pwm_lowerhalf_s *dev)
{
	FAR struct rda5981x_pwmtimer_s *priv = (FAR struct rda5981x_pwmtimer_s *)dev;
	pwmout_stop(&(priv->obj));
	return OK;
}

/****************************************************************************
 * Name: rda5981x_pwm_shutdown
 *
 * Description:
 *   This method is called when the driver is closed.  The lower half driver
 *   stop pulsed output, free any resources, disable the timer hardware, and
 *   put the system into the lowest possible power usage state
 *
 * Input parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/
static int rda5981x_pwm_shutdown(FAR struct pwm_lowerhalf_s *dev)
{
	FAR struct rda5981x_pwmtimer_s *priv = (FAR struct rda5981x_pwmtimer_s *)dev;

	/* Make sure that the output has been stopped */
	pwmout_stop(&(priv->obj));

	/* Then put the GPIO pins back to the default state */
	return 0;
//	return rda5981x_unconfiggpio(priv->pincfg);
}

/****************************************************************************
 * Name: rda5981x_pwm_ioctl
 *
 * Description:
 *   Lower-half logic may support platform-specific ioctl commands
 *
 * Input parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *   cmd - The ioctl command
 *   arg - The argument accompanying the ioctl command
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/
static int rda5981x_pwm_ioctl(FAR struct pwm_lowerhalf_s *dev, int cmd,
						 unsigned long arg)
{
	return -ENOTTY;
}

/****************************************************************************
 * Private Data
 ****************************************************************************/
static const struct pwm_ops_s g_pwm_ops = {
	.setup		= rda5981x_pwm_setup,
	.shutdown	= rda5981x_pwm_shutdown,
	.start		= rda5981x_pwm_start,
	.stop		= rda5981x_pwm_stop,
	.ioctl		= rda5981x_pwm_ioctl,
};

static struct rda5981x_pwmtimer_s g_pwm0 = {
	.ops	= &g_pwm_ops,
	.id		= 0,
	.pincfg	= GPIO_PWM_TOUT0,
};

static struct rda5981x_pwmtimer_s g_pwm1 = {
	.ops	= &g_pwm_ops,
	.id		= 1,
	.pincfg	= GPIO_PWM_TOUT1,
};

static struct rda5981x_pwmtimer_s g_pwm2 = {
	.ops	= &g_pwm_ops,
	.id		= 2,
	.pincfg	= GPIO_PWM_TOUT2,
};

static struct rda5981x_pwmtimer_s g_pwm3 = {
	.ops	= &g_pwm_ops,
	.id		= 3,
	.pincfg	= GPIO_PWM_TOUT3,
};

static struct rda5981x_pwmtimer_s g_pwm4 = {
	.ops	= &g_pwm_ops,
	.id		= 4,
	.pincfg	= GPIO_PWM_TOUT4,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rda5981x_pwminitialize
 *
 * Description:
 *   Initialize one timer for use with the upper_level PWM driver.
 *
 * Input Parameters:
 *   timer - A number identifying the timer use. The number of valid timer
 *     IDs varies with the S5J family but is somewhere in the range of
 *     {0,...,5}.
 *
 * Returned Value:
 *   On success, a pointer to the lower-half PWM driver is returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/
FAR struct pwm_lowerhalf_s *rda5981x_pwminitialize(int timer)
{
	struct pwm_lowerhalf_s *lower = NULL;

	if (timer == 0) {
		lower = (struct pwm_lowerhalf_s *)&g_pwm0;
	} 

	else if (timer == 1) {
		lower = (struct pwm_lowerhalf_s *)&g_pwm1;
	} 

	else if (timer == 2) {
		lower = (struct pwm_lowerhalf_s *)&g_pwm2;
	} 
	else if (timer == 3) {
		lower = (struct pwm_lowerhalf_s *)&g_pwm3;
	} 
	else if (timer == 4) {
		lower = (struct pwm_lowerhalf_s *)&g_pwm4;
	} 
	
	else
	{
		lldbg("ERROR: invalid PWM is requested\n");
	}
	return lower;
}
