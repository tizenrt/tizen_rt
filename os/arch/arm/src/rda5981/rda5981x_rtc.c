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
 * arch/arm/src/rda5981x/rda5981x_rtc.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>
#include <tinyara/arch.h>
#include <tinyara/irq.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#include "up_arch.h"

#include "rda5981x_rtc.h"

#ifdef CONFIG_RTC
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#if !defined(CONFIG_RTC_DATETIME)
#error "RDA5981X does not support other than CONFIG_RTC_DATETIME."
#endif

extern uint32_t SystemCoreClock;

#define RDA5991H_HW_VER 4

#if RDA5991H_HW_VER <= 4
uint32_t lpo_ticks_cal = 0U;
uint32_t lpo_tmr_flag  = 0U;
int32_t  lpo_delta_us  = 0;
#endif


static time_t sw_timebase = 0U;
static time_t sw_timeofst = 0U;
static uint32_t round_ticks = 0U;
static uint32_t remain_ticks = 0U;
static int is_rtc_enabled = 0;


/****************************************************************************
 * Private Data
 ****************************************************************************/
#ifdef CONFIG_RTC_ALARM
/* Callback to use when the alarm expires */
static alarmcb_t g_alarmcb;
#endif


static uint8_t is_timer_irq_set = 0;

/****************************************************************************
 * Public Data
 ****************************************************************************/
/* g_rtc_enabled is set true after the RTC has successfully initialized */
volatile bool g_rtc_enabled = false;

/****************************************************************************
 * Private Types
 ****************************************************************************/

#if 0 //zhang
struct rtc_regvals_s {
	uint8_t bcdsec;
	uint8_t bcdmin;
	uint8_t bcdhour;
	uint8_t bcdday;
#if defined(CONFIG_LIBC_LOCALTIME) || defined(CONFIG_TIME_EXTENDED)
	uint8_t bcddayweek;
#endif
	uint8_t bcdmon;
	uint16_t bcdyear;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/
static uint32_t rtc_bin2bcd(int value)
{
	uint32_t hundreds;
	uint32_t tens;

	hundreds = tens = 0;

	while (value >= 100) {
		hundreds++;
		value -= 100;
	}

	while (value >= 10) {
		tens++;
		value -= 10;
	}

	return (hundreds << 8) | (tens << 4) | value;
}

static int rtc_bcd2bin(uint32_t value)
{
	uint32_t tens = ((value >> 4) & 0xf) * 10;
	uint32_t hundreds = ((value >> 8) & 0xf) * 100;
	return (int)(hundreds + tens + (value & 0xf));
}

static void rtc_breakout(FAR const struct tm *tm,
		FAR struct rtc_regvals_s *regvals)
{
	regvals->bcdsec     = rtc_bin2bcd(tm->tm_sec);
	regvals->bcdmin     = rtc_bin2bcd(tm->tm_min);
	regvals->bcdhour    = rtc_bin2bcd(tm->tm_hour);
	regvals->bcdday     = rtc_bin2bcd(tm->tm_mday);
#if defined(CONFIG_LIBC_LOCALTIME) || defined(CONFIG_TIME_EXTENDED)
	regvals->bcddayweek = rtc_bin2bcd(tm->tm_wday + 1);
#endif
	regvals->bcdmon     = rtc_bin2bcd(tm->tm_mon + 1);
	regvals->bcdyear    = rtc_bin2bcd(tm->tm_year);
}


#if defined(CONFIG_RTC_ALARM)
static int rtc_alarm_handler(int irq, void *context, FAR void *arg)
{
	if (getreg32(RDA5981X_RTC_INTP) & RTC_INTP_ALARM) {
		/* Clear pending flags */
		putreg32(RTC_INTP_ALARM, RDA5981X_RTC_INTP);

		/* Disable alarm */
		modifyreg32(RDA5981X_RTC_RTCALM, RTC_RTCALM_ALMEN_MASK,
						RTC_RTCALM_ALMEN_DISABLE);

		/* Alarm callback */
		g_alarmcb();
		g_alarmcb = NULL;
	}

	return OK;
}
#endif
#endif //zhang
/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * Name: up_rtc_getdatetime
 *
 * Description:
 *   Get the current date and time from the date/time RTC. This interface
 *   is only supported by the date/time RTC hardware implementation.
 *   It is used to replace the system timer. It is only used by the RTOS
 *   during initialization to set up the system time when CONFIG_RTC and
 *   CONFIG_RTC_DATETIME are selected (and CONFIG_RTC_HIRES is not).
 *
 *   NOTE: Some date/time RTC hardware is capability of sub-second accuracy.
 *   That sub-second accuracy is lost in this interface. However, since the
 *   system time is reinitialized on each power-up/reset, there will be no
 *   timing inaccuracy in the long run.
 *
 * Input Parameters:
 *   tp - The location to return the high resolution time value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/
int up_rtc_getdatetime(FAR struct tm *tp)
{
#if 0
	irqstate_t flags;
	struct rtc_regvals_s regvals;

	flags = irqsave();

	/* read bcd counters */
	do {
		regvals.bcdsec     = getreg32(RDA5981X_RTC_BCDSEC);
		regvals.bcdmin     = getreg32(RDA5981X_RTC_BCDMIN);
		regvals.bcdhour    = getreg32(RDA5981X_RTC_BCDHOUR);
		regvals.bcdday     = getreg32(RDA5981X_RTC_BCDDAY);
#if defined(CONFIG_LIBC_LOCALTIME) || defined(CONFIG_TIME_EXTENDED)
		regvals.bcddayweek = getreg32(RDA5981X_RTC_BCDDAYWEEK);
#endif
		regvals.bcdmon     = getreg32(RDA5981X_RTC_BCDMON);
		regvals.bcdyear    = getreg32(RDA5981X_RTC_BCDYEAR);
	} while (regvals.bcdsec != getreg32(RDA5981X_RTC_BCDSEC));

	irqrestore(flags);

	tp->tm_sec  = rtc_bcd2bin(regvals.bcdsec);
	tp->tm_min  = rtc_bcd2bin(regvals.bcdmin);
	tp->tm_hour = rtc_bcd2bin(regvals.bcdhour);
	tp->tm_mday = rtc_bcd2bin(regvals.bcdday);
#if defined(CONFIG_LIBC_LOCALTIME) || defined(CONFIG_TIME_EXTENDED)
	tp->tm_wday = rtc_bcd2bin(regvals.bcddayweek) - 1;
#endif
	tp->tm_mon  = rtc_bcd2bin(regvals.bcdmon) - 1;
	tp->tm_year = rtc_bcd2bin(regvals.bcdyear);
#endif
	return OK;
}

/****************************************************************************
 * Name: up_rtc_setdatetime
 *
 * Description:
 *   Set the RTC to the provided time. RTC implementations which provide
 *   up_rtc_getdatetime() (CONFIG_RTC_DATETIME is selected) should provide
 *   this function.
 *
 * Input Parameters:
 *   tp - the time to use
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/
int up_rtc_setdatetime(FAR struct tm *tm)
{
	irqstate_t flags;
	//struct rtc_regvals_s regvals;

	//rtc_breakout(tm, &regvals);

	flags = irqsave();

	//write

	irqrestore(flags);

	return OK;
}

/****************************************************************************
 * Name: up_rtc_settime
 *
 * Description:
 *   Set the RTC to the provided time.  All RTC implementations must be able
 *   to set their time based on a standard timespec.
 *
 * Input Parameters:
 *   tp - the time to use
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/
int up_rtc_settime(FAR const struct timespec *tp)
{
	struct tm t;

	/* convert timepsec to tm */
	gmtime_r(&tp->tv_sec, &t);

	return up_rtc_setdatetime(&t);
}



//RDA CODE

void rtc_base_update(void)
{
    if(is_rtc_enabled) {
        uint32_t sw_rpl = 0U;
        remain_ticks += round_ticks & 0x00007FFFUL;
        sw_rpl = remain_ticks >> 15;
        remain_ticks &= 0x00007FFFUL;
        sw_timebase += (time_t)((round_ticks >> 15) + sw_rpl);
    }
}

// interput handle
int rda_timer_isr(int irq, FAR void *context, FAR void *arg)
{
	uint32_t regval;

    uint32_t int_status = getreg32(RDA_TIMER_INTSTATE);
	int_status &= 0x000FUL;
	
    if(int_status & (0x01UL << 2)) {
        us_ticker_irq_handler();
    }

    if (int_status & (0x01UL << 3)) {
#if DEVICE_LOWPOWERTIMER
        lp_ticker_irq_handler();
#endif /* DEVICE_LOWPOWERTIMER */
    }
    if(int_status & (0x01UL << 1)) 
	{
		regval = getreg32(RDA_POWER_CONTROL);
		regval |= ((0x01UL << 28) | (0x01UL << 27)); // clear int & ts
		putreg32(regval,RDA_POWER_CONTROL);
        //__DSB();
        do{
			regval = getreg32(RDA_POWER_CONTROL);
		}while(regval & (0x01UL << 28));

        rtc_base_update();
    }
	return 0;
}

void rda_timer_irq_set(void)
{
	int ret = 0;
    
    if(0 == is_timer_irq_set) {
        is_timer_irq_set = 1;
			/* Attach the timer interrupt vector */
		irq_attach(RDA_IRQ_TIMER, rda_timer_isr, NULL);	
		if (ret == OK)
		{
			up_enable_irq(RDA_IRQ_TIMER);
		}
    }
}


time_t rtc_read(void) {
    /* Get hw timestamp in seconds, ">>15" equals "/RTC_TIMER_CLOCK_SOURCE" (="/32768") */
    time_t hw_ts = (time_t)((getreg32(RTC_TIMER_TIMESTAMP) + remain_ticks) >> 15);
    /* Calculate current timestamp */
    time_t t = sw_timebase + hw_ts - sw_timeofst;
    return t;
}

void rtc_write(time_t t) {
    /* Get hw timestamp in seconds */
    uint32_t rtc_cur_ticks = getreg32(RTC_TIMER_TIMESTAMP);
    uint32_t rtc_rpl_ticks = (rtc_cur_ticks + remain_ticks) & 0x00007FFFUL;
    uint32_t sw_rpl = 0U;
    time_t hw_ts = (time_t)((rtc_cur_ticks + remain_ticks) >> 15);
    /* Set remaining ticks */
    remain_ticks += rtc_rpl_ticks;
    sw_rpl = remain_ticks >> 15;
    remain_ticks &= 0x00007FFFUL;
    /* Set sw timestamp in seconds */
    if(t < hw_ts) {
        sw_timebase = sw_rpl;
        sw_timeofst = hw_ts - t;
    } else {
        sw_timebase = sw_rpl + t - hw_ts;
        sw_timeofst = 0U;
    }
}


/****************************************************************************
 * Name: up_rtc_initialize
 *
 * Description:
 *   Initialize the hardware RTC per the selected configuration. This
 *   function is called once during the OS initialization sequence.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/
int up_rtc_initialize(void)
{
    uint32_t start_time;
    /* Make sure us_ticker is running */
    start_time = us_ticker_read();
    /* To fix compiling warning */
    start_time = start_time;
    /* Record the ticks */
    round_ticks = RTC_TIMER_INITVAL_REG;
    is_rtc_enabled = 1;
	
	return OK;
}
#endif /* CONFIG_RTC */
