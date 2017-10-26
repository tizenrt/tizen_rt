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
#include "rda5981x_ticker.h"
#include "rda5981x_rtc.h"

static int32_t  us_ticker_inited = 0;
static ticker_event_queue_t events;

extern uint32_t SystemCoreClock;

#if RDA5991H_HW_VER <= 4
extern unsigned int lpo_ticks_cal;
extern unsigned int lpo_tmr_flag;
extern signed int   lpo_delta_us;
#endif


void us_ticker_irq_handler(void);


void us_ticker_init(void) 
{
    uint32_t regval;
    if (us_ticker_inited)
        return;

    /* Enable apb timer clock */
	regval = getreg32(RDA_CLKGATE1); // CLKGATE1
	regval |= (0x01UL << 3);
	putreg32(regval, RDA_CLKGATE1);

    /* Set timer count */
	putreg32(0xFFFFFFFFUL, RDA_TIMER0_LDCNT);

    /* Set timer mode */
	regval = getreg32(RDA_TIMER0_TCTRL); // CLKGATE1
	regval |= TIMER0_CONTROL_MODE;
	putreg32(regval, RDA_TIMER0_TCTRL);

    /* Set free timer power */
    rda_ccfg_aontmr();

    /* Delay 300us at least */
    {
        unsigned int idx = (SystemCoreClock / US_TIMER_HZ / 4UL) * 300UL;
        regval = 0U;
        while(idx--) {
            regval += getreg32(RTC_TIMER_TIMESTAMP);
        }
    }

    /* Set free timer write_en */
	regval = getreg32(RDA_POWER_CONTROL); // CLKGATE1
	regval |= (uint32_t)(0x01UL << 8);
	putreg32(regval, RDA_POWER_CONTROL);

    /* Set init value */
	regval = getreg32(RTC_TIMER_INITVAL_REG);
#if RDA5991H_HW_VER <= 4
    if((0U != lpo_tmr_flag) && (0U != lpo_ticks_cal)) 
	{
        regval = (uint32_t)((uint64_t)FREE_TIMER_INIT_VAL * lpo_ticks_cal / FREE_TIMER_CLOCK_SOURCE);
    } else {
        regval = FREE_TIMER_INIT_VAL;
    }
#else
    regval = FREE_TIMER_INIT_VAL;
#endif

	putreg32(regval, RTC_TIMER_INITVAL_REG);

    /* Enable int */
	regval = getreg32(RDA_TIMER1_TCTRL);
	regval |= (0x01UL << 4);
	putreg32(regval, RDA_TIMER1_TCTRL);

	regval = getreg32(RDA_POWER_CONTROL);
	regval |= (0x01UL << 18);
	putreg32(regval, RDA_POWER_CONTROL);


    /* Unmask int */
	regval = getreg32(RDA_TIMER1_TCTRL);
	regval  &= ~(0x01UL << 3);
	putreg32(regval, RDA_TIMER1_TCTRL);

    /* Set free timer prescale */
	putreg32(0U, RTC_TIMER_PRESCALE);

    /* Enable free timer */	
	regval = getreg32(RDA_POWER_CONTROL);
	regval |= (uint32_t)(0x01UL << 17);
	putreg32(regval, RDA_POWER_CONTROL);

    /* Clr free timer write_en */
	regval = getreg32(RDA_POWER_CONTROL);
	regval &= (~(uint32_t)(1 << 8));
	putreg32(regval, RDA_POWER_CONTROL);

    rda_timer_irq_set();

    /* Set us_ticker_inited true, after all settings done */
    us_ticker_inited = 1U;
}


uint32_t us_ticker_read(void) 
{
    if (!us_ticker_inited)
        us_ticker_init();
	
	uint32_t regval = getreg32(RTC_TIMER_TIMESTAMP);

#if RDA5991H_HW_VER <= 4
		if((0U != lpo_tmr_flag) && (0U != lpo_ticks_cal)) {
			return (uint32_t)((((int64_t)regval) * (int64_t)US_TIMER_HZ) / (int64_t)lpo_ticks_cal + (int64_t)lpo_delta_us);
		} else {
			return (uint32_t)((((uint64_t)regval) * US_TIMER_HZ) >> 15);
		}
#else
		/* ">>15" gets the same result as "/FREE_TIMER_CLOCK_SOURCE" (="/32768") */
		return (uint32_t)((((uint64_t)regval) * US_TIMER_HZ) >> 15);
#endif

}


void us_ticker_set_interrupt(timestamp_t timestamp) 
{
    uint32_t regval;
    uint32_t ftmr_ticks = 0U;
    int32_t delta = (int32_t)(timestamp - us_ticker_read());
    if (delta <= 0) {
        // This event was in the past:
        us_ticker_irq_handler();
        return;
    }

    /* Disable timer */
	regval = getreg32(RDA_TIMER0_TCTRL);
	regval &= (~TIMER0_CONTROL_ENABLE);
	putreg32(regval, RDA_TIMER0_TCTRL);

    /* Convert the delta to the times of US_TIMER_HZ/FREE_TIMER_CLOCK_SOURCE */
    ftmr_ticks = (uint32_t)(((uint64_t)delta << 15) / US_TIMER_HZ);
    delta = (uint32_t)((((uint64_t)ftmr_ticks) * US_TIMER_HZ) >> 15);
	putreg32(delta * (US_TIMER_CLOCK_SOURCE / US_TIMER_HZ), RDA_TIMER0_LDCNT);

    /* Enable timer */
	regval = getreg32(RDA_TIMER0_TCTRL);
	regval |= (TIMER0_CONTROL_ENABLE);
	putreg32(regval, RDA_TIMER0_TCTRL);

    /* Unmask timer, enable an overflow int */
	regval = getreg32(RDA_TIMER0_TCTRL);
	regval &= (~(TIMER0_CONTROL_INT_MSK));
	putreg32(regval, RDA_TIMER0_TCTRL);
}

void us_ticker_disable_interrupt(void)
{
    uint32_t regval;

    /* Mask timer, disable an overflow int */
	regval = getreg32(RDA_TIMER0_TCTRL);
	regval |= (TIMER0_CONTROL_INT_MSK);
	putreg32(regval, RDA_TIMER0_TCTRL);
}

void us_ticker_clear_interrupt(void) 
{
    volatile uint32_t temp = getreg32(RDA_TIMER0_INTCLR);

    /* To fix compiling warning */
    temp = temp;
}


//Ticker interfacecs
static const ticker_interface_t us_interface = {
    .init = us_ticker_init,
    .read = us_ticker_read,
    .disable_interrupt = us_ticker_disable_interrupt,
    .clear_interrupt = us_ticker_clear_interrupt,
    .set_interrupt = us_ticker_set_interrupt,
};

static const ticker_data_t us_data = {
    .interface = &us_interface,
    .queue = &events,
};

const ticker_data_t* get_us_ticker_data(void)
{
    return &us_data;
}


void us_ticker_irq_handler(void)
{
    ticker_irq_handler(&us_data);
}


