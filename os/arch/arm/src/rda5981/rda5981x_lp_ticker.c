/****************************************************************************
 * arch/arm/src/rda5981x/rda5981x_lowputc.c
 *
 *   Copyright (C) 2010-2013 Gregory Nutt. All rights reserved.
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



#include "rda5981x_ticker.h"
#include "rda5981x_rtc.h"

#if DEVICE_LOWPOWERTIMER

#define TIMER1_CONTROL_ENABLE       (0x20)
#define TIMER1_CONTROL_MODE         (0x40)
#define TIMER1_CONTROL_INT_MSK      (0x80)

#define LP_TIMER_CLOCK_SOURCE       (32768)
#define US_TIMER_HZ                 (1000000)

static int32_t  lp_ticker_inited = 0;
extern uint32_t SystemCoreClock;


#if RDA5991H_HW_VER <= 4
extern unsigned int lpo_ticks_cal;
extern unsigned int lpo_tmr_flag;
extern signed int   lpo_delta_us;
#endif

extern void rda_timer_irq_set(void);


#define RDA_TIMER1_TCTRL       (RDA_TIM1_BASE+0x00)
#define RDA_TIMER1_LDCNT       (RDA_TIM1_BASE+0x0c)
#define RDA_TIMER1_CVAL        (RDA_TIM1_BASE+0x04)
#define RDA_TIMER1_INTCLR      (RDA_TIM1_BASE+0x18)



void lp_ticker_init(void)
{
	uint32_t regval;

	if (lp_ticker_inited) {
		return;
	}

	/* Enable apb timer clock */
	regval = getreg32(RDA_CLKGATE1);
	regval |= (0x01UL << 3);
	putreg32(regval, RDA_CLKGATE1);

	/* Set timer load count */
	putreg32(0xFFFFFFFF, RDA_TIMER1_LDCNT);

	/* Set timer mode */
	regval = getreg32(RDA_TIMER1_TCTRL);
	regval |= TIMER1_CONTROL_MODE;
	putreg32(regval, RDA_TIMER1_TCTRL);

	/* Enable timer */
	regval = getreg32(RDA_TIMER1_TCTRL);
	regval |= (TIMER1_CONTROL_ENABLE);
	putreg32(regval, RDA_TIMER1_TCTRL);

	rda_timer_irq_set();

	/* Set lp_ticker_inited true, after all settings done */
	lp_ticker_inited = 1;
}

uint32_t lp_ticker_read(void)
{
	if (!lp_ticker_inited) {
		lp_ticker_init();
	}

	/* Get timestamp from us_ticker */
	return us_ticker_read();
}

void lp_ticker_set_interrupt(timestamp_t timestamp)
{
	uint32_t regval;
	int32_t delta = (int32_t)(timestamp - lp_ticker_read());
	uint32_t delay_ticks = SystemCoreClock / LP_TIMER_CLOCK_SOURCE / 5;
	if (delta <= 0) {
		// This event was in the past:
		lp_ticker_irq_handler();
		return;
	}

	/* Disable timer */
	regval = getreg32(RDA_TIMER1_TCTRL);
	regval &= (~TIMER1_CONTROL_ENABLE);
	putreg32(regval, RDA_TIMER1_TCTRL);

	/* Set timer load count */
#if RDA5991H_HW_VER <= 4
	if ((0U != lpo_tmr_flag) && (0U != lpo_ticks_cal)) {
		//LP_TICKER_TIMER->LDCNT = (uint32_t)(((uint64_t)delta * (uint64_t)lpo_ticks_cal) / US_TIMER_HZ) + 1;
		regval = (uint32_t)(((uint64_t)delta * (uint64_t)lpo_ticks_cal) / US_TIMER_HZ) + 1;
		putreg32(regval, RDA_TIMER1_LDCNT);
	} else {
		regval = (uint32_t)(((uint64_t)delta << 15) / US_TIMER_HZ) + 1;
		putreg32(regval, RDA_TIMER1_LDCNT);
	}
#else
	regval = (uint32_t)(((uint64_t)delta << 15) / US_TIMER_HZ) + 1;
	putreg32(regval, RDA_TIMER1_LDCNT);
#endif

	/* Delay for Clock Sync */
	for (; delay_ticks > 1; delay_ticks--);

	/* Enable timer */
	regval = getreg32(RDA_TIMER1_TCTRL);
	regval |= (TIMER1_CONTROL_ENABLE);
	putreg32(regval, RDA_TIMER1_TCTRL);

	/* Unmask timer, enable an overflow int */
	regval = getreg32(RDA_TIMER1_TCTRL);
	regval &= (~(TIMER1_CONTROL_INT_MSK));
	putreg32(regval, RDA_TIMER1_TCTRL);
}

void lp_ticker_disable_interrupt(void)
{
	/* Mask timer, disable an overflow int */
	LP_TICKER_TIMER->TCTRL |= (TIMER1_CONTROL_INT_MSK);
}

void lp_ticker_clear_interrupt(void)
{
	volatile uint32_t temp = LP_TICKER_TIMER->INTCLR;

	/* To fix compiling warning */
	temp = temp;

	/* Disable timer */
	LP_TICKER_TIMER->TCTRL &= (~TIMER1_CONTROL_ENABLE);
}




static ticker_event_queue_t events;

static const ticker_interface_t lp_interface = {
	.init = lp_ticker_init,
	.read = lp_ticker_read,
	.disable_interrupt = lp_ticker_disable_interrupt,
	.clear_interrupt = lp_ticker_clear_interrupt,
	.set_interrupt = lp_ticker_set_interrupt,
};

static const ticker_data_t lp_data = {
	.interface = &lp_interface,
	.queue = &events,
};

const ticker_data_t *get_lp_ticker_data(void)
{
	return &lp_data;
}

void lp_ticker_irq_handler(void)
{
	ticker_irq_handler(&lp_data);
}

#endif

