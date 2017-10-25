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
 * arch/arm/src/rad5981/rad5981_rtc_lowhalf.c
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

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#include <tinyara/arch.h>
#include <tinyara/rtc.h>

#include "up_arch.h"

#include "rda5981x_rtc.h"

#ifdef CONFIG_RTC_DRIVER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/
#ifdef CONFIG_RTC_ALARM
struct rda5981x_cbinfo_s {
	volatile rtc_alarm_callback_t cb; /* Callback when the alarm expires */
	volatile FAR void *priv;          /* Private arg passed to callback */
};
#endif /* CONFIG_RTC_ALARM */

struct rda5981x_lowerhalf_s {
	/*
	 * This is the contained reference to the read-only, lower-half
	 * operations vtable (which may lie in FLASH or ROM
	 */
	FAR const struct rtc_ops_s *ops;

#ifdef CONFIG_RTC_ALARM
	/*
	 * Data following is private to this driver and not visible
	 * outside of this file.
	 */
	struct rda5981x_cbinfo_s cbinfo;
#endif /* CONFIG_RTC_ALARM */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rda5981x_alarm_callback
 *
 * Description:
 *   This is the function that is called from the RTC driver when the alarm
 *   goes off. It just invokes the upper half drivers callback.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
static int rtc_rdtime(FAR struct rtc_lowerhalf_s *lower,
		      FAR struct rtc_time *rtctime)
{
	return up_rtc_getdatetime((FAR struct tm *)rtctime);
}

static int rtc_settime(FAR struct rtc_lowerhalf_s *lower,
		       FAR const struct rtc_time *rtctime)
{
	return up_rtc_setdatetime((FAR struct tm *)rtctime);
}

/****************************************************************************
 * Private Data
 ****************************************************************************/
static const struct rtc_ops_s g_rtc_ops = {
	.rdtime      = rtc_rdtime,
	.settime     = rtc_settime,
#ifdef CONFIG_RTC_ALARM
	.setalarm    = NULL,
	.setrelative = NULL,
	.cancelalarm = NULL,
#endif
#ifdef CONFIG_RTC_IOCTL
	.ioctl       = NULL,
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
	.destroy     = NULL,
#endif
};

static struct rda5981x_lowerhalf_s g_rtc_lowerhalf = {
	.ops = &g_rtc_ops,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * Name: rda5981x_rtc_lowerhalf
 *
 * Description:
 *   Instantiate the RTC lower half driver for the S5J.
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
FAR struct rtc_lowerhalf_s *rda5981x_rtc_lowerhalf(void)
{
	return (FAR struct rtc_lowerhalf_s *)&g_rtc_lowerhalf;
}
#endif /* CONFIG_RTC_DRIVER */
