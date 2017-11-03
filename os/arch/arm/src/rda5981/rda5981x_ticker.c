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
#include "rda5981x_ticker.h"

//Define rda ticker pai
void ticker_set_handler(const ticker_data_t *const data, ticker_event_handler handler)
{
	data->interface->init();

	data->queue->event_handler = handler;
}

void ticker_irq_handler(const ticker_data_t *const data)
{
	data->interface->clear_interrupt();

	/* Go through all the pending TimerEvents */
	while (1) {
		if (data->queue->head == NULL) {
			// There are no more TimerEvents left, so disable matches.
			data->interface->disable_interrupt();
			return;
		}

		if ((int)(data->queue->head->timestamp - data->interface->read()) <= 0) {
			// This event was in the past:
			//      point to the following one and execute its handler
			ticker_event_t *p = data->queue->head;
			data->queue->head = data->queue->head->next;
			if (data->queue->event_handler != NULL) {
				(*data->queue->event_handler)(p->id); // NOTE: the handler can set new events
			}
			/* Note: We continue back to examining the head because calling the
			 * event handler may have altered the chain of pending events. */
		} else {
			// This event and the following ones in the list are in the future:
			//      set it as next interrupt and return
			data->interface->set_interrupt(data->queue->head->timestamp);
			return;
		}
	}
}

void ticker_insert_event(const ticker_data_t *const data, ticker_event_t *obj, timestamp_t timestamp, uint32_t id)
{
	/* disable interrupts for the duration of the function */

	// initialise our data
	obj->timestamp = timestamp;
	obj->id = id;

	/* Go through the list until we either reach the end, or find
	   an element this should come before (which is possibly the
	   head). */
	ticker_event_t *prev = NULL, *p = data->queue->head;
	while (p != NULL) {
		/* check if we come before p */
		if ((int)(timestamp - p->timestamp) < 0) {
			break;
		}
		/* go to the next element */
		prev = p;
		p = p->next;
	}
	/* if prev is NULL we're at the head */
	if (prev == NULL) {
		data->queue->head = obj;
		data->interface->set_interrupt(timestamp);
	} else {
		prev->next = obj;
	}
	/* if we're at the end p will be NULL, which is correct */
	obj->next = p;

}

void ticker_remove_event(const ticker_data_t *const data, ticker_event_t *obj)
{
	// remove this object from the list
	if (data->queue->head == obj) {
		// first in the list, so just drop me
		data->queue->head = obj->next;
		if (data->queue->head == NULL) {
			data->interface->disable_interrupt();
		} else {
			data->interface->set_interrupt(data->queue->head->timestamp);
		}
	} else {
		// find the object before me, then drop me
		ticker_event_t *p = data->queue->head;
		while (p != NULL) {
			if (p->next == obj) {
				p->next = obj->next;
				break;
			}
			p = p->next;
		}
	}

}

timestamp_t ticker_read(const ticker_data_t *const data)
{
	return data->interface->read();
}

int ticker_get_next_timestamp(const ticker_data_t *const data, timestamp_t *timestamp)
{
	int ret = 0;

	/* if head is NULL, there are no pending events */
	if (data->queue->head != NULL) {
		*timestamp = data->queue->head->timestamp;
		ret = 1;
	}

	return ret;
}


