/************************************************************************************
 * arch/arm/src/rda5981x/rda5981x_ticker.h
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
#include <tinyara/config.h>
	 
#include <sys/types.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>


typedef uint32_t timestamp_t;

/** Ticker's event structure
 */
typedef struct ticker_event_s {
    timestamp_t            timestamp; /**< Event's timestamp */
    uint32_t               id;        /**< TimerEvent object */
    struct ticker_event_s *next;      /**< Next event in the queue */
} ticker_event_t;

typedef void (*ticker_event_handler)(uint32_t id);

/** Ticker's interface structure - required API for a ticker
 */
typedef struct {
    void (*init)(void);                           /**< Init function */
    uint32_t (*read)(void);                       /**< Read function */
    void (*disable_interrupt)(void);              /**< Disable interrupt function */
    void (*clear_interrupt)(void);                /**< Clear interrupt function */
    void (*set_interrupt)(timestamp_t timestamp); /**< Set interrupt function */
} ticker_interface_t;

/** Ticker's event queue structure
 */
typedef struct {
    ticker_event_handler event_handler; /**< Event handler */
    ticker_event_t *head;               /**< A pointer to head */
} ticker_event_queue_t;

/** Ticker's data structure
 */
typedef struct {
    const ticker_interface_t *interface; /**< Ticker's interface */
    ticker_event_queue_t *queue;         /**< Ticker's event queue */
} ticker_data_t;

