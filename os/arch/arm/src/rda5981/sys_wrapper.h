/* RDA Wi-Fi Library
 * Copyright (c) 2004-2017 RDA Microelectronics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef _RDA_SYS_WRAPPER_H_
#define _RDA_SYS_WRAPPER_H_

#include "tinyara/config.h"
#include "tinyara/wdog.h"
#include "sys/types.h"
#include "arch/irq.h"
#include "pthread.h"
#include "semaphore.h"
#include "mqueue.h"
#include "time.h"
#include "debug.h"

#define RDA_MBOX_MAXSIZE        (128)
#define RDA_TMR_WAIT_FOREVER    (0xFFFFFFFF)

typedef sem_t rda_sem_t;
typedef pthread_mutex_t rda_mutex_t;

struct rda_msgq {
    unsigned char id;
    unsigned char is_valid;
    unsigned int queue_size;
    unsigned int wait_send;
    unsigned int wait_fetch;
    unsigned int front;
    unsigned int rear;
    void *msgs[RDA_MBOX_MAXSIZE];
    rda_sem_t mail;
    rda_sem_t mutex;
};
typedef struct rda_msgq rda_msgq_t;

/* Function prototype for timer callback functions */
typedef void (*tmr_cb_t)(unsigned int arg);
struct rda_tmr {
    tmr_cb_t func;
    unsigned int arg;
    WDOG_ID  wdg;
};
typedef struct rda_tmr rda_tmr_t;

/* Function prototype for thread functions */
typedef void (*rda_thread_fn)(void *arg);
typedef pthread_t rda_thread_t;

/* Function prototype for ISR functions */
typedef void (*rda_isr_fn)(void);

#ifdef __cplusplus
extern "C" {
#endif

/*
* Interrupt
*/
/* Standard functions */
extern int std_interrupt_create (unsigned int vec, unsigned int pri, rda_isr_fn isr);
extern int std_interrupt_delete (unsigned int vec);
extern int std_interrupt_enable (unsigned int vec);
extern int std_interrupt_disable(unsigned int vec);
extern unsigned int std_irqstate_get(void);
extern void         std_irqstate_set(unsigned int st);
/* RDA style */
extern void * rda_interrupt_create (unsigned int vec, unsigned int pri, void *isr);
extern void   rda_interrupt_delete (unsigned int vec);
extern void   rda_interrupt_enable (unsigned int vec);
extern void   rda_interrupt_disable(unsigned int vec);
extern void   rda_critical_sec_start(void);
extern void   rda_critical_sec_end  (void);

/*
* Timer
*/
/* Standard functions */
extern unsigned int std_curtime_get_ms(void);
extern int std_timer_create(rda_tmr_t *tmr, tmr_cb_t func, unsigned int arg);
extern int std_timer_delete(rda_tmr_t *tmr);
extern int std_timer_start (rda_tmr_t *tmr, unsigned int to_ms);
extern int std_timer_stop  (rda_tmr_t *tmr);
/* RDA style */
#define rda_curtime_get_ms      std_curtime_get_ms
extern void * rda_timer_create(void *func, unsigned int arg);
extern void   rda_timer_delete(void **tmr);
extern void   rda_timer_start (void *tmr, unsigned int to_ms);
extern void   rda_timer_stop  (void *tmr);

/*
* Semaphore
*/
extern int std_semaphore_create(rda_sem_t *sem, unsigned char cnt);
extern int std_semaphore_delete(rda_sem_t *sem);
extern int std_semaphore_wait  (rda_sem_t *sem, unsigned int to_ms);
extern int std_semaphore_post  (rda_sem_t *sem);

/*
* Mutex
*/
extern int std_mutex_create(rda_mutex_t *mtx);
extern int std_mutex_delete(rda_mutex_t *mtx);
extern int std_mutex_lock  (rda_mutex_t *mtx);
extern int std_mutex_unlock(rda_mutex_t *mtx);

/*
* Message Queue
*/
extern int std_msgqueue_create(rda_msgq_t *mq, unsigned int mq_sz);
extern int std_msgqueue_delete(rda_msgq_t *mq);
extern int std_msgqueue_get   (rda_msgq_t *mq, void **msg, unsigned int to_ms);
extern int std_msgqueue_put   (rda_msgq_t *mq, void *msg);

/*
* Thread
*/
extern int std_thread_create(rda_thread_t *pid, rda_thread_fn entry_func, void *arg, unsigned int stk_sz, int pri);
extern int std_thread_delete(rda_thread_t *pid);

#endif /* _RDA_SYS_WRAPPER_H_ */

