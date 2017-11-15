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
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>

#include "tinyara/clock.h"
#include "tinyara/arch.h"
#include "tinyara/cancelpt.h"
#include "tinyara/kthread.h"
#include "tinyara/irq.h"
#include "sys/types.h"
#include "sys_wrapper.h"

#define NO_ERR  0
#define ERR     -1

#define RDA_SYS_DEBUG
#ifdef RDA_SYS_DEBUG
#define RDA_SYS_PRINT(fmt, ...) do {\
        lldbg(fmt, ##__VA_ARGS__);  \
    } while (0)
#else
#define RDA_SYS_PRINT(fmt, ...)
#endif

#define CONFIG_DISABLE_ALL_INT
#define CRI_SEC_START_PRI_LEVEL     0xF8
#define CRI_SEC_END_PRI_LEVEL       0x00
static unsigned int g_critical_sec_counter = 0U;
static unsigned int g_critical_ctxt_saved  = 0U;
/*
* Interrupt
*/
int std_interrupt_create(unsigned int vec, unsigned int pri, rda_isr_fn isr)
{
    int ret;
    int irq = RDA_IRQ_EXTINT + (int)vec;
    if(!isr || (NR_IRQS < irq)) {
        return -1;
    }
    ret = irq_attach(irq, (xcpt_t)isr, NULL);
    if(ret) {
        return -2;
    }

    ret = up_prioritize_irq(irq, (int)pri);
    if(ret) {
        return -3;
    }
    return 0;
}

int std_interrupt_delete(unsigned int vec)
{
    int ret;
    int irq = RDA_IRQ_EXTINT + (int)vec;
    if(NR_IRQS < irq) {
        return -1;
    }
    ret = irq_attach(irq, NULL, NULL);
    if(ret) {
        return -2;
    }
    return 0;
}

int std_interrupt_enable(unsigned int vec)
{
    int irq = RDA_IRQ_EXTINT + (int)vec;
    if(NR_IRQS < irq) {
        return -1;
    }
    up_enable_irq(irq);
    return 0;
}

int std_interrupt_disable(unsigned int vec)
{
    int irq = RDA_IRQ_EXTINT + (int)vec;
    if(NR_IRQS < irq) {
        return -1;
    }
    up_disable_irq(irq);
    return 0;
}

unsigned int std_irqstate_get(void)
{
#if defined(CONFIG_DISABLE_ALL_INT)
    return (unsigned int)irqsave();
#else  /* CONFIG_DISABLE_ALL_INT */
    unsigned int st = (unsigned int)getbasepri();
    setbasepri(CRI_SEC_START_PRI_LEVEL);
    return st;
#endif /* CONFIG_DISABLE_ALL_INT */
}

void std_irqstate_set(unsigned int st)
{
#if defined(CONFIG_DISABLE_ALL_INT)
    irqrestore((irqstate_t)st);
#else  /* CONFIG_DISABLE_ALL_INT */
    setbasepri(st);
#endif /* CONFIG_DISABLE_ALL_INT */
}

void * rda_interrupt_create(unsigned int vec, unsigned int pri, void *isr)
{
    int ret;
    ret = std_interrupt_create(vec, pri, (rda_isr_fn)isr);
    if(ret) {
        RDA_SYS_PRINT("int create err:%d\r\n", ret);
        return 0;
    }
    return isr;
}

void rda_interrupt_delete(unsigned int vec)
{
    int ret;
    ret = std_interrupt_delete(vec);
    if(ret) {
        RDA_SYS_PRINT("int delete err:%d\r\n", ret);
    }
}

void rda_interrupt_enable(unsigned int vec)
{
    int ret;
    ret = std_interrupt_enable(vec);
    if(ret) {
        RDA_SYS_PRINT("int en err:%d\r\n", ret);
    }
}

void rda_interrupt_disable(unsigned int vec)
{
    int ret;
    ret = std_interrupt_disable(vec);
    if(ret) {
        RDA_SYS_PRINT("int dis err:%d\r\n", ret);
    }
}

void rda_critical_sec_start(void)
{
    if(getipsr() == 0U) {
        if(0U == g_critical_sec_counter) {
            g_critical_ctxt_saved = std_irqstate_get();
        }
        g_critical_sec_counter++;
    }
}

void rda_critical_sec_end(void)
{
    if(getipsr() == 0U) {
        g_critical_sec_counter--;
        if(0U == g_critical_sec_counter) {
            std_irqstate_set(g_critical_ctxt_saved);
        }
    }
}

/*
* Timer
*/
unsigned int std_curtime_get_ms(void)
{
    return TICK2MSEC(clock_systimer());
}

void rda_timer_generic_callback(int argc, unsigned int arg)
{
    if(arg) {
        rda_tmr_t *tmr = (rda_tmr_t *)arg;
        tmr->func(tmr->arg);
    }
}

int std_timer_create(rda_tmr_t *tmr, tmr_cb_t func, unsigned int arg)
{
    WDOG_ID wdg_new;
    if((!tmr) || (!func)) {
        return -1;
    }
    wdg_new = wd_create();
    if(!wdg_new) {
        return -2;
    }
    tmr->wdg = wdg_new;
    tmr->func = func;
    tmr->arg = arg;
    return 0;
}

int std_timer_delete(rda_tmr_t *tmr)
{
    if(!tmr) {
        return -1;
    }
    return wd_delete(tmr->wdg);
}

int std_timer_start(rda_tmr_t *tmr, unsigned int to_ms)
{
    int ticks;
    if(!tmr) {
        return -1;
    }
    ticks = (int)((to_ms + MSEC_PER_TICK - 1) / MSEC_PER_TICK);
    return wd_start(tmr->wdg, ticks, (wdentry_t)rda_timer_generic_callback, 1, (unsigned int)tmr);
}

int std_timer_stop(rda_tmr_t *tmr)
{
    if(!tmr) {
        return -1;
    }
    return wd_cancel(tmr->wdg);
}

void * rda_timer_create(void *func, unsigned int arg)
{
    rda_tmr_t *tmr;
    tmr = (rda_tmr_t *)malloc(sizeof(rda_tmr_t));
    if(!tmr) {
        RDA_SYS_PRINT("tmr create, mem err\r\n");
        return 0;
    }
    if(std_timer_create(tmr, (tmr_cb_t)func, arg)) {
        RDA_SYS_PRINT("tmr create err\r\n");
        free((void *)tmr);
        return 0;
    }
    return (void *)tmr;
}

void rda_timer_delete(void **tmr)
{
    if(std_timer_delete(*((rda_tmr_t **)tmr))) {
        RDA_SYS_PRINT("tmr delete err\r\n");
        return;
    }
    free(*tmr);
    *tmr = 0;
}

void rda_timer_start(void *tmr, unsigned int to_ms)
{
    if(std_timer_start((rda_tmr_t *)tmr, to_ms)) {
        RDA_SYS_PRINT("tmr start err\r\n");
    }
}

void rda_timer_stop(void *tmr)
{
    if(std_timer_stop((rda_tmr_t *)tmr)) {
        RDA_SYS_PRINT("tmr stop err\r\n");
    }
}

/*
* Semaphore
*/
int std_semaphore_create(rda_sem_t *sem, unsigned char cnt)
{
    if(!sem) {
        return -1;
    }
    if(sem_init(sem, 0, cnt)) {
        return -2;
    }
    if(!cnt) {
        if(sem_setprotocol(sem, SEM_PRIO_NONE)) {
            return -3;
        }
    }
    return 0;
}

int std_semaphore_delete(rda_sem_t *sem)
{
    if(!sem) {
        return -1;
    }
    return sem_destroy(sem);
}

int std_semaphore_wait(rda_sem_t *sem, unsigned int to_ms)
{
    systime_t start = clock_systimer();
    if(!sem) {
        return -1;
    }
    if(RDA_TMR_WAIT_FOREVER == to_ms) {
        while(sem_wait(sem) != OK) {
            int err = get_errno();
            if(EINTR == err) { // wait again if signal is EINTR
                continue;
            } else if(ECANCELED == err) {
                return -2; // RDA_SEM_CANCELED
            }
            return -3;
        }
    } else {
        while(sem_tickwait(sem, clock_systimer(), MSEC2TICK(to_ms)) != OK) {
            int err = get_errno();
            if(ETIMEDOUT == err) {
                return -1; // RDA_SEM_TIMEOUT 1 OR -1?
            } else if(ECANCELED == err) {
                return -4;
            }
            to_ms -= TICK2MSEC(clock_systimer() - start);
        }
    }
    return 0;
}

int std_semaphore_post(rda_sem_t *sem)
{
    if(!sem) {
        return -1;
    }
    return sem_post(sem);
}

/*
* Mutex
*/
int rda_mutex_create(rda_mutex_t *mtx)
{
    if(!mtx) {
        return -1;
    }
    return pthread_mutex_init(mtx, NULL);
}

int rda_mutex_delete(rda_mutex_t *mtx)
{
    if(!mtx) {
        return -1;
    }
    return pthread_mutex_destroy(mtx);
}

int rda_mutex_lock(rda_mutex_t *mtx)
{
    if(!mtx) {
        return -1;
    }
    return pthread_mutex_lock(mtx);
}

int rda_mutex_unlock(rda_mutex_t *mtx)
{
    if(!mtx) {
        return -1;
    }
    return pthread_mutex_unlock(mtx);
}




/*
* Message Queue Begin
*/
#define RDA_SEM_TIMEOUT  0xffffffffUL // -1
#define RDA_SEM_CANCELED 0xfffffffeUL // -2
static int g_mbox_id = 0;

int std_msgqueue_create(rda_msgq_t *mq, unsigned int mq_sz)
{
    mq->is_valid = 1;
    mq->id = g_mbox_id++;
    mq->queue_size = mq_sz;
    mq->wait_send = 0;
    mq->wait_fetch = 0;
    mq->front = mq->rear = 0;
    std_semaphore_create(&(mq->mail), 0);
    std_semaphore_create(&(mq->mutex), 1);

//#if SYS_STATS
//  SYS_STATS_INC_USED(mq);
//#endif                            /* SYS_STATS */

    RDA_SYS_PRINT("Succesfully Created MBOX with id %d\n", mq->id);

    return 0;
}

int std_msgqueue_delete(rda_msgq_t *mq)
{
    if (mq != NULL) {

        RDA_SYS_PRINT("Deleting MBOX with id %d", mq->id);

        mq->is_valid = 0;
        mq->id = 0;
        mq->queue_size = 0;
        mq->wait_send = 0;
        mq->wait_fetch = 0;
        std_semaphore_delete(&(mq->mail));
        std_semaphore_delete(&(mq->mutex));

        RDA_SYS_PRINT("Succesfully deleted MBOX with id %d", mq->id);
//#if SYS_STATS
//      SYS_STATS_DEC(mq.used);
//#endif                            /* SYS_STATS */
    }
    
    return 0;
}
int std_msgqueue_get(rda_msgq_t *mq, void **msg, unsigned int to_ms)
{
    uint32_t time = 0;
    uint32_t status = OK;
    
     /* The mutex lock is quick so we don't bother with the timeout
       stuff here. */
    status = std_semaphore_wait(&(mq->mutex), 0);
    if (status == RDA_SEM_CANCELED) {
        return RDA_SEM_CANCELED;
    }

    /* wait while the queue is empty */
    while (mq->front == mq->rear) {
        mq->wait_fetch++;
        std_semaphore_post(&(mq->mutex));

        /* We block while waiting for a mail to arrive in the mailbox. We
           must be prepared to timeout. */
        if (to_ms != 0) {
            time = std_semaphore_wait(&(mq->mail), to_ms);

            if (time == RDA_SEM_TIMEOUT) {
                std_semaphore_wait(&(mq->mutex), 0);
                mq->wait_fetch--;
                std_semaphore_post(&(mq->mutex));
                return RDA_SEM_TIMEOUT;
            }
        } else {
            std_semaphore_wait(&(mq->mail), 0);
        }

        status = std_semaphore_wait(&(mq->mutex), 0);
        mq->wait_fetch--;
        if (status == RDA_SEM_CANCELED) {
            return RDA_SEM_CANCELED;
        }       
    }

    mq->front = (mq->front + 1) % mq->queue_size;
    if (msg != NULL) {
        *msg = mq->msgs[mq->front];
        RDA_SYS_PRINT(" mbox %p msg %p\n", (void *)mq, *msg);
        printf(" mbox %p msg %p\n", (void *)mq, *msg);
    } else {
        RDA_SYS_PRINT(" mbox %p, null msg\n", (void *)mq);
        printf(" mbox %p, null msg\n", (void *)mq);
    }

    /* We just fetched a msg, Release semaphore for
       some post api blocked on this sem due to queue full. */
    if (mq->wait_send) {
        std_semaphore_post(&(mq->mail));
    }

    std_semaphore_post(&(mq->mutex));

    return time;
}

int std_msgqueue_put(rda_msgq_t *mq, void *msg)
{
    uint8_t first_msg = 0;
    uint32_t tmp = 0;
    uint32_t status = OK;
    
    status = std_semaphore_wait(&(mq->mutex), 0);
    if (status == RDA_SEM_CANCELED) {
        return RDA_SEM_CANCELED;
    }
    RDA_SYS_PRINT("mbox %p msg %p\n", (void *)mq, (void *)msg);
    /* Wait while the queue is full */
    tmp = (mq->rear + 1) % mq->queue_size;
    if (tmp == mq->front) {
        RDA_SYS_PRINT("Queue Full, Wait until gets free\n");
    }
    while (tmp == mq->front) {
        mq->wait_send++;
        std_semaphore_post(&(mq->mutex));
        std_semaphore_wait(&(mq->mail), 0);
        status = std_semaphore_wait(&(mq->mutex), 0);
        mq->wait_send--;
        if (status == RDA_SEM_CANCELED) {
            return RDA_SEM_CANCELED;
        }
    }

    if (mq->rear == mq->front) {
        first_msg = 1;
    } else {
        first_msg = 0;
    }

    mq->rear = tmp;
    mq->msgs[mq->rear] = msg;
    RDA_SYS_PRINT("Post SUCCESS\n");

    /* If msg was posted to an empty queue, Release semaphore for
       some fetch api blocked on this sem due to Empty queue. */
    if (first_msg && mq->wait_fetch) {
        std_semaphore_post(&(mq->mail));
    }

    std_semaphore_post(&(mq->mutex));

    return 0;
}
/*
* Message Queue End
*/



/*
* Thread Begin
*/
int std_thread_create(rda_thread_t *pid, rda_thread_fn entry_func, void *arg, unsigned int stk_sz, int pri)
{
    int ret;
    pthread_attr_t attr;
    struct sched_param param;
    if(!pid || !entry_func) {
        return -1;
    }
    ret = pthread_attr_init(&attr);
    if(ret) {
        return -2;
    }
    attr.stacksize = stk_sz;
    param.sched_priority = pri;
    ret = pthread_attr_setschedparam(&attr, &param);
    if(ret) {
        return -3;
    }
    return pthread_create(pid, &attr, (pthread_startroutine_t)entry_func, arg);
}
#if 0
rda_thread_t std_thread_create(const char *name, rda_thread_fn entry_func, void *arg, int stksz, int pri)
{
    sys_thread_t new_thread = task_create(name, pri, stksz, (main_t)entry_func, (char * const *)NULL);
    if(new_thread < 0) {
        int err = get_errno();
        return -err;
    }
    return new_thread;
}
#endif

int std_thread_delete(rda_thread_t *pid)
{
    int ret;
    void *result;
    if(!pid) {
        return -1;
    }
    pthread_cancel(*pid);
    ret = pthread_join(*pid, &result);
    if(!ret && (result == PTHREAD_CANCELED)) {
        *pid = 0;
    }
    return ret;
}
/*
* Thread End
*/



