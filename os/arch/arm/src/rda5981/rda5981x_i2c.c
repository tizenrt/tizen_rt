/****************************************************************************
 * arch/arm/src/rda5981xxx/rda5981x_i2c.c
 *
 *   Copyright (C) 2012, 2014-2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@tinyara.org>
 *
 *   Copyright (C) 2011 Li Zhuoyi. All rights reserved.
 *   Author: Li Zhuoyi <lzyy.cn@gmail.com> (Original author)
 *
 * Derived from arch/arm/src/lpc31xx/lpc31_i2c.c
 *
 *   Author: David Hewson
 *
 *   Copyright (C) 2010-2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@tinyara.org>
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <tinyara/arch.h>
#include <tinyara/semaphore.h>
#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"

#include "rda5981x_gpio.h"
#include "rda5981x_i2c.h"

#if defined(CONFIG_LPC17_I2C0) || defined(CONFIG_LPC17_I2C1) || defined(CONFIG_LPC17_I2C2)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef GPIO_I2C1_SCL
#  define GPIO_I2C1_SCL GPIO_I2C1_SCL_1
#  define GPIO_I2C1_SDA GPIO_I2C1_SDA_1
#endif

#ifndef CONFIG_LPC17_I2C0_FREQUENCY
#  define CONFIG_LPC17_I2C0_FREQUENCY 100000
#endif

#define I2C_TIMEOUT  (20 * 1000/CONFIG_USEC_PER_TICK) /* 20 mS */
#define LPC17_I2C1_FREQUENCY 400000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rda5981x_i2cdev_s
{
  struct i2c_dev_s  dev;     /* Generic I2C device */
  unsigned int     base;       /* Base address of registers */
  uint16_t         irqid;      /* IRQ for this device */

  sem_t            mutex;      /* Only one thread can access at a time */
  sem_t            wait;       /* Place to wait for state machine completion */
  volatile uint8_t state;      /* State of state machine */
  WDOG_ID          timeout;    /* Watchdog to timeout when bus hung */
  uint32_t         frequency;  /* Current I2C frequency */

  struct i2c_msg_s *msgs;      /* remaining transfers - first one is in progress */
  unsigned int     nmsg;       /* number of transfer remaining */

  uint16_t         wrcnt;      /* number of bytes sent to tx fifo */
  uint16_t         rdcnt;      /* number of bytes read from rx fifo */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  rda5981x_i2c_start(struct rda5981x_i2cdev_s *priv);
static void rda5981x_i2c_stop(struct rda5981x_i2cdev_s *priv);
static int  rda5981x_i2c_interrupt(int irq, FAR void *context, void *arg);
static void rda5981x_i2c_timeout(int argc, uint32_t arg, ...);
static void rda5981x_stopnext(struct rda5981x_i2cdev_s *priv);

/* I2C device operations */
static void rda5981x_i2c_setfrequency(struct i2c_dev_s *priv,
              uint32_t frequency);
static int  rda5981x_i2c_transfer(FAR struct i2c_dev_s *dev,
              FAR struct i2c_msg_s *msgs, int count);

int rda5981x_i2c_read(FAR struct i2c_dev_s *dev, FAR uint8_t *buffer, int buflen);
int rda5981x_i2c_write(FAR struct i2c_dev_s *dev, FAR const uint8_t *buffer, int buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_LPC17_I2C0
static struct rda5981x_i2cdev_s g_i2c0dev;
#endif
static const struct i2c_ops_s rda5981x_i2c_ops = {
	.setfrequency = rda5981x_i2c_setfrequency,
	.setaddress = rda5981x_i2c_setownaddress,
	.write = rda5981x_i2c_write,
	.read = rda5981x_i2c_read,
#ifdef CONFIG_I2C_TRANSFER
	.transfer = rda5981x_i2c_transfer,
#endif
#ifdef CONFIG_I2C_SLAVE
	.setownaddress = NULL,
	.registercallback = NULL,
#endif
};


/****************************************************************************
 * Name: rda5981x_i2c_setfrequency
 *
 * Description:
 *   Set the frequency for the next transfer
 *
 ****************************************************************************/

int rda5981x_i2c_read(FAR struct i2c_dev_s *dev, FAR uint8_t *buffer, int buflen)
{
	struct rda5981x_i2c_priv_s *priv = (struct rda5981x_i2c_priv_s *)dev;
	struct i2c_msg_s msg;
	unsigned int flags;

	/* 7- or 10-bit? */
	flags = (priv->addrlen == 10) ? I2C_M_TEN : 0;

	/* Setup for the transfer */
	msg.addr = priv->slave_addr, msg.flags = (flags | I2C_M_READ);
	msg.buffer = (FAR uint8_t *)buffer;
	msg.length = buflen;

	/*
	 * Then perform the transfer
	 *
	 * REVISIT:  The following two operations must become atomic in order to
	 * assure thread safety.
	 */

	return rda5981x_i2c_transfer(dev, &msg, 1);
}

int rda5981x_i2c_write(FAR struct i2c_dev_s *dev, FAR const uint8_t *buffer,
				int buflen)
{
	struct rda5981x_i2c_priv_s *priv = (struct rda5981x_i2c_priv_s *)dev;
	struct i2c_msg_s msg;

	/* Setup for the transfer */
	msg.addr = priv->slave_addr;
	msg.flags = (priv->addrlen == 10) ? I2C_M_TEN : 0;
	msg.buffer = (FAR uint8_t *)buffer; /* Override const */
	msg.length = buflen;

	/*
	 * Then perform the transfer
	 *
	 * REVISIT:  The following two operations must become atomic in order to
	 * assure thread safety.
	 */
	return rda5981x_i2c_transfer(dev, &msg, 1);
}

static void rda5981x_i2c_setfrequency(struct rda5981x_i2cdev_s *priv,
                                   uint32_t frequency)
{
  if (frequency != priv->frequency)
    {
      if (frequency > 100000)
        {
          /* Asymetric per 400Khz I2C spec */

          putreg32(LPC17_CCLK / (83 + 47) * 47 / frequency,
                   priv->base + LPC17_I2C_SCLH_OFFSET);
          putreg32(LPC17_CCLK / (83 + 47) * 83 / frequency,
                   priv->base + LPC17_I2C_SCLL_OFFSET);
        }
      else
        {
          /* 50/50 mark space ratio */

          putreg32(LPC17_CCLK / 100 * 50 / frequency,
                   priv->base + LPC17_I2C_SCLH_OFFSET);
          putreg32(LPC17_CCLK / 100 * 50 / frequency,
                   priv->base + LPC17_I2C_SCLL_OFFSET);
        }

      priv->frequency = frequency;
    }
}

/****************************************************************************
 * Name: rda5981x_i2c_start
 *
 * Description:
 *   Perform a I2C transfer start
 *
 ****************************************************************************/

static int rda5981x_i2c_start(struct rda5981x_i2cdev_s *priv)
{
  putreg32(I2C_CONCLR_STAC | I2C_CONCLR_SIC,
           priv->base + LPC17_I2C_CONCLR_OFFSET);
  putreg32(I2C_CONSET_STA, priv->base + LPC17_I2C_CONSET_OFFSET);

  wd_start(priv->timeout, I2C_TIMEOUT, rda5981x_i2c_timeout, 1, (uint32_t)priv);
  sem_wait(&priv->wait);

  wd_cancel(priv->timeout);

  return priv->nmsg;
}

/****************************************************************************
 * Name: rda5981x_i2c_stop
 *
 * Description:
 *   Perform a I2C transfer stop
 *
 ****************************************************************************/

static void rda5981x_i2c_stop(struct rda5981x_i2cdev_s *priv)
{
  if (priv->state != 0x38)
    {
      putreg32(I2C_CONSET_STO | I2C_CONSET_AA,
               priv->base + LPC17_I2C_CONSET_OFFSET);
    }

  sem_post(&priv->wait);
}

/****************************************************************************
 * Name: rda5981x_i2c_timeout
 *
 * Description:
 *   Watchdog timer for timeout of I2C operation
 *
 ****************************************************************************/

static void rda5981x_i2c_timeout(int argc, uint32_t arg, ...)
{
  struct rda5981x_i2cdev_s *priv = (struct rda5981x_i2cdev_s *)arg;

  irqstate_t flags = enter_critical_section();
  priv->state = 0xff;
  sem_post(&priv->wait);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: rda5981x_i2c_transfer
 *
 * Description:
 *   Perform a sequence of I2C transfers
 *
 ****************************************************************************/

static int rda5981x_i2c_transfer(FAR struct i2c_dev_s *dev,
                              FAR struct i2c_msg_s *msgs, int count)
{
  struct rda5981x_i2cdev_s *priv = (struct rda5981x_i2cdev_s *)dev;
  int ret;

   DEBUGASSERT(dev != NULL && msgs != NULL && count > 0);

  /* Get exclusive access to the I2C bus */

  sem_wait(&priv->mutex);

  /* Set up for the transfer */

  priv->wrcnt = 0;
  priv->rdcnt = 0;
  priv->msgs  = msgs;
  priv->nmsg  = count;

  /* Configure the I2C frequency.
   * REVISIT: Note that the frequency is set only on the first message.
   * This could be extended to support different transfer frequencies for
   * each message segment.
   */

  rda5981x_i2c_setfrequency(priv->dev, msgs->frequency);

  /* Perform the transfer */

  ret = rda5981x_i2c_start(priv);

  sem_post(&priv->mutex);
  return ret;
}

/****************************************************************************
 * Name: rda5981x_stopnext
 *
 * Description:
 *   Check if we need to issue STOP at the next message
 *
 ****************************************************************************/

static void rda5981x_stopnext(struct rda5981x_i2cdev_s *priv)
{
  priv->nmsg--;

  if (priv->nmsg > 0)
    {
      priv->msgs++;
      putreg32(I2C_CONSET_STA, priv->base + LPC17_I2C_CONSET_OFFSET);
    }
  else
    {
      rda5981x_i2c_stop(priv);
    }
}

/****************************************************************************
 * Name: rda5981x_i2c_interrupt
 *
 * Description:
 *   The I2C Interrupt Handler
 *
 ****************************************************************************/

static int rda5981x_i2c_interrupt(int irq, FAR void *context, void *arg)
{
  struct rda5981x_i2cdev_s *priv = (struct rda5981x_i2cdev_s *)arg;
  struct i2c_msg_s *msg;
  uint32_t state;

  DEBUGASSERT(priv != NULL);

  /* Reference UM10360 19.10.5 */

  state = getreg32(priv->base + LPC17_I2C_STAT_OFFSET);
  msg  = priv->msgs;

  priv->state = state;
  state &= 0xf8;  /* state mask, only 0xX8 is possible */
  switch (state)
    {

    case 0x08:     /* A START condition has been transmitted. */
    case 0x10:     /* A Repeated START condition has been transmitted. */
      /* Set address */

      putreg32(((I2C_M_READ & msg->flags) == I2C_M_READ) ?
        I2C_READADDR8(msg->addr) :
        I2C_WRITEADDR8(msg->addr), priv->base + LPC17_I2C_DAT_OFFSET);

      /* Clear start bit */

      putreg32(I2C_CONCLR_STAC, priv->base + LPC17_I2C_CONCLR_OFFSET);
      break;

    /* Write cases */

    case 0x18: /* SLA+W has been transmitted; ACK has been received  */
      priv->wrcnt = 0;
      putreg32(msg->buffer[0], priv->base + LPC17_I2C_DAT_OFFSET); /* put first byte */
      break;

    case 0x28: /* Data byte in DAT has been transmitted; ACK has been received. */
      priv->wrcnt++;

      if (priv->wrcnt < msg->length)
        {
          putreg32(msg->buffer[priv->wrcnt], priv->base + LPC17_I2C_DAT_OFFSET); /* Put next byte */
        }
      else
        {
          rda5981x_stopnext(priv);
        }
      break;

    /* Read cases */

    case 0x40:  /* SLA+R has been transmitted; ACK has been received */
      priv->rdcnt = 0;
      if (msg->length > 1)
        {
          putreg32(I2C_CONSET_AA, priv->base + LPC17_I2C_CONSET_OFFSET); /* Set ACK next read */
        }
      else
        {
          putreg32(I2C_CONCLR_AAC, priv->base + LPC17_I2C_CONCLR_OFFSET);  /* Do not ACK because only one byte */
        }
      break;

    case 0x50:  /* Data byte has been received; ACK has been returned. */
      priv->rdcnt++;
      msg->buffer[priv->rdcnt - 1] = getreg32(priv->base + LPC17_I2C_BUFR_OFFSET);

      if (priv->rdcnt >= (msg->length - 1))
        {
          putreg32(I2C_CONCLR_AAC, priv->base + LPC17_I2C_CONCLR_OFFSET);  /* Do not ACK any more */
        }
      break;

    case 0x58:  /* Data byte has been received; NACK has been returned. */
      msg->buffer[priv->rdcnt] = getreg32(priv->base + LPC17_I2C_BUFR_OFFSET);
      rda5981x_stopnext(priv);
      break;

    default:
      rda5981x_i2c_stop(priv);
      break;
    }

  putreg32(I2C_CONCLR_SIC, priv->base + LPC17_I2C_CONCLR_OFFSET); /* clear interrupt */

  return OK;
}

/************************************************************************************
 * Name: rda5981x_i2c_reset
 *
 * Description:
 *   Perform an I2C bus reset in an attempt to break loose stuck I2C devices.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ************************************************************************************/

#ifdef CONFIG_I2C_RESET
static int rda5981x_i2c_reset(FAR struct i2c_dev_s * dev)
{
  return OK;
}
#endif /* CONFIG_I2C_RESET */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rda5981x_i2cbus_initialize
 *
 * Description:
 *   Initialise an I2C device
 *
 ****************************************************************************/

struct i2c_dev_s *rda5981x_i2cbus_initialize(int port)
{
  struct rda5981x_i2cdev_s *priv;

  if (port > 1)
    {
      i2cerr("ERROR: LPC I2C Only supports ports 0 and 1\n");
      return NULL;
    }

  irqstate_t flags;
  uint32_t regval;

  flags = enter_critical_section();

 if (port == 0)
    {
      priv        = &g_i2c0dev;
      priv->base  = LPC17_I2C0_BASE;
      priv->irqid = LPC17_IRQ_I2C0;

      /* Enable clocking */

      regval  = getreg32(LPC17_SYSCON_PCONP);
      regval |= SYSCON_PCONP_PCI2C0;
      putreg32(regval, LPC17_SYSCON_PCONP);

      regval  = getreg32(LPC17_SYSCON_PCLKSEL0);
      regval &= ~SYSCON_PCLKSEL0_I2C0_MASK;
      regval |= (SYSCON_PCLKSEL_CCLK << SYSCON_PCLKSEL0_I2C0_SHIFT);
      putreg32(regval, LPC17_SYSCON_PCLKSEL0);

      /* Pin configuration */

      rda5981x_configgpio(GPIO_I2C0_SCL);
      rda5981x_configgpio(GPIO_I2C0_SDA);

      /* Set default frequency */

      rda5981x_i2c_setfrequency(priv->dev, CONFIG_LPC17_I2C0_FREQUENCY);
    }


  leave_critical_section(flags);

  putreg32(I2C_CONSET_I2EN, priv->base + LPC17_I2C_CONSET_OFFSET);

  /* Initialize semaphores */

  sem_init(&priv->mutex, 0, 1);
  sem_init(&priv->wait, 0, 0);

  /* The wait semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  sem_setprotocol(&priv->wait, SEM_PRIO_NONE);

  /* Allocate a watchdog timer */

  priv->timeout = wd_create();
  DEBUGASSERT(priv->timeout != 0);

  /* Attach Interrupt Handler */

  irq_attach(priv->irqid, rda5981x_i2c_interrupt, priv);

  /* Enable Interrupt Handler */

  up_enable_irq(priv->irqid);

  /* Install our operations */

  priv->dev.ops = &rda5981x_i2c_ops;
  return &priv->dev;
}

/****************************************************************************
 * Name: rda5981x_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialise an I2C device
 *
 ****************************************************************************/

int rda5981x_i2cbus_uninitialize(FAR struct i2c_dev_s * dev)
{
  struct rda5981x_i2cdev_s *priv = (struct rda5981x_i2cdev_s *) dev;

  /* Disable I2C */

  putreg32(I2C_CONCLRT_I2ENC, priv->base + LPC17_I2C_CONCLR_OFFSET);

  /* Reset data structures */

  sem_destroy(&priv->mutex);
  sem_destroy(&priv->wait);

  /* Free the watchdog timer */

  wd_delete(priv->timeout);
  priv->timeout = NULL;

  /* Disable interrupts */

  up_disable_irq(priv->irqid);

  /* Detach Interrupt Handler */

  irq_detach(priv->irqid);
  return OK;
}


int up_i2cuninitialize(FAR struct i2c_dev_s *dev)
{
	return rda5981x_i2cbus_uninitialize(dev);
}

/**
 * @brief    Unitialize one I2C bus
 * @param    struct i2c_dev_s *dev :
 * @return   ==0 : OK
 * @note
 */

int rda5981x_i2cbus_uninitialize(struct i2c_dev_s *dev)
{
	struct rda5981x_i2c_priv_s *priv = (struct rda5981x_i2c_priv_s *)dev;
	int flags;

	DEBUGASSERT(priv && priv->config && priv->refs > 0);

	/* Decrement reference count and check for underflow */
	flags = irqsave();

	/* Check if the reference count will decrement to zero */
	if (priv->refs < 2) {
		/* Yes.. Disable power and other HW resource (GPIO's) */
		rda5981x_i2c_uninitialize(priv);
		priv->refs = 0;

		/* Release unused resources */
		rda5981x_i2c_sem_destroy(priv);
	} else {
		/* No.. just decrement the number of references to the device */
		priv->refs--;
	}

	irqrestore(flags);
	return OK;
}

/**
 * @brief   Unitialize one I2C bus for the I2C character driver
 * @param   struct i2c_dev_s *dev :
 * @return  ==0 : OK
 * @note
 */
void rda5981x_i2c_register(int bus)
{
	FAR struct i2c_dev_s *i2c;
	char path[16];
	int ret;

	i2c = rda5981x_i2cbus_initialize(bus);
	if (i2c != NULL) {
		snprintf(path, 16, "/dev/i2c-%d", bus);
		ret = i2c_uioregister(path, i2c);
		if (ret < 0) {
			rda5981x_i2cbus_uninitialize(i2c);
		}
	}
}


