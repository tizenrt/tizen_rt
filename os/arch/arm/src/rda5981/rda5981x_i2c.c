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

#include <tinyara/i2c.h>
#include <tinyara/arch.h>
#include <tinyara/semaphore.h>
#include <arch/board/board.h>


#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"

#include "rda5981x_gpio.h"
#include "rda5981x_i2c.h"
#include "rda5981x_system.h"
#include "chip/rda5981x_memorymap.h"
#include "chip/rda5981x_pinconfig.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define I2C_CLKGATE_REG1        (RDA_SCU_BASE + 0x08)
#define I2C_CLOCK_SOURCE        (RDA_AHB_CLK_FREQUENCY >> 1)
#define I2C_DEFAULT_CLOCK       (400000)
/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rda5981x_i2cdev_s
{
  struct i2c_dev_s  dev;     /* Generic I2C device */
  uint32_t base;       /* Base address of registers */
  uint32_t frequency;  /* Current I2C frequency */  
  int slave_address;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static int  rda5981x_i2c_start(struct rda5981x_i2cdev_s *priv);
static void rda5981x_i2c_stop(struct rda5981x_i2cdev_s *priv);
static int rda5981x_i2c_reset(struct rda5981x_i2cdev_s *priv);


/* I2C device operations */
static uint32_t rda5981x_i2c_setfrequency(FAR struct i2c_dev_s *priv,
              uint32_t frequency);
static int  rda5981x_i2c_transfer(FAR struct i2c_dev_s *dev,
              FAR struct i2c_msg_s *msgs, int count);

static int rda5981x_i2c_setaddress(FAR struct i2c_dev_s *dev, int addr, int nbits);

static int rda5981x_i2c_read(FAR struct i2c_dev_s *dev, FAR uint8_t *buffer, int length);

static int rda5981x_i2c_write(FAR struct i2c_dev_s *dev, FAR const uint8_t *buffer, int length);

/****************************************************************************
 * Private Data
 ****************************************************************************/
static struct rda5981x_i2cdev_s g_i2c0dev;
static const struct i2c_ops_s rda5981x_i2c_ops = {
	.setfrequency = rda5981x_i2c_setfrequency,
	.setaddress = rda5981x_i2c_setaddress,
	.write = rda5981x_i2c_write,
	.read = rda5981x_i2c_read,
	.transfer = rda5981x_i2c_transfer,
};

static void i2c_cmd_cfg(struct rda5981x_i2cdev_s *priv, int start, int stop,int acknowledge, int write, int read)
{
    uint32_t reg_val =  (start << 16) | (stop << 8) | (acknowledge << 0) |
			(write << 12) | (read << 4);
    putreg32(reg_val, priv->base + I2C_CMD);
}

static void i2c_do_write(struct rda5981x_i2cdev_s *priv, int value)
{
    /* write data */
    putreg32(value, priv->base + I2C_DR);
}

static void i2c_do_read(struct rda5981x_i2cdev_s *priv)
{
    uint32_t reg_val;

    /* config read number */
    reg_val = getreg32(priv->base + I2C_CR0) & ~(0x1FUL << 9);
    putreg32(reg_val | (0x01UL << 9), priv->base + I2C_CR0);
}

static void i2c_clear_fifo(struct rda5981x_i2cdev_s *priv)
{
    uint32_t reg_val;

    /* clear read number */
    reg_val = getreg32(priv->base + I2C_CR0) & ~(0x1FUL << 9);
    putreg32(reg_val, priv->base + I2C_CR0);

    /* clear fifo */
    reg_val = getreg32(priv->base + I2C_CR0) | (0x01UL << 7);
    putreg32(reg_val, priv->base + I2C_CR0);

    reg_val = getreg32(priv->base + I2C_CR0)  & ~(0x01UL << 7);
    putreg32(reg_val, priv->base + I2C_CR0);
}


static int rda5981x_i2c_setaddress(FAR struct i2c_dev_s *dev, int addr, 
					int nbits)
{
     struct rda5981x_i2cdev_s *priv = (struct rda5981x_i2cdev_s *)dev;
     priv->slave_address = addr;
     return 0;
}


static int rda5981x_i2c_read(FAR struct i2c_dev_s *dev, 
				FAR uint8_t *buffer, int length)
{
     struct rda5981x_i2cdev_s *priv = (struct rda5981x_i2cdev_s *)dev;
     int count;
     int stop = 1;

    /* wait tx fifo empty */
    while (((getreg32(priv->base+I2C_SR) & (0x1FUL << 26)) >> 26) != 0x1F) {
        /* timeout */
        if (getreg32(priv->base+I2C_SR) & (0x01UL << 31)) {
            i2c_clear_fifo(priv);
            return 0;
        }
    }
    i2c_do_write(priv, priv->slave_address | 0x1);
    i2c_cmd_cfg(priv, 1, 0, 0, 1, 0);

    /* wait tx fifo empty */
    while (((getreg32(priv->base+I2C_SR) & (0x1FUL << 26)) >> 26) != 0x1F) {
        if (getreg32(priv->base+I2C_SR) & (0x01UL << 31)) {
            i2c_clear_fifo(priv);
            return 0;
        }
    }

    /* Read in all except last byte */
    for (count = 0; count < (length - 1); count++) {
        i2c_do_read(priv);
        i2c_cmd_cfg(priv, 0, 0, 0, 0, 1);
        
        /* wait data */
        while ((getreg32(priv->base+I2C_SR) & (0x1F << 21)) == 0) {
            if (getreg32(priv->base+I2C_SR) & (0x01UL << 31)) {
                i2c_clear_fifo(priv);
                return count;
            }
        }
        buffer[count] = (char) (getreg32(priv->base+I2C_DR) & 0xFF);
    }


    if (stop) {
        i2c_do_read(priv);
        i2c_cmd_cfg(priv, 0, 1, 1, 0, 1);

        /* wait for i2c idle */
        while (getreg32(priv->base+I2C_SR) & (0x01UL << 16)) {
            if (getreg32(priv->base+I2C_SR) & (0x01UL << 31)) {
                i2c_clear_fifo(priv);
                return (length - 1);
            }
        }

        if (getreg32(priv->base+I2C_SR) & (0x1F << 21)) {
            buffer[count] = (char) (getreg32(priv->base+I2C_DR) & 0xFF);
            i2c_clear_fifo(priv);
        } else {
            i2c_clear_fifo(priv);
            return (length - 1);
        }
    }

    return length;
}

static int rda5981x_i2c_write(FAR struct i2c_dev_s *dev, 
		FAR const uint8_t *buffer, int length)
{
    struct rda5981x_i2cdev_s *priv = (struct rda5981x_i2cdev_s *)dev; 
    int count; 
    int stop = 1;
  
    if(length == 1)
	stop = 0;

    /* wait tx fifo empty */
    while (((getreg32(priv->base+I2C_SR) & (0x1FUL << 26)) >> 26) != 0x1F) {
        if (getreg32(priv->base+I2C_SR) & (0x01UL << 31)) {
            i2c_clear_fifo(priv);
            return 0;
        }
    }

    i2c_do_write(priv, priv->slave_address);
    i2c_cmd_cfg(priv, 1, 0, 0, 1, 0);

    for (count = 0; count < length - 1; count++) {
        /* wait tx fifo empty */
       while ((((getreg32(priv->base+I2C_SR)) & (0x1FUL << 26)) >> 26) != 0x1F) {
            if (getreg32(priv->base+I2C_SR) & (0x01UL << 31)) {
                i2c_clear_fifo(priv);
                return (count > 0 ? (count - 1) : 0);
            }
        }
        i2c_do_write(priv, buffer[count]);
        i2c_cmd_cfg(priv, 0, 0, 0, 1, 0);
    }

    /* wait tx fifo empty */
    while (((getreg32(priv->base+I2C_SR) & (0x1FUL << 26)) >> 26) != 0x1F) {
        if (getreg32(priv->base+I2C_SR) & (0x01UL << 31)) {
            i2c_clear_fifo(priv);
            return (count > 0 ? (count - 1) : 0);
        }
    }

    i2c_do_write(priv, buffer[length - 1]);
    if (stop) {
        i2c_cmd_cfg(priv, 0, 1, 0, 1, 0);
    } else {
        i2c_cmd_cfg(priv, 0, 0, 0, 1, 0);
    }

    /* wait tx fifo empty */
    while (((getreg32(priv->base+I2C_SR) & (0x1FUL << 26)) >> 26) != 0x1F) {
        if (getreg32(priv->base+I2C_SR) & (0x01UL << 31)) {
            i2c_clear_fifo(priv);
            return (length - 1);
        }
    }
    
    if (stop) {
        i2c_clear_fifo(priv);
    }
    
    return length; 	
}

/****************************************************************************
 * Name: rda5981x_i2c_setfrequency
 *
 * Description:
 *   Set the frequency for the next transfer
 *
 ****************************************************************************/

static uint32_t rda5981x_i2c_setfrequency(FAR struct i2c_dev_s *dev,
                                   uint32_t frequency)
{
    struct rda5981x_i2cdev_s *priv = (struct rda5981x_i2cdev_s *)dev; 
   /* Set I2C frequency */
  if (frequency != priv->frequency)
    {


    uint32_t prescale = I2C_CLOCK_SOURCE / ((uint32_t)frequency * 5U) - 1U;
    uint32_t reg_val;
    
    reg_val = getreg32(priv->base + I2C_CR0) & ~(0xFFFFUL << 16);
    putreg32(reg_val |(prescale << 16), priv->base + I2C_CR0);
    priv->frequency = frequency;
    }
    return 0;
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
  struct i2c_msg_s *pmsg; 
  int ret;
  int i;
  DEBUGASSERT(dev != NULL && msgs != NULL && count > 0);

  /* Get exclusive access to the I2C bus */
  
  for (i = 0; i < count; i++) {
	pmsg = &msgs[i];
  
   rda5981x_i2c_setaddress(dev, pmsg->addr, 7);
   
   if (pmsg->flags & I2C_M_READ) 
   {
	ret = rda5981x_i2c_read(dev, pmsg->buffer, pmsg->length);
   }

   else 
   {
	ret = rda5981x_i2c_write(dev, pmsg->buffer, pmsg->length);
   }

 }
  /* Perform the transfer */

  return ret;
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
   i2c_cmd_cfg(priv, 1, 0, 0, 0, 0); 
   return 0;
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
 i2c_cmd_cfg(priv, 0, 1, 0, 0, 0); 
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

static int rda5981x_i2c_reset(FAR struct rda5981x_i2cdev_s *priv)
{
  i2c_cmd_cfg(priv, 0, 1, 0, 0, 0);
  return 0;
}


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
  uint32_t regval;

  if (port > 1)
    {
      i2cerr("ERROR: LPC I2C Only supports ports 0 and 1\n");
      return NULL;
    }

  if (port == 0)
    {
      priv        = &g_i2c0dev;
      priv->base  = RDA_I2C0_BASE;
      priv->frequency = I2C_DEFAULT_CLOCK;

/* Enable I2C clock */
    regval = getreg32(I2C_CLKGATE_REG1) | (0x01UL << 6);
    putreg32(regval, I2C_CLKGATE_REG1);

/* set default frequency at 100k */
    rda5981x_i2c_setfrequency(&(priv->dev), 100000);
    i2c_cmd_cfg(priv, 0, 0, 0, 0, 0);

/* Enable I2C */
    regval = getreg32(priv->base + I2C_CR0) |= 0x01UL;    
    putreg32(regval, priv->base + I2C_CR0);

/* Pin configuration */
     rda_configgpio(GPIO_I2C0_SCL);
     rda_configgpio(GPIO_I2C0_SDA);
    }

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
  putreg32(0, priv->base + I2C_CR0);
  return OK;
}

FAR struct i2c_dev_s *up_i2cinitialize(int port)
{
	return rda5981x_i2cbus_initialize(port);
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


