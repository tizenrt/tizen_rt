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
 * arch/arm/src/rda5981x/rda5981x_spi.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

/*****************************************************************************
 * Included Files
 *****************************************************************************/
#include <tinyara/config.h>

#include <stdio.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <errno.h>

#include <tinyara/arch.h>
#include <tinyara/spi/spi.h>

#include "up_arch.h"
#include "rda5981x_spi.h"
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <tinyara/irq.h>
#include <arch/irq.h>
#include <arch/board/board.h>
#include <tinyara/kmalloc.h>
#include <poll.h>
#include <tinyara/fs/fs.h>
#include <stddef.h>
#include <chip.h>
#include "rda5981x_gpio.h"
#include "chip/rda5981x_pinconfig.h"
/****************************************************************************
 * Definitions
 ****************************************************************************/
#define ENABLE_RDA_SPI_MODE 0
#define RDA_BUS_CLK_FREQUENCY_80M                       ( 80000000UL)


/****************************************************************************
 * Private Types
 ****************************************************************************/
struct rda5981x_spidev_s {
	struct spi_dev_s spidev;
	uint32_t base;
	uint8_t port;
	unsigned int freqid;
	s32 gpio_clk;
	s32 gpio_nss;
	s32 gpio_miso;
	s32 gpio_mosi;

#ifndef CONFIG_SPI_POLLWAIT
	sem_t xfrsem;				/* Wait for transfer to complete */
#endif

#ifndef CONFIG_SPI_OWNBUS
	sem_t exclsem;
#endif
};


struct spi_s spi_str;


/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* SPI Driver Methods */
static int spi_lock(FAR struct spi_dev_s *dev, bool lock);
static void spi_select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected);
static uint32_t spi_setfrequency(FAR struct spi_dev_s *dev, uint32_t frequency);
static void spi_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode);
static void spi_setbits(FAR struct spi_dev_s *dev, int nbits);
static uint16_t spi_send(FAR struct spi_dev_s *dev, uint16_t wd);
static void spi_exchange(FAR struct spi_dev_s *dev, const void *txbuffer, void *rxbuffer, size_t nwords);

#ifndef CONFIG_SPI_EXCHANGE
static void spi_sndblock(FAR struct spi_dev_s *dev, const void *txbuffer, size_t nwords);
static void spi_recvblock(FAR struct spi_dev_s *dev, void *rxbuffer, size_t nwords);
#endif


/****************************************************************************
 * Private Data
 ****************************************************************************/
static const struct spi_ops_s g_spiops = {
#ifndef CONFIG_SPI_OWNBUS
	.lock				= spi_lock,
#endif
	.select				= spi_select,
	.setfrequency		= spi_setfrequency,
	.setmode			= (void *)spi_setmode,
	.setbits			= (void *)spi_setbits,
	.status				= 0,
#ifdef CONFIG_SPI_CMDDATA
	.cmddata			= 0,
#endif
	.send				= spi_send,
#ifdef CONFIG_SPI_EXCHANGE
	.exchange			= spi_exchange,
#else
	.sndblock			= spi_sndblock,
	.recvblock			= spi_recvblock,
#endif
	.registercallback	= 0,
};

static struct rda5981x_spidev_s g_spi0dev = {
	.spidev		= { .ops = &g_spiops },
	.base		= RDA_SPI0_BASE,
	.port		= SPI_PORT0,
	.freqid		= 0,
	.gpio_clk	= GPIO_SPI0_CLK,
	.gpio_nss	= GPIO_SPI0_CS,
	.gpio_miso	= GPIO_SPI0_MISO,
	.gpio_mosi	= GPIO_SPI0_MOSI,
};


/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_lock
 *
 * Description:
 *   On SPI busses where there are multiple devices, it will be necessary to
 *   lock SPI to have exclusive access to the busses for a sequence of
 *   transfers.  The bus should be locked before the chip is selected. After
 *   locking the SPI bus, the caller should then also call the setfrequency,
 *   setbits, and setmode methods to make sure that the SPI is properly
 *   configured for the device.  If the SPI buss is being shared, then it
 *   may have been left in an incompatible state.
 *
 ****************************************************************************/
static int spi_lock(struct spi_dev_s *dev, bool lock)
{
	FAR struct rda5981x_spidev_s *priv = (FAR struct rda5981x_spidev_s *)dev;

	if (lock) {
		while (sem_wait(&priv->exclsem) != 0) {
			DEBUGASSERT(errno == EINTR);
		}
	} else {
		(void)sem_post(&priv->exclsem);
	}

	return OK;
}

/****************************************************************************
 * Name: spi_setfrequency
 *
 * Description:
 *   Set the SPI frequency.
 *
 ****************************************************************************/
static uint32_t spi_setfrequency(struct spi_dev_s *dev, uint32_t frequency)
{
    uint32_t clk_rate = ((RDA_BUS_CLK_FREQUENCY_80M / frequency) >> 2) - 1U;
    uint32_t reg_val;

    /* Check for valid frequency */
    if(clk_rate <= 0x3FUL)
    {
	return -1;
    }

    /* Set clk rate field */
    reg_val = getreg32(RDA_SPI0_BASE+0x0);
    reg_val = reg_val & ~(0x3FUL << 4);
    reg_val |= ((clk_rate & 0x3FUL) << 4);
    putreg32(RDA_SPI0_BASE+0x0, reg_val);

    return 0;
}

/****************************************************************************
 * Name: spi_select
 *
 * Description:
 *   Enable/disable the SPI slave select.   The implementation of this method
 *   must include handshaking:  If a device is selected, it must hold off
 *   all other attempts to select the device until the device is deselecte.
 *
 ****************************************************************************/
static void spi_select(struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
   // Only support Master Mode on RDA
#if 0
	FAR struct rda5981x_spidev_s *priv = (FAR struct rda5981x_spidev_s *)dev;

	SPI_SFR *pSPIRegs;
	pSPIRegs = (SPI_SFR *)priv->base;

	unsigned int cs_reg;
	cs_reg = getreg32(&pSPIRegs->CS_REG);
	cs_reg |= CS_REG_nSS_INACTIVE;

	if (selected == TRUE) {
		cs_reg &= ~CS_REG_nSS_INACTIVE;
	}

	putreg32(cs_reg, &pSPIRegs->CS_REG);
#endif
}

/****************************************************************************
 * Name: spi_setmode
 *
 * Description:
 *   Set the SPI mode.  see enum spi_mode_e for mode definitions
 *
 ****************************************************************************/
static void spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode)
{
	FAR struct rda5981x_spidev_s *priv = (FAR struct rda5981x_spidev_s *)dev;
	uint32_t polarity = (mode & 0x2) ? (0x01UL) : (0x00UL);

	spi_str.spi = (SPI_SFR *)priv->base;
	unsigned int mod_cfg;
	mod_cfg = spi_str.spi->CFG;

	/* Set number of frame bits and clock phase */
	mod_cfg = mod_cfg & ~(0x7FUL << 16) & ~(0x01UL << 1);
	mod_cfg = mod_cfg | (polarity << 1);

    spi_str.spi->CFG = mod_cfg;	
}

/****************************************************************************
 * Name: spi_setbits
 *
 * Description:
 *   Set the number of bits per word.
 *
 ****************************************************************************/
static void spi_setbits(struct spi_dev_s *dev, int nbits)
{
	FAR struct rda5981x_spidev_s *priv = (FAR struct rda5981x_spidev_s *)dev;

	spi_str.spi = (SPI_SFR *)priv->base;
	unsigned int bit_cfg;
	bit_cfg = spi_str.spi->CFG;

	/* Set number of frame bits and clock phase */
	bit_cfg = bit_cfg & ~(0x7FUL << 16) & ~(0x01UL << 1);
	bit_cfg = bit_cfg | ((uint32_t)nbits << 16);

    spi_str.spi->CFG = bit_cfg;


#if ENABLE_RDA_SPI_MODE
    /* Set bit offset value */
    spi_str.bit_ofst[0] = 0;
    spi_str.bit_ofst[1] = 0;
    if(2 > (nbits >> 5)) {
        spi_str.bit_ofst[nbits >> 5] = (uint8_t)(32 - (nbits & 0x1F));
    }
#else  /* ENABLE_RDA_SPI_MODE */
    spi_str.bit_ofst[0] = (uint8_t)(32 - nbits);
#endif /* ENABLE_RDA_SPI_MODE */
	
}

/****************************************************************************
 * Name: spi_busy
 *
 * Description:
 *   Set the number of bits per word.
 *
 ****************************************************************************/
static int spi_busy(void)
{
    return (spi_str.spi->CFG & (0x01UL << 31)) ? (1) : (0);
}


static void spi_write(int value)
{
#if ENABLE_RDA_SPI_MODE
    /* Write data register */
    if(spi_str.bit_ofst[0] != 0) {
        spi_str.spi->D1CMD = (uint32_t)value << spi_str.bit_ofst[0];
    } else {
        spi_str.spi->D1CMD = (uint32_t)value;
        spi_str.spi->D0CMD = (uint32_t)value << spi_str.bit_ofst[1];
    }
    /* Set write bit & start bit */
    spi_str.spi->CFG = (spi_str.spi->CFG & ~(0x01UL << 3)) | 0x01UL;
#else  /* ENABLE_RDA_SPI_MODE */
    /* Write data reg */
    if(spi_str.bit_ofst[0] != 0) {
        spi_str.spi->D1CMD = ((uint32_t)value << spi_str.bit_ofst[0]) | (0xFFFFFFFFUL >> (32 - spi_str.bit_ofst[0]));
    } else {
        spi_str.spi->D1CMD = (uint32_t)value;
        spi_str.spi->D0CMD = 0xFFFFFFFFUL;
    }
    /* Set start bit */
    spi_str.spi->CFG |= 0x01UL;
#endif /* ENABLE_RDA_SPI_MODE */
    while(spi_busy());
}


static int spi_read(void)
{
    uint32_t ret_val;

#if ENABLE_RDA_SPI_MODE
    /* Set read bit & start bit */
    spi_str.spi->CFG |= ((0x01UL << 3) | 0x01UL);
    while (spi_busy());
    /* Read data register */
    if(spi_str.bit_ofst[0] != 0) {
        ret_val = spi_str.spi->D0CMD & ((0x01UL << (32UL - spi_str.bit_ofst[0])) - 1UL);
    } else {
        ret_val = spi_str.spi->D0CMD;
        ret_val = spi_str.spi->D1CMD & ((0x01UL << (32UL - spi_str.bit_ofst[1])) - 1UL);
    }
#else  /* ENABLE_RDA_SPI_MODE */
    /* Read data register */
    ret_val = spi_str.spi->D0CMD & ((0x01UL << (32UL - spi_str.bit_ofst[0])) - 1UL);
#endif /* ENABLE_RDA_SPI_MODE */
    return (int)ret_val;
}


/****************************************************************************
 * Name: spi_send
 *
 * Description:
 *   Exchange one word on SPI. Currently support only byte transfers.
 *
 ****************************************************************************/
static uint16_t spi_send(struct spi_dev_s *dev, uint16_t wd)
{
	uint8_t txbyte;
	uint8_t rxbyte;

	txbyte = (uint8_t)wd;
	rxbyte = (uint8_t)0;
	spi_exchange(dev, &txbyte, &rxbyte, 1);

	return (uint16_t)rxbyte;
}

/****************************************************************************
 * Name: spi_exchange
 *
 * Description:
 *   Exchange a block data with the SPI device. Support only byte transfers.
 *
 ****************************************************************************/

static void spi_exchange(struct spi_dev_s *dev, const void *txbuffer, void *rxbuffer, size_t nwords)
{
	FAR struct rda5981x_spidev_s *priv = (FAR struct rda5981x_spidev_s *)dev;

	size_t sent = 0;
	size_t received = 0;
	unsigned int dummy_rx;

	SPI_SFR *pSPIRegs;
	pSPIRegs = (SPI_SFR *)priv->base;

	/* TX/RX */
	if ((rxbuffer == NULL) && (txbuffer == NULL)) 
	{
		while (received < nwords) 
		{
			if (sent < nwords)
			{
				spi_write(0);
				sent++;
			}

			spi_read();
			received++;
		}
		return;
	}

	if (rxbuffer == NULL) {
		while (received < nwords) {
			if (sent < nwords)
			{
				spi_write(((unsigned char *)txbuffer)[sent++]);
			}

			dummy_rx = spi_read();
			received++;
		}
		return;
	}

	if (txbuffer == NULL) {
		while (received < nwords) {
			if (sent < nwords)
			{
				spi_write(((unsigned char *)rxbuffer)[sent++]);
			}

			((unsigned char *)rxbuffer)[received++] = spi_read();
		}
		return;
	}

	while (received < nwords) 
	{
		if (sent < nwords)
		{
			spi_write(((unsigned char *)txbuffer)[sent++]);
		}

		((unsigned char *)rxbuffer)[received++] = spi_read();
	}
}

/****************************************************************************
 * Name: spi_sndblock
 *
 * Description:
 *   Send a block of data on SPI. Support only byte transfers
 *
 ****************************************************************************/
static void spi_sndblock(struct spi_dev_s *dev, const void *txbuffer, size_t nwords)
{
	spi_exchange(dev, txbuffer, NULL, nwords);
}

/****************************************************************************
 * Name: spi_recvblock
 *
 * Description:
 *   Revice a block of data from SPI. Support only byte transfers.
 *
 ****************************************************************************/
static void spi_recvblock(struct spi_dev_s *dev, void *rxbuffer, size_t nwords)
{
	spi_exchange(dev, NULL, rxbuffer, nwords);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/



/****************************************************************************
 * Name: up_spiinitialize
 *
 * Description:
 *   Initialize the selected SPI port
 *
 ****************************************************************************/
struct spi_dev_s *up_spiinitialize(int port)
{
	FAR struct rda5981x_spidev_s *priv = NULL;
	uint32_t regval;

	if (port < 0 || port >= SPI_PORT_MAX) {
		return NULL;
	}
	
	priv = &g_spi0dev;

	lldbg("Prepare SPI%d for Master operation\n", priv->port);

	spi_str.spi = (SPI_SFR *)priv->base;

#ifndef CONFIG_SPI_POLLWAIT
	sem_init(&priv->xfrsem, 0, 0);
#endif

#ifndef CONFIG_SPI_OWNBUS
	sem_init(&priv->exclsem, 0, 1);
#endif

    /* Enable power and clocking */  //Clock Gating 2 
	regval = getreg32(RDA_SCU_BASE+0x0C);
	regval |=	(0x01UL << 18);
	putreg32(RDA_SCU_BASE+0x0C, regval);

	/* Select 4-wire SPI mode */
	regval = getreg32(RDA_GPIO_BASE+0x0);
    regval  &= ~(0x01UL << 14);
	putreg32(RDA_GPIO_BASE+0x0, regval);

	//Set Config Reg
	/* Normal SPI mode */
	regval = getreg32(RDA_SPI0_BASE+0x0);
    regval &= ~(0x01UL << 2);
    /* Set read flag */
    regval |=  (0x01UL << 3);

   //set SEL
	regval &= ~(0x03UL << 23);
	regval |= ((1 & 0x03UL) << 23);

	putreg32(RDA_SPI0_BASE+0x0, regval);
	

	/* SET GPIO for the port */
	rda_configgpio(priv->gpio_clk);
	rda_configgpio(priv->gpio_nss);
	rda_configgpio(priv->gpio_miso);
	rda_configgpio(priv->gpio_mosi);


	return (struct spi_dev_s *)priv;
}
