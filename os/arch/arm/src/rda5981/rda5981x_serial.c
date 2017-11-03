/****************************************************************************
 * arch/arm/src/rda5981x/rda5981x_serial.c
 *
 *   Copyright (C) 2010-2013, 2017 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <semaphore.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#endif

#include <tinyara/irq.h>
#include <tinyara/arch.h>
#include <tinyara/serial/serial.h>

#include <arch/serial.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"

#include "chip.h"
#include "chip/rda5981x_uart.h"
#include "rda5981x_gpio.h"
#include "rda5981x_serial.h"

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/* If we are not using the serial driver for the console, then we still must
 * provide some minimal implementation of up_putc.
 */

#if defined(USE_SERIALDRIVER) && defined(HAVE_UART)

/* Configuration ************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s {
	uint32_t uartbase;  /* Base address of UART registers */
	uint32_t baud;      /* Configured baud */
	uint32_t ier;       /* Saved IER value */
	uint8_t  irq;       /* IRQ associated with this UART */
	uint8_t  parity;    /* 0=none, 1=odd, 2=even */
	uint8_t  bits;      /* Number of bits (7 or 8) */
	bool     stopbits2; /* true: Configure with 2 stop bits instead of 1 */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  up_setup(struct uart_dev_s *dev);
static void up_shutdown(struct uart_dev_s *dev);
static int  up_attach(struct uart_dev_s *dev);
static void up_detach(struct uart_dev_s *dev);
static int  up_interrupt(int irq, void *context, void *arg);
static int  up_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  up_receive(struct uart_dev_s *dev, uint32_t *status);
static void up_rxint(struct uart_dev_s *dev, bool enable);
static bool up_rxavailable(struct uart_dev_s *dev);
static void up_send(struct uart_dev_s *dev, int ch);
static void up_txint(struct uart_dev_s *dev, bool enable);
static bool up_txready(struct uart_dev_s *dev);
static bool up_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* I/O buffers */

#ifdef CONFIG_RDA5981X_UART0
static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];
#endif

#ifdef CONFIG_RDA5981X_UART1
static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];
#endif

/* This describes the state of the RDA5981x uart0 port. */

#ifdef CONFIG_RDA5981X_UART0
static struct up_dev_s g_uart0priv;
static uart_dev_t g_uart0port;
static struct uart_ops_s g_uart_ops;
#endif

/* This describes the state of the RDA5981x uart1 port. */

#ifdef CONFIG_RDA5981X_UART1
static struct up_dev_s g_uart1priv = {
	.uartbase       = RDA_UART1_BASE,
	.baud           = CONFIG_UART1_BAUD,
	.irq            = RDA_IRQ_UART1,
	.parity         = CONFIG_UART1_PARITY,
	.bits           = CONFIG_UART1_BITS,
	.stopbits2      = CONFIG_UART1_2STOP,
};

static uart_dev_t g_uart1port = {
	.recv     =
	{
		.size   = CONFIG_UART1_RXBUFSIZE,
		.buffer = g_uart1rxbuffer,
	},
	.xmit     =
	{
		.size   = CONFIG_UART1_TXBUFSIZE,
		.buffer = g_uart1txbuffer,
	},
	.ops      = &g_uart_ops,
	.priv     = &g_uart1priv,
};
#endif

/* Which UART with be tty0/console and which tty1? */

#ifdef HAVE_CONSOLE
#  if defined(CONFIG_UART0_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart0port     /* UART0=console */
#    define TTYS0_DEV       g_uart0port     /* UART0=ttyS0 */
#    ifdef CONFIG_RDA5981X_UART1
#      define TTYS1_DEV     g_uart1port     /* UART0=ttyS0;UART1=ttyS1 */
#    else
#      undef TTYS1_DEV                      /* No ttyS1 */
#    endif
#  elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart1port     /* UART1=console */
#    define TTYS0_DEV       g_uart1port     /* UART1=ttyS0 */
#    ifdef CONFIG_RDA5981X_UART0
#      define TTYS1_DEV     g_uart0port     /* UART1=ttyS0;UART0=ttyS1 */
#    else
#      undef TTYS1_DEV                      /* No ttyS1 */
#    endif
#  endif
#else /* No console */
#  define TTYS0_DEV       g_uart0port       /* UART0=ttyS0 */
#  ifdef CONFIG_RDA5981X_UART1
#    define TTYS1_DEV     g_uart1port       /* UART0=ttyS0;UART1=ttyS1 */
#  else
#    undef TTYS1_DEV                        /* No ttyS1 */
#  endif
#endif /* HAVE_CONSOLE */

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

/****************************************************************************
 * Name: up_serialin
 ****************************************************************************/

static inline uint32_t up_serialin(struct up_dev_s *priv, int offset)
{
	return getreg32(priv->uartbase + offset);
}

/****************************************************************************
 * Name: up_serialout
 ****************************************************************************/

static inline void up_serialout(struct up_dev_s *priv, int offset, uint32_t value)
{

	putreg32(value, priv->uartbase + offset);
}

/****************************************************************************
 * Name: up_disableuartint
 ****************************************************************************/

static inline void up_disableuartint(struct up_dev_s *priv, uint32_t *ier)
{
	if (ier) {
		*ier = priv->ier & UART_IER_ALLIE;
	}

	priv->ier &= ~UART_IER_ALLIE;
	up_serialout(priv, RDA_UART_IER_OFFSET, priv->ier);
}

/****************************************************************************
 * Name: up_restoreuartint
 ****************************************************************************/

static inline void up_restoreuartint(struct up_dev_s *priv, uint32_t ier)
{
	priv->ier |= ier & UART_IER_ALLIE;
	up_serialout(priv, RDA_UART_IER_OFFSET, priv->ier);
}

/****************************************************************************
 * Name: up_enablebreaks
 ****************************************************************************/

static inline void up_enablebreaks(struct up_dev_s *priv, bool enable)
{
	uint32_t lcr = up_serialin(priv, RDA_UART_LCR_OFFSET);

	if (enable) {
		lcr |= UART_LCR_BRK;
	} else {
		lcr &= ~UART_LCR_BRK;
	}

	up_serialout(priv, RDA_UART_LCR_OFFSET, lcr);
}

/************************************************************************************
 * Name: rda_uart0config, rda_uart1config
 *
 * Descrption:
 *   Configure the UART.  UART0/1 peripherals are configured using the following
 *   registers:
 *
 *   1. Power.
 *   2. Peripheral clock.
 *   3. Pins: Select UART pins through the PINSEL registers and pin modes
 *      through the PINMODE registers. UART receive pins should not have
 *      pull-down resistors enabled.
 *
 ************************************************************************************/

#ifdef CONFIG_RDA5981X_UART0
static inline void rda_uart0config(void)
{
	/* Step 1: Enable power on UART0 */

	/* Step 2: Enable clocking on UART */

	/* Step 3: Configure I/O pins */

	rda_configgpio(GPIO_UART0_TXD);
	rda_configgpio(GPIO_UART0_RXD);
};
#endif

#ifdef CONFIG_RDA5981X_UART1
static inline void rda_uart1config(void)
{
	/* Step 1: Enable power on UART1 */

	/* Step 2: Enable clocking on UART */

	/* Step 3: Configure I/O pins */

	rda_configgpio(GPIO_UART1_TXD);
	rda_configgpio(GPIO_UART1_RXD);
#if defined(CONFIG_UART1_IFLOWCONTROL) || defined(CONFIG_UART1_OFLOWCONTROL)
	rda_configgpio(GPIO_UART1_CTS);
	rda_configgpio(GPIO_UART1_RTS);
#endif
};
#endif

/************************************************************************************
 * Name: rda_uartdl
 *
 * Descrption:
 *   Select a divider to produce the BAUD from the UART PCLK.
 *
 *     BAUD = AHBCLK / (16 * DL), or
 *     DL   = AHBCLK / BAUD / 16
 *
 *   Ignoring the fractional divider for now. (If you want to extend this driver
 *   to support the fractional divider, see lpc43xx_uart.c.  The LPC43xx uses
 *   the same peripheral and that logic could easily leveraged here).
 *
 ************************************************************************************/

static inline uint32_t rda_uartdl(uint32_t baud, uint8_t *dl2)
{
	uint32_t dl;
	uint8_t mod;

	dl = (RDA_AHB_CLK_FREQUENCY / baud) >> 4;
	mod = (uint8_t)((RDA_AHB_CLK_FREQUENCY / baud) & 0x0F);
	*dl2 = ((mod - (mod >> 1)) << 4) | (mod >> 1);

	return dl;
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, fifos, etc. This method is
 *   called the first time that the serial port is opened.
 *
 ****************************************************************************/

static int up_setup(struct uart_dev_s *dev)
{
#ifndef CONFIG_SUPPRESS_UART_CONFIG
	struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
	uint16_t dl;
	uint8_t dl2;
	uint32_t lcr;

	/* Clear fifos */

	up_serialout(priv, RDA_UART_FCR_OFFSET, (UART_FCR_RXRST | UART_FCR_TXRST));

	/* Set trigger */

	up_serialout(priv, RDA_UART_FCR_OFFSET, (UART_FCR_FIFOEN | UART_FCR_RXTRIGGER_L0));

	/* Set up the IER */

	priv->ier = up_serialin(priv, RDA_UART_IER_OFFSET);

	/* Set up the LCR */

	lcr = 0;

	if (priv->bits == 7) {
		lcr |= UART_LCR_WLS_7BIT;
	} else {
		lcr |= UART_LCR_WLS_8BIT;
	}

	if (priv->stopbits2) {
		lcr |= UART_LCR_STOP;
	}

	if (priv->parity == 1) {
		lcr |= (UART_LCR_PE | UART_LCR_PS_ODD);
	} else if (priv->parity == 2) {
		lcr |= (UART_LCR_PE | UART_LCR_PS_EVEN);
	}

	/* Enter DLAB=1 */

	up_serialout(priv, RDA_UART_LCR_OFFSET, (lcr | UART_LCR_DLAB));

	/* Set the BAUD divisor */

	dl = rda_uartdl(priv->baud, &dl2);
	up_serialout(priv, RDA_UART_DLM_OFFSET, dl >> 8);
	up_serialout(priv, RDA_UART_DLL_OFFSET, dl & 0xff);
	up_serialout(priv, RDA_UART_DL2_OFFSET, dl2);

	/* Clear DLAB */

	up_serialout(priv, RDA_UART_LCR_OFFSET, lcr);

	/* Configure the FIFOs */

	up_serialout(priv, RDA_UART_FCR_OFFSET,
				 (UART_FCR_RXTRIGGER_L0 | UART_FCR_TXRST | UART_FCR_RXRST |
				  UART_FCR_FIFOEN));

	/* Enable Auto Flow Control in the Modem Control Register */

#if defined(CONFIG_UART1_IFLOWCONTROL) || defined(CONFIG_UART1_OFLOWCONTROL)
	if (priv->uartbase == RDA_UART1_BASE) {
		up_serialout(priv, RDA_UART_MCR_OFFSET, UART_MCR_AFCE | UART_MCR_CSEL);
	}
#endif

#endif
	return OK;
}

/****************************************************************************
 * Name: up_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial port is closed
 *
 ****************************************************************************/

static void up_shutdown(struct uart_dev_s *dev)
{
	struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
	up_disableuartint(priv, NULL);
}

/****************************************************************************
 * Name: up_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.  This method is
 *   called when the serial port is opened.  Normally, this is just after the
 *   the setup() method is called, however, the serial console may operate in
 *   a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method (unless the
 *   hardware supports multiple levels of interrupt enabling).  The RX and TX
 *   interrupts are not enabled until the txint() and rxint() methods are called.
 *
 ****************************************************************************/

static int up_attach(struct uart_dev_s *dev)
{
	struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
	int ret;

	/* Attach and enable the IRQ */

	ret = irq_attach(priv->irq, up_interrupt, dev);
	if (ret == OK) {
		/* Enable the interrupt (RX and TX interrupts are still disabled
		 * in the UART
		 */
		up_enable_irq(priv->irq);
	}

	return ret;
}

/****************************************************************************
 * Name: up_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The exception is
 *   the serial console which is never shutdown.
 *
 ****************************************************************************/

static void up_detach(struct uart_dev_s *dev)
{
	struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
	up_disable_irq(priv->irq);
	irq_detach(priv->irq);
}

/****************************************************************************
 * Name: up_interrupt
 *
 * Description:
 *   This is the UART interrupt handler.  It will be invoked when an
 *   interrupt received on the 'irq'  It should call uart_transmitchars or
 *   uart_receivechar to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'irq' number into the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int up_interrupt(int irq, void *context, void *arg)
{
	struct uart_dev_s *dev = (struct uart_dev_s *)arg;
	struct up_dev_s   *priv;
	uint32_t           status;
	int                passes;

	DEBUGASSERT(dev != NULL && dev->priv != NULL);
	priv = (struct up_dev_s *)dev->priv;

	/* Loop until there are no characters to be transferred or,
	 * until we have been looping for a long time.
	 */

	for (passes = 0; passes < 256; passes++) {
		/* Get the current UART status and check for loop
		 * termination conditions
		 */

		status = up_serialin(priv, RDA_UART_IIR_OFFSET);

		/* The UART_IIR_INTSTATUS bit should be zero if there are pending
		 * interrupts
		 */

		if ((status & UART_IIR_INTSTATUS) != 0) {
			/* Break out of the loop when there is no longer a
			 * pending interrupt
			 */

			break;
		}

		/* Handle the interrupt by its interrupt ID field */

		switch (status & UART_IIR_INTID_MASK) {
		/* Handle incoming, receive bytes (with or without timeout) */

		case UART_IIR_INTID_RDA:
		case UART_IIR_INTID_CTI: {
			uart_recvchars(dev);
			break;
		}

		/* Handle outgoing, transmit bytes */

		case UART_IIR_INTID_THRE: {
			uart_xmitchars(dev);
			break;
		}

		/* Just clear modem status interrupts (UART1 only) */

		case UART_IIR_INTID_MSI: {
			/* Read the modem status register (MSR) to clear */

			status = up_serialin(priv, RDA_UART_MSR_OFFSET);
			//_info("MSR: %02x\n", status);
			break;
		}

		/* Just clear any line status interrupts */

		case UART_IIR_INTID_RLS: {
			/* Read the line status register (LSR) to clear */

			status = up_serialin(priv, RDA_UART_LSR_OFFSET);
			//_info("LSR: %02x\n", status);
			break;
		}

		/* There should be no other values */

		default: {
			//_err("ERROR: Unexpected IIR: %02x\n", status);
			break;
		}
		}
	}
	return OK;
}

/****************************************************************************
 * Name: up_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int up_ioctl(struct file *filep, int cmd, unsigned long arg)
{
	struct inode      *inode = filep->f_inode;
	struct uart_dev_s *dev   = inode->i_private;
	struct up_dev_s   *priv  = (struct up_dev_s *)dev->priv;
	int                ret    = OK;

	switch (cmd) {
#ifdef CONFIG_SERIAL_TIOCSERGSTRUCT
	case TIOCSERGSTRUCT: {
		struct up_dev_s *user = (struct up_dev_s *)arg;
		if (!user) {
			ret = -EINVAL;
		} else {
			memcpy(user, dev, sizeof(struct up_dev_s));
		}
	}
	break;
#endif

	case TIOCSBRK: { /* BSD compatibility: Turn break on, unconditionally */
		// irqstate_t flags = enter_critical_section();
		up_enablebreaks(priv, true);
		// leave_critical_section(flags);
	}
	break;

	case TIOCCBRK: { /* BSD compatibility: Turn break off, unconditionally */
		//irqstate_t flags;
		//flags = enter_critical_section();
		up_enablebreaks(priv, false);
		// leave_critical_section(flags);
	}
	break;

#ifdef CONFIG_SERIAL_TERMIOS
	case TCGETS: {
		struct termios *termiosp = (struct termios *)arg;

		if (!termiosp) {
			ret = -EINVAL;
			break;
		}

		/* TODO:  Other termios fields are not yet returned.
		 * Note that cfsetospeed is not necessary because we have
		 * knowledge that only one speed is supported.
		 * Both cfset(i|o)speed() translate to cfsetspeed.
		 */

		cfsetispeed(termiosp, priv->baud);
	}
	break;

	case TCSETS: {
		struct termios *termiosp = (struct termios *)arg;
		uint32_t           lcr;  /* Holds current values of line control register */
		uint16_t           dl;   /* Divisor latch */
		uint8_t            dl2;  /* Divisor latch adjust */

		if (!termiosp) {
			ret = -EINVAL;
			break;
		}

		/* TODO:  Handle other termios settings.
		 * Note that only cfgetispeed is used because we have knowledge
		 * that only one speed is supported.
		 */

		/* Get the c_speed field in the termios struct */

		priv->baud = cfgetispeed(termiosp);

		/* TODO: Re-calculate the optimal CCLK divisor for the new baud and
		 * and reset the divider in the CLKSEL0/1 register.
		 */

		/* DLAB open latch */
		/* REVISIT:  Shouldn't we just call up_setup() to do all of the following? */

		lcr = getreg32(priv->uartbase + RDA_UART_LCR_OFFSET);
		up_serialout(priv, RDA_UART_LCR_OFFSET, (lcr | UART_LCR_DLAB));

		/* Set the BAUD divisor */

		dl = rda_uartdl(priv->baud, &dl2);
		up_serialout(priv, RDA_UART_DLM_OFFSET, dl >> 8);
		up_serialout(priv, RDA_UART_DLL_OFFSET, dl & 0xff);
		up_serialout(priv, RDA_UART_DL2_OFFSET, dl2);

		/* Clear DLAB */

		up_serialout(priv, RDA_UART_LCR_OFFSET, lcr);
	}
	break;
#endif

	default:
		ret = -ENOTTY;
		break;
	}

	return ret;
}

/****************************************************************************
 * Name: up_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int up_receive(struct uart_dev_s *dev, uint32_t *status)
{
	struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
	uint32_t rbr;

	*status = up_serialin(priv, RDA_UART_LSR_OFFSET);
	rbr     = up_serialin(priv, RDA_UART_RBR_OFFSET) &  0x00FFUL;
	return rbr;
}

/****************************************************************************
 * Name: up_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void up_rxint(struct uart_dev_s *dev, bool enable)
{

	struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
	if (enable) {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
		priv->ier |= UART_IER_RBRIE;
#endif
	} else {
		priv->ier &= ~UART_IER_RBRIE;
	}

	up_serialout(priv, RDA_UART_IER_OFFSET, priv->ier);
}

/****************************************************************************
 * Name: up_rxavailable
 *
 * Description:
 *   Return true if the receive fifo is not empty
 *
 ****************************************************************************/

static bool up_rxavailable(struct uart_dev_s *dev)
{
	struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
	return ((up_serialin(priv, RDA_UART_LSR_OFFSET) & UART_LSR_RDR) != 0);
}

/****************************************************************************
 * Name: up_send
 *
 * Description:
 *   This method will send one byte on the UART
 *
 ****************************************************************************/

static void up_send(struct uart_dev_s *dev, int ch)
{
	struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
	up_serialout(priv, RDA_UART_THR_OFFSET, (uint32_t)ch);
}

/****************************************************************************
 * Name: up_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void up_txint(struct uart_dev_s *dev, bool enable)
{
	struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
//  irqstate_t flags;

	//flags = enter_critical_section();
	if (enable) {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
		priv->ier |= UART_IER_THREIE;
		up_serialout(priv, RDA_UART_IER_OFFSET, priv->ier);

		/* Fake a TX interrupt here by just calling uart_xmitchars() with
		 * interrupts disabled (note this may recurse).
		 */
#if 0
		uart_xmitchars(dev);
#endif
#endif
	} else {
		priv->ier &= ~UART_IER_THREIE;
		up_serialout(priv, RDA_UART_IER_OFFSET, priv->ier);
	}

	//leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_txready
 *
 * Description:
 *   Return true if the tranmsit fifo is not full
 *
 ***`*************************************************************************/

static bool up_txready(struct uart_dev_s *dev)
{
	struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
	return ((up_serialin(priv, RDA_UART_LSR_OFFSET) & UART_LSR_THRE) != 0);
}

/****************************************************************************
 * Name: up_txempty
 *
 * Description:
 *   Return true if the transmit fifo is empty
 *
 ****************************************************************************/

static bool up_txempty(struct uart_dev_s *dev)
{
	struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
	return ((up_serialin(priv, RDA_UART_LSR_OFFSET) & UART_LSR_THRE) != 0);
}

/****************************************************************************
 * Public Funtions
 ****************************************************************************/

/****************************************************************************
 * Name: up_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before up_serialinit.
 *
 *   NOTE: Configuration of the CONSOLE UART was performed by up_lowsetup()
 *   very early in the boot sequence.
 *
 ****************************************************************************/

void up_earlyserialinit(void)
{
	/* Configure all UARTs (except the CONSOLE UART) and disable interrupts */

#ifdef CONFIG_RDA5981X_UART0
#ifndef CONFIG_UART0_SERIAL_CONSOLE
	rda_uart0config();
#endif

	g_uart0priv.uartbase       = RDA_UART0_BASE;
	g_uart0priv.baud           = CONFIG_UART0_BAUD;
	g_uart0priv.irq            = RDA_IRQ_UART0;
	g_uart0priv.parity         = CONFIG_UART0_PARITY;
	g_uart0priv.bits           = CONFIG_UART0_BITS;
	g_uart0priv.stopbits2      = CONFIG_UART0_2STOP;
	up_disableuartint(&g_uart0priv, NULL);
#endif

#ifdef CONFIG_RDA5981X_UART1
#ifndef CONFIG_UART1_SERIAL_CONSOLE
	rda_uart1config();
#else
#endif
	up_disableuartint(&g_uart1priv, NULL);
#endif

	/* Configuration whichever one is the console */

	CONSOLE_DEV.isconsole = true;
	g_uart_ops.setup = up_setup;
	g_uart_ops.shutdown = up_shutdown;
	g_uart_ops.attach  = up_attach;
	g_uart_ops.detach  = up_detach;
	g_uart_ops.ioctl    = up_ioctl;
	g_uart_ops.receive   = up_receive;
	g_uart_ops.rxint          = up_rxint;
	g_uart_ops.rxavailable    = up_rxavailable;
#ifdef CONFIG_SERIAL_IFLOWCONTROL
	g_uart_ops.rxflowcontrol  = NULL;
#endif
	g_uart_ops.send = up_send;
	g_uart_ops.txint  = up_txint;
	g_uart_ops.txready        = up_txready;
	g_uart_ops.txempty        = up_txempty;

	g_uart0port.recv.size = CONFIG_UART0_RXBUFSIZE;
	g_uart0port.recv.buffer = g_uart0rxbuffer;

	g_uart0port.xmit.size = CONFIG_UART0_TXBUFSIZE;
	g_uart0port.xmit.buffer = g_uart0txbuffer;
	g_uart0port.ops      = &g_uart_ops;
	g_uart0port.priv     = &g_uart0priv;

	up_setup(&CONSOLE_DEV);
}

/****************************************************************************
 * Name: up_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes that
 *   up_earlyserialinit was called previously.
 *
 ****************************************************************************/

void up_serialinit(void)
{
#ifdef CONSOLE_DEV
	(void)uart_register("/dev/console", &CONSOLE_DEV);
#endif
#ifdef TTYS0_DEV
	(void)uart_register("/dev/ttyS0", &TTYS0_DEV);
#endif
#ifdef TTYS1_DEV
	(void)uart_register("/dev/ttyS1", &TTYS1_DEV);
#endif
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug  writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
#ifdef HAVE_CONSOLE
	struct up_dev_s *priv = (struct up_dev_s *)CONSOLE_DEV.priv;
	uint32_t ier;
	up_disableuartint(priv, &ier);
#endif

	/* Check for LF */

	if (ch == '\n') {
		/* Add CR */

		up_lowputc('\r');
	}

	up_lowputc(ch);
#ifdef HAVE_CONSOLE
	up_restoreuartint(priv, ier);
#endif

	return ch;
}

#else /* USE_SERIALDRIVER */

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
#ifdef HAVE_UART
	/* Check for LF */

	if (ch == '\n') {
		/* Add CR */

		up_lowputc('\r');
	}

	up_lowputc(ch);
#endif
	return ch;
}

#endif
