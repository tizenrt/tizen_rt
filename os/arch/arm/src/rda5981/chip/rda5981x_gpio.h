/************************************************************************************
 * arch/arm/src/rda5981x/chip/rda5981x_gpio.h
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

#ifndef __ARCH_ARM_SRC_RDA5981X_CHIP_RDA5981X_GPIO_H
#define __ARCH_ARM_SRC_RDA5981X_CHIP_RDA5981X_GPIO_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <tinyara/config.h>

#include "chip.h"
#include "chip/rda5981x_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/
/* GPIO block register offsets ******************************************************/

#define RDA_GPIO_CTRL_OFFSET            0x0000 /* GPIO Control                */
#define RDA_GPIO_DOUT_OFFSET            0x0008 /* GPIO Data Output            */
#define RDA_GPIO_DIN_OFFSET             0x000C /* GPIO Data Input             */
#define RDA_GPIO_DIR_OFFSET             0x0010 /* GPIO Direction              */
#define RDA_GPIO_SLEW0_OFFSET           0x0014 /* GPIO Slew Config 0          */
#define RDA_GPIO_SLEWIOMUX_OFFSET       0x0018 /* GPIO IOMUX Slew Config      */
#define RDA_GPIO_INTCTRL_OFFSET         0x001C /* GPIO Interrupt Control      */
#define RDA_GPIO_IFCTRL_OFFSET          0x0020 /* Interface Control           */
#define RDA_GPIO_SLEW1_OFFSET           0x0024 /* GPIO Slew Config 1          */
#define RDA_GPIO_REVID_OFFSET           0x0028 /* ASIC Reversion ID           */
#define RDA_GPIO_LPOSEL_OFFSET          0x002C /* LPO Select                  */
#define RDA_GPIO_INTSEL_OFFSET          0x0034 /* GPIO Interrupt Select       */
#define RDA_GPIO_SDIOCFG_OFFSET         0x003C /* SDIO Config                 */
#define RDA_GPIO_MEMCFG_OFFSET          0x0040 /* Memory Config               */
#define RDA_GPIO_PCCTRL_OFFSET          0x0064 /* Pulse Counter Control       */

/* Register addresses ***************************************************************/
/* GPIO block register addresses ****************************************************/

#define RDA_GPIO_CTRL       (RDA_GPIO_BASE + RDA_GPIO_CTRL_OFFSET     ) /* 0x0000 : GPIO Control           */
#define RDA_GPIO_DOUT       (RDA_GPIO_BASE + RDA_GPIO_DOUT_OFFSET     ) /* 0x0008 : GPIO Data Output       */
#define RDA_GPIO_DIN        (RDA_GPIO_BASE + RDA_GPIO_DIN_OFFSET      ) /* 0x000C : GPIO Data Input        */
#define RDA_GPIO_DIR        (RDA_GPIO_BASE + RDA_GPIO_DIR_OFFSET      ) /* 0x0010 : GPIO Direction         */
#define RDA_GPIO_SLEW0      (RDA_GPIO_BASE + RDA_GPIO_SLEW0_OFFSET    ) /* 0x0014 : GPIO Slew Config 0     */
#define RDA_GPIO_SLEWIOMUX  (RDA_GPIO_BASE + RDA_GPIO_SLEWIOMUX_OFFSET) /* 0x0018 : GPIO IOMUX Slew Config */
#define RDA_GPIO_INTCTRL    (RDA_GPIO_BASE + RDA_GPIO_INTCTRL_OFFSET  ) /* 0x001C : GPIO Interrupt Control */
#define RDA_GPIO_IFCTRL     (RDA_GPIO_BASE + RDA_GPIO_IFCTRL_OFFSET   ) /* 0x0020 : Interface Control      */
#define RDA_GPIO_SLEW1      (RDA_GPIO_BASE + RDA_GPIO_SLEW1_OFFSET    ) /* 0x0024 : GPIO Slew Config 1     */
#define RDA_GPIO_REVID      (RDA_GPIO_BASE + RDA_GPIO_REVID_OFFSET    ) /* 0x0028 : ASIC Reversion ID      */
#define RDA_GPIO_LPOSEL     (RDA_GPIO_BASE + RDA_GPIO_LPOSEL_OFFSET   ) /* 0x002C : LPO Select             */
#define RDA_GPIO_INTSEL     (RDA_GPIO_BASE + RDA_GPIO_INTSEL_OFFSET   ) /* 0x0034 : GPIO Interrupt Select  */
#define RDA_GPIO_SDIOCFG    (RDA_GPIO_BASE + RDA_GPIO_SDIOCFG_OFFSET  ) /* 0x003C : SDIO Config            */
#define RDA_GPIO_MEMCFG     (RDA_GPIO_BASE + RDA_GPIO_MEMCFG_OFFSET   ) /* 0x0040 : Memory Config          */
#define RDA_GPIO_PCCTRL     (RDA_GPIO_BASE + RDA_GPIO_PCCTRL_OFFSET   ) /* 0x0064 : Pulse Counter Control  */

/* Register bit definitions *********************************************************/
/* GPIO block register bit definitions **********************************************/

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_RDA5981X_CHIP_RDA5981X_GPIO_H */
