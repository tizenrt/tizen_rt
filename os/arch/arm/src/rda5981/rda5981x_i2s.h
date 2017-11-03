/* mbed Microcontroller Library
 * Copyright (c) 2006-2015 ARM Limited
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
#ifndef RDA_I2S_API_H
#define RDA_I2S_API_H

#include <stdint.h>
#include "rda5981x_pwm.h"
#include "rda5981x_gpio.h"
#include "chip/rda5981x_pinconfig.h"
#include "chip/rda5981x_memorymap.h"


#ifndef BOOL
#define BOOL uint8_t
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

/* I2S HAL enumeration
 */
#define __O volatile
#define __IO volatile
#define __I volatile
typedef enum {
	I2S_MD_MASTER_RX = 0x00,
	I2S_MD_MASTER_TX = 0x01,
	I2S_MD_SLAVE_RX  = 0x02,
	I2S_MD_M_TX_S_RX = 0x03
} RDA_I2S_MODE_T;

typedef enum {
	I2S_ST_RESET = 0,
	I2S_ST_READY,
	I2S_ST_BUSY
} RDA_I2S_SW_STATE_T;

typedef struct {
	volatile RDA_I2S_SW_STATE_T state;
	uint32_t *buffer;
	uint16_t bufferCntr;
	uint16_t bufferSize;
	uint8_t  regBitOfst;
	uint8_t  fifoThreshold;
	uint8_t  channel;
	uint8_t  dataWidth;
} RDA_I2S_SW_PARAM_T;


struct i2s_s {
	RDA_I2S_TypeDef *i2s;
};


typedef enum {
	I2S_64FS  = 0x00,
	I2S_32FS  = 0x01
} I2S_FS_T;

typedef enum {
	I2S_WS_NEG  = 0x00,
	I2S_WS_POS  = 0x01
} I2S_WS_POLAR_T;

typedef enum {
	I2S_OTHER_M = 0x00,
	I2S_STD_M   = 0x01
} I2S_STD_MODE_T;

typedef enum {
	I2S_RIGHT_JM = 0x00,
	I2S_LEFT_JM  = 0x01
} I2S_JUSTIFIED_MODE_T;

typedef enum {
	I2S_DL_8b   = 0x00,
	I2S_DL_12b  = 0x01,
	I2S_DL_16b  = 0x02,
	I2S_DL_20b  = 0x03,
	I2S_DL_24b  = 0x04
} I2S_DATA_LEN_T;

typedef enum {
	I2S_LSB     = 0x00,
	I2S_MSB     = 0x01
} I2S_MSB_LSB_T;

typedef enum {
	I2S_WF_CNTLFT_1W    = 0x00,
	I2S_WF_CNTLFT_2W    = 0x01,
	I2S_WF_CNTLFT_4W    = 0x02,
	I2S_WF_CNTLFT_8W    = 0x03
} I2S_WRFIFO_CNTLEFT_T;

typedef enum {
	TX_SEM = 0x00,
	RX_SEM = 0x01
} RDA_I2S_SEM_T;

/** I2S HAL structure
 */
typedef struct {
	uint8_t fs;
	uint8_t ws_polarity;
	uint8_t std_mode;
	uint8_t justified_mode;
	uint8_t data_len;
	uint8_t msb_lsb;
	uint8_t wrfifo_cntleft;
} RDA_I2S_HW_PARAM_T;

typedef struct {
	struct i2s_s hw;
	RDA_I2S_SW_PARAM_T sw_tx;
	RDA_I2S_SW_PARAM_T sw_rx;
} i2s_t;

typedef struct {
	__IO uint32_t CTRL;                   /* 0x00 : GPIO Control                */
	uint32_t RESERVED0;
	__IO uint32_t DOUT;                   /* 0x08 : GPIO Data Output            */
	__IO uint32_t DIN;                    /* 0x0C : GPIO Data Input             */
	__IO uint32_t DIR;                    /* 0x10 : GPIO Direction              */
	__IO uint32_t SLEW0;                  /* 0x14 : GPIO Slew Config 0          */
	__IO uint32_t SLEWIOMUX;              /* 0x18 : GPIO IOMUX Slew Config      */
	__IO uint32_t INTCTRL;                /* 0x1C : GPIO Interrupt Control      */
	__IO uint32_t IFCTRL;                 /* 0x20 : Interface Control           */
	__IO uint32_t SLEW1;                  /* 0x24 : GPIO Slew Config 1          */
	__IO uint32_t REVID;                  /* 0x28 : ASIC Reversion ID           */
	__IO uint32_t LPOSEL;                 /* 0x2C : LPO Select                  */
	uint32_t RESERVED1;
	__IO uint32_t INTSEL;                 /* 0x34 : GPIO Interrupt Select       */
	uint32_t RESERVED2;
	__IO uint32_t SDIOCFG;                /* 0x3C : SDIO Config                 */
	__IO uint32_t MEMCFG;                 /* 0x40 : Memory Config               */
	__IO uint32_t IOMUXCTRL[8];           /* 0x44 - 0x60 : IOMUX Control        */
	__IO uint32_t PCCTRL;                 /* 0x64 : Pulse Counter Control       */
} RDA_GPIO_TypeDef;

typedef struct {
	RDA_I2S_MODE_T     mode;
	RDA_I2S_HW_PARAM_T tx;
	RDA_I2S_HW_PARAM_T rx;
} i2s_cfg_t;



/*------------- External Interface (EXIF) ------------------------------------*/

#define RDA_SCU  ((RDA_SCU_TypeDef*) RDA_SCU_BASE)
#define RDA_EXIF ((RDA_EXIF_TypeDef*) RDA_EXIF_BASE)
#define RDA_GPIO ((RDA_GPIO_TypeDef*) RDA_GPIO_BASE)
extern const RDA_I2S_SEM_T i2s_tx_sem;
extern const RDA_I2S_SEM_T i2s_rx_sem;
extern BOOL i2s_tx_enabled;
extern BOOL i2s_rx_enabled;

#ifdef __cplusplus
extern "C" {
#endif




/** Initialize the i2s module
 *
 */
void rda_i2s_init(i2s_t *obj);

/** De-Initialize the i2s module
 *
 */
void rda_i2s_deinit(i2s_t *obj);

/** Format the i2s signal
 *
 */
void rda_i2s_format(i2s_t *obj, i2s_cfg_t *cfg);

/** Enable i2s tx bclk out
 *
 */
void rda_i2s_enable_tx(i2s_t *obj);

/** Disable i2s tx bclk out
 *
 */
void rda_i2s_disable_tx(i2s_t *obj);

/** Enable i2s rx bclk in
 *
 */
void rda_i2s_enable_rx(i2s_t *obj);

/** Disable i2s rx bclk in
 *
 */
void rda_i2s_disable_rx(i2s_t *obj);

/** Enable i2s master rx, i2s_clk/ws out, i2s_data in
 *
 */
void rda_i2s_enable_master_rx(void);

/** Disable i2s master rx, i2s_clk/ws/data all in(out)
 *
 */
void rda_i2s_disable_master_rx(void);

/** Enable I2S out mute
 *
 */
void rda_i2s_out_mute(i2s_t *obj);

/** Set channel(MONO:1 STEREO:2)
 *
 */
uint8_t rda_i2s_set_channel(i2s_t *obj, uint8_t channel);

/** Set tx channel(MONO:1 STEREO:2)
 *
 */
uint8_t rda_i2s_set_tx_channel(i2s_t *obj, uint8_t channel);

/** Set rx channel(MONO:1 STEREO:2)
 *
 */
uint8_t rda_i2s_set_rx_channel(i2s_t *obj, uint8_t channel);

/** Set ws(LRCLK) and MCLK/LRCK ratio
 *
 */
uint8_t rda_i2s_set_ws(i2s_t *obj, uint32_t ws, uint32_t ratio);


/** Send data in blocking mode
 *
 */
uint8_t rda_i2s_blk_send(i2s_t *obj, uint32_t *buf, uint16_t size);

/** Receive data in blocking mode
 *
 */
uint8_t rda_i2s_blk_recv(i2s_t *obj, uint32_t *buf, uint16_t size);

/** Send data in interrupt mode
 *
 */
uint8_t rda_i2s_int_send(i2s_t *obj, uint32_t *buf, uint16_t size);

/** Receive data in interrupt mode
 *
 */
uint8_t rda_i2s_int_recv(i2s_t *obj, uint32_t *buf, uint16_t size);

/** The wrapper for i2s_irq_handler
 *
 */
void rda_i2s_irq_handler(uint32_t int_status);

/** I2S semaphore wait
 *
 */
int32_t rda_i2s_sem_wait(RDA_I2S_SEM_T i2s_sem);

/** I2S semaphore release
 *
 */
int rda_i2s_sem_release(RDA_I2S_SEM_T i2s_sem);

/** I2S semaphore delete
 *
 */
int rda_i2s_sem_delete(RDA_I2S_SEM_T i2s_sem);


/**@}*/

#ifdef __cplusplus
}
#endif

#endif

