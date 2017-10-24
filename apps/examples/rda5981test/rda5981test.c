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
 * examples/hello_tash/hello_tash_main.c
 *
 *   Copyright (C) 2008, 2011-2012 Gregory Nutt. All rights reserved.
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
#include <stdio.h>
#include <unistd.h>
#include <apps/shell/tash.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/
#define HELLO_TASH_PRI      100
#define HELLO_TASH_STAKSIZE 1024

/****************************************************************************
 * Private Data & Functions
 ****************************************************************************/
/* example */
#include <tinyara/i2c.h>
#include <tinyara/config.h>
#define RDA_WDT_BASE  ((0x40000000UL)+ 0x0000C)
#include "../arch/arm/src/rda5981/rda5981x_watchdog.h"

static struct i2c_dev_s *i2c_dev;
static struct i2c_config_s configs;

void i2c_test(void)
{
        uint8_t data[3];
        int ret;
	int port = 0;

	printf("This is an example to add it in tash\n");
	printf("Hello, World!!\n");

        printf("transfer begin\n");
        i2c_dev = up_i2cinitialize(port);
        if (i2c_dev == NULL) {
                printf("i2ctest_main: up_i2cinitialize(i2c:%d) failed\n", port);
		return;
        }

        configs.frequency = 100000;
        configs.address = 0x14;
        configs.addrlen = 7;

        data[0] = 0xCD;
        data[1] = 0x0F;
        data[1] = 0x07;

        ret = i2c_write(i2c_dev, &configs, data, 3);
        if (ret < 0) {
                printf("i2c_write fail(%d)\n", ret);
                return;
        }

        printf("transfer end\n");
       // up_mdelay(1);

#if 0
        ret = i2c_read(i2c_dev, &configs, data, 1);
        if (ret < 0) {
                printf("i2c_read fail(%d)\n", ret);
                return -ret;
        }
#endif	

	return;
}

void watchdog_test(void)
{

    wdt_t obj;
    int cnt = 0;

    rda_wdt_init(&obj, 2);
    rda_wdt_start(&obj);
    while(true) {
        printf("Alive loop\r\n");
        //mdelay(100); 
	if(cnt == 10) {
            while(true) {
                printf("Dead loop\r\n");
            }
        }
        cnt++;
        rda_wdt_feed(&obj);
    }

}

static void *hello_example(void *arg)
{
/*I2C TEST */
	//i2c_test();

/*WDG TEST*/
	watchdog_test();
	
	return NULL;
}

/*  Call-back function registered in TASH.
 *   This creates pthread to run an example with ASYNC TASH excution type.
 *   Only three points can be modified
 *   1. priority
 *   2. stacksize
 *   3. register entry function of pthread (example)
 */
static int hello_tash_cb(int argc, char **args)
{
	pthread_t hello_tash;
	pthread_attr_t attr;
	struct sched_param sparam;
	int status;
#ifdef SDCC
	pthread_addr_t result;
#endif

	/* Initialize the attribute variable */
	status = pthread_attr_init(&attr);
	if (status != 0) {
		printf("hello_tash : pthread_attr_init failed, status=%d\n", status);
	}

	/* 1. set a priority */
	sparam.sched_priority = HELLO_TASH_PRI;
	status = pthread_attr_setschedparam(&attr, &sparam);
	if (status != OK) {
		printf("hello_tash : pthread_attr_setschedparam failed, status=%d\n", status);
	}

	/* 2. set a stacksize */
	status = pthread_attr_setstacksize(&attr, HELLO_TASH_STAKSIZE);
	if (status != OK) {
		printf("hello_tash : pthread_attr_setstacksize failed, status=%d\n", status);
	}

	/* 3. create pthread with entry function */
	status = pthread_create(&hello_tash, &attr, hello_example, (void *)args);
	if (status != 0) {
		printf("hello_tash: pthread_create failed, status=%d\n", status);
	}

	/* Wait for the threads to stop */
#ifdef SDCC
	pthread_join(hello_tash, &result);
#else
	pthread_join(hello_tash, NULL);
#endif

	printf("hello_tash is finished\n");
	return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * hello_tash_main
 ****************************************************************************/
#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int rda5981_test_main(int argc, char **args)
#endif
{
	tash_cmd_install("rda5981_test", hello_tash_cb, TASH_EXECMD_ASYNC);

	return 0;
}
