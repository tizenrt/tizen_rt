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
 * examples/rda5981test/rda5981test.c
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
#include <tinyara/wdog.h>
#define RDA_WDT_BASE  ((0x40000000UL)+ 0x0000C)
#include "../arch/arm/src/rda5981/rda5981x_watchdog.h"
#include "../arch/arm/src/rda5981/rda5981x_pwm.h"

#include <tinyara/rtc.h>
#include <fcntl.h>


static struct i2c_dev_s *i2c_dev;
static struct i2c_config_s configs;

void i2c_test(void)
{
        uint8_t txbuf=0x03;
        uint8_t rxbuf[2];
        int ret;
	int port = 0;
        
	i2c_dev = up_i2cinitialize(port);
        if (i2c_dev == NULL) {
                printf("i2ctest_main: up_i2cinitialize(i2c:%d) failed\n", port);
		return;
        }

        configs.frequency = 100000;
        configs.address = 0x30;
        configs.addrlen = 7;

        ret = i2c_write(i2c_dev, &configs, &txbuf, 1);
        if (ret < 0) {
                printf("i2c_write fail(%d)\n", ret);
                return;
        }

	ret = i2c_read(i2c_dev, &configs, rxbuf, 2);
        if (ret < 0) {
                printf("i2c_read fail(%d)\n", ret);
                return;
        }
	printf("%02x %02x \r\n", rxbuf[0], rxbuf[1]);
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

// For ADC test
float adc_read(void)
{
    float val=0;
    val = analogin_read();
    return val;	
}

unsigned short adc_read_u16(void)
{
    unsigned short val=0;
    val = analogin_read_u16();
    return val;
}


void adc_test(void)
{
    int read_times=0;
    int loop_count;
    printf("Start AnalogIn test...\r\n");
    analogin_init();
    while (read_times < 10) {
        float fval = adc_read();
        unsigned short ival = adc_read_u16();
        /* Print the percentage and 16-bit normalized values */
        printf("percentage: %3.3f%%\r\n", fval*100.0f);
        printf("normalized: 0x%04X\r\n\r\n", ival);
        loop_count = 100000000;
        while(loop_count>0)
	{
            loop_count--;
        }
        read_times++;
    }
}


/*For PWM test*/
struct pwmout_s pwm0;
struct pwmout_s pwm1;
struct pwmout_s pwm2;
struct pwmout_s pwm3;

void pwm_test(void) 
{
/* pwm0-pwm3 is avaliable*/
    pwmout_init(&pwm0, 0);
    pwmout_period_ms(&pwm0, 4.0f); //4ms
    pwmout_write(&pwm0,0.25f);   //duty ratio

    pwmout_init(&pwm1, 1);
    pwmout_period_ms(&pwm1, 4.0f); 
    pwmout_write(&pwm1,0.50f);   
   
    pwmout_init(&pwm2, 2);
    pwmout_period_ms(&pwm2, 4.0f); 
    pwmout_write(&pwm2,0.75f);   
 
    pwmout_init(&pwm3, 3);
    pwmout_period_ms(&pwm3, 8.0f); 
    pwmout_write(&pwm3,0.25f);   
 }


//SPI test
enum spi_mode_e {
	SPIDEV_MODE0 = 0,			/* CPOL=0 CHPHA=0 */
	SPIDEV_MODE1,				/* CPOL=0 CHPHA=1 */
	SPIDEV_MODE2,				/* CPOL=1 CHPHA=0 */
	SPIDEV_MODE3				/* CPOL=1 CHPHA=1 */
};


void spi_test(void)
{
    int rdata, wdata = 0x00;

    printf("Start SPI test...\r\n");

    /* Setup the spi for 8 bit data, high state clock, */
    /* second edge capture, with a 1MHz clock rate     */
    //spi.format(8, 3);
    up_spiinitialize(0);
	printf("after up_spiinitialize!\n");
    //spi_setbits(NULL,8);
    //spi_setmode(NULL,3);
    spi_format(8,3,0);
    spi_setfrequency(NULL,10000000);// 1MHZ, 10MHZ 


    while(true) {
        /* Send 1 byte */
        //spi.write(wdata);
        spi_send(NULL,wdata);
        wdata++;
        wdata &= 0x00FF;
        printf("Send data: 0x%X\r\n", wdata);

        /* Send 1 dummy byte to receive data from slave SPI */
        rdata = spi_send(NULL,0x00);
        printf("Recv data: 0x%X\r\n", rdata);
    }
}


// RTC test
#define CUSTOM_TIME  1256729737



int rtc_openDev(void)
{
	const char * devName = "/dev/rtc0";
	int fd;
	fd = open(devName, O_RDONLY);
	if (fd < 0) {
		printf("open %s failed\n", devName);
		return -1;
	}

	return fd;
}


int rtc_setDate(int fd, struct rtc_time *currentTime)
{
	int ret;
	
	ret = ioctl(fd, RTC_SET_TIME, currentTime);
	if(ret < 0)
	{
		printf("rtc_setDate failed!\n");
		return -1;
	}	

	return 0;
}


int rtc_getDateTime(int fd, struct rtc_time *currentTime)
{
	int ret;
	
	ret = ioctl(fd, RTC_RD_TIME, currentTime);
	if(ret < 0)
	{
		printf("rtc_getDateTime failed!\n");
		return -1;
	}	

	return 0;
}


void rtc_closeDev(int fd)
{	
	close(fd);
}


void convertTime(struct rtc_time *currentTime, bool setflage)
{
	if(currentTime)
	{
		if(setflage)
		{
			currentTime->tm_mon = currentTime->tm_mon - 1; //convert month
			currentTime->tm_year = currentTime->tm_year - 1900; //convert year
		}
		else
		{
			currentTime->tm_mon = currentTime->tm_mon + 1; //convert month
			currentTime->tm_year = currentTime->tm_year + 1900; //convert year
		}
	}

}


void rtc_test(void) 
{
   // char buffer[32] = {0};
	struct rtc_time currentTime,newTime;
	int fd;
	
	fd = rtc_openDev();
	if(fd <0)
	{
		return;
	}

	// Set RTC time to Wed, 28 Oct 2009 11:35:37
	currentTime.tm_sec = 37;
	currentTime.tm_min = 35;
	currentTime.tm_hour = 11;
	currentTime.tm_mday = 28;
	currentTime.tm_mon = 10;
	currentTime.tm_year = 2009;
	convertTime(&currentTime,true);
	
	rtc_setDate(fd,&currentTime);

    while (true) 
	{
        /* Delay 1s using systick timer */
        usleep(1000*1000);

        /* Get RTC timer */
		//time_t seconds;
        //seconds = time(NULL);
        //strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S %p", localtime(&seconds));
        //printf("[%ld] %s\r\n", seconds, buffer);

		rtc_getDateTime(fd,&newTime);
		convertTime(&newTime,false); //get time
		printf("Current time: %u-%u-%u-%u-%u-%u\n",
			newTime.tm_year,newTime.tm_mon,newTime.tm_mday,newTime.tm_hour,newTime.tm_min,newTime.tm_sec);	
    }	

	rtc_closeDev(fd);
	
	return;
}

void busfault_raise_test(void)
{
    int *p = (int *)0xffffffff;
    *p = 1;
}

void heap_allocate_test(void)
{
    int *p;
    bool I_SRAM = 0;
    bool D_SRAM = 0;

    while(1)
    {
        p = (int*)malloc(16*1024);
        
        if(p == NULL)
        {
            printf("error\n");
            return;
        }

        if( (unsigned int)p > 0x00180000)
        {
           printf("heap2 D_SRAM memory 0x%08x\n", p);
           *p = 1;
           if(D_SRAM == false)
               D_SRAM = true;
        }

        else
        {
            printf("heap1 I_SRAM memory 0x%08x\n", p);
            *p=1;

                if(I_SRAM == false)
                    I_SRAM = true;
        }

        if(I_SRAM && D_SRAM)
        {
            return;
        }
        
    }

}

#if 0
#define DELAY 10 * 10000
static void test_timeout(int argc, uint32_t arg)
{
        printf("test timeout\n");
}
WDOG_ID dog;	
#endif
static void *hello_example(void *arg)
{

   heap_allocate_test();

//  busfault_raise_test();

/*I2C TEST */
	//i2c_test();

/*WDG TEST*/
	//watchdog_test();

/*ADC TEST*/
	//adc_test();

/*PWM TEST*/
	//pwm_test();
	
/*SPI TEST*/
	//spi_test();	

/*I2S TEST*/
	//i2s_test();

/*RTC TEST*/
	//rtc_test();
	
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
