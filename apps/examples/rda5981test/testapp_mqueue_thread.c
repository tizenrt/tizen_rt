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
#include <stdlib.h>
#include <unistd.h>
#include <apps/shell/tash.h>
#include "../../../os/arch/arm/src/chip/sys_wrapper.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/
#define HELLO_TASH_PRI      100
#define HELLO_TASH_STAKSIZE 1024

/****************************************************************************
 * Private Data & Functions
 ****************************************************************************/
/* example */
static void *hello_example(void *arg)
{
	(void)arg;
	printf("This is an example to add it in tash\n");
	printf("Hello, World!!\n");
	return NULL;
}

/*  Call-back function registered in TASH.
 *   This creates pthread to run an example with ASYNC TASH excution type.
 *   Only three points can be modified
 *   1. priority
 *   2. stacksize
 *   3. register entry function of pthread (example)
 */
static int testapp_mqueue_thread_tash_cb(int argc, char **args)
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


static rda_thread_t master_thread, slave_thread;
static int master_status, slave_status;
static rda_msgq_t msgq;
static int mq_status;


static void show_cli_menu(void){
    printf("***** console client menu                *****\n");
	if (master_status == 0)
    printf("***** 11. create master thread           *****\n");
	if (slave_status == 0)
    printf("***** 12. create slave thread            *****\n"); 
	if (master_status == 1) 
    printf("***** 13. destroy master thread          *****\n");
	if (slave_status == 1)
    printf("***** 14. destry slave thread            *****\n");
	if (mq_status == 0)
    printf("***** 21. create message queue           *****\n");
	if (mq_status == 1)
    printf("***** 22. destry message queue           *****\n");
	if (mq_status == 1)
    printf("***** 23. send to message queue          *****\n");
	if (mq_status == 1)
    printf("***** 24. get from message queue         *****\n");
    printf("*****  9. help                           *****\n");
    printf("***** 99. exit                           *****\n");
	
}

const char* msg[10] = {
	"welcome",
	"good",
	"good",
	"study",
	"day",
	"day",
	"up",
	"fignting",
	"expect",
	"future"
};
static int msg_index;
static int msg_index_recieve;

void* master_thread_function(void* user_data) {
		if (mq_status && msg_index < 10) {
            char* write_msg = calloc(20, 1);
            strncpy(write_msg, msg[msg_index++%10], 19);
            write_msg[19] = '\0';
			std_msgqueue_put(&msgq, write_msg);
			printf("write_msg is %s\n", write_msg);
		}
		
return NULL;
}

void* slave_thread_function(void* user_data) {
        int ret;
        sleep(1);

		if (mq_status && msg_index_recieve > 0) {
			char *read_msg;
			ret = std_msgqueue_get(&msgq, (void**)&read_msg, 10000);
			printf("read_msg is %s\n",read_msg);
			--msg_index_recieve;
		}

	return NULL;
}

static int std_thread_std_messagequeue_test_fun(int argc, char *argv[])
{
	master_status = 0;
	slave_status = 0;
	mq_status = 0;
	msg_index_recieve = 10;

    printf("This is an example to add it in tash\n");
    show_cli_menu();

   // bool running = true;


#if 0
    while (running) {
        printf("Please enter your choice ...\n");
        char consoleCmd[64] = { 0, };
        if (fgets(consoleCmd, 64, stdin) != NULL) {
            while (consoleCmd[strlen(consoleCmd)-1] == '\n') {
                consoleCmd[strlen(consoleCmd)-1] = '\0';
            }
            if (strlen(consoleCmd) == 0) {
                continue;
            }
            int intCmd = atoi(consoleCmd);
            printf("Test Command is [%s][%d]\n", consoleCmd, intCmd);

            switch (intCmd) {
                case 11:
                {
                    if (std_thread_create(&master_thread, (rda_thread_fn)master_thread_function, NULL, 1024, 100) >= 0) {
						master_status = 1;
					}
                    break;
                }
                case 12:
				{
                    if (std_thread_create(&slave_thread, (rda_thread_fn)slave_thread_function, NULL, 1024, 100) >= 0) {
						slave_thread = 1;
					}
                    break;
                }
                case 13:
                {
					if (std_thread_delete(&master_thread) >= 0) {
						master_status = 0;
					}
					break;
                }
                case 14:
				{
					if (std_thread_delete(&slave_thread) >= 0) {
						slave_thread = 0;
					}
					break;
				}
                case 21:
                {
                    std_msgqueue_create(&msgq, 20);
					mq_status = 1;
                    break;
                }
                case 22:
				{
					std_msgqueue_delete(&msgq);
					mq_status = 0;
					break;
				}


                case 23:
                    break;

                case 24:
                    break;

                case 9:
                    show_cli_menu();
                    break;

                case 99:
                    running = false;
                    break;

                default:
                    break;
            }
        }
    }
    
#endif
    
        std_msgqueue_create(&msgq, 20);
        mq_status = 1;
        std_thread_create(&master_thread, (rda_thread_fn)master_thread_function, NULL, 1024, 100);
        std_thread_create(&slave_thread, (rda_thread_fn)slave_thread_function, NULL, 1024, 100);
       
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
int testapp_mqueue_thread_main(int argc, char **args)
#endif
{
	tash_cmd_install("test_mq_thread", std_thread_std_messagequeue_test_fun, TASH_EXECMD_ASYNC);

	return 0;
}
