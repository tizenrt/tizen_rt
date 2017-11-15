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
 * examples/kernel_sample/kernel_sample_main.c
 *
 *   Copyright (C) 2007-2009, 2011-2012, 2014 Gregory Nutt. All rights reserved.
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

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <sched.h>
#include <errno.h>

#include <tinyara/init.h>
#include "kernel_sample.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define PRIORITY         100
#define NARGS              4
#define HALF_SECOND_USEC 500000L

/****************************************************************************
 * Name: user_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
static int user_main(int argc, char *argv[])
#endif
{

        printf("\nuser_main: mutex test\n");
		mutex_test();

		printf("\nuser_main: semaphore test\n");
		sem_test();

		printf("\nuser_main: timed semaphore test\n");
		semtimed_test();
		
        printf("\nuser_main: watchdog timer test\n");
		timer_test();
    	
        printf("user_main: Exitting\n");
	    return 0;
}


/****************************************************************************
 * Public Functions
 ****************************************************************************/
static const char arg1[] = "Arg1";
static const char arg2[] = "Arg2";
static const char arg3[] = "Arg3";
static const char arg4[] = "Arg4";

static const char *g_argv[NARGS + 1] = { arg1, arg2, arg3, arg4, NULL };

#ifndef CONFIG_BUILD_KERNEL
int kernel_sample_main(int argc, FAR char *argv[])
{
/* Verify that we can spawn a new task */
    int result;
	result = task_create("kernel_sample", PRIORITY, STACKSIZE, user_main,
						 (FAR char *const *)g_argv);
	if (result == ERROR) {
		printf("kernel_sample_main: ERROR Failed to start user_main\n");
	} else {
		printf("kernel_sample_main: Started user_main at PID=%d\n", result);
	}

	printf("kernel_sample_main: Exiting\n");
	return 0;
}
#endif
