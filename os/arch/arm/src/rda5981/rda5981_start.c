/****************************************************************************
 * arch/arm/src/nuc1xx/nuc_start.c
 * arch/arm/src/chip/nuc_start.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <tinyara/init.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"
volatile bool g_rtc_enabled = false;
volatile uint32_t *current_regs;

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Memory Map:
 *
 * 0x0000:0000 - Beginning of FLASH. Address of exception vectors.
 * 0x0001:ffff - End of flash (assuming 128KB of FLASH)
 * 0x2000:0000 - Start of SRAM and start of .data (_sdata)
 *             - End of .data (_edata) abd start of .bss (_sbss)
 *             - End of .bss (_ebss) and bottom of idle stack
 *             - _ebss + CONFIG_IDLETHREAD_STACKSIZE = end of idle stack,
 *               start of heap
 * 0x2000:3fff - End of SRAM and end of heap (assuming 16KB of SRAM)
 */

#define IDLE_STACK ((uint32_t)&_ebss+CONFIG_IDLETHREAD_STACKSIZE-4)
#define HEAP_BASE  ((uint32_t)&_ebss+CONFIG_IDLETHREAD_STACKSIZE)

/****************************************************************************
 * Public Data
 ****************************************************************************/


/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: showprogress
 *
 * Description:
 *   Print a character on the UART to show boot status.
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG_FEATURES) && defined(HAVE_SERIAL_CONSOLE)
#  define showprogress(c) nuc_lowputc((uint32_t)c)
#else
#  define showprogress(c)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _start
 *
 * Description:
 *   This is the reset entry point.
 *
 ****************************************************************************/

void SystemInit (void)
{
    __asm__ __volatile__ ("cpsie i" ::: "memory");

}

void up_serialinit(void)
{

}


#define RDA_AHB0_BASE         (0x40000000UL)
#define RDA_UART0_BASE        (RDA_AHB0_BASE + 0x12000)
#define UART_TXH 0x0



void board_initialize(void)
{


}

void up_rtc_initialize(void)
{

}

int up_rtc_getdatetime(FAR struct tm *tp)
{
    return 0;
}

int up_rtc_settime(FAR const struct timespec *tp)
{
    return 0;

}


void __start(void)
{
  uint32_t *dest;
  
  rda_printf("===TizenRT Entry point===\n");

  SystemInit();
  /* Configure the uart so that we can get debug output as soon as possible */

 // nuc_clockconfig();
 // nuc_lowsetup();
 // showprogress('A');

  /* Clear .bss.  We'll do this inline (vs. calling memset) just to be
   * certain that there are no issues with the state of global variables.
   */

  for (dest = &_sbss; dest < &_ebss; )
    {
      *dest++ = 0;
    }
  
  //showprogress('B');

  /* Move the initialized data section from his temporary holding spot in
   * FLASH into the correct place in SRAM.  The correct place in SRAM is
   * give by _sdata and _edata.  The temporary location is in FLASH at the
   * end of all of the other read-only data (.text, .rodata) at _eronly.
   */

//  for (src = &_eronly, dest = &_sdata; dest < &_edata; )
  //  {
    //  *dest++ = *src++;
   // }
 // showprogress('C');

  /* Perform early serial initialization */

#ifdef USE_EARLYSERIALINIT
//  up_earlyserialinit();
#endif
  //showprogress('D');

  /* For the case of the separate user-/kernel-space build, perform whatever
   * platform specific initialization of the user memory is required.
   * Normally this just means initializing the user space .data and .bss
   * segments.
   */

#ifdef CONFIG_BUILD_PROTECTED
  nuc_userspace();
  showprogress('E');
#endif

  /* Initialize onboard resources */

 // nuc_boardinitialize();
 // showprogress('F');

  /* Then start NuttX */

  //showprogress('\r');
  //showprogress('\n');
  os_start();

  /* Shoulnd't get here */

  for (; ; );
}
