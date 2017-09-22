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
 * arch/arm/src/armv7-r/arm_head.S
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#include "up_internal.h"
#include "up_arch.h"

#include <arch/board/board.h>

/**********************************************************************************
 * Configuration
 **********************************************************************************/

/* Hard-coded options */

#undef CPU_ALIGNMENT_TRAP
#undef CPU_CACHE_ROUND_ROBIN
#define CPU_DCACHE_DISABLE
#undef CPU_ICACHE_DISABLE
#define CPU_SCTLR_CCP15BEN 1
#define CPU_BACKGROUND_REGION 1
#undef CPU_DIV0_FAULT
#undef CPU_FAST_INTERRUPT
#undef CPU_IMPL_VECTORS
#undef CPU_NONMASKABLE_FIQ

/* There are three operational memory configurations:
 *
 * 1. We execute in place in FLASH (CONFIG_BOOT_RUNFROMFLASH=y).  In this case
 *    the boot logic must:
 *
 *    - Configure SDRAM (if present),
 *    - Initialize the .data section in RAM, and
 *    - Clear .bss section
 *
 * 2. We boot in FLASH but copy ourselves to SDRAM from better performance.
 *    (CONFIG_BOOT_RUNFROMFLASH=n && CONFIG_BOOT_COPYTORAM=y).  In this case
 *    the boot logic must:
 *
 *    - Configure SDRAM (if present),
 *    - Copy ourself to RAM, and
 *    - Clear .bss section (data should be fully initialized)
 *
 *   In this case, we assume that the logic within this file executes from FLASH.
 *
 * 3. There is bootloader that copies us to SDRAM (CONFIG_BOOT_RUNFROMFLASH=n &&
 *    CONFIG_BOOT_COPYTORAM=n). In this case SDRAM was initialized by the boot
 *    loader, and this boot logic must:
 *
 *    - Clear .bss section (data should be fully initialized)
 */

/* Beginning (BOTTOM/BASE) and End+1 (TOP) of the IDLE stack.
 *
 * The IDLE stack is the stack that is used during intialization and,
 * eventually, becomes the stack of the IDLE task when initialization
 * is complete.
 *
 * REVISIT:  There are issues here in some configurations.  The stack
 * pointer is initialized very early in the boot sequence.  But in some
 * architectures the memory supporting the stack may not yet be
 * initialized (SDRAM, for example, would not be ready yet).  In that
 * case, ideally the IDLE stack should be in some other memory that does
 * not require initialization (such as internal SRAM)
 */

#define IDLE_STACK      (_ebss+CONFIG_IDLETHREAD_STACKSIZE-4)
#define HEAP_BASE       (_ebss+CONFIG_IDLETHREAD_STACKSIZE)


/****************************************************************************
 * Global Symbols
 ****************************************************************************/

/* Imported symbols */


	.global	_sbss				/* Start of .bss in RAM */
	.global	_ebss				/* End+1 of .bss in RAM */
#ifdef CONFIG_BOOT_RUNFROMFLASH
	.global	_eronly				/* Where .data defaults are stored in FLASH */
	.global	_sdata				/* Where .data needs to reside in SDRAM */
	.global	_edata
#endif

    /* Exported symbols */

	.global	__start				/* Power-up/Reset entry point */
	.global	g_idle_topstack		/* Top of the initial/IDLE stack */

	.cpu	cortex-m4
	.syntax	unified
    .thumb
/****************************************************************************
 * OS Entry Point
 ****************************************************************************/
    .arch armv7-m

    .section    .vectors, "ax"
    .code       16  
    .align      2   
    .globl      _vectors
    .type       _vectors, function

_vectors:
    .long    IDLE_STACK            /* Top of Stack */
    .long    Reset_Handler         /* Reset Handler */
    .long    NMI_Handler           /* NMI Handler */
    .long    HardFault_Handler     /* Hard Fault Handler */
    .long    MemManage_Handler     /* MPU Fault Handler */
    .long    BusFault_Handler      /* Bus Fault Handler */
    .long    UsageFault_Handler    /* Usage Fault Handler */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    SVC_Handler           /* SVCall Handler */
    .long    DebugMon_Handler      /* Debug Monitor Handler */
    .long    0                     /* Reserved */
    .long    PendSV_Handler        /* PendSV Handler */
    .long    SysTick_Handler       /* SysTick Handler */

    /* External interrupts */
    .long   SPIFLASH_IRQHandler         /* 16: SPI Flash                    */
    .long   PTA_IRQHandler              /* 17: PTA                          */
    .long   SDIO_IRQHandler             /* 18: SDIO                         */
    .long   USBDMA_IRQHandler           /* 19: USB DMA                      */
    .long   USB_IRQHandler              /* 20: USB                          */
    .long   GPIO_IRQHandler             /* 21: GPIO                         */
    .long   TIMER0_IRQHandler           /* 22: Timer0                       */
    .long   UART0_IRQHandler            /* 23: UART0                        */
    .long   MACHW_IRQHandler            /* 24: MAC Hardware                 */
    .long   UART1_IRQHandler            /* 25: UART1                        */
    .long   AHBDMA_IRQHandler           /* 26: AHB DMA                      */
    .long   PSRAM_IRQHandler            /* 27: PSRAM                        */
    .long   SDMMC_IRQHandler            /* 28: SDMMC                        */
    .long   EXIF_IRQHandler             /* 29: EXIF                         */
    .long   I2C_IRQHandler              /* 30: I2C                          */


    .size    _vectors, . - _vectors

    .text
    .thumb
    .thumb_func
    .align 2
    .globl    Reset_Handler
    .type    Reset_Handler, %function


	/* .text Data */


loop:
    ldr    r0, =loop
    bx    r0
    
    Reset_Handler:
/*     Loop to copy data from read only memory to RAM. The ranges
 *      of copy from/to are specified by following symbols evaluated in 
 *      linker script.
 *      _etext: End of code section, i.e., begin of data sections to copy from.
 *      __data_start__/__data_end__: RAM address range that data should be
 *      copied to. Both must be aligned to 4 bytes boundary.  */

  //  ldr    r0, =IDLE_STACK
   // msr    msp, r0

       //ldr    r0, =rda_ccfg_boot
   
    
   

       
       // blx    r0
    

       
       // cmp    r0, #0x01
    // bne    Soft_Reset
    


    ldr    r1, =_eronly
    ldr    r2, = _sdata
    ldr    r3, =_edata


.Lflash_to_ram_loop:
    cmp     r2, r3
    ittt    lt
    ldrlt   r0, [r1], #4
    strlt   r0, [r2], #4
    blt    .Lflash_to_ram_loop



    ldr    r0, =SystemInit
    blx    r0
 
    
    ldr    r0, =__start
    bx     r0




Soft_Reset:
    mov    r1, #0x04
    ldr    r0, [r1]
    bx     r0

    .pool
    .size Reset_Handler, . - Reset_Handler

    .text
/*    Macro to define default handlers. Default handler
 *    will be weak symbol and just dead loops. They can be
 *    overwritten by other handlers */
    .macro    def_default_handler    handler_name
    .align 1
    .thumb_func
    .weak    \handler_name
    .type    \handler_name, %function
\handler_name :
    b    .
    .size    \handler_name, . - \handler_name
    .endm

    def_default_handler    NMI_Handler
    def_default_handler    HardFault_Handler
    def_default_handler    MemManage_Handler
    def_default_handler    BusFault_Handler
    def_default_handler    UsageFault_Handler
    def_default_handler    SVC_Handler
    def_default_handler    DebugMon_Handler
    def_default_handler    PendSV_Handler
    def_default_handler    SysTick_Handler
    def_default_handler    Default_Handler

    .macro    def_irq_default_handler    handler_name
    .weak     \handler_name
    .set      \handler_name, Default_Handler
    .endm
 
    def_irq_default_handler     SPIFLASH_IRQHandler
    def_irq_default_handler     PTA_IRQHandler
    def_irq_default_handler     SDIO_IRQHandler
    def_irq_default_handler     USBDMA_IRQHandler
    def_irq_default_handler     USB_IRQHandler
    def_irq_default_handler     GPIO_IRQHandler
    def_irq_default_handler     TIMER0_IRQHandler
    def_irq_default_handler     UART0_IRQHandler
    def_irq_default_handler     MACHW_IRQHandler
    def_irq_default_handler     UART1_IRQHandler
    def_irq_default_handler     AHBDMA_IRQHandler
    def_irq_default_handler     PSRAM_IRQHandler
    def_irq_default_handler     SDMMC_IRQHandler
    def_irq_default_handler     EXIF_IRQHandler
    def_irq_default_handler     I2C_IRQHandler
   



    .section    .rodata, "a"

/* Variables: _sbss is the start of the BSS region (see ld.script) _ebss is the end
 * of the BSS regsion (see ld.script). The idle task stack starts at the end of BSS
 * and is of size CONFIG_IDLETHREAD_STACKSIZE.  The IDLE thread is the thread that
 * the system boots on and, eventually, becomes the idle, do nothing task that runs
 * only when there is nothing else to run.  The heap continues from there until the
 * end of memory.  See g_idle_topstack below.
 */
    
     .globl  g_idle_topstack
    .type   g_idle_topstack, object
g_idle_topstack:
    .word   HEAP_BASE
    .size   g_idle_topstack, .-g_idle_topstack 
    .end
 


