/*
** ###################################################################
**     Processor:           RDA5981
**     Compilers:           ARM Compiler
**                          Freescale C/C++ for Embedded ARM
**                          GNU C Compiler
**                          GNU C Compiler - CodeSourcery Sourcery G++
**                          IAR ANSI C/C++ Compiler for ARM
**
**     Reference manual:    RDA5981
**     Version:             rev. 1.0, 2017-01-09
**
**     Abstract:
**         Provides a system configuration function and a global variable that
**         contains the system frequency. It configures the device and initializes
**         the oscillator (PLL) that is part of the microcontroller device.
**
**     Copyright: 2016 RDA Microelectronics, Inc. All Rights Reserved.
**
**     http:                 www.rdamicro.com
**     mail:                 support@rdamicro.com
**
**     Revisions:
**     - rev. 1.0 (2017-01-09)
**         Initial version.
**
** ###################################################################
*/

/*!
 * @file RDA5981
 * @version 1.0
 * @date 2017-01-09
 * @brief
 *
 */

#ifndef SYSTEM_MK64F12_H_
#define SYSTEM_MK64F12_H_                        /**< Symbol preventing repeated inclusion */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief System clock frequency (core clock)
 *
 * The system clock frequency supplied to the SysTick timer and the processor
 * core clock. This variable can be used by the user application to setup the
 * SysTick timer or configure other parameters. It may also be used by debugger to
 * query the frequency of the debug timer or configure the trace clock speed
 * SystemCoreClock is initialized with a correct predefined value.
 */
extern uint32_t SystemCoreClock;

/**
 * @brief Setup the microcontroller system.
 *
 * Typically this function configures the oscillator (PLL) that is part of the
 * microcontroller device. For systems with variable clock speed it also updates
 * the variable SystemCoreClock. SystemInit is called from startup_device file.
 */
void SystemInit(void);

/**
 * @brief Updates the SystemCoreClock variable.
 *
 * It must be called whenever the core clock is changed during program
 * execution. SystemCoreClockUpdate() evaluates the clock register settings and calculates
 * the current core clock.
 */
void SystemCoreClockUpdate(void);

#ifdef __cplusplus
}
#endif

#endif  /* #if !defined(SYSTEM_MK64F12_H_) */
