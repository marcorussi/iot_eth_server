/*
 * This file is part of the µOS++ distribution.
 *   (https://github.com/micro-os-plus)
 * Copyright (c) 2014 Liviu Ionescu.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

// ----------------------------------------------------------------------------

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_cortex.h"

// ----------------------------------------------------------------------------

// The external clock frequency is specified as a preprocessor definition
// passed to the compiler via a command line option (see the 'C/C++ General' ->
// 'Paths and Symbols' -> the 'Symbols' tab, if you want to change it).
// The value selected during project creation was HSE_VALUE=8000000.
//
// The code to set the clock is at the end.
//
// Note1: The default clock settings assume that the HSE_VALUE is a multiple
// of 1MHz, and try to reach the maximum speed available for the
// board. It does NOT guarantee that the required USB clock of 48MHz is
// available. If you need this, please update the settings of PLL_M, PLL_N,
// PLL_P, PLL_Q to match your needs.
//
// Note2: The external memory controllers are not enabled. If needed, you
// have to define DATA_IN_ExtSRAM or DATA_IN_ExtSDRAM and to configure
// the memory banks in system/src/cmsis/system_stm32f4xx.c to match your needs.

// ----------------------------------------------------------------------------

extern unsigned int __vectors_start;

// Forward declarations.

void
__initialize_hardware_early(void);

void
__initialize_hardware(void);

void
SystemClock_Config(void);

// ----------------------------------------------------------------------------

// This is the early hardware initialisation routine, it can be
// redefined in the application for more complex cases that
// require early inits (before BSS init).
//
// Called early from _start(), right before data & bss init.
//
// After Reset the Cortex-M processor is in Thread mode,
// priority is Privileged, and the Stack is set to Main.

void
__attribute__((weak))
__initialize_hardware_early(void)
{
  // Call the CSMSIS system initialisation routine.
  SystemInit();

#if defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__)
  // Set VTOR to the actual address, provided by the linker script.
  // Override the manual, possibly wrong, SystemInit() setting.
  SCB->VTOR = (uint32_t)(&__vectors_start);
#endif

  // The current version of SystemInit() leaves the value of the clock
  // in a RAM variable (SystemCoreClock), which will be cleared shortly,
  // so it needs to be recomputed after the RAM initialisations
  // are completed.

#if defined(OS_INCLUDE_STARTUP_INIT_FP) || (defined (__VFP_FP__) && !defined (__SOFTFP__))

  // Normally FP init is done by SystemInit(). In case this is not done
  // there, it is possible to force its inclusion by defining
  // OS_INCLUDE_STARTUP_INIT_FP.

  // Enable the Cortex-M4 FPU only when -mfloat-abi=hard.
  // Code taken from Section 7.1, Cortex-M4 TRM (DDI0439C)

  // Set bits 20-23 to enable CP10 and CP11 coprocessor
  SCB->CPACR |= (0xF << 20);

#endif // (__VFP_FP__) && !(__SOFTFP__)

#if defined(OS_DEBUG_SEMIHOSTING_FAULTS)
  SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk;
#endif
}


// This is the application hardware initialisation routine,
// redefined to add more inits.
//
// Called early from _start(), right after data & bss init, before
// constructors.
//
// After Reset the Cortex-M processor is in Thread mode,
// priority is Privileged, and the Stack is set to Main.
//
// Warning: The HAL requires the system timer, running at 1000 Hz
// and calling HAL_IncTick().

void
__initialize_hardware(void)
{
  // Initialise the HAL Library; it must be the first function
  // to be executed before the call of any HAL function.
  HAL_Init();

  // Enable HSE Oscillator and activate PLL with HSE as source
  SystemClock_Config();

  // Call the CSMSIS system clock routine to store the clock frequency
  // in the SystemCoreClock global RAM location.
  SystemCoreClockUpdate();
}

// Disable when using RTOSes, since they have their own handler.
#if 0

// This is a sample SysTick handler, use it if you need HAL timings.
void __attribute__ ((section(".after_vectors")))
SysTick_Handler(void)
{
#if defined(USE_HAL_DRIVER)
	HAL_IncTick();
#endif
}

#endif

// ----------------------------------------------------------------------------

/**
 * @brief  System Clock Configuration
 * @param  None
 * @retval None
 */
void
__attribute__((weak))
SystemClock_Config(void)
{
  // Enable Power Control clock
  __PWR_CLK_ENABLE();

  // The voltage scaling allows optimizing the power consumption when the
  // device is clocked below the maximum system frequency, to update the
  // voltage scaling value regarding system frequency refer to product
  // datasheet.
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

#warning "Please check if the SystemClock_Config() settings match your board!"
  // Comment out the warning after checking and updating.

  RCC_OscInitTypeDef RCC_OscInitStruct;

#if defined(HSE_VALUE) && (HSE_VALUE != 0)
  // Enable HSE Oscillator and activate PLL with HSE as source.
  // This is tuned for STM32F4-DISCOVERY; update it for your board.
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  // This assumes the HSE_VALUE is a multiple of 1 MHz. If this is not
  // your case, you have to recompute these PLL constants.
  RCC_OscInitStruct.PLL.PLLM = (HSE_VALUE/1000000u);
#else
  // Use HSI and activate PLL with HSI as source.
  // This is tuned for NUCLEO-F411; update it for your board.
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  // 16 is the average calibration value, adjust for your own board.
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  // This assumes the HSI_VALUE is a multiple of 1 MHz. If this is not
  // your case, you have to recompute these PLL constants.
  RCC_OscInitStruct.PLL.PLLM = (HSI_VALUE/1000000u);
#endif

  RCC_OscInitStruct.PLL.PLLN = 336;
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F411xE)
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4; /* 84 MHz */
#elif defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx)
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2; /* 168 MHz */
#elif defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F417xx)
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2; /* 168 MHz */
#elif defined(STM32F446xx)
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2; /* 168 MHz */
#else
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4; /* 84 MHz, conservative */
#endif
  RCC_OscInitStruct.PLL.PLLQ = 7; /* To make USB work. */
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  // Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
  // clocks dividers
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK
      | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F411xE)
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
#else
  // This is expected to work for most large cores.
  // Check and update it for your own configuration.
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
#endif

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
}

// ----------------------------------------------------------------------------
