/*
 * The MIT License (MIT)
 *
 * Copyright (c) [2018] [Marco Russi]
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/

#ifndef __KSZ8851SNL_H
#define __KSZ8851SNL_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "conf_eth.h"

#ifdef __cplusplus
extern "C" {
#endif




#define INT_SPI_CALLBACK						HAL_SPI_TxRxCpltCallback
#define SPI_HANDLE_TYPE							SPI_HandleTypeDef
#define INT_EXT_GPIO_CALLBACK					EXTI1_IRQHandler




typedef enum
{
	INT_SPI_READY,
	INT_SPI_BUSY,
	INT_SPI_ERROR
} spi_int_codes;




#define SET_SPI_CS_PIN_NO_DELAY()			{																				\
															HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);		\
														}

#define SET_SPI_CS_PIN()						{																				\
															osDelay(2);																\
															HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);		\
														}

#define RESET_SPI_CS_PIN()						{																				\
															HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);	\
															osDelay(2);																\
														}

#define CLEAR_GPIO_INT_FLAG()					__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1)




extern uint8_t		ksz8851snl_get_spi_state	(SPI_HandleTypeDef *);
extern uint16_t	ksz8851_reg_read				(uint16_t);
extern void 		ksz8851_reg_write				(uint16_t, uint16_t);
extern void 		ksz8851_reg_setbits			(uint16_t, uint16_t);
extern void 		ksz8851_reg_clrbits			(uint16_t, uint16_t);
extern void 		ksz8851_fifo_read				(uint8_t *, uint16_t);
extern void 		ksz8851_fifo_write			(uint8_t *, uint16_t);
extern bool 		ksz8851snl_init				(void);




#ifdef __cplusplus
}
#endif

#endif
