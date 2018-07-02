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


#include <stdio.h>
#include <stdlib.h>
#include "string.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "dbg.h"




#define LED_3			            	GPIO_PIN_13
#define LED_4			            	GPIO_PIN_12
#define LED_5			            	GPIO_PIN_14
#define LED_6			            	GPIO_PIN_15
#define LED_PORT		            	GPIOD

#define UART_BUFFER_DEPTH				5
#define UART_BUFFER_SIZE				DBG_UART_BUFFER_MSG_LENGTH
#define UART_HAL_TIMEOUT_MS			200

#define UART_TASK_PERIOD_MS			(UART_HAL_TIMEOUT_MS * UART_BUFFER_DEPTH)




static UART_HandleTypeDef UartHandle;

static char uartBuffer[UART_BUFFER_DEPTH][UART_BUFFER_SIZE];

static uint8_t uartBufferHeadIndex = 0;

static uint8_t uartBufferTailIndex = 0;




static void initLEDs			( void );
static void initUART			( UART_HandleTypeDef * );
static void errorHandler	(void);




/* ----------------- Exported functions ------------------- */

void DBG_Task( void const *argument )
{
	uint32_t lastWakeTime = osKernelSysTick();

	initLEDs();
	initUART(&UartHandle);

	for( ;; )
	{
		osDelayUntil( &lastWakeTime, UART_TASK_PERIOD_MS );

		HAL_GPIO_TogglePin(LED_PORT, LED_4);

		/* send messages in the queue */
		while( uartBufferTailIndex < uartBufferHeadIndex )
		{
			if( HAL_UART_Transmit(	&UartHandle,
											(uint8_t*)uartBuffer[uartBufferTailIndex],
											sizeof(uartBuffer[uartBufferTailIndex]),
											UART_HAL_TIMEOUT_MS) != HAL_OK )
			{
				errorHandler();
			}

			/* clear buffer location */
			memset(uartBuffer[uartBufferTailIndex], '\0', UART_BUFFER_SIZE);

			/* next location to send */
			uartBufferTailIndex++;
		}

		/* reset buffer */
		uartBufferHeadIndex = 0;
		uartBufferTailIndex = 0;
	}
}


void DBG_sendString( char * pString, uint8_t length )
{
	/* limit length */
	if( length > UART_BUFFER_SIZE )
	{
		length = UART_BUFFER_SIZE;
	}

	/* copy message in the free location */
	memcpy(uartBuffer[uartBufferHeadIndex], pString, length);

	/* next index; do not wrap around */
	if( uartBufferHeadIndex < UART_BUFFER_DEPTH )
	{
		uartBufferHeadIndex++;
	}
	else
	{
		/* next message will overwrite last location */
	}
}


void DBG_toggleDbgLED( void )
{
	HAL_GPIO_TogglePin(LED_PORT, LED_3);
}




/* ----------------- Local functions ------------------- */

static void initLEDs( void )
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* Enable the GPIO Clock */
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* Configure the GPIO_LED pin */
  GPIO_InitStruct.Pin = (LED_3 | LED_4 | LED_5 | LED_6);
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;

  HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

  HAL_GPIO_WritePin(LED_PORT, LED_3, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_PORT, LED_4, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_PORT, LED_5, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_PORT, LED_6, GPIO_PIN_RESET);
}


static void initUART( UART_HandleTypeDef *pUartHandle )
{
	GPIO_InitTypeDef GPIO_InitStruct;

	/* Enable peripherals and GPIO Clocks */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_USART2_CLK_ENABLE();

	/* Configure peripheral GPIO */
	/* UART TX GPIO pin configuration  */
	GPIO_InitStruct.Pin       = GPIO_PIN_2;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull      = GPIO_NOPULL;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	/* UART RX GPIO pin configuration  */
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* Init UART handle */
	pUartHandle->Instance          = USART2;
	pUartHandle->Init.BaudRate     = 115200;
	pUartHandle->Init.WordLength   = UART_WORDLENGTH_8B;
	pUartHandle->Init.StopBits     = UART_STOPBITS_1;
	pUartHandle->Init.Parity       = UART_PARITY_NONE;
	pUartHandle->Init.HwFlowCtl    = UART_HWCONTROL_NONE;
	pUartHandle->Init.Mode         = UART_MODE_TX_RX;
	pUartHandle->Init.OverSampling = UART_OVERSAMPLING_16;
	if(HAL_UART_Init(pUartHandle) != HAL_OK)
	{
		errorHandler();
	}
}


static void errorHandler(void)
{
  /* Turn LED5 on */
  HAL_GPIO_WritePin(LED_PORT, LED_5, GPIO_PIN_SET);

  while(1);
}




/* End of file */




