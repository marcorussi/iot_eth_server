/**
  ******************************************************************************
  * @file    stm32f4xx_hal_msp_template.c
  * @author  MCD Application Team
  * @version V1.5.0
  * @date    06-May-2016
  * @brief   This file contains the HAL System and Peripheral (PPP) MSP initialization
  *          and de-initialization functions.
  *          It should be copied to the application folder and renamed into 'stm32f4xx_hal_msp.c'.           
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"


// [ILG]
#if defined ( __GNUC__ )
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-prototypes"
#endif

/** @addtogroup STM32F4xx_HAL_Driver
  * @{
  */

/** @defgroup HAL_MSP HAL MSP
  * @brief HAL MSP module.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup HAL_MSP_Private_Functions HAL MSP Private Functions
  * @{
  */

/**
  * @brief  Initializes the Global MSP.
  * @note   This function is called from HAL_Init() function to perform system
  *         level initialization (GPIOs, clock, DMA, interrupt).
  * @retval None
  */
void HAL_MspInit(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;

	/* Enable GPIOB clock */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	/* Enable GPIOD clock */
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/* SPI CS GPIO pin configuration  */
	GPIO_InitStruct.Pin       = GPIO_PIN_12;
	GPIO_InitStruct.Pull      = GPIO_NOPULL;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_MEDIUM;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

	/* Configure pin as input with External interrupt */
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/* Enable and set Button EXTI Interrupt to the lowest priority */
	HAL_NVIC_SetPriority((IRQn_Type)(EXTI1_IRQn), 0x0F, 0x06);
	HAL_NVIC_EnableIRQ((IRQn_Type)(EXTI1_IRQn));

	/* RSTN GPIO pin configuration  */
	GPIO_InitStruct.Pin       = GPIO_PIN_2;
	GPIO_InitStruct.Pull      = GPIO_NOPULL;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_MEDIUM;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
}

/**
  * @brief  DeInitializes the Global MSP.
  * @note   This functiona is called from HAL_DeInit() function to perform system
  *         level de-initialization (GPIOs, clock, DMA, interrupt).
  * @retval None
  */
void HAL_MspDeInit(void)
{
	HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12);

	HAL_GPIO_DeInit(GPIOD, GPIO_PIN_1);
	HAL_NVIC_DisableIRQ(EXTI1_IRQn);
}

/**
  * @brief  Initializes the PPP MSP.
  * @note   This functiona is called from HAL_PPP_Init() function to perform 
  *         peripheral(PPP) system level initialization (GPIOs, clock, DMA, interrupt)
  * @retval None
  */
void HAL_PPP_MspInit(void)
{

}

/**
  * @brief  DeInitializes the PPP MSP.
  * @note   This functiona is called from HAL_PPP_DeInit() function to perform 
  *         peripheral(PPP) system level de-initialization (GPIOs, clock, DMA, interrupt)
  * @retval None
  */
void HAL_PPP_MspDeInit(void)
{

}




void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
  static DMA_HandleTypeDef hdma_tx;
  static DMA_HandleTypeDef hdma_rx;

  GPIO_InitTypeDef  GPIO_InitStruct;

	/* Enable GPIOB clock */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	/* Enable SPI2 clock */
	__HAL_RCC_SPI2_CLK_ENABLE();
	/* Enable DMA1 clock */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* SPI SCK GPIO pin configuration  */
	GPIO_InitStruct.Pin       = GPIO_PIN_13;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull      = GPIO_PULLUP;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* SPI MISO GPIO pin configuration  */
	GPIO_InitStruct.Pin = GPIO_PIN_14;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* SPI MOSI GPIO pin configuration  */
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


  /* Configure the DMA handler for Transmission process */
  hdma_tx.Instance                 = DMA1_Stream4;
  hdma_tx.Init.Channel             = DMA_CHANNEL_0;
  hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
  hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  hdma_tx.Init.Mode                = DMA_NORMAL;
  hdma_tx.Init.Priority            = DMA_PRIORITY_LOW;
  hdma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
  hdma_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  hdma_tx.Init.MemBurst            = DMA_MBURST_INC4;
  hdma_tx.Init.PeriphBurst         = DMA_PBURST_INC4;

  HAL_DMA_Init(&hdma_tx);

  /* Associate the initialized DMA handle to the the SPI handle */
  __HAL_LINKDMA(hspi, hdmatx, hdma_tx);

  /* Configure the DMA handler for Transmission process */
  hdma_rx.Instance                 = DMA1_Stream3;
  hdma_rx.Init.Channel             = DMA_CHANNEL_0;
  hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  hdma_rx.Init.Mode                = DMA_NORMAL;
  hdma_rx.Init.Priority            = DMA_PRIORITY_HIGH;
  hdma_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
  hdma_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  hdma_rx.Init.MemBurst            = DMA_MBURST_INC4;
  hdma_rx.Init.PeriphBurst         = DMA_PBURST_INC4;

  HAL_DMA_Init(&hdma_rx);

  /* Associate the initialized DMA handle to the the SPI handle */
  __HAL_LINKDMA(hspi, hdmarx, hdma_rx);

  /* NVIC configuration for DMA transfer complete interrupt */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0x0F, 0x08);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

  /* NVIC configuration for DMA transfer complete interrupt */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0x0F, 0x0A);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

  HAL_NVIC_SetPriority(SPI2_IRQn, 0x0F, 0x0C);
  HAL_NVIC_EnableIRQ(SPI2_IRQn);
}




void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi)
{
	UNUSED(hspi);

  static DMA_HandleTypeDef hdma_tx;
  static DMA_HandleTypeDef hdma_rx;

  __HAL_RCC_SPI2_FORCE_RESET();
  __HAL_RCC_SPI2_RELEASE_RESET();

  /* Disable SPI pins */
  HAL_GPIO_DeInit(GPIOB, GPIO_PIN_13);
  HAL_GPIO_DeInit(GPIOB, GPIO_PIN_14);
  HAL_GPIO_DeInit(GPIOB, GPIO_PIN_15);

  /* De-Initialize the DMA Stream associate to transmission process */
  HAL_DMA_DeInit(&hdma_tx);
  /* De-Initialize the DMA Stream associate to reception process */
  HAL_DMA_DeInit(&hdma_rx);

  HAL_NVIC_DisableIRQ(DMA1_Stream4_IRQn);
  HAL_NVIC_DisableIRQ(DMA1_Stream3_IRQn);
  HAL_NVIC_DisableIRQ(SPI2_IRQn);
}




/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

// [ILG]
#if defined ( __GNUC__ )
#pragma GCC diagnostic pop
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
