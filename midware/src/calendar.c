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




#include "stddef.h"
#include "stdint.h"
#include "stm32f4xx_hal.h"

#include "calendar.h"




#define RTC_ASYNCH_PREDIV  0x7F   /* LSE as RTC clock */
#define RTC_SYNCH_PREDIV   0x00FF /* LSE as RTC clock */




static RTC_HandleTypeDef RtcHandle;




static void Error_Handler        (void);
static void configure_calendar   (RTC_DateTypeDef *, RTC_TimeTypeDef *);




void CALENDAR_Init( CALENDAR_InfoStruct *pInfoStruct )
{
   RTC_DateTypeDef sdatestructure;
   RTC_TimeTypeDef stimestructure;

   /* Configure the RTC peripheral */
   /* Configure RTC prescaler and RTC data registers */
   /* RTC configured as follows:
   - Hour Format    = Format 24
   - Asynch Prediv  = Value according to source clock
   - Synch Prediv   = Value according to source clock
   - OutPut         = Output Disable
   - OutPutPolarity = High Polarity
   - OutPutType     = Open Drain */ 
   RtcHandle.Instance = RTC; 
   RtcHandle.Init.HourFormat = RTC_HOURFORMAT_24;
   RtcHandle.Init.AsynchPrediv = RTC_ASYNCH_PREDIV;
   RtcHandle.Init.SynchPrediv = RTC_SYNCH_PREDIV;
   RtcHandle.Init.OutPut = RTC_OUTPUT_DISABLE;
   RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
   RtcHandle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
   __HAL_RTC_RESET_HANDLE_STATE(&RtcHandle);
   if (HAL_RTC_Init(&RtcHandle) != HAL_OK)
   {
      /* Initialization Error */
      Error_Handler();
   }

   HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);

   /* if info struct is different than NULL */
   if( pInfoStruct != NULL )
   {
      /* set date */
      sdatestructure.Year = pInfoStruct->year;
      sdatestructure.Month = pInfoStruct->month;
      sdatestructure.Date = pInfoStruct->day;
      sdatestructure.WeekDay = pInfoStruct->weekday;
      /* set time */
      stimestructure.Hours = pInfoStruct->hours;
      stimestructure.Minutes = pInfoStruct->minutes;
      stimestructure.Seconds = pInfoStruct->seconds;
      stimestructure.SubSeconds = 0;
   	stimestructure.SecondFraction = 0;
      stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
      stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
      stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

      /* Configure RTC Calendar */
      configure_calendar(&sdatestructure, &stimestructure);
   }
   else
   {
      /* Check if Data stored in BackUp register1: No Need to reconfigure RTC */
      /* Read the Back Up Register 1 Data */
      if (HAL_RTCEx_BKUPRead(&RtcHandle, RTC_BKP_DR1) != 0x32F2)
      {
         /* set default date */
         sdatestructure.Year = 0x18;
         sdatestructure.Month = RTC_MONTH_MAY;
         sdatestructure.Date = 0x12;
         sdatestructure.WeekDay = RTC_WEEKDAY_SATURDAY;
         /* set default time */
         stimestructure.Hours = 0x02;
         stimestructure.Minutes = 0x00;
         stimestructure.Seconds = 0x00;  
         stimestructure.SubSeconds = 0;
         stimestructure.SecondFraction = 0;
         stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
         stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
         stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

         /* Configure default RTC Calendar */
         configure_calendar(&sdatestructure, &stimestructure);
      }
      else
      {
         /* Check if the Power On Reset flag is set */
         if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) != RESET)
         {
            HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
         }
         /* Check if Pin Reset flag is set */
         if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) != RESET)
         {
            HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
         }
         /* Clear source Reset Flag */
         __HAL_RCC_CLEAR_RESET_FLAGS();
      }
   }
}




void CALENDAR_setDateTime( CALENDAR_InfoStruct *pInfoStruct )
{
   RTC_DateTypeDef sdatestructure;
   RTC_TimeTypeDef stimestructure;

   /* set date */
   sdatestructure.Year = pInfoStruct->year;
   sdatestructure.Month = pInfoStruct->month;
   sdatestructure.Date = pInfoStruct->day;
   sdatestructure.WeekDay = pInfoStruct->weekday;
   /* set time */
   stimestructure.Hours = pInfoStruct->hours;
   stimestructure.Minutes = pInfoStruct->minutes;
   stimestructure.Seconds = pInfoStruct->seconds;

   stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
   stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
   stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

   /* Configure RTC Calendar */
   configure_calendar(&sdatestructure, &stimestructure);
}




void CALENDAR_getDate( char *pDateString )
{
   RTC_DateTypeDef sdatestructureget;

   /* Get the RTC current Date */
   HAL_RTC_GetDate(&RtcHandle, &sdatestructureget, RTC_FORMAT_BIN);

   /* format date string : dd-mm-yy */
   sprintf((char *)pDateString, "%2d-%2d-%2d", sdatestructureget.Date, sdatestructureget.Month, 2000 + sdatestructureget.Year);
}




void CALENDAR_getTime( char *pTimeString )
{
   RTC_TimeTypeDef stimestructureget;

   /* Get the RTC current Time */
   HAL_RTC_GetTime(&RtcHandle, &stimestructureget, RTC_FORMAT_BIN);

   /* format time string : hh:mm:ss */
   sprintf((char *)pTimeString, "%2d:%2d:%2d", stimestructureget.Hours, stimestructureget.Minutes, stimestructureget.Seconds);
}




void HAL_RTC_MspInit(RTC_HandleTypeDef* hrtc)
{
	RCC_OscInitTypeDef        RCC_OscInitStruct;
	RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;

	/* Enables the PWR Clock and Enables access to the backup domain */
	/* To change the source clock of the RTC feature (LSE, LSI), You have to:
	   - Enable the power clock using __HAL_RCC_PWR_CLK_ENABLE()
	   - Enable write access using HAL_PWR_EnableBkUpAccess() function before to
		 configure the RTC clock source (to be done once after reset).
	   - Reset the Back up Domain using __HAL_RCC_BACKUPRESET_FORCE() and
		 __HAL_RCC_BACKUPRESET_RELEASE().
	   - Configure the needed RTc clock source */
	__HAL_RCC_PWR_CLK_ENABLE();
	HAL_PWR_EnableBkUpAccess();

	/* Configure LSE as RTC clock source */
	RCC_OscInitStruct.OscillatorType =  RCC_OSCILLATORTYPE_LSI;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
	PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/* Enable RTC peripheral Clocks */
	/* Enable RTC Clock */
	__HAL_RCC_RTC_ENABLE();
}




void HAL_RTC_MspDeInit(RTC_HandleTypeDef *hrtc)
{
  /*##-1- Reset peripherals ##################################################*/
  __HAL_RCC_RTC_DISABLE();

  /*##-2- Disables the PWR Clock and Disables access to the backup domain ###################################*/
  HAL_PWR_DisableBkUpAccess();
  __HAL_RCC_PWR_CLK_DISABLE();

}




static void configure_calendar( RTC_DateTypeDef *pDateStruct, RTC_TimeTypeDef *pTimeStruct)
{
   /* Configure the Date */
   if(HAL_RTC_SetDate(&RtcHandle, pDateStruct, RTC_FORMAT_BCD) != HAL_OK)
   {
      /* Initialization Error */
      Error_Handler();
   }

   /* Configure the Time */
   if (HAL_RTC_SetTime(&RtcHandle, pTimeStruct, RTC_FORMAT_BCD) != HAL_OK)
   {
      /* Initialization Error */
      Error_Handler();
   }

   /* Writes a data in a RTC Backup data Register1 */
   HAL_RTCEx_BKUPWrite(&RtcHandle, RTC_BKP_DR1, 0x32F2);
}




static void Error_Handler(void)
{
  /* Turn LED5 on */
  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);

  while(1);
}





