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
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "dbg.h"
#include "server.h"




// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"




static osThreadId dgbTaskHandle;
static osThreadId serverTaskHandle;




int main(int argc, char* argv[])
{
	SERVER_Init();

	osThreadDef(DBG_TASK, DBG_Task, osPriorityNormal, 0, 128);
	dgbTaskHandle = osThreadCreate(osThread(DBG_TASK), NULL);

	osThreadDef(SERVER_TASK, SERVER_Task, osPriorityNormal, 0, 128);
	serverTaskHandle = osThreadCreate(osThread(SERVER_TASK), NULL);

	/* Start scheduler */
	osKernelStart();

	while(1);
}


void SysTick_Handler (void)
{
#if defined(USE_HAL_DRIVER)
  HAL_IncTick();
#endif

  osSystickHandler();
}




#pragma GCC diagnostic pop



