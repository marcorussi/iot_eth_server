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




#ifndef __CALENDAR_H
#define __CALENDAR_H




/* --------------- Includes ----------------*/

#include "stddef.h"
#include "stdint.h"




/* Exported types ------------------------------------------------------------*/

typedef struct 
{
   uint8_t weekday;
   uint8_t day;
   uint8_t month;
   uint8_t year;
   uint8_t hours;
   uint8_t minutes;
   uint8_t seconds;
} CALENDAR_InfoStruct;




/* Exported constants --------------------------------------------------------*/
/* Defines related to Clock configuration */

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
extern void CALENDAR_Init           (CALENDAR_InfoStruct *);
extern void CALENDAR_setDateTime	   (CALENDAR_InfoStruct *);
extern void CALENDAR_getDate			(char *);
extern void CALENDAR_getTime			(char *);




#endif /* __CLOCK_H */



