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
#include "calendar.h"
#include "dbg.h"
#include "FreeRTOS_IP.h"
#include "FreeRTOS_Sockets.h"
#include "NetworkBufferManagement.h"
#include "NetworkInterface.h"
#include "conf_eth.h"
#include "server.h"




/* -------------- Local defines ------------------ */

#define CALENDAR_BUFF_SIZE					20

#define TCP_DATA_BUFFER_SIZE 				128




/* -------------- Local variables ------------------ */

/* The MAC address array is not declared const as the MAC address will
	normally be read from an EEPROM and not hard coded (in real deployed
	applications).*/
static uint8_t ucMACAddress[ 6 ] = { 	ETHERNET_CONF_ETHADDR0,
													ETHERNET_CONF_ETHADDR1,
													ETHERNET_CONF_ETHADDR2,
													ETHERNET_CONF_ETHADDR3,
													ETHERNET_CONF_ETHADDR4,
													ETHERNET_CONF_ETHADDR5 };

/* Define the network addressing.  These parameters will be used if either
	ipconfigUDE_DHCP is 0 or if ipconfigUSE_DHCP is 1 but DHCP auto configuration
	failed. */
static const uint8_t ucIPAddress[ 4 ] = { ETHERNET_CONF_IPADDR0,
														ETHERNET_CONF_IPADDR1,
														ETHERNET_CONF_IPADDR2,
														ETHERNET_CONF_IPADDR3 };

static const uint8_t ucNetMask[ 4 ] = { 	ETHERNET_CONF_NET_MASK0,
														ETHERNET_CONF_NET_MASK1,
														ETHERNET_CONF_NET_MASK2,
														ETHERNET_CONF_NET_MASK3 };

static const uint8_t ucGatewayAddress[ 4 ] = { 	ETHERNET_CONF_GATEWAY_ADDR0,
																ETHERNET_CONF_GATEWAY_ADDR1,
																ETHERNET_CONF_GATEWAY_ADDR2,
																ETHERNET_CONF_GATEWAY_ADDR3 };

/* The following is the address of an OpenDNS server. */
static const uint8_t ucDNSServerAddress[ 4 ] = { 	ETHERNET_CONF_DNS_SERVER_ADDR0,
																	ETHERNET_CONF_DNS_SERVER_ADDR1,
																	ETHERNET_CONF_DNS_SERVER_ADDR2,
																	ETHERNET_CONF_DNS_SERVER_ADDR3 };

const char hostname_string[] = "iot_server.local";

const char on_conn_string[] = "Connected to IoT Server! Press Enter...\r\n";

const char reply_string[] = "* %d | %s | %s - put your sensors data here\r\n";

static char DbgData[DBG_UART_BUFFER_MSG_LENGTH];

CALENDAR_InfoStruct calendarInfoStruct;




/* -------------- Local functions prototypes ------------------ */

static void vTCPRxDataTask( void const * );




/* -------------- Exported functions ------------------ */

void SERVER_Init( void )
{
	/* Initialise the RTOS's TCP/IP stack.  The tasks that use the network
		are created in the vApplicationIPNetworkEventHook() hook function
		below. The hook function is called when the network connects. */
	FreeRTOS_IPInit(	ucIPAddress,
							ucNetMask,
							ucGatewayAddress,
							ucDNSServerAddress,
							ucMACAddress );

	/* set calendar date and time */
	calendarInfoStruct.year = 0x18;
	calendarInfoStruct.month = 0x07;
	calendarInfoStruct.day = 0x01;
	calendarInfoStruct.weekday = 7;
	calendarInfoStruct.hours = 0x10;
	calendarInfoStruct.minutes = 0x00;
	calendarInfoStruct.seconds = 0x00;

	/* init calendar */
	CALENDAR_Init(&calendarInfoStruct);

   DBG_sendString("Server initialised!\r\n", (uint8_t)21);
}


void SERVER_Task( void const *argument )
{
	struct freertos_sockaddr xClient, xBindAddress;
	uint32_t ulIPAddress = 0;
	Socket_t xListeningSocket, xConnectedSocket;
	socklen_t xSize = sizeof( xClient );
	static const TickType_t xReceiveTimeOut = portMAX_DELAY;
	const BaseType_t xBacklog = 20;

	do
	{
		FreeRTOS_GetAddressConfiguration( &ulIPAddress, NULL, NULL, NULL );
	}
	while( 0 == ulIPAddress );

	sprintf(DbgData, "IP address obtained: %d:%d:%d:%d\r\n",	(uint8_t)(ulIPAddress & 0x000000FF),
																				(uint8_t)((ulIPAddress & 0x0000FF00) >> 8),
																				(uint8_t)((ulIPAddress & 0x00FF0000) >> 16),
																				(uint8_t)((ulIPAddress & 0xFF000000) >> 24));
	DBG_sendString(DbgData, (uint8_t)strlen(DbgData));

	/* Attempt to open the socket. */
	xListeningSocket = FreeRTOS_socket(	FREERTOS_AF_INET,
													FREERTOS_SOCK_STREAM,
													FREERTOS_IPPROTO_TCP );

	/* Check the socket was created. */
	configASSERT( xListeningSocket != FREERTOS_INVALID_SOCKET );

	/* If FREERTOS_SO_RCVBUF or FREERTOS_SO_SNDBUF are to be used with
	FreeRTOS_setsockopt() to change the buffer sizes from their default then do
	it here!.  (see the FreeRTOS_setsockopt() documentation. */

	/* If ipconfigUSE_TCP_WIN is set to 1 and FREERTOS_SO_WIN_PROPERTIES is to
	be used with FreeRTOS_setsockopt() to change the sliding window size from
	its default then do it here! (see the FreeRTOS_setsockopt()
	documentation. */

	/* Set a time out so accept() will just wait for a connection. */
	FreeRTOS_setsockopt(	xListeningSocket,
								0,
								FREERTOS_SO_RCVTIMEO,
								&xReceiveTimeOut,
								sizeof( xReceiveTimeOut ) );

	/* Set the listening port */
	xBindAddress.sin_port = ( uint16_t )ETHERNET_DST_PORT;
	xBindAddress.sin_port = FreeRTOS_htons( xBindAddress.sin_port );

	/* Bind the socket to the port that the client RTOS task will send to. */
	FreeRTOS_bind( xListeningSocket, &xBindAddress, sizeof( xBindAddress ) );

	/* Set the socket into a listening state so it can accept connections.
	The maximum number of simultaneous connections is limited to 20. */
	FreeRTOS_listen( xListeningSocket, xBacklog );

	for( ;; )
	{
		/* Wait for incoming connections. */
		xConnectedSocket = FreeRTOS_accept( xListeningSocket, &xClient, &xSize );
		configASSERT( xConnectedSocket != FREERTOS_INVALID_SOCKET );

		DBG_sendString("Connection established!\r\n", (uint8_t)25);
		FreeRTOS_send(xConnectedSocket, on_conn_string, strlen(on_conn_string), 0);

		/* Spawn a RTOS task to handle the connection. */
		osThreadDef(TCP_DATA_TASK, vTCPRxDataTask, osPriorityNormal, 0, 512);
		(void)osThreadCreate(osThread(TCP_DATA_TASK), (void *)xConnectedSocket);
	}

   /* Must not drop off the end of the RTOS task - delete the RTOS task. */
	vTaskDelete( NULL );
}


BaseType_t xApplicationDNSQueryHook( const char *pcName )
{
	BaseType_t match_ret = pdFALSE;

	if( 0 == strcmp(hostname_string, pcName) )
	{
		match_ret = pdTRUE;
	}

	return match_ret;
}


void vApplicationPingReplyHook( ePingReplyStatus_t eStatus, uint16_t usIdentifier )
{
	if(eSuccess == eStatus)
	{
		DBG_toggleDbgLED();
	}
}


const char *pcApplicationHostnameHook( void )
{
	return (const char *)hostname_string;
}




/* -------------- Local functions ------------------ */

void vTCPRxDataTask( void const *argument )
{
	BaseType_t lBytesReceived;
	uint8_t dateBuffer[CALENDAR_BUFF_SIZE];
	uint8_t timeBuffer[CALENDAR_BUFF_SIZE];
	static char RxData[TCP_DATA_BUFFER_SIZE];
	static char TxData[TCP_DATA_BUFFER_SIZE];
   static uint8_t counter = 0;

	for( ;; )
	{
		/* Receive another block of data into the cRxedData buffer. */
		lBytesReceived = FreeRTOS_recv( (Socket_t *)argument, &RxData, TCP_DATA_BUFFER_SIZE, 0 );

		if( lBytesReceived > 0 )
		{
			CALENDAR_getDate((char *)dateBuffer);
			CALENDAR_getTime((char *)timeBuffer);
			sprintf(TxData, reply_string, ++counter, dateBuffer, timeBuffer);

			DBG_sendString(TxData, strlen(TxData));
			FreeRTOS_send((Socket_t *)argument, TxData, strlen(TxData), 0);
		}
		else if( lBytesReceived == 0 )
		{
			/* No data was received, but FreeRTOS_recv() did not return an error. Timeout? */
			DBG_sendString("\r\n - No received data!\r\n", 15);
		}
		else
		{
         sprintf(TxData, "\r\n - Done! %d\r\n", (int)lBytesReceived);
			DBG_sendString(TxData, strlen(TxData));

         /* Error (maybe the connected socket already shut down the socket?).
         Attempt graceful shutdown. */
         FreeRTOS_shutdown( (Socket_t *)argument, FREERTOS_SHUT_RDWR );
         break;
		}
	}

	/* The RTOS task will get here if an error is received on a read.  Ensure the
	socket has shut down (indicated by FreeRTOS_recv() returning a FREERTOS_EINVAL
	error before closing the socket). */

	while( FreeRTOS_recv( (Socket_t *)argument, &RxData, TCP_DATA_BUFFER_SIZE, 0 ) >= 0 )
	{
		/* Wait for shutdown to complete.  If a receive block time is used then
		this delay will not be necessary as FreeRTOS_recv() will place the RTOS task
		into the Blocked state anyway. */
		osDelay( 250 );

		/* Note - real applications should implement a timeout here, not just
		loop forever. */
	}

	/* Shutdown is complete and the socket can be safely closed. */
	FreeRTOS_closesocket( (Socket_t *)argument );

	/* Must not drop off the end of the RTOS task - delete the RTOS task. */
	vTaskDelete( NULL );
}




#if 0

void vSendingPingrequest( void const *argument )
{
	uint32_t lastWakeTime = osKernelSysTick();
	uint32_t ulIPAddress = 0;

	//ulIPAddress = FreeRTOS_inet_addr_quick( 8, 8, 8, 8 );
	ulIPAddress = FreeRTOS_gethostbyname("www.marcorussi.net");

   for( ;; )
   {
   	if( ulIPAddress != 0 )
   	{
   		(void)FreeRTOS_SendPingRequest(ulIPAddress, 8, 100 / portTICK_PERIOD_MS);
   	}
   	else
   	{
   		HAL_GPIO_TogglePin(LED_PORT, LED_5);
   	}

		osDelayUntil( &lastWakeTime, 3000 );
   }
}


void vUDPSendingUsingZeroCopyInterface( void const *argument )
{
	uint32_t lastWakeTime = osKernelSysTick();
	Socket_t xSocket;
	uint8_t *pucBuffer;
	struct freertos_sockaddr xDestinationAddress;
	BaseType_t lReturned;
	uint8_t count = 0;
	const uint8_t pucStringToSend[] = "Zero copy send message number ";
	/* 15 is added to ensure the number, \r\n and terminating zero fit. */
	const size_t xStringLength = strlen( ( char * ) pucStringToSend ) + 15;

   /* Send strings to port 10000 on IP address 192.168.0.50. */
   xDestinationAddress.sin_addr = FreeRTOS_inet_addr_quick( ETHERNET_DST_IPADDR0,
   																			ETHERNET_DST_IPADDR1,
																				ETHERNET_DST_IPADDR2,
																				ETHERNET_DST_IPADDR3 );
   xDestinationAddress.sin_port = FreeRTOS_htons( ETHERNET_DST_PORT );

   /* Create the socket. */
   xSocket = FreeRTOS_socket( FREERTOS_AF_INET,
                              FREERTOS_SOCK_DGRAM,/*FREERTOS_SOCK_DGRAM for UDP.*/
                              FREERTOS_IPPROTO_UDP );

   /* Check the socket was created. */
   configASSERT( xSocket != FREERTOS_INVALID_SOCKET );

   /* NOTE: FreeRTOS_bind() is not called.  This will only work if
   ipconfigALLOW_SOCKET_SEND_WITHOUT_BIND is set to 1 in FreeRTOSIPConfig.h. */

   for( ;; )
   {
       /* This RTOS task is going to send using the zero copy interface.  The
       data being sent is therefore written directly into a buffer that is
       passed into, rather than copied into, the FreeRTOS_sendto()
       function.

       First obtain a buffer of adequate length from the TCP/IP stack into which
       the string will be written. */
       pucBuffer = FreeRTOS_GetUDPPayloadBuffer( xStringLength, portMAX_DELAY );

       /* Check a buffer was obtained. */
       configASSERT( pucBuffer );

       /* Create the string that is sent. */
       memset( pucBuffer, 0x00, xStringLength );
       sprintf( (char *)pucBuffer, "%s%d\r\n", (const char *)pucStringToSend, count );

       /* Pass the buffer into the send function.  ulFlags has the
       FREERTOS_ZERO_COPY bit set so the TCP/IP stack will take control of the
       buffer rather than copy data out of the buffer. */
       lReturned = FreeRTOS_sendto( xSocket,
                                   ( void * ) pucBuffer,
                                   strlen( ( const char * ) pucBuffer ) + 1,
                                   FREERTOS_ZERO_COPY,
                                   &xDestinationAddress,
                                   sizeof( xDestinationAddress ) );

       if( lReturned == 0 )
       {
           /* The send operation failed, so this RTOS task is still responsible
           for the buffer obtained from the TCP/IP stack.  To ensure the buffer
           is not lost it must either be used again, or, as in this case,
           returned to the TCP/IP stack using FreeRTOS_ReleaseUDPPayloadBuffer().
           pucBuffer can be safely re-used after this call. */
           FreeRTOS_ReleaseUDPPayloadBuffer( ( void * ) pucBuffer );
       }
       else
       {
           /* The send was successful so the TCP/IP stack is now managing the
           buffer pointed to by pucBuffer, and the TCP/IP stack will
           return the buffer once it has been sent.  pucBuffer can
           be safely re-used. */
       }

       count++;

       osDelayUntil( &lastWakeTime, 10000 );
   }
}

#endif




/* End of file */



