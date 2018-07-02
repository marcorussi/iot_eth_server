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


/* Standard includes. */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* FreeRTOS+TCP includes. */
#include "FreeRTOS_IP.h"
#include "FreeRTOS_Sockets.h"
#include "FreeRTOS_IP_Private.h"
#include "NetworkBufferManagement.h"
#include "NetworkInterface.h"

#include "conf_eth.h"
#include "ksz8851snl.h"
#include "ksz8851snl_reg.h"




#define PHY_REG_00_BMCR            			0x00 // Basic mode control register
#define PHY_REG_01_BMSR            			0x01 // Basic mode status register
#define PHY_REG_02_PHYSID1         			0x02 // PHYS ID 1
#define PHY_REG_03_PHYSID2         			0x03 // PHYS ID 2
#define PHY_REG_04_ADVERTISE       			0x04 // Advertisement control reg
#define PHY_REG_05_LPA             			0x05 // Link partner ability reg
#define PHY_REG_06_ANER            			0x06 //	6	RW		Auto-Negotiation Expansion Register
#define PHY_REG_07_ANNPTR          			0x07 //	7	RW		Auto-Negotiation Next Page TX
#define PHY_REG_08_RESERVED0       			0x08 // 0x08..0x0Fh	8-15	RW		RESERVED

#define BMSR_LINK_STATUS            		0x0004  //!< Link status

#ifndef PHY_LS_HIGH_CHECK_TIME_MS
	/* Check if the LinkSStatus in the PHY is still high after 15 seconds of not
	receiving packets. */
	#define PHY_LS_HIGH_CHECK_TIME_MS		15000
#endif

#ifndef PHY_LS_LOW_CHECK_TIME_MS
	/* Check if the LinkSStatus in the PHY is still low every second. */
	#define PHY_LS_LOW_CHECK_TIME_MS	1000
#endif

/* Interrupt events to process.  Currently only the Rx event is processed
although code for other events is included to allow for possible future
expansion. */
#define EMAC_IF_RX_EVENT						1UL
#define EMAC_IF_TX_EVENT						2UL
#define EMAC_IF_ERR_EVENT						4UL
#define EMAC_IF_ALL_EVENT						( EMAC_IF_RX_EVENT | EMAC_IF_TX_EVENT | EMAC_IF_ERR_EVENT )

#define ETHERNET_CONF_PHY_ADDR				BOARD_GMAC_PHY_ADDR

#ifdef ipconfigHAS_TX_CRC_OFFLOADING
	#undef ipconfigHAS_TX_CRC_OFFLOADING
#endif
/* Override this define because the KSZ8851 is programmed to set all outgoing CRC's */
#define ipconfigHAS_TX_CRC_OFFLOADING		1

#ifndef EMAC_MAX_BLOCK_TIME_MS
	#define EMAC_MAX_BLOCK_TIME_MS			100ul
#endif

/* Default the size of the stack used by the EMAC deferred handler task to 4x
the size of the stack used by the idle task - but allow this to be overridden in
FreeRTOSConfig.h as configMINIMAL_STACK_SIZE is a user definable constant. */
#ifndef configEMAC_TASK_STACK_SIZE
	#define configEMAC_TASK_STACK_SIZE 		( 6 * configMINIMAL_STACK_SIZE )
#endif

#define SPI_PDC_IDLE								0
#define SPI_PDC_RX_START						1
#define SPI_PDC_TX_ERROR						2
#define SPI_PDC_RX_COMPLETE					3
#define SPI_PDC_TX_START						4
#define SPI_PDC_RX_ERROR						5
#define SPI_PDC_TX_COMPLETE					6

#define MICREL_RX_BUFFERS						NETIF_RX_BUFFERS
#define MICREL_TX_BUFFERS						NETIF_TX_BUFFERS




/**
 * ksz8851snl driver structure.
 */
typedef struct
{
	/** Set to 1 when owner is software (ready to read), 0 for Micrel. */
	uint32_t rx_ready[MICREL_RX_BUFFERS];
	/** Set to 1 when owner is Micrel, 0 for software. */
	uint32_t tx_busy[MICREL_TX_BUFFERS];
	/** RX NetworkBufferDescriptor_t pointer list */
	NetworkBufferDescriptor_t *rx_buffers[MICREL_RX_BUFFERS];
	/** TX NetworkBufferDescriptor_t pointer list */
	NetworkBufferDescriptor_t *tx_buffers[MICREL_TX_BUFFERS];
	NetworkBufferDescriptor_t *tx_cur_buffer;

	/** Circular buffer head pointer for packet received. */
	uint32_t us_rx_head;
	/** Circular buffer tail pointer for packet to be read. */
	uint32_t us_rx_tail;
	/** Circular buffer head pointer by upper layer (buffer to be sent). */
	uint32_t us_tx_head;
	/** Circular buffer tail pointer incremented by handlers (buffer sent). */
	uint32_t us_tx_tail;

	uint32_t ul_total_tx;
	uint32_t ul_total_rx;
	uint32_t tx_space;

	/** Still experimental: hash table to allow certain multicast addresses. */
	uint16_t pusHashTable[ 4 ];

	/* ul_spi_pdc_status has "SPI_PDC_xxx" values. */
	volatile uint32_t ul_spi_pdc_status;

	/* ul_had_intn_interrupt becomes true within the INTN interrupt. */
	volatile uint32_t ul_had_intn_interrupt;

	uint16_t us_pending_frame;
} xKSZ8851_Device_t;




//TODO: check this alignment
//COMPILER_ALIGNED(8)
static xKSZ8851_Device_t xMicrelDevice;

//static TaskHandle_t xTransmitHandle;

/* Bit map of outstanding ETH interrupt events for processing.  Currently only
	the Rx interrupt is handled, although code is included for other events to
	enable future expansion. */
static volatile uint32_t ulISREvents;

/* A copy of PHY register 1: 'PHY_REG_01_BMSR' */
static uint32_t ulPHYLinkStatus = 0;

/* Holds the handle of the task used as a deferred interrupt processor.  The
	handle is used so direct notifications can be sent to the task for all EMAC/DMA
	related interrupts. */
TaskHandle_t xEMACTaskHandle = NULL;




/*-----------------------------------------------------------*/

static int ks8851_phy_reg( int );
static BaseType_t xGMACWaitLS( TickType_t );
static void prvEMACHandlerTask( void * );
static uint32_t prvEMACRxPoll( void );
static inline unsigned long ulReadMDIO( unsigned );
static void ksz8851snl_low_level_init( void );
static NetworkBufferDescriptor_t *ksz8851snl_low_level_input( void );
static void ksz8851snl_update( void );
static void ksz8851snl_rx_init( void );
static void ksz8851snl_tx_init( void );




/*------------------------ Exported functions --------------------------*/

BaseType_t xNetworkInterfaceInitialise( void )
{
	const TickType_t x5_Seconds = 5000UL;

	if( xEMACTaskHandle == NULL )
	{
		ksz8851snl_low_level_init();

		/* Wait at most 5 seconds for a Link Status in the PHY. */
		xGMACWaitLS( pdMS_TO_TICKS( x5_Seconds ) );

		/* The handler task is created at the highest possible priority to
		ensure the interrupt handler can return directly to it. */
		xTaskCreate( prvEMACHandlerTask, "KSZ8851", configEMAC_TASK_STACK_SIZE, NULL, configMAX_PRIORITIES - 1, &xEMACTaskHandle );
		configASSERT( xEMACTaskHandle );
	}

	/* When returning non-zero, the stack will become active and
    start DHCP (in configured) */
	ulPHYLinkStatus = ulReadMDIO( PHY_REG_01_BMSR );

	return ( ulPHYLinkStatus & BMSR_LINK_STATUS ) != 0;
}


BaseType_t xGetPhyLinkStatus( void )
{
	BaseType_t xResult;

	/* This function returns true if the Link Status in the PHY is high. */
	if( ( ulPHYLinkStatus & BMSR_LINK_STATUS ) != 0 )
	{
		xResult = pdTRUE;
	}
	else
	{
		xResult = pdFALSE;
	}

	return xResult;
}


BaseType_t xNetworkInterfaceOutput( NetworkBufferDescriptor_t * const pxNetworkBuffer, BaseType_t bReleaseAfterSend )
{
	BaseType_t xResult = pdFALSE;
	int txHead = xMicrelDevice.us_tx_head;

	/* Make sure the next descriptor is free. */
	if( xMicrelDevice.tx_busy[ txHead ] != pdFALSE )
	{
		/* All TX buffers busy. */
	}
	else if( ( ulPHYLinkStatus & BMSR_LINK_STATUS ) == 0 )
	{
		/* Output: LS low. */
	}
	else
	{
		/* Pass the packet. */
		xMicrelDevice.tx_buffers[ txHead ] = pxNetworkBuffer;
		/* The descriptor is now owned by Micrel. */
		xMicrelDevice.tx_busy[ txHead ] = pdTRUE;

		/* Move the head pointer. */
		if( ++txHead == MICREL_TX_BUFFERS )
		{
			txHead = 0;
		}
		xMicrelDevice.us_tx_head = txHead;
		if( xEMACTaskHandle != NULL )
		{
			xTaskNotifyGive( xEMACTaskHandle );
		}

#if( ipconfigZERO_COPY_TX_DRIVER != 1 )
	#warning Please set ipconfigZERO_COPY_TX_DRIVER to 1
#endif
		configASSERT( bReleaseAfterSend != pdFALSE );
		xResult = pdTRUE;
	}

	if( ( xResult == pdFALSE ) && ( bReleaseAfterSend  != pdFALSE ) )
	{
		vReleaseNetworkBufferAndDescriptor( pxNetworkBuffer );
	}

	return xResult;
}




/*------------------------ Local functions --------------------------*/

/* This Micrel has numbered it's PHY registers in a different way.
Translate the register index. */
static int ks8851_phy_reg( int reg )
{
	switch (reg) {
	case PHY_REG_00_BMCR:
		return REG_PHY_CNTL;	// P1MBCR;
	case PHY_REG_01_BMSR:
		return REG_PHY_STATUS;
	case PHY_REG_02_PHYSID1:
		return REG_PHY_ID_LOW;
	case PHY_REG_03_PHYSID2:
		return REG_PHY_ID_HIGH;
	case PHY_REG_04_ADVERTISE:
		return REG_PHY_AUTO_NEGOTIATION;
	case PHY_REG_05_LPA:
		return REG_PHY_REMOTE_CAPABILITY;
	}

	return 0x0;
}


static inline unsigned long ulReadMDIO( unsigned uAddress )
{
	uint16_t usPHYStatus;
	int ks8851_reg = ks8851_phy_reg( uAddress );

	if( ks8851_reg != 0 )
	{
		usPHYStatus = ksz8851_reg_read( ks8851_reg );
	}
	else
	{
		/* Other addresses not yet implemented. */
		usPHYStatus = 0;
	}

	return usPHYStatus;
}


static BaseType_t xGMACWaitLS( TickType_t xMaxTime )
{
	TickType_t xStartTime = xTaskGetTickCount();
	TickType_t xEndTime;
	BaseType_t xReturn;
	const TickType_t xShortTime = pdMS_TO_TICKS( 100UL );

	//TODO: implement a while loop instead of an infinite for
	for( ;; )
	{
		xEndTime = xTaskGetTickCount();

		if( ( xEndTime - xStartTime ) > xMaxTime )
		{
			/* Waited more than xMaxTime, return. */
			xReturn = pdFALSE;
			break;
		}

		/* Check the link status again. */
		ulPHYLinkStatus = ulReadMDIO( PHY_REG_01_BMSR );

		if( ( ulPHYLinkStatus & BMSR_LINK_STATUS ) != 0 )
		{
			/* Link is up - return. */
			xReturn = pdTRUE;
			break;
		}

		/* Link is down - wait in the Blocked state for a short while (to allow
		other tasks to execute) before checking again. */
		vTaskDelay( xShortTime );
	}

	return xReturn;
}


void INT_SPI_CALLBACK( SPI_HANDLE_TYPE *spi_handle )
{
	BaseType_t xDoWakeup = pdFALSE;
	BaseType_t xKSZTaskWoken = pdFALSE;
	uint8_t u8IntSpiCode;

	SET_SPI_CS_PIN_NO_DELAY();

	u8IntSpiCode = ksz8851snl_get_spi_state(spi_handle);

	switch( xMicrelDevice.ul_spi_pdc_status )
	{
		case SPI_PDC_RX_START:
		{
			if( INT_SPI_ERROR == u8IntSpiCode )
			{
				//TODO: abort any ongoing SPI operation

				xMicrelDevice.ul_spi_pdc_status = SPI_PDC_RX_ERROR;
				xDoWakeup = pdTRUE;
			}
			else
			{
				if( INT_SPI_READY == u8IntSpiCode )
				{
					xMicrelDevice.ul_spi_pdc_status = SPI_PDC_RX_COMPLETE;
					xDoWakeup = pdTRUE;
				}
			}
			//TODO: handle INT_SPI_BUSY
		}
		break;

		case SPI_PDC_TX_START:
		{
			if( INT_SPI_ERROR == u8IntSpiCode )
			{
				//TODO: abort any ongoing SPI operation

				xMicrelDevice.ul_spi_pdc_status = SPI_PDC_TX_ERROR;
				xDoWakeup = pdTRUE;
			}
			else
			{
				/* End of TX. */
				if( INT_SPI_READY == u8IntSpiCode )
				{
					xMicrelDevice.ul_spi_pdc_status = SPI_PDC_TX_COMPLETE;
					xDoWakeup = pdTRUE;
				}
			}
			//TODO: handle INT_SPI_BUSY
		}
		break;
	}

	if( xDoWakeup != pdFALSE )
	{
		if( xEMACTaskHandle != NULL )
		{
			vTaskNotifyGiveFromISR( xEMACTaskHandle, ( BaseType_t * ) &xKSZTaskWoken );
		}
	}
	else
	{
		/* do nothing */
	}

	portEND_SWITCHING_ISR( xKSZTaskWoken );
}


void INT_EXT_GPIO_CALLBACK( void )
{
	BaseType_t xKSZTaskWoken = pdFALSE;

	CLEAR_GPIO_INT_FLAG();

	/* Set the INTN flag. */
	xMicrelDevice.ul_had_intn_interrupt++;
	if( xEMACTaskHandle != NULL )
	{
		vTaskNotifyGiveFromISR( xEMACTaskHandle, &( xKSZTaskWoken ) );
	}

	portEND_SWITCHING_ISR( xKSZTaskWoken );
}


static void ksz8851snl_rx_populate_queue( void )
{
	uint32_t ul_index = 0;
	NetworkBufferDescriptor_t *pxNetworkBuffer;

	/* Set up the RX descriptors */
	for (ul_index = 0; ul_index < MICREL_RX_BUFFERS; ul_index++)
	{
		if( xMicrelDevice.rx_buffers[ ul_index ] == NULL )
		{
			/* Allocate a new NetworkBufferDescriptor_t with the maximum size. */
			pxNetworkBuffer = pxGetNetworkBufferWithDescriptor( ipconfigNETWORK_MTU + 36, 100 );
			if( pxNetworkBuffer == NULL )
			{
				//fail...
				configASSERT( 1 == 2 );
			}

			/* Make sure lwIP is well configured so one NetworkBufferDescriptor_t can contain the maximum packet size. */
			//LWIP_ASSERT("ksz8851snl_rx_populate_queue: NetworkBufferDescriptor_t size too small!", pbuf_clen(pxNetworkBuffer) <= 1);

			/* Save NetworkBufferDescriptor_t pointer to be sent to lwIP upper layer. */
			xMicrelDevice.rx_buffers[ ul_index ] = pxNetworkBuffer;
			/* Pass it to Micrel for reception. */
			xMicrelDevice.rx_ready[ ul_index ] = pdFALSE;
		}
	}
}


static void ksz8851snl_update()
{
	uint16_t txmir = 0;
	uint16_t fhr_status;

	switch( xMicrelDevice.ul_spi_pdc_status )
	{
		case SPI_PDC_TX_ERROR:
		{
			/* end TX transfer. */
			SET_SPI_CS_PIN();
			vTaskDelay( 2 );
			RESET_SPI_CS_PIN();
			vTaskDelay( 1 );
			SET_SPI_CS_PIN();
			vTaskDelay( 1 );

			/* Disable asynchronous transfer mode. */
			xMicrelDevice.ul_spi_pdc_status = SPI_PDC_IDLE;

			/* disable TXQ write access. */
			ksz8851_reg_clrbits( REG_RXQ_CMD, RXQ_START );

			(void)ksz8851snl_init();

			xMicrelDevice.tx_space = ksz8851_reg_read( REG_TX_MEM_INFO ) & TX_MEM_AVAILABLE_MASK;
		}
		break;

		case SPI_PDC_RX_ERROR:
		{
			/* end TX transfer. */
			SET_SPI_CS_PIN();
			vTaskDelay( 2 );
			RESET_SPI_CS_PIN();
			vTaskDelay( 1 );
			SET_SPI_CS_PIN();
			vTaskDelay( 1 );

			/* Disable asynchronous transfer mode. */
			xMicrelDevice.ul_spi_pdc_status = SPI_PDC_IDLE;

			/* disable TXQ write access. */
			ksz8851_reg_clrbits( REG_RXQ_CMD, RXQ_START );

			(void)ksz8851snl_init();

			xGMACWaitLS( pdMS_TO_TICKS( 5000UL ) );
		}
		break;
	}

	switch( xMicrelDevice.ul_spi_pdc_status )
	{
		case SPI_PDC_IDLE:
		{
			int txTail = xMicrelDevice.us_tx_tail;

			/* ========================== Handle RX ========================== */
			if( ( xMicrelDevice.ul_had_intn_interrupt != 0 ) || ( xMicrelDevice.us_pending_frame > 0 ) )
			{
				int rxHead = xMicrelDevice.us_rx_head;
				NetworkBufferDescriptor_t *pxNetworkBuffer;

				xMicrelDevice.ul_had_intn_interrupt = 0;

				if( xMicrelDevice.us_pending_frame == 0 )
				{
					/* RX step1: read and discard interrupt status for INT_RX flag. */
					(void)ksz8851_reg_read( REG_INT_STATUS );

					/* RX step2: disable all interrupts. */
					ksz8851_reg_write( REG_INT_MASK, 0 );

					/* RX step3: clear INT_RX flag. */
					ksz8851_reg_setbits( REG_INT_STATUS, INT_RX );

					/* RX step4-5: check for received frames. */
					xMicrelDevice.us_pending_frame = ((ksz8851_reg_read(REG_RX_FRAME_CNT_THRES) >> 8) & 0x00FF);
					if( xMicrelDevice.us_pending_frame == 0 )
					{
						/* RX step24: enable INT_RX flag. */
						ksz8851_reg_write(REG_INT_MASK, INT_RX);
						return;
					}
				}

				/* Now xMicrelDevice.us_pending_frame != 0 */
				/* Don't break Micrel state machine, wait for a free descriptor first! */
				if( xMicrelDevice.rx_ready[ rxHead ] != pdFALSE )
				{
					return;
				}

				pxNetworkBuffer = xMicrelDevice.rx_buffers[ rxHead ];
				if( pxNetworkBuffer == NULL )
				{
					ksz8851snl_rx_populate_queue();
					return;
				}

				/* RX step6: get RX packet status. */
				fhr_status = ksz8851_reg_read( REG_RX_FHR_STATUS );
				if( ( ( fhr_status & RX_VALID ) == 0 ) || ( ( fhr_status & RX_ERRORS ) != 0 ) )
				{
					ksz8851_reg_setbits(REG_RXQ_CMD, RXQ_CMD_FREE_PACKET);

					/* RX step4-5: check for received frames. */
					xMicrelDevice.us_pending_frame = ksz8851_reg_read(REG_RX_FRAME_CNT_THRES) >> 8;
					if( xMicrelDevice.us_pending_frame == 0 )
					{
						/* RX step24: enable INT_RX flag. */
						ksz8851_reg_write(REG_INT_MASK, INT_RX);
					}
					ulISREvents |= EMAC_IF_ERR_EVENT;
				}
				else
				{
					size_t xLength;
					/* RX step7: read frame length. */
					xLength = ksz8851_reg_read(REG_RX_FHR_BYTE_CNT) & RX_BYTE_CNT_MASK;

					/* RX step8: Drop packet if len is invalid or no descriptor available. */
					if( xLength == 0 )
					{
						ksz8851_reg_setbits( REG_RXQ_CMD, RXQ_CMD_FREE_PACKET );
						ulISREvents |= EMAC_IF_ERR_EVENT;
					}
					else
					{
						size_t xReadLength = xLength;

						xMicrelDevice.ul_total_rx++;
						/* RX step9: reset RX frame pointer. */
						ksz8851_reg_clrbits(REG_RX_ADDR_PTR, ADDR_PTR_MASK);

						/* RX step10: start RXQ read access. */
						ksz8851_reg_setbits(REG_RXQ_CMD, RXQ_START);
						/* RX step11-17: start asynchronous FIFO read operation. */
						xMicrelDevice.ul_spi_pdc_status = SPI_PDC_RX_START;

						/* Pass the buffer minus 2 bytes, see ksz8851snl.c: RXQ_TWOBYTE_OFFSET. */
						ksz8851_fifo_read( (pxNetworkBuffer->pucEthernetBuffer - 2), xReadLength );
						/* Remove CRC and update buffer length. */
						xLength -= 4;
						pxNetworkBuffer->xDataLength = xLength;
						/* Wait for SPI interrupt to set status 'SPI_PDC_RX_COMPLETE'. */
					}
				}
				break;
			}

			/* ========================== Handle TX ========================== */
			/* Fetch next packet to be sent. */
			if( ( xMicrelDevice.tx_busy[ txTail ] != pdFALSE ) &&
				 ( xMicrelDevice.us_pending_frame == 0 ) &&
				 ( xMicrelDevice.ul_had_intn_interrupt == 0 ) )
			{
				NetworkBufferDescriptor_t *pxNetworkBuffer = xMicrelDevice.tx_buffers[ txTail ];
				size_t xLength = pxNetworkBuffer->xDataLength;
				int iIndex = xLength;

				xLength = 4 * ( ( xLength + 3 ) / 4 );
				while( iIndex < ( int ) xLength )
				{
					pxNetworkBuffer->pucEthernetBuffer[ iIndex ] = '\0';
					iIndex++;
				}
				pxNetworkBuffer->xDataLength = xLength;

				/* TX step1: check if TXQ memory size is available for transmit. */
				txmir = ksz8851_reg_read( REG_TX_MEM_INFO );
				txmir = txmir & TX_MEM_AVAILABLE_MASK;

				if( txmir < ( xLength + 8 ) )
				{
					//TODO: space can be requested and interrupt is then generated when available...
					/* wait... */
				}
				else
				{
					/* TX step2: disable all interrupts. */
					ksz8851_reg_write( REG_INT_MASK, 0 );

					xMicrelDevice.tx_space -= xLength;

					/* TX step3: enable TXQ write access. */
					ksz8851_reg_setbits( REG_RXQ_CMD, RXQ_START );
					/* TX step4-8: perform FIFO write operation. */
					xMicrelDevice.ul_spi_pdc_status = SPI_PDC_TX_START;
					xMicrelDevice.tx_cur_buffer = pxNetworkBuffer;
					xMicrelDevice.ul_total_tx++;

					ksz8851_fifo_write( pxNetworkBuffer->pucEthernetBuffer, xLength );
				}
			}
		}
		break;

		case SPI_PDC_RX_COMPLETE:
		{
			int rxHead = xMicrelDevice.us_rx_head;

			/* Disable asynchronous transfer mode. */
			xMicrelDevice.ul_spi_pdc_status = SPI_PDC_IDLE;

			/* RX step21: end RXQ read access. */
			ksz8851_reg_clrbits(REG_RXQ_CMD, RXQ_START);

			/* RX step22-23: update frame count to be read. */
			xMicrelDevice.us_pending_frame -= 1;

			/* RX step24: enable INT_RX flag if transfer complete. */
			if( xMicrelDevice.us_pending_frame == 0 )
			{
				ksz8851_reg_write(REG_INT_MASK, INT_RX);
			}

			/* Mark descriptor ready to be read. */
			xMicrelDevice.rx_ready[ rxHead ] = pdTRUE;
			if( ++rxHead == MICREL_RX_BUFFERS )
			{
				rxHead = 0;
			}
			xMicrelDevice.us_rx_head = rxHead;
			/* Tell prvEMACHandlerTask that RX packets are available. */
			ulISREvents |= EMAC_IF_RX_EVENT;
		}
		break;

		case SPI_PDC_TX_COMPLETE:
		{
			int txTail = xMicrelDevice.us_tx_tail;
			NetworkBufferDescriptor_t *pxNetworkBuffer = xMicrelDevice.tx_buffers[ txTail ];

			/* Disable asynchronous transfer mode. */
			xMicrelDevice.ul_spi_pdc_status = SPI_PDC_IDLE;

			/* TX step12: disable TXQ write access. */
			ksz8851_reg_clrbits( REG_RXQ_CMD, RXQ_START );

			xMicrelDevice.tx_space = ksz8851_reg_read( REG_TX_MEM_INFO ) & TX_MEM_AVAILABLE_MASK;

			/* TX step12.1: enqueue frame in TXQ. */
			ksz8851_reg_setbits( REG_TXQ_CMD, TXQ_ENQUEUE );

			/* RX step13: enable INT_RX flag. */
			//ksz8851_reg_write( REG_INT_MASK, INT_RX );
			/* Buffer sent, free the corresponding buffer and mark descriptor as owned by software. */
			vReleaseNetworkBufferAndDescriptor( pxNetworkBuffer );

			xMicrelDevice.tx_buffers[ txTail ] = NULL;
			xMicrelDevice.tx_busy[ txTail ] = pdFALSE;
			if( ++txTail == MICREL_TX_BUFFERS )
			{
				txTail = 0;
			}

			xMicrelDevice.us_tx_tail = txTail;
			//if( xTransmitHandle != NULL )
			//{
			//	xTaskNotifyGive( xTransmitHandle );
			//}

			/* RX step13: enable INT_RX flag. */
			ksz8851_reg_write( REG_INT_MASK, INT_RX );
			/* Prevent the EMAC task from sleeping a single time. */
			ulISREvents |= EMAC_IF_TX_EVENT;
		}
		break;
	}
}


static void ksz8851snl_rx_init()
{
	uint32_t ul_index = 0;

	/* Init pointer index. */
	xMicrelDevice.us_rx_head = 0;
	xMicrelDevice.us_rx_tail = 0;

	/* Set up the RX descriptors. */
	for (ul_index = 0; ul_index < MICREL_RX_BUFFERS; ul_index++)
	{
		xMicrelDevice.rx_buffers[ul_index] = NULL;
		xMicrelDevice.rx_ready[ul_index] = pdFALSE;
	}

	/* Build RX buffer and descriptors. */
	ksz8851snl_rx_populate_queue();
}


static void ksz8851snl_tx_init()
{
	uint32_t ul_index = 0;

	/* Init TX index pointer. */
	xMicrelDevice.us_tx_head = 0;
	xMicrelDevice.us_tx_tail = 0;

	/* Set up the TX descriptors */
	for( ul_index = 0; ul_index < MICREL_TX_BUFFERS; ul_index++ )
	{
		xMicrelDevice.tx_busy[ul_index] = pdFALSE;
	}

	xMicrelDevice.tx_space = 6144;
}


static void ksz8851snl_low_level_init( void )
{
	ksz8851snl_rx_init();
	ksz8851snl_tx_init();

	/* init SPI link. */
	if( false == ksz8851snl_init() )
	{
		//fail...
		configASSERT(0 == 1);
	}

	memset( xMicrelDevice.pusHashTable, 255, sizeof( xMicrelDevice.pusHashTable ) );
	ksz8851_reg_write( REG_MAC_HASH_0, FreeRTOS_htons( xMicrelDevice.pusHashTable[ 0 ] ) );
	ksz8851_reg_write( REG_MAC_HASH_2, FreeRTOS_htons( xMicrelDevice.pusHashTable[ 1 ] ) );
	ksz8851_reg_write( REG_MAC_HASH_4, FreeRTOS_htons( xMicrelDevice.pusHashTable[ 2 ] ) );
	ksz8851_reg_write( REG_MAC_HASH_6, FreeRTOS_htons( xMicrelDevice.pusHashTable[ 3 ] ) );
}


static NetworkBufferDescriptor_t *ksz8851snl_low_level_input( void )
{
	NetworkBufferDescriptor_t *pxNetworkBuffer = NULL;
	int rxTail = xMicrelDevice.us_rx_tail;

	/* Check that descriptor is owned by software (ie packet received). */
	if( xMicrelDevice.rx_ready[ rxTail ] != pdFALSE )
	{
		/* Fetch pre-allocated buffer */
		pxNetworkBuffer = xMicrelDevice.rx_buffers[ rxTail ];

		/* Remove this pbuf from its descriptor. */
		xMicrelDevice.rx_buffers[ rxTail ] = NULL;

		/* Clears rx_ready and sets rx_buffers. */
		ksz8851snl_rx_populate_queue();

		if( ++rxTail == MICREL_RX_BUFFERS )
		{
			rxTail = 0;
		}

		xMicrelDevice.us_rx_tail = rxTail;
	}

	return pxNetworkBuffer;
}


static uint32_t prvEMACRxPoll( void )
{
	NetworkBufferDescriptor_t *pxNetworkBuffer;
	IPStackEvent_t xRxEvent = { eNetworkRxEvent, NULL };
	uint32_t ulReturnValue = 0;

	//TODO: use while(pxNetworkBuffer != NULL) instead of infinite loop and break
	for( ;; )
	{
		pxNetworkBuffer = ksz8851snl_low_level_input();
	
		if( pxNetworkBuffer == NULL )
		{
			break;
		}

		ulReturnValue++;

		xRxEvent.pvData = ( void * )pxNetworkBuffer;
		/* Send the descriptor to the IP task for processing. */
		if( xSendEventStructToIPTask( &xRxEvent, 100UL ) != pdTRUE )
		{
			vReleaseNetworkBufferAndDescriptor( pxNetworkBuffer );
		}
	}

	return ulReturnValue;
}


static void prvEMACHandlerTask( void *pvParameters )
{
	TimeOut_t xPhyTime;
	TickType_t xPhyRemTime;
	BaseType_t xResult = 0;
	uint32_t xStatus;
	const TickType_t ulMaxBlockTime = pdMS_TO_TICKS( EMAC_MAX_BLOCK_TIME_MS );

	/* Remove compiler warnings about unused parameters. */
	( void ) pvParameters;

	configASSERT( xEMACTaskHandle );

	vTaskSetTimeOutState( &xPhyTime );
	xPhyRemTime = pdMS_TO_TICKS( PHY_LS_LOW_CHECK_TIME_MS );

	for( ;; )
	{
		/* Run the state-machine of the ksz8851 driver. */
		ksz8851snl_update();

		if( ( ulISREvents & EMAC_IF_ALL_EVENT ) == 0 )
		{
			/* No events to process now, wait for the next. */
			ulTaskNotifyTake( pdTRUE, ulMaxBlockTime );
		}

		if( ( ulISREvents & EMAC_IF_RX_EVENT ) != 0 )
		{
			ulISREvents &= ~EMAC_IF_RX_EVENT;

			/* Wait for the EMAC interrupt to indicate that another packet has been received. */
			xResult = prvEMACRxPoll();
		}

		if( ( ulISREvents & EMAC_IF_TX_EVENT ) != 0 )
		{
			/* TX buffers already released. */
			ulISREvents &= ~EMAC_IF_TX_EVENT;
		}

		if( ( ulISREvents & EMAC_IF_ERR_EVENT ) != 0 )
		{
			/* Future extension: logging about errors that occurred. */
			ulISREvents &= ~EMAC_IF_ERR_EVENT;
		}

		if( xResult > 0 )
		{
			/* As long as packets are being received, assume that
			the Link Status is high. */
			ulPHYLinkStatus |= BMSR_LINK_STATUS;
			/* A packet was received. No need to check for the PHY status now,
			but set a timer to check it later on. */
			vTaskSetTimeOutState( &xPhyTime );
			xPhyRemTime = pdMS_TO_TICKS( PHY_LS_HIGH_CHECK_TIME_MS );
			xResult = 0;
		}
		else if( ( xTaskCheckForTimeOut( &xPhyTime, &xPhyRemTime ) != pdFALSE ) &&
			      ( xMicrelDevice.ul_spi_pdc_status == SPI_PDC_IDLE ) )
		{
			/* Check the link status again. */
			xStatus = ulReadMDIO( PHY_REG_01_BMSR );

			if( ( ulPHYLinkStatus & BMSR_LINK_STATUS ) != ( xStatus & BMSR_LINK_STATUS ) )
			{
				ulPHYLinkStatus = xStatus;
			}

			vTaskSetTimeOutState( &xPhyTime );
			if( ( ulPHYLinkStatus & BMSR_LINK_STATUS ) != 0 )
			{
				xPhyRemTime = pdMS_TO_TICKS( PHY_LS_HIGH_CHECK_TIME_MS );
			}
			else
			{
				xPhyRemTime = pdMS_TO_TICKS( PHY_LS_LOW_CHECK_TIME_MS );
			}
		}
	}
}




/* ------------ End of file -------------- */



