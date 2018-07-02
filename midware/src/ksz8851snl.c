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



/* ------------------- Local inclusions -------------------- */


#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "conf_eth.h"
#include "ksz8851snl.h"
#include "ksz8851snl_reg.h"




/* ------------------- Local variables -------------------- */

static SPI_HandleTypeDef SpiHandle;




/* ------------------- Local functions prototypes -------------------- */

static bool ksz8851snl_interface_init(void);
static inline void ksz8851snl_hard_reset(void);




/* ------------------- Local functions -------------------- */

void DMA1_Stream3_IRQHandler(void)
{
	HAL_DMA_IRQHandler(SpiHandle.hdmarx);
}


void DMA1_Stream4_IRQHandler(void)
{
	HAL_DMA_IRQHandler(SpiHandle.hdmatx);
}


void SPI2_IRQHandler(void)
{
	HAL_SPI_IRQHandler(&SpiHandle);
}




static bool ksz8851snl_interface_init(void)
{
	bool success = true;

	/* Configure the SPI peripheral */
	/* Set the SPI parameters */
	SpiHandle.Instance               = SPI2;
	SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
	SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
	SpiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
	SpiHandle.Init.CLKPolarity       = SPI_POLARITY_LOW;
	SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
	SpiHandle.Init.CRCPolynomial     = 7;
	SpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
	SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
	SpiHandle.Init.NSS               = SPI_NSS_SOFT;
	SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLE;
	SpiHandle.Init.Mode              = SPI_MODE_MASTER;
	if(HAL_SPI_Init(&SpiHandle) != HAL_OK)
	{
		success = false;
	}

	return success;
}


static inline void ksz8851snl_hard_reset(void)
{
	/* Perform hardware reset with respect to the reset timing from the datasheet. */
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
	osDelay(100);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
	osDelay(100);
}




/* ------------------- Exported functions -------------------- */

uint8_t ksz8851snl_get_spi_state(SPI_HandleTypeDef *hspi)
{
	HAL_SPI_StateTypeDef spiState;
	uint8_t spiIntCode;

	spiState = HAL_SPI_GetState(hspi);

	switch(spiState)
	{
		case HAL_SPI_STATE_READY:
		{
			spiIntCode = INT_SPI_READY;
			break;
		}
		case HAL_SPI_STATE_BUSY:
		case HAL_SPI_STATE_BUSY_TX:
		case HAL_SPI_STATE_BUSY_RX:
		case HAL_SPI_STATE_BUSY_TX_RX:
		{
			spiIntCode = INT_SPI_BUSY;
			break;
		}
		case HAL_SPI_STATE_ERROR:
		default:
		{
			//TODO: get and understand the error code
			//error_code = HAL_SPI_GetError(hspi);
			spiIntCode = INT_SPI_ERROR;
		}
	}

	return spiIntCode;
}


void ksz8851_fifo_read(uint8_t *buf, uint16_t len)
{
	uint8_t inbuf[9];
	uint8_t pad_bytes;

	//TODO: check len value

	RESET_SPI_CS_PIN();

	/* calculate number of dummy pad bytes to read a 32-bits aligned buffer */
	pad_bytes = ((len & 0x03) != 0) ? (4 - (len & 0x03)) : 0;

	inbuf[0] = FIFO_READ;

	/* Perform blocking SPI transfer. */
	(void)HAL_SPI_TransmitReceive(&SpiHandle, (uint8_t*)inbuf, (uint8_t*)inbuf, 9, 2000);

	/* update length to a 32-bits aligned value */
	len += pad_bytes;
	/* Perform non-blocking DMA SPI transfer */
	(void)HAL_SPI_TransmitReceive_DMA(&SpiHandle, (uint8_t*)buf, (uint8_t*)buf, len);

	/* an interrupt will occur */
}


void ksz8851_fifo_write(uint8_t *buf, uint16_t len)
{
	uint8_t outbuf[5];
	uint8_t pad_bytes;
	static uint8_t frameID = 0;

	RESET_SPI_CS_PIN();

	//TODO: check len value

	/* length is 11 bits long */
	len &= 0x07FF;
	/* calculate number of dummy pad bytes to send a 32-bits aligned buffer */
	pad_bytes = ((len & 0x03) != 0) ? (4 - (len & 0x03)) : 0;

	/* Prepare control word and byte count. */
	outbuf[0] = FIFO_WRITE;
	outbuf[1] = frameID++ & 0x3f;
	outbuf[2] = 0;
	outbuf[3] = len & 0xff;
	outbuf[4] = len >> 8;

	/* Perform blocking SPI transfer. */
	(void)HAL_SPI_Transmit(&SpiHandle, (uint8_t*)outbuf, 5, 2000);

	/* update length to a 32-bits aligned value */
	len += pad_bytes;
	/* ATTENTION: pad bytes are the bytes beyond buffer length (can be any rubbish value) */
	/* Perform non-blocking DMA SPI transfer */
	(void)HAL_SPI_TransmitReceive_DMA(&SpiHandle, (uint8_t*)buf, (uint8_t*)buf, len);

	/* an interrupt will occur */
}


uint16_t ksz8851_reg_read(uint16_t reg)
{
	uint8_t	inbuf[4];
	uint8_t	outbuf[4];
	uint16_t cmd = 0;
	uint16_t res = 0;

	RESET_SPI_CS_PIN();

	/* Move register address to cmd bits 9-2, make 32-bit address. */
	cmd = (reg << 2) & REG_ADDR_MASK;

	/* Last 2 bits still under "don't care bits" handled with byte enable. */
	/* Select byte enable for command. */
	if (reg & 2) {
		/* Odd word address writes bytes 2 and 3 */
		cmd |= (0xc << 10);
	} else {
		/* Even word address write bytes 0 and 1 */
		cmd |= (0x3 << 10);
	}

	/* Add command read code. */
	cmd |= CMD_READ;
	outbuf[0] = cmd >> 8;
	outbuf[1] = cmd & 0xff;
	outbuf[2] = 0xff;
	outbuf[3] = 0xff;

	/* Perform blocking SPI transfer. Discard function returned value! TODO: handle it? */
	(void)HAL_SPI_TransmitReceive(&SpiHandle, (uint8_t*)outbuf, (uint8_t *)inbuf, 4, 2000);

	SET_SPI_CS_PIN();

	res = (inbuf[3] << 8) | inbuf[2];
	return res;
}


void ksz8851_reg_write(uint16_t reg, uint16_t wrdata)
{
	uint8_t	outbuf[4];
	uint16_t cmd = 0;

	RESET_SPI_CS_PIN();

	/* Move register address to cmd bits 9-2, make 32-bit address. */
	cmd = (reg << 2) & REG_ADDR_MASK;

	/* Last 2 bits still under "don't care bits" handled with byte enable. */
	/* Select byte enable for command. */
	if (reg & 2) {
		/* Odd word address writes bytes 2 and 3 */
		cmd |= (0xc << 10);
	} else {
		/* Even word address write bytes 0 and 1 */
		cmd |= (0x3 << 10);
	}

	/* Add command write code. */
	cmd |= CMD_WRITE;
	outbuf[0] = cmd >> 8;
	outbuf[1] = cmd & 0xff;
	outbuf[2] = wrdata & 0xff;
	outbuf[3] = wrdata >> 8;

	/* Perform blocking SPI transfer. Discard function returned value! TODO: handle it? */
	(void)HAL_SPI_Transmit(&SpiHandle, (uint8_t*)outbuf, 4, 2000);

	SET_SPI_CS_PIN();
}


void ksz8851_reg_setbits(uint16_t reg, uint16_t bits_to_set)
{
   uint16_t	temp;

   temp = ksz8851_reg_read(reg);
   temp |= bits_to_set;
   ksz8851_reg_write(reg, temp);
}


void ksz8851_reg_clrbits(uint16_t reg, uint16_t bits_to_clr)
{
   uint16_t	temp;

   temp = ksz8851_reg_read(reg);
   temp &= ~(uint32_t) bits_to_clr;
   ksz8851_reg_write(reg, temp);
}


bool ksz8851snl_init(void)
{
	uint32_t count = 0;
	uint16_t dev_id = 0;
	bool success = true;

	/* Initialize the SPI interface. */
	if( true == ksz8851snl_interface_init() )
	{
		/* Reset the Micrel in a proper state. */
		do
		{
			ksz8851snl_hard_reset();

			/* Init step1: read chip ID. */
			dev_id = ksz8851_reg_read(REG_CHIP_ID);
			if (++count > 10)
			{
				return 1;
			}
		} while ((dev_id & 0xFFF0) != CHIP_ID_8851_16);

		/* Init step2-4: write QMU MAC address (low, middle then high). */
		ksz8851_reg_write(REG_MAC_ADDR_0, (ETHERNET_CONF_ETHADDR4 << 8) | ETHERNET_CONF_ETHADDR5);
		ksz8851_reg_write(REG_MAC_ADDR_2, (ETHERNET_CONF_ETHADDR2 << 8) | ETHERNET_CONF_ETHADDR3);
		ksz8851_reg_write(REG_MAC_ADDR_4, (ETHERNET_CONF_ETHADDR0 << 8) | ETHERNET_CONF_ETHADDR1);

		/* Init step5: enable QMU Transmit Frame Data Pointer Auto Increment. */
		ksz8851_reg_write(REG_TX_ADDR_PTR, ADDR_PTR_AUTO_INC);

		/* Init step6: configure QMU transmit control register. */
		ksz8851_reg_write(REG_TX_CTRL,
				TX_CTRL_ICMP_CHECKSUM |
				TX_CTRL_UDP_CHECKSUM |
				TX_CTRL_TCP_CHECKSUM |
				TX_CTRL_IP_CHECKSUM |
				TX_CTRL_FLOW_ENABLE |
				TX_CTRL_PAD_ENABLE |
				TX_CTRL_CRC_ENABLE
			);

		/* Init step7: enable QMU Receive Frame Data Pointer Auto Increment. */
		ksz8851_reg_write(REG_RX_ADDR_PTR, ADDR_PTR_AUTO_INC);

		/* Init step8: configure QMU Receive Frame Threshold for one frame. */
		ksz8851_reg_write(REG_RX_FRAME_CNT_THRES, 1);

		/* Init step9: configure QMU receive control register1. */
		ksz8851_reg_write(REG_RX_CTRL1,
								RX_CTRL_UDP_CHECKSUM |
								RX_CTRL_TCP_CHECKSUM |
								RX_CTRL_IP_CHECKSUM |
								RX_CTRL_MAC_FILTER |
								RX_CTRL_FLOW_ENABLE |
								RX_CTRL_BROADCAST |
								RX_CTRL_ALL_MULTICAST|
								RX_CTRL_UNICAST |
								RX_CTRL_PROMISCUOUS);

		/* Init step10: configure QMU receive control register2. */
		ksz8851_reg_write(REG_RX_CTRL2,
								RX_CTRL_IPV6_UDP_NOCHECKSUM |
								RX_CTRL_UDP_LITE_CHECKSUM |
								RX_CTRL_ICMP_CHECKSUM |
								RX_CTRL_BURST_LEN_FRAME);

		/* Init step11: configure QMU receive queue: trigger INT and auto-dequeue frame. */
		ksz8851_reg_write(REG_RXQ_CMD, RXQ_CMD_CNTL | RXQ_TWOBYTE_OFFSET);

		/* Init step12: adjust SPI data output delay. */
		ksz8851_reg_write(REG_BUS_CLOCK_CTRL, BUS_CLOCK_166 | BUS_CLOCK_DIVIDEDBY_1);

		/* Init step13: restart auto-negotiation. */
		ksz8851_reg_setbits(REG_PORT_CTRL, PORT_AUTO_NEG_RESTART);

		/* Init step13.1: force link in half duplex if auto-negotiation failed. */
		if ((ksz8851_reg_read(REG_PORT_CTRL) & PORT_AUTO_NEG_RESTART) != PORT_AUTO_NEG_RESTART)
		{
			ksz8851_reg_clrbits(REG_PORT_CTRL, PORT_FORCE_FULL_DUPLEX);
		}

		/* Init step14: clear interrupt status. */
		ksz8851_reg_write(REG_INT_STATUS, 0xFFFF);

		/* Init step15: set interrupt mask. */
		ksz8851_reg_write(REG_INT_MASK, INT_RX);

		/* Init step16: enable QMU Transmit. */
		ksz8851_reg_setbits(REG_TX_CTRL, TX_CTRL_ENABLE);

		/* Init step17: enable QMU Receive. */
		ksz8851_reg_setbits(REG_RX_CTRL1, RX_CTRL_ENABLE);
	}
	else
	{
		success = false;
	}

	return success;
}



