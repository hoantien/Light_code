/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms,
 * with or without modification, are strictly prohibited without prior
 * permission of The LightCo.
 *
 * @file    hal_spi.c
 * @author  The LightCo
 * @version V1.0.0
 * @date    01-Mar-2016
 * @brief   This file provides firmware functions to manage the following
 *          functionalities of SPI peripheral:
 *			  + Initialization and Configuration
 *			  + Data transfers functions
 *			  + Interrupts management
 *
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "hal_spi.h"
#include "assert.h"
#include "hal_vic.h"
#include "board_config.h"
#include <stdio.h>

/* HW definition of Max size of XFER register */
#define SSI_MAX_XFER_SIZE_32_BIT
#define TX_FIFO_DEPTH 32

/*
 * TODO: undefine MACRO below if you don't want to test SPI in loop back mode
 */
/* #define TEST_LOOPBACK */

/* Private typedef -----------------------------------------------------------*/
spi_hw_config_t spi_hw[3] =
{
	{.spix = SPI0_MASTER},		/* SPI hardware channel 0 */
	{.spix = SPI1_SLAVE},		/* SPI hardware channel 1 */
	{.spix = SPI2_SLAVE}		/* SPI hardware channel 2 */
};

/* Store interrupt handler function pointer */
spi_interrupt_handle_t spi_interrupt[3] =
{
	/* Initializing default value for SPI CH0 interrupt */
	{
		.multi_master_handler		= NULL,
		.rx_fifo_full_handler		= NULL,
		.rx_fifo_overflow_handler	= NULL,
		.rx_fifo_underflow_handler	= NULL,
		.tx_fifo_overflow_handler	= NULL
	},

	/* Initializing default value for SPI CH1 interrupt */
	{
		.multi_master_handler		= NULL,
		.rx_fifo_full_handler		= NULL,
		.rx_fifo_overflow_handler	= NULL,
		.rx_fifo_underflow_handler	= NULL,
		.tx_fifo_overflow_handler	= NULL
	},

	/* Initializing default value for SPI CH2 interrupt */
	{
		.multi_master_handler		= NULL,
		.rx_fifo_full_handler		= NULL,
		.rx_fifo_overflow_handler	= NULL,
		.rx_fifo_underflow_handler	= NULL,
		.tx_fifo_overflow_handler	= NULL
	}
};

flag_status_t spi_interrupt_enable[3] = {DISABLE, DISABLE, DISABLE};
/* Private define ------------------------------------------------------------*/

/* Define time out for wait fifo */
#define TIME_OUT_WAIT_TX_FIFO 	20000
#define TIME_OUT_TX				50000

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
spi_config_t test_hal_spi_init;
int hal_spi_init_count;
spi_config_t test_hal_spi_interrupt_deinit;
int hal_spi_interrupt_deinit_count;
spi_config_t test_hal_spi_interrupt_init_conf;
spi_interrupt_init_t test_hal_spi_interrupt_init_interrupt;
int hal_spi_interrupt_init_count;
spi_config_t test_hal_spi_receive_buf;
int hal_spi_receive_buf_count;
/* Private function prototypes -----------------------------------------------*/
static void hal_spi_irq_handler(void);
/* Private functions ---------------------------------------------------------*/
static void spi_delay(__IO uint32_t time)
{
}

static uint8_t wait_tx_fifo_empty(spi_config_t *spi)
{
}

/* Exported functions --------------------------------------------------------*/
spi_status_t hal_spi_init(spi_config_t *spi)
{
	test_hal_spi_init = *spi;
	hal_spi_init_count ++;
	return SPI_OK;
}

spi_status_t hal_spi_slave_select(uint8_t cs_pin, flag_status_t value)
{
}

void hal_spi_disable(spi_config_t *spi)
{
}

void hal_spi_enable(spi_config_t *spi)
{
}

void hal_spi_reset(spi_config_t *spi)
{
}

/*
 * Data buffer need to receive has to typecast to uint8_t
 * e.g: uint8_t rx_data[len]; // len is number of data
 * 		hal_spi_receive_buf(spi, (uint8_t *)rx_buf, len);

 * e.g: uint16_t rx_data[len]; // len is number of data
 * 		hal_spi_receive_buf(spi, (uint8_t *)rx_buf, len);

 * e.g: uint32_t rx_data[len]; // len is number of data
 * 		hal_spi_receive_buf(spi, (uint8_t *)rx_buf, len);
 */
spi_status_t hal_spi_receive_buf(spi_config_t *spi, uint8_t *rx_buf, uint16_t len)
{
	test_hal_spi_receive_buf = *spi;
	hal_spi_receive_buf_count ++;
	for (uint16_t i=0; i<len; i++)
	{
		*(rx_buf+i) = 1 + i;
	}
	return SPI_OK;
}

/*
 * Data buffer need to transmit has to typecast to uint8_t
 * e.g: uint8_t tx_data[len]; // len is number of data
 * 		hal_spi_transmit_buf(spi, (uint8_t *)tx_buf, len);

 * e.g: uint16_t tx_data[len]; // len is number of data
 * 		hal_spi_transmit_buf(spi, (uint8_t *)tx_buf, len);

 * e.g: uint32_t tx_data[len]; // len is number of data
 * 		hal_spi_transmit_buf(spi, (uint8_t *)tx_buf, len);
 */
spi_status_t hal_spi_transmit_buf(spi_config_t *spi, uint8_t *tx_buf,
									uint16_t len)
{
}

/*
 * Data buffer need to transmit and receive has to typecast to uint8_t
 * e.g: uint8_t tx_data[len]; // len is number of data
 * 		spi_buf.txbuf.buf = (uint8_t *)tx_data;

 * e.g: uint16_t tx_data[len]; // len is number of data
 * 		spi_buf.txbuf.buf = (uint8_t *)tx_data;

 * e.g: uint32_t tx_data[len]; // len is number of data
 * 		spi_buf.txbuf.buf = (uint8_t *)tx_data;
 */
spi_status_t hal_spi_transceiver(spi_config_t *spi, spi_buffer_t *buff)
{
}

void hal_spi_dma_init(spi_config_t *spi, dma_config_t *dma)
{
}

void hal_spi_interrupt_init(spi_config_t *spi, spi_interrupt_init_t *interrupt)
{
	test_hal_spi_interrupt_init_conf = *spi;
	test_hal_spi_interrupt_init_interrupt = *interrupt;
	hal_spi_interrupt_init_count ++;
}

void hal_spi_interrupt_deinit(spi_config_t *spi)
{
	test_hal_spi_interrupt_deinit = *spi;
	hal_spi_interrupt_deinit_count ++;
}

static void hal_spi_irq_handler(void)
{
}
/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
