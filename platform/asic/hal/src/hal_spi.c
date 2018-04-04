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
#include "assert.h"
#include "board_config.h"
#include "cortex_r4.h"
#include "hal_vic.h"
#include "hal_spi.h"
#include "hal_cache.h"

/* Private macro -------------------------------------------------------------*/
/* HW definition of Max size of XFER register */
#define SSI_MAX_XFER_SIZE_32_BIT
#define SPI_FIFO_DEPTH 32	/* The real FIFO is 32 data registers */

/*
 * TODO: undefined MACRO below if you don't want to test SPI in loop back mode
 */
/* #define TEST_LOOPBACK */

/* Private typedef -----------------------------------------------------------*/
spi_hw_config_t spi_hw[3] =
{
	{.spix = SPI_MASTER},		/* SPI hardware channel 0 */
	{.spix = SPI_SLAVE0},		/* SPI hardware channel 1 */
	{.spix = SPI_SLAVE1}		/* SPI hardware channel 2 */
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
#define TIME_OUT_RX				2000

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void hal_spi_0_irq(void);
static void hal_spi_1_irq(void);
static void hal_spi_2_irq(void);
static void hal_spi_irq_handler(ssi_channel_t channel);

static spi_status_t hal_spi_transceiver_dma(spi_config_t *spi,
											spi_buffer_t *buff);
static spi_status_t hal_spi_transceiver_normal(spi_config_t *spi,
												spi_buffer_t *buff);
static spi_status_t hal_spi_transmit_buf_dma(spi_config_t *spi, uint8_t *tx_buf,
												uint16_t len);
static spi_status_t hal_spi_transmit_buf_normal(spi_config_t *spi,
												uint8_t *tx_buf, uint16_t len);
static spi_status_t hal_spi_receive_buf_dma(spi_config_t *spi, uint8_t *rx_buf,
											uint16_t len);
static spi_status_t hal_spi_receive_buf_normal(spi_config_t *spi,
												uint8_t *rx_buf, uint16_t len);




/* Private functions ---------------------------------------------------------*/
static void spi_delay(__IO uint32_t time)
{
	while(time > 0)
		time--;
}

static uint8_t wait_tx_fifo_empty(spi_config_t *spi)
{
	/*
	 * TODO: have to calculate time out with other baud rate
	 */
	__IO uint32_t time_out = TIME_OUT_WAIT_TX_FIFO;
	uint32_t status_reg = 0;
	uint8_t rx_fifo = 0;
	/* Check received data in fifo */
	do
	{
		status_reg = spi_hw[spi->channel].spix->SR;
		time_out--;
	} while(((status_reg & (SR_TF_EMPT | SR_BUSY)) != SR_TF_EMPT)
			&& (time_out > 0));

	rx_fifo = spi_hw[spi->channel].spix->RXFLR;

	return rx_fifo;
}

/* Exported functions --------------------------------------------------------*/
spi_status_t hal_spi_init(spi_config_t *spi)
{
	uint32_t control_0_reg = 0;

	/* Check parameters */
	assert_param(IS_SPI_DATA_SIZE(spi->data_size));
	assert_param(IS_SPI_PHASE_MODE(spi->spi_mode));
	assert_param(IS_SPI_TRANSFER_MODE(spi->transfer_mode));
	assert_param(IS_SPI_TYPE(spi->spi_type));
	assert_param(IS_SSI_MODE(spi->ssi_mode));
	assert_param(IS_SSI_CHANNEL(spi->channel));

#if ((ASIC_NUM == ASIC2) || (ASIC_NUM == ASIC3))
	if (SSI_CH2 == spi->channel)
	{
		/* Bit [5,7:8] of OxA8 register have to be high */
		*(volatile uint32_t*)(SCU_BASE + 0xA8) &= ~(0x000001A0);
	}
#endif
	/* Clear interrupt mark register */
	spi_hw[spi->channel].spix->IMR = 0;

	/* Disable SSI bit 0- Offset 0x08 - SSI Enable Register */
	spi_hw[spi->channel].spix->SSIENR = 0;

	/* Configure baud rate
	 * Fclk_out = Fssi_clk/SCKDV Offset 0x14 - Baud Rate Select
	 */
	spi_hw[spi->channel].spix->BAUDR = BOARD_PCLOCK / spi->master_clk_freq;

#ifdef TEST_LOOPBACK
	control_0_reg |= (uint32_t)(1 << 11);
#endif

	/* Configure SSI mode */
	switch(spi->ssi_mode)
	{
		case SSI_MOTO_SPI:
		{
			/* Configure SPI mode - bit 4 Offset 0x00 - Control Register 0  */
			control_0_reg |= SPI_FRF_SPI;
			/* Configure SPI frame size 32bit mode - bit 20 : 16 */
			control_0_reg |= ((spi->data_size - 1) << SPI_DFS_32_OFFSET);
			break;
		}
		case SSI_TI_SSP:
		{
			control_0_reg |= SPI_FRF_SSP;
			break;
		}
		case SSI_NS_MICROWIRE:
		{
			/* Configure Microwire mode */
			control_0_reg |= SPI_FRF_MICROWIRE;

			/* Configure Microwire frame size */
			if (spi->data_size == DATASIZE_8BIT ||
					spi->data_size == DATASIZE_16BIT)
			{
				control_0_reg |= ((spi->data_size - 1) << SPI_CFS_OFFSET);
			}
			else
			{
				return SPI_ERROR;
			}
			break;
		}
		default:
		{
			return SPI_ERROR;
		}
	}

	/* Configure SPI frame format type - bit [22:21] */
	control_0_reg |= (spi->spi_type << SPI_FRF_FRAME_OFFSET);

	/* Configure transfer mode - bit[9:8] */
	control_0_reg |= (spi->transfer_mode << SPI_TMOD_OFFSET);

	/* Configure clock polarity and phase */
	switch (spi->spi_mode)
	{
		/* bit [7:6] */
		case SPI_MODE_0:
		{
			control_0_reg &= ~SPI_SCOL;
			control_0_reg &= ~SPI_SCPH;
			break;
		}
		case SPI_MODE_1:
		{
			control_0_reg &= ~SPI_SCOL;
			control_0_reg |= SPI_SCPH;
			break;
		}
		case SPI_MODE_2:
		{
			control_0_reg |= SPI_SCOL;
			control_0_reg &= ~SPI_SCPH;
			break;
		}
		case SPI_MODE_3:
		{
			control_0_reg |= SPI_SCOL | SPI_SCPH;
			break;
		}
		default:
		{
			return SPI_ERROR;
		}
	}

	/* Enable slave output function */
	if(spi_hw[spi->channel].spix != SPI_MASTER)
	{
		if(spi->transfer_mode == RX_ONLY)
		{
			/* Bit 10 - offset 0 : disable TX */
			control_0_reg |= (SPI_SLVOE);
		}
		else
		{
			/* Bit 10 - offset 0 : enable TX */
			control_0_reg &= ~(SPI_SLVOE);
		}
	}

	spi_hw[spi->channel].spix->CTRL0 = control_0_reg;
	spi_hw[spi->channel].spix->SSIENR = 1;

	/* Configure number of data frame Offset 0x04 - Control Register 1 */
	if(spi->channel == SSI_CH0
		&& (spi->transfer_mode == RX_ONLY
		|| spi->transfer_mode == EEPROM_READ))
	{
		/* Configure number of data frame */
		spi_hw[spi->channel].spix->CTRL1 = spi->master_data_frame;
	}

	return SPI_OK;
}

spi_status_t hal_spi_slave_select(uint8_t cs_pin, flag_status_t value)
{
	__IO uint32_t tmp = 0;

	if(cs_pin >= 1 && cs_pin <= 32)
	{
		tmp = spi_hw[SSI_CH0].spix->SER;
		tmp = (tmp & ~(1 << (cs_pin - 1))) | (value << (cs_pin - 1));
		spi_hw[SSI_CH0].spix->SER = tmp;
		return SPI_OK;
	}
	return SPI_ERROR;
}

void hal_spi_disable(spi_config_t *spi)
{
	/* check parameters */
	assert_param(IS_SSI_CHANNEL(spi->channel));
	spi_hw[spi->channel].spix->SSIENR = 0;
}

void hal_spi_enable(spi_config_t *spi)
{
	/* check parameters */
	assert_param(IS_SSI_CHANNEL(spi->channel));
	spi_hw[spi->channel].spix->SSIENR = 1;
}

void hal_spi_reset(spi_config_t *spi)
{
	/* check parameters */
	assert_param(IS_SSI_CHANNEL(spi->channel));
	spi_hw[spi->channel].spix->SSIENR = 0;
	spi_delay(0xff);
	spi_hw[spi->channel].spix->SSIENR = 1;
}

static spi_status_t hal_spi_receive_buf_dma(spi_config_t *spi, uint8_t *rx_buf,
											uint16_t len)
{
	dma_init_type_t dma_rx = { 0 };
	dma_config_t spi_dma;

	/* Check parameters */
	assert_param(IS_SSI_CHANNEL(spi->channel));
	assert_param(!(rx_buf == NULL));

	hal_spi_disable(spi);
	hal_dma_deinit(spi->dma_rx.chid);

	/* Configure spi dma */
	spi_dma.dma_receive_data_level = 0;
	spi_dma.dma_transmit_data_level = 0;
	spi_dma.dma_receive_enable = ENABLE;
	spi_dma.dma_transmit_enable = DISABLE;
	hal_spi_dma_init(spi, &spi_dma);
	hal_spi_init(spi);

	/* Configure DMA SPI RX */
	dma_rx.chid = spi->dma_rx.chid;
	dma_rx.length = len * (spi->data_size / 8);
	dma_rx.trans_type = PER_TO_MEM;
	dma_rx.src_addr = (uint32_t)&(spi_hw[spi->channel].spix->DR[0]);
	dma_rx.dst_addr = (uint32_t)rx_buf;
	dma_rx.src_bsize = BURST_SIZE_1;
	dma_rx.dst_bsize = BURST_SIZE_1;

	switch(spi->data_size)
	{
		case DATASIZE_8BIT:
		{
			dma_rx.src_wdata = DATA_WIDTH_8;
			dma_rx.dst_wdata = DATA_WIDTH_8;
			break;
		}
		case DATASIZE_16BIT:
		{
			dma_rx.src_wdata = DATA_WIDTH_16;
			dma_rx.dst_wdata = DATA_WIDTH_16;
			break;
		}
		case DATASIZE_32BIT:
		{
			dma_rx.src_wdata = DATA_WIDTH_32;
			dma_rx.dst_wdata = DATA_WIDTH_32;
			break;
		}
		default:
		{
			hal_spi_disable(spi);
			hal_dma_deinit(spi->dma_rx.chid);
			return SPI_ERROR;
		}
	}

	switch(spi->channel)
	{
		case SSI_CH0:
			dma_rx.periph_type = SPI0_DMA_RX;
			break;
		case SSI_CH1:
			dma_rx.periph_type = SPI1_DMA_RX;
			break;
		case SSI_CH2:
			dma_rx.periph_type = SPI2_DMA_RX;
			break;
		default:
		{
			hal_spi_disable(spi);
			hal_dma_deinit(spi->dma_rx.chid);
			return SPI_ERROR;
		}
	}

	dma_rx.clb_func = spi->dma_rx.callback_func;
	hal_dma_enable_global_interrupt(0);

	if(hal_dma_init(&dma_rx) != HAL_DMA_OK)
		return SPI_ERROR;

	flush_cache((unsigned long)rx_buf, len);
	if(hal_dma_channel_enable(spi->dma_rx.chid) != HAL_DMA_OK)
		return SPI_ERROR;

	return SPI_OK;
}

static spi_status_t hal_spi_receive_buf_normal(spi_config_t *spi,
												uint8_t *rx_buf, uint16_t len)
{
	volatile uint8_t status = 0;
	uint32_t time_out = 0;
	uint32_t rx_tmp = 0;
	uint16_t index_rx = 0;
	uint16_t buff_index = 0;

	/* Check parameters */
	assert_param(IS_SSI_CHANNEL(spi->channel));
	assert_param(!(rx_buf == NULL));

	if(spi_hw[spi->channel].spix->SSIENR == 0)
	{
		/* Enable SPI if which is disable */
		spi_hw[spi->channel].spix->SSIENR = 1;
	}

	time_out = TIME_OUT_RX;

	while(time_out)
	{
		time_out--;
		/* Read number of data in receiver FIFO */
		status = spi_hw[spi->channel].spix->RXFLR;
		if(status)
		{
			time_out = TIME_OUT_RX;
			/* Read data in RX FIFO */
			rx_tmp = spi_hw[spi->channel].spix->DR[0];

			switch(spi->data_size)
			{
				case DATASIZE_8BIT:
				{
					*(uint8_t *)(&rx_buf[index_rx]) = (uint8_t)rx_tmp;
					index_rx++;
					break;
				}
				case DATASIZE_16BIT:
				{
					*(uint16_t *)(&rx_buf[index_rx]) = (uint16_t)rx_tmp;
					index_rx += 2;
					break;
				}
				case DATASIZE_32BIT:
				{
					*(uint32_t *)(&rx_buf[index_rx]) = (uint32_t)rx_tmp;
					index_rx += 4;
					break;
				}
				default:
				{
					/* Disable SPI */
					spi_hw[spi->channel].spix->SSIENR = 0;
					return SPI_ERROR;
				}
			}
			buff_index++;

			if(buff_index == len)
			{
				/* Receive full data */
				break;
			}
		}
	}

	if(time_out != 0)
		return SPI_OK;
	else
		return SPI_ERROR;
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
spi_status_t hal_spi_receive_buf(spi_config_t *spi, uint8_t *rx_buf,
									uint16_t len)
{
	spi_status_t ret;
	if(spi->dma_rx.enable == ENABLE)
		ret = hal_spi_receive_buf_dma(spi, rx_buf, len);
	else
		ret = hal_spi_receive_buf_normal(spi, rx_buf, len);

	return ret;
}

static spi_status_t hal_spi_transmit_buf_dma(spi_config_t *spi, uint8_t *tx_buf,
												uint16_t len)
{
	dma_init_type_t dma_tx = { 0 };
	dma_config_t spi_dma;

	/* Check parameters */
	assert_param(IS_SSI_CHANNEL(spi->channel));
	assert_param(!(tx_buf == NULL));

	hal_spi_disable(spi);
	hal_dma_deinit(spi->dma_tx.chid);

	/* Configure spi dma */
	spi_dma.dma_receive_data_level = 0;
	spi_dma.dma_transmit_data_level = 0;
	spi_dma.dma_receive_enable = DISABLE;
	spi_dma.dma_transmit_enable = ENABLE;
	hal_spi_dma_init(spi, &spi_dma);
	hal_spi_init(spi);

	/* Configure DMA SPI TX */
	dma_tx.chid = spi->dma_tx.chid;
	dma_tx.length = len * (spi->data_size / 8);
	dma_tx.trans_type = MEM_TO_PER;
	dma_tx.src_addr = (uint32_t)tx_buf;
	dma_tx.dst_addr = (uint32_t)&(spi_hw[spi->channel].spix->DR[0]);
	dma_tx.src_bsize = BURST_SIZE_1;
	dma_tx.dst_bsize = BURST_SIZE_1;

	switch(spi->data_size)
	{
		case DATASIZE_8BIT:
		{
			dma_tx.src_wdata = DATA_WIDTH_8;
			dma_tx.dst_wdata = DATA_WIDTH_8;
			break;
		}
		case DATASIZE_16BIT:
		{
			dma_tx.src_wdata = DATA_WIDTH_16;
			dma_tx.dst_wdata = DATA_WIDTH_16;
			break;
		}
		case DATASIZE_32BIT:
		{
			dma_tx.src_wdata = DATA_WIDTH_32;
			dma_tx.dst_wdata = DATA_WIDTH_32;
			break;
		}
		default:
		{
			hal_spi_disable(spi);
			hal_dma_deinit(spi->dma_rx.chid);
			return SPI_ERROR;
		}
	}
	switch(spi->channel)
	{
		case SSI_CH0:
			dma_tx.periph_type = SPI0_DMA_TX;
			break;
		case SSI_CH1:
			dma_tx.periph_type = SPI1_DMA_TX;
			break;
		case SSI_CH2:
			dma_tx.periph_type = SPI2_DMA_TX;
			break;
		default:
		{
			hal_spi_disable(spi);
			hal_dma_deinit(spi->dma_rx.chid);
			return SPI_ERROR;
		}
	}

	dma_tx.clb_func = spi->dma_tx.callback_func;
	hal_dma_enable_global_interrupt(0);

	if(hal_dma_init(&dma_tx) != HAL_DMA_OK)
		return SPI_ERROR;

	flush_cache((unsigned long)tx_buf, len);
	if(hal_dma_channel_enable(spi->dma_tx.chid) != HAL_DMA_OK)
		return SPI_ERROR;

	return SPI_OK;
}

static spi_status_t hal_spi_transmit_buf_normal(spi_config_t *spi,
												uint8_t *tx_buf, uint16_t len)
{
	uint32_t time_out = 0;
	uint32_t i = 0;
	uint16_t index_tx = 0;
	uint8_t count = 0;
	uint32_t tx_tmp = 0;

	/* Check parameters */
	assert_param(IS_SSI_CHANNEL(spi->channel));
	assert_param(!(tx_buf == NULL));

	while(len > 0)
	{
		/* Check data length */
		if(len <= SPI_FIFO_DEPTH)
		{
			count = len;
			len = 0;
		}
		else
		{
			count = SPI_FIFO_DEPTH;
			len -= SPI_FIFO_DEPTH;
		}

		/*
		 * Transmit data full buffer
		 */
		for(i = 0; i < count; i++)
		{
			/* Format transmit data */
			switch(spi->data_size)
			{
				case DATASIZE_8BIT:
				{
					tx_tmp = *(uint8_t *)(&tx_buf[index_tx]);
					index_tx++;
					break;
				}
				case DATASIZE_16BIT:
				{
					tx_tmp = *(uint16_t *)(&tx_buf[index_tx]);
					index_tx += 2;
					break;
				}
				case DATASIZE_32BIT:
				{
					tx_tmp = *(uint32_t *)(&tx_buf[index_tx]);
					index_tx += 4;
					break;
				}
				default:
				{
					return SPI_ERROR;
				}
			}
			/* Move data need to send to FIFO */
			spi_hw[spi->channel].spix->DR[0] = tx_tmp;
		}

		time_out = TIME_OUT_TX * SPI_FIFO_DEPTH;
		while(spi_hw[spi->channel].spix->TXFLR != 0)
		{
			time_out--;
			if(time_out == 0)
				return SPI_ERROR;
		}
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
	spi_status_t ret;
	if(spi->dma_tx.enable == ENABLE)
		ret = hal_spi_transmit_buf_dma(spi, tx_buf, len);
	else
		ret = hal_spi_transmit_buf_normal(spi, tx_buf, len);

	return ret;
}

static spi_status_t hal_spi_transceiver_dma(spi_config_t *spi,
											spi_buffer_t *buff)
{
	uint16_t ret = SPI_ERROR;
	dma_init_type_t dma_rx = { 0 };
	dma_init_type_t dma_tx = { 0 };
	dma_config_t spi_dma;
	uint8_t *rx_buf, *tx_buf;
	uint16_t rx_len, tx_len;

	/* Check parameters */
	assert_param(IS_SPI_DATA_SIZE(spi->data_size));
	assert_param(IS_SPI_PHASE_MODE(spi->spi_mode));
	assert_param(IS_SPI_TYPE(spi->spi_type));
	assert_param(IS_SSI_MODE(spi->ssi_mode));
	assert_param(IS_SSI_CHANNEL(spi->channel));

	assert_param(!(buff->txbuf.buf == NULL  || buff->txbuf.len == 0 ||
					buff->rxbuf.buf == NULL || buff->rxbuf.len == 0 ||
					buff->rxbuf.len != buff->txbuf.len));

	rx_buf = buff->rxbuf.buf;
	rx_len = buff->rxbuf.len * (spi->data_size / 8);
	tx_buf = buff->txbuf.buf;
	tx_len = buff->txbuf.len * (spi->data_size / 8);

	hal_dma_deinit(spi->dma_rx.chid);
	hal_dma_deinit(spi->dma_tx.chid);
	hal_spi_disable(spi);

	/* Configure DMA for SPI */
	spi_dma.dma_receive_data_level = 0;
	spi_dma.dma_transmit_data_level = 0;
	spi_dma.dma_receive_enable = ENABLE;
	spi_dma.dma_transmit_enable = ENABLE;
	hal_spi_dma_init(spi, &spi_dma);

	/* Configure DMA SPI RX */
	dma_rx.chid = spi->dma_rx.chid;
	dma_rx.length = rx_len;
	dma_rx.trans_type = PER_TO_MEM;
	dma_rx.src_addr = (uint32_t)&(spi_hw[spi->channel].spix->DR[0]);
	dma_rx.dst_addr = (uint32_t)rx_buf;
	dma_rx.src_bsize = BURST_SIZE_1;
	dma_rx.dst_bsize = BURST_SIZE_1;
	switch(spi->data_size)
	{
		case DATASIZE_8BIT:
		{
			dma_rx.src_wdata = DATA_WIDTH_8;
			dma_rx.dst_wdata = DATA_WIDTH_8;
			break;
		}
		case DATASIZE_16BIT:
		{
			dma_rx.src_wdata = DATA_WIDTH_16;
			dma_rx.dst_wdata = DATA_WIDTH_16;
			break;
		}
		case DATASIZE_32BIT:
		{
			dma_rx.src_wdata = DATA_WIDTH_32;
			dma_rx.dst_wdata = DATA_WIDTH_32;
			break;
		}
		default:
		{
			hal_spi_disable(spi);
			hal_dma_deinit(spi->dma_rx.chid);
			return SPI_ERROR;
		}
	}
	switch(spi->channel)
	{
		case SSI_CH0:
			dma_rx.periph_type = SPI0_DMA_RX;
			break;
		case SSI_CH1:
			dma_rx.periph_type = SPI1_DMA_RX;
			break;
		case SSI_CH2:
			dma_rx.periph_type = SPI2_DMA_RX;
			break;
		default:
		{
			hal_spi_disable(spi);
			hal_dma_deinit(spi->dma_rx.chid);
			return SPI_ERROR;
		}
	}
	dma_rx.clb_func = spi->dma_rx.callback_func;

	/* Configure DMA SPI TX */
	dma_tx.chid = spi->dma_tx.chid;
	dma_tx.length = tx_len;
	dma_tx.trans_type = MEM_TO_PER;
	dma_tx.src_addr = (uint32_t)tx_buf;
	dma_tx.dst_addr = (uint32_t)&(spi_hw[spi->channel].spix->DR[0]);
	switch(spi->data_size)
	{
		case DATASIZE_8BIT:
		{
			dma_tx.src_wdata = DATA_WIDTH_8;
			dma_tx.dst_wdata = DATA_WIDTH_8;
			break;
		}
		case DATASIZE_16BIT:
		{
			dma_tx.src_wdata = DATA_WIDTH_16;
			dma_tx.dst_wdata = DATA_WIDTH_16;
			break;
		}
		case DATASIZE_32BIT:
		{
			dma_tx.src_wdata = DATA_WIDTH_32;
			dma_tx.dst_wdata = DATA_WIDTH_32;
			break;
		}
		default:
		{
			hal_spi_disable(spi);
			hal_dma_deinit(spi->dma_rx.chid);
			return SPI_ERROR;
		}
	}
	switch(spi->channel)
	{
		case SSI_CH0:
			dma_tx.periph_type = SPI0_DMA_TX;
			break;
		case SSI_CH1:
			dma_tx.periph_type = SPI1_DMA_TX;
			break;
		case SSI_CH2:
			dma_tx.periph_type = SPI2_DMA_TX;
			break;
		default:
		{
			hal_spi_disable(spi);
			hal_dma_deinit(spi->dma_rx.chid);
			return SPI_ERROR;
		}
	}
	dma_tx.clb_func = spi->dma_tx.callback_func;

	hal_dma_enable_global_interrupt(0);
	ret = hal_dma_init(&dma_rx);
	if(ret != HAL_DMA_OK)
		return SPI_ERROR;

	ret = hal_dma_init(&dma_tx);
	if(ret != HAL_DMA_OK)
		return SPI_ERROR;

	flush_cache((unsigned long)tx_buf, tx_len);
	flush_cache((unsigned long)rx_buf, rx_len);
	ret = hal_spi_init(spi);
	if(ret != SPI_OK)
		return SPI_ERROR;

	ret = hal_dma_channel_enable(spi->dma_rx.chid);
	if(ret != HAL_DMA_OK)
		return SPI_ERROR;

	ret = hal_dma_channel_enable(spi->dma_tx.chid);
	if(ret != HAL_DMA_OK)
		return SPI_ERROR;

	return SPI_OK;
}

static spi_status_t hal_spi_transceiver_normal(spi_config_t *spi,
												spi_buffer_t *buff)
{
	__IO uint32_t time_out = 0;
	uint32_t buff_size = 0;
	uint32_t tx_tmp = 0, rx_tmp = 0, i = 0;
	int idx_tx = 0, idx_rx = 0, count = 0;
	int rx_fifo = 0;

	/* Check parameters */
	assert_param(IS_SPI_DATA_SIZE(spi->data_size));
	assert_param(IS_SPI_PHASE_MODE(spi->spi_mode));
	assert_param(IS_SPI_TYPE(spi->spi_type));
	assert_param(IS_SSI_MODE(spi->ssi_mode));
	assert_param(IS_SSI_CHANNEL(spi->channel));

	assert_param(!(buff->txbuf.buf == NULL  || buff->txbuf.len == 0 ||
					buff->rxbuf.buf == NULL || buff->rxbuf.len == 0 ||
					buff->rxbuf.len != buff->txbuf.len));

	buff_size = buff->txbuf.len;
	buff->rxbuf.counter = 0;
	buff->txbuf.counter = 0;

	/* Enable SPI */
	spi_hw[spi->channel].spix->SSIENR = 1;

	while(buff_size != 0)
	{
		/* Check data length */
		if(buff_size <= SPI_FIFO_DEPTH)
		{
			count = buff_size;
			buff_size = 0;
		}
		else
		{
			count = SPI_FIFO_DEPTH;
			buff_size -= SPI_FIFO_DEPTH;
		}

		/*
		 * Transmit data full buffer
		 */
		for(i = 0; i < count; i++)
		{
			/* Format transmit data */
			switch(spi->data_size)
			{
				case DATASIZE_8BIT:
				{
					tx_tmp = *(uint8_t *)(&buff->txbuf.buf[idx_tx]);
					idx_tx++;
					break;
				}
				case DATASIZE_16BIT:
				{
					tx_tmp = *(uint16_t *)(&buff->txbuf.buf[idx_tx]);
					idx_tx += 2;
					break;
				}
				case DATASIZE_32BIT:
				{
					tx_tmp = *(uint32_t *)(&buff->txbuf.buf[idx_tx]);
					idx_tx += 4;
					break;
				}
				default:
				{
					spi_hw[spi->channel].spix->SSIENR = 0;
					return SPI_ERROR;
				}
			}
			/* Move data need to send to FIFO */
			spi_hw[spi->channel].spix->DR[0] = tx_tmp;
			buff->txbuf.counter = idx_tx;
		}

		if(spi->channel == SSI_CH0)
		{
			/* Read all datas from Rx FIFO */
			rx_fifo = wait_tx_fifo_empty(spi);
		}
		else
		{
			/* Wait for SPI slave transmission complete all data */
			time_out = TIME_OUT_TX * SPI_FIFO_DEPTH;
			while(spi_hw[spi->channel].spix->RXFLR != count)
			{
				time_out--;
				if(time_out == 0)
				{
					spi_hw[spi->channel].spix->SSIENR = 0;
					return SPI_ERROR;
				}
			}
			rx_fifo = spi_hw[spi->channel].spix->RXFLR;
		}

		while(rx_fifo > 0)
		{
			rx_tmp = spi_hw[spi->channel].spix->DR[0];

			switch(spi->data_size)
			{
				case DATASIZE_8BIT:
				{
					*(uint8_t *)(&buff->rxbuf.buf[idx_rx]) = (uint8_t)rx_tmp;
					idx_rx++;
					break;
				}
				case DATASIZE_16BIT:
				{
					*(uint16_t *)(&buff->rxbuf.buf[idx_rx]) = (uint16_t)rx_tmp;
					idx_rx += 2;
					break;
				}
				case DATASIZE_32BIT:
				{
					*(uint32_t *)(&buff->rxbuf.buf[idx_rx]) = (uint32_t)rx_tmp;
					idx_rx += 4;
					break;
				}
				default:
				{
					/* Disable SPI */
					spi_hw[spi->channel].spix->SSIENR = 0;
					return SPI_ERROR;
				}
			}
			buff->rxbuf.counter = idx_rx;
			rx_fifo--;
		}
	}
	/* Disable SPI */
	spi_hw[spi->channel].spix->SSIENR = 0;
	return SPI_OK;
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
	spi_status_t ret;
	if(spi->dma_rx.enable == ENABLE && spi->dma_tx.enable == ENABLE)
		ret = hal_spi_transceiver_dma(spi, buff);
	else
		ret = hal_spi_transceiver_normal(spi, buff);

	return ret;
}

void hal_spi_dma_init(spi_config_t *spi, dma_config_t *dma)
{
	uint32_t reg_tmp = 0;

	/* Check parameters */
	assert_param(IS_SPI_DATA_SIZE(spi->data_size));
	assert_param(IS_SPI_PHASE_MODE(spi->spi_mode));
	assert_param(IS_SPI_TRANSFER_MODE(spi->transfer_mode));
	assert_param(IS_SPI_TYPE(spi->spi_type));
	assert_param(IS_SSI_MODE(spi->ssi_mode));
	assert_param(IS_SSI_CHANNEL(spi->channel));
	assert_param(IS_STATUS(dma->dma_receive_enable));
	assert_param(IS_STATUS(dma->dma_transmit_enable));

	/* Configure DMA transmit data level */
	spi_hw[spi->channel].spix->DMATDLR = dma->dma_transmit_data_level;
	/* Configure DMA receive / receive data level */
	spi_hw[spi->channel].spix->DMARDLR = dma->dma_transmit_data_level;
	/* Enable/Disable transmit DMA enable bit */
	reg_tmp = ((uint32_t)dma->dma_transmit_enable << 1) |
			((uint32_t)dma->dma_receive_enable << 0);

	/* Initializing DMA controller */
	spi_hw[spi->channel].spix->DMACR = reg_tmp;
}

void hal_spi_interrupt_init(spi_config_t *spi, spi_interrupt_init_t *interrupt)
{
	uint32_t reg_tmp = 0;

	/* Check parameters */
	assert_param(IS_SPI_DATA_SIZE(spi->data_size));
	assert_param(IS_SPI_PHASE_MODE(spi->spi_mode));
	assert_param(IS_SPI_TRANSFER_MODE(spi->transfer_mode));
	assert_param(IS_SPI_TYPE(spi->spi_type));
	assert_param(IS_SSI_MODE(spi->ssi_mode));
	assert_param(IS_SSI_CHANNEL(spi->channel));

	/* Check interrupt parameters */
	assert_param(IS_STATUS(interrupt->rx_fifo_full));
	assert_param(IS_STATUS(interrupt->rx_fifo_overflow));
	assert_param(IS_STATUS(interrupt->rx_fifo_underflow));
	assert_param(IS_STATUS(interrupt->tx_fifo_empty));
	assert_param(IS_STATUS(interrupt->tx_fifo_overflow));
	assert_param(IS_STATUS(interrupt->multi_master_contention));
	assert_param(!(interrupt->rx_fifo_threshold > 31));
	assert_param(!(interrupt->tx_fifo_threshold > 31));


	/* Configure transmit FIFO threshold */
	spi_hw[spi->channel].spix->TXFTLR = interrupt->tx_fifo_threshold;

	/* Configure receive FIFO threshold */
	spi_hw[spi->channel].spix->RXFTLR = interrupt->rx_fifo_threshold;

	/* Configure interrupts */
	reg_tmp = (interrupt->multi_master_contention << SPI_INT_MSTI_OFFSET)	|
				(interrupt->rx_fifo_full << SPI_INT_RXFI_OFFSET)			|
				(interrupt->rx_fifo_overflow << SPI_INT_RXOI_OFFSET)		|
				(interrupt->rx_fifo_underflow << SPI_INT_RXUI_OFFSET)		|
				(interrupt->tx_fifo_overflow << SPI_INT_TXOI_OFFSET)		|
				(interrupt->tx_fifo_empty << SPI_INT_TXEI_OFFSET);

	spi_interrupt_enable[spi->channel] = ENABLE;

	/* Copy interrupt function pointer to global variable */
	spi_interrupt[spi->channel].multi_master_handler =
			interrupt->callback_handler.multi_master_handler;

	spi_interrupt[spi->channel].rx_fifo_full_handler =
			interrupt->callback_handler.rx_fifo_full_handler;

	spi_interrupt[spi->channel].rx_fifo_overflow_handler =
			interrupt->callback_handler.rx_fifo_overflow_handler;

	spi_interrupt[spi->channel].rx_fifo_underflow_handler =
			interrupt->callback_handler.rx_fifo_underflow_handler;

	spi_interrupt[spi->channel].tx_fifo_empty_handler =
			interrupt->callback_handler.tx_fifo_empty_handler;

	spi_interrupt[spi->channel].tx_fifo_overflow_handler =
			interrupt->callback_handler.tx_fifo_overflow_handler;

	switch(spi->channel)
	{
		case SSI_CH0:
			vic_register_irq(SPI_IRQn, hal_spi_0_irq);
			break;
		case SSI_CH1:
			vic_register_irq(SPIS0_IRQn, hal_spi_1_irq);
			break;
		case SSI_CH2:
			vic_register_irq(SPIS1_IRQn, hal_spi_2_irq);
			break;
		default:
			break;
	}

	/* Init interrupt */
	spi_hw[spi->channel].spix->IMR = reg_tmp;
}

void hal_spi_interrupt_deinit(spi_config_t *spi)
{
	/* check parameters */
	assert_param(IS_SPI_DATA_SIZE(spi->data_size));
	assert_param(IS_SPI_PHASE_MODE(spi->spi_mode));
	assert_param(IS_SPI_TRANSFER_MODE(spi->transfer_mode));
	assert_param(IS_SPI_TYPE(spi->spi_type));
	assert_param(IS_SSI_MODE(spi->ssi_mode));
	assert_param(IS_SSI_CHANNEL(spi->channel));

	/* Disable all interrupt */
	spi_hw[spi->channel].spix->IMR = 0;
	spi_hw[spi->channel].spix->TXFTLR = 0;
	spi_hw[spi->channel].spix->RXFTLR = 0;
	spi_interrupt_enable[spi->channel] = DISABLE;
	spi_interrupt[spi->channel].multi_master_handler = NULL;
	spi_interrupt[spi->channel].rx_fifo_full_handler = NULL;
	spi_interrupt[spi->channel].rx_fifo_overflow_handler = NULL;
	spi_interrupt[spi->channel].rx_fifo_underflow_handler = NULL;
	spi_interrupt[spi->channel].tx_fifo_empty_handler = NULL;
	spi_interrupt[spi->channel].tx_fifo_overflow_handler = NULL;

	switch(spi->channel)
	{
		case SSI_CH0:
			vic_unregister_irq(SPI_IRQn);
			break;
		case SSI_CH1:
			vic_unregister_irq(SPIS0_IRQn);
			break;
		case SSI_CH2:
			vic_unregister_irq(SPIS1_IRQn);
			break;
		default:
			break;
	}
}

static void hal_spi_0_irq(void)
{
	hal_spi_irq_handler(SSI_CH0);
}

static void hal_spi_1_irq(void)
{
	hal_spi_irq_handler(SSI_CH1);
}

static void hal_spi_2_irq(void)
{
	hal_spi_irq_handler(SSI_CH2);
}

static void hal_spi_irq_handler(ssi_channel_t channel)
{
	uint32_t tmp = 0;

	/* 1. Check multi-master interrupt */
	if(spi_hw[channel].spix->ISR & SPI_INT_MSTI)
	{
		/* Read multi-master interrupt clear register */
		tmp = spi_hw[channel].spix->MSTICR;
		if(spi_interrupt[channel].multi_master_handler != NULL)
			spi_interrupt[channel].multi_master_handler();
	}
	/* 2. Check receive FIFO full interrupt */
	if(spi_hw[channel].spix->ISR & SPI_INT_RXFI)
	{
		if(spi_interrupt[channel].rx_fifo_full_handler != NULL)
			spi_interrupt[channel].rx_fifo_full_handler();
	}
	/* 3. Check receive FIFO overflow interrupt */
	if(spi_hw[channel].spix->ISR & SPI_INT_RXOI)
	{
		/* Read RXOICR to clear SPI_INT_RXOI interrupt flag */
		tmp = spi_hw[channel].spix->RXOICR;
		if(spi_interrupt[channel].rx_fifo_overflow_handler != NULL)
			spi_interrupt[channel].rx_fifo_overflow_handler();
	}
	/* 4. Check receive FIFO underflow interrupt */
	if(spi_hw[channel].spix->ISR & SPI_INT_RXUI)
	{
		/* Read RXUICR to clear SPI_INT_RXUI interrupt flag */
		tmp = spi_hw[channel].spix->RXUICR;
		if(spi_interrupt[channel].rx_fifo_underflow_handler != NULL)
			spi_interrupt[channel].rx_fifo_underflow_handler();
	}
	/* 5. Check transmit FIFO overflow interrupt */
	if(spi_hw[channel].spix->ISR & SPI_INT_TXOI)
	{
		/* Read TXOICR to clear SPI_INT_TXOI interrupt flag */
		tmp = spi_hw[channel].spix->TXOICR;
		if(spi_interrupt[channel].tx_fifo_overflow_handler != NULL)
			spi_interrupt[channel].tx_fifo_overflow_handler();
	}
	/* 6. Check transmit FIFO empty interrupt */
	if(spi_hw[channel].spix->ISR & SPI_INT_TXEI)
	{
		if(spi_interrupt[channel].tx_fifo_empty_handler != NULL)
			spi_interrupt[channel].tx_fifo_empty_handler();
	}
	/* Read to clear all interrupt flag */
	tmp |= spi_hw[channel].spix->ICR;
}

/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
