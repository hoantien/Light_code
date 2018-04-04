/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification,are strictly prohibited without prior permission of The LightCo.
 *
 * @file    spi_slave.c
 * @author  The LightCo
 * @version V1.0.0
 * @date    Feb-04-2016
 * @brief   This file contains expand for spi_slave
 *
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "os.h"
#include "lcc_system.h"
#include "spis.h"


/* Private define------------------------------------------------------------*/
#define SLOGF_ID					SLOG_ID_SPI_SLAVE

/*
 * spi_slave_sm typedef
 */
typedef enum spi_slave_sm
{
	SM_IDLE,
	SM_RX,
	SM_TX,
} spi_slave_sm_t;

/*
 * spi_slave_buf typedef
 */
typedef struct spi_slave_buf
{
	uint8_t *data;
	uint16_t len;
} spi_slave_buf_t;

/* Privated functions --------------------------------------------------------*/

/* Interrupt call back function */
static void spi_slaver_rx_complete(void);
static void spi_slaver_tx_complete(void);
/* Privated variable ---------------------------------------------------------*/
static spi_config_t spi_slv_hw_cfg =
{
	.channel			= SSI_CH1,
	.data_size			= SPI_SLAVE_SPI_DATASIZE,
	.spi_mode			= SPI_SLAVE_SPI_DATA_MODE,
	.spi_type			= SPI_STD,
	.ssi_mode			= SSI_MOTO_SPI,
	.transfer_mode		= TX_AND_RX,
	.master_clk_freq	= SPI_SLAVE_SPI_CLOCK,
	.master_data_frame	= 0	/* only valid for SPI master */
};

static spi_interrupt_init_t spi_slv_isr_cfg =
{
	/* Configure interrupt t */
	.rx_fifo_threshold	= 16 - 1,	/* RX FIFO threshold = 16 (max buff size)*/
	.tx_fifo_threshold	= 1 - 1,	/* TX FIFO threshold = 1 */
	.rx_fifo_full 		= DISABLE,
	.tx_fifo_empty		= DISABLE,
	.rx_fifo_overflow	= DISABLE,
	.rx_fifo_underflow	= DISABLE,
	.tx_fifo_overflow	= DISABLE,
	.multi_master_contention = DISABLE,

	/* Point to call back function */
	.callback_handler.rx_fifo_full_handler		= spi_slaver_rx_complete,
	.callback_handler.tx_fifo_empty_handler		= spi_slaver_tx_complete,
	.callback_handler.rx_fifo_overflow_handler	= NULL,
	.callback_handler.rx_fifo_underflow_handler	= NULL,
	.callback_handler.tx_fifo_overflow_handler	= NULL,
	.callback_handler.multi_master_handler		= NULL
};

static spi_slave_sm_t sm = SM_IDLE;
static void (*rx_clbk)(uint8_t *data, uint16_t len);

static uint8_t tx_data[SPI_SLAVE_MAX_RX_BUFFER_SIZE] = { 0 };
static uint8_t tx_len = SPI_SLAVE_MAX_RX_BUFFER_SIZE;

static uint8_t rx_data[SPI_SLAVE_MAX_RX_BUFFER_SIZE] = { 0 };
static uint8_t rx_len = SPI_SLAVE_MAX_RX_BUFFER_SIZE;

xSemaphoreHandle spi_slave_semaphore;

/* Private functions ---------- ----------------------------------------------*/

static void spi_slaver_rx_complete(void)
{
	/* Clear buffer */
	memset(rx_data, 0x00, SPI_SLAVE_MAX_RX_BUFFER_SIZE);
	rx_len = SPI_SLAVE_MAX_RX_BUFFER_SIZE;

	/* Read data from FIFO to buffer */
	hal_spi_receive_buf(&spi_slv_hw_cfg, rx_data, rx_len);

	/* Call back to update data to upper layer */
	(*rx_clbk)(rx_data, rx_len);
}

static void spi_slaver_tx_complete(void)
{
	/* If TX FIFO = 0 is transmission completed */
	if(SPI_SLAVE0->TXFLR == 0)
	{
		/* Disable transmission interrupt */
		spi_slv_isr_cfg.tx_fifo_empty = DISABLE;
		hal_spi_interrupt_init(&spi_slv_hw_cfg, &spi_slv_isr_cfg);

		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		sm = SM_RX;
		xSemaphoreGiveFromISR(spi_slave_semaphore, &xHigherPriorityTaskWoken);
		if(xHigherPriorityTaskWoken)
		{
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
	}
}

/* Exported functions ------------------------------------------------------- */
/*
 * task_spi_slave
 *
 */
void task_spi_slave(void *vParameter)
{
	task_handle_t *hdl = (task_handle_t *)(vParameter);
	uint8_t EXIT = 0;
	/*
	 * TODO: remove EXIT = 1 line
	 * when have implemented tasks lcccmd, console, slog
	 */
	EXIT = 1;

	while(!EXIT)
	{
		if(TASK_READY == task_handler[task_query_tid("lcccmd")].state
				&& TASK_READY == task_handler[task_query_tid("console")].state
				&& TASK_READY == task_handler[task_query_tid("slog")].state)
		{
			/* break out while loop */
			EXIT = 1;
		}
		vTaskDelay(1);
	}

	/* Enable SPI receiver interrupt */
	spi_slv_isr_cfg.rx_fifo_full = ENABLE;
	hal_spi_interrupt_init(&spi_slv_hw_cfg, &spi_slv_isr_cfg);
	hal_spi_reset(&spi_slv_hw_cfg);
	/* Task ready id */
	hdl->state = TASK_READY;
	/* Get task id */
	sm = SM_RX;
	vSemaphoreCreateBinary(spi_slave_semaphore);
	taskENTER_CRITICAL();
	/* Log_msg("Start %s\r\n", __FUNCTION__); */
	taskEXIT_CRITICAL();
	while(1)
	{
		/* Waiting for semaphore to unlock task*/
		xSemaphoreTake(spi_slave_semaphore, portMAX_DELAY);

		/* Transmit data */
		if(SM_TX == sm)
		{
			/* Put data to FIFO */
			if(spi_slv_isr_cfg.tx_fifo_empty == DISABLE)
			{
				hal_spi_transmit_buf(&spi_slv_hw_cfg, tx_data,
						tx_len);
				/* Enable transmission interrupt */
				spi_slv_isr_cfg.tx_fifo_empty = ENABLE;
				hal_spi_interrupt_init(&spi_slv_hw_cfg, &spi_slv_isr_cfg);
			}
		}
	}
}
/*
 * spi slave init
 *
 */
void spi_slave_init(spi_slave_t *spi_slave)
{
	/* Receive call back function */
	rx_clbk = spi_slave->rx_clbk;

	/* Configure data buffer */
	tx_len = SPI_SLAVE_MAX_RX_BUFFER_SIZE;
	rx_len = SPI_SLAVE_MAX_RX_BUFFER_SIZE;

	/* Initialize SPI and Interrupt */
	hal_spi_init(&spi_slv_hw_cfg);
	hal_spi_interrupt_deinit(&spi_slv_hw_cfg);
	hal_spi_interrupt_init(&spi_slv_hw_cfg, &spi_slv_isr_cfg);
}

/**
 * spi_slave_tx
 *
 */
void spi_slave_tx(uint8_t *data, uint16_t len, uint8_t tx_en)
{
	if(data)
	{
		memcpy(tx_data, data, len);
		tx_len = len;
		if(tx_en == FALSE)
		{
			/* Received mode */
			sm = SM_RX;
		}
		else
		{
			/* Transmission mode */
			sm = SM_TX;
		}
		xSemaphoreGive(spi_slave_semaphore);
	}
}

void spi_slave_release_semaphore(void)
{
	sm = SM_RX;
	xSemaphoreGive(spi_slave_semaphore);
}
/*********** Portions COPYRIGHT 2016 Light.Co., Ltd.*****END OF FILE***********/
