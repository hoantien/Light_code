/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    i2cm.c
 * @author  The LightCo
 * @version V1.0.0
 * @date    May-29-2016
 * @brief   This file contains expand for I2C master driver
 *
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "os.h"
#include "assert.h"
#include "lcc_system.h"
#include "log.h"
#include "hal_i2c.h"
#include "hal_vic.h"
#include "i2cm.h"

/* Private define-------------------------------------------------------------*/
#define SLOGF_ID					SLOG_ID_I2C_MASTER
#define I2CM_TIMEOUT				1000

/* Private typedef------------------------------------------------------------*/
typedef struct
{
	SemaphoreHandle_t sem_trans;
	SemaphoreHandle_t sem_msg;
	i2cm_error_t status;
	hal_i2c_buffer_t txbuf;
	hal_i2c_buffer_t rxbuf;
	uint8_t addr;
	uint8_t is_initialized;
} i2cm_t;

/* Private macro--------------------------------------------------------------*/
/* Private function prototype ------------------------------------------------*/
static void i2cm_handler(hal_i2c_channel_t chid, hal_i2c_status_t status);

/* Private variables----------------------------------------------------------*/
static i2cm_t i2cm_tbl[I2CM_CH_MAX_IDX];

/* Exported functions implementation -----------------------------------------*/
/*
 * i2cm_init
 */
i2cm_error_t i2cm_init(i2c_t chid)
{
	i2cm_t *i2cm = &i2cm_tbl[chid];
	hal_i2c_t i2cm_cfg;
	i2cm_error_t ret = I2CM_ERROR_NONE;

	if (i2cm->is_initialized == pdFALSE) /* Un-initialization */
	{
		/* Create semaphore to protect resource while using transceiver */
		i2cm->sem_trans = xSemaphoreCreateMutex();
		assert_param(i2cm->sem_trans);
		/* Create semaphore for waiting transfer completion */
		i2cm->sem_msg = xSemaphoreCreateBinary();
		assert_param(i2cm->sem_msg);

		/* Setup i2c hardware and interrupt */
		i2cm_cfg.chid = chid;
#ifdef EVT3_REWORK
		if ((I2C_FORWARD_ASIC2_CHANNEL == chid) ||
			(I2C_FORWARD_ASIC3_CHANNEL == chid))
		{
			i2cm_cfg.clock_speed = I2C_SPEED_1MHz;
		}
		else
		{
			i2cm_cfg.clock_speed = I2C_SPEED_400KHz;
		}
#else
		i2cm_cfg.clock_speed = I2C_SPEED_100KHz;
#endif
		i2cm_cfg.address_mode = I2C_7BIT;
		i2cm_cfg.operation_mode = I2C_MASTER;
		i2cm_cfg.owner_addr = 0x00;
		i2cm_cfg.irq_handler = i2cm_handler;

		hal_i2c_init(&i2cm_cfg);
		hal_i2c_enable_irq(&i2cm_cfg);

		i2cm->is_initialized = pdTRUE;
	}

	return ret;
}

/*
 * i2cm_deinit
 */
i2cm_error_t i2cm_deinit(i2c_t chid)
{
	i2cm_t *i2cm = &i2cm_tbl[chid];

	if (i2cm->is_initialized == pdTRUE) /* Initialized */
	{
		hal_i2c_deinit(chid);
		vSemaphoreDelete(i2cm->sem_trans);
		vSemaphoreDelete(i2cm->sem_msg);

		i2cm->is_initialized = pdFALSE;
		return I2CM_ERROR_NONE;
	}

	return I2CM_ERROR_INVALID;
}

/*
 * i2cm_transceiver
 */
i2cm_error_t i2cm_transceiver(i2c_t chid, uint8_t addr,
								uint8_t *tx, uint16_t txlen,
								uint8_t *rx, uint16_t rxlen)
{
	i2cm_t *i2cm = &i2cm_tbl[chid];

	assert_param((txlen != 0) || (rxlen != 0));		/* Have not data */
	assert_param((txlen == 0) || (tx != NULL));		/* Check TX buffer */
	assert_param((rxlen == 0) || (rx != NULL));		/* Check RX buffer */

	if (i2cm->is_initialized == pdFALSE)
		return I2CM_ERROR_UNINITIALIZED;

	/* Wait for the completion of current transfer */
	if (xSemaphoreTake(i2cm->sem_trans, I2CM_TIMEOUT) == pdPASS)
	{
		volatile i2cm_error_t status;

		/* Setup message */
		i2cm->addr = addr;
		i2cm->rxbuf.length = rxlen;
		i2cm->rxbuf.bytes = rx;
		i2cm->txbuf.length = txlen;
		i2cm->txbuf.bytes = tx;

		if (txlen > 0)	/* Send I2C message to slave*/
			hal_i2c_master_tx(chid, i2cm->addr, &i2cm->txbuf);
		else			/* Get I2C message from slave */
			hal_i2c_master_rx(chid, i2cm->addr, &i2cm->rxbuf);

		/* Make sure that the current transfer must be done */
		xSemaphoreTake(i2cm->sem_msg, portMAX_DELAY);
		/* Store the transfer status temporarily for reentrancy */
		status = i2cm->status;
		/* Release resource for other transfers */
		xSemaphoreGive(i2cm->sem_trans);
		/* Return transfer status */
		return status;
	}
	else
		return I2CM_ERROR_BUSY;
}

/*
 * i2cm_read
 */
i2cm_error_t i2cm_read(i2c_t i2c, i2cm_mode_t mode,
				uint8_t sla, uint16_t reg, uint8_t *byte)
{
	uint8_t addr[2];
	uint8_t addr_len = 0;
	uint8_t data_len = 1;

	switch (mode)
	{
		case WORD_ADDR16:
		case BYTE_ADDR16:
			addr[addr_len++] = (uint8_t)(reg >> 8);
		case WORD_ADDR8:
		case BYTE_ADDR8:
		default:
			addr[addr_len++] = (uint8_t)reg;
			if ((mode == WORD_ADDR8) || (mode == WORD_ADDR16))
				data_len = 2;
			break;
	}
	return i2cm_transceiver(i2c, sla, addr, addr_len, byte, data_len);
}

/*
 * i2cm_write
 */
i2cm_error_t i2cm_write(i2c_t i2c, i2cm_mode_t mode,
				uint8_t sla, uint16_t reg, uint8_t *byte)
{
	uint8_t buf[4];
	uint8_t len = 0;

	switch (mode)
	{
		case WORD_ADDR16:
			buf[len++] = (uint8_t)(reg >> 8);
		case WORD_ADDR8:
			buf[len++] = (uint8_t)reg;
			buf[len++] = *byte++;
			buf[len++] = *byte;
			break;

		case BYTE_ADDR16:
			buf[len++] = (uint8_t)(reg >> 8);
		case BYTE_ADDR8:
		default:
			buf[len++] = (uint8_t)reg;
			buf[len++] = *byte++;
			break;
	}
	return i2cm_transceiver(i2c, sla, buf, len, NULL, 0);
}

/* Private functions implementation ------------------------------------------*/
/*
 * i2cm_handler
 */

static void i2cm_handler(hal_i2c_channel_t chid, hal_i2c_status_t status)
{
	i2cm_t *i2cm = &i2cm_tbl[chid];
	bool completed = false;

	switch (status)
	{
		case I2C_TX_COMPLETED:
			if (i2cm->rxbuf.length)
			{
				/* Trigger receive data from slave */
				hal_i2c_master_rx(chid, i2cm->addr, &i2cm->rxbuf);
			}
			else
			{
				i2cm->status = I2CM_ERROR_TRANSMITTED;
				/* Completed current transaction */
				completed = true;
			}
			break;

		case I2C_RX_COMPLETED:
			if (i2cm->txbuf.length)
				i2cm->status = I2CM_ERROR_TRANSCEIVED;
			else
				i2cm->status = I2CM_ERROR_RECEIVED;
			/* Completed current transaction */
			completed = true;
			break;

		case I2C_ERROR:
			i2cm->status = I2CM_ERROR_BUSY;
			/* Completed current transaction */
			completed = true;
			break;

		default:
			break;
	}

	if (completed)
	{
		BaseType_t higher_pri_task_woken = pdFALSE;
		xSemaphoreGiveFromISR(i2cm->sem_msg, &higher_pri_task_woken);
		portEND_SWITCHING_ISR(higher_pri_task_woken);
	}
}

/* Exported variables --------------------------------------------------------*/
/*
 * i2cm_dev
 */
i2cm_dev_t i2cm =
{
	.init =			i2cm_init,
	.deinit =		i2cm_deinit,
	.transceiver =	i2cm_transceiver,
	.read =			i2cm_read,
	.write =		i2cm_write
};
/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
