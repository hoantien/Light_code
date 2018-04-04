/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    i2c_slave.c
 * @author  The LightCo
 * @version V1.0.0
 * @date    April-5-2016
 * @brief   This file contains I2C Slave source code
 *
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "os.h"
#include "lcc_system.h"
#include "assert.h"
#include "i2cs.h"

/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private functions prototype -----------------------------------------------*/
/* I2C handler function implemented here */
/* i2c event handler */
static void i2c_handler(hal_i2c_channel_t chid, hal_i2c_status_t status);

/* Private variable ---------------------------------------------------------*/
/* I2C configuration */
static hal_i2c_t i2c =
{
#if   (ASIC_NUM == ASIC1)
	.chid = 			I2C_SLAVE_HW_CHANNEL,
	.clock_speed =		I2C_SPEED_400KHz,
#else
	.chid = 			I2C_CH9,
	.clock_speed =		I2C_SPEED_1MHz,
#endif

	.address_mode =		I2C_7BIT,
	.operation_mode = 	I2C_SLAVE,
	.owner_addr = 		I2C_SLAVE_ADDRESS,
	.irq_handler = 		i2c_handler,
};

/* I2C configuration */
static hal_i2c_buffer_t tx_buf;
static SemaphoreHandle_t sem_tx_data_ready;
static SemaphoreHandle_t sem_tx_data_pending;

static void  (*rx_hdl)(uint8_t data);
static void  (*receiver_hdl)(void);
static void  (*transmitter_hdl)(void);
static void  (*restart_hdl)(void);
/* Exported functions --------------------------------------------------------*/
/**
 * @brief i2c_slave_init
 * The function shall initialize i2c slave driver
 * @param event - point to event flag at upper layer
 * @return None
 */
void i2c_slave_init(i2c_slave_t *i2c_slave)
{
	/* Store rx call back function */
	rx_hdl = i2c_slave->clbk_hdl;
	receiver_hdl = i2c_slave->receiver_hdl;
	transmitter_hdl = i2c_slave->transmitter_hdl;
	restart_hdl = i2c_slave->restart_hdl;
	/* Init i2c */
	hal_i2c_init(&i2c);
	hal_i2c_enable_irq(&i2c);

	tx_buf.bytes = NULL;
	tx_buf.length = 0;
	sem_tx_data_ready = xSemaphoreCreateBinary();
	sem_tx_data_pending = xSemaphoreCreateBinary();
}

/**
 * @brief i2c_slave_write
 * The function writes data to i2c slave
 * @param buf - point to buffer write
 * @param len - data length to write
 * @return None
 */
void i2c_slave_write(uint8_t *buf, uint16_t len)
{
    assert((buf != NULL) && (len > 0));
    // Free previous buffer
    vPortFree(tx_buf.bytes);

    tx_buf.bytes = buf;
    tx_buf.length = len;
    // Notify interrupt handler that data is ready
	xSemaphoreGive(sem_tx_data_ready);

	if (xSemaphoreTake(sem_tx_data_pending, 0) == pdTRUE)
	{
	    // Interrupt handler run before data was ready, start transmission here
	    xSemaphoreTake(sem_tx_data_ready, 0);
	    hal_i2c_slave_tx(i2c.chid, &tx_buf);
	}
}

/*
 * i2c_slave_evt_handler
 */
static void i2c_handler(hal_i2c_channel_t chid, hal_i2c_status_t status)
{
	/* Preparing buffer to receive */
	switch(status)
	{
		case I2C_READ_REQUESTED:
			(*restart_hdl)();
			if (xSemaphoreTakeFromISR(sem_tx_data_ready, NULL))
			{
                assert((tx_buf.length > 0) && (tx_buf.bytes != NULL));
                hal_i2c_slave_tx(i2c.chid, &tx_buf);
			}
			else
			{
			    // Data still no ready, notify task that transmission is pending
			    xSemaphoreGiveFromISR(sem_tx_data_pending, NULL);
			}
			break;
		case I2C_RX_RECEIVING:
		    {
		        hal_i2c_buffer_t buf;
		        uint8_t data = 0xFF;
		        buf.bytes = &data;
		        buf.length = 1;
                hal_i2c_slave_rx(i2c.chid, &buf);
                /* Transfer to upper layer */
                (*rx_hdl)(data);
		    }
			break;
		case I2C_RX_COMPLETED:
			(*receiver_hdl)();
			break;
		case I2C_RESTARTED:
			break;
		case I2C_ERROR:
			break;
		default:
		    break;
	}
}

/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
