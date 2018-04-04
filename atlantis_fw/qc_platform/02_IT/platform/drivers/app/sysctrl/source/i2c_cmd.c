/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    i2c_cmd.c
 * @author  The LightCo.
 * @version 1.0.0
 * @date    June 20, 2016
 * @brief   
 *
 ******************************************************************************/

/******************************************************************************/
/**  						Revision history
 *
 * * 1.0.0	June 20, 2016	Initial revision
 */
/******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "os.h"
#include "log.h"
#include "sys_cfg.h"
#include "queue.h"
#include "board_config.h"
#include "i2c_slave.h"
#include "i2c_cmd.h"
/* Private define ------------------------------------------------------------*/
#define _PAYLOAD_LENGTH_MAX 256
/* Private typedef -----------------------------------------------------------*/
typedef enum
{
	I2C_DATA_PAYLOAD_BODY = 0,  //! Data byte is normal byte
	I2C_DATA_PAYLOAD_END    = 1,  //! Data byte is an end of command
}i2c_data_ctrl_t;

typedef struct
{
	i2c_data_ctrl_t type;
	uint16_t        index;
	uint8_t         value;
}i2c_data_t;

/* Private macro -------------------------------------------------------------*/
#if (ASIC_NUM == 1)
	#define I2C_RECEPTIONISTS_NUM    1
	#define I2C_RECEPTINIST_ADDR  0x08
#elif (ASIC_NUM == 2) || (ASIC_NUM == 3)
	#define I2C_RECEPTIONISTS_NUM    1
	#define I2C_RECEPTINIST_ADDR  0x08
#else
	#define I2C_RECEPTIONISTS_NUM    1
	#define I2C_RECEPTINIST_ADDR  0x08
#endif
/* Static functions prototype-------------------------------------------------*/
/**
 * @brief Callback handler for upper layer
 * @param[in] 	NA
 * @param[out] 	NA
 * @return 		NA
 * @details Callback handler for upper layer
 */
LOCAL void _recept_callback_handler(uint8_t byte);
/**
 * @brief Receiving completed handler
 * @param[in] 	NA
 * @param[out] 	NA
 * @return 		NA
 * @details Receiving completed handler
 */
LOCAL void _stop_handler(void);
/**
 * @brief Handler for restart request on bus
 * @param[in] 	NA
 * @param[out] 	NA
 * @return 		NA
 * @details Handler for restart request on bus
 */
LOCAL void _restart_handler(void);

/* Private variables ---------------------------------------------------------*/
#if (ASIC_NUM == 1)
LOCAL i2c_slave_t _reception[I2C_RECEPTIONISTS_NUM] =
{
	{
		.clbk_hdl        = _recept_callback_handler,
		.receiver_hdl    = _stop_handler,
		.restart_hdl     = _restart_handler,
		.slave_address   = I2C_RECEPTINIST_ADDR
	}
};
#elif (ASIC_NUM == 2) || (ASIC_NUM == 3)
LOCAL i2c_slave_t _reception[I2C_RECEPTIONISTS_NUM] =
{
	{
		.clbk_hdl        = _recept_callback_handler,
		.receiver_hdl    = _stop_handler,
		.restart_hdl     = _restart_handler,
		.slave_address   = I2C_RECEPTINIST_ADDR
	}
};
#else
/**
 * I2C receptionist
 */
LOCAL i2c_slave_t _reception =
{
	.clbk_hdl        = _recept_callback_handler,
	.receiver_hdl    = _stop_handler           ,
	.restart_hdl     = _restart_handler        ,
	.slave_address   = I2C_RECEPTINIST_ADDR
};
#endif
/**
 * I2C Soft buffer queue
 */
LOCAL QueueHandle_t _i2c_queue = NULL;

/* Exported global variables -------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/**
 * @brief I2C command receptionist
 * @param[in] 	pv	: own task handler
 * @param[out] 	NA
 * @return 		NA
 * @details I2C command receptionist
 */
void task_i2c_recept(void* pv)
{
	uint8_t    buffer[_PAYLOAD_LENGTH_MAX];
	uint8_t*   payload = NULL;
	uint16_t   length = 0;
	i2c_data_t data;
	//! Start bash/shell task
	task_handle_t *handler = (task_handle_t *)(pv);
	//! Initialize I2C receptionist
	i2c_slave_init(_reception);
	//! Create I2C queue
	_i2c_queue = xQueueCreate(_PAYLOAD_LENGTH_MAX, sizeof(i2c_data_t));
	//! Waiting for log task ready
	while (TASK_READY != xGetTaskState("slog"))
	{
		//! Call task delay to let the others run
		vTaskDelay(1);
	}
	//! Indicate task is ready
	handler->state = TASK_READY;
	taskENTER_CRITICAL();
	log_msg("[Task] %s() is ready.\r\n", __FUNCTION__);
	taskEXIT_CRITICAL();
	while(1)
	{
		//! Checkout a data from queue
		xQueueReceive(_i2c_queue, &data, portMAX_DELAY);

		if (I2C_DATA_PAYLOAD_BODY == data.type)
		{
			buffer[length++] = data.value;
		}
		else //! I2C_DATA_PAYLOAD_END
		{
			//! Provide message memory space
			payload = pvPortMalloc(length * sizeof(uint8_t));
			//! Verify memory allocation
			if (NULL == payload)
			{
				slogf(SLOG_ID_LCC_SYSTEM, SLOG_ERROR,     \
						"[ERR] Malloc error: %s, %d\r\n", \
						               __FILE__, __LINE__);
			}
			else
			{
				//! Send message to system mailbox
				message_t* msg = create_msg(i2c_slave_task_id,    \
						                    sys_ctrl_task_id,     \
											RECEIVED_I2C_COMMAND, \
											length, payload);
				//! Check message
				if (NULL != msg)
				{
					//! Send to system mailbox
					mail_send(msg, &sys_ctrl_mailbox);
				}
				else
				{
					slogf(SLOG_ID_LCC_SYSTEM, SLOG_ERROR,           \
					     "[ERR]Could not send message: %s, %d\r\n", \
							                     __FILE__, __LINE__);
					//! Free command
					vPortFree(payload);
				}
			}
			//! Reset buffer
			memset(buffer, 0x00, _PAYLOAD_LENGTH_MAX);
			//! Reset length
			length = 0;
		}
		//! Reset data
		data.type = I2C_DATA_PAYLOAD_BODY;
		data.value = 0;
		//! Call task delay to let the others run
		vTaskDelay(1);
	}
}
/* Local functions -----------------------------------------------------------*/
/**
 * @brief Callback handler for upper layer
 * @param[in] 	NA
 * @param[out] 	NA
 * @return 		NA
 * @details Callback handler for upper layer
 */
LOCAL void _recept_callback_handler(uint8_t byte)
{
	i2c_data_t data;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	data.type = I2C_DATA_PAYLOAD_BODY;
	data.value = byte;
	//! Send data to queue
	xQueueSendFromISR(_i2c_queue, &data, &xHigherPriorityTaskWoken);
}
/**
 * @brief Receiving completed handler
 * @param[in] 	NA
 * @param[out] 	NA
 * @return 		NA
 * @details Receiving completed handler
 */
LOCAL void _stop_handler(void)
{
	i2c_data_t data;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	data.type = I2C_DATA_PAYLOAD_END;
	data.value = 0xFF;
	//! Send data to queue
	xQueueSendFromISR(_i2c_queue, &data, &xHigherPriorityTaskWoken);
	//! Wake up higher priority task if any
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
/**
 * @brief Handler for restart request on bus
 * @param[in] 	NA
 * @param[out] 	NA
 * @return 		NA
 * @details Handler for restart request on bus
 */
LOCAL void _restart_handler(void)
{
	uint8_t  buffer[_PAYLOAD_LENGTH_MAX];
	#if (ASIC_NUM == 1)
	uint8_t* payload = NULL;
	#endif
	uint16_t length = 0;
	i2c_data_t data;
	volatile uint32_t timeout = 512;

	data.type = I2C_DATA_PAYLOAD_BODY;
	data.value = 0x00;

	//! TODO: Get command
	while ((I2C_DATA_PAYLOAD_BODY == data.type) && (--timeout))
	{
		//! Checkout a data from queue
		xQueueReceive(_i2c_queue, &data, 0);
		//! Extract data
		if(I2C_DATA_PAYLOAD_BODY == data.type)
			buffer[length++] = data.value;
	}
	//! TODO: Parsing command

	#if (ASIC_NUM == 1) //! Forward message to I2C master for ASIC 2, 3
	//! Provide message memory space
	payload = pvPortMalloc(length * sizeof(uint8_t));
	//! Verify memory allocation
	if (NULL == payload)
	{
		slogf(SLOG_ID_LCC_SYSTEM, SLOG_ERROR,     \
				"[ERR] Malloc error: %s, %d\r\n", \
				               __FILE__, __LINE__);
	}
	else
	{
		//! Copy data to payload
		memcpy(payload, buffer, length);
		//! Send message to system mailbox
		message_t* msg = create_msg(i2c_slave_task_id,    \
				                    sys_ctrl_task_id,     \
									RECEIVED_I2C_COMMAND, \
									length, payload);
		//! Check message
		if (NULL != msg)
		{
			//! Send to system mailbox
			mail_send(msg, &sys_ctrl_mailbox);
		}
		else
		{
			slogf(SLOG_ID_LCC_SYSTEM, SLOG_ERROR,           \
			     "[ERR]Could not send message: %s, %d\r\n", \
					                     __FILE__, __LINE__);
			//! Free command
			vPortFree(payload);
		}
	}
	#endif /* (ASIC_NUM == 1) */

	//! TODO: Prepare data for it-self

	//! Send back to commander
	i2c_slave_write(buffer, length);
}
/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
