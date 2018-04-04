/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    register_dump.c
 * @author  The LightCo
 * @version 1.0.0
 * @date    July 28, 2016
 * @brief   To support testing the LCC command low level judgment
 *
 ******************************************************************************/
/**							Revision history
 *
 * 		* 1.0.0	July 28, 2016	Initial revision
 */
/******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "register_dump.h"
#include "board_config.h"
#include "log.h"
#include "hal_i2c.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"
#include "i2cm.h"

/* Private macro -------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Local functions prototype -------------------------------------------------*/
static void reg_dump_callback(i2cm_error_t status, void *param);
/* Exported global variables -------------------------------------------------*/
//! Dumping cameras registers queue
xQueueHandle	 queue_cam_registers;
//! Camera registers dumping semaphore
xSemaphoreHandle sempr_cam_register_dump;
/* Private variables ---------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/**
 * @brief Task to verify register setting from camera modules
 * @details Task to verify register setting from camera modules
 * @param[in] 	vParameter: task parameter
 * @param[out]	NA
 * @return 		NA
 */
void task_cam_register_dump(void *vParameter)
{
	task_handle_t *hdl = (task_handle_t *)(vParameter);
	uint8_t exit = 0;
	//! Data buffer
	uint8_t* actual;
	//! Data length
	uint16_t length;
	//! I2C TX buffer
	hal_i2c_buffer_t i2c_tx_buffer;
	hal_i2c_buffer_t i2c_rx_buffer;
	//! Dumped register information
	cam_regs_dump_t reg;
	while (!exit)
	{
		/* #task_console done for slogf */
		if(TASK_READY == task_handler[task_query_tid("slog")].state)
			exit = 1;
		vTaskDelay(1);
	}
	//! Create queue
	queue_cam_registers = xQueueCreate(256, sizeof(cam_regs_dump_t));
	//! Verify created queue
	if (NULL == queue_cam_registers)
	{
		slogf(SLOG_ID_CAMERA, SLOG_DEBUG,
				"[ERROR] %s() task will be standby", __FUNCTION__);
		while (1)
		{
			vTaskDelay(1);
		}
	}
	//! Create semaphore
	sempr_cam_register_dump = xSemaphoreCreateBinary();
	/* Task start */
	taskENTER_CRITICAL();
	log_msg("Start %s\r\n", __FUNCTION__);
	taskEXIT_CRITICAL();
	//! Mark as task ready
	hdl->state = TASK_READY;
	uint32_t count = 0;
	while (1)
	{
		//! Take semaphore for register dump task
		xSemaphoreTake(sempr_cam_register_dump, portMAX_DELAY);
		//! Read all queued registers
		for (;;)
		{
			if (pdPASS == xQueueReceive(queue_cam_registers, &reg, 0))
			{
				slogf(SLOG_ID_CAMERA, SLOG_DEBUG, "TEST-POINT: %08u", count++);
				//! Get data length
				length = reg.size;
				//*! Provide buffer
				actual = pvPortMalloc(length);
				//! Reset actual buffer
				memset(actual, 0x00, length);
				//! Write info to read
				i2c_tx_buffer.bytes  = (uint8_t*)&reg.addr;
				i2c_tx_buffer.length = sizeof(reg.addr);
				i2c_rx_buffer.bytes  = actual;
				i2c_rx_buffer.length = length;
				//! Transmit data
				i2cm_transceiver(reg.child, reg.id,
								 i2c_tx_buffer.bytes, i2c_tx_buffer.length,
								 i2c_rx_buffer.bytes, i2c_rx_buffer.length,
								 reg_dump_callback,   (void *)&reg);
				//! Waiting for finish transmitting
				while (reg.check == FALSE);
				//! Judgment
				if (memcmp(actual, reg.data, length))
				{
					slogf(SLOG_ID_CAMERA, SLOG_DEBUG, "******************************************");
					//! Report FAILED via slogf
					slogf(SLOG_ID_CAMERA, SLOG_DEBUG, "Tested register %s FAILED", reg.name);
					//! Provide temporary buffer
					char* buffer = pvPortMalloc(2*length + 1);
					//! Print dumped output
					memset(buffer, 0x00, length + 1);
					for (uint16_t i = 0; i < length; i++)
					{
						sprintf(&buffer[2*i], "%02x", reg.data[i]);
					}
					slogf(SLOG_ID_CAMERA, SLOG_DEBUG, "Write: 0x%s", buffer);
					memset(buffer, 0x00, length + 1);
					for (uint16_t i = 0; i < length; i++)
					{
						sprintf(&buffer[2*i], "%02x", actual[i]);
					}
					slogf(SLOG_ID_CAMERA, SLOG_DEBUG, "Read : 0x%s", buffer);
					slogf(SLOG_ID_CAMERA, SLOG_DEBUG, "******************************************");
					//! Free memory
					vPortFree(buffer);
				}
				else
				{
					//! Report PASSED via slogf
					slogf(SLOG_ID_CAMERA, SLOG_DEBUG, "Test register %s PASSED", reg.name);
				}
				//! Free memory
				vPortFree(actual);
				//! Free memory
				vPortFree(reg.data);
			}
			else
				break;
		}
	}
}

/* Local functions -----------------------------------------------------------*/
static void reg_dump_callback(i2cm_error_t status, void *param)
{
	cam_regs_dump_t* reg = (cam_regs_dump_t*)param;
	reg->check = TRUE;
}
/*********** Portions COPYRIGHT 2016 Light.Co., Ltd.***** END OF FILE *********/
