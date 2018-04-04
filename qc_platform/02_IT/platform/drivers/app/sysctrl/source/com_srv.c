/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    com_srv.c
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
#include "std_type.h"
#include "board_config.h"
#include "com_srv.h"
#include "os.h"
#include "log.h"
#include "i2c_slave.h"
#include "i2cm.h"
#include "sys_tasks.h"
/* Private define ------------------------------------------------------------*/
#define COM_TASK_NUM 2
/* Private typedef -----------------------------------------------------------*/
/**
 * Internal task ID
 */
#define _I2C_TASK 0
#define _SPI_TASK 1
/* Private macro -------------------------------------------------------------*/

/* Static functions prototype-------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Exported global variables --------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/**
 * @brief Communication service task
 * @param[in] 	pv	: own task handler
 * @param[out] 	NA
 * @return 		NA
 * @details Communication service task
 */
void task_com_srv(void* pv)
{
	//! Start communication service task
	task_handle_t *handler = (task_handle_t *)(pv);
	//! Initialize all channels
	for (i2c_t ch = I2C_CH0; ch <= I2C_CH18; ch++)
	{
		//! Initialize indicated channel
		i2cm_init(ch);
	}
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
		//! TODO: Communication task

		//! Call task delay to let the others run
		vTaskDelay(1);
	}
}

/**
 * @brief I2C communication control task
 * @param[in] 	pv	: own task handler
 * @param[out] 	NA
 * @return 		NA
 * @details I2C communication control task
 */
void task_i2c_ctrl(void* pv)
{
	//! Start bash/shell task
	task_handle_t *handler = (task_handle_t *)(pv);
	//! Define all channels
	for(i2c_t child = I2C_CH0; child <= I2C_CH17; child++)
	{
		//! Initialize specified channel
		i2cm_init(child);
	}
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
		//! TODO: Implement i2c driver control task
		//! Call task delay to let the others run
		vTaskDelay(1);
	}
}

/**
 * @brief SPI communication control task
 * @param[in] 	pv	: own task handler
 * @param[out] 	NA
 * @return 		NA
 * @details SPI communication control task
 */
void task_spi_ctrl(void* pv)
{
	//! Start bash/shell task
	task_handle_t *handler = (task_handle_t *)(pv);
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
		//! TODO: Implement i2c commander task

		//! Call task delay to let the others run
		vTaskDelay(1);
	}
}


/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
