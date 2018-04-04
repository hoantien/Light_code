/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    spi_cmd.c
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
#include "board_config.h"
#include "spi_cmd.h"
#include "os.h"
#include "log.h"
#include "sys_cfg.h"
/* Private define ------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Static functions prototype-------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Exported global variables --------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/**
 * @brief SPI command receptionist
 * @param[in] 	pv	: own task handler
 * @param[out] 	NA
 * @return 		NA
 * @details SPI command receptionist
 */
void task_spi_recept(void* pv)
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
		//! TODO: Implement SPI command receptionist task

		//! Call task delay to let the others run
		vTaskDelay(1);
	}
}


/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
