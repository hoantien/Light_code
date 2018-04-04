/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    bash.c
 * @author  The LightCo.
 * @version 1.0.0
 * @date    Jun 15, 2016
 * @brief   
 *
 ******************************************************************************/

/******************************************************************************/
/**  						Revision history
 *
 * * 1.0.0	Jun 15, 2016	Initial revision
 */
/******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "board_config.h"
#include "bash.h"
#include "sys_cfg.h"
#include "fifo.h"
#include "ascii.h"
#include "mailbox.h"
#include "log.c"

/* Private define ------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
#define COM_FIFO_SIZE 256
/* Private macro -------------------------------------------------------------*/

/* Static functions prototype-------------------------------------------------*/

/**
 * @brief To get bash/shell command
 * @param[in] NA
 * @param[out] length : length of command
 * @return NULL  : failed
 *         Others: memory block of received command
 * @details COM RX callback handler
 */
LOCAL uint8_t* _get_command(uint16_t* length);
/* Private variables ---------------------------------------------------------*/

/* Exported global variables -------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/**
 * @brief Initialize bash
 * @param[in] 	NA
 * @param[out] 	NA
 * @return 		NA
 * @details Initialize bash
 */
void bash_init(void)
{
	/* Initialize slog queue */
	slog_queue = (slog_queue_t *)assert_malloc(slog_queue, sizeof(slog_queue_t));
	if(slog_queue)
	{
		slog_queue->head =
				(struct slog_data_t *) assert_malloc(slog_queue->head, sizeof(*slog_queue->head));
		slog_queue->tail =
				(struct slog_data_t *) assert_malloc(slog_queue->tail, sizeof(*slog_queue->tail));
		slog_queue->head->str = NULL;
		slog_queue->tail->str = NULL;
		if(!slog_queue->head)
		{
			log_error("%s: slog_queue->head assert_malloc failed\r\n", __FUNCTION__);
		}
		else if(!slog_queue->tail)
		{
			log_error("%s: slog_queue->head assert_malloc failed\r\n", __FUNCTION__);
		}
		else
		{
			slog_queue->head->next = slog_queue->tail;
			slog_queue->tail->next = slog_queue->tail;
			slog_queue->total = 0;
		}
	}
	else
	{
		log_error("%s: slog_queue assert_malloc failed\r\n", __FUNCTION__);
	}
}
/**
 * @brief Bash/shell task
 * @param[in] 	NA
 * @param[out] 	NA
 * @return 		NA
 * @details Bash/shell task
 */
void task_bash(void* pv)
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
	while (1)
	{
		uint16_t length = 0;
		uint8_t* cmd = _get_command(&length);
		if (NULL != cmd)
		{
			//! Send message to system mailbox
			message_t* msg = create_msg(bash_task_id,          \
					                    sys_ctrl_task_id,      \
										RECEIVED_BASH_COMMAND, \
										length, cmd);
			//! Check message
			if (NULL != msg)
			{
				//! Send to system mailbox
				mail_send(msg, &sys_ctrl_mailbox);
			}
			else
			{
				//! Send error log onto screen
				slogf(SLOG_ID_LCC_SYSTEM, SLOG_ERROR,               \
				     "[ERR]Could not send message: %s, %d, %s\r\n", \
						                    __FILE__, __LINE__, cmd);
				//! Free command
				vPortFree(cmd);
			}
		}
		//! Call task delay to let the others run
		vTaskDelay(1);
	}
}
/* Private functions ---------------------------------------------------------*/
/**
 * @brief To get bash/shell command
 * @param[in] NA
 * @param[out] length : length of command
 * @return NULL  : failed
 *         Others: memory block of received command
 * @details COM RX callback handler
 */
LOCAL uint8_t* _get_command(uint16_t* length)
{
	char buffer[COM_FIFO_SIZE] = {0};
	//! Clear buffer
	memset(buffer, 0x00, COM_FIFO_SIZE);
	//! Get line
	getline(buffer, COM_FIFO_SIZE, 0);
	*length = strlen(buffer);
	//! Provide memory
	uint8_t* cmd = pvPortMalloc(*length + 1);
	//! Check memory allocation
	if (NULL == cmd)
	{
		slogf(SLOG_ID_LCC_SYSTEM, SLOG_ERROR,         \
				"[ERR] Malloc error: %s, %d, %s\r\n", \
				              __FILE__, __LINE__, cmd);
		*length = 0;
		return NULL;
	}
	//*! Clear memory
	memset(cmd, 0x00, *length + 1);
	//! Get data
	strncpy((char*)cmd, buffer, *length);
	//! Exit
	return cmd;
}
/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
