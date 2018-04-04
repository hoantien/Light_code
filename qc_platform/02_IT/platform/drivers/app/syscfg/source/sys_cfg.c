/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    sys_cfg.c
 * @author  The LightCo.
 * @version 1.0.0
 * @date    June 15, 2016
 * @brief   
 *
 ******************************************************************************/

/******************************************************************************/
/**  						Revision history
 *
 * * 1.0.0	June 15, 2016	Initial revision
 */
/******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "sys_cfg.h"
#include "mailbox.h"
#include "stdlib.h"
#include "semphr.h"
#include "queue.h"
/* Private define ------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Static functions prototype-------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Exported global variables -------------------------------------------------*/

/**
 * High level task IDs
 */
uint16_t log_task_id       = 0;
uint16_t bash_task_id      = 0;
uint16_t com_srv_task_id   = 0;
uint16_t storage_task_id   = 0;
uint16_t sys_ctrl_task_id  = 0;
uint16_t spi_slave_task_id = 0;
uint16_t i2c_slave_task_id = 0;
/**
 * Mailboxes
 */
mailbox_t   sys_ctrl_mailbox = \
{
		.size = SYS_MAILBOX_SIZE,
		.top  = 0,
		.bottom = 0,
		.repeat = FALSE,
		.inbox = NULL,
		.sempr = NULL
};
mailbox_t   com_srv_mailbox = \
{
		.size = COM_SRV_MAILBOX_SIZE,
		.top = 0,
		.bottom = 0,
		.repeat = FALSE,
		.inbox = NULL,
		.sempr = NULL
};
mailbox_t   storage_mailbox = \
{
		.size = STORAGE_MAILBOX_SIZE,
		.top = 0,
		.bottom = 0,
		.repeat = FALSE,
		.inbox = NULL,
		.sempr = NULL
};
/* Exported functions --------------------------------------------------------*/
/**
 * @brief Configure none-volatile system information
 * @param[in] 	NA
 * @param[out] 	NA
 * @return 		NA
 * @details Configure none-volatile system information
 */
void sys_config(void)
{
	//! Create task id
	log_task_id       = rand();
	bash_task_id      = rand();
	com_srv_task_id   = rand();
	storage_task_id   = rand();
	sys_ctrl_task_id  = rand();
	spi_slave_task_id = rand();
	i2c_slave_task_id = rand();
	//! Create all mailboxes
	com_srv_mailbox.inbox  = pvPortMalloc(SYS_MAILBOX_SIZE    * sizeof(message_t*));
	storage_mailbox.inbox  = pvPortMalloc(COM_SRV_MAILBOX_SIZE* sizeof(message_t*));
	sys_ctrl_mailbox.inbox = pvPortMalloc(STORAGE_MAILBOX_SIZE* sizeof(message_t*));
	//! Create semaphores
	vSemaphoreCreateBinary(com_srv_mailbox.sempr);
	vSemaphoreCreateBinary(storage_mailbox.sempr);
	vSemaphoreCreateBinary(sys_ctrl_mailbox.sempr);
}
/**
 * @brief Get task state
 * @param[in] 	NA
 * @param[out] 	NA
 * @return 		NA
 * @details Get task state
 */
task_state_t xGetTaskState(const char * const name)
{
	int id = xGetTaskID(name, sys_task_list, SYS_TASK_NUM);
	return sys_task_handler[id].state;
}
/**
 * @brief   Task_query_tid
 * @details The function will query task id from task name
 * @param   task_name - name of task to get task id
 * @return  None
 */
int xGetTaskID(const char * const name, const task_configure_t* list, uint16_t size)
{
	int i = 0;
	int ret = -1;
	/* loop check task name */
	while (size > i)
	{
		if(0 == strncmp(name, list[i].name, strlen(name)))
		{
			ret = i;
			break;
		}
		i++;
	}
	return ret;
}
/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
