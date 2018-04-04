/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    os.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    June-26-2015
 * @brief   This file contains definitions related to OS in the firmware
 *
 ******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __OS_H__
#define __OS_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Standard library. */
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
/* Common definition */
#include "std_type.h"

/* OS */
/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"

/* Exported define -----------------------------------------------------------*/
#define TASK_NUM			5
/**
 * IMPORTANT!!! This defined is used to select the task that interrupt signal
 * from other ASICs will notify.
 * IF YOU CHANGE THE ORDER OF task_handler PLEASE UPDATE THIS NUMBER OR THE
 * SYSTEM WILL MALFUNCTION.
 */
#define LCC_CMD_TSK_NUM		2

/* Exported typedef ----------------------------------------------------------*/

/**
 * @brief task_priority_t
 * Task priority definitions
 */
typedef enum task_priority
{
	__TASK_PRIO_1		= tskIDLE_PRIORITY + 1,
	__TASK_PRIO_2		= tskIDLE_PRIORITY + 2,
	__TASK_PRIO_HIGHEST	= tskIDLE_PRIORITY + 15
} task_priority_t;

/**
 * @brief task_stacksize_t
 * Task stack size definitions
 */
typedef enum task_stacksize {
	__TASK_STACK_SIZE_64	= configMINIMAL_STACK_SIZE + 64,
	__TASK_STACK_SIZE_128	= configMINIMAL_STACK_SIZE + 128,
	__TASK_STACK_SIZE_256	= configMINIMAL_STACK_SIZE + 256,
	__TASK_STACK_SIZE_512	= configMINIMAL_STACK_SIZE + 512,
	__TASK_STACK_SIZE_1024	= configMINIMAL_STACK_SIZE + 1024,
	__TASK_STACK_SIZE_1536	= configMINIMAL_STACK_SIZE + 1536,
	__TASK_STACK_SIZE_2048	= configMINIMAL_STACK_SIZE + 2048,
	__TASK_STACK_SIZE_4096	= configMINIMAL_STACK_SIZE + 4096,
} task_stacksize_t;

/**
 * @brief __task_configure_t
 *
 */
typedef struct task_configure_t
{
	void (*task)(void *);				/** pointer to task */
	const char * const name;			/** task name */
	const task_stacksize_t stacksize;	/** task stack size */
	const task_priority_t prio;			/** task priority */
	const uint16_t time_sleep;			/** task time sleep */
} task_configure_t;

/**
 * @brief task_state_t
 *
 */
typedef enum task_state
{
	TASK_INITIALIZE	= 0,
	TASK_READY		= 1
} task_state_t;

/**
 * @brief task_handle_t
 *
 */
typedef struct task_handle
{
	task_state_t		state;	/** Indicate that task init done or not*/
	uint8_t				idx;	/** Index of task in task list */
	TaskHandle_t		handle;	/** Task handler */
	uint16_t			time_sleep;
} task_handle_t;

/* Exported variables --------------------------------------------------------*/
/* Task handler */
extern task_handle_t task_handler[TASK_NUM];

/* Task list configure */
extern const task_configure_t task_list[TASK_NUM];

/* Exported functions --------------------------------------------------------*/
/**
 * @brief init_platform
 * The function starts all driver
 * @param None
 * @return None
 */
void init_platform(void);

/**
 * @brief task_query_tid
 * The function will query task id from task name
 * @param task_name - name of task to get task id
 * @return an integer indicates task id
 */
int task_query_tid(const char * const task_name);

#ifdef __cplusplus
}
#endif
#endif /* __OS_H__ */

/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
