/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    task_list.c
 * @author  The LightCo
 * @version V1.0.0
 * @date    April-6-2016
 * @brief   This file contains expand of task_list
 *
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "os.h"
#include "log.h"
#include "spis.h"
#include "lcc_cmd.h"
#include "task_cam_ctrl.h"
#include "task_ccb_ctrl.h"
#include "sections.h"
#include "ina231.h"

/* Exported variables---------------------------------------------------------*/
/* Exported functions---------------------------------------------------------*/
/* task handler */
task_handle_t  task_handler[TASK_NUM];

/* task list configure */
const task_configure_t task_list[TASK_NUM] =
{
	/* task_lcc_cmd */
	{
		.task =			task_lcc_cmd,
		.name =			"lcccmd",
		.prio =			__TASK_PRIO_HIGHEST - 2,
		.stacksize =	__TASK_STACK_SIZE_1024,
		.time_sleep =	1,
	},/* task_lcc_cmd */
	{
		.task =			task_ccb_ctrl,
		.name =			"ccb_ctrl",
		.prio =			__TASK_PRIO_HIGHEST - 3,
		.stacksize =	__TASK_STACK_SIZE_256,
		.time_sleep =	1,
	},
	/* task_queue_log */
	{
		.task =			task_queue_log,
		.name =			"queuelog",
		.prio =			__TASK_PRIO_HIGHEST - 13,
		.stacksize =	__TASK_STACK_SIZE_128,
		.time_sleep =	1,
	},
    /* task_log */
    {
        .task =         task_log,
        .name =         "slog",
        .prio =         __TASK_PRIO_HIGHEST - 13,
        .stacksize =    __TASK_STACK_SIZE_128,
        .time_sleep =   1,
    },
	/* task_console */
	{
		.task =			task_console,
		.name =			"console",
		.prio =			__TASK_PRIO_HIGHEST - 14,
		.stacksize =	__TASK_STACK_SIZE_128,
		.time_sleep =	1,
	}
	/**
	 * @brief ADD MORE TASK HERE <<<<<<<<<<<<<<<<<<<
	 */
};

/* Exported function ---------------------------------------------------------*/
/**
 * @brief task_query_tid
 * The function will query task id from task name
 * @param task_name - name of task to get task id
 * @return None
 */
int task_query_tid(const char * const task_name)
{
	int i = 0;
	int ret = -1;
	int EXIT = 0;
	/* loop check task name */
	while (TASK_NUM > i && !EXIT)
	{
		if(!strcmp(task_name, task_list[i].name))
		{
			EXIT = 1;
			ret = i;
		}
		i++;
	}
	return ret;
}

/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
