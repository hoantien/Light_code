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
#include "spi_slave.h"
#include "ccb_cmd.h"
#include "sections.h"

/* Exported variables---------------------------------------------------------*/
void system_tick(void *pv);

/**
 * task_log
 * Command line
 */

void system_tick(void *pv)
{
}

/* task handler */
task_handle_t  task_handler[TASK_NUM];

/* task list configure */
const task_configure_t task_list[TASK_NUM] =
{

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
}

/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
