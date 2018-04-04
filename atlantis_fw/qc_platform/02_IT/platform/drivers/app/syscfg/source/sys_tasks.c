/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    sys_tasks.c
 * @author  The LightCo.
 * @version 1.0.0
 * @date    June 17, 2016
 * @brief
 *
 ******************************************************************************/

/******************************************************************************/
/**  						Revision history
 *
 * * 1.0.0	June 17, 2016	Initial revision
 */
/******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include <string.h>

#include "sys_ctrl.h"
#include "sys_cfg.h"
#include "log.h"
#include "bash.h"
#include "spi_cmd.h"
#include "i2c_cmd.h"
#include "com_srv.h"
#include "storage.h"

/* Exported variables---------------------------------------------------------*/
/* Exported functions---------------------------------------------------------*/
/* task handler */
task_handle_t  sys_task_handler[SYS_TASK_NUM];
/* task list configure */
const task_configure_t sys_task_list[SYS_TASK_NUM] =
{
	/* task_log */
	{
		.task =			task_log,
		.name =			"slog",
		.prio =			__TASK_PRIO_HIGHEST,
		.stacksize =	__TASK_STACK_SIZE_128,
		.time_sleep =	1,
	},
	/* task_bash */
	{
		.task =			task_bash,
		.name =			"uart_commander",
		.prio =			__TASK_PRIO_HIGHEST - 1,
		.stacksize =	__TASK_STACK_SIZE_64,
		.time_sleep =	1,
	},
	/* task_spi_slave */
	{
		.task =			task_spi_recept,
		.name =			"spi_commander",
		.prio =			__TASK_PRIO_HIGHEST - 2,
		.stacksize =	__TASK_STACK_SIZE_64,
		.time_sleep	=	1,
	},
	/* task_i2c_slave */
	{
		.task =			task_i2c_recept,
		.name =			"i2c_commander",
		.prio =			__TASK_PRIO_HIGHEST - 3,
		.stacksize =	__TASK_STACK_SIZE_64,
		.time_sleep =	1,
	},
	/* task_syscall */
	{
		.task =			task_sysctrl,
		.name =			"sysctrl",
		.prio =			__TASK_PRIO_HIGHEST - 4,
		.stacksize =	__TASK_STACK_SIZE_512,
		.time_sleep =	1,
	},
	/* task_storage */
	{
		.task =			task_storage,
		.name =			"storage",
		.prio =			__TASK_PRIO_HIGHEST - 5,
		.stacksize =	__TASK_STACK_SIZE_64,
		.time_sleep =	1,
	},
	/* task_i2c_ctrl */
	{
		.task =			task_i2c_ctrl,
		.name =			"i2c_m",
		.prio =			__TASK_PRIO_HIGHEST - 6,
		.stacksize =	__TASK_STACK_SIZE_64,
		.time_sleep =	1,
	},
	/* task_spi_ctrl */
	{
		.task =			task_spi_ctrl,
		.name =			"spi_m",
		.prio =			__TASK_PRIO_HIGHEST - 7,
		.stacksize =	__TASK_STACK_SIZE_64,
		.time_sleep =	1,
	},
	/* task_com_srv */
	{
		.task =			task_com_srv,
		.name =			"com_service",
		.prio =			__TASK_PRIO_HIGHEST - 8,
		.stacksize =	__TASK_STACK_SIZE_64,
		.time_sleep =	1,
	}
};

/* Exported function ---------------------------------------------------------*/
/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
