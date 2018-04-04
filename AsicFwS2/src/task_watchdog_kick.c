/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    task_watchdog_kick.c
 * @author  The LightCo
 * @version V1.0.0
 * @date    Mar-10-2016
 * @brief   This file contains expand of __task_watchdog_kick
 *
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "os.h"
#include "log.h"
#include "hal_wdt.h"

/* Exported functions --------------------------------------------------------*/
/*
 * task_watchdog_kick
 *
 */
void task_watchdog_kick(void *vParameter)
{
	task_handle_t *hdl = (task_handle_t *)(vParameter);
	portTickType current_tick = xTaskGetTickCount();
	/* Start Watchdog hardware */
	hal_wdt_start();
	/* Task ready id */
	hdl->state = TASK_READY;
	/* Task start */
	taskENTER_CRITICAL();
	log_msg("Start %s\r\n", __FUNCTION__);
	taskEXIT_CRITICAL();
	/* Loop forever */
	while(1)
	{
		/* Refreshes Watchdog timer, if don't refreshes him when timeout
		reaches by WDG_CFG_IWDG_TIMEOUT, he will reset your system when the
		counter register over value is configured.
		*/
		hal_wdt_kick_dog();
		/* Task sleep __TASK_WATCHDOG_KICK_TIME_REFRESH */
		vTaskDelayUntil(&current_tick, hdl->time_sleep);
	}
}

/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
