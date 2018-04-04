/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    optical_bc.c
 * @author  The LightCo
 * @version V1.0.0
 * @date    Mar-10-2016
 * @brief   This file contains expand of task_optical_bc
 *
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "os.h"
#include "log.h"
#include "lcc_cmd.h"
#include "stdlib.h"
#include "light_system.h"
#include "assert.h"
#include "lcc_cmd_log.h"
#include "actuator.h"

/* Private define-------------------------------------------------------------*/
#define SLOGF_ID				SLOG_ID_OPTICAL

/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private function ----------------------------------------------------------*/
static void nudge_isr(void *cb_param)
{
	*((bool *)cb_param) = FALSE;
}

/* Private variables----------------------------------------------------------*/
#if (ASIC_NUM != ASIC1)
static SemaphoreHandle_t gpio_intr = NULL;
#endif
/* Exported variables --------------------------------------------------------*/

/* Export functions ----------------------------------------------------------*/
/*
 * task_actuator_create
 *
 */
void task_actuator_create(void)
{
	uint8_t i;
	cam_typedef_t *pcam;
	char task_name[16];
	BaseType_t ret = pdFALSE;
#if (ASIC_NUM != ASIC1)
	 /* Create mutex to protect shared resource */
	gpio_intr = xSemaphoreCreateMutex();
	if(gpio_intr == NULL )
	{
		SLOGF(SLOG_ERROR, "Failed to create mutex");
	}
#endif
	/* Piezo task to control LENS creation */
	for (i = 0; i < light_system->cam_tbl_size; i++)
	{
		pcam = &light_system->cam_tbl[i];
		if (pcam->info.ch > 0)
		{
			if ((pcam->info.grp == GRP_B) || (pcam->info.grp == GRP_C))
			{
				snprintf (task_name, 16, "lens_%x", pcam->info.module);
				ret = xTaskCreate(
								task_actuator_lens,		/* task function */
								task_name,				/* task name */
								__TASK_STACK_SIZE_256,	/* stack size */
								pcam,					/* passing parameters */
								__TASK_PRIO_HIGHEST - 5,/* task priority */
								NULL					/* handle */
							);
				/* Assert the creation task return to make sure all of
					piezo task was created */
				assert_param(pdPASS == ret);
#if (ASIC_NUM != ASIC1)
				if (xSemaphoreTake(gpio_intr, (TickType_t)10) == pdTRUE)
				{
					light_system->check_asic_ready++;
					/* Release resource */
					xSemaphoreGive(gpio_intr);
				}
#endif
			}
		}
	}

	/* Piezo task to control MIRROR creation */
	for (i = 0; i < light_system->cam_tbl_size; i++)
	{
		pcam = &light_system->cam_tbl[i];
		if (pcam->info.ch > 0)
		{
			if (((pcam->info.grp == GRP_B) || (pcam->info.grp == GRP_C)) &&
				(pcam->info.module != CAM_B4) &&
				(pcam->info.module != CAM_C5) &&
				(pcam->info.module != CAM_C6))
			{
				snprintf (task_name, 16, "mirr_%x", pcam->info.module);
				ret = xTaskCreate(
								task_actuator_mirr,		/* task function */
								task_name,				/* task name */
								__TASK_STACK_SIZE_256,	/* stack size */
								pcam,					/* passing parameters */
								__TASK_PRIO_HIGHEST - 5,/* task priority */
								NULL					/* handle */
							);
				/* Assert the creation task return to make sure all of
					piezo task was created */
				assert_param(pdPASS == ret);
#if (ASIC_NUM != ASIC1)
		if (xSemaphoreTake(gpio_intr, (TickType_t)10) == pdTRUE)
		{
			light_system->check_asic_ready++;
			/* Release resource */
			xSemaphoreGive(gpio_intr);
		}
#endif
			}
		}
	}
}

/*
 * task_actuator_ctrl
 *
 */
void task_actuator_lens(void *vParameter)
{
	cam_typedef_t *pcam = (cam_typedef_t *)(vParameter);
	uint32_t event;
	uint16_t cmd_tid;
	lcc_cmd_tid_t pcmd;
	uint16_t cmd_status = 0;

	/* Start initialize #task_actuator_ctrl is dependency to #lcccmd */
	while (TASK_READY != task_handler[task_query_tid("lcccmd")].state ||
			TASK_READY != task_handler[task_query_tid("ccb_ctrl")].state)
	{
		vTaskDelay(1);
	}

	/* Task start */
	taskENTER_CRITICAL();
	log_msg("Start %s: %x\r\n", __FUNCTION__, pcam->info.module);
	taskEXIT_CRITICAL();

	actuator_t *piezo = (actuator_t *)pcam->optical->lens;

	piezo->queue = xQueueCreate(10, sizeof(lcc_cmd_tid_t));
	assert_param(piezo->queue);

	actuator_init(piezo);
	/* Get the initial position */
	piezo->hall_obj->read_position(piezo->hall_i2c,
					(uint16_t *)pcam->optical->settings->lens_position);
#if (ASIC_NUM != ASIC1)
	if (xSemaphoreTake(gpio_intr, (TickType_t)10) == pdTRUE)
	{
		if(--light_system->check_asic_ready == 0)
		{
			SLOGF(SLOG_INFO, "Raise up the interrupt pin");
			set_intr_pin(ON);
		}
		/* Release resource */
		xSemaphoreGive(gpio_intr);
	}
#endif

	while(1)
	{
		xQueueReceive(piezo->queue, &pcmd, portMAX_DELAY);
		cmd_tid = pcmd.cmd_tid;
		event = pcmd.event;

		if (event & OPT_EVENT_LENS_POSITION)
		{
			uint16_t *p_lens_position =
							(uint16_t *)pcam->optical->settings->lens_position;
			uint16_t tolerance =  pcam->optical->settings->lens_position_tolerance;
			SLOGF(SLOG_DEBUG, "CAM-%X : Lens position to %u with tolerance %d",
			        pcam->info.module, *p_lens_position, tolerance);
			if(actuator_move_to_position(piezo, p_lens_position, tolerance) == ACTUATOR_OK)
				cmd_status = LCC_CMD_SUCCESS;
			else
				cmd_status = LCC_CMD_UNSUCCESS;
		}
		else if (event & OPT_EVENT_FOCUS_DISTANCE)
		{
			uint32_t *p_focus_distance =
							(uint32_t *)pcam->optical->settings->focus_distance;

			SLOGF(SLOG_DEBUG, "CAM-%X : Focus distance", pcam->info.module);
			if(actuator_move_to_distance(piezo, p_focus_distance) == ACTUATOR_OK)
				cmd_status = LCC_CMD_SUCCESS;
			else
				cmd_status = LCC_CMD_UNSUCCESS;
		}
		else /* if (event & OPT_EVENT_LENS_NUDGE) */
		{
			uint8_t *lens_nudge = pcam->optical->settings->lens_nudge;

			SLOGF(SLOG_DEBUG, "CAM-%X : Nudge lens", pcam->info.module);
			if(actuator_nudge(piezo, lens_nudge[0],
					*((uint16_t *)(lens_nudge + 1)), nudge_isr, &piezo->moving) == ACTUATOR_OK)
				cmd_status = LCC_CMD_SUCCESS;
			else
				cmd_status = LCC_CMD_UNSUCCESS;

			while (piezo->moving == TRUE)
			{
				vTaskDelay(1);
			}
			/* update current position to camera setting */
			piezo->hall_obj->read_position(piezo->hall_i2c,
							(uint16_t *)pcam->optical->settings->lens_position);
		}

		/* take semaphore to access settings memory */
		xSemaphoreTake(piezo->semaphore, portMAX_DELAY);
		/* update moving status */
		piezo->moving = FALSE;
		/* release semaphore after accessing data */
		xSemaphoreGive(piezo->semaphore);
		/* call back to upper layer */
		if (piezo->cb_func != NULL)
		{
			piezo->cb_func(piezo->cb_param);
			/* disable call-back function in the case of LCC command */
			piezo->cb_func = NULL;
		}
		lcc_cmd_log_update_status(cmd_tid, cmd_status, object_to_cam_bitmask(pcam));
	}
}

/*
 * task_actuator_ctrl
 *
 */
void task_actuator_mirr(void *vParameter)
{
	cam_typedef_t *pcam = (cam_typedef_t *)(vParameter);
	uint16_t *p_mirr_position;

	uint32_t event;
	uint16_t cmd_tid;
	lcc_cmd_tid_t pcmd;
	uint16_t cmd_status = 0;

	/* Start initialize #task_actuator_ctrl is dependency to #lcccmd */
	while (TASK_READY != task_handler[task_query_tid("lcccmd")].state ||
			TASK_READY != task_handler[task_query_tid("ccb_ctrl")].state)
	{
		vTaskDelay(1);
	}

	/* Task start */
	taskENTER_CRITICAL();
	log_msg("Start %s: %x\r\n", __FUNCTION__, pcam->info.module);
	taskEXIT_CRITICAL();

	actuator_t *piezo = (actuator_t *)pcam->optical->mirr;

	piezo->queue = xQueueCreate(10, sizeof(lcc_cmd_tid_t));
	assert_param(piezo->queue);

	actuator_init(piezo);
	/* Get the initial position */
	piezo->hall_obj->read_position(piezo->hall_i2c,
					(uint16_t *)pcam->optical->settings->mirr_position);
#if (ASIC_NUM != ASIC1)
	if (xSemaphoreTake(gpio_intr, (TickType_t)10) == pdTRUE)
	{
		if(--light_system->check_asic_ready == 0)
		{
			SLOGF(SLOG_INFO, "Raise up the interrupt pin");
			set_intr_pin(ON);
		}
		/* Release resource */
		xSemaphoreGive(gpio_intr);
	}
#endif
	while(1)
	{
		xQueueReceive(piezo->queue, &pcmd, portMAX_DELAY);
		cmd_tid = pcmd.cmd_tid;
		event = pcmd.event;

		if (event & OPT_EVENT_MIRROR_POSITION)
		{
			p_mirr_position =
							(uint16_t *)pcam->optical->settings->mirr_position;

			SLOGF(SLOG_DEBUG, "CAM-%X : Mirror position", pcam->info.module);
			if(actuator_move_to_position(piezo, p_mirr_position, 1) == ACTUATOR_OK)
				cmd_status = LCC_CMD_SUCCESS;
			else
				cmd_status = LCC_CMD_UNSUCCESS;
		}
		else if (event & OPT_EVENT_MIRROR_CALIB)
		{
			SLOGF(SLOG_DEBUG, "CAM-%X : Mirror calibration", pcam->info.module);
			if(actuator_calibrate(piezo) == ACTUATOR_OK)
				cmd_status = LCC_CMD_SUCCESS;
			else
				cmd_status = LCC_CMD_UNSUCCESS;
		}
		else /* if (event & OPT_EVENT_MIRROR_NUDGE) */
		{
			uint8_t *mirr_nudge = pcam->optical->settings->mirr_nudge;

			SLOGF(SLOG_DEBUG, "CAM-%X : Nudge mirror", pcam->info.module);

			if(actuator_nudge(piezo, mirr_nudge[0],
					*((uint16_t *)(mirr_nudge + 1)), nudge_isr, &piezo->moving) == ACTUATOR_OK)
				cmd_status = LCC_CMD_SUCCESS;
			else
				cmd_status = LCC_CMD_UNSUCCESS;

			while (piezo->moving == TRUE)
			{
				vTaskDelay(1);
			}
			/* update current position to camera setting */
			piezo->hall_obj->read_position(piezo->hall_i2c,
							(uint16_t *)pcam->optical->settings->mirr_position);
		}

		/* take semaphore to access settings memory */
		xSemaphoreTake(piezo->semaphore, portMAX_DELAY);
		/* update moving status */
		piezo->moving = FALSE;
		/* release semaphore after accessing data */
		xSemaphoreGive(piezo->semaphore);
		/* call back to upper layer */
		if (piezo->cb_func != NULL)
		{
			piezo->cb_func(piezo->cb_param);
			/* disable call-back function in the case of LCC command */
			piezo->cb_func = NULL;
		}
		lcc_cmd_log_update_status(cmd_tid, cmd_status, object_to_cam_bitmask(pcam));
	}
}

/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
