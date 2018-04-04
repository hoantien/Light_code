/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    cbb_cmd_base_1000.c
 * @author  The LightCo
 * @version V1.0.0
 * @date    Mar-10-2016
 * @brief   This file contains expand of cbb_cmd_base_1000
 *
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "os.h"
#include "assert.h"
#include "log.h"
#include "light_system.h"
#include "task_cam_ctrl.h"
#include "lcc_cmd.h"
#include "lcc_cmd_log.h"
#include "lcc_system.h"
#include "task_cam_ctrl.h"
#include "task_ccb_ctrl.h"

/* Privated define------------------------------------------------------------*/
#define SLOGF_ID				SLOG_ID_LCC_CMD_BASE_1000

/* Privated typedef-----------------------------------------------------------*/
/*
 * ucid_mode_data_t
 */
typedef enum ucid_mode_data {
	CMD_UCID_DISABLED			= 0x00,
	CMD_UCID_UNKNOWN,
	CMD_UCID_DEBUG,
	CMD_UCID_PREVIEW,
	CMD_UCID_VIDEO,
	CMD_UCID_HIRES_CAPTURE,
	CMD_UCID_FOCAL_STACKING,
	CMD_UCID_HDR_CAPTURE,
	CMD_UCID_RESERVE,
	CMD_UCID_FTM_QUICK_CHECK,
	CMD_UCID_FTM_CALIBRATION,
	CMD_UCID_MAX
} ucid_mode_data_t;

/* Private function-----------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/**
 * cmd_light_active_ucid
 *
 */
void cmd_light_active_ucid(lcc_cmd_t *in, lcc_cmd_t *out)
{
	lcc_cmd_tid_t p_cmd;
	lcc_cmd_info_t cmd = (lcc_cmd_info_t)(LCC_CMD_LIGHT_ACTIVE_UCID);
	cam_typedef_t *p_cam;
	if(in->action == CMD_READ)
	{
		out->data = pvPortMalloc(cmd.size);
		if (NULL == out->data)
		{
			SLOGF(SLOG_ERROR, "%s: Lack of memory", __FUNCTION__);
			in->status = LCC_CMD_ERROR_MODULE_FAULT;
			return;
		}
		out->data[out->len++] = (uint8_t)(light_system->active_ucid >> 8);
		out->data[out->len++] = (uint8_t)(light_system->active_ucid);
		/* The command status will be updated in lcc_cmd task */
	}
	else
	{
		uint16_t ucid = *(uint16_t *)in->data;
		if (CMD_UCID_MAX > ucid)
		{
			/* TODO: Restore all setting to camera */
			/* Set active ucid to the received ucid */
			light_system->prev_ucid = light_system->active_ucid;
			light_system->active_ucid = ucid;
			uint8_t i = 0;
			uint16_t fps;
			uint64_t exposure;
			in->m_bitmask = 0;
			for(i = 0; i < light_system->cam_tbl_size; i++)
			{
				p_cam = &light_system->cam_tbl[i];
				SLOGF(SLOG_INFO, "Apply CAM-%X", p_cam->info.module);
				if ((0 < p_cam->info.ch) &&
					(p_cam->settings->open == CAM_MODULE_SW_STANDBY))
				{
					in->m_bitmask |= object_to_cam_bitmask(p_cam);
					memcpy((uint8_t *)&fps, p_cam->image->settings[
						light_system->active_ucid]->fps, sizeof(uint16_t));
					memcpy((uint8_t *)&exposure, p_cam->image->settings[
						light_system->active_ucid]->exposure, sizeof(uint64_t));
					p_cmd.cmd_tid = in->tid;
					p_cmd.event = 0;
					p_cmd.event = IMG_EVENT_CAM_SENSITIVITY |
						IMG_EVENT_CAM_RESOLUTION;
					if(fps)
						p_cmd.event |= IMG_EVENT_CAM_FPS;
					if(exposure)
						p_cmd.event |= IMG_EVENT_CAM_EXPOSURE_TIME;
					xQueueSend(p_cam->queue, &p_cmd, (TickType_t)10);
					SLOGF(SLOG_DEBUG, "Restore setting CAM-%X",	p_cam->info.module);
				}
			}
			/* Update the command status */
			in->status = LCC_CMD_SUCCESS;
			SLOGF(SLOG_DEBUG, "Received UCID: 0x%x", ucid);
		}
		else
		{
			in->status = LCC_CMD_INVALID_ARG;
			SLOGF(SLOG_WARN, "UCID Invalid!");
		}
	}
}

/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
