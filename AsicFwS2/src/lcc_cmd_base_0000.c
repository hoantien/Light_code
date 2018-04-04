/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    lcc_cmd_base_0000.c
 * @author  The LightCo
 * @version V1.0.0
 * @date    Mar-10-2016
 * @brief   This file contains expand of lcc_cmd_base_0000
 *
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "os.h"
#include "assert.h"
#include "log.h"
#include "i2cm.h"
#include "lcc_cmd.h"
#include "lcc_cmd_log.h"
#include "lcc_system.h"
#include "light_system.h"
#include "task_cam_ctrl.h"
#include "task_ccb_ctrl.h"
#include "optical.h"
#include "usecase.h"
#include "actuator.h"

/* Private defines------------------------------------------------------------*/
#define SLOGF_ID				SLOG_ID_LCC_CMD_BASE_0000

#define CAM_COMMAND_STATUS_LEN	4

#define I2C_RW_MASK				0x01
#define I2C_RW_READ				0x01
#define I2C_RW_WRITE			0x00
#define TIMEOUT					30

extern float i2c_write_delay_sec_asic1;
extern float i2c_write_delay_sec_asic23;
/* Exported functions --------------------------------------------------------*/
/**
 * cmd_cam_module_open
 * Handle camera open command
 */
void cmd_cam_module_open(lcc_cmd_t *in, lcc_cmd_t *out)
{
	lcc_cmd_tid_t p_cmd;
	cam_typedef_t *p_cam;
	lcc_cmd_info_t cmd = (lcc_cmd_info_t)(LCC_CMD_CAM_MODULE_OPEN);
	uint32_t m_bitmask = in->m_bitmask;
	uint8_t i, cam_idx, data_cnt = 0;
	if (CMD_READ == in->action)
	{
		out->data = pvPortMalloc(in->m_number*cmd.size);
		if (NULL == out->data)
		{
			SLOGF(SLOG_ERROR, "%s: Lack of memory", __FUNCTION__);
			in->status = LCC_CMD_UNSUCCESS;
			return;
		}
	}

	/* Scan camera bitmask */
	for (i = 0; i < in->m_number; i++)
	{
		cam_idx = __builtin_ctz(m_bitmask);
		m_bitmask &= ~(1 << cam_idx);
		/* Get camera object from camera object list */
		p_cam = idx_to_object(cam_idx);
		if (NULL == p_cam)	/* Verify object is valid */
		{
			in->status = LCC_CMD_ERROR_INVALID_MBITMASK;
			return;
		}

		/* Read command processing */
		if (CMD_READ == in->action)
		{
			/* TODO: Take semaphore to access settings memory */
			xSemaphoreTake(p_cam->semaphore, portMAX_DELAY);
			out->data[out->len++] = p_cam->settings->open;
			/* TODO: Release semaphore after read data */
			xSemaphoreGive(p_cam->semaphore);
		}
		/* Write command processing */
		else
		{
			if (CAM_MODULE_MODE_MAX > in->data[data_cnt])
			{
				SLOGF(SLOG_DEBUG, "Open CAM-%X request", p_cam->info.module);
				xSemaphoreTake(p_cam->semaphore, portMAX_DELAY);
				p_cam->settings->open = in->data[data_cnt];
				p_cmd.cmd_tid = in->tid;
				p_cmd.event = IMG_EVENT_CAM_OPEN;
				xSemaphoreGive(p_cam->semaphore);
				/* Raise a updating signal to camera task for this ASIC */
				if (light_system->m_filter & (1 << cam_idx))
				{
					xQueueSend(p_cam->queue, &p_cmd, (TickType_t)10);
				}
			}
			else
			{
				SLOGF(SLOG_WARN, "Open mode invalid!");
				in->status = LCC_CMD_INVALID_ARG;
			}
			/* Set data pointer when command was set for global */
			data_cnt += in->global ? 0 : cmd.size;
		}
	}
	/* The command status will be updated in either lcc_cmd task
	 * (read command) or task_cam_ctrl (write command)*/
}

extern uint8_t rx_end_flag;
/**
 * cmd_cam_streaming
 *
 */
void cmd_cam_streaming(lcc_cmd_t *in, lcc_cmd_t *out)
{
	lcc_cmd_tid_t p_cmd;
	uint8_t idx = 0;
	uint16_t data_idx = 0;
	uint8_t cam_idx = 0;
	uint32_t m_bitmask = in->m_bitmask;
	uint8_t control_status = 0;
	uint32_t cam_status = 0;
	uint32_t stream_on_msk = 0;
	uint8_t wait_asics = 0;
	cam_data_t	*cache;
	cam_typedef_t *pcam = NULL;
	lcc_cmd_info_t cmd = (lcc_cmd_info_t)(LCC_CMD_CAM_STREAMING);

	if (CMD_READ == in->action)
	{
		out->data = pvPortMalloc(in->m_number * cmd.size);
		if (NULL == out->data)
		{
			SLOGF(SLOG_ERROR, "%s: Lack of memory", __FUNCTION__);
			in->status = LCC_CMD_UNSUCCESS;
			return;
		}
	}

	for(idx = 0; idx < in->m_number; idx++)
	{
		cam_idx = __builtin_ctz(m_bitmask);
		m_bitmask &= ~(1 << cam_idx);
		pcam = idx_to_object(cam_idx);
		if(pcam)
		{
			if(CMD_READ == in->action)
			{
				xSemaphoreTake(pcam->semaphore, portMAX_DELAY);
				cache = pcam->settings;
				memcpy(&out->data[out->len], cache->stream, cmd.size);
				xSemaphoreGive(pcam->semaphore);
				out->len += cmd.size;
			}
			else
			{
				cache = pcam->settings;
				p_cmd.cmd_tid = in->tid;
				p_cmd.event = IMG_EVENT_CAM_STREAM;
				memcpy(&cam_status, cache->status, sizeof(uint32_t));
				control_status = in->data[data_idx + 0];
				uint8_t tx_ins = control_status & (BIT7 | BIT6 | BIT5 | BIT4);
				tx_ins = tx_ins >> 4;
				if(CSI_0 != tx_ins && CSI_1 != tx_ins)
				{
					SLOGF(SLOG_ERROR, "TX%d is not supported", tx_ins);
					continue;
				}
				uint32_t old_stream_cache = 0;
				memcpy(&old_stream_cache, cache->stream, cmd.size);
				memcpy(cache->stream, in->data + data_idx, cmd.size);

				if((control_status & CAM_STREAM_ENABLE) &&
					(light_system->active_ucid != UCID_DEBUG))
				{
					if(light_system->m_filter & (1 << cam_idx))
					{
						if(!(cam_status & S_MODULE_SW_STANDBY))
						{
							SLOGF(SLOG_ERROR, "CAM-%X not in SW standby",
															pcam->info.module);
							SLOGF(SLOG_ERROR, "Canceled stream on request");
							in->m_bitmask &= ~(light_system->m_filter);
							in->status = LCC_CMD_UNSUCCESS;
							memcpy(cache->stream, &old_stream_cache, cmd.size);
							return;
						}
						stream_on_msk |= (1 << cam_idx);
					}
					else
					{
						if(light_system->m_filter_asic1 & (1 << cam_idx))
						{
#if (ASIC_NUM == ASIC1)
							/* Indicate that ASIC1 waiting interrupt from
							 * ASIC2 in HDR capture mode */
							/*taskENTER_CRITICAL();*/
							light_system->wait_intr_stream |= ASIC_2;
							/*taskEXIT_CRITICAL();*/
#endif
							wait_asics |= ASIC_2;
						}
						if(light_system->m_filter_asic2 & (1 << cam_idx))
						{
#if (ASIC_NUM == ASIC1)
							/* Indicate that ASIC1 waiting interrupt from
							 * ASIC2 in HDR capture mode */
							/*taskENTER_CRITICAL();*/
							light_system->wait_intr_stream |= ASIC_3;
							/*taskEXIT_CRITICAL();*/
#endif
							wait_asics |= ASIC_3;
						}
					}
				}
				else
				{
					if (light_system->m_filter & (1 << cam_idx))
						xQueueSend(pcam->queue, &p_cmd, (TickType_t)10);
				}

				data_idx += in->global ? 0 : cmd.size;
			}
		}
	}

	if(stream_on_msk)
	{
		switch(light_system->active_ucid)
		{
			case UCID_PREVIEW:
			{
				int stream_off_bitmask = ~(stream_on_msk) & light_system->m_filter;
				cam_typedef_t *pcam = NULL;
				uint8_t cam_idx;
				uint32_t cs;
				for (uint8_t i = 0; __builtin_popcount(stream_off_bitmask) > 0; i++)
				{
					cam_idx = __builtin_ctz(stream_off_bitmask);
					stream_off_bitmask &= ~(1 << cam_idx);
					pcam = idx_to_object(cam_idx);
					if (pcam != NULL)
					{
						memcpy(&cs, pcam->settings->status, sizeof(uint32_t));
						if (cs & S_MODULE_STREAM_ON)
						{
							SLOGF(SLOG_ERROR, "Streaming off CAM-%X", pcam->info.module);
							cam_ctrl_stream(pcam);
						}
					}
				}
				while(rx_end_flag)
				{
					vTaskDelay(1);
				}
				ucid_preview_hdl(stream_on_msk, in->m_number);
				in->m_bitmask &= ~(light_system->m_filter);
				in->status = LCC_CMD_SUCCESS;
				break;
			}
			case UCID_HIRES_CAPTURE:
			{
				ucid_hires_hdl(stream_on_msk, in->m_number, wait_asics);
				in->status = LCC_CMD_SUCCESS;
#if (ASIC_NUM == ASIC1)
				in->m_bitmask = 0;
#else
				in->m_bitmask &= ~(light_system->m_filter);
#endif
				break;
			}
			default:
			{
				SLOGF(SLOG_ERROR, "Unsupported UCID %x", light_system->active_ucid);
#if (ASIC_NUM == ASIC1)
				in->m_bitmask = 0;
#else
				in->m_bitmask &= ~(light_system->m_filter);
#endif
				in->status = LCC_CMD_UNSUCCESS;
				break;
			}
		}
	}
	else
	{
#if (ASIC_NUM == ASIC1)
		if(light_system->active_ucid == UCID_HIRES_CAPTURE)
		{
			if(!wait_asics)
			{
				in->status = LCC_CMD_SUCCESS;
			}
			else if(wait_asics == wait_intr_signal(wait_asics, 1000, 1))
			{
				SLOGF(SLOG_INFO, "Sending interrupt signal");
				send_hw_sync_trigger();
				in->status = LCC_CMD_SUCCESS;
			}
			else
			{
				SLOGF(SLOG_ERROR, "Timeout waiting for interrupt signal");
				in->status = LCC_CMD_UNSUCCESS;
			}
		}
		else
		{
			in->status = LCC_CMD_UNSUCCESS;
		}
#endif
	}
	/* The command status will be updated in either lcc_cmd task
	 * (read command) or task_cam_ctrl (write command) */
}

/**
 * cmd_cam_burst_requested
 *
 */
void cmd_cam_burst_requested(lcc_cmd_t *in, lcc_cmd_t *out)
{
	lcc_cmd_info_t cmd = (lcc_cmd_info_t)(LCC_CMD_CAM_BURST_REQUESTED);
	uint16_t ucid = in->ucid;
	/* Read command processing */
	if (CMD_READ == in->action)
	{
		out->data = pvPortMalloc(cmd.size);
		if (NULL == out->data)
		{
			SLOGF(SLOG_ERROR, "%s: Lack of memory", __FUNCTION__);
			in->status = LCC_CMD_UNSUCCESS;
			return;
		}
		SLOGF(SLOG_DEBUG, "Requested burst %d",
									light_system->settings->burst[ucid].burst_requested);
		/* TODO: Take semaphore to access settings memory */
		memcpy(out->data, &light_system->settings->burst[ucid].burst_requested, cmd.size);
		/* TODO: Release semaphore after read data */
		out->len = cmd.size;
	}
	else
	{


		SLOGF(SLOG_DEBUG, "Requested burst %d", in->data[0]);
		/* TODO: Take semaphore to access settings memory */
		light_system->settings->burst[ucid].burst_requested = in->data[0];
		/* TODO: Release semaphore after read data */
		/* Update the command status */
		in->status = LCC_CMD_SUCCESS;
	}
}
/**
 * cmd_cam_burst_available
 *
 */
void cmd_cam_burst_available(lcc_cmd_t *in, lcc_cmd_t *out)
{
	lcc_cmd_info_t cmd = (lcc_cmd_info_t)(LCC_CMD_CAM_BURST_AVAILABLE);
	uint16_t ucid = in->ucid;
	/* Read command processing */
	if (CMD_READ == in->action)
	{
		out->data = pvPortMalloc(cmd.size);
		if (NULL == out->data)
		{
			SLOGF(SLOG_ERROR, "%s: Lack of memory", __FUNCTION__);
			in->status = LCC_CMD_UNSUCCESS;
			return;
		}
		SLOGF(SLOG_DEBUG, "Available burst %d",
									light_system->settings->burst[ucid].burst_available);
		/* TODO: Take semaphore to access settings memory */
		memcpy(out->data, &light_system->settings->burst[ucid].burst_available, cmd.size);
		/* TODO: Release semaphore after read data */
		out->len = cmd.size;
	}
}

/**
 * cmd_cam_burst_actual
 *
 */
void cmd_cam_burst_actual(lcc_cmd_t *in, lcc_cmd_t *out)
{
	lcc_cmd_info_t cmd = (lcc_cmd_info_t)(LCC_CMD_CAM_BURST_ACTUAL);
	uint16_t ucid = in->ucid;
	/* Read command processing */
	if (CMD_READ == in->action)
	{
		out->data = pvPortMalloc(cmd.size);
		if (NULL == out->data)
		{
			SLOGF(SLOG_ERROR, "%s: Lack of memory", __FUNCTION__);
			in->status = LCC_CMD_UNSUCCESS;
			return;
		}
		SLOGF(SLOG_DEBUG, "Actual burst %d",
									light_system->settings->burst[ucid].burst_actual);
		/* TODO: Take semaphore to access settings memory */
		memcpy(out->data, &light_system->settings->burst[ucid].burst_actual, cmd.size);
		/* TODO: Release semaphore after read data */
		out->len = cmd.size;
	}
}

/**
 * cmd_cam_snapshot_uuid
 *
 */
void cmd_cam_snapshot_uuid(lcc_cmd_t *in, lcc_cmd_t *out)
{
	lcc_cmd_info_t cmd = (lcc_cmd_info_t)(LCC_CMD_CAM_SNAPSHOT_UUID);

	/* Read command processing */
	if (CMD_READ == in->action)
	{
		out->data = pvPortMalloc(cmd.size);
		if (NULL == out->data)
		{
			SLOGF(SLOG_ERROR, "%s: Lack of memory", __FUNCTION__);
			in->status = LCC_CMD_UNSUCCESS;
			return;
		}
		/* TODO: Take semaphore to access settings memory */
		memcpy(out->data, light_system->settings->snapshot_uuid, cmd.size);
		/* TODO: Release semaphore after read data */
		out->len = cmd.size;
	}
	else
	{
		uint32_t suuid = 0;
		uint8_t *puuid = light_system->settings->snapshot_uuid;
		SLOGF(SLOG_DEBUG, "Received snapshot UUID");
		/* TODO: Take semaphore to access settings memory */
		memcpy(light_system->settings->snapshot_uuid, in->data, cmd.size);
		suuid = (puuid[3] << 24 | puuid[2] << 16 | puuid[1] << 8 | puuid[0]);
		SLOGF(SLOG_DEBUG, "SUUID: 0x%04x", (unsigned int)suuid);
		/* TODO: Release semaphore after read data */
		/* Update the command status */
		in->status = LCC_CMD_SUCCESS;
	}
}
/**
 * cmd_cam_snapshot_tid
 *
 */
void cmd_cam_snapshot_tid(lcc_cmd_t *in, lcc_cmd_t *out)
{
	cam_typedef_t *p_cam;
	lcc_cmd_info_t cmd = (lcc_cmd_info_t)(LCC_CMD_CAM_SNAPSHOT_TID);
	uint32_t m_bitmask = in->m_bitmask;
	uint8_t i, cam_idx, data_cnt = 0;

	if (CMD_READ == in->action)
	{
		out->data = pvPortMalloc(in->m_number*cmd.size);
		if (NULL == out->data)
		{
			SLOGF(SLOG_ERROR, "%s: Lack of memory", __FUNCTION__);
			in->status = LCC_CMD_UNSUCCESS;
			return;
		}
	}

	/* Scan camera bitmask */
	for (i = 0; i < in->m_number; i++)
	{
		cam_idx = __builtin_ctz(m_bitmask);
		m_bitmask &= ~(1 << cam_idx);
		/* Get camera object from camera object list */
		p_cam = idx_to_object(cam_idx);
		if (NULL == p_cam)	/* Verify object is valid */
		{
			in->status = LCC_CMD_ERROR_INVALID_MBITMASK;
			return;
		}

		/* Read command processing */
		if (CMD_READ == in->action)
		{
			/* TODO: Take semaphore to access settings memory */
			xSemaphoreTake(p_cam->semaphore, portMAX_DELAY);
			memcpy(&out->data[out->len], p_cam->settings->thumb_id, cmd.size);
			out->len += cmd.size;
			/* TODO: Release semaphore after read data */
			xSemaphoreGive(p_cam->semaphore);
		}
		/* Write command processing */
		else
		{
			/* TODO: Take semaphore to access settings memory */
			xSemaphoreTake(p_cam->semaphore, portMAX_DELAY);
			memcpy(p_cam->settings->thumb_id, &in->data[data_cnt], cmd.size);
			xSemaphoreGive(p_cam->semaphore);

			/* Set data pointer when command was set for global */
			data_cnt += in->global ? 0 : cmd.size;
			/* Update the command status */
			in->status = LCC_CMD_SUCCESS;
		}
	}
}
/**
 * cmd_cam_command_status
 *
 */
void cmd_cam_command_status(lcc_cmd_t *in, lcc_cmd_t *out)
{
	uint16_t id_query = 0;
	struct lcc_cmd_log_t *cmd_info = NULL;

	if (CMD_READ == in->action)
	{
	    /* Read command processing */
	}
	else
	{
	    /* Write command processing */
		out->data = pvPortMalloc(LCC_CMD_CAM_COMMAND_STATUS_READ_SIZE);
		if (NULL == out->data)
		{
			SLOGF(SLOG_ERROR, "%s: Lack of memory", __FUNCTION__);
			in->status = LCC_CMD_UNSUCCESS;
			return;
		}

		id_query = U16(in->data[0]);
		cmd_info = lcc_cmd_log_pull(id_query);
		if(cmd_info != NULL)
		{
			SLOGF(SLOG_DEBUG, "%s: CMD_TID %x is queried status %x",
				__FUNCTION__, id_query, cmd_info->status);

			memcpy(out->data, &cmd_info->status,
					LCC_CMD_CAM_COMMAND_STATUS_READ_SIZE);
			if(cmd_info->cmd_bitmask == 0)
			{
				lcc_cmd_log_delete_tid(cmd_info);
			}
			out->len = LCC_CMD_CAM_COMMAND_STATUS_READ_SIZE;
			cmd_info = NULL;
		}
		else
		{
			memset(out->data, 0xFF, LCC_CMD_CAM_COMMAND_STATUS_READ_SIZE);
			out->len = LCC_CMD_CAM_COMMAND_STATUS_READ_SIZE;
		}
		/* set Action to READ to send data to i2c buffer */
		in->action = CMD_READ;
	}
}

/**
 * cmd_cam_module_status
 *
 */
void cmd_cam_module_status(lcc_cmd_t *in, lcc_cmd_t *out)
{
	cam_typedef_t *p_cam;
	lcc_cmd_info_t cmd = (lcc_cmd_info_t)(LCC_CMD_CAM_MODULE_STATUS);
	uint32_t m_bitmask = in->m_bitmask;
	uint8_t i, cam_idx;

	out->data = pvPortMalloc(in->m_number * cmd.size);
	if (NULL == out->data)
	{
		SLOGF(SLOG_ERROR, "%s: Lack of memory", __FUNCTION__);
		in->status = LCC_CMD_UNSUCCESS;
		return;
	}

	/* Scan camera bitmask */
	for (i = 0; i < in->m_number; i++)
	{
		cam_idx = __builtin_ctz(m_bitmask);
		m_bitmask &= ~(1 << cam_idx);
		/* Get camera object from camera object list */
		p_cam = idx_to_object(cam_idx);
		if (NULL == p_cam)	/*Verify object is valid */
		{
			in->status = LCC_CMD_ERROR_INVALID_MBITMASK;
			return;
		}

		/* Take semaphore to access settings memory */
		xSemaphoreTake(p_cam->semaphore, portMAX_DELAY);
		memcpy(&out->data[out->len], p_cam->settings->status, cmd.size);
		/* Release semaphore after read data */
		xSemaphoreGive(p_cam->semaphore);
		out->len += cmd.size;
	}
}

/**
 * cmd_cam_module_resolution
 *
 */
void cmd_cam_module_resolution(lcc_cmd_t *in, lcc_cmd_t *out)
{
	lcc_cmd_tid_t p_cmd;
	cam_typedef_t *p_cam;
	lcc_cmd_info_t cmd = (lcc_cmd_info_t)(LCC_CMD_CAM_MODULE_RESOLUTION);
	uint32_t m_bitmask = in->m_bitmask;
	uint8_t i, cam_idx, data_cnt = 0;

	if (CMD_READ == in->action)
	{
		out->data = pvPortMalloc(in->m_number*cmd.size);
		if (NULL == out->data)
		{
			SLOGF(SLOG_ERROR, "%s: Lack of memory", __FUNCTION__);
			in->status = LCC_CMD_UNSUCCESS;
			return;
		}
	}

	/* Scan camera bitmask  */
	for (i = 0; i < in->m_number; i++)
	{
		cam_idx = __builtin_ctz(m_bitmask);
		m_bitmask &= ~(1 << cam_idx);
		/* Get camera object from camera object list */
		p_cam = idx_to_object(cam_idx);
		if (NULL == p_cam)	/*Verify object is valid */
		{
			in->status = LCC_CMD_ERROR_INVALID_MBITMASK;
			return;
		}

		/* Read command processing */
		if (CMD_READ == in->action)
		{
			/* TODO: Take semaphore to access settings memory */
			xSemaphoreTake(p_cam->image->semaphore, portMAX_DELAY);
			memcpy(&out->data[out->len],
					p_cam->image->settings[in->ucid]->resolution, cmd.size);
			out->len += cmd.size;
			/* TODO: Release semaphore after read data */
			xSemaphoreGive(p_cam->image->semaphore);
		}
		/* Write command processing */
		else
		{
			/* TODO: Take semaphore to access settings memory */
			xSemaphoreTake(p_cam->image->semaphore, portMAX_DELAY);
			p_cmd.cmd_tid = in->tid;
			p_cmd.event = IMG_EVENT_CAM_RESOLUTION;
			memcpy(p_cam->image->settings[in->ucid]->resolution,
											&in->data[data_cnt], cmd.size);
			/* TODO: Release semaphore after read data */
			xSemaphoreGive(p_cam->image->semaphore);

			/* Raise a updating signal to camera task for this ASIC */
			if (light_system->m_filter & (1 << cam_idx))
			{
				if (in->ucid == light_system->active_ucid)
					xQueueSend(p_cam->queue, &p_cmd, (TickType_t)10);
				else
				{
					in->status = LCC_CMD_SUCCESS;
					/* Reset cam bit_mask to update the command status */
					in->m_bitmask &= ~(1 << cam_idx);
				}
			}

			/* Set data pointer when command was set for global */
			data_cnt += in->global ? 0 : cmd.size;
		}
	}
	/* The command status will be updated in either lcc_cmd task
	 * (read command) or task_cam_ctrl (write command) */
}

/**
 * cmd_cam_module_sensitivity
 */
void cmd_cam_module_sensitivity(lcc_cmd_t *in, lcc_cmd_t *out)
{
	lcc_cmd_tid_t p_cmd;
	cam_typedef_t *p_cam;
	lcc_cmd_info_t cmd = (lcc_cmd_info_t)(LCC_CMD_CAM_MODULE_SENSITIVITY);
	uint32_t m_bitmask = in->m_bitmask;
	uint8_t i, cam_idx, data_cnt = 0;

	if (CMD_READ == in->action)
	{
		out->data = pvPortMalloc(in->m_number*cmd.size);
		if (NULL == out->data)
		{
			SLOGF(SLOG_ERROR, "%s: Lack of memory", __FUNCTION__);
			in->status = LCC_CMD_UNSUCCESS;
			return;
		}
	}

	/* Scan camera bitmask  */
	for (i = 0; i < in->m_number; i++)
	{
		cam_idx = __builtin_ctz(m_bitmask);
		m_bitmask &= ~(1 << cam_idx);
		/* Get camera object from camera object list */
		p_cam = idx_to_object(cam_idx);
		if (NULL == p_cam)	/*Verify object is valid */
		{
			in->status = LCC_CMD_INVALID_ARG;
			return;
		}

		/* Read command processing */
		if (CMD_READ == in->action)
		{
			/* TODO: Take semaphore to access settings memory */
			xSemaphoreTake(p_cam->image->semaphore, portMAX_DELAY);
			memcpy(&out->data[out->len],
					p_cam->image->settings[in->ucid]->sensitivity, cmd.size);
			out->len += cmd.size;
			/* TODO: Release semaphore after read data */
			xSemaphoreGive(p_cam->image->semaphore);
		}
		/* Write command processing */
		else
		{
			/* TODO: Take semaphore to access settings memory */
			p_cmd.cmd_tid = in->tid;
			p_cmd.event = IMG_EVENT_CAM_SENSITIVITY;
			xSemaphoreTake(p_cam->image->semaphore, portMAX_DELAY);
			memcpy(p_cam->image->settings[in->ucid]->sensitivity,
												&in->data[data_cnt], cmd.size);
			/* TODO: Release semaphore after read data */
			xSemaphoreGive(p_cam->image->semaphore);

			/* Raise a updating signal to camera task for this ASIC */
			if (light_system->m_filter & (1 << cam_idx))
			{
				if (in->ucid == light_system->active_ucid)
					xQueueSend(p_cam->queue, &p_cmd, (TickType_t)10);
				else
				{
					in->status = LCC_CMD_SUCCESS;
					/* Reset cam bit_mask to update the command status */
					in->m_bitmask &= ~(1 << cam_idx);
				}
			}

			/* Set data pointer when command was set for global */
			data_cnt += in->global ? 0 : cmd.size;
		}
	}
	/* The command status will be updated in either lcc_cmd task
	 * (read command) or task_cam_ctrl (write command) */
}

/**
 * cmd_cam_module_exposure_time
 *
 */
void cmd_cam_module_exposure_time(lcc_cmd_t *in, lcc_cmd_t *out)
{
	lcc_cmd_tid_t p_cmd;
	cam_typedef_t *p_cam;
	lcc_cmd_info_t cmd = (lcc_cmd_info_t)(LCC_CMD_CAM_MODULE_EXPOSURE_TIME);
	uint32_t m_bitmask = in->m_bitmask;
	uint8_t i, cam_idx, data_cnt = 0;

	if (CMD_READ == in->action)
	{
		out->data = pvPortMalloc(in->m_number*cmd.size);
		if (NULL == out->data)
		{
			SLOGF(SLOG_ERROR, "%s: Lack of memory", __FUNCTION__);
			in->status = LCC_CMD_UNSUCCESS;
			return;
		}
	}

	/* Scan camera bitmask  */
	for (i = 0; i < in->m_number; i++)
	{
		cam_idx = __builtin_ctz(m_bitmask);
		m_bitmask &= ~(1 << cam_idx);
		/* Get camera object from camera object list */
		p_cam = idx_to_object(cam_idx);
		if (NULL == p_cam)	/*Verify object is valid */
		{
			in->status = LCC_CMD_INVALID_ARG;
			return;
		}

		/* Read command processing */
		if (CMD_READ == in->action)
		{
			/* TODO: Take semaphore to access settings memory */
			xSemaphoreTake(p_cam->image->semaphore, portMAX_DELAY);
			memcpy(&out->data[out->len],
						p_cam->image->settings[in->ucid]->exposure, cmd.size);
			out->len += cmd.size;
			/* TODO: Release semaphore after read data */
			xSemaphoreGive(p_cam->image->semaphore);
		}
		/* Write command processing */
		else
		{
			SLOGF(SLOG_DEBUG, "Exposure setting request for CAM-%X",
														p_cam->info.module);
			p_cmd.cmd_tid = in->tid;
			p_cmd.event = IMG_EVENT_CAM_EXPOSURE_TIME;
			/* TODO: Take semaphore to access settings memory */
			memcpy(p_cam->image->settings[in->ucid]->exposure,
												&in->data[data_cnt], cmd.size);
			/* TODO: Release semaphore after read data */
			/* Raise a updating signal to camera task for this ASIC */
			if (light_system->m_filter & (1 << cam_idx))
			{
				if (in->ucid == light_system->active_ucid)
					xQueueSend(p_cam->queue, &p_cmd, (TickType_t)10);
				else
				{
					in->status = LCC_CMD_SUCCESS;
					/* Reset cam bit_mask to update the command status */
					in->m_bitmask &= ~(1 << cam_idx);
				}
			}

			/* Set data pointer when command was set for global */
			data_cnt += in->global ? 0 : cmd.size;
		}
	}
	/* The command status will be updated in either lcc_cmd task
	 * (read command) or task_cam_ctrl (write command) */
}

/**
 * cmd_cam_module_fps
 *
 */
void cmd_cam_module_fps(lcc_cmd_t *in, lcc_cmd_t *out)
{
	lcc_cmd_tid_t p_cmd;
	cam_typedef_t *p_cam;
	lcc_cmd_info_t cmd = (lcc_cmd_info_t)(LCC_CMD_CAM_MODULE_FPS);
	uint32_t m_bitmask = in->m_bitmask;
	uint8_t i, cam_idx, data_cnt = 0;

	if (CMD_READ == in->action)
	{
		out->data = pvPortMalloc(in->m_number*cmd.size);
		if (NULL == out->data)
		{
			SLOGF(SLOG_ERROR, "%s: Lack of memory", __FUNCTION__);
			in->status = LCC_CMD_UNSUCCESS;
			return;
		}
	}

	/* Scan camera bitmask  */
	for (i = 0; i < in->m_number; i++)
	{
		cam_idx = __builtin_ctz(m_bitmask);
		m_bitmask &= ~(1 << cam_idx);
		/* Get camera object from camera object list */
		p_cam = idx_to_object(cam_idx);
		if (NULL == p_cam)	/*Verify object is valid */
		{
			in->status = LCC_CMD_INVALID_ARG;
			return;
		}

		/* Read command processing */
		if (CMD_READ == in->action)
		{
			/* TODO: Take semaphore to access settings memory */
			xSemaphoreTake(p_cam->image->semaphore, portMAX_DELAY);
			out->data[out->len++] = p_cam->image->settings[in->ucid]->fps[0];
			out->data[out->len++] = p_cam->image->settings[in->ucid]->fps[1];
			/* TODO: Release semaphore after read data */
			xSemaphoreGive(p_cam->image->semaphore);
		}
		/* Write command processing */
		else
		{
			/* TODO: Take semaphore to access settings memory */
			p_cmd.cmd_tid = in->tid;
			p_cmd.event = IMG_EVENT_CAM_FPS;
			xSemaphoreTake(p_cam->image->semaphore, portMAX_DELAY);
			p_cam->image->settings[in->ucid]->fps[0] = in->data[data_cnt++];
			p_cam->image->settings[in->ucid]->fps[1] = in->data[data_cnt++];
			/* TODO: Release semaphore after read data */
			xSemaphoreGive(p_cam->image->semaphore);

			/* Raise a updating signal to camera task for this ASIC */
			if (light_system->m_filter & (1 << cam_idx))
			{
				if (in->ucid == light_system->active_ucid)
					xQueueSend(p_cam->queue, &p_cmd, (TickType_t)10);
				else
				{
					in->status = LCC_CMD_SUCCESS;
					/* Reset cam bit_mask to update the command status */
					in->m_bitmask &= ~(1 << cam_idx);
				}
			}

			/* Set data pointer when command was set for global */
			if (in->global)
				data_cnt = 0;
		}
	}
	/* The command status will be updated in either lcc_cmd task
	 * (read command) or task_cam_ctrl (write command) */}

/**
 * cmd_cam_module_focal_length
 *
 */
void cmd_cam_module_focal_length(lcc_cmd_t *in, lcc_cmd_t *out)
{
	cam_typedef_t *p_cam;
	lcc_cmd_info_t cmd = (lcc_cmd_info_t)(LCC_CMD_CAM_MODULE_FOCAL_LENGTH);
	uint32_t m_bitmask = in->m_bitmask;
	uint8_t i, cam_idx;

	out->data = pvPortMalloc(in->m_number*cmd.size);
	if (NULL == out->data)
	{
		SLOGF(SLOG_ERROR, "%s: Lack of memory", __FUNCTION__);
		in->status = LCC_CMD_UNSUCCESS;
		return;
	}

	/* Scan camera bitmask  */
	for (i = 0; i < in->m_number; i++)
	{
		cam_idx = __builtin_ctz(m_bitmask);
		m_bitmask &= ~(1 << cam_idx);
		/* Get camera object from camera object list */
		p_cam = idx_to_object(cam_idx);
		if (NULL == p_cam)	/*Verify object is valid */
		{
			in->status = LCC_CMD_INVALID_ARG;
			return;
		}

		/* TODO: Take semaphore to access settings memory */
		xSemaphoreTake(p_cam->optical->semaphore, portMAX_DELAY);
		out->data[out->len++] = p_cam->optical->settings->focal_length[0];
		out->data[out->len++] = p_cam->optical->settings->focal_length[1];

		/* TODO: Release semaphore after read data */
		xSemaphoreGive(p_cam->optical->semaphore);
	}
}
/**
 * cmd_cam_module_focus_distance
 *
 */
void cmd_cam_module_focus_distance(lcc_cmd_t *in, lcc_cmd_t *out)
{
	lcc_cmd_tid_t p_cmd;
	cam_typedef_t *p_cam;
	lcc_cmd_info_t cmd = (lcc_cmd_info_t)(LCC_CMD_CAM_MODULE_FOCUS_DISTANCE);
	uint32_t m_bitmask = in->m_bitmask;
	uint8_t i, cam_idx, data_cnt = 0;

	if (CMD_READ == in->action)
	{
		out->data = pvPortMalloc(in->m_number * cmd.size);
		if (NULL == out->data)
		{
			SLOGF(SLOG_ERROR, "%s: Lack of memory", __FUNCTION__);
			in->status = LCC_CMD_UNSUCCESS;
			return;
		}
	}

	/* Scan camera bitmask */
	for (i = 0; i < in->m_number; i++)
	{
		cam_idx = __builtin_ctz(m_bitmask);
		m_bitmask &= ~(1 << cam_idx);
		/* Get camera object from camera object list */
		p_cam = idx_to_object(cam_idx);
		if (NULL == p_cam)	/*Verify object is valid */
		{
			in->status = LCC_CMD_INVALID_ARG;
			return;
		}

		/* Read command processing */
		if (CMD_READ == in->action)
		{
			/* Take semaphore to access settings memory */
			xSemaphoreTake(p_cam->optical->semaphore, portMAX_DELAY);
			memcpy(&out->data[out->len],
					p_cam->optical->settings->focus_distance, cmd.size);
			/* Release semaphore after reading data */
			xSemaphoreGive(p_cam->optical->semaphore);
			out->len += cmd.size;
		}
		/* Write command processing */
		else
		{
			p_cmd.cmd_tid = in->tid;
			/* Raise a updating signal to camera task for this ASIC */
			if (light_system->m_filter & (1 << cam_idx))
			{
				if(p_cam->info.grp == GRP_A)
				{
					vcm_t *vcm = p_cam->optical->lens;
					/* Take semaphore to access settings memory */
					xSemaphoreTake(vcm->semaphore, portMAX_DELAY);
					if (vcm->moving == FALSE)
					{
						/* set moving status */
						vcm->moving = TRUE;
						/* Release semaphore after accessing data */
						xSemaphoreGive(vcm->semaphore);
						/* Take semaphore to access settings memory */
						xSemaphoreTake(p_cam->optical->semaphore, portMAX_DELAY);
						memcpy(p_cam->optical->settings->focus_distance,
								&in->data[data_cnt], cmd.size);
						/* Release semaphore after writing data */
						xSemaphoreGive(p_cam->optical->semaphore);
						p_cmd.event = IMG_EVENT_VCM_DISTANCE;
						xQueueSend(p_cam->queue, &p_cmd, (TickType_t)10);
					}
					else
					{
						/* Release semaphore after accessing data */
						xSemaphoreGive(vcm->semaphore);
						SLOGF(SLOG_ERROR, "CAM-%X : VCM is moving",
								p_cam->info.module);
						in->status = LCC_CMD_UNSUCCESS;
						return;
					}
				}
				else
				{
					actuator_t *plens = (actuator_t *)p_cam->optical->lens;
					/* Take semaphore to access settings memory */
					xSemaphoreTake(plens->semaphore, portMAX_DELAY);
					if (plens->moving == FALSE)
					{
						/* set moving status */
						plens->moving = TRUE;
						/* Release semaphore after accessing data */
						xSemaphoreGive(plens->semaphore);
						/* Take semaphore to access settings memory */
						xSemaphoreTake(p_cam->optical->semaphore, portMAX_DELAY);
						memcpy(p_cam->optical->settings->focus_distance,
								&in->data[data_cnt], cmd.size);
						/* Release semaphore after writing data */
						xSemaphoreGive(p_cam->optical->semaphore);
						p_cmd.event = OPT_EVENT_FOCUS_DISTANCE;
						xQueueSend(p_cam->queue, &p_cmd, (TickType_t)10);
					}
					else
					{
						/* Release semaphore after accessing data */
						xSemaphoreGive(plens->semaphore);
						SLOGF(SLOG_ERROR, "CAM-%X : Lens is moving",
								p_cam->info.module);
						in->status = LCC_CMD_UNSUCCESS;
						return;
					}
				}
			}

			/* Set data pointer when command was not set for global */
			data_cnt += in->global ? 0 : cmd.size;
		}
	}
	/* The command status will be updated in either lcc_cmd task
	 * (read command) or task_cam_ctrl (write command) */
}

/**
 * cmd_cam_module_lens_position
 *
 */
void cmd_cam_module_lens_position(lcc_cmd_t *in, lcc_cmd_t *out)
{
	lcc_cmd_tid_t p_cmd;
	cam_typedef_t *p_cam;
	lcc_cmd_info_t cmd = (lcc_cmd_info_t)(LCC_CMD_CAM_MODULE_LENS_POSITION);
	uint32_t m_bitmask = in->m_bitmask;
	uint8_t i, cam_idx, data_cnt = 0;

	if (CMD_READ == in->action)
	{
		out->data = pvPortMalloc(in->m_number * cmd.size);
		if (NULL == out->data)
		{
			SLOGF(SLOG_ERROR, "%s: Lack of memory", __FUNCTION__);
			in->status = LCC_CMD_UNSUCCESS;
			return;
		}
	}

	/* Scan camera bitmask */
	for (i = 0; i < in->m_number; i++)
	{
		cam_idx = __builtin_ctz(m_bitmask);
		m_bitmask &= ~(1 << cam_idx);
		/* Get camera object from camera object list */
		p_cam = idx_to_object(cam_idx);
		if (NULL == p_cam)	/*Verify object is valid */
		{
			in->status = LCC_CMD_INVALID_ARG;
			return;
		}

		/* Read command processing */
		if (CMD_READ == in->action)
		{
			/* Take semaphore to access settings memory */
			xSemaphoreTake(p_cam->optical->semaphore, portMAX_DELAY);
			memcpy(&out->data[out->len],
					p_cam->optical->settings->lens_position, cmd.size);
			/* Release semaphore after reading data */
			xSemaphoreGive(p_cam->optical->semaphore);
			out->len += cmd.size;
		}
		/* Write command processing */
		else
		{
			p_cmd.cmd_tid = in->tid;
			/* Take semaphore to access settings memory */
			xSemaphoreTake(p_cam->optical->semaphore, portMAX_DELAY);
			memcpy(p_cam->optical->settings->lens_position,
					&in->data[data_cnt], cmd.size);
			/* Release semaphore after writing data */
			xSemaphoreGive(p_cam->optical->semaphore);
			/* Raise a updating signal to camera task for this ASIC */
			if (light_system->m_filter & (1 << cam_idx))
			{
				if(p_cam->info.grp == GRP_A)
				{
					vcm_t *vcm = p_cam->optical->lens;
					/* Take semaphore to access settings memory */
					xSemaphoreTake(vcm->semaphore, portMAX_DELAY);
					if (vcm->moving == FALSE)
					{
						/* set moving status */
						vcm->moving = TRUE;
						/* Release semaphore after accessing data */
						xSemaphoreGive(vcm->semaphore);

						p_cmd.event = IMG_EVENT_VCM_POSITION;
						xQueueSend(p_cam->queue, &p_cmd, (TickType_t)10);
					}
					else
					{
						/* Release semaphore after accessing data */
						xSemaphoreGive(vcm->semaphore);
						SLOGF(SLOG_ERROR, "CAM-%X : VCM is moving",
								p_cam->info.module);
						in->status = LCC_CMD_UNSUCCESS;
						return;
					}
				}
				else
				{
					actuator_t *plens = (actuator_t *)p_cam->optical->lens;
					/* Take semaphore to access settings memory */
					xSemaphoreTake(plens->semaphore, portMAX_DELAY);
					if (plens->moving == FALSE)
					{
						/* set moving status */
						plens->moving = TRUE;
						/* Release semaphore after accessing data */
						xSemaphoreGive(plens->semaphore);
						/* Take semaphore to access settings memory */
						xSemaphoreTake(p_cam->optical->semaphore, portMAX_DELAY);
			            // FIXME set tolerance from command
			            p_cam->optical->settings->lens_position_tolerance = 2;
						/* Release semaphore after writing data */
						xSemaphoreGive(p_cam->optical->semaphore);
						p_cmd.event = OPT_EVENT_LENS_POSITION;
						xQueueSend(plens->queue, &p_cmd, (TickType_t)10);
					}
					else
					{
						/* Release semaphore after accessing data */
						xSemaphoreGive(plens->semaphore);
						SLOGF(SLOG_ERROR, "CAM-%X : Lens is moving",
								p_cam->info.module);
						in->status = LCC_CMD_UNSUCCESS;
						return;
					}
				}
			}

			/* Set data pointer when command was not set for global */
			data_cnt += in->global ? 0 : cmd.size;
		}
	}
	/* The command status will be updated in either lcc_cmd task
	 * (read command) or task_cam_ctrl (write command) */
}

/**
 * cmd_cam_module_mirror_position
 *
 */
void cmd_cam_module_mirror_position(lcc_cmd_t *in, lcc_cmd_t *out)
{
	lcc_cmd_tid_t p_cmd;
	cam_typedef_t *p_cam;
	lcc_cmd_info_t cmd = (lcc_cmd_info_t)(LCC_CMD_CAM_MODULE_MIRROR_POSITION);
	uint32_t m_bitmask = in->m_bitmask;
	uint8_t i, cam_idx, data_cnt = 0;

	if (CMD_READ == in->action)
	{
		out->data = pvPortMalloc(in->m_number * cmd.size);
		if (NULL == out->data)
		{
			SLOGF(SLOG_ERROR, "%s: Lack of memory", __FUNCTION__);
			in->status = LCC_CMD_UNSUCCESS;
			return;
		}
	}

	/* Scan camera bitmask */
	for (i = 0; i < in->m_number; i++)
	{
		cam_idx = __builtin_ctz(m_bitmask);
		m_bitmask &= ~(1 << cam_idx);
		/* Get camera object from camera object list */
		p_cam = idx_to_object(cam_idx);
		if (NULL == p_cam)	/*Verify object is valid */
		{
			in->status = LCC_CMD_INVALID_ARG;
			return;
		}

		/* Read command processing */
		if (CMD_READ == in->action)
		{
			/* Take semaphore to access settings memory */
			xSemaphoreTake(p_cam->optical->semaphore, portMAX_DELAY);
			memcpy(&out->data[out->len],
					p_cam->optical->settings->mirr_position, cmd.size);
			/* Release semaphore after reading data */
			xSemaphoreGive(p_cam->optical->semaphore);
			out->len += cmd.size;
		}
		/* Write command processing */
		else
		{
			p_cmd.cmd_tid = in->tid;
			p_cmd.event = OPT_EVENT_MIRROR_POSITION;
			/* Raise a updating signal to camera task for this ASIC */
			if ((p_cam->info.grp != GRP_A) &&
				(p_cam->info.module != CAM_B4) &&
				(p_cam->info.module != CAM_C5) &&
				(p_cam->info.module != CAM_C6))
			{
				/* Take semaphore to access settings memory */
				xSemaphoreTake(p_cam->optical->semaphore, portMAX_DELAY);
				memcpy(p_cam->optical->settings->mirr_position,
					&in->data[data_cnt], cmd.size);
				/* Release semaphore after writing data */
				xSemaphoreGive(p_cam->optical->semaphore);
				if(light_system->m_filter & (1 << cam_idx))
				{
					actuator_t *pmirr = (actuator_t *)p_cam->optical->mirr;
					/* Take semaphore to access settings memory */
					xSemaphoreTake(pmirr->semaphore, portMAX_DELAY);
					if (pmirr->moving == FALSE)
					{
						/* set moving status */
						pmirr->moving = TRUE;
						/* Release semaphore after accessing data */
						xSemaphoreGive(pmirr->semaphore);
						xQueueSend(pmirr->queue, &p_cmd, (TickType_t)10);
					}
					else
					{
						/* Release semaphore after accessing data */
						xSemaphoreGive(pmirr->semaphore);
						SLOGF(SLOG_ERROR, "CAM-%X : Mirror is moving",
								p_cam->info.module);
						in->status = LCC_CMD_UNSUCCESS;
						return;
					}
				}
			}
			else
			{
				in->status = LCC_CMD_SUCCESS;
				/* Reset bit of CAM if it have not MIRROR */
				in->m_bitmask &= ~(object_to_cam_bitmask(p_cam));
			}

			/* Set data pointer when command was not set for global */
			data_cnt += in->global ? 0 : cmd.size;
		}
	}
	/* The command status will be updated in either lcc_cmd task
	 * (read command) or task_cam_ctrl (write command) */
}
/**
 * cmd_cam_roi
 *
 */
void cmd_cam_roi(lcc_cmd_t *in, lcc_cmd_t *out)
{
	uint16_t data_cnt = 0;
	SLOGF(SLOG_DEBUG, "Entering %s", __FUNCTION__);

	/* Read command processing */
	if (CMD_READ == in->action)
	{
		/* Read data from cache */
		uint8_t roi_cnt_af = light_system->settings->roi_count_af;
		uint8_t roi_cnt_ae = light_system->settings->roi_count_ae;
		out->len = CAM_ROI_DATA_SIZE * (roi_cnt_af + roi_cnt_ae);
		if(0 == out->len)
		{
			SLOGF(SLOG_DEBUG, "ROI hasn't been set");
			in->status = LCC_CMD_UNSUCCESS;
			return;
		}
		out->data = pvPortMalloc(out->len);
		if (NULL == out->data)
		{
			SLOGF(SLOG_ERROR, "%s: Lack of memory", __FUNCTION__);
			in->status = LCC_CMD_UNSUCCESS;
			return;
		}
		SLOGF(SLOG_DEBUG, "Get ROI settings");
		/* TODO: Take semaphore to access settings memory */
		/* Get af ROI data */
		uint8_t n_bytes = CAM_ROI_DATA_SIZE * roi_cnt_af;
		memcpy(out->data, light_system->settings->roi_af, n_bytes);
		/* Get ae ROI data */
		n_bytes = CAM_ROI_DATA_SIZE * roi_cnt_ae;
		memcpy(out->data + n_bytes, light_system->settings->roi_ae, n_bytes);
#if(LOG_VERBOSE == STD_ON)
		uint8_t *pd = out->data;
		SLOGF(SLOG_INFO, "AF ROI data");
		for(int i = 0; i < roi_cnt_af; i++)
		{
			SLOGF(SLOG_DEBUG,
				"ROI: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x",
				pd[0], pd[1], pd[2], pd[3], pd[4], pd[5], pd[6], pd[7],
				pd[8], pd[9]);
			pd += CAM_ROI_DATA_SIZE;
		}
		SLOGF(SLOG_INFO, "AE ROI data");
		pd = light_system->settings->roi_ae;
		for(int i = 0; i < roi_cnt_ae; i++)
		{
			SLOGF(SLOG_DEBUG,
				"ROI: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x",
				pd[0], pd[1], pd[2], pd[3], pd[4], pd[5], pd[6], pd[7],
				pd[8], pd[9]);
			pd += CAM_ROI_DATA_SIZE;
		}
#endif
		/* TODO: Release semaphore after read data */
	}
	/* Write command processing */
	else
	{
		/* Write data to cache */
		uint8_t roi_cnt_af = in->data[data_cnt++];
		uint8_t roi_cnt_ae = in->data[data_cnt++];
		SLOGF(SLOG_DEBUG, "Set ROI cnt_af: %d cnt_ae: %d", roi_cnt_af, roi_cnt_ae);
		if(in->len != (CAM_ROI_DATA_SIZE * (roi_cnt_af + roi_cnt_ae) + 2))
		{
			SLOGF(SLOG_ERROR, "%s:%d Data Length is invalid %d",
					__FUNCTION__, __LINE__, in->len);
			in->status = LCC_CMD_INVALID_ARG;
			return;
		}
		else if(CAM_ROI_DATA_MAX_COUNT < roi_cnt_af
				|| CAM_ROI_DATA_MAX_COUNT < roi_cnt_ae)
		{
			SLOGF(SLOG_ERROR, "%s:%d Number of ROI data is too large %d",
					__FUNCTION__, __LINE__, (roi_cnt_af + roi_cnt_ae));
			in->status = LCC_CMD_INVALID_ARG;
			return;
		}
		else
		{
			/* TODO: Take semaphore to access settings memory */
			SLOGF(SLOG_DEBUG, "roi_cnt_af: %d, offset: %d", roi_cnt_af, data_cnt);
			light_system->settings->roi_count_af = roi_cnt_af;
			uint8_t n_bytes = roi_cnt_af * CAM_ROI_DATA_SIZE;
			memcpy(light_system->settings->roi_af, in->data+data_cnt, n_bytes);
			data_cnt += n_bytes;
			SLOGF(SLOG_DEBUG, "roi_cnt_ae: %d, offset: %d", roi_cnt_ae, data_cnt);
			n_bytes = roi_cnt_ae * CAM_ROI_DATA_SIZE;
			light_system->settings->roi_count_ae = roi_cnt_ae;
			memcpy(light_system->settings->roi_ae, in->data+data_cnt, n_bytes);
			/* TODO: Release semaphore after read data */
			/* Queue request to process received data */
	        lcc_cmd_tid_t p_cmd;
	        p_cmd.cmd_tid = in->tid;
	        p_cmd.event = AF_EVENT_AFAE;
	        if (xQueueSend(light_system->af_ctrl->queue, &p_cmd, (TickType_t)10) == pdPASS)
	            in->status = LCC_CMD_PENDING;
	        else
	        {
	            SLOGF(SLOG_ERROR, "Failed to queue AF command");
	            in->status = LCC_CMD_UNSUCCESS;
	        }
		}
	}

	SLOGF(SLOG_DEBUG, "Exiting %s", __FUNCTION__);
	/* The command status will be updated in either lcc_cmd task
	 * (read command) or task_cam_ctrl (write command) */
}
/**
 * cmd_cam_roi_transfer
 *
 */
void cmd_cam_roi_transfer(lcc_cmd_t *in, lcc_cmd_t *out)
{
    cam_typedef_t *p_cam;
    //lcc_cmd_info_t cmd = (lcc_cmd_info_t)(LCC_CMD_CAM_ROI_TRANSFER);
    uint32_t m_bitmask = in->m_bitmask;
    uint8_t i, cam_idx, data_cnt = 0;

    SLOGF(SLOG_DEBUG, "Entering %s", __FUNCTION__);

    /* Only write supported */
    assert(CMD_WRITE == in->action);

    roi_transfer_t roi_transfer;
    SLOGF(SLOG_DEBUG, "Set ROI transfer");
    uint8_t n_bytes = sizeof(roi_transfer.distance_mm);
    memcpy(&roi_transfer.distance_mm, in->data+data_cnt, n_bytes);
    data_cnt += n_bytes;
    n_bytes = sizeof(roi_transfer.ref_mod_idx);
    memcpy(&roi_transfer.ref_mod_idx, in->data+data_cnt, n_bytes);
    data_cnt += n_bytes;
    n_bytes = sizeof(roi_transfer.ref_mod_roi);
    memcpy(&roi_transfer.ref_mod_roi, in->data+data_cnt, n_bytes);
    data_cnt += n_bytes;

    /* TODO: Take semaphore to access settings memory */
    light_system->settings->roi_transfer  = roi_transfer;
    /* TODO: Release semaphore after read data */

    SLOGF(SLOG_DEBUG, "ROI transfer distance: %u, ref_mod_idx: %u, roi_left_x: %u, roi_top_y: %u, widht: %u, height: %u",
          (unsigned int)roi_transfer.distance_mm, (unsigned int)roi_transfer.ref_mod_idx,
          (unsigned int)roi_transfer.ref_mod_roi.left_x, (unsigned int)roi_transfer.ref_mod_roi.top_y,
          (unsigned int)roi_transfer.ref_mod_roi.width, (unsigned int)roi_transfer.ref_mod_roi.height);

    uint32_t events_to_set = 0;
    /* Scan camera bitmask */
    for (i = 0; i < in->m_number; i++)
    {
        cam_idx = __builtin_ctz(m_bitmask);
        m_bitmask &= ~(1 << cam_idx);
        /* Get camera object from camera object list */
        p_cam = idx_to_object(cam_idx);
        if (NULL == p_cam)  /*Verify object is valid */
        {
            in->status = LCC_CMD_ERROR_INVALID_MBITMASK;
            return;
        }

        SLOGF(SLOG_DEBUG, "Set ROI transfer for CAM-%X", p_cam->info.module);
        if (light_system->m_filter & (1 << cam_idx))
        {
            events_to_set |= (AF_EVENT_ROI_TRANSFER_CH0 << (p_cam->info.ch - 1));
        }

        /* Update command status */
        in->status = LCC_CMD_SUCCESS;
    }
    if (events_to_set != 0)
    {
        lcc_cmd_tid_t p_cmd;
        p_cmd.cmd_tid = in->tid;
        p_cmd.event = events_to_set;
        if (xQueueSend(light_system->af_ctrl->queue, &p_cmd, (TickType_t)10) == pdPASS)
            in->status = LCC_CMD_PENDING;
        else
        {
            SLOGF(SLOG_ERROR, "Failed to queue AF command");
            in->status = LCC_CMD_UNSUCCESS;
        }
    }

    SLOGF(SLOG_DEBUG, "Exiting %s", __FUNCTION__);
}
/**
 * cmd_roi_calibration
 *
 */
void cmd_cam_roi_calibration(lcc_cmd_t *in, lcc_cmd_t *out)
{
	cam_typedef_t *p_cam;
	lcc_cmd_info_t cmd = (lcc_cmd_info_t)(LCC_CMD_CAM_ROI_CALIBRATION);
	uint32_t m_bitmask = in->m_bitmask;
	uint8_t i, cam_idx, data_cnt = 0;

	SLOGF(SLOG_DEBUG, "Entering %s", __FUNCTION__);

	/* Read command processing */
	if (CMD_READ == in->action && in->m_number != 0)
	{
		out->data = pvPortMalloc(in->m_number * cmd.size);
		if (NULL == out->data)
		{
			SLOGF(SLOG_ERROR, "%s: Lack of memory", __FUNCTION__);
			in->status = LCC_CMD_UNSUCCESS;
			return;
		}
	}

	uint32_t events_to_set = 0;
	/* Scan camera bitmask */
	for (i = 0; i < in->m_number; i++)
	{
		cam_idx = __builtin_ctz(m_bitmask);
		m_bitmask &= ~(1 << cam_idx);
		/* Get camera object from camera object list */
		p_cam = idx_to_object(cam_idx);
		if (NULL == p_cam)	/*Verify object is valid */
		{
			in->status = LCC_CMD_ERROR_INVALID_MBITMASK;
			return;
		}

		/* Read command processing */
		if (CMD_READ == in->action)
		{
			SLOGF(SLOG_DEBUG, "Get ROI calibration of CAM-%X",
								p_cam->info.module);
			/* Take semaphore to access settings memory */
			xSemaphoreTake(p_cam->semaphore, portMAX_DELAY);
			memcpy(&out->data[out->len], &p_cam->cam_common_data.roi_rectangle,
					cmd.size);
			/* Release semaphore after read data */
			xSemaphoreGive(p_cam->semaphore);
			out->len += cmd.size;
#if(LOG_VERBOSE == STD_ON)
			SLOGF(SLOG_DEBUG,
				"ROI calib: %u, %u, %u, %u",
				(unsigned int)p_cam->cam_common_data.roi_rectangle.left_x,
                (unsigned int)p_cam->cam_common_data.roi_rectangle.top_y,
                (unsigned int)p_cam->cam_common_data.roi_rectangle.width,
                (unsigned int)p_cam->cam_common_data.roi_rectangle.height);
#endif
		}
		/* Write command processing */
		else
		{
			SLOGF(SLOG_DEBUG, "Set ROI calibration for CAM-%X",
					p_cam->info.module);
			/* Take semaphore to access settings memory */
			xSemaphoreTake(p_cam->semaphore, portMAX_DELAY);
			memcpy(&p_cam->cam_common_data.roi_rectangle,
					&in->data[data_cnt], cmd.size);
			/* Release semaphore after read data */
			xSemaphoreGive(p_cam->semaphore);
            if (light_system->m_filter & (1 << cam_idx))
            {
                events_to_set |= (AF_EVENT_FULL_SWEEP_CH0 << (p_cam->info.ch - 1));
            }

			/* Set data pointer when command was set for global */
			data_cnt += in->global ? 0 : cmd.size;

			/* Update command status */
			in->status = LCC_CMD_SUCCESS;
		}
	}
	if (events_to_set != 0)
	{
        lcc_cmd_tid_t p_cmd;
        p_cmd.cmd_tid = in->tid;
        p_cmd.event = events_to_set;
        if (xQueueSend(light_system->af_ctrl->queue, &p_cmd, (TickType_t)10) == pdPASS)
            in->status = LCC_CMD_PENDING;
        else
        {
            SLOGF(SLOG_ERROR, "Failed to queue AF command");
            in->status = LCC_CMD_UNSUCCESS;
        }
	}

	SLOGF(SLOG_DEBUG, "Exiting %s", __FUNCTION__);
}
/**
 * cmd_cam_module_uuid
 *
 */
void cmd_cam_module_uuid(lcc_cmd_t *in, lcc_cmd_t *out)
{
	cam_typedef_t *p_cam;
	lcc_cmd_info_t cmd = (lcc_cmd_info_t)(LCC_CMD_CAM_MODULE_UUID);
	uint32_t m_bitmask = in->m_bitmask;
	uint8_t i, cam_idx;

	out->data = pvPortMalloc(in->m_number * cmd.size);
	if (NULL == out->data)
	{
		SLOGF(SLOG_ERROR, "%s: Lack of memory", __FUNCTION__);
		in->status = LCC_CMD_UNSUCCESS;
		return;
	}

	/* Scan camera bitmask */
	for (i = 0; i < in->m_number; i++)
	{
		cam_idx = __builtin_ctz(m_bitmask);
		m_bitmask &= ~(1 << cam_idx);
		/* Get camera object from camera object list */
		p_cam = idx_to_object(cam_idx);
		if (NULL == p_cam)	/*Verify object is valid */
		{
			in->status = LCC_CMD_INVALID_ARG;
			return;
		}

		/* Take semaphore to access settings memory */
		xSemaphoreTake(p_cam->semaphore, portMAX_DELAY);
		memcpy(&out->data[out->len], p_cam->settings->uuid, cmd.size);
		/* Release semaphore after read data */
		xSemaphoreGive(p_cam->semaphore);
		out->len += cmd.size;
	}
}
/**
 * cmd_cam_module_type
 *
 */
void cmd_cam_module_type(lcc_cmd_t *in, lcc_cmd_t *out)
{
	cam_typedef_t *p_cam;
	lcc_cmd_info_t cmd = (lcc_cmd_info_t)(LCC_CMD_CAM_MODULE_TYPE);
	uint32_t m_bitmask = in->m_bitmask;
	uint8_t i, cam_idx;

	out->data = pvPortMalloc(in->m_number * cmd.size);
	if (NULL == out->data)
	{
		SLOGF(SLOG_ERROR, "%s: Lack of memory", __FUNCTION__);
		in->status = LCC_CMD_UNSUCCESS;
		return;
	}

	/* Scan camera bitmask */
	for (i = 0; i < in->m_number; i++)
	{
		cam_idx = __builtin_ctz(m_bitmask);
		m_bitmask &= ~(1 << cam_idx);
		/* Get camera object from camera object list */
		p_cam = idx_to_object(cam_idx);
		if (NULL == p_cam)	/*Verify object is valid */
		{
			in->status = LCC_CMD_INVALID_ARG;
			return;
		}

		/* Take semaphore to access settings memory */
		xSemaphoreTake(p_cam->semaphore, portMAX_DELAY);
		out->data[out->len] = p_cam->settings->type;
		/* Release semaphore after read data */
		xSemaphoreGive(p_cam->semaphore);
		out->len += cmd.size;
	}
}

void cmd_i2c_forward(lcc_cmd_t *in, lcc_cmd_t *out)
{
	static i2cm_error_t i2c_status = 0;
	static uint8_t *i2c_rx_data = NULL;
	static uint16_t rx_len = 0;
	uint8_t rw_mode = 0;
	uint8_t slave_addr = 0;
	uint8_t chid = 0;
	uint8_t *i2c_tx_data = NULL;
	uint16_t data_len = 0;
	uint16_t tx_len = 0;
	uint8_t data_ptr = 0;

	/* Write command processing */
	if (CMD_WRITE == in->action)
	{
		if(in->len > 2)
		{
			rw_mode = in->data[data_ptr] & I2C_RW_MASK;
			data_ptr++;
			chid = in->data[data_ptr++];
			slave_addr = in->data[data_ptr++];

			if(rw_mode == I2C_RW_WRITE)
			{
				tx_len = in->data[data_ptr++];
				data_len = in->len - data_ptr;
				if(data_len == tx_len)
				{
					assert_malloc(i2c_tx_data,tx_len * sizeof(uint8_t));
					i2c_rx_data = NULL;
					rx_len = 0;
				}
				else
				{
					SLOGF(SLOG_ERROR, "%s: Invalid input", __FUNCTION__);
					in->status = LCC_CMD_UNSUCCESS;
					return;
				}
			}
			else
			{
				rx_len = in->data[data_ptr++];
				tx_len = in->len - data_ptr;

				if(tx_len > 0 && tx_len < 3)
				{
					while(i2c_rx_data != NULL)
					{
						if(i2c_status)
						{
							vPortFree(i2c_rx_data);
							i2c_rx_data = NULL;
							break;
						}
						vTaskDelay(1);
					}

					assert_malloc(i2c_tx_data, tx_len * sizeof(uint8_t));
					assert_malloc(i2c_rx_data, tx_len * sizeof(uint8_t));
				}
				else
				{
					SLOGF(SLOG_ERROR, "%s: Invalid input ", __FUNCTION__);
					in->status = LCC_CMD_UNSUCCESS;
					return;
				}

			}

			if (NULL == i2c_tx_data)
			{
				SLOGF(SLOG_ERROR, "%s: Lack of memory", __FUNCTION__);
				in->status = LCC_CMD_UNSUCCESS;
				return;
			}
			else
			{

				SLOGF(SLOG_INFO,	"Start forwarding data to"
								" Channel ID: \033[91m[%d]\033[92m--"
								"Slave address: \033[91m[0x%x]\033[92m--"
								"Transmitter Length: \033[91m[%d]\033[92m--"
								"Receiver Length: \033[91m[%d]\033[0m",
								(int)chid, slave_addr,
								(int)tx_len,
								(int)rx_len);

				memcpy(i2c_tx_data, (in->data + data_ptr), tx_len);
				i2cm.transceiver(chid, slave_addr, i2c_tx_data, tx_len,
								i2c_rx_data, rx_len);
				vPortFree(i2c_tx_data);
				i2c_tx_data = NULL;
				in->status = LCC_CMD_SUCCESS;
			}

		}
		else
		{
			SLOGF(SLOG_ERROR, "%s: Invalid input ", __FUNCTION__);
			in->status = LCC_CMD_UNSUCCESS;
		}

	}
	else
	{
		if((i2c_rx_data != NULL) && (rx_len != 0))
		{
			assert_malloc(out->data, rx_len * sizeof(uint8_t));
			memcpy(out->data, i2c_rx_data, rx_len);
			out->len = rx_len;
			vPortFree(i2c_rx_data);
			rx_len = 0;
			i2c_rx_data = NULL;

			SLOGF(SLOG_INFO, "Data %s", i2c_status == I2CM_ERROR_TRANSCEIVED
				? "is"
			" ready" : i2c_status == I2CM_ERROR_BUSY ? "is pending" :
			"is not exist because of the i2c timeout");
			in->status = LCC_CMD_SUCCESS;
		}
		else
		{
			SLOGF(SLOG_ERROR, "%s: Please send a read request first ",
			 __FUNCTION__);
			in->status = LCC_CMD_INVALID_ARG;
		}

	}
}

/**
 * cmd_zoom_factor
 * [TID][CMD][UCID][4 bytes data]
 *
 */
void cmd_zoom_factor(lcc_cmd_t *in, lcc_cmd_t *out)
{
	lcc_cmd_info_t cmd = (lcc_cmd_info_t)(LCC_CMD_ZOOM_FACTOR);

	/* Read command processing */
	if (CMD_READ == in->action)
	{
		out->data = pvPortMalloc(cmd.size);
		if (NULL == out->data)
		{
			SLOGF(SLOG_ERROR, "%s: Lack of memory", __FUNCTION__);
			in->status = LCC_CMD_UNSUCCESS;
			return;
		}
        float zoom_factor = light_system->settings->zoom_factor;
		SLOGF(SLOG_DEBUG, "Read zoom factor data: %.02f", zoom_factor);
		/* TODO: Take semaphore to access settings memory */
		memcpy(out->data, &zoom_factor, cmd.size);
		/* TODO: Release semaphore after read data */
		out->len = cmd.size;
	}
	else
	{
		float zoom_factor;
		memcpy(&zoom_factor, &in->data[0], cmd.size);
		SLOGF(SLOG_DEBUG, "Write zoom factor data: %.02f (wrt 28mm)", zoom_factor);

		/* TODO: Take semaphore to access settings memory */
		light_system->settings->zoom_factor = zoom_factor;

		/* TODO: Release semaphore after read data */

		/* Update the command status */
		in->status = LCC_CMD_SUCCESS;
	}
}

/**
 * cmd_cam_module_mirror_calib
 * [TID][CMD_ID][M_BITMASK][DUMMY_BYTES]
 */
void cmd_cam_module_mirror_calib(lcc_cmd_t *in, lcc_cmd_t *out)
{
	lcc_cmd_tid_t p_cmd;
	cam_typedef_t *p_cam;
	actuator_t *pmirr;
	uint32_t m_bitmask = in->m_bitmask;
	uint8_t i, cam_idx;

	p_cmd.cmd_tid = in->tid;
	p_cmd.event = OPT_EVENT_MIRROR_CALIB;
	/* Scan camera bitmask */
	for (i = 0; i < in->m_number; i++)
	{
		cam_idx = __builtin_ctz(m_bitmask);
		m_bitmask &= ~(1 << cam_idx);
		/* Get camera object from camera object list */
		p_cam = idx_to_object(cam_idx);
		if (NULL == p_cam)	/*Verify object is valid */
		{
			in->status = LCC_CMD_INVALID_ARG;
			return;
		}

		if((p_cam->info.grp != GRP_A) &&
			(p_cam->info.module != CAM_B4) &&
			(p_cam->info.module != CAM_C5) &&
			(p_cam->info.module != CAM_C6))
		{
			/* Raise a updating signal to camera task for this ASIC */
			if (light_system->m_filter & (1 << cam_idx))
			{
				pmirr = p_cam->optical->mirr;
				/* Take semaphore to access settings memory */
				xSemaphoreTake(pmirr->semaphore, portMAX_DELAY);
				if (pmirr->moving == FALSE)
				{
					/* set moving status */
					pmirr->moving = TRUE;
					/* Release semaphore after accessing data */
					xSemaphoreGive(pmirr->semaphore);
					xQueueSend(pmirr->queue, &p_cmd, (TickType_t)10);
				}
				else
				{
					/* Release semaphore after accessing data */
					xSemaphoreGive(pmirr->semaphore);
					SLOGF(SLOG_ERROR, "CAM-%X : Mirror is moving",
							p_cam->info.module);
					in->status = LCC_CMD_UNSUCCESS;
					return;
				}
			}
		}
		else
		{
			in->status = LCC_CMD_SUCCESS;
			/* Reset bit of CAM if it have not MIRROR */
			in->m_bitmask &= ~(object_to_cam_bitmask(p_cam));
		}
	}
	/* The command status will be updated in either lcc_cmd task
	 * (read command) or task_cam_ctrl (write command) */
}

/**
 * cmd_cam_module_mirror_calib
 * [TID][CMD_ID][I2C_CH_BITMASK][N*SPPED]
 */
void cmd_i2c_speed(lcc_cmd_t *in, lcc_cmd_t *out)
{
	lcc_cmd_info_t cmd = (lcc_cmd_info_t)(LCC_CMD_I2C_SPEED);
	uint8_t i, chid, data_cnt = 0;
	uint32_t c_bitmask = 0;
	uint8_t c_number = 0;
	uint8_t global = 0;
	uint8_t i2c_speed = 0;

	SLOGF(SLOG_DEBUG, "Entering %s", __FUNCTION__);

	uint32_t c_bitmask_all = 0;
	for (i = 0; i < I2C_CH_MAX_IDX; i++)
	{
		c_bitmask_all |= (1 << i);
	}
	/* Read command processing */
	if (CMD_READ == in->action)
	{
		c_bitmask = c_bitmask_all;
		/* Get Number of i2c channel selected in c_bitmask */
		c_number = __builtin_popcount(c_bitmask);
		out->data = pvPortMalloc(c_number * cmd.size);
		if (NULL == out->data)
		{
			SLOGF(SLOG_ERROR, "%s: Lack of memory", __FUNCTION__);
			in->status = LCC_CMD_UNSUCCESS;
			return;
		}
	}
	else
	{
		/* Get I2C channel bitmask */
		c_bitmask = (uint32_t)in->data[data_cnt++];
		c_bitmask |= (uint32_t)in->data[data_cnt++] << 8;
		c_bitmask |= (uint32_t)in->data[data_cnt++] << 16;
		/* Check global in c_bitmask */
		global = c_bitmask & ((uint32_t)1<<I2C_CH_MAX_IDX);
		/*SLOGF(SLOG_DEBUG, "Global: %d %X %X", global, c_bitmask, ((uint32_t)1<<I2C_CH_MAX_IDX));*/
		/* Bitmask for all i2c channel manage */
		if(global)
		{
			c_bitmask = c_bitmask_all;
			SLOGF(SLOG_DEBUG, "Global is selected %X", c_bitmask);
		}
		/* Get Number of i2c channel selected in c_bitmask */
		c_number = __builtin_popcount(c_bitmask);
		/* Check valid data length */
		if((global || in->len != (cmd.size * c_number + 3))
			&& (!global || in->len != cmd.size))
		{
			SLOGF(SLOG_ERROR, "%s:%d Data Length is invalid %d",
					__FUNCTION__, __LINE__, in->len);
			in->status = LCC_CMD_INVALID_ARG;
			return;
		}
	}

	/* Scan i2c channel bitmask */
	for (i = 0; i < c_number; i++)
	{
		chid = __builtin_ctz(c_bitmask);
		c_bitmask &= ~(1 << chid);
		if (!IS_I2C_CHANNEL(chid))	/*Verify i2c channel is valid */
		{
			SLOGF(SLOG_ERROR, "%s:%d I2C channel is invalid %d",
					__FUNCTION__, __LINE__, chid);
			in->status = LCC_CMD_INVALID_ARG;
			return;
		}

		/* Read command processing */
		if (CMD_READ == in->action)
		{
			SLOGF(SLOG_DEBUG, "Get speed of I2C channel-%d", chid);
			/* Take semaphore to access settings memory */
			hal_i2c_clockspeed_t speed = hal_i2c_get_speed(chid);
			memcpy(&out->data[out->len], &speed, cmd.size);
			/* Release semaphore after read data */
			out->len += cmd.size;
#if(LOG_VERBOSE == STD_ON)
			SLOGF(SLOG_DEBUG, "I2C speed: %d", speed);
#endif
		}
		/* Write command processing */
		else
		{
			SLOGF(SLOG_DEBUG, "Set speed for I2C channel-%d", chid);
			hal_i2c_clockspeed_t speed = hal_i2c_get_speed(chid);
			i2c_speed = (uint32_t)in->data[data_cnt];
			if(!IS_I2C_SPEEDMODE(i2c_speed))
			{
				SLOGF(SLOG_ERROR, "%s:%d I2C speed is invalid %d",
						__FUNCTION__, __LINE__, i2c_speed);
				in->status = LCC_CMD_INVALID_ARG;
				return;
			}
			if (i2c_speed != speed)
			{
				vTaskSuspendAll();
				hal_i2c_set_speed(chid, i2c_speed);
				xTaskResumeAll();
				/* Adjust I2C_WRITE_DELAY_SEC to capture */
				if ((I2C_CH0 <= chid && chid <= I2C_CH5) ||
						(I2C_CH9 == chid) ||
						(I2C_CH15 == chid))
				{
					switch(i2c_speed)
					{
						case I2C_SPEED_1MHz:
							i2c_write_delay_sec_asic1 =
									I2C_WRITE_DELAY_SEC_ASIC1_1MHZ;
							i2c_write_delay_sec_asic23 =
									I2C_WRITE_DELAY_SEC_ASIC23_1MHZ;
							break;
						case I2C_SPEED_400KHz:
							i2c_write_delay_sec_asic1 =
									I2C_WRITE_DELAY_SEC_ASIC1_400KHZ;
							i2c_write_delay_sec_asic23 =
									I2C_WRITE_DELAY_SEC_ASIC23_400KHZ;
							break;
						case I2C_SPEED_100KHz:
							i2c_write_delay_sec_asic1 =
									I2C_WRITE_DELAY_SEC_ASIC1_100KHZ;
							i2c_write_delay_sec_asic23 =
									I2C_WRITE_DELAY_SEC_ASIC23_100KHZ;
							break;
					}
				}
			}

			/* Set data pointer when command was set for global */
			data_cnt += global ? 0 : cmd.size;
#if(LOG_VERBOSE == STD_ON)
			SLOGF(SLOG_DEBUG, "I2C speed: cur: %d set: %d", speed, i2c_speed);
#endif
		}
	}
	/* Update command status */
	in->status = LCC_CMD_SUCCESS;

	SLOGF(SLOG_DEBUG, "Exiting %s", __FUNCTION__);
}

/**
 * cmd_cam_module_mirror_calib
 * [TID][CMD_ID][SPI_SPEED]
 */
void cmd_spi_speed(lcc_cmd_t *in, lcc_cmd_t *out)
{
	lcc_cmd_info_t cmd = (lcc_cmd_info_t)(LCC_CMD_SPI_SPEED);

	SLOGF(SLOG_DEBUG, "Entering %s", __FUNCTION__);

	/* Read command processing */
	if (CMD_READ == in->action)
	{
		out->data = pvPortMalloc(cmd.size);
		if (NULL == out->data)
		{
			SLOGF(SLOG_ERROR, "%s: Lack of memory", __FUNCTION__);
			in->status = LCC_CMD_UNSUCCESS;
			return;
		}
		uint32_t spim_speed = light_system->settings->spim_speed;
		SLOGF(SLOG_DEBUG, "Read spim speed: %d", spim_speed);
		/* TODO: Take semaphore to access settings memory */
		memcpy(out->data, &spim_speed, cmd.size);
		/* TODO: Release semaphore after read data */
		out->len = cmd.size;
	}
	/* Write command processing */
	else
	{
		/* Get selected spi speed */
		uint32_t spim_speed = 0;
		memcpy(&spim_speed, &in->data[0], cmd.size);
		SLOGF(SLOG_DEBUG, "Set SPI speed: %d", spim_speed);
		/* TODO: Check valib spi speed */
		if(false)
		{
			SLOGF(SLOG_ERROR, "%s:%d SPI speed is invalid %d",
					__FUNCTION__, __LINE__, spim_speed);
			in->status = LCC_CMD_INVALID_ARG;
			return;
		}
		/* Get current spi speed */
		uint32_t cur_speed = light_system->settings->spim_speed;
		/* TODO: Change spi speed */
		if (spim_speed != cur_speed)
		{
			/* TODO: Take semaphore to access settings memory */
			light_system->settings->spim_speed = spim_speed;
			/* TODO: Release semaphore after read data */
		}
#if(LOG_VERBOSE == STD_ON)
		SLOGF(SLOG_DEBUG, "SPI speed: cur: %d set: %d", cur_speed, spim_speed);
#endif
	}

	/* Update command status */
	in->status = LCC_CMD_SUCCESS;

	SLOGF(SLOG_DEBUG, "Exiting %s", __FUNCTION__);
}

/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
