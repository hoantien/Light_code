/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    cbb_cmd_base_0200.c
 * @author  The LightCo
 * @version V1.0.0
 * @date    Mar-10-2016
 * @brief   This file contains expand of cbb_cmd_base_0200
 *
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "os.h"
#include "log.h"
#include "assert.h"
#include "lcc_cmd.h"
#include "temperature.h"
#include "light_system.h"
#include "task_cam_ctrl.h"
#include "tof.h"

/* Private define------------------------------------------------------------*/
#define SLOGF_ID						SLOG_ID_LCC_CMD_BASE_0200

/* Export functions ----------------------------------------------------------*/
/**
 * cmd_asic_light_protocol
 *
 */
void cmd_asic_light_protocol(lcc_cmd_t *in, lcc_cmd_t *out)
{
	lcc_cmd_info_t cmd = (lcc_cmd_info_t)(LCC_CMD_ASIC_LIGHT_PROTOCOL);

	out->data = pvPortMalloc(cmd.size);
	if(NULL == out->data)
	{
		in->status = LCC_CMD_UNSUCCESS;
		SLOGF(SLOG_ERROR, "%s:[%d]: Malloc failed", __FUNCTION__, __LINE__);
		return;
	}

	memcpy(out->data, light_system->settings->light_protocol_version,
			cmd.size);
	out->len = cmd.size;
	/* The command status will be updated in lcc_cmd task */
}

/**
 * cmd_asic_fw_version
 *
 */
void cmd_asic_fw_version(lcc_cmd_t *in, lcc_cmd_t *out)
{
	lcc_cmd_info_t cmd = (lcc_cmd_info_t)(LCC_CMD_ASIC_FW_VERSION);

	out->data = pvPortMalloc(cmd.size);
	if(NULL == out->data)
	{
		in->status = LCC_CMD_UNSUCCESS;
		SLOGF(SLOG_ERROR, "%s:[%d]: Malloc failed", __FUNCTION__, __LINE__);
		return;
	}

	memcpy(out->data, light_system->settings->fw_version, cmd.size);
	out->len = cmd.size;

	/* The command status will be updated in lcc_cmd task */
}

/**
 * cmd_asic_calib_version
 *
 */
void cmd_asic_calib_version(lcc_cmd_t *in, lcc_cmd_t *out)
{
	lcc_cmd_info_t cmd = (lcc_cmd_info_t)(LCC_CMD_ASIC_CALIB_VERSION);

	out->data = pvPortMalloc(cmd.size);
	if(NULL == out->data)
	{
		in->status = LCC_CMD_UNSUCCESS;
		SLOGF(SLOG_ERROR, "%s:[%d]: Malloc failed", __FUNCTION__, __LINE__);
		return;
	}

	memcpy(out->data, light_system->settings->light_calib_version, cmd.size);
	out->len = cmd.size;

	/* The command status will be updated in lcc_cmd task */
}

/**
 * cmd_asic_status
 *
 */
void cmd_asic_status(lcc_cmd_t *in, lcc_cmd_t *out)
{
	lcc_cmd_info_t cmd = (lcc_cmd_info_t)(LCC_CMD_ASIC_STATUS);
#if (ASIC_NUM != ASIC1)
	if(!light_system->is_first_command)
	{
		light_system->is_first_command = TRUE;
		SLOGF(SLOG_INFO, "Pull down the interrupt pin");
		set_intr_pin(OFF);
	}
#endif
	out->data = pvPortMalloc(cmd.size);
	if (NULL == out->data)
	{
		SLOGF(SLOG_ERROR, "%s:%d Lack of memory", __FUNCTION__, __LINE__);
		in->status = LCC_CMD_UNSUCCESS;
		return;
	}

	memcpy(out->data, light_system->settings->asic_status, cmd.size);
	out->len = cmd.size;

	/* The command status will be updated in lcc_cmd task */
}

/**
 * cmd_asic_log_ctrl
 *
 */
void cmd_asic_log_ctrl(lcc_cmd_t *in, lcc_cmd_t *out)
{
	lcc_cmd_info_t cmd = (lcc_cmd_info_t)(LCC_CMD_ASIC_LOG_CTRL);

	/* Read command processing */
	if (CMD_READ == in->action)
	{
		out->data = pvPortMalloc(cmd.size);
		if (NULL == out->data)
		{
			SLOGF(SLOG_ERROR, "%s: Lack of memory", __FUNCTION__);
			in->status = LCC_CMD_ERROR_MODULE_FAULT;
			return;
		}
		memcpy(out->data, &light_system->settings->log_ctrl, cmd.size);
		out->len = cmd.size;
	}
	else
	{
		memcpy(&light_system->settings->log_ctrl, in->data, cmd.size);
		in->status = LCC_CMD_SUCCESS;
		SLOGF(SLOG_DEBUG, "%s: Set Log control to %X successfully",
			__FUNCTION__, light_system->settings->log_ctrl);
	}

	/* The command status will be updated in lcc_cmd task */
}

/**
 * cmd_asic_tmpx
 *
 */
void cmd_asic_tmpx(lcc_cmd_t *in, lcc_cmd_t *out)
{
	lcc_cmd_info_t cmd = (lcc_cmd_info_t)(LCC_CMD_ASIC_TMPx);
	uint8_t ret_status = 0;
	uint8_t tmp_idx = 0;
	uint8_t *buf = light_system->settings->temperature;
	out->data = pvPortMalloc(cmd.size);
	if (NULL != out->data)
	{
		for(uint8_t i = 0; i < 4; i++)
		{
			tmp_idx = i * 2;
			if(TEMP_OK ==
				temp_sensor_raw_data_get((uint8_t *)&buf[tmp_idx]))
				SLOGF(SLOG_INFO, "%s: Read temp channel %d successfully, values"
					" are %X and %X", __FUNCTION__, i, buf[tmp_idx],
					buf[tmp_idx + 1]);
			else
			{
				ret_status |= LCC_CMD_ERROR_MODULE_FAULT;
				SLOGF(SLOG_WARN, "%s: Read temp channel %d failed!", i);
			}
		}
		if(ret_status & LCC_CMD_ERROR_MODULE_FAULT)
		{
			in->status = LCC_CMD_ERROR_MODULE_FAULT;
			out->len = 0;
			memset(buf, 0x00, cmd.size);
		}
		else
		{
			out->len = cmd.size;
			memcpy(out->data, buf, cmd.size);
		}
	}
	else
	{
		SLOGF(SLOG_ERROR, "%s:%d Lack of memory", __FUNCTION__, __LINE__);
		in->status = LCC_CMD_ERROR_MODULE_FAULT;
	}
}

/**
 * cmd_asic_cmd_dump
 *
 */
void cmd_asic_cmd_dump(lcc_cmd_t *in, lcc_cmd_t *out)
{
	/* To prevent the undefined data */
	out->len = 0;
	out->data = NULL;
	in->status = LCC_CMD_UNSUCCESS;
}

/**
 * cmd_asic_module_metadata
 *
 */
#define CAM_EEPROM_MAX_OFFSET		0x005D
void cmd_asic_module_metadata(lcc_cmd_t *in, lcc_cmd_t *out)
{
	lcc_cmd_tid_t p_cmd;
	cam_typedef_t *p_cam;
	lcc_cmd_info_t cmd = (lcc_cmd_info_t)(LCC_CMD_ASIC_MODULE_METADATA);
	uint32_t m_bitmask = in->m_bitmask;
	uint8_t cam_idx;
	static uint32_t read_buf_size = 0;
	uint8_t data_cnt = 0;
	/* Read request command */
	if(((in->m_number * cmd.size) == in->len) ||
		(in->global && (in->len == cmd.size)))
	{
		/* Reset the buffer size to prevent input mistake:
		 * invoke read request many times without read */
		read_buf_size = 0;
		for (uint8_t i = 0; i < in->m_number; i++)
		{
			cam_idx = __builtin_ctz(m_bitmask);
			m_bitmask &= ~(1 << cam_idx);
#if (ASIC_NUM != ASIC1)
			if(light_system->m_filter & (1 << cam_idx))
#endif
			{
				p_cam = idx_to_object(cam_idx);
				if (NULL == p_cam)	/* Verify object is valid */
				{
					in->status = LCC_CMD_ERROR_INVALID_MBITMASK;
					return;
				}
				else
				{
					vPortFree(p_cam->eeprom.buf);
					p_cam->eeprom.offset = U16(in->data[data_cnt]);
					p_cam->eeprom.len = U16(in->data[data_cnt + 2]);
					if((p_cam->eeprom.offset + p_cam->eeprom.len) >
						CAM_EEPROM_MAX_OFFSET)
						SLOGF(SLOG_WARN, "CAM-%X EEPROM Read address %X is"
							" invalid", p_cam->info.module,
							p_cam->eeprom.offset + p_cam->eeprom.len);

					if(p_cam->eeprom.len)
						p_cam->eeprom.buf = pvPortMalloc(p_cam->eeprom.len);
					else
						SLOGF(SLOG_ERROR, "CAM-%X: EEPROM write with length ="
							" %d does not take effect", p_cam->info.module,
							p_cam->eeprom.len);
					read_buf_size += p_cam->eeprom.len;
					p_cam->eeprom.flag = EEPROM_ACTION_READ;
#if (ASIC_NUM == ASIC1)
					if(light_system->m_filter & (1 << cam_idx))
#endif
					{
						SLOGF(SLOG_INFO, "Start read data from CAM-%X: offset"
							" %X, len %d", p_cam->info.module,
							p_cam->eeprom.offset, p_cam->eeprom.len);
						p_cmd.cmd_tid = in->tid;
						p_cmd.event = IMG_EVENT_METADATA;
						xQueueSend(p_cam->queue, &p_cmd, (TickType_t)10);
					}
				}
			}
#if (ASIC_NUM != ASIC1)
			else
			{
				/* The m_bitmask need to reset to update the cmd status */
				in->m_bitmask &= ~(1 << cam_idx);
			}
#endif
			data_cnt += in->global ? 0 : cmd.size;
		}
	}
	/* Read back data command */
	else if(in->len == 0)
	{
		out->len = read_buf_size;
		if(out->len)
			out->data = pvPortMalloc(out->len);
		data_cnt = 0;
		for(uint8_t i = 0; i < in->m_number; i++)
		{
			cam_idx = __builtin_ctz(m_bitmask);
			m_bitmask &= ~(1 << cam_idx);
			p_cam = idx_to_object(cam_idx);
			if (NULL == p_cam)	/* Verify object is valid */
			{
				in->status = LCC_CMD_ERROR_INVALID_MBITMASK;
				return;
			}
			else
			{
				if(p_cam->eeprom.buf)
				{
					memcpy(&out->data[data_cnt], p_cam->eeprom.buf,
						p_cam->eeprom.len);
					data_cnt += p_cam->eeprom.len;
					vPortFree(p_cam->eeprom.buf);
					p_cam->eeprom.buf = NULL;
				}
				else
				{
					SLOGF(SLOG_ERROR, "%s:[%d] Send read request command first",
						__FUNCTION__, __LINE__);
				}
			}
		}
		/* Reset for next time */
		read_buf_size = 0;
	}
	/* Write command */
	else if((in->m_number * cmd.size) <= in->len)
	{
		uint8_t idx = in->m_number * cmd.size;
		for(uint8_t i = 0; i < in->m_number; i++)
		{
			cam_idx = __builtin_ctz(m_bitmask);
			m_bitmask &= ~(1 << cam_idx);
#if (ASIC_NUM != ASIC1)
			if(light_system->m_filter & (1 << cam_idx))
#endif
			{
				p_cam = idx_to_object(cam_idx);
				if (NULL == p_cam)	/* Verify object is valid */
				{
					in->status = LCC_CMD_ERROR_INVALID_MBITMASK;
					return;
				}
				else
				{
					p_cam->eeprom.offset = U16(in->data[data_cnt]);
					p_cam->eeprom.len = U16(in->data[data_cnt + 2]);
#if (ASIC_NUM == ASIC1)
					if(light_system->m_filter & 1 << (cam_idx))
#endif
					{
						vPortFree(p_cam->eeprom.buf);
						if(p_cam->eeprom.len)
						{
							p_cam->eeprom.buf = pvPortMalloc(p_cam->eeprom.len);
							memcpy(p_cam->eeprom.buf, &in->data[idx],
								p_cam->eeprom.len);
							SLOGF(SLOG_INFO, "Start write data CAM-%X: offset"
								" %X, len %d", p_cam->info.module,
								p_cam->eeprom.offset, p_cam->eeprom.len);
							if((p_cam->eeprom.offset + p_cam->eeprom.len) >
								CAM_EEPROM_MAX_OFFSET)
								SLOGF(SLOG_WARN, "CAM-%X EEPROM address %X is"
									" invalid", p_cam->info.module,
									p_cam->eeprom.offset + p_cam->eeprom.len);
							p_cam->eeprom.flag = EEPROM_ACTION_WRITE;
							p_cmd.cmd_tid = in->tid;
							p_cmd.event = IMG_EVENT_METADATA;
							xQueueSend(p_cam->queue, &p_cmd, (TickType_t)10);
						}
						else
							SLOGF(SLOG_ERROR, "CAM-%X EEPROM Write with length"
								" = %d does not take effrct",
								p_cam->info.module, p_cam->eeprom.len);

					}
				}
			}
			idx += U16(in->data[data_cnt + 2]);
			data_cnt += in->global ? 0 : cmd.size;
		}
	}
	else
	{
		in->status = LCC_CMD_INVALID_ARG;
		SLOGF(SLOG_INFO, "Message length invalid");
	}
}

/**
 * cmd_asic_device_calibration
 *
 */
void cmd_asic_device_calibration(lcc_cmd_t *in, lcc_cmd_t *out)
{
	/* To prevent the undefined data */
	out->len = 0;
	out->data = NULL;
	in->status = LCC_CMD_UNSUCCESS;
}

/**
 * cmd_asic_pwr_ctrl
 *
 */
void cmd_asic_pwr_ctrl(lcc_cmd_t *in, lcc_cmd_t *out)
{
	/* To prevent the undefined data */
	out->len = 0;
	out->data = NULL;
	in->status = LCC_CMD_UNSUCCESS;
}

/**
 * cmd_cam_asic_intr_src
 *
 */
void cmd_cam_asic_intr_src(lcc_cmd_t *in, lcc_cmd_t *out)
{
	/* Read command processing */
	if (CMD_READ == in->action)
	{
		out->data = pvPortMalloc(INTR_DATA_SIZE);
		if (NULL == out->data)
		{
			SLOGF(SLOG_ERROR, "%s:%d Lack off memory",
											 __FUNCTION__, __LINE__);
			in->status = LCC_CMD_ERROR_MODULE_FAULT;
			return;
		}

		if(intr_queue_pop(out->data) > 0)
		{
			out->len = INTR_DATA_SIZE;
			/* The command status will be updated in lcc_cmd task */
		}
		else
		{
			out->len = 0;
			in->status = LCC_CMD_UNSUCCESS;
			SLOGF(SLOG_WARN, "%s:%d Interrupt source queue is empty",
				__FUNCTION__, __LINE__);
		}
	}
	else /* Write command*/
	{
		in->status = LCC_CMD_INVALID_ARG;
		SLOGF(SLOG_ERROR, "%s:%d Get interrupt source command does not support"
			" write functionality", __FUNCTION__, __LINE__);
	}
	lcc_cmd_log_update_status(in->tid, in->status, 0x1FFFE);
}

#define FLASH_FLASH_MODE_LENGTH		7
#define FLASH_TORCH_MODE_LENGTH		5
#define FLASH_IR_MODE_LENGTH		3
#define FLASH_LED_1_ENABLE			1
#define FLASH_LED_2_ENABLE			2
typedef union
{
	uint8_t data;
	struct
	{
		uint8_t mode : 4;
		uint8_t led_en : 4;
	};
} flash_type_cmd_t;
void cmd_asic_flash_light(lcc_cmd_t *in, lcc_cmd_t *out)
{
#if (ASIC_NUM == ASIC1)
	flash_type_cmd_t flash_type;
	flash_type.data = in->data[0];
	uint8_t valid_params = TRUE;
	uint16_t led1_cur;
	uint16_t led2_cur;
	uint16_t duration;
	out->len = 0;
	out->data = NULL;

	if(flash_type.led_en > 3)
	{
		in->status = LCC_CMD_INVALID_ARG;
		SLOGF(SLOG_ERROR, "LED enable bit is invalid");
		return;
	}
	/* if flash device is not busy */
	if(light_system->flash_dev.flag != FLASH_FLAG_BUSY)
	{
		switch(flash_type.mode)
		{
			case FLASH_FLASH_MODE:
				if(in->len == FLASH_FLASH_MODE_LENGTH)
				{
					led1_cur = U16(in->data[1]);
					led2_cur = U16(in->data[3]);
					duration = U16(in->data[5]);
					if(led1_cur > FLASH_MAX_FLASH_CURRENT ||
						led2_cur > FLASH_MAX_FLASH_CURRENT)
					{
						in->status = LCC_CMD_INVALID_ARG;
						valid_params = FALSE;
						SLOGF(SLOG_ERROR, "%s:[%d], Invalid led curent param",
							__FUNCTION__, __LINE__);
					}
					else if(duration > FLASH_MAX_DURATION)
					{
						in->status = LCC_CMD_INVALID_ARG;
						SLOGF(SLOG_ERROR, "%s[%d], Invalid duration param %d",
							__FUNCTION__, __LINE__, duration);
						valid_params = FALSE;
					}
					else
					{
						light_system->flash_dev.led1_cur_mA = led1_cur;
						light_system->flash_dev.led2_cur_mA = led2_cur;
						light_system->flash_dev.flash_time = duration;
						light_system->flash_dev.enable_reg.LED1 =
							flash_type.led_en;
						light_system->flash_dev.enable_reg.LED2 =
							flash_type.led_en >> 1;
						light_system->flash_dev.flash_mode =
							FLASH_FLASH_MODE;
					}
				}
				else
				{
					in->status = LCC_CMD_INVALID_ARG;
					valid_params = FALSE;
					SLOGF(SLOG_ERROR, "%s:[%d] Command format is in correct",
						__FUNCTION__, __LINE__);
				}
				break;
			case FLASH_TORCH_MODE:
				if(in->len == FLASH_TORCH_MODE_LENGTH)
				{
					led1_cur = U16(in->data[1]);
					led2_cur = U16(in->data[3]);
					if(led1_cur > FLASH_MAX_TORCH_CURRENT ||
						led2_cur > FLASH_MAX_TORCH_CURRENT)
					{
						in->status = LCC_CMD_INVALID_ARG;
						valid_params = FALSE;
						SLOGF(SLOG_ERROR, "%s:[%d], Invalid led curent param",
							__FUNCTION__, __LINE__);
						return;
					}
					else
					{
						light_system->flash_dev.led1_cur_mA = led1_cur;
						light_system->flash_dev.led2_cur_mA = led2_cur;
						light_system->flash_dev.enable_reg.LED1 =
							flash_type.led_en;
						light_system->flash_dev.enable_reg.LED2 =
							flash_type.led_en >> 1;
						light_system->flash_dev.flash_mode = FLASH_TORCH_MODE;
					}
				}
				break;
			case FLASH_IR_MODE:
				if(in->len == FLASH_IR_MODE_LENGTH)
				{
					duration = U16(in->data[1]);
					if(duration > FLASH_MAX_DURATION)
					{
						in->status = LCC_CMD_INVALID_ARG;
						valid_params = FALSE;
						SLOGF(SLOG_ERROR, "%s[%d], Invalid duration param %d",
							__FUNCTION__, __LINE__, duration);
						return;
					}
					light_system->flash_dev.flash_time = duration;
					light_system->flash_dev.enable_reg.LED1 =
						flash_type.led_en;
					light_system->flash_dev.enable_reg.LED2 =
						flash_type.led_en >> 1;
					light_system->flash_dev.flash_mode = FLASH_IR_MODE;
				}
				else
				{
					in->status = LCC_CMD_INVALID_ARG;
					valid_params = FALSE;
					SLOGF(SLOG_ERROR, "%s:[%d] Command format is in correct",
						__FUNCTION__, __LINE__);
					return;
				}
				break;
			default:
				in->status = LCC_CMD_INVALID_ARG;
				valid_params = FALSE;
				SLOGF(SLOG_ERROR, "%s:[%d] Command format is invalid",
					__FUNCTION__, __LINE__);
				break;
		}
		/* Check if command format is invalid, do nothing */
		if(valid_params)
		{
			light_system->flash_dev.flag = FLASH_FLAG_FLASH;
			xEventGroupSetBits(light_system->event, CCB_EVENT_FLASH);
		}
	}
	else
	{
		in->status = LCC_CMD_UNSUCCESS;
		SLOGF(SLOG_ERROR, "%s:[%d] Flash device is busy", __FUNCTION__,
			__LINE__);
	}
#endif /* (ASIC_NUM == ASIC1) */
}

void cmd_asic_tof(lcc_cmd_t *in, lcc_cmd_t *out)
{
#if (ASIC_NUM == ASIC1)
#ifdef USING_TOF_FUNC
	uint8_t Mode = in->data[0];
	out->len = 0;
	if(in->action == CMD_WRITE)
	{
		if((Mode <= VL53L0X_MODE_HIGH_SPEED && Mode >= VL53L0X_MODE_STANDARD) ||
			(Mode <= VL53L0X_CALIB_XTALK && Mode >= VL53L0X_CALIB_REF))
		{
			light_system->ToF_dev.Mode = Mode;
			if(Mode == VL53L0X_CALIB_XTALK || Mode == VL53L0X_CALIB_OFFSET)
			{
				if(in->len == 3)
					light_system->ToF_dev.CalibDistance = U16(in->data[1]);
				else
				{
					SLOGF(SLOG_ERROR, "%s:[%d] Wrong command data length %d",
						__FUNCTION__, __LINE__, in->len);
					out->status = LCC_CMD_INVALID_ARG;
					return;
				}
			}
			if(light_system->ToF_dev.Flag == VL53L0X_FLAG_DONE ||
				light_system->ToF_dev.Flag == VL53L0X_FLAG_ERROR)
				light_system->ToF_dev.Flag = VL53L0X_FLAG_EXECUTE;
			xEventGroupSetBits(light_system->event, CCB_EVENT_TOF);
		}
		else
		{
			SLOGF(SLOG_ERROR, "%s:[%d]: Parameter = %d is invalid",
				__FUNCTION__, __LINE__, Mode);
			in->status = LCC_CMD_INVALID_ARG;
		}
	}
	else
	{
		if(light_system->ToF_dev.Flag == VL53L0X_FLAG_DONE)
		{
			out->len = sizeof(uint16_t);
			out->data = pvPortMalloc(out->len);
			if(NULL != out->data)
			{
				memcpy((uint8_t *)out->data, (uint8_t *)&light_system->
					ToF_dev.Data.LastRangeMeasure.RangeMilliMeter, out->len);
				in->status = LCC_CMD_SUCCESS;
				SLOGF(SLOG_INFO, "%s:[%d] Measurement done with DISTANCE is %i",
					__FUNCTION__, __LINE__,
				light_system->ToF_dev.Data.LastRangeMeasure.RangeMilliMeter);
			}
			else
			{
				in->status = LCC_CMD_ERROR_MODULE_FAULT;
				SLOGF(SLOG_ERROR, "%s:[%d]: Malloc failed", __FUNCTION__,
					__LINE__);
			}
		}
		else if(light_system->ToF_dev.Flag == VL53L0X_FLAG_BUSY)
		{
			out->len = 0;
			in->status = LCC_CMD_PENDING;
			SLOGF(SLOG_WARN, "%s:[%d]: The ToF is BUSY", __FUNCTION__,
				__LINE__);
		}
		else
		{
			out->len = 0;
			in->status = LCC_CMD_ERROR_MODULE_FAULT;
			SLOGF(SLOG_ERROR, "%s:[%d]: Something went wrong, please trace back"
				" the log", __FUNCTION__, __LINE__);
		}
	}
#else
	/* To prevent the undefined data */
	out->len = 0;
	out->data = NULL;
	in->status = LCC_CMD_UNSUCCESS;
#endif /* USING_TOF_FUNC */
#endif /* (ASIC_NUM == ASIC1) */
}

#define GYRO_LIMIT_SAMPLE_NUM	42 /* 42 * LCC_CMD_GYRO_SIZE ~ 512 bytes */
void cmd_asic_gyro(lcc_cmd_t *in, lcc_cmd_t *out)
{
#if (ASIC_NUM == ASIC1)
#ifdef USING_GYRO_FUNC
	if(in->action == CMD_READ)
	{
		if(light_system->gyro_dev.flag == GYRO_FLAG_DONE)
		{
			out->len = light_system->gyro_dev.sample_num *
					LCC_CMD_GYRO_DATA_SIZE;
			/* Prevent the user mistake send cmd read first,
			 * when the sample num is still 0 */
			if(out->len)
				out->data = pvPortMalloc(out->len);
			else
			{
				out->data = NULL;
				SLOGF(SLOG_WARN, "%s:[%d] Send command write first",
					__FUNCTION__, __LINE__);
				return;
			}
			if(NULL != out->data)
			{
				if(NULL != light_system->gyro_dev.buf)
				{
					memcpy(out->data, light_system->gyro_dev.buf, out->len);
					vPortFree(light_system->gyro_dev.buf);
					light_system->gyro_dev.buf = NULL;
					in->status = LCC_CMD_SUCCESS;
					SLOGF(SLOG_INFO, "%s[%d] Data copied", __FUNCTION__,
						__LINE__);
				}
			}
			else
			{
				in->status = LCC_CMD_UNSUCCESS;
				SLOGF(SLOG_ERROR, "%s[%d] Malloc failed", __FUNCTION__,
					__LINE__);
			}
		}
		else
		{
			in->status = LCC_CMD_UNSUCCESS;
			light_system->gyro_dev.flag = GYRO_FLAG_ERROR;
			SLOGF(SLOG_WARN, "%s:[%d] Gyro data is not available", __FUNCTION__,
				__LINE__);
		}
	}
	else /* write cmd */
	{
		light_system->gyro_dev.sample_num = U16(in->data[0]);
		/* The timer period is 10us, so the interval should devide by 10 */
		light_system->gyro_dev.interval_us = (uint32_t)(U32(in->data[2])/10);
		if(light_system->gyro_dev.sample_num > 0 &&
			light_system->gyro_dev.interval_us > 0)
		{
			/* Check if the gyro data too long */
			if(light_system->gyro_dev.sample_num > GYRO_LIMIT_SAMPLE_NUM)
			{
				SLOGF(SLOG_ERROR, "%s:[%d] The sample num %d too big",
					__FUNCTION__, __LINE__, light_system->gyro_dev.sample_num);
			}
			else
			{
				vPortFree(light_system->gyro_dev.buf);
				light_system->gyro_dev.buf = pvPortMalloc(
					light_system->gyro_dev.sample_num * LCC_CMD_GYRO_DATA_SIZE);
				if(NULL != light_system->gyro_dev.buf)
				{
					/* Reset count variable */
					light_system->gyro_dev.count = 0;
					if(light_system->gyro_dev.flag != GYRO_FLAG_GET_SAMPLE)
					{
						SLOGF(SLOG_INFO, "%s:[%d] Start read gyro with %d times"
							" and interval is %d", __FUNCTION__, __LINE__,
							light_system->gyro_dev.sample_num,
							light_system->gyro_dev.interval_us);
						light_system->gyro_dev.flag = GYRO_FLAG_READ;
						xEventGroupSetBits(light_system->event, CCB_EVENT_GYRO);
					}
					else
					{
						in->status = LCC_CMD_UNSUCCESS;
						SLOGF(SLOG_WARN, "%s:[%d] Gyro device is busy",
							__FUNCTION__, __LINE__);
					}
				}
				else
				{
					in->status = LCC_CMD_UNSUCCESS;
					SLOGF(SLOG_ERROR, "%s[%d] Malloc failed", __FUNCTION__,
						__LINE__);
				}
			}
		}
		else
		{
			in->status = LCC_CMD_INVALID_ARG;
			SLOGF(SLOG_ERROR, "%s:[%d] Invalid arguments",
				__FUNCTION__, __LINE__);
		}
	}
#else
	/* To prevent the undefined data */
	out->len = 0;
	out->data = NULL;
	in->status = LCC_CMD_UNSUCCESS;
#endif /* USING_GYRO_FUNC */
#endif /* (ASIC_NUM == ASIC1) */
}

#if (ASIC_NUM == ASIC1)
void cmd_asic_control(lcc_cmd_t *in, lcc_cmd_t *out)
{
	hal_gpio_t asic_pwr_on_seq;
	hal_pwm_pin_t pwm_pin;
	for(int i = 0; i < 2; i++)
	{
		if(i)
		{
			asic_pwr_on_seq.port      = GPIO_PORTC;
			asic_pwr_on_seq.pin       = GPIO_PIN_11;
			asic_pwr_on_seq.direction = GPIO_DIR_OUT;
			pwm_pin = PWM_PIN_NEG;
		}
		else
		{
			asic_pwr_on_seq.port      = GPIO_PORTC;
			asic_pwr_on_seq.pin       = GPIO_PIN_10;
			asic_pwr_on_seq.direction = GPIO_DIR_OUT;
			pwm_pin = PWM_PIN_POS;
		}

		switch(in->data[i])
		{
		case 1:
			SLOGF(SLOG_DEBUG, "%s power off ASIC%d", __FUNCTION__, i + 2);
			hal_gpio_init(&asic_pwr_on_seq);
			hal_gpio_set_low(&asic_pwr_on_seq);
			break;
		case 2:
			if(hal_gpio_read(&asic_pwr_on_seq) == GPIO_LEVEL_HIGH)
			{
				SLOGF(SLOG_DEBUG, "%s ASIC%d power is on",
						__FUNCTION__, i + 2);
				break;
			}
			SLOGF(SLOG_DEBUG, "%s power on ASIC%d", __FUNCTION__, i + 2);
			hal_gpio_init(&asic_pwr_on_seq);
			hal_gpio_set_high(&asic_pwr_on_seq);
		case 3:
			if(hal_gpio_read(&asic_pwr_on_seq) == GPIO_LEVEL_LOW)
			{
				SLOGF(SLOG_DEBUG, "%s ASIC%d power is off, please power on first",
						__FUNCTION__, i + 2);
				break;
			}
			SLOGF(SLOG_DEBUG, "%s reset ASIC%d", __FUNCTION__, i + 2);
			hal_pwm_set_low(PWM1_CH1, pwm_pin);
			hal_pwm_set_low(PWM1_CH3, pwm_pin);
			vTaskDelay(10);
			hal_pwm_set_high(PWM1_CH1, pwm_pin);
			vTaskDelay(1);
			hal_pwm_set_high(PWM1_CH3, pwm_pin);
			break;
		default:
			break;
		}
	}
}

void cmd_asic_power_info(lcc_cmd_t *in, lcc_cmd_t *out)
{
	lcc_cmd_info_t cmd = (lcc_cmd_info_t)(LCC_CMD_ASIC_POWER_INFO);
	uint8_t i = 0;
	if (CMD_READ == in->action)
	{
		vPortFree(out->data);
		out->data = pvPortMalloc(cmd.size);
		if (NULL == out->data)
		{
			SLOGF(SLOG_ERROR, "%s: Lack of memory", __FUNCTION__);
			in->status = LCC_CMD_UNSUCCESS;
			return;
		}
		SLOGF(SLOG_DEBUG, "%s : INA[%d]: Volt: %d(mV), Current: %d(mA), Power: %d(mW)",
								__FUNCTION__,
								light_system->power_ch,
								light_system->volt[light_system->power_ch],
								light_system->curr[light_system->power_ch],
								light_system->power[light_system->power_ch]);
		out->data[i++] = light_system->volt[light_system->power_ch];
		out->data[i++] = light_system->volt[light_system->power_ch] >> 8;
		out->data[i++] = light_system->volt[light_system->power_ch] >> 16;
		out->data[i++] = light_system->volt[light_system->power_ch] >> 24;
		out->data[i++] = light_system->curr[light_system->power_ch];
		out->data[i++] = light_system->curr[light_system->power_ch] >> 8;
		out->data[i++] = light_system->curr[light_system->power_ch] >> 16;
		out->data[i++] = light_system->curr[light_system->power_ch] >> 24;
		out->data[i++] = light_system->power[light_system->power_ch];
		out->data[i++] = light_system->power[light_system->power_ch] >> 8;
		out->data[i++] = light_system->power[light_system->power_ch] >> 16;
		out->data[i++] = light_system->power[light_system->power_ch] >> 24;
		out->len = cmd.size;
		in->status = LCC_CMD_SUCCESS;
	}
	else
	{
		light_system->power_ch = in->data[0];
		xEventGroupSetBits(light_system->event, CCB_EVENT_POWER);
		SLOGF(SLOG_DEBUG, "%s : power channel %d", __FUNCTION__,
											light_system->power_ch);
	}
}
#endif /* (ASIC_NUM == ASIC1) */

void cmd_asic_calib_data(lcc_cmd_t *in, lcc_cmd_t *out)
{
	char *str = assert_malloc(str, in->len * 3 * sizeof(char));
	char *str_byte = assert_malloc(str_byte, 3 * sizeof(char));
	memset(str, 0, in->len * 3);
	memset(str_byte, 0, 3);
	/* Dump data */
	for (int i = 0; i <  in->len; ++i)
	{
		sprintf(str_byte, "%02x ", in->data[i]);
		strcat(str, str_byte);
		memset(str_byte, 0, 3);
	}
	SLOGF(SLOG_DEBUG, "%s: %s", __FUNCTION__ ,str);
	vPortFree(str);
	vPortFree(str_byte);
}

#if (ASIC_NUM == ASIC1)
#define LCC_PZT_PWR_SELECT_LENGTH 	2
#define LCC_PZT_PWR_CONFIG_LENGTH 	6
#define LCC_PZT_RETURN_DATA_LENGTH	10
#define LCC_PZT_NUM_1			0x01
#define LCC_PZT_NUM_2			0x02
typedef union
{
	uint8_t buf[12];
	struct
	{
		uint16_t dummy_data;
		uint8_t pzt_supply_cam_70mm;
		uint8_t pzt_supply_cam_150mm;
		float pzt_pwr_1_voltage;
		float pzt_pwr_2_voltage;
	};
} lcc_pzt_config_option_t;
static lcc_pzt_config_option_t pzt_config = { .pzt_supply_cam_70mm = LCC_PZT_NUM_1,
	.pzt_supply_cam_150mm = LCC_PZT_NUM_1, .pzt_pwr_1_voltage = PZT1_VOLTAGE_DEFAULT_VALUE,
	.pzt_pwr_2_voltage = PZT1_VOLTAGE_DEFAULT_VALUE};
#endif
void cmd_asic_pzt_pwr_ctrl(lcc_cmd_t * in, lcc_cmd_t *out)
{
#if (ASIC_NUM == ASIC1)
	/* Read action */
	if(in->action == CMD_READ)
	{
		out->len = LCC_PZT_RETURN_DATA_LENGTH;
		out->data = pvPortMalloc(out->len);
		memcpy(out->data, pzt_config.buf + 2, LCC_PZT_RETURN_DATA_LENGTH);
	}
	/* Write action */
	else if(in->len == LCC_PZT_PWR_SELECT_LENGTH)
	{
		uint8_t pzt_70mm_pwr_sel = in->data[0];
		uint8_t pzt_150mm_pwr_sel = in->data[1];
		if(pzt_70mm_pwr_sel == LCC_PZT_NUM_1)
		{
			hal_pwm_set_high(PWM0_CH5, PWM_PIN_POS);
			pzt_config.pzt_supply_cam_70mm = LCC_PZT_NUM_1;
			SLOGF(SLOG_INFO, "PZT_PWR_70 is switched to PZT_VCAM_1");
			in->status = LCC_CMD_SUCCESS;
		}
		else if(pzt_70mm_pwr_sel == LCC_PZT_NUM_2)
		{
			hal_pwm_set_low(PWM0_CH5, PWM_PIN_POS);
			pzt_config.pzt_supply_cam_70mm = LCC_PZT_NUM_2;
			SLOGF(SLOG_INFO, "PZT_PWR_70 is switched to PZT_VCAM_2");
			in->status = LCC_CMD_SUCCESS;
		}
		else
		{
			in->status = LCC_CMD_INVALID_ARG;
			SLOGF(SLOG_ERROR, "%s[%d] Invalid arguments piezo power select %d",
				__FUNCTION__, __LINE__, pzt_70mm_pwr_sel);
		}

		if(pzt_150mm_pwr_sel == LCC_PZT_NUM_1)
		{
			hal_pwm_set_high(PWM0_CH5, PWM_PIN_NEG);
			pzt_config.pzt_supply_cam_150mm = LCC_PZT_NUM_1;
			SLOGF(SLOG_INFO, "PZT_PWR_150 is switched to PZT_VCAM_1");
			in->status = LCC_CMD_SUCCESS;
		}
		else if (pzt_150mm_pwr_sel == LCC_PZT_NUM_2)
		{
			hal_pwm_set_low(PWM0_CH5, PWM_PIN_NEG);
			pzt_config.pzt_supply_cam_150mm = LCC_PZT_NUM_2;
			SLOGF(SLOG_INFO, "PZT_PWR_150 is switched to PZT_VCAM_2");
			in->status = LCC_CMD_SUCCESS;
		}
		else
		{
			in->status = LCC_CMD_INVALID_ARG;
			SLOGF(SLOG_ERROR, "%s[%d] Invalid arguments piezo power select %d",
				__FUNCTION__, __LINE__, pzt_70mm_pwr_sel);
		}
	}
	else if(in->len == LCC_PZT_PWR_CONFIG_LENGTH)
	{
		uint8_t pzt_num = in->data[0];
		float *pzt_voltage = (float *) &in->data[1];
		uint8_t pzt_eeprom_write_option = in->data[5];
		if(pzt_num == LCC_PZT_NUM_1)
		{
			if(pzt_voltage_config(LCC_PZT_NUM_1, *pzt_voltage, pzt_eeprom_write_option))
			{
				SLOGF(SLOG_INFO, "Config PZT1 to %f (V) was succeeded", *pzt_voltage);
				pzt_config.pzt_pwr_1_voltage = *pzt_voltage;
				in->status = LCC_CMD_SUCCESS;
			}
			else
			{
				SLOGF(SLOG_ERROR, "Config POT of PZT1 was failed");
				in->status = LCC_CMD_UNSUCCESS;
			}
		}
		else if(pzt_num == LCC_PZT_NUM_2)
		{
			if(pzt_voltage_config(LCC_PZT_NUM_2, *pzt_voltage, pzt_eeprom_write_option))
			{
				SLOGF(SLOG_INFO, "Config PZT2 to %f (V) was succeeded", *pzt_voltage);
				pzt_config.pzt_pwr_2_voltage = *pzt_voltage;
				in->status = LCC_CMD_SUCCESS;
			}
			else
			{
				SLOGF(SLOG_ERROR, "Config POT of PZT2 was failed");
				in->status = LCC_CMD_UNSUCCESS;
			}
		}
		else
		{
			SLOGF(SLOG_ERROR, "The input param PZT to config %d is invalid", pzt_num);
			in->status = LCC_CMD_INVALID_ARG;
		}
	}
	else
	{
		SLOGF(SLOG_ERROR, "Msg length is invalid %d", in->len);
		in->status = LCC_CMD_INVALID_ARG;
	}
#endif /* ASIC_NUM == ASIC1*/
}
/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
