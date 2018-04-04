/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    light_system.c
 * @author  The LightCo
 * @version V1.0.0
 * @date    Juk-16-2016
 * @brief   This file contains expand for Light system object
 *
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "std_type.h"
#include "assert.h"
#include "log.h"
#include "os.h"
#include "board_config.h"
#include "lcc_system.h"
#include "lcc_cmd.h"
#include "task_ccb_ctrl.h"
#include "task_cam_ctrl.h"
#include "light_system.h"
#include "img_sensor.h"
#include "usecase.h"
#include "tof.h"
#include "gyro.h"
#include "timer.h"
#include "ina231.h"

/* Private define-------------------------------------------------------------*/
#define SLOGF_ID						SLOG_ID_CCB_CTRL

/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables----------------------------------------------------------*/
/* Exported Global variables--------------------------------------------------*/
/* Private function-----------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

#if (ASIC_NUM == ASIC1)
#ifdef USING_GYRO_FUNC
static void gyro_timer_handler(gyro_info_t *gyro_info)
{
	gyro_info->count++;
	if(gyro_info->count == gyro_info->sample_num)
		gyro_info->flag = GYRO_FLAG_DONE;
	xEventGroupSetBitsFromISR(light_system->event, CCB_EVENT_GYRO, NULL);
}
#endif /* USING_GYRO_FUNC */

static void flash_timer_handler(lm3644_info_t *flash_info)
{
	flash_info->flag = FLASH_FLAG_DONE;
	xEventGroupSetBitsFromISR(light_system->event, CCB_EVENT_FLASH, NULL);
}
#endif /* (ASIC_NUM == ASIC1) */

/*
 * task_cam_ctrl
 *
 */
void task_ccb_ctrl(void *vParameter)
{
	task_handle_t *hdl = (task_handle_t *)(vParameter);
	EventBits_t event;
	uint16_t cmd_tid = 0;
	uint8_t exit = 0;
	uint8_t cam_idx;
	uint32_t m_bitmask;
	uint8_t  m_number;
	uint32_t cam_status;
	cam_typedef_t *pcam;
#if (ASIC_NUM == ASIC1)
	int ret;
	struct ltimer_t *flash_timer = NULL;
	lm3644_info_t *Flash_info = &light_system->flash_dev;
#ifdef USING_TOF_FUNC
	VL53L0X_Dev_t *ToF_Dev = &light_system->ToF_dev;
#endif
#ifdef USING_GYRO_FUNC
	gyro_info_t *gyro_dev = &light_system->gyro_dev;
#endif /* USING_GYRO_FUNC */
	ina231_info_t ina231[3] = {
		{
			.addr = 0x80 >> 1,
			.current_unit = CURRENT_LSB_1mA,
			.rsense = 8
		},
		{
			.addr = 0x8A >> 1,
			.current_unit = CURRENT_LSB_1mA,
			.rsense = 8
		},
		{
			.addr = 0x82 >> 1,
			.current_unit = CURRENT_LSB_1mA,
			.rsense = 8
		}
	};
#endif /* (ASIC_NUM == ASIC1) */

	/* Start initialize #task_cam_ctrl is dependency to #lcccmd */
	while (!exit)
	{
		/* #lcccmd done for slogf */
		if(TASK_READY == task_handler[task_query_tid("lcccmd")].state)
			exit = 1;
		vTaskDelay(1);
	}

	/* Task start */
	taskENTER_CRITICAL();
	log_msg("Start %s\r\n", __FUNCTION__);
	taskEXIT_CRITICAL();
#if (ASIC_NUM == ASIC1)

	pzt_voltage_config(PZT_POT_1, PZT1_VOLTAGE_DEFAULT_VALUE, 0);
	pzt_voltage_config(PZT_POT_2, PZT2_VOLTAGE_DEFAULT_VALUE, 0);

	if(LM3644_OK == flash_light_init(HAL_I2C_MASTER, Flash_info))
		SLOGF(SLOG_INFO, "Flash LED Init OK");
	else
		SLOGF(SLOG_ERROR, "Flash LED Init Failed");
#ifdef USING_TOF_FUNC
	uint8_t Status;
	ToF_Dev->I2cDevAddr = CAM_TOF_ADDRESS;
	Status = Init_ToF_Device(ToF_Dev);
#endif
#ifdef USING_GYRO_FUNC
	struct ltimer_t *gyro_timer = NULL;
	light_system->gyro_dev.i2c_addr = CAM_GYRO_ADDRESS;
	gyro_init(HAL_I2C_MASTER, &light_system->gyro_dev);
#endif

	for (int i = 0; i < sizeof(ina231)/sizeof(ina231_info_t); i++)
	{
		SLOGF(SLOG_INFO, "INA231 [%d]", i);
		ret = ina231_config(I2C_CH10, &ina231[i]);
		vTaskDelay(1);
		if (ret != INA231_OK)
			SLOGF(SLOG_ERROR, "Error [%x]\n", ret);
	}

	/* Init to using temp sensor */
	temp_sensor_init();
    log_msg("PZT Voltage 12 V \r\n");
#else /* (ASIC_NUM == ASIC1) */
    vTaskDelay(5);
#endif
	hdl->state = TASK_READY;
	/* Attempt to create the event group. */
	light_system->event = xEventGroupCreate();
	assert_param(light_system->event);

	while (1)
	{
		/* Waiting for any event to be set within the event group.
		Clear the bits before exiting. */
		event = xEventGroupWaitBits(light_system->event,
									CCB_EVENT_ALLS,
									pdTRUE, pdFALSE, portMAX_DELAY);
		cmd_tid = light_system->cur_tid;
		/* Every UCID should be unique at the system runtime
		 * No more than 1 UCID is active
		 */
		/* The DEBUG UCID */
		if (event & CCB_EVENT_STREAM_UCID_DEBUG)
		{
			uint16_t cmd_status = LCC_CMD_PENDING;
			m_bitmask = light_system->m_filter;
			m_number = __builtin_popcount(m_bitmask);
			for(uint8_t i = 0; i < m_number; i++)
			{
				cam_idx = __builtin_ctz(m_bitmask);
				m_bitmask &= ~(1 << cam_idx);
				if(NULL != (pcam = idx_to_object(cam_idx)))
				{
					xSemaphoreTake(pcam->semaphore, portMAX_DELAY);
					memcpy((uint8_t *)&cam_status, pcam->settings->status,
						sizeof(uint32_t));
					xSemaphoreGive(pcam->semaphore);
					if(cam_status & S_MODULE_SW_STANDBY)
					{
						if(cam_status & S_MODULE_STREAM_ON)
						{
							img_sensor_stream_on(pcam->image, I2C_MODE);
							cam_status |= S_MODULE_STREAM_ON;
							xSemaphoreTake(pcam->semaphore, portMAX_DELAY);
							memcpy(pcam->settings->status,
								(uint8_t *)&cam_status, sizeof(uint32_t));
							xSemaphoreGive(pcam->semaphore);
						}
						else
							SLOGF(SLOG_WARN, "%s:[%d] The CAM-%X is already"
								" stream ON", __FUNCTION__, __LINE__,
								pcam->info.module);
						cmd_status = LCC_CMD_SUCCESS;
					}
					else
					{
						cmd_status = LCC_CMD_UNSUCCESS;
						SLOGF(SLOG_WARN, "%s:[%d] The CAM-%X is not in SW"
							" Standby", __FUNCTION__, __LINE__,
							pcam->info.module);
					}
				}
				else
				{
					cmd_status = LCC_CMD_ERROR_MODULE_FAULT;
					SLOGF(SLOG_ERROR, "%s:[%d] Can't found cam module idx %d",
						__FUNCTION__, __LINE__, cam_idx);
				}
				lcc_cmd_log_update_status(cmd_tid,
					cmd_status, cam_idx);
			}
		}
#if 0
		/* The PREVIEW UCID */
		if ((event & CCB_EVENT_PREVIEW) || (event & CCB_EVENT_SNAPSHOT_HDR))
		{
			uint16_t cmd_status = LCC_CMD_PENDING;
			m_bitmask = light_system->m_filter;
			m_number = __builtin_popcount(m_bitmask);
			uint32_t preview_mask = 0;

			for (uint8_t i = 0; i < m_number; i++)
			{
				cam_idx = __builtin_ctz(m_bitmask);
				m_bitmask &= ~(1 << cam_idx);
				if(NULL != (pcam = idx_to_object(cam_idx)))
				{
					xSemaphoreTake(pcam->semaphore, portMAX_DELAY);
					memcpy(&cam_status, pcam->settings->status,
							sizeof(uint32_t));
					xSemaphoreGive(pcam->semaphore);
					/* TODO: Should fix magic number */
					if(cam_status & S_MODULE_SW_STANDBY)
					{
						if(!(cam_status & S_MODULE_STREAM_ON))
							preview_mask |= (1 << cam_idx);
						/* Cam status will be updated inside the usecase id
						 * function */
						/* Limitation: Cmd status will be incorrect when
						 * streaming on was failed */
						cmd_status = LCC_CMD_SUCCESS;
					}
					else
					{
						cmd_status = LCC_CMD_UNSUCCESS;
						SLOGF(SLOG_WARN, "%s:[%d] The CAM-%X is not in SW"
						" Standby", __FUNCTION__, __LINE__,
						pcam->info.module);
					}
				}
				else
				{
					cmd_status = LCC_CMD_ERROR_MODULE_FAULT;
					SLOGF(SLOG_ERROR, "%s:[%d] Cannot found CAM module idx %X",
						__FUNCTION__, __LINE__, cam_idx);
				}
				lcc_cmd_log_update_status(cmd_tid,
					cmd_status, cam_idx);
			}

			if (preview_mask)
			{
				m_number = __builtin_popcount(preview_mask);
				if (event & CCB_EVENT_PREVIEW)
					ucid_preview_hdl(preview_mask, m_number);
				else
					ucid_hires_hdl(preview_mask, m_number);
			}
		}
#endif
#if (ASIC_NUM == ASIC1)
		if(event & CCB_EVENT_FLASH)
		{
			uint16_t cmd_status = LCC_CMD_PENDING;
			if(Flash_info->flag == FLASH_FLAG_FLASH)
			{
				Flash_info->flag = FLASH_FLAG_BUSY;
				if(LM3644_OK == flash_light_control(HAL_I2C_MASTER, Flash_info))
				{
					if(Flash_info->flash_mode == FLASH_FLASH_MODE ||
					Flash_info->flash_mode == FLASH_IR_MODE)
					{
						if(flash_timer == NULL)
						{
							flash_timer = timer_create();
							if(NULL != flash_timer)
							{
								flash_timer->autoreload = OFF;
								flash_timer->interval =
									Flash_info->flash_time * 100;
								flash_timer->handler =
									(void *)flash_timer_handler;
								flash_timer->param = (void *)Flash_info;
								flash_triger_pin(ON);
								timer_start(flash_timer);
							}
							else
							{
								cmd_status = LCC_CMD_UNSUCCESS;
								SLOGF(SLOG_ERROR, "%s[%d] Timer create failed",
									__FUNCTION__, __LINE__);
							}
						}
						else
						{
							flash_timer->interval =
								Flash_info->flash_time * 100;
							flash_triger_pin(ON);
							timer_start(flash_timer);
							cmd_status = LCC_CMD_PENDING;
						}
					}
					Flash_info->flag = FLASH_FLAG_TRIGGER_FLASH;
				}
				else
				{
					cmd_status = LCC_CMD_ERROR_MODULE_FAULT;
					Flash_info->flag = FLASH_FLAG_ERROR;
					SLOGF(SLOG_ERROR, "%s:[%d] Flash failed", __FUNCTION__,
						__LINE__);
				}
			}
			else if(Flash_info->flag == FLASH_FLAG_DONE)
			{
				flash_triger_pin(OFF);
				if(flash_timer != NULL)
				{
					timer_stop(flash_timer);
					timer_delete(&flash_timer);
					flash_timer = NULL;
				}
				cmd_status = LCC_CMD_SUCCESS;
				SLOGF(SLOG_INFO, "FLASH DONE!!!");
			}
			lcc_cmd_log_update_status(cmd_tid,
					cmd_status, 0);
		}
#ifdef USING_TOF_FUNC
		if(event & CCB_EVENT_TOF)
		{
			if(ToF_Dev->Flag == VL53L0X_FLAG_EXECUTE)
			{
				Status = Perform_ToF_Device(ToF_Dev);
				if(VL53L0X_ERROR_NONE == Status)
				{
					ToF_Dev->Flag = VL53L0X_FLAG_DONE;
					SLOGF(SLOG_INFO, "ToF perform done!!!");
					lcc_cmd_log_update_status(cmd_tid,
						LCC_CMD_SUCCESS, 0);
				}
				else
				{
					ToF_Dev->Flag = VL53L0X_FLAG_ERROR;
					lcc_cmd_log_update_status(cmd_tid,
						LCC_CMD_UNSUCCESS, 0);
					SLOGF(SLOG_ERROR, "ToF perform failed, ERROR: %d",
						Status);
				}
			}
		}
#endif /* USING_TOF_FUNC */
#ifdef USING_GYRO_FUNC
		if(event & CCB_EVENT_GYRO)
		{
			if(gyro_dev->flag == GYRO_FLAG_READ)
			{
				if(gyro_timer == NULL)
				{
					gyro_dev->flag = GYRO_FLAG_GET_SAMPLE;
					gyro_timer = timer_create();
					if(NULL != gyro_timer)
					{
						gyro_timer->autoreload = OFF;
						gyro_timer->interval = gyro_dev->interval_us;
						gyro_timer->handler = (void *)gyro_timer_handler;
						gyro_timer->param = (void *)gyro_dev;
						if(GYRO_OK == gyro_get_info(HAL_I2C_MASTER, gyro_dev))
						{
							memcpy(&gyro_dev->buf[gyro_dev->count *
								LCC_CMD_GYRO_DATA_SIZE],
								gyro_dev->gyro_data.buf,
								LCC_CMD_GYRO_DATA_SIZE);
							timer_start(gyro_timer);
						}
						else
						{
							lcc_cmd_log_update_status(cmd_tid,
								LCC_CMD_UNSUCCESS, 0);
							SLOGF(SLOG_ERROR, "Read Gyro failed");
						}
					}
					else
					{
						lcc_cmd_log_update_status(cmd_tid,
								LCC_CMD_UNSUCCESS, 0);
						SLOGF(SLOG_ERROR, "%s[%d] Timer create failed",
							__FUNCTION__, __LINE__);
					}
				}
				else
				{
					gyro_timer->interval = gyro_dev->interval_us;
					timer_start(gyro_timer);
				}
			}
			else if(gyro_dev->flag == GYRO_FLAG_DONE)
			{
				if(gyro_timer != NULL)
				{
					timer_stop(gyro_timer);
					timer_delete(&gyro_timer);
					gyro_timer = NULL;
				}

				lcc_cmd_log_update_status(cmd_tid,
					LCC_CMD_SUCCESS, 0);
				SLOGF(SLOG_INFO, "Get sample %d times done!!!",
					gyro_dev->count);
			}
			else if(gyro_dev->flag == GYRO_FLAG_GET_SAMPLE)
			{
				if(GYRO_OK == gyro_get_info(HAL_I2C_MASTER, gyro_dev))
				{
					memcpy(&gyro_dev->buf[gyro_dev->count *
						LCC_CMD_GYRO_DATA_SIZE], gyro_dev->gyro_data.buf,
						LCC_CMD_GYRO_DATA_SIZE);
					timer_start(gyro_timer);
				}
				else
				{
					/* Terminate getting sample sequence */
					if(gyro_timer != NULL)
					{
						timer_stop(gyro_timer);
						timer_delete(&gyro_timer);
						gyro_timer = NULL;
					}
					lcc_cmd_log_update_status(cmd_tid,
						LCC_CMD_UNSUCCESS, 0);
					SLOGF(SLOG_ERROR, "Get sample %d failed, process will be"
						" terminated", gyro_dev->sample_num);
				}
			}
		}
#endif /* USING_GYRO_FUNC */
		if(event & CCB_EVENT_POWER)
		{
			ret = ina231_get_info(I2C_CH10, &ina231[light_system->power_ch]);
			if(ret)
				SLOGF(SLOG_ERROR, "%s[%d] Error: 0x%02x",
									__FUNCTION__, __LINE__, ret);
			else
				SLOGF(SLOG_INFO,
						"INA[%d]: Volt: %d(mV), Current: %d(mA), Power: %d(mW)",
						light_system->power_ch,
						ina231[light_system->power_ch].volt,
						ina231[light_system->power_ch].curr,
						ina231[light_system->power_ch].power);

			light_system->volt[light_system->power_ch] =
					ina231[light_system->power_ch].volt;
			light_system->curr[light_system->power_ch] =
					ina231[light_system->power_ch].curr;
			light_system->power[light_system->power_ch] =
					ina231[light_system->power_ch].power;

			lcc_cmd_log_update_status(cmd_tid, LCC_CMD_SUCCESS, 0);
		}
#endif /* (ASIC_NUM == ASIC1) */
	}
}
/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
