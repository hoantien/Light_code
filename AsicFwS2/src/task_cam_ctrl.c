/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    task_cam_ctrl.c
 * @author  The LightCo
 * @version V1.0.0
 * @date    Mar-10-2016
 * @brief   This file contains expand of task_cam_ctrl
 *
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "os.h"
#include "assert.h"
#include "log.h"
#include "lcc_cmd.h"
#include "lcc_cmd_log.h"
#include "light_system.h"
#include "hal_axi2mipi.h"
#include "hal_mipi2axi.h"
#include "usecase.h"
#include "task_cam_ctrl.h"

/* Private define-------------------------------------------------------------*/
#define SLOGF_ID				SLOG_ID_CAMERA_CTRL
#define EEPROM_UUID_OFFSET		0x151C
#define EEPROM_UUID_SIZE		16
#define UPDATE_CMD_STATUS(new_status, current_status)		(new_status \
														!= LCC_CMD_UNSUCCESS) ?\
											(current_status |= new_status ) : \
											(current_status = new_status)
/* Private macro -------------------------------------------------------------*/
/* Private function ----------------------------------------------------------*/
static uint16_t cam_ctrl_open(cam_typedef_t *pcam);
static uint16_t cam_ctrl_metadata(cam_typedef_t *pcam);
static eeprom_status_t reading_eeprom_uuid(cam_typedef_t *pcam);
/* Private variables----------------------------------------------------------*/
extern uint8_t tx_end_flag;
extern uint8_t rx_end_flag;

/* Export functions ----------------------------------------------------------*/

/*
 * task_cam_ctrl
 *
 */
void task_cam_create(void)
{
	uint8_t i;
	cam_typedef_t *pcam;
	char task_name[16];
	BaseType_t ret = pdFALSE;

	/* Camera task creation */
	for (i = 0; i < light_system->cam_tbl_size; i++)
	{
		pcam = &light_system->cam_tbl[i];
		if (pcam->info.ch > 0)
		{
			snprintf (task_name, 16, "cam_%x", pcam->info.module);
			ret = xTaskCreate(
								task_cam_ctrl,			/* task function */
								task_name,				/* task name */
								__TASK_STACK_SIZE_256,	/* stack size */
								pcam,					/* passing parameters */
								__TASK_PRIO_HIGHEST - 5,/* task priority */
								NULL					/* handle */
							);
			/* Assert the creation task return to make sure all of
				camera task was created */
			assert_param(pdPASS == ret);
		}
	}
}

/*
 * task_cam_ctrl
 *
 */
void task_cam_ctrl(void *vParameter)
{
	cam_typedef_t *pcam = (cam_typedef_t *)(vParameter);
	img_sensor_t *img = pcam->image;
	lcc_cmd_tid_t pcmd;
	uint8_t exit = 0;
	uint32_t cam_status = 0;
	uint32_t event;
	hal_gpio_t gpio;
	uint16_t cmd_status = 0;
	uint16_t cmd_tid;
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
	log_msg("Start %s: %x\r\n", __FUNCTION__, pcam->info.module);
	taskEXIT_CRITICAL();

	/*TODO: Initial for camera image sensor task */
	/* I2C initialization */
	i2cm.init(img->i2c_dev);

	/* GPIO */
	/* Camera standby */
	gpio.direction = GPIO_DIR_OUT;
	gpio.port = GPIO_PORT(img->standby);
	gpio.pin = GPIO_PIN(img->standby);
	hal_gpio_init(&gpio);
	hal_gpio_set_low(&gpio);

	/* PPG Init */
	/* Init default settings */
	/* Init .... */
	img_sensor_init(img);
	if(pcam->info.grp == GRP_A)
	{
		vcm_t *vcm = (vcm_t *)pcam->optical->lens;
		if(VCM_OK == vcm_init(vcm))
		{
			int16_t cur_pos = 0;
			vcm_get_hall(vcm, &cur_pos);
			uint8_t *lens_pos = pcam->optical->settings->lens_position;
			memcpy(lens_pos,(uint8_t *)&cur_pos, sizeof(int16_t));
			SLOGF(SLOG_INFO, "CAM-%X Current LENS position: 0x%x", pcam->info.module, cur_pos);
		}
	}
	/* read eeprom for cache */
	/* Verify ASIC 2 */
#if (ASIC_NUM == ASIC2)
	if(pcam->info.module == 0xA2)
	{
		if(EEPROM_OK == reading_eeprom_uuid(pcam))
			SLOGF(SLOG_INFO, "CAM-%X exists, so ASIC2 is valid",
				pcam->info.module);
		else
			SLOGF(SLOG_ERROR, "CAM-%X does not exist, so ASIC2 may be invalid"
				" please continue check CAM B3", pcam->info.module);
	}
	else if(pcam->info.module == 0xB3)
	{
		if(EEPROM_OK == reading_eeprom_uuid(pcam))
			SLOGF(SLOG_INFO, "CAM-%X exists, so ASIC2 is valid",
				pcam->info.module);
		else
			SLOGF(SLOG_ERROR, "CAM-%X does not exist, so ASIC2 is invalid",
				pcam->info.module);
	}
	else
	{
		reading_eeprom_uuid(pcam);
	}
#else
	reading_eeprom_uuid(pcam);
#endif /* (ASIC_NUM == ASIC2) */

	/* Attempt to create the event group. */
	pcam->queue = xQueueCreate(10, sizeof(lcc_cmd_tid_t));
	assert_param(pcam->queue);

	cam_status = S_MODULE_HW_STANDBY;
	memcpy(pcam->settings->status, (uint8_t *)&cam_status, sizeof(uint32_t));

	while (1)
	{
		/* Waiting for any event to be set within the event group.
		Clear the bits before exiting. */
		xQueueReceive(pcam->queue, &pcmd, portMAX_DELAY);
		cmd_tid = pcmd.cmd_tid;
		event = pcmd.event;
		/* Camera open processing */
		if (event & IMG_EVENT_CAM_OPEN)
		{
			uint16_t p_cmd_status = cam_ctrl_open(pcam);
			UPDATE_CMD_STATUS(p_cmd_status, cmd_status);
		}

		if(event & IMG_EVENT_CAM_STREAM)
		{
			uint16_t p_cmd_status = cam_ctrl_stream(pcam);
			UPDATE_CMD_STATUS(p_cmd_status, cmd_status);
		}

		/* The RESOLUTION command execution */
		if (event & IMG_EVENT_CAM_RESOLUTION)
		{
			uint16_t p_cmd_status = cam_ctrl_resolution(pcam);
			UPDATE_CMD_STATUS(p_cmd_status, cmd_status);
		}

		if (event & IMG_EVENT_CAM_SENSITIVITY)
		{
			uint16_t p_cmd_status = cam_ctrl_sensitivity(pcam);
			UPDATE_CMD_STATUS(p_cmd_status, cmd_status);
		}

		if (event & IMG_EVENT_CAM_FPS)
		{
			uint16_t p_cmd_status = cam_ctrl_fps(pcam);
			UPDATE_CMD_STATUS(p_cmd_status, cmd_status);
		}

		if (event & IMG_EVENT_CAM_EXPOSURE_TIME)
		{
			uint16_t p_cmd_status = cam_ctrl_exposure(pcam);
			UPDATE_CMD_STATUS(p_cmd_status, cmd_status);
		}

		if ((event & IMG_EVENT_VCM_POSITION) ||
			(event & IMG_EVENT_VCM_DISTANCE) ||
			(event & IMG_EVENT_VCM_NUDGE))
		{
			uint8_t vcm_status;
			vcm_t *vcm = (vcm_t *)pcam->optical->lens;
			int16_t current_hall = 0;
			if (event & IMG_EVENT_VCM_POSITION)
			{
				uint16_t *p_lens_position =
							(uint16_t *)pcam->optical->settings->lens_position;

				vcm_status = vcm_goto_hall(vcm, *p_lens_position);
			}
			else if (event & IMG_EVENT_VCM_DISTANCE)
			{
				uint32_t *p_focus_distance =
							(uint32_t *)pcam->optical->settings->focus_distance;

				vcm_status = vcm_focus_distance(vcm, *p_focus_distance);
			}
			else
			{
				vcm_status = vcm_get_hall(vcm, &current_hall);
				if (vcm_status == VCM_OK)
				{
					uint8_t *lens_nudge = pcam->optical->settings->lens_nudge;
					uint16_t lens_position =
							(lens_nudge[0] == CAM_DIR_EXTEND_WIDE) ?
								current_hall + *((uint16_t *)(lens_nudge + 1)) :
								current_hall - *((uint16_t *)(lens_nudge + 1));
					vcm_status = vcm_goto_hall(vcm, lens_position);
				}
			}
			vcm_get_hall(vcm, &current_hall);
			memcpy(pcam->optical->settings->lens_position,
											&current_hall, sizeof(uint16_t));
			/* take semaphore to access settings memory */
			xSemaphoreTake(vcm->semaphore, portMAX_DELAY);
			/* update moving status */
			vcm->moving = FALSE;
			/* release semaphore after accessing data */
			xSemaphoreGive(vcm->semaphore);
			/* call back to upper layer */
			if (vcm->cb_func != NULL)
			{
				vcm->cb_func(vcm->cb_param);
				/* disable call-back function in the case of LCC command */
				vcm->cb_func = NULL;
			}
			if(VCM_OK == vcm_status)
				cmd_status = LCC_CMD_SUCCESS;
			else
				cmd_status = LCC_CMD_UNSUCCESS;
		}

		if (event & IMG_EVENT_METADATA)
		{
			uint16_t p_cmd_status = cam_ctrl_metadata(pcam);
			UPDATE_CMD_STATUS(p_cmd_status, cmd_status);
		}

		lcc_cmd_log_update_status(cmd_tid, cmd_status, object_to_cam_bitmask(pcam));
	}
}

/*
 * @brief reading_eeprom_uuid
 * Read eeprom to initialize for light_system cache
 */
static eeprom_status_t reading_eeprom_uuid(cam_typedef_t *pcam)
{
	eeprom_status_t	eeprom_ret = EEPROM_OK;
	eeprom_ret = eeprom_read(pcam->image->i2c_dev, EEPROM_UUID_OFFSET,
			EEPROM_UUID_SIZE, pcam->settings->uuid);

	if (eeprom_ret == EEPROM_OK)
	{
#if(LOG_VERBOSE == STD_ON)
		/* print EEPROM data */
		char *str = assert_malloc(str, EEPROM_UUID_SIZE * 4 * sizeof(char));
		char *str_byte = assert_malloc(str_byte, 4 * sizeof(char));
		memset(str, 0, EEPROM_UUID_SIZE * 4);
		memset(str_byte, 0, 4);
		/* Dump data reading eeprom */
		for (int i = 0; i <  EEPROM_UUID_SIZE; ++i)
		{
			sprintf(str_byte, "%02x", pcam->settings->uuid[i]);
			strcat(str, str_byte);
			memset(str_byte, 0, 4);
		}
		SLOGF(SLOG_DEBUG, "Cam-%X UUID: %s", pcam->info.module, str);
		vPortFree(str);
		vPortFree(str_byte);
#endif
	}
	else
	{
		SLOGF(SLOG_ERROR, "Cam-%X: Reading EEPROM timed out",
				pcam->info.module);
	}
	return eeprom_ret;
}

static uint16_t cam_ctrl_open(cam_typedef_t *pcam)
{
	uint32_t cam_status;
	img_sensor_t *img = pcam->image;
	uint16_t cmd_status = LCC_CMD_PENDING;
	uint8_t cam_open;

	xSemaphoreTake(pcam->semaphore, portMAX_DELAY);
	/* Get camera status */
	memcpy(&cam_status, pcam->settings->status, sizeof(uint32_t));
	/* Get camera open */
	cam_open = pcam->settings->open;
	xSemaphoreGive(pcam->semaphore);

	if (S_MODULE_STREAM_ON & cam_status)
	{
		SLOGF(SLOG_INFO, "CAM-%X stream off first", pcam->info.module);
		return LCC_CMD_UNSUCCESS;
	}
	/* TODO: Need to review about cam_module_status */
	/* Checking the status of camera module was in SW standby or not */
	switch (cam_open)
	{
		case CAM_MODULE_CLOSE:
			if (!(S_MODULE_POWER_ON & cam_status))
			{
				SLOGF(SLOG_INFO, "CAM-%X already closed", pcam->info.module);
				cmd_status = LCC_CMD_SUCCESS;
				break;
			}
			img_sensor_close(img);
			/* Reset SCU */
			/* mipi_rx_reset(pcam->info.ch - 1); */

			SLOGF(SLOG_INFO, "CAM-%X changed to close", pcam->info.module);
			cam_status = 0;	/* Clear all camera status */
			cmd_status = LCC_CMD_SUCCESS;
			/*TODO: here, all camera power going down
			 * We can't recover it with just re-open in sw standby
			 */
			break;

		case CAM_MODULE_HW_STANDBY:
			if(cam_status & S_MODULE_HW_STANDBY)
			{
				SLOGF(SLOG_INFO, "CAM-%X already in HW standby",
											pcam->info.module);
				cmd_status = LCC_CMD_SUCCESS;
				break;
			}
			if(img_sensor_open(img, HW_STANDBY))
			{
				cam_status = S_MODULE_POWER_ON | S_MODULE_HW_STANDBY;
				SLOGF(SLOG_INFO, "CAM-%X changed to HW standby",
											pcam->info.module);
				cmd_status = LCC_CMD_SUCCESS;
			}
			else
			{
				cam_open = CAM_MODULE_CLOSE;
				SLOGF(SLOG_ERROR, "CAM-%X changed to HW standby failed",
														pcam->info.module);
				/* Update camera status */
				/* If HW_STANDBY called from SW_STANDBY */
				if(cam_status & S_MODULE_SW_STANDBY)
					cam_status = S_MODULE_POWER_ON | S_MODULE_SW_STANDBY |
						S_MODULE_CLOCK_ON |
						S_ERROR_SENSOR_I2C_DETECT_FAILURE |
						(cam_status | S_MODULE_STREAM_ON);
				/* If HW_STANDBY called from CAM_CLOSE */
				else
					cam_status = S_ERROR_SENSOR_I2C_DETECT_FAILURE;
				cmd_status = LCC_CMD_ERROR_MODULE_FAULT;
			}
			break;

		case CAM_MODULE_SW_STANDBY:
			if(cam_status & S_MODULE_SW_STANDBY)
			{
				SLOGF(SLOG_INFO, "CAM-%X already in SW standby",
										pcam->info.module);
				cmd_status = LCC_CMD_SUCCESS;
				break;
			}

			if (!(cam_status & S_MODULE_POWER_ON))
			{
				if(img_sensor_open(img, HW_STANDBY))
				{
					cam_status = S_MODULE_POWER_ON | S_MODULE_HW_STANDBY;
				}
				else
				{
					cam_open = CAM_MODULE_CLOSE;
					SLOGF(SLOG_ERROR, "CAM-%X changed to SW standby failed",
														pcam->info.module);
					/* Update camera status */
					cam_status = S_ERROR_SENSOR_I2C_DETECT_FAILURE;
					cmd_status = LCC_CMD_ERROR_MODULE_FAULT;
				}
			}

			/* Check if switching CAM to HW_STANDBY in previous is
			 * successfully or failed */
			if(cam_status & S_MODULE_POWER_ON)
			{
				if(img_sensor_open(img, SW_STANDBY))
				{
					/* Apply all settings */
					/*cam_ctrl_resolution(pcam);
					cam_ctrl_sensitivity(pcam);
					cam_ctrl_fps(pcam);
					cam_ctrl_exposure(pcam);*/

					/* Recover the status to un-error state */
					cam_open = CAM_MODULE_SW_STANDBY;
					cam_status = S_MODULE_POWER_ON |
						S_MODULE_SW_STANDBY | S_MODULE_CLOCK_ON;
					SLOGF(SLOG_INFO, "%s[%d]: Open CAM-%X was successfully",
						__FUNCTION__, __LINE__, pcam->info.module);
					cmd_status = LCC_CMD_SUCCESS;
				}
				else
				{
					/* Update cache of cam open */
					cam_open = CAM_MODULE_HW_STANDBY;
					/* We use write in open to send configuration
					 * to sensor */
					cam_status = S_MODULE_POWER_ON |
						S_MODULE_HW_STANDBY |
						S_ERROR_SENSOR_I2C_DETECT_FAILURE;
					cmd_status = LCC_CMD_ERROR_MODULE_FAULT;
					SLOGF(SLOG_ERROR, "%s:[%d] Cannot switch CAM-%X to SW STANDBY",
						__FUNCTION__, __LINE__, pcam->info.module);
				}
			}
			else
			{
				cam_open = CAM_MODULE_CLOSE;
				cam_status = S_ERROR_SENSOR_I2C_DETECT_FAILURE;
				cmd_status = LCC_CMD_ERROR_MODULE_FAULT;
				SLOGF(SLOG_ERROR, "%s:[%d] Cannot switch CAM-%X to HW STANDBY",
					__FUNCTION__, __LINE__, pcam->info.module);
			}
			break;
		default:
			break;
	}

	xSemaphoreTake(pcam->semaphore, portMAX_DELAY);
	/* Update camera status */
	memcpy(pcam->settings->status, &cam_status, sizeof(uint32_t));
	/* Update camera open */
	pcam->settings->open = cam_open;
	xSemaphoreGive(pcam->semaphore);
	return cmd_status;
}
extern SemaphoreHandle_t preview_sem;
uint16_t cam_ctrl_stream(cam_typedef_t *pcam)
{
	uint32_t cam_status = 0;
	img_sensor_t *img = pcam->image;

	xSemaphoreTake(pcam->semaphore, portMAX_DELAY);
	/* Get camera status */
	memcpy(&cam_status, pcam->settings->status, sizeof(uint32_t));
	xSemaphoreGive(pcam->semaphore);
	/* Stream off request */
	if(cam_status & S_MODULE_STREAM_ON)
	{
		uint8_t cam_ch = pcam->info.ch - 1;
		if(light_system->active_ucid == UCID_PREVIEW)
		{
			uint8_t vc = pcam->settings->stream[1];
			uint8_t tx = (pcam->settings->stream[0] & 0xF0) >> 4;
			stop_mipi_for_cam(cam_ch, vc, tx-1);
			uint32_t timeout = MIPI_STOP_WAIT_TIME;
			while((rx_end_flag & (1 << cam_ch)) && timeout)
			{
				timeout --;
				vTaskDelay(1);
			}
			if(timeout == 0)
			{
				SLOGF(SLOG_ERROR, "Failed to stop MIPI");
				xSemaphoreTake(preview_sem, portMAX_DELAY);
				rx_end_flag &= ~(1 << cam_ch);
				xSemaphoreGive(preview_sem);
			}
			hal_syncio_disable(cam_ch);
			hal_mipi2axi_stop(cam_ch);
			hal_axi2mipi_stop(tx-1, (1 << vc));
			img_sensor_stream_off(img, SLAVE_MODE);
			img_sensor_write_reg(img, LINE_LENGTH_PCLK_CAM_REG,
					pcam->image->default_llpclk, TWO_BYTES);
			SLOGF(SLOG_INFO, "Stream off CAM-%X", pcam->info.module);
		}
		else if(light_system->active_ucid == UCID_HIRES_CAPTURE)
		{
			img_sensor_stream_off(img, SLAVE_MODE);
			hal_syncio_disable(cam_ch);
			SLOGF(SLOG_INFO, "Stream off CAM-%X", pcam->info.module);
		}
		else
		{
			img_sensor_stream_off(img, I2C_MODE);
		}
		cam_status &= ~S_MODULE_STREAM_ON;
		xSemaphoreTake(pcam->semaphore, portMAX_DELAY);
		/* Update camera status */
		memcpy(pcam->settings->status,
						(uint8_t *)&cam_status, sizeof(uint32_t));
		xSemaphoreGive(pcam->semaphore);
	}
	else
		SLOGF(SLOG_DEBUG, "%s:[%d] CAM-%X are already stream off",
			__FUNCTION__, __LINE__, pcam->info.module);

	return LCC_CMD_SUCCESS;
}

uint16_t cam_ctrl_resolution(cam_typedef_t *pcam)
{
	img_sensor_t *img = pcam->image;
	cam_reg_array_t *reg = NULL;
	uint8_t ret = FALSE;
	uint16_t cmd_status = LCC_CMD_PENDING;
	uint16_t ucid_cur = light_system->active_ucid;
	if(pcam->settings->open == CAM_MODULE_SW_STANDBY)
	{

		uint32_t x_res, y_res;
		memcpy((uint8_t *)&x_res, pcam->image->settings
			[ucid_cur]->resolution, sizeof(uint32_t));
		memcpy((uint8_t *)&y_res, pcam->image->settings
			[ucid_cur]->resolution + sizeof(uint32_t),
			sizeof(uint32_t));
		if(pcam->image->x_size == x_res && pcam->image->y_size == y_res)
		{
			SLOGF(SLOG_DEBUG, "Resolution was not changed");
			if(light_system->active_ucid == UCID_HIRES_CAPTURE
					||light_system->active_ucid == UCID_HDR_CAPTURE)
				img_sensor_flip_capture(img, img->flip);
			else
			{
				img_sensor_flip_preview(img, img->flip);
			}
			cmd_status = LCC_CMD_SUCCESS;
			return cmd_status;
		}
#ifdef MIPI_SPEED_1500MHZ
		if(x_res == X_13M && y_res == Y_13M)
			reg = &resolution_testing_reg_array[4];
		else if(x_res == X_13M_P2 && y_res == Y_13M)
			reg = &resolution_testing_reg_array[0];
		else if(x_res == X_3M_P2 && y_res == Y_3M_P2)
			reg = &resolution_testing_reg_array[1];
		else if(x_res == X_4K_UHD && y_res == Y_4K_UHD_CINEMA)
			reg = &resolution_testing_reg_array[2];
		else if(x_res == X_1080P && y_res == Y_1080P)
			reg = &resolution_testing_reg_array[3];
		else
		{	/* FIXME: remove this piece of code below */
			SLOGF(SLOG_WARN, "%s:[%d] Invalid resolution params",
							__FUNCTION__, __LINE__);
			reg = &resolution_testing_reg_array[4];
		}
#else /* MIPI_SPEED_400MHZ */
		if(x_res == X_13M && y_res == Y_13M)
			reg = &resolution_testing_reg_array[2];
		else if(x_res == X_13M_P2 && y_res == Y_13M)
			reg = &resolution_testing_reg_array[0];
		else if(x_res == X_3M_P2 && y_res == Y_3M_P2)
			reg = &resolution_testing_reg_array[1];
		else
		{
			/* FIXME: remove this piece of code below */
			SLOGF(SLOG_WARN, "%s:[%d] Invalid resolution params",
				__FUNCTION__, __LINE__);
			reg = &resolution_testing_reg_array[2];
		}
#endif /* MIPI_SPEED */
		if(reg != NULL)
			ret = img_sensor_send_config(img, reg, 1);
		else
		{
			SLOGF(SLOG_WARN, "%s:[%d] Invalid resolution params",
				__FUNCTION__, __LINE__);
			return ret;
		}
		if(ret)
		{
			img->default_fll = img_sensor_read_reg(img, FLL_CAM_REG, TWO_BYTES);
			img->default_llpclk = img_sensor_read_reg(img,
				LINE_LENGTH_PCLK_CAM_REG, TWO_BYTES);
			img->x_size = img_sensor_read_reg(img,
												X_OUTPUT_SIZE_CAM_REG,
												TWO_BYTES);
			img->y_size = img_sensor_read_reg(img,
												Y_OUTPUT_SIZE_CAM_REG,
												TWO_BYTES);

			if(light_system->active_ucid == UCID_HIRES_CAPTURE
					||light_system->active_ucid == UCID_HDR_CAPTURE)
				img_sensor_flip_capture(img, img->flip);
			else
			{
				img_sensor_flip_preview(img, img->flip);
			}
		}

		if(ret)
		{
			SLOGF(SLOG_INFO,
			"CAM-%X : Resolution is updated to %d x %d",
			pcam->info.module, x_res, y_res);
			cmd_status = LCC_CMD_SUCCESS;
		}
		else
		{
			SLOGF(SLOG_WARN,
			"CAM-%X : Resolution update failed",
			pcam->info.module, x_res, y_res);
			cmd_status = LCC_CMD_UNSUCCESS;
		}
	}
	else
	{
		SLOGF(SLOG_INFO, "%s:[%d]: Store resolution for CAM-%X",
			__FUNCTION__, __LINE__, pcam->info.module);
		cmd_status = LCC_CMD_SUCCESS;
	}
	return cmd_status;
}

uint16_t cam_ctrl_sensitivity(cam_typedef_t *pcam)
{
	img_sensor_t *img = pcam->image;
	uint16_t cmd_status = 0;

	if(pcam->settings->open == CAM_MODULE_SW_STANDBY)
	{
		uint32_t sensitivity;
		uint16_t cfg;
		float *requested_gain = NULL;
		float total_gain, analog_gain;
		memcpy((uint8_t *)&sensitivity, pcam->image->settings
			[light_system->active_ucid]->sensitivity, sizeof(uint32_t));
		requested_gain = (float *)&(sensitivity);
		if(*requested_gain)
		{

			if (*requested_gain > 4.0f)
			{
				*requested_gain = 4.0f;
				SLOGF(SLOG_INFO, "%s:[%d]: Sensitivity of CAM-%X was clamped"
											" total gain to [%.02f]", __FUNCTION__, __LINE__,
											pcam->info.module, *requested_gain);
			}

			cfg = calculate_gain(&analog_gain, &total_gain, *requested_gain,
															light_system->active_ucid);
			/* Apply the gain configuration*/
			img_sensor_set_sensitivity(img, cfg);
			SLOGF(SLOG_INFO, "%s:[%d]: Sensitivity of CAM-%X was updated"
			" total gain to [%.02f]", __FUNCTION__, __LINE__,
			pcam->info.module, total_gain);
			cmd_status |= LCC_CMD_SUCCESS;
		}
		else
		{
			SLOGF(SLOG_WARN, "%s:[%d]: Sensitivity of CAM-%X"
			" was not updated", __FUNCTION__, __LINE__,
			pcam->info.module);
			cmd_status |= LCC_CMD_ERROR_MODULE_FAULT;
		}
	}
	else
	{
		SLOGF(SLOG_INFO, "%s:[%d]: Store sensitivity for CAM-%X",
			__FUNCTION__, __LINE__, pcam->info.module);
		cmd_status |= LCC_CMD_SUCCESS;
	}
	return cmd_status;
}

uint16_t cam_ctrl_fps(cam_typedef_t *pcam)
{
	uint16_t cmd_status = LCC_CMD_PENDING;

	if(pcam->settings->open == CAM_MODULE_SW_STANDBY)
	{
		uint16_t cam_fps = 0;
		memcpy((uint8_t *)&cam_fps, pcam->image->
			settings[light_system->active_ucid]->fps, sizeof(uint16_t));
		if(cam_fps)
		{
			/* Set fps value for camera*/
			img_sensor_set_fps(pcam, cam_fps);
			SLOGF(SLOG_INFO, "%s:[%d]: FPS of CAM-%X updated to %d ",
				__FUNCTION__, __LINE__, pcam->info.module, cam_fps);
			cmd_status = LCC_CMD_SUCCESS;
		}
		else
		{
			cmd_status = LCC_CMD_INVALID_ARG;
			SLOGF(SLOG_WARN, "%s:[%d] Invalid param", __FUNCTION__, __LINE__);
		}
	}
	else
	{
		SLOGF(SLOG_INFO, "%s:[%d]: Store fps for CAM-%X",
			__FUNCTION__, __LINE__, pcam->info.module);
		cmd_status = LCC_CMD_SUCCESS;
	}

	return cmd_status;
}

uint16_t cam_ctrl_exposure(cam_typedef_t *pcam)
{
	uint16_t cmd_status = LCC_CMD_PENDING;
	if(pcam->settings->open == CAM_MODULE_SW_STANDBY)
	{
		uint64_t exposure;
		memcpy((uint8_t *)&exposure, pcam->image->settings
			[light_system->active_ucid]->exposure, sizeof(uint64_t));
		if(exposure)
		{
			img_sensor_set_exposure_time(pcam, exposure);
			cmd_status = LCC_CMD_SUCCESS;
			SLOGF(SLOG_INFO, "CAM-%X applying exposure setting",
											pcam->info.module);
		}
		else
		{
			cmd_status = LCC_CMD_INVALID_ARG;
			SLOGF(SLOG_WARN, "%s:[%d] Invalid param", __FUNCTION__, __LINE__);
		}
	}
	else
	{
		SLOGF(SLOG_INFO, "%s:[%d]: Store exposure for CAM-%X",
			__FUNCTION__, __LINE__, pcam->info.module);
		cmd_status = LCC_CMD_SUCCESS;
	}

	return cmd_status;
}

static uint16_t cam_ctrl_metadata(cam_typedef_t *pcam)
{
	eeprom_status_t status = EEPROM_OK;
	if(pcam->eeprom.flag == EEPROM_ACTION_READ)
	{
		if(pcam->eeprom.buf)
		{
			status = eeprom_read(pcam->image->i2c_dev, pcam->eeprom.offset,
							pcam->eeprom.len,
							pcam->eeprom.buf);
			if (status != EEPROM_OK)
			{
				pcam->eeprom.flag = EEPROM_ACTION_FAILED;
				memset(pcam->eeprom.buf, 0xFF, sizeof(pcam->eeprom.len));
				SLOGF(SLOG_ERROR, "Cam-%X: Reading EEPROM timed out",
						pcam->info.module);
				return LCC_CMD_UNSUCCESS;
			}
			pcam->eeprom.flag = EEPROM_ACTION_SUCCESS;
		}
	}
	else if(pcam->eeprom.flag == EEPROM_ACTION_WRITE)
	{
		if(pcam->eeprom.buf)
		{
			status = eeprom_write(pcam->image->i2c_dev, pcam->eeprom.offset,
							pcam->eeprom.len,
							pcam->eeprom.buf);
			vPortFree(pcam->eeprom.buf);
			pcam->eeprom.buf = NULL;
			if (status != EEPROM_OK)
			{
				pcam->eeprom.flag = EEPROM_ACTION_FAILED;
				SLOGF(SLOG_ERROR, "Cam-%X: Write EEPROM timed out",
						pcam->info.module);
				return LCC_CMD_UNSUCCESS;
			}
			pcam->eeprom.flag = EEPROM_ACTION_SUCCESS;
		}
	}
	return LCC_CMD_SUCCESS;
}
/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
