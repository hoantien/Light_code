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
#include "ar1335.h"
#include "camera.h"
#include "vcm.h"
#include "hal_gpio.h"
#include "lcc_cmd.h"
#include "hal_mipi2axi.h"
#include "hal_axi2mipi.h"
//TODO: for testing
#include "register_dump.h"
/* Private define-------------------------------------------------------------*/
#define 	SLOGF_ID						SLOG_ID_CAMERA_CTRL
#define 	SM_IDLE							0x00
#define 	SM_UPDATE_CHANGE				0x01

#define		X_ADDR_START_CAM_REG			0x0344
#define		X_ADDR_END_CAM_REG				0x0348

#define		Y_ADDR_START_CAM_REG			0x0346
#define		Y_ADDR_END_CAM_REG				0x034A

#define 	STREAM_ON						1
#define 	STREAM_OFF						0
#define		STREAM_MASK						(BIT0 | BIT1 | BIT2 | BIT3)
#define		SHIFT_BIT30		30

uint8_t is_preview_stop;
uint8_t mipirx_instance;
uint8_t mipi_data_type;
uint8_t mipi_vc_channel;
uint8_t axi2mipi_fifo_x;

/* Private macro -------------------------------------------------------------*/
#define  CAM_PRE_BUILD_CREATE(c_name, c_chid, c_type, c_slave) \
{							\
	.chid = c_chid,			\
	.name = #c_name,		\
	.type = c_type,			\
	.slave_addr = c_slave	\
}
/* Private function ----------------------------------------------------------*/

static void cam_update_changes(uint8_t ucid_mode, cam_module_t *cam);
uint8_t mipi_preview_config(cam_module_t *cam);
/* Private variables----------------------------------------------------------*/

/* Camera list */
static cam_module_t cam_module_list[CAM_CH_MAX_NUM] =
{
	CAM_PRE_BUILD_CREATE(0, 0, 0, 0),
	CAM_PRE_BUILD_CREATE(CAM_A1, CAM_CH_A1, CAM_TYPE_35MM, 0x6C >> 1),	/* CAM A1 */
	CAM_PRE_BUILD_CREATE(CAM_A2, CAM_CH_A2, CAM_TYPE_35MM, 0x6C >> 1),	/* CAM A2 */
	CAM_PRE_BUILD_CREATE(CAM_A3, CAM_CH_A3, CAM_TYPE_35MM, 0x6C >> 1),	/* CAM A3 */
	CAM_PRE_BUILD_CREATE(CAM_A4, CAM_CH_A4, CAM_TYPE_35MM, 0x6C >> 1),	/* CAM A4 */
	CAM_PRE_BUILD_CREATE(CAM_A5, CAM_CH_A5, CAM_TYPE_35MM, 0x6C >> 1),	/* CAM A5 */
	CAM_PRE_BUILD_CREATE(CAM_B1, CAM_CH_B1, CAM_TYPE_70MM, 0x6C >> 1),	/* CAM B1 */
	CAM_PRE_BUILD_CREATE(CAM_B2, CAM_CH_B2, CAM_TYPE_70MM, 0x6C >> 1),	/* CAM B2 */
	CAM_PRE_BUILD_CREATE(CAM_B3, CAM_CH_B3, CAM_TYPE_70MM, 0x6C >> 1),	/* CAM B3 */
	CAM_PRE_BUILD_CREATE(CAM_B4, CAM_CH_B4, CAM_TYPE_70MM, 0x6C >> 1),	/* CAM B4 */
	CAM_PRE_BUILD_CREATE(CAM_B5, CAM_CH_B5, CAM_TYPE_70MM, 0x6C >> 1),	/* CAM B5 */
	CAM_PRE_BUILD_CREATE(CAM_C1, CAM_CH_C1, CAM_TYPE_150MM, 0x6C >> 1),	/* CAM C1 */
	CAM_PRE_BUILD_CREATE(CAM_C2, CAM_CH_C2, CAM_TYPE_150MM, 0x6C >> 1),	/* CAM C2 */
	CAM_PRE_BUILD_CREATE(CAM_C3, CAM_CH_C3, CAM_TYPE_150MM, 0x6C >> 1),	/* CAM C3 */
	CAM_PRE_BUILD_CREATE(CAM_C4, CAM_CH_C4, CAM_TYPE_150MM, 0x6C >> 1),	/* CAM C4 */
	CAM_PRE_BUILD_CREATE(CAM_C5, CAM_CH_C5, CAM_TYPE_150MM, 0x6C >> 1),	/* CAM C5 */
	CAM_PRE_BUILD_CREATE(CAM_C6, CAM_CH_C6, CAM_TYPE_150MM, 0x6C >> 1),	/* CAM C6 */
};

/* Export functions ----------------------------------------------------------*/
cam_reg_array_t *resolution_selection(uint16_t width, uint16_t height,
													uint16_t fps, uint8_t *size)
{
	unsigned int i = 0;
	msm_camera_stream_type_t *stream_type = NULL;
	for(i = 0; i < stream_types_size; i++)
	{
		stream_type = (msm_camera_stream_type_t *)&(stream_types[i]);
		if(stream_type->fps == fps
				&& stream_type->height == height
				&& stream_type->width == width)
		{
			if(size)
				*size = stream_type->sizeofconfig;
			return stream_type->config;
		}
	}
	return NULL;
}
/*
 * task_cam_ctrl
 *
 */
void task_cam_ctrl(void *vParameter)
{
	task_handle_t *hdl = (task_handle_t *)(vParameter);
	uint8_t exit = 0;
	uint8_t cam_scan_idx = CAM_A1_INDEX;
	uint32_t check_flag_eeprom = 0;
	uint8_t sm = SM_UPDATE_CHANGE;
	uint8_t i = 0;
	/* List of camera module controlled by Light System */
	light_system->cam_list = cam_module_list;
	hal_gpio_t gpio;
	gpio.port = GPIO_PORTB;
	gpio.direction = GPIO_DIR_OUT;
	gpio.pin = GPIO_PIN_0;
	hal_gpio_init(&gpio);
	hal_gpio_set_high(&gpio);
	for(i = 1 ; i <= 6; i++)
	{
		gpio.pin = i;
		hal_gpio_init(&gpio);
		hal_gpio_set_high(&gpio);
	}
	/* Start initialize #__task_cam_ctrl is dependency to #task_console */
	while (!exit)
	{
		/* #task_console done for slogf */
		if((TASK_READY == task_handler[task_query_tid("lcccmd")].state) &&
		   (TASK_READY == task_handler[task_query_tid("cam_dump")].state))
			exit = 1;
		vTaskDelay(1);
	}
	/* Task ready id */
	hdl->state = TASK_READY;
	/* Task start */
	taskENTER_CRITICAL();
	log_msg("Start %s\r\n", __FUNCTION__);
	taskEXIT_CRITICAL();
	light_system->task_cam_ctrl = xSemaphoreCreateBinary();
	xSemaphoreTake(light_system->task_cam_ctrl, 0);
	/* Initialize cam module */
	while(cam_scan_idx < CAM_CH_MAX_NUM)
	{
		cam_init(&light_system->cam_list[cam_scan_idx]);
		cam_scan_idx++;
	}
	cam_scan_idx = CAM_A1_INDEX;
	while(1)
	{
		if (sm == SM_IDLE)
		{
			xSemaphoreTake(light_system->task_cam_ctrl, portMAX_DELAY);
			sm = SM_UPDATE_CHANGE;
		}
		if (cam_scan_idx < CAM_CH_MAX_NUM)
		{
			cam_update_changes(light_system->ucid_mode,
									&light_system->cam_list[cam_scan_idx]);

			if (light_system->cam_list[cam_scan_idx].eeprom.flag &
											CAM_EEPROM_MODULE_READ_FLAG)
			{
				check_flag_eeprom |= (1 << cam_scan_idx) ;
			}
			else if (light_system->cam_list[cam_scan_idx].eeprom.flag &
											CAM_EEPROM_MODULE_WRITE_FLAG)
			{
				check_flag_eeprom |= (1 << cam_scan_idx);
			}
			else
			{
				check_flag_eeprom &= ~(1 << cam_scan_idx);
			}
			cam_scan_idx++;
			/* Unlock task cam ctrl */
			xSemaphoreGive(sempr_cam_register_dump);
			/* Sleep for 1 tick */
			vTaskDelay(1);
		}
		else
		{
			cam_scan_idx = CAM_A1_INDEX;
			if ((check_flag_eeprom & 0xFFFFFFFF) == 0)
			{
				sm = SM_IDLE;
			}
		}
	}
}
/*
 * cam_update_changes
 */
static void cam_update_changes(uint8_t ucid_mode, cam_module_t *cam)
{
	uint16_t loop_index = 0;
	/* Check camera module open/close */
	if(cam->updated_flag & CAM_UPDATED_M_O_STATUS)
	{
		/* clear flag */
		cam->updated_flag &= ~CAM_UPDATED_M_O_STATUS;
		SLOGF(SLOG_INFO, "%s status has changed to %s mode",
				cam->name, cam->m_o_status == CAM_MODULE_CLOSE ? "CLOSE" :
				cam->m_o_status == CAM_MODULE_SW_STANDBY ? "SW STANDBY" :
				cam->m_o_status == CAM_MODULE_HW_STANDBY ? "HW STANDBY" :
				COLOR_YELLOW"Unknown"COLOR_WHITE
		);
		if((cam->ucid[ucid_mode].host_updated & CAM_UPDATED_FPS)
			&& (cam->ucid[ucid_mode].host_updated & CAM_UPDATED_RESOLUTION)
			&& (cam->ucid[ucid_mode].host_updated & CAM_UPDATED_EXPOSURE)
			&& (cam->ucid[ucid_mode].host_updated & CAM_UPDATED_SENSITIVITY)
			&& (cam->m_o_status == CAM_MODULE_SW_STANDBY))
		{
			uint32_t host_updated = cam->ucid[ucid_mode].host_updated;
			/* Apply setting for camera module */
			cam_open(cam);
			if(cam->m_o_status != CAM_MODULE_SW_STANDBY)
			{
				SLOGF(SLOG_ERROR, "Open camera %s failed", cam->name);
				return;
			}
			cam->ucid[ucid_mode].updated_flag = host_updated;
			/* Check camera module type is 35MM */
			switch (cam->type)
			{
				case CAM_TYPE_28MM:
				case CAM_TYPE_35MM:
				{
					if(cam->m_o_status == CAM_MODULE_SW_STANDBY)
					{
						SLOGF(SLOG_INFO, "Initialize VCM module");
						drv_vcm_init(cam->chid);
					}
					if (host_updated & CAM_UPDATED_VCM_POSITION)
						cam->ucid[ucid_mode].drive.updated_flag |=
													CAM_UPDATED_VCM_POSITION;
					break;
				}
				case CAM_TYPE_70MM:
				case CAM_TYPE_150MM:
				{
					if(host_updated & CAM_UPDATED_LENS_POSITION)
						cam->ucid[ucid_mode].drive.updated_flag |=
													CAM_UPDATED_LENS_POSITION;
					if(host_updated & CAM_UPDATED_MIRROR_POSITION)
						cam->ucid[ucid_mode].drive.updated_flag |=
												CAM_UPDATED_MIRROR_POSITION;
					break;
				}
				default:
				{
					SLOGF(SLOG_WARN,
						"%s: %s Unsupported Camera Module Type %d !",
						__FUNCTION__,
						lcc_ucid_mode_text(light_system->ucid_mode), cam->type);
					break;
				}
			}
		}
		else if(cam->m_o_status == CAM_MODULE_SW_STANDBY)
		{
			uint32_t host_updated = cam->ucid[ucid_mode].host_updated;
			/* Apply sensor setting */
			cam_open(cam);
			if(cam->m_o_status != CAM_MODULE_SW_STANDBY)
			{
				SLOGF(SLOG_ERROR, "Open camera %s failed", cam->name);
				return;
			}
			if(cam->m_o_status == CAM_MODULE_SW_STANDBY &&
				(cam->type == CAM_TYPE_35MM || cam->type == CAM_TYPE_28MM))
			{
				SLOGF(SLOG_INFO, "Initialize VCM module");
				drv_vcm_init(cam->chid);
			}
			/* Check host was set FPS */
			if(!(host_updated & CAM_UPDATED_FPS))
			{
				SLOGF(SLOG_ERROR,
					"%s: Host must updates FPS for %s !", __FUNCTION__,
					lcc_ucid_mode_text(light_system->ucid_mode));
					lcc_cmd_log_update_status(light_system->cmd_tid_cur,
															ERROR_MODULE_FAULT);
			}
			/* Check host was set resolution */
			if(!(host_updated & CAM_UPDATED_RESOLUTION))
			{
				SLOGF(SLOG_ERROR,
					"%s: Host must updates RESOLUTION for %s !", __FUNCTION__,
					lcc_ucid_mode_text(light_system->ucid_mode));
				lcc_cmd_log_update_status(light_system->cmd_tid_cur,
															ERROR_MODULE_FAULT);
			}
			/* Check host was set Exposure time */
			if(!(host_updated & CAM_UPDATED_EXPOSURE))
			{
				SLOGF(SLOG_ERROR,
					"%s: Host must updates EXPOSURE for %s !", __FUNCTION__,
					lcc_ucid_mode_text(light_system->ucid_mode));
					lcc_cmd_log_update_status(light_system->cmd_tid_cur,
															ERROR_MODULE_FAULT);
			}
			/* Check host was set Sensitivity */
			if(!(host_updated & CAM_UPDATED_SENSITIVITY))
			{
				SLOGF(SLOG_ERROR,
					"%s: Host must updates SENSITIVITY for %s !", __FUNCTION__,
					lcc_ucid_mode_text(light_system->ucid_mode));
				lcc_cmd_log_update_status(light_system->cmd_tid_cur,
															ERROR_MODULE_FAULT);
			}
		}
		else if(cam->m_o_status == CAM_MODULE_HW_STANDBY)
		{
			/* do nothing */
			cam_close(cam);
			lcc_cmd_log_update_status(light_system->cmd_tid_cur, CMD_SUCCESS);
		}
		/* CAM_MODULE_CLOSE */
		else
		{
			cam_close(cam);
			/* do nothing */
			lcc_cmd_log_update_status(light_system->cmd_tid_cur, CMD_SUCCESS);
		}
	}
	/* Stream is updated. */
	if(cam->updated_flag & CAM_UPDATED_M_STREAM)
	{
		/* Clear flag for the next time. */
		cam->updated_flag &= ~CAM_UPDATED_M_STREAM;

		SLOGF(SLOG_INFO, "%s STREAM %s Virtual Channel 0x%02X, Data Type %02X  were changed",
			cam->name,
			(cam->m_stream.ctrl_status & 0x01) ? "ENABLE" : "DISABLE",
			cam->m_stream.vc, cam->m_stream.dt
			);
		if((cam->m_stream.ctrl_status & STREAM_MASK) == STREAM_ON)
		{
			mipi_preview_config(cam);
			/* Start stream on */
			cam_write_reg(cam, 0x3F3C, 0x0003, DATA_16BIT);
			cam_write_reg(cam, 0x0100, 0x01, DATA_8BIT);
			SLOGF(SLOG_INFO, "%s: STREAM ON %s!", __FUNCTION__, cam->name);
		}
		else
		{
			cam_write_reg(cam, 0x3F3C, 0x0002, DATA_16BIT);
			cam_write_reg(cam, 0x3FE0, 0x0001, DATA_16BIT);
			cam_write_reg(cam, 0x0100, 0x00, DATA_8BIT);
			cam_write_reg(cam, 0x3FE0, 0x0000, DATA_16BIT);
			mipi_streamming_setup(cam);
			SLOGF(SLOG_INFO, "%s: STREAM OFF %s!", __FUNCTION__, cam->name);
		}
		lcc_cmd_log_update_status(light_system->cmd_tid_cur, CMD_SUCCESS);
	}
	/* Sensitivity is updated */
	if(cam->ucid[ucid_mode].updated_flag & CAM_UPDATED_SENSITIVITY)
	{
		/* Clear flag */
		cam->ucid[ucid_mode].updated_flag &= ~CAM_UPDATED_SENSITIVITY;
		if(cam->m_o_status == CAM_MODULE_SW_STANDBY)
		{
			SLOGF(SLOG_INFO, "%s %s Sensitivity updated to 0x%04X ",
				cam->name, lcc_ucid_mode_text(ucid_mode),
				cam->ucid[ucid_mode].sensitivity);
			/* Open Control Group in camera module */
			cam_ctrl_reg_group(cam, ON);
			/* Send command to camera module */
			cam_write_reg(cam, CAM_REG_SENSITIVITY,
								cam->ucid[ucid_mode].sensitivity, DATA_16BIT);
			/* Close Control Group in camera module */
			cam_ctrl_reg_group(cam, OFF);
			lcc_cmd_log_update_status(light_system->cmd_tid_cur, CMD_SUCCESS);
		}
		else
		{
			lcc_cmd_log_update_status(light_system->cmd_tid_cur,
															ERROR_MODULE_FAULT);
		}
	}
	if(cam->ucid[ucid_mode].updated_flag & CAM_UPDATED_FOCAL_LEN)
	{
		cam->ucid[ucid_mode].updated_flag &= ~CAM_UPDATED_FOCAL_LEN;
		if(cam->m_o_status == CAM_MODULE_SW_STANDBY)
		{
			SLOGF(SLOG_INFO, "%s:%d: Focal length updated to %d on %s",
					__FUNCTION__, __LINE__,
				cam->ucid[ucid_mode].focal_len, cam->name);
			lcc_cmd_log_update_status(light_system->cmd_tid_cur, CMD_SUCCESS);
		}
		else
		{
			SLOGF(SLOG_ERROR, "%s:%d: Camera module %s isn't in SW_STANDBY",
					__FUNCTION__, __LINE__, cam->name);
			lcc_cmd_log_update_status(light_system->cmd_tid_cur,
															ERROR_MODULE_FAULT);
		}
	}
	/* Sensitivity is updated */
	if(cam->ucid[ucid_mode].updated_flag & CAM_UPDATED_EXPOSURE)
	{
		/* Clear flag */
		cam->ucid[ucid_mode].updated_flag &= ~CAM_UPDATED_EXPOSURE;
		if(cam->m_o_status == CAM_MODULE_SW_STANDBY)
		{
			SLOGF(SLOG_INFO, "%s %s Exposure has updated to 0x%llx",
					cam->name, lcc_ucid_mode_text(ucid_mode),
					cam->ucid[ucid_mode].exposure_time);
			/* Open Control Group in camera module */
			cam_ctrl_reg_group(cam, ON);
			/* send command to camera module */
			cam_write_reg(cam, CAM_REG_EXPOSURE,
					cam->ucid[ucid_mode].exposure_time, DATA_16BIT);
			/* Close Control Group in camera module */
			cam_ctrl_reg_group(cam, OFF);
			lcc_cmd_log_update_status(light_system->cmd_tid_cur, CMD_SUCCESS);
		}
		else
		{
			lcc_cmd_log_update_status(light_system->cmd_tid_cur,
			ERROR_MODULE_FAULT);
		}
	}
	/* FPS is updated */
	if(cam->ucid[ucid_mode].updated_flag & CAM_UPDATED_FPS)
	{
		cam->ucid[ucid_mode].updated_flag &= ~CAM_UPDATED_FPS;
		if(cam->m_o_status == CAM_MODULE_SW_STANDBY)
		{
			SLOGF(SLOG_INFO, "%s: %s %s updated fps to 0x%04X",
				__FUNCTION__, cam->name,
				lcc_ucid_mode_text(ucid_mode),
				cam->ucid[ucid_mode].fps);
			lcc_cmd_log_update_status(light_system->cmd_tid_cur,
														CMD_SUCCESS);
		}
		else
		{
			lcc_cmd_log_update_status(light_system->cmd_tid_cur,
														ERROR_MODULE_FAULT);
		}
	}
	/* Resolution is updated */
	if(cam->ucid[ucid_mode].updated_flag & CAM_UPDATED_RESOLUTION)
	{
		cam->ucid[ucid_mode].updated_flag &= ~CAM_UPDATED_RESOLUTION;
		if(cam->m_o_status != CAM_MODULE_SW_STANDBY)
		{
			lcc_cmd_log_update_status(light_system->cmd_tid_cur,
														ERROR_MODULE_FAULT);
		}
		else if(cam_is_fps_supported(cam->ucid[ucid_mode].fps))
		{
			uint8_t index;
			cam_reg_array_t *data_config = NULL;
			data_config = resolution_selection(cam->ucid[ucid_mode].res.x,
				cam->ucid[ucid_mode].res.y, cam->ucid[ucid_mode].fps, &index);
			/* Found stream type, start sending configurations */
			if(data_config)
			{
				msm_camera_i2c_reg_array_t *preg = NULL;
				loop_index = 0;
				while(loop_index < index)
				{
					preg = data_config[loop_index].regs;
					if(data_config[loop_index].reg_type == CAM_REG_CONTINUOUS)
					{
						cam_set_reg_continuous(cam,
								data_config[loop_index].reg_size, preg,
								data_config[loop_index].data_size);
					}
					else
					{
						cam_set_reg_noncontinuous(cam,
								data_config[loop_index].reg_size, preg,
								data_config[loop_index].data_size);
					}
					loop_index++;
				}
				lcc_cmd_log_update_status(light_system->cmd_tid_cur,
														CMD_SUCCESS);
			}
		}
		else
		{
			SLOGF(SLOG_ERROR, "%s %s has not been set FPS",
					cam->name, lcc_ucid_mode_text(ucid_mode));
			lcc_cmd_log_update_status(light_system->cmd_tid_cur,
														ERROR_MODULE_FAULT);
		}

	}

	if(cam->ucid[ucid_mode].drive.updated_flag & CAM_UPDATED_VCM_POSITION)
	{
		cam->ucid[ucid_mode].drive.updated_flag &= ~CAM_UPDATED_VCM_POSITION;
		if(cam->m_o_status == CAM_MODULE_SW_STANDBY)
		{
			SLOGF(SLOG_INFO, "%s %s VCM position updated to 0x%04X ",
					cam->name, lcc_ucid_mode_text(ucid_mode),
					cam->ucid[ucid_mode].drive.vcm_position);
			drv_vcm_move_to_hall(cam->chid,
								cam->ucid[ucid_mode].drive.vcm_position);
			lcc_cmd_log_update_status(light_system->cmd_tid_cur, CMD_SUCCESS);
		}
#if(LOG_VERBOSE == STD_ON)
		else
		{
			lcc_cmd_log_update_status(light_system->cmd_tid_cur,
														ERROR_MODULE_FAULT);
		}
#endif
	}

	if (cam->ucid[ucid_mode].updated_flag & CAM_UPDATED_FOCUS_DISTANCE)
	{
		cam->ucid[ucid_mode].updated_flag &= ~CAM_UPDATED_FOCUS_DISTANCE;
		SLOGF(SLOG_INFO, "%s %s CAM FOCUS DISTANCE updated to 0x%04X ",
				cam->name, lcc_ucid_mode_text(ucid_mode),
				cam->ucid[ucid_mode].focus_distance);
		if (cam->type == CAM_TYPE_35MM)
			drv_vcm_focus(cam->chid, cam->ucid[ucid_mode].focus_distance);
		lcc_cmd_log_update_status(light_system->cmd_tid_cur, CMD_SUCCESS);
	}
	/* Read camera module METADATA */
	if(cam->eeprom.flag & CAM_EEPROM_MODULE_READ_FLAG)
	{
		cam_eeprom_read(cam);
		lcc_cmd_log_update_status(light_system->cmd_tid_cur, CMD_SUCCESS);
	}
	/* Write camera module METADATA */
	if(cam->eeprom.flag & CAM_EEPROM_MODULE_WRITE_FLAG)
	{
		cam_eeprom_write(cam);
		lcc_cmd_log_update_status(light_system->cmd_tid_cur, CMD_SUCCESS);
	}
}

/* TODO */
void mipi_rx_isr(uint8_t iidx,	mipi2axi_irq_raw_num_t intnum, uint32_t detail)
{
	if(is_preview_stop)
	{
		hal_mipi2axi_stop(iidx);
	}
}

void mipi_tx_hdl_isr(axi2mipi_channel_t channel, axi2mipi_isr_flag_t detail)
{
	if(is_preview_stop)
	{
		if(detail & FE_INT_VC0)
			hal_axi2mipi_stop(channel, AXI2MIPI_FIFO_A);
		if(detail & FE_INT_VC1)
			hal_axi2mipi_stop(channel, AXI2MIPI_FIFO_B);
		if(detail & FE_INT_VC2)
			hal_axi2mipi_stop(channel, AXI2MIPI_FIFO_C);
		if(detail & FE_INT_VC3)
			hal_axi2mipi_stop(channel, AXI2MIPI_FIFO_D);
		/* printf("Stopped MIPI TX \r\n"); */
	}
}

uint8_t mipi_preview_config(cam_module_t *cam)
{
	uint16_t x_start, x_end, y_start, y_end;
	uint16_t width;
	uint16_t heigth;

	mipirx_instance = cam->chid;
	mipi_data_type = cam->m_stream.dt;
	mipi_vc_channel = cam->m_stream.vc;

	SLOGF(SLOG_DEBUG, "CAMERA CH[%d]", mipirx_instance);

	switch(mipi_vc_channel)
	{
		case VC0_FOR_FIFO:
			axi2mipi_fifo_x = AXI2MIPI_FIFO_A;
			break;
		case VC1_FOR_FIFO:
			axi2mipi_fifo_x = AXI2MIPI_FIFO_B;
			break;
		case VC2_FOR_FIFO:
			axi2mipi_fifo_x = AXI2MIPI_FIFO_C;
			break;
		case VC3_FOR_FIFO:
			axi2mipi_fifo_x = AXI2MIPI_FIFO_D;
			break;
	}


	if((cam->m_stream.ctrl_status & STREAM_MASK) == STREAM_ON)
	{
		is_preview_stop = DISABLE;

		/* Read camera resolution */
		x_start = cam_read_reg(cam, X_ADDR_START_CAM_REG, DATA_16BIT);
		x_end = cam_read_reg(cam, X_ADDR_END_CAM_REG, DATA_16BIT);
		width = x_end - x_start;
		width = ((width & 1) == 1) ? (width + 1) : width;
		SLOGF(SLOG_INFO, "CAMERA WIDTH : DEC[%d] HEX[0x%04X]", width, width);

		y_start = cam_read_reg(cam, Y_ADDR_START_CAM_REG, DATA_16BIT);
		y_end = cam_read_reg(cam, Y_ADDR_END_CAM_REG, DATA_16BIT);
		heigth = y_end - y_start;
		heigth = ((heigth & 1) == 1) ? (heigth + 1) : heigth;
		SLOGF(SLOG_INFO, "CAMERA HEIGHT: DEC[%d] HEX[0x%04X]", heigth, heigth);

		/*
		 * CONFIGURE MIPI TO AXI -----------------------------------------------
		 */
		hal_mipi2axi_init(mipirx_instance);

		/* Configure TX base address */
		hal_mipi2axi_set_tx_base_addr(mipirx_instance, TXS0_BASE);

		/* Configure stream mode */
		mipi2axi_property_t mipirx_opt;
		mipirx_opt.vc_option = MIPI2AXI_VC0_CH;
		mipirx_opt.addr.dst_addr = (uint32_t)(mipi_vc_channel << SHIFT_BIT30);
		mipirx_opt.img_type.vc_will_captured	= MIPI2AXI_VC0_CH;
		mipirx_opt.img_type.data_type			= MIPI2AXI_RAW10;
		mipirx_opt.img_size.height				= heigth;
		mipirx_opt.img_size.width				= width;
		mipirx_opt.stream_mode					= MIPI2AXI_PREVIEW_MODE;
		hal_mipi2axi_set_stream_property(mipirx_instance, &mipirx_opt);

		/* Configure interrupt */
		mipi2axi_interrupt_t mipi2axi_irq;
		mipi2axi_irq.irq_num = MASK_INT1;
		mipi2axi_irq.irq_detail = R1_VC0_FRAME_START | R1_VC0_FRAME_END;
		hal_mipi2axi_irq_init(mipirx_instance, &mipi2axi_irq);

		mipi2axi_irq.irq_num = MASK_INT2;
		mipi2axi_irq.irq_detail = 0xffffffff;
		hal_mipi2axi_irq_init(mipirx_instance, &mipi2axi_irq);

		mipi2axi_callback callback_hdl = mipi_rx_isr;
		hal_mipi2axi_irq_enable(mipirx_instance, callback_hdl);

		/* Start MIPI 2 AXI */
		hal_mipi2axi_start(mipirx_instance);

		/*
		 * CONFIGURE AXI TO MIPI -----------------------------------------------
		 */
		/* Initialize DPHY */
		hal_axi2mipi_init(AXI2MIPI_BRIDGE_0);


		/* Configure stream property */
		axi2mipi_property_t axi2mipi_opt;
		axi2mipi_opt.fifo_x				= axi2mipi_fifo_x;
		axi2mipi_opt.src_addr			= TXS0_BASE;
		axi2mipi_opt.vc_for_ff			= mipi_vc_channel; /* VC0_FOR_FIFO; */
		axi2mipi_opt.stream_mode		= AXI2MIPI_PREVIEW_MODE;
		axi2mipi_opt.img_size.height	= heigth;
		axi2mipi_opt.img_size.width		= width;
		axi2mipi_opt.img_type.data_type = AXI2MIPI_DATA_RAW_10;
		axi2mipi_opt.img_type.ff_bypass = NONE_BYPASS;
		axi2mipi_opt.wrap.wrap_en		= DISABLE;
		axi2mipi_opt.wrap.wrap_num		= 0;
		hal_axi2mipi_set_stream_property(AXI2MIPI_BRIDGE_0, &axi2mipi_opt);

		/* Configure interrupt */
		axi2mipi_isr_flag_t axi2mipi_interrupt;
		axi2mipi_interrupt = 0xffffffff;
		hal_axi2mipi_irq_init(AXI2MIPI_BRIDGE_0, axi2mipi_interrupt);

		axi2mipi_callback tx_callback;
		tx_callback = mipi_tx_hdl_isr;
		hal_axi2mipi_irq_enable(AXI2MIPI_BRIDGE_0, tx_callback);

		/* Configure valid */
		hal_axi2mipi_programming_valid(AXI2MIPI_BRIDGE_0, axi2mipi_fifo_x, ENABLE);

		/* Configure continues */
		hal_axi2mipi_set_frame_mode(AXI2MIPI_BRIDGE_0, axi2mipi_fifo_x, FRAME_CONTINUOUS);

		/* Start AXI to MIPI */
		hal_axi2mipi_start(AXI2MIPI_BRIDGE_0, axi2mipi_fifo_x);
	}
	else
	{
		is_preview_stop = ENABLE;
	}

	return 0;
}
/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
