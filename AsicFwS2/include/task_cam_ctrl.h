/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    task_cam_ctrl.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    Mar-10-2016
 * @brief   This file contains header of task_cam_ctrl
 *
 ******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TASK_CAM_CTRL_H__
#define __TASK_CAM_CTRL_H__

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
#define IMG_EVENT_CAM_OPEN				BIT0
#define IMG_EVENT_CAM_STREAM			BIT1
#define IMG_EVENT_CAM_FPS				BIT2
#define IMG_EVENT_CAM_RESOLUTION		BIT3
#define IMG_EVENT_CAM_SENSITIVITY		BIT4
#define IMG_EVENT_CAM_EXPOSURE_TIME		BIT5
#define IMG_EVENT_CAM_EEPROM			BIT6
#define IMG_EVENT_VCM_DISTANCE			BIT7
#define IMG_EVENT_VCM_POSITION			BIT8
#define IMG_EVENT_METADATA				BIT9
#define IMG_EVENT_VCM_NUDGE				BIT10
#define IMG_EVENT_ALLS					((IMG_EVENT_VCM_NUDGE << 1) - 1)

/* Exported typedef  ---------------------------------------------------------*/
/**
* @brief cam_module_cmd_t typedef
* Camera module status
*/
typedef enum cam_module_cmd
{
	CAM_MODULE_CLOSE		= 0,
	CAM_MODULE_HW_STANDBY	= 1,
	CAM_MODULE_SW_STANDBY	= 2,
	CAM_MODULE_MODE_MAX
} cam_mode_t;

/**
* @brief cam_status_t typedef
* Camera status
*/
typedef enum cam_status
{
	S_MODULE_POWER_ON					= 1 << 0,
	S_MODULE_CLOCK_ON					= 1 << 1,
	S_MODULE_HW_STANDBY					= 1 << 2,
	S_MODULE_SW_STANDBY					= 1 << 3,
	S_MODULE_STREAM_ON					= 1 << 4,
	S_ERROR_SENSOR_I2C_DETECT_FAILURE	= 1 << 5,
	S_ERROR_I2C_READ_FAILURE			= 1 << 6,
	S_ERROR_I2C_WRITE_FAILURE			= 1 << 7,
	S_MODULE_HALL_I2C_DETECT_FAILURE	= 1 << 8,
	S_ERROR_MIRROR_DETECT				= 1 << 9,
	S_ERROR_LENS_DETECT					= 1 << 10,
	S_MOVE_LENS_FAILURE					= 1 << 11,
	S_MOVE_MIRROR_FAILURE				= 1 << 12,
	S_UCID_FAILURE						= 1 << 13,
} cam_status_t;

/* Exported functions --------------------------------------------------------*/
/**
 * @brief task_cam_create
 * The camera task create, number of task depend on ASIC
 * @param	None
 * @return	None
 */
void task_cam_create(void);
/**
 * @brief task_cam_ctrl
 * This task will process all updated data of cameras
 * @param vParameter: point to parameter if used
 * @return None
 */
void task_cam_ctrl(void *vParameter);

uint16_t cam_ctrl_stream(cam_typedef_t *pcam);

uint16_t cam_ctrl_resolution(cam_typedef_t *pcam);

uint16_t cam_ctrl_exposure(cam_typedef_t *pcam);
uint16_t cam_ctrl_sensitivity(cam_typedef_t *pcam);
uint16_t cam_ctrl_fps(cam_typedef_t *pcam);


#ifdef __cplusplus
}
#endif
#endif /* __TASK_CAM_CTRL_H__ */

/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
