/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    optical.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    Mar-10-2016
 * @brief   This file contains header of optical system group BC control
 *
 ******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __OPTICAL_H__
#define __OPTICAL_H__

/* Includes ------------------------------------------------------------------*/
#include "os.h"
#include "log.h"
#include "lcc_cmd.h"
#include "vcm.h"
#include "lcc_system.h"


#ifdef __cplusplus
extern "C" {
#endif


/* Exported define -----------------------------------------------------------*/
#define OPT_EVENT_FOCUS_DISTANCE		BIT0
#define OPT_EVENT_LENS_POSITION			BIT1
#define OPT_EVENT_MIRROR_POSITION		BIT2
#define OPT_EVENT_LENS_NUDGE			BIT3
#define OPT_EVENT_MIRROR_NUDGE			BIT4
#define OPT_EVENT_MIRROR_CALIB			BIT5
#define OPT_EVENT_ALLS					((OPT_EVENT_MIRROR_CALIB << 1) - 1)

/* Exported typedef ----------------------------------------------------------*/
/*
 * @brief opt_settings_t
 * Image sensor setting for each of stream on UCID
 */
typedef struct
{
	uint8_t focal_length[2];
	uint8_t focus_distance[4];
	uint8_t lens_position[2];
	uint16_t lens_position_tolerance;
	uint8_t mirr_position[2];
	uint8_t lens_nudge[3];
	uint8_t mirr_nudge[3];
} opt_data_t;

/*
 * @brief optical_t
 * Optical system definition
 */
typedef struct
{
	void *lens;			/* Lens using for focus to target object */
	void *mirr;			/* Mirror using for change angle of lens system */
	opt_data_t *settings;		/* Data caching for each of UCID */
	SemaphoreHandle_t	semaphore;		/* Semaphore to access settings */
} optical_t;

/* Exported functions --------------------------------------------------------*/
/*
 * @brief task_actuator_create
 * The actuator task create, number of task depend on ASIC
 * @param	None
 * @return	None
 */
void task_actuator_create(void);
/**
 * @brief task_actuator_lens
 * This task will process all updated data of actuator lens
 * @param vParameter: point to parameter if used
 * @return None
 */
void task_actuator_lens(void *vParameter);
/**
 * @brief task_actuator_mirr
 * This task will process all updated data of actuator mirror
 * @param vParameter: point to parameter if used
 * @return None
 */
void task_actuator_mirr(void *vParameter);



#ifdef __cplusplus
}
#endif
#endif /* __OPTICAL_H__ */

/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
