/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    qc_camera.h
 * @author  The LightCo
 * @version V1.0.1
 * @date    26-Jul-2016
 * @brief   This file contains expand of the qc_camera driver
 *
 ******************************************************************************/

/******************************************************************************/
/**								Revision history
 * * 1.0.0	26-Jul-2016	Initial revision:
 *                      - Infrastructure.
 */
/******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _QC_CAMERA_H_
#define _QC_CAMERA_H_
/* Includes ------------------------------------------------------------------*/
#include "qc_common.h"
#include "it_log_swapper.h"
#include "i2cm.h"
#include "ar1335.h"
#include "hal_gpio.h"
#include "hal_syncio.h"
#include "board_config.h"

/* Exported define------------------------------------------------------------*/

/**
* @brief cam_channel_t typedef
* Camera channel
*/
typedef enum cam_channel
{
	CAM_CH_A1 = I2C_CH0,
	CAM_CH_A2 = I2C_CH1,
	CAM_CH_A3 = I2C_CH2,
	CAM_CH_A4 = I2C_CH3,
	CAM_CH_A5 = I2C_CH4,
	CAM_CH_B1 = I2C_CH5,
	CAM_CH_B2 = 6,
	CAM_CH_B3 = 8,
	CAM_CH_B4 = 9,
	CAM_CH_B5 = 10,
	CAM_CH_C1 = 11,
	CAM_CH_C2 = 12,
	CAM_CH_C3 = 13,
	CAM_CH_C4 = 14,
	CAM_CH_C5 = 15,
	CAM_CH_C6 = 16,
	CAM_CH_MAX_NUM
} cam_channel_t;

/**
* @brief cam_type typedef
* Type of camera, This should be corresponding with predifined in lookup_tbl.mk
*/
typedef enum cam_type
{
	CAM_TYPE_28MM =		28,			/* APTINA_28MM */
	CAM_TYPE_35MM =		35,			/* APTINA_35MM */
	CAM_TYPE_70MM =		70,			/* APTINA_70MM */
	CAM_TYPE_150MM =	150			/* APTINA_150MM */
} cam_type_t;

/**
* @brief cam_data_mode_t typedef
* Indicate the size of access data
*/
typedef enum cam_data_mode
{
	DATA_8BIT = 1,
	DATA_16BIT = 2
} cam_data_mode_t;

/**
* @brief cam_module_t typedef
* Camera structure
*/
typedef struct cam_module
{
	/* Camera module channel id */
	const cam_channel_t	chid;
	/* Camera module name */
	const char 			*name;
	/* Camera module type */
	const cam_type_t	type;
	/* Camera module slave addr */
	const uint8_t		slave_addr;
} cam_module_t;

/* Exported typedef ----------------------------------------------------------*/
#define ONE_SECOND 			(50000000)
#define MICROSECOND_TIME	(ONE_SECOND / 1000000)
#define MILISECOND_TIME		(ONE_SECOND / 1000)

#define  CAM_PRE_BUILD_CREATE(c_name, c_chid, c_type, c_slave) \
{							\
	.chid = c_chid,			\
	.name = #c_name,		\
	.type = c_type,			\
	.slave_addr = c_slave	\
}

typedef struct
{
	hal_syncio_channel_t	channel;
	syncio_infinite_mode_t	inf_mode;		/*!< Infinite mode */
	syncio_trig_mode_t		trig_mode;		/*!< Trigger mode */
	uint32_t				lat_pulse;		/*!< Latency 1 */
	uint32_t				width_pulse;	/*!< Pulse width 1 */
	uint8_t					num_pulse;		/*!< Pulse generator repeat cycle */
} cam_syncio_cfg_t;

/* Exported functions ------------------------------------------------------- */
int cam_write_reg_t(volatile cam_module_t *,
		uint16_t , uint16_t data, uint8_t d_mode);
i2cm_error_t i2cm_transceiver_t(i2c_t,uint8_t,uint8_t *, size_t
								,void *, void *);
int open_cam(uint8_t);
int cam_open_t(volatile cam_module_t *cam);
uint8_t cam_set_reg_noncontinuous_t(cam_module_t *, uint16_t ,
		msm_camera_i2c_reg_array_t *,  cam_data_mode_t );
uint8_t cam_set_reg_continuous_t(cam_module_t *, uint16_t ,
		msm_camera_i2c_reg_array_t *,  cam_data_mode_t );

void cam_configure(volatile cam_module_t *);
void cam_syncio_init(cam_module_t *, cam_syncio_cfg_t *);
void init_syncio(uint8_t, uint8_t);
void cam_syncio_stream_on(hal_syncio_channel_t );

#endif /* _QC_CAMERA_H_ */
/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
