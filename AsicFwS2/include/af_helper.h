/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification,are strictly prohibited without prior permission of The LightCo.
 *
 * @file    af_helper.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    Jul-06-2016
 * @brief   This file contains expand of the af helper function
 *
 ******************************************************************************/
#ifndef __AF_HELPER_H__
#define __AF_HELPER_H__

#include "cortex_r4.h"
#include "std_type.h"
#include "assert.h"
#include "ar1335.h"
#include "hal_axi2mipi.h"
#include "hal_mipi2axi.h"
#include "hal_syncio.h"
#include "hal_pwm.h"
#include "timer.h"
#include "log.h"
#include "light_system.h"
#include "usecase.h"
#include "usecase_hires.h"
#include "task_cam_ctrl.h"
#include "optical.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    FLIP_MODE_CAPTURE,
    FLIP_MODE_NONE
} flip_mode_af_t;

 /*
 * @brief af_setting_t
 */
typedef struct
{

	cam_typedef_t *cam;		/* Camera object */
	uint8_t mipi_rx_chid;		/* Mipi RX channel Id */
	uint16_t x_position;		/* ROI x position */
	uint16_t y_position;		/* ROI y position */
	uint16_t x_width;		/* ROI x width */
	uint16_t y_width;		/* ROI y width */
	uint16_t scale;         /* resolution scale used to compute output size */
	uint32_t destination;		/* Address which stored snapshot frame */
	mipi2axi_callback af_cb;	/* Mipi interrupt callback */
	void *cb_user_data;     	/* user_data arg for callback */
	img_data_t cam_settings;	/* Image sensor setting */
	flip_mode_af_t flip_mode;   /* flip mode to be used */
} af_setting_t;

 /**
 * @brief setup_roi_mipi
 * This function used to configure ROI setting and mipi rx snapshot
 * @param pcam: the camera channel will take snapshot
 * @param x_position: ROI x position
 * @param y_position: ROI y position
 * @param x_width: ROI x width
 * @param y_width: ROI y width
 * @param destination: Address which the frame will be dumped to
 * @param mipi_rx_chid: the mipi rx channel will take snapshot
 * @param af_cb: the call back function
 * @retval refer
 */
int setup_roi_mipi(cam_typedef_t *pcam, uint16_t x_position,
		uint16_t y_position, uint16_t x_width, uint16_t y_width, uint16_t scale,
		uint32_t destination, uint8_t mipi_rx_chid, mipi2axi_callback af_cb,
		void* cb_user_data);

int af_configure_sensor_stream(af_setting_t *af_setting, uint8_t is_preview_cam);

int trigger_sensor(af_setting_t *af_setting);

flip_mode_t get_af_flip_mode(flip_mode_af_t flip_mode_af, const img_sensor_t* img_sensor);

void af_test(cam_typedef_t *pcam);

#ifdef __cplusplus
}
#endif
#endif /* __AF_HELPER_H__ */
