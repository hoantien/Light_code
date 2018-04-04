/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms,
 * with or without modification,
 * are strictly prohibited without prior permission of The LightCo.
 *
 * @file    usecase_hires.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    July-07-2016
 * @brief   This file contains functions for snapshot camera
 *
 ******************************************************************************/
/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __USECASE_HIRES_H__
#define __USECASE_HIRES_H__

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "os.h"
#include "std_type.h"
#include "hal_syncio.h"
#include "timer.h"

/* Exported typedef -----------------------------------------------------------*/
typedef struct
{
	volatile uint8_t active;	/* this variable indicate that camera is active or not */
	volatile uint8_t fs_flag;/* this variable indicate that MIPI has frame start */
	volatile uint8_t fe_flag;/* this variable indicate that MIPI has frame end */
	volatile uint8_t fs_cnt; /* this variable used to count the number of frame start */
	volatile uint8_t fe_cnt; /* this variable used to count the number of frame end */
	volatile uint8_t fdone_cnt; /* this variable used to count when finish a frame */
} mipi_frame_t;

typedef struct
{
	volatile uint8_t fs_flag;	/* frame start */
	volatile uint8_t fe_flag;	/* frame end */
	volatile uint32_t fs_mask;	/* frame start mask */
	volatile uint32_t fe_mask;	/* frame end mask */
} mipi_tx_frame_t;

typedef struct capt_priv_data
{
	volatile uint32_t mbitmask;
	volatile EventGroupHandle_t hires_evhdl;
	volatile uint16_t lh_ev_cnt;
	volatile uint64_t max_it;
} capt_priv_data_t;
/*
 * CAM return status
 */
typedef enum usecase_hires_cfg_status
{
	USERCASE_HIRES_NO_ERROR = 0,
	USECASE_HIRES_CAM_TOTAL_ERROR,
	USECASE_HIRES_CAM_RESOLUTION_ERROR,
	USECASE_HIRES_CAM_FRAME_ERROR,
	USECASE_HIRES_CAM_IMG_DATA_TYPE_ERROR,
	USECASE_HIRES_CSI_INVALID,
	USECASE_HIRES_VC_THUMBNAIL_INVALID,
	USECASE_HIRES_VC_SNAPSHOT_INVALID
} usecase_hires_cfg_status_t;

typedef enum ucid_hires_events
{
	EVENT_RX_FRAME_START				= BIT3,
	EVENT_LIGHT_HEADER_READY			= BIT0,
	EVENT_TX_FRAME_COMPLETE				= BIT1,
	EVENT_SNAPSHOT_COMPLETE				= BIT2,
	EVENT_HIRES_ALL						= BIT0 | BIT1 | BIT2 | BIT3
} ucid_hires_events_t;

#define MAX_AREA_MEM				6
/*
 * @brief capt_grp_t
 * Captured Camera group name
 */
typedef enum {
	CAPT_GRP_A		= 0x0000003E,
	CAPT_GRP_B		= 0x000007C0,
	CAPT_GRP_C		= 0x0001F800,
	CAPT_GRP_AB		= CAPT_GRP_A | CAPT_GRP_B,
	CAPT_GRP_BC		= CAPT_GRP_B | CAPT_GRP_C,
	CAPT_GRP_ABC	= CAPT_GRP_A | CAPT_GRP_B | CAPT_GRP_C,
} capt_grp_t ;
/* Exported define -----------------------------------------------------------*/
#define MAX_NUM_CAPT_CAMERA			6
#define MAX_INTR_WAIT_TIME			5000 /* In system ticks unit */
#define SPG_SECOND			CLOCK_133MHZ
#define SPG_MICROSECOND		(SPG_SECOND / 1000000)
#define SPG_MILISECOND		(SPG_SECOND / 1000)
#define MAX_IT_TO_SKIP		(double)0.015
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/**
 * @brief usecase_hires_setup
 * This function used to configure snapshot parameters
 * @param m_bitmask: the bit mask of camera channel will take snapshot
 * @param width: width of image
 * @param height: height of image
 * @param image_type: RAW8/10/12/14
 * @param frame: the frame number will capture
 * @param snapshot_done_hdl: the call back function
 * @retval refer
 */
usecase_hires_cfg_status_t usecase_hires_setup(uint32_t m_bitmask,
											uint8_t image_type, uint8_t frame,
											void (*snapshot_done_hdl)(void),
											TaskHandle_t *lhtask);
uint8_t snapshot_hdl(uint32_t time_out, uint8_t skip_frame_mask);
uint8_t cam_capture_syncio_config(cam_typedef_t *cam, uint8_t frame_num,
								double latency_time, syncio_trig_mode_t mode);

double config_syncio_hires(cam_typedef_t *cam, uint32_t captgrp,
							syncio_trig_mode_t mode, uint8_t delay, double *it);

#endif /* __USECASE_HIRES_H__ */

/*********** Portions COPYRIGHT 2016 Light.Co., Ltd. ********END OF FILE*******/
