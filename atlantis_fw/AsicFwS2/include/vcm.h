/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification,are strictly prohibited without prior permission of The LightCo.
 * @file    vcm.h
 * @author  The LightCo
 * @version V2.0.0
 * @date    Jun-07-2016
 * @brief   This file contain declaration for the vcm_t object and its APIs.
 *		Every API will receive at least the id parameter which is the camera
 *		id of the camera that contain VCM object. User should ensure that
 *		the camera has been initialized before passing its id to VCM API.
 *
 ******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VCM_H__
#define __VCM_H__
/* Includes ------------------------------------------------------------------*/
#include "lcc_system.h"
#include "os.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported define -----------------------------------------------------------*/
#define VCM_SLAVE_ADDR				0x72
#define VCM_CHIP_VERSION			0x42
#define VCM_OP_MAX_TRY_CNT			20

/* VCM registers */
#define VCM_PIDZO_REG				0x02
#define VCM_EQENBL_REG				0x87
#define VCM_STMVENDH_REG			0xA0
#define VCM_STMVEN_REG				0x8A
#define VCM_CVER_REG				0xF0
#define VCM_AWDLCTRL_REG			0xE0
#define VCM_STANDBY_REG				0x84
#define VCM_ADOFFSET_REG			0x3C
#define VCM_MSSET_REG				0x8F

/* VCM register bits */
#define VCM_STMVEN_STMVEN			(0x1 << 0)
#define VCM_STMVEN_STMFST			(0x1 << 4)
#define VCM_STMVEN_STMCHTGST		(0x1 << 3)
#define VCM_MSSET_CHTGST			(0x1 << 0)

/* EEPROM registers used in VCM */
#define EEPROM_AF_POS_MACRO			0x25
#define EEPROM_AF_POS_INF			0x27
#define EEPROM_AF_HARD_STOP_MACRO	0x2B
#define EEPROM_AF_HARD_STOP_INF		0x2D

/* AF information */
#define NUM_CALIB_POINTS			2
/* Exported typedef ----------------------------------------------------------*/
/*
 * @brief vcm_t
 * VCM controller driver typedef
 */
typedef struct
{
	i2c_t i2c;				/* i2c dev channel */
	void *af_calib;			/* Calibration data backup from eeprom */
	void *af_data;			/* Auto focus variable data */
	SemaphoreHandle_t semaphore;	/* Semaphore to access settings */
	void (*cb_func)(void *param);
	void *cb_param;
	bool moving;
	int16_t target_hall_code; /* Target hall code value */
} vcm_t;

/*
 * @brief vcm_return_t
 *
 * VCM return codes
 */
typedef enum vcm_return
{
	VCM_OK,
	VCM_ERROR,
	VCM_OP_TIMEOUT,
	VCM_INVALID_CVER,
	VCM_INVALID_ARG,
	VCM_PENDING
} vcm_return_t;

/* Exported functions ------------------------------------------------------- */
/*
 * @brief vcm_init
 * Initialize VCM module
 * @param id: the camera id that contain this module
 * @return: reference to vcm_return_t
 */
vcm_return_t vcm_init(vcm_t *vcm);
/*
 * @brief vcm_deinit
 * De-Initialize VCM module
 * @param id: the camera id that contain this module
 * @return: reference to vcm_return_t
 */
vcm_return_t vcm_deinit(vcm_t *vcm);
/*
 * @brief vcm_get_status
 * Get current status of VCM module
 * @param id: the camera id that contain this module
 * @return: VCM current status
 */
int vcm_get_status(vcm_t *vcm);
/*
 * @brief vcm_goto_hall
 * Move VCM to a specified HALL code
 * @param id: the camera id that contain this module
 * @param hall: the HALL code that VCM will move to
 * @return: reference to vcm_return_t
 */
vcm_return_t vcm_goto_hall(vcm_t *vcm, int16_t hall);
/*
 * @brief vcm_get_hall
 * Get current VCM hall position
 * @param id: the camera id that contain this module
 * @param hall: point to area that current VCM hall code is write to
 * @return: reference to vcm_return_t
 */
vcm_return_t vcm_get_hall(vcm_t *vcm, int16_t *hall);
/*
 * @brief vcm_focus_distance
 * Moving VCM to focus to specified distance (in mm)
 * @param id: the camera id that contain this module
 * @param distance: the distance that VCM will focus
 * @return: reference to vcm_return_t
 */
vcm_return_t vcm_focus_distance(vcm_t *vcm, uint32_t distance);

/*
 * @brief vcm_get_hard_stop_hall
 * Returns hard stop hall codes
 */
vcm_return_t vcm_get_hard_stop_hall(vcm_t *vcm, int16_t* hall_min, int16_t* hall_max);

/*
 * @brief vcm_object_distance_to_hallcode
 * Returns hallcode for focusing at given object distance
 */
vcm_return_t vcm_object_distance_to_hallcode(const vcm_t *vcm, uint32_t distance, int16_t *hallcode);

/*
 * @brief vcm_hallcode_to_object_distance
 * Returns Object distance for given hallcode
 */
vcm_return_t vcm_hallcode_to_object_distance(const vcm_t *vcm, int16_t hallcode, uint32_t *distance);

#ifdef __cplusplus
}
#endif
#endif /* __VCM_H__ */
/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
