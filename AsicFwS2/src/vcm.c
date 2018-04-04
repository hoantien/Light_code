/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification,are strictly prohibited without prior permission of The LightCo.
 * @file    vcm.c
 * @author  The LightCo
 * @version V2.0.0
 * @date    Jun-07-2016
 * @brief   This file contain definition for the vcm_t object and its APIs.
 *		Every API will receive at least the id parameter which is the camera
 *		id of the camera that contain VCM object. User should ensure that
 *		the camera has been initialized before passing its id to VCM API.
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "std_type.h"
#include "assert.h"
#include "os.h"
#include "log.h"
#include "i2cm.h"
#include "eeprom.h"
#include "lcc_system.h"
#include "af_lens_calib.h"
#include "vcm.h"

/* Private define ------------------------------------------------------------*/
#define SLOGF_ID			SLOG_ID_VCM_DRIVER

#define VCM_HARD_STOP_BUFFER 512

/* Global variable -----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/*
 * @brief interp_type_t
 * Interpolate type
 */
typedef enum interp_type
{
	INTERP_TYPE_NONE = 0,
	INTERP_TYPE_FLOAT = 1,
	INTERP_TYPE_MAX
} interp_type_t;

/*
 * @brief float_point_t
 * A point structure which x,y is float
 */
typedef struct float_point
{
	float x;
	float y;
} float_point_t;

/*
 * @brief interp_table_t
 * Interpolate table
 */
typedef struct interp_table
{
	void *table;
	unsigned int size;
	interp_type_t datatype;
} interp_table_t;

/*
 * @brief af_data_t
 * Data used for auto focus calculation
 */
typedef struct af_data
{
    af_calib_point_t  af_calib_points[NUM_CALIB_POINTS];
    af_calib_t  af_calib;
    int16_t hard_stop_inf;
    int16_t hard_stop_macro;
    uint8_t is_valid;	/* AF data is valid or not */
	uint8_t is_on;		/* VCM module has initialized or not */
} af_data_t;

/* Private functions implementation ------------------------------------------*/
/*
 * @brief read_eeprom
 * A wrapper function to read camera EEPROM
 */
static uint8_t read_eeprom(i2c_t chid, uint16_t addr, void *val, uint8_t dm)
{
	uint8_t len = 1;
	if(dm == WORD_ADDR16)
		len = 2;
	eeprom_read(chid, addr, len, val);
	return 0;
}

/*
 * @brief init_af_data
 * Initialize AF data for focusing operation
 */
static int init_af_data(vcm_t *vcm)
{
	int ret = 0;

	if(!vcm->af_data)
	{
		SLOGF(SLOG_DEBUG, "Initialize AF data for i2c channel %d", vcm->i2c);
		vcm->af_data = assert_malloc(vcm->af_data, sizeof(af_data_t));
		memset(vcm->af_data, 0x00, sizeof(af_data_t));
	}

	af_data_t *info = (af_data_t *)vcm->af_data;
	if(info && info->is_on && info->is_valid)
		return 1;

	uint16_t read_buf;
	ret |= read_eeprom(vcm->i2c, EEPROM_AF_HARD_STOP_MACRO,
	                   &read_buf, WORD_ADDR16);
	info->hard_stop_macro = __builtin_bswap16(read_buf);
    ret |= read_eeprom(vcm->i2c, EEPROM_AF_HARD_STOP_INF,
                        &read_buf, WORD_ADDR16);
    info->hard_stop_inf = __builtin_bswap16(read_buf);
	ret |= read_eeprom(vcm->i2c, EEPROM_AF_POS_MACRO,
						&read_buf, WORD_ADDR16);
	info->af_calib_points[0].hallcode = __builtin_bswap16(read_buf);
	ret |= read_eeprom(vcm->i2c, EEPROM_AF_POS_INF,
						&read_buf, WORD_ADDR16);
	info->af_calib_points[1].hallcode = __builtin_bswap16(read_buf);
	info->af_calib_points[0].object_distance_mm = 100;
    info->af_calib_points[1].object_distance_mm = 5000;
	SLOGF(SLOG_DEBUG, "Macro distance: %d", info->af_calib_points[0].object_distance_mm);
	SLOGF(SLOG_DEBUG, "Infinity distance: %d", info->af_calib_points[1].object_distance_mm);
	SLOGF(SLOG_DEBUG, "Macro position: %d", info->af_calib_points[0].hallcode);
	SLOGF(SLOG_DEBUG, "Infinity position: %d", info->af_calib_points[1].hallcode);
	SLOGF(SLOG_DEBUG, "Hard stop macro: %d", info->hard_stop_macro);
    SLOGF(SLOG_DEBUG, "Hard stop infinity: %d", info->hard_stop_inf);
    info->af_calib.focal_length_mm = 3.68f;
    info->af_calib.calib_point_arr = info->af_calib_points;
	info->af_calib.num_calib_points = NUM_CALIB_POINTS;
    info->is_valid = 1;
    info->is_on = 1;

	return (ret == 0);
}

/* Exported functions ------------------------------------------------------- */
vcm_return_t vcm_init(vcm_t *vcm)
{
	uint8_t cver = 0;
	uint8_t byte_data;
	uint32_t cnt = 0;

	assert_param(NULL != vcm);

	af_data_t *afdata = (af_data_t *)vcm->af_data;
	if(afdata && afdata->is_on)
	{
		SLOGF(SLOG_INFO, "VCM already initialized");
		return VCM_OK;
	}

	i2cm.read(vcm->i2c, BYTE_ADDR8, VCM_SLAVE_ADDR, VCM_CVER_REG, &cver);
	if(cver != VCM_CHIP_VERSION)
	{
		SLOGF(SLOG_ERROR, "Unsupported VCM chip version");
		return VCM_INVALID_CVER;
	}
	byte_data = 0x01;
	i2cm.write(vcm->i2c, BYTE_ADDR8, VCM_SLAVE_ADDR, VCM_AWDLCTRL_REG, &byte_data);
	vTaskDelay(5);
	i2cm.read(vcm->i2c, BYTE_ADDR8, VCM_SLAVE_ADDR, VCM_AWDLCTRL_REG, &byte_data);
	while(cnt < VCM_OP_MAX_TRY_CNT && byte_data != 0x00)
	{
		vTaskDelay(5);
		cnt++;
		i2cm.read(vcm->i2c, BYTE_ADDR8, VCM_SLAVE_ADDR, VCM_AWDLCTRL_REG, &byte_data);
	}

	/* Check time out condition */
	if(cnt == VCM_OP_MAX_TRY_CNT)
	{
		SLOGF(SLOG_ERROR, "Downloading timeout");
		return VCM_OP_TIMEOUT;
	}
	if(!init_af_data(vcm))
	{
		SLOGF(SLOG_ERROR, "AF data can't be initialized");
		return VCM_ERROR;
	}
	vcm->target_hall_code = 0;
	SLOGF(SLOG_INFO, "VCM initialize done");
	return VCM_OK;
}

vcm_return_t vcm_deinit(vcm_t *vcm)
{
	assert_param(NULL != vcm);

	af_data_t *afdata = (af_data_t *)vcm->af_data;
	if (NULL == afdata)
		return VCM_INVALID_ARG;

	afdata->is_on = 0;
	afdata->is_valid = 0;
	return VCM_OK;
}

int vcm_get_status(vcm_t *vcm)
{
	assert_param(NULL != vcm);

	af_data_t *afdata = (af_data_t *)vcm->af_data;
	if (NULL == afdata)
		return VCM_INVALID_ARG;

	return afdata->is_on;
}

vcm_return_t vcm_get_hard_stop_hall(vcm_t *vcm, int16_t* hall_min, int16_t* hall_max)
{
    af_data_t *afdata = (af_data_t *)vcm->af_data;
    if (NULL == afdata)
    {
        SLOGF(SLOG_ERROR, "Invalid VCM data");
        return VCM_INVALID_ARG;
    }
    *hall_min = afdata->hard_stop_macro + VCM_HARD_STOP_BUFFER;
    *hall_max = afdata->hard_stop_inf - VCM_HARD_STOP_BUFFER;
    return VCM_OK;
}

vcm_return_t vcm_goto_hall(vcm_t *vcm, int16_t hall)
{
	uint8_t bdata = 0;
	uint8_t hall_value[2];
	int16_t real_hall;
	uint32_t cnt = 0;

	assert_param(NULL != vcm);

	int16_t hall_min;
	int16_t hall_max;
	vcm_return_t ret = vcm_get_hard_stop_hall(vcm, &hall_min, &hall_max);
	if (VCM_OK != ret)
	    return ret;
	hall = CLAMP(hall, hall_min, hall_max);
	hall &= 0xFFF0;
	vcm->target_hall_code = hall;
	hall_value[0] = hall >> 8;
	hall_value[1] = hall;
	i2cm.write(vcm->i2c, WORD_ADDR8, VCM_SLAVE_ADDR, VCM_STMVENDH_REG, hall_value);
	vTaskDelay(5);
	i2cm.read(vcm->i2c, BYTE_ADDR8, VCM_SLAVE_ADDR, VCM_MSSET_REG, &bdata);

	while((bdata & VCM_MSSET_CHTGST) == VCM_MSSET_CHTGST
							&& cnt < VCM_OP_MAX_TRY_CNT)
	{
		vTaskDelay(3);
		i2cm.read(vcm->i2c, BYTE_ADDR8, VCM_SLAVE_ADDR, VCM_MSSET_REG, &bdata);
		cnt++;
	}

    vcm_get_hall(vcm, &real_hall);
    SLOGF(SLOG_INFO, "VCM i2c channel %d finished moving. Request: %d. Current: %d", (int)vcm->i2c, hall, real_hall);
	if(cnt == VCM_OP_MAX_TRY_CNT)
	{
		SLOGF(SLOG_ERROR, "VCM moving time out");
		return VCM_OP_TIMEOUT;
	}
	return VCM_OK;
}



vcm_return_t vcm_get_hall(vcm_t *vcm, int16_t *hall)
{
	uint8_t bdata[2];
	vcm_return_t ret = VCM_OK;
	i2cm_error_t i2c_ret = I2CM_ERROR_NONE;

	assert_param(NULL != vcm);
	if (NULL == hall)
		return VCM_INVALID_ARG;

	i2c_ret = i2cm.read(vcm->i2c,
						WORD_ADDR8,
						VCM_SLAVE_ADDR,
						VCM_ADOFFSET_REG,
						bdata);
	if(I2CM_ERROR_TRANSCEIVED == i2c_ret)
	{
		uint16_t uhall = bdata[0];
		uhall = (uhall << 8) | bdata[1];
		*hall = (uhall < 0x8000u) ? uhall : (uhall - 65536);
	}
	else
	{
		SLOGF(SLOG_ERROR, "Timeout when reading hall code");
		ret = VCM_OP_TIMEOUT;
	}
	return ret;
}

vcm_return_t vcm_object_distance_to_hallcode(const vcm_t *vcm, uint32_t distance, int16_t *hallcode)
{
    af_data_t *afdata = (af_data_t *)vcm->af_data;
    if (NULL == afdata)
        return VCM_INVALID_ARG;
    if (!afdata->is_on || !afdata->is_valid)
    {
        SLOGF(SLOG_ERROR, "AF data is not valid");
        return VCM_ERROR;
    }

    if (distance >= 65535)
        distance = 65535;
    *hallcode = object_distance_to_hallcode(&afdata->af_calib, distance);
    return VCM_OK;
}

vcm_return_t vcm_focus_distance(vcm_t *vcm, uint32_t distance)
{
	assert_param(NULL != vcm);
    int16_t daccode = 0;
    vcm_return_t ret = vcm_object_distance_to_hallcode(vcm, distance, &daccode);
	if (ret != VCM_OK)
	    return ret;
	return vcm_goto_hall(vcm, daccode);
}

vcm_return_t vcm_hallcode_to_object_distance(const vcm_t *vcm, int16_t hallcode, uint32_t *distance)
{
    af_data_t *afdata = (af_data_t *)vcm->af_data;
    if (NULL == afdata)
        return VCM_INVALID_ARG;
    if (!afdata->is_on || !afdata->is_valid)
    {
        SLOGF(SLOG_ERROR, "AF data is not valid");
        return VCM_ERROR;
    }

    *distance = hallcode_to_object_distance(&afdata->af_calib, hallcode);
    return VCM_OK;
}
/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
