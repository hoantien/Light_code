/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification,are strictly prohibited without prior permission of The LightCo.
 * @file    drv_vcm.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    Apr-13-2016
 * @brief   This file contains definitions of VCM driver
 *
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "os.h"
#include "std_type.h"
#include "vcm.h"
#include "hal_i2c.h"
#include "board_config.h"
#include "i2cm.h"
#include "log.h"
//! TODO: For testing
#include "register_dump.h"
/* Private define ------------------------------------------------------------*/
#define SLOGF_ID					SLOG_ID_VCM_DRIVER
#define INIT_VCM_INFO(vcm, chid) \
	{\
		vcm->chid = chid;\
		vcm->is_init = 0;\
		vcm->vcm_state = VCM_STATE_STOP;\
		vcm->state_detail = SE_NOT_INIT;\
	}
#define NUM_CALIB_POINTS		2
#define AF_CROP_FACTOR			(float)7.61
#define I2C_RW_TRY				20
#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif
#ifndef CLAMP
#define CLAMP(a, min, max) MIN(MAX(a, min), max)
#endif
/* Private typedef -----------------------------------------------------------*/
/*
 * @brief effective_focal_length_t
 * Effective focal length that the VCM module supported
 */
typedef enum effective_focal_length
{
	EFOCAL_LENGTH_28 = 28,
	EFOCAL_LENGTH_35 = 35
} effective_focal_length_t;
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
	interp_type_t datatype;
	unsigned int size;
	void *table;
} interp_table_t;
/*
 * @brief af_data_t
 * Data used for auto focus calculation
 */
typedef struct af_data
{
	uint8_t isvalid; /* AF data is valid or not */
	uint8_t is_on;
	uint32_t efl; /* Effective focal length */
	float focal_length;
	float crop_factor;
	uint16_t calib_distance[NUM_CALIB_POINTS];	/* In millimeters */
	uint16_t calib_position[NUM_CALIB_POINTS];	/* DAC code */
	interp_table_t dac_interp;
	float_point_t tbl_data[NUM_CALIB_POINTS];
} af_data_t;
/*
 * @brief i2c_flags_t
 * Status for synchronization when read/write registers via I2C
 */
typedef enum i2c_flags
{
	RW_READY		= 0x1,
	READ_REG_DONE	= 0x2,
	WRITE_REG_DONE	= 0x4,
	RW_ERROR		= 0x8,
	RW_TIME_OUT		= 0x10,
	RW_DONE			= 0x20
} i2c_flags_t;

/* Private variables ---------------------------------------------------------*/
static af_data_t af_data[VCM_MAX_CHANNEL];
/* Private function ----------------------------------------------------------*/
/**
 * @brief print_float
 * Helper function for print out floating point number
 */
static void print_float(float val, int num, char *prefix)
{
	int fact = 1, i = 0;
	for(i = 0; i < num; i++)
		fact = fact * 10;
	int number = (int)(val * fact);
	int first = number / (fact);
	int second = (number % (fact));
	SLOGF(SLOG_DEBUG, "%s %d.%d", prefix, first, second);
}
/*
 * @brief vcm_read_reg
 * Function to read only 8 bit data from VCM registers
 */
static int vcm_read_reg(uint8_t chid, uint8_t regaddr, uint8_t *regval)
{
	return i2cm_read(chid, BYTE_ADDR8, VCM_SLAVE_ADDR, regaddr, regval);
}
/*
 * @brief read_eeprom
 * A wrapper function to read camera EEPROM
 */
static uint8_t read_eeprom(uint8_t chid, uint16_t addr, uint8_t *val, uint8_t dm)
{
	uint8_t ret = 0;
	uint8_t eepops = 0;
	uint16_t eepromaddr = 0;
	uint8_t regval[2];

	ret = i2cm_read(chid, BYTE_ADDR16, CAM_EEPROM_ADDR, eepromaddr, &eepops);
	if(I2CM_ERROR_TRANSCEIVED != ret)
	{
		SLOGF(SLOG_ERROR, "Read EEPROM timeout");
		return ret;
	}
	else
	{
		if(eepops == 0x00)
		{
			SLOGF(SLOG_ERROR, "Read EEPROM failed");
			return ret;
		}
		else
		{
			if(dm == BYTE_ADDR16)
			{
				ret = i2cm_read(chid, BYTE_ADDR16, CAM_EEPROM_ADDR, addr, regval);
				if(I2CM_ERROR_TRANSCEIVED == ret)
					*val = regval[0];
			}
			else
			{
				ret = i2cm_read(chid, WORD_ADDR16, CAM_EEPROM_ADDR, addr, regval);
				if(I2CM_ERROR_TRANSCEIVED == ret)
				{
					*val = regval[1];
					*(val + 1) = regval[0];
				}
			}
			if(I2CM_ERROR_TRANSCEIVED != ret)
				SLOGF(SLOG_ERROR, "Read EEPROM timeout");
			return ret;
		}
	}
}
/*
 * @brief is_camera_valid
 * Whether the camera is supported or not
 */
static int is_camera_valid(uint8_t chid)
{
	uint8_t minfo;
	if(read_eeprom(chid, 0x0001, &minfo, BYTE_ADDR16) != I2CM_ERROR_TRANSCEIVED)
		return 0;
	minfo = (minfo & CAM_MODULE_MASK) & 0xFF;
	/* We are expecting that the focal length is 28mm (0x00) or 35mm (0x01) */
	if(minfo != CAM_MODULE_FL_28MM && minfo != CAM_MODULE_FL_35MM)
	{
		SLOGF(SLOG_ERROR, "The module type [0x%x] is not supported", minfo);
		return 0;
	}
	else if(minfo == CAM_MODULE_FL_28MM)
		af_data[chid].efl = EFOCAL_LENGTH_28;
	else if(minfo == CAM_MODULE_FL_35MM)
		af_data[chid].efl = EFOCAL_LENGTH_35;

	return 1;
}
/*
 * @brief validate_params
 * Validate channel ID and camera type
 */
static vcm_return_code_t validate_params(uint8_t chid)
{
	if(chid >= VCM_MAX_CHANNEL)
		return VCM_INVALID_ARG;
	if(!is_camera_valid(chid))
		return VCM_INVALID_ARG;

	return VCM_OK;
}

/*
 * @brief calculate_lens_position
 * Calculate lens position based on focal length and focus distance
 */
static inline float calculate_lens_position(float focal_length, float focus_dis)
{
	float lens_position = 0.0f;
	if (focal_length != focus_dis)
	{
		lens_position = focal_length * focus_dis / (focus_dis - focal_length);
	}
	return lens_position;
}
/*
 * @brief setup_interp_table
 * Setting up interpolate table
 */
static int setup_interp_table(interp_table_t *tbl)
{
	float tmp;
	float_point_t *t = NULL_PTR;
	if(!tbl)
		return 0;

	t = (float_point_t *)tbl->table;

	if(t[0].x == t[1].x)
		return 0;
	/* Ensure the table is in order of increasing x.
	 * Reverse the table if needed.
	 */
	if(t[0].x > t[1].x)
	{
		/* Swapping x*/
		tmp = t[1].x;
		t[1].x = t[0].x;
		t[0].x = tmp;
		/* Swapping y */
		tmp = t[1].y;
		t[1].y = t[0].y;
		t[0].y = tmp;
	}
	return 1;
}
/*
 * @brief init_af_data
 * Initialize AF data for focusing operation
 */
static int init_af_data(uint8_t chid)
{
	uint8_t ret = RW_DONE;
	/* Assume that the chid is checked before */
	af_data_t *info = (af_data_t *)&(af_data[chid]);
	if(info && info->isvalid == 1)
		return 1;
	info->crop_factor = AF_CROP_FACTOR;
	info->focal_length = info->efl / info->crop_factor;
	ret |= read_eeprom(chid, EEPROM_AF_DST_MACRO,
						(uint8_t *)&(info->calib_distance[0]), WORD_ADDR16);
	ret |= read_eeprom(chid, EEPROM_AF_POS_MACRO,
						(uint8_t *)&(info->calib_position[0]), WORD_ADDR16);
	ret |= read_eeprom(chid, EEPROM_AF_DST_INF,
						(uint8_t *)&(info->calib_distance[1]), WORD_ADDR16);
	ret |= read_eeprom(chid, EEPROM_AF_POS_INF,
						(uint8_t *)&(info->calib_position[1]), WORD_ADDR16);
	SLOGF(SLOG_DEBUG, "Macro distance %d", info->calib_distance[0]);
	SLOGF(SLOG_DEBUG, "Infinity distance %d", info->calib_distance[1]);
	SLOGF(SLOG_DEBUG, "Macro position 0x%x", info->calib_position[0]);
	SLOGF(SLOG_DEBUG, "Infinity position 0x%x", info->calib_position[1]);

	float_point_t *tbl_data = info->tbl_data;
	tbl_data[0].x = calculate_lens_position(info->focal_length,
													info->calib_distance[0]);
	tbl_data[0].y = info->calib_position[0];
	tbl_data[1].x = calculate_lens_position(info->focal_length,
													info->calib_distance[1]);
	tbl_data[1].y = info->calib_position[1];

	print_float(tbl_data[0].x, 5, "[TX0] ");
	print_float(tbl_data[0].y, 2, "[TY0] ");
	print_float(tbl_data[1].x, 5, "[TX1] ");
	print_float(tbl_data[1].y, 2, "[TY1] ");
	info->dac_interp.datatype = INTERP_TYPE_FLOAT;
	info->dac_interp.size = NUM_CALIB_POINTS;
	info->dac_interp.table = (void *)tbl_data;
	return setup_interp_table(&(info->dac_interp));
}
/* Exported functions ------------------------------------------------------- */
/*
 * @brief drv_vcm_init
 * Initialize VCM module
 * @param chid: VCM channel ID
 * @return: reference to vcm_return_code_t
 */
vcm_return_code_t drv_vcm_init(uint8_t chid)
{
	uint8_t byte_data;
	uint8_t rwret;
	uint8_t cver = 0;
	uint32_t cnt = 0;

	vcm_return_code_t ret = validate_params(chid);

	if(VCM_OK != ret)
		return ret;

	if(af_data[chid].is_on)
		return VCM_OK;

	/* Check chip version */
	rwret = vcm_read_reg(chid, VCM_CVER_REG, &cver);
	if(rwret != I2CM_ERROR_TRANSCEIVED || cver != VCM_CHIP_VERSION)
	{
		SLOGF(SLOG_ERROR, "Unsupported VCM chip version");
		return VCM_INVALID_CVER;
	}
	SLOGF(SLOG_DEBUG, "VCM chip version 0x%x", cver);

	/* Start download configuration */
	byte_data = 0x01;
	rwret = i2cm_write(chid, BYTE_ADDR8, VCM_SLAVE_ADDR,
										VCM_AWDLCTRL_REG, &byte_data);

	/* Check whether downloading done */
	vcm_read_reg(chid, VCM_AWDLCTRL_REG, &byte_data);
	while(cnt < VCM_OP_MAX_TRY_CNT && byte_data != 0x00)
	{
		cnt++;
		vcm_read_reg(chid, VCM_AWDLCTRL_REG, &byte_data);
	}

	/* Check time out condition */
	if(cnt == VCM_OP_MAX_TRY_CNT)
	{
		SLOGF(SLOG_ERROR, "Downloading timeout");
		return VCM_OP_TIMEOUT;
	}
	if(!init_af_data(chid))
	{
		SLOGF(SLOG_ERROR, "AF data can't be initialized");
		return VCM_ERROR;
	}
	af_data[chid].is_on = 1;
	af_data[chid].isvalid = 1;
	SLOGF(SLOG_INFO, "VCM initialize done");
	return VCM_OK;
}
/*
 * @brief drv_vcm_move_to_hall
 * Move VCM module to a specified DAC code.
 * User can know detail about VCM information in vcm_info_t structure
 * @param chid: VCM channel ID
 * @param hall: DAC code to move to
 * @param inf: VCM information
 * @return: reference to vcm_return_code_t
 */
vcm_return_code_t drv_vcm_move_to_hall(uint8_t chid, int16_t hall)
{
	vcm_return_code_t ret = validate_params(chid);
	uint8_t bdata = 0;
	uint8_t hall_value[2];
	int16_t real_hall;
	uint32_t cnt = 0;

	if(VCM_OK != ret)
		return ret;

	if(!af_data[chid].isvalid || !af_data[chid].is_on)
		return VCM_ERROR;

	hall = CLAMP(hall, (int16_t)af_data[chid].calib_position[0],
							(int16_t)af_data[chid].calib_position[1]);
	hall = hall & 0xFFF0;
	hall_value[0] = hall >> 8;
	hall_value[1] = hall;
	i2cm_write(chid, WORD_ADDR8, VCM_SLAVE_ADDR, VCM_STMVENDH_REG, hall_value);
	vTaskDelay(5);
	vcm_read_reg(chid, VCM_MSSET_REG, &bdata);
	if((bdata & VCM_MSSET_CHTGST) == VCM_MSSET_CHTGST && cnt < VCM_OP_MAX_TRY_CNT)
	{
		vTaskDelay(5);
		vcm_read_reg(chid, VCM_MSSET_REG, &bdata);
		cnt++;
	}
	if(cnt == VCM_OP_MAX_TRY_CNT)
	{
		SLOGF(SLOG_ERROR, "VCM moving time out");
		return VCM_OP_TIMEOUT;
	}
	drv_vcm_get_hall(chid, &real_hall);
	SLOGF(SLOG_INFO, "Request: 0x%02x. Current: 0x%02x",
								hall & 0xFFFF, real_hall & 0xFFFF);
	//! TODO: for testing push info to dump queue
	cam_regs_dump_t cam_reg;
	//! VCM_STMVENDH_REG
	{
		cam_reg.child = chid;
		cam_reg.id    = VCM_SLAVE_ADDR;
		cam_reg.addr  = VCM_STMVENDH_REG;
		cam_reg.size  = WORD_ADDR8;
		cam_reg.data  = pvPortMalloc(cam_reg.size);
		memcpy(cam_reg.data, hall_value, cam_reg.size);
		memset(cam_reg.name, 0x00, sizeof(cam_reg.name));
		sprintf(cam_reg.name, "[CAM][VCM] 0x%04x", VCM_STMVENDH_REG);
		cam_reg.check = FALSE;
		xQueueSend(queue_cam_registers, (void *)&cam_reg, 0);
	}
	return VCM_OK;
}
/*
 * @brief drv_vcm_calculate_daccode
 * Calculate DAC code (signed 16 bit integer) from input distance (mm)
 * @param chid: VCM channel ID
 * @param distance: real distance in mm
 * @return: return computed DAC code
 */
vcm_return_code_t drv_vcm_calculate_daccode(uint8_t chid, uint32_t distance,
																int16_t *dac)
{
	float lens_pos = 0.0f;
	float dac_float = 0.0f;
	float dx = 0.0f;
	float_point_t *points;
	vcm_return_code_t ret = validate_params(chid);

	if(VCM_OK != ret || !dac)
	{
		SLOGF(SLOG_ERROR, "Invalid parameter. Line %d", __LINE__);
		return ret;
	}

	if(!af_data[chid].isvalid || !af_data[chid].is_on)
	{
		SLOGF(SLOG_ERROR, "AFDATA has not been initialized. Line %d", __LINE__);
		return VCM_ERROR;
	}

	lens_pos = calculate_lens_position(af_data[chid].focal_length, distance);
	print_float(lens_pos, 5, "Lens position: ");
	interp_table_t *tbl = &(af_data[chid].dac_interp);
	if(!tbl || tbl->datatype != INTERP_TYPE_FLOAT)
	{
		SLOGF(SLOG_ERROR, "AFDATA invalid. Line %d", __LINE__);
		return VCM_ERROR;
	}

	points = (float_point_t *)tbl->table;

	if(points[0].x == points[1].x)
	{
		SLOGF(SLOG_ERROR, "AFDATA invalid. Line %d", __LINE__);
		return VCM_ERROR;
	}

	if(lens_pos <= points[0].x)
	{
		*dac = points[0].y;
		return VCM_OK;
	}

	if(lens_pos >= points[1].x)
	{
		*dac = points[1].y;
		return VCM_OK;
	}

	dx = points[1].x - points[0].x;

	if(dac)
	{
		dac_float = points[0].y +
				((points[1].y - points[0].y) * (lens_pos - points[0].x) / dx);
		print_float(dac_float, 5, "DAC position float: ");
		*dac = (int16_t)dac_float;
	}
	return VCM_OK;

}
/*
 * @brief drv_vcm_get_hall
 * Get current Hall position (DAC code) of VCM module
 * @param chid: VCM channel ID
 * @param hall: current HALL code returned
 * @return: reference to vcm_return_code_t
 */
vcm_return_code_t drv_vcm_get_hall(uint8_t chid, int16_t *hall)
{
	uint8_t bdata = 0;
	vcm_return_code_t ret = validate_params(chid);
	if(VCM_OK != ret)
		return ret;

	if(!hall)
		return VCM_INVALID_ARG;

	vcm_read_reg(chid, VCM_ADOFFSET_REG, (uint8_t *)hall);
	vcm_read_reg(chid, VCM_ADOFFSET_REG + 1, &bdata);
	*hall = (*hall << 8) | bdata;
	return VCM_OK;
}
/*
 * @brief drv_vcm_deinit
 * De-initialize VCM module
 * @param chid: VCM channel ID
 * @return: reference to vcm_return_code_t
 */
vcm_return_code_t drv_vcm_deinit(uint8_t chid)
{
	volatile uint8_t bdata;
	vcm_return_code_t ret = validate_params(chid);
	if(VCM_OK != ret)
		return ret;

	drv_vcm_move_to_hall(chid, 0x00);
	vTaskDelay(30);
	vcm_read_reg(chid, VCM_EQENBL_REG, (uint8_t *)&bdata);

	bdata &= 0xEF;
	i2cm_write(chid, BYTE_ADDR8, VCM_SLAVE_ADDR,
							VCM_EQENBL_REG, (uint8_t *)&bdata);
	bdata = 0x00;
	i2cm_write(chid, BYTE_ADDR8, VCM_SLAVE_ADDR,
							VCM_PIDZO_REG, (uint8_t *)&bdata);
	i2cm_write(chid, BYTE_ADDR8, VCM_SLAVE_ADDR,
							VCM_STANDBY_REG, (uint8_t *)&bdata);
	return VCM_OK;
}
/*
 * @brief drv_vcm_focus
 * Focus to a specified distance using VCM
 * @param chid: VCM channel ID
 * @param distance: distance to focus
 * @return: reference to vcm_return_code_t
 */
vcm_return_code_t drv_vcm_focus(uint8_t chid, uint32_t distance)
{
	int16_t daccode = 0;
	vcm_return_code_t ret = validate_params(chid);
	if(VCM_OK != ret)
		return ret;
	if(!af_data[chid].isvalid || !af_data[chid].is_on)
		return VCM_ERROR;
	ret = drv_vcm_calculate_daccode(chid, distance, &daccode);
	if(VCM_OK != ret)
		return VCM_ERROR;
	return drv_vcm_move_to_hall(chid, daccode);
}
/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
