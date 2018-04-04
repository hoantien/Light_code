/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    hall_sensor.c
 * @author  The LightCo
 * @version V1.0.0
 * @date    Jun-27-2016
 * @brief   This file contains hall sensor source code
 *
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "assert.h"
#include "stdbool.h"
#include "stddef.h"
#include "hall.h"
#include "i2cm.h"

/* Private define ------------------------------------------------------------*/
#define A1457_ADDR						0x0C

#define IS_A1457_SENSITIVITY(SENS)		(SENS < 8)

#define A1457_OUTPUT_REG				0x05
#define A1457_PIDCTL0_REG				0x16
#define A1457_PIDCTL5_REG				0x1B
#define A1457_FACTRES3_REG				0x1F

#define A1457_NUM_SENSITIVITIES			8

/* Private typedef -----------------------------------------------------------*/
/* Private function prototype ------------------------------------------------*/

static hall_return_t write_register(i2c_t i2c_chid,
											uint8_t reg_addr, uint8_t reg_val);

/* Private variable ----------------------------------------------------------*/
/* Exported function ---------------------------------------------------------*/

/**
 * @brief a1457_read_register
 * The function reads register value from independent A1457 hall sensor.
 * Limitation: this supports for reading non-factory registers only
 */
hall_return_t a1457_read_register(i2c_t i2c_chid,
							uint8_t reg_addr, uint8_t reg_len, uint8_t *reg_val)
{
	i2cm_error_t ret = i2cm.read(i2c_chid,
									(reg_len == 1) ? BYTE_ADDR8 : WORD_ADDR8,
									A1457_ADDR, reg_addr, reg_val);
	return (ret == I2CM_ERROR_BUSY) ? HALL_TIMED_OUT : HALL_OK;
}

/**
 * @brief a1457_init
 * The function initializes independent A1457 hall sensor
 */
hall_return_t a1457_init(i2c_t i2c_chid, bool polarity_reversal)
{
	hall_return_t	ret;
	uint8_t			PIDCTL5_reg;

	/* get the PIDCTL5 register value */
	ret = a1457_read_register(i2c_chid, A1457_PIDCTL5_REG, 1, &PIDCTL5_reg);
	if (ret == HALL_OK)
	{
		/* set polarity */
		PIDCTL5_reg = (PIDCTL5_reg & ~0x80) | (polarity_reversal << 7);
		ret = write_register(i2c_chid, A1457_PIDCTL5_REG, PIDCTL5_reg);
	}

	return ret;
}

/**
 * @brief a1457_set_sensitivity
 * The function set sensitivity of independent A1457 hall sensor
 */
hall_return_t a1457_set_sensitivity(i2c_t i2c_chid, uint8_t sensitivity)
{
	hall_return_t	ret;

	if (!(IS_A1457_SENSITIVITY(sensitivity)))
		ret = HALL_INVALID_SENSITIVITY;
	else
	{
		uint8_t PIDCTL5_reg = 0;	/* HGAIN[1:0] (bit 5:4) */
		uint8_t FACTRES3_reg = 0;	/* HGAIN[2] (bit 6) */
		uint8_t PIDCTL0_reg = 0;	/* LOCKF (bit 7) */

		/* read the PIDCTL0 register */
		ret = a1457_read_register(i2c_chid, A1457_PIDCTL0_REG, 1, &PIDCTL0_reg);
		if (ret == HALL_OK)
		{
			/* unlock the FACTRES3 register */
			PIDCTL0_reg &= ~(1 << 7);
			ret = write_register(i2c_chid, A1457_PIDCTL0_REG, PIDCTL0_reg);
		}
		/* read the registers containing sensitivity */
		ret |= a1457_read_register(i2c_chid, A1457_PIDCTL5_REG, 1,
									&PIDCTL5_reg);
		ret |= a1457_read_register(i2c_chid, A1457_FACTRES3_REG, 1,
									&FACTRES3_reg);
		if (ret == HALL_OK)
		{
			/* configure sensitivity with "Sensitivity" argument */
			/* configure bit [5:4] : HGAIN[1:0] */
			PIDCTL5_reg = (PIDCTL5_reg & ~(0x03 << 4)) |
							((sensitivity & 0x03) << 4);
			/* configure bit 6 HGAIN[2] */
			FACTRES3_reg = (FACTRES3_reg & ~(0x01 << 6)) |
							((sensitivity & 0x04) << 4);
			ret = write_register(i2c_chid, A1457_PIDCTL5_REG, PIDCTL5_reg);
			ret |= write_register(i2c_chid, A1457_FACTRES3_REG, FACTRES3_reg);
			/* lock the FACTRES3 register */
			PIDCTL0_reg |= (1 << 7);
			ret |= write_register(i2c_chid, A1457_PIDCTL0_REG, PIDCTL0_reg);
		}
	}

	return ret;
}

/**
 * @brief a1457_read_position
 * The function read position from independent A1457 hall sensor
 */
hall_return_t a1457_read_position(i2c_t i2c_chid, uint16_t *position)
{
	hall_return_t	ret;
	uint16_t		output_value;
	uint8_t			regs_val[2];

	/* assert parameters */
	assert_param(position != NULL);

	/* read output registers */
	ret = a1457_read_register(i2c_chid, A1457_OUTPUT_REG, 2, regs_val);
	if (ret == HALL_OK)
	{
		/* get output value */
		output_value = (regs_val[0] << 4) | regs_val[1];
		/* get position */
		*position = (output_value + 0x0800) & 0x0FFF;
	}

	return ret;
}

/**
 * @brief a1457_scan_sensitivities
 * The function reads position values
 * 		corresponding to the configurable sensitivities of A1457 hall sensor
 */
hall_return_t a1457_scan_sensitivities(i2c_t i2c_chid, uint16_t *position_data)
{
	hall_return_t	ret = HALL_OK;
	uint8_t			i;

	/* assert parameters */
	assert_param(position_data != NULL);

	for (i = 0; (i < A1457_NUM_SENSITIVITIES) && (ret == HALL_OK); i++)
	{
		ret = a1457_set_sensitivity(i2c_chid, i);
		ret |= a1457_read_position(i2c_chid, position_data + i);
	}

	return ret;
}

/* Private function --------------------------------------------------------- */
static hall_return_t write_register(i2c_t i2c_chid,
											uint8_t reg_addr, uint8_t reg_val)
{
	i2cm_error_t ret = i2cm.write(i2c_chid, BYTE_ADDR8,
									A1457_ADDR, reg_addr, &reg_val);
	return (ret == I2CM_ERROR_BUSY) ? HALL_TIMED_OUT : HALL_OK;
}

/* Exported variable ---------------------------------------------------------*/

hall_t a1457 =
{
	.nbo_sensitivities =	A1457_NUM_SENSITIVITIES,
	.max_position =			4095,
	.init =					a1457_init,
	.set_sensitivity =		a1457_set_sensitivity,
	.read_position =		a1457_read_position,
	.scan_sensitivities =	a1457_scan_sensitivities,
	.read_register =		a1457_read_register
};

/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
