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
#define HALL_MAX_RETRIES				3

#define AS5510_ADDR						0x57

#define IS_AS5510_SENSITIVITY(SENS)		(SENS < 4)

#define AS5510_OUTPUT_REG				0x00
#define AS5510_CONFIG_REG				0x02
#define AS5510_SENSE_REG				0x0B

#define AS5510_OUTPUT_OCF_MASK			0x0800
#define AS5510_OUTPUT_PARITY_MASK		0x0400
#define AS5510_OUTPUT_POSITION_MASK		0x03FF
#define AS5510_CONFIG_SPEED_MASK		0x04

#define AS5510_NUM_SENSITIVITIES		4

/* Private typedef -----------------------------------------------------------*/
/* Private function prototype ------------------------------------------------*/

static hall_return_t write_register(i2c_t i2c_chid,
											uint8_t reg_addr, uint8_t reg_val);

static bool check_parity_error(uint16_t value, uint8_t parity_bit);

/* Private variable ----------------------------------------------------------*/
/* Exported function ---------------------------------------------------------*/

/**
 * @brief hall_read_register
 * The function reads register value from independent AS5510 hall sensor
 */
hall_return_t as5510_read_register(i2c_t i2c_chid,
							uint8_t reg_addr, uint8_t reg_len, uint8_t *reg_val)
{
	i2cm_error_t i2cm_ret = i2cm.read(i2c_chid,
								(reg_len == 1) ? BYTE_ADDR8 : WORD_ADDR8,
								AS5510_ADDR, reg_addr, reg_val);
	return (i2cm_ret == I2CM_ERROR_BUSY) ? HALL_TIMED_OUT : HALL_OK;
}

/**
 * @brief as5510_init
 * The function initializes independent AS5510 hall sensor
 */
hall_return_t as5510_init(i2c_t i2c_chid, bool polarity_reversal)
{
	/* set polarity */
	return write_register(i2c_chid, AS5510_CONFIG_REG,
					AS5510_CONFIG_SPEED_MASK | (polarity_reversal << 1));
}

/**
 * @brief as5510_set_sensitivity
 * The function set sensitivity of independent AS5510 hall sensor
 */
hall_return_t as5510_set_sensitivity(i2c_t i2c_chid, uint8_t sensitivity)
{
	if (!(IS_AS5510_SENSITIVITY(sensitivity)))
		return HALL_INVALID_SENSITIVITY;
	else
	{
		/* set sensitivity */
		return write_register(i2c_chid, AS5510_SENSE_REG, sensitivity);
	}
}

/**
 * @brief as5510_read_position
 * The function read position from independent AS5510 hall sensor
 */
hall_return_t as5510_read_position(i2c_t i2c_chid, uint16_t *position)
{
	hall_return_t	ret;
	uint16_t	output_value;
	uint8_t		regs_val[2];
	uint8_t		parity_bit;
	uint8_t		offset_comp_status;
	uint8_t		read_count = 0; /* number of attempts made */

	/* assert parameters */
	assert_param(position != NULL);

	/* try to read position many times if the received value is incorrect */
	do
	{
		/* read output registers */
		ret = as5510_read_register(i2c_chid, AS5510_OUTPUT_REG, 2, regs_val);
		if (ret == HALL_OK)
		{
			/* update read count */
			read_count++;

			/* get output value */
			output_value = (regs_val[1] << 8) | regs_val[0];
			/* get offset compensation loop status */
			offset_comp_status =
							(output_value & AS5510_OUTPUT_OCF_MASK) >> 11;
			/* get parity bit */
			parity_bit = (output_value & AS5510_OUTPUT_PARITY_MASK) >> 10;
			/* get position */
			*position = (output_value & AS5510_OUTPUT_POSITION_MASK);
			/* check whether the received value is incorrect */
			if ((offset_comp_status == 0) ||
				(check_parity_error(*position, parity_bit) == TRUE))
				ret = HALL_INCORRECT_VALUE;
		}

	} while ((ret == HALL_INCORRECT_VALUE) &&
			(read_count < HALL_MAX_RETRIES + 1));

	return ret;
}

/**
 * @brief as5510_scan_sensitivities
 * The function reads position values
 * 		corresponding to the configurable sensitivities of AS5510 hall sensor
 */
hall_return_t as5510_scan_sensitivities(i2c_t i2c_chid, uint16_t *position_data)
{
	hall_return_t	ret = HALL_OK;
	uint8_t				i;

	/* assert parameters */
	assert_param(position_data != NULL);

	for (i = 0; (i < AS5510_NUM_SENSITIVITIES) && (ret == HALL_OK); i++)
	{
		ret = as5510_set_sensitivity(i2c_chid, i);
		ret |= as5510_read_position(i2c_chid, position_data + i);
	}

	return ret;
}

/* Private function --------------------------------------------------------- */

static hall_return_t write_register(i2c_t i2c_chid,
											uint8_t reg_addr, uint8_t reg_val)
{
	i2cm_error_t i2cm_ret = i2cm.write(i2c_chid, BYTE_ADDR8,
										AS5510_ADDR, reg_addr, &reg_val);
	return (i2cm_ret == I2CM_ERROR_BUSY) ? HALL_TIMED_OUT : HALL_OK;
}

static bool check_parity_error(uint16_t value, uint8_t parity_bit)
{
	uint8_t num_bit1 = 0;

	while (value != 0)
	{
		num_bit1++;
		value &= (value - 1);
	}

	return (parity_bit != (num_bit1 & 0x01));
}

/* Exported variable ---------------------------------------------------------*/

hall_t as5510 =
{
	.nbo_sensitivities =	AS5510_NUM_SENSITIVITIES,
	.max_position =			1023,
	.init =					as5510_init,
	.set_sensitivity =		as5510_set_sensitivity,
	.read_position =		as5510_read_position,
	.scan_sensitivities =	as5510_scan_sensitivities,
	.read_register =		as5510_read_register
};

/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
