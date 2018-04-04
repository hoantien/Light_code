/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    hall.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    Jul-13-2016
 * @brief   This file contains expand of the mirror hall sensor driver
 *
 ******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HALL_H__
#define __HALL_H__

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "i2cm.h"

/* Exported define -----------------------------------------------------------*/
/* Exported typedef-----------------------------------------------------------*/

/**
 * @brief hall_return_t
 * Hall sensor return type
 */
typedef enum
{
	HALL_OK = 0,
	HALL_TIMED_OUT,
	HALL_INVALID_SENSITIVITY,
	HALL_INCORRECT_VALUE
} hall_return_t;

/**
 * @brief hall_t
 * Hall sensor type definition
 */
typedef struct
{
	/* The number of configurable sensitivities */
	const uint8_t nbo_sensitivities;
	/* The applicable maximum position value */
	const uint16_t max_position;
	/**
	 * @brief hall_init
	 * The function initializes independent hall sensor
	 * @param i2c_chid I2C channel ID
	 * @param polarity_reversal Polarity is reversed or not
	 * @return Reference to hall_return_t structure
	 */
	hall_return_t (*init)(i2c_t i2c_chid, bool polarity_reversal);
	/**
	 * @brief hall_set_sensitivity
	 * The function sets sensitivity of independent hall sensor
	 * @param i2c_chid I2C channel ID
	 * @param sensitivity Sensitivity
	 * @return Reference to hall_return_t structure
	 */
	hall_return_t (*set_sensitivity)(i2c_t i2c_chid, uint8_t sensitivity);
	/**
	 * @brief hall_read_position
	 * The function reads position from independent hall sensor
	 * @param i2c_chid I2C channel ID
	 * @param position Position which is read
	 * @return Reference to hall_return_t structure
	 */
	hall_return_t (*read_position)(i2c_t i2c_chid, uint16_t *position);

	/**
	 * @brief hall_scan_sensitivities
	 * The function reads position values corresponding to the configurable
	 * sensitivities of hall sensor
	 * @param i2c_chid I2C channel ID
	 * @param position_data Position values corresponding to the sensitivities
	 * @return Reference to hall_return_t structure
	 */
	hall_return_t (*scan_sensitivities)(i2c_t i2c_chid, uint16_t *position_data);

	/**
	 * @brief hall_read_register
	 * The function reads register value from independent hall sensor
	 * @param i2c_chid I2C channel ID
	 * @param reg_addr Register address
	 * @param reg_len Register length
	 * @param reg_val Register value
	 * @return Reference to hall_return_t structure
	 */
	hall_return_t (*read_register)(i2c_t i2c_chid,
						uint8_t reg_addr, uint8_t reg_len, uint8_t *reg_val);

} hall_t;
/* Exported functions --------------------------------------------------------*/
/* Exported variable ---------------------------------------------------------*/

extern hall_t a1457;
extern hall_t as5510;

#ifdef __cplusplus
}
#endif
#endif /* __HALL_H__ */
/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
