/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    ina231.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    April-1-2016
 * @brief   Header file of INA231 current/power sensor driver
 *
 ******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __INA231_H__
#define __INA231_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "i2cm.h"

/* Private define ------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
/* Exported typedef ----------------------------------------------------------*/
/*
 * @brief ina231_status_t
 * INA231 return status
 */
typedef enum
{
	INA231_OK = 			0,
	INA231_I2C_BUSY = 		1,
	INA231_UNCALIBRATED =	2,
	INA231_OVERFLOW = 		3,
	INA231_VALUE_INVALID =	4,
} ina231_status_t;

/*
 * @brief ina_current_t
 * INA231 current lsb
 */
typedef enum
{
	CURRENT_LSB_1uA =		1,
	CURRENT_LSB_10uA =		10,
	CURRENT_LSB_100uA =		100,
	CURRENT_LSB_1mA =		1000,
} ina_current_t;

/*
 * @brief ina231_status_t
 * INA231 return status
 */
typedef struct
{
	int curr;				/* The current value (uA)*/
	int volt;				/* The voltafe value (mV)*/
	int power;				/* The power value (mW)*/
	uint16_t rsense;		/* SHUNT resistor value in mOhm */
	uint16_t current_unit;	/* Current unit in uA */
	uint8_t addr;			/* INA slave address */
	uint8_t is_calibrated;	/* The calibration status */
} ina231_info_t;

/* Exported functions --------------------------------------------------------*/
/*
 * @brief ina231_configure
 * Configure the INA devices, input the rsense value, current_unit
 * @param chid I2C channel ID
 * @param *info INA device pointer
 * @return ina231_status_t
 */
int ina231_config(hal_i2c_channel_t chid, ina231_info_t *info);

/*
 * @brief ina231_get_info
 * Query INA information include: Current, Voltage, Power
 * @param chid I2C channel ID
 * @param *info INA device pointer
 * @return ina231_status_t
 */
int ina231_get_info(hal_i2c_channel_t chid, ina231_info_t *info);
#ifdef __cplusplus
}
#endif
#endif /* __INA231_H__ */
/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
