/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    temperature.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    April-1-2016
 * @brief   This file contains definitions for temperature sensor driver
 *
 ******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TEMPERATURE_H__
#define __TEMPERATURE_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Private define------------------------------------------------------------*/
#define TEMP_SENSOR_WAIT_TIMEOUT	50 /* ms */

/* Exported define------------------------------------------------------------*/
/*
 * @brief tmp_return_t
 *
 * I2C return type
 */
typedef enum
{
	TEMP_OK = 0,
	TEMP_TIMED_OUT,
	TEMP_INPUT_INVALID,
	TEMP_FAILED
} tmp_return_t;

/* Exported typedef  ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/*
 * @brief temp_sensor_init
 * Init I2C channel and send config data for TMP112 temperature sensor
 * This channel must be called before use temperature get value function
 * @param None
 * @ruturn None
 */
void temp_sensor_init(void);

/*
 * @brief temp_sensor_raw_data_init
 * This function used to send config data to TMP112 temperature sensor
 * @param mode - define operating mode of temp sensor
 * chid - External temperature channel ID
 * @return status of config data sending
 */
int temp_sensor_raw_data_init(uint16_t mode);

/* @brief temp_sensor_raw_data_get
 * This function used to get 2 bytes value from temp sensor
 * @param chid - External temperature channel ID
 * data - pointer to archive temp value
 * return status of temperature getting
 */
int temp_sensor_raw_data_get(uint8_t *data);

/*
 * @brief temp_sensor_read
 * Read temperature sensor
 * @param chid - External temperature channel ID
 * @return None
 */
float temp_sensor_read(void);

#ifdef __cplusplus
}
#endif
#endif /* __TEMPERATURE_H__ */
/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
