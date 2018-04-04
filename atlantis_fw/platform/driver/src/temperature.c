/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    temperature.c
 * @author  The LightCo
 * @version V1.0.0
 * @date    April-1-2016
 * @brief   This file contains expand for temperature sensor driver
 *
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "std_type.h"
#include "board_config.h"
#include "os.h"
#include "log.h"
#include "i2cm.h"
#include "temperature.h"

/* Privated define------------------------------------------------------------*/
#define SLOGF_ID					SLOG_ID_LIGHT_SYSTEM
#define TMP112_INIT_VALUE			0x60A0
#define TMP112_REG_TEMP_ADDR		0x00
#define TMP112_REG_CONFIG_ADDR		0x01

/* Privated variables --------------------------------------------------------*/
/* Privated functions --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/* I2C initialization for temperature TMP211 sensor */
void temp_sensor_init(void)
{
	i2cm.init(HAL_I2C_TEMP);
	if(TEMP_OK ==temp_sensor_raw_data_init(TMP112_INIT_VALUE))
	{
		SLOGF(SLOG_INFO, "Init device temperature done!!!");
	}
	else
	{
		SLOGF(SLOG_ERROR, "Init device temperature fail!!!");
	}
}

/* temp_sensor_raw_data_init */
int temp_sensor_raw_data_init(uint16_t mode)
{
	uint8_t tx_data[2];
	int ret = TEMP_FAILED;
	tx_data[0] = (mode >> 8) & 0x00FF;
	tx_data[1] = mode & 0x00FF;

	if(I2CM_ERROR_TRANSMITTED == i2cm.write(HAL_I2C_TEMP, WORD_ADDR8,
	TEMP_SLAVE_ADDR, TMP112_REG_CONFIG_ADDR, &tx_data[0]))
		ret = TEMP_OK;
	else
		ret = TEMP_FAILED;

	return ret;
}

/* temp_sensor_raw_data_get */
int temp_sensor_raw_data_get(uint8_t *data)
{
	int ret = TEMP_FAILED;

	if(I2CM_ERROR_TRANSCEIVED == i2cm.read(HAL_I2C_TEMP, WORD_ADDR8,
			TEMP_SLAVE_ADDR, TMP112_REG_TEMP_ADDR, data))
		ret = TEMP_OK;
	else
		ret = TEMP_FAILED;

	return ret;
}

/*
 * temp_sensor_read
 * Read temperature sensor
 */
float temp_sensor_read(void)
{
	float ret = 0.0;
	uint8_t data[2] = {0, 0};
	int16_t tmp;
	if(TEMP_OK == temp_sensor_raw_data_get((uint8_t *)&data[0]))
	{
		/* calculate temperature */
		tmp = (int16_t)((data[0] << 8) | data[1]);
		tmp >>= 4;
		if(tmp & (1 << 11))
			tmp |= 0xF000;
		ret = ((float)(tmp) * 0.0625);
		/*printf("\n\rTMP112: Read temperature successfully: %f\r\n", ret);*/
	}
	else
	{
		/*printf("\n\rTMP112: Read temperature failed\r\n");*/
		ret = 0;
	}
	return ret;
}
/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
