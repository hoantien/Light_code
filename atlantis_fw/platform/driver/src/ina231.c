/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file	ina231.c
 * @author	The LightCo
 * @version	V1.0.0
 * @date	April-1-2016
 * @brief	This file contains expand for INA231 current/power sensor driver
 *
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "std_type.h"
#include "lcc_system.h"
#include "os.h"
#include "log.h"
#include "ina231.h"

/* Privated define------------------------------------------------------------*/
#define INA231_TIMEOUT				50		/* ms */

/* INA231 Register addresses*/
#define INA231_CFG					0x00
#define INA231_SHUNT_VOLT			0x01
#define INA231_BUS_VOLT				0x02
#define INA231_POWER				0x03
#define INA231_CURRENT				0x04
#define INA231_CALIB				0x05
#define INA231_MSK_EN				0x06
#define INA231_ALERT				0x07

/* INA232 Configurate mode */
#define INA231_CFG_MODE_MASK		(7 << 0)
#define INA231_CFG_MODE_PWRDWN		(0 << 0)
#define INA231_CFG_MODE_SHUNT		(1 << 0)
#define INA231_CFG_MODE_BUS			(1 << 1)
#define INA231_CFG_MODE_TRG			(0 << 2)
#define INA231_CFG_MODE_CONT		(1 << 2)

/* INA232 Configurate convert time for the shunt voltage measurement */
#define INA231_CFG_CT_SHUNT_140		(0 << 3)
#define INA231_CFG_CT_SHUNT_204		(1 << 3)
#define INA231_CFG_CT_SHUNT_332		(2 << 3)
#define INA231_CFG_CT_SHUNT_588		(3 << 3)
#define INA231_CFG_CT_SHUNT_1100	(4 << 3)
#define INA231_CFG_CT_SHUNT_2116	(5 << 3)
#define INA231_CFG_CT_SHUNT_4156	(6 << 3)
#define INA231_CFG_CT_SHUNT_8244	(7 << 3)

/* INA232 Configurate convert time for the bus voltage measurement */
#define INA231_CFG_CT_BUS_140		(0 << 6)
#define INA231_CFG_CT_BUS_204		(1 << 6)
#define INA231_CFG_CT_BUS_332		(2 << 6)
#define INA231_CFG_CT_BUS_588		(3 << 6)
#define INA231_CFG_CT_BUS_1100		(4 << 6)
#define INA231_CFG_CT_BUS_2116		(5 << 6)
#define INA231_CFG_CT_BUS_4156		(6 << 6)
#define INA231_CFG_CT_BUS_8244		(7 << 6)

/* INA232 Configurate the number of samples that are collected and averaged */
#define INA231_CFG_AVG_1			(0 << 9)
#define INA231_CFG_AVG_4			(1 << 9)
#define INA231_CFG_AVG_16			(2 << 9)
#define INA231_CFG_AVG_64			(3 << 9)
#define INA231_CFG_AVG_128			(4 << 9)
#define INA231_CFG_AVG_256			(5 << 9)
#define INA231_CFG_AVG_512			(6 << 9)
#define INA231_CFG_AVG_1024			(7 << 9)

/* INA232 mask/enable*/
#define INA231_MSK_EN_LEN			(1 << 0)	/* Alert Latch Enable */
#define INA231_MSK_EN_APOL			(1 << 1)	/* Alert Polarity */
#define INA231_MSK_EN_OVF			(1 << 2)	/* Math Overflow Flag */
#define INA231_MSK_EN_CVRF			(1 << 3)	/* Conversion Ready Flag */
#define INA231_MSK_EN_AFF			(1 << 4)	/* Alert Function Flag */
#define INA231_MSK_EN_CNVR			(1 << 10)	/* Conversion Ready */
#define INA231_MSK_EN_POL			(1 << 11)	/* Over-Limit Power */
#define INA231_MSK_EN_BUL			(1 << 12)	/* Bus Voltage Under-Voltage */
#define INA231_MSK_EN_BOL			(1 << 13)	/* Bus Voltage Over-Voltage */
#define INA231_MSK_EN_SUL			(1 << 14)	/* Shunt Voltage Under-Voltage*/
#define INA231_MSK_EN_SOL			(1 << 15)	/* Shunt Voltage Over-Voltage */

#define INA231_CFG_RSHUNT_LSB		0.0025		/* mV */
#define INA231_CFG_DEFAULT			0x4527		/* averages=16 */
#define INA231_CFG_CALIB_DIVIDED	2048

/* Calibration value to get current LSB = cur_lsb */
#define CALIB_VALUE(cur_lsb, rsense)	(5120000/(rsense * cur_lsb))
/* Bus voltage LSB : 1.25mV / bit */
#define INA231_BUS_mV(val)				((reg * 125) / 100)
/* Power LSB : 25mW / current_lsb (uA) */
#define INA231_POW_mW(val, cur_lsb)		((reg * 25 * cur_lsb)/1000)

#define SLOGF_ID					SLOG_ID_INA231

#define USE_I2CM_RW_FUNCTION

/* Privated functions prototypes ---------------------------------------------*/
/* Privated variables --------------------------------------------------------*/
#ifndef USE_I2CM_RW_FUNCTION
static volatile i2cm_error_t ina231_status;
#endif
/* Privated functions implementation -----------------------------------------*/
#ifndef USE_I2CM_RW_FUNCTION
static void ina231_callback(i2cm_error_t i2c_status, void *param)
{
	i2cm_callback_msg_t *i2c_callback_msg =(i2cm_callback_msg_t *)param;
	BaseType_t higher_pri_task_woken = pdFALSE;

	*((uint8_t *)i2c_callback_msg->status) = i2c_status;
	xSemaphoreGiveFromISR(i2c_callback_msg->sem_msg, &higher_pri_task_woken);
	portEND_SWITCHING_ISR(higher_pri_task_woken);

}

/*
 * ina231_read
 * Read word value
 */
static int ina231_read(i2c_t i2c, uint8_t sla, uint8_t reg, uint16_t *data)
{
	i2cm_callback_msg_t i2c_callback_msg;
	volatile i2cm_error_t status = I2CM_ERROR_TRANSCEIVED;
	uint8_t buf[3] = {reg, 0, 0};

	ina231_status = I2CM_ERROR_BUSY;
	/* Update register pointer want to read value */
	i2cm_transceiver(i2c, sla, &buf[0], 1, &buf[1], 2, ina231_callback, &(i2c_callback_msg));

	i2c_callback_msg.status = &status;
	i2c_callback_msg.sem_msg = xSemaphoreCreateBinary();

	/* Waiting for updating register pointer INA231_TIMEOUT */
	xSemaphoreTake(i2c_callback_msg.sem_msg, portMAX_DELAY);
	vSemaphoreDelete(i2c_callback_msg.sem_msg);
	i2c_callback_msg.sem_msg = NULL;
	/* Convert data value to word format */
	*data = (buf[1]<<8) | buf[2];
	return status ? INA231_I2C_BUSY : INA231_OK;
}

/*
 * ina231_read
 * Write configure
 */
static int ina231_write(i2c_t i2c, uint8_t sla, uint8_t reg, uint16_t *data)
{
	i2cm_callback_msg_t i2c_callback_msg;
	volatile i2cm_error_t status = I2CM_ERROR_TRANSCEIVED;
	uint8_t buf[3];

	buf[0] = reg;
	buf[1] = (uint8_t)(data[0]>>8);
	buf[2] = (uint8_t)(data[0]);

	ina231_status = I2CM_ERROR_BUSY;
	/* Write data */
	i2cm_transceiver(i2c, sla, buf, 3, NULL, 0, ina231_callback, &i2c_callback_msg);
	i2c_callback_msg.status = &status;
	i2c_callback_msg.sem_msg = xSemaphoreCreateBinary();

	/* Wait for transaction to complete INA231_TIMEOUT */
	xSemaphoreTake(i2c_callback_msg.sem_msg, portMAX_DELAY);
	vSemaphoreDelete(i2c_callback_msg.sem_msg);
	i2c_callback_msg.sem_msg = NULL;
	return  status ? INA231_I2C_BUSY : INA231_OK;
}
#endif
/* Exported functions implementation -----------------------------------------*/
#ifndef USE_I2CM_RW_FUNCTION
/*
 * ina231_configure
 * Configure the INA devices, input the rsense value, current_unit
 */
int ina231_config(hal_i2c_channel_t chid, ina231_info_t *info)
{
	ina231_status_t ret = INA231_OK;
	uint16_t reg = 0;

	if ((info->rsense == 0) || (info->current_unit == 0) || (info->addr == 0))
		return INA231_VALUE_INVALID;

	ret = ina231_read(chid, info->addr, INA231_CFG, &reg);
	if (INA231_I2C_BUSY != ret)		/* Cannot connect to devices */
	{
		/* Keep all configuration as default
		 * Averaging Mode: AVG = 1;
		 * Bus Voltage Conversion Time: VCT BUS = 1.1ms
		 * Shunt Voltage Conversion Time: VCT SH = 1.1ms
		 * Mode: Shunt and bus, continuous
		 * Enable all alert flags
		 */
		reg = 0xF801;
		ret = ina231_write(chid, info->addr, INA231_CALIB, &reg);
		ret |= ina231_read(chid, info->addr, INA231_CFG, &reg);
		reg = CALIB_VALUE(info->rsense, info->current_unit);
		ret |= ina231_write(chid, info->addr, INA231_CALIB, &reg);
		info->is_calibrated = 1;
	}
	return ret;
}

/*
 * ina231_get_info
 * Query INA information include: Current, Voltage, Power
 */
int ina231_get_info(hal_i2c_channel_t chid, ina231_info_t *info)
{
	ina231_status_t ret = INA231_OK;
	uint16_t reg = 0;

	/* Checking the input parameter and devices status */
	if (info->is_calibrated == INA231_UNCALIBRATED)
		return INA231_UNCALIBRATED;
	if (info->addr == 0)
		return INA231_VALUE_INVALID;

	ret = ina231_read(chid, info->addr, INA231_ALERT, &reg);
	if (INA231_I2C_BUSY == ret)		/* Cannot connect to devices */
		return ret;

	if (reg & 0xF800)
		return INA231_OVERFLOW;		/* Overflow found */

	ret = ina231_read(chid, info->addr, INA231_CURRENT, &reg);
	info->curr = ((signed short)reg)*info->current_unit;
	ret |= ina231_read(chid, info->addr, INA231_BUS_VOLT, &reg);
	info->volt = INA231_BUS_mV((int)reg);
	ret |= ina231_read(chid, info->addr, INA231_POWER, &reg);
	info->power = INA231_POW_mW((int)reg, info->current_unit);
	return ret;
}
#else
/*
 * ina231_configure
 * Configure the INA devices, input the rsense value, current_unit
 */
int ina231_config(hal_i2c_channel_t chid, ina231_info_t *info)
{
	i2cm_error_t ret = I2CM_ERROR_NONE;
	uint16_t reg = 0;
	uint8_t buf[2];

	if ((info->rsense == 0) || (info->current_unit == 0) || (info->addr == 0))
		return INA231_VALUE_INVALID;

	ret = i2cm.read(chid, WORD_ADDR8, info->addr, INA231_CFG, buf);
	if (ret == I2CM_ERROR_TRANSCEIVED)
	{
		reg = INA231_CFG_DEFAULT;
		buf[0] = (uint8_t)(reg >> 8);
		buf[1] = (uint8_t)(reg);
		ret = i2cm.write(chid, WORD_ADDR8, info->addr, INA231_CFG, buf);
		ret = i2cm.read(chid, WORD_ADDR8, info->addr, INA231_CFG, buf);
		reg = buf[1] | (uint16_t)(buf[0] << 8);
		SLOGF(SLOG_INFO, "INA231_CFG: %X",reg);
		reg = (INA231_CFG_RSHUNT_LSB * info->current_unit *
			INA231_CFG_CALIB_DIVIDED) / info->rsense;
		buf[0] = (uint8_t)(reg >> 8);
		buf[1] = (uint8_t)(reg);
		ret = i2cm.write(chid, WORD_ADDR8, info->addr, INA231_CALIB, buf);
		ret = i2cm.read(chid, WORD_ADDR8, info->addr, INA231_CALIB, buf);
		reg = buf[1] | (uint16_t)(buf[0] << 8);
		SLOGF(SLOG_INFO, "INA231_CALIB: %X",reg);
		info->is_calibrated = 1;
		return INA231_OK;
	}
	return INA231_I2C_BUSY;
}

/*
 * ina231_get_info
 * Query INA information include: Current, Voltage, Power
 */
int ina231_get_info(hal_i2c_channel_t chid, ina231_info_t *info)
{
	i2cm_error_t ret = I2CM_ERROR_NONE;
	uint16_t reg = 0;
	uint8_t buf[2];

	/* Checking the input parameter and devices status */
	if (info->is_calibrated == INA231_UNCALIBRATED)
		return INA231_UNCALIBRATED;
	if (info->addr == 0)
		return INA231_VALUE_INVALID;

	ret = i2cm.read(chid, WORD_ADDR8, info->addr, INA231_ALERT, buf);
	if (I2CM_ERROR_BUSY == ret)		/* Cannot connect to devices */
		return INA231_I2C_BUSY;

	reg = (((uint16_t)buf[0]) << 8) | buf[1];
	if (reg & 0xF800)
		return INA231_OVERFLOW;		/* Overflow found */

	ret = i2cm.read(chid, WORD_ADDR8, info->addr, INA231_CURRENT, buf);
	if (ret != I2CM_ERROR_TRANSCEIVED)
		return INA231_I2C_BUSY;
	reg = (((uint16_t)buf[0]) << 8) | buf[1];
	info->curr = (int)reg;

	ret = i2cm.read(chid, WORD_ADDR8, info->addr, INA231_BUS_VOLT, buf);
	if (ret != I2CM_ERROR_TRANSCEIVED)
		return INA231_I2C_BUSY;
	reg = (((uint16_t)buf[0]) << 8) | buf[1];
	info->volt = INA231_BUS_mV((int)reg);

	ret = i2cm.read(chid, WORD_ADDR8, info->addr, INA231_POWER, buf);
	if (ret != I2CM_ERROR_TRANSCEIVED)
		return INA231_I2C_BUSY;
	reg = (((uint16_t)buf[0]) << 8) | buf[1];
	info->power = INA231_POW_mW((int)reg, info->current_unit);

	return INA231_OK;
}
#endif
/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
