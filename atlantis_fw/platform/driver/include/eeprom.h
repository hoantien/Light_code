/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification,are strictly prohibited without prior permission of The LightCo.
 *
 * @file    camera_eeprom.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    July-11-2016
 * @brief   This file contains expand of the eeprom driver
 *
 ******************************************************************************/
/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __EEPROM_H__
#define __EEPROM_H__

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "std_type.h"
#include "i2cm.h"

/* Exported define------------------------------------------------------------*/
#define CAM_EEPROM_MIN_RANGE		0x0000
#define CAM_EEPROM_MAX_RANGE		0x153C
#define EEPROM_PAGE_SIZE			32
#define EEPROM_SECTOR_NUM			256
#define EEPROM_MAX_SIZE				(EEPROM_PAGE_SIZE * EEPROM_SECTOR_NUM)
/* Exported typedef  ---------------------------------------------------------*/
/*
 * @brief eeprom_status
 *
 * EEPROM return type
 */
typedef enum eeprom_status
{
	EEPROM_OK,
	EEPROM_TIMEOUT,
	EEPROM_UNKNOWN_ERROR
} eeprom_status_t;

/*
 * @brief cam_eeprom_t
 *
 * EEPROM data struct define
 */
typedef struct cam_eeprom
{
	uint16_t	offset;			/* The offset address to read/write */
	uint16_t	len;			/* The len of data to read/write */
	uint8_t		flag;			/* The flag indicate action read/write */
	uint8_t		*buf;			/* The buffer to store the data */
}cam_eeprom_t;

/*
 * @brief cam_eeprom_action
 *
 * EEPROM action read/write
 */
typedef enum eeprom_action
{
	EEPROM_ACTION_SUCCESS,
	EEPROM_ACTION_READ,
	EEPROM_ACTION_WRITE,
	EEPROM_ACTION_FAILED
} eeprom_action_t;

/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/*
 * @brief eeprom_read
 * Read data from eeprom.
 * @param i2c_dev I2C channel ID
 * @param offset : Internal address to write to.
 * @param len: Number of bytes to read from eeprom
 * @param rx_buf: Pointer to the buffer containing the data to be read.
 * @return eeprom_status_t
 */
eeprom_status_t eeprom_read(i2c_t i2c_dev, uint16_t offset,
												uint16_t len, uint8_t *rxbuf);

/*
 * @brief eeprom_write
 * Write data to eeprom.
 * @param i2c_dev I2C channel ID
 * @param offset : Internal address to write to.
 * @param len: Number of bytes to read from eeprom
 * @param tx_buf: Pointer to the buffer containing the data to be written.
 * @return eeprom_status_t
 */
eeprom_status_t eeprom_write(i2c_t i2c_dev, uint16_t offset,
												uint16_t len, uint8_t *txbuf);

#ifdef __cplusplus
}
#endif
#endif /* __EEPROM_H__ */
/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
