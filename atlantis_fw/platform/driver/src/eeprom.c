/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification,are strictly prohibited without prior permission of The LightCo.
 * @file    camera_eeprom.c
 * @author  The LightCo
 * @version V2.0.0
 * @date    July-11-2016
 * @brief   This file contain definition for the eeprom's APIs.
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "os.h"
#include "assert.h"
#include "eeprom.h"
#include "lcc_system.h"

/* Private define ------------------------------------------------------------*/
#define EEPROM_SLAVE_ADDR			IMAGE_EEPROM_ADDRESS
#define TIMEOUT						1000

/* Private macro -------------------------------------------------------------*/
#define IS_OFFSET_VALID(offset)		(offset < EEPROM_MAX_SIZE ? TRUE : FALSE)
#define IS_SIZE_VALID(offset, len)	(offset + len <= EEPROM_MAX_SIZE ? \
										TRUE : FALSE)

/* Global variable -----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private prototype functions -----------------------------------------------*/
static eeprom_status_t eeprom_write_page(i2c_t i2c_dev, uint16_t offset,
												uint16_t len, uint8_t *txbuf);

/* Exported functions --------------------------------------------------------*/
/*
 * eeprom_read
 */
eeprom_status_t eeprom_read(i2c_t i2c_dev, uint16_t offset,
							uint16_t len, uint8_t *rxbuf)
{
	/* Checking parameter */
	assert_param(rxbuf);
	assert_param(IS_OFFSET_VALID(offset));
	assert_param(IS_SIZE_VALID(offset, len));
	uint16_t addr = offset;
	uint8_t buf[2];
	buf[0] = (uint8_t)(addr >> 8);
	buf[1] = (uint8_t)addr;

	return (i2cm.transceiver(i2c_dev, EEPROM_SLAVE_ADDR, buf, 2, rxbuf, len)
				== I2CM_ERROR_TRANSCEIVED) ?
			EEPROM_OK : EEPROM_TIMEOUT;
}

/*
 * eeprom_write
 */
eeprom_status_t eeprom_write(i2c_t i2c_dev, uint16_t offset,
								uint16_t len, uint8_t *txbuf)
{
	eeprom_status_t ret = EEPROM_OK;
	uint16_t num_of_data, page_offset;

	/* Checking parameter */
	assert_param(txbuf);
	assert_param(IS_OFFSET_VALID(offset));
	assert_param(IS_SIZE_VALID(offset, len));

	while ((len > 0) && (ret == EEPROM_OK))
	{
		page_offset = offset % EEPROM_PAGE_SIZE;
		if ((len + page_offset) <= EEPROM_PAGE_SIZE)
			num_of_data = len;
		else
			num_of_data = EEPROM_PAGE_SIZE - page_offset;

		ret = eeprom_write_page(i2c_dev, offset, num_of_data, txbuf);
		offset += num_of_data;
		txbuf += num_of_data;
		len -= num_of_data;
	}

	return ret;
}

/* Private functions implementation ------------------------------------------*/
/*
 * eeprom_write_page
 */
static eeprom_status_t eeprom_write_page(i2c_t i2c_dev, uint16_t offset,
												uint16_t len, uint8_t *txbuf)
{
	uint8_t *data = NULL;
	data = assert_malloc(data, (len + 2) * sizeof(uint8_t));
	/* Copy offset to first pointer position*/
	*data = (uint8_t)(offset >> 8);
	*(data + 1) = (uint8_t)offset;
	memcpy(data + 2, txbuf, len);

	/* Increase size of transmission data by 2 */
	len += 2;

	i2cm_error_t status = i2cm.transceiver(i2c_dev, EEPROM_SLAVE_ADDR,
											data, len, NULL, 0);
	/* Free tx data after it was transmitted*/
	vPortFree(data);

	return (status == I2CM_ERROR_TRANSMITTED) ?
			EEPROM_OK : EEPROM_TIMEOUT;
}

/*********** Portions COPYRIGHT 2016 Light.Co., Ltd. ********END OF FILE*******/
