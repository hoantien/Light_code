/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms,
 * with or without modification,
 * are strictly prohibited without prior permission of The LightCo.
 *
 * @file    flash.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    01-Feb-2016
 * @brief   This file contains all the functions prototype of the flash driver.
 *
 ******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FLASH_H__
#define __FLASH_H__

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "hal_qspi.h"

/* Exported define -----------------------------------------------------------*/
/* Exported typedef ----------------------------------------------------------*/
/**
 * @brief flash_status_t
 * Flash status type definition
 */
typedef enum flash_status
{
	FLASH_ERROR	= 0,
	FLASH_BUSY,
	FLASH_SUCCESS
} flash_status_t;

/* Exported variables --------------------------------------------------------*/
/**
 * @brief string_flash_status
 * Get string status
 */
extern char *string_flash_status[];

/* Exported functions ------------------------------------------------------- */
/*
 * @brief flash_init
 * Flash initialize
 * @param void
 * @return void
 */
void flash_init(void);

/*
 * @brief read_id
 * Read id number
 * @param void
 * @return	id number
 */
uint32_t read_id(void);

/*
 * @brief bulk_erase
 * Bulk erase flash
 * @param void
 * @return	flash status
 */
flash_status_t bulk_erase(void);

/*
 * @brief flash_sector_erase
 * Sector erase
 * @param first_sector : sector begin to erase (unit : sector)
 * @param number_sectors : number sectors erase
 * @return	flash status
 */
flash_status_t flash_sector_erase(uint16_t first_sector,
									uint16_t number_sectors);

/*
 * flash_sub_sector_erase
 * Sub sector erase
 * @param first_sector : sector begin to erase (unit : sub sector)
 * @param number_sectors : the number of sub sectors erased
 * @return	flash status
 */
flash_status_t flash_sub_sector_erase(uint16_t first_sector,
										uint16_t number_sectors);
/*
 * @brief flash_read
 * Read flash
 * @param addr : address begin to read (unit : sector)
 * @param len : length to read
 * @param *buf : buffer to read
 * @param qmode : qspi mode (quad mode or nomal mode)
 * @return	flash status
 */
flash_status_t flash_read(uint32_t addr, uint32_t len,\
		uint8_t *buf, qspi_mode_t qmode);

/*
 * @brief flash_write
 * Write flash
 * @param from : address begin to write (unit : sector)
 * @param len : length to write
 * @param *buf : buffer to write
 * @param qmode : qspi mode (quad mode or nomal mode)
 * @return	flash status
 */
flash_status_t flash_write(uint32_t addr, uint32_t len,\
		const uint8_t *buf, qspi_mode_t qmode);
#ifdef __cplusplus
extern "C"
}
#endif
#endif /* __FLASH_H__ */
/*********** Portions COPYRIGHT 2016 Light.Co., Ltd. ********END OF FILE*******/
