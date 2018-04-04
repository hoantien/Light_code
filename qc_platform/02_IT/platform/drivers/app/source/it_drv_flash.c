/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    it_drv_flash.c
 * @author  The LightCo.
 * @version 1.0.0
 * @date    June, 3, 2016
 * @brief   
 *
 ******************************************************************************/

/******************************************************************************/
/**  						Revision history
 *
 * * 1.0.0	June, 3, 2016	Initial revision
 */
/******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdlib.h>
#include "it_drv_flash.h"
#include "qc_common.h"

/* Private define ------------------------------------------------------------*/
#define						NUMBER_SECTOR			0x01
#define						SIZE_SECTOR				0x00
/* Static functions ----------------------------------------------------------*/
/**
 * @brief Test FLASH driver
 * @detail Test application for testing flash_write API
 * @parameter[in]  :
 * 					- Address memory
 * 					- Data
 * 					- Length
 * 					- QSPI mode
 * @parameter[out] :N/A
 * @return 2		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_flash_001(char** argv, int argc);

/**
 * @brief Test FLASH driver
 * @detail Test application for testing flash_read API
 * @parameter[in]  :
 * 					- Address memory
 * 					- Length
 * 					- QSPI mode
 * @parameter[out] :N/A
 * @return 2		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_flash_002(char** argv, int argc);

/**
 * @brief Test FLASH driver
 * @detail Test application for testing flash_write and flash_read API
 * @parameter[in]  :
 * 					- Address memory
 * 					- Data
 * 					- Length
 * 					- QSPI mode
 * @parameter[out] :N/A
 * @return 2		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_flash_003(char** argv, int argc);

/**
 * @brief Test FLASH driver
 * @detail Test application for testing flash_sector_erase API
 * @parameter[in]  :
 * 					- Address memory
 * 					- Data
 * 					- Length
 * 					- QSPI mode
 * @parameter[out] :N/A
 * @return 2		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_flash_004(char** argv, int argc);

LOCAL it_map_t it_flash_tests_table[] =
{
		{"FLASH_001", it_drv_flash_001},
		{"FLASH_002", it_drv_flash_002},
		{"FLASH_003", it_drv_flash_003},
		{"FLASH_004", it_drv_flash_004},
		{"",  NULL}
};

/* Exported functions ------------------------------------------------------- */
/**
 * @brief FLASH module's testing handler
 * @detail to FLASH module's testing handler
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @return 0	 : success
 *         Others: instruction failed
 */
IMPORT int it_drv_flash_handler(char** argv, int argc)
{
	/**< Check command counter >*/
	if (1 >= argc) return -1;
	if (NULL == argv) return -1;
	/**< Get command index, mean test case name >*/
	int index = handler_parser(*argv, it_flash_tests_table);
	if (-1 != index)
	{
		return it_flash_tests_table[index].handler(&argv[1], argc - 1);
	}
	else
		return index;
}

/**
 * @brief Test FLASH driver
 * @detail Test application for testing flash_write API
 * @parameter[in]  :
 * 					- Address memory
 * 					- Data
 * 					- Length
 * 					- QSPI mode
 * @parameter[out] :N/A
 * @return 2		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_flash_001(char** argv, int argc)
{
	/**! Check command parameter and argument */
	if((4 != argc) || (NULL == argv))
	{
		log_printf("Error: Wrong number of parameter or Empty argument.\r\n");
	}

	uint32_t address = 0x00;
	uint32_t length = 0;
	uint8_t data[4] = {00, 00, 00, 00};
	uint16_t temp = 0x00;
	qspi_mode_t mode = SINGLE_MODE;

	/**! Get address via argv[0] */
	address = (uint32_t)strtol(argv[0], NULL, 16);

	/**! Get data length via argv[1] */
	length = (uint32_t)strtol(argv[1], NULL, 10);

	/**! Get data via argv[2] */
	temp = (uint16_t)strtol(argv[2], NULL, 16);
	data[0] = ((temp >> 8) & 0xFF) ;
	data[1] = (temp & 0xFF);

	/**! Get mode QSPI via argv[3] */
	mode = (qspi_mode_t)strtol(argv[3], NULL, 10);
	if((SINGLE_MODE != mode) && (QSPI_MODE != mode))
	{
		log_printf("Error: Do not support mode .\r\n");
		return -1;
	}
	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Verify write data to flash */
	qc_assert(FLASH_SUCCESS == flash_write(address, length, data, mode));

	/**! Do judgment */
	qc_report();

	return FLASH_SUCCESS;
}

/**
 * @brief Test FLASH driver
 * @detail Test application for testing flash_read API
 * @parameter[in]  :
 * 					- Address memory
 * 					- Length
 * 					- QSPI mode
 * @parameter[out] :N/A
 * @return 2		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_flash_002(char** argv, int argc)
{
	/**! Check command parameter and argument */
	if((4 != argc) || (NULL == argv))
	{
		log_printf("Error: Wrong number of parameter or Empty argument.\r\n");
	}

	uint32_t address = 0x00;
	uint32_t length = 0;
	uint8_t data[4] = {00, 00, 00, 00};
	qspi_mode_t mode = SINGLE_MODE;

	/**! Get address via argv[0] */
	address = (uint32_t)strtol(argv[0], NULL, 16);

	/**! Get data length via argv[1] */
	length = (uint32_t)strtol(argv[1], NULL, 10);

	/**! Get mode QSPI via argv[3] */
	mode = (qspi_mode_t)strtol(argv[2], NULL, 10);
	if((SINGLE_MODE != mode) && (QSPI_MODE != mode))
	{
		log_printf("Error: Do not support mode .\r\n");
		return -1;
	}

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Verify read data to flash */
	qc_assert(FLASH_SUCCESS == flash_read(address, length, data, mode));

	/**! Print data read to flash */
	log_printf("DATA: %x \r\n", data);

	/**! Do judgment */
	qc_report();
	return FLASH_SUCCESS;
}

/**
 * @brief Test FLASH driver
 * @detail Test application for testing flash_write and flash_read API
 * @parameter[in]  :
 * 					- Address memory
 * 					- Data
 * 					- Length
 * 					- QSPI mode
 * @parameter[out] :N/A
 * @return 2		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_flash_003(char** argv, int argc)
{
	/**! Check command parameter and argument */
	if((4 != argc) || (NULL == argv))
	{
		log_printf("Error: Wrong number of parameter or Empty argument.\r\n");
	}

	uint32_t address = 0x00;
	uint32_t length = 0;
	uint8_t data[4] = {00, 00, 00, 00};
	uint8_t buf[4] = {00, 00, 00, 00};
	uint32_t temp = 0x00;
	uint32_t time_out = 20000;
	qspi_mode_t mode = SINGLE_MODE;

	/**! Get address via argv[0] */
	address = (uint32_t)strtol(argv[0], NULL, 16);

	/**! Get data length via argv[1] */
	length = (uint32_t)strtol(argv[1], NULL, 10);

	/**! Get data via argv[2] */
	temp = (uint32_t)strtol(argv[2], NULL, 16);
	data[0] = ((temp >> 8) & 0xFF);
	data[1] = (temp & 0xFF);

	/**! Get mode QSPI via argv[3] */
	mode = (qspi_mode_t)strtol(argv[3], NULL, 10);
	if((SINGLE_MODE != mode) && (QSPI_MODE != mode))
	{
		log_printf("Error: Do not support mode .\r\n");
		return -1;
	}
	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Verify write data to flash */
	qc_assert(FLASH_SUCCESS == flash_write(address, length, data, mode));

	/** Wait flash read data !*/
	while(--time_out);

	/**! Verify read data from flash */
	qc_assert(FLASH_SUCCESS == flash_read(address, length, buf, mode));

	/**! Check data read from flash */
	for(int i = 0; i < length; i++)
	{
		qc_assert(data[i] == buf[i]);
	}

	/**! Do judgment */
	qc_report();

	return FLASH_SUCCESS;
}

/**
 * @brief Test FLASH driver
 * @detail Test application for testing flash_sector_erase API
 * @parameter[in]  :
 * 					- Address memory
 * 					- Data
 * 					- Length
 * 					- QSPI mode
 * @parameter[out] :N/A
 * @return 2		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_flash_004(char** argv, int argc)
{
	/**! Check command parameter and argument */
	if((4 != argc) || (NULL == argv))
	{
		log_printf("Error: Wrong number of parameter or Empty argument.\r\n");
	}

	uint32_t address = 0x00;
	uint32_t length = 0;
	uint8_t data[4] = {00, 00, 00, 00};
	uint8_t buf[4] = {00, 00, 00, 00};
	uint32_t temp = 0x00;
	uint32_t time_out = 20000;
	qspi_mode_t mode = SINGLE_MODE;

	/**! Get address via argv[0] */
	address = (uint32_t)strtol(argv[0], NULL, 16);

	/**! Get data length via argv[1] */
	length = (uint32_t)strtol(argv[1], NULL, 10);

	/**! Get data via argv[2] */
	temp = (uint32_t)strtol(argv[2], NULL, 16);
	data[0] = ((temp >> 8) & 0xFF);
	data[1] = (temp & 0xFF);

	/**! Get mode QSPI via argv[3] */
	mode = (qspi_mode_t)strtol(argv[3], NULL, 10);
	if((SINGLE_MODE != mode) && (QSPI_MODE != mode))
	{
		log_printf("Error: Do not support mode .\r\n");
		return -1;
	}
	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Verify write data to flash */
	qc_assert(FLASH_SUCCESS == flash_write(address, length, data, mode));

	/** Wait flash read data !*/
	while(--time_out);

	/**! Verify read data from flash */
	qc_assert(FLASH_SUCCESS == flash_read(address, length, buf, mode));

	/**! Check data read from flash */
	for(int i = 0; i < length; i++)
	{
		qc_assert(data[i] == buf[i]);
	}
	/**! Erase data to memory address */
	qc_assert(FLASH_SUCCESS ==
				flash_sector_erase(address, NUMBER_SECTOR, SIZE_SECTOR));

	time_out = 20000;
	/** Wait flash erase data !*/
	while(--time_out);

	/**! Verify read data from flash */
	qc_assert(FLASH_SUCCESS == flash_read(address, length, buf, mode));

	/**! Check data erase from flash */
	for(int i = 0; i < length; i++)
	{
		qc_assert(data[i] != buf[i]);
	}

	/**! Do judgment */
	qc_report();

	return FLASH_SUCCESS;
}
/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
