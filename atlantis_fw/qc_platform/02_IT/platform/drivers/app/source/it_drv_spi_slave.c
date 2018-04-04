/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    it_drv_spi_slave.c
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
#include "it_drv_spi_slave.h"
#include "qc_common.h"

/* Private define ------------------------------------------------------------*/
volatile uint8_t spi_data[4] = {00, 00,00, 00};
volatile uint16_t spi_length = 0;
volatile uint16_t count = 0;
/* Static functions ----------------------------------------------------------*/

/**
 * @brief Test SPI_SLAVE driver
 * @detail Test application for testing spi_slave_init API
 * @parameter[in]  :
 * 					- Data
 * 					- Length
 * @parameter[out] :N/A
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_spi_slave_001(char** argv, int argc);

/**
 * @brief Test SPI_SLAVE driver
 * @detail Test application for testing spi_slave_tx API
 * @parameter[in]  :
 * 					- Data
 * 					- Length
 * @parameter[out] :N/A
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_spi_slave_002(char** argv, int argc);

LOCAL it_map_t it_spi_slave_tests_table[] =
{
		{"SPI_SLAVE_001", it_drv_spi_slave_001},
		{"SPI_SLAVE_002", it_drv_spi_slave_002},
		{"",  NULL}
};

/* Exported functions ------------------------------------------------------- */
/**
 * @brief SPI SLAVE module's testing handler
 * @detail to SPI SLAVE module's testing handler
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @return 0	 : success
 *         Others: instruction failed
 */
IMPORT int it_drv_spi_slave_handler(char** argv, int argc)
{
	/**< Check command counter >*/
	if (1 >= argc) return -1;
	if (NULL == argv) return -1;
	/**< Get command index, mean test case name >*/
	int index = handler_parser(*argv, it_spi_slave_tests_table);
	if (-1 != index)
	{
		return it_spi_slave_tests_table[index].handler(&argv[1], argc - 1);
	}
	else
		return index;
}

/**
 * @brief Test Interrupt spi_slave driver
 * @detail Test application for testing Interrupt spi_slave
 * @parameter[in]  : N/A
 * @parameter[out] :
 * 					- Data
 * 					- Length
 * @return 0		: success
 * Others			: instruction failed
 */
void spi_callback(uint8_t* data, uint16_t len)
{
	spi_data[count] = *data;
	spi_length = len;
	count++;
}
/**
 * @brief Test SPI_SLAVE driver
 * @detail Test application for testing spi_slave_init API
 * @parameter[in]  :
 * @parameter[out] :N/A
 * @return 2		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_spi_slave_001(char** argv, int argc)
{
	/**! Check command parameter and argument */
	if((2 != argc) || (NULL == argv))
	{
		log_printf("Error: Wrong number of parameter or Empty argument.\r\n");
	}

	uint8_t data[4] = {00, 00, 00, 00};
	uint16_t data_len = 0;
	uint16_t temp = 0x00;
	uint32_t time_out = 20000;
	spi_slave_t spi;
	spi.rx_clbk = spi_callback;

	/**! Get data via argv[0] */
	temp = (uint16_t)strtol(argv[0], NULL, 16);
	data[0] = ((temp >> 8)& 0xFF);
	data[1] = (temp & 0xFF);

	/**! Get data length via argv[1] */
	data_len = (uint16_t)strtol(argv[1], NULL, 10);

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initialize spi_slave*/
	spi_slave_init(&spi);
	spi_slave_release_semaphore();

	while(--time_out);

	/**! Check data receive from master */
	for(int i = 0; i < spi_length; i++)
	{
		qc_assert(data[i] == spi_data[i]);
	}

	/**! Check length receive from master */
	qc_assert(data_len == spi_length);

	/**! Do judgment */
	qc_report();

	return 0;
}

/**
 * @brief Test SPI_SLAVE driver
 * @detail Test application for testing spi_slave_tx API
 * @parameter[in]  :
 * 					- Data
 * 					- Length
 * 					- TX ENABLE
 * @parameter[out] :N/A
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_spi_slave_002(char** argv, int argc)
{
	/**! Check command parameter and argument */
	if((2 != argc) || (NULL == argv))
	{
		log_printf("Error: Wrong number of parameter or Empty argument.\r\n");
	}

	uint8_t data[4] = {00, 00, 00, 00};
	uint16_t temp = 0x00;
	uint16_t data_len = 0;
	uint8_t tx_enable = FALSE;
	uint32_t time_out = 20000;
	spi_slave_t spi;
	spi.rx_clbk = spi_callback;

	/**! Get data via argv[0] */
	temp = (uint16_t)strtol(argv[0], NULL, 16);
	data[0] = ((temp >> 8)& 0xFF);
	data[1] = (temp & 0xFF);

	/**! Get data length via argv[1] */
	data_len = (uint16_t)strtol(argv[1], NULL, 16);

	/**! Get variable transfer data via argv[2] */
	tx_enable = (uint8_t)strtol(argv[2], NULL, 10);
	if((FALSE != tx_enable) && (TRUE != tx_enable))
	{
		log_printf("Error: variable tx_enable invalid.\r\n");
		return -1;
	}
	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initialize spi_slave*/
	spi_slave_init(&spi);
	spi_slave_tx(data, data_len, tx_enable);

	while(--time_out);
	time_out = 20000;
	spi_slave_release_semaphore();

	while(--time_out);
	/**! Check data receive from master */
	for(int i = 0; i < spi_length; i++)
	{
		qc_assert(data[i] == spi_data[i]);
	}

	/**! Do judgment */
	qc_report();
	return 0;
}
/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
