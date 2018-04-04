/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    it_drv_i2c_slave.c
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
#include "it_drv_i2c_slave.h"
#include "qc_common.h"

/* Private define ------------------------------------------------------------*/
#define					DATA_LENGTH			2

LOCAL volatile uint8_t data_receive[2] = {00, 00};
LOCAL volatile uint8_t count = 0;
/* Static functions ----------------------------------------------------------*/

/**
 * @brief Test I2C SLAVE driver
 * @detail Test application for testing receive from host
 * @parameter[in]  :
 * 					- Slave address
 * 					- Data
 * @parameter[out] :NA
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_i2c_slave_001(char**argv, int argc);

/**
 * @brief Test I2C SLAVE driver
 * @detail Test application for testing send data to host
 * @parameter[in]  :
 * 					- Slave address
 * 					- Data
 * @parameter[out] :NA
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_i2c_slave_002(char**argv, int argc);

LOCAL it_map_t it_i2c_slave_tests_table[] =
{
		{"I2C_SLAVE_001", it_drv_i2c_slave_001},
		{"I2C_SLAVE_002", it_drv_i2c_slave_002},
		{"",  NULL}
};

/* Exported functions ------------------------------------------------------- */
/**
 * @brief I2C SLAVE module's testing handler
 * @detail to I2C SLAVE module's testing handler
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @return 0	 : success
 *         Others: instruction failed
 */
IMPORT int it_drv_i2c_slave_handler(char** argv, int argc)
{
	/**< Check command counter >*/
	if (1 >= argc) return -1;
	if (NULL == argv) return -1;
	/**< Get command index, mean test case name >*/
	int index = handler_parser(*argv, it_i2c_slave_tests_table);
	if (-1 != index)
	{
		return it_i2c_slave_tests_table[index].handler(&argv[1], argc - 1);
	}
	else
		return index;
}

LOCAL void i2c_slave_callback(uint8_t data)
{
	/**! Get data from host */
	data_receive[count] = data;
	count++;
}

LOCAL void i2c_slave_complete(void)
{
	/**! Reset counter */
	count = 0;
}

/**
 * @brief Test I2C SLAVE driver
 * @detail Test application for testing receive from host
 * @parameter[in]  :
 * 					- Slave address
 * 					- Data
 * @parameter[out] :NA
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_i2c_slave_001(char**argv, int argc)
{
	/**! Check command parameter and argument */
	if((2 != argc) || (NULL == argv))
	{
		log_printf("Error: Wrong number of parameter or Empty argument.\r\n");
	}

	uint32_t time_out = 200000;
	uint16_t  temp = 0x00;
	uint8_t data[2] = {00, 00};
	i2c_slave_t i2c_slave =
				{
					i2c_slave_callback,
					i2c_slave_complete,
					NULL,
					NULL,
					0x00,
				};

	/**! Get slave address via argv[0] */
	i2c_slave.slave_address = (uint8_t)strtol(argv[0], NULL, 16);

	/**! Get data via argv[1] */
	temp = (uint16_t)strtol(argv[1], NULL, 16);
	data[0] = ((temp >> 8) & 0xFF);
	data[1] = (temp & 0xFF);

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Call function i2c_slave_init */
	i2c_slave_init(&i2c_slave);

	/**! Wait receive data from host */
	while(--time_out);

	/**! Verify data receive from host */
	qc_assert(data[0] == data_receive[0]);
	qc_assert(data[1] == data_receive[1]);

	/**! Do judgment */
	qc_report();
	return 0;
}

/**
 * @brief Test I2C SLAVE driver
 * @detail Test application for testing send data to host
 * @parameter[in]  :
 * 					- Slave address
 * 					- Data
 * @parameter[out] :NA
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_i2c_slave_002(char**argv, int argc)
{
	/**! Check command parameter and argument */
	if((2 != argc) || (NULL == argv))
	{
		log_printf("Error: Wrong number of parameter or Empty argument.\r\n");
	}

	uint32_t time_out = 200000;
	uint16_t  temp = 0x00;
	uint8_t data[2] = {00, 00};
	i2c_slave_t i2c_slave =
				{
					i2c_slave_callback,
					i2c_slave_complete,
					NULL,
					NULL,
					0x00,
				};

	/**! Get slave address via argv[0] */
	i2c_slave.slave_address = (uint8_t)strtol(argv[0], NULL, 16);

	/**! Get data via argv[1] */
	temp = (uint16_t)strtol(argv[1], NULL, 16);
	data[0] = ((temp >> 8) & 0xFF);
	data[1] = (temp & 0xFF);

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Call function i2c_slave_init */
	i2c_slave_init(&i2c_slave);

	/**! Send data to host */
	i2c_slave_write(data, DATA_LENGTH);

	/**! Wait receive data from host */
	while(--time_out);

	/**! Verify data receive from host */
	qc_assert(data[0] == data_receive[0]);
	qc_assert(data[1] == data_receive[1]);

	/**! Do judgment */
	qc_report();
	return 0;
}
/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
