/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    it_drv_temp_sensor.c
 * @author  The LightCo.
 * @version 1.0.0
 * @date    May 30, 2016
 * @brief   
 *
 ******************************************************************************/

/******************************************************************************/
/**  						Revision history
 *
 * * 1.0.0	May 30, 2016	Initial revision
 */
/******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdlib.h>
#include "it_drv_temp_sensor.h"
#include "qc_common.h"

/* Private define ------------------------------------------------------------*/
#define TEMP_TOLERANCE					2
/* Static functions ----------------------------------------------------------*/

/**
 * @brief Test temp_sensor driver
 * @detail Test application for testing temp_sensor_read API
 * @param[in]  :
 * 					- Temperature channel
 * @param[out] :N/A
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_temp_sensor_001(char** argv, int argc);

LOCAL it_map_t it_temp_sensor_tests_table[] =
{
		{"TEMP_SENSOR_001", it_drv_temp_sensor_001},
		{"", NULL}
};

/* Exported functions ------------------------------------------------------- */
/**
 * @brief TEMP_SENSOR module's testing handler
 * @detail to TEMP_SENSOR module's testing handler
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @return 0	 : success
 * Others: instruction failed
 */
int it_drv_temp_sensor_handler(char** argv, int argc)
{
	/**< Check command counter >*/
	if (1 >= argc) return -1;
	if (NULL == argv) return -1;
	/**< Get command index, mean test case name >*/
	int index = handler_parser(*argv, it_temp_sensor_tests_table);
	if (-1 != index)
	{
		return it_temp_sensor_tests_table[index].handler(&argv[1], argc - 1);
	}
	else
		return index;
}

/**
 * @brief Test temp_sensor driver
 * @detail Test application for testing temp_sensor_read API
 * @parameter[in]  :
 * 					- Temperature channel
 * @parameter[out] :N/A
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_temp_sensor_001(char** argv, int argc)
{
	/**! Check command parameter and argument */
	if((2 != argc) || (NULL == argv))
	{
		log_printf("Error: Wrong number of parameter or Empty argument.\r\n");
	}

	/**! Configuration */
	uint16_t channel_01 = 0;
	uint16_t channel_02 = 0;
	float temp1 = 0;
	float temp2 = 0;
	uint16_t time_out = 20000;

	/**! Get temperature channel 1 to via argv[0] */
	channel_01 = (uint16_t)strtol(argv[0], NULL, 10);

	/**! Check value channel invalid */
	if((TEMP_SENS_CH1 > channel_01) || (TEMP_SENS_CH4 < channel_01))
	{
		log_printf("Error: Do not support channel.\r\n");
	}
	/**! Get temperature channel 2 to via argv[1] */
	channel_02 = (uint16_t)strtol(argv[1], NULL, 10);

	/**! Check value channel invalid */
	if((channel_01 == channel_02) || (TEMP_SENS_CH1 > channel_02)	\
									|| (TEMP_SENS_CH4 < channel_02))
	{
		log_printf("Error: Do not support channel.\r\n");
	}
	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initialize temperature sensor */
	temp_sensor_init();

	/**! Wait temperature configuration */
	while(--time_out);

	/**! Read value temperature from sensor */
	temp1 = temp_sensor_read(channel_01);
	temp2 = temp_sensor_read(channel_02);

	log_printf("Data temperature sensor 1 : %f\r\n", temp1);
	log_printf("Data temperature sensor 2 : %f\r\n", temp2);
	/**! Verify temperature is working */
	qc_assert(abs(temp1 - temp2) <= TEMP_TOLERANCE);

	/**! Do judgment */
	qc_report();
	return TEMP_OK;
}

/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
