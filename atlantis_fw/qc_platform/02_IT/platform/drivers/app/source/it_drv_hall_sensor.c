/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    it_drv_hall_sensor.c
 * @author  The LightCo.
 * @version 1.0.0
 * @date    June, 2, 2016
 * @brief   
 *
 ******************************************************************************/

/******************************************************************************/
/**  						Revision history
 *
 * * 1.0.0	June, 2, 2016	Initial revision
 */
/******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdlib.h>
#include "it_drv_hall_sensor.h"
#include "qc_common.h"

/* Private define ------------------------------------------------------------*/
/* Static functions ----------------------------------------------------------*/

/**
 * @brief Test HALL SENSOR driver
 * @detail Test application for testing  hall_init API
 * @param[in]  :
 * 					- I2C channel
 * 					- Hall type
 * 					- Polarity
 * @param[out] :N/A
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_hall_sensor_001(char** argv, int argc);

/**
 * @brief Test HALL SENSOR driver
 * @detail Test application for testing hall_set_sensitivity API
 * @param[in]  :
 * 					- I2C Channel
 * 					- Hall type
 * 					- Sensitivity
 * @param[out] :N/A
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_hall_sensor_002(char** argv, int argc);

/**
 * @brief Test HALL SENSOR driver
 * @detail Test application for testing hall_read_position API
 * @param[in]  :
 * 					- Channel
 * 					- Hall type
 * @param[out] :N/A
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_hall_sensor_003(char** argv, int argc);

LOCAL it_map_t it_hall_sensor_tests_table[] =
{
		{"HALL_SENSOR_001", it_drv_hall_sensor_001},
		{"HALL_SENSOR_002", it_drv_hall_sensor_002},
		{"HALL_SENSOR_003", it_drv_hall_sensor_003},
		{"", NULL}
};

/* Exported functions ------------------------------------------------------- */
/**
 * @brief HALL SENSOR module's testing handler
 * @detail to HALL SENSOR module's testing handler
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @return 0	 : success
 *         Others: instruction failed
 */
int it_drv_hall_sensor_handler(char** argv, int argc)
{
	/**< Check command counter >*/
	if (1 >= argc) return -1;
	if (NULL == argv) return -1;
	/**< Get command index, mean test case name >*/
	int index = handler_parser(*argv, it_hall_sensor_tests_table);
	if (-1 != index)
	{
		return it_hall_sensor_tests_table[index].handler(&argv[1], argc - 1);
	}
	else
		return index;
}

/**
 * @brief Test HALL SENSOR driver
 * @detail Test application for testing hall_init API
 * @param[in]  :
 * 					- I2C Channel
 * 					- Hall type
 * 					- Polarity
 * @param[out] :NA
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_hall_sensor_001(char** argv, int argc)
{
	/**! Check command parameter and argument */
	if((3 != argc) || (NULL == argv))
	{
		log_printf("Error: Wrong number of parameter or Empty argument.\r\n");
	}

	hall_sensor_t test_hall;
	uint8_t type;
	bool polarity;

	/**! Configuration default */
	test_hall.i2c_chid = I2C_CH6;
	test_hall.hall_type = HALL_LENS;
	polarity = FALSE;
	type = 0;

	/**! Get Hall I2C channel to via argv[0] */
	test_hall.i2c_chid = (hal_i2c_channel_t)strtol(argv[0], NULL, 10);

	if((I2C_CH6 > test_hall.i2c_chid) || (I2C_CH17 < test_hall.i2c_chid))
	{
		log_printf("Error: Do not support channel.\r\n");
		return -1;
	}

	/**! Get Hall type to via argv[1] */
	type = (uint8_t)strtol(argv[1], NULL, 10);

	/**! Check value hall type */
	switch(type)
	{
		case 0:
		{
			test_hall.hall_type = HALL_LENS;
			break;
		}
		case 1:
		{
			test_hall.hall_type = HALL_MIRROR;
			break;
		}
		default:
		{
			log_printf("Error: Do not support hall type.\r\n");
			return -1;
		}
	}

	/**! Get Polarity to via argv[2] */
	polarity = (bool)strtol(argv[2], NULL, 10);

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Check hall_init() API */
	qc_assert(HALL_OK == hall_init(&test_hall, polarity));

	/**! Do judgment */
	qc_report();

	return HALL_OK;
}

/**
 * @brief Test HALL SENSOR driver
 * @detail Test application for testing hall_set_sensitivity() API
 * @param[in]  :
 * 					- I2C channel
 * 					- Hall type
 * 					- Polarity
 * 					- Sensitivity
 * @param[out] :N/A
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_hall_sensor_002(char** argv, int argc)
{
	/**! Check command parameter and argument */
	if((4 != argc) || (NULL == argv))
	{
		log_printf("Error: Wrong number of parameter or Empty argument.\r\n");
	}

	hall_sensor_t test_hall;
	hall_return_t ret = HALL_OK;
	uint8_t type;
	bool polarity;
	uint8_t sensitivity;

	/**! Configuration default */
	test_hall.i2c_chid = I2C_CH6;
	test_hall.hall_type = HALL_LENS;
	polarity = FALSE;
	sensitivity = 0;
	type = 0;

	/**! Get Hall I2C channel to via argv[0] */
	test_hall.i2c_chid = (hal_i2c_channel_t)strtol(argv[0], NULL, 10);

	if((I2C_CH6 > test_hall.i2c_chid) || (I2C_CH17 < test_hall.i2c_chid))
	{
		log_printf("Error: Do not support channel.\r\n");
		return -1;
	}

	/**! Get Hall type to via argv[1] */
	type = (uint8_t)strtol(argv[1], NULL, 10);

	/**! Check value hall type */
	switch(type)
	{
		case 0:
		{
			test_hall.hall_type = HALL_LENS;
			break;
		}
		case 1:
		{
			test_hall.hall_type = HALL_MIRROR;
			break;
		}
		default:
		{
			log_printf("Error: Do not support hall type.\r\n");
			return -1;
		}
	}

	/**! Get Polarity to via argv[2] */
	polarity = (bool)strtol(argv[2], NULL, 10);

	/**! Set Sensitivity to via argv[3] */
	sensitivity = (uint8_t)strtol(argv[3], NULL, 10);

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Check hall_init() API */
	qc_assert(HALL_OK == hall_init(&test_hall, polarity));

	/**! Check hall_set_sensitivity() API */
	ret = hall_set_sensitivity(&test_hall, sensitivity);
	qc_assert(HALL_OK == ret);

	if(HALL_INVALID_SENSITIVITY == ret)
	{
		log_printf("Error: Value sensitivity invalid.\r\n");
		return -1;
	}

	if(HALL_TIMED_OUT == ret)
	{
		log_printf("Error: I2C Time out!.\r\n");
		return -1;
	}
	/**! Do judgment */
	qc_report();

	return HALL_OK;
}

/**
 * @brief Test HALL SENSOR driver
 * @detail Test application for testing hall_read_position API
 * @param[in]  :
 * 					- I2C channel
 * 					- Polarity
 * 					- Hall type
 * @param[out] :N/A
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_hall_sensor_003(char** argv, int argc)
{
	/**! Check command parameter and argument */
	if((3 != argc) || (NULL == argv))
	{
		log_printf("Error: Wrong number of parameter or Empty argument.\r\n");
	}

	hall_return_t ret = HALL_OK;
	hall_sensor_t test_hall;
	uint8_t type;
	uint8_t sensitivity;
	uint16_t* temp1;
	uint16_t* temp2;
	bool polarity;

	/**! Configuration default */
	test_hall.i2c_chid = I2C_CH6;
	test_hall.hall_type = HALL_LENS;
	polarity = FALSE;
	sensitivity = 10;
	temp1 = malloc(sizeof(uint16_t));
	temp2 = malloc(sizeof(uint16_t));
	type = 0;

	/**! Get Hall I2C channel to via argv[0] */
	test_hall.i2c_chid = (hal_i2c_channel_t)strtol(argv[0], NULL, 10);

	if((I2C_CH6 > test_hall.i2c_chid) || (I2C_CH17 < test_hall.i2c_chid))
	{
		log_printf("Error: Do not support channel.\r\n");
		return -1;
	}

	/**! Get Hall type to via argv[1] */
	type = (uint8_t)strtol(argv[1], NULL, 10);

	/**! Check value hall type */
	switch(type)
	{
		case 0:
		{
			test_hall.hall_type = HALL_LENS;
			break;
		}
		case 1:
		{
			test_hall.hall_type = HALL_MIRROR;
			break;
		}
		default:
		{
			log_printf("Error: Do not support hall type.\r\n");
			return -1;
		}
	}

	/**! Get Polarity to via argv[2] */
	polarity = (bool)strtol(argv[2], NULL, 10);

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Check hall_init() API */
	qc_assert(HALL_OK == hall_init(&test_hall, polarity));

	/**! Check hall_read_position() API */
	qc_assert(HALL_OK == hall_read_position(&test_hall, temp1));

	/**! Check hall_set_sensitivity() API */
	qc_assert(HALL_OK == hall_set_sensitivity(&test_hall, sensitivity));

	/**! Check hall_read_position() API */
	ret = hall_read_position(&test_hall, temp2);
	qc_assert(HALL_OK == ret);
	if(HALL_INCORRECT_VALUE == ret)
	{
		log_printf("Error: Incorrect value hall sensor.\r\n");
		return -1;
	}
	/**! Verify different between two position */
	qc_assert((*temp1) != (*temp2));

	/**! Free pointer */
	free(temp1);
	free(temp2);

	/**! Do judgment */
	qc_report();

	return HALL_OK;
}

/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
