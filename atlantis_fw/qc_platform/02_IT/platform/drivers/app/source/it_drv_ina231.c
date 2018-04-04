/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    it_drv_ina231.c
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
#include "it_drv_ina231.h"
#include "qc_common.h"

/* Private define ------------------------------------------------------------*/
/* Static functions ----------------------------------------------------------*/

/**
 * @brief Test INA231 driver
 * @detail Test application for testing ina231_config API
 * @param[in]  :
 * 					- I2C channel
 * 					- SHUNT resistor
 * 					- Current LSB
 * 					- ina231 address
 * @param[out] :N/A
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_ina231_001(char** argv, int argc);

/**
 * @brief Test INA231 driver
 * @detail Test application for testing ina231_get_info API
 * @param[in]  :
 * 					- I2C channel
 * 					- SHUNT resistor
 * 					- Current LSB
 * 					- ina231 address
 * @param[out] :N/A
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_ina231_002(char** argv, int argc);

LOCAL it_map_t it_ina231_tests_table[] =
{
		{"INA231_001", it_drv_ina231_001},
		{"INA231_002", it_drv_ina231_002},
		{"", NULL}
};

/* Exported functions ------------------------------------------------------- */
/**
 * @brief INA231 module's testing handler
 * @detail to INA231 module's testing handler
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @return 0	 : success
 *         Others: instruction failed
 */
int it_drv_ina231_handler(char** argv, int argc)
{
	/**< Check command counter >*/
	if (1 >= argc) return -1;
	if (NULL == argv) return -1;
	/**< Get command index, mean test case name >*/
	int index = handler_parser(*argv, it_ina231_tests_table);
	if (-1 != index)
	{
		return it_ina231_tests_table[index].handler(&argv[1], argc - 1);
	}
	else
		return index;
}

/**
 * @brief Test INA231 driver
 * @detail Test application for testing ina231_config API
 * @param[in]  :
 * 					- SHUNT resistor
 * 					- Current LSB
 * 					- ina231 address
 * @param[out] :NA
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_ina231_001(char** argv, int argc)
{
	/**! Check command parameter and argument */
	if((3 != argc) || (NULL == argv))
	{
		log_printf("Error: Wrong number of parameter or Empty argument.\r\n");
	}

	ina231_info_t info;
	hal_i2c_channel_t chid;
	int ret = INA231_OK;

	/**! Configuration default */
	chid = I2C_CH0;

	info.addr = 0x80;
	info.is_calibrated = 0;
	info.curr = 0;
	info.power = 0;
	info.volt = 0;
	info.rsense = 1;
	info.current_unit = 1;

	/**! Get I2C channel via argv[0] */
	chid = (hal_i2c_channel_t)strtol(argv[0], NULL, 16);

	/**! Get Resistor SHUNT to via argv[1] */
	info.rsense = (uint16_t)strtol(argv[1], NULL, 10);

	/**! Get current LSB to via argv[2] */
	info.current_unit = (uint16_t)strtol(argv[2], NULL, 10);

	/**! Get address ina231 to via argv[3] */
	info.addr = (uint8_t)strtol(argv[3], NULL, 16);

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Check API ina231_config */
	ret = ina231_config(chid, &info);
	qc_assert(INA231_OK == ret);
	switch(ret)
	{
		case 1 :
			{
				log_printf("Error: INA231 channel busy.\r\n");
				return -1;
			}
		case 4:
			{
				log_printf("Error: Value input invalid.\r\n");
				return -1;
			}
		default: break;
	}
	/**! Do judgment */
	qc_report();

	return INA231_OK;
}

/**
 * @brief Test INA231 driver
 * @detail Test application for testing ina231_get_info API
 * @param[in]  :
 * 					- I2C channel
 * 					- SHUNT Resistor
 * 					- Current LSB
 * 					- Ina231 address
 * @param[out] :N/A
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_ina231_002(char** argv, int argc)
{
	/**! Check command parameter and argument */
	if((3 != argc) || (NULL == argv))
	{
		log_printf("Error: Wrong number of parameter or Empty argument.\r\n");
	}

	ina231_info_t info;
	hal_i2c_channel_t chid;
	uint16_t time_out = 20000;
	int ret = INA231_OK;

	/**! Configuration default */
	chid = I2C_CH0;

	info.addr = 0x80;
	info.is_calibrated = 0;
	info.curr = 0;
	info.power = 0;
	info.volt = 0;
	info.rsense = 1;
	info.current_unit = 1;

	/**! Get I2C channel via argv[0] */
	chid = (hal_i2c_channel_t)strtol(argv[0], NULL, 16);

	/**! Get Resistor SHUNT to via argv[1] */
	info.rsense = (uint16_t)strtol(argv[1], NULL, 10);

	/**! Get current LSB to via argv[2] */
	info.current_unit = (uint16_t)strtol(argv[2], NULL, 10);

	/**! Get address ina231 to via argv[3] */
	info.addr = (uint8_t)strtol(argv[3], NULL, 16);

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Check API ina231_config */
	qc_assert(INA231_OK == ina231_config(chid, &info));

	/**! Wait configuration */
	while(--time_out);

	/**! Verify ina231_get_info API */
	ret = ina231_get_info(chid, &info);

	if(INA231_OK == ret)
	{
		qc_assert(TRUE);
		/**! Print value Volt,Power,Current */
		log_printf("Volt(mV) : %d\r\n", info.volt);
		log_printf("Power(mW) : %d\r\n", info.power);
		log_printf("Current(uA) : %d\r\n", info.curr);
	}
	else
	{
		qc_assert(FALSE);
		/**! Print Error */
		switch(ret)
		{
			case INA231_I2C_BUSY:
			{
				log_printf("Error: I2C Busy.\r\n");
				break;
			}
			case INA231_UNCALIBRATED:
			{
				log_printf("Error: INA231 module do not calibrate.\r\n");
				break;
			}
			case INA231_OVERFLOW:
			{
				log_printf("Error: INA231 module overflow.\r\n");
				break;
			}
			case INA231_VALUE_INVALID:
			{
				log_printf("Error: INA231 module address error.\r\n");
				break;
			}
			default: break;
		}
	}
	/**! Do judgment */
	qc_report();

	return INA231_OK;
}

/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
