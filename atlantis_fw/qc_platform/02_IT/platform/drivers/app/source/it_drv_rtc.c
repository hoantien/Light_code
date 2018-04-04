/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    it_drv_rtc.c
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
#include <stdint.h>
#include "lcc_system.h"
#include "it_drv_rtc.h"
#include "qc_common.h"

/* Private define ------------------------------------------------------------*/
/* Static functions ----------------------------------------------------------*/

/**
 * @brief Test RTC driver
 * @detail Test application for testing rtc_init() API
 * @parameter[in]  :
 * 					- Variable Loop
 * @parameter[out] :N/A
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_rtc_001(char** argv, int argc);

LOCAL it_map_t it_rtc_tests_table[] =
{
		{"RTC_001", it_drv_rtc_001},
		{"",  NULL}
};

/* Exported functions ------------------------------------------------------- */
/**
 * @brief RTC module's testing handler
 * @detail to RTC module's testing handler
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @return 0	 : success
 *         Others: instruction failed
 */
IMPORT int it_drv_rtc_handler(char** argv, int argc)
{
	/**< Check command counter >*/
	if (1 >= argc) return -1;
	if (NULL == argv) return -1;
	/**< Get command index, mean test case name >*/
	int index = handler_parser(*argv, it_rtc_tests_table);
	if (-1 != index)
	{
		return it_rtc_tests_table[index].handler(&argv[1], argc - 1);
	}
	else
		return index;
}

/**
 * @brief Test RTC driver
 * @detail Test application for testing rtc_init() API
 * @parameter[in]  :
 * 					- Variable Loop
 * @parameter[out] :NA
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_rtc_001(char** argv, int argc)
{
	/**! Check command parameter and argument */
	if((1 != argc) || (NULL == argv))
	{
		log_printf("Error: Wrong number of parameter or Empty argument.\r\n");
	}

	uint32_t time_out = 2000000;
	uint16_t loop;

	/**! Get variable Loop to via argv[0] */
	loop = (uint16_t)strtol(argv[0], NULL, 10);
	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initialize trace timer */
	rtc_init();

	while(--loop)
	{
		while(--time_out);
		log_printf("Time: %l \r\n", light_system->t_time.raw);
		time_out = 2000000;
	}

	/**! Do judgment */
	qc_report();
	return 0;
}

/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
