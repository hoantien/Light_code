/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    it_drv_timestamp.c
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
#include "it_drv_timestamp.h"
#include "qc_common.h"

/* Private define ------------------------------------------------------------*/
/* Static functions ----------------------------------------------------------*/

/**
 * @brief Test TIME_STAMP driver
 * @detail Test application for testing drv_timestamp_start API
 * and verify interrupt
 * @parameter[in]  :
 * 					- Time wait
 * @parameter[out] :N/A
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_timestamp_001(char** argv, int argc);

LOCAL it_map_t it_timestamp_tests_table[] =
{
	{"TIMESTAMP_001", it_drv_timestamp_001},
	{"",  NULL}
};

/* Exported functions ------------------------------------------------------- */
/**
 * @brief TIMESTAMP module's testing handler
 * @detail to TIMESTAMP module's testing handler
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @return 0	 : success
 *         Others: instruction failed
 */
IMPORT int it_drv_timestamp_handler(char** argv, int argc)
{
	/**< Check command counter >*/
	if (1 >= argc) return -1;
	if (NULL == argv) return -1;
	/**< Get command index, mean test case name >*/
	int index = handler_parser(*argv, it_timestamp_tests_table);
	if (-1 != index)
	{
		return it_timestamp_tests_table[index].handler(&argv[1], argc - 1);
	}
	else
		return index;
}

/**
 * @brief Test TIMESTAMP driver
 * @detail Test application for testing drv_timestamp_start API
 * @parameter[in]  :
 * 					- Time wait
 * @parameter[out] :NA
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_timestamp_001(char** argv, int argc)
{
	/**! Check command parameter and argument */
	if((1 != argc) || (NULL == argv))
	{
		log_printf("Error: Wrong number of parameter or Empty argument.\r\n");
	}

	uint16_t time_wait = 0;

	/**! Get time wait to via argv[0] */
	time_wait = (uint16_t)strtol(argv[0], NULL, 10);
	time_wait *= 200000;

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initialize trace timer */
	timestamp_start();

	/**! Wait flag interrupt enable */
	while(--time_wait);

	/**! Verify API timestamp_ms */
	qc_assert(timestamp_ms());

	return 0;
}

/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
