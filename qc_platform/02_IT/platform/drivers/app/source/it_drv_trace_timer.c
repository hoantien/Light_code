/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    it_drv_trace_timer.c
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
#include "it_drv_trace_timer.h"
#include "qc_common.h"

/* Private define ------------------------------------------------------------*/
/* Static functions ----------------------------------------------------------*/

/**
 * @brief Test TRACE_TIMER driver
 * @detail Test application for testing drv_trace_timer_init API
 * and verify interrupt
 * @parameter[in]  : Time wait
 * @parameter[out] :N/A
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_trace_timer_001(char** argv, int argc);

LOCAL it_map_t it_trace_timer_tests_table[] =
{
		{"TRACE_TIMER_001", it_drv_trace_timer_001},
		{"",  NULL}
};

/* Exported functions ------------------------------------------------------- */
/**
 * @brief TRACE_TIMER module's testing handler
 * @detail to TRACE_TIMER module's testing handler
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @return 0	 : success
 *         Others: instruction failed
 */
IMPORT int it_drv_trace_timer_handler(char** argv, int argc)
{
	/**< Check command counter >*/
	if (1 >= argc) return -1;
	if (NULL == argv) return -1;
	/**< Get command index, mean test case name >*/
	int index = handler_parser(*argv, it_trace_timer_tests_table);
	if (-1 != index)
	{
		return it_trace_timer_tests_table[index].handler(&argv[1], argc - 1);
	}
	else
		return index;
}

/**
 * @brief Test TRACE_TIMER driver
 * @detail Test application for testing it_drv_trace_timer_init API
 * @parameter[in]  :
 * 					- Time wait
 * @parameter[out] :NA
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_trace_timer_001(char** argv, int argc)
{
	/**! Check command parameter and argument */
	if((1 != argc) || (NULL == argv))
	{
		log_printf("Error: Wrong number of parameter or Empty argument.\r\n");
	}

	uint16_t time_wait = 0;

	/**! Get timer wait to via argv[0] */
	time_wait = (uint16_t)strtol(argv[0], NULL, 10);
	time_wait *= 200000;

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initialize trace timer */
	trace_timer_init();

	/**! Wait flag interrupt enable */
	while(--time_wait);

	/**! Verify interrupt work */
	qc_assert(trace_timer_get_runtime_value());

	/**! Do judgment */
	qc_report();

	return 0;
}

/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
