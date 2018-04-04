/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    it_drv_timer.c
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
#include "it_drv_timer.h"
#include "qc_common.h"

/* Private define ------------------------------------------------------------*/
/* Static functions ----------------------------------------------------------*/

/**
 * @brief Test TIMER driver
 * @detail Test application for testing timer_start() API
 * and verify interrupt
 * @parameter[in]  :
 * 					- Interval
 * @parameter[out] :N/A
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_timer_001(char** argv, int argc);

/**
 * @brief Test TIMER driver
 * @detail Test application for testing timer_stop API
 * and verify interrupt
 * @parameter[in]  :
 * 					- Interval
 * @parameter[out] :N/A
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_timer_002(char** argv, int argc);

/**
 * @brief Test TIMER driver
 * @detail Test application for testing timer_start after call timer_stop API
 * and verify interrupt
 * @parameter[in]  :
 * 					- Interval
 * @parameter[out] :N/A
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_timer_003(char** argv, int argc);

/**
 * @brief Test TIMER driver
 * @detail Test application for testing mode auto reload
 * and verify interrupt
 * @parameter[in]  :
 * 					- Interval
 * 					- Auto reload
 * @parameter[out] :N/A
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_timer_004(char** argv, int argc);

LOCAL it_map_t it_timer_tests_table[] =
{
		{"TIMER_001", it_drv_timer_001},
		{"TIMER_002", it_drv_timer_002},
		{"TIMER_003", it_drv_timer_003},
		{"TIMER_004", it_drv_timer_004},
		{"",  NULL}
};

/* Exported functions ------------------------------------------------------- */
/**
 * @brief TIMER module's testing handler
 * @detail to TIMER module's testing handler
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @return 0	 : success
 *         Others: instruction failed
 */
int it_drv_timer_handler(char** argv, int argc)
{
	/**< Check command counter >*/
	if (1 >= argc) return -1;
	if (NULL == argv) return -1;
	/**< Get command index, mean test case name >*/
	int index = handler_parser(*argv, it_timer_tests_table);
	if (-1 != index)
	{
		return it_timer_tests_table[index].handler(&argv[1], argc - 1);
	}
	else
		return index;
}

/**
 * @brief Test TIMER driver
 * @detail Test application for testing timer_start() API
 * @parameter[in]  :
 * 					- Interval
 * @parameter[out] :NA
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_timer_001(char** argv, int argc)
{
	/**! Check command parameter and argument */
	if((1 != argc) || (NULL == argv))
	{
		log_printf("Error: Wrong number of parameter or Empty argument.\r\n");
	}

	/**! define pointer test timer and variable */
	uint32_t interval = 0;
	uint32_t time_out = 0;
	struct timer_t *test_timer[5] = {NULL, NULL, NULL, NULL, NULL};
	/**! Get interval to via argv[0] */
	interval = (uint32_t)strtol(argv[0], NULL, 10);

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initialize timer */
	timer_init();

	for(int i = 0; i < TIMER_MAX_NBR; i++)
	{
		time_out = 2000;
		/**! Create timer  */
		test_timer[i] = timer_create();

		/**! Verify create timer successful */
		qc_assert(NULL != test_timer[i]);

		/**! Configuration timer */
		test_timer[i]->interval = interval;
		test_timer[i]->param = test_timer;

		/**! Check timer_start API */
		qc_assert(!timer_start(test_timer[i]));
		qc_assert(TIMER_STARTED == test_timer[i]->state);

		/**! Wait timer counter */
		while(--time_out);

		qc_assert(test_timer[i]->time_expired);
	}

	/**! Free memory timer */
	for(int i = 0; i < TIMER_MAX_NBR; i++)
	{
		timer_delete(&test_timer[i]);
	}
	/**! Do judgment */
	qc_report();

	return 0;
}

/**
 * @brief Test TIMER driver
 * @detail Test application for testing timer_stop() API
 * and verify interrupt
 * @parameter[in]  :
 * 					- Interval
 * @parameter[out] :N/A
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_timer_002(char** argv, int argc)
{
	/**! Check command parameter and argument */
	if((1 != argc) || (NULL == argv))
	{
		log_printf("Error: Wrong number of parameter or Empty argument.\r\n");
	}

	/**! define pointer test timer and variable */
	struct timer_t *test_timer[5] = {NULL, NULL, NULL, NULL, NULL};
	uint32_t interval = 0;
	uint32_t time_out = 0;
	uint32_t temp1 = 0;
	uint32_t temp2 = 0;

	/**! Get interval to via argv[0] */
	interval = (uint32_t)strtol(argv[0], NULL, 10);

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initialize timer */
	timer_init();

	for(int i = 0; i < TIMER_MAX_NBR; i++)
	{
		time_out = 2000;
		/**! Create timer  */
		test_timer[i] = timer_create();

		/**! Verify create timer successful */
		qc_assert(test_timer[i]);

		/**! Configuration timer */
		test_timer[i]->interval = interval;
		test_timer[i]->param = test_timer;

		/**! Check timer_start API */
		qc_assert(!timer_start(test_timer[i]));
		qc_assert(TIMER_STARTED == test_timer[i]->state);

		/**! Wait time */
		while(--time_out);
		temp1 = test_timer[i]->time_expired;
		time_out = 1000;

		/**! Check timer_stop API */
		qc_assert(!timer_stop(test_timer[i]));
		qc_assert(TIMER_STOPPED == test_timer[i]->state);

		/**! Wait time */
		while(--time_out);
		temp2 = test_timer[i]->time_expired;

		/**! Verify timer stop */
		qc_assert(temp1 == temp2);

	}

	/**! Free memory timer */
	for(int i = 0; i < TIMER_MAX_NBR; i++)
	{
		timer_delete(&test_timer[i]);
	}

	/**! Do judgment */
	qc_report();

	return 0;
}

/**
 * @brief Test TIMER driver
 * @detail Test application for testing timer_start after call timer_stop API
 * and verify interrupt
 * @parameter[in]  :
 * 					- Interval
 * @parameter[out] :N/A
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_timer_003(char** argv, int argc)
{
	/**! Check command parameter and argument */
	if((1 != argc) || (NULL == argv))
	{
		log_printf("Error: Wrong number of parameter or Empty argument.\r\n");
	}

	/**! define pointer test timer and variable */
	struct timer_t *test_timer[5] = {NULL, NULL, NULL, NULL, NULL};
	uint32_t interval = 0;
	uint32_t time_out = 0;
	uint32_t temp1 = 0;
	uint32_t temp2 = 0;

	/**! Get interval to via argv[0] */
	interval = (uint32_t)strtol(argv[0], NULL, 10);

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initialize timer */
	timer_init();

	for(int i = 0; i < TIMER_MAX_NBR; i++)
	{
		time_out = 2000;
		/**! Create timer  */
		test_timer[i] = timer_create();

		/**! Verify create timer successful */
		qc_assert(test_timer[i]);

		/**! Configuration timer */
		test_timer[i]->interval = interval;
		test_timer[i]->param = test_timer;

		/**! Check timer_start API */
		qc_assert(!timer_start(test_timer[i]));
		qc_assert(TIMER_STARTED == test_timer[i]->state);

		/**! Wait time */
		while(--time_out);
		temp1 = test_timer[i]->time_expired;
		time_out = 1000;

		/**! Check timer_stop API */
		qc_assert(!timer_stop(test_timer[i]));
		qc_assert(TIMER_STOPPED == test_timer[i]->state);

		/**! Check timer_start API */
		qc_assert(!timer_start(test_timer[i]));
		qc_assert(TIMER_STARTED == test_timer[i]->state);

		/**! Wait time */
		while(--time_out);
		temp2 = test_timer[i]->time_expired;

		/**! Verify timer stop */
		qc_assert(temp1 != temp2);
	}

	/**! Free memory timer */
	for(int i = 0; i < TIMER_MAX_NBR; i++)
	{
		timer_delete(&test_timer[i]);
	}

	/**! Do judgment */
	qc_report();

	return 0;
}

/**
 * @brief Test TIMER driver
 * @detail Test application for testing mode auto reload
 * and verify interrupt
 * @parameter[in]  :
 * 					- Interval
 * 					- Auto reload
 * @parameter[out] :N/A
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_timer_004(char** argv, int argc)
{
	/**! Check command parameter and argument */
	if((2 != argc) || (NULL == argv))
	{
		log_printf("Error: Wrong number of parameter or Empty argument.\r\n");
	}

	/**! Define pointer test timer and variable */
	struct timer_t *test_timer[5] = {NULL, NULL, NULL, NULL, NULL};
	uint32_t interval = 0;
	uint8_t auto_reload = 0;
	uint32_t time_out = 0;
	uint32_t temp1 = 0;
	uint32_t temp2 = 0;

	/**! Get interval to via argv[0] */
	interval = (uint32_t)strtol(argv[0], NULL, 10);

	/**! Get auto reload to via argv[1] */
	auto_reload =(uint8_t)strtol(argv[1], NULL, 10);

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initialize timer */
	timer_init();

	for(int i = 0; i < TIMER_MAX_NBR; i++)
	{
		time_out = 2000;
		/**! Create timer  */
		test_timer[i] = timer_create();

		/**! Verify create timer successful */
		qc_assert(test_timer[i]);

		/**! Configuration timer */
		test_timer[i]->interval = interval;
		test_timer[i]->param = test_timer;
		test_timer[i]->autoreload = auto_reload;

		/**! Check timer_start API */
		qc_assert(!timer_start(test_timer[i]));
		qc_assert(TIMER_STARTED == test_timer[i]->state);

		if(ON == test_timer[i]->autoreload)
		{
			/**! Wait overflow timer */
			while(test_timer[i]->time_expired < test_timer[i]->interval);
			temp1 = test_timer[i]->time_expired;
			while(--time_out);
			temp2 = test_timer[i]->time_expired;

			/**! Verify mode auto reload do work */
			qc_assert(TIMER_STARTED == test_timer[i]->state);
			qc_assert(temp1 != temp2);
		}
		else
		{
			while(--time_out);
			qc_assert(test_timer[i]->time_expired);
		}
	}
	/**! Free memory timer */
	for(int i = 0; i < TIMER_MAX_NBR; i++)
	{
		timer_delete(&test_timer[i]);
	}

	/**! Do judgment */
	qc_report();

	return 0;
}
/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
