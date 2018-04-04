/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    qc_assert.c
 * @author  The LightCo
 * @version V1.0.1
 * @date    19-Apr-2016
 * @brief   This file contains expand of the hal_timer driver
 *
 ******************************************************************************/

/******************************************************************************/
/**								Revision history
 * * 1.0.0	20-Feb-2016	Initial revision:
 *                      - Infrastructure.
 * * 1.0.1	19-Apr-2016 Test HAL baseline 1.0.0
 */
/******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "qc_assert.h"
#include "it_log_swapper.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define TEST_FAILED "FAILED"
#define TEST_PASSED "PASSED"
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Public variables ----------------------------------------------------------*/
uint8_t test_points[QC_NUM_OF_TEST_POINT_MAX] = {0x00};
volatile uint8_t  test_top_id = ZERO;
volatile bool     test_overflow = FALSE;
char     qc_judgment[QC_JUDGMENT_LENTH] = {0x00};
LOCAL volatile bool param_asserted = FALSE;
/* Static functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
 * @brief to check logical expression
 * @detail to check logical expression is correct or not.
 * @param[in] exp logical expression
 * @param[out] qc_test_points_table		test points table
 * @param[out] test_point_index			current test point index
 * @param[out] qc_test_overflow			Overflow flag
 * @return NA
 */
void qc_assert(bool exp)
{
	/**! To avoid system crashing when test overflow appears */
	if (TRUE == test_overflow)
	{
		/**! Assign maximum test point is FAILED to notify tester re-check */
		test_points[test_top_id-1] = FALSE;
		return;
	}

	/**! Judgment */
	test_points[test_top_id++] = exp;

	/**! Overflow check */
	if (QC_NUM_OF_TEST_POINT_MAX == test_top_id)
	{
		/**! Overflow situation appears */
		test_overflow = TRUE;
	}
}

/**
 * @brief to reset situation to default
 * @detail reset all test point to TRUE (PASSED) and current test point is
 *         zero.
 * @param[out] qc_test_points_table		test points table
 * @param[out] test_point_index			current test point index
 * @param[out] qc_test_overflow			Overflow flag
 * @return NA
 */
void qc_assert_reset(void)
{
	uint8_t i = 0;

	/**! Reset test results table */
	for (i = 0; i < QC_NUM_OF_TEST_POINT_MAX; i++)
	{
		test_points[i] = TRUE;
	}
	
	/**! Reset all test point */
	test_top_id = ZERO;
	test_overflow = FALSE;

	for (int i = 0; i < QC_JUDGMENT_LENTH; i++)
	{
		qc_judgment[i] = 0x00;
	}

	/**! Reset param asserted status */
	param_asserted = FALSE;
}

/**
 * @brief do judgment for test sequence
 * @detail verify all test point to do judgment
 * @param[in] NA
 * @param[out] qc_judgment	Judgment result
 * @return NA
 */
void qc_report(void)
{
	bool result = TRUE;
	int i = 0;

	/**! Check test overflow */
	if (TRUE == test_overflow)
	{
		log_printf("Test overflow.\r\n");
		strncpy(qc_judgment, "FAILED", strlen("FAILED"));
		return;
	}

	/**! Verify all test points */
	for (i = 0; i < test_top_id; i++)
	{
		if (FALSE == test_points[i])
		{
			result = FALSE;
			break;
		}
	}

	/**! Check result */
	if (FALSE == result)
	{
		/**! There is a FAILED point */
		strncpy(qc_judgment, TEST_FAILED, strlen(TEST_FAILED));

		/**! Print failed info */
		log_printf("Test results:\r\n");
		log_printf("Total test points: %u \r\n", test_top_id);
		log_printf("First fail test point: %03u\r\n", i);
		for (i = 0; i < test_top_id; i++)
		{
			/**! Print test point */
			log_printf("TP %03u: %01u\r\n", i, test_points[i]);
		}
	}
	else
	{
		/**! All test points are PASSED */
		strncpy(qc_judgment, TEST_PASSED, strlen(TEST_PASSED));
	}
}

/**
 * @brief To get param asserted status
 * @detail To get param asserted status
 * @param[in] NA
 * @param[out] NA
 * @return TRUE	: param is asserted
 *         FALSE: param is not asserted
 */
bool qc_assert_status(void)
{
	return param_asserted;
}

/**
 * @brief Assert swapper for development parameter verification
 * @detail To do swapper for development parameters verification
 * @param[in] NA
 * @param[out] qc_judgment	Judgment result
 * @return NA
 */
void assert_failed(uint8_t *func, uint32_t line)
{
	param_asserted = TRUE;
}

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_malloc error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void *malloc_failed(uint8_t *func, uint32_t line)
{
	/* Infinite loop */
	printf("[%s: %d] %s !\r\n", func, (unsigned int)line, __FUNCTION__);
	/* TODO: Add disable interrupt here */
	while (1);
	return NULL;
}
/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
