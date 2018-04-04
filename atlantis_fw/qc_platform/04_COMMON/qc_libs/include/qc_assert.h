/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    qc_assert.h
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _QC_ASSERT_H_
#define _QC_ASSERT_H_
/* Includes ------------------------------------------------------------------*/
#include "qc_common.h"
/* Exported typedef ----------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
#define QC_NUM_OF_TEST_POINT_MAX	128
#define QC_JUDGMENT_LENTH           10
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
IMPORT uint8_t test_points[QC_NUM_OF_TEST_POINT_MAX];
IMPORT volatile uint8_t  test_top_id;
IMPORT volatile bool     test_overflow;
IMPORT char    	qc_judgment[QC_JUDGMENT_LENTH];
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/**
 * @brief to check logical expression
 * @detail to check logical expression is correct or not.
 * @param[in] exp logical expression
 * @param[out] qc_test_points_table		test points table
 * @param[out] test_point_index			current test point index
 * @param[out] qc_test_overflow			Overflow flag
 * @return NA
 */
IMPORT void qc_assert(bool exp);

/**
 * @brief to reset situation to default
 * @detail reset all test point to TRUE (PASSED) and current test point is
 *         zero.
 * @param[out] qc_test_points_table		test points table
 * @param[out] test_point_index			current test point index
 * @param[out] qc_test_overflow			Overflow flag
 * @return NA
 */
IMPORT void qc_assert_reset(void);

/**
 * @brief do judgment for test sequence
 * @detail verify all test point to do judgment
 * @param[in] NA
 * @param[out] qc_judgment	Judgment result
 * @return NA
 */
IMPORT void qc_report(void);

/**
 * @brief To get param asserted status
 * @detail To get param asserted status
 * @param[in] NA
 * @param[out] NA
 * @return TRUE	: param is asserted
 *         FALSE: param is not asserted
 */
IMPORT bool qc_assert_status(void);

/**
 * @brief Assert swapper for development parameter verification
 * @detail To do swapper for development parameters verification
 * @param[in] NA
 * @param[out]
 * @return NA
 */
IMPORT void assert_failed(uint8_t *func, uint32_t line);

#endif /* _QC_ASSERT_H_ */

/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
