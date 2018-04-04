/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    it_drv_vcm.c
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
#include "it_drv_vcm.h"
#include "qc_common.h"

/* Private define ------------------------------------------------------------*/
#define VCM_TOLERANCE					1

/* Static functions ----------------------------------------------------------*/

/**
 * @brief Test VCM driver
 * @detail Test application for testing drv_vcm_init API
 * @parameter[in]  :
 * 				- Channel
 * @parameter[out] :NA
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_vcm_001(char** argv, int argc);

/**
 * @brief Test VCM driver
 * @detail Test application for testing drv_vcm_move_to_hall API
 * @parameter[in]  :
 * 				- Channel
 * 				- Hall position
 * @parameter[out] :NA
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_vcm_002(char** argv, int argc);

/**
 * @brief Test VCM driver
 * @detail Test application for testing drv_vcm_focus API
 * @parameter[in]  :
 * 				- Channel
 * 				- Distance position
 * @parameter[out] :NA
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_vcm_003(char** argv, int argc);

/**
 * @brief Test VCM driver
 * @detail Test application for testing drv_vcm_deinit API
 * @parameter[in]  :
 * 				- Channel
 * 				- Distance position
 * @parameter[out] :NA
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_vcm_004(char** argv, int argc);

LOCAL it_map_t it_vcm_tests_table[] = {
		{"VCM_001", it_drv_vcm_001},
		{"VCM_002", it_drv_vcm_002},
		{"VCM_003", it_drv_vcm_003},
		{"VCM_004", it_drv_vcm_004},
		{"",  NULL}
};

/* Exported functions ------------------------------------------------------- */
/**
 * @brief VCM module's testing handler
 * @detail to VCM module's testing handler
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @return 0	 : success
 *         Others: instruction failed
 */
int it_drv_vcm_handler(char** argv, int argc)
{
	/**< Check command counter >*/
	if (1 >= argc) return -1;
	if (NULL == argv) return -1;
	/**< Get command index, mean test case name >*/
	int index = handler_parser(*argv, it_vcm_tests_table);
	if (-1 != index)
	{
		return it_vcm_tests_table[index].handler(&argv[1], argc - 1);
	}
	else
		return index;
}

/**
 * @brief Test VCM driver
 * @detail Test application for testing drv_vcm_init API
 * @parameter[in]  :
 * 				- Channel
 * @parameter[out] :NA
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_vcm_001(char** argv, int argc)
{
	/**! Check command parameter and argument*/
	if((1 != argc) || (NULL == argv))
	{
		log_printf("Error: Wrong number of parameter or Empty argument.\r\n");
	}

	uint8_t channel = 0;

	/**! Get channel via argv[0] */
	channel = (uint8_t)strtol(argv[0], NULL, 10);

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Check API drv_vcm_init */
	qc_assert(VCM_OK == drv_vcm_init(channel));
	
	/**! Do judgment */
	qc_report();
	return VCM_OK;	
}

/**
 * @brief Test VCM driver
 * @detail Test application for testing drv_vcm_move_to_hall API
 * @parameter[in]  :
 * 				- Channel
 * 				- Hall position
 * @parameter[out] :NA
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_vcm_002(char** argv, int argc)
{
	/**! Check command parameter and argument*/
	if((2 != argc) || (NULL == argv))
	{
		log_printf("Error: Wrong number of parameter or Empty argument.\r\n");
	}

	uint8_t channel = 0;
	int16_t temp1 = 0;
	int16_t temp2 = 0;
	int16_t hall   = 0;

	/**! Get channel via argv[0] */
	channel = (uint8_t)strtol(argv[0], NULL, 10);

	/**! Get hall value via argv[1] !*/
	hall = (uint16_t)strtol(argv[1], NULL, 10);

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Check API drv_vcm_init */
	qc_assert(VCM_OK == drv_vcm_init(channel));

	/**! Check API drv_vcm_get_hall and get position entry */
	qc_assert(VCM_OK == drv_vcm_get_hall(channel, &temp1));

	/**! Check API drv_vcm_move_to_hall and move to hall position */
	qc_assert(VCM_OK == drv_vcm_move_to_hall(channel, hall));

	/**! Check API drv_vcm_get_hall and get position current */
	qc_assert(VCM_OK == drv_vcm_get_hall(channel, &temp2));

	/**! Check current position*/
	qc_assert(abs((temp1 + hall) - temp2) <= VCM_TOLERANCE);

	/**! Do judgment */
	qc_report();

	return VCM_OK;
}

/**
 * @brief Test VCM driver
 * @detail Test application for testing drv_vcm_focus API
 * @parameter[in]  :
 * 				- Channel
 * 				- Distance position
 * @parameter[out] :NA
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_vcm_003(char** argv, int argc)
{
	/**! Check command parameter and argument */
	if((2 != argc) || (NULL == argv))
	{
		log_printf("Error: Wrong number of parameter or Empty argument.\r\n");
	}

	uint8_t channel;
	uint32_t distance;

	/**! Get channel via argv[0] */
	channel = (uint8_t)strtol(argv[0], NULL, 10);

	/**! Get value distance via argv[1] */
	distance = (uint32_t)strtol(argv[1], NULL, 10);

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Check API drv_vcm_init */
	qc_assert(VCM_OK == drv_vcm_init(channel));

	/**! Check API drv_vcm_focus */
	qc_assert(VCM_OK == drv_vcm_focus(channel, distance));

	/**! Do judgment */
	qc_report();

	return VCM_OK;
}

/**
 * @brief Test VCM driver
 * @detail Test application for testing drv_vcm_deinit API
 * @parameter[in]  :
 * 				- Channel
 * 				- Distance position
 * @parameter[out] :NA
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_vcm_004(char** argv, int argc)
{
	/**! Check command parameter and argument */
	if((1 != argc) || (NULL == argv))
	{
		log_printf("Error: Wrong number of parameter or Empty argument.\r\n");
	}

	uint8_t channel = 0;
	uint16_t hall   = 50;

	/**! Get channel via argv[0] */
	channel = (uint8_t)strtol(argv[0], NULL, 10);

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Check API drv_vcm_init */
	qc_assert(VCM_OK == drv_vcm_init(channel));

	/**! Check API drv_vcm_move_to_hall */
	qc_assert(VCM_OK == drv_vcm_move_to_hall(channel, hall));

	/**! Check API drv_vcm_deinit */
	qc_assert(VCM_OK == drv_vcm_deinit(channel));

	hall = -50;
	/**! Check API drv_vcm_move_to_hall when Call drv_vcm_deinit */
	qc_assert(VCM_OK != drv_vcm_move_to_hall(channel, hall));

	/**! Do judgment */
	qc_report();
	return VCM_OK;
}

/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
