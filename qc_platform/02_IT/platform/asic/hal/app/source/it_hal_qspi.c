/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    it_hal_spi.c
 * @author  The LightCo
 * @version V1.0.1
 * @date    19-Apr, 2016
 * @brief   This file contains expand of the hal_spi driver
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
#include "it_hal_qspi.h"
#include "it_log_swapper.h"
#include "hal_gpio.h"
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported global variables -------------------------------------------------*/
/* Static functions ----------------------------------------------------------*/

/**
 * @brief To verify PP mode of QSPI
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @return 0	 : success
 *         Others: instruction failed
 * @details To verify PP mode of QSPI
 */
LOCAL int it_hal_qspi_001(char** argv, int argc);
/* Exported global variables -------------------------------------------------*/
LOCAL it_map_t it_qspi_tests_table[] = {
		{"QSPI_001",  it_hal_qspi_001},
		{"",  NULL}
};

/* Exported functions --------------------------------------------------------*/

/**
 * @brief QSPI module's testing handler
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @return 0	 : success
 *         Others: instruction failed
 * @details to QSPI module's testing handler
 */
int it_hal_qspi_handler(char** argv, int argc)
{
	/**< Check command counter >*/
	if (1 >= argc) return -1;
	if (NULL == argv) return -1;
	/**< Get command index, mean test case name >*/
	int index = handler_parser(*argv, it_qspi_tests_table);
	if (-1 != index)
	{
		return it_qspi_tests_table[index].handler(&argv[1], argc - 1);
	}
	else
		return index;
}
/* Exported functions declaration---------------------------------------------*/
/**
 * @brief To verify PP mode of QSPI
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @return 0	 : success
 *         Others: instruction failed
 * @details To verify PP mode of QSPI
 */
LOCAL int it_hal_qspi_001(char** argv, int argc)
{

	/**! Indicate QSPI test success */
	return 0;
}

/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
