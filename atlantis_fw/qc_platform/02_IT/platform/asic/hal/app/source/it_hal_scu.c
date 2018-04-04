/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    it_hal_scu.c
 * @author  The LightCo
 * @version V1.0
 * @date    13-May-2016
 * @brief   This file contains expand of the hal_scu driver
 *
 ******************************************************************************/

/******************************************************************************/
/**								Revision history
 * * 1.0.0	13-May-2016	Initial revision:
 */
/******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "it_hal_scu.h"
#include "it_log_swapper.h"
#include "board_config.h"
/* Private typedef -----------------------------------------------------------*/
typedef struct scu_map_reg
{
	__IO uint32_t CLOCK_DIV_0;
	__IO uint32_t CLOCK_OUT_0;
	__IO uint32_t CLOCK_TEST_0;
	__IO uint32_t CLOCK_DIV_1;
	__IO uint32_t CLOCK_OUT_1;
	__IO uint32_t CLOCK_TEST_1;
	__IO uint32_t CLOCK_DIV_2;
	__IO uint32_t CLOCK_OUT_2;
	__IO uint32_t CLOCK_TEST_2;
	__IO uint32_t CLOCK_SEL;
	__IO uint32_t REMAP;
	__IO uint32_t REG_SRAM;
	__IO uint32_t CLOCK_266;
	__IO uint32_t CLOCK_133;
	__IO uint32_t CLOCK_266_SOFT;
	__IO uint32_t SOFT_RESET_APB_0;
	__IO uint32_t SOFT_RESET_APB_1;
	__IO uint32_t WDT_266;
	__IO uint32_t WDT_APB_0;
	__IO uint32_t WDT_APB_1;
	__IO uint32_t REG_CACHES;
	__IO uint32_t SPIM;
	__IO uint32_t CAM_CLOCK;
	__IO uint32_t BOOT_SPI_0;
	__IO uint32_t BOOT_SPI_1;
	__IO uint32_t BOOT_SPI_2;
	__IO uint32_t BOOT_SPI_3;
	__IO uint32_t CAM_DIV_0;
	__IO uint32_t CAM_DIV_1;
	__IO uint32_t CAM_DIV_2;
	__IO uint32_t CAM_DIV_3;
	__IO uint32_t CAM_DIV_4;
	__IO uint32_t CAM_DIV_5;
	__IO uint32_t CAM_NUMBER_DIV_0;
	__IO uint32_t CAM_NUMBER_DIV_1;
	__IO uint32_t CAM_DIV_SEL;
	__IO uint32_t DDR_CONTROL0;
	__IO uint32_t DDR_CONTROL1;
	__IO uint32_t SUC_DEBUG;
	__IO uint32_t IP_DEBUG_SEL;
	__O uint32_t I2C_DEBUG_0_1;
	__O uint32_t I2C_DEBUG_2_3;
	__O uint32_t I2C_DEBUG_4_5;
	__O uint32_t I2C_DEBUG_6_7;
	__O uint32_t I2C_DEBUG_8_9;
	__O uint32_t I2C_DEBUG_10_11;
	__O uint32_t I2C_DEBUG_12_13;
	__O uint32_t I2C_DEBUG_14_15;
	__O uint32_t I2C_DEBUG_16_17;
	__O uint32_t I2C_DEBUG_18_S0;
	__O uint32_t I2CS1_DEBUG;
	__O uint32_t DDR_CONTROL_2;
	__O uint32_t DDR_CONTROL_3;
	__O uint32_t TIMER_ENABLE;

}scu_map_reg_t;
/* Private define ------------------------------------------------------------*/
#define SCU 						((scu_map_reg_t *)SCU_BASE)
/* Private macro -------------------------------------------------------------*/

/* Exported global varibles --------------------------------------------------*/

/* Static functions ----------------------------------------------------------*/

/**
 * @brief SCU module's testing
 * @detail to SCU module's testing
 * @param[in] argv	: N/A
 * @param[in] argc	: N/A
 * @return 0	 : success
 *         Others: instruction failed
 */
LOCAL int it_hal_scu_001(char** argv, int argc);

LOCAL it_map_t it_scu_tests_table[] =
{
		{"SCU_001", it_hal_scu_001},
		{"",  NULL}
};

/* Exported functions --------------------------------------------------------*/

/**
 * @brief SCU module's testing handler
 * @detail to SCU module's testing handler
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @return 0	 : success
 *         Others: instruction failed
 */
int it_hal_scu_handler(char** argv, int argc)
{
	/**< Get command index, mean test case name >*/
	int index = handler_parser(*argv, it_scu_tests_table);
	if (-1 != index)
	{
		return it_scu_tests_table[index].handler(&argv[1], argc - 1);
	}
	else
		return index;
}

/**
 * @brief SCU module's testing
 * @detail to SCU module's testing
 * @param[in] argv	: N/A
 * @param[in] argc	: N/A
 * @return 0	 : success
 *         Others: instruction failed
 */
LOCAL int it_hal_scu_001(char** argv, int argc)
{
	uint32_t* ptr = (uint32_t*)SCU;

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Print register SCU */
	for(ptr = (uint32_t*)SCU; ptr <= &SCU->TIMER_ENABLE; ptr++)
	{
		log_printf("Address: 0x%08x\r\n", ptr);
		log_printf("Data:    0x%08x\r\n", *ptr);
	}

	/**! Dummy assert and report */
	qc_assert(TRUE);

	/**! Dummy report */
	qc_report();

	return 0;

}

/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/

