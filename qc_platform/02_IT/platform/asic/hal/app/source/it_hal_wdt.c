/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    it_hal_wdt.c
 * @author  The LightCo
 * @version V1.0.1
 * @date    19-Apr-2016
 * @brief   This file contains expand of the hal_wdt driver
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
#include "it_hal_wdt.h"
#include "it_log_swapper.h"
#include "board_config.h"
#include "hal_gpio.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Exported global varibles --------------------------------------------------*/
/* Static functions ----------------------------------------------------------*/
/**
 * @brief initialize and start WatchDog
 * @detail to test hal_wdt_init and hal_wdt_start APIs
 * @param[in] time_out
 * @param[out] NA
 * @return NA
 */
LOCAL int it_hal_wdt_001(char** argv, int argc);

/**
 * @brief Stop WatchDog after it was started 
 * @detail to test hal_wdt_stop API
 * @param[in] time_out
 * @param[out] NA
 * @return NA
 */
LOCAL int it_hal_wdt_002(char** argv, int argc);

/**
 * @brief test performance API
 * @detail to test performance API
 * @param[in] time_out
 * @param[out] NA
 * @return NA
 */
LOCAL int it_hal_wdt_003(char** argv, int argc);

/**
 * @brief delay function
 * @detail add delay time to support for testing APIs of timer module
 * @param[in] time: delay time (ms)
 * @param[out] NA
 */
LOCAL void delay_ms(__IO uint32_t time)
{
	__IO uint32_t i = time * (BOARD_PCLOCK / 1000)/6;
	while (i--);
}

/**
 * @brief Watchdog callback handler
 * @detail To be called by watchdog ISR()
 * @param[in] NA
 * @param[out] NA
 * @return NA
 */
LOCAL void it_wdt_callback(void);

/* Private variables ---------------------------------------------------------*/

/** 
 *  Watchdog test cases table
 */
LOCAL it_map_t it_wdt_tests_table[] = {
		{"WDT_001", it_hal_wdt_001},
		{"WDT_002", it_hal_wdt_002},
		{"WDT_003", it_hal_wdt_003},
		{"",  NULL}
};

/**
 *  Watchdog callback flags
 */
volatile bool it_wdt_reset_enable 	= FALSE;
volatile bool it_wdt_timeout_enable = FALSE;

/* Exported functions --------------------------------------------------------*/

/**
 * @brief WDT module's testing handler
 * @detail to WDT module's testing handler
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @return 0	 : success
 *         Others: instruction failed
 */
int it_hal_wdt_handler(char** argv, int argc)
{
	/**< Check command counter >*/
	if (0 == argc) return -1;
	if (NULL == argv) return -1;
	
	/**< Get command index, mean test case name >*/
	int index = handler_parser(*argv, it_wdt_tests_table);
	if (-1 != index)
	{
		return it_wdt_tests_table[index].handler(&argv[1], argc - 1);
	}
	else
		return index;
}

/**
 * @brief initialize and start WatchDog
 * @detail To verify that watchdog timer module is initialized and enabled after
 *         call hal_wdt_init and hal_wdt_start APIs. System reset occurs because
 *         watchdog is not reseted.
 * @param[in] time_out
 * @param[out] NA
 * @return NA
 */
LOCAL int it_hal_wdt_001(char** argv, int argc)
{
	if (1 != argc)
	{
		log_printf("Error: Wrong number of parameters\r\n");
		return -1;
	}

	if (NULL == argv)
	{
		log_printf("Error: Empty argument\r\n");
		return -1;
	}

	/**! Configuration */
	uint32_t time_out = 0;

	/**! Get timeout via argv[0] */
	time_out = (uint32_t)(strtol(argv[0], NULL, 10));

	/**! Step 1: Reset all test point to default */
	qc_assert_reset();

	/**! Step 2: Call hal_wdt_init API with indicated timeout value */
	hal_wdt_init(time_out, it_wdt_callback, 1);

	/**! Step 3: Enable watchdog timeout test flag to help do judgment this 
	 *  test case at ISR()
	 */
	it_wdt_timeout_enable = TRUE;
	
	/**! Step 4: Call hal_wdt_start API */
	hal_wdt_start();

	/**! Step 5: Delay a period of time that is greater than timeout value to 
	 *  let watchdog timeout event occur
	 */
	delay_ms((time_out*1000) << 1);

	/**! Step 6: Assert FALSE and report */
	qc_assert(FALSE);
	
	/**! Do Judgment */
	qc_report();

	return 0;
}

/**
 * @brief Stop WatchDog after it was started 
 * @detail To verify that timer module is initialized and enabled after call 
  *        hal_timer_init and hal_timer_start APIs and system reset does not 
  *        appears if watchdog is reset before timeout even occurs.
 * @param[in] time_out
 * @param[out] NA
 * @return NA
 */
LOCAL int it_hal_wdt_002(char** argv, int argc)
{
	if (1 != argc)
	{
		log_printf("Error: Wrong number of parameters\r\n");
		return -1;
	}

	if (NULL == argv)
	{
		log_printf("Error: Empty argument\r\n");
		return -1;
	}

	/**! Configuration */
	uint32_t time_out = 0;

	/**! Get timeout via argv[0] */
	time_out = (uint32_t)(strtol(argv[0], NULL, 10));

	/**! Step 1: Reset all test point to default */
	qc_assert_reset();

	/**! Step 2: Call hal_wdt_init API with indicated timeout value */
	hal_wdt_init(time_out, it_wdt_callback, 1);

	/**! Step 3: Set watchdog timeout test flag to FALSE. */
	it_wdt_timeout_enable = FALSE;

	/**! Step 4: Call hal_wdt_start API */
	hal_wdt_start();

	/**! Step 5: Delay a period of time that smaller that timeout value. */
	delay_ms((time_out - 1)*1000);

	/**! Step 6: Call hal_wdt_kick_dog() to reset watchdog. */
	hal_wdt_kick_dog();
	
	/**! Step 7: Delay a period of time that smaller that timeout value and 
   *  greater than a haft of indicated timeout value.
   */
	delay_ms(((time_out>>1) + 1)*1000);

	/**! Step 8: Invoke hal_wdt_stop() to stop WDT. */
	hal_wdt_stop();
	
	/**! Do Judgment */
	qc_report();

	return 0;
}

/**
 * @brief test performance API
 * @detail to test performance API
 * @param[in] time_out
 * @param[out] NA
 * @return NA
 */
LOCAL int it_hal_wdt_003(char** argv, int argc)
{
	if (2 != argc)
	{
		log_printf("Error: Wrong number of parameters\r\n");
		return -1;
	}

	if (NULL == argv)
	{
		log_printf("Error: Empty argument\r\n");
		return -1;
	}

	/**! Configuration */
	uint32_t time_out = 0;
	hal_gpio_t gpio = {0, 0, 1};
	uint8_t select;
	/**! Get timeout via argv[0] */
	time_out = (uint32_t)strtol(argv[0], NULL, 10);
	/**! Get variable select API via argv[1] */
	select = (uint8_t)strtol(argv[1], NULL, 10);

	/**! Initialize GPIO */
	hal_gpio_init(&gpio);

	/**! Step 1: Reset all test point to default */
	qc_assert_reset();

	if(0 == select)
	{
		hal_gpio_set_high(&gpio);
	}
	/**! Step 2: Call hal_wdt_init API with indicated timeout value */
	hal_wdt_init(time_out, it_wdt_callback, 1);
	if(0 == select)
	{
		hal_gpio_set_low(&gpio);
	}
	/**! Step 3: Set watchdog timeout test flag to FALSE. */
	it_wdt_timeout_enable = FALSE;

	if(1 == select)
	{
		hal_gpio_set_high(&gpio);
	}
	/**! Step 4: Call hal_wdt_start API */
	hal_wdt_start();
	if(1 == select)
	{
		hal_gpio_set_low(&gpio);
	}
	/**! Step 5: Delay a period of time that smaller that timeout value. */
	delay_ms((time_out - 1)*1000);

	if(2 == select)
	{
		hal_gpio_set_high(&gpio);
	}
	/**! Step 6: Call hal_wdt_kick_dog() to reset watchdog. */
	hal_wdt_kick_dog();
	if(2 == select)
	{
		hal_gpio_set_low(&gpio);
	}
	/**! Step 7: Delay a period of time that smaller that timeout value and
   *  greater than a haft of indicated timeout value.
   */
	delay_ms(((time_out>>1) + 1)*1000);

	if(3 == select)
	{
		hal_gpio_set_high(&gpio);
	}
	/**! Step 8: Invoke hal_wdt_stop() to stop WDT. */
	hal_wdt_stop();
	if(3 == select)
	{
		hal_gpio_set_low(&gpio);
	}
	/**! Do Judgment */
	qc_report();

	return 0;
}

/**
 * @brief Watchdog callback handler
 * @detail To be called by watchdog ISR()
 * @param[in] NA
 * @param[out] NA
 * @return NA
 */
LOCAL void it_wdt_callback(void)
{
	if (it_wdt_timeout_enable)
	{
		qc_assert(TRUE);
		qc_report();
		log_printf("Auto result: ");
		log_printf(qc_judgment);
		log_printf("\r\n");
	}
	else
	{
		qc_assert(FALSE);
		qc_report();
		log_printf("Auto result: ");
		log_printf(qc_judgment);
		log_printf("\r\n");
	}
}
 

/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
