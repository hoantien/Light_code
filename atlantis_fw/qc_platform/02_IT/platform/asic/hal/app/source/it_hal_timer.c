/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    it_hal_timer.c
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
#include "it_hal_timer.h"
#include "it_log_swapper.h"
#include "hal_gpio.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Exported global varibles --------------------------------------------------*/
LOCAL volatile uint32_t timer_irq_counter = 0;
LOCAL volatile uint32_t timer_irq_counter_temp = 0;
LOCAL volatile uint32_t count1 = 0,count2 = 0;
LOCAL volatile bool timer_irq_enable = 0;
/* Static functions ----------------------------------------------------------*/

/**
 * @brief Initialize and enable timer module
 * @detail - To verify that timer module is initialized and enabled
 *           after call hal_timer_init and hal_timer_start APIs
 * @param[in] argv	: arguments list (Params=0,1000,0)
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 0	 : success
 *         Others: instruction failed
 */
LOCAL int it_hal_timer_001(char** argv, int argc);

/**
 * @brief Disable timer module
 * @detail To verify that timer module is disabled
 *         after call hal_timer_stop API
 * @param[in] argv	: arguments list (Params=0,1000,0)
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 0	 : success
 *         Others: instruction failed
 */
LOCAL int it_hal_timer_002(char** argv, int argc);

/**
 * @brief Updates timer period
 * @detail To verify that timer module is updated timer period
 *         after call hal_timer_update_period API
 * @param[in] argv	: arguments list (Params=0,1000,1)
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 0	 : success
 *         Others: instruction failed
 */
LOCAL int it_hal_timer_003(char** argv, int argc);

/**
 * @brief testing hal_timer_update_period API
 * @detail To verify that timer module is updated timer period
 *         after call hal_timer_update_period  API
 * @param[in] argv	: arguments list (Params=7,1000,0)
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 0	 : success
 *         Others: instruction failed
 */
LOCAL int it_hal_timer_004(char** argv, int argc);

/**
 * @brief testing hal_timer_init API
 * @detail To verify that timer module report error in case
 *       the requested channel is invalid and return without any action.
 * @param[in] argv	: arguments list (Params=8,1000,0)
 * @param[in] argc	: argument counter
 * @return 0	 : success
 *         Others: instruction failed
 */
LOCAL int it_hal_timer_005(char** argv, int argc);

/**
 * @brief testing performance API
 * @detail To verify that timer module performance
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @return 0	 : success
 *         Others: instruction failed
 */
LOCAL int it_hal_timer_006(char** argv, int argc);
/**
 * @brief timer callback function
 * @detail timer callback function
 * @param[in] NA
 * @param[out] timer_irq_counter
 */
LOCAL void timer_irq(void *param)
{
	timer_irq_counter++;
}

/**
 * @brief delay function
 * @detail add delay time to support for testing APIs of timer module
 * @param[in] time: delay time (ms)
 * @param[out] NA
 */
LOCAL void delay_ms(__IO uint32_t time)
{
	_delay_ms(time);
}

/* Private variables ---------------------------------------------------------*/
LOCAL it_map_t it_timer_tests_table[] = {
		{"TIMER_001", it_hal_timer_001},
		{"TIMER_002", it_hal_timer_002},
		{"TIMER_003", it_hal_timer_003},
		{"TIMER_004", it_hal_timer_004},
		{"TIMER_005", it_hal_timer_005},
		{"TIMER_006", it_hal_timer_006},
		{"",  NULL}
};
/* Exported functions --------------------------------------------------------*/

/**
 * @brief TIMER module's testing handler
 * @detail to TIMER module's testing handler
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @return 0	 : success
 *         Others: instruction failed
 */
int it_hal_timer_handler(char** argv, int argc)
{
	/**< Check command counter >*/
	if (2 > argc) return -1;
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
 * @brief Initialize and enable timer module
 * @detail - To verify that timer module is initialized and enabled
 *           after call hal_timer_init and hal_timer_start APIs
 * @param[in] argv	: arguments list (Params=0,1000,0)
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 0	 : success
 *         Others: instruction failed
 */
LOCAL int it_hal_timer_001(char** argv, int argc)
{
	if (3 != argc)
	{
		console_putstring("Error: Wrong number of parameters\r\n");
		return -1;
	}
	if (NULL == argv)
	{
		console_putstring("Error: Empty argument\r\n");
		return -1;
	}

	/**! Configuration */
	hal_timer_t tim = {HAL_TIM_CH1, 1000, (void*)0};

	/**! Get channel ID via argv[0] */
	tim.chid = strtol(argv[0], NULL, 10);
	/** Check value channel */
	if(HAL_TIM_MAX <= tim.chid)
	{
		console_putstring("Error: Do not support channel\r\n");
		return -1;
	}
	/**! Get period via argv[1] */
	tim.period = (uint32_t)strtol(argv[1], NULL, 10);

	timer_irq_enable = strtol(argv[2], NULL, 10);

	/**! Check value interrupt */
	if(1 < timer_irq_enable)
	{
		console_putstring("Error: Interrupt Invalid\r\n");
		return -1;
	}

	/**! Callback enable flag */
	if (1 == timer_irq_enable)
	{
		tim.callback_handler = timer_irq;
	}
	timer_irq_counter = 0;
	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initialize timer module */
	qc_assert(HAL_TIM_OK == hal_timer_init(&tim));

	/**! Enable timer module */
	qc_assert(HAL_TIM_OK == hal_timer_start(tim.chid));

	delay_ms(5000);

	/**! Check interrupt timer */
	if (1 == timer_irq_enable)
	{
		qc_assert(0 != timer_irq_counter);
	}
	else
	{
		qc_assert(0 == timer_irq_counter);
	}

	/**! Do judgment */
	qc_report();

	return (int)HAL_TIM_OK;
}

/**
 * @brief Disable timer module
 * @detail To verify that timer module is disabled
 *         after call hal_timer_stop API
 * @param[in] argv	: arguments list (Params=0,1000,0)
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 0	 : success
 *         Others: instruction failed
 */
LOCAL int it_hal_timer_002(char** argv, int argc)
{
	if (3 != argc)
	{
		console_putstring("Error: Wrong number of parameters\r\n");
		return -1;
	}
	if (NULL == argv)
	{
		console_putstring("Error: Empty argument\r\n");
		return -1;
	}

	/**! Configuration */
	hal_timer_t tim = {HAL_TIM_CH1, 1000, (void*)0};

	/**! Get channel ID via argv[0] */
	tim.chid = (hal_timer_channel_t)strtol(argv[0], NULL, 10);
	/** Check value channel */
	if(HAL_TIM_MAX <= tim.chid)
	{
		console_putstring("Error: Do not support channel\r\n");
		return -1;
	}

	/**! Get period via argv[1] */
	tim.period = (uint32_t)strtol(argv[1], NULL, 10);

	timer_irq_enable = strtol(argv[2], NULL, 10);

	if(1 < timer_irq_enable)
	{
		console_putstring("Interrupt Invalid\r\n");
		return -1;
	}
	/**! Check callback enable flag */
	if (1 == timer_irq_enable)
	{
		tim.callback_handler = timer_irq;
	}
	timer_irq_counter = 0;
	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initialize timer module */
	qc_assert(HAL_TIM_OK == hal_timer_init(&tim));

	/**! Enable timer module */
	qc_assert(HAL_TIM_OK == hal_timer_start(tim.chid));

	delay_ms(5000);
	/**! Disable timer module */
	qc_assert(HAL_TIM_OK == hal_timer_stop(tim.chid));

	if (1 == timer_irq_enable)
	{
		/**! Check stop timer */
		timer_irq_counter_temp = timer_irq_counter;
		delay_ms(2000);
		qc_assert(timer_irq_counter_temp == timer_irq_counter);
	}
	else
	{
		qc_assert(0 == timer_irq_counter);
	}

	/**! Do judgment */
	qc_report();
	return (int)HAL_TIM_OK;
}

/**
 * @brief Updates timer period
 * @details To verify that timer module is updated timer period
 *         after call hal_timer_update_period API
 * @param[in] argv	: arguments list (Params=0,1000,2000,1)
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 0	 : success
 *         Others: instruction failed
 */
LOCAL int it_hal_timer_003(char** argv, int argc)
{
	if (4 != argc)
	{
		console_putstring("Error: Wrong number of parameters\r\n");
		return -1;
	}
	if (NULL == argv)
	{
		console_putstring("Error: Empty argument\r\n");
		return -1;
	}

	/**! Configuration */
	hal_timer_t tim = {HAL_TIM_CH1, 1000, (void*)0};

	/**! Get channel ID via argv[0] */
	tim.chid = (hal_timer_channel_t)strtol(argv[0], NULL, 10);

	/**! Get period 1 via argv[1] */
	tim.period = (uint32_t)strtol(argv[1], NULL, 10);

	/**! Get interrupt timer */
	timer_irq_enable = strtol(argv[3], NULL, 10);
	if (1 != timer_irq_enable)
	{
		console_putstring("Error: Interrupt Invalid\r\n");
		return -1;
	}

	/**! Callback enable flag */
	tim.callback_handler = timer_irq;
	timer_irq_counter = 0;

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initialize timer module */
	qc_assert (HAL_TIM_OK == hal_timer_init(&tim));

	/**! Enable timer module */
	qc_assert(HAL_TIM_OK == hal_timer_start(tim.chid));

	/**! Disable timer module */
	delay_ms(3000);
	qc_assert(HAL_TIM_OK == hal_timer_stop(tim.chid));
	count1 = timer_irq_counter;

	/**! Updates timer period */
	tim.period = strtol(argv[2], NULL, 10);
	timer_irq_counter = 0;

	/** Check updates timer period */
	qc_assert (HAL_TIM_OK == hal_timer_update_period(&tim));

	/**! Restart timer module */
	qc_assert(HAL_TIM_OK == hal_timer_start(tim.chid));

	/**! Disable timer module */
	delay_ms(3000);
	qc_assert (HAL_TIM_OK == hal_timer_stop(tim.chid));
	count2 = timer_irq_counter;
	/**! Difference value count1 and count 2 */
	qc_assert(count1 != count2);

	/**! Do judgment */
	qc_report();
	return (int)HAL_TIM_OK;
}

/**
 * @brief testing hal_timer_update_period API
 * @details To verify that timer module is updated timer period
 *         after call hal_timer_update_period  API
 * @param[in] argv	: arguments list (Params=7,1000,0)
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 0	 : success
 *         Others: instruction failed
 */
LOCAL int it_hal_timer_004(char** argv, int argc)
{
	if (3 != argc)
	{
		console_putstring("Error: Wrong number of parameters\r\n");
		return -1;
	}
	if (NULL == argv)
	{
		console_putstring("Error: Empty argument\r\n");
		return -1;
	}

	/**! Configuration */
	hal_timer_t tim = {HAL_TIM_CH1, 1000, (void*)0};

	/**! Get channel ID via argv[0] */
	tim.chid = (hal_timer_channel_t)strtol(argv[0], NULL, 10);

	/** Check value channel */
	if (HAL_TIM_MAX <= tim.chid)
	{
		console_putstring("Error: Do not support channel\r\n");
		return -1;
	}

	/**! Get period via argv[1] */
	tim.period = (uint32_t)(strtol(argv[1], NULL, 10));

	/**! Check callback enable flag */
	timer_irq_enable = strtol(argv[2], NULL, 10);

	/** Check value interrupt */
	if(1 < timer_irq_enable)
	{
		console_putstring("Error: Interrupt Invalid\r\n");
		return -1;
	}

	if (1 == timer_irq_enable)
	{
		tim.callback_handler = timer_irq;
	}
	timer_irq_counter = 0;

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initialize timer module */
	qc_assert(HAL_TIM_OK == hal_timer_init(&tim));

	/**! Enable timer module */
	qc_assert(HAL_TIM_OK == hal_timer_start(tim.chid));

	/**! Disable timer module */
	delay_ms(3000);
	qc_assert(HAL_TIM_OK == hal_timer_stop(tim.chid));

	/**! Updates timer period */
	tim.period = 5000;
	qc_assert(HAL_TIM_OK == hal_timer_update_period(&tim));

	if (1 == timer_irq_enable)
	{
		qc_assert(0 != timer_irq_counter);
	}
	else
	{
		qc_assert(0 == timer_irq_counter);
	}

	/**! Do judgment */
	qc_report();
	return (int)HAL_TIM_OK;
}

/**
 * @brief testing hal_timer_init API
 * @details To verify that timer module report error in case
 *       the requested channel is invalid and return without any action.
 * @param[in] argv	: arguments list (Params=8,1000,0)
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 0	 : success
 *         Others: instruction failed
 */
LOCAL int it_hal_timer_005(char** argv, int argc)
{
	if (2 != argc)
	{
		console_putstring("Error: Wrong number of parameters\r\n");
		return -1;
	}
	if (NULL == argv)
	{
		console_putstring("Error: Empty argument\r\n");
		return -1;
	}

	/**! Configuration */
	hal_timer_t tim = {HAL_TIM_CH1, 1000, (void*)0};

	/**! Get channel ID via argv[0] */
	tim.chid = (hal_timer_channel_t)strtol(argv[0], NULL, 10);

	/**! Get period via argv[1] */
	tim.period = (uint32_t)(strtol(argv[1], NULL, 10));

	/** Check value interrupt */
	if(1 < timer_irq_enable)
	{
		console_putstring("Error: Interrupt Invalid\r\n");
		return -1;
	}

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initialize timer module */
	qc_assert(HAL_TIM_OK != hal_timer_init(&tim));

	/**! Do judgment */
	qc_report();

	return (int)HAL_TIM_OK;
}

/**
 * @brief testing performance API
 * @details To verify that timer module performance
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @return 0	 : success
 *         Others: instruction failed
 */
LOCAL int it_hal_timer_006(char** argv, int argc)
{
	if (5 != argc)
	{
		console_putstring("Error: Wrong number of parameters\r\n");
		return -1;
	}
	if (NULL == argv)
	{
		console_putstring("Error: Empty argument\r\n");
		return -1;
	}

	/**! Configuration */
	hal_timer_t tim = {HAL_TIM_CH1, 1000, (void*)0};
	hal_gpio_t gpio = {0, 0, 1};
	uint8_t select = 0;

	/**! Get channel ID via argv[0] */
	tim.chid = (hal_timer_channel_t)strtol(argv[0], NULL, 10);

	/**! Get period 1 via argv[1] */
	tim.period = (uint32_t)strtol(argv[1], NULL, 10);

	/**! Get interrupt timer */
	timer_irq_enable = strtol(argv[3], NULL, 10);
	if (1 != timer_irq_enable)
	{
		console_putstring("Error: Interrupt Invalid\r\n");
		return -1;
	}

	/**! Get select performance API via argv[4] */
	select = (uint8_t)strtol(argv[4], NULL, 10);

	/**! Callback enable flag */
	tim.callback_handler = timer_irq;
	timer_irq_counter = 0;

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initialize GPIO */
	hal_gpio_init(&gpio);

	if(0 == select)
	{
		hal_gpio_set_high(&gpio);
	}
	/**! Initialize timer module */
	qc_assert (HAL_TIM_OK == hal_timer_init(&tim));
	if(0 == select)
	{
		hal_gpio_set_low(&gpio);
	}

	if(1 == select)
	{
		hal_gpio_set_high(&gpio);
	}
	/**! Enable timer module */
	qc_assert(HAL_TIM_OK == hal_timer_start(tim.chid));
	if(1 == select)
	{
		hal_gpio_set_low(&gpio);
	}

	/**! Disable timer module */
	delay_ms(3000);

	if(2 == select)
	{
		hal_gpio_set_high(&gpio);
	}
	qc_assert(HAL_TIM_OK == hal_timer_stop(tim.chid));
	if(2 == select)
	{
		hal_gpio_set_low(&gpio);
	}
	count1 = timer_irq_counter;

	/**! Updates timer period */
	tim.period = strtol(argv[2], NULL, 10);
	timer_irq_counter = 0;

	if(3 == select)
	{
		hal_gpio_set_high(&gpio);
	}
	/** Check updates timer period */
	qc_assert (HAL_TIM_OK == hal_timer_update_period(&tim));
	if(3 == select)
	{
		hal_gpio_set_low(&gpio);
	}

	if(4 == select)
	{
		hal_gpio_set_high(&gpio);
	}
	/**! Restart timer module */
	qc_assert(HAL_TIM_OK == hal_timer_start(tim.chid));
	if(4 == select)
	{
		hal_gpio_set_low(&gpio);
	}

	/**! Disable timer module */
	delay_ms(3000);

	if(5 == select)
	{
		hal_gpio_set_high(&gpio);
	}
	qc_assert (HAL_TIM_OK == hal_timer_stop(tim.chid));
	if(5 == select)
	{
		hal_gpio_set_low(&gpio);
	}
	count2 = timer_irq_counter;
	/**! Difference value count1 and count 2 */
	qc_assert(count1 != count2);

	/**! Do judgment */
	qc_report();
	return (int)HAL_TIM_OK;
}
/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
