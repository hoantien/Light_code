/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    it_hal_gpio.c
 * @author  The LightCo
 * @version V1.0.1
 * @date    19-Apr-2016
 * @brief   Test application for HAL GPIO module
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
#include "it_hal_gpio.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Exported global variables -------------------------------------------------*/
/* Static functions ----------------------------------------------------------*/

/**
 * @brief To verify interrupt GPIO module is initialized
 * @detail to test hal_gpio_enable_exti API
 * @param[in]	: N/A
 * @param[out]	: N/A
 * @return 		: N/A
 */
LOCAL void interrupt_input_exti(void);

/**
 * @brief  To verify that GPIO module is initialized to default level in case
 * it's output pin.
 * @detail to test hal_gpio_init API
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] 		: N/A
 * @return 0		: success
 *  Others			: instruction failed
 */
LOCAL int it_hal_gpio_001(char** argv, int argc);

/**
 * @brief To verify that GPIO module could invoke callback function interrupt
 * @detail to test hal_gpio_enable_exti API
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] 		: N/A
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_hal_gpio_002(char** argv, int argc);

/**
 * @brief To verify that GPIO module could invoke
 * callback function interrupt both edge
 * @detail  hal_gpio_enable_exti API
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] 		: N/A
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_hal_gpio_003(char** argv, int argc);

/**
 * @brief To verify that GPIO module test level low
 * @detail to test hal_gpio_set_low API
 * @param[in] argv	: arguments list
 * @param[in] argc	: arguments counter
 * @param[out] 		: N/A
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_hal_gpio_004(char** argv, int argc);

/**
 * @brief To verify that GPIO module test level high
 * @detail to test hal_gpio_set_high API
 * @param[in] argv	: arguments list
 * @param[in] argc	: arguments counter
 * @param[out]		:N/A
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_hal_gpio_005(char** argv, int argc);

/**
 * @brief To verify that GPIO module test read state input
 * @detail to test hal_gpio_read_input API
 * @param[in] argv	: arguments list
 * @param[in] argc	: arguments counter
 * @param[out]		: N/A
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_hal_gpio_006(char** argv, int argc);

/**
 * @brief To verify that GPIO module test toggle state
 * @detail to test hal_gpio_toggle API
 * @param[in] argv	: arguments list
 * @param[in] argc	: arguments counter
 * @param[out]		: N/A
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_hal_gpio_007(char** argv, int argc);

/**
 * @brief To verify that GPIO module test Invalid API
 * @detail to test parameter input
 * @param[in] argv	: arguments list
 * @param[in] argc	: arguments counter
 * @param[out]		: N/A
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_hal_gpio_008(char** argv, int argc);

/**
 * @brief to test hal_gpio_init , hal_gpio_set_low,
 * hal_gpio_set_high, hal_gpio_toggle API in abnormal case
 * @detail to test parameter input
 * @param[in] argv	: arguments list
 * @param[in] argc	: arguments counter
 * @param[out]		: N/A
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_hal_gpio_009(char** argv, int argc);

/**
 * @brief to test performance API Output
 * @detail to test Performance API Output
 * @param[in] argv	: arguments list
 * @param[in] argc	: arguments counter
 * @param[out]		: N/A
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_hal_gpio_010(char** argv, int argc);

/**
 * @brief to test performance API Input
 * @detail to test Performance API Input
 * @param[in] argv	: arguments list
 * @param[in] argc	: arguments counter
 * @param[out]		: N/A
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_hal_gpio_011(char** argv, int argc);
/* Private variables ---------------------------------------------------------*/
LOCAL volatile uint32_t count = 0;

/**
 *  @Brief: GPIO module testing map
 */
LOCAL it_map_t it_gpio_tests_table[] = {
		{"GPIO_001", it_hal_gpio_001},
		{"GPIO_002", it_hal_gpio_002},
		{"GPIO_003", it_hal_gpio_003},
		{"GPIO_004", it_hal_gpio_004},
		{"GPIO_005", it_hal_gpio_005},
		{"GPIO_006", it_hal_gpio_006},
		{"GPIO_007", it_hal_gpio_007},
		{"GPIO_008", it_hal_gpio_008},
		{"GPIO_009", it_hal_gpio_009},
		{"GPIO_010", it_hal_gpio_010},
		{"GPIO_011", it_hal_gpio_011},
		{"", NULL}
};

/**
 * @brief GPIO module's testing handler
 * @detail to GPIO module's testing handler
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @return 0	: success
 *         Others: instruction failed
 */
int it_hal_gpio_handler(char** argv, int argc)
{
	/**! Check command counter */
	if (1 >= argc) return -1;
	if (NULL == argv) return -1;
	/**! Get command index, mean test case name */
	int index = handler_parser(*argv, it_gpio_tests_table);
	if (-1 != index)
	{
		return it_gpio_tests_table[index].handler(&argv[1], argc - 1);
	}
	else
		return index;
}

/**
 * @brief To verify interrupt GPIO module is initialized
 * @detail to test hal_gpio_enable_exti API
 * @param[in]	: N/A
 * @param[out]	: N/A
 * @return 		: N/A
 */
LOCAL void interrupt_input_exti(void)
{
	count ++;
}

/**
 * @brief  To verify that GPIO module is initialized to default level in case
 * it's output pin.
 * @detail to test hal_gpio_init API
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] 		: N/A
 * @return 0		: success
 *  Others			: instruction failed
 */
LOCAL int it_hal_gpio_001(char** argv, int argc)
{
	/**!! Check command parameter */
	if (2 != argc)
	{
		console_putstring("Error: Wrong number of parameters\r\n");
		return -1;
	}

	/**!! Check argument */
	if (NULL == argv)
	{
		console_putstring("Error: Empty argument\r\n");
		return -1;
	}

	/**!! Configuration */
	hal_gpio_t test_gpio = {GPIO_PORTA, GPIO_PIN_0, GPIO_DIR_OUT};

	/**!! Get GPIO port number via argv[0] */
	test_gpio.port = (hal_gpio_port_t)strtol(argv[0], NULL, 10);

	/**! check Port Valid PortA -> port D */
	if(test_gpio.port >= GPIO_PORT_MAX_IDX)
	{
		console_putstring("Error: Do not support Port name.\r\n");
		return -1;
	}

	/**! Get parameter Pin name */
	test_gpio.pin = (hal_gpio_pin_t)strtol(argv[1], NULL, 10);

	/**! Check Pin Valid Pin0 -> Pin7 */
	if(test_gpio.pin >= GPIO_PIN_MAX_IDX)
	{
		console_putstring("Error : Do not support Port pin.\r\n");
		return -1;
	}

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Check initialized GPIO */
	hal_gpio_init(&test_gpio);

	/**! Do judgment */
	qc_report();

	return 0;
}

/**
 * @brief To verify that GPIO module could invoke callback function interrupt
 * @detail to test hal_gpio_enable_exti API
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] 		: N/A
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_hal_gpio_002(char** argv, int argc)
{
	/**! Check command parameter */
	if (4 != argc)
	{
		console_putstring("Error: Wrong number of parameters\r\n");
		return -1;
	}

	/**! Check argument */
	if (NULL == argv)
	{
		console_putstring("Error: Empty argument\r\n");
		return -1;
	}

	/**! Configuration */
	hal_gpio_t test_gpio = {GPIO_PORTA, GPIO_PIN_0, GPIO_DIR_IN};
	hal_gpio_exti_t test_exti;
	volatile uint32_t test_timeout;

	/**! Get GPIO port number via argv[0] */
	test_gpio.port = (hal_gpio_port_t)strtol(argv[0], NULL, 10);

	/**! Check Port Valid PortA -> port D */
	if(test_gpio.port >= GPIO_PORT_MAX_IDX)
	{
		console_putstring("Error: Do not support Port name.\r\n");
		return -1;
	}

	/**! Get parameter Pin name */
	test_gpio.pin = (hal_gpio_pin_t)strtol(argv[1], NULL, 10);

	/**! Check Pin Valid Pin0 -> Pin7 */
	if(test_gpio.pin >= GPIO_PIN_MAX_IDX)
	{
		console_putstring("Error : Do not support Port Pin.\r\n");
		return -1;
	}

	/**! Get value external interrupt */
	test_exti.int_type = (hal_gpio_exti_type_t)strtol(argv[2], NULL, 10);

	/**! Check value input external interrupt */
	if(test_exti.int_type > GPIO_EXTI_RISING_EDGE)
	{
		console_putstring("Error : Do not support exti type.\r\n");
		return -1;
	}

	/**! Get pin interrupt */
	test_exti.pin = test_gpio.pin;
	test_exti.irq_handler = interrupt_input_exti;

	/**! Get value time out */
	test_timeout = (uint32_t)strtol(argv[3], NULL, 10);
	test_timeout = test_timeout * 20000000;

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Check initialized GPIO */
	hal_gpio_init(&test_gpio);

	/**! Check enable external interrupt */
	hal_gpio_enable_exti(&test_exti);

	/**! Wait time out */
	while(--test_timeout);

	/**! Check external interrupt */
	qc_assert(0 != count);

	/**! Do judgment */
	qc_report();
	return 0;
}

/**
 * @brief To verify that GPIO module could invoke
 * callback function interrupt both edge
 * @detail  hal_gpio_enable_exti API
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] 		: N/A
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_hal_gpio_003(char** argv, int argc)
{
	/**! Check command parameter */
	if (4 != argc)
	{
		console_putstring("Error: Wrong number of parameters\r\n");
		return -1;
	}

	/**! Check argument */
	if (NULL == argv)
	{
		console_putstring("Error: Empty argument\r\n");
		return -1;
	}

	/**! Configuration */
	hal_gpio_t test_gpio = {GPIO_PORTA, GPIO_PIN_0, GPIO_DIR_IN};
	hal_gpio_exti_t test_exti;
	volatile uint32_t test_timeout;

	/**! Get GPIO port number via argv[0] */
	test_gpio.port = (hal_gpio_port_t)strtol(argv[0], NULL, 10);

	/**! Check Port Valid PortA -> port D */
	if(test_gpio.port >= GPIO_PORT_MAX_IDX)
	{
		console_putstring("Error: Do not support Port name.\r\n");
		return -1;
	}

	/**! Get parameter Pin name */
	test_gpio.pin = (hal_gpio_pin_t)strtol(argv[1], NULL, 10);

	/* Check Pin Valid Pin0 -> Pin7 */
	if(test_gpio.pin >= GPIO_PIN_MAX_IDX)
	{
		console_putstring("Error :Do not support Port pin.\r\n");
		return -1;
	}

	/**! Get value external interrupt */
	test_exti.int_type = (hal_gpio_exti_type_t)strtol(argv[2], NULL, 10);

	/**! Check value input extern interrupt */
	if(test_exti.int_type > GPIO_EXTI_RISING_EDGE)
	{
		console_putstring("Error : Do not support exti type.\r\n");
		return -1;
	}

	/**! Get pin interrupt */
	test_exti.pin = test_gpio.pin;
	test_exti.irq_handler = interrupt_input_exti;

	/**! Get value time out */
	test_timeout = (uint32_t)strtol(argv[3], NULL, 10);
	test_timeout = test_timeout * 20000000;

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Check initialized GPIO */
	hal_gpio_init(&test_gpio);

	/**! Check enable interrupt external */
	hal_gpio_enable_exti(&test_exti);

	/**! Wait time out */
	while(--test_timeout);

	/**! Check external interrupt */
	qc_assert((count % 2) == 0);

	/**! Do judgment */
	qc_report();
	return 0;
}

/**
 * @brief To verify that GPIO module test level low
 * @detail to test hal_gpio_set_low API
 * @param[in] argv	: arguments list
 * @param[in] argc	: arguments counter
 * @param[out] 		: N/A
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_hal_gpio_004(char** argv, int argc)
{
	/**! Check command parameter */
	if (2 != argc)
	{
		console_putstring("Error: Wrong number of parameters\r\n");
		return -1;
	}

	/**! Check argument */
	if (NULL == argv)
	{
		console_putstring("Error: Empty argument\r\n");
		return -1;
	}

	/**! Configuration */
	hal_gpio_t test_gpio = {GPIO_PORTA, GPIO_PIN_0, GPIO_DIR_OUT};

	/**! Get GPIO port number via argv[0] */
	test_gpio.port = (hal_gpio_port_t)strtol(argv[0], NULL, 10);

	/**! Check Port Valid PortA -> port D */
	if(test_gpio.port >= GPIO_PORT_MAX_IDX)
	{
		console_putstring("Error: Do not support Port name.\r\n");
		return -1;
	}

	/**! Get parameter Pin name */
	test_gpio.pin = (hal_gpio_pin_t)strtol(argv[1], NULL, 10);

	/**! Check Pin Valid Pin0 -> Pin7 */
	if(test_gpio.pin >= GPIO_PIN_MAX_IDX)
	{
		console_putstring("Error : Do not support Port pin.\r\n");
		return -1;
	}

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Check initialized GPIO */
	hal_gpio_init(&test_gpio);

	/**! Check set low GPIO */
	hal_gpio_set_low(&test_gpio);

	/**! Do judgment */
	qc_report();
	return 0;
}

/**
 * @brief To verify that GPIO module test level high
 * @detail to test hal_gpio_set_high API
 * @param[in] argv	: arguments list
 * @param[in] argc	: arguments counter
 * @param[out]		:N/A
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_hal_gpio_005(char** argv, int argc)
{
	/**! Check command parameter */
	if (2 != argc)
	{
		console_putstring("Error: Wrong number of parameters\r\n");
		return -1;
	}

	/**! Check argument */
	if (NULL == argv)
	{
		console_putstring("Error: Empty argument\r\n");
		return -1;
	}

	/**! Configuration */
	hal_gpio_t test_gpio = {GPIO_PORTA, GPIO_PIN_0, GPIO_DIR_OUT};

	/**! Get GPIO port number via argv[0] */
	test_gpio.port = (hal_gpio_port_t)strtol(argv[0], NULL, 10);

	/**! check Port Valid PortA -> port D */
	if(test_gpio.port >= GPIO_PORT_MAX_IDX)
	{
		console_putstring("Error: Do not support Port name.\r\n");
		return -1;
	}

	/**! Get parameter Pin name */
	test_gpio.pin = (hal_gpio_pin_t)strtol(argv[1], NULL, 10);

	/**! Check Pin Valid Pin0 -> Pin7 */
	if(test_gpio.pin >= GPIO_PIN_MAX_IDX)
	{
		console_putstring("Error : Do not support Port pin.\r\n");
		return -1;
	}

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Check initialized GPIO */
	hal_gpio_init(&test_gpio);

	/**! Check set high GPIO */
	hal_gpio_set_high(&test_gpio);

	/**! Do judgment */
	qc_report();
	return 0;
}
/**
 * @brief To verify that GPIO module test read state input
 * @detail to test hal_gpio_read_input API
 * @param[in] argv	: arguments list
 * @param[in] argc	: arguments counter
 * @param[out]		: N/A
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_hal_gpio_006(char** argv, int argc)
{
	/**! Check command parameter */
	if (3 != argc)
	{
		console_putstring("Error: Wrong number of parameters\r\n");
		return -1;
	}

	/**! Check argument */
	if (NULL == argv)
	{
		console_putstring("Error: Empty argument\r\n");
		return -1;
	}

	/**! Configuration */
	hal_gpio_t test_gpio = {GPIO_PORTA, GPIO_PIN_0, GPIO_DIR_IN};
	hal_gpio_level_t test_state;

	/**! Get GPIO port number via argv[0] */
	test_gpio.port = (hal_gpio_port_t)strtol(argv[0], NULL, 10);

	/**! check Port Valid PortA -> port D */
	if(test_gpio.port >= GPIO_PORT_MAX_IDX)
	{
		console_putstring("Error: Do not support Port name. \r\n");
		return -1;
	}

	/**! Get parameter Pin name */
	test_gpio.pin = (hal_gpio_pin_t)strtol(argv[1], NULL, 10);

	/**! Check Pin Valid Pin0 -> Pin7 */
	if(test_gpio.pin >= GPIO_PIN_MAX_IDX)
	{
		console_putstring("Error : Do not support Port pin.\r\n");
		return -1;
	}

	/**! Get parameter check state input */
	test_state = (hal_gpio_level_t)strtol(argv[2], NULL, 10);

	/**! Check input State */
	if(test_state > GPIO_LEVEL_HIGH)
	{
		console_putstring("Error: Do not support Port State.\r\n");
		return -1;
	}

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Check initialized GPIO */
	hal_gpio_init(&test_gpio);

	/**! Check state level GPIO */
	qc_assert(test_state == hal_gpio_read(&test_gpio));

	/**! Do judgment */
	qc_report();
	return 0;
}

/**
 * @brief To verify that GPIO module test toggle state
 * @detail to test hal_gpio_toggle API
 * @param[in] argv	: arguments list
 * @param[in] argc	: arguments counter
 * @param[out]		: N/A
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_hal_gpio_007(char** argv, int argc)
{
	/**! Check command parameter */
	if (2 != argc)
	{
		console_putstring("Error: Wrong number of parameters\r\n");
		return -1;
	}

	/**! Check argument */
	if (NULL == argv)
	{
		console_putstring("Error: Empty argument\r\n");
		return -1;
	}

	/**! Configuration */
	hal_gpio_t test_gpio = {GPIO_PORTA, GPIO_PIN_0, GPIO_DIR_OUT};

	/**! Get GPIO port number via argv[0] */
	test_gpio.port = (hal_gpio_port_t)strtol(argv[0], NULL, 10);

	/**! check Port Valid PortA -> port D */
	if(test_gpio.port >= GPIO_PORT_MAX_IDX)
	{
		console_putstring("Error: Do not support Port name.\r\n");
		return -1;
	}

	/**! Get parameter Pin name */
	test_gpio.pin = (hal_gpio_pin_t)strtol(argv[1], NULL, 10);

	/**! Check Pin Valid Pin0 -> Pin7 */
	if(test_gpio.pin >= GPIO_PIN_MAX_IDX)
	{
		console_putstring("Error : Do not support Port Pin.\r\n");
		return -1;
	}

	/**! Reset all test point to default */
	qc_assert_reset();

	volatile uint32_t time_out;
	/**! Check initialized GPIO */
	hal_gpio_init(&test_gpio);

	/**! Set toggle state */
	uint32_t time_wait = 50;
	while(--time_wait)
	{
		time_out = 20000;
		while(--time_out);
		hal_gpio_toggle(&test_gpio);
	}

	/**! Do judgment */
	qc_report();
	return 0;
}

/**
 * @brief to test hal_gpio_init API in abnormal case
 * @detail to test parameter input
 * @param[in] argv	: arguments list
 * @param[in] argc	: arguments counter
 * @param[out]		: N/A
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_hal_gpio_008(char** argv, int argc)
{
	/**! Check command parameter */
	if (3 != argc)
	{
		console_putstring("Error: Wrong number of parameters\r\n");
		return -1;
	}

	/**! Check argument parameter*/
	if (NULL == argv)
	{
		console_putstring("Error: Empty argument\r\n");
		return -1;
	}

	/**! Configuration */
	hal_gpio_t test_gpio = {GPIO_PORTA, GPIO_PIN_0, GPIO_DIR_OUT};

	/**! Get GPIO Port name via argv[0] */
	test_gpio.port = (hal_gpio_port_t)strtol(argv[0], NULL, 10);

	/**! Get GPIO Port pin via argv[1]  */
	test_gpio.pin = (hal_gpio_pin_t)strtol(argv[1], NULL, 10);

	/**! Get GPIO Port direction via argv[2]*/
	test_gpio.direction = (hal_gpio_dir_t)strtol(argv[2], NULL, 10);

	/**! Reset all test point to default */
	qc_assert_reset();

	/**!Initialized GPIO */
	hal_gpio_init(&test_gpio);

	/**! Check assert error */
	qc_assert(qc_assert_status());

	/**! Do judgment */
	qc_report();
	return 0;
}

/**
 * @brief to test hal_gpio_set_low,hal_gpio_read
 * hal_gpio_set_high, hal_gpio_toggle API in abnormal case
 * @detail to test parameter input
 * @param[in] argv	: arguments list
 * @param[in] argc	: arguments counter
 * @param[out]		: N/A
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_hal_gpio_009(char** argv, int argc)
{
	/**! Check command parameter */
	if (4 != argc)
	{
		console_putstring("Error: Wrong number of parameters\r\n");
		return -1;
	}

	/**! Check argument parameter*/
	if (NULL == argv)
	{
		console_putstring("Error: Empty argument\r\n");
		return -1;
	}

	/**! Configuration */
	hal_gpio_t test_gpio = {GPIO_PORTA, GPIO_PIN_0, GPIO_DIR_OUT};
	uint8_t select = 0;

	/**!Initialized GPIO */
	hal_gpio_init(&test_gpio);
	/**! Get GPIO Port name via argv[0] */
	test_gpio.port = (hal_gpio_port_t)strtol(argv[0], NULL, 10);

	/**! Get GPIO Port pin via argv[1]  */
	test_gpio.pin = (hal_gpio_pin_t)strtol(argv[1], NULL, 10);

	/**! Get GPIO Port direction via argv[2]*/
	test_gpio.direction = (hal_gpio_dir_t)strtol(argv[2], NULL, 10);

	/**! Get select API */
	select = (uint8_t)strtol(argv[3], NULL, 10);
	if(3 < select)
	{
		console_putstring("Error: Wrong parameter select test cases.\r\n");
		return -1;
	}
	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Select test case */
	switch(select)
	{
		case 0:
		{
			hal_gpio_set_low(&test_gpio);
			break;
		}
		case 1:
		{
			hal_gpio_set_high(&test_gpio);
			break;
		}
		case 2:
		{
			hal_gpio_toggle(&test_gpio);
			break;
		}
		case 3:
		{
			hal_gpio_read(&test_gpio);
			break;
		}
		default:break;
	}
	/**! Check assert error */
	qc_assert(qc_assert_status());

	/**! Do judgment */
	qc_report();
	return 0;
}

/**
 * @brief to test performance API output
 * @detail to test Performance API output
 * @param[in] argv	: arguments list
 * @param[in] argc	: arguments counter
 * @param[out]		: N/A
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_hal_gpio_010(char** argv, int argc)
{
	/**! Check command parameter */
	if (2 != argc)
	{
		console_putstring("Error: Wrong number of parameters\r\n");
		return -1;
	}

	/**! Check argument */
	if (NULL == argv)
	{
		console_putstring("Error: Empty argument\r\n");
		return -1;
	}

	/**! Configuration */
	hal_gpio_t test_gpio = {GPIO_PORTA, GPIO_PIN_0, GPIO_DIR_OUT};
	hal_gpio_t gpio = {GPIO_PORTA, GPIO_PIN_1, GPIO_DIR_OUT};
	uint8_t select = 0;

	/**! Get GPIO port number via argv[0] */
	test_gpio.port = (hal_gpio_port_t)strtol(argv[0], NULL, 10);

	/**! check Port Valid PortA -> port D */
	if(test_gpio.port >= GPIO_PORT_MAX_IDX)
	{
		console_putstring("Error: Do not support Port name.\r\n");
		return -1;
	}

	/**! Get parameter Pin name */
	test_gpio.pin = (hal_gpio_pin_t)strtol(argv[1], NULL, 10);

	/**! Get variable select API via argv[2]*/
	select = (uint8_t)strtol(argv[2], NULL, 10);

	/**! Check Pin Valid Pin0 -> Pin7 */
	if(test_gpio.pin >= GPIO_PIN_MAX_IDX)
	{
		console_putstring("Error : Do not support Port pin.\r\n");
		return -1;
	}

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initialize GPIO test performance */
	hal_gpio_init(&gpio);

	if(0 == select)
	{
		hal_gpio_set_high(&gpio);
	}
	/**! Check initialized GPIO */
	hal_gpio_init(&test_gpio);
	if(0 == select)
	{
		hal_gpio_set_low(&gpio);
	}

	if(1 == select)
	{
		hal_gpio_set_high(&gpio);
	}
	/**! Check set high GPIO */
	hal_gpio_set_high(&test_gpio);
	if(1 == select)
	{
		hal_gpio_set_low(&gpio);
	}

	if(2 == select)
	{
		hal_gpio_set_high(&gpio);
	}
	/**! Check set low GPIO */
	hal_gpio_set_low(&test_gpio);
	if(2 == select)
	{
		hal_gpio_set_low(&gpio);
	}

	if(3 == select)
	{
		hal_gpio_set_high(&gpio);
	}
	/**! Toggle GPIO */
	hal_gpio_toggle(&test_gpio);
	if(3 == select)
	{
		hal_gpio_set_low(&gpio);
	}
	/**! Do judgment */
	qc_report();
	return 0;
}

/**
 * @brief to test performance API Input
 * @detail to test Performance API Input
 * @param[in] argv	: arguments list
 * @param[in] argc	: arguments counter
 * @param[out]		: N/A
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_hal_gpio_011(char** argv, int argc)
{
	/**! Check command parameter */
	if (4 != argc)
	{
		console_putstring("Error: Wrong number of parameters\r\n");
		return -1;
	}

	/**! Check argument */
	if (NULL == argv)
	{
		console_putstring("Error: Empty argument\r\n");
		return -1;
	}

	/**! Configuration */
	hal_gpio_t test_gpio = {GPIO_PORTA, GPIO_PIN_0, GPIO_DIR_IN};
	hal_gpio_t gpio = {GPIO_PORTA, GPIO_PIN_1, GPIO_DIR_OUT};
	hal_gpio_exti_t test_exti = {GPIO_PIN_0, GPIO_EXTI_FALLING_EDGE};
	uint32_t time_out = 20000;
	uint8_t select = 0;

	/**! Get GPIO port number via argv[0] */
	test_gpio.port = (hal_gpio_port_t)strtol(argv[0], NULL, 10);

	/**! Check Port Valid PortA -> port D */
	if(test_gpio.port >= GPIO_PORT_MAX_IDX)
	{
		console_putstring("Error: Do not support Port name.\r\n");
		return -1;
	}

	/**! Get parameter Pin name */
	test_gpio.pin = (hal_gpio_pin_t)strtol(argv[1], NULL, 10);

	/**! Check Pin Valid Pin0 -> Pin7 */
	if(test_gpio.pin >= GPIO_PIN_MAX_IDX)
	{
		console_putstring("Error : Do not support Port Pin.\r\n");
		return -1;
	}

	/**! Get variable select API test performance via argv[2] */
	select = (uint8_t)strtol(argv[2], NULL, 10);

	/**! Get pin interrupt */
	test_exti.pin = test_gpio.pin;
	test_exti.irq_handler = interrupt_input_exti;

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initialize GPIO test performance */
	hal_gpio_init(&gpio);

	if(0 == select)
	{
		hal_gpio_set_high(&gpio);
	}
	/**! Check initialized GPIO */
	hal_gpio_init(&test_gpio);
	if(0 == select)
	{
		hal_gpio_set_low(&gpio);
	}

	if(1 == select)
	{
		hal_gpio_set_high(&gpio);
	}
	/**! Read input */
	hal_gpio_read(&test_gpio);
	if(1 == select)
	{
		hal_gpio_set_low(&gpio);
	}

	if(2 == select)
	{
		hal_gpio_set_high(&gpio);
	}
	/**! Check enable external interrupt */
	hal_gpio_enable_exti(&test_exti);
	if(2 == select)
	{
		hal_gpio_set_low(&gpio);
	}
	/**! Wait time out */
	while(--time_out);

	/**! Check external interrupt */
	qc_assert(0 != count);

	/**! Do judgment */
	qc_report();
	return 0;
}
/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
