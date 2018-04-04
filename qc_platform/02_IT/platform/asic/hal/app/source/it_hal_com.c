/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    it_hal_com.c
 * @author  The LightCo
 * @version V1.0.1
 * @date    19-Apr, 2016
 * @brief   This file contains expand of the hal_com driver
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
#include "it_hal_com.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define PARAM_NUM	2
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Static functions ----------------------------------------------------------*/
/**
 * @brief print specific string over usart
 * @detail to test hal_com_sendbyte API
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 0	: success
 *         Others: instruction failed
 */
LOCAL int it_hal_com_001(char** argv, int argc);

/**
 * @brief Reads string number of characters from COM port
 * @detail to test hal_com_readbyte API
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 0	: success
 *         Others: instruction failed
 */
LOCAL int it_hal_com_002(char** argv, int argc);

/**
 * @brief can not read string from hardware when using invalid COM port
 * @detail to test hal_com_readbyte API (abnormal test cases)
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 0	 : success
 *         Others: instruction failed
 */
LOCAL int it_hal_com_003(char** argv, int argc);

/**
 * @brief testing hal_com_init API
 * @detail To verify that fail to initialize COM1 with invalid baud rate.
 *        [Configure]
 *          - Port COM1
 *          - Baudrate: 115200 + 1
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 0	: success
 *         Others: instruction failed
 */
LOCAL int it_hal_com_004(char** argv, int argc);

/**
 * @brief testing hal_com_readbyte API
 * @detail To verify that there is timeout error will be reported from 
 *         COM1 driver in case there is no any data is transferred.
 *        [Configure]
 *          - Port COM1
 *          - Baudrate: 115200
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 0	: success
 *         Others: instruction failed
 */
LOCAL int it_hal_com_005(char** argv, int argc);

/* Exported global varibles --------------------------------------------------*/
/**
 *  @Brief: COM module testing map
 */
LOCAL it_map_t it_com_tests_table[] = {
		{"COM_001", it_hal_com_001},
		{"COM_002", it_hal_com_002},
		{"COM_003", it_hal_com_003},
		{"COM_004", it_hal_com_004},
		{"COM_005", it_hal_com_005},
		{"",  NULL}
};
/* Exported functions --------------------------------------------------------*/

/**
 * @brief COM module's testing handler
 * @detail to COM module's testing handler
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @return 0	: success
 *         Others: instruction failed
 */
int it_hal_com_handler(char** argv, int argc)
{
	/**< Check command counter >*/
	if (1 >= argc) return -1;
	if (NULL == argv) return -1;
	/**< Get command index, mean test case name >*/
	int index = handler_parser(argv[0], it_com_tests_table);
	if (-1 != index)
	{
		return it_com_tests_table[index].handler(&argv[1], argc - 1);
	}
	else
		return index;
}

/**
 * @brief print specific string over usart
 * @detail to test hal_com_sendbyte API
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 0	: success
 *         Others: instruction failed
 */
LOCAL int it_hal_com_001(char** argv, int argc)
{
	/**! Verify input parameters */
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
	hal_com_t test_com = {0, 0, (void*)0, (void*)0};
	hal_com_name_t	port_name = 0; /* hal com port name */
	hal_com_baud_t	baudrate  = 0; /* hal com baud rate */

	/**! Get COM port number via argv[0] */
	port_name = (hal_com_name_t)strtol(argv[0], NULL, 10);

	memcpy((void*)&test_com.port_name, (void*)&port_name, sizeof(int));
	
	/**! Get baudrate via argv[1] */
	baudrate = (hal_com_baud_t)strtol(argv[1], NULL, 10);

	memcpy((void*)&test_com.baudrate, (void*)&baudrate, sizeof(int));

	/**! Local variables */
	char* ptr = argv[2];
	int i = 0;

	/* Reset all test point to default */
	qc_assert_reset();

	/**! Initialize com port*/
	hal_com_init(&test_com);

	for (i = 0; i < strlen(argv[2]); i++)
	{
		/**! Provide one byte data */
		test_com.data = (uint8_t*)(ptr + i);

		/**! Verify Call sending API*/
		qc_assert(COM_OK == hal_com_sendbyte(&test_com));
	}

	/**! Do judgment */
	qc_report();
	return (int)COM_OK;
}

/**
 * @brief Reads string number of characters from COM port
 * @detail to test hal_com_readbyte API
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 0	: success
 *         Others: instruction failed
 */
LOCAL int it_hal_com_002(char** argv, int argc)
{
	/**! Verify input parameters */
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
	hal_com_t test_com = {0, 0, (void*)0, (void*)0};
	hal_com_name_t	port_name = 0; /* hal com port name */
	hal_com_baud_t	baudrate  = 0; /* hal com baud rate */

	/**! Get COM port number via argv[0] */
	port_name = (hal_com_name_t)strtol(argv[0], NULL, 10);

	memcpy((void*)&test_com.port_name, (void*)&port_name, sizeof(int));

	/**! Get baudrate via argv[1] */
	baudrate = (hal_com_baud_t)strtol(argv[1], NULL, 10);

	memcpy((void*)&test_com.baudrate, (void*)&baudrate, sizeof(int));

	/**! Get number of bytes to read */
	uint16_t length = (uint16_t)strtol(argv[2], NULL, 10);
	int i = 0;
	/**! Provide buffer */
	uint8_t* rx_data = malloc(length + 1);
	if (NULL == rx_data)
	{
		console_putstring("Error: Malloc error\r\n");
		return -1;
	}

	/* Reset all test point to default */
	qc_assert_reset();

	/**! Initialize com port*/
	hal_com_init(&test_com);
	
	/**! Read data */
	for (i = 0; i < length; i++)
	{
		/**! Provide one byte data */
		test_com.data = rx_data + i;
		/**! Call sending method */
		qc_assert(COM_OK == hal_com_readbyte(&test_com));
	}

	/**! Show back */
	rx_data[length] = '\0';
	/**! Write back to console */
	console_putstring("Received data: ");
	console_putstring((char*)rx_data);
	console_putstring("\r\b");

	/**! Free memory */
	free(rx_data);
	/**! Do judgment */
	qc_report();
	return (int)COM_OK;
}


/**
 * @brief can not read string from hardware when using invalid COM port
 * @detail to test hal_com_readbyte API (abnormal test cases)
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 0	 : success
 *         Others: instruction failed
 */
LOCAL int it_hal_com_003(char** argv, int argc)
{
	/**! Verify input parameters */
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
	hal_com_t test_com = {COM1, COM_BAUD_115200, (void*)0, (void*)0};
	hal_com_name_t	port_name = 0; /* hal com port name */

	/**! Get COM port number via argv[0] */
	port_name = (hal_com_name_t)strtol(argv[0], NULL, 10);
	if ((COM1 <= port_name) && (COM_MAX_IDX > port_name))
	{
		console_putstring("Error: Need provide invalid COM port instead.\r\n");
		return -1;
	}
	memcpy((void*)&test_com.port_name, (void*)&port_name, sizeof(int));

	/**! Get number of bytes to read from command parameter */
	uint16_t length = (uint16_t)strtol(argv[1], NULL, 10);
	
	/**! Provide buffer */
	uint8_t* rx_data = malloc(length + 1);
	if (NULL == rx_data)
	{
		console_putstring("Error: Malloc error\r\n");
		return -1;
	}

	int i = 0;
	
	/* Reset all test point to default */
	qc_assert_reset();

	/**! Initialize com port  */
	hal_com_init(&test_com);
	
	/**! Invoke read data */
	for (i = 0; i < length; i++)
	{
		test_com.data = (uint8_t*)(rx_data + i);
		qc_assert(COM_OK != hal_com_readbyte(&test_com));
	}

	/**! Free memory */
	free(rx_data);

	/**! Do judgment */
	qc_report();

	return 0;
}

/**
 * @brief testing hal_com_init API
 * @detail To verify that fail to initialize COM1 with invalid baud rate
 *   or Port[Configure]
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 0	: success
 *         Others: instruction failed
 */
LOCAL int it_hal_com_004(char** argv, int argc)
{
	/**! Verify input parameters */
	if (2 != argc)
	{
		console_putstring("Error: Wrong number of parameters\r\n");
		return -1;
	}
	if (NULL != argv)
	{
		console_putstring("Error: Empty argument\r\n");
		return -1;
	}

	/**! Configuration */
	hal_com_t test_com = {0, 0, (void*)0, (void*)0};
	hal_com_name_t	port_name = 0; /* hal com port name */
	hal_com_baud_t	baudrate  = 0; /* hal com baud rate */

	/**! Get COM port number via argv[0] */
	port_name = (hal_com_name_t)(strtol(argv[0], NULL, 10));

	memcpy((void*)&test_com.port_name, (void*)&port_name, sizeof(int));

	/**! Get baudrate via argv[1] */
	baudrate = (hal_com_baud_t)strtol(argv[1], NULL, 10);

	memcpy((void*)&test_com.baudrate, (void*)&baudrate, sizeof(int));

	/* Reset all test point to default */
	qc_assert_reset();

	/**! Verify initialize com port */
	hal_com_init(&test_com);

	/**! Check assert error */
	qc_assert(qc_assert_status());

	/**! Do judgment */
	qc_report();
	return (int)COM_OK;
}

/**
 * @brief testing hal_com_readbyte API
 * @detail To verify that there is timeout error will be reported from 
 *         COM1 driver in case there is no any data is transferred.
 *        [Configure]
 *          - Port COM1
 *          - Baudrate: 115200
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 0	: success
 *         Others: instruction failed
 */
LOCAL int it_hal_com_005(char** argv, int argc)
{
	/**! Verify input parameters */
	if (3 != argc)
	{
		console_putstring("Error: Wrong number of parameters\r\n");
		return -1;
	}
	if (NULL != argv)
	{
		console_putstring("Error: Empty argument\r\n");
		return -1;
	}

	/**! Configuration */
	hal_com_t test_com = {0, 0, (void*)0, (void*)0};
	hal_com_name_t	port_name = 0; /* hal com port name */
	hal_com_baud_t	baudrate  = 0; /* hal com baud rate */

	/**! Get COM port number via argv[0] */
	port_name = (hal_com_name_t)strtol(argv[0], NULL, 10);
	if (COM_MAX_IDX <= port_name)
	{
		console_putstring("Error: Need provide valid COM port instead.\r\n");
		return -1;
	}
	memcpy((void*)&test_com.port_name, (void*)&port_name, sizeof(int));

	/**! Get baudrate via argv[1] */
	baudrate = (hal_com_baud_t)strtol(argv[1], NULL, 10);
	if (COM_BAUD_115200 != baudrate)
	{
		console_putstring("Error: Need provide valid baudrate instead.\r\n");
		return -1;
	}
	memcpy((void*)&test_com.baudrate, (void*)&baudrate, sizeof(int));

	/**! Get number of bytes to read */
	uint16_t length = (uint16_t)strtol(argv[2], NULL, 10);

	int i = 0;
	char str[32];
	/**! Provide buffer */
	uint8_t* rx_data = malloc(length + 1);
	if (NULL == rx_data)
	{
		console_putstring("Error: Malloc error\r\n");
		return -1;
	}

	/* Reset all test point to default */
	qc_assert_reset();

	/* Initialize com port */
	hal_com_init(&test_com);

	/**! Read data */
	for (i = 0; i < length; i++)
	{
		test_com.data = rx_data + i;
		qc_assert(COM_TIMEOUT == hal_com_readbyte(&test_com));
	}

	sprintf(str, "Data: %s\r\n", rx_data);
	console_putstring(str);
	/**! Free memory */
	free(rx_data);
	/**! Do judgment */
	qc_report();
	return (int)COM_OK;
}

/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
