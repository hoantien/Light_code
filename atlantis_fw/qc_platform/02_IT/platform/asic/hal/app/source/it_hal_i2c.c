/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    it_hal_i2c.c
 * @author  The LightCo
 * @version V1.0.1
 * @date    19-Apr, 2016
 * @brief   This file contains expand of the hal_i2c driver
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
#include "it_hal_i2c.h"
#include <string.h>
#include <stdlib.h>
#include "hal_gpio.h"

/* Private typedef -----------------------------------------------------------*/
typedef enum
{
    CMD_REQUEST = 0, /**! HW CMD at request mode */
    CMD_RESPOND = 1   /**! HW CMD respond mode */
}i2c_hw_cmd_t;

typedef enum
{
	CMD_NEGATIVE = 0,
	CMD_POSITIVE = 1,
}cmd_t;
/* Private define ------------------------------------------------------------*/
/**! For CCB protocol */
#define I2C_TEST_DATA_LENGTH		16
#define I2C_CCB_COMMAND_LENGTH		4

/**! For Temperature sensor protocol */
#define I2C_TEMP_SENSOR_CMD_LENGTH 	1
#define I2C_TEMP_REG				((uint8_t)0x00)
#define I2C_TEMP_CONFIG_REG			((uint8_t)0x01)
#define I2C_TEMP_VAL_LOW_REG		((uint8_t)0x02)
#define I2C_TEMP_VAL_HIGH_REG		((uint8_t)0x03)
#define I2C_TEMP_SENSOR_DATA_LENGTH ((uint8_t)0x02)
#define I2C_TEMP_SENSOR_ADDR		((uint8_t)0x90)
/**! For temperature configuration register high */
#define I2C_TEMP_CFG_EXTEND_MODE							     ((uint8_t)0x10)
#define I2C_TEMP_CFG_NORMAL_MODE							     ((uint8_t)0x00)
/**! For temperature limit registers */
#define I2C_TEMP_LIMIT_HIGH									       ((uint8_t)36)
#define I2C_TEMP_LIMIT_LOW								           ((uint8_t)20)
#define I2C_TEMP_CONFIGURABLE_MASK                            ((uint16_t)0x9FC0)
#define I2C_TEMP_VAL(temp_reg_val)				    ((temp_reg_val & 0xFFF0)>>4)

#define I2C_DEFAULT_CFG 	                                    \
						  {                                     \
							.chid           = I2C_CH0,          \
							.clock_speed    = I2C_SPEED_100KHz, \
							.address_mode   = I2C_7BIT,         \
							.operation_mode = I2C_MASTER,       \
							.owner_addr     = 0x55,             \
							.irq_handler    = NULL              \
						   };
#define GPIO_DEFAULT      {0, 0, 1}

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
LOCAL volatile	uint8_t callback_index = 0;
LOCAL hal_gpio_t    hw_cmd_gpio   =  {GPIO_PORTA, GPIO_PIN_2, GPIO_DIR_IN};
LOCAL volatile bool hw_cmd_enable = FALSE;
/* Static functions ----------------------------------------------------------*/
LOCAL void it_callback(hal_i2c_channel_t chid, hal_i2c_status_t status);
/**
 * @brief Test I2C hall driver
 * @detail Test application for testing hal_i2c_master_tx API
 * @parameter[in] :
 * 			- Operation mode
 * 			- Address mode
 * 			- Slave address
 * 			- I2C clock speed
 * 			- I2C hardware channel
 * 			- ID command
 * 			- Data transmit
 * 			- Time out
 * 			- Configure interrupt
 * @parameter[out] NA
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_hal_i2c_001(char** argv, int argc);

/**
 * @brief Test I2C hall driver
 * @detail Test application for testing hal_i2c_slave_read API
 * @parameter[in] :
 * 			- Operation mode
 * 			- Address mode
 * 			- Slave address
 * 			- I2C clock speed
 * 			- I2C hardware channel
 * 			- ID command
 * 			- Data transmit
 * 			- Time out
 * 			- Configure interrupt
 * @parameter[out] NA
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_hal_i2c_002(char** argv, int argc);

/**
 * @brief Test I2C hall driver
 * @detail Test application for testing hal_i2c_slave_write API
 * @parameter[in] :
 * 			- Operation mode
 * 			- Address mode
 * 			- Slave address
 * 			- I2C clock speed
 * 			- I2C hardware channel
 * 			- ID command
 * 			- Data transmit
 * 			- Time out
 * 			- Configure interrupt
 * @parameter[out] NA
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_hal_i2c_003(char** argv, int argc);

/**
 * @brief Test I2C hall driver
 * @detail Test application for testing hal_i2c_master_rx API
 * @parameter[in] :
 * 			- Operation mode
 * 			- Address mode
 * 			- Slave address
 * 			- I2C clock speed
 * 			- I2C hardware channel
 * 			- ID command
 * 			- Data transmit
 * 			- Time out
 * 			- Configure interrupt
 * @parameter[out] NA
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_hal_i2c_004(char** argv, int argc);

/**
 * @brief Test I2C hall driver
 * @detail Test application write/read configure temperature sensor
 * @parameter[in]
 * 			- Temperature address
 * 			- Clock speed
 * 			- Register address
 * 			- 16 bit Data
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_hal_i2c_005(char** argv, int argc);

/**
 * @brief Test I2C hall driver
 * @detail Test application read value temperature sensor
 * @parameter[in]
 * 			- Temperature address
 * 			- Clock speed
 * @parameter[out] NA
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_hal_i2c_006(char** argv, int argc);

/**
 * @brief Test I2C hall driver
 * @detail Test API hal_i2c_init() using parameter invalid
 * @parameter[in]
 * 			- Operation mode
 * 			- Address mode
 * 			- Slave address
 * 			- I2C clock speed
 * 			- I2C hardware channel
 * @parameter[out] NA
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_hal_i2c_007(char** argv, int argc);

/**
 * @brief Test I2C hall driver
 * @detail Test API  using parameter invalid
 * @parameter[in]
 * 			- Operation mode
 * 			- Address mode
 * 			- Slave address
 * 			- I2C clock speed
 * 			- I2C hardware channel
 * @parameter[out] NA
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_hal_i2c_008(char** argv, int argc);

/**
 * @brief Test I2C hall driver
 * @detail Test Performance API
 * @parameter[in]
 * 			- Operation mode
 * 			- Address mode
 * 			- Slave address
 * 			- I2C clock speed
 * 			- I2C hardware channel
 * 			- Select API
 * @parameter[out] NA
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_hal_i2c_009(char** argv, int argc);
/**
 * @brief identify hardware command mode or generate hardware command mode
 * @param[in] cmd  : cmd mode (request or respond)
 * @param[out] NA
 * @return  0   : inactive CMD
 *          1   : activated CMD
 *          -1  : error
 * @Detail identify hardware command mode or generate hardware command mode
 *         this function is only available for test and module is testing at
 *         transmit only mode or receive only mode.
 */
LOCAL int it_hal_i2c_hw_cmd(i2c_hw_cmd_t cmd, cmd_t dir);

/* Exported variables --------------------------------------------------------*/
LOCAL it_map_t it_i2c_tests_table[] = {
		{"I2C_001", it_hal_i2c_001},
		{"I2C_002", it_hal_i2c_002},
		{"I2C_003", it_hal_i2c_003},
		{"I2C_004", it_hal_i2c_004},
		{"I2C_005", it_hal_i2c_005},
		{"I2C_006", it_hal_i2c_006},
		{"I2C_007", it_hal_i2c_007},
		{"I2C_008", it_hal_i2c_008},
		{"I2C_009", it_hal_i2c_009},
		{"",  NULL}
};
/* Exported functions --------------------------------------------------------*/

/**
 * @brief I2C module's testing handler
 * @detail to I2C module's testing handler
 * @parameter[in] argv	: arguments list
 * @parameter[in] argc	: argument counter
 * @return 0	 : success
 *         Others: instruction failed
 */
int it_hal_i2c_handler(char** argv, int argc)
{
	/**< Check command counter >*/
	if (1 >= argc) return -1;
	if (NULL == argv) return -1;
	/**< Get command index, mean test case name >*/
	int index = handler_parser(*argv, it_i2c_tests_table);
	if (-1 != index)
	{
		return it_i2c_tests_table[index].handler(&argv[1], argc - 1);
	}
	else
		return index;
}

/**
 * @brief I2C module's testing callback handler
 * @parameter[in] argv	: arguments list
 * @parameter[in] argc	: argument counter
 * @return 0	 NA
 * @detail I2C module's testing callback handler
 */
LOCAL void it_callback(hal_i2c_channel_t chid, hal_i2c_status_t status)
{
	callback_index++;
}
/**
 * @brief Test I2C hall driver
 * @detail Test application for testing hal_i2c_master_tx API
 * @parameter[in] :
 * 			- Operation mode
 * 			- Address mode
 * 			- Slave address
 * 			- I2C clock speed
 * 			- I2C hardware channel
 * 			- ID command
 * 			- Data transmit
 * 			- Time out
 * 			- Configure interrupt
 * @parameter[out] NA
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_hal_i2c_001(char** argv, int argc)
{
	/**! Check command parameter */
	if (9 != argc)
	{
		log_printf("Error: Wrong number of parameters.\r\n");
		return -1;
	}

	/**! Check argument */
	if (NULL == argv)
	{
		log_printf("Error: Empty argument.\r\n");
		return -1;
	}

	/**! Device default configuration */
	hal_i2c_t hal_i2c = I2C_DEFAULT_CFG;

	/**! Configure Input parameter */
	hal_i2c_buffer_t buffer;
	volatile uint32_t time_out;
	uint8_t call_back_enable;

	/**! Buffer configuration */
	buffer.bytes  = NULL;
	buffer.length = ZERO;

	/**! Get I2C mode number via argv[0] */
	hal_i2c.operation_mode = strtol(argv[0], NULL, 10);
	/**! Get Address mode via argv[1] */
	hal_i2c.address_mode   = strtol(argv[1], NULL, 10);
	/**! Get Address slave via argv[2] */
	hal_i2c.owner_addr     = strtol(argv[2], NULL, 16);
	/**! Get Clock mode via argv[3] */
	hal_i2c.clock_speed    = strtol(argv[3], NULL, 10);
	/**! Get i2c channel via argv[4] */
	hal_i2c.chid           = strtol(argv[4], NULL, 10);
	/**! Get CMD */
	uint16_t cmd_len       = strlen(argv[5]);
	/**! Get data length */
	uint16_t length        = strtol(argv[6], NULL, 10);
	char* data             = malloc(length + cmd_len);
	/**! Get time out via argv[7] */
	time_out               = strtol(argv[7], NULL, 10);
	/**! Get control interrupt via argv[8] */
	call_back_enable       = strtol(argv[8], NULL, 10);

	if(hal_i2c.operation_mode > I2C_SLAVE)
	{
		log_printf("Error: Do not support operation_mode I2C.\r\n");
		return -1;
	}

	if(hal_i2c.address_mode > I2C_10BIT)
	{
		log_printf("Error: Do not support address mode .\r\n");
		return -1;
	}

	/**! Check Valid clock input */
	if((hal_i2c.clock_speed != I2C_SPEED_100KHz) &&
		(hal_i2c.clock_speed != I2C_SPEED_400KHz))
	{
		log_printf("Error: Do not support clock speed.\r\n");
		return -1;
	}

	/**! Verify channel ID */
	if(I2C_CH_MAX_IDX <= hal_i2c.chid)
	{
		log_printf("Error: Do not support I2C channel.\r\n");
		return -1;
	}

	/**! Verify callback setting */
	if(1 < call_back_enable)
	{
		log_printf("Error: Interrupt Invalid.\r\n");
		return -1;
	}

	/**! Verify data */
	if (NULL == data)
	{
		log_printf("Error malloc() for data \r\n");
		return -1;
	}
	else
	{
		/**! Get command frame */
		memcpy(data, argv[5], cmd_len);
		/**! Set all data */
		memset(&data[cmd_len], 0x00, length);
		uint16_t index = 0;
		for (index = 0; index < length; index++)
		{
			data[cmd_len + index] = index;
		}
	}

	/**! Re-calculate timeout */
	time_out *= 50000000;

	/**! Configuration callback */
	if(call_back_enable)
	{
		hal_i2c.irq_handler = it_callback;
		callback_index = 0;
	}

	/**! Data configuration */
	buffer.length = cmd_len + length;
	buffer.bytes = (uint8_t *)data;

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initialize I2C module */
	hal_i2c_init(&hal_i2c);

	/**! Configure interrupt I2C */
	hal_i2c_enable_irq(&hal_i2c);

	/**! Waiting for hardware CMD from HOST via HW CMD pin */
	it_hal_i2c_hw_cmd(CMD_REQUEST, CMD_POSITIVE);

	/**! Send data to slave */
	qc_assert(I2C_OK == hal_i2c_master_tx(hal_i2c.chid, \
			                      hal_i2c.owner_addr, &buffer));

	/**! Check transmit I2C module*/
	while ((I2C_TX_COMPLETED != hal_i2c_get_status(hal_i2c.chid)) \
		                                          && (--time_out));
	/**! Assert timeout */
	qc_assert(time_out);

	/**! Assert callback status */
	qc_assert(!(call_back_enable ^ (0 != callback_index)));

	/**! Reset HW CMD */
	it_hal_i2c_hw_cmd(CMD_REQUEST, CMD_NEGATIVE);

	/**! Free I2C buffer */
	free(data);

	/**! Do judgment */
	qc_report();

	return I2C_OK;
}

/**
 * @brief Test I2C hall driver
 * @detail Test application for testing hal_i2c_slave_read API
 * @parameter[in] :
 * 			- Operation mode
 * 			- Address mode
 * 			- Slave address
 * 			- I2C clock speed
 * 			- I2C hardware channel
 * 			- ID command
 * 			- Data transmit
 * 			- Time out
 * 			- Configure interrupt
 * @parameter[out] NA
  * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_hal_i2c_002(char** argv, int argc)
{
	/**! Check command parameter */
	if (9 != argc)
	{
		log_printf("Error: Wrong number of parameters.\r\n");
		return -1;
	}

	/**! Check argument */
	if (NULL == argv)
	{
		log_printf("Error: Empty argument.\r\n");
		return -1;
	}

	/**! Device default configuration */
	hal_i2c_t hal_i2c = I2C_DEFAULT_CFG;

	/**! Configure Input parameter */
	hal_i2c_buffer_t buffer;
	volatile uint32_t time_out;
	uint8_t call_back_enable;

	/**! Buffer configuration */
	buffer.bytes  = NULL;
	buffer.length = ZERO;

	/**! Get I2C mode number via argv[0] */
	hal_i2c.operation_mode = strtol(argv[0], NULL, 10);
	/**! Get Address mode via argv[1] */
	hal_i2c.address_mode   = strtol(argv[1], NULL, 10);
	/**! Get Address slave via argv[2] */
	hal_i2c.owner_addr     = strtol(argv[2], NULL, 16);
	/**! Get Clock mode via argv[3] */
	hal_i2c.clock_speed    = strtol(argv[3], NULL, 10);
	/**! Get i2c channel via argv[4] */
	hal_i2c.chid           = strtol(argv[4], NULL, 10);
	/**! Get CMD */
	uint16_t cmd_len       = strlen(argv[5]);
	/**! Get data length */
	uint16_t length        = strtol(argv[6], NULL, 10);
	char* data             = malloc(length + cmd_len);
	/**! Get time out via argv[7] */
	time_out               = strtol(argv[7], NULL, 10);
	/**! Get control interrupt via argv[8] */
	call_back_enable       = strtol(argv[8], NULL, 10);

	if(hal_i2c.operation_mode > I2C_SLAVE)
	{
		log_printf("Error: Do not support operation_mode I2C.\r\n");
		return -1;
	}

	if(hal_i2c.address_mode > I2C_10BIT)
	{
		log_printf("Error: Do not support address mode .\r\n");
		return -1;
	}

	/**! Check Valid clock input */
	if((hal_i2c.clock_speed != I2C_SPEED_100KHz) &&
		(hal_i2c.clock_speed != I2C_SPEED_400KHz))
	{
		log_printf("Error: Do not support clock speed.\r\n");
		return -1;
	}

	/**! Verify channel ID */
	if(I2C_CH_MAX_IDX <= hal_i2c.chid)
	{
		log_printf("Error: Do not support I2C channel.\r\n");
		return -1;
	}

	/**! Verify callback setting */
	if(1 < call_back_enable)
	{
		log_printf("Error: Interrupt Invalid.\r\n");
		return -1;
	}

	/**! Verify data */
	if (NULL == data)
	{
		log_printf("Error malloc() for data \r\n");
		return -1;
	}
	else
	{
		/**! Set data default all zeros */
		memset(data, 0x00, length + cmd_len);
	}

	/**! Re-calculate timeout */
	time_out *= 50000000;

	/**! Configuration callback */
	if(call_back_enable)
	{
		hal_i2c.irq_handler = it_callback;
		callback_index = 0;
	}

	/**! Data configuration */
	buffer.length = cmd_len + length;
	buffer.bytes = (uint8_t *)data;

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initialize I2C module */
	hal_i2c_init(&hal_i2c);

	/**! Configure interrupt I2C */
	hal_i2c_enable_irq(&hal_i2c);

	/**! Waiting for CMD in multiple QC scenarios */
	while (I2C_OK != it_hal_i2c_hw_cmd(CMD_RESPOND, CMD_POSITIVE));

	/**! Check status receive I2C module*/
	while((I2C_RX_RECEIVING == hal_i2c_get_status(hal_i2c.chid)) \
			                                     && (--time_out));
	/**! Request read from slave */
	qc_assert(I2C_OK == hal_i2c_slave_rx(hal_i2c.chid, &buffer));

	/**! Verify status */
	qc_assert(I2C_RX_COMPLETED == hal_i2c_get_status(hal_i2c.chid));

	/**! Assert callback status */
	qc_assert(!(call_back_enable ^ (0 != callback_index)));

	/**! Check data receive */
	char* ptr = argv[5];

	/**! Verify data */
	for(int i = 0; i < buffer.length; i++)
	{
		if (((i < cmd_len) && (buffer.bytes[i] != ptr[i]))
		|| ((i >= cmd_len) && (buffer.bytes[i] != i)))
		{
			qc_assert(FALSE);
			break;
		}
	}

	/**! Free I2C buffer */
	free(data);

	/**! Do judgment */
	qc_report();

	return I2C_OK;
}

/**
 * @brief Test I2C hall driver
 * @detail Test application for testing hal_i2c_slave_write API
 * @parameter[in] :
 * 			- Operation mode
 * 			- Address mode
 * 			- Slave address
 * 			- I2C clock speed
 * 			- I2C hardware channel
 * 			- ID command
 * 			- Data transmit
 * 			- Time out
 * 			- Configure interrupt
 * @parameter[out] NA
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_hal_i2c_003(char** argv, int argc)
{
	/**! Check command parameter */
	if (9 != argc)
	{
		log_printf("Error: Wrong number of parameters.\r\n");
		return -1;
	}

	/**! Check argument */
	if (NULL == argv)
	{
		log_printf("Error: Empty argument.\r\n");
		return -1;
	}

	/**! Device default configuration */
	hal_i2c_t hal_i2c = I2C_DEFAULT_CFG;

	/**! Configure Input parameter */
	hal_i2c_buffer_t buffer = {NULL, ZERO};
	hal_i2c_buffer_t cmd    = {NULL, ZERO};

	volatile uint32_t time_out;
	volatile uint32_t time;
	uint8_t call_back_enable;

	/**! Get I2C mode number via argv[0] */
	hal_i2c.operation_mode = strtol(argv[0], NULL, 10);
	/**! Get Address mode via argv[1] */
	hal_i2c.address_mode   = strtol(argv[1], NULL, 10);
	/**! Get Address slave via argv[2] */
	hal_i2c.owner_addr     = strtol(argv[2], NULL, 16);
	/**! Get Clock mode via argv[3] */
	hal_i2c.clock_speed    = strtol(argv[3], NULL, 10);
	/**! Get i2c channel via argv[4] */
	hal_i2c.chid           = strtol(argv[4], NULL, 10);
	/**! Get CMD */
	uint16_t cmd_len       = strlen(argv[5]);
	/**! Get data length */
	uint16_t length        = strtol(argv[6], NULL, 10);
	char* data             = malloc(length + cmd_len);
	/**! Get time out via argv[7] */
	time_out               = strtol(argv[7], NULL, 10);
	/**! Get control interrupt via argv[8] */
	call_back_enable       = strtol(argv[8], NULL, 10);

	if(hal_i2c.operation_mode > I2C_SLAVE)
	{
		log_printf("Error: Do not support operation_mode I2C.\r\n");
		return -1;
	}

	if(hal_i2c.address_mode > I2C_10BIT)
	{
		log_printf("Error: Do not support address mode .\r\n");
		return -1;
	}

	/**! Check Valid clock input */
	if((hal_i2c.clock_speed != I2C_SPEED_100KHz) &&
		(hal_i2c.clock_speed != I2C_SPEED_400KHz))
	{
		log_printf("Error: Do not support clock speed.\r\n");
		return -1;
	}

	/**! Verify channel ID */
	if(I2C_CH_MAX_IDX <= hal_i2c.chid)
	{
		log_printf("Error: Do not support I2C channel.\r\n");
		return -1;
	}

	/**! Verify callback setting */
	if(1 < call_back_enable)
	{
		log_printf("Error: Interrupt Invalid.\r\n");
		return -1;
	}

	/**! Verify data */
	if (NULL == data)
	{
		log_printf("Error malloc() for data \r\n");
		return -1;
	}
	else
	{
		/**! Set data default all zeros */
		memset(data, 0x00, length + cmd_len);
	}

	/**! Re-calculate timeout */
	time_out *= 50000000;

	/**! Configuration callback */
	if(call_back_enable)
	{
		hal_i2c.irq_handler = it_callback;
		callback_index = 0;
	}

	/**! CMD configuration */
	cmd.length = cmd_len;
	cmd.bytes  = (uint8_t*)data;

	/**! Data configuration */
	buffer.length = length;
	buffer.bytes = (uint8_t *)&data[cmd_len];

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initialize I2C module */
	hal_i2c_init(&hal_i2c);

	/**! Configure interrupt I2C */
	hal_i2c_enable_irq(&hal_i2c);

	/**! Waiting for CMD in multiple QC scenarios */
	while (I2C_OK != it_hal_i2c_hw_cmd(CMD_RESPOND, CMD_POSITIVE));

	/**! Request read from slave */
	qc_assert(I2C_OK == hal_i2c_slave_rx(hal_i2c.chid, &cmd));

	/**! Check status receive I2C module*/
	time = time_out;
	while((I2C_RX_COMPLETED == hal_i2c_get_status(hal_i2c.chid)) && (--time));

	/**! Assert timeout */
	qc_assert(time);

	/**! Check status receive I2C module*/
	time = time_out;
	while((I2C_READ_REQUESTED == hal_i2c_get_status(hal_i2c.chid)) && (--time));

	/**! Send from slave */
	qc_assert(I2C_OK == hal_i2c_slave_tx(hal_i2c.chid, &buffer));

	/**! Check status */
	qc_assert(I2C_RX_COMPLETED == hal_i2c_get_status(hal_i2c.chid));

	/**! Assert timeout */
	qc_assert(time);

	/**! Assert callback status */
	qc_assert(!(call_back_enable ^ (0 != callback_index)));

	/**! Check data receive */
	char* ptr = argv[5];

	/**! Verify data */
	for(int i = 0; i < buffer.length; i++)
	{
		if (((i < cmd_len) && (buffer.bytes[i] != ptr[i]))
		|| ((i >= cmd_len) && (buffer.bytes[i] != i)))
		{
			qc_assert(FALSE);
			break;
		}
	}

	/**! Free I2C buffer */
	free(data);

	/**! Do judgment */
	qc_report();

	return I2C_OK;
}

/**
 * @brief Test I2C hall driver
 * @detail Test application for testing hal_i2c_master_rx API
 * @parameter[in] :
 * 			- Operation mode
 * 			- Address mode
 * 			- Slave address
 * 			- I2C clock speed
 * 			- I2C hardware channel
 * 			- ID command
 * 			- Data transmit
 * 			- Time out
 * 			- Configure interrupt
 * @parameter[out] NA
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_hal_i2c_004(char** argv, int argc)
{
	/**! Check command parameter */
	if (9 != argc)
	{
		log_printf("Error: Wrong number of parameters.\r\n");
		return -1;
	}

	/**! Check argument */
	if (NULL == argv)
	{
		log_printf("Error: Empty argument.\r\n");
		return -1;
	}

	/**! Device default configuration */
	hal_i2c_t hal_i2c = I2C_DEFAULT_CFG;

	/**! Configure Input parameter */
	hal_i2c_buffer_t buffer = {NULL, ZERO};
	hal_i2c_buffer_t cmd    = {NULL, ZERO};

	volatile uint32_t time_out;
	uint8_t call_back_enable;

	/**! Get I2C mode number via argv[0] */
	hal_i2c.operation_mode = strtol(argv[0], NULL, 10);
	/**! Get Address mode via argv[1] */
	hal_i2c.address_mode   = strtol(argv[1], NULL, 10);
	/**! Get Address slave via argv[2] */
	hal_i2c.owner_addr     = strtol(argv[2], NULL, 16);
	/**! Get Clock mode via argv[3] */
	hal_i2c.clock_speed    = strtol(argv[3], NULL, 10);
	/**! Get i2c channel via argv[4] */
	hal_i2c.chid           = strtol(argv[4], NULL, 10);
	/**! Get CMD */
	uint16_t cmd_len       = strlen(argv[5]);
	/**! Get data length */
	uint16_t length        = strtol(argv[6], NULL, 10);
	char* data             = malloc(length + cmd_len);
	/**! Get time out via argv[7] */
	time_out               = strtol(argv[7], NULL, 10);
	/**! Get control interrupt via argv[8] */
	call_back_enable       = strtol(argv[8], NULL, 10);

	if(hal_i2c.operation_mode > I2C_SLAVE)
	{
		log_printf("Error: Do not support operation_mode I2C.\r\n");
		return -1;
	}

	if(hal_i2c.address_mode > I2C_10BIT)
	{
		log_printf("Error: Do not support address mode .\r\n");
		return -1;
	}

	/**! Check Valid clock input */
	if((hal_i2c.clock_speed != I2C_SPEED_100KHz) &&
		(hal_i2c.clock_speed != I2C_SPEED_400KHz))
	{
		log_printf("Error: Do not support clock speed.\r\n");
		return -1;
	}

	/**! Verify channel ID */
	if(I2C_CH_MAX_IDX <= hal_i2c.chid)
	{
		log_printf("Error: Do not support I2C channel.\r\n");
		return -1;
	}

	/**! Verify callback setting */
	if(1 < call_back_enable)
	{
		log_printf("Error: Interrupt Invalid.\r\n");
		return -1;
	}

	/**! Verify data */
	if (NULL == data)
	{
		log_printf("Error malloc() for data \r\n");
		return -1;
	}
	else
	{
		/**! Get command frame */
		memcpy(data, argv[5], cmd_len);
		/**! reset all data */
		memset(&data[cmd_len], 0x00, length);
		/**! Set CMD */
		cmd.bytes = (uint8_t*)data;
		cmd.length = cmd_len;
		/**! Data configuration */
		buffer.length = length;
		buffer.bytes = (uint8_t *)&data[cmd_len];
	}

	/**! Re-calculate timeout */
	time_out *= 50000000;

	/**! Configuration callback */
	if(call_back_enable)
	{
		hal_i2c.irq_handler = it_callback;
		callback_index = 0;
	}

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initialize I2C module */
	hal_i2c_init(&hal_i2c);

	/**! Configure interrupt I2C */
	hal_i2c_enable_irq(&hal_i2c);

	/**! Waiting for hardware CMD from HOST via HW CMD pin */
	it_hal_i2c_hw_cmd(CMD_REQUEST, CMD_POSITIVE);

	/**! Transfer command */
	qc_assert(I2C_OK == hal_i2c_master_tx(hal_i2c.chid, \
			                             hal_i2c.owner_addr, &cmd));
	/**! Verify complete */
	while((I2C_TX_COMPLETED == hal_i2c_get_status(hal_i2c.chid)) \
			                                     && (--time_out));
	/**! Assert timeout */
	qc_assert(0 < time_out);

	/**! Read data */
	qc_assert((I2C_OK == hal_i2c_master_rx(hal_i2c.chid, \
			                         hal_i2c.owner_addr, &buffer)));
	/**! Verify complete */
	while((I2C_RX_COMPLETED == hal_i2c_get_status(hal_i2c.chid)) \
			                                     && (--time_out));
	/**! Assert timeout */
	qc_assert(0 < time_out);

	/**! Assert callback status */
	qc_assert(!(call_back_enable ^ (0 != callback_index)));

	/**! Check data receive from slave */
	for(int i = 0; i < buffer.length; i++)
	{
		if (buffer.bytes[i] != i)
		{
			qc_assert(FALSE);
			break;
		}
	}

	/**! Free I2C buffer */
	free(data);

	/**! Do judgment */
	qc_report();

	return I2C_OK;
}

/**
 * @brief Test I2C hall driver
 * @detail Test application write/read configure temperature sensor
 * @parameter[in]
 * 			- Temperature address
 * 			- Clock speed
 * 			- Register address
 * 			- 16 bit Data
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_hal_i2c_005(char** argv, int argc)
{

	/**! Check command parameter */
	if (4 != argc)
	{
		log_printf("Error: Wrong number of parameters\r\n");
		return -1;
	}

	/**! Check argument */
	if (NULL == argv)
	{
		log_printf("Error: Empty argument\r\n");
		return -1;
	}

	/**! Buffer preparation */
	hal_i2c_t p_i2c_obj;
	hal_i2c_buffer_t tx_buffer;
	hal_i2c_buffer_t rx_buffer;
	volatile uint32_t time_out = 20000000;
	uint16_t data = 0;
	char* ePtr = NULL;
	uint8_t temp_cfg[I2C_TEMP_SENSOR_CMD_LENGTH + I2C_TEMP_SENSOR_DATA_LENGTH];
	/**< Configure I2C module >*/
	p_i2c_obj.address_mode   = I2C_7BIT;
	p_i2c_obj.clock_speed    = I2C_SPEED_100KHz;
	p_i2c_obj.operation_mode = I2C_MASTER;
	p_i2c_obj.chid           = I2C_CH18;
	p_i2c_obj.irq_handler    = NULL;

	/**! Buffer configuration */
	tx_buffer.length = I2C_TEMP_SENSOR_CMD_LENGTH + I2C_TEMP_SENSOR_DATA_LENGTH;
	tx_buffer.bytes  = NULL;
	rx_buffer.length = ZERO;
	rx_buffer.bytes  = NULL;

	/**! Get address temperature sensor via argv[0] */
	p_i2c_obj.owner_addr = (uint16_t)strtol(argv[0], NULL, 16);

	/**! Get clock speed via argv[1] */
	p_i2c_obj.clock_speed = (hal_i2c_clockspeed_t)strtol(argv[1], NULL, 10);
	/**! Check Valid clock input */
	if((p_i2c_obj.clock_speed != I2C_SPEED_100KHz) &&
		(p_i2c_obj.clock_speed != I2C_SPEED_400KHz))
	{
		log_printf("Error: Do not support clock speed.\r\n");
		return -1;
	}

	/**! Data configuration */
	temp_cfg[0] = (uint8_t)strtol(argv[2], NULL, 16);
	data = (uint16_t)strtol(argv[3], NULL, 16);
	temp_cfg[1] = (uint8_t)((data >> 8) & 0xFF);
	temp_cfg[2] = (uint8_t)(data & 0xFF);

	/**!  Reset all test point to default */
	qc_assert_reset();

	/**! Initialize I2C module */
	hal_i2c_init(&p_i2c_obj);

	/**! Initialize I2C interrupt */
	hal_i2c_enable_irq(&p_i2c_obj);

	/**! Send data configure to temperature sensor */
	tx_buffer.bytes = temp_cfg;

	/**! Write configuration information */
	qc_assert(I2C_OK == hal_i2c_master_tx(p_i2c_obj.chid, \
			                      p_i2c_obj.owner_addr, &tx_buffer));

	/**! Check transmit I2C module*/
	while ((I2C_TX_COMPLETED != hal_i2c_get_status(p_i2c_obj.chid)) \
		                                             && (--time_out));
	/**! Assert timeout */
	qc_assert(0 < time_out);

	/** Receive data configure from temperature sensor */
	tx_buffer.length = I2C_TEMP_SENSOR_CMD_LENGTH;
	tx_buffer.bytes  = &temp_cfg[0];
	rx_buffer.length = I2C_TEMP_SENSOR_DATA_LENGTH;
	rx_buffer.bytes  = &temp_cfg[1];

	rx_buffer.bytes[0] = 0;
	rx_buffer.bytes[1] = 0;

	time_out = 20000;
	/**! Wait time out */
	qc_assert(I2C_OK == hal_i2c_master_tx(p_i2c_obj.chid, \
			                    p_i2c_obj.owner_addr, &tx_buffer));

	time_out = 20000;
	/**! Check transmit I2C module*/
	while ((I2C_TX_COMPLETED != hal_i2c_get_status(p_i2c_obj.chid)) \
		                                             && (--time_out));
	/**! Assert timeout */
	qc_assert(0 < time_out);

	/**! Receive data register from temperature sensor */
	qc_assert(I2C_OK == hal_i2c_master_rx(p_i2c_obj.chid, \
			               p_i2c_obj.owner_addr, &rx_buffer));

	time_out = 20000;
	/**! Check transmit I2C module*/
	while ((I2C_RX_COMPLETED != hal_i2c_get_status(p_i2c_obj.chid)) \
		                                             && (--time_out));
	/**! Assert timeout */
	qc_assert(0 < time_out);

	/**! Check data receive from slave */
	ePtr = (char*)&temp_cfg[1];
	qc_assert(data == ((*((uint16_t*)ePtr)) & I2C_TEMP_CONFIGURABLE_MASK));

	/**! Report */
	qc_report();

	return I2C_OK;
}

/**
 * @brief Test I2C hall driver
 * @detail Test application read value temperature sensor
 * @parameter[in]
 * 			- Temperature address
 * 			- Clock speed
 * @parameter[out] NA
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_hal_i2c_006(char** argv, int argc)
{

	/**! Check command parameter */
	if (2 != argc)
	{
		log_printf("Error: Wrong number of parameters\r\n");
		return -1;
	}

	/**! Check argument */
	if (NULL == argv)
	{
		log_printf("Error: Empty argument\r\n");
		return -1;
	}

	/**! Buffer preparation */
	hal_i2c_t p_i2c_obj;
	hal_i2c_buffer_t tx_buffer;
	hal_i2c_buffer_t rx_buffer;
	uint8_t temp_cfg[I2C_TEMP_SENSOR_CMD_LENGTH + I2C_TEMP_SENSOR_DATA_LENGTH];
	volatile uint32_t time_out = 20000000;

	/**< Configure I2C module >*/
	p_i2c_obj.address_mode   = I2C_7BIT;
	p_i2c_obj.clock_speed    = I2C_SPEED_100KHz;
	p_i2c_obj.operation_mode = I2C_MASTER;
	p_i2c_obj.chid           = I2C_CH18;
	p_i2c_obj.irq_handler    = NULL;

	/**! Buffer configuration */
	tx_buffer.length = I2C_TEMP_SENSOR_CMD_LENGTH + I2C_TEMP_SENSOR_DATA_LENGTH;
	tx_buffer.bytes  = NULL;
	rx_buffer.length = I2C_TEMP_SENSOR_DATA_LENGTH;
	rx_buffer.bytes  = NULL;

	/**! Data configure temperature sensor normal mode */
	temp_cfg[0] = I2C_TEMP_CONFIG_REG;
	temp_cfg[1] = 0x79;
	temp_cfg[2] = 0xD0;

	/**! Get address temperature sensor via argv[0] */
	p_i2c_obj.owner_addr = (uint16_t)strtol(argv[0], NULL, 16);

	/**! Get clock speed via argv[1] */
	p_i2c_obj.clock_speed = (hal_i2c_clockspeed_t)strtol(argv[1], NULL, 10);

	/**! Check Valid clock input */
	if((p_i2c_obj.clock_speed != I2C_SPEED_100KHz) &&
		(p_i2c_obj.clock_speed != I2C_SPEED_400KHz))
	{
		log_printf("Error: Do not support clock speed.\r\n");
		return -1;
	}

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initialize I2C module */
	hal_i2c_init(&p_i2c_obj);

	/**! Initialize I2C interrupt */
	hal_i2c_enable_irq(&p_i2c_obj);

	/**! Write to configure of temperature sensor */
	tx_buffer.bytes = temp_cfg;

	/**! Wait time out */
	qc_assert(I2C_OK == hal_i2c_master_tx(p_i2c_obj.chid, \
			                       p_i2c_obj.owner_addr, &tx_buffer));

	/**! Check transmit I2C module*/
	while ((I2C_TX_COMPLETED != hal_i2c_get_status(p_i2c_obj.chid)) \
		                                             && (--time_out));
	/**! Assert timeout */
	qc_assert(0 < time_out);

	/**! Time wait parser temperature sensor */
	uint32_t time_wait = 5;
	while(--time_wait)
	{
		time_out = 200000;
		while(--time_out);
	}

	/**! Receive Data from temperature sensor */
	tx_buffer.length = I2C_TEMP_SENSOR_CMD_LENGTH;
	tx_buffer.bytes[0]  = I2C_TEMP_REG;

	/**! Wait time out */
	qc_assert(I2C_OK == hal_i2c_master_tx(p_i2c_obj.chid, \
			                      p_i2c_obj.owner_addr, &tx_buffer));

	/**! Check transmit I2C module*/
	time_out = 20000000;
	while ((I2C_TX_COMPLETED != hal_i2c_get_status(p_i2c_obj.chid)) \
		                                             && (--time_out));

	/**! Assert timeout */
	qc_assert(0 < time_out);

	/**! Receive data register from temperature sensor */
	rx_buffer.bytes = &temp_cfg[I2C_TEMP_SENSOR_CMD_LENGTH];
	rx_buffer.bytes[0] = 0;
	rx_buffer.bytes[1] = 0;
	qc_assert(I2C_OK == hal_i2c_master_rx(p_i2c_obj.chid, \
			                      p_i2c_obj.owner_addr, &rx_buffer));

	/**! Check transmit I2C module*/
	time_out = 20000000;
	while ((I2C_RX_COMPLETED != hal_i2c_get_status(p_i2c_obj.chid)) \
		                                             && (--time_out));

	/**! Assert timeout */
	qc_assert(0 < time_out);

	/**! Print result */
	uint16_t hex = I2C_TEMP_VAL(*((uint16_t*)rx_buffer.bytes));
	log_printf("Temperature = 0x%04x\r\n", hex);

	/**! Report */
	qc_report();

	return I2C_OK;
}

/**
 * @brief Test I2C hall driver
 * @detail Test API hal_i2c_init() using parameter invalid
 * @parameter[in]
 * 			- Operation mode
 * 			- Address mode
 * 			- Slave address
 * 			- I2C clock speed
 * 			- I2C hardware channel
 * @parameter[out] NA
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_hal_i2c_007(char** argv, int argc)
{
	/**! Check command parameter */
	if (5 != argc)
	{
		log_printf("Error: Wrong number of parameters.\r\n");
		return -1;
	}

	/**! Check argument */
	if (NULL == argv)
	{
		log_printf("Error: Empty argument.\r\n");
		return -1;
	}

	/**! Buffer preparation */
	hal_i2c_t p_i2c_obj;

	/**! Configure I2C module */
	p_i2c_obj.address_mode   = I2C_7BIT;
	p_i2c_obj.clock_speed    = I2C_SPEED_100KHz;
	p_i2c_obj.operation_mode = I2C_MASTER;
	p_i2c_obj.chid           = I2C_CH0;
	p_i2c_obj.owner_addr     = ZERO;
	p_i2c_obj.irq_handler    = NULL;

	/**! Get I2C mode number via argv[0] */
	p_i2c_obj.operation_mode =
						(hal_i2c_operation_mode_t)strtol(argv[0], NULL, 10);

	/**! Get Address mode via argv[1] */
	p_i2c_obj.address_mode = (hal_i2c_address_mode_t)strtol(argv[1], NULL, 10);

	/**! Get Address slave via argv[2] */
	p_i2c_obj.owner_addr = (uint16_t)strtol(argv[2], NULL, 16);

	/**! Get Clock mode via argv[3] */
	p_i2c_obj.clock_speed = (hal_i2c_clockspeed_t)strtol(argv[3], NULL, 10);

	/**! Get i2c channel via argv[4] */
	p_i2c_obj.chid = (hal_i2c_channel_t)strtol(argv[4], NULL, 10);

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initialize I2C module */
	hal_i2c_init(&p_i2c_obj);

	/**! Check assert error */
	qc_assert(qc_assert_status());

	qc_report();

	return I2C_OK;
}

/**
 * @brief Test I2C hall driver
 * @detail Test API  using parameter invalid
 * @parameter[in]
 * 			- Operation mode
 * 			- Address mode
 * 			- Slave address
 * 			- I2C clock speed
 * 			- I2C hardware channel
 * @parameter[out] NA
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_hal_i2c_008(char** argv, int argc)
{
	/**! Check command parameter */
	if (6 != argc)
	{
		log_printf("Error: Wrong number of parameters.\r\n");
		return -1;
	}

	/**! Check argument */
	if (NULL == argv)
	{
		log_printf("Error: Empty argument.\r\n");
		return -1;
	}

	/**! Buffer preparation */
	hal_i2c_t p_i2c_obj;
	uint8_t select;
	hal_i2c_buffer_t buf;
	/**! Configure I2C module */
	p_i2c_obj.address_mode   = I2C_7BIT;
	p_i2c_obj.clock_speed    = I2C_SPEED_100KHz;
	p_i2c_obj.operation_mode = I2C_MASTER;
	p_i2c_obj.chid           = I2C_CH0;
	p_i2c_obj.owner_addr     = ZERO;
	p_i2c_obj.irq_handler    = NULL;

	/**! Configure buffer data*/
	buf.bytes = 0x00;
	buf.length = 0x01;
	/**! Initialize I2C module */
	hal_i2c_init(&p_i2c_obj);

	/**! Get I2C mode number via argv[0] */
	p_i2c_obj.operation_mode =
						(hal_i2c_operation_mode_t)strtol(argv[0], NULL, 10);

	/**! Get Address mode via argv[1] */
	p_i2c_obj.address_mode = (hal_i2c_address_mode_t)strtol(argv[1], NULL, 10);

	/**! Get Address slave via argv[2] */
	p_i2c_obj.owner_addr = (uint16_t)strtol(argv[2], NULL, 16);

	/**! Get Clock mode via argv[3] */
	p_i2c_obj.clock_speed = (hal_i2c_clockspeed_t)strtol(argv[3], NULL, 10);

	/**! Get i2c channel via argv[4] */
	p_i2c_obj.chid = (hal_i2c_channel_t)strtol(argv[4], NULL, 10);

	/**! Get select API */
	select = (uint8_t)strtol(argv[5], NULL, 10);
	if(3 < select)
	{
		log_printf(" Error: Do not support API.\r\n");
		return -1;
	}
	/**! Reset all test point to default */
	qc_assert_reset();

	switch(select)
	{
		case 0:
		{
			/**! Check API hal_i2c_master_tx() */
			hal_i2c_master_tx(p_i2c_obj.chid, p_i2c_obj.owner_addr, &buf);
			break;
		}
		case 1:
		{
			/**! Check API hal_i2c_master_rx() */
			hal_i2c_master_rx(p_i2c_obj.chid, p_i2c_obj.owner_addr, &buf);
			break;
		}
		case 2:
		{
			/**! Check API hal_i2c_slave_tx() */
			hal_i2c_slave_tx(p_i2c_obj.chid, &buf);
			break;
		}
		case 3:
		{
			/**! Check API hal_i2c_slave_rx() */
			hal_i2c_slave_rx(p_i2c_obj.chid, &buf);
			break;
		}
		default:
		{
			break;
		}
	}
	/**! Check assert error */
	qc_assert(qc_assert_status());

	qc_report();

	return I2C_OK;
}

/**
 * @brief Test I2C hall driver
 * @detail Test Performance API
 * @parameter[in]
 * 			- Operation mode
 * 			- Address mode
 * 			- Slave address
 * 			- I2C clock speed
 * 			- I2C hardware channel
 * 			- Select API
 * @parameter[out] NA
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_hal_i2c_009(char** argv, int argc)
{
	/**! Check command parameter */
	if (3 != argc)
	{
		log_printf("Error: Wrong number of parameters\r\n");
		return -1;
	}

	/**! Check argument */
	if (NULL == argv)
	{
		log_printf("Error: Empty argument\r\n");
		return -1;
	}

	/**! Buffer preparation */
	hal_i2c_t p_i2c_obj;
	hal_i2c_buffer_t tx_buffer;
	hal_i2c_buffer_t rx_buffer;
	uint8_t temp_cfg[I2C_TEMP_SENSOR_CMD_LENGTH + I2C_TEMP_SENSOR_DATA_LENGTH];
	volatile uint32_t time_out = 20000000;
	uint8_t select = 0;
	hal_gpio_t gpio = {0, 0, 1};
	/**< Configure I2C module >*/
	p_i2c_obj.address_mode   = I2C_7BIT;
	p_i2c_obj.clock_speed    = I2C_SPEED_100KHz;
	p_i2c_obj.operation_mode = I2C_MASTER;
	p_i2c_obj.chid           = I2C_CH18;
	p_i2c_obj.irq_handler    = NULL;

	/**! Buffer configuration */
	tx_buffer.length = I2C_TEMP_SENSOR_CMD_LENGTH + I2C_TEMP_SENSOR_DATA_LENGTH;
	tx_buffer.bytes  = NULL;
	rx_buffer.length = I2C_TEMP_SENSOR_DATA_LENGTH;
	rx_buffer.bytes  = NULL;

	/**! Data configure temperature sensor normal mode */
	temp_cfg[0] = I2C_TEMP_CONFIG_REG;
	temp_cfg[1] = 0x79;
	temp_cfg[2] = 0xD0;

	/**! Get address temperature sensor via argv[0] */
	p_i2c_obj.owner_addr = (uint16_t)strtol(argv[0], NULL, 16);

	/**! Get clock speed via argv[1] */
	p_i2c_obj.clock_speed = (hal_i2c_clockspeed_t)strtol(argv[1], NULL, 10);

	/**! Get select API */
	select = (uint8_t)strtol(argv[2], NULL, 10);

	/**! Check Valid clock input */
	if((p_i2c_obj.clock_speed != I2C_SPEED_100KHz) &&
		(p_i2c_obj.clock_speed != I2C_SPEED_400KHz))
	{
		log_printf("Error: Do not support clock speed.\r\n");
		return -1;
	}

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initialize GPIO */
	hal_gpio_init(&gpio);

	if(0 == select)
	{
		hal_gpio_set_high(&gpio);
	}
	/**! Initialize I2C module */
	hal_i2c_init(&p_i2c_obj);
	if(0 == select)
	{
		hal_gpio_set_low(&gpio);
	}
	/**! Initialize I2C interrupt */
	hal_i2c_enable_irq(&p_i2c_obj);

	/**! Write to configure of temperature sensor */
	tx_buffer.bytes = temp_cfg;

	if(1 == select)
	{
		hal_gpio_set_high(&gpio);
	}
	/**! Wait time out */
	qc_assert(I2C_OK == hal_i2c_master_tx(p_i2c_obj.chid, \
			                       p_i2c_obj.owner_addr, &tx_buffer));
	if(1 == select)
	{
		hal_gpio_set_low(&gpio);
	}

	if(2 == select)
	{
		hal_gpio_set_high(&gpio);
	}
	hal_i2c_get_status(p_i2c_obj.chid);
	if(2 == select)
	{
		hal_gpio_set_low(&gpio);
	}

	/**! Check transmit I2C module*/
	while ((I2C_TX_COMPLETED != hal_i2c_get_status(p_i2c_obj.chid)) \
		                                             && (--time_out));
	/**! Assert timeout */
	qc_assert(0 < time_out);

	/**! Time wait parser temperature sensor */
	uint32_t time_wait = 5;
	while(--time_wait)
	{
		time_out = 200000;
		while(--time_out);
	}

	/**! Receive Data from temperature sensor */
	tx_buffer.length = I2C_TEMP_SENSOR_CMD_LENGTH;
	tx_buffer.bytes[0]  = I2C_TEMP_REG;

	if(3 == select)
	{
		hal_gpio_set_high(&gpio);
	}
	/**! Wait time out */
	qc_assert(I2C_OK == hal_i2c_master_tx(p_i2c_obj.chid, \
			                      p_i2c_obj.owner_addr, &tx_buffer));

	if(3 == select)
	{
		hal_gpio_set_low(&gpio);
	}
	/**! Check transmit I2C module*/
	time_out = 20000000;

	if(4 == select)
	{
		hal_gpio_set_high(&gpio);
	}
	hal_i2c_get_status(p_i2c_obj.chid);
	if(4 == select)
	{
		hal_gpio_set_low(&gpio);
	}
	while ((I2C_TX_COMPLETED != hal_i2c_get_status(p_i2c_obj.chid)) \
		                                             && (--time_out));

	/**! Assert timeout */
	qc_assert(0 < time_out);

	/**! Receive data register from temperature sensor */
	rx_buffer.bytes = &temp_cfg[I2C_TEMP_SENSOR_CMD_LENGTH];
	rx_buffer.bytes[0] = 0;
	rx_buffer.bytes[1] = 0;

	if(5 == select)
	{
		hal_gpio_set_high(&gpio);
	}
	qc_assert(I2C_OK == hal_i2c_master_rx(p_i2c_obj.chid, \
			                      p_i2c_obj.owner_addr, &rx_buffer));
	if(5 == select)
	{
		hal_gpio_set_low(&gpio);
	}
	/**! Check transmit I2C module*/
	time_out = 20000000;

	if(6 == select)
	{
		hal_gpio_set_high(&gpio);
	}
	hal_i2c_get_status(p_i2c_obj.chid);
	if(6 == select)
	{
		hal_gpio_set_low(&gpio);
	}

	while ((I2C_RX_COMPLETED != hal_i2c_get_status(p_i2c_obj.chid)) \
		                                             && (--time_out));
	/**! Assert timeout */
	qc_assert(0 < time_out);

	/**! Print result */
	uint16_t hex = I2C_TEMP_VAL(*((uint16_t*)rx_buffer.bytes));
	log_printf("Temperature = 0x%04x\r\n", hex);

	/**! Report */
	qc_report();

	return I2C_OK;
}
/**
 * @brief identify hardware command mode or generate hardware command mode
 * @param[in] cmd  : cmd mode (request or respond)
 * @param[out] NA
 * @return  0   : inactive CMD
 *          1   : activated CMD
 *          -1  : error
 * @Detail identify hardware command mode or generate hardware command mode
 *         this function is only available for test and module is testing at
 *         transmit only mode or receive only mode.
 */
LOCAL int it_hal_i2c_hw_cmd(i2c_hw_cmd_t cmd, cmd_t dir)
{
    if (CMD_REQUEST == cmd) /**! Request */
    {
        /**! Provide high level output to trigger slave */
    	if ((GPIO_DIR_IN == hw_cmd_gpio.direction) || (!hw_cmd_enable))
    	{
    		/**! Set direction as output */
    		hw_cmd_gpio.direction = GPIO_DIR_OUT;

    		/**! Force set HW as output pin */
            hal_gpio_init(&hw_cmd_gpio);

            /**! Mark HW CMD as enabled */
            hw_cmd_enable = TRUE;
    	}

        /**! Set indicated pin as HIGH to start CMD */
    	if (dir == CMD_POSITIVE)
    		hal_gpio_set_high(&hw_cmd_gpio);
    	else
    		hal_gpio_set_low(&hw_cmd_gpio);

        return I2C_OK;
    }
    else                    /**! Respond */
    {
    	if ((GPIO_DIR_IN == hw_cmd_gpio.direction) || (!hw_cmd_enable))
    	{
            /**! Set direction as input */
            hw_cmd_gpio.direction = GPIO_DIR_IN;

            /**! Force set HW as input pin */
            hal_gpio_init(&hw_cmd_gpio);

            /**! Mark HW CMD as enabled */
            hw_cmd_enable = TRUE;
    	}

        /**! Verify input */
        if ((hal_gpio_level_t)dir == hal_gpio_read(&hw_cmd_gpio)) return I2C_OK;
    }
    /**! Return error */
    return -1;
}
/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
