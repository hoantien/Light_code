/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    it_drv_i2c_master.c
 * @author  The LightCo.
 * @version 1.0.0
 * @date    June, 3, 2016
 * @brief   
 *
 ******************************************************************************/

/******************************************************************************/
/**  						Revision history
 *
 * * 1.0.0	June, 3, 2016	Initial revision
 */
/******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdlib.h>
#include "it_drv_i2cm.h"
#include "qc_common.h"
#include "log.h"
#include "os.h"
#include "board_config.h"
/* Private define ------------------------------------------------------------*/
/**
 * Buffer information
 */
typedef struct
{
	uint8_t* tx_addr;
	size_t   tx_len;
	uint8_t* rx_addr;
	size_t   rx_len;
}i2cm_buffer_t;
/**!
 * Mapping with physical names on schematic
 */
typedef enum
{
	I2C_CAM_01         = I2C_CH0, //!< I2C_CAM_01
	I2C_CAM_02         = I2C_CH1, //!< I2C_CAM_02
	I2C_CAM_03         = I2C_CH2, //!< I2C_CAM_03
	I2C_CAM_04         = I2C_CH3, //!< I2C_CAM_04
	I2C_CAM_05         = I2C_CH4, //!< I2C_CAM_05
	I2C_CAM_06         = I2C_CH5, //!< I2C_CAM_06
	I2C_HALL_LENS_01   = I2C_CH6, //!< I2C_HALL_LENS_01
	I2C_HALL_LENS_02   = I2C_CH7, //!< I2C_HALL_LENS_02
	I2C_HALL_LENS_03   = I2C_CH8, //!< I2C_HALL_LENS_03
	I2C_FORWARD_ASIC2  = I2C_CH9, //!< I2C_FORWARD_ASIC2
	I2C_HALL_LENS_05   = I2C_CH10,//!< I2C_HALL_LENS_05
	I2C_HALL_LENS_06   = I2C_CH11,//!< I2C_HALL_LENS_06
	#ifdef P2_BOARD
	I2C_UNUSED_01      = I2C_CH12,//!< I2C_UNUSED_01
	#else
	I2C_HALL_MIRROR_01 = I2C_CH12,//!< I2C_HALL_MIRROR_01
	#endif /**/
	I2C_HALL_MIRROR_02 = I2C_CH13,//!< I2C_HALL_MIRROR_02
	I2C_HALL_MIRROR_03 = I2C_CH14,//!< I2C_HALL_MIRROR_03
	I2C_FORWARD_ASIC3  = I2C_CH15,//!< I2C_FORWARD_ASIC3
	#ifdef P2_BOARD
	I2C_UNUSED_02      = I2C_CH16,//!< I2C_UNUSED_02
	#else
	I2C_HALL_MIRROR_05 = I2C_CH16,//!< I2C_HALL_MIRROR_05
	#endif
	I2C_HALL_MIRROR_06 = I2C_CH17,//!< I2C_HALL_MIRROR_06
	I2C_SYS            = I2C_CH18 //!< I2C_SYS
}i2c_tunnel_t;
/**!
 * I2C master parameters
 */
typedef struct
{
	i2c_tunnel_t  tunnel  ;
	uint8_t       to      ;
	char*         name    ;
	task_handle_t handler ;
	i2cm_error_t  error   ;
	i2cm_buffer_t database;
}i2cm_param_t;
/* Static functions ----------------------------------------------------------*/

/**
 * @brief Test I2C MASTER driver
 * @details Test application for testing i2cm_init and i2cm_write() API
 * @param[in]  :
 * 					-Channel
 * 					- I2C mode
 * 					- Slave address
 * 					- Register address
 * 					- Data
 * @param[out] :NA
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_i2cm_001(char**argv, int argc);

/**
 * @brief Test I2C MASTER driver
 * @detail Test application for testing i2cm_read() API
 * @parameter[in]  :
 * 					-Channel
 * 					- I2C mode
 * 					- Slave address
 * 					- Register address
 * 					- Data
 * @parameter[out] :NA
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_i2cm_002(char**argv, int argc);

/**
 * @brief Test I2C MASTER driver
 * @detail Test application for testing read/write API
 * @parameter[in]  :
 * 					-Channel
 * 					- I2C mode
 * 					- Slave address
 * 					- Register address
 * 					- Data
 * @parameter[out] :NA
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_i2cm_003(char**argv, int argc);

/**
 * @brief Test I2C MASTER driver
 * @detail Test application for testing i2cm_deinit() API
 * @parameter[in]  :
 * 					-Channel
 * 					- I2C mode
 * 					- Slave address
 * 					- Register address
 * 					- Data
 * @parameter[out] :NA
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_i2cm_004(char**argv, int argc);
/**
 * @brief Test multiple-threading for I2C master
 * @details Test multiple-threading for I2C master.
 *          To evaluate 18 I2C master channels are accessed
 *          continuously in separated threads.
 * @param[in]  		:
 * @param[out] 		:
 * @return 0		: success
 *         Others	: instruction failed
 */
LOCAL int it_drv_i2cm_005(char**argv, int argc);

LOCAL it_map_t it_i2cm_tests_table[] =
{
		{"I2CM_001", it_drv_i2cm_001},
		{"I2CM_002", it_drv_i2cm_002},
		{"I2CM_003", it_drv_i2cm_003},
		{"I2CM_004", it_drv_i2cm_004},
		{"I2CM_005", it_drv_i2cm_005},
		{"",  NULL}
};
/**
 * @brief Common task function for test handle it_drv_i2cm_005()
 * @details Task to use for specified i2c master channel
 * @param[in]  	param	: task param
 * @param[out] 			: NA
 * @return 		0		: success
 *         		Others	: instruction failed
 */
LOCAL void i2cm_task(void* pvParam);
/**
 * @brief Common callback function for test handle it_drv_i2cm_005()
 * @param[in]  	status	: tunnel status
 * @param[in]  	param	: callback param
 * @param[out] 			: NA
 * @return 		NA
 * @details Common callback function for test handle it_drv_i2cm_005()
 */
LOCAL void i2cm_alarm(i2cm_error_t status, void *param);
/* Exported functions ------------------------------------------------------- */
/**
 * @brief I2C MASTER module's testing handler
 * @details to I2C MASTER module's testing handler
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @return 0	 : success
 *         Others: instruction failed
 */
int it_drv_i2cm_handler(char** argv, int argc)
{
	//! TODO: debug
	slogf(SLOG_ID_LCC_SYSTEM, SLOG_DEBUG, "Enter %s()\r\n", __FUNCTION__);
	/**< Check command counter >*/
	if (0    == argc) return -1;
	if (NULL == argv) return -1;
	/**< Get command index, mean test case name >*/
	int index = handler_parser(argv[0], it_i2cm_tests_table);
	if (-1 != index)
	{
		return it_i2cm_tests_table[index].handler(&argv[1], argc - 1);
	}
	else
		return index;
}

LOCAL int it_drv_i2cm_001(char**argv, int argc)
{
	/**! Check command parameter and argument */
	if((4 != argc) || (NULL == argv))
	{
		log_printf("Error: Wrong number of parameter or Empty argument.\r\n");
	}

	i2c_t channel = I2C_CH0;
	i2cm_mode_t mode = BYTE_ADDR8;
	uint8_t slave_address = 0x00;
	uint8_t reg_address = 0x00;
	uint8_t data[2] = {00, 00};
	uint16_t temp = 0x00;
	/**! Get I2C channel via argv[0] */
	channel = (i2c_t)strtol(argv[0], NULL, 10);

	/**! Get I2C mode via argv[1] */
	mode = (i2cm_mode_t)strtol(argv[1], NULL, 10);

	/**! Get Slave address via argv[2] */
	slave_address = (uint8_t)strtol(argv[2], NULL,16);

	/**! Get Register address via argv[3] */
	reg_address = (uint8_t)strtol(argv[3], NULL, 16);

	/**! Get data to via argv[4]*/
	temp = (uint16_t)strtol(argv[4], NULL, 16);
	data[0] = (temp & 0xFF);
	data[1] = ((temp >> 8) & 0xFF);

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initialize I2C Master */
	qc_assert(I2CM_ERROR_NONE != i2cm_init(channel));

	/**! Verify I2C Master send data */
	qc_assert(I2CM_ERROR_TRANSMITTED != i2cm_write(channel, mode,slave_address,
			reg_address, data));

	/**! Do judgment */
	qc_report();

	return I2CM_ERROR_NONE;
}

/**
 * @brief Test I2C MASTER driver
 * @detail Test application for testing i2cm_read() API
 * @parameter[in]  :
 * 					-Channel
 * 					- I2C mode
 * 					- Slave address
 * 					- Register address
 * 					- Data
 * @parameter[out] :NA
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_i2cm_002(char**argv, int argc)
{
	/**! Check command parameter and argument */
	if((4 != argc) || (NULL == argv))
	{
		log_printf("Error: Wrong number of parameter or Empty argument.\r\n");
	}

	i2c_t channel = I2C_CH0;
	i2cm_mode_t mode = BYTE_ADDR8;
	uint8_t slave_address = 0x00;
	uint8_t reg_address = 0x00;
	uint8_t data[2] = {00, 00};
	uint16_t temp = 0x00;
	uint8_t* buffer;
	/**! Get I2C channel via argv[0] */
	channel = (i2c_t)strtol(argv[0], NULL, 10);

	/**! Get I2C mode via argv[1] */
	mode = (i2cm_mode_t)strtol(argv[1], NULL, 10);

	/**! Get Slave address via argv[2] */
	slave_address = (uint8_t)strtol(argv[2], NULL,16);

	/**! Get Register address via argv[3] */
	reg_address = (uint8_t)strtol(argv[3], NULL, 16);

	/**! Get data to via argv[4]*/
	temp = (uint16_t)strtol(argv[4], NULL, 16);
	data[1] = (temp & 0xFF);
	data[2] = ((temp >> 8) & 0xFF);

	/**! Create buffer */
	buffer = malloc(sizeof(data));

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Check Initialize I2C Master */
	qc_assert(I2CM_ERROR_NONE != i2cm_init(channel));

	/**! Receive data from slave */
	qc_assert(I2CM_ERROR_TRANSCEIVED != i2cm_read(channel, mode, slave_address,
										reg_address, buffer));
	/**! Verify data */
	qc_assert(data[0] == buffer[0]);
	qc_assert(data[1] == buffer[1]);

	/**! Do judgment */
	qc_report();

	return I2CM_ERROR_NONE;

}

/**
 * @brief Test I2C MASTER driver
 * @detail Test application for testing read/write API
 * @parameter[in]  :
 * 					-Channel
 * 					- I2C mode
 * 					- Slave address
 * 					- Register address
 * 					- Data
 * @parameter[out] :NA
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_i2cm_003(char**argv, int argc)
{
	/**! Check command parameter and argument */
	if((4 != argc) || (NULL == argv))
	{
		log_printf("Error: Wrong number of parameter or Empty argument.\r\n");
	}

	i2c_t channel = I2C_CH0;
	i2cm_mode_t mode = BYTE_ADDR8;
	uint8_t slave_address = 0x00;
	uint8_t reg_address = 0x00;
	uint8_t data[2] = {00, 00};
	uint16_t temp = 0x00;
	uint8_t* buffer;
	uint32_t time_out = 2000;
	/**! Get I2C channel via argv[0] */
	channel = (i2c_t)strtol(argv[0], NULL, 10);

	/**! Get I2C mode via argv[1] */
	mode = (i2cm_mode_t)strtol(argv[1], NULL, 10);

	/**! Get Slave address via argv[2] */
	slave_address = (uint8_t)strtol(argv[2], NULL,16);

	/**! Get Register address via argv[3] */
	reg_address = (uint8_t)strtol(argv[3], NULL, 16);

	/**! Get data to via argv[4]*/
	temp = (uint16_t)strtol(argv[4], NULL, 16);
	data[0] = (temp & 0xFF);
	data[1] = ((temp >> 8) & 0xFF);

	/**! Create buffer */
	buffer = malloc(sizeof(data));

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Check Initialize I2C Master */
	qc_assert(I2CM_ERROR_NONE != i2cm_init(channel));

	qc_assert(I2CM_ERROR_TRANSMITTED != i2cm_write(channel, mode, slave_address,
										reg_address, data));
	/**! Wait before slave send data */
	while(--time_out);

	/**! Receive data from slave */
	qc_assert(I2CM_ERROR_TRANSCEIVED != i2cm_read(channel, mode, slave_address,
										reg_address, buffer));
	/**! Verify data */
	qc_assert(data[0] == buffer[0]);
	qc_assert(data[1] == buffer[1]);

	/**! Do judgment */
	qc_report();

	return I2CM_ERROR_NONE;
}

/**
 * @brief Test I2C MASTER driver
 * @param[in]  :
 * 					-Channel
 * 					- I2C mode
 * 					- Slave address
 * 					- Register address
 * 					- Data
 * @param[out] :NA
 * @return 0		: success
 *         Others	: instruction failed
 * @details Test application for testing i2cm_deinit() API
 */
LOCAL int it_drv_i2cm_004(char**argv, int argc)
{
	/**! Check command parameter and argument */
	if((4 != argc) || (NULL == argv))
	{
		log_printf("Error: Wrong number of parameter or Empty argument.\r\n");
	}

	i2c_t channel = I2C_CH0;
	i2cm_mode_t mode = BYTE_ADDR8;
	uint8_t slave_address = 0x00;
	uint8_t reg_address = 0x00;
	uint8_t data[2] = {00, 00};
	uint16_t temp = 0x00;
	/**! Get I2C channel via argv[0] */
	channel = (i2c_t)strtol(argv[0], NULL, 10);

	/**! Get I2C mode via argv[1] */
	mode = (i2cm_mode_t)strtol(argv[1], NULL, 10);

	/**! Get Slave address via argv[2] */
	slave_address = (uint8_t)strtol(argv[2], NULL,16);

	/**! Get Register address via argv[3] */
	reg_address = (uint8_t)strtol(argv[3], NULL, 16);

	/**! Get data to via argv[4]*/
	temp = (uint16_t)strtol(argv[4], NULL, 16);
	data[0] = (temp & 0xFF);
	data[1] = ((temp >> 8) & 0xFF);

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initialize I2C Master */
	qc_assert(I2CM_ERROR_NONE != i2cm_init(channel));

	/**! Verify I2C Master send data */
	qc_assert(I2CM_ERROR_TRANSMITTED != i2cm_write(channel, mode,slave_address,
			reg_address, data));

	/**! Check i2cm_deinit API */
	qc_assert(I2CM_ERROR_NONE != i2cm_deinit(channel));

	/**! Call back i2c_write send data to slave */
	qc_assert(I2CM_ERROR_TRANSMITTED != i2cm_write(channel, mode,slave_address,
				reg_address, data));

	/**! Do judgment */
	qc_report();

	return I2CM_ERROR_NONE;
}
/**
 * @brief Test multiple-threading for I2C master
 * @details Test multiple-threading for I2C master.
 *          To evaluate 18 I2C master channels are accessed
 *          continuously in separated threads.
 * @param[in]  		:
 * @param[out] 		:
 * @return 0		: success
 *         Others	: instruction failed
 */
LOCAL int it_drv_i2cm_005(char**argv, int argc)
{
	//! TODO: debug
	slogf(SLOG_ID_LCC_SYSTEM, SLOG_DEBUG, "Enter %s()\r\n", __FUNCTION__);
	//! Create all tasks
	uint16_t num_tasks = 19;
	uint16_t task_id = 0;
	i2cm_param_t param[19];
	int xResult = 0;
	char* name[20] = {
						"i2cm_0" , "i2cm_1" , "i2cm_2" , "i2cm_3" , "i2cm_4" ,
						"i2cm_5" , "i2cm_6" , "i2cm_7" , "i2cm_8" , "i2cm_9" ,
						"i2cm_10", "i2cm_11", "i2cm_12", "i2cm_13", "i2cm_14",
						"i2cm_15", "i2cm_16", "i2cm_17", "i2cm_18", ""       ,
					};
	//! Task operation data
	uint8_t tx_data[19][2] = {{0x00}};
	uint8_t rx_data[19][2] = {{0x00}};
	//! Create all tasks
	for(task_id = 0; task_id < num_tasks; task_id++)
	{
		/**!< Create param information >*/
		param[task_id].name          = name[task_id];
		param[task_id].tunnel        = (i2c_t)task_id;
		param[task_id].handler.idx   = task_id;
		param[task_id].handler.state = TASK_INITIALIZE;
		param[task_id].error         = I2CM_ERROR_NONE;
		param[task_id].database.tx_addr = tx_data[task_id];
		param[task_id].database.rx_addr = rx_data[task_id];
		//! Clear buffer
		memset(tx_data[task_id], 0x00, 2);
		memset(rx_data[task_id], 0x00, 2);
		//! Prepare data
		switch (param[task_id].tunnel)
		{
			case I2C_CAM_01:
			{
				/**
				 * Connector to 35mm camera
				 * I2C addr (8bit representation):
				 * - Aptina sensor:                          0x6C (Wr) 0x6D (Rd)
				 * - EEPROM:                                 0xA0 (Wr) 0xA1 (Rd)
				 * - LC898214 VCM driver IC:                 0xE4 (Wr) 0xE5 (Rd)
				 *
				 * Connector to 70/150mm camera
				 * I2C addr (8 bit representation):
				 * - Aptina sensor:                          0x6C (Wr) 0x6D (Rd)
				 * - EEPROM:                                 0xA0 (Wr) 0xA1 (Rd)
				 * - Mirror hall sensor AMS AS5510:          0xAE (Wr) 0xAF (Rd)
				 * - Lens hall sensor Allegro A1457:         0x18 (Wr) 0x19 (Rd)
				 * - Temperature (B4 mirror flex) TI TMP112: 0x90 (Wr) 0x91 (Rd)
				 */
				param[task_id].to               = 0xA1; //!< Read EEPROM
				param[task_id].database.tx_len  = 2;
				param[task_id].database.rx_len  = 2;
				break;
			}
			case I2C_CAM_02:
			{
				/**
				 * Connector to 35mm camera
				 * I2C addr (8bit representation):
				 * - Aptina sensor:                          0x6C (Wr) 0x6D (Rd)
				 * - EEPROM:                                 0xA0 (Wr) 0xA1 (Rd)
				 * - LC898214 VCM driver IC:                 0xE4 (Wr) 0xE5 (Rd)
				 *
				 * Connector to 70/150mm camera
				 * I2C addr (8 bit representation):
				 * - Aptina sensor:                          0x6C (Wr) 0x6D (Rd)
				 * - EEPROM:                                 0xA0 (Wr) 0xA1 (Rd)
				 * - Mirror hall sensor AMS AS5510:          0xAE (Wr) 0xAF (Rd)
				 * - Lens hall sensor Allegro A1457:         0x18 (Wr) 0x19 (Rd)
				 * - Temperature (B4 mirror flex) TI TMP112: 0x90 (Wr) 0x91 (Rd)
				 */
				param[task_id].to               = 0xA1; //!< Read EEPROM
				param[task_id].database.tx_len  = 2;
				param[task_id].database.rx_len  = 2;
				break;
			}
			case I2C_CAM_03:
			{
				/**
				 * Connector to 35mm camera
				 * I2C addr (8bit representation):
				 * - Aptina sensor:                          0x6C (Wr) 0x6D (Rd)
				 * - EEPROM:                                 0xA0 (Wr) 0xA1 (Rd)
				 * - LC898214 VCM driver IC:                 0xE4 (Wr) 0xE5 (Rd)
				 *
				 * Connector to 70/150mm camera
				 * I2C addr (8 bit representation):
				 * - Aptina sensor:                          0x6C (Wr) 0x6D (Rd)
				 * - EEPROM:                                 0xA0 (Wr) 0xA1 (Rd)
				 * - Mirror hall sensor AMS AS5510:          0xAE (Wr) 0xAF (Rd)
				 * - Lens hall sensor Allegro A1457:         0x18 (Wr) 0x19 (Rd)
				 * - Temperature (B4 mirror flex) TI TMP112: 0x90 (Wr) 0x91 (Rd)
				 */
				param[task_id].to               = 0xA1; //!< Read EEPROM
				param[task_id].database.tx_len  = 2;
				param[task_id].database.rx_len  = 2;
				break;
			}
			case I2C_CAM_04:
			{
				/**
				 * Connector to 35mm camera
				 * I2C addr (8bit representation):
				 * - Aptina sensor:                          0x6C (Wr) 0x6D (Rd)
				 * - EEPROM:                                 0xA0 (Wr) 0xA1 (Rd)
				 * - LC898214 VCM driver IC:                 0xE4 (Wr) 0xE5 (Rd)
				 *
				 * Connector to 70/150mm camera
				 * I2C addr (8 bit representation):
				 * - Aptina sensor:                          0x6C (Wr) 0x6D (Rd)
				 * - EEPROM:                                 0xA0 (Wr) 0xA1 (Rd)
				 * - Mirror hall sensor AMS AS5510:          0xAE (Wr) 0xAF (Rd)
				 * - Lens hall sensor Allegro A1457:         0x18 (Wr) 0x19 (Rd)
				 * - Temperature (B4 mirror flex) TI TMP112: 0x90 (Wr) 0x91 (Rd)
				 */
				param[task_id].to               = 0xA1; //!< Read EEPROM
				param[task_id].database.tx_len  = 2;
				param[task_id].database.rx_len  = 2;
				break;
			}
			case I2C_CAM_05:
			{
				/**
				 * Connector to 35mm camera
				 * I2C addr (8bit representation):
				 * - Aptina sensor:                          0x6C (Wr) 0x6D (Rd)
				 * - EEPROM:                                 0xA0 (Wr) 0xA1 (Rd)
				 * - LC898214 VCM driver IC:                 0xE4 (Wr) 0xE5 (Rd)
				 *
				 * Connector to 70/150mm camera
				 * I2C addr (8 bit representation):
				 * - Aptina sensor:                          0x6C (Wr) 0x6D (Rd)
				 * - EEPROM:                                 0xA0 (Wr) 0xA1 (Rd)
				 * - Mirror hall sensor AMS AS5510:          0xAE (Wr) 0xAF (Rd)
				 * - Lens hall sensor Allegro A1457:         0x18 (Wr) 0x19 (Rd)
				 * - Temperature (B4 mirror flex) TI TMP112: 0x90 (Wr) 0x91 (Rd)
				 */
				param[task_id].to               = 0xA1; //!< Read EEPROM
				param[task_id].database.tx_len  = 2;
				param[task_id].database.rx_len  = 2;
				break;
			}
			case I2C_CAM_06:
			{
				/**
				 * Connector to 35mm camera
				 * I2C addr (8bit representation):
				 * - Aptina sensor:                          0x6C (Wr) 0x6D (Rd)
				 * - EEPROM:                                 0xA0 (Wr) 0xA1 (Rd)
				 * - LC898214 VCM driver IC:                 0xE4 (Wr) 0xE5 (Rd)
				 *
				 * Connector to 70/150mm camera
				 * I2C addr (8 bit representation):
				 * - Aptina sensor:                          0x6C (Wr) 0x6D (Rd)
				 * - EEPROM:                                 0xA0 (Wr) 0xA1 (Rd)
				 * - Mirror hall sensor AMS AS5510:          0xAE (Wr) 0xAF (Rd)
				 * - Lens hall sensor Allegro A1457:         0x18 (Wr) 0x19 (Rd)
				 * - Temperature (B4 mirror flex) TI TMP112: 0x90 (Wr) 0x91 (Rd)
				 */
				param[task_id].to               = 0xA1; //!< Read EEPROM
				param[task_id].database.tx_len  = 2;
				param[task_id].database.rx_len  = 2;
				break;
			}
			case I2C_HALL_LENS_01:
			{
				/**
				 * Connector to 35mm camera
				 * I2C addr (8bit representation):
				 * - Aptina sensor:                          0x6C (Wr) 0x6D (Rd)
				 * - EEPROM:                                 0xA0 (Wr) 0xA1 (Rd)
				 * - LC898214 VCM driver IC:                 0xE4 (Wr) 0xE5 (Rd)
				 *
				 * Connector to 70/150mm camera
				 * I2C addr (8 bit representation):
				 * - Aptina sensor:                          0x6C (Wr) 0x6D (Rd)
				 * - EEPROM:                                 0xA0 (Wr) 0xA1 (Rd)
				 * - Mirror hall sensor AMS AS5510:          0xAE (Wr) 0xAF (Rd)
				 * - Lens hall sensor Allegro A1457:         0x18 (Wr) 0x19 (Rd)
				 * - Temperature (B4 mirror flex) TI TMP112: 0x90 (Wr) 0x91 (Rd)
				 */
				param[task_id].to               = 0x19; //! Magnetic register
				param[task_id].database.tx_len  = 1;
				param[task_id].database.rx_len  = 1;
				break;
			}
			case I2C_HALL_LENS_02:
			{
				/**
				 * Connector to 35mm camera
				 * I2C addr (8bit representation):
				 * - Aptina sensor:                          0x6C (Wr) 0x6D (Rd)
				 * - EEPROM:                                 0xA0 (Wr) 0xA1 (Rd)
				 * - LC898214 VCM driver IC:                 0xE4 (Wr) 0xE5 (Rd)
				 *
				 * Connector to 70/150mm camera
				 * I2C addr (8 bit representation):
				 * - Aptina sensor:                          0x6C (Wr) 0x6D (Rd)
				 * - EEPROM:                                 0xA0 (Wr) 0xA1 (Rd)
				 * - Mirror hall sensor AMS AS5510:          0xAE (Wr) 0xAF (Rd)
				 * - Lens hall sensor Allegro A1457:         0x18 (Wr) 0x19 (Rd)
				 * - Temperature (B4 mirror flex) TI TMP112: 0x90 (Wr) 0x91 (Rd)
				 */
				param[task_id].to               = 0x19; //! Magnetic register
				param[task_id].database.tx_len  = 1;
				param[task_id].database.rx_len  = 1;
				break;
			}
			case I2C_HALL_LENS_03:
			{
				/**
				 * Connector to 35mm camera
				 * I2C addr (8bit representation):
				 * - Aptina sensor:                          0x6C (Wr) 0x6D (Rd)
				 * - EEPROM:                                 0xA0 (Wr) 0xA1 (Rd)
				 * - LC898214 VCM driver IC:                 0xE4 (Wr) 0xE5 (Rd)
				 *
				 * Connector to 70/150mm camera
				 * I2C addr (8 bit representation):
				 * - Aptina sensor:                          0x6C (Wr) 0x6D (Rd)
				 * - EEPROM:                                 0xA0 (Wr) 0xA1 (Rd)
				 * - Mirror hall sensor AMS AS5510:          0xAE (Wr) 0xAF (Rd)
				 * - Lens hall sensor Allegro A1457:         0x18 (Wr) 0x19 (Rd)
				 * - Temperature (B4 mirror flex) TI TMP112: 0x90 (Wr) 0x91 (Rd)
				 */
				param[task_id].to               = 0x19; //! Magnetic register
				param[task_id].database.tx_len  = 1;
				param[task_id].database.rx_len  = 1;
				break;
			}
			case I2C_FORWARD_ASIC2:
			{
				param[task_id].to               = 0x08;
				param[task_id].database.tx_addr = NULL;
				param[task_id].database.tx_len  = 0;
				param[task_id].database.rx_addr = NULL;
				param[task_id].database.rx_len  = 0;
				break;
			}
			case I2C_HALL_LENS_05:
			{
				/**
				 * Connector to 35mm camera
				 * I2C addr (8bit representation):
				 * - Aptina sensor:                          0x6C (Wr) 0x6D (Rd)
				 * - EEPROM:                                 0xA0 (Wr) 0xA1 (Rd)
				 * - LC898214 VCM driver IC:                 0xE4 (Wr) 0xE5 (Rd)
				 *
				 * Connector to 70/150mm camera
				 * I2C addr (8 bit representation):
				 * - Aptina sensor:                          0x6C (Wr) 0x6D (Rd)
				 * - EEPROM:                                 0xA0 (Wr) 0xA1 (Rd)
				 * - Mirror hall sensor AMS AS5510:          0xAE (Wr) 0xAF (Rd)
				 * - Lens hall sensor Allegro A1457:         0x18 (Wr) 0x19 (Rd)
				 * - Temperature (B4 mirror flex) TI TMP112: 0x90 (Wr) 0x91 (Rd)
				 */
				param[task_id].to               = 0x19; //! Magnetic register
				param[task_id].database.tx_len  = 1;
				param[task_id].database.rx_len  = 1;
				break;
			}
			case I2C_HALL_LENS_06:
			{
				/**
				 * Connector to 35mm camera
				 * I2C addr (8bit representation):
				 * - Aptina sensor:                          0x6C (Wr) 0x6D (Rd)
				 * - EEPROM:                                 0xA0 (Wr) 0xA1 (Rd)
				 * - LC898214 VCM driver IC:                 0xE4 (Wr) 0xE5 (Rd)
				 *
				 * Connector to 70/150mm camera
				 * I2C addr (8 bit representation):
				 * - Aptina sensor:                          0x6C (Wr) 0x6D (Rd)
				 * - EEPROM:                                 0xA0 (Wr) 0xA1 (Rd)
				 * - Mirror hall sensor AMS AS5510:          0xAE (Wr) 0xAF (Rd)
				 * - Lens hall sensor Allegro A1457:         0x18 (Wr) 0x19 (Rd)
				 * - Temperature (B4 mirror flex) TI TMP112: 0x90 (Wr) 0x91 (Rd)
				 */
				param[task_id].to               = 0x19; //! Magnetic register
				param[task_id].database.tx_len  = 1;
				param[task_id].database.rx_len  = 1;
				break;
			}
			#ifdef P2_BOARD
			case I2C_UNUSED_01:
			#else
			case I2C_HALL_MIRROR_01:
			#endif
			{
				param[task_id].to               = 0x00;
				param[task_id].database.tx_addr = NULL;
				param[task_id].database.tx_len  = 0;
				param[task_id].database.rx_addr = NULL;
				param[task_id].database.rx_len  = 0;
				break;
			}
			case I2C_HALL_MIRROR_02:
			{
				/**
				 * Connector to 35mm camera
				 * I2C addr (8bit representation):
				 * - Aptina sensor:                          0x6C (Wr) 0x6D (Rd)
				 * - EEPROM:                                 0xA0 (Wr) 0xA1 (Rd)
				 * - LC898214 VCM driver IC:                 0xE4 (Wr) 0xE5 (Rd)
				 *
				 * Connector to 70/150mm camera
				 * I2C addr (8 bit representation):
				 * - Aptina sensor:                          0x6C (Wr) 0x6D (Rd)
				 * - EEPROM:                                 0xA0 (Wr) 0xA1 (Rd)
				 * - Mirror hall sensor AMS AS5510:          0xAE (Wr) 0xAF (Rd)
				 * - Lens hall sensor Allegro A1457:         0x18 (Wr) 0x19 (Rd)
				 * - Temperature (B4 mirror flex) TI TMP112: 0x90 (Wr) 0x91 (Rd)
				 */
				param[task_id].to                  = 0xAF;
				param[task_id].database.tx_addr[0] = 0x14; //Read PREG register
				param[task_id].database.tx_len     = 1;
				param[task_id].database.rx_len     = 2;
				break;
			}
			case I2C_HALL_MIRROR_03:
			{
				/**
				 * Connector to 35mm camera
				 * I2C addr (8bit representation):
				 * - Aptina sensor:                 0x6C (Wr) 0x6D (Rd)
				 * - EEPROM:                        0xA0 (Wr) 0xA1 (Rd)
				 * - LC898214 VCM driver IC:        0xE4 (Wr) 0xE5 (Rd)
				 *
				 * Connector to 70/150mm camera
				 * I2C addr (8 bit representation):
				 * - Aptina sensor:                          0x6C (Wr) 0x6D (Rd)
				 * - EEPROM:                                 0xA0 (Wr) 0xA1 (Rd)
				 * - Mirror hall sensor AMS AS5510:          0xAE (Wr) 0xAF (Rd)
				 * - Lens hall sensor Allegro A1457:         0x18 (Wr) 0x19 (Rd)
				 * - Temperature (B4 mirror flex) TI TMP112: 0x90 (Wr) 0x91 (Rd)
				 */
				param[task_id].to                  = 0xAF;
				param[task_id].database.tx_addr[0] = 0x14; //Read PREG register
				param[task_id].database.tx_len     = 1;
				param[task_id].database.rx_len     = 2;
				break;
			}
			case I2C_FORWARD_ASIC3:
			{
				param[task_id].to               = 0x08;
				param[task_id].database.tx_addr = NULL;
				param[task_id].database.tx_len  = 0;
				param[task_id].database.rx_addr = NULL;
				param[task_id].database.rx_len  = 0;
				break;
			}
			#ifdef P2_BOARD
			case I2C_UNUSED_01:
			#else
			case I2C_HALL_MIRROR_05:
			#endif
			{
				param[task_id].to               = 0x00;
				param[task_id].database.tx_addr = NULL;
				param[task_id].database.tx_len  = 0;
				param[task_id].database.rx_addr = NULL;
				param[task_id].database.rx_len  = 0;
				break;
			}
			case I2C_HALL_MIRROR_06:
			{
				/**
				 * Connector to 35mm camera
				 * I2C addr (8bit representation):
				 * - Aptina sensor:                          0x6C (Wr) 0x6D (Rd)
				 * - EEPROM:                                 0xA0 (Wr) 0xA1 (Rd)
				 * - LC898214 VCM driver IC:                 0xE4 (Wr) 0xE5 (Rd)
				 *
				 * Connector to 70/150mm camera
				 * I2C addr (8 bit representation):
				 * - Aptina sensor:                          0x6C (Wr) 0x6D (Rd)
				 * - EEPROM:                                 0xA0 (Wr) 0xA1 (Rd)
				 * - Mirror hall sensor AMS AS5510:          0xAE (Wr) 0xAF (Rd)
				 * - Lens hall sensor Allegro A1457:         0x18 (Wr) 0x19 (Rd)
				 * - Temperature (B4 mirror flex) TI TMP112: 0x90 (Wr) 0x91 (Rd)
				 */
				param[task_id].to                  = 0xAF;
				param[task_id].database.tx_addr[0] = 0x14; //Read PREG register
				param[task_id].database.tx_len     = 1;
				param[task_id].database.rx_len     = 2;
				break;
			}
			case I2C_SYS:
			{
				/**!
				 * ASB -> TMP112:
				 *                     - 0x90 (wr) 0x91 (rd)
				 *                     - 0x92 (wr) 0x93 (rd)
				 *                     - 0x94 (wr) 0x95 (rd)
				 * P2 -> INA231
				 * INA231 I2C Address:
				 *                     - 0x80 wr, 0x81 rd
				 *                     - 0x82 wr, 0x83 rd
				 *                     - 0x88 wr, 0x89 rd
				 *                     - 0x8A wr, 0x8B rd
				 **/
				#ifndef P2_BOARD
				param[task_id].to               = 0x91; //! Temperature Register
				#else // ASB_BOARD
				param[task_id].to               = 0x81;
				#endif /**! P2_BOARD */
				param[task_id].database.tx_len  = 1;
				param[task_id].database.rx_len  = 2;
				break;
			}
		}
		//! Create task
		if ((I2C_FORWARD_ASIC2 != param[task_id].tunnel)
		 && (I2C_FORWARD_ASIC2 != param[task_id].tunnel))
		{
			//! Create task for indicated channel
			if(pdPASS != xTaskCreate(i2cm_task,
									param[task_id].name,
									__TASK_STACK_SIZE_128,
									&param[task_id],
									__TASK_PRIO_HIGHEST>>1,
									&param[task_id].handler.handle))
				return -1;
			//! TODO: debug
			slogf(SLOG_ID_LCC_SYSTEM, SLOG_DEBUG, "Created %s\r\n", param[task_id].name);
		}
	}
	//! Waiting to all tasks is finished
	while (1)
	{
		//! Check all task is deleted or not
		for(task_id = 0; task_id < num_tasks; task_id++)
		{
			//! Check indicated task is deleted or not
			if (eDeleted != eTaskGetState(param[task_id].handler.handle))
			{
				//! Indicated task have not finished
				break;
			}
		}
		//! Check all task is deleted or not
		if(task_id == num_tasks)
		{
			//! All task is done, exit.
			break;
		}
		//! Delay for other task
		vTaskDelay(5);
	}
	//! Check all own statuses
	for(task_id = 0; task_id < num_tasks; task_id++)
	{
		if (I2CM_ERROR_NONE != param[task_id].error)
		{
			//! Return FALSED
			xResult = -1;
		}
	}
	//! Send information in case FAILED case
	if (-1 == xResult)
	{
		for(task_id = 0; task_id < num_tasks; task_id++)
		{
			//! Test PASSED
			slogf(SLOG_ID_LCC_SYSTEM, SLOG_INFO, "%u: %u\r\n", \
					             task_id, param[task_id].error);
		}
	}
	//! Return PASSED
	return xResult;
}
/**
 * @brief Common task function for test handle it_drv_i2cm_005()
 * @details Task to use for specified i2c master channel
 * @param[in]  	param	: task param
 * @param[out] 			: NA
 * @return 		0		: success
 *         		Others	: instruction failed
 */
LOCAL void i2cm_task(void* pvParam)
{
	//! Cast parameter
	i2cm_param_t* param = pvParam;
	volatile uint32_t timeout = 0;
	//! Change stage to active
	param->handler.state = TASK_READY;

	//! TODO: debug
	slogf(SLOG_ID_LCC_SYSTEM, SLOG_DEBUG, "Enter %s()\r\n", __FUNCTION__);
	//! TODO: debug
	slogf(SLOG_ID_LCC_SYSTEM, SLOG_DEBUG, "Tunnel: %u\r\n", param->tunnel);

	//! Re-init the tunnel
	i2cm_init(param->tunnel);
	//! Start looping action
	while(1)
	{
		if (I2CM_ERROR_NONE == param->error)
		{
			//! Sending data via indicated channel
			param->error = i2cm_transceiver(param->tunnel,            \
					                        param->to,                \
											param->database.tx_addr,  \
											param->database.tx_len,   \
											param->database.rx_addr,  \
											param->database.rx_len,   \
											i2cm_alarm, param);
		}
		//! Waiting for complete
		while ((I2CM_ERROR_BUSY == param->error) && (timeout++ < 10))
		{
			//! Delay for other task work
			vTaskDelay(1);
		}
		//! Check timeout
		if (timeout == 10)
		{
			//! For judgment FAILED
			param->error = I2CM_ERROR_INVALID;
		}
		else
		{
			//! For judgment PASSED
			param->error = I2CM_ERROR_NONE;
		}
		//! Exit loop
		break;
	}
	//! De-initialize indicated i2c channel
	i2cm_deinit(param->tunnel);
	//! Exit current task
	vTaskDelete(xTaskGetCurrentTaskHandle());
}
/**
 * @brief Common callback function for test handle it_drv_i2cm_005()
 * @param[in]  	status	: tunnel status
 * @param[in]  	param	: callback param
 * @param[out] 			: NA
 * @return 		NA
 * @details Common callback function for test handle it_drv_i2cm_005()
 */
LOCAL void i2cm_alarm(i2cm_error_t status, void *param)
{
	i2cm_param_t* pvParam = param;
	pvParam->error = status;
}
/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
