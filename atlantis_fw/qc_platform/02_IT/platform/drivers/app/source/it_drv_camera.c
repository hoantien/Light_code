/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    it_drv_camera.c
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

/* Define to prevent recursive inclusion -------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdlib.h>
#include "it_drv_camera.h"
#include "qc_common.h"

/* Private define ------------------------------------------------------------*/
#define 								REG_R		0x00
#define									REG_W		0x01
/* Static functions ----------------------------------------------------------*/

/**
 * @brief Test CAMERA driver
 * @detail Test application for testing cam_open API
 * @parameter[in]  :
 * 					- CAM Channel
 * 					- CAM Name
 * 					- CAM Type
 * 					- CAM Address
 * @parameter[out] :NA
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_cam_001(char** argv, int argc);

/**
 * @brief Test CAMERA driver
 * @detail Test application for testing cam_read_reg or cam_write_reg API
 * @parameter[in]  :
 * 					- CAM Channel
 * 					- CAM Name
 * 					- CAM Type
 * 					- CAM Address
 * 					- REGISTER Address
 * 					- DATA mode
 * @parameter[out] :NA
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_cam_002(char** argv, int argc);

/**
 * @brief Test CAMERA driver
 * @detail Test application for testing camera update resolution
 * @parameter[in]  :
 * 					- CAM Channel
 * 					- CAM Name
 * 					- CAM Type
 * 					- CAM Address
 * @parameter[out] :NA
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_cam_003(char** argv, int argc);

LOCAL it_map_t it_camera_tests_table[] =
{		{"CAM_001", it_drv_cam_001},
		{"CAM_002", it_drv_cam_002},
		{"CAM_003", it_drv_cam_003},
		{"",  NULL}
};

/* Exported functions ------------------------------------------------------- */
/**
 * @brief CAMERA module's testing handler
 * @detail to CAMERA module's testing handler
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @return 0	 : success
 *         Others: instruction failed
 */
int it_drv_camera_handler(char** argv, int argc)
{
	/**< Check command counter >*/
	if (1 >= argc) return -1;
	if (NULL == argv) return -1;
	/**< Get command index, mean test case name >*/
	int index = handler_parser(*argv, it_camera_tests_table);
	if (-1 != index)
	{
		return it_camera_tests_table[index].handler(&argv[1], argc - 1);
	}
	else
		return index;
}

/**
 * @brief Test CAMERA driver
 * @detail Test application for testing cam_open API
 * @parameter[in]  :
 * 					- CAM Channel
 * 					- CAM Name
 * 					- CAM Type
 * 					- CAM Address
 * @parameter[out] :NA
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_cam_001(char** argv, int argc)
{
	/**! Check command parameter and argument*/
	if((4 != argc) || (NULL == argv))
	{
		log_printf("Error: Wrong number of parameter or Empty argument.\r\n");
	}

	cam_module_t cam;
	cam_channel_t	channel = 0;
	cam_type_t type = 0;
	uint8_t address = 0;
	cam.m_o_status = CAM_MODULE_SW_STANDBY;

	/**! Get CAM channel number via argv[0] */
	channel = (cam_channel_t)strtol(argv[0], NULL, 10);
	memcpy((void*)&cam.chid, (void*)&channel, sizeof(int));

	/**! Get CAM name via argv[1] */
	memcpy((void*)&cam.name, (void*)&argv[1], sizeof(char));

	/**! Get CAM type via argv[2] */
	type = (cam_type_t)strtol(argv[2], NULL, 10);
	memcpy((void*)&cam.type, (void*)&type, sizeof(int));

	/**! Get CAM address via argv[3] */
	address = (uint8_t)strtol(argv[3], NULL, 16);
	memcpy((void*)&cam.slave_addr, (void*)&address, sizeof(int));

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Call API cam_init() */
	cam_init(&cam);

	/**! Call API cam_open */
	cam_open(&cam);

	qc_assert(CAM_MODULE_CLOSE != cam.m_o_status);

	/**! Do judgment */
	qc_report();

	return CAM_MODULE_CLOSE;
}

/**
 * @brief Test CAMERA driver
 * @detail Test application for testing cam_read_reg or cam_write_reg API
 * @parameter[in]  :
 * 					- CAM Channel
 * 					- CAM Name
 * 					- CAM Type
 * 					- CAM Address
 * 					- REGISTER Address
 * 					- DATA mode
 * @parameter[out] :NA
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_cam_002(char** argv, int argc)
{
	/**! Check command parameter and argument*/
	if((4 != argc) || (NULL == argv))
	{
		log_printf("Error: Wrong number of parameter or Empty argument.\r\n");
	}

	cam_module_t cam;
	cam_channel_t	channel = 0;
	cam_type_t type = 0;
	uint8_t address = 0;
	uint16_t reg_config = 0x00;
	uint16_t data = 0x00;
	uint8_t data_mode = DATA_8BIT;
	uint8_t r_w = 0;
	cam.m_o_status = CAM_MODULE_SW_STANDBY;

	/**! Get CAM channel number via argv[0] */
	channel = (cam_channel_t)strtol(argv[0], NULL, 10);
	memcpy((void*)&cam.chid, (void*)&channel, sizeof(int));

	/**! Get CAM name via argv[1] */
	memcpy((void*)&cam.name, (void*)&argv[1], sizeof(char));

	/**! Get CAM type via argv[2] */
	type = (cam_type_t)strtol(argv[2], NULL, 10);
	memcpy((void*)&cam.type, (void*)&type, sizeof(int));

	/**! Get CAM address via argv[3] */
	address = (uint8_t)strtol(argv[3], NULL, 16);
	memcpy((void*)&cam.slave_addr, (void*)&address, sizeof(int));

	/**! Get address register */
	reg_config = (uint16_t)strtol(argv[4], NULL, 16);

	/**! Get data write register */
	data = (uint16_t)strtol(argv[5], NULL, 16);

	/**! Get data mode */
	data_mode = (uint8_t)strtol(argv[5], NULL, 10);

	if((DATA_8BIT != data_mode) && (DATA_16BIT != data_mode))
	{
		log_printf("Error: Do not support data mode .\r\n");
		return -1;
	}

	/**! Get variable select read or write data */
	r_w = (uint8_t)strtol(argv[6], NULL, 10);
	if((REG_R != r_w) && (REG_W != r_w))
	{
		log_printf("Error: Do not support mode read or write register.\r\n");
		return -1;
	}

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Call API cam_init() */
	cam_init(&cam);

	if(REG_R == r_w)
	{
		/**! Call API cam_read_reg */
		qc_assert(FALSE != cam_read_reg(&cam, reg_config, data_mode));
	}
	else
	{
		/**! Call API cam_write_reg */
		qc_assert(TRUE == cam_write_reg(&cam, reg_config, data, data_mode));
	}

	/**! Do judgment */
	qc_report();

	return CAM_MODULE_CLOSE;
}

/**
 * @brief Test CAMERA driver
 * @detail Test application for testing camera update resolution
 * @parameter[in]  :
 * 					- CAM Channel
 * 					- CAM Name
 * 					- CAM Type
 * 					- CAM Address
 * 					- X resolution
 * 					- Y resolution
 * 					- UCID
 * @parameter[out] :NA
 * @return 0		: success
 * Others			: instruction failed
 */
LOCAL int it_drv_cam_003(char** argv, int argc)
{

	/**! Check command parameter and argument*/
	if((4 != argc) || (NULL == argv))
	{
		log_printf("Error: Wrong number of parameter or Empty argument.\r\n");
	}

	cam_module_t cam;
	cam_channel_t	channel = 0;
	cam_type_t type = 0;
	uint8_t address = 0;
	cam_x_resolution_type_t x_res = X_RES_3M;
	cam_y_resolution_type_t y_res = Y_RES_3M;
	uint8_t temp1 = 0;
	uint8_t temp2 = 0;
	uint16_t ucid = 0;
	cam.m_o_status = CAM_MODULE_SW_STANDBY;

	/**! Get CAM channel number via argv[0] */
	channel = (cam_channel_t)strtol(argv[0], NULL, 10);
	memcpy((void*)&cam.chid, (void*)&channel, sizeof(int));

	/**! Get CAM name via argv[1] */
	memcpy((void*)&cam.name, (void*)&argv[1], sizeof(char));

	/**! Get CAM type via argv[2] */
	type = (cam_type_t)strtol(argv[2], NULL, 10);
	memcpy((void*)&cam.type, (void*)&type, sizeof(int));

	/**! Get CAM address via argv[3] */
	address = (uint8_t)strtol(argv[3], NULL, 16);
	memcpy((void*)&cam.slave_addr, (void*)&address, sizeof(int));

	/**! Get x resolution via argv[4] */
	temp1 = (uint8_t)strtol(argv[4], NULL, 10);

	/**! Get y resolution via argv[5] */
	temp2 = (uint8_t)strtol(argv[5], NULL, 10);

	/**! Get camera_ucid via argv[6] */
	ucid = (uint16_t)strtol(argv[6], NULL, 10);

	switch(temp1)
	{
		case 0:
		{
			x_res = X_RES_3M;
			break;
		}
		case 1:
		{
			x_res = X_RES_13M;
			break;
		}
		case 2:
		{
			x_res = X_RES_720P;
			break;
		}
		case 3:
		{
			x_res = X_RES_1080P;
			break;
		}
		case 4:
		{
			x_res = X_RES_4K_UHD;
			break;
		}
		case 5:
		{
			x_res = X_RES_4K_CINEMA;
			break;
		}
		default:
		{
			log_printf("Error: Do not support x resolution .\r\n");
			return -1;
		}
	}

	switch(temp2)
	{
		case 0:
		{
			y_res = Y_RES_3M;
			break;
		}
		case 1:
		{
			y_res = Y_RES_13M;
			break;
		}
		case 2:
		{
			y_res = Y_RES_720P;
			break;
		}
		case 3:
		{
			y_res = Y_RES_1080P;
			break;
		}
		case 4:
		{
			y_res = Y_RES_4K_UHD_CINEMA;
			break;
		}
		default:
		{
			log_printf("Error: Do not support y resolution .\r\n");
			return -1;
		}
	}
	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Call API cam_init() */
	cam_init(&cam);

	/**! Call API cam_open */
	cam_open(&cam);

	/**! Verify cam_open */
	qc_assert(CAM_MODULE_CLOSE != cam.m_o_status);

	/**! Verify change resolution */
	qc_assert(CAM_MODULE_CLOSE !=
			cam_update_resolution(x_res, y_res, &cam, ucid));

	/**! Do judgment */
	qc_report();

	return CAM_MODULE_CLOSE;
}
/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
