/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    com_srv.h
 * @author  The LightCo.
 * @version 1.0.0
 * @date    June 20, 2016
 * @brief   
 *
 ******************************************************************************/

/******************************************************************************/
/**  						Revision history
 *
 * * 1.0.0	June 20, 2016	Initial revision
 */
/******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "qc_common.h"
/* Private define ------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Static functions prototype-------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Exported global variables --------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/**
 * @brief Communication service task
 * @param[in] 	NA
 * @param[out] 	NA
 * @return 		NA
 * @details Communication service task
 */
IMPORT void task_com_srv(void* pv);
/**
 * @brief I2C communication control task
 * @param[in] 	pv	: own task handler
 * @param[out] 	NA
 * @return 		NA
 * @details I2C communication control task
 */
IMPORT void task_i2c_ctrl(void* pv);
/**
 * @brief SPI communication control task
 * @param[in] 	pv	: own task handler
 * @param[out] 	NA
 * @return 		NA
 * @details SPI communication control task
 */
IMPORT void task_spi_ctrl(void* pv);
/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
