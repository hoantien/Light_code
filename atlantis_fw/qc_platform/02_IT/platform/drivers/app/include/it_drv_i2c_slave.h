/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    it_drv_i2c_slave.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    June, 3, 2016
 * @brief   
 *
 ******************************************************************************/

/******************************************************************************/
/**							Revision history
 *
 * * 1.0.0	June, 3, 2016	Initial revision:
 */
/******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef IT_DRV_I2C_SLAVE_H_
#define IT_DRV_I2C_SLAVE_H_
/* Includes ------------------------------------------------------------------*/
#include "qc_assert.h"
#include "i2c_slave.h"
#include "it_log_swapper.h"
/* Exported typedef ----------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/

/* Inline functions  ---------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/**
 * @brief I2C SLAVE module's testing handler
 * @detail to I2C SLAVE module's testing handler
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @return 0	 : success
 *         Others: instruction failed
 */
IMPORT int it_drv_i2c_slave_handler(char** argv, int argc);

#endif /* IT_DRV_I2C_SLAVE_H_ */

/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
