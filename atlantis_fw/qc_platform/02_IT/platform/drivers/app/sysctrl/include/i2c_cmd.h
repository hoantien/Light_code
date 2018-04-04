/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    i2c_cmd.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    June 20, 2016
 * @brief   
 *
 ******************************************************************************/

/******************************************************************************/
/**							Revision history
 *
 * * 1.0.0	June 20, 2016	Initial revision:
 */
/******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef I2C_CMD_H_
#define I2C_CMD_H_
/* Includes ------------------------------------------------------------------*/
#include "qc_common.h"
/* Exported define -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Inline functions  ---------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/**
 * @brief I2C command receptionist
 * @param[in] 	NA
 * @param[out] 	NA
 * @return 		NA
 * @details I2C command receptionist
 */
IMPORT void task_i2c_recept(void* pv);

#endif /* I2C_CMD_H_ */

/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
