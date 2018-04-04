/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    it_hal_scu.h
 * @author  The LightCo
 * @version V1.0
 * @date    13-May-2016
 * @brief   This file contains expand of the hal_scu driver
 *
 ******************************************************************************/

/******************************************************************************/
/**								Revision history
 * * 1.0.0	13-May-2016	Initial revision:
 */
/******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _IT_HAL_SCU_H_
#define _IT_HAL_SCU_H_
/* Includes ------------------------------------------------------------------*/
#include "qc_assert.h"
#include "cortex_r4.h"
#include "std_type.h"

/* Exported typedef ----------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/**
 * @brief SCU module's testing handler
 * @detail to PWM module's testing handler
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @return 0	 : success
 *         Others: instruction failed
 */
IMPORT int it_hal_scu_handler(char** argv, int argc);
#endif /* _IT_HAL_SCU_H_ */
/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
