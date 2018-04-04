/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    it_hal_gpio.h
 * @author  The LightCo
 * @version V1.0.1
 * @date    19-Apr, 2016
 * @brief   This file contains expand of the hal_gpio driver
 *
 ******************************************************************************/

/******************************************************************************/
/**								Revision history
 * * 1.0.0	27-Jan-2016	Initial revision:
 *                      - Infrastructure.
 * * 1.0.1	19-Apr-2016 Test HAL baseline 1.0.0
 */
/******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _IT_HAL_GPIO_H_
#define _IT_HAL_GPIO_H_
/* Includes ------------------------------------------------------------------*/
#include "hal_gpio.h"
#include "qc_assert.h"
#include "it_log_swapper.h"
/* Exported typedef ----------------------------------------------------------*/

/* Exported define -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */

/**
 * @brief GPIO module's testing handler
 * @detail to GPIO module's testing handler
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @return 0	: success
 *         Others: instruction failed
 */
IMPORT int it_hal_gpio_handler(char** argv, int argc);

#endif /**! _IT_HAL_GPIO_H_ */

/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
