/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    it_hal_app.h
 * @author  The LightCo
 * @version V1.0.1
 * @date    Apr 19, 2016
 * @brief   This file contains expand of the hal application
 *
 ******************************************************************************/

/******************************************************************************/
/**								Revision history
 * * 1.0.0	20-Feb-2016	Initial revision:
 *                      - Infrastructure.
 * * 1.0.1	19-Apr-2016 Test HAL baseline 1.0.0
 */
/******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _IT_HAL_APP_H_
#define _IT_HAL_APP_H_

/* Includes ------------------------------------------------------------------*/
//#include "it_hal_irq.h"
//#include "it_hal_mipi.h"
#ifdef QC_ASIC2
#include "it_hal_com.h"
#include "it_hal_i2c.h"
#include "it_hal_pwm.h"
#include "it_hal_scu.h"
#include "it_hal_syncio.h"
#include "it_hal_spi.h"
#include "it_hal_wdt.h"
#include "it_hal_timer.h"
#include "it_hal_gpio.h"
//#include "it_hal_qspi.h"
#include "it_hal_mipi.h"
#include "it_hal_ddr.h"
#endif

/* Exported typedef ----------------------------------------------------------*/

/* Exported define -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported global variables -------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /**! _IT_HAL_APP_H_ */
/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/


