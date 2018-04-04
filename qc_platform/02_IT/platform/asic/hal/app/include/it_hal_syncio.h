/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    it_hal_syncio.h
 * @author  The LightCo
 * @version V1.0.1
 * @date    19-Apr, 2016
 * @brief   This file contains expand of the hal_syncio driver
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
#ifndef _IT_HAL_SYNCIO_H_
#define _IT_HAL_SYNCIO_H_

/* Includes ------------------------------------------------------------------*/
#include "qc_assert.h"
#include "hal_syncio.h"
/* Exported typedef ----------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/**
 * @brief SYNCIO module's testing handler
 * @detail to SYNCIO module's testing handler
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @return 0	 : success
 *         Others: instruction failed
 */
IMPORT int it_hal_syncio_handler(char** argv, int argc);

#endif
/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
