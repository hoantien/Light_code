/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    it_hal_spi.h
 * @author  The LightCo
 * @version V1.0.1
 * @date    19-Apr, 2016
 * @brief   This file contains expand of the hal_spi driver
 *
 ******************************************************************************/

/******************************************************************************/
/**								Revision history
 * * 1.0.0	20-Jan-2016	Initial revision:
 *                      - Infrastructure.
 * * 1.0.1	19-Apr-2016 Test HAL baseline 1.0.0
 */
/******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _IT_HAL_QSPI_H_
#define _IT_HAL_QSPI_H_
/* Includes ------------------------------------------------------------------*/
#include "hal_qspi.h"
#include "qc_assert.h"

/* Exported typedef ----------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/**
 * @brief QSPI module's testing handler
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @return 0	 : success
 *         Others: instruction failed
 * @details to QSPI module's testing handler
 */
int it_hal_qspi_handler(char** argv, int argc);

/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
#endif /* _IT_HAL_QSPI_H_ */
