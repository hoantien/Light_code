/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification,are strictly prohibited without prior permission of The LightCo.
 *
 * @file	hal_ddr.h
 * @author	The LightCo
 * @version	V1.0.0
 * @date	Jun-20-2016
 * @brief	This file contains definitions of the LPDDR config
 *
 ******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HAL_DDR_H__
#define __HAL_DDR_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
/* Exported typedef ----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/*
 * @brief hal_ddr_init
 * Initialize LPDDR3
 * @return: none
 */
void hal_ddr_init(void);

#ifdef __cplusplus
}
#endif
#endif /* __HAL_DDR_H__ */

/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
