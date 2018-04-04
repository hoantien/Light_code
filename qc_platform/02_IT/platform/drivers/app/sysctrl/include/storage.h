/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    storage.h
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
#ifndef STORAGE_H_
#define STORAGE_H_
/* Includes ------------------------------------------------------------------*/
#include "qc_common.h"
/* Exported define -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Inline functions  ---------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/**
 * @brief Storage control task
 * @param[in] 	NA
 * @param[out] 	NA
 * @return 		NA
 * @details Storage control task
 */
IMPORT void task_storage(void* pv);

#endif /* STORAGE_H_ */

/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
