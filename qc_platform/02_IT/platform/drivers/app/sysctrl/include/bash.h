/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    bash.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    June 15, 2016
 * @brief   
 *
 ******************************************************************************/

/******************************************************************************/
/**							Revision history
 *
 * * 1.0.0	June 15, 2016	Initial revision:
 */
/******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BASH_H_
#define BASH_H_
/* Includes ------------------------------------------------------------------*/
#include "qc_common.h"
/* Exported define -----------------------------------------------------------*/
#define BASH_PREFIX '$'
/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Inline functions  ---------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/**
 * @brief Initialize bash
 * @param[in] 	NA
 * @param[out] 	NA
 * @return 		NA
 * @details Initialize bash
 */
IMPORT void bash_init(void);
/**
 * @brief Bash/shell task
 * @param[in] 	NA
 * @param[out] 	NA
 * @return 		NA
 * @details Bash/shell task
 */
IMPORT void task_bash(void* pv);

#endif /* BASH_H_ */

/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
