/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    time.c
 * @author  The LightCo
 * @version V1.0.0
 * @date    Jun 21, 2016
 * @brief   
 *
 ******************************************************************************/

/******************************************************************************/
/**							Revision history
 *
 * * 1.0.0	Jun 21, 2016	Initial revision:
 */
/******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef TIME_C_
#define TIME_C_
/* Includes ------------------------------------------------------------------*/
#include "time.h"
#include "lcc_system.h"
/* Exported define -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Inline functions  ---------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
time_t time(time_t *__timer)
{
	if (NULL == __timer)
		return (light_system->t_time.raw);
	else
		*__timer = light_system->t_time.raw;
	return (light_system->t_time.milisec);
}
#endif /* TIME_C_ */

/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
