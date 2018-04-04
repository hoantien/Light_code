/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    register_dump.h
 * @author  The LightCo
 * @version V1.0.1
 * @date    Jul 28, 2016
 * @brief   To dump register setting from CAMERA
 *
 ******************************************************************************/

/******************************************************************************/
/**							Revision history
 * * 1.0.0	Jul 28, 2016		Initial revision
 */
/******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _REGISTER_DUMP_H_
#define _REGISTER_DUMP_H_
//! Support Cpp
#ifdef __cplusplus
extern "C" 
{
#endif /**! __cplusplus */
/* Includes ------------------------------------------------------------------*/
#include "os.h"
/* Exported define -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
//! Camera register data information
typedef struct
{
	char     name[32];  /**!< Dumped register's name        >*/
	uint16_t child;     /**!< I2C channel                   >*/
	uint8_t  id;        /**!< I2C slave address             >*/
	uint16_t addr;      /**!< Register address on device    >*/
	uint8_t* data;      /**!< Transmitted data              >*/
	uint16_t size;      /**!< Transmitted data size         >*/
	volatile _Bool check;/**!< Flag to indicate verification status  >*/
}cam_regs_dump_t;
/* Exported constants --------------------------------------------------------*/

/* Exported global variables -------------------------------------------------*/
//! Dumping cameras registers queue
extern xQueueHandle	 queue_cam_registers;
//! Camera registers dumping semaphore
extern xSemaphoreHandle sempr_cam_register_dump;
/* Exported functions ------------------------------------------------------- */
/**
 * @brief Task to verify register setting from camera modules
 * @details Task to verify register setting from camera modules
 * @param[in] 	vParameter: task parameter
 * @param[out]	NA
 * @return 		NA
 */
extern void task_cam_register_dump(void *vParameter);
//! Support Cpp
#ifdef __cplusplus
}
#endif /**! __cplusplus */

#endif /**! _REGISTER_DUMP_H_ */
/*********** Portions COPYRIGHT 2016 Light.Co., Ltd.***** END OF FILE *********/
