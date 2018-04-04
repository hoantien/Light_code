/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms,
 * with or without modification,
 * are strictly prohibited without prior permission of The LightCo.
 *
 * @file    light_header.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    July-25-2016
 * @brief
 *
 ******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _LIGHT_HEADER_H_
#define _LIGHT_HEADER_H_
#ifdef __cplusplus
extern "C"
{
#endif
/* Includes ------------------------------------------------------------------*/
#include "lightheader.pb-c.h"
#include "camera_module.pb-c.h"
#include "camera_id.pb-c.h"
#include "point2f.pb-c.h"
#include "point2i.pb-c.h"
#include "protobuf-c/protobuf-c.h"

/* 1MB protobuf header size */
#define MAX_PROTOBUF_HEADER_SIZE	(1024 * 1024)
/* Private typedef -----------------------------------------------------------*/
typedef enum lightheader_error
{
	LIGHT_HEADER_ERROR_NONE = 0,
	LIGHT_HEADER_NO_CAM_ERROR,
	LIGHT_HEADER_INVALID_ARG
}lightheader_error_t;

typedef struct __attribute__((__packed__)) lightheader_info
{
	volatile uint8_t magic_id[4];
	volatile uint64_t record_size;
	volatile uint64_t header_offset;
	volatile uint32_t header_size;
	volatile uint64_t reversed;
} lightheader_info_t;
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/*
 * @brief alloc_init_light_header
 * This function used to allocate and initialize the light header
 * @return Allocated LighHeder pointer
 */
Ltpb__LightHeader * alloc_init_light_header(void);

/*
 * @brief get_light_header
 * This function used to get the light header
 * @param m_bitmask: the bit mask of camera channel
 * @param pheader : point to light header variable of user
 * @return a LighHeder pointer include light header informations
 */
lightheader_error_t get_light_header(uint32_t m_bitmask, Ltpb__LightHeader *pheader);

/*
 * @brief free_mem_light_header
 * This function used to free memory for all malloc function in light header
 * @return none
 */
void free_mem_light_header(Ltpb__LightHeader *pheader);

#ifdef __cplusplus
}
#endif
#endif /* _LIGHT_HEADER_H_ */
/*********** Portions COPYRIGHT 2016 Light.Co., Ltd. ********END OF FILE*******/
