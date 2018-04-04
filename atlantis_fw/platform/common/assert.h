/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    assert.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    June-26-2015
 * @brief   This file contains definitions of the assert function
 *
 ******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ASSERT_H__
#define __ASSERT_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdint.h>
#include "os.h"

/* Exported define -----------------------------------------------------------*/
#ifdef _USE_FULL_ASSERT
/**
  * @brief  The assert_param macro is used for function's malloc check.
  * @param  expr: If expr is false, it calls assert_failed function
  *   which reports the name of the source file and the source
  *   line number of the call that failed.
  *   If expr is true, it returns no value.
  * @retval None
  */
#define assert_param(prt) ((prt) ? (void)0 : \
						assert_failed((uint8_t *)__FUNCTION__, __LINE__))
/**
 * @brief  The assert_param macro is used for function's malloc check.
 * @param  expr: If expr is false, it calls malloc_failed function
 *   which reports the name of the source file and the source
 *   line number of the call that failed.
 *   If expr is true, it returns pointer to memory was malloc.
 * @retval None
 */
#define assert_malloc(ptr, size) ((ptr = pvPortMalloc(size)) ? ptr : \
						malloc_failed((uint8_t *)__FUNCTION__, __LINE__))

/* Exported functions --------------------------------------------------------*/
void assert_failed(uint8_t *func, uint32_t line);
void *malloc_failed(uint8_t *func, uint32_t line);

#define assert(prt) assert_param(prt)

#else
#define assert(prt)
#define assert_param(prt)
#define assert_malloc(ptr, size) malloc(size)
#endif

#ifdef __cplusplus
}
#endif
#endif /* __ASSERT_H__ */

/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
