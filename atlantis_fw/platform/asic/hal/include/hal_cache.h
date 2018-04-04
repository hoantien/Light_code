/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification,are strictly prohibited without prior permission of The LightCo.
 *
 * @file    hal_cache.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    Mar-15-2016
 * @brief   ARMv7 cache managing
 *
 ******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HAL_CACHE_H__
#define __HAL_CACHE_H__

#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

extern void flush_cache(uint32_t start, uint32_t size);

extern void invalidate_cache(uint32_t start, uint32_t size);

extern void dcache_disable(void);

extern void dcache_enable(void);

extern void dcache_invalidate(void);

extern uint32_t get_cache_line_len(void);

#ifdef __cplusplus
}
#endif
#endif  /* __HAL_CACHE_H__ */
/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
