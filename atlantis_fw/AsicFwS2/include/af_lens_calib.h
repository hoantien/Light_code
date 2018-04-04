/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    af_lens_calib.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    Oct-1-2016
 * @brief   Lens position calibration for auto-focus
 *
 ******************************************************************************/

#ifndef __AF_LENS_CALIB_H__
#define __AF_LENS_CALIB_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint16_t object_distance_mm;
    int16_t  hallcode;
} af_calib_point_t;

typedef struct
{
    const af_calib_point_t* calib_point_arr;
    uint8_t num_calib_points;
    float focal_length_mm;
} af_calib_t;

int16_t object_distance_to_hallcode(const af_calib_t* af_calib, uint16_t object_distance_mm);

uint16_t hallcode_to_object_distance(const af_calib_t* af_calib, int16_t hallcode);

#ifdef __cplusplus
}
#endif
#endif /* __AF_LENS_CALIB_H__ */

/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
