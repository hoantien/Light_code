/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    af_lens_calib.c
 * @author  The LightCo
 * @version V1.0.0
 * @date    Oct-1-2016
 * @brief   Lens position calibration for auto-focus
 *
 ******************************************************************************/

#include <stdbool.h>
#include "af_lens_calib.h"

static const int16_t HC_MAX = 32767;
static const int16_t HC_MIN = -32767;
static const uint16_t MAX_OBJECT_DISTANCE_MM = 65535;

static float object_distance_to_lens_distance(float focal_length, float object_distance)
{
    return focal_length * object_distance / (object_distance - focal_length);
}

static float lens_distance_to_object_distance(float focal_length, float lens_distance)
{
    return object_distance_to_lens_distance(focal_length, lens_distance);
}

static int16_t interpolate_hallcode(const af_calib_t* calib, uint8_t i, uint16_t object_distance_mm)
{
    const af_calib_point_t* cp1 = &calib->calib_point_arr[i-1];
    const af_calib_point_t* cp2 = &calib->calib_point_arr[i];
    float lp1 = object_distance_to_lens_distance(calib->focal_length_mm, cp1->object_distance_mm);
    float lp2 = object_distance_to_lens_distance(calib->focal_length_mm, cp2->object_distance_mm);
    float l = object_distance_to_lens_distance(calib->focal_length_mm, object_distance_mm);
    float hc = (cp1->hallcode * (lp2 - l) + cp2->hallcode * (l - lp1)) / (lp2 - lp1);
    if (hc > HC_MAX)
        hc = HC_MAX;
    else if (hc < HC_MIN)
        hc = HC_MIN;
    return (int16_t)hc;
}

int16_t object_distance_to_hallcode(const af_calib_t* af_calib, uint16_t object_distance_mm)
{
    uint8_t i;
    for (i = 0; i < af_calib->num_calib_points; ++i)
    {
        if (af_calib->calib_point_arr[i].object_distance_mm >= object_distance_mm)
            break;
    }
    if (i == 0)
        i = 1;
    else if (i == af_calib->num_calib_points)
        i = af_calib->num_calib_points - 1;
    return interpolate_hallcode(af_calib, i, object_distance_mm);
}

static uint16_t interpolate_object_distance(const af_calib_t* calib, uint8_t i, int16_t hallcode)
{
    const af_calib_point_t* cp1 = &calib->calib_point_arr[i-1];
    const af_calib_point_t* cp2 = &calib->calib_point_arr[i];
    float lp1 = object_distance_to_lens_distance(calib->focal_length_mm, cp1->object_distance_mm);
    float lp2 = object_distance_to_lens_distance(calib->focal_length_mm, cp2->object_distance_mm);
    float l = (lp1 * (cp2->hallcode - hallcode) + lp2 * (hallcode - cp1->hallcode)) / (cp2->hallcode - cp1->hallcode);
    float object_d = MAX_OBJECT_DISTANCE_MM;
    if (l > calib->focal_length_mm) {
        object_d = lens_distance_to_object_distance(calib->focal_length_mm, l);
        if (object_d > MAX_OBJECT_DISTANCE_MM)
            object_d = MAX_OBJECT_DISTANCE_MM;
    }
    return (uint16_t)object_d;
}

uint16_t hallcode_to_object_distance(const af_calib_t* af_calib, int16_t hallcode)
{
    bool is_hc_increasing = (af_calib->calib_point_arr[0].hallcode < af_calib->calib_point_arr[1].hallcode);
    uint8_t i;
    for (i = 0; i < af_calib->num_calib_points; ++i)
    {
        bool is_gt = (af_calib->calib_point_arr[i].hallcode >= hallcode);
        if ((is_gt && is_hc_increasing) || !(is_gt || is_hc_increasing))
            break;
    }
    if (i == 0)
        i = 1;
    else if (i == af_calib->num_calib_points)
        i = af_calib->num_calib_points - 1;
    return interpolate_object_distance(af_calib, i, hallcode);
}

