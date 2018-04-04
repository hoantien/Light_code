/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    lens_position_control.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    August-15-2016
 * @brief   Control of lens position common interface for VCM and piezos
 *
 ******************************************************************************/

#include <cstdlib>
#include "vcm.h"
#include "af_helper.h"
#include "actuator.h"
#include "lens_position_control.h"

#define SLOGF_ID                SLOG_ID_AF

extern "C" void move_done_callback_c(void* lpc) {
    static_cast<lens_position_control*>(lpc)->move_done();
}

void lens_position_control::move_done() {
    // FIXME check if move succeeded
    (*move_done_cb_.cb_fun)(move_done_cb_.user_data, true, current_hall_code());
}

lens_position_control::lens_position_control(cam_typedef_t& cam) :
        cam_(cam),
        is_vcm_{cam.info.grp == GRP_A} {
}

std::int16_t lens_position_control::get_hall_code(std::uint32_t object_distance_mm, std::int16_t lens_temp_degc_10) const {
    // FIXME add temperature correction
    std::int16_t hallcode;
    if (is_vcm()) {
        vcm_object_distance_to_hallcode(static_cast<vcm_t*>(cam_.optical->lens), object_distance_mm, &hallcode);
    } else {
        std::uint16_t hc;
        actuator_object_distance_to_lens_hallcode(static_cast<actuator_t*>(cam_.optical->lens), object_distance_mm, &hc);
        hallcode = hc;
    }
    return hallcode;
}

std::uint32_t lens_position_control::get_object_distance_mm(std::int16_t hall_code, std::int16_t lens_temp_degc_10) const {
    // FIXME add temperature correction
    std::uint32_t object_distance_mm;
    if (is_vcm()) {
        vcm_hallcode_to_object_distance(static_cast<vcm_t*>(cam_.optical->lens), hall_code, &object_distance_mm);
    } else {
        actuator_lens_hallcode_to_object_distance(static_cast<actuator_t*>(cam_.optical->lens), hall_code, &object_distance_mm);
    }
    return object_distance_mm;
}


void lens_position_control::get_hard_stop_hall_codes(std::int16_t* hc_min, std::int16_t* hc_max) const {
    if (is_vcm())
        vcm_get_hard_stop_hall(static_cast<vcm_t*>(cam_.optical->lens), hc_min, hc_max);
    else {
        std::uint16_t hmin = *hc_min;
        std::uint16_t hmax = *hc_max;
        actuator_get_hard_stop_hall(static_cast<actuator_t*>(cam_.optical->lens), &hmin, &hmax);
        *hc_min = hmin;
        *hc_max = hmax;
    }
}

std::int16_t lens_position_control::current_hall_code() const {
    std::int16_t hc;
    if (is_vcm())
        hc = static_cast<vcm_t*>(cam_.optical->lens)->target_hall_code;
    else
        hc = static_cast<actuator_t*>(cam_.optical->lens)->last_move_hall_code;
    return hc;
}

bool lens_position_control::start_move_lens(std::int16_t hall_code, std::uint16_t tolerance, move_done_callback_t callback) {
    SLOGF(SLOG_DEBUG, "Request to move lens %X to position %d with tolerance %d", (int)cam_.info.module, (int)hall_code, (int)tolerance);
    move_done_cb_ = callback;
    bool move_started{true};
    if (std::abs(current_hall_code() - hall_code) < tolerance) {
        move_done();
    } else {
        cam_return_t ret = cam_move_lens_to_position(&cam_, hall_code, tolerance, &move_done_callback_c, this, 0); // FIXME tid = 0?
        move_started = (ret == CAM_OK);
    }
    return move_started;
}
