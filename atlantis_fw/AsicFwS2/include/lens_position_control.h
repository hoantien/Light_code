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

#ifndef __LENS_POSITION_CONTROL_H__
#define __LENS_POSITION_CONTROL_H__

#ifdef __cplusplus

#include <cstdint>
#include "light_system.h"

extern "C" void move_done_callback_c(void* lpc);

class lens_position_control {
    friend void move_done_callback_c(void* lpc);

public:
    lens_position_control(cam_typedef_t& cam);

    std::int16_t get_hall_code(std::uint32_t object_distance_mm, std::int16_t lens_temp_degc_10) const;

    std::uint32_t get_object_distance_mm(std::int16_t hall_code, std::int16_t lens_temp_degc_10) const;

    void get_hard_stop_hall_codes(std::int16_t* hc_min, std::int16_t* hc_max) const;

    std::int16_t current_hall_code() const;

    struct move_done_callback_t {
        void (*cb_fun)(void*, bool, std::int16_t);
        void* user_data;
    };

    bool start_move_lens(std::int16_t hall_code, std::uint16_t tolerance, move_done_callback_t callback);

    cam_typedef_t& camera() {
        return cam_;
    }

    bool is_vcm() const {
        return is_vcm_;
    }

    bool is_piezo() const {
        return !is_vcm();
    }

private:
    void move_done();

    cam_typedef_t& cam_;
    bool is_vcm_;
    move_done_callback_t move_done_cb_{nullptr, nullptr};
};

#endif
#endif /* __LENS_POSITION_CONTROL_H__ */
