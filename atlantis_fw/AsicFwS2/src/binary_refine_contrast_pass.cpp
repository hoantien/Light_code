/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    binary_refine_contrast_pass.cpp
 * @author  The LightCo
 * @version V1.0.0
 * @date    Oct-31-2016
 * @brief   Contrast pass which aggressively refines search interval
 *
 ******************************************************************************/

#include "log.h"
#include "binary_refine_contrast_pass.h"

#define SLOGF_ID                SLOG_ID_AF

binary_refine_contrast_pass::binary_refine_contrast_pass(std::uint16_t tolerance,
        bool use_metric1) :
            tolerance_{tolerance},
            use_metric1_{use_metric1} {
}

void binary_refine_contrast_pass::initialize(const contrast_pass& prev_pass) {
    // Update measurements
    const measurement_t* meas = prev_pass.measurements();
    meas_arr_[0] = meas[0];
    meas_arr_[1] = meas[1];
    initialize();
}

void binary_refine_contrast_pass::initialize() {
    // Compute initial next_pos_
    if (measurement_t::is_better(use_metric1_, meas_arr_[1], meas_arr_[0]))
        best_meas_idx_ = 1;
    else
        best_meas_idx_ = 0;
    delta_ = (meas_arr_[1 - best_meas_idx_].pos - meas_arr_[best_meas_idx_].pos + 3) / 4;
    next_pos_ = meas_arr_[best_meas_idx_].pos + delta_;
    second_meas_ongoing_ = false;
    done_ = false;
    SLOGF(SLOG_INFO, "Starting contrast binary refine pass with interval (%d, %d) and best_idx %d",
            (int)meas_arr_[0].pos, (int)meas_arr_[1].pos, (int)best_meas_idx_);
}

bool binary_refine_contrast_pass::is_done() const {
    return done_;
}

std::int16_t binary_refine_contrast_pass::next_pos() {
    return next_pos_;
}

std::uint16_t binary_refine_contrast_pass::tolerance() const {
    return tolerance_;
}

void binary_refine_contrast_pass::set_measurement(const measurement_t& meas) {
    bool is_new_best{measurement_t::is_better(use_metric1_, meas, meas_arr_[best_meas_idx_])};
    if (!second_meas_ongoing_) {
        meas_arr_[1 - best_meas_idx_] = meas;
        if (!is_new_best)
            done_ = true;
        else {
            second_meas_ongoing_ = true;
            best_meas_idx_ = 1 - best_meas_idx_;
            next_pos_ = meas_arr_[best_meas_idx_].pos + delta_;
        }
    } else {
        bool is_new_second{!is_new_best && measurement_t::is_better(use_metric1_, meas, meas_arr_[1 - best_meas_idx_])};
        if (is_new_best || is_new_second) {
            meas_arr_[1 - best_meas_idx_] = meas_arr_[best_meas_idx_];
            meas_arr_[best_meas_idx_] = meas;
            if (is_new_second)
                best_meas_idx_ = 1 - best_meas_idx_;
        }
        done_ = true;
    }
}

std::int16_t binary_refine_contrast_pass::out_min_pos() const {
    // New start position
    return meas_arr_[0].pos;
}

std::int16_t binary_refine_contrast_pass::out_max_pos() const {
    // New end position
    return meas_arr_[1].pos;
}

std::int16_t binary_refine_contrast_pass::out_best_pos() const {
    return meas_arr_[best_meas_idx_].pos;
}

std::int64_t binary_refine_contrast_pass::out_best_metric() const {
    return use_metric1_ ? meas_arr_[best_meas_idx_].metric1 : meas_arr_[best_meas_idx_].metric2;
}

std::size_t binary_refine_contrast_pass::num_measurements() const {
    return meas_arr_.size();
}

auto binary_refine_contrast_pass::measurements() const -> const measurement_t* {
    return meas_arr_.data();
}
